#!/usr/bin/env python3
#
#  Copyright (C) 2025 wise-vision
#
#  SPDX-License-Identifier: MPL-2.0
#
#  This Source Code Form is subject to the terms of the Mozilla Public
#  License, v. 2.0. If a copy of the MPL was not distributed with this
#  file, You can obtain one at https://mozilla.org/MPL/2.0/.
#
"""Timed publishing must not block the server, must be stoppable, and the
stop message must be the last one on the topic.

Regression tests for wise-vision/ros2_mcp#77 (review round 2): a continuous
``ros2_topic_publish`` (frequency x duration) runs as a coroutine on the
server's event loop inside the request's own task. That single design choice
is what these tests pin down:

- the loop never blocks the event loop (the original #77 defect),
- a single publish to the same topic, ``cancel_active_publish=true`` and
  cancelling the request task all stop the loop (F4),
- no message of a stopped loop can follow the stop message (F1),
- the 300 s duration cap is enforced before anything is created.

Every long wait is bounded with timeouts plus explicit assertions, so on the
unfixed code these tests FAIL rather than hang. The pure timing/cancel logic
is covered on the host by scripts/tests/test_timed_publish.py.
"""

import asyncio
import json
import time
from unittest.mock import MagicMock, patch

import pytest
import rclpy

from server import server as srv
import server.ros2_manager as _rm
import server.tools_ros2 as _tools
from server.ros2_manager import ROS2Manager
from server.ros2_manager_extended import ExtendedROS2Manager


def _ensure_rclpy():
    if not rclpy.ok():
        rclpy.init()


def _new_manager(mock_node_cls) -> ROS2Manager:
    mock_node = MagicMock()
    mock_node_cls.return_value = mock_node
    manager = ROS2Manager()
    # Skip the container discovery warm-up (it would spin the mocked node for
    # up to MCP_ROS_DISCOVERY_TIMEOUT_SEC on the first tool call).
    manager._discovery_warmed_up = True
    return manager


def _result_json(contents) -> dict:
    assert contents, "tool call returned no content"
    return json.loads(contents[0].text)


@patch.object(_rm, "ServiceNode")
def test_timed_publish_does_not_block_event_loop(mock_node_cls):
    """While a timed publish runs, another tool call must be served promptly."""
    _ensure_rclpy()
    try:
        manager = _new_manager(mock_node_cls)
        with patch.object(_tools, "_ros_instance", manager):

            async def scenario():
                timed = asyncio.create_task(
                    srv.call_tool(
                        "ros2_topic_publish",
                        {
                            "topic_name": "/cmd_vel",
                            "message_type": "std_msgs/msg/String",
                            "data": {"data": "moving"},
                            "frequency": 10.0,
                            "duration": 2.5,
                        },
                    )
                )
                stop = asyncio.create_task(
                    srv.call_tool(
                        "ros2_topic_publish",
                        {
                            "topic_name": "/emergency_stop",
                            "message_type": "std_msgs/msg/String",
                            "data": {"data": "stop"},
                        },
                    )
                )
                start = time.monotonic()
                stop_contents = await stop
                stop_wait = time.monotonic() - start
                timed_contents = await timed
                return stop_wait, stop_contents, timed_contents

            stop_wait, _stop_contents, timed_contents = asyncio.run(scenario())

        # Before the fix the publish loop blocked the event loop (round 0)
        # or a worker of the bounded to_thread pool (round 1), so the stop
        # call could not be served while it ran.
        assert stop_wait < 1.0, (
            f"a single publish waited {stop_wait:.2f}s while a timed publish "
            "was running on another topic"
        )
        timed_result = _result_json(timed_contents)
        assert timed_result["status"] == "published"
        assert timed_result["published_count"] >= 1
        # Backward compatibility: the pre-existing result keys stay.
        assert {
            "status",
            "data",
            "published_count",
            "frequency",
            "duration",
        } <= set(timed_result)
    finally:
        if rclpy.ok():
            rclpy.shutdown()


@patch.object(_rm, "ServiceNode")
def test_duration_above_maximum_is_rejected_without_publishing(mock_node_cls):
    _ensure_rclpy()
    try:
        manager = _new_manager(mock_node_cls)
        result = asyncio.run(
            manager.publish_to_topic(
                "/chatter",
                "std_msgs/msg/String",
                {"data": "hello"},
                frequency=10.0,
                duration=301.0,
            )
        )
        assert "error" in result
        assert "300" in result["error"]
        # A rejected request must not leave a publisher behind on the node.
        manager.node.create_publisher.assert_not_called()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


@patch.object(_rm, "ServiceNode")
def test_single_publish_to_same_topic_stops_and_orders(mock_node_cls):
    """F1: a later publish on the same topic stops the running loop, and the
    stop message is the last message the topic ever sees from the pair."""
    _ensure_rclpy()
    try:
        manager = _new_manager(mock_node_cls)
        timed_pub, stop_pub = MagicMock(), MagicMock()
        manager.node.create_publisher.side_effect = [timed_pub, stop_pub]
        records = []
        timed_pub.publish.side_effect = lambda msg: records.append("move")
        stop_pub.publish.side_effect = lambda msg: records.append("STOP")

        async def scenario():
            timed = asyncio.create_task(
                manager.publish_to_topic(
                    "/cmd_vel",
                    "std_msgs/msg/String",
                    {"data": "go"},
                    frequency=100.0,
                    duration=30.0,
                )
            )
            while len(records) < 3:
                await asyncio.sleep(0.005)

            stop = await manager.publish_to_topic(
                "/cmd_vel", "std_msgs/msg/String", {"data": "stop"}
            )
            assert stop["status"] == "published"

            with pytest.raises(asyncio.CancelledError):
                await timed

        asyncio.run(scenario())
        time.sleep(0.05)  # any late publish of the stopped loop would land here

        assert records, "nothing was published"
        assert records[-1] == "STOP", records
        assert "move" not in records[records.index("STOP"):], records
        assert manager._timed_publish_registry.active_topics() == []
    finally:
        if rclpy.ok():
            rclpy.shutdown()


@patch.object(_rm, "ServiceNode")
def test_cancel_active_publish_tool_stops_running_timed_publish(mock_node_cls):
    _ensure_rclpy()
    try:
        manager = _new_manager(mock_node_cls)
        timed_pub = MagicMock()
        manager.node.create_publisher.return_value = timed_pub

        async def scenario():
            timed = asyncio.create_task(
                manager.publish_to_topic(
                    "/cmd_vel",
                    "std_msgs/msg/String",
                    {"data": "go"},
                    frequency=100.0,
                    duration=30.0,
                )
            )
            while timed_pub.publish.call_count < 2:
                await asyncio.sleep(0.005)

            with patch.object(_tools, "_ros_instance", manager):
                contents = await srv.call_tool(
                    "ros2_topic_publish",
                    {
                        "topic_name": "/cmd_vel",
                        "message_type": "std_msgs/msg/String",
                        "data": {"data": "unused"},
                        "cancel_active_publish": True,
                    },
                )
            cancel_result = _result_json(contents)
            assert cancel_result["status"] == "cancel_requested"
            assert cancel_result["was_active"] is True

            with pytest.raises(asyncio.CancelledError):
                await timed

        asyncio.run(scenario())
        publish_count = timed_pub.publish.call_count
        time.sleep(0.05)
        assert timed_pub.publish.call_count == publish_count, (
            "publishing continued after cancel_active_publish"
        )
    finally:
        if rclpy.ok():
            rclpy.shutdown()


@patch.object(_rm, "ServiceNode")
def test_cancel_active_publish_with_no_active_publish(mock_node_cls):
    _ensure_rclpy()
    try:
        manager = _new_manager(mock_node_cls)
        with patch.object(_tools, "_ros_instance", manager):
            contents = asyncio.run(
                srv.call_tool(
                    "ros2_topic_publish",
                    {
                        "topic_name": "/quiet_topic",
                        "message_type": "std_msgs/msg/String",
                        "data": {"data": "unused"},
                        "cancel_active_publish": True,
                    },
                )
            )
        cancel_result = _result_json(contents)
        assert cancel_result["status"] == "no_active_timed_publish"
        assert cancel_result["was_active"] is False
    finally:
        if rclpy.ok():
            rclpy.shutdown()


@patch.object(_rm, "ServiceNode")
def test_cancelling_the_request_stops_publishing(mock_node_cls):
    """F4: MCP request cancellation cancels the timed loop, because the loop
    runs inside the request's own task."""
    _ensure_rclpy()
    try:
        manager = _new_manager(mock_node_cls)
        timed_pub = MagicMock()
        manager.node.create_publisher.return_value = timed_pub

        async def scenario():
            with patch.object(_tools, "_ros_instance", manager):
                request = asyncio.create_task(
                    srv.call_tool(
                        "ros2_topic_publish",
                        {
                            "topic_name": "/cmd_vel",
                            "message_type": "std_msgs/msg/String",
                            "data": {"data": "moving"},
                            "frequency": 100.0,
                            "duration": 30.0,
                        },
                    )
                )
                while timed_pub.publish.call_count < 2:
                    await asyncio.sleep(0.005)

                request.cancel()
                with pytest.raises(asyncio.CancelledError):
                    await request

        asyncio.run(scenario())
        publish_count = timed_pub.publish.call_count
        time.sleep(0.05)
        assert timed_pub.publish.call_count == publish_count, (
            "publishing continued after the request was cancelled"
        )
        assert manager._timed_publish_registry.active_topics() == []
    finally:
        if rclpy.ok():
            rclpy.shutdown()


@patch.object(_rm, "ServiceNode")
def test_timed_publishes_on_different_topics_run_concurrently(mock_node_cls):
    """Two timed publishes on different topics must both make progress."""
    _ensure_rclpy()
    try:
        manager = _new_manager(mock_node_cls)
        publishers = [MagicMock(), MagicMock()]
        manager.node.create_publisher.side_effect = publishers
        topics = ("/stream_a", "/stream_b")

        async def scenario():
            tasks = {
                topic: asyncio.create_task(
                    manager.publish_to_topic(
                        topic,
                        "std_msgs/msg/String",
                        {"data": "x"},
                        frequency=100.0,
                        duration=30.0,
                    )
                )
                for topic in topics
            }
            await asyncio.sleep(0.3)
            # Both loops are registered at the same time: neither blocked
            # the other.
            assert set(manager._timed_publish_registry.active_topics()) == set(topics)
            for publisher in publishers:
                assert publisher.publish.call_count >= 5, (
                    "a timed publish made no progress while another ran"
                )
            for topic in topics:
                assert manager.cancel_timed_publish(topic)["was_active"] is True
            for topic in topics:
                with pytest.raises(asyncio.CancelledError):
                    await tasks[topic]

        asyncio.run(scenario())
    finally:
        if rclpy.ok():
            rclpy.shutdown()


@patch.object(_rm, "ServiceNode")
def test_publish_multiple_topics_rejects_over_max_duration(mock_node_cls):
    _ensure_rclpy()
    try:
        manager = _new_manager(mock_node_cls)
        ros = ExtendedROS2Manager(base_manager=manager)

        result = asyncio.run(
            ros.publish_multiple_topics(
                [
                    {
                        "topic_name": "/multi",
                        "message_type": "std_msgs/msg/String",
                        "data": {"data": "x"},
                        "frequency": 10.0,
                        "duration": 301.0,
                    }
                ]
            )
        )
        assert result["/multi"].startswith("Invalid config")
        assert "300" in result["/multi"]
        manager.node.create_publisher.assert_not_called()
    finally:
        if rclpy.ok():
            rclpy.shutdown()
