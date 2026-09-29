#!/usr/bin/env python3
#
#  Copyright (C) 2025 wisevision
#
#  SPDX-License-Identifier: MPL-2.0
#
#  This Source Code Form is subject to the terms of the Mozilla Public
#  License, v. 2.0. If a copy of the MPL was not distributed with this
#  file, You can obtain one at https://mozilla.org/MPL/2.0/.
#
"""Read-only mode and tool-safety classification.

Every registered tool must be explicitly reviewed and classified as either
read-only or mutating. A new tool that nobody classified fails
``test_every_registered_tool_is_classified``.
"""

import asyncio

import pytest

from server import server as srv
from server import tool_safety


@pytest.fixture(autouse=True)
def _restore_full_toolset():
    """Each test leaves the module-level registry in the default (full) state."""
    yield
    srv.configure_tools(read_only=False)


def _listed_names() -> set[str]:
    return {t.name for t in asyncio.run(srv.list_tools())}


def test_readonly_list_tools_excludes_ros2_topic_publish():
    srv.configure_tools(read_only=True)
    names = _listed_names()
    assert "ros2_topic_publish" not in names
    assert "ros2_topic_list" in names


def test_readonly_call_tool_ros2_topic_publish_is_unknown_tool():
    srv.configure_tools(read_only=True)
    with pytest.raises(Exception) as exc_info:
        asyncio.run(
            srv.call_tool(
                "ros2_topic_publish",
                {"topic_name": "/cmd_vel", "message_type": "geometry_msgs/msg/Twist", "data": {}},
            )
        )
    assert "Unknown tool: ros2_topic_publish" in str(exc_info.value)


def test_readonly_hides_every_mutating_tool():
    srv.configure_tools(read_only=True)
    names = _listed_names()
    assert names.isdisjoint(tool_safety.MUTATING_TOOLS)
    assert names == set(tool_safety.READ_ONLY_TOOLS)


def test_default_mode_registers_everything():
    srv.configure_tools(read_only=False)
    names = _listed_names()
    assert "ros2_topic_publish" in names
    assert tool_safety.MUTATING_TOOLS <= names


def test_every_registered_tool_is_classified():
    srv.configure_tools(read_only=False)
    registered = set(srv.tool_handlers)
    classified = set(tool_safety.READ_ONLY_TOOLS) | set(tool_safety.MUTATING_TOOLS)
    unreviewed = registered - classified
    stale = classified - registered
    assert not unreviewed, f"Tools not classified in server/tool_safety.py: {sorted(unreviewed)}"
    assert not stale, f"Classified tools that are not registered: {sorted(stale)}"


def test_read_only_and_mutating_sets_are_disjoint():
    assert tool_safety.READ_ONLY_TOOLS.isdisjoint(tool_safety.MUTATING_TOOLS)


def test_minimum_mutating_set():
    required = {
        "ros2_topic_publish",
        "ros2_service_call",
        "ros2_send_action_goal",
        "ros2_cancel_action_goal",
        "ros2_publish_multiple_topics",
    }
    assert required <= tool_safety.MUTATING_TOOLS
    for name in required:
        assert tool_safety.is_mutating(name) is True
    assert tool_safety.is_mutating("ros2_topic_list") is False


def test_api_types():
    assert isinstance(tool_safety.MUTATING_TOOLS, frozenset)
    assert isinstance(tool_safety.READ_ONLY_TOOLS, frozenset)


@pytest.mark.parametrize(
    "env,argv,expected",
    [
        ({}, [], False),
        ({"ROS2_MCP_READONLY": "1"}, [], True),
        ({"ROS2_MCP_READONLY": "true"}, [], True),
        ({"ROS2_MCP_READONLY": "0"}, [], False),
        ({}, ["mcp_ros_2_server", "--read-only"], True),
        ({"ROS2_MCP_READONLY": "0"}, ["mcp_ros_2_server", "--read-only"], True),
    ],
)
def test_read_only_requested(env, argv, expected):
    assert srv.read_only_requested(environ=env, argv=argv) is expected
