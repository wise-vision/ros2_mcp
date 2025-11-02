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

from unittest.mock import MagicMock, patch
from server.tools_ros2 import (
    ROS2TopicList,
    ROS2ServiceList,
    ROS2InterfaceList,
    ROS2ServiceCall,
    ROS2TopicSubscribe,
    ROS2TopicPublish,
    ROS2GetMessageFields,
    ROS2GetMessages,
    get_ros,
)
import json
import rclpy


# Test get_ros singleton
@patch("server.tools_ros2.ros2_manager.ROS2Manager")
@patch("rclpy.ok")
def test_get_ros_creates_instance(mock_rclpy_ok, mock_ros2_manager):
    import server.tools_ros2
    server.tools_ros2._ros_instance = None
    
    mock_rclpy_ok.return_value = True
    mock_manager = MagicMock()
    mock_manager.node.context.ok.return_value = True
    mock_ros2_manager.return_value = mock_manager
    
    result = get_ros()
    
    assert result is not None
    mock_ros2_manager.assert_called_once()


@patch("rclpy.ok")
def test_get_ros_raises_if_not_initialized(mock_rclpy_ok):
    import server.tools_ros2
    server.tools_ros2._ros_instance = None
    
    mock_rclpy_ok.return_value = False
    
    try:
        get_ros()
        assert False, "Should have raised RuntimeError"
    except RuntimeError as e:
        assert "rclpy is not initialized" in str(e)


# Test ROS2TopicList
@patch("server.tools_ros2.get_ros")
def test_ros2_topic_list_tool_description(mock_get_ros):
    tool = ROS2TopicList()
    desc = tool.get_tool_description()
    
    assert desc.name == "ros2_topic_list"
    assert "topics" in desc.description.lower()


@patch("server.tools_ros2.get_ros")
def test_ros2_topic_list_run_tool(mock_get_ros):
    mock_ros = MagicMock()
    mock_ros.list_topics.return_value = [
        {"topic_name": "/test", "topic_type": "std_msgs/msg/String", "request_fields": {"data": "string"}}
    ]
    mock_get_ros.return_value = mock_ros
    
    tool = ROS2TopicList()
    result = tool.run_tool({})
    
    assert len(result) == 1
    assert result[0].type == "text"
    data = json.loads(result[0].text)
    assert data[0]["topic_name"] == "/test"


# Test ROS2ServiceList
@patch("server.tools_ros2.get_ros")
def test_ros2_service_list_tool_description(mock_get_ros):
    tool = ROS2ServiceList()
    desc = tool.get_tool_description()
    
    assert desc.name == "ros2_service_list"
    assert "services" in desc.description.lower()


@patch("server.tools_ros2.get_ros")
def test_ros2_service_list_run_tool(mock_get_ros):
    mock_ros = MagicMock()
    mock_ros.list_services.return_value = [
        {"service_name": "/add", "service_type": "example_interfaces/srv/AddTwoInts", "request_fields": {"a": "int64", "b": "int64"}}
    ]
    mock_get_ros.return_value = mock_ros
    
    tool = ROS2ServiceList()
    result = tool.run_tool({})
    
    assert len(result) == 1
    assert result[0].type == "text"
    data = json.loads(result[0].text)
    assert data[0]["service_name"] == "/add"


# Test ROS2InterfaceList
@patch("server.tools_ros2.get_ros")
def test_ros2_interface_list_tool_description(mock_get_ros):
    tool = ROS2InterfaceList()
    desc = tool.get_tool_description()
    
    assert desc.name == "ros2_interface_list"
    assert "interfaces" in desc.description.lower()


@patch("server.tools_ros2.get_ros")
def test_ros2_interface_list_run_tool(mock_get_ros):
    mock_ros = MagicMock()
    mock_ros.list_interfaces.return_value = ["std_msgs/msg/String", "std_msgs/msg/Int32"]
    mock_get_ros.return_value = mock_ros
    
    tool = ROS2InterfaceList()
    result = tool.run_tool({})
    
    assert len(result) == 1
    assert result[0].type == "text"
    data = json.loads(result[0].text)
    assert "std_msgs/msg/String" in data


# Test ROS2ServiceCall
@patch("server.tools_ros2.get_ros")
def test_ros2_service_call_tool_description(mock_get_ros):
    tool = ROS2ServiceCall()
    desc = tool.get_tool_description()
    
    assert desc.name == "ros2_service_call"
    assert "service" in desc.description.lower()
    assert "service_name" in desc.inputSchema["properties"]
    assert "service_type" in desc.inputSchema["properties"]
    assert "fields" in desc.inputSchema["properties"]


@patch("server.tools_ros2.get_ros")
def test_ros2_service_call_service_not_available(mock_get_ros):
    mock_ros = MagicMock()
    mock_ros.list_services.return_value = [{"service_name": "/other", "service_type": "test"}]
    mock_get_ros.return_value = mock_ros
    
    tool = ROS2ServiceCall()
    result = tool.run_tool({
        "service_name": "/not_exist",
        "service_type": "example_interfaces/srv/AddTwoInts",
        "fields": {}
    })
    
    # Should return error dict directly
    assert "error" in result


@patch("server.tools_ros2.get_ros")
def test_ros2_service_call_missing_fields_no_force(mock_get_ros):
    mock_ros = MagicMock()
    mock_ros.list_services.return_value = [
        {"service_name": "/add", "service_type": "example_interfaces/srv/AddTwoInts", "request_fields": {}}
    ]
    mock_ros.get_request_fields.return_value = {"a": "int64", "b": "int64"}
    mock_get_ros.return_value = mock_ros
    
    tool = ROS2ServiceCall()
    result = tool.run_tool({
        "service_name": "/add",
        "service_type": "example_interfaces/srv/AddTwoInts",
        "fields": {"a": 5},
        "force_call": False
    })
    
    assert len(result) == 1
    text = result[0].text
    assert "missing fields" in text.lower()


@patch("server.tools_ros2.get_ros")
def test_ros2_service_call_success(mock_get_ros):
    mock_ros = MagicMock()
    mock_ros.list_services.return_value = [
        {"service_name": "/add", "service_type": "example_interfaces/srv/AddTwoInts", "request_fields": {}}
    ]
    mock_ros.get_request_fields.return_value = {"a": "int64", "b": "int64"}
    mock_ros.call_service.return_value = {"result": "sum: 8"}
    mock_get_ros.return_value = mock_ros
    
    tool = ROS2ServiceCall()
    result = tool.run_tool({
        "service_name": "/add",
        "service_type": "example_interfaces/srv/AddTwoInts",
        "fields": {"a": 5, "b": 3}
    })
    
    assert len(result) == 1
    data = json.loads(result[0].text)
    assert "result" in data


@patch("server.tools_ros2.get_ros")
def test_ros2_service_call_error_in_request_fields(mock_get_ros):
    mock_ros = MagicMock()
    mock_ros.list_services.return_value = [
        {"service_name": "/test", "service_type": "test", "request_fields": {}}
    ]
    mock_ros.get_request_fields.return_value = {"error": "Invalid type"}
    mock_get_ros.return_value = mock_ros
    
    tool = ROS2ServiceCall()
    result = tool.run_tool({
        "service_name": "/test",
        "service_type": "invalid/type",
        "fields": {}
    })
    
    assert "error" in result


# Test ROS2TopicSubscribe
@patch("server.tools_ros2.get_ros")
def test_ros2_topic_subscribe_tool_description(mock_get_ros):
    tool = ROS2TopicSubscribe()
    desc = tool.get_tool_description()
    
    assert desc.name == "ros2_topic_subscribe"
    assert "subscribe" in desc.description.lower()
    assert "topic_name" in desc.inputSchema["properties"]


@patch("server.tools_ros2.get_ros")
def test_ros2_topic_subscribe_with_duration(mock_get_ros):
    mock_ros = MagicMock()
    mock_ros.subscribe_topic.return_value = {
        "messages": [{"data": "test"}],
        "count": 1,
        "duration": 2.0
    }
    mock_get_ros.return_value = mock_ros
    
    tool = ROS2TopicSubscribe()
    result = tool.run_tool({
        "topic_name": "/chatter",
        "duration": 2.0,
        "message_limit": None
    })
    
    assert len(result) == 2
    assert "[/chatter] 1 messages received." in result[0].text
    data = json.loads(result[1].text)
    assert data["/chatter#0"]["data"] == "test"


@patch("server.tools_ros2.get_ros")
def test_ros2_topic_subscribe_with_message_limit(mock_get_ros):
    mock_ros = MagicMock()
    mock_ros.subscribe_topic.return_value = {
        "messages": [{"data": "msg1"}, {"data": "msg2"}],
        "count": 2,
        "duration": 0.5
    }
    mock_get_ros.return_value = mock_ros
    
    tool = ROS2TopicSubscribe()
    result = tool.run_tool({
        "topic_name": "/chatter",
        "duration": None,
        "message_limit": 2
    })
    
    assert len(result) == 3
    assert "[/chatter] 2 messages received." in result[0].text
    data1 = json.loads(result[1].text)
    assert data1["/chatter#0"]["data"] == "msg1"
    data2 = json.loads(result[2].text)
    assert data2["/chatter#1"]["data"] == "msg2"


@patch("server.tools_ros2.get_ros")
def test_ros2_topic_subscribe_empty_strings_converted_to_none(mock_get_ros):
    mock_ros = MagicMock()
    mock_ros.subscribe_topic.return_value = {"messages": [], "count": 0, "duration": 5.0}
    mock_get_ros.return_value = mock_ros
    
    tool = ROS2TopicSubscribe()
    result = tool.run_tool({
        "topic_name": "/test",
        "duration": "",
        "message_limit": ""
    })
    
    # Verify that subscribe_topic was called with None for empty strings
    mock_ros.subscribe_topic.assert_called_once_with("/test", None, None)


# Test ROS2TopicPublish
@patch("server.tools_ros2.get_ros")
def test_ros2_topic_publish_tool_description(mock_get_ros):
    tool = ROS2TopicPublish()
    desc = tool.get_tool_description()
    
    assert desc.name == "ros2_topic_publish"
    assert "publish" in desc.description.lower()
    assert "topic_name" in desc.inputSchema["properties"]
    assert "message_type" in desc.inputSchema["properties"]
    assert "data" in desc.inputSchema["properties"]


@patch("server.tools_ros2.get_ros")
def test_ros2_topic_publish_success(mock_get_ros):
    mock_ros = MagicMock()
    mock_ros.publish_to_topic.return_value = {"status": "published", "data": {"data": "hello"}}
    mock_get_ros.return_value = mock_ros
    
    tool = ROS2TopicPublish()
    result = tool.run_tool({
        "topic_name": "/chatter",
        "message_type": "std_msgs/msg/String",
        "data": {"data": "hello"}
    })
    
    assert len(result) == 1
    data = json.loads(result[0].text)
    assert data["status"] == "published"


@patch("server.tools_ros2.get_ros")
def test_ros2_topic_publish_error(mock_get_ros):
    mock_ros = MagicMock()
    mock_ros.publish_to_topic.return_value = {"error": "Invalid message type"}
    mock_get_ros.return_value = mock_ros
    
    tool = ROS2TopicPublish()
    result = tool.run_tool({
        "topic_name": "/test",
        "message_type": "invalid/type",
        "data": {}
    })
    
    assert len(result) == 1
    data = json.loads(result[0].text)
    assert "error" in data


# Test ROS2GetMessageFields
@patch("server.tools_ros2.get_ros")
def test_ros2_get_message_fields_tool_description(mock_get_ros):
    tool = ROS2GetMessageFields()
    desc = tool.get_tool_description()
    
    assert desc.name == "ros2_get_message_fields"
    assert "fields" in desc.description.lower()
    assert "message_type" in desc.inputSchema["properties"]


@patch("server.tools_ros2.get_ros")
def test_ros2_get_message_fields_success(mock_get_ros):
    mock_ros = MagicMock()
    mock_ros.get_request_fields.return_value = {"data": "string"}
    mock_get_ros.return_value = mock_ros
    
    tool = ROS2GetMessageFields()
    result = tool.run_tool({
        "message_type": "std_msgs/msg/String"
    })
    
    assert len(result) == 1
    data = json.loads(result[0].text)
    assert data["data"] == "string"


@patch("server.tools_ros2.get_ros")
def test_ros2_get_message_fields_error(mock_get_ros):
    mock_ros = MagicMock()
    mock_ros.get_request_fields.return_value = {"error": "Invalid type"}
    mock_get_ros.return_value = mock_ros
    
    tool = ROS2GetMessageFields()
    result = tool.run_tool({
        "message_type": "invalid"
    })
    
    assert len(result) == 1
    data = json.loads(result[0].text)
    assert "error" in data


# Test ROS2GetMessages
@patch("server.tools_ros2.get_ros")
def test_ros2_get_messages_tool_description(mock_get_ros):
    tool = ROS2GetMessages()
    desc = tool.get_tool_description()
    
    assert desc.name == "ros2_get_messages_stored_in_influx_data_base"
    assert "influxdb" in desc.description.lower()
    assert "topic_name" in desc.inputSchema["properties"]
    assert "message_type" in desc.inputSchema["properties"]


@patch("server.tools_ros2.get_ros")
def test_ros2_get_messages_success(mock_get_ros):
    mock_ros = MagicMock()
    mock_ros.call_get_messages_service_any.return_value = {
        "messages": [{"data": "test"}],
        "timestamps": [{"sec": 12345}]
    }
    mock_get_ros.return_value = mock_ros
    
    tool = ROS2GetMessages()
    result = tool.run_tool({
        "topic_name": "/sensor",
        "message_type": "std_msgs/msg/String",
        "number_of_messages": 10,
        "time_start": None,
        "time_end": None
    })
    
    assert len(result) == 1
    data = json.loads(result[0].text)
    assert "messages" in data


@patch("server.tools_ros2.get_ros")
def test_ros2_get_messages_with_time_filters(mock_get_ros):
    mock_ros = MagicMock()
    mock_ros.call_get_messages_service_any.return_value = {"messages": [], "timestamps": []}
    mock_get_ros.return_value = mock_ros
    
    tool = ROS2GetMessages()
    result = tool.run_tool({
        "topic_name": "/sensor",
        "message_type": "std_msgs/msg/String",
        "number_of_messages": 5,
        "time_start": "2025-01-01T00:00:00Z",
        "time_end": "2025-01-02T00:00:00Z"
    })
    
    assert len(result) == 1
    mock_ros.call_get_messages_service_any.assert_called_once()
    call_args = mock_ros.call_get_messages_service_any.call_args[0][0]
    assert call_args["time_start"] == "2025-01-01T00:00:00Z"
    assert call_args["time_end"] == "2025-01-02T00:00:00Z"
