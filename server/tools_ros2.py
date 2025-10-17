#
#  Copyright (C) 2025 wisevision
#
#  SPDX-License-Identifier: MPL-2.0
#
#  This Source Code Form is subject to the terms of the Mozilla Public
#  License, v. 2.0. If a copy of the MPL was not distributed with this
#  file, You can obtain one at https://mozilla.org/MPL/2.0/.
#
from collections.abc import Sequence
from typing import Optional
from mcp.types import (
    Tool,
    TextContent,
    EmbeddedResource,
    LoggingLevel,
)

import json
from . import toolhandler
from . import ros2_manager

_ros_instance = None


def get_ros() -> ros2_manager.ROS2Manager:
    global _ros_instance
    if _ros_instance is None or not _ros_instance.node.context.ok():
        import rclpy

        if not rclpy.ok():
            raise RuntimeError(
                "rclpy is not initialized. Make sure rclpy.init() was called in main()."
            )
        _ros_instance = ros2_manager.ROS2Manager()
    return _ros_instance


class ROS2TopicList(toolhandler.ToolHandler):
    def __init__(self):
        super().__init__("ros2_topic_list")

    def get_tool_description(self):
        return Tool(
            name=self.name,
            inputSchema={"type": "object", "properties": {}},
            description="""Returns a list of available ROS 2 topics and their types.""",
        )

    def run_tool(self, args: dict) -> Sequence[TextContent | EmbeddedResource]:

        ros = get_ros()
        topics_list_with_types = ros.list_topics()

        return [
            TextContent(type="text", text=json.dumps(topics_list_with_types, indent=2))
        ]


class ROS2ServiceList(toolhandler.ToolHandler):
    def __init__(self):
        super().__init__("ros2_service_list")

    def get_tool_description(self):
        return Tool(
            name=self.name,
            inputSchema={"type": "object", "properties": {}},
            description="""Returns a list of available ROS 2 services and their request fields.""",
        )

    def run_tool(self, args: dict) -> Sequence[TextContent | EmbeddedResource]:

        ros = get_ros()
        service_list_with_types = ros.list_services()

        return [
            TextContent(type="text", text=json.dumps(service_list_with_types, indent=2))
        ]


class ROS2InterfaceList(toolhandler.ToolHandler):
    def __init__(self):
        super().__init__("ros2_interface_list")

    def get_tool_description(self):
        return Tool(
            name=self.name,
            inputSchema={"type": "object", "properties": {}},
            description="""Returns a list of available ROS 2 interfaces.""",
        )

    def run_tool(self, args: dict) -> Sequence[TextContent | EmbeddedResource]:

        ros = get_ros()
        interfaces_list = ros.list_interfaces()

        return [TextContent(type="text", text=json.dumps(interfaces_list, indent=2))]


class ROS2ServiceCall(toolhandler.ToolHandler):
    def __init__(self):
        super().__init__("ros2_service_call")

    def get_tool_description(self):
        return Tool(
            name=self.name,
            description="""Call a ROS 2 service by name and type using provided fields.
            Will ask the user to confirm if some fields are missing unless 'force_call' is set to True.
            Before **every** use of this tool, the agent must call 'ros2 service list' and 'ros2 interface list' to ensure the latest interface information is available.""",
            inputSchema={
                "type": "object",
                "properties": {
                    "service_name": {
                        "type": "string",
                        "description": "Name of the service to call",
                    },
                    "service_type": {
                        "type": "string",
                        "description": "Full ROS 2 service type, before pass, check service type using tool ros2_service_list",
                    },
                    "fields": {
                        "type": "object",
                        "description": "Dictionary of fields to send in the service request.",
                    },
                    "force_call": {
                        "type": "boolean",
                        "description": "Whether to call the service even if some fields are missing",
                        "default": False,
                    },
                },
                "required": ["service_name", "service_type", "fields"],
            },
        )

    def run_tool(self, args: dict) -> Sequence[TextContent | EmbeddedResource]:

        service_name = args.get("service_name")
        service_type = args.get("service_type")
        fields = args.get("fields")
        force_call = args.get("force_call")

        ros = get_ros()
        # Check if service exists
        available_services = [srv["service_name"] for srv in ros.list_services()]
        if service_name not in available_services:
            return {"error": f"Service '{service_name}' is not available."}
        # Get required fields
        required_fields = ros.get_request_fields(service_type)
        if "error" in required_fields:
            return {"error": required_fields["error"]}

        missing_fields = [key for key in required_fields if key not in fields]

        if missing_fields and not force_call:
            message = f"You're missing fields: {missing_fields}. "
            message += "Would you like to call the service anyway (set 'force_call' = true) or add more inputs?"
            return [TextContent(type="text", text=json.dumps(message, indent=2))]
        response = ros.call_service(service_name, service_type, fields)

        return [TextContent(type="text", text=json.dumps(response, indent=2))]


class ROS2TopicSubscribe(toolhandler.ToolHandler):
    def __init__(self):
        super().__init__("ros2_topic_subscribe")

    def get_tool_description(self):
        return Tool(
            name=self.name,
            description="""Subscribe to a ROS 2 topic by name collecting messages for a given time or count limit.
            Before **every** use of this tool, the agent must call 'ros2_topic_list'
            to ensure it has the latest available topics""",
            inputSchema={
                "type": "object",
                "properties": {
                    "topic_name": {
                        "type": "string",
                        "description": "Name of the topic to subscribe to",
                    },
                    "duration": {
                        "type": "number",
                        "description": "If provided, collects messages for this many seconds.",
                    },
                    "message_limit": {
                        "type": "integer",
                        "description": "If provided, stops after receiving this number of messages.",
                    },
                },
                "required": ["topic_name", "message_type"],
            },
        )

    def run_tool(self, args: dict) -> Sequence[TextContent | EmbeddedResource]:

        topic_name = args.get("topic_name")
        duration = args.get("duration")
        message_limit = args.get("message_limit")
        if duration == "":
            duration = None
        if message_limit == "":
            message_limit = None

        ros = get_ros()
        messages = ros.subscribe_topic(
            topic_name, duration, message_limit
        )

        return [TextContent(type="text", text=json.dumps(messages, indent=2))]


# Legacy support for wisevision_data_black_box
class ROS2GetMessages(toolhandler.ToolHandler):
    def __init__(self):
        super().__init__("ros2_get_messages_stored_in_influx_data_base")

    def get_tool_description(self):
        return Tool(
            name=self.name,
            description="""Calls the ROS2 ‘/get_messages’ service to retrieve past messages from a topic for data which is stored in InfluxDB.
            Check if the /get_messages service is available before calling.
            """,
            inputSchema={
                "type": "object",
                "properties": {
                    "topic_name": {
                        "type": "string",
                        "description": "Name of the topic to retrieve messages from.",
                    },
                    "message_type": {
                        "type": "string",
                        "description": "Full ROS2 message type used for decoding",
                    },
                    "number_of_messages": {
                        "type": "integer",
                        "description": "Number of messages to fetch.",
                        "default": 0,
                    },
                    "time_start": {
                        "type": "string",
                        "description": "ISO8601 timestamp string to filter messages after a point in time.",
                    },
                    "time_end": {
                        "type": "string",
                        "description": "ISO8601 timestamp string to filter messages before a point in time.",
                    },
                },
                "required": ["topic_name", "message_type"],
            },
        )

    def run_tool(self, args: dict) -> Sequence[TextContent | EmbeddedResource]:

        topic_name = args.get("topic_name")
        message_type = args.get("message_type")
        number_of_messages = args.get("number_of_messages")
        time_start = args.get("time_start")
        time_end = args.get("time_end")

        ros = get_ros()
        response = ros.call_get_messages_service_any(
            {
                "topic_name": topic_name,
                "message_type": message_type,
                "number_of_msgs": number_of_messages,
                "time_start": time_start,
                "time_end": time_end,
            }
        )
        return [TextContent(type="text", text=json.dumps(response, indent=2))]


class ROS2GetMessageFields(toolhandler.ToolHandler):
    def __init__(self):
        super().__init__("ros2_get_message_fields")

    def get_tool_description(self):
        return Tool(
            name=self.name,
            description="Returns the fields of a given ROS2 message type.",
            inputSchema={
                "type": "object",
                "properties": {
                    "message_type": {
                        "type": "string",
                        "description": "Full ROS2 message type, e.g., std_msgs/msg/String",
                    },
                },
                "required": ["message_type"],
            },
        )

    def run_tool(self, args: dict) -> Sequence[TextContent | EmbeddedResource]:

        message_type = args.get("message_type")

        ros = get_ros()
        request_fields = ros.get_request_fields(message_type)
        return [TextContent(type="text", text=json.dumps(request_fields, indent=2))]


class ROS2TopicPublish(toolhandler.ToolHandler):
    def __init__(self):
        super().__init__("ros2_topic_publish")

    def get_tool_description(self):
        return Tool(
            name=self.name,
            description="""Publish a message to a ROS 2 topic by name and message type using provided field values.
            Before **every** use of this tool, the agent must call 'ros2_topic_list' and 'ros2_interface_list'
            to ensure the latest available topics and message types are known.""",
            inputSchema={
                "type": "object",
                "properties": {
                    "topic_name": {
                        "type": "string",
                        "description": "Name of the topic to publish to",
                    },
                    "message_type": {
                        "type": "string",
                        "description": "Full ROS 2 message type, e.g., 'std_msgs/msg/String'",
                    },
                    "data": {
                        "type": "object",
                        "description": "Dictionary containing the message fields and values",
                    },
                },
                "required": ["topic_name", "message_type", "data"],
            },
        )

    def run_tool(self, args: dict) -> Sequence[TextContent | EmbeddedResource]:

        topic_name = args.get("topic_name")
        message_type = args.get("message_type")
        data = args.get("data")

        ros = get_ros()
        publish_to_topic = ros.publish_to_topic(topic_name, message_type, data)
        return [TextContent(type="text", text=json.dumps(publish_to_topic, indent=2))]

class ROS2ListActions(toolhandler.ToolHandler):
    def __init__(self):
        super().__init__("ros2_list_actions")

    def get_tool_description(self):
        return Tool(
            name=self.name,
            description="""List all available ROS 2 actions with their types and request fields.
            This tool queries the current ROS graph to discover active action servers and
            returns a structured description for each action.""",
            inputSchema={
                "type": "object",
                "properties": {},
                "additionalProperties": False,
            },
        )

    def run_tool(self, args: dict) -> Sequence[TextContent | EmbeddedResource]:
        ros = get_ros()
        result = ros.list_actions()
        return [TextContent(type="text", text=json.dumps(result, indent=2))]
    
class ROS2SendActionGoal(toolhandler.ToolHandler):
    def __init__(self):
        super().__init__("ros2_send_action_goal")

    def get_tool_description(self):
        return Tool(
            name=self.name,
            description=(
                "Send a goal to a ROS 2 action by name and action type using provided goal fields. "
                "Before **every** use of this tool, the agent must call 'ros2_list_actions' "
                "and 'ros2_interface_list' to ensure the latest available actions and types are known. "
                "Optionally wait for the result with a timeout."
            ),
            inputSchema={
                "type": "object",
                "properties": {
                    "action_name": {
                        "type": "string",
                        "description": "Action name, e.g. '/fibonacci' or '/navigate_to_pose'",
                    },
                    "action_type": {
                        "type": "string",
                        "description": "Full action type, e.g. 'example_interfaces/action/Fibonacci' or 'pkg/ActionName'",
                    },
                    "goal_fields": {
                        "type": "object",
                        "description": "Dictionary with goal message fields (1st section of .action file)",
                    },
                    "wait_for_result": {
                        "type": "boolean",
                        "description": "If true, wait for GetResult and include final status/result",
                        "default": False,
                    },
                    "timeout_sec": {
                        "type": "number",
                        "description": "Timeout (seconds) for waiting on the result when wait_for_result=true",
                        "default": 60.0,
                    },
                },
                "required": ["action_name", "action_type", "goal_fields"],
                "additionalProperties": False,
            },
        )

    def run_tool(self, args: dict) -> Sequence[TextContent | EmbeddedResource]:
        action_name = args.get("action_name")
        action_type = args.get("action_type")
        goal_fields = args.get("goal_fields") or {}
        wait_for_result = bool(args.get("wait_for_result", False))
        timeout_sec = float(args.get("timeout_sec", 60.0))

        ros = get_ros()
        resp = ros.send_action_goal(
            action_name=action_name,
            action_type=action_type,
            goal_fields=goal_fields,
            wait_for_result=wait_for_result,
            timeout_sec=timeout_sec,
        )
        return [TextContent(type="text", text=json.dumps(resp, indent=2))]
    
class ROS2CancelActionGoal(toolhandler.ToolHandler):
    def __init__(self):
        super().__init__("ros2_cancel_action_goal")

    def get_tool_description(self):
        return Tool(
            name=self.name,
            description=(
                "Cancel a ROS 2 action goal via '/<action>/cancel_goal' (action_msgs/srv/CancelGoal). "
                "You can cancel a specific goal by goal_id_hex or cancel all matching goals using cancel_all=True. "
                "Optionally limit cancellation to goals accepted before a given stamp (sec/nanosec). "
                "Before **every** use of this tool, the agent should ensure the target action exists "
                "(e.g., by calling 'ros2_list_actions')."
            ),
            inputSchema={
                "type": "object",
                "properties": {
                    "action_name": {
                        "type": "string",
                        "description": "Action name, e.g. '/fibonacci' or '/navigate_to_pose'",
                    },
                    "goal_id_hex": {
                        "type": "string",
                        "description": "32-char UUID hex of the goal (no dashes). Omit when cancel_all=True.",
                    },
                    "cancel_all": {
                        "type": "boolean",
                        "description": "If true, send zero-UUID to cancel all matching goals.",
                        "default": False,
                    },
                    "stamp_sec": {
                        "type": "integer",
                        "description": "Cancel goals accepted BEFORE this sec (default 0 → usually all).",
                        "default": 0,
                    },
                    "stamp_nanosec": {
                        "type": "integer",
                        "description": "Nanoseconds part for the acceptance time filter.",
                        "default": 0,
                    },
                    "wait_timeout_sec": {
                        "type": "number",
                        "description": "Timeout (seconds) to wait for the service and response.",
                        "default": 3.0,
                    },
                },
                "required": ["action_name"],
                "additionalProperties": False,
            },
        )

    def run_tool(self, args: dict) -> Sequence[TextContent | EmbeddedResource]:
        action_name = args.get("action_name")
        goal_id_hex = args.get("goal_id_hex")
        cancel_all = bool(args.get("cancel_all", False))
        stamp_sec = int(args.get("stamp_sec", 0))
        stamp_nanosec = int(args.get("stamp_nanosec", 0))
        wait_timeout_sec = float(args.get("wait_timeout_sec", 3.0))

        ros = get_ros()

        node = getattr(ros, "node", None) or getattr(ros, "get_node", lambda: None)()
        if node is None:
            return [TextContent(type="text", text=json.dumps({"error": "ROS node is not available."}, indent=2))]

        result = ros.cancel_action_goal(
            node=node,
            action_name=action_name,
            goal_id_hex=goal_id_hex,
            cancel_all=cancel_all,
            stamp_sec=stamp_sec,
            stamp_nanosec=stamp_nanosec,
            wait_timeout_sec=wait_timeout_sec,
        )
        return [TextContent(type="text", text=json.dumps(result, indent=2))]
    
class ROS2ActionRequestResult(toolhandler.ToolHandler):
    def __init__(self):
        super().__init__("ros2_action_request_result")

    def get_tool_description(self):
        return Tool(
            name=self.name,
            description=(
                "Wait for the RESULT of a ROS 2 action (GetResult) for a given goal_id. "
                "Before **every** use of this tool, the agent should ensure the target action exists "
                "(e.g., by calling 'ros2_list_actions')."
            ),
            inputSchema={
                "type": "object",
                "properties": {
                    "action_name": {
                        "type": "string",
                        "description": "Action name, e.g. '/fibonacci'",
                    },
                    "action_type": {
                        "type": "string",
                        "description": "Full action type, e.g. 'example_interfaces/action/Fibonacci' or 'pkg/ActionName'",
                    },
                    "goal_id_hex": {
                        "type": "string",
                        "description": "32-char UUID hex of the goal (no dashes).",
                    },
                    "timeout_sec": {
                        "description": "Seconds to wait for GetResult; null → wait indefinitely.",
                        "anyOf": [
                            {"type": "number"},
                            {"type": "null"}
                        ],
                        "default": 60.0,
                    },
                    "wait_for_service_sec": {
                        "type": "number",
                        "description": "Seconds to wait for the GetResult service to appear.",
                        "default": 3.0,
                    },
                },
                "required": ["action_name", "action_type", "goal_id_hex"],
                "additionalProperties": False,
            },
        )

    def run_tool(self, args: dict) -> Sequence[TextContent | EmbeddedResource]:
        action_name = args.get("action_name")
        action_type = args.get("action_type")
        goal_id_hex = args.get("goal_id_hex")
        timeout_sec = args.get("timeout_sec", 60.0)  # can be None
        wait_for_service_sec = float(args.get("wait_for_service_sec", 3.0))

        ros = get_ros()
        resp = ros.action_request_result(
            action_name=action_name,
            action_type=action_type,
            goal_id_hex=goal_id_hex,
            timeout_sec=timeout_sec,
            wait_for_service_sec=wait_for_service_sec,
        )
        return [TextContent(type="text", text=json.dumps(resp, indent=2))]
    
class ROS2ActionSubscribeFeedback(toolhandler.ToolHandler):
    def __init__(self):
        super().__init__("ros2_action_subscribe_feedback")

    def get_tool_description(self):
        return Tool(
            name=self.name,
            description=(
                "Subscribe to feedback messages of a ROS 2 action. "
                "Optionally filter by goal_id_hex. Collects feedback for a fixed duration "
                "or until a maximum number of messages is received. "
                "Each message contains goal_id, feedback payload, and receive timestamp."
            ),
            inputSchema={
                "type": "object",
                "properties": {
                    "action_name": {
                        "type": "string",
                        "description": "Action name, e.g. '/fibonacci'",
                    },
                    "action_type": {
                        "type": "string",
                        "description": "Full action type, e.g. 'example_interfaces/action/Fibonacci' or 'pkg/ActionName'",
                    },
                    "goal_id_hex": {
                        "type": ["string", "null"],
                        "description": "Optional 32-char UUID hex of the goal (no dashes) to filter feedbacks.",
                    },
                    "duration_sec": {
                        "type": "number",
                        "description": "How many seconds to keep spinning and collecting feedback.",
                        "default": 5.0,
                    },
                    "max_messages": {
                        "type": "integer",
                        "description": "Maximum number of feedback messages to collect.",
                        "default": 100,
                    },
                },
                "required": ["action_name", "action_type"],
                "additionalProperties": False,
            },
        )

    def run_tool(self, args: dict) -> Sequence[TextContent | EmbeddedResource]:
        action_name = args.get("action_name")
        action_type = args.get("action_type")
        goal_id_hex = args.get("goal_id_hex")
        duration_sec = float(args.get("duration_sec", 5.0))
        max_messages = int(args.get("max_messages", 100))

        ros = get_ros()
        resp = ros.action_subscribe_feedback(
            action_name=action_name,
            action_type=action_type,
            goal_id_hex=goal_id_hex,
            duration_sec=duration_sec,
            max_messages=max_messages,
        )
        return [TextContent(type="text", text=json.dumps(resp, indent=2))]
    
class ROS2ActionSubscribeStatus(toolhandler.ToolHandler):
    def __init__(self):
        super().__init__("ros2_action_subscribe_status")

    def get_tool_description(self):
        return Tool(
            name=self.name,
            description=(
                "Subscribe to '/<action>/_action/status' (action_msgs/msg/GoalStatusArray) "
                "and return a snapshot of status frames over a time window. "
                "Each frame contains goal_id, accept_stamp, status_code and status text."
            ),
            inputSchema={
                "type": "object",
                "properties": {
                    "action_name": {
                        "type": "string",
                        "description": "Action name, e.g. '/fibonacci'",
                    },
                    "duration_sec": {
                        "type": "number",
                        "description": "How many seconds to collect status frames.",
                        "default": 5.0,
                    },
                    "max_messages": {
                        "type": "integer",
                        "description": "Max number of individual statuses to collect in total.",
                        "default": 100,
                    },
                },
                "required": ["action_name"],
                "additionalProperties": False,
            },
        )

    def run_tool(self, args: dict) -> Sequence[TextContent | EmbeddedResource]:
        action_name = args.get("action_name")
        duration_sec = float(args.get("duration_sec", 5.0))
        max_messages = int(args.get("max_messages", 100))

        ros = get_ros()
        resp = ros.action_subscribe_status(
            action_name=action_name,
            duration_sec=duration_sec,
            max_messages=max_messages,
        )
        return [TextContent(type="text", text=json.dumps(resp, indent=2))]