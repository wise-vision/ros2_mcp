#
#  Copyright (C) 2025 wisevision
#
#  SPDX-License-Identifier: MPL-2.0
#
#  This Source Code Form is subject to the terms of the Mozilla Public
#  License, v. 2.0. If a copy of the MPL was not distributed with this
#  file, You can obtain one at https://mozilla.org/MPL/2.0/.
#
from .tools_ros2 import get_ros
from .ros2_manager_extended import ExtendedROS2Manager, TopicSubscriptionConfig

from collections.abc import Sequence
from mcp.types import (
    Tool,
    TextContent,
    ImageContent,
    EmbeddedResource,
    LoggingLevel,
)

import json
from .toolhandler import ToolHandler

_ros_extended: ExtendedROS2Manager | None = None


def get_ros_extended() -> ExtendedROS2Manager:
    global _ros_extended
    if _ros_extended is None or not _ros_extended.base_manager.node.context.ok():
        _ros_extended = ExtendedROS2Manager(base_manager=get_ros())
    return _ros_extended

class SubscribeMultipleTopicsTool(ToolHandler):
    def __init__(self):
        super().__init__("ros2_subscribe_multiple_topics")

    def get_tool_description(self):
        return Tool(
            name=self.name,
            description="Subscribe to multiple ROS 2 topics at once and return received messages.",
            inputSchema={
                "type": "object",
                "properties": {
                    "topics": {
                        "type": "array",
                        "description": "List of topics to subscribe to.",
                        "items": {
                            "type": "object",
                            "properties": {
                                "name": {"type": "string"},
                                "duration": {
                                    "type": "number",
                                    "description": "How many seconds to listen (optional).",
                                    "default": 0.0,
                                },
                                "message_limit": {
                                    "type": "integer",
                                    "description": "Number of messages to collect (optional).",
                                    "default": 0,
                                },
                            },
                            "required": ["name"],
                        },
                    }
                },
                "required": ["topics"],
            },
        )

    def run_tool(self, args: dict) -> Sequence[TextContent | ImageContent]:
        topics_arg = args.get("topics", [])
        configs: list[TopicSubscriptionConfig] = [
            TopicSubscriptionConfig(
                name=t.get("name"),
                duration=t.get("duration"),
                message_limit=t.get("message_limit"),
            )
            for t in topics_arg
        ]

        ros = get_ros_extended()
        result = ros.subscribe_multiple_topics(configs)

        outputs: list[TextContent | ImageContent] = []

        for topic_name, topic_data in result.items():
            if "error" in topic_data:
                outputs.append(
                    TextContent(
                        type="text", text=f"[{topic_name}] ERROR: {topic_data['error']}"
                    )
                )
                continue

            count = topic_data.get("count", 0)
            outputs.append(
                TextContent(
                    type="text", text=f"[{topic_name}] {count} messages received."
                )
            )

            messages = topic_data.get("messages", [])
            for i, msg in enumerate(messages):
                if (
                    isinstance(msg, dict)
                    and msg.get("type") == "image"
                    and "data" in msg
                    and "mimeType" in msg
                ):
                    outputs.append(
                        ImageContent(
                            type="image", data=msg["data"], mimeType=msg["mimeType"]
                        )
                    )
                else:
                    formatted = json.dumps({f"{topic_name}#{i}": msg}, indent=2)
                    outputs.append(TextContent(type="text", text=formatted))

        return outputs


class PublishMultipleTopicsTool(ToolHandler):
    def __init__(self):
        super().__init__("ros2_publish_multiple_topics")

    def get_tool_description(self):
        return Tool(
            name=self.name,
            description="Publish messages to multiple ROS 2 topics simultaneously with optional frequency and duration.",
            inputSchema={
                "type": "object",
                "properties": {
                    "topics": {
                        "type": "array",
                        "description": "List of topics to publish to.",
                        "items": {
                            "type": "object",
                            "properties": {
                                "topic_name": {"type": "string"},
                                "message_type": {"type": "string"},
                                "data": {
                                    "type": "object",
                                    "description": "Message content as dictionary",
                                },
                                "frequency": {
                                    "type": "number",
                                    "description": "Publishing frequency in Hz (optional)",
                                    "default": 1.0,
                                },
                                "duration": {
                                    "type": "number",
                                    "description": "Duration in seconds to publish (optional)",
                                    "default": 5.0,
                                },
                            },
                            "required": ["topic_name", "message_type", "data"],
                        },
                    }
                },
                "required": ["topics"],
            },
        )

    def run_tool(self, args: dict) -> Sequence[TextContent]:
        topics_arg = args.get("topics", [])

        ros = get_ros_extended()
        result = ros.publish_multiple_topics(topics_arg)

        outputs: list[TextContent] = []

        for topic_name, status in result.items():
            outputs.append(TextContent(type="text", text=f"[{topic_name}] {status}"))

        return outputs


class GetMapAsImage(ToolHandler):
    def __init__(self):
        super().__init__("ros2_get_map_as_image")

    def get_tool_description(self):
        return Tool(
            name=self.name,
            description=(
                "Get one nav_msgs/msg/OccupancyGrid message from a topic and return it as a PNG image "
                "(base64-encoded). Unknown cells rendered as gray, free as white, occupied as black."
            ),
            inputSchema={
                "type": "object",
                "properties": {
                    "topic_name": {
                        "type": "string",
                        "description": "The name of the ROS 2 topic publishing nav_msgs/msg/OccupancyGrid (e.g., /map).",
                    },
                },
                "required": ["topic_name"],
            },
        )

    def run_tool(self, args: dict) -> Sequence[TextContent, ImageContent]:
        topic_name = args.get("topic_name")

        ros = get_ros_extended()

        map_img_content = ros.get_single_map_from_topic(topic_name)

        return [
            TextContent(
                type="text", text=json.dumps("Here's your map (PNG):", indent=2)
            ),
            map_img_content,
        ]


class GetPointCloudAsBEV(ToolHandler):
    def __init__(self):
        super().__init__("ros2_get_pointcloud_as_bev")

    def get_tool_description(self):
        return Tool(
            name=self.name,
            description=(
                "Get one sensor_msgs/msg/PointCloud2 message from a topic and return it as a PNG image "
                "(base64-encoded) rendered as a bird's-eye view (XY projection). "
                "Supports coloring by intensity/height/rgb."
            ),
            inputSchema={
                "type": "object",
                "properties": {
                    "topic_name": {
                        "type": "string",
                        "description": "ROS 2 topic with PointCloud2 (e.g., /points).",
                    },
                    "resolution": {
                        "type": "number",
                        "description": "Meters per pixel for the BEV image.",
                        "default": 0.05,
                    },
                    "zmin": {
                        "type": "number",
                        "description": "Min Z to include (meters).",
                    },
                    "zmax": {
                        "type": "number",
                        "description": "Max Z to include (meters).",
                    },
                    "timeout": {
                        "type": "number",
                        "description": "Seconds to wait for a message.",
                        "default": 5.0,
                    },
                    "max_pixels": {
                        "type": "integer",
                        "description": "Max image size per axis (to cap huge clouds).",
                        "default": 2048,
                    },
                    "color_mode": {
                        "type": "string",
                        "description": "Coloring mode: 'intensity' | 'height' | 'rgb'.",
                        "enum": ["intensity", "height", "rgb"],
                        "default": "intensity",
                    },
                    "colormap": {
                        "type": "string",
                        "description": "Colormap for intensity/height: 'jet' | 'gray'.",
                        "enum": ["jet", "gray"],
                        "default": "jet",
                    },
                },
                "required": ["topic_name"],
            },
        )

    def run_tool(self, args: dict) -> Sequence[TextContent, ImageContent]:
        topic_name = args.get("topic_name")
        resolution = float(args.get("resolution", 0.05))
        timeout = float(args.get("timeout", 5.0))
        max_pixels = int(args.get("max_pixels", 2048))
        color_mode = str(args.get("color_mode", "intensity")).lower()
        colormap = str(args.get("colormap", "jet")).lower()

        zmin = args.get("zmin")
        zmax = args.get("zmax")
        z_filter = (
            (float(zmin), float(zmax))
            if (zmin is not None and zmax is not None)
            else None
        )

        ros = get_ros_extended()

        bev_img_content = ros.get_single_bev_from_pointcloud(
            topic_name=topic_name,
            timeout=timeout,
            resolution=resolution,
            z_filter=z_filter,
            max_pixels=max_pixels,
            color_mode=color_mode,
            colormap=colormap,
        )

        return [
            TextContent(
                type="text",
                text=json.dumps("Here's your point cloud BEV (PNG):", indent=2),
            ),
            bev_img_content,
        ]
