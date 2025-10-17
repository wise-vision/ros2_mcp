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
import logging
import traceback
from typing import Any
from collections.abc import Sequence
from mcp.server import Server
from mcp.types import (
    Tool,
    TextContent,
    ImageContent,
    EmbeddedResource,
)

from . import toolhandler
from . import tools_ros2


app = Server("mcp-ros2-server")

tool_handlers = {}


def add_tool_handler(tool_class: toolhandler.ToolHandler):
    global tool_handlers

    tool_handlers[tool_class.name] = tool_class


def get_tool_handler(name: str) -> toolhandler.ToolHandler | None:
    if name not in tool_handlers:
        return None

    return tool_handlers[name]


add_tool_handler(tools_ros2.ROS2TopicList())
add_tool_handler(tools_ros2.ROS2ServiceList())
add_tool_handler(tools_ros2.ROS2InterfaceList())
add_tool_handler(tools_ros2.ROS2ServiceCall())
add_tool_handler(tools_ros2.ROS2TopicSubscribe())
add_tool_handler(tools_ros2.ROS2GetMessages())
add_tool_handler(tools_ros2.ROS2GetMessageFields())
add_tool_handler(tools_ros2.ROS2TopicPublish())
add_tool_handler(tools_ros2.ROS2ListActions())
add_tool_handler(tools_ros2.ROS2SendActionGoal())
add_tool_handler(tools_ros2.ROS2CancelActionGoal())
add_tool_handler(tools_ros2.ROS2ActionRequestResult())
add_tool_handler(tools_ros2.ROS2ActionSubscribeFeedback())
add_tool_handler(tools_ros2.ROS2ActionSubscribeStatus())

@app.list_tools()
async def list_tools() -> list[Tool]:
    """List available tools."""

    return [th.get_tool_description() for th in tool_handlers.values()]


@app.call_tool()
async def call_tool(
    name: str, arguments: Any
) -> Sequence[TextContent | ImageContent | EmbeddedResource]:
    try:
        if not isinstance(arguments, dict):
            raise RuntimeError("arguments must be dictionary")

        tool_handler = get_tool_handler(name)
        if not tool_handler:
            raise ValueError(f"Unknown tool: {name}")

        return tool_handler.run_tool(arguments)
    except Exception as e:
        logging.error(traceback.format_exc())
        logging.error(f"Error during call_tool: {e}")
        raise RuntimeError(f"Caught Exception. Error: {str(e)}")
