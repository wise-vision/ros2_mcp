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
from typing import Any, Sequence
from mcp.server import Server
from mcp.types import (
    Tool,
    TextContent,
    ImageContent,
    EmbeddedResource,
    Prompt,
    GetPromptResult,
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

# Prompts
from . import prompthandler
from . import prompts_ros2

# Prompts handlers
_prompt_handlers: dict[str, "prompthandler.BasePromptHandler"] = {}


def add_prompt_handler(handler: "prompthandler.BasePromptHandler") -> None:
    if handler.name in _prompt_handlers:
        raise ValueError(f"Prompt already registered: {handler.name}")
    _prompt_handlers[handler.name] = handler


def get_prompt_handler(name: str) -> "prompthandler.BasePromptHandler | None":
    return _prompt_handlers.get(name)


def list_prompt_handlers() -> list["prompthandler.BasePromptHandler"]:
    return list(_prompt_handlers.values())

# Place for adding prompts
add_prompt_handler(prompts_ros2.DroneMissionWithMAVROS2Prompt())
add_prompt_handler(prompts_ros2.Nav2NavigateToPosePrompt())
add_prompt_handler(prompts_ros2.DroneSimpleTakeoffPrompt())
add_prompt_handler(prompts_ros2.DroneCircleFlightPrompt())

# Functions to handle list and getting prompts
@app.list_prompts()
async def handle_list_prompts() -> list[Prompt]:
    return [ph.get_prompt_description() for ph in list_prompt_handlers()]

@app.get_prompt()
async def handle_get_prompt(
    name: str, arguments: dict[str, str] | None
) -> GetPromptResult:
    try:
        ph = get_prompt_handler(name)
        if not ph:
            raise ValueError(f"Unknown prompt: {name}")
        return ph.render(arguments)
    except Exception as e:
        logging.error(traceback.format_exc())
        logging.error(f"Error during get_prompt: {e}")
        raise RuntimeError(f"Caught Exception. Error: {str(e)}")
    