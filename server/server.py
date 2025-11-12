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
import os, sys, pathlib, importlib
from dataclasses import dataclass
from importlib.metadata import entry_points
try:
    import tomllib as toml  # py311+
except (ModuleNotFoundError, ImportError):
    import tomli as toml     # py310 fallback
import argparse



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
    full_name = f"{handler.namespace}.{handler.name}"
    if full_name in _prompt_handlers:
        raise ValueError(f"Prompt already registered: {full_name}")
    _prompt_handlers[full_name] = handler


def get_prompt_handler(name: str) -> "prompthandler.BasePromptHandler | None":
    return _prompt_handlers.get(name)


def list_prompt_handlers() -> list["prompthandler.BasePromptHandler"]:
    return list(_prompt_handlers.values())

add_prompt_handler(prompts_ros2.ROS2NodeHealthCheckPrompt())
add_prompt_handler(prompts_ros2.ROS2TopicDiffMonitorPrompt())
add_prompt_handler(prompts_ros2.ROS2TopicEchoAndAnalyzePrompt())
add_prompt_handler(prompts_ros2.ROS2TopicRelayPrompt())


@dataclass
class Settings:
    custom_prompts: bool = False
    prompts_local: bool = False
    prompts_path: str = ""
    prompts_module: str = "extension_prompts"

def _read_pyproject() -> dict:
    p = pathlib.Path("pyproject.toml")
    if not p.exists():
        return {}
    with p.open("rb") as f:
        data = toml.load(f)
    return data.get("tool", {}).get("mcp_ros_2", {}) or {}

def _load_settings_from_pyproject() -> Settings:
    cfg = _read_pyproject()
    return Settings(
        custom_prompts=bool(cfg.get("custom_prompts", False)),
        prompts_local=bool(cfg.get("prompts_local", False)),
        prompts_path=str(cfg.get("prompts_path", "")),
        prompts_module=str(cfg.get("prompts_module", "extension_prompts")),
    )

def _apply_env_overrides(s: Settings) -> Settings:
    def _bool(envname: str, cur: bool) -> bool:
        val = os.getenv(envname)
        return cur if val is None else (val.lower() in ("1","true","yes","on"))
    s.custom_prompts = _bool("MCP_CUSTOM_PROMPTS", s.custom_prompts)
    s.prompts_local   = _bool("MCP_PROMPTS_LOCAL", s.prompts_local)
    s.prompts_path    = os.getenv("MCP_PROMPTS_PATH", s.prompts_path)
    s.prompts_module  = os.getenv("MCP_PROMPTS_MODULE", s.prompts_module)
    return s

def _parse_cli_overrides(s: Settings) -> Settings:
    def str2bool(v: str) -> bool:
        if isinstance(v, bool):
            return v
        v = v.lower()
        if v in ("yes", "true", "t", "1", "on"):
            return True
        elif v in ("no", "false", "f", "0", "off"):
            return False
        else:
            raise argparse.ArgumentTypeError(f"Boolean value expected, got '{v}'")

    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--custom-prompts", type=str2bool, default=None)
    parser.add_argument("--prompts-local", type=str2bool, default=None)
    parser.add_argument("--prompts-path", type=str, default=None)
    parser.add_argument("--prompts-module", type=str, default=None)
    args, _unknown = parser.parse_known_args()

    if args.custom_prompts is not None:
        s.custom_prompts = args.custom_prompts
    if args.prompts_local is not None:
        s.prompts_local = args.prompts_local
    if args.prompts_path:
        s.prompts_path = args.prompts_path
    if args.prompts_module:
        s.prompts_module = args.prompts_module
    return s

def _ensure_sys_path(path: str | None):
    if not path:
        return
    rp = str(pathlib.Path(path).expanduser().resolve())
    if rp not in sys.path:
        sys.path.insert(0, rp)

def _load_custom_prompts_via_entry_points() -> int:
    """Loading plugins installed as packages (Git/extras) via entry points."""
    count = 0
    for ep in entry_points().select(group="mcp_ros_2.prompts"):
        try:
            obj = ep.load()
            val = obj() if callable(obj) else obj
            if isinstance(val, list):
                for item in val:
                    add_prompt_handler(item)
                    count += 1
            else:
                add_prompt_handler(val)
                count += 1
            logging.info("Custom prompt EP loaded: %s", ep.name)
        except Exception as e:
            logging.warning("Failed to load EP '%s': %s", ep.name, e)
    return count

def _load_custom_prompts_from_local(module_name: str) -> int:
    """Loading from local folder (path given)."""
    mod = importlib.import_module(module_name)
    loaded = 0
    if hasattr(mod, "register_prompts"):
        for h in mod.register_prompts():
            add_prompt_handler(h); loaded += 1
    elif hasattr(mod, "ALL_PROMPTS"):
        for h in getattr(mod, "ALL_PROMPTS"):
            add_prompt_handler(h); loaded += 1
    elif hasattr(mod, "get_prompts"):
        for h in mod.get_prompts():
            add_prompt_handler(h); loaded += 1
    else:
        logging.info("Module '%s' imported, but no register_prompts/ALL_PROMPTS/get_prompts found.", module_name)
    return loaded

def _maybe_load_custom_prompts():
    s = _load_settings_from_pyproject()
    s = _apply_env_overrides(s)
    s = _parse_cli_overrides(s)

    if not s.custom_prompts:
        logging.info("Custom prompts: disabled.")
        return

    if s.prompts_local:
        if not s.prompts_path:
            logging.error("--prompts-local=true requires 'prompts_path' to be set.")
            return
        _ensure_sys_path(s.prompts_path)
        try:
            n = _load_custom_prompts_from_local(s.prompts_module)
            logging.info("Custom prompts (LOCAL) loaded: %d", n)
        except Exception as e:
            logging.warning("Custom prompts (LOCAL) load failed: %s", e)
    else:
        n = _load_custom_prompts_via_entry_points()
        if n == 0:
            logging.info("No entry-point plugins found. Did you install extra '.[custom-prompts]'?")
        else:
            logging.info("Custom prompts (GIT/EP) loaded: %d", n)

_maybe_load_custom_prompts()

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
    