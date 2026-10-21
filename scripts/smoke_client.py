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
"""End-to-end smoke client: drives the real server over stdio against a live ROS 2 graph.

Runs inside a ros2_mcp image while ``scripts/smoke_e2e.sh`` publishes on ``$SMOKE_TOPIC`` (a name
unique to that run, so a foreign publisher on the same ROS domain cannot satisfy the checks) from
a second container. It starts the server through ``/app/scripts/run_server.sh``, the same script
the image's CMD runs, with ``ROS2_MCP_READONLY=1``, and asserts through the MCP protocol:

1. the server initialises and reports a real release version;
2. read-only mode registers only tools on the server's own read-only allowlist
   (``server/tool_safety.py``), and none of its mutating tools;
3. ``ros2_topic_list`` sees ``$SMOKE_TOPIC`` as ``std_msgs/msg/String``;
4. ``ros2_topic_subscribe`` receives ``$SMOKE_PAYLOAD`` from the other container.

Prints ``SMOKE OK`` and exits 0 on success; prints ``SMOKE FAIL: <reason>`` and exits 1 otherwise.
"""

import asyncio
import importlib.util
import json
import os
import pathlib
import sys

from mcp import ClientSession, StdioServerParameters
from mcp.client.stdio import stdio_client

APP = pathlib.Path(__file__).resolve().parents[1]
RUN_SERVER = APP / "scripts" / "run_server.sh"
TOPIC = os.environ.get("SMOKE_TOPIC", "/chatter")
PAYLOAD = os.environ.get("SMOKE_PAYLOAD", "smoke")
TOPIC_TYPE = "std_msgs/msg/String"


def fail(msg: str) -> None:
    print(f"SMOKE FAIL: {msg}", flush=True)
    sys.exit(1)


def load_tool_safety():
    # By path, so the client does not import the server package (and rclpy) just for two sets.
    spec = importlib.util.spec_from_file_location("tool_safety", APP / "server" / "tool_safety.py")
    if spec is None or spec.loader is None:
        fail("cannot load server/tool_safety.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def text_of(result) -> str:
    return "".join(getattr(c, "text", "") for c in result.content)


async def call(session, tool: str, args: dict) -> str:
    """Call a tool; the server reports tool failures as error results, not exceptions."""
    result = await session.call_tool(tool, args)
    if getattr(result, "isError", False):
        fail(f"{tool} returned an error: {text_of(result)[:300]}")
    return text_of(result)


async def main() -> None:
    safety = load_tool_safety()
    env = dict(os.environ, ROS2_MCP_READONLY="1")
    params = StdioServerParameters(command=str(RUN_SERVER), args=[], env=env)
    async with stdio_client(params) as (read, write), ClientSession(read, write) as session:
        init = await asyncio.wait_for(session.initialize(), 120)
        info = getattr(init, "serverInfo", None) or init.server_info  # mcp 1.x vs 2.x name
        if not info.name or not info.version or info.version == "unknown":
            fail(f"server did not report a release version: {info.name!r} {info.version!r}")
        print(f"server: {info.name} {info.version}", flush=True)

        names = {t.name for t in (await session.list_tools()).tools}
        leaked = sorted(names & set(safety.MUTATING_TOOLS))
        if leaked:
            fail(f"read-only mode registered state-changing tools: {leaked}")
        unreviewed = sorted(names - set(safety.READ_ONLY_TOOLS))
        if unreviewed:
            fail(f"read-only mode registered tools outside the read-only allowlist: {unreviewed}")
        # An empty or partial registration must not pass as "all on the allowlist".
        missing = sorted(set(safety.READ_ONLY_TOOLS) - names)
        if missing:
            fail(f"read-only mode did not register allowlisted tools: {missing}")
        print(f"tools: {len(names)}, all on the read-only allowlist", flush=True)

        # Discovery takes a few seconds (more under a router), and a just-seen publisher can
        # briefly report its type as "unknown": wait for the type, not just the name.
        topics: dict[str, str] = {}
        for _ in range(20):
            listing = json.loads(await call(session, "ros2_topic_list", {}))
            topics = {t["topic_name"]: t["topic_type"] for t in listing}
            if topics.get(TOPIC) == TOPIC_TYPE:
                break
            await asyncio.sleep(1)
        else:
            fail(f"{TOPIC} not discovered as {TOPIC_TYPE}; saw {sorted(topics.items())}")
        print(f"topic_list: {TOPIC} {TOPIC_TYPE}", flush=True)

        sub = await call(session, "ros2_topic_subscribe",
                         {"topic_name": TOPIC, "message_limit": 2, "duration": 10})
        if PAYLOAD not in sub:
            fail(f"no messages carrying {PAYLOAD!r} received on {TOPIC}: {sub[:300]}")
        print("topic_subscribe: messages received", flush=True)
    print("SMOKE OK", flush=True)


if __name__ == "__main__":
    asyncio.run(main())
