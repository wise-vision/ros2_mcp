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

Runs inside a ros2_mcp image while ``scripts/smoke_e2e.sh`` publishes ``/chatter`` from a second
container. It spawns the server exactly as the image's CMD does (``ROS2_MCP_READONLY=1``) and
asserts, through the MCP protocol:

1. the server initialises and reports its release;
2. no state-changing tool is registered in read-only mode;
3. ``ros2_topic_list`` sees ``/chatter`` with its type;
4. ``ros2_topic_subscribe`` receives real messages from the other container.

Exit 0 on success, 1 with a reason on the first failed check.
"""

import asyncio
import json
import os
import sys

from mcp import ClientSession, StdioServerParameters
from mcp.client.stdio import stdio_client

MUTATING = {
    "ros2_topic_publish",
    "ros2_publish_multiple_topics",
    "ros2_service_call",
    "ros2_send_action_goal",
    "ros2_cancel_action_goal",
}

SERVER_CMD = (
    "source /opt/ros/$ROS_DISTRO/setup.bash && "
    "source /root/wisevision_ws/install/setup.bash && "
    "cd /app && exec uv run --locked mcp_ros_2_server"
)


def fail(msg: str) -> None:
    print(f"SMOKE FAIL: {msg}", flush=True)
    sys.exit(1)


def text_of(result) -> str:
    return "".join(getattr(c, "text", "") for c in result.content)


async def main() -> None:
    env = dict(os.environ, ROS2_MCP_READONLY="1")
    params = StdioServerParameters(command="bash", args=["-c", SERVER_CMD], env=env)
    async with stdio_client(params) as (read, write), ClientSession(read, write) as session:
        init = await asyncio.wait_for(session.initialize(), 120)
        info = getattr(init, "serverInfo", None) or init.server_info  # mcp 1.x vs 2.x name
        print(f"server: {info.name} {info.version}", flush=True)

        names = {t.name for t in (await session.list_tools()).tools}
        leaked = sorted(MUTATING & names)
        if leaked:
            fail(f"read-only mode registered state-changing tools: {leaked}")
        print(f"tools: {len(names)} (no state-changing tools)", flush=True)

        topics = []
        for _ in range(15):  # discovery can take a few seconds, more under a router
            listing = json.loads(text_of(await session.call_tool("ros2_topic_list", {})))
            topics = {t["topic_name"]: t["topic_type"] for t in listing}
            if "/chatter" in topics:
                break
            await asyncio.sleep(1)
        if topics.get("/chatter") != "std_msgs/msg/String":
            fail(f"/chatter not discovered as std_msgs/msg/String; saw {sorted(topics)}")
        print("topic_list: /chatter std_msgs/msg/String", flush=True)

        sub = text_of(
            await session.call_tool(
                "ros2_topic_subscribe",
                {"topic_name": "/chatter", "message_limit": 2, "duration": 10},
            )
        )
        if "smoke" not in sub:
            fail(f"no messages received on /chatter: {sub[:300]}")
        print("topic_subscribe: messages received", flush=True)
    print("SMOKE OK", flush=True)


if __name__ == "__main__":
    asyncio.run(main())
