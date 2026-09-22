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
"""Forklift rosbag demo: drive the ROS 2 MCP server over stdio against a replayed bag.

Run `ros2 bag play <bag>` in another terminal first, then:

    python3 examples/forklift_rosbag_demo/demo.py

The script spawns `mcp_ros_2_server` (stdio transport) as an MCP client would,
(using the same interpreter that runs this script, started from the repo root)
and then performs three calls:

  1. ros2_topic_list          -> which topics the bag is publishing
  2. ros2_get_message_fields  -> the schema of std_msgs/msg/String
  3. ros2_topic_subscribe     -> a few live messages from a bag topic

Every call is a plain MCP `tools/call`; nothing here is specific to the demo
bag except the default topic name (override with --topic).
"""

import argparse
import asyncio
import json
import os
import sys

from mcp import ClientSession, StdioServerParameters
from mcp.client.stdio import stdio_client

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))


def _text(result) -> str:
    """Join the text parts of a tools/call result; note non-text parts."""
    parts = []
    for c in result.content:
        if getattr(c, "type", None) == "text":
            parts.append(c.text)
        else:
            parts.append(f"<{c.type} content omitted>")
    return "\n".join(parts)


async def main(topic: str, message_limit: int, duration: float, server_cmd: list[str]):
    params = StdioServerParameters(
        command=server_cmd[0], args=server_cmd[1:], env=os.environ.copy(), cwd=REPO_ROOT
    )
    async with stdio_client(params) as (read, write):
        async with ClientSession(read, write) as session:
            await session.initialize()

            print("=== 1. ros2_topic_list")
            r = await session.call_tool("ros2_topic_list", {})
            print(_text(r))

            print("\n=== 2. ros2_get_message_fields(std_msgs/msg/String)")
            r = await session.call_tool("ros2_get_message_fields", {"message_type": "std_msgs/msg/String"})
            print(_text(r))

            print(f"\n=== 3. ros2_topic_subscribe({topic}, message_limit={message_limit}, duration={duration})")
            r = await session.call_tool(
                "ros2_topic_subscribe",
                {"topic_name": topic, "message_limit": message_limit, "duration": duration},
            )
            print(_text(r))


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--topic", default="/pulse/front/radar/detections", help="topic to subscribe to (default: forklift radar detections)")
    ap.add_argument("--messages", type=int, default=3, help="stop after this many messages")
    ap.add_argument("--duration", type=float, default=10.0, help="give up after this many seconds")
    ap.add_argument(
        "--server-cmd",
        default=f"{sys.executable} -m server.main",
        help="command that starts the MCP server on stdio (default: this interpreter, so it must see rclpy)",
    )
    a = ap.parse_args()
    try:
        asyncio.run(main(a.topic, a.messages, a.duration, a.server_cmd.split()))
    except KeyboardInterrupt:
        sys.exit(130)
