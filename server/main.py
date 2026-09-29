#
#  Copyright (C) 2025 wisevision
#
#  SPDX-License-Identifier: MPL-2.0
#
#  This Source Code Form is subject to the terms of the Mozilla Public
#  License, v. 2.0. If a copy of the MPL was not distributed with this
#  file, You can obtain one at https://mozilla.org/MPL/2.0/.
#
import sys
import rclpy
import argparse
import asyncio
from .server import app, configure_tools, is_read_only, read_only_requested
from .transport import TransportMixin


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-t",
        "--transport",
        type=str,
        default="stdio",
        choices=["stdio", "sse"],
        help="Transport being use in MCP server",
    )
    parser.add_argument(
        "--read-only",
        action="store_true",
        help="Do not register tools that publish, call services or send/cancel action goals "
        "(same as ROS2_MCP_READONLY=1)",
    )
    args, _ = parser.parse_known_args()
    transport: str = args.transport
    configure_tools(args.read_only or read_only_requested())
    mode = "read-only" if is_read_only() else "read-write"
    print(f'Starting ROS2 MCP server using "{transport}" transport ({mode})', file=sys.stderr)

    rclpy.init()

    try:
        transport_mixin = TransportMixin(app)
        transport_mixin.run(transport)
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
