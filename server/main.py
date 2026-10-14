#
#  Copyright (C) 2025 wisevision
#
#  SPDX-License-Identifier: MPL-2.0
#
#  This Source Code Form is subject to the terms of the Mozilla Public
#  License, v. 2.0. If a copy of the MPL was not distributed with this
#  file, You can obtain one at https://mozilla.org/MPL/2.0/.
#
import os
import sys
import rclpy
import argparse
import asyncio
from .server import app, configure_tools, is_read_only, read_only_requested
from .transport import TransportMixin


TRANSPORTS = ["stdio", "sse", "streamable-http"]


def build_parser() -> argparse.ArgumentParser:
    """CLI definition. Every network option also has an env var so that the
    Docker image (whose CMD cannot take extra arguments) can be configured."""
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-t",
        "--transport",
        type=str,
        default=os.environ.get("ROS2_MCP_TRANSPORT", "stdio"),
        choices=TRANSPORTS,
        help="Transport: stdio (default), streamable-http (current MCP spec, "
        "endpoint /mcp) or sse (deprecated by the MCP spec). "
        "Env: ROS2_MCP_TRANSPORT",
    )
    parser.add_argument(
        "--host",
        default=os.environ.get("ROS2_MCP_HOST", "127.0.0.1"),
        help="Bind address for sse / streamable-http (default 127.0.0.1; use "
        "0.0.0.0 inside a container). There is no authentication. Env: ROS2_MCP_HOST",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=int(os.environ.get("ROS2_MCP_PORT", "8123")),
        help="Port for sse / streamable-http (default 8123). Env: ROS2_MCP_PORT",
    )
    parser.add_argument(
        "--allowed-host",
        action="append",
        default=[h for h in os.environ.get("ROS2_MCP_ALLOWED_HOSTS", "").split(",") if h],
        metavar="HOST[:PORT]",
        help="streamable-http only: accept only these Host headers (DNS-rebinding "
        "protection); '*' wildcards the port, e.g. 'localhost:*'. Repeatable. "
        "Env: ROS2_MCP_ALLOWED_HOSTS (comma separated). Without it, a loopback "
        "bind allows localhost only and other binds are unchecked.",
    )
    parser.add_argument(
        "--stateless",
        action="store_true",
        help="streamable-http only: no per-client session state",
    )
    parser.add_argument(
        "--json-response",
        action="store_true",
        help="streamable-http only: answer POSTs with plain JSON instead of an SSE stream",
    )
    parser.add_argument(
        "--read-only",
        action="store_true",
        help="Do not register tools that publish, call services or send/cancel action goals "
        "(same as ROS2_MCP_READONLY=1)",
    )
    return parser


def main():
    parser = build_parser()
    args, _ = parser.parse_known_args()
    transport: str = args.transport
    configure_tools(args.read_only or read_only_requested())
    mode = "read-only" if is_read_only() else "read-write"
    print(f'Starting ROS2 MCP server using "{transport}" transport ({mode})', file=sys.stderr)

    rclpy.init()

    try:
        transport_mixin = TransportMixin(
            app,
            host=args.host,
            port=args.port,
            allowed_hosts=args.allowed_host,
        )
        transport_mixin.run(
            transport,
            json_response=args.json_response,
            stateless=args.stateless,
        )
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
