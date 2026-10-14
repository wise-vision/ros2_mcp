#
#  Copyright (C) 2025 wisevision
#
#  SPDX-License-Identifier: MPL-2.0
#
#  This Source Code Form is subject to the terms of the Mozilla Public
#  License, v. 2.0. If a copy of the MPL was not distributed with this
#  file, You can obtain one at https://mozilla.org/MPL/2.0/.
#

from mcp.server.lowlevel.server import Server as BaseServer
from mcp.server.sse import SseServerTransport
from mcp.server.stdio import stdio_server
from mcp.server.streamable_http_manager import StreamableHTTPSessionManager
from mcp.server.transport_security import TransportSecuritySettings
from starlette.requests import Request
from starlette.responses import Response
from starlette.routing import Route, Mount
from starlette.applications import Starlette
from starlette.types import Scope, Receive, Send
import contextlib
import ipaddress
import sys
from collections.abc import AsyncIterator, Sequence

import uvicorn
import anyio


def _is_loopback(host: str) -> bool:
    if host == "localhost":
        return True
    try:
        return ipaddress.ip_address(host.strip("[]")).is_loopback
    except ValueError:
        return False


class TransportMixin:
    def __init__(
        self,
        server: BaseServer,
        *,
        host: str = "127.0.0.1",
        port: int = 8123,
        log_level: str = "info",
        allowed_hosts: Sequence[str] | None = None,
    ):
        self._mcp_server = server
        self._host = host
        self._port = port
        self._log_level = log_level
        self._allowed_hosts = list(allowed_hosts or [])

    async def run_stdio_async(self) -> None:
        async with stdio_server() as (read_stream, write_stream):
            await self._mcp_server.run(
                read_stream,
                write_stream,
                self._mcp_server.create_initialization_options(),
            )

    async def run_sse_async(
        self, sse_path: str = "/sse", message_path: str = "/messages/"
    ) -> None:
        """Run the legacy HTTP+SSE transport.

        NOTE: SSE transport is deprecated by the MCP spec in favor of
        Streamable HTTP (see https://modelcontextprotocol.io/docs/concepts/transports#server-sent-events-sse-deprecated).
        Kept here for backward compatibility with existing SSE clients;
        prefer "streamable-http" for new integrations.
        """
        sse = SseServerTransport(message_path)

        async def handle_sse(scope: Scope, receive: Receive, send: Send):
            async with sse.connect_sse(scope, receive, send) as streams:
                await self._mcp_server.run(
                    streams[0],
                    streams[1],
                    self._mcp_server.create_initialization_options(),
                )

        # Wrapping ASGI app in a HTTP-style endpoint
        async def sse_endpoint(request: Request) -> Response:
            async def _asgi_app(scope, receive, send):
                await handle_sse(scope, receive, send)

            return await _asgi_app(request.scope, request.receive, request._send)

        app = Starlette(
            debug=True,
            routes=[
                Route(sse_path, endpoint=sse_endpoint, methods=["GET"]),
                Mount(message_path, app=sse.handle_post_message),
            ],
        )

        config = uvicorn.Config(
            app,
            host=self._host,
            port=self._port,
            log_level=self._log_level.lower(),
        )
        server = uvicorn.Server(config)
        await server.serve()

    def _transport_security(self) -> TransportSecuritySettings | None:
        """DNS-rebinding protection for the Streamable HTTP endpoint.

        This server can publish to topics and call services, so a web page the
        operator visits must not be able to reach a loopback-bound instance by
        rebinding a DNS name to 127.0.0.1. Explicit ``allowed_hosts`` always
        enable the check; a loopback bind gets a localhost-only allow-list.
        Any other bind without an allow-list is left unchecked, because the
        correct Host values (container name, public DNS name) are unknowable here.
        """
        if self._allowed_hosts:
            hosts = list(self._allowed_hosts)
        elif _is_loopback(self._host):
            hosts = ["127.0.0.1:*", "localhost:*", "[::1]:*"]
        else:
            return None
        origins = [f"http://{h}" for h in hosts] + [f"https://{h}" for h in hosts]
        return TransportSecuritySettings(
            enable_dns_rebinding_protection=True,
            allowed_hosts=hosts,
            allowed_origins=origins,
        )

    def build_streamable_http_app(
        self,
        *,
        path: str = "/mcp",
        json_response: bool = False,
        stateless: bool = False,
    ) -> Starlette:
        """Build the ASGI app for the Streamable HTTP transport.

        One endpoint handles POST (client messages), GET (server->client SSE
        stream) and DELETE (session end), per the MCP spec. It is registered as
        a ``Route``, not a ``Mount``: a ``Mount`` answers ``POST /mcp`` with a
        307 redirect to ``/mcp/``, which many MCP clients do not follow.
        """
        session_manager = StreamableHTTPSessionManager(
            app=self._mcp_server,
            json_response=json_response,
            stateless=stateless,
            security_settings=self._transport_security(),
        )

        class _StreamableHTTPApp:
            async def __call__(
                self, scope: Scope, receive: Receive, send: Send
            ) -> None:
                await session_manager.handle_request(scope, receive, send)

        @contextlib.asynccontextmanager
        async def lifespan(app: Starlette) -> AsyncIterator[None]:
            async with session_manager.run():
                yield

        return Starlette(
            routes=[Route(path, endpoint=_StreamableHTTPApp())],
            lifespan=lifespan,
        )

    async def run_streamable_http_async(
        self,
        *,
        path: str = "/mcp",
        json_response: bool = False,
        stateless: bool = False,
    ) -> None:
        """Run the Streamable HTTP transport (the MCP spec's SSE replacement).

        See https://modelcontextprotocol.io/docs/concepts/transports#streamable-http.
        """
        if not _is_loopback(self._host):
            print(
                f"WARNING: ROS2 MCP is listening on {self._host}:{self._port} with no "
                "authentication. Anyone who can reach this port can drive the robot. "
                "Keep it on a private network or behind an authenticating reverse "
                "proxy, and consider --read-only.",
                file=sys.stderr,
            )
        app = self.build_streamable_http_app(
            path=path, json_response=json_response, stateless=stateless
        )
        config = uvicorn.Config(
            app,
            host=self._host,
            port=self._port,
            log_level=self._log_level.lower(),
        )
        server = uvicorn.Server(config)
        await server.serve()

    def run(
        self,
        transport: str = "stdio",
        *,
        json_response: bool = False,
        stateless: bool = False,
    ) -> None:
        if transport == "stdio":
            anyio.run(self.run_stdio_async)
        elif transport == "sse":
            anyio.run(lambda: self.run_sse_async())
        elif transport == "streamable-http":
            anyio.run(
                lambda: self.run_streamable_http_async(
                    json_response=json_response, stateless=stateless
                )
            )
        else:
            raise ValueError(f"Unsupported transport: {transport}")
