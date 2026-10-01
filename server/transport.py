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
from starlette.requests import Request
from starlette.responses import Response
from starlette.routing import Route, Mount
from starlette.applications import Starlette
from starlette.types import Scope, Receive, Send
import contextlib
from collections.abc import AsyncIterator

import uvicorn
import anyio


class TransportMixin:
    def __init__(
        self,
        server: BaseServer,
        *,
        host: str = "127.0.0.1",
        port: int = 8123,
        log_level: str = "info",
    ):
        self._mcp_server = server
        self._host = host
        self._port = port
        self._log_level = log_level

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

    async def run_streamable_http_async(
        self,
        *,
        path: str = "/mcp",
        json_response: bool = False,
        stateless: bool = False,
    ) -> None:
        """Run the Streamable HTTP transport (MCP spec's SSE replacement).

        See https://modelcontextprotocol.io/docs/concepts/transports#streamable-http.
        A single ASGI endpoint handles both the initial POST (which may
        upgrade to an SSE stream for server->client messages) and
        subsequent session-scoped requests, per the MCP spec.
        """
        session_manager = StreamableHTTPSessionManager(
            app=self._mcp_server,
            json_response=json_response,
            stateless=stateless,
        )

        async def handle_streamable_http(
            scope: Scope, receive: Receive, send: Send
        ) -> None:
            await session_manager.handle_request(scope, receive, send)

        @contextlib.asynccontextmanager
        async def lifespan(app: Starlette) -> AsyncIterator[None]:
            async with session_manager.run():
                yield

        app = Starlette(
            debug=True,
            routes=[
                Mount(path, app=handle_streamable_http),
            ],
            lifespan=lifespan,
        )

        config = uvicorn.Config(
            app,
            host=self._host,
            port=self._port,
            log_level=self._log_level.lower(),
        )
        server = uvicorn.Server(config)
        await server.serve()

    def run(self, transport: str = "stdio") -> None:
        if transport == "stdio":
            anyio.run(self.run_stdio_async)
        elif transport == "sse":
            anyio.run(lambda: self.run_sse_async())
        elif transport == "streamable-http":
            anyio.run(lambda: self.run_streamable_http_async())
        else:
            raise ValueError(f"Unsupported transport: {transport}")
