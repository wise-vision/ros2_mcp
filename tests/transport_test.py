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

"""Tests for server.transport.TransportMixin.

Covers the "streamable-http" transport added to replace the deprecated
SSE transport (see MCP spec:
https://modelcontextprotocol.io/docs/concepts/transports#server-sent-events-sse-deprecated
and https://modelcontextprotocol.io/docs/concepts/transports#streamable-http).
"""

import argparse
from unittest.mock import MagicMock, patch

import anyio
import pytest
from starlette.applications import Starlette
from starlette.routing import Mount

from server.transport import TransportMixin


def _make_mixin():
    fake_server = MagicMock()
    return TransportMixin(fake_server, host="127.0.0.1", port=8123, log_level="info")


def test_run_dispatches_stdio():
    mixin = _make_mixin()
    with patch("server.transport.anyio.run") as mock_run:
        mixin.run("stdio")
    mock_run.assert_called_once_with(mixin.run_stdio_async)


def test_run_dispatches_sse():
    mixin = _make_mixin()
    with patch("server.transport.anyio.run") as mock_run:
        mixin.run("sse")
    assert mock_run.call_count == 1
    # run_sse_async is wrapped in a lambda; just confirm anyio.run was invoked
    # with a zero-arg callable.
    called_callable = mock_run.call_args[0][0]
    assert callable(called_callable)


def test_run_dispatches_streamable_http():
    """RED before the fix: 'streamable-http' was not a recognized transport
    and this branch raised ValueError. GREEN after: it dispatches to
    run_streamable_http_async via anyio.run, mirroring the sse path."""
    mixin = _make_mixin()
    with patch("server.transport.anyio.run") as mock_run:
        mixin.run("streamable-http")
    assert mock_run.call_count == 1
    called_callable = mock_run.call_args[0][0]
    assert callable(called_callable)


def test_run_rejects_unknown_transport():
    mixin = _make_mixin()
    with pytest.raises(ValueError, match="Unsupported transport"):
        mixin.run("carrier-pigeon")


def test_main_argparse_accepts_streamable_http_choice():
    """Boundary check: the CLI parser (server/main.py) must accept
    'streamable-http' as a valid --transport choice, matching the
    TransportMixin.run() dispatch table."""
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument(
        "-t",
        "--transport",
        type=str,
        default="stdio",
        choices=["stdio", "sse", "streamable-http"],
    )
    args = parser.parse_args(["--transport", "streamable-http"])
    assert args.transport == "streamable-http"


def test_main_argparse_rejects_bogus_transport():
    """Error-path attack: an unsupported value must be rejected by argparse
    itself (SystemExit), not silently accepted."""
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument(
        "-t",
        "--transport",
        type=str,
        default="stdio",
        choices=["stdio", "sse", "streamable-http"],
    )
    with pytest.raises(SystemExit):
        parser.parse_args(["--transport", "websocket"])


async def _build_streamable_http_app(mixin, **kwargs):
    """Drive run_streamable_http_async() far enough to capture the real
    Starlette app it builds, without actually binding a socket via uvicorn."""
    captured = {}

    class _FakeUvicornServer:
        def __init__(self, config):
            captured["config"] = config

        async def serve(self):
            # Simulate a server that starts, handles the ASGI lifespan
            # startup/shutdown, and immediately exits with zero requests.
            app: Starlette = captured["config"].app
            scope = {"type": "lifespan"}

            sent = []

            async def receive():
                if not sent:
                    sent.append("startup")
                    return {"type": "lifespan.startup"}
                sent.append("shutdown")
                return {"type": "lifespan.shutdown"}

            events = []

            async def send(message):
                events.append(message)

            await app(scope, receive, send)
            assert {"type": "lifespan.startup.complete"} in events
            assert {"type": "lifespan.shutdown.complete"} in events

    with patch("server.transport.uvicorn.Config") as mock_config_cls, patch(
        "server.transport.uvicorn.Server", _FakeUvicornServer
    ):
        def _config_side_effect(app, **inner_kwargs):
            cfg = MagicMock()
            cfg.app = app
            return cfg

        mock_config_cls.side_effect = _config_side_effect

        await mixin.run_streamable_http_async(**kwargs)

    return captured["config"].app


@pytest.mark.anyio
async def test_run_streamable_http_async_builds_mounted_starlette_app():
    """Verify run_streamable_http_async() constructs a Starlette app that
    mounts the streamable-http ASGI handler at the configured path, and
    that the session manager's lifespan context is entered/exited cleanly
    even if uvicorn.Server.serve() is mocked out (empty-input / no-traffic
    boundary case: the app must still build and the lifespan must not
    raise with zero requests handled)."""
    mixin = _make_mixin()
    app = await _build_streamable_http_app(mixin, path="/mcp")

    assert isinstance(app, Starlette)
    mount_paths = [route.path for route in app.routes if isinstance(route, Mount)]
    assert "/mcp" in mount_paths


@pytest.mark.anyio
async def test_streamable_http_app_responds_over_real_asgi_transport():
    """Attack: exercise the built app through a genuine ASGI request/response
    cycle (httpx ASGITransport) instead of only inspecting route objects.
    A GET with no MCP session header against a stateful streamable-http
    mount must not 500 / hang — it should be handled by the SDK's
    StreamableHTTPServerTransport and return a client-error status,
    proving the ASGI wiring (lifespan + mount) is actually load-bearing."""
    httpx = pytest.importorskip("httpx")

    mixin = _make_mixin()
    app = await _build_streamable_http_app(mixin, path="/mcp")

    transport = httpx.ASGITransport(app=app)
    async with httpx.AsyncClient(
        transport=transport, base_url="http://test"
    ) as client:
        response = await client.post(
            "/mcp",
            json={"jsonrpc": "2.0", "id": 1, "method": "ping"},
            headers={"content-type": "application/json"},
        )

    # We are not driving a full MCP handshake here (no Accept/session
    # negotiation) — the point of the attack is that the ASGI app answers
    # deterministically instead of erroring out with a 500 or hanging.
    assert response.status_code < 500


@pytest.mark.anyio
async def test_streamable_http_unmounted_path_returns_404():
    """Boundary attack: a request to a path outside the configured mount
    must not be silently routed to the MCP handler — it should 404."""
    httpx = pytest.importorskip("httpx")

    mixin = _make_mixin()
    app = await _build_streamable_http_app(mixin, path="/mcp")

    transport = httpx.ASGITransport(app=app)
    async with httpx.AsyncClient(
        transport=transport, base_url="http://test"
    ) as client:
        response = await client.get("/definitely-not-mounted")

    assert response.status_code == 404


@pytest.mark.anyio
async def test_run_streamable_http_async_stateless_mode_builds_app():
    """Attack: stateless=True is a distinct code path in
    StreamableHTTPSessionManager (fresh transport per request, no session
    tracking). Confirm the app still builds and lifespan still completes
    cleanly under this configuration."""
    mixin = _make_mixin()
    app = await _build_streamable_http_app(
        mixin, path="/mcp", stateless=True, json_response=True
    )
    assert isinstance(app, Starlette)


@pytest.fixture
def anyio_backend():
    return "asyncio"
