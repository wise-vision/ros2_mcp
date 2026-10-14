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

"""Tests for the transports in server.transport and the CLI in server.main.

The Streamable HTTP tests talk to a real MCP lowlevel server through the real
ASGI app, and one test goes through a real socket with the MCP SDK client, so a
wiring mistake (route type, lifespan, security middleware) fails here rather
than in a user's client. The server under test is a tiny stand-in tool server so
the tests do not depend on ROS 2 graph state.

Spec: https://modelcontextprotocol.io/docs/concepts/transports#streamable-http
"""

import json
import socket
import threading
import time
from unittest.mock import MagicMock, patch

import anyio
import httpx
import pytest
import uvicorn
from mcp import types
from mcp.client.session import ClientSession
from mcp.client.streamable_http import streamablehttp_client
from mcp.server.lowlevel.server import Server

from server.main import TRANSPORTS, build_parser
from server.transport import TransportMixin, _is_loopback

ACCEPT = "application/json, text/event-stream"
INIT = {
    "jsonrpc": "2.0",
    "id": 1,
    "method": "initialize",
    "params": {
        "protocolVersion": "2025-06-18",
        "capabilities": {},
        "clientInfo": {"name": "pytest", "version": "0"},
    },
}


@pytest.fixture
def anyio_backend():
    return "asyncio"


@pytest.fixture(autouse=True)
def _fresh_sse_starlette_exit_event():
    """sse-starlette keeps one process-global asyncio.Event; each test runs its own
    event loop, so a stale Event raises 'bound to a different event loop'."""
    from sse_starlette.sse import AppStatus

    AppStatus.should_exit_event = None
    yield
    AppStatus.should_exit_event = None


def _echo_server() -> Server:
    srv = Server("transport-test", version="0")

    @srv.list_tools()
    async def _list_tools():
        return [
            types.Tool(
                name="ping",
                description="stand-in tool",
                inputSchema={"type": "object", "properties": {}},
            )
        ]

    return srv


def _mixin(server=None, **kwargs) -> TransportMixin:
    return TransportMixin(server or _echo_server(), **kwargs)


def _body_json(response: httpx.Response) -> dict:
    """Streamable HTTP answers a POST with JSON or a one-event SSE stream."""
    if response.headers["content-type"].startswith("application/json"):
        return response.json()
    data = [l[5:].strip() for l in response.text.splitlines() if l.startswith("data:")]
    return json.loads(data[0])


# --- dispatch -----------------------------------------------------------------


def test_run_dispatches_stdio():
    mixin = _mixin(MagicMock())
    with patch("server.transport.anyio.run") as mock_run:
        mixin.run("stdio")
    mock_run.assert_called_once_with(mixin.run_stdio_async)


@pytest.mark.parametrize("name", ["sse", "streamable-http"])
def test_run_dispatches_http_transports(name):
    mixin = _mixin(MagicMock())
    with patch("server.transport.anyio.run") as mock_run:
        mixin.run(name)
    assert mock_run.call_count == 1
    assert callable(mock_run.call_args[0][0])


def test_run_passes_streamable_options_through():
    mixin = _mixin(MagicMock())
    mixin.run_streamable_http_async = MagicMock(name="run_streamable_http_async")
    with patch("server.transport.anyio.run", side_effect=lambda fn: fn()):
        mixin.run("streamable-http", json_response=True, stateless=True)
    mixin.run_streamable_http_async.assert_called_once_with(
        json_response=True, stateless=True
    )


def test_run_rejects_unknown_transport():
    with pytest.raises(ValueError, match="Unsupported transport"):
        _mixin(MagicMock()).run("carrier-pigeon")


# --- CLI (the real parser, not a copy of it) ------------------------------------


def test_cli_transport_choices_match_dispatch_table():
    assert TRANSPORTS == ["stdio", "sse", "streamable-http"]
    for name in TRANSPORTS:
        assert build_parser().parse_args(["--transport", name]).transport == name


def test_cli_rejects_bogus_transport():
    with pytest.raises(SystemExit):
        build_parser().parse_args(["--transport", "websocket"])


def test_cli_defaults_are_loopback_stdio(monkeypatch):
    for var in (
        "ROS2_MCP_TRANSPORT",
        "ROS2_MCP_HOST",
        "ROS2_MCP_PORT",
        "ROS2_MCP_ALLOWED_HOSTS",
    ):
        monkeypatch.delenv(var, raising=False)
    args = build_parser().parse_args([])
    assert (args.transport, args.host, args.port) == ("stdio", "127.0.0.1", 8123)
    assert args.allowed_host == [] and not args.stateless and not args.json_response


def test_cli_env_vars_configure_the_docker_image(monkeypatch):
    """The Docker CMD cannot take extra arguments, so env vars must be enough."""
    monkeypatch.setenv("ROS2_MCP_TRANSPORT", "streamable-http")
    monkeypatch.setenv("ROS2_MCP_HOST", "0.0.0.0")
    monkeypatch.setenv("ROS2_MCP_PORT", "9000")
    monkeypatch.setenv("ROS2_MCP_ALLOWED_HOSTS", "localhost:*,robot.lan:9000")
    args = build_parser().parse_args([])
    assert args.transport == "streamable-http"
    assert (args.host, args.port) == ("0.0.0.0", 9000)
    assert args.allowed_host == ["localhost:*", "robot.lan:9000"]


@pytest.mark.parametrize(
    "host,expected",
    [
        ("127.0.0.1", True),
        ("127.5.5.5", True),
        ("::1", True),
        ("[::1]", True),
        ("localhost", True),
        ("0.0.0.0", False),
        ("::", False),
        ("192.168.1.10", False),
        ("robot.lan", False),
    ],
)
def test_is_loopback(host, expected):
    assert _is_loopback(host) is expected


# --- Streamable HTTP over the real ASGI app ---------------------------------------


def _client(app, host="127.0.0.1:8123"):
    return httpx.AsyncClient(
        transport=httpx.ASGITransport(app=app),
        base_url=f"http://{host}",
        headers={"Accept": ACCEPT},
    )


@pytest.mark.anyio
async def test_post_to_mcp_path_is_handled_without_redirect():
    """A Mount answered POST /mcp with 307 -> /mcp/, which clients may not follow."""
    app = _mixin().build_streamable_http_app()
    async with app.router.lifespan_context(app), _client(app) as client:
        r = await client.post("/mcp", json=INIT, follow_redirects=False)
    assert r.status_code == 200, r.text
    result = _body_json(r)["result"]
    assert result["serverInfo"]["name"] == "transport-test"
    assert result["protocolVersion"] == "2025-06-18"


@pytest.mark.anyio
async def test_full_session_initialize_then_list_tools():
    app = _mixin().build_streamable_http_app()
    async with app.router.lifespan_context(app), _client(app) as client:
        r = await client.post("/mcp", json=INIT)
        sid = r.headers["mcp-session-id"]
        headers = {"mcp-session-id": sid, "mcp-protocol-version": "2025-06-18"}
        r = await client.post(
            "/mcp",
            headers=headers,
            json={"jsonrpc": "2.0", "method": "notifications/initialized"},
        )
        assert r.status_code == 202
        r = await client.post(
            "/mcp",
            headers=headers,
            json={"jsonrpc": "2.0", "id": 2, "method": "tools/list"},
        )
        assert [t["name"] for t in _body_json(r)["result"]["tools"]] == ["ping"]
        # DELETE ends the session; reusing the id afterwards is a client error.
        assert (await client.delete("/mcp", headers=headers)).status_code == 200
        r = await client.post(
            "/mcp",
            headers=headers,
            json={"jsonrpc": "2.0", "id": 3, "method": "tools/list"},
        )
        assert r.status_code == 404


@pytest.mark.anyio
async def test_json_response_mode_returns_plain_json():
    app = _mixin().build_streamable_http_app(json_response=True)
    async with app.router.lifespan_context(app), _client(app) as client:
        r = await client.post("/mcp", json=INIT)
    assert r.headers["content-type"].startswith("application/json")
    assert r.json()["result"]["serverInfo"]["name"] == "transport-test"


@pytest.mark.anyio
async def test_stateless_mode_issues_no_session_id():
    app = _mixin().build_streamable_http_app(stateless=True)
    async with app.router.lifespan_context(app), _client(app) as client:
        r = await client.post("/mcp", json=INIT)
        assert r.status_code == 200
        assert "mcp-session-id" not in r.headers
        # Any request stands alone: no handshake or session header required.
        r = await client.post(
            "/mcp", json={"jsonrpc": "2.0", "id": 2, "method": "tools/list"}
        )
    assert [t["name"] for t in _body_json(r)["result"]["tools"]] == ["ping"]


@pytest.mark.anyio
async def test_custom_path_and_unmounted_paths_404():
    app = _mixin().build_streamable_http_app(path="/rpc")
    async with app.router.lifespan_context(app), _client(app) as client:
        assert (await client.post("/rpc", json=INIT)).status_code == 200
        assert (await client.post("/mcp", json=INIT)).status_code == 404
        assert (await client.get("/definitely-not-mounted")).status_code == 404


@pytest.mark.anyio
async def test_get_without_session_is_a_client_error_not_a_crash():
    app = _mixin().build_streamable_http_app()
    async with app.router.lifespan_context(app), _client(app) as client:
        r = await client.get("/mcp")
    assert 400 <= r.status_code < 500


# --- DNS-rebinding protection -----------------------------------------------------


@pytest.mark.anyio
@pytest.mark.parametrize("bind", ["127.0.0.1", "localhost", "::1"])
async def test_loopback_bind_rejects_foreign_host_header(bind):
    app = _mixin(host=bind).build_streamable_http_app()
    async with app.router.lifespan_context(app):
        async with _client(app, host="evil.example") as bad:
            assert (await bad.post("/mcp", json=INIT)).status_code == 421
        async with _client(app, host="localhost:8123") as good:
            assert (await good.post("/mcp", json=INIT)).status_code == 200


@pytest.mark.anyio
async def test_loopback_bind_rejects_foreign_origin():
    app = _mixin(host="127.0.0.1").build_streamable_http_app()
    async with app.router.lifespan_context(app), _client(app) as client:
        r = await client.post(
            "/mcp", json=INIT, headers={"Origin": "https://evil.example"}
        )
        assert r.status_code == 403
        r = await client.post(
            "/mcp", json=INIT, headers={"Origin": "http://localhost:3000"}
        )
        assert r.status_code == 200


@pytest.mark.anyio
async def test_explicit_allowed_hosts_apply_to_a_wildcard_bind():
    app = _mixin(
        host="0.0.0.0", allowed_hosts=["robot.lan:8123"]
    ).build_streamable_http_app()
    async with app.router.lifespan_context(app):
        async with _client(app, host="robot.lan:8123") as good:
            assert (await good.post("/mcp", json=INIT)).status_code == 200
        async with _client(app, host="evil.example") as bad:
            assert (await bad.post("/mcp", json=INIT)).status_code == 421


@pytest.mark.anyio
async def test_wildcard_bind_without_allow_list_is_unchecked():
    """Documented limit: we cannot guess the right Host for a container/LAN bind."""
    app = _mixin(host="0.0.0.0").build_streamable_http_app()
    async with app.router.lifespan_context(app):
        async with _client(app, host="robot.lan:8123") as client:
            assert (await client.post("/mcp", json=INIT)).status_code == 200


class _NoServe:
    """Stands in for uvicorn.Server so run_streamable_http_async returns at once."""

    def __init__(self, config):
        pass

    async def serve(self):
        return None


def test_non_loopback_bind_prints_no_auth_warning(capsys):
    with patch("server.transport.uvicorn.Server", _NoServe):
        anyio.run(_mixin(host="0.0.0.0").run_streamable_http_async)
    assert "no authentication" in capsys.readouterr().err


def test_loopback_bind_prints_no_warning(capsys):
    with patch("server.transport.uvicorn.Server", _NoServe):
        anyio.run(_mixin(host="127.0.0.1").run_streamable_http_async)
    assert capsys.readouterr().err == ""


# --- real socket + the MCP SDK client ----------------------------------------------


def _free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


@pytest.mark.anyio
async def test_sdk_client_over_a_real_socket():
    """What an actual MCP client does: connect, handshake, list tools, close."""
    port = _free_port()
    mixin = _mixin(host="127.0.0.1", port=port)
    config = uvicorn.Config(
        mixin.build_streamable_http_app(),
        host="127.0.0.1",
        port=port,
        log_level="warning",
    )
    server = uvicorn.Server(config)
    thread = threading.Thread(target=server.run, daemon=True)
    thread.start()
    try:
        for _ in range(100):
            if server.started:
                break
            time.sleep(0.05)
        assert server.started, "uvicorn did not start"
        url = f"http://127.0.0.1:{port}/mcp"
        with anyio.fail_after(20):
            async with streamablehttp_client(url) as (read, write, _get_session_id):
                async with ClientSession(read, write) as session:
                    await session.initialize()
                    tools = await session.list_tools()
        assert [t.name for t in tools.tools] == ["ping"]
    finally:
        server.should_exit = True
        thread.join(timeout=10)
