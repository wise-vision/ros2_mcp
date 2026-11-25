#!/usr/bin/env python3
#
#  Copyright (C) 2025 wise-vision
#
#  SPDX-License-Identifier: MPL-2.0
#
#  This Source Code Form is subject to the terms of the Mozilla Public
#  License, v. 2.0. If a copy of the MPL was not distributed with this
#  file, You can obtain one at https://mozilla.org/MPL/2.0/.
#
"""server/toolhandler.execute_tool_handler: how one tool call is dispatched.

- Handlers with ``run_tool_async`` (the publish tools) run natively on the
  event loop, inside the request's own task.
- Sync handlers run in an ``asyncio.to_thread`` worker so blocking calls
  (service calls, subscribes) cannot stall the loop.

F3 proof (review round 1): while every worker of the shared default
``ThreadPoolExecutor`` is blocked inside long sync handlers, an async
handler — the shape of a stop publish — still completes immediately; a
(further) sync handler demonstrably queues behind the blockers.

Pure Python: toolhandler.py imports only the MCP SDK types, so this runs on
the host without ROS 2. The rclpy-level dispatch is covered by
tests/timed_publish_test.py in the ROS 2 CI containers.
"""
import asyncio
import importlib.util
import os
import pathlib
import sys
import threading

REPO_ROOT = pathlib.Path(__file__).resolve().parents[2]
MODULE = REPO_ROOT / "server" / "toolhandler.py"


def _load():
    spec = importlib.util.spec_from_file_location("toolhandler_under_test", MODULE)
    mod = importlib.util.module_from_spec(spec)
    sys.modules["toolhandler_under_test"] = mod
    spec.loader.exec_module(mod)
    return mod


th = _load()


class _AsyncTool(th.ToolHandler):
    def get_tool_description(self):
        raise NotImplementedError

    async def run_tool_async(self, args):
        return ["async-result"]


class _SyncTool(th.ToolHandler):
    def get_tool_description(self):
        raise NotImplementedError

    def run_tool(self, args):
        return ["sync-result"]


def test_async_handler_runs_on_the_event_loop_thread():
    async def scenario():
        seen = {}

        class Tool(_AsyncTool):
            async def run_tool_async(self, args):
                seen["thread"] = threading.get_ident()
                seen["task"] = asyncio.current_task()
                return ["async-result"]

        task_before = asyncio.current_task()
        result = await th.execute_tool_handler(Tool("t"), {})
        assert result == ["async-result"]
        assert seen["task"] is task_before, "async handler left its request task"
        assert seen["thread"] == threading.get_ident(), (
            "async handler ran in a worker thread instead of on the event loop"
        )

    asyncio.run(scenario())


def test_sync_handler_runs_in_a_worker_thread():
    async def scenario():
        seen = {}

        class Tool(_SyncTool):
            def run_tool(self, args):
                seen["thread"] = threading.get_ident()
                return ["sync-result"]

        result = await th.execute_tool_handler(Tool("t"), {})
        assert result == ["sync-result"]
        assert seen["thread"] != threading.get_ident(), (
            "sync handler ran on the event loop thread and can block it"
        )

    asyncio.run(scenario())


def test_stop_shaped_call_completes_while_every_worker_is_blocked():
    """F3: occupy every default-executor worker with blocked sync handlers;
    an async handler (the stop-publish shape) must not queue behind them."""
    async def scenario():
        loop = asyncio.get_running_loop()
        # asyncio.to_thread delegates to run_in_executor(None, ...): the same
        # default ThreadPoolExecutor used below to saturate the pool.
        workers = min(32, (os.cpu_count() or 1) + 4)
        release = threading.Event()
        running = {"n": 0}
        guard = threading.Lock()

        def occupy_worker():
            with guard:
                running["n"] += 1
            release.wait(30.0)

        blockers = [loop.run_in_executor(None, occupy_worker) for _ in range(workers)]
        try:
            while running["n"] < workers:
                await asyncio.sleep(0.005)

            class StopPublish(_AsyncTool):
                async def run_tool_async(self, args):
                    return ["stopped"]

                def run_tool(self, args):  # pragma: no cover - must not be used
                    raise AssertionError("stop publish took the sync path")

            start = loop.time()
            result = await th.execute_tool_handler(StopPublish("t"), {})
            elapsed = loop.time() - start

            assert result == ["stopped"]
            assert elapsed < 1.0, (
                f"async stop call waited {elapsed:.2f}s while the worker pool "
                "was saturated"
            )

            # Saturation was real: a sync handler queues behind the blockers.
            queued = asyncio.create_task(th.execute_tool_handler(_SyncTool("t"), {}))
            await asyncio.sleep(0.2)
            assert not queued.done(), "worker pool was not actually saturated"
            release.set()
            assert await asyncio.wait_for(queued, 30.0) == ["sync-result"]
        finally:
            release.set()
            await asyncio.wait_for(asyncio.gather(*blockers, return_exceptions=True), 30.0)

    asyncio.run(scenario())
