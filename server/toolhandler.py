#
#  Copyright (C) 2025 wisevision
#
#  SPDX-License-Identifier: MPL-2.0
#
#  This Source Code Form is subject to the terms of the Mozilla Public
#  License, v. 2.0. If a copy of the MPL was not distributed with this
#  file, You can obtain one at https://mozilla.org/MPL/2.0/.
#
import asyncio
from collections.abc import Sequence
from mcp.types import (
    Tool,
    TextContent,
    ImageContent,
    EmbeddedResource,
)


class ToolHandler:
    def __init__(self, tool_name: str, *, ui_only: bool = False):
        self.name = tool_name
        self.ui_only = ui_only

    def get_tool_description(self) -> Tool:
        raise NotImplementedError()

    def run_tool(
        self, args: dict
    ) -> Sequence[TextContent | ImageContent | EmbeddedResource]:
        raise NotImplementedError()


async def execute_tool_handler(
    handler: ToolHandler, arguments: dict
) -> Sequence[TextContent | ImageContent | EmbeddedResource]:
    """Dispatch one tool call.

    Handlers that define ``run_tool_async`` run natively on the event loop:
    a timed ``ros2_topic_publish`` must live inside its request's own task so
    that cancelling the request (a newer publish on the same topic, or an MCP
    ``notifications/cancelled``) cancels the publish loop, and it must never
    occupy one of ``asyncio.to_thread``'s bounded workers — otherwise a stop
    call could queue behind timed publishes. Every other handler is sync and
    blocking (service calls, subscribes with a duration), so it runs in a
    worker thread to keep the event loop responsive.
    """
    run_async = getattr(handler, "run_tool_async", None)
    if run_async is not None:
        return await run_async(arguments)
    return await asyncio.to_thread(handler.run_tool, arguments)
