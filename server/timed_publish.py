#
#  Copyright (C) 2025 wisevision
#
#  SPDX-License-Identifier: MPL-2.0
#
#  This Source Code Form is subject to the terms of the Mozilla Public
#  License, v. 2.0. If a copy of the MPL was not distributed with this
#  file, You can obtain one at https://mozilla.org/MPL/2.0/.
#
"""Asyncio machinery for the timed (frequency x duration) publishing tools.

Design (documented on the ``ros2_topic_publish`` tool description):

- A timed publish runs as a coroutine on the server's event loop, inside the
  MCP request's own task. It never occupies an ``asyncio.to_thread`` worker,
  so timed publishes cannot fill the bounded worker pool and delay a stop
  call, and cancelling the request (an MCP ``notifications/cancelled``, or a
  newer publish on the same topic) cancels the publish loop itself.
- Only one timed publish runs per topic at a time, tracked by
  :class:`TimedPublishRegistry`. A new publish to the topic (timed or single)
  first cancels the old task and only then publishes, with no ``await`` in
  between: the event loop is single-threaded and the old task is suspended
  inside its ``asyncio.sleep``, so it can never publish again afterwards —
  the stop message is always the last message on the topic.
- Every inter-publish wait is bounded by the remaining time to the deadline,
  so a publish interval longer than the remaining duration cannot outlive
  the request (``frequency=0.001, duration=300`` ends after 300 s, not after
  the 1000 s interval).
- ``duration`` is bounded by ``MAX_PUBLISH_DURATION_SEC``.

This module is deliberately free of rclpy imports: the timing/cancel logic
is pure ``asyncio`` and is exercised on the host without ROS 2 by
``scripts/tests/test_timed_publish.py``. The rclpy-side behaviour (real
publishers, real dispatch) is covered by ``tests/timed_publish_test.py`` in
the ROS 2 CI containers.
"""

import asyncio
import math
from collections.abc import Awaitable, Callable

# Longest continuous publish a single tool call may run. A robot safety bound:
# a caller that needs longer re-issues the call (or streams separate publishes).
MAX_PUBLISH_DURATION_SEC = 300.0


class TimedPublishRegistry:
    """The running timed-publish task per topic (key = absolute topic name).

    Event-loop only: every method must be called from the server's event loop
    thread (the tool dispatch already guarantees this), so a plain dict needs
    no lock. Loops are stopped by cancelling their task: a cancelled task is
    suspended at its ``await`` and can never publish again.
    """

    def __init__(self) -> None:
        self._tasks: dict[str, asyncio.Task] = {}

    @staticmethod
    def key_for(topic_name: str) -> str:
        """Topics register under their absolute name (the server's node lives
        in the default namespace, so a bare name and its absolute form denote
        the same topic)."""
        name = (topic_name or "").strip()
        if not name:
            return name
        return name if name.startswith("/") else f"/{name}"

    def begin(self, topic_key: str) -> None:
        """Become the timed publish on ``topic_key``: cancel the loop already
        registered there (if any) and register the current task."""
        self.cancel(topic_key)
        self._tasks[topic_key] = asyncio.current_task()

    def end(self, topic_key: str) -> None:
        """Drop ``topic_key``, but only while it still maps to the current
        task: a loop that was replaced must not unregister its successor."""
        if self._tasks.get(topic_key) is asyncio.current_task():
            del self._tasks[topic_key]

    def cancel(self, topic_key: str) -> bool:
        """Cancel the loop registered for ``topic_key``. Returns True when a
        loop was registered, False when no timed publish runs on the topic."""
        task = self._tasks.get(topic_key)
        if task is None:
            return False
        task.cancel()
        return True

    def active_topics(self) -> list[str]:
        return sorted(self._tasks)


def validate_timed_publish_args(frequency: float | None, duration: float | None) -> str | None:
    """Return an error message when a timed publish request is invalid, else None.

    ``frequency``/``duration`` must be provided together for a timed publish;
    when either is None this is a single publish and there is nothing to
    validate. Error strings keep the long-standing wording of
    ``ros2_topic_publish`` where it existed.
    """
    if frequency is None or duration is None:
        return None
    for name, value in (("Frequency", frequency), ("Duration", duration)):
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            return f"{name} must be a number."
        if not math.isfinite(value):
            return f"{name} must be a finite number."
    if frequency <= 0:
        return "Frequency must be greater than 0."
    if duration <= 0:
        return "Duration must be greater than 0."
    if duration > MAX_PUBLISH_DURATION_SEC:
        return (
            f"Duration must be at most {MAX_PUBLISH_DURATION_SEC:g} seconds, got {duration}. "
            "For longer streams, publish in repeated chunks or stream single publishes."
        )
    return None


async def run_timed_publish_loop(
    publish: Callable[[], None],
    *,
    frequency: float,
    duration: float,
    context_ok: Callable[[], bool] | None = None,
    clock: Callable[[], float] | None = None,
    sleep: Callable[[float], Awaitable[None]] | None = None,
) -> dict:
    """Publish ``publish()`` every ``1/frequency`` seconds until one of:

    - ``duration`` elapsed (checked against the deadline before every publish
      and every wait, so the loop cannot outlive its duration),
    - ``context_ok()`` returns False (ROS 2 context shut down),
    - the surrounding task is cancelled (``CancelledError`` propagates).

    Returns ``{"published_count", "elapsed_sec"}``. The first publish is
    immediate; each wait is ``min(interval, remaining-to-deadline)``.

    ``clock``/``sleep`` are injection points for the tests (virtual time);
    they default to the running event loop's clock and ``asyncio.sleep``.
    """
    if clock is None or sleep is None:
        loop = asyncio.get_running_loop()
        clock = loop.time
        sleep = asyncio.sleep
    interval = 1.0 / frequency
    start = clock()
    end = start + duration
    published_count = 0
    while True:
        remaining = end - clock()
        if remaining <= 0:
            break
        if context_ok is not None and not context_ok():
            break
        publish()
        published_count += 1
        await sleep(min(interval, remaining))
    return {
        "published_count": published_count,
        "elapsed_sec": round(clock() - start, 3),
    }


async def run_registered_timed_publish(
    registry: TimedPublishRegistry,
    topic_key: str,
    publish: Callable[[], None],
    *,
    frequency: float,
    duration: float,
    context_ok: Callable[[], bool] | None = None,
) -> dict:
    """Run one timed-publish loop as the current task, registered for
    ``topic_key``: starting it cancels any loop already registered on that
    topic, and the teardown is identity-guarded so a replaced loop cannot
    unregister its successor. Used by both publishing tools; see the module
    docstring for the ordering guarantees this provides."""
    registry.begin(topic_key)
    try:
        return await run_timed_publish_loop(
            publish, frequency=frequency, duration=duration, context_ok=context_ok
        )
    finally:
        registry.end(topic_key)
