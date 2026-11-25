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
"""Tests for server/timed_publish.py, the asyncio timing/cancel logic behind
the ros2_topic_publish (and ros2_publish_multiple_topics) continuous-publish
loops.

Pure Python (asyncio only): the module deliberately does not import rclpy,
so these run on the host without a ROS 2 installation. The rclpy-side
behaviour (real publishers, real tool dispatch) is covered by
tests/timed_publish_test.py in the ROS 2 CI containers.

The guarantees proven here map to the round-1 review findings:

- F1 (critical): a cancelled loop never publishes after the stop message
  (``test_stop_message_is_never_followed_by_an_old_loop_message`` and
  ``test_replacement_timed_publish_never_publishes_after_the_new_loop``).
- F4: cancelling the awaiting task stops publishing and the identity-guarded
  registry teardown keeps the replacement registered.
- F5: a publish interval longer than the remaining duration cannot outlive
  the deadline (virtual-clock tests).
"""
import asyncio
import importlib.util
import pathlib
import sys

import pytest

REPO_ROOT = pathlib.Path(__file__).resolve().parents[2]
MODULE = REPO_ROOT / "server" / "timed_publish.py"


def _load():
    spec = importlib.util.spec_from_file_location("timed_publish", MODULE)
    mod = importlib.util.module_from_spec(spec)
    sys.modules["timed_publish"] = mod
    spec.loader.exec_module(mod)
    return mod


tp = _load()


async def _registered_sleeper(registry, topic_key):
    """A stand-in timed loop that registers itself and sleeps forever, so
    tests can observe registry begin/end and cancellation in isolation."""
    registry.begin(topic_key)
    try:
        await asyncio.sleep(30.0)
    finally:
        registry.end(topic_key)


# ---------------------------------------------------------------- validation


def test_duration_above_maximum_is_rejected():
    error = tp.validate_timed_publish_args(10.0, tp.MAX_PUBLISH_DURATION_SEC + 1)
    assert error is not None
    assert "300" in error


def test_duration_at_maximum_is_accepted():
    assert tp.validate_timed_publish_args(10.0, tp.MAX_PUBLISH_DURATION_SEC) is None


def test_duration_and_frequency_must_be_positive():
    assert "Duration" in tp.validate_timed_publish_args(10.0, 0.0)
    assert "Frequency" in tp.validate_timed_publish_args(0.0, 1.0)
    assert "Frequency" in tp.validate_timed_publish_args(-1.0, 1.0)


def test_non_numeric_and_non_finite_timing_is_rejected():
    assert "Frequency" in tp.validate_timed_publish_args("10", 1.0)
    assert "Duration" in tp.validate_timed_publish_args(10, "5")
    assert "Duration" in tp.validate_timed_publish_args(10, float("nan"))
    assert "Duration" in tp.validate_timed_publish_args(10, float("inf"))
    assert "Frequency" in tp.validate_timed_publish_args(float("nan"), 1.0)
    assert "Frequency" in tp.validate_timed_publish_args(True, 1.0)


def test_single_publish_has_nothing_to_validate():
    assert tp.validate_timed_publish_args(None, None) is None
    assert tp.validate_timed_publish_args(10.0, None) is None
    assert tp.validate_timed_publish_args(None, 5.0) is None


# ------------------------------------------------------------------- registry


def test_key_normalizes_bare_and_absolute_names():
    assert tp.TimedPublishRegistry.key_for("cmd_vel") == "/cmd_vel"
    assert tp.TimedPublishRegistry.key_for("/cmd_vel") == "/cmd_vel"
    assert tp.TimedPublishRegistry.key_for(" /cmd_vel ") == "/cmd_vel"


def test_begin_registers_and_cancels_the_previous_loop():
    async def scenario():
        registry = tp.TimedPublishRegistry()
        first = asyncio.create_task(_registered_sleeper(registry, "/t"))
        await asyncio.sleep(0.01)
        assert registry.active_topics() == ["/t"]

        second = asyncio.create_task(_registered_sleeper(registry, "/t"))
        # 'second' starting cancels 'first' before doing anything else.
        with pytest.raises(asyncio.CancelledError):
            await first
        await asyncio.sleep(0.01)
        assert registry.active_topics() == ["/t"]
        assert registry.cancel("/t") is True  # 'second' is still registered
        with pytest.raises(asyncio.CancelledError):
            await second
        await asyncio.sleep(0.01)
        assert registry.active_topics() == []

    asyncio.run(scenario())


def test_cancel_reports_whether_a_loop_was_registered():
    async def scenario():
        registry = tp.TimedPublishRegistry()
        assert registry.cancel("/cmd_vel") is False
        task = asyncio.create_task(_registered_sleeper(registry, "/cmd_vel"))
        await asyncio.sleep(0.01)
        assert registry.cancel("/cmd_vel") is True
        with pytest.raises(asyncio.CancelledError):
            await task

    asyncio.run(scenario())


def test_replaced_loop_teardown_keeps_successor_registered():
    """F4 identity guard: the old request's finally must not unregister (nor
    be able to cancel) its replacement on the same topic."""
    async def scenario():
        registry = tp.TimedPublishRegistry()
        first = asyncio.create_task(_registered_sleeper(registry, "/t"))
        await asyncio.sleep(0.01)
        second = asyncio.create_task(_registered_sleeper(registry, "/t"))

        with pytest.raises(asyncio.CancelledError):
            await first  # first's finally runs here, after second began
        await asyncio.sleep(0.01)

        assert registry.active_topics() == ["/t"], (
            "the replaced loop unregistered its successor"
        )
        assert registry.cancel("/t") is True, "successor is no longer stoppable"
        with pytest.raises(asyncio.CancelledError):
            await second

    asyncio.run(scenario())


# ---------------------------------------------------------------------- loop


def test_loop_publishes_at_frequency_and_stops_at_duration():
    count = {"n": 0}

    def publish():
        count["n"] += 1

    async def scenario():
        return await tp.run_timed_publish_loop(
            publish, frequency=100.0, duration=0.3
        )

    outcome = asyncio.run(scenario())
    # First publish is immediate, then one per interval (10 ms).
    assert 20 <= outcome["published_count"] <= 40
    assert 0.25 <= outcome["elapsed_sec"] < 2.0


def test_loop_stops_when_ros_context_is_gone():
    calls = {"n": 0}

    async def scenario():
        await tp.run_timed_publish_loop(
            lambda: calls.__setitem__("n", calls["n"] + 1),
            frequency=10.0,
            duration=30.0,
            context_ok=lambda: False,
        )

    asyncio.run(scenario())
    assert calls["n"] == 0


def test_duration_shorter_than_one_interval_ends_at_the_duration():
    """F5 (real time): frequency=2 Hz (0.5 s interval), duration=0.1 s —
    the loop must end after ~0.1 s, not after the first full interval."""

    async def scenario():
        loop = asyncio.get_running_loop()
        calls = []
        start = loop.time()
        outcome = await tp.run_timed_publish_loop(
            lambda: calls.append(1), frequency=2.0, duration=0.1
        )
        return loop.time() - start, outcome, calls

    wall, outcome, calls = asyncio.run(scenario())
    assert len(calls) == 1
    assert outcome["elapsed_sec"] <= 0.45
    assert wall < 1.0


def test_interval_longer_than_duration_never_outlives_the_deadline():
    """F5 (virtual clock): frequency=0.001 Hz (1000 s interval) with
    duration=300 s must publish exactly once and end at the 300 s deadline —
    the round-1 Event.wait(interval) design stayed alive for ~1000 s."""
    async def scenario():
        now = 500.0

        def clock():
            return now

        async def virtual_sleep(seconds):
            nonlocal now
            now += seconds

        calls = []
        outcome = await tp.run_timed_publish_loop(
            lambda: calls.append(1),
            frequency=0.001,
            duration=300.0,
            clock=clock,
            sleep=virtual_sleep,
        )
        return now, outcome, calls

    ended_at, outcome, calls = asyncio.run(scenario())
    assert len(calls) == 1
    assert outcome["published_count"] == 1
    assert ended_at == 500.0 + 300.0
    assert outcome["elapsed_sec"] == 300.0


def test_wait_is_bounded_by_the_remaining_deadline():
    """F5 (virtual clock): a mid-range frequency sleeps interval, interval,
    then only the remaining 0.2 s — never past the deadline."""
    async def scenario():
        now = 0.0

        def clock():
            return now

        async def virtual_sleep(seconds):
            nonlocal now
            now += seconds

        calls = []
        outcome = await tp.run_timed_publish_loop(
            lambda: calls.append(1),
            frequency=2.0,
            duration=1.2,
            clock=clock,
            sleep=virtual_sleep,
        )
        return now, outcome

    ended_at, outcome = asyncio.run(scenario())
    assert outcome["published_count"] == 3  # t=0, 0.5, 1.0
    assert ended_at == 1.2
    assert outcome["elapsed_sec"] == 1.2


# ----------------------------------------------------- stop/replace ordering


def test_stop_message_is_never_followed_by_an_old_loop_message():
    """F1 (critical): the single-publish stop path cancels the old loop and
    then publishes the stop command with no await in between; the cancelled
    loop is suspended at its await, so it can never publish afterwards."""
    async def scenario():
        registry = tp.TimedPublishRegistry()
        records = []

        loop_task = asyncio.create_task(
            tp.run_registered_timed_publish(
                registry,
                "/cmd_vel",
                lambda: records.append("move"),
                frequency=200.0,
                duration=30.0,
            )
        )
        while len(records) < 3:
            await asyncio.sleep(0.005)

        # Exactly what the manager's single-publish path does.
        assert registry.cancel("/cmd_vel") is True
        records.append("STOP")

        with pytest.raises(asyncio.CancelledError):
            await loop_task
        await asyncio.sleep(0.05)  # any straggler publish would land here

        assert records[-1] == "STOP", records
        assert "move" not in records[records.index("STOP"):], records

    asyncio.run(scenario())


def test_replacement_timed_publish_never_publishes_after_the_new_loop():
    """F1 (critical), timed-replaces-timed: the new loop's registration
    cancels the old one before its first publish."""
    async def scenario():
        registry = tp.TimedPublishRegistry()
        records = []

        old = asyncio.create_task(
            tp.run_registered_timed_publish(
                registry,
                "/cmd_vel",
                lambda: records.append("old"),
                frequency=200.0,
                duration=30.0,
            )
        )
        while records.count("old") < 3:
            await asyncio.sleep(0.005)

        new = asyncio.create_task(
            tp.run_registered_timed_publish(
                registry,
                "/cmd_vel",
                lambda: records.append("new"),
                frequency=200.0,
                duration=0.2,
            )
        )
        with pytest.raises(asyncio.CancelledError):
            await old
        await new

        first_new = records.index("new")
        assert "old" not in records[first_new:], records
        assert records[-1] == "new", records

    asyncio.run(scenario())


def test_cancelling_the_awaiting_task_stops_publishing():
    """F4: the loop lives inside its request's task, so cancelling that task
    (MCP notifications/cancelled) stops publishing immediately and the
    registry entry is cleaned up."""
    async def scenario():
        registry = tp.TimedPublishRegistry()
        calls = {"n": 0}

        task = asyncio.create_task(
            tp.run_registered_timed_publish(
                registry,
                "/cmd_vel",
                lambda: calls.__setitem__("n", calls["n"] + 1),
                frequency=200.0,
                duration=30.0,
            )
        )
        while calls["n"] < 2:
            await asyncio.sleep(0.005)

        task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await task

        publish_count = calls["n"]
        await asyncio.sleep(0.05)
        assert calls["n"] == publish_count, "publishing continued after cancel"
        assert registry.active_topics() == []

    asyncio.run(scenario())
