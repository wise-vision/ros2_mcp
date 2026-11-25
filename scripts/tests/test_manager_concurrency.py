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
"""ROS2Manager concurrency contracts that must hold without a ROS install.

These tests import the real ``server`` package in a subprocess with a fake
rclpy standing in for the global executor, so the real manager code runs on
the host (and, in CI, deterministically — the fakes win over the real rclpy
because they are installed into sys.modules first):

- F2 (review round 1): two concurrent service-call-shaped operations through
  the manager must not re-enter rclpy's global executor. The fake
  ``rclpy.spin_until_future_complete`` raises the real Jazzy error
  ("Executor is already spinning") on overlap; on the unfixed code the second
  call returns that error, with the spin lock both succeed.
- Bounds: the multi-topic publish schema advertises the same 300 s duration
  maximum as the single-topic schema (round 1 only enforced it server-side).
"""
import os
import pathlib
import subprocess
import sys
import tempfile

REPO_ROOT = pathlib.Path(__file__).resolve().parents[2]

# Installs fake rclpy/rosidl/message modules and a global-executor stand-in
# that raises on overlapping spins, exactly like the real executor does
# (rclpy/executors.py: "Executor is already spinning").
FAKES = r'''
import sys, threading, time, types
from unittest import mock

sys.path.insert(0, sys.argv[1])

def _mod(name, **attrs):
    m = types.ModuleType(name)
    for k, v in attrs.items():
        setattr(m, k, v)
    sys.modules[name] = m
    return m

class _FakeNodeBase:
    def __init__(self, *args, **kwargs):
        pass

_spin_state_lock = threading.Lock()
_spinning = False
_spin_entered = threading.Event()

def _enter_spin():
    global _spinning
    with _spin_state_lock:
        if _spinning:
            raise RuntimeError("Executor is already spinning")
        _spinning = True
    _spin_entered.set()

def _leave_spin():
    global _spinning
    with _spin_state_lock:
        _spinning = False

def spin_until_future_complete(node, future, timeout_sec=None):
    _enter_spin()
    try:
        time.sleep(0.2)  # hold the "executor" so a concurrent spin overlaps
        return future
    finally:
        _leave_spin()

def spin_once(node, timeout_sec=None):
    _enter_spin()
    try:
        time.sleep(0.05)
    finally:
        _leave_spin()

rclpy = _mod("rclpy",
    spin_until_future_complete=spin_until_future_complete,
    spin_once=spin_once,
    ok=lambda *a, **k: True,
    shutdown=lambda *a, **k: None,
    init=lambda *a, **k: None,
)
# rclpy.node.Node is subclassed by the manager: it must be a real class.
# Every other fake module is a MagicMock: `from rclpy.qos import QoSProfile`
# etc. then resolve to auto-created mocks.
def _mock_mod(name):
    m = mock.MagicMock()
    sys.modules[name] = m
    setattr(rclpy, name.split(".", 1)[1], m)
    return m

_mod("rclpy.node", Node=_FakeNodeBase)
for _name in ("rclpy.qos", "rclpy.action", "rclpy.executors", "rclpy.task",
              "rclpy.serialization"):
    _mock_mod(_name)
for _name in ("rosidl_runtime_py", "rosidl_runtime_py.utilities",
              "rosidl_runtime_py.set_message", "builtin_interfaces.msg",
              "std_msgs.msg", "sensor_msgs.msg", "nav_msgs.msg"):
    sys.modules[_name] = mock.MagicMock()
'''

F2_SCENARIO = r'''
import threading
from unittest import mock

import server.ros2_manager as rm

node = mock.MagicMock()  # create_client/wait_for_service/call_async all mocked
with mock.patch.object(rm, "ServiceNode", return_value=node):
    manager = rm.ROS2Manager()

# A tiny fake service package: "fakesrv/srv" -> module attribute `srv`.
class _Request:
    pass
class _Srv:
    Request = _Request
import types as _types
_srv_mod = _types.ModuleType("fakesrv.srv")
_srv_mod.srv = _Srv
sys.modules["fakesrv.srv"] = _srv_mod

results = {}
def call_service(tag):
    results[tag] = manager.call_service("/some_service", "fakesrv/srv", {})

first = threading.Thread(target=call_service, args=("first",))
first.start()
assert _spin_entered.wait(5.0), "first service call never reached its spin"
second = threading.Thread(target=call_service, args=("second",))
second.start()
first.join(10.0)
second.join(10.0)
assert not first.is_alive() and not second.is_alive(), "service call hung"

for tag in ("first", "second"):
    assert "error" not in results[tag], (
        f"concurrent service call '{tag}' failed: {results[tag]}"
    )
print("F2 OK:", results)
'''

BOUNDS_SCENARIO = r'''
from server.tools_ros2 import ROS2TopicPublish
from server.tools_ros2_extended import PublishMultipleTopicsTool

multi = PublishMultipleTopicsTool().get_tool_description().inputSchema
dur = multi["properties"]["topics"]["items"]["properties"]["duration"]
assert dur.get("maximum") == 300.0, (
    f"multi-topic duration schema is not bounded: {dur}"
)

single = ROS2TopicPublish().get_tool_description().inputSchema
assert single["properties"]["duration"].get("maximum") == 300.0
print("BOUNDS OK")
'''


def _run_child(scenario: str) -> subprocess.CompletedProcess:
    with tempfile.NamedTemporaryFile(
        "w", suffix=".py", delete=False, encoding="utf-8"
    ) as f:
        f.write(FAKES + scenario)
        script = f.name
    try:
        return subprocess.run(
            [sys.executable, script, str(REPO_ROOT)],
            capture_output=True,
            text=True,
            timeout=120,
            cwd=str(REPO_ROOT),
        )
    finally:
        os.unlink(script)


def test_concurrent_service_calls_do_not_reenter_the_executor():
    """F2: two overlapping call_service spins must both succeed."""
    proc = _run_child(F2_SCENARIO)
    assert proc.returncode == 0, (
        f"child failed:\n{proc.stdout}\n{proc.stderr}"
    )
    assert "F2 OK" in proc.stdout


def test_multi_topic_duration_schema_is_bounded():
    """Bounds: both publish tool schemas advertise the 300 s maximum."""
    proc = _run_child(BOUNDS_SCENARIO)
    assert proc.returncode == 0, (
        f"child failed:\n{proc.stdout}\n{proc.stderr}"
    )
    assert "BOUNDS OK" in proc.stdout
