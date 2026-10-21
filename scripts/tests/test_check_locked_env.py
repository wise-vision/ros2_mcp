#
#  Copyright (C) 2025 wisevision
#
#  SPDX-License-Identifier: MPL-2.0
#
#  This Source Code Form is subject to the terms of the Mozilla Public
#  License, v. 2.0. If a copy of the MPL was not distributed with this
#  file, You can obtain one at https://mozilla.org/MPL/2.0/.
#
"""scripts/check_locked_env.py: a distro copy must never stand in for a locked package."""

import importlib.util
import pathlib
from types import SimpleNamespace

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location("check_locked_env", ROOT / "scripts" / "check_locked_env.py")
cle = importlib.util.module_from_spec(spec)
spec.loader.exec_module(cle)


def test_both_lock_parsers_agree_on_the_real_lock(monkeypatch):
    text = (ROOT / "uv.lock").read_text()
    full = cle.locked_versions(text)
    assert "numpy" in full and len(full) > 20
    import builtins

    real_import = builtins.__import__

    def no_tomllib(name, *a, **k):  # what Humble's Python 3.10 sees
        if name == "tomllib":
            raise ModuleNotFoundError(name)
        return real_import(name, *a, **k)

    monkeypatch.setattr(builtins, "__import__", no_tomllib)
    assert cle.locked_versions(text) == full


def _dist(name, version, where):
    return SimpleNamespace(metadata={"Name": name}, version=version,
                           locate_file=lambda _p, w=where: pathlib.Path(w))


@pytest.fixture
def env(monkeypatch):
    def setup(*dists):
        monkeypatch.setattr(cle.md, "distributions", lambda: list(dists))
        first = {}
        for d in dists:
            first.setdefault(cle._norm(d.metadata["Name"]), d)
        monkeypatch.setattr(cle.md, "distribution", lambda n: first[cle._norm(n)])
    return setup


VENV = pathlib.Path("/app/.venv")
SITE = "/app/.venv/lib/python3.12/site-packages"
LOCK = {"numpy": {"1.26.4", "2.3.5"}, "pyyaml": {"6.0.3"}}


def test_locked_copies_in_the_venv_pass(env):
    env(_dist("numpy", "1.26.4", SITE), _dist("PyYAML", "6.0.3", SITE), _dist("rclpy", "7", "/opt/ros"))
    assert cle.check(LOCK, VENV) == []


def test_a_system_copy_is_reported_even_at_a_locked_version(env):
    env(_dist("numpy", "1.26.4", "/usr/lib/python3/dist-packages"))
    (problem,) = cle.check(LOCK, VENV)
    assert "numpy" in problem and "not the venv" in problem


def test_an_unlocked_version_in_the_venv_is_reported(env):
    env(_dist("PyYAML", "5.4.1", SITE))
    (problem,) = cle.check(LOCK, VENV)
    assert "pyyaml 5.4.1" in problem


def test_a_shadowed_system_duplicate_is_not_reported(env):
    # venv copy first on sys.path; the distro copy underneath is never imported
    env(_dist("numpy", "1.26.4", SITE), _dist("numpy", "1.21.5", "/usr/lib/python3/dist-packages"))
    assert cle.check(LOCK, VENV) == []


def test_refuses_to_run_outside_a_venv(monkeypatch):
    import sys
    monkeypatch.setattr(sys, "prefix", sys.base_prefix)
    assert cle.main(["x", str(ROOT / "uv.lock")]) == 2
