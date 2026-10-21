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
"""Fail if a locked dependency is not the copy uv.lock pins.

The venv uses --system-site-packages (rclpy and the message packages come from the distro), so
a package the distro also ships (numpy, pillow, PyYAML, ...) could be imported from the system
instead of from the venv: an untested version, and for numpy the wrong ABI. For every package in
uv.lock that is installed, the copy Python resolves must live in the venv and carry a version
the lock pins. Packages the lock lists for other platforms or Python versions are not installed
and are skipped.

Usage (inside the venv): python scripts/check_locked_env.py [path/to/uv.lock]
"""

from __future__ import annotations

import importlib.metadata as md
import pathlib
import re
import sys


def _norm(name: str) -> str:
    return re.sub(r"[-_.]+", "-", name).lower()


def locked_versions(lock_text: str) -> dict[str, set[str]]:
    """{normalized name: {versions}} for every registry package in a uv.lock."""
    try:
        import tomllib
    except ModuleNotFoundError:  # Python 3.10 (Humble): the lock is simple enough to scan
        tomllib = None
    out: dict[str, set[str]] = {}
    if tomllib is not None:
        for pkg in tomllib.loads(lock_text).get("package", []):
            if "registry" in pkg.get("source", {}):
                out.setdefault(_norm(pkg["name"]), set()).add(pkg["version"])
        return out
    for block in lock_text.split("[[package]]")[1:]:
        name = re.search(r'^name = "([^"]+)"', block, re.MULTILINE)
        version = re.search(r'^version = "([^"]+)"', block, re.MULTILINE)
        if name and version and re.search(r"^source = \{ registry", block, re.MULTILINE):
            out.setdefault(_norm(name.group(1)), set()).add(version.group(1))
    return out


def check(lock: dict[str, set[str]], venv: pathlib.Path) -> list[str]:
    problems = []
    for dist in md.distributions():
        name = _norm(dist.metadata["Name"] or "")
        if name not in lock:
            continue
        where = pathlib.Path(str(dist.locate_file(""))).resolve()
        resolved = md.distribution(name)  # the copy imports actually get (first on sys.path)
        resolved_at = pathlib.Path(str(resolved.locate_file(""))).resolve()
        if resolved_at != where:
            continue  # a shadowed duplicate; the resolved copy is checked on its own entry
        if venv not in where.parents and where != venv:
            problems.append(f"{name} {dist.version} resolves from {where}, not the venv")
        elif dist.version not in lock[name]:
            problems.append(f"{name} {dist.version} is not a locked version {sorted(lock[name])}")
    return problems


def main(argv: list[str]) -> int:
    lock_path = pathlib.Path(argv[1] if len(argv) > 1 else "uv.lock")
    venv = pathlib.Path(sys.prefix).resolve()
    if venv == pathlib.Path(sys.base_prefix).resolve():
        print("check_locked_env: not running inside a virtual environment", file=sys.stderr)
        return 2
    problems = check(locked_versions(lock_path.read_text()), venv)
    for p in problems:
        print(f"::error::{p}", file=sys.stderr)
    if problems:
        return 1
    print(f"locked environment OK ({venv})")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
