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
"""MCP ``initialize`` must report the ROS2 MCP release, not the MCP SDK version."""

import pathlib
from importlib.metadata import version as pkg_version

try:
    import tomllib as toml  # py311+
except ModuleNotFoundError:  # py310
    import tomli as toml

from server import server as srv

REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]


def _pyproject_version() -> str:
    with open(REPO_ROOT / "pyproject.toml", "rb") as f:
        return str(toml.load(f)["project"]["version"])


def test_server_info_version_is_the_package_version():
    init = srv.app.create_initialization_options()
    assert init.server_name == "ROS2 MCP"
    assert init.server_version == pkg_version("mcp_server_ros_2")
    assert init.server_version == _pyproject_version()


def test_server_info_version_is_not_the_sdk_version():
    init = srv.app.create_initialization_options()
    assert init.server_version != pkg_version("mcp")


def test_package_version_tracks_the_release_scheme():
    # Releases are tagged YYMM (see CHANGELOG.md); the package version follows it.
    v = _pyproject_version()
    assert v.isdigit() and len(v) == 4, v
    changelog = (REPO_ROOT / "CHANGELOG.md").read_text(encoding="utf-8")
    assert f"## {v} " in changelog or f"## {v}\n" in changelog


def test_resolve_version_falls_back_when_not_installed(monkeypatch):
    from importlib.metadata import PackageNotFoundError

    def _missing(_name):
        raise PackageNotFoundError(_name)

    monkeypatch.setattr(srv, "_pkg_version", _missing)
    assert srv._resolve_server_version() == _pyproject_version()
