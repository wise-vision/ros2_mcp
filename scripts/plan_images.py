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
"""Decide which Docker images a workflow run builds and which tags it pushes.

Tag scheme (README "Docker image tags"):

=====================  =========================================  ==========================
tag                    contents                                   moves?
=====================  =========================================  ==========================
``<distro>``           latest release                             yes: each release + weekly
                                                                  security rebuild
``<distro>-<YYMM>``    that release                               never (refused if present)
``<distro>-main``      main branch (merged, not yet released)     every merge to main
``latest``             ``<latest distro>`` (newest LTS)           with ``<distro>``
=====================  =========================================  ==========================

Experimental distros (Rolling) get floating tags only: Rolling changes in place, so a frozen
``rolling-<YYMM>`` image would soon stop talking to a current Rolling graph.

Every tag is also published per architecture as ``<tag>-amd64`` / ``<tag>-arm64``.

Modes, from the triggering event:

- ``test``     pull_request: build + smoke test, push nothing.
- ``main``     push to main (or a manual run without ``release``): ``<distro>-main``.
- ``release``  push of a ``YYMM`` tag, or a manual run with ``release``: immutable
               ``<distro>-<YYMM>`` + floating ``<distro>`` (+ ``latest``).
- ``refresh``  weekly schedule: rebuild the latest release on fresh base images and move only
               the floating tags. Immutable tags are never rebuilt.

The distro list is read from ``.github/distros.json`` *at the ref being built*, so rebuilding an
old release only builds the distros that release supported (releases before the file existed
shipped Humble and Jazzy).

As a CLI (in CI) it reads the event from the environment, inspects the git checkout and writes
``mode``, ``ref``, ``release``, ``has_work``, ``immutable`` and ``matrix`` to ``$GITHUB_OUTPUT``.
"""

from __future__ import annotations

import json
import os
import re
import subprocess
import sys

RELEASE_RE = re.compile(r"^[0-9]{4}$")
DISTROS_FILE = ".github/distros.json"
# What every release shipped before .github/distros.json existed.
LEGACY_DISTROS = {"humble": {"status": "supported"}, "jazzy": {"status": "supported", "default": True}}
STATUSES = {"supported", "experimental"}


class PlanError(Exception):
    pass


def validate_distros(distros: dict) -> None:
    if not distros:
        raise PlanError("no distros configured")
    for name, spec in distros.items():
        if not re.fullmatch(r"[a-z]+", name):
            raise PlanError(f"invalid distro name {name!r}")
        if spec.get("status") not in STATUSES:
            raise PlanError(f"{name}: status must be one of {sorted(STATUSES)}")
    for flag in ("default", "latest"):
        flagged = [n for n, s in distros.items() if s.get(flag)]
        if len(flagged) > 1:
            raise PlanError(f"more than one distro marked {flag}: {flagged}")
        if flagged and distros[flagged[0]]["status"] != "supported":
            raise PlanError(f"{flag} distro {flagged[0]} must be supported, not experimental")


def resolve_mode(event: str, ref_type: str, ref_name: str, input_release: str,
                 release_tags: list[str]) -> tuple[str, str]:
    """``(mode, release)`` for the triggering event. ``release`` is '' outside release/refresh."""
    if event == "pull_request":
        return "test", ""
    if event == "push" and ref_type == "tag":
        release = ref_name
    elif event == "workflow_dispatch" and input_release:
        release = input_release
    elif event == "schedule":
        released = sorted(t for t in release_tags if RELEASE_RE.match(t))
        return ("refresh", released[-1]) if released else ("refresh", "")
    elif event in ("push", "workflow_dispatch"):
        return "main", ""
    else:
        raise PlanError(f"unsupported event {event!r}")
    if not RELEASE_RE.match(release):
        raise PlanError(f"release must be a YYMM tag like 2610, got {release!r}")
    if release not in release_tags:
        raise PlanError(f"release {release} is not a tag in this repository")
    return "release", release


def plan_tags(mode: str, release: str, distros: dict) -> list[dict]:
    """One matrix entry per distro: the tags to push and which of them must not already exist."""
    if mode == "refresh" and not release:
        return []  # nothing released yet: nothing to refresh
    entries = []
    for name in sorted(distros):
        spec = distros[name]
        experimental = spec["status"] == "experimental"
        floating = [name] + (["latest"] if spec.get("latest") else [])
        if mode == "test":
            tags, immutable = [], []
        elif mode == "main":
            tags, immutable = [f"{name}-main"], []
        elif mode == "release":
            immutable = [] if experimental else [f"{name}-{release}"]
            tags = immutable + floating
        elif mode == "refresh":
            tags, immutable = floating, []
        else:
            raise PlanError(f"unknown mode {mode!r}")
        entries.append({
            "distro": name,
            "experimental": experimental,
            "tags": " ".join(tags),
            "immutable": " ".join(immutable),
        })
    return entries


def plan(event: str, ref_type: str, ref_name: str, input_release: str, sha: str,
         release_tags: list[str], distros_at) -> dict:
    """Full plan. ``distros_at(ref)`` returns the parsed distros mapping at a git ref (or None)."""
    mode, release = resolve_mode(event, ref_type, ref_name, input_release, release_tags)
    ref = release or sha
    distros = distros_at(ref)
    if distros is None:
        if mode in ("test", "main"):
            raise PlanError(f"{DISTROS_FILE} is missing at {ref}")
        distros = LEGACY_DISTROS
    validate_distros(distros)
    matrix = plan_tags(mode, release, distros)
    return {
        "mode": mode,
        "ref": ref,
        "release": release,
        "has_work": bool(matrix),
        "immutable": " ".join(t for e in matrix for t in e["immutable"].split()),
        "matrix": {"include": matrix},
    }


def _git(*args: str) -> str:
    return subprocess.run(["git", *args], check=True, capture_output=True, text=True).stdout


def _distros_at(ref: str):
    try:
        raw = _git("show", f"{ref}:{DISTROS_FILE}")
    except subprocess.CalledProcessError:
        return None
    return json.loads(raw)["distros"]


def main() -> int:
    env = os.environ
    try:
        result = plan(
            event=env["GITHUB_EVENT_NAME"],
            ref_type=env.get("GITHUB_REF_TYPE", ""),
            ref_name=env.get("GITHUB_REF_NAME", ""),
            input_release=env.get("INPUT_RELEASE", "").strip(),
            sha=env["GITHUB_SHA"],
            release_tags=_git("tag", "--list").split(),
            distros_at=_distros_at,
        )
    except PlanError as exc:
        print(f"::error::{exc}")
        return 1
    print(json.dumps(result, indent=2))
    out = env.get("GITHUB_OUTPUT")
    if out:
        with open(out, "a", encoding="utf-8") as f:
            f.writelines(f"{key}={result[key]}\n" for key in ("mode", "ref", "release", "immutable"))
            f.write(f"has_work={'true' if result['has_work'] else 'false'}\n")
            f.write(f"matrix={json.dumps(result['matrix'], separators=(',', ':'))}\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
