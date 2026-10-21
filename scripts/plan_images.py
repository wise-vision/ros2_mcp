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
``<distro>-<release>`` that release (``2610``, ``2610.1``)        never (refused if present)
``<distro>-main``      main branch (merged, not yet released)     every merge to main
``latest``             ``<latest distro>`` (newest LTS)           with ``<distro>``
=====================  =========================================  ==========================

Experimental distros (Rolling) get floating tags only: Rolling changes in place, so a frozen
``rolling-<YYMM>`` image would soon stop talking to a current Rolling graph.

Every tag is also published per architecture as ``<tag>-amd64`` / ``<tag>-arm64``.

Modes, from the triggering event:

- ``test``     pull_request: build + smoke test, push nothing.
- ``main``     push to main (or a manual run on ``main`` without ``release``): ``<distro>-main``.
               A manual run on any other branch is refused: ``-main`` means the main branch.
- ``release``  push of a release tag, or a manual run with ``release``: immutable
               ``<distro>-<release>``, plus the floating ``<distro>`` (+ ``latest``) only when it
               is the newest release. Backfilling an older release never moves floating tags.

Release tags are ``YYMM`` (``2610``) or, for another release in the same month, ``YYMM.N``
(``2610.1``, ``2610.2``, ...). They order numerically: 2610 < 2610.1 < 2610.10 < 2611. A release
whose source carries ``.github/distros.json`` must also carry that version in ``server.json`` and
``pyproject.toml`` (release prep bumps both before tagging): the plan fails before anything builds
otherwise, so neither the registry metadata nor the generated docs can claim a version the image is
not.
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

RELEASE_RE = re.compile(r"^[0-9]{4}(\.[1-9][0-9]*)?$")
DISTROS_FILE = ".github/distros.json"
# What every release shipped before .github/distros.json existed. Only releases up to
# LAST_LEGACY_RELEASE may lack the file; a newer release without it is refused, never treated as
# legacy (that would silently drop distros and skip the version gate).
LEGACY_DISTROS = {"humble": {"status": "supported"}, "jazzy": {"status": "supported", "default": True}}
LAST_LEGACY_RELEASE = "2610"
STATUSES = {"supported", "experimental"}


class PlanError(Exception):
    pass


def release_key(tag: str) -> tuple[int, int]:
    """Numeric order of a release tag: ``2610`` -> (2610, 0), ``2610.10`` -> (2610, 10)."""
    yymm, _, n = tag.partition(".")
    return int(yymm), int(n or 0)


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
        released = [t for t in release_tags if RELEASE_RE.match(t)]
        return ("refresh", max(released, key=release_key)) if released else ("refresh", "")
    elif event == "push" or (event == "workflow_dispatch" and ref_name == "main"):
        return "main", ""
    elif event == "workflow_dispatch":
        raise PlanError(f"a manual run without `release` publishes <distro>-main, so it must run on "
                        f"main, not {ref_name!r}")
    else:
        raise PlanError(f"unsupported event {event!r}")
    if not RELEASE_RE.match(release):
        raise PlanError(f"release must be a YYMM or YYMM.N tag like 2610 or 2610.1, got {release!r}")
    if release not in release_tags:
        raise PlanError(f"release {release} is not a tag in this repository")
    return "release", release


def is_newest(release: str, release_tags: list[str]) -> bool:
    """True when no release tag orders after ``release``."""
    return all(release_key(t) <= release_key(release) for t in release_tags if RELEASE_RE.match(t))


def plan_tags(mode: str, release: str, distros: dict, newest: bool = True) -> list[dict]:
    """One matrix entry per distro: the tags to push and which of them must not already exist.
    Distros with nothing to push (Rolling in an older-release backfill) are left out, except in
    ``test`` mode, which builds everything and pushes nothing."""
    if mode == "refresh" and not release:
        return []  # nothing released yet: nothing to refresh
    entries = []
    for name in sorted(distros):
        spec = distros[name]
        experimental = spec["status"] == "experimental"
        floating = [name] + (["latest"] if spec.get("latest") else [])
        if mode == "release" and not newest:
            floating = []  # backfilling an older release must not move the floating tags back
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
        if mode != "test" and not tags:
            continue
        entries.append({
            "distro": name,
            "experimental": experimental,
            "tags": " ".join(tags),
            "immutable": " ".join(immutable),
        })
    return entries


def plan(event: str, ref_type: str, ref_name: str, input_release: str, sha: str,
         release_tags: list[str], distros_at, versions_at, on_main=lambda ref: True) -> dict:
    """Full plan. ``distros_at(ref)`` returns the parsed distros mapping at a git ref (or None);
    ``versions_at(ref)`` returns ``{file: version}`` for every file that must carry the release;
    ``on_main(ref)`` says whether ``ref`` is in main's history (releases are cut on main)."""
    mode, release = resolve_mode(event, ref_type, ref_name, input_release, release_tags)
    ref = release or sha
    if release and not on_main(ref):
        raise PlanError(f"release {release} is not on main; release tags are cut on main")
    distros = distros_at(ref)
    if distros is None:
        if mode in ("test", "main") or not release or release_key(release) > release_key(LAST_LEGACY_RELEASE):
            raise PlanError(f"{DISTROS_FILE} is missing at {ref}")
        distros = LEGACY_DISTROS
    elif mode in ("release", "refresh") and release:
        # refresh too: a tag whose release was refused must not become the floating tags' target.
        stale = {f: v for f, v in versions_at(ref).items() if v != release}
        if stale:
            raise PlanError(f"release {release} is tagged on a commit whose versions are {stale}; "
                            f"bump them (and server.json's image tag) to {release!r} before tagging, "
                            f"or delete the tag")
    validate_distros(distros)
    newest = mode != "release" or is_newest(release, release_tags)
    matrix = plan_tags(mode, release, distros, newest)
    return {
        "mode": mode,
        "ref": ref,
        "release": release,
        "newest": newest,
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


def _show(ref: str, path: str):
    try:
        return _git("show", f"{ref}:{path}")
    except subprocess.CalledProcessError:
        return None


def versions_from(server_json: str | None, pyproject_version: str | None) -> dict:
    """Every version string a release commit must carry, keyed by where it lives. A missing or
    unparsable value is None (and so never equals a release)."""
    versions = {"pyproject.toml": pyproject_version, "server.json": None}
    try:
        server = json.loads(server_json or "{}")
        versions["server.json"] = server.get("version")
        for pkg in server.get("packages", []):
            if pkg.get("registryType") == "oci":
                # docker.io/wisevision/ros2_mcp:jazzy-2610.1 -> 2610.1
                tag = pkg["identifier"].rsplit(":", 1)[-1]
                versions[f"server.json image {pkg['identifier']}"] = tag.split("-", 1)[1] if "-" in tag else None
    except (ValueError, TypeError, KeyError, AttributeError):
        versions["server.json"] = None
    return versions


def _versions_at(ref: str) -> dict:
    import tomllib  # Python 3.11+: only the CI plan job calls this; tests import the module on 3.10

    try:
        pyproject = tomllib.loads(_show(ref, "pyproject.toml") or "")["project"]["version"]
    except (tomllib.TOMLDecodeError, KeyError):
        pyproject = None
    return versions_from(_show(ref, "server.json"), pyproject)


def _on_main(ref: str) -> bool:
    """Ancestry check against origin/main (the CI checkouts fetch every branch). A missing
    origin/main fails closed: it raises, and the plan is refused."""
    _git("rev-parse", "--verify", "origin/main")
    return subprocess.run(["git", "merge-base", "--is-ancestor", f"{ref}^{{commit}}", "origin/main"],
                          check=False).returncode == 0


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
            versions_at=_versions_at,
            on_main=_on_main,
        )
    except subprocess.CalledProcessError as exc:
        print(f"::error::git {' '.join(exc.cmd[1:])} failed: {exc.stderr or exc}")
        return 1
    except PlanError as exc:
        print(f"::error::{exc}")
        return 1
    except (KeyError, ValueError, TypeError, OSError) as exc:  # malformed distros.json, missing env
        print(f"::error::cannot plan images: {type(exc).__name__}: {exc}")
        return 1
    result["sha"] = _git("rev-parse", f"{result['ref']}^{{commit}}").strip()
    print(json.dumps(result, indent=2))
    out = env.get("GITHUB_OUTPUT")
    if out:
        with open(out, "a", encoding="utf-8") as f:
            f.writelines(f"{key}={result[key]}\n" for key in ("mode", "ref", "sha", "release", "immutable"))
            f.write(f"has_work={'true' if result['has_work'] else 'false'}\n")
            f.write(f"matrix={json.dumps(result['matrix'], separators=(',', ':'))}\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
