#
#  Copyright (C) 2025 wisevision
#
#  SPDX-License-Identifier: MPL-2.0
#
#  This Source Code Form is subject to the terms of the Mozilla Public
#  License, v. 2.0. If a copy of the MPL was not distributed with this
#  file, You can obtain one at https://mozilla.org/MPL/2.0/.
#
"""Tests for scripts/plan_images.py (Docker tag plan) and the files that must agree with it:
.github/distros.json, the CI matrices, server.json and the README support grid.
Pure Python: no ROS, no network, no Docker."""
import importlib.util
import json
import pathlib
import re
import sys

import pytest

REPO_ROOT = pathlib.Path(__file__).resolve().parents[2]
SCRIPT = REPO_ROOT / "scripts" / "plan_images.py"
DISTROS = json.loads((REPO_ROOT / ".github" / "distros.json").read_text())["distros"]


def _load():
    spec = importlib.util.spec_from_file_location("plan_images", SCRIPT)
    mod = importlib.util.module_from_spec(spec)
    sys.modules["plan_images"] = mod
    spec.loader.exec_module(mod)
    return mod


pi = _load()

SAMPLE = {
    "humble": {"status": "supported"},
    "jazzy": {"status": "supported", "default": True},
    "lyrical": {"status": "supported", "latest": True},
    "rolling": {"status": "experimental"},
}
TAGS = ["2508", "2511", "2606", "2610", "2610.1", "not-a-release"]


def run(event, ref_type="", ref_name="", release="", tags=TAGS, distros=SAMPLE, server_version=None):
    # By default server.json at the released ref carries the release (the correct release commit).
    def versions(ref):
        return {"server.json": server_version if server_version is not None else ref, "pyproject.toml": ref}
    return pi.plan(event, ref_type, ref_name, release, "abc123", tags, lambda ref: distros, versions)


def by_distro(result):
    return {e["distro"]: e for e in result["matrix"]["include"]}


# --- modes -------------------------------------------------------------------------------------

def test_pull_request_builds_every_distro_and_pushes_nothing():
    r = run("pull_request")
    assert r["mode"] == "test" and r["ref"] == "abc123"
    assert sorted(by_distro(r)) == ["humble", "jazzy", "lyrical", "rolling"]
    assert all(e["tags"] == "" and e["immutable"] == "" for e in by_distro(r).values())
    assert by_distro(r)["rolling"]["experimental"] is True
    assert by_distro(r)["jazzy"]["experimental"] is False


def test_push_to_main_publishes_only_main_tags():
    r = run("push", "branch", "main")
    assert r["mode"] == "main"
    assert {d: e["tags"] for d, e in by_distro(r).items()} == {
        "humble": "humble-main", "jazzy": "jazzy-main", "lyrical": "lyrical-main", "rolling": "rolling-main"}
    assert r["immutable"] == ""


def test_main_never_moves_a_floating_release_tag():
    r = run("push", "branch", "main")
    for e in by_distro(r).values():
        assert e["distro"] not in e["tags"].split() and "latest" not in e["tags"].split()


def test_release_tag_push():
    r = run("push", "tag", "2610")
    assert (r["mode"], r["ref"], r["release"]) == ("release", "2610", "2610")
    tags = {d: e["tags"].split() for d, e in by_distro(r).items()}
    assert tags["humble"] == ["humble-2610", "humble"]
    assert tags["jazzy"] == ["jazzy-2610", "jazzy"]
    assert tags["lyrical"] == ["lyrical-2610", "lyrical", "latest"]
    assert tags["rolling"] == ["rolling"], "rolling gets no immutable tag"
    assert r["immutable"].split() == ["humble-2610", "jazzy-2610", "lyrical-2610"]


def test_manual_backfill_of_existing_release():
    r = run("workflow_dispatch", release="2606")
    assert (r["mode"], r["ref"]) == ("release", "2606")


def test_manual_run_without_release_is_main():
    assert run("workflow_dispatch")["mode"] == "main"


def test_same_month_release():
    r = run("push", "tag", "2610.1")
    assert (r["mode"], r["release"]) == ("release", "2610.1")
    assert by_distro(r)["jazzy"]["tags"].split() == ["jazzy-2610.1", "jazzy"]
    assert by_distro(r)["lyrical"]["tags"].split() == ["lyrical-2610.1", "lyrical", "latest"]
    assert by_distro(r)["rolling"]["tags"].split() == ["rolling"]


@pytest.mark.parametrize("tags, latest", [
    (["2606", "2610", "2610.1"], "2610.1"),
    (["2610", "2610.2", "2610.10"], "2610.10"),
    (["2610.9", "2611", "2510.3"], "2611"),
])
def test_releases_order_numerically(tags, latest):
    assert run("schedule", tags=tags)["release"] == latest


def test_release_commit_must_carry_its_version_in_server_json():
    with pytest.raises(pi.PlanError, match="server.json"):
        run("push", "tag", "2610.1", server_version="2610")


def test_release_commit_must_carry_its_version_in_pyproject():
    with pytest.raises(pi.PlanError, match="pyproject.toml"):
        pi.plan("push", "tag", "2610.1", "", "abc", TAGS, lambda ref: SAMPLE,
                lambda ref: {"server.json": "2610.1", "pyproject.toml": "2610"})


def test_refresh_refuses_a_latest_release_that_was_never_bumped():
    # 2610.9 pushed without the bump: its publish was refused, so the weekly refresh must not
    # quietly move :jazzy onto it either.
    with pytest.raises(pi.PlanError, match="2610.9"):
        pi.plan("schedule", "", "", "", "abc", ["2610", "2610.9"], lambda ref: SAMPLE,
                lambda ref: {"server.json": "2610", "pyproject.toml": "2610"})


def test_release_workflow_runs_the_same_gate():
    text = (REPO_ROOT / ".github" / "workflows" / "release.yml").read_text()
    gate, create = text.find("python3 scripts/plan_images.py;"), text.find("gh release create")
    assert 0 < gate < create, "release.yml must run the plan gate before creating the release"


def test_legacy_release_is_not_held_to_the_server_json_rule():
    # 2606/2610 predate distros.json and shipped server.json version "1.0.0".
    r = pi.plan("workflow_dispatch", "", "", "2610", "abc", TAGS, lambda ref: None,
                lambda ref: {"server.json": "1.0.0", "pyproject.toml": "0.1.0"})
    assert r["immutable"].split() == ["humble-2610", "jazzy-2610"]


def test_weekly_refresh_rebuilds_latest_release_floating_tags_only():
    r = run("schedule", tags=["2508", "2610", "not-a-release"])
    assert (r["mode"], r["ref"], r["release"]) == ("refresh", "2610", "2610")
    assert r["immutable"] == ""
    tags = {d: e["tags"].split() for d, e in by_distro(r).items()}
    assert tags == {"humble": ["humble"], "jazzy": ["jazzy"], "lyrical": ["lyrical", "latest"],
                    "rolling": ["rolling"]}


def test_refresh_with_no_release_yet_does_nothing():
    r = run("schedule", tags=["v1", "x"])
    assert r["has_work"] is False and r["matrix"]["include"] == []


# --- rejections --------------------------------------------------------------------------------

@pytest.mark.parametrize("release", ["26100", "v2610", "2610; touch /tmp/x", "abcd",
                                     "2610.", "2610.0", "2610.01", "2610.1.1", "2610-1"])
def test_malformed_release_is_rejected(release):
    with pytest.raises(pi.PlanError, match="YYMM"):
        run("workflow_dispatch", release=release)


def test_release_that_is_not_a_tag_is_rejected():
    with pytest.raises(pi.PlanError, match="not a tag"):
        run("workflow_dispatch", release="2699")


def test_unknown_event_is_rejected():
    with pytest.raises(pi.PlanError):
        run("issue_comment")


# --- distros at the built ref ------------------------------------------------------------------

def test_old_release_without_distros_file_builds_legacy_distros():
    r = pi.plan("workflow_dispatch", "", "", "2606", "abc", TAGS, lambda ref: None, lambda ref: {})
    assert sorted(by_distro(r)) == ["humble", "jazzy"]
    assert r["immutable"].split() == ["humble-2606", "jazzy-2606"]
    assert "latest" not in " ".join(e["tags"] for e in r["matrix"]["include"])


def test_main_without_distros_file_fails_loudly():
    with pytest.raises(pi.PlanError, match="missing"):
        pi.plan("push", "branch", "main", "", "abc", TAGS, lambda ref: None, lambda ref: {})


def test_distros_are_read_at_the_ref_being_built():
    seen = []
    pi.plan("workflow_dispatch", "", "", "2606", "abc", TAGS, lambda ref: seen.append(ref) or SAMPLE,
            lambda ref: {"server.json": ref})
    assert seen == ["2606"]


@pytest.mark.parametrize("bad, msg", [
    ({}, "no distros"),
    ({"Jazzy": {"status": "supported"}}, "invalid distro"),
    ({"jazzy": {"status": "lts"}}, "status"),
    ({"a": {"status": "supported", "latest": True}, "b": {"status": "supported", "latest": True}}, "more than one"),
    ({"rolling": {"status": "experimental", "latest": True}}, "must be supported"),
])
def test_invalid_distros_config(bad, msg):
    with pytest.raises(pi.PlanError, match=msg):
        run("pull_request", distros=bad)


# --- the real repository files agree -----------------------------------------------------------

def test_repo_distros_file_is_valid():
    pi.validate_distros(DISTROS)
    assert sum(1 for s in DISTROS.values() if s.get("default")) == 1
    assert sum(1 for s in DISTROS.values() if s.get("latest")) == 1


def test_server_tests_matrix_comes_from_distros_file():
    text = (REPO_ROOT / ".github" / "workflows" / "server_tests.yml").read_text()
    assert ".github/distros.json" in text
    assert "ros_distro: [" not in text, "hard-coded distro list in server_tests.yml"


def test_docker_workflow_matrix_comes_from_plan():
    text = (REPO_ROOT / ".github" / "workflows" / "docker_image.yml").read_text()
    assert "scripts/plan_images.py" in text
    assert "ros_distro: [" not in text, "hard-coded distro list in docker_image.yml"


def _glob_to_regex(glob):
    # GitHub filter syntax as used by our tag filters: [..] classes, '+' = one or more, '.' literal.
    return re.compile("^" + glob.replace(".", r"\.") + "$")


@pytest.mark.parametrize("workflow", ["docker_image.yml", "release.yml"])
def test_every_release_tag_triggers_the_workflow(workflow):
    # A release tag the workflow does not match would publish nothing, silently.
    text = (REPO_ROOT / ".github" / "workflows" / workflow).read_text()
    on_tags = text.split("tags:", 1)[1].split("\n\n", 1)[0]
    globs = re.findall(r"^\s*- '([^']+)'", on_tags, re.MULTILINE)
    assert globs, f"no tag filters in {workflow}"
    for tag in ["2610", "2610.1", "2610.12", "2611"]:
        assert pi.RELEASE_RE.match(tag)
        assert any(_glob_to_regex(g).match(tag) for g in globs), f"{workflow} ignores release {tag}"
    for tag in ["v2610", "2610-rc1", "latest"]:
        assert not any(_glob_to_regex(g).match(tag) for g in globs), f"{workflow} fires on {tag}"


def test_docs_generator_agrees_on_what_a_release_tag_is():
    gen = (REPO_ROOT / "scripts" / "gen_tool_docs.py").read_text()
    assert f're.compile(r"{pi.RELEASE_RE.pattern}")' in gen


def test_server_json_pins_the_default_distro_release_image():
    server = json.loads((REPO_ROOT / "server.json").read_text())
    default = next(n for n, s in DISTROS.items() if s.get("default"))
    identifiers = [p["identifier"] for p in server["packages"] if p["registryType"] == "oci"]
    assert identifiers == [f"docker.io/wisevision/ros2_mcp:{default}-{server['version']}"]


def test_readme_grid_lists_every_distro_with_its_status():
    readme = (REPO_ROOT / "README.md").read_text()
    grid = readme.split("## 🧭 Supported ROS 2 distributions", 1)[1].split("\n## ", 1)[0]
    rows = {}
    for line in grid.splitlines():
        m = re.match(r"\|\s*\**([A-Z][a-z]+) [A-Z][a-z]+\**\s*\|(.*)", line)
        if m:
            rows[m.group(1).lower()] = m.group(2)
    for name, spec in DISTROS.items():
        assert name in rows, f"README grid has no row for {name}"
        mark = "✅ Supported" if spec["status"] == "supported" else "🧪 Experimental"
        assert mark in rows[name], f"README row for {name} does not say {mark!r}"
    for name, row in rows.items():
        if name not in DISTROS:
            assert "❌" in row, f"README lists {name} as shipped but distros.json does not"
