#
#  Copyright (C) 2025 wisevision
#
#  SPDX-License-Identifier: MPL-2.0
#
#  This Source Code Form is subject to the terms of the Mozilla Public
#  License, v. 2.0. If a copy of the MPL was not distributed with this
#  file, You can obtain one at https://mozilla.org/MPL/2.0/.
#
"""Tests for scripts/gen_tool_docs.py (tool-docs generator + drift gate).

The fake-registry tests need no ROS. The real-registry tests import
``server.server`` and therefore need a sourced ROS 2 environment; they are
skipped when ``rclpy`` is not importable.
"""
import copy
import hashlib
import importlib.util
import json
import pathlib
import sys
from types import SimpleNamespace

import pytest

REPO_ROOT = pathlib.Path(__file__).resolve().parents[2]
SCRIPT = REPO_ROOT / "scripts" / "gen_tool_docs.py"


def _load_gen():
    if not SCRIPT.exists():  # keeps the RED run readable before the script exists
        return None
    spec = importlib.util.spec_from_file_location("gen_tool_docs", SCRIPT)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


gen = _load_gen()

PROVENANCE = {"release": "9999", "package_version": "9999", "git_describe": "t-0-gabc", "source_sha": "a" * 40}


class FakeHandler:
    def __init__(self, name, description, schema, ui_only=False):
        self.name = name
        self.ui_only = ui_only
        self._description = description
        self._schema = schema

    def get_tool_description(self):
        return SimpleNamespace(
            name=self.name, description=self._description, inputSchema=copy.deepcopy(self._schema)
        )


def _fake_registry():
    return {
        "ros2_topic_publish": FakeHandler(
            "ros2_topic_publish",
            "Publish a message | to a topic.\nSecond line.",
            {
                "type": "object",
                "properties": {
                    "topic_name": {"type": "string", "description": "Topic"},
                    "message": {"type": "object"},
                    "count": {"type": "integer", "default": 1},
                },
                "required": ["topic_name", "message"],
            },
        ),
        "ros2_topic_list": FakeHandler(
            "ros2_topic_list", "List topics.", {"type": "object", "properties": {}}
        ),
        "ros2_stream_stop": FakeHandler(
            "ros2_stream_stop",
            "Stop a stream.",
            {"type": "object", "properties": {"ids": {"type": "array", "items": {"type": "string"}}}},
            ui_only=True,
        ),
    }


def _mutating(name):
    return name == "ros2_topic_publish"


def _run(registry, out_dir, *extra, provenance=PROVENANCE, is_mutating=_mutating):
    argv = ["--out-dir", str(out_dir), *extra]
    return gen.main(argv, registry=registry, provenance=provenance, is_mutating=is_mutating)


@pytest.fixture
def committed(tmp_path):
    """A docs dir generated from the unchanged fake registry."""
    out = tmp_path / "generated"
    assert _run(_fake_registry(), out) == 0
    return out


# --- output shape -----------------------------------------------------------

def test_json_shape_sorted_and_stable(committed):
    raw = (committed / "tools.json").read_text(encoding="utf-8")
    assert raw.endswith("}\n")
    data = json.loads(raw)
    assert list(data.keys()) == ["provenance", "tools"]
    assert data["provenance"] == PROVENANCE
    names = [t["name"] for t in data["tools"]]
    assert names == sorted(names) == ["ros2_stream_stop", "ros2_topic_list", "ros2_topic_publish"]
    for t in data["tools"]:
        assert list(t.keys()) == ["name", "description", "input_schema", "mutating"]
    by = {t["name"]: t for t in data["tools"]}
    assert by["ros2_topic_publish"]["mutating"] is True
    assert by["ros2_topic_list"]["mutating"] is False
    assert raw == json.dumps(data, indent=2, ensure_ascii=False) + "\n"


def test_mutating_unknown_without_tool_safety(tmp_path):
    out = tmp_path / "g"
    assert _run(_fake_registry(), out, is_mutating=None) == 0
    data = json.loads((out / "tools.json").read_text())
    assert {t["mutating"] for t in data["tools"]} == {"unknown"}


def test_markdown_frontmatter_table_sections_and_badge(committed):
    md = (committed / "tools.md").read_text(encoding="utf-8")
    assert md.startswith("---\ntitle: Tool reference\n")
    assert md.endswith("\n") and not md.endswith("\n\n")
    assert "| Tool | Mutates robot state | Summary |" in md
    for name in ("ros2_stream_stop", "ros2_topic_list", "ros2_topic_publish"):
        assert f"## `{name}`" in md
    # badge only on the mutating tool
    pub = md.split("## `ros2_topic_publish`")[1]
    assert "⚠ mutates robot state" in pub
    lst = md.split("## `ros2_topic_list`")[1].split("## `")[0]
    assert "⚠ mutates robot state" not in lst
    # parameter table with required flag, default and escaped pipe
    assert "| `topic_name` | string | yes | Topic |" in pub
    assert "| `count` | integer | no | default: `1` |" in pub
    assert "Publish a message \\| to a topic." in md
    assert "array<string>" in md
    assert PROVENANCE["source_sha"][:7] in md
    assert f"release `{PROVENANCE['release']}`" in md


# --- determinism --------------------------------------------------------------

def _sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def test_byte_deterministic_across_two_runs(tmp_path):
    a, b = tmp_path / "a", tmp_path / "b"
    assert _run(_fake_registry(), a) == 0
    # registry insertion order must not matter
    rev = dict(reversed(list(_fake_registry().items())))
    assert _run(rev, b) == 0
    for f in ("tools.json", "tools.md"):
        assert _sha(a / f) == _sha(b / f)


# --- drift gate: unchanged → 0, five RED proofs → 1 ----------------------------

def test_check_unchanged_registry_exits_0(committed, capsys):
    assert _run(_fake_registry(), committed, "--check") == 0


def _added(r):
    r["ros2_new_tool"] = FakeHandler("ros2_new_tool", "New.", {"type": "object", "properties": {}})


def _removed(r):
    del r["ros2_topic_list"]


def _renamed(r):
    h = r.pop("ros2_topic_list")
    h.name = "ros2_topics_list"
    r[h.name] = h


def _schema_changed(r):
    r["ros2_topic_publish"]._schema["properties"]["qos"] = {"type": "string"}


def _description_changed(r):
    r["ros2_topic_list"]._description = "List ROS 2 topics."


@pytest.mark.parametrize(
    "mutate",
    [_added, _removed, _renamed, _schema_changed, _description_changed],
    ids=["added", "removed", "renamed", "schema", "description"],
)
def test_check_red_proofs_exit_1(committed, capsys, mutate):
    reg = _fake_registry()
    mutate(reg)
    assert _run(reg, committed, "--check") == 1
    out = capsys.readouterr().out
    assert "--- a/" in out and "+++ b/" in out  # unified diff printed


def test_check_does_not_write(committed):
    before = (committed / "tools.json").read_bytes()
    reg = _fake_registry()
    _added(reg)
    _run(reg, committed, "--check")
    assert (committed / "tools.json").read_bytes() == before


def test_check_ignores_provenance_unless_strict(committed):
    newer = dict(PROVENANCE, release="9998", git_describe="t-5-gdef", source_sha="b" * 40)
    assert _run(_fake_registry(), committed, "--check", provenance=newer) == 0
    assert _run(_fake_registry(), committed, "--check", "--strict-provenance", provenance=newer) == 1
    assert _run(_fake_registry(), committed, "--check", "--strict-provenance") == 0


def test_check_missing_committed_files_exits_1(tmp_path):
    assert _run(_fake_registry(), tmp_path / "nothing", "--check") == 1


def test_compute_provenance_reads_pyproject():
    prov = gen.compute_provenance(REPO_ROOT)
    assert list(prov.keys()) == ["release", "package_version", "git_describe", "source_sha"]
    assert prov["package_version"] != "unknown"
    assert len(prov["source_sha"]) == 40 or prov["source_sha"] == "unknown"


# --- provenance against a real throwaway git history ---------------------------

def _git(repo, *args):
    import subprocess

    return subprocess.run(
        ["git", "-C", str(repo), "-c", "user.name=t", "-c", "user.email=t@t", "-c", "commit.gpgsign=false",
         "-c", "tag.gpgsign=false", *args],
        check=True, capture_output=True, text=True,
    ).stdout.strip()


def _commit(repo, path, text, msg):
    p = repo / path
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(text, encoding="utf-8")
    _git(repo, "add", "-A")
    _git(repo, "commit", "-q", "-m", msg)
    return _git(repo, "rev-parse", "HEAD")


def _repo(tmp_path, version):
    repo = tmp_path / "repo"
    repo.mkdir()
    _git(repo, "init", "-q", "-b", "main")
    _commit(repo, "pyproject.toml", f'[project]\nname = "x"\nversion = "{version}"\n', "init")
    return repo


def _history(tmp_path, version):
    """2606 on a server commit, one more server commit, then a docs-only commit tagged 2610.

    Mirrors the real history where the page said ``2606-3-g47b4918`` at tag 2610.
    """
    repo = _repo(tmp_path, version)
    _commit(repo, "server/a.py", "a = 1\n", "server a")
    _git(repo, "tag", "2606")
    src = _commit(repo, "server/a.py", "a = 2\n", "server b")
    _commit(repo, "docs/x.md", "docs\n", "docs only")
    _git(repo, "tag", "not-a-release")
    _git(repo, "tag", "2610")
    return repo, src


def test_provenance_head_at_release_tag_names_the_tag(tmp_path):
    repo, src = _history(tmp_path, "0.1.0")
    prov = gen.compute_provenance(repo)
    assert prov["release"] == "2610"
    assert prov["source_sha"] == src
    assert prov["git_describe"].startswith("2606-1-g")  # json keeps the exact source describe


def test_provenance_tag_wins_over_stale_pyproject_version(tmp_path):
    # A release tag on a commit whose pyproject was not bumped must surface as a
    # strict-provenance diff, so the tag (not pyproject) is what gets stamped.
    repo, _ = _history(tmp_path, "2606")
    assert gen.compute_provenance(repo)["release"] == "2610"


def test_provenance_between_releases_uses_package_release_version(tmp_path):
    repo, _ = _history(tmp_path, "2610")
    newer = _commit(repo, "server/a.py", "a = 3\n", "server c")
    prov = gen.compute_provenance(repo)
    assert prov["release"] == "2610"
    assert prov["package_version"] == "2610"
    assert prov["source_sha"] == newer


def test_provenance_release_prep_before_tag_is_stable_across_tagging(tmp_path):
    # Release prep bumps pyproject to the upcoming tag; the stamp must not change
    # when the tag is then pushed, or the strict tag gate could never pass.
    repo, _ = _history(tmp_path, "2610")
    _commit(repo, "pyproject.toml", '[project]\nname = "x"\nversion = "2611"\n', "release prep")
    before = gen.compute_provenance(repo)
    _git(repo, "tag", "2611")
    assert before["release"] == "2611"
    assert gen.compute_provenance(repo) == before


def test_provenance_non_release_version_falls_back_to_nearest_release_tag(tmp_path):
    repo, _ = _history(tmp_path, "0.1.0")
    _commit(repo, "server/a.py", "a = 3\n", "server c")
    assert gen.compute_provenance(repo)["release"] == "2610"


def test_provenance_without_any_release(tmp_path):
    repo = _repo(tmp_path, "0.1.0")
    _commit(repo, "server/a.py", "a = 1\n", "server a")
    _git(repo, "tag", "v-something")
    assert gen.compute_provenance(repo)["release"] == "unreleased"


def test_markdown_provenance_line_names_release_not_package_version(tmp_path):
    prov = {"release": "2610", "package_version": "2610", "git_describe": "2610", "source_sha": "c" * 40}
    assert _run(_fake_registry(), tmp_path, provenance=prov) == 0
    md = (tmp_path / "tools.md").read_text(encoding="utf-8")
    line = next(l for l in md.split("\n") if l.startswith("Generated from ROS2 MCP"))
    assert line == "Generated from ROS2 MCP release `2610` (source commit `ccccccc`)."
    assert "package version" not in md


# --- real registry (needs ROS 2) ----------------------------------------------

def _need_ros():
    pytest.importorskip("rclpy")


def test_enumeration_equals_real_handler_registry():
    _need_ros()
    registry = gen.load_registry()
    import server.server as srv  # noqa: E402  (after load_registry sets env/path)

    tools = gen.collect_tools(registry, is_mutating=None)
    assert [t["name"] for t in tools] == sorted(srv.tool_handlers.keys())
    assert len(tools) >= 20
    for t in tools:
        assert t["description"]
        assert t["input_schema"].get("type") == "object"


def test_real_registry_byte_deterministic(tmp_path):
    _need_ros()
    reg = gen.load_registry()
    a, b = tmp_path / "a", tmp_path / "b"
    assert gen.main(["--out-dir", str(a)], registry=reg, provenance=PROVENANCE) == 0
    assert gen.main(["--out-dir", str(b)], registry=gen.load_registry(), provenance=PROVENANCE) == 0
    for f in ("tools.json", "tools.md"):
        assert _sha(a / f) == _sha(b / f)
