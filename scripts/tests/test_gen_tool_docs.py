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

PROVENANCE = {"package_version": "9.9.9", "git_describe": "t-0-gabc", "source_sha": "a" * 40}


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
    assert PROVENANCE["source_sha"] in md


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
    newer = dict(PROVENANCE, git_describe="t-5-gdef", source_sha="b" * 40)
    assert _run(_fake_registry(), committed, "--check", provenance=newer) == 0
    assert _run(_fake_registry(), committed, "--check", "--strict-provenance", provenance=newer) == 1
    assert _run(_fake_registry(), committed, "--check", "--strict-provenance") == 0


def test_check_missing_committed_files_exits_1(tmp_path):
    assert _run(_fake_registry(), tmp_path / "nothing", "--check") == 1


def test_compute_provenance_reads_pyproject():
    prov = gen.compute_provenance(REPO_ROOT)
    assert list(prov.keys()) == ["package_version", "git_describe", "source_sha"]
    assert prov["package_version"] != "unknown"
    assert len(prov["source_sha"]) == 40 or prov["source_sha"] == "unknown"


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
