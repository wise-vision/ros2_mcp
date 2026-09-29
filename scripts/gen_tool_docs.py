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
"""Generate ROS2 MCP tool reference docs from the REAL tool registry.

Imports ``server.server`` (which registers every tool handler at import time
via ``add_tool_handler``) and reads ``server.server.tool_handlers``. Each
handler's ``get_tool_description()`` is pure metadata, so no ROS graph is
needed, but importing the server pulls in ``rclpy``: run inside a sourced
ROS 2 (Jazzy) environment. See docs/generated/README.md.

Outputs (in --out-dir, default docs/generated):
  tools.json  sorted, indent=2, trailing newline, with a provenance stamp
  tools.md    Starlight-friendly reference page

--check regenerates to a temp dir and exits 1 with a unified diff when the
committed files differ (drift gate). Provenance is ignored by --check unless
--strict-provenance is given (used on release tags).

Provenance: package_version (pyproject), source_sha (last commit touching
server/ or pyproject.toml) and git_describe of that commit.
"""
from __future__ import annotations

import argparse
import difflib
import json
import os
import pathlib
import subprocess
import sys
import tempfile
from typing import Any, Callable, Mapping

REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
DEFAULT_OUT = REPO_ROOT / "docs" / "generated"
FILES = ("tools.json", "tools.md")
SOURCE_PATHS = ("server", "pyproject.toml")
MUTATING_BADGE = "⚠ mutates robot state"

# --------------------------------------------------------------------------
# Registry + metadata
# --------------------------------------------------------------------------


def load_registry() -> Mapping[str, Any]:
    """Import the real server module and return its tool handler registry."""
    # Deterministic output: never load optional custom prompt plugins.
    os.environ["MCP_CUSTOM_PROMPTS"] = "false"
    os.environ["MCP_PROMPTS_LOCAL"] = "false"
    if str(REPO_ROOT) not in sys.path:
        sys.path.insert(0, str(REPO_ROOT))
    # server.server parses CLI flags at import; hide ours from it.
    saved_argv = sys.argv
    sys.argv = [saved_argv[0]]
    try:
        import server.server as srv  # noqa: WPS433
    finally:
        sys.argv = saved_argv
    return srv.tool_handlers


def load_is_mutating() -> Callable[[str], bool] | None:
    try:
        from server.tool_safety import is_mutating  # type: ignore
    except ImportError:
        return None
    return is_mutating


def _normalize(obj: Any) -> Any:
    """Round-trip through JSON so schemas are plain, key-sorted data."""
    return json.loads(json.dumps(obj, sort_keys=True, ensure_ascii=False))


def collect_tools(registry: Mapping[str, Any], is_mutating: Callable[[str], bool] | None) -> list[dict]:
    tools = []
    for handler in registry.values():
        tool = handler.get_tool_description()
        name = tool.name
        mutating: bool | str = "unknown" if is_mutating is None else bool(is_mutating(name))
        tools.append(
            {
                "name": name,
                "description": (tool.description or "").strip(),
                "input_schema": _normalize(tool.inputSchema or {}),
                "mutating": mutating,
            }
        )
    tools.sort(key=lambda t: t["name"])
    return tools


def _run_git(root: pathlib.Path, *args: str) -> str:
    try:
        return subprocess.run(
            ["git", "-c", "safe.directory=*", "-C", str(root), *args], check=True, capture_output=True, text=True
        ).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return "unknown"


def compute_provenance(root: pathlib.Path = REPO_ROOT) -> dict:
    try:
        import tomllib as toml  # py311+
    except ImportError:  # pragma: no cover
        import tomli as toml  # type: ignore
    try:
        with open(root / "pyproject.toml", "rb") as f:
            version = str(toml.load(f)["project"]["version"])
    except (OSError, KeyError):
        version = "unknown"
    # The stamp names the last commit that changed the documented source, not
    # HEAD: committed docs cannot contain their own commit SHA, and docs-only
    # commits must not invalidate the stamp. Needs full history (fetch-depth 0).
    source_sha = _run_git(root, "log", "-1", "--format=%H", "--", *SOURCE_PATHS) or "unknown"
    describe = _run_git(root, "describe", "--tags", "--always", source_sha) if source_sha != "unknown" else "unknown"
    return {
        "package_version": version,
        "git_describe": describe,
        "source_sha": source_sha,
    }


# --------------------------------------------------------------------------
# Rendering
# --------------------------------------------------------------------------


def render_json(tools: list[dict], provenance: dict) -> str:
    doc = {"provenance": provenance, "tools": tools}
    return json.dumps(doc, indent=2, ensure_ascii=False) + "\n"


def _cell(text: Any) -> str:
    s = str(text).replace("\r", "").strip()
    s = " ".join(line.strip() for line in s.split("\n") if line.strip())
    return s.replace("|", "\\|")


def _type_str(schema: Mapping[str, Any]) -> str:
    t = schema.get("type")
    if isinstance(t, list):
        return " \\| ".join(str(x) for x in t)
    if t == "array" and isinstance(schema.get("items"), Mapping):
        return f"array<{_type_str(schema['items'])}>"
    if t:
        return str(t)
    for key in ("anyOf", "oneOf"):
        if isinstance(schema.get(key), list):
            return " \\| ".join(_type_str(s) for s in schema[key] if isinstance(s, Mapping))
    return "any"


def _param_notes(schema: Mapping[str, Any]) -> str:
    parts = []
    if schema.get("description"):
        parts.append(_cell(schema["description"]))
    if "enum" in schema:
        parts.append("one of: " + ", ".join(f"`{json.dumps(v, ensure_ascii=False)}`" for v in schema["enum"]))
    if "default" in schema:
        parts.append(f"default: `{json.dumps(schema['default'], ensure_ascii=False)}`")
    return _cell(" · ".join(parts)) if parts else ""


def _summary(desc: str) -> str:
    first = desc.strip().split("\n", 1)[0] if desc.strip() else ""
    return _cell(first)


def _mut_label(m: bool | str) -> str:
    if m is True:
        return "⚠ yes"
    if m is False:
        return "no"
    return "unknown"


def render_md(tools: list[dict], provenance: dict) -> str:
    out: list[str] = [
        "---",
        "title: Tool reference",
        "description: Every tool registered by the ROS2 MCP server, generated from the source registry.",
        "---",
        "",
        "<!-- GENERATED by scripts/gen_tool_docs.py. Do not edit by hand. -->",
        "",
        f"Generated from ROS2 MCP `{provenance.get('git_describe')}` "
        f"(package version `{provenance.get('package_version')}`, source `{provenance.get('source_sha')}`).",
        "",
        f"{len(tools)} tools are registered.",
        "",
        "| Tool | Mutates robot state | Summary |",
        "| --- | --- | --- |",
    ]
    for t in tools:
        out.append(f"| [`{t['name']}`](#{t['name']}) | {_mut_label(t['mutating'])} | {_summary(t['description'])} |")
    for t in tools:
        out += ["", f"## `{t['name']}`", ""]
        if t["mutating"] is True:
            out += [f"> **{MUTATING_BADGE}**", ""]
        elif t["mutating"] == "unknown":
            out += ["> Mutation status: unknown (safety classification not available in this build).", ""]
        desc = t["description"].strip()
        if desc:
            out += [line.rstrip() for line in desc.split("\n")] + [""]
        schema = t["input_schema"]
        props = schema.get("properties") or {}
        required = set(schema.get("required") or [])
        if props:
            out += ["| Parameter | Type | Required | Notes |", "| --- | --- | --- | --- |"]
            for pname in sorted(props):
                p = props[pname] if isinstance(props[pname], Mapping) else {}
                out.append(
                    f"| `{pname}` | {_type_str(p)} | {'yes' if pname in required else 'no'} | {_param_notes(p)} |"
                )
        else:
            out.append("No parameters.")
        out += ["", "<details><summary>Input schema (JSON)</summary>", "", "```json"]
        out += json.dumps(schema, indent=2, ensure_ascii=False).split("\n")
        out += ["```", "", "</details>"]
    # collapse any accidental trailing blank lines to exactly one newline
    return "\n".join(out).rstrip("\n") + "\n"


def render(tools: list[dict], provenance: dict) -> dict[str, str]:
    return {"tools.json": render_json(tools, provenance), "tools.md": render_md(tools, provenance)}


# --------------------------------------------------------------------------
# Drift gate
# --------------------------------------------------------------------------


def _strip_provenance(name: str, text: str) -> str:
    if name == "tools.json":
        try:
            doc = json.loads(text)
        except json.JSONDecodeError:
            return text
        if isinstance(doc, dict):
            doc.pop("provenance", None)
        return json.dumps(doc, indent=2, ensure_ascii=False) + "\n"
    if name == "tools.md":
        return "\n".join(l for l in text.split("\n") if not l.startswith("Generated from ROS2 MCP `"))
    return text


def check(out_dir: pathlib.Path, rendered: dict[str, str], strict_provenance: bool) -> int:
    drift = False
    with tempfile.TemporaryDirectory() as td:
        for name, fresh in rendered.items():
            (pathlib.Path(td) / name).write_text(fresh, encoding="utf-8")
            path = out_dir / name
            committed = path.read_text(encoding="utf-8") if path.exists() else ""
            a, b = committed, fresh
            if not strict_provenance:
                a, b = _strip_provenance(name, a), _strip_provenance(name, b)
            if a != b:
                drift = True
                sys.stdout.writelines(
                    difflib.unified_diff(
                        a.splitlines(keepends=True),
                        b.splitlines(keepends=True),
                        fromfile=f"a/{name} (committed)",
                        tofile=f"b/{name} (regenerated)",
                    )
                )
                sys.stdout.write("\n")
    if drift:
        print(
            "DRIFT: committed tool docs differ from the registry. Regenerate with "
            "`python3 scripts/gen_tool_docs.py` (see docs/generated/README.md) and commit."
        )
        return 1
    print("OK: tool docs match the registry" + (" (strict provenance)." if strict_provenance else "."))
    return 0


def main(argv=None, *, registry=None, provenance=None, is_mutating="auto") -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n", 1)[0])
    ap.add_argument("--out-dir", default=str(DEFAULT_OUT))
    ap.add_argument("--check", action="store_true", help="exit 1 with a diff if committed docs drift")
    ap.add_argument("--strict-provenance", action="store_true", help="also compare the provenance stamp")
    args = ap.parse_args(argv)

    if registry is None:
        registry = load_registry()
    if is_mutating == "auto":
        is_mutating = load_is_mutating()
    if provenance is None:
        provenance = compute_provenance(REPO_ROOT)

    rendered = render(collect_tools(registry, is_mutating), provenance)
    out_dir = pathlib.Path(args.out_dir)
    if args.check:
        return check(out_dir, rendered, args.strict_provenance)
    out_dir.mkdir(parents=True, exist_ok=True)
    for name, text in rendered.items():
        with open(out_dir / name, "w", encoding="utf-8", newline="\n") as f:
            f.write(text)
    print(f"Wrote {', '.join(str(out_dir / n) for n in FILES)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
