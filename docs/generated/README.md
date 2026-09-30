# Generated tool reference

`tools.json` and `tools.md` are **generated** by `scripts/gen_tool_docs.py` from the tool
registry in `server/server.py` (`tool_handlers`, filled by `add_tool_handler(...)` at import).
Do not edit them by hand. The `docs-gen` CI job (`.github/workflows/docs_gen.yml`) fails when
they are out of date.

## Regenerate locally

Importing the server needs `rclpy`, so run the generator inside ROS 2 Jazzy. From the repo root:

```bash
docker run --rm -v "$PWD":/ws -w /ws \
  -v ros2mcp-uvcache:/root/.cache/uv \
  -e UV_PROJECT_ENVIRONMENT=/opt/venv -e UV_LINK_MODE=copy \
  ros:jazzy-ros-base bash -lc '
    set -e
    apt-get update -qq && apt-get install -y -qq python3-pip git \
      ros-jazzy-example-interfaces ros-jazzy-mavros-msgs ros-jazzy-action-tutorials-interfaces
    pip install -q --break-system-packages uv
    uv sync --dev -q --python /usr/bin/python3
    . /opt/ros/jazzy/setup.sh
    export PYTHONPATH=.:$PYTHONPATH  # keep the ROS site-packages that setup.sh added
    uv run --python /usr/bin/python3 python scripts/gen_tool_docs.py
    chown -R '"$(id -u):$(id -g)"' docs/generated'
```

The named volume `ros2mcp-uvcache` caches the Python downloads between runs.
Git worktrees: also mount the main repo's `.git` directory at the same path
(`-v /path/to/repo/.git:/path/to/repo/.git:ro`). Otherwise git is unreachable and the
provenance becomes `unknown`.

Other modes:

- `python3 scripts/gen_tool_docs.py --check` compares against the committed files and exits 1 with a unified diff on drift. The provenance stamp is ignored.
- `--check --strict-provenance` also compares the provenance stamp. CI uses it on release tags.
- `pytest scripts/tests` runs the generator tests: the 5 drift RED proofs, determinism, and registry equality.

## Provenance

`tools.json` carries a `provenance` object with these fields:

- `release`: the ROS2 MCP release (`YYMM`) the docs describe. It is the release tag at HEAD
  when there is one, else the `pyproject.toml` version when that is a `YYMM` release (release
  prep bumps it before tagging, so tagging does not change the stamp), else the nearest release
  tag, else `unreleased`. `tools.md` shows this with the short source commit.
- `package_version`: the version from `pyproject.toml` (also reported as MCP `serverInfo.version`).
- `source_sha`: the last commit that touched `server/` or `pyproject.toml`.
- `git_describe`: `git describe --tags` of that commit against the release tags before it.

Merge PRs that change `server/` or `pyproject.toml` with a merge commit, or regenerate after a
squash merge: squashing rewrites `source_sha`, and `--strict-provenance` on the next tag fails.

On tag pushes, CI also uploads `release-stamp.json` (`release_tag`, `release_sha` plus the fields
above) in the `tool-docs` artifact for the docs site.

## Mutating flag

`mutating` comes from `server.tool_safety.is_mutating(name)` when that module exists. Until then it
is `"unknown"`. Regenerate after `server/tool_safety.py` lands.
