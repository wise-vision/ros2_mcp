# Agent instructions

This repo is a **Python MCP server for ROS 2** (Humble/Jazzy). The packaged entrypoint is `mcp_ros_2_server` (see `pyproject.toml`).

## Project layout
- `server/` — MCP server (`server.py`), tool handlers (`tools_ros2.py`), ROS 2 logic (`ros2_manager.py`), prompts (`prompts_ros2.py`), transports (`transport.py`).
- `tests/` — `pytest` tests (often mocking ROS 2 nodes).
- `Dockerfile` — container build for running the server with ROS 2.
- `server.json` — MCP server manifest (runtime + env var knobs).

## Dev setup (preferred)
Use `uv` (deps are pinned in `uv.lock`):
- `uv venv`
- `uv sync --dev`

Avoid editing `uv.lock` manually.

## Running locally
The server requires a sourced ROS 2 environment before start (so `rclpy` works).
- Stdio (default): `uv run mcp_ros_2_server`
- Streamable HTTP (current MCP spec): `uv run mcp_ros_2_server --transport streamable-http` (serves `http://127.0.0.1:8123/mcp`; `--host/--port`, env `ROS2_MCP_TRANSPORT/HOST/PORT`)
- SSE (deprecated by the MCP spec): `uv run mcp_ros_2_server --transport sse` (binds `127.0.0.1:8123`)

Custom prompts can be toggled via `pyproject.toml` `[tool.mcp_ros_2]`, env vars (`MCP_CUSTOM_PROMPTS`, `MCP_PROMPTS_LOCAL`, `MCP_PROMPTS_PATH`, `MCP_PROMPTS_MODULE`) or CLI flags (`--custom-prompts`, `--prompts-local`, `--prompts-path`, `--prompts-module`) — see `server/server.py`.

## Tests
Tests require ROS 2 Python libs (`rclpy`) and some message/interface packages (see CI in `.github/workflows/server_tests.yml`).
Typical run inside a ROS 2 environment:
- `source /opt/ros/<distro>/setup.sh`
- `export PYTHONPATH=.:/opt/ros/<distro>/lib/python3.*/site-packages`
- `uv run pytest`

## Adding a tool
- Implement a `ToolHandler` (usually in `server/tools_ros2.py`) and return `TextContent`/`ImageContent` outputs.
- Register it in `server/server.py` via `add_tool_handler(...)`.
- Add/adjust `pytest` coverage under `tests/`.

## Adding a prompt
- Implement a `BasePromptHandler` (template-based) in `server/prompts_ros2.py`.
- Register it in `server/server.py` via `add_prompt_handler(...)`.
- For custom prompt authoring, also see `docs/CREATE_PROMPT.md`.

## Docker
Build:
- `docker build -t ros2_mcp:jazzy --build-arg ROS_DISTRO=jazzy .`

Run (stdio):
- `docker run -i --rm ros2_mcp:jazzy`

## Repo hygiene
- Keep diffs minimal; avoid broad reformatting.
- Don’t commit generated artifacts (`__pycache__/`, `.venv/`, `.pytest_cache/`).
- New source files should follow the license header conventions in `.licenserc.json`.
