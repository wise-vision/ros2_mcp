# Changelog

All notable changes to ROS2 MCP are recorded here. Releases are tagged `YYMM`.

## 2611 (unreleased, proposed)

### Added
- **Streamable HTTP transport** (closes #8): `--transport streamable-http` serves the MCP endpoint at `http://127.0.0.1:8123/mcp`, the transport the current MCP specification defines. New options: `--host`, `--port`, `--allowed-host`, `--stateless`, `--json-response`; env vars `ROS2_MCP_TRANSPORT`, `ROS2_MCP_HOST`, `ROS2_MCP_PORT`, `ROS2_MCP_ALLOWED_HOSTS` (the Docker image `CMD` takes no arguments, so env vars configure it). A loopback bind validates the `Host` and `Origin` headers (DNS-rebinding protection); a non-loopback bind prints a no-authentication warning. There is still no authentication.
- README: 30-second quickstart, complete tool list, supported distros, comparison with other ROS MCP servers, a section for agents, and a "Commercial support & integration" section.

### Changed
- `mcp[cli]` is capped at `<2`. Installs that ignore `uv.lock` (uvx, pip, rosdep) resolved mcp 2.0.0 and the server crashed at import (`Server.list_tools`). Porting to the mcp 2.x server API is not done.
- Security floors raised to patched versions: `pillow>=12.3.0`, `python-multipart>=0.0.31`, `anyio>=4.14.2`, `pyjwt>=2.15.0` (locked: anyio 4.15.1, PyJWT 2.15.1).
- `server.json` now validates against the MCP registry schema `2025-12-11` (description length, version, OCI identifier `docker.io/wisevision/ros2_mcp`).

### Deprecated
- The `sse` transport is deprecated by the MCP specification. It is unchanged and still works; prefer `streamable-http`.

## 2610 (2026-09-29)

### Added
- **Former Pro tools are now free and open source under MPL-2.0**, in the main package, with no licence key:
  - `ros2_subscribe_multiple_topics`: subscribe to several topics at once; `Image`/`CompressedImage` returned as PNG.
  - `ros2_publish_multiple_topics`: publish to several topics at a set frequency and duration.
  - `ros2_get_map_as_image`: `nav_msgs/OccupancyGrid` as a PNG.
  - `ros2_get_pointcloud_as_bev`: `sensor_msgs/PointCloud2` as a bird's-eye-view PNG.
- **Read-only mode**: `ROS2_MCP_READONLY=1` or `--read-only`. Tools that publish, call services or send/cancel
  action goals (`ros2_topic_publish`, `ros2_publish_multiple_topics`, `ros2_service_call`, `ros2_send_action_goal`,
  `ros2_cancel_action_goal`) are not registered: they are absent from `list_tools` and calling one is an unknown-tool error.
- `server/tool_safety.py`: every tool is classified read-only or mutating; a test fails on an unclassified tool.
- CI runs the test suite on both ROS 2 Humble and Jazzy.

### Changed
- Product renamed to **ROS2 MCP** (README, package description, MCP `serverInfo` name, Docker labels, `server.json` title).
  The repository slug `ros2_mcp`, the `mcp_ros_2_server` entrypoint and the `mcp/ros2` Docker image are unchanged.
- Documentation now lives at https://wisevision.tech/docs.

### Removed
- The optional private `extensions` import hook (the tools it loaded are now built in).
