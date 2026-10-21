# Changelog

All notable changes to ROS2 MCP are recorded here. Releases are tagged `YYMM`.

## Unreleased

### Added
- **ROS 2 Lyrical Luth** (LTS, Ubuntu 26.04, Python 3.14) is supported: CI tests, Docker images.
- **ROS 2 Rolling** is tested and published as `:rolling`, best-effort: its CI never blocks a release.
- **Release-based Docker tags** (scheme in the README, logic in `scripts/plan_images.py`):
  - `:<distro>` is the latest release, rebuilt weekly for base-image security updates;
  - `:<distro>-<YYMM>` is immutable (the workflow refuses to overwrite it);
  - `:<distro>-main` is the unreleased main branch;
  - `:latest` is the newest LTS.
  Every tag is multi-arch with `-amd64`/`-arm64` variants. `workflow_dispatch` with `release` backfills a release.
- **Same-month releases**: `YYMM.N` (e.g. `2610.1`) is a release tag too, ordered numerically. The Docker workflow
  refuses to publish a release whose `server.json` doesn't carry that version.
- **End-to-end smoke test** (`scripts/smoke_e2e.sh`): a real publisher plus the server over stdio, for Fast DDS,
  Cyclone DDS and Zenoh. Runs on every PR and gates every image push.
- `.github/distros.json` is the one list of shipped distros. CI matrices read it, and a test keeps the README
  support grid in sync with it.

### Changed
- `:<distro>` now follows the latest release instead of `main` (use `:<distro>-main` for unreleased code).
  `server.json` points at the immutable `:jazzy-<version>` image.
- Images build from the official `ros:<distro>-ros-base` and compile `wisevision_msgs` themselves (no prebuilt base
  image), so every distro builds and picks up current base-image updates.

### Fixed
- Same-host robots: topics were discovered but no messages arrived (Fast DDS shared memory can't cross the
  container's IPC namespace). Images now set `FASTDDS_BUILTIN_TRANSPORTS=UDPv4`. `RMW_FASTRTPS_USE_SHM` had no effect
  and is removed.

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
