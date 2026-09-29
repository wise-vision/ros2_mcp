# Changelog

All notable changes to ROS2 MCP are recorded here. Releases are tagged `YYMM`.

## 2610 (unreleased)

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
