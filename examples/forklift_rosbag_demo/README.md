# Forklift rosbag demo — ROS 2 MCP in 10 minutes

Replay a real forklift sensor recording and let an MCP client discover and read
its topics through `ros2_mcp`. No robot, no simulator: one bag, one server,
three tool calls.

Verified on Ubuntu 24.04 + ROS 2 Jazzy with the bag described below. Other
distros (Humble or later) should work; the tool names and output shapes are the
same.

## The bag

`mockup_warehouse_fork_lift_data_rosbag` (MCAP, 26.7 s, 2128 messages,
recorded on ROS 2 Jazzy) — a front-facing radar + camera pair from a warehouse
forklift. `ros2 bag info` reports:

| Topic | Type | Count |
|---|---|---|
| `/pulse/front/camera/image_compressed` | `sensor_msgs/msg/CompressedImage` | 797 |
| `/pulse/front/camera/image_raw` | `sensor_msgs/msg/Image` | 797 |
| `/pulse/front/radar/detections` | `std_msgs/msg/String` (JSON payload) | 267 |
| `/pulse/front/radar/points` | `sensor_msgs/msg/PointCloud2` | 267 |

The bag is ~5 GiB and is not in this repo. Ask in
[Discussions → Q&A](https://github.com/wise-vision/ros2_mcp/discussions/categories/q-a)
if you want a copy, or point the steps below at **any** ROS 2 bag you already
have — only the `--topic` argument in step 4 is bag-specific.

## 1. Install (≈3 min)

```bash
source /opt/ros/jazzy/setup.bash                      # or your distro
git clone https://github.com/wise-vision/ros2_mcp.git && cd ros2_mcp
uv venv --python /usr/bin/python3 --system-site-packages .venv   # rclpy comes from ROS, not PyPI
uv sync --python .venv/bin/python
.venv/bin/python -c "import rclpy, mcp; print('ok')"
```

`--system-site-packages` matters: `rclpy` is installed by ROS into the system
Python and is not on PyPI. If `uv` picks a different interpreter than the one
ROS was built for, the import above fails with `No module named
'rclpy._rclpy_pybind11'`.

## 2. Replay the bag

In a second terminal:

```bash
source /opt/ros/jazzy/setup.bash
ros2 bag play /path/to/forklift_real_rosbag --loop
```

Wait until `ros2 topic list` shows the four `/pulse/...` topics — opening a
5 GiB MCAP takes several seconds, and until then the server sees no bag topics.

## 3. Start the server

You don't start it by hand — an MCP client launches it over stdio. The demo
script does exactly that (`python -m server.main`, with the same interpreter
that runs the script, so it inherits `rclpy`). To run it standalone for
inspection:

```bash
npx @modelcontextprotocol/inspector uv --directory "$PWD" run mcp_ros_2_server
```

## 4. Three MCP calls

```bash
.venv/bin/python examples/forklift_rosbag_demo/demo.py
# optional: --topic /pulse/front/radar/points --messages 1 --duration 5
#           --server-cmd "uv --directory $PWD run mcp_ros_2_server"   (any stdio launcher)
```

The script is an ordinary MCP stdio client; each block below is one
`tools/call`. Output shown is from a real run against the bag above (long
values trimmed with `…`).

### Call 1 — `ros2_topic_list`

Arguments: `{}`

```json
[
  {
    "topic_name": "/pulse/front/radar/detections",
    "topic_type": "std_msgs/msg/String",
    "request_fields": { "data": "string" }
  },
  {
    "topic_name": "/pulse/front/radar/points",
    "topic_type": "sensor_msgs/msg/PointCloud2",
    "request_fields": {
      "header": "std_msgs/Header", "height": "uint32", "width": "uint32",
      "fields": "sequence<sensor_msgs/PointField>", "is_bigendian": "boolean",
      "point_step": "uint32", "row_step": "uint32",
      "data": "sequence<uint8>", "is_dense": "boolean"
    }
  },
  …  /pulse/front/camera/image_raw, /pulse/front/camera/image_compressed,
  …  plus /rosout, /parameter_events, /events/read_split (from bag play itself)
]
```

Every topic comes back with its type **and** the field schema, so an agent can
decide what to read without a second round-trip.

### Call 2 — `ros2_get_message_fields`

Arguments: `{"message_type": "std_msgs/msg/String"}`

```json
{
  "data": "string"
}
```

### Call 3 — `ros2_topic_subscribe`

Arguments: `{"topic_name": "/pulse/front/radar/detections", "message_limit": 3, "duration": 10.0}`

```
[/pulse/front/radar/detections] 3 messages received.
{
  "/pulse/front/radar/detections#0": {
    "_data": "{\"frame\":\"t0\",\"data\":{\"Vun\":[[0.0]],\"det_info\":{\"SNR\":[[41.50,41.50,40.46,32.49, …",
    "_check_fields": false
  }
}
{
  "/pulse/front/radar/detections#1": { "_data": "{\"frame\":\"t1\", …", "_check_fields": false }
}
{
  "/pulse/front/radar/detections#2": { "_data": "{\"frame\":\"t2\", …", "_check_fields": false }
}
```

The radar publishes its detections as a JSON string inside `std_msgs/String`;
the server hands the raw payload to the client, which can parse it from there.
Subscribing to `/pulse/front/camera/image_compressed` instead returns
`ImageContent` blocks (the client sees the frame, not base64 text).

## What this shows

- **Discovery**: an agent finds topics and their schemas from a running graph
  with no prior knowledge of the robot.
- **Read path**: bounded subscriptions (`message_limit` / `duration`) so a call
  always returns.
- **Zero infrastructure**: stdio transport; the bag replay is the only ROS
  process besides the server.

No services or actions are recorded in this bag, so `ros2_service_call` /
`ros2_send_action_goal` are not exercised here. Attach the server to a live
stack (or a bag with a service-hosting node running alongside) to try those —
same client, same call shape.

## Troubleshooting

- `ERROR: Topic '/pulse/front/radar/detections' not found` or `0 messages
  received` — the bag isn't playing yet (or the topic name is wrong). Wait for
  `ros2 topic list` to show it and rerun.
- `No module named 'rclpy._rclpy_pybind11'` — the venv's Python is not the
  one ROS was built for; recreate it with `--python /usr/bin/python3`.
- Topic list is empty or short right after start — DDS discovery is still
  warming up; retry after a second or two (see the main README's discovery
  note).
- `ROS 2 Viewer UI index.html not found` on stderr — harmless; the optional
  web viewer isn't built in this checkout.
