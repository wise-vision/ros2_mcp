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

import rclpy
from rclpy.node import Node
import importlib
import os
import threading
from typing import Any
import secrets
import struct
from rclpy.serialization import deserialize_message
from rosidl_runtime_py import get_interfaces
from rosidl_runtime_py.utilities import get_service, get_message
from rosidl_runtime_py.set_message import set_message_fields
from rosidl_runtime_py import message_to_ordereddict

from rclpy.action import get_action_names_and_types
from dateutil import parser
import numpy as np
import array
from PIL import Image as PILImage
import base64
import io
import time
from rclpy.task import Future
from builtin_interfaces.msg import Time, Duration
from std_msgs.msg import Header
from sensor_msgs.msg import Image, CompressedImage
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSPresetProfiles
from rclpy.qos import QoSReliabilityPolicy
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from .timed_publish import (
    TimedPublishRegistry,
    run_registered_timed_publish,
    validate_timed_publish_args,
)

QOS_DEPTH = 1_000
SUBCRIPTION_DURATION_TIME = 5.0
GOAL_STATUS = {
    -1: "NOT_ACCEPTED",
    0: "UNKNOWN",
    1: "ACCEPTED",
    2: "EXECUTING",
    3: "CANCELING",
    4: "SUCCEEDED",
    5: "CANCELED",
    6: "ABORTED",
}
GOAL_CANCEL_RET = {
    0: "ERROR_NONE",
    1: "ERROR_REJECTED",
    2: "ERROR_UNKNOWN_GOAL_ID",
    3: "ERROR_GOAL_TERMINATED",
}

_TRUE_STRINGS = {"1", "true", "yes", "on"}
_FALSE_STRINGS = {"0", "false", "no", "off"}


def _parse_env_bool(name: str) -> bool | None:
    val = os.getenv(name)
    if val is None:
        return None
    norm = val.strip().lower()
    if norm in _TRUE_STRINGS:
        return True
    if norm in _FALSE_STRINGS:
        return False
    return None


def _parse_env_float(name: str, default: float) -> float:
    val = os.getenv(name)
    if val is None:
        return default
    try:
        return float(val)
    except ValueError:
        return default


def _is_container_environment() -> bool:
    try:
        if os.path.exists("/.dockerenv") or os.path.exists("/run/.containerenv"):
            return True

        # Fallback for container runtimes that don't expose the marker files.
        with open("/proc/1/cgroup", "rt", encoding="utf-8", errors="ignore") as f:
            cgroup = f.read()
        return any(token in cgroup for token in ("docker", "containerd", "kubepods", "podman", "libpod"))
    except Exception:
        return False


class ServiceNode(Node):
    def __init__(self):
        super().__init__("mcp_service_lister")


class ROS2Manager:
    def __init__(self):
        self.node = ServiceNode()
        self._discovery_warmed_up = False
        self._discovery_warmup_lock = threading.Lock()
        self._streams_lock = threading.Lock()
        self._streams: dict[str, "_MCPStreamSession"] = {}
        # Running timed publish tasks per topic. Registry methods are only
        # called from the server's event loop (the publish tools dispatch
        # through ToolHandler.run_tool_async); see server/timed_publish.py.
        self._timed_publish_registry = TimedPublishRegistry()
        # Tool handlers run concurrently (sync ones in asyncio.to_thread
        # workers). rclpy's global executor rejects overlapping spins
        # ("Executor is already spinning"), so every spin of the shared node
        # is serialized through this one lock. The publish/stop path never
        # spins the node, so a stop never waits on it.
        self._node_spin_lock = threading.Lock()

    def _spin_shared_node_until_future_complete(self, future, timeout_sec=None, node=None):
        """Spin the (shared) node on rclpy's global executor, serialized by
        ``_node_spin_lock`` so concurrent handlers cannot re-enter the
        executor. See ROS2Manager.__init__."""
        with self._node_spin_lock:
            node = self.node if node is None else node
            if timeout_sec is None:
                return rclpy.spin_until_future_complete(node, future)
            return rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)

    def _spin_shared_node_once(self, timeout_sec: float):
        """Spin the shared node once on rclpy's global executor, serialized by
        ``_node_spin_lock``. See ROS2Manager.__init__."""
        with self._node_spin_lock:
            return rclpy.spin_once(self.node, timeout_sec=timeout_sec)

    def _get_topic_type(self, topic_name: str) -> str | None:
        try:
            for name, types in self.node.get_topic_names_and_types():
                if name == topic_name and types:
                    return types[0]
        except Exception:
            return None
        return None

    def _new_session_id(self) -> str:
        # Short, URL-safe session id for use by UI clients.
        return secrets.token_urlsafe(10)

    def _load_msg_class(self, msg_type: str):
        parts = msg_type.split("/")
        if len(parts) == 3:
            pkg, _, msg = parts
        elif len(parts) == 2:
            pkg, msg = parts
        else:
            raise ValueError(f"Invalid message type format: {msg_type}")
        module = importlib.import_module(f"{pkg}.msg")
        return getattr(module, msg)

    def _encode_image_msg_to_jpeg_b64(
        self,
        msg: Any,
        msg_type: str,
        *,
        max_width: int | None,
        max_height: int | None,
        jpeg_quality: int,
    ) -> dict:
        if msg_type == "sensor_msgs/msg/CompressedImage":
            fmt = getattr(msg, "format", "") or ""
            data = getattr(msg, "data", b"") or b""
            if isinstance(data, (bytes, bytearray, memoryview)) and ("jpeg" in fmt.lower() or "jpg" in fmt.lower()):
                return {
                    "mimeType": "image/jpeg",
                    "data": base64.b64encode(bytes(data)).decode("utf-8"),
                }

            try:
                pil_img = PILImage.open(io.BytesIO(bytes(data)))
                if pil_img.mode not in ("RGB", "RGBA", "L"):
                    pil_img = pil_img.convert("RGB")
            except Exception as e:
                return {"error": f"Failed to decode CompressedImage: {e}"}
        elif msg_type == "sensor_msgs/msg/Image":
            enc = getattr(msg, "encoding", "") or ""
            width = int(getattr(msg, "width", 0) or 0)
            height = int(getattr(msg, "height", 0) or 0)
            raw = getattr(msg, "data", b"") or b""

            try:
                if enc in ("rgb8", "bgr8", "rgba8"):
                    channels = 4 if enc == "rgba8" else 3
                    img_array = np.frombuffer(raw, dtype=np.uint8).reshape((height, width, channels))
                    if enc == "bgr8":
                        img_array = img_array[..., ::-1]
                    mode = "RGBA" if channels == 4 else "RGB"
                    pil_img = PILImage.fromarray(img_array, mode=mode)
                elif enc == "mono8":
                    img_array = np.frombuffer(raw, dtype=np.uint8).reshape((height, width))
                    pil_img = PILImage.fromarray(img_array, mode="L")
                else:
                    return {"error": f"Unsupported Image encoding: {enc}"}
            except Exception as e:
                return {"error": f"Failed to process Image: {e}"}
        else:
            return {"error": f"Unsupported message type for image stream: {msg_type}"}

        if max_width and max_height:
            try:
                w, h = pil_img.size
                scale = min(max_width / max(w, 1), max_height / max(h, 1), 1.0)
                if scale < 1.0:
                    pil_img = pil_img.resize((int(w * scale), int(h * scale)), PILImage.BILINEAR)
            except Exception:
                pass

        if pil_img.mode == "RGBA":
            pil_img = pil_img.convert("RGB")

        buffer = io.BytesIO()
        try:
            q = int(jpeg_quality)
            q = max(10, min(95, q))
            pil_img.save(buffer, format="JPEG", quality=q)
        except Exception as e:
            return {"error": f"Failed to encode JPEG: {e}"}

        return {
            "mimeType": "image/jpeg",
            "data": base64.b64encode(buffer.getvalue()).decode("utf-8"),
        }

    def _extract_pointcloud2_xyz_b64(self, msg: Any, *, max_points: int) -> dict:
        try:
            fields = {f.name: f for f in getattr(msg, "fields", [])}
            if not all(k in fields for k in ("x", "y", "z")):
                return {"error": "PointCloud2 is missing x/y/z fields."}

            data = getattr(msg, "data", b"") or b""
            data_bytes = bytes(data)

            endian = ">" if bool(getattr(msg, "is_bigendian", False)) else "<"
            off_x = int(fields["x"].offset)
            off_y = int(fields["y"].offset)
            off_z = int(fields["z"].offset)

            # Check for color fields (rgb packed float, rgba, or separate r/g/b)
            has_rgb_packed = "rgb" in fields
            has_rgba_packed = "rgba" in fields
            has_rgb_separate = all(c in fields for c in ("r", "g", "b"))
            has_intensity = "intensity" in fields

            off_rgb = None
            off_rgba = None
            off_r = off_g = off_b = None
            off_intensity = None

            if has_rgb_packed:
                off_rgb = int(fields["rgb"].offset)
            elif has_rgba_packed:
                off_rgba = int(fields["rgba"].offset)
            elif has_rgb_separate:
                off_r = int(fields["r"].offset)
                off_g = int(fields["g"].offset)
                off_b = int(fields["b"].offset)
            elif has_intensity:
                off_intensity = int(fields["intensity"].offset)

            step = int(getattr(msg, "point_step", 0) or 0)
            width = int(getattr(msg, "width", 0) or 0)
            height = int(getattr(msg, "height", 0) or 0)
            n = width * height
            if step <= 0 or n <= 0:
                return {"error": "PointCloud2 has invalid dimensions/point_step."}

            if max_points <= 0:
                stride = 1
            else:
                stride = max(1, n // max_points)
            out_pos = array.array("f")
            out_col = array.array("f")
            intensities = []
            has_colors = has_rgb_packed or has_rgba_packed or has_rgb_separate
            unpack = struct.unpack_from

            # First pass: collect positions and intensities
            for i in range(0, n, stride):
                base = i * step
                try:
                    x = unpack(endian + "f", data_bytes, base + off_x)[0]
                    y = unpack(endian + "f", data_bytes, base + off_y)[0]
                    z = unpack(endian + "f", data_bytes, base + off_z)[0]
                except Exception:
                    continue
                if not (np.isfinite(x) and np.isfinite(y) and np.isfinite(z)):
                    continue
                out_pos.extend((float(x), float(y), float(z)))

                # Extract color or intensity
                if has_colors:
                    try:
                        if off_rgb is not None:
                            # RGB packed as float (reinterpret as uint32)
                            rgb_float = unpack(endian + "f", data_bytes, base + off_rgb)[0]
                            rgb_int = struct.unpack("I", struct.pack("f", rgb_float))[0]
                            r = ((rgb_int >> 16) & 0xFF) / 255.0
                            g = ((rgb_int >> 8) & 0xFF) / 255.0
                            b = (rgb_int & 0xFF) / 255.0
                        elif off_rgba is not None:
                            # RGBA packed as float (reinterpret as uint32)
                            rgba_float = unpack(endian + "f", data_bytes, base + off_rgba)[0]
                            rgba_int = struct.unpack("I", struct.pack("f", rgba_float))[0]
                            r = ((rgba_int >> 16) & 0xFF) / 255.0
                            g = ((rgba_int >> 8) & 0xFF) / 255.0
                            b = (rgba_int & 0xFF) / 255.0
                        elif off_r is not None:
                            # Separate r/g/b fields (usually uint8)
                            r = unpack("B", data_bytes, base + off_r)[0] / 255.0
                            g = unpack("B", data_bytes, base + off_g)[0] / 255.0
                            b = unpack("B", data_bytes, base + off_b)[0] / 255.0
                        else:
                            r = g = b = 0.7
                        out_col.extend((float(r), float(g), float(b)))
                    except Exception:
                        out_col.extend((0.7, 0.7, 0.7))
                elif off_intensity is not None:
                    try:
                        intensity = unpack(endian + "f", data_bytes, base + off_intensity)[0]
                        if np.isfinite(intensity):
                            intensities.append(intensity)
                        else:
                            intensities.append(0.0)
                    except Exception:
                        intensities.append(0.0)

            # Convert intensities to colors using a rainbow colormap (like RViz)
            if intensities and not has_colors:
                min_i = min(intensities) if intensities else 0.0
                max_i = max(intensities) if intensities else 1.0
                range_i = max_i - min_i if max_i > min_i else 1.0

                for intensity in intensities:
                    # Normalize to 0-1
                    t = (intensity - min_i) / range_i
                    # Rainbow colormap: blue -> cyan -> green -> yellow -> red
                    if t < 0.25:
                        r, g, b = 0.0, t * 4, 1.0
                    elif t < 0.5:
                        r, g, b = 0.0, 1.0, 1.0 - (t - 0.25) * 4
                    elif t < 0.75:
                        r, g, b = (t - 0.5) * 4, 1.0, 0.0
                    else:
                        r, g, b = 1.0, 1.0 - (t - 0.75) * 4, 0.0
                    out_col.extend((float(r), float(g), float(b)))

            result = {
                "positionsF32B64": base64.b64encode(out_pos.tobytes()).decode("utf-8"),
                "pointCount": len(out_pos) // 3,
            }
            if len(out_col) > 0:
                result["colorsF32B64"] = base64.b64encode(out_col.tobytes()).decode("utf-8")

            return result
        except Exception as e:
            return {"error": f"Failed to decode PointCloud2: {e}"}

    def start_stream(
        self,
        *,
        topic_name: str,
        kind: str,
        target_fps: float | None = None,
        max_width: int | None = None,
        max_height: int | None = None,
        jpeg_quality: int = 80,
        max_points: int | None = None,
        qos_preset: str = "auto",
    ) -> dict:
        topic_type = self._get_topic_type(topic_name)
        if not topic_type:
            return {"error": f"Unknown topic: {topic_name}"}

        if kind == "image":
            if topic_type not in ("sensor_msgs/msg/Image", "sensor_msgs/msg/CompressedImage"):
                return {"error": f"Topic '{topic_name}' is '{topic_type}', not an Image/CompressedImage."}
        elif kind == "pointcloud":
            if topic_type != "sensor_msgs/msg/PointCloud2":
                return {"error": f"Topic '{topic_name}' is '{topic_type}', not a PointCloud2."}
        elif kind == "message":
            # Allow any topic type for generic message streaming.
            pass
        else:
            return {"error": f"Unknown stream kind: {kind}"}

        try:
            msg_class = self._load_msg_class(topic_type)
        except Exception as e:
            return {"error": f"Failed to import message class for {topic_type}: {e}"}

        session_id = self._new_session_id()
        suffix = "".join(ch for ch in session_id if ch.isalnum())[:8] or secrets.token_hex(4)
        node_name = f"mcp_stream_{suffix}"
        try:
            stream_node = Node(node_name)
        except Exception as e:
            return {"error": f"Failed to create stream node: {e}"}
        # Some camera publishers (sensor_msgs/Image/CompressedImage) use sensor_data QoS
        # (best-effort, volatile). In early discovery windows, publisher introspection can
        # return empty and default QoS may fail to match. Provide an "auto" fallback.
        qos_choice = (qos_preset or "auto").strip().lower()
        if qos_choice == "sensor_data":
            qos = QoSPresetProfiles.SENSOR_DATA.value
        elif qos_choice == "system_default":
            qos = QoSPresetProfiles.SYSTEM_DEFAULT.value
        else:
            qos = self.get_qos_profile_for_topic(stream_node, topic_name)
            try:
                pubs = stream_node.get_publishers_info_by_topic(topic_name)
            except Exception:
                pubs = []
            # If discovery hasn't surfaced publishers yet, prefer a RELIABLE default first.
            # This matches many camera pipelines publishing RELIABLE CompressedImage.
            if not pubs and kind in ("image", "pointcloud"):
                qos = QoSPresetProfiles.SYSTEM_DEFAULT.value

        max_points_val = 0 if max_points is None else int(max_points)
        if max_points_val < 0:
            max_points_val = 0

        session = _MCPStreamSession(
            session_id=session_id,
            kind=kind,
            topic_name=topic_name,
            topic_type=topic_type,
            node=stream_node,
            target_fps=float(target_fps) if target_fps is not None else None,
            max_width=max_width,
            max_height=max_height,
            jpeg_quality=int(jpeg_quality),
            max_points=max_points_val,
        )

        def _cb(msg):
            now = time.monotonic()
            session.rx_count += 1
            session.last_rx_monotonic = now
            if session.target_fps and session.target_fps > 0:
                min_dt = 1.0 / session.target_fps
                if (now - session.last_emit_monotonic) < min_dt:
                    return

            if session.kind == "image":
                payload = self._encode_image_msg_to_jpeg_b64(
                    msg,
                    session.topic_type,
                    max_width=session.max_width,
                    max_height=session.max_height,
                    jpeg_quality=session.jpeg_quality,
                )
            elif session.kind == "pointcloud":
                payload = self._extract_pointcloud2_xyz_b64(msg, max_points=session.max_points)
            else:
                payload = {"message": session.serialize_msg(msg)}

            if isinstance(payload, dict):
                err = payload.get("error")
                if not err and isinstance(payload.get("message"), dict):
                    err = payload["message"].get("error")
                if err:
                    session.last_error = str(err)

            header = getattr(msg, "header", None)
            if header is not None:
                stamp = getattr(header, "stamp", None)
                if stamp is not None:
                    payload["stamp"] = {"sec": int(getattr(stamp, "sec", 0)), "nanosec": int(getattr(stamp, "nanosec", 0))}
                payload["frame_id"] = str(getattr(header, "frame_id", ""))

            with session.lock:
                session.seq += 1
                payload["seq"] = session.seq
                session.last_payload = payload
                session.last_emit_monotonic = now

        session.subscription = stream_node.create_subscription(msg_class, topic_name, _cb, qos)
        session.executor = SingleThreadedExecutor(context=stream_node.context)
        session.executor.add_node(stream_node)

        def _spin():
            try:
                while rclpy.ok() and stream_node.context.ok() and not session.stop_event.is_set():
                    session.executor.spin_once(timeout_sec=0.05)
            finally:
                try:
                    session.executor.remove_node(stream_node)
                except Exception:
                    pass
                try:
                    session.executor.shutdown()
                except Exception:
                    pass
                try:
                    stream_node.destroy_node()
                except Exception:
                    pass

        session.thread = threading.Thread(target=_spin, name=f"mcp_stream_{session_id}", daemon=True)
        session.thread.start()

        with self._streams_lock:
            self._streams[session_id] = session

        return {
            "session_id": session_id,
            "kind": kind,
            "topic_name": topic_name,
            "topic_type": topic_type,
        }

    def stream_next(self, *, session_id: str, after_seq: int | None = None) -> dict:
        with self._streams_lock:
            session = self._streams.get(session_id)
        if not session:
            return {"error": f"Unknown session_id: {session_id}"}

        with session.lock:
            payload = dict(session.last_payload) if session.last_payload else None
            cur_seq = session.seq
            rx_count = session.rx_count
            last_rx = session.last_rx_monotonic
            last_err = session.last_error

        if payload is None:
            out = {"available": False, "seq": cur_seq, "rx_count": rx_count}
            if last_rx:
                out["last_rx_age_sec"] = round(max(0.0, time.monotonic() - last_rx), 3)
            if last_err:
                out["last_error"] = last_err
            return out

        if after_seq is not None and cur_seq <= int(after_seq):
            out = {"available": False, "seq": cur_seq, "rx_count": rx_count}
            if last_rx:
                out["last_rx_age_sec"] = round(max(0.0, time.monotonic() - last_rx), 3)
            if last_err:
                out["last_error"] = last_err
            return out

        payload["available"] = True
        payload["rx_count"] = rx_count
        if last_rx:
            payload["last_rx_age_sec"] = round(max(0.0, time.monotonic() - last_rx), 3)
        if last_err and not payload.get("error"):
            payload["last_error"] = last_err
        return payload

    def stop_stream(self, *, session_id: str) -> dict:
        with self._streams_lock:
            session = self._streams.pop(session_id, None)
        if not session:
            return {"error": f"Unknown session_id: {session_id}"}

        session.stop_event.set()
        try:
            if session.thread:
                session.thread.join(timeout=2.0)
        except Exception:
            pass

        return {"stopped": True, "session_id": session_id}

    def _graph_snapshot(self) -> tuple[tuple[tuple[str, tuple[str, ...]], ...], tuple[tuple[str, tuple[str, ...]], ...]]:
        topics = tuple(
            sorted((name, tuple(types)) for name, types in self.node.get_topic_names_and_types())
        )
        services = tuple(
            sorted((name, tuple(types)) for name, types in self.node.get_service_names_and_types())
        )
        return topics, services

    def warm_up_discovery(self) -> dict:
        """
        Mitigate DDS discovery timing issues (common in containers) by waiting
        on the first tool invocation until the ROS graph stabilizes.

        Env vars:
        - MCP_ROS_DISCOVERY_WARMUP: true/false (default: auto; enabled in containers)
        - MCP_ROS_DISCOVERY_TIMEOUT_SEC: max wait time (default: 5.0)
        - MCP_ROS_DISCOVERY_STABLE_SEC: required "no changes" window (default: 1.0)
        - MCP_ROS_DISCOVERY_POLL_SEC: polling interval (default: 0.1)
        - MCP_ROS_DISCOVERY_DELAY_SEC: minimum wait before proceeding (default: 0.0)
        """
        if self._discovery_warmed_up:
            return {"status": "already_warmed"}

        with self._discovery_warmup_lock:
            if self._discovery_warmed_up:
                return {"status": "already_warmed"}

            enabled_env = _parse_env_bool("MCP_ROS_DISCOVERY_WARMUP")
            enabled = _is_container_environment() if enabled_env is None else enabled_env

            if not enabled:
                self._discovery_warmed_up = True
                return {"status": "disabled"}

            timeout_sec = max(_parse_env_float("MCP_ROS_DISCOVERY_TIMEOUT_SEC", 5.0), 0.0)
            stable_sec = max(_parse_env_float("MCP_ROS_DISCOVERY_STABLE_SEC", 1.0), 0.0)
            poll_sec = max(_parse_env_float("MCP_ROS_DISCOVERY_POLL_SEC", 0.1), 0.01)
            delay_sec = max(_parse_env_float("MCP_ROS_DISCOVERY_DELAY_SEC", 0.0), 0.0)

            max_wait_sec = max(timeout_sec, delay_sec)
            if max_wait_sec <= 0.0 and stable_sec <= 0.0:
                self._discovery_warmed_up = True
                return {"status": "skipped"}

            start = time.monotonic()
            last_snapshot = None
            last_change = start
            polls = 0

            while True:
                now = time.monotonic()
                elapsed = now - start
                if elapsed >= max_wait_sec:
                    break

                remaining = max_wait_sec - elapsed
                step = min(poll_sec, remaining)

                # Give rclpy a chance to process graph updates, but avoid busy looping.
                step_start = time.monotonic()
                try:
                    self._spin_shared_node_once(step)
                except Exception:
                    pass
                spun = time.monotonic() - step_start
                if spun < step:
                    time.sleep(step - spun)

                snapshot = self._graph_snapshot()
                polls += 1
                now = time.monotonic()
                elapsed = now - start

                if snapshot != last_snapshot:
                    last_snapshot = snapshot
                    last_change = now

                if elapsed >= delay_sec and (now - last_change) >= stable_sec:
                    break

            self._discovery_warmed_up = True
            elapsed_total = time.monotonic() - start
            topics_count = len(last_snapshot[0]) if last_snapshot else 0
            services_count = len(last_snapshot[1]) if last_snapshot else 0
            return {
                "status": "warmed",
                "elapsed_sec": elapsed_total,
                "polls": polls,
                "topics": topics_count,
                "services": services_count,
            }

    def get_qos_profile_for_topic(self, node, topic_name: str):

        qos_profile = QoSPresetProfiles.SYSTEM_DEFAULT.value
        reliability_reliable_endpoints_count = 0
        durability_transient_local_endpoints_count = 0

        pubs_info = node.get_publishers_info_by_topic(topic_name)
        publishers_count = len(pubs_info)
        if publishers_count == 0:
            return qos_profile

        for info in pubs_info:
            if info.qos_profile.reliability == QoSReliabilityPolicy.RELIABLE:
                reliability_reliable_endpoints_count += 1
            if info.qos_profile.durability == QoSDurabilityPolicy.TRANSIENT_LOCAL:
                durability_transient_local_endpoints_count += 1

        # If all endpoints are reliable, ask for reliable
        if reliability_reliable_endpoints_count == publishers_count:
            qos_profile.reliability = QoSReliabilityPolicy.RELIABLE
        else:
            if reliability_reliable_endpoints_count > 0:
                print(
                    "Some, but not all, publishers are offering "
                    "QoSReliabilityPolicy.RELIABLE. Falling back to "
                    "QoSReliabilityPolicy.BEST_EFFORT as it will connect "
                    "to all publishers"
                )
            qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT

        # If all endpoints are transient_local, ask for transient_local
        if durability_transient_local_endpoints_count == publishers_count:
            qos_profile.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        else:
            if durability_transient_local_endpoints_count > 0:
                print(
                    "Some, but not all, publishers are offering "
                    "QoSDurabilityPolicy.TRANSIENT_LOCAL. Falling back to "
                    "QoSDurabilityPolicy.VOLATILE as it will connect "
                    "to all publishers"
                )
            qos_profile.durability = QoSDurabilityPolicy.VOLATILE

        return qos_profile

    def get_qos_for_publisher_topic(self, node, topic_name: str) -> QoSProfile:

        base = QoSPresetProfiles.SYSTEM_DEFAULT.value
        qos = QoSProfile(
            depth=base.depth,
            reliability=base.reliability,
            durability=base.durability,
            history=base.history,
            deadline=base.deadline,
            lifespan=base.lifespan,
            liveliness=base.liveliness,
            liveliness_lease_duration=base.liveliness_lease_duration,
            avoid_ros_namespace_conventions=base.avoid_ros_namespace_conventions,
        )

        subs_info = node.get_subscriptions_info_by_topic(topic_name)
        if not subs_info:
            return qos  # no subscription -> use default profile

        any_reliable = False
        any_transient_local = False
        any_keep_all = False
        max_keep_last_depth = 0

        for info in subs_info:
            sp = info.qos_profile

            if sp.reliability == QoSReliabilityPolicy.RELIABLE:
                any_reliable = True

            if sp.durability == QoSDurabilityPolicy.TRANSIENT_LOCAL:
                any_transient_local = True

            if sp.history == QoSHistoryPolicy.KEEP_ALL:
                any_keep_all = True
            else:  # KEEP_LAST
                try:
                    max_keep_last_depth = max(max_keep_last_depth, int(sp.depth))
                except Exception:
                    # Ignore errors converting depth to int; fallback to default depth if conversion fails.
                    pass

        # Reliability
        qos.reliability = QoSReliabilityPolicy.RELIABLE if any_reliable else QoSReliabilityPolicy.BEST_EFFORT
        # Durability
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL if any_transient_local else QoSDurabilityPolicy.VOLATILE
        # History/depth
        if any_keep_all:
            qos.history = QoSHistoryPolicy.KEEP_ALL
        else:
            qos.history = QoSHistoryPolicy.KEEP_LAST
            if max_keep_last_depth > 0:
                qos.depth = max(max_keep_last_depth, qos.depth)

        return qos
    
    def list_topics(self):
        topic_names_and_types = self.node.get_topic_names_and_types()
        result = []
        for name, types in topic_names_and_types:
            topic_type = types[0] if types else "unknown"
            request_fields = self.get_request_fields(topic_type)
            result.append(
                {
                    "topic_name": name,
                    "topic_type": topic_type,
                    "request_fields": request_fields,
                }
            )
        return result

    def list_services(self):
        service_list = self.node.get_service_names_and_types()
        result = []
        for name, types in service_list:
            service_type = types[0] if types else "unknown"
            request_fields = self.get_request_fields(service_type)
            result.append(
                {
                    "service_name": name,
                    "service_type": service_type,
                    "request_fields": request_fields,
                }
            )
        return result

    def list_actions(self) -> dict:
        try:
            actions = []
            action_list = get_action_names_and_types(self.node)

            for name, type_list in action_list:
                for t in type_list:
                    request_fields = self.get_request_fields(t)
                    actions.append(
                        {
                            "name": name,
                            "types": [t],
                            "request_fields": request_fields,
                        }
                    )

            return {"actions": actions}

        except Exception as e:
            return {"error": str(e)}
    def list_interfaces(self):
        interfaces = get_interfaces()
        result = []
        for pkg_name, iface_list in interfaces.items():
            for iface in iface_list:
                # iface like "msg/String" or "srv/SetBool"
                result.append(f"{pkg_name}/{iface}")

        return result

    def get_request_fields(self, ros_type: str):
        try:
            parts = ros_type.split("/")
            if len(parts) == 3:
                pkg, kind, name = parts
            elif len(parts) == 2:
                pkg, name = parts
                kind = "msg"
            else:
                return {"error": f"Invalid type format: {ros_type}"}

            if kind == "msg":
                module = importlib.import_module(f"{pkg}.msg")
                msg_class = getattr(module, name)
                return msg_class.get_fields_and_field_types()

            elif kind == "srv":
                module = importlib.import_module(f"{pkg}.srv")
                srv_class = getattr(module, name)
                return srv_class.Request.get_fields_and_field_types()

            elif kind == "action":
                module = importlib.import_module(f"{pkg}.action")
                action_class = getattr(module, name)
                return action_class.Goal.get_fields_and_field_types()

            else:
                return {"error": f"Unsupported ROS type kind: {kind}"}

        except Exception as e:
            return {"error": f"Failed to load {ros_type}: {str(e)}"}

    def call_service(self, service_name: str, service_type: str, fields: dict) -> dict:
        try:
            parts = service_type.split("/")
            if len(parts) == 2:
                pkg, srv = parts
            elif len(parts) == 3:
                pkg, _, srv = parts
            else:
                return {"error": f"Invalid service type format: {service_type}"}
            module = importlib.import_module(f"{pkg}.srv")
            srv_class = getattr(module, srv)
            client = self.node.create_client(srv_class, service_name)

            if not client.wait_for_service(timeout_sec=3.0):
                return {"error": f"Service '{service_name}' not available (timed out)."}

            request = srv_class.Request()
            set_message_fields(request, fields)

            future = client.call_async(request)
            self._spin_shared_node_until_future_complete(future)
            if future.result() is not None:
                return {"result": str(future.result())}
            else:
                return {"error": "Service call failed."}
        except Exception as e:
            return {"error": str(e)}


class _MCPStreamSession:
    def __init__(
        self,
        *,
        session_id: str,
        kind: str,
        topic_name: str,
        topic_type: str,
        node: Node,
        target_fps: float | None,
        max_width: int | None,
        max_height: int | None,
        jpeg_quality: int,
        max_points: int,
    ):
        self.session_id = session_id
        self.kind = kind
        self.topic_name = topic_name
        self.topic_type = topic_type
        self.node = node

        self.target_fps = target_fps
        self.max_width = max_width
        self.max_height = max_height
        self.jpeg_quality = jpeg_quality
        self.max_points = max_points

        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.seq = 0
        self.rx_count = 0
        self.last_rx_monotonic = 0.0
        self.last_error: str | None = None
        self.last_payload: dict | None = None
        self.last_emit_monotonic = 0.0

        self.subscription = None
        self.executor: SingleThreadedExecutor | None = None
        self.thread: threading.Thread | None = None
    def serialize_msg(self, msg: Any) -> Any:
        try:
            max_list_items = 512
            max_bytes = 2048
            if isinstance(msg, memoryview):
                try:
                    data = list(msg.cast("d"))
                except TypeError:
                    data = list(msg)
                return data[:max_list_items]

            elif isinstance(msg, (bytes, bytearray)):
                return list(msg[:max_bytes])

            elif isinstance(msg, array.array):
                data = msg.tolist()
                return data[:max_list_items]

            elif isinstance(msg, np.ndarray):
                data = msg.flatten().tolist()
                return data[:max_list_items]

            elif isinstance(msg, (int, float, str, bool)) or msg is None:
                return msg

            elif isinstance(msg, (list, tuple)):
                seq = msg[:max_list_items] if len(msg) > max_list_items else msg
                return [self.serialize_msg(item) for item in seq]

            elif hasattr(msg, "__slots__"):
                return {
                    slot: self.serialize_msg(getattr(msg, slot))
                    for slot in msg.__slots__
                }

            elif hasattr(msg, "data"):
                return self.serialize_msg(msg.data)

            elif isinstance(msg, dict):
                return {key: self.serialize_msg(value) for key, value in msg.items()}

            else:
                return str(msg)

        except Exception as e:
            return {"error": f"Failed to serialize message: {str(e)}"}

    def subscribe_topic(
        self,
        topic_name: str,
        duration: float | None = None,
        message_limit: int | None = None,
    ) -> dict:
        import time
        from rclpy.task import Future

        available_topics = self.node.get_topic_names_and_types()
        topic_map = {name: types for name, types in available_topics}
        if topic_name not in topic_map:
            return {
                "error": f"Topic '{topic_name}' not found. Available topics: {list(topic_map.keys())}"
            }

        types = topic_map[topic_name]
        if not types:
            return {"error": f"Topic '{topic_name}' has no associated message types."}

        msg_type = types[0]

        # Fallback to avoid infinite wait
        if not duration and not message_limit:
            duration = SUBCRIPTION_DURATION_TIME  # default duration in seconds

        # Dynamically load message class
        parts = msg_type.split("/")
        if len(parts) == 3:
            pkg, _, msg = parts
        elif len(parts) == 2:
            pkg, msg = parts
        else:
            return {"error": f"Invalid message type format: {msg_type}"}

        try:
            module = importlib.import_module(f"{pkg}.msg")
            msg_class = getattr(module, msg)
        except Exception as e:
            return {"error": f"Failed to import message type: {str(e)}"}

        tmp_node = Node("mcp_subscribe_tmp")
        received = []
        done_future = Future()
        
        qos = self.get_qos_profile_for_topic(tmp_node, topic_name)

        def callback(msg):
            received.append(msg)
            if message_limit and len(received) >= message_limit:
                done_future.set_result(True)

        tmp_node.create_subscription(msg_class, topic_name, callback, qos)
        executor = SingleThreadedExecutor(context=tmp_node.context)
        executor.add_node(tmp_node)

        start = time.time()
        try:
            while rclpy.ok() and not done_future.done():
                executor.spin_once(timeout_sec=0.1)
                if duration and (time.time() - start) >= duration:
                    break
        finally:
            executor.remove_node(tmp_node)
            executor.shutdown()
            tmp_node.destroy_node()

        elapsed = time.time() - start

        if msg_type == "sensor_msgs/msg/Image":
            images = []
            for ros_msg in received:
                if ros_msg.encoding not in ("rgb8", "bgr8", "rgba8"):
                    images.append({"error": f"Unsupported encoding: {ros_msg.encoding}"})
                    continue

                try:
                    if ros_msg.encoding == "rgba8":
                        img_array = np.frombuffer(ros_msg.data, dtype=np.uint8).reshape(
                            (ros_msg.height, ros_msg.width, 4)
                        )
                        pil_img = PILImage.fromarray(img_array, mode="RGBA")
                    else:
                        img_array = np.frombuffer(ros_msg.data, dtype=np.uint8).reshape(
                            (ros_msg.height, ros_msg.width, 3)
                        )
                        if ros_msg.encoding == "bgr8":
                            img_array = img_array[..., ::-1]  # Convert BGR to RGB
                        pil_img = PILImage.fromarray(img_array, mode="RGB")

                    buffer = io.BytesIO()
                    pil_img.save(buffer, format="PNG")
                    image_bytes = buffer.getvalue()

                    images.append({
                        "type": "image",
                        "data": base64.b64encode(image_bytes).decode("utf-8"),
                        "mimeType": "image/png"
                    })
                except Exception as e:
                        images.append({"error": f"Failed to process image: {str(e)}"})
        
            return {
                "messages": images,
                "count": len(received),
                "duration": round(elapsed, 2),
            }
        elif msg_type == "sensor_msgs/msg/CompressedImage":
            images = []
            for ros_msg in received:
                try:
                    pil_img = PILImage.open(io.BytesIO(ros_msg.data))
                    if pil_img.mode not in ("RGB", "RGBA"):
                        pil_img = pil_img.convert("RGB")

                    buffer = io.BytesIO()
                    pil_img.save(buffer, format="PNG")
                    image_bytes = buffer.getvalue()

                    images.append(
                        {
                            "type": "image",
                            "data": base64.b64encode(image_bytes).decode("utf-8"),
                            "mimeType": "image/png",
                        }
                    )
                except Exception as e:
                    images.append(
                        {"error": f"Failed to process compressed image: {str(e)}"}
                    )
            return {
                "messages": images,
                "count": len(received),
                "duration": round(elapsed, 2),
            }
        else:
            return {
                "messages": [self.serialize_msg(msg) for msg in received],
                "count": len(received),
                "duration": round(elapsed, 2),
            }

    def call_get_messages_service_any(self, params: dict) -> dict:
        service_type = get_service("lora_msgs/srv/GetMessages")
        if not service_type:
            raise ImportError("Service type not found for 'GetMessages'")

        client = self.node.create_client(service_type, "/get_messages")
        if not client.wait_for_service(timeout_sec=3.0):
            return {"error": "Service '/get_messages' not available (timeout)."}

        request = service_type.Request()
        request.topic_name = params.get("topic_name")
        request.message_type = "any"
        request.number_of_msgs = params.get("number_of_msgs", 0)

        def parse_iso8601_to_fulldatetime(ts):
            FullDateTime = get_message("lora_msgs/msg/FullDateTime")
            dt = parser.isoparse(ts)
            full = FullDateTime()
            full.year, full.month, full.day = dt.year, dt.month, dt.day
            full.hour, full.minute, full.second = dt.hour, dt.minute, dt.second
            full.nanosecond = dt.microsecond * 1000
            return full

        if params.get("time_start"):
            request.time_start = parse_iso8601_to_fulldatetime(params["time_start"])
        if params.get("time_end"):
            request.time_end = parse_iso8601_to_fulldatetime(params["time_end"])

        # client.call blocks on the response and (depending on the rclpy
        # version) processes it through the global executor: serialize it
        # with every other shared-node spin.
        with self._node_spin_lock:
            response = client.call(request, timeout_sec=5.0)
        if response is None:
            return {"error": "Service call timed out"}

        try:
            MessageType = get_message(params.get("message_type"))
            messages, data, offset = [], response.data, 0

            while offset < len(data):
                length = int.from_bytes(data[offset : offset + 4], "big")
                offset += 4
                msg_bin = bytes(data[offset : offset + length])
                offset += length
                messages.append(deserialize_message(msg_bin, MessageType()))

            def to_dict(msg):
                out = {}
                for f, _ in msg.get_fields_and_field_types().items():
                    v = getattr(msg, f)
                    if hasattr(v, "get_fields_and_field_types"):
                        out[f] = to_dict(v)
                    elif isinstance(v, (list, tuple)):
                        out[f] = [
                            (
                                to_dict(x)
                                if hasattr(x, "get_fields_and_field_types")
                                else x
                            )
                            for x in v
                        ]
                    elif isinstance(v, (np.ndarray, array.array)):
                        out[f] = list(v)
                    elif isinstance(v, (bytes, bytearray)):
                        out[f] = v.decode("utf-8", errors="ignore")
                    else:
                        out[f] = v
                return out

            return {
                "timestamps": [to_dict(t) for t in response.timestamps],
                "messages": [to_dict(m) for m in messages],
            }

        except Exception as e:
            return {"error": f"Deserialization error: {e}"}

    async def publish_to_topic(
        self,
        topic_name: str,
        message_type: str,
        data: dict,
        frequency: float | None = None,
        duration: float | None = None
    ) -> dict:
        # Validate topic_name
        if not isinstance(topic_name, str) or not topic_name.strip():
            return {"error": "Invalid topic name. It must be a non-empty string."}

        # Validate message_type
        if not isinstance(message_type, str) or "/" not in message_type:
            return {
                "error": "Invalid message type. It must be a valid ROS2 message type string."
            }

        # Validate data
        if not isinstance(data, dict):
            return {"error": "Invalid data. It must be a dictionary."}

        try:
            parts = message_type.split("/")
            if len(parts) == 3:
                pkg, _, msg = parts
            elif len(parts) == 2:
                pkg, msg = parts
            else:
                return {"error": f"Invalid message type format: {message_type}"}

            module = importlib.import_module(f"{pkg}.msg")
            msg_class = getattr(module, msg)
        except Exception as e:
            return {"error": f"Failed to load message type: {str(e)}"}

        # Validate timing parameters before creating anything, so a rejected
        # request does not leak a publisher on the shared node.
        timed_publish = frequency is not None and duration is not None
        if timed_publish:
            timing_error = validate_timed_publish_args(frequency, duration)
            if timing_error:
                return {"error": timing_error}

        tmp_node = Node("mcp_publish_tmp")
        try:
            qos = self.get_qos_for_publisher_topic(tmp_node, topic_name)
        finally:
            tmp_node.destroy_node()

        pub = None
        try:
            pub = self.node.create_publisher(msg_class, topic_name, qos)
            msg_instance = msg_class()
            set_message_fields(msg_instance, data)

            if timed_publish:
                # The loop runs on the server's event loop inside this
                # request's own task (the tool dispatches publish through
                # run_tool_async): cancelling the request, or a newer publish
                # on this topic, cancels the loop; the stop message published
                # by that newer call can never be followed by a message of
                # this loop. See server/timed_publish.py.
                outcome = await run_registered_timed_publish(
                    self._timed_publish_registry,
                    self._timed_publish_registry.key_for(topic_name),
                    lambda: pub.publish(msg_instance),
                    frequency=frequency,
                    duration=duration,
                    context_ok=self.node.context.ok,
                )
                return {
                    "status": "published",
                    "data": data,
                    "published_count": outcome["published_count"],
                    "frequency": frequency,
                    "duration": duration,
                    "elapsed_sec": outcome["elapsed_sec"],
                }
            else:
                # Single publish; stop a timed publish running on this topic
                # first. There is no await between the cancel and this
                # publish, so the stop command is the last message on the
                # topic.
                self._timed_publish_registry.cancel(
                    self._timed_publish_registry.key_for(topic_name)
                )
                pub.publish(msg_instance)
                return {"status": "published", "data": data}

        except Exception as e:
            return {"error": "Failed to publish message due to an internal error."}
        finally:
            if pub is not None:
                try:
                    self.node.destroy_publisher(pub)
                except Exception:
                    pass

    def cancel_timed_publish(self, topic_name: str) -> dict:
        """Stop the timed publish running on ``topic_name`` without publishing
        anything (the stopped request then ends as cancelled). Returns whether
        a loop was actually running. Must be called on the event loop."""
        topic_key = self._timed_publish_registry.key_for(topic_name)
        was_active = self._timed_publish_registry.cancel(topic_key)
        return {
            "status": "cancel_requested" if was_active else "no_active_timed_publish",
            "topic": topic_name,
            "was_active": was_active,
        }

    def shutdown(self):
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception as e:
            print(f"ROS shutdown skipped: {e}")

    def send_action_goal(
        self,
        action_name: str,
        action_type: str,
        goal_fields: dict,
        wait_for_result: bool = False,
        timeout_sec: float = 60.0,
    ) -> dict:
        try:
            parts = action_type.split("/")
            if len(parts) == 3:
                pkg, kind, action = parts
                if kind != "action":
                    return {
                        "error": f"Invalid action type (middle part must be 'action'): {action_type}"
                    }
            elif len(parts) == 2:
                pkg, action = parts
            else:
                return {"error": f"Invalid action type format: {action_type}"}

            module = importlib.import_module(f"{pkg}.action")
            action_cls = getattr(module, action)

            client = ActionClient(self.node, action_cls, action_name)

            if not client.wait_for_server(timeout_sec=3.0):
                return {
                    "error": f"Action server '{action_name}' not available (timed out)."
                }

            goal_msg = action_cls.Goal()
            set_message_fields(goal_msg, goal_fields)

            send_future = client.send_goal_async(goal_msg)
            self._spin_shared_node_until_future_complete(send_future)

            goal_handle = send_future.result()
            if goal_handle is None:
                return {"error": "Failed to send goal (no goal handle returned)."}

            accepted = bool(goal_handle.accepted)

            goal_id_hex = None
            try:
                uuid_bytes = getattr(
                    getattr(goal_handle, "goal_id", None), "uuid", None
                )
                if uuid_bytes:
                    goal_id_hex = "".join(f"{b:02x}" for b in uuid_bytes)
            except Exception:
                # Ignore errors extracting goal_id; goal_id_hex will remain None if extraction fails.
                pass

            send_goal_stamp = None
            try:
                stamp = getattr(send_future.result(), "stamp", None)
                if stamp is not None:
                    send_goal_stamp = {
                        "sec": int(getattr(stamp, "sec", 0)),
                        "nanosec": int(getattr(stamp, "nanosec", 0)),
                    }
            except Exception:
                # Ignore errors extracting goal_id; goal_id_hex will remain None if extraction fails.
                pass

            response = {
                "accepted": accepted,
                "goal_id": goal_id_hex,
                "send_goal_stamp": send_goal_stamp,
                "waited": False,
                "result_timeout_sec": None,
                "status_code": None,
                "status": None,
                "result": None,
            }

            if not wait_for_result:
                if not accepted:
                    response["status_code"] = -1
                    response["status"] = GOAL_STATUS[-1]
                else:
                    response["status_code"] = 1
                    response["status"] = GOAL_STATUS[1]
                return response

            if not accepted:
                response["status_code"] = -1
                response["status"] = GOAL_STATUS[-1]
                return response

            response["waited"] = True
            response["result_timeout_sec"] = float(timeout_sec)

            result_future = goal_handle.get_result_async()
            self._spin_shared_node_until_future_complete(
                result_future, timeout_sec=timeout_sec
            )

            if not result_future.done():
                try:
                    cancel_future = goal_handle.cancel_goal_async()
                    self._spin_shared_node_until_future_complete(
                        cancel_future, timeout_sec=3.0
                    )
                except Exception:
                    # Ignore errors extracting goal_id; goal_id_hex will remain None if extraction fails.
                    pass
                response["status_code"] = 0
                response["status"] = "TIMEOUT"
                return response

            result_msg = result_future.result()
            status_code = int(getattr(result_msg, "status", 0))
            status_text = GOAL_STATUS.get(status_code, str(status_code))

            result_payload = getattr(result_msg, "result", None)
            if result_payload is not None:
                try:
                    result_dict = message_to_ordereddict(result_payload)
                except Exception:
                    result_dict = {"repr": repr(result_payload)}
            else:
                result_dict = None

            response["status_code"] = status_code
            response["status"] = status_text
            response["result"] = result_dict
            return response

        except Exception as e:
            return {"error": str(e)}

    def _hex_to_uuid_bytes(self, hex_str: str) -> list[int]:
        hex_clean = hex_str.replace("-", "").strip().lower()
        if len(hex_clean) != 32:
            raise ValueError("goal_id must be 32 hex chars (no dashes)")
        return list(bytes.fromhex(hex_clean))

    def cancel_action_goal(
        self,
        node: Node,
        action_name: str,
        goal_id_hex: str | None = None,
        *,
        cancel_all: bool = False,
        stamp_sec: int = 0,
        stamp_nanosec: int = 0,
        wait_timeout_sec: float = 3.0,
    ) -> dict:
        try:
            srv_mod = importlib.import_module("action_msgs.srv")
            CancelGoal = getattr(srv_mod, "CancelGoal")

            service_name = action_name.rstrip("/") + "/_action/cancel_goal"
            client = node.create_client(CancelGoal, service_name)
            if not client.wait_for_service(timeout_sec=wait_timeout_sec):
                return {"error": f"Service '{service_name}' not available (timed out)."}

            req = CancelGoal.Request()
            if cancel_all:
                req.goal_info.goal_id.uuid = [0] * 16
            else:
                if not goal_id_hex:
                    return {"error": "goal_id is required unless cancel_all=True"}
                req.goal_info.goal_id.uuid = self._hex_to_uuid_bytes(goal_id_hex)

            req.goal_info.stamp.sec = int(stamp_sec)
            req.goal_info.stamp.nanosec = int(stamp_nanosec)

            future = client.call_async(req)
            self._spin_shared_node_until_future_complete(
                future, timeout_sec=wait_timeout_sec, node=node
            )
            resp = future.result()
            if resp is None:
                return {"error": "CancelGoal call failed or timed out."}

            out = {
                "service": service_name,
                "return_code": int(getattr(resp, "return_code", -1)),
                "return_code_text": GOAL_CANCEL_RET.get(
                    int(getattr(resp, "return_code", -1)), "UNKNOWN"
                ),
                "goals_canceling": [],
            }

            try:
                resp_dict = message_to_ordereddict(resp)
                for g in resp_dict.get("goals_canceling", []):
                    uuid_list = g.get("goal_id", {}).get("uuid", [])
                    hex_id = "".join(f"{b:02x}" for b in uuid_list)
                    out["goals_canceling"].append(
                        {
                            "goal_id": hex_id,
                            "stamp": g.get("stamp", None),
                        }
                    )
            except Exception:
                try:
                    for gi in getattr(resp, "goals_canceling", []):
                        uuid_bytes = getattr(getattr(gi, "goal_id", None), "uuid", b"")
                        hex_id = "".join(f"{b:02x}" for b in uuid_bytes)
                        stamp = getattr(gi, "stamp", None)
                        out["goals_canceling"].append(
                            {
                                "goal_id": hex_id or None,
                                "stamp": (
                                    {
                                        "sec": getattr(stamp, "sec", None),
                                        "nanosec": getattr(stamp, "nanosec", None),
                                    }
                                    if stamp
                                    else None
                                ),
                            }
                        )
                except Exception:
                    out["repr"] = repr(resp)

            return out

        except Exception as e:
            return {"error": str(e)}

    def action_request_result(
        self,
        action_name: str,
        action_type: str,
        goal_id_hex: str,
        timeout_sec: float | None = 60.0,
        wait_for_service_sec: float = 3.0,
    ) -> dict:
        try:
            parts = action_type.split("/")
            if len(parts) == 3:
                pkg, kind, action = parts
                if kind != "action":
                    return {
                        "error": f"Invalid action type (middle part must be 'action'): {action_type}"
                    }
            elif len(parts) == 2:
                pkg, action = parts
            else:
                return {"error": f"Invalid action type format: {action_type}"}

            if not goal_id_hex or len(goal_id_hex) != 32:
                return {"error": "goal_id_hex must be a 32-character hex string (no dashes)"}

            module = importlib.import_module(f"{pkg}.action")
            action_cls = getattr(module, action)

            service_name = action_name.rstrip("/") + "/_action/get_result"
            GetResult = action_cls.GetResult
            client = self.node.create_client(GetResult, service_name)
            if not client.wait_for_service(timeout_sec=wait_for_service_sec):
                return {"error": f"Service '{service_name}' not available (timed out)."}

            req = GetResult.Request()
            try:
                req.goal_id.uuid = self._hex_to_uuid_bytes(goal_id_hex)
            except Exception as e:
                return {"error": f"Invalid goal_id_hex: {e}"}

            future = client.call_async(req)
            self._spin_shared_node_until_future_complete(future, timeout_sec=timeout_sec)

            if not future.done():
                return {
                    "service": service_name,
                    "goal_id": goal_id_hex,
                    "waited": True,
                    "result_timeout_sec": float(timeout_sec) if timeout_sec is not None else None,
                    "status_code": 0,
                    "status": "TIMEOUT",
                    "result": None,
                }

            resp = future.result()
            if resp is None:
                return {"error": "GetResult call failed or returned None."}

            status_code = int(getattr(resp, "status", 0))
            status_text = GOAL_STATUS.get(status_code, str(status_code))

            result_payload = getattr(resp, "result", None)
            result_dict = None
            if result_payload is not None:
                try:
                    result_dict = message_to_ordereddict(result_payload)
                except Exception:
                    result_dict = {"repr": repr(result_payload)}

            return {
                "service": service_name,
                "goal_id": goal_id_hex,
                "waited": True,
                "result_timeout_sec": float(timeout_sec) if timeout_sec is not None else None,
                "status_code": status_code,
                "status": status_text,
                "result": result_dict,
            }

        except Exception as e:
            return {"error": str(e)}

    def action_subscribe_feedback(
        self,
        action_name: str,
        action_type: str,
        goal_id_hex: str | None = None,
        duration_sec: float = 5.0,
        max_messages: int = 100,
    ) -> dict:
        sub = None
        try:
            parts = (action_type or "").split("/")
            if len(parts) == 3:
                pkg, kind, action = parts
                if kind != "action":
                    return {"error": f"Invalid action type (middle part must be 'action'): {action_type}"}
            elif len(parts) == 2:
                pkg, action = parts
            else:
                return {"error": f"Invalid action type format: {action_type}"}

            module = importlib.import_module(f"{pkg}.action")
            action_cls = getattr(module, action)
            FeedbackMessage = action_cls.Impl.FeedbackMessage

            action_name = (action_name or "").strip()
            if not action_name:
                return {"error": "action_name is required"}
            topic = action_name.rstrip("/") + "/_action/feedback"

            if goal_id_hex:
                goal_id_hex = goal_id_hex.replace("-", "").strip().lower()
                if len(goal_id_hex) != 32:
                    return {"error": "goal_id_hex must be 32 hex chars (no dashes)"}

            out = {
                "topic": topic,
                "action_type": action_type,
                "goal_id_filter": goal_id_hex,
                "duration_sec": float(duration_sec),
                "messages": [],
            }

            tmp_node = Node("mcp_subscribe_tmp")
            qos = self.get_qos_profile_for_topic(tmp_node, topic)

            def _cb(msg):
                try:
                    uuid_field = getattr(getattr(msg, "goal_id", None), "uuid", None)
                    gid_hex = None
                    if uuid_field is not None:
                        gid_hex = "".join(f"{int(b):02x}" for b in list(uuid_field))

                    if goal_id_hex and gid_hex != goal_id_hex:
                        return

                    fb = getattr(msg, "feedback", None)
                    try:
                        fb_dict = message_to_ordereddict(fb) if fb is not None else None
                    except Exception:
                        fb_dict = {"repr": repr(fb)}

                    now = self.node.get_clock().now().to_msg()
                    out["messages"].append(
                        {
                            "goal_id": gid_hex,
                            "feedback": fb_dict,
                            "recv_stamp": {"sec": int(now.sec), "nanosec": int(now.nanosec)},
                        }
                    )
                except Exception as e:
                    out.setdefault("warn", []).append(str(e))

            if not hasattr(self, "_tmp_subs"):
                self._tmp_subs = []
            sub = self.node.create_subscription(FeedbackMessage, topic, _cb, qos)
            self._tmp_subs.append(sub)

            end_time = self.node.get_clock().now().nanoseconds + int(duration_sec * 1e9)
            while self.node.get_clock().now().nanoseconds < end_time:
                if len(out["messages"]) >= max_messages:
                    break
                self._spin_shared_node_once(0.1)

            return out

        except Exception as e:
            return {"error": str(e)}

        finally:
            try:
                if sub is not None:
                    self.node.destroy_subscription(sub)
            finally:
                try:
                    if sub is not None and hasattr(self, "_tmp_subs"):
                        self._tmp_subs = [s for s in self._tmp_subs if s is not sub]
                except Exception:
                    # Ignore errors extracting goal_id; goal_id_hex will remain None if extraction fails.
                    pass

    def action_subscribe_status(
        self,
        action_name: str,
        duration_sec: float = 5.0,
        max_messages: int = 100,
    ) -> dict:
        try:
            from action_msgs.msg import GoalStatusArray

            action_name = (action_name or "").strip()
            if not action_name:
                return {"error": "action_name is required"}
            topic = action_name.rstrip("/") + "/_action/status"

            out = {
                "topic": topic,
                "duration_sec": float(duration_sec),
                "frames": [],
            }

            tmp_node = Node("mcp_subscribe_tmp")
            qos = self.get_qos_profile_for_topic(tmp_node, topic)

            def _cb(msg: "GoalStatusArray"):
                try:
                    frame = {"stamp": None, "statuses": []}
                    for st in getattr(msg, "status_list", []) or []:
                        gi = getattr(st, "goal_info", None)
                        uuid_field = getattr(getattr(gi, "goal_id", None), "uuid", None)
                        if uuid_field is not None:
                            try:
                                gid_hex = "".join(f"{int(b):02x}" for b in list(uuid_field))
                            except Exception:
                                gid_hex = None
                        else:
                            gid_hex = None
                        accept_stamp = getattr(gi, "stamp", None)
                        accept_stamp_dict = (
                            {"sec": int(getattr(accept_stamp, "sec", 0)),
                            "nanosec": int(getattr(accept_stamp, "nanosec", 0))}
                            if accept_stamp else None
                        )
                        code = int(getattr(st, "status", 0))
                        frame["statuses"].append(
                            {
                                "goal_id": gid_hex,
                                "accept_stamp": accept_stamp_dict,
                                "status_code": code,
                                "status": GOAL_STATUS.get(code, str(code)),
                            }
                        )
                    try:
                        stamp = getattr(msg, "stamp", None)
                        frame["stamp"] = (
                            {"sec": int(getattr(stamp, "sec", 0)),
                            "nanosec": int(getattr(stamp, "nanosec", 0))}
                            if stamp else None
                        )
                    except Exception:
                        frame["stamp"] = None
                    out["frames"].append(frame)
                except Exception as e:
                    out.setdefault("warn", []).append(str(e))

            if not hasattr(self, "_tmp_subs"):
                self._tmp_subs = []
            sub = self.node.create_subscription(GoalStatusArray, topic, _cb, qos)
            self._tmp_subs.append(sub)

            end_time = self.node.get_clock().now().nanoseconds + int(duration_sec * 1e9)
            while self.node.get_clock().now().nanoseconds < end_time:
                if len(out["frames"]) >= max_messages:
                    break
                self._spin_shared_node_once(0.1)

            return out

        except Exception as e:
            return {"error": str(e)}

        finally:
            try:
                if sub is not None:
                    self.node.destroy_subscription(sub)
            finally:
                try:
                    if sub is not None and hasattr(self, "_tmp_subs"):
                        self._tmp_subs = [s for s in self._tmp_subs if s is not sub]
                except Exception:
                    pass


# Backward-compatibility: during incremental edits, some ROS2Manager methods may
# temporarily live on _MCPStreamSession. Expose them on ROS2Manager so existing
# tools keep working.
for _name in (
    "serialize_msg",
    "subscribe_topic",
    "call_get_messages_service_any",
    "publish_to_topic",
    "cancel_timed_publish",
    "shutdown",
    "send_action_goal",
    "cancel_action_goal",
    "action_request_result",
    "action_subscribe_feedback",
    "action_subscribe_status",
):
    if not hasattr(ROS2Manager, _name) and hasattr(_MCPStreamSession, _name):
        setattr(ROS2Manager, _name, getattr(_MCPStreamSession, _name))
