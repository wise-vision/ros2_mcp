#
#  Copyright (C) 2025 wisevision
#
#  SPDX-License-Identifier: MPL-2.0
#
#  This Source Code Form is subject to the terms of the Mozilla Public
#  License, v. 2.0. If a copy of the MPL was not distributed with this
#  file, You can obtain one at https://mozilla.org/MPL/2.0/.
#
import typing as t
from typing import TypedDict, Optional, List, Dict
import importlib
from rclpy.task import Future

from .ros2_manager import ROS2Manager
import time
import rclpy
from mcp.types import ImageContent
import numpy as np
from PIL import Image as PILImage
import base64
import io
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
import threading
from rclpy.qos import QoSDurabilityPolicy, QoSPresetProfiles, QoSReliabilityPolicy
from rosidl_runtime_py.set_message import set_message_fields
import math
from sensor_msgs.msg import PointCloud2, PointField


class TopicSubscriptionConfig(TypedDict):
    name: str
    duration: Optional[float]
    message_limit: Optional[int]


class TopicPublishConfig(TypedDict):
    topic_name: str
    message_type: str
    data: dict
    frequency: Optional[float]
    duration: Optional[float]


class ExtendedROS2Manager:
    """Multi-topic pub/sub and image rendering (map, point cloud BEV) on top of ROS2Manager."""

    def __init__(self, base_manager: t.Optional[ROS2Manager] = None):
        if base_manager is None:
            from .tools_ros2 import get_ros

            base_manager = get_ros()
        self.base_manager = base_manager

    def subscribe_multiple_topics(
        self, topic_configs: List[TopicSubscriptionConfig]
    ) -> dict:
        """
        Subscribes to multiple ROS2 topics concurrently.

        :param topic_configs: List of dicts like:
            {"name": str, "duration": float (optional), "message_limit": int (optional)}
        :return: Dict mapping topic names to received messages and metadata
        """
        tmp_node = Node("mcp_multi_subscriber")
        executor = SingleThreadedExecutor(context=tmp_node.context)
        executor.add_node(tmp_node)

        available_topics = tmp_node.get_topic_names_and_types()
        topic_map = {name: types for name, types in available_topics}

        subscriptions = []
        result = {}
        now = time.time()

        for config in topic_configs:
            name = config.get("name")
            if not name:
                result["<invalid_or_empty>"] = {"error": "Missing or empty topic name"}
                continue
            duration = config.get("duration")
            limit = config.get("message_limit")
            if not duration and not limit:
                duration = 5.0

            if name not in topic_map or not topic_map[name]:
                result[name] = {"error": f"Topic not found or has no types"}
                continue

            msg_type_str = topic_map[name][0]
            parts = msg_type_str.split("/")
            if len(parts) < 2:
                result[name] = {"error": f"Invalid msg type: {msg_type_str}"}
                continue

            try:
                module = importlib.import_module(f"{parts[0]}.msg")
                msg_class = getattr(module, parts[-1])
            except Exception as e:
                result[name] = {"error": f"Import failed: {str(e)}"}
                continue

            received = []
            future = Future()
            qos = self.base_manager.get_qos_profile_for_topic(tmp_node, name)

            def make_cb(topic_name, rec_list, fut, lim):
                def _cb(msg):
                    rec_list.append(msg)
                    if lim and len(rec_list) >= lim and not fut.done():
                        fut.set_result(True)

                return _cb

            cb = make_cb(name, received, future, limit)
            sub = tmp_node.create_subscription(msg_class, name, cb, qos)

            subscriptions.append(
                {
                    "topic": name,
                    "msg_type_str": msg_type_str,
                    "msg_class": msg_class,
                    "received": received,
                    "future": future,
                    "deadline": now + duration if duration else None,
                    "subscription": sub,
                }
            )

        try:
            while rclpy.ok():
                executor.spin_once(timeout_sec=0.1)
                all_done = True
                now = time.time()
                for subscription in subscriptions:
                    if subscription["future"].done():
                        if subscription.get("subscription") is not None:
                            tmp_node.destroy_subscription(subscription["subscription"])
                            subscription["subscription"] = None
                        continue
                    if subscription["deadline"] and now >= subscription["deadline"]:
                        subscription["future"].set_result(True)
                    else:
                        all_done = False
                if all_done:
                    break
        finally:
            executor.remove_node(tmp_node)
            executor.shutdown()
            tmp_node.destroy_node()

        for subscription in subscriptions:
            topic = subscription["topic"]
            msg_type_str = subscription["msg_type_str"]
            received = subscription["received"]
            if msg_type_str == "sensor_msgs/msg/Image":
                # process images into base64 PNGs
                images = []
                for ros_msg in received:
                    if ros_msg.encoding not in ("rgb8", "bgr8", "rgba8"):
                        images.append(
                            {"error": f"Unsupported encoding: {ros_msg.encoding}"}
                        )
                        continue

                    try:
                        if ros_msg.encoding == "rgba8":
                            img_array = np.frombuffer(
                                ros_msg.data, dtype=np.uint8
                            ).reshape((ros_msg.height, ros_msg.width, 4))
                            pil_img = PILImage.fromarray(img_array, mode="RGBA")
                        else:
                            img_array = np.frombuffer(
                                ros_msg.data, dtype=np.uint8
                            ).reshape((ros_msg.height, ros_msg.width, 3))
                            if ros_msg.encoding == "bgr8":
                                img_array = img_array[..., ::-1]  # Convert BGR to RGB
                            pil_img = PILImage.fromarray(img_array, mode="RGB")

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
                        images.append({"error": f"Failed to process image: {str(e)}"})

                result[topic] = {"count": len(received), "messages": images}

            elif msg_type_str == "sensor_msgs/msg/CompressedImage":
                # process compressed images (jpeg/png) into base64 PNGs
                images = []
                for ros_msg in received:
                    try:
                        # ros_msg.format is e.g. "jpeg" or "jpeg; quality=90"; Pillow
                        # detects the codec from the bytes, so it is not needed here.
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

                result[topic] = {"count": len(received), "messages": images}
            else:
                result[topic] = {
                    "count": len(subscription["received"]),
                    "messages": [
                        self.base_manager.serialize_msg(messages)
                        for messages in subscription["received"]
                    ],
                }

        return result

    def publish_multiple_topics(
        self, configs: List[TopicPublishConfig]
    ) -> Dict[str, str]:
        results = {}

        def publish_loop(pub, msg_instance, freq, duration, topic_name):
            interval = 1.0 / freq
            end_time = time.time() + duration
            while time.time() < end_time and self.base_manager.node.context.ok():
                pub.publish(msg_instance)
                time.sleep(interval)

        threads = []
        for config in configs:
            topic_name = config.get("topic_name")
            msg_type = config.get("message_type")
            data = config.get("data")

            if not topic_name or not msg_type or not isinstance(data, dict):
                results[topic_name or "<unknown>"] = "Invalid config"
                continue

            try:
                pkg, msg = msg_type.replace("msg/", "").split("/")
                module = importlib.import_module(f"{pkg}.msg")
                msg_class = getattr(module, msg)
            except Exception as e:
                results[topic_name] = f"Invalid message type: {e}"
                continue

            try:
                qos = self.base_manager.get_qos_for_publisher_topic(
                    self.base_manager.node, topic_name
                )
                if qos is None:
                    pub = self.base_manager.node.create_publisher(
                        msg_class, topic_name, 10
                    )
                else:
                    pub = self.base_manager.node.create_publisher(
                        msg_class, topic_name, qos
                    )
                msg_instance = msg_class()
                set_message_fields(msg_instance, data)
                freq = config.get("frequency", 1.0)
                duration = config.get("duration", 5.0)

                thread = threading.Thread(
                    target=publish_loop,
                    args=(pub, msg_instance, freq, duration, topic_name),
                    daemon=True,
                )
                thread.start()
                threads.append(thread)
                results[topic_name] = "Publishing started"
            except Exception as e:
                results[topic_name] = f"Failed to publish: {e}"

        for thread in threads:
            thread.join()
        return results

    def get_single_map_from_topic(
        self, topic_name: str, timeout: float = 5.0
    ) -> ImageContent:
        received_msg = {"msg": None}

        def map_callback(msg: OccupancyGrid):
            received_msg["msg"] = msg

        tmp_node = Node(
            "mcp_single_map_subscriber",
            context=self.base_manager.node.context,
            namespace=self.base_manager.node.get_namespace(),
        )
        executor = SingleThreadedExecutor(context=tmp_node.context)
        executor.add_node(tmp_node)

        map_sub = None
        ros_msg: OccupancyGrid | None = None
        try:
            topic_types = dict(tmp_node.get_topic_names_and_types())
            if topic_name not in topic_types:
                raise ValueError(f"Topic {topic_name} not found.")
            if "nav_msgs/msg/OccupancyGrid" not in topic_types[topic_name]:
                raise ValueError(
                    f"Unsupported type on '{topic_name}': {topic_types[topic_name]} (expected nav_msgs/msg/OccupancyGrid)"
                )

            qos_profile = self.base_manager.get_qos_profile_for_topic(
                tmp_node, topic_name
            )

            map_sub = tmp_node.create_subscription(
                OccupancyGrid, topic_name, map_callback, qos_profile
            )

            start_time = time.time()
            while rclpy.ok(context=tmp_node.context) and received_msg["msg"] is None:
                executor.spin_once(timeout_sec=0.1)
                if time.time() - start_time > timeout:
                    raise TimeoutError(
                        f"No map received from topic '{topic_name}' within {timeout} seconds."
                    )

            ros_msg = received_msg["msg"]
            if ros_msg is None:
                raise RuntimeError(
                    f"Failed to receive a map from '{topic_name}' despite the executor loop completing."
                )
        finally:
            if map_sub is not None:
                tmp_node.destroy_subscription(map_sub)
            executor.remove_node(tmp_node)
            executor.shutdown()
            tmp_node.destroy_node()

        w = int(ros_msg.info.width)
        h = int(ros_msg.info.height)
        data = np.array(ros_msg.data, dtype=np.int16)

        if data.size != w * h:
            raise ValueError(
                f"OccupancyGrid data size mismatch: got {data.size}, expected {w*h} ({w}x{h})"
            )

        img = np.empty_like(data, dtype=np.uint8)
        unknown_mask = data < 0
        occupied_mask = data >= 65
        free_mask = (~unknown_mask) & (~occupied_mask)

        img[unknown_mask] = 205
        img[occupied_mask] = 0
        img[free_mask] = 255

        img = img.reshape((h, w))
        img = np.flipud(img)

        pil_img = PILImage.fromarray(img, mode="L")
        buffer = io.BytesIO()
        pil_img.save(buffer, format="PNG")
        image_bytes = buffer.getvalue()

        return ImageContent(
            type="image",
            data=base64.b64encode(image_bytes).decode("utf-8"),
            mimeType="image/png",
        )

    def _build_dtype_for_pointcloud2(self, msg: PointCloud2):
        np_types = {
            PointField.INT8: np.int8,
            PointField.UINT8: np.uint8,
            PointField.INT16: np.int16,
            PointField.UINT16: np.uint16,
            PointField.INT32: np.int32,
            PointField.UINT32: np.uint32,
            PointField.FLOAT32: np.float32,
            PointField.FLOAT64: np.float64,
        }
        fields = []
        for f in msg.fields:
            baset = np_types.get(f.datatype)
            if baset is None:
                raise ValueError(
                    f"Unsupported PointField datatype: {f.datatype} for '{f.name}'"
                )
            fields.append((f.name, baset, (f.count or 1), f.offset))
        fields.sort(key=lambda t: t[3])
        dtype_fields = []
        last_off = 0
        for name, baset, count, off in fields:
            if off > last_off:
                dtype_fields.append(("_pad_" + str(off), np.uint8, off - last_off))
            base = (baset, (count,)) if count != 1 else baset
            dtype_fields.append((name, base))
            last_off = off + np.dtype(base).itemsize
        if last_off < msg.point_step:
            dtype_fields.append(("_pad_end", np.uint8, msg.point_step - last_off))
        return np.dtype(dtype_fields)

    def _colormap_jet_0_255(self, vals_u8: np.ndarray) -> np.ndarray:
        v = vals_u8.astype(np.float32) / 255.0
        r = np.clip(1.5 - np.abs(4.0 * v - 3.0), 0.0, 1.0)
        g = np.clip(1.5 - np.abs(4.0 * v - 2.0), 0.0, 1.0)
        b = np.clip(1.5 - np.abs(4.0 * v - 1.0), 0.0, 1.0)
        return (np.stack([r, g, b], axis=-1) * 255.0).astype(np.uint8)

    def get_single_bev_from_pointcloud(
        self,
        topic_name: str,
        timeout: float = 5.0,
        resolution: float = 0.05,
        z_filter: tuple[float, float] | None = None,
        max_pixels: int = 2048,
        color_mode: str = "intensity",
        colormap: str = "jet",
    ) -> "ImageContent":
        if max_pixels <= 0:
            raise ValueError("max_pixels must be a positive integer")

        received = {"msg": None}

        def cb(msg: PointCloud2):
            received["msg"] = msg

        tmp_node = Node(
            "mcp_single_pointcloud_subscriber",
            context=self.base_manager.node.context,
            namespace=self.base_manager.node.get_namespace(),
        )
        executor = SingleThreadedExecutor(context=tmp_node.context)
        executor.add_node(tmp_node)

        pc_sub = None
        ros_msg: PointCloud2 | None = None
        try:
            topic_types = dict(tmp_node.get_topic_names_and_types())
            if topic_name not in topic_types:
                raise ValueError(f"Topic {topic_name} not found.")
            if "sensor_msgs/msg/PointCloud2" not in topic_types[topic_name]:
                raise ValueError(
                    f"Unsupported type on '{topic_name}': {topic_types[topic_name]} (expected PointCloud2)"
                )

            qos = self.base_manager.get_qos_profile_for_topic(tmp_node, topic_name)
            pc_sub = tmp_node.create_subscription(PointCloud2, topic_name, cb, qos)

            start = time.time()
            while rclpy.ok(context=tmp_node.context) and received["msg"] is None:
                executor.spin_once(timeout_sec=0.1)
                if time.time() - start > timeout:
                    raise TimeoutError(
                        f"No PointCloud2 received from '{topic_name}' within {timeout} s."
                    )

            ros_msg = received["msg"]
            if ros_msg is None:
                raise RuntimeError(
                    f"Failed to receive a PointCloud2 from '{topic_name}' despite the executor loop completing."
                )
        finally:
            if pc_sub is not None:
                tmp_node.destroy_subscription(pc_sub)
            executor.remove_node(tmp_node)
            executor.shutdown()
            tmp_node.destroy_node()

        dtype = self._build_dtype_for_pointcloud2(ros_msg)
        count = ros_msg.width * ros_msg.height
        arr = np.frombuffer(ros_msg.data, dtype=dtype, count=count)

        if not all(k in arr.dtype.names for k in ("x", "y")):
            raise ValueError("PointCloud2 missing x or y")
        x = arr["x"].astype(np.float32)
        y = arr["y"].astype(np.float32)
        z = arr["z"].astype(np.float32) if "z" in arr.dtype.names else np.zeros_like(x)

        mask = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
        if z_filter:
            zmin, zmax = z_filter
            mask &= (z >= zmin) & (z <= zmax)
        x, y, z = x[mask], y[mask], z[mask]

        if x.size == 0:
            img = np.zeros((256, 256, 3), dtype=np.uint8)
            pil_img = PILImage.fromarray(img, mode="RGB")
            buf = io.BytesIO()
            pil_img.save(buf, format="PNG")
            return ImageContent(
                type="image",
                data=base64.b64encode(buf.getvalue()).decode("utf-8"),
                mimeType="image/png",
            )

        xmin, xmax = np.percentile(x, [1, 99])
        ymin, ymax = np.percentile(y, [1, 99])
        if xmax - xmin < 1e-3:
            xmin, xmax = xmin - 1, xmax + 1
        if ymax - ymin < 1e-3:
            ymin, ymax = ymin - 1, ymax + 1

        W = int(math.ceil((xmax - xmin) / resolution)) + 1
        H = int(math.ceil((ymax - ymin) / resolution)) + 1
        scale = max(W / max_pixels, H / max_pixels, 1.0)
        if scale > 1.0:
            resolution *= scale
            W = int(math.ceil((xmax - xmin) / resolution)) + 1
            H = int(math.ceil((ymax - ymin) / resolution)) + 1

        ix = ((x - xmin) / resolution).astype(np.int32)
        iy = ((ymax - y) / resolution).astype(np.int32)
        valid = (ix >= 0) & (ix < W) & (iy >= 0) & (iy < H)
        ix, iy, z = ix[valid], iy[valid], z[valid]

        if color_mode == "rgb":
            img = np.zeros((H, W, 3), dtype=np.uint8)
            if "rgb" in arr.dtype.names or "rgba" in arr.dtype.names:
                field = "rgb" if "rgb" in arr.dtype.names else "rgba"
                packed = arr[field][mask][valid].view(np.uint32)
                r = ((packed >> 16) & 0xFF).astype(np.uint8)
                g = ((packed >> 8) & 0xFF).astype(np.uint8)
                b = (packed & 0xFF).astype(np.uint8)
            elif all(c in arr.dtype.names for c in ("r", "g", "b")):
                r = np.clip(arr["r"][mask][valid], 0, 255).astype(np.uint8)
                g = np.clip(arr["g"][mask][valid], 0, 255).astype(np.uint8)
                b = np.clip(arr["b"][mask][valid], 0, 255).astype(np.uint8)
            else:
                r = g = b = np.full(ix.shape, 255, dtype=np.uint8)
            img[iy, ix, 0] = r
            img[iy, ix, 1] = g
            img[iy, ix, 2] = b

        else:
            if color_mode == "intensity" and "intensity" in arr.dtype.names:
                s = arr["intensity"][mask][valid].astype(np.float32)
            elif color_mode == "height":
                s = z.astype(np.float32)
            else:
                s = np.hypot(x[valid], y[valid])
            lo, hi = (
                np.percentile(s, [1, 99])
                if s.size > 10
                else (float(s.min()), float(s.max() + 1))
            )
            s_norm = np.clip((s - lo) / (hi - lo + 1e-9), 0, 1)
            s_u8 = (s_norm * 255).astype(np.uint8)

            img_scalar = np.zeros((H, W), dtype=np.uint8)
            lin = iy * W + ix
            np.maximum.at(img_scalar.ravel(), lin, s_u8)

            if colormap == "gray":
                img = np.repeat(img_scalar[..., None], 3, axis=2)
            else:
                img = self._colormap_jet_0_255(img_scalar)

        pil_img = PILImage.fromarray(img, mode="RGB")
        buf = io.BytesIO()
        pil_img.save(buf, format="PNG")
        return ImageContent(
            type="image",
            data=base64.b64encode(buf.getvalue()).decode("utf-8"),
            mimeType="image/png",
        )
