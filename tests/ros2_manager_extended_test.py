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

import base64
from server.ros2_manager_extended import ExtendedROS2Manager
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Image
import numpy as np

import threading
import time


FREQUENCY = 200

class ThreadedPublisher(Node):
    def __init__(self, topic_name: str, x_val: float, rate_hz: float = 10.0):
        super().__init__("threaded_publisher")
        self.publisher = self.create_publisher(Imu, topic_name, 10)
        self.x_val = x_val
        self.rate_hz = rate_hz
        self._running = False
        self._thread = None

    def start(self):
        if not self._running:
            self._running = True
            self._thread = threading.Thread(target=self._run_loop, daemon=True)
            self._thread.start()

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join()
        self.destroy_node()

    def _run_loop(self):
        period = 1.0 / self.rate_hz
        while self._running and rclpy.ok():
            msg = Imu()
            msg.linear_acceleration.x = self.x_val
            self.publisher.publish(msg)
            self.get_logger().info(f"Publishing x={self.x_val} to {self.publisher.topic}")
            time.sleep(period)
            
class ThreadedImagePublisher(Node):
    def __init__(self, topic_name: str, rate_hz: float = 1.0):
        super().__init__('threaded_image_publisher')
        self.publisher = self.create_publisher(Image, topic_name, 10)
        self.rate_hz = rate_hz
        self._running = False
        self._thread = None

    def start(self):
        if not self._running:
            self._running = True
            self._thread = threading.Thread(target=self._run_loop, daemon=True)
            self._thread.start()

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join()
        self.destroy_node()

    def _run_loop(self):
        period = 1.0 / self.rate_hz
        height, width = 480, 640

        red_image = np.zeros((height, width, 3), dtype=np.uint8)
        red_image[:, :] = [0, 0, 255]

        while self._running and rclpy.ok():
            msg = Image()
            msg.height, msg.width = height, width
            msg.encoding = "bgr8"
            msg.step = width * 3
            msg.data = red_image.tobytes()
            msg.header.frame_id = "camera_frame"
            self.publisher.publish(msg)
            self.get_logger().info(f"Publishing red image to {self.publisher.topic}")
            time.sleep(period)

def test_subscribe_multiple_topics():

    rclpy.init()
    topic_1 = "/topic_1"
    topic_2 = "/topic_2"
    pub_1 = ThreadedPublisher(topic_1, x_val=1.0, rate_hz=5.0)
    pub_2 = ThreadedPublisher(topic_2, x_val=2.0, rate_hz=10.0)

    pub_1.start()
    pub_2.start()

    configs = [
        {"name": topic_1, "message_limit": 1},
        {"name": topic_2, "message_limit": 1}
    ]

    ros = ExtendedROS2Manager()
    result = ros.subscribe_multiple_topics(configs)

    pub_1.stop()
    pub_2.stop()

    rclpy.shutdown()

    assert topic_1 in result
    assert result[topic_1]["count"] == 1
    imu_1 = result[topic_1]["messages"][0]
    assert abs(imu_1["_linear_acceleration"]["_x"] - 1.0) < 0.001

    assert topic_2 in result
    assert result[topic_2]["count"] == 1
    imu_2 = result[topic_2]["messages"][0]
    assert abs(imu_2["_linear_acceleration"]["_x"] - 2.0) < 0.001

def test_subscribe_multiple_topics_with_one_no_exist():

    rclpy.init()
    topic_1 = "/topic_1"
    topic_2 = "/topic_2"
    pub_1 = ThreadedPublisher(topic_1, x_val=1.0, rate_hz=5.0)

    pub_1.start()

    configs = [
        {"name": topic_1, "message_limit": 1},
        {"name": topic_2, "message_limit": 1}
    ]

    ros = ExtendedROS2Manager()
    result = ros.subscribe_multiple_topics(configs)

    pub_1.stop()

    rclpy.shutdown()

    assert topic_1 in result
    assert result[topic_1]["count"] == 1
    imu_1 = result[topic_1]["messages"][0]
    assert abs(imu_1["_linear_acceleration"]["_x"] - 1.0) < 0.001

    assert topic_2 in result
    assert result[topic_2]["error"] == "Topic not found or has no types"

def test_subscribe_multiple_topics_with_image_message():

    rclpy.init()
    topic_1 = "/topic_1"
    topic_2 = "/topic_2"
    pub_1 = ThreadedImagePublisher(topic_1, rate_hz=5.0)
    pub_2 = ThreadedImagePublisher(topic_2, rate_hz=10.0)

    pub_1.start()
    pub_2.start()

    configs = [
        {"name": topic_1, "message_limit": 1},
        {"name": topic_2, "message_limit": 1}
    ]

    ros = ExtendedROS2Manager()
    result = ros.subscribe_multiple_topics(configs)

    pub_1.stop()
    pub_2.stop()

    rclpy.shutdown()

    assert topic_1 in result
    assert result[topic_1]["count"] == 1
    msg_1 = result[topic_1]["messages"][0]
    assert isinstance(msg_1, dict)
    assert msg_1["type"] == "image"
    assert msg_1["mimeType"] == "image/png"

    # Checking if base64 produces the correct image
    decoded_bytes = base64.b64decode(msg_1["data"])
    assert decoded_bytes.startswith(b"\x89PNG\r\n\x1a\n") 

    assert topic_2 in result
    assert result[topic_2]["count"] == 1
    msg_2 = result[topic_2]["messages"][0]
    assert isinstance(msg_2, dict)
    assert msg_2["type"] == "image"
    assert msg_2["mimeType"] == "image/png"

    # Checking if base64 produces the correct image
    decoded_bytes = base64.b64decode(msg_2["data"])
    assert decoded_bytes.startswith(b"\x89PNG\r\n\x1a\n") 

def test_subscribe_multiple_topics_with_empty_config():
    rclpy.init()
    try:
        configs = [
            {"name": ""},
            {"name": None},
        ]

        ros = ExtendedROS2Manager()
        result = ros.subscribe_multiple_topics(configs)

        assert isinstance(result, dict)
        assert "<invalid_or_empty>" in result
        assert "error" in result["<invalid_or_empty>"]
        assert "missing" in result["<invalid_or_empty>"]["error"].lower() or \
               "empty" in result["<invalid_or_empty>"]["error"].lower()
    finally:
        rclpy.shutdown()

def test_subscribe_multiple_topics_with_nonexistent_topic():
    rclpy.init()
    try:
        configs = [
            {"name": "/nonexistent_topic", "message_limit": 1}
        ]

        ros = ExtendedROS2Manager()
        result = ros.subscribe_multiple_topics(configs)

        assert "/nonexistent_topic" in result
        assert "error" in result["/nonexistent_topic"]
        assert "not found" in result["/nonexistent_topic"]["error"].lower() or \
               "no types" in result["/nonexistent_topic"]["error"].lower()
    finally:
        rclpy.shutdown()

def test_subscribe_multiple_topics_missing_name_field():
    rclpy.init()
    try:
        configs = [
            {},
            {"name": ""}
        ]

        ros = ExtendedROS2Manager()
        result = ros.subscribe_multiple_topics(configs)

        assert "<invalid_or_empty>" in result
        assert "error" in result["<invalid_or_empty>"]
        assert "topic name" in result["<invalid_or_empty>"]["error"].lower()
    finally:
        rclpy.shutdown()

# publish_multiple_topics
FREQUENCY_PUBLISHERS = 20
PUBLISH_TIME = 1
NUMBER_OF_PUBLISHED_MESSAGES = FREQUENCY_PUBLISHERS * PUBLISH_TIME


class ThreadedSubscriber(Node):
    def __init__(self, topic_name: str, msg_type, message_limit: int = 1):
        super().__init__(f"test_subscriber_{topic_name.strip('/')}")
        self.received_messages = []
        self.message_limit = message_limit
        self._done = threading.Event()
        self.sub = self.create_subscription(msg_type, topic_name, self.callback, 10)

    def callback(self, msg):
        self.received_messages.append(msg)
        if len(self.received_messages) >= self.message_limit:
            self._done.set()

    def wait_for_messages(self, timeout: float = 5.0):
        self._done.wait(timeout)
        return self.received_messages

def test_publish_multiple_topics():
    rclpy.init()
    try:
        topic_1 = "/test_topic_publisher_1"
        topic_2 = "/test_topic_publisher_2"

        sub1 = ThreadedSubscriber(topic_1, Imu, NUMBER_OF_PUBLISHED_MESSAGES)
        sub2 = ThreadedSubscriber(topic_2, Imu, NUMBER_OF_PUBLISHED_MESSAGES)

        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(sub1)
        executor.add_node(sub2)

        def spin_until_done():
            while not sub1._done.is_set() or not sub2._done.is_set():
                executor.spin_once(timeout_sec=0.1)

        spin_thread = threading.Thread(target=spin_until_done, daemon=True)
        spin_thread.start()

        ros = ExtendedROS2Manager()
        configs = [
            {
                "topic_name": topic_1,
                "message_type": "sensor_msgs/msg/Imu",
                "data": {"linear_acceleration": {"x": 1.0}},
                "frequency": FREQUENCY,
                "duration": PUBLISH_TIME
            },
            {
                "topic_name": topic_2,
                "message_type": "sensor_msgs/msg/Imu",
                "data": {"linear_acceleration": {"x": 2.0}},
                "frequency": FREQUENCY,
                "duration": PUBLISH_TIME
            }
        ]

        result = ros.publish_multiple_topics(configs)

        spin_thread.join(timeout=5.0)

        msgs1 = sub1.received_messages
        msgs2 = sub2.received_messages

        assert len(msgs1) > 0, "No messages received on topic 1"
        assert len(msgs2) > 0, "No messages received on topic 2"
        assert abs(msgs1[0].linear_acceleration.x - 1.0) < 0.001
        assert abs(msgs2[0].linear_acceleration.x - 2.0) < 0.001
        assert topic_1 in result and result[topic_1] == "Publishing started"
        assert topic_2 in result and result[topic_2] == "Publishing started"
    finally:
        rclpy.shutdown()

def test_publish_multiple_topics_with_invalid_message_type():
    rclpy.init()
    try:
        topic = "/invalid_type_topic"
        ros = ExtendedROS2Manager()

        configs = [
            {
                "topic_name": topic,
                "message_type": "sensor_msgs/msg/NonExistent",  # invalid type
                "data": {"linear_acceleration": {"x": 1.0}},
                "frequency": 10,
                "duration": 1.0
            }
        ]

        result = ros.publish_multiple_topics(configs)
        assert topic in result
        assert "Invalid message type" in result[topic]
    finally:
        rclpy.shutdown()

def test_publish_multiple_topics_with_invalid_data():
    rclpy.init()
    try:
        topic = "/invalid_data_topic"
        ros = ExtendedROS2Manager()

        configs = [
            {
                "topic_name": topic,
                "message_type": "sensor_msgs/msg/Imu",
                "data": {"linear_acceleration": {"x": "not_a_number"}},  # invalid type
                "frequency": 10,
                "duration": 1.0
            }
        ]

        result = ros.publish_multiple_topics(configs)
        assert topic in result
        assert "Failed to publish" in result[topic]
    finally:
        rclpy.shutdown()

def test_publish_multiple_topics_with_missing_fields():
    rclpy.init()
    try:
        ros = ExtendedROS2Manager()

        configs = [
            {
                # Missing topic_name
                "message_type": "sensor_msgs/msg/Imu",
                "data": {"linear_acceleration": {"x": 1.0}},
            },
            {
                "topic_name": "/missing_data",
                "message_type": "sensor_msgs/msg/Imu",
                # Missing data
            },
            {
                "topic_name": "/missing_type",
                # Missing message_type
                "data": {"linear_acceleration": {"x": 1.0}},
            }
        ]

        result = ros.publish_multiple_topics(configs)

        assert "<unknown>" in result or "/missing_data" in result or "/missing_type" in result
        for topic, status in result.items():
            assert status == "Invalid config"
    finally:
        rclpy.shutdown()


# get_single_map_from_topic / get_single_bev_from_pointcloud
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2, PointField
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from PIL import Image as PILImage
import io


class ThreadedLatchedPublisher(Node):
    """Publishes one message repeatedly (transient-local, like map_server)."""

    def __init__(self, topic_name: str, msg_type, msg, rate_hz: float = 10.0):
        super().__init__("threaded_latched_publisher")
        qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(msg_type, topic_name, qos)
        self.msg = msg
        self.rate_hz = rate_hz
        self._running = False
        self._thread = None

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join()
        self.destroy_node()

    def _run_loop(self):
        while self._running and rclpy.ok():
            self.publisher.publish(self.msg)
            time.sleep(1.0 / self.rate_hz)


def _decode_png(image_content):
    assert image_content.type == "image"
    assert image_content.mimeType == "image/png"
    raw = base64.b64decode(image_content.data)
    assert raw.startswith(b"\x89PNG\r\n\x1a\n")
    return PILImage.open(io.BytesIO(raw))


def test_get_single_map_from_topic_renders_occupancy_grid():
    rclpy.init()
    pub = None
    try:
        grid = OccupancyGrid()
        grid.info.width = 3
        grid.info.height = 2
        grid.info.resolution = 0.05
        # row 0 (bottom in map frame): unknown, free, occupied
        # row 1 (top): free, free, occupied
        grid.data = [-1, 0, 100, 0, 0, 100]
        pub = ThreadedLatchedPublisher("/test_map", OccupancyGrid, grid)
        pub.start()
        time.sleep(0.5)

        ros = ExtendedROS2Manager()
        img = _decode_png(ros.get_single_map_from_topic("/test_map", timeout=5.0))

        assert img.mode == "L"
        assert img.size == (3, 2)
        pixels = list(img.getdata())
        # image is flipped vertically: first image row is map row 1
        assert pixels == [255, 255, 0, 205, 255, 0]
    finally:
        if pub:
            pub.stop()
        rclpy.shutdown()


def test_get_single_map_from_topic_missing_topic_raises():
    rclpy.init()
    try:
        ros = ExtendedROS2Manager()
        try:
            ros.get_single_map_from_topic("/no_such_map", timeout=0.5)
        except ValueError as e:
            assert "not found" in str(e)
        else:
            raise AssertionError("expected ValueError")
    finally:
        rclpy.shutdown()


def _make_pointcloud(points):
    msg = PointCloud2()
    msg.header.frame_id = "map"
    msg.height = 1
    msg.width = len(points)
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 16
    msg.row_step = 16 * len(points)
    msg.is_dense = True
    msg.data = np.array(points, dtype=np.float32).tobytes()
    return msg


def test_get_single_bev_from_pointcloud_renders_png():
    rclpy.init()
    pub = None
    try:
        pts = [[x * 0.1, y * 0.1, 0.5, float(x + y)] for x in range(20) for y in range(20)]
        pub = ThreadedLatchedPublisher("/test_points", PointCloud2, _make_pointcloud(pts))
        pub.start()
        time.sleep(0.5)

        ros = ExtendedROS2Manager()
        img = _decode_png(
            ros.get_single_bev_from_pointcloud(
                "/test_points", timeout=5.0, resolution=0.1, max_pixels=64
            )
        )
        assert img.mode == "RGB"
        w, h = img.size
        assert 1 <= w <= 64 and 1 <= h <= 64
        # at least some cells are coloured (not all black)
        assert any(p != (0, 0, 0) for p in img.getdata())
    finally:
        if pub:
            pub.stop()
        rclpy.shutdown()


def test_get_single_bev_from_pointcloud_z_filter_empty_returns_blank():
    rclpy.init()
    pub = None
    try:
        pts = [[0.0, 0.0, 0.5, 1.0], [1.0, 1.0, 0.5, 2.0]]
        pub = ThreadedLatchedPublisher("/test_points_z", PointCloud2, _make_pointcloud(pts))
        pub.start()
        time.sleep(0.5)

        ros = ExtendedROS2Manager()
        img = _decode_png(
            ros.get_single_bev_from_pointcloud(
                "/test_points_z", timeout=5.0, z_filter=(5.0, 6.0)
            )
        )
        assert img.size == (256, 256)
        assert set(img.getdata()) == {(0, 0, 0)}
    finally:
        if pub:
            pub.stop()
        rclpy.shutdown()


def test_get_single_bev_rejects_non_positive_max_pixels():
    rclpy.init()
    try:
        ros = ExtendedROS2Manager()
        try:
            ros.get_single_bev_from_pointcloud("/whatever", max_pixels=0)
        except ValueError as e:
            assert "max_pixels" in str(e)
        else:
            raise AssertionError("expected ValueError")
    finally:
        rclpy.shutdown()
