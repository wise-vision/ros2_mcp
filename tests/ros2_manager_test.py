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

from unittest.mock import MagicMock, patch
from server.ros2_manager import ROS2Manager
import rclpy

@patch("server.ros2_manager.ServiceNode")
def test_list_topics(mock_node_cls):
    mock_node = MagicMock()
    mock_node.get_topic_names_and_types.return_value = [
        ("/chatter", ["std_msgs/msg/String"]),
        ("/sensor", ["std_msgs/msg/Float32"])
    ]
    mock_node_cls.return_value = mock_node

    manager = ROS2Manager()
    result = manager.list_topics()

    assert result == [
        {
            "topic_name": "/chatter",
            "topic_type": "std_msgs/msg/String",
            "request_fields": {"data": "string"},
        },
        {
            "topic_name": "/sensor",
            "topic_type": "std_msgs/msg/Float32",
            "request_fields": {"data": "float"},
        },
    ]

@patch("server.ros2_manager.ServiceNode")
def test_list_topics_empty(mock_node_cls):
    mock_node = MagicMock()
    mock_node.get_topic_names_and_types.return_value = []
    mock_node_cls.return_value = mock_node

    manager = ROS2Manager()
    result = manager.list_topics()

    assert result == [] 

@patch("server.ros2_manager.ROS2Manager.get_request_fields")
@patch("server.ros2_manager.ServiceNode")
def test_list_services_with_services(mock_node_cls, mock_get_request_fields):
    mock_node = MagicMock()
    mock_node.get_service_names_and_types.return_value = [
        ("/add_two_ints", ["example_interfaces/srv/AddTwoInts"]),
        ("/reset_robot", ["my_msgs/srv/Reset"])
    ]
    mock_node_cls.return_value = mock_node

    mock_get_request_fields.side_effect = [
        {"a": "int64", "b": "int64"},
        {"confirm": "bool"}
    ]

    manager = ROS2Manager()
    result = manager.list_services()

    assert result == [
        {
            "service_name": "/add_two_ints",
            "service_type": "example_interfaces/srv/AddTwoInts",
            "request_fields": {"a": "int64", "b": "int64"}
        },
        {
            "service_name": "/reset_robot",
            "service_type": "my_msgs/srv/Reset",
            "request_fields": {"confirm": "bool"}
        }
    ]

@patch("server.ros2_manager.ServiceNode")
def test_list_services_empty(mock_node_cls):
    mock_node = MagicMock()
    mock_node.get_service_names_and_types.return_value = []
    mock_node_cls.return_value = mock_node

    manager = ROS2Manager()
    result = manager.list_services()

    assert result == []

from example_interfaces.srv import AddTwoInts

@patch("server.ros2_manager.ServiceNode") 
@patch("server.ros2_manager.rclpy.spin_until_future_complete")
def test_call_service_success_real_type(mock_spin, mock_node_cls):
    try:
        if not rclpy.ok():
            rclpy.init()

        mock_node = MagicMock()
        mock_node_cls.return_value = mock_node

        manager = ROS2Manager()

        mock_client = MagicMock()
        mock_client.wait_for_service.return_value = True
        mock_future = MagicMock()

        resp = AddTwoInts.Response()
        resp.sum = 8
        mock_future.result.return_value = resp
        mock_client.call_async.return_value = mock_future
        mock_node.create_client.return_value = mock_client

        result = manager.call_service(
            service_name="/add_two_ints",
            service_type="example_interfaces/srv/AddTwoInts",
            fields={"a": 5, "b": 3}
        )

        assert "result" in result
        mock_client.wait_for_service.assert_called_once()
        mock_client.call_async.assert_called_once()

    finally:
        if rclpy.ok():
            rclpy.shutdown()

# Test for serialize message
class FakeSlotMsg:
    __slots__ = ["x", "y"]
    def __init__(self):
        self.x = 10
        self.y = 20

def test_serialize_msg_slots():
    rclpy.init()
    try:
        msg = FakeSlotMsg()
        manager = ROS2Manager()
        result = manager.serialize_msg(msg)
        assert result == {"x": 10, "y": 20}
    finally:
        rclpy.shutdown()

class FakeDataMsg:
    def __init__(self):
        self.data = 42

def test_serialize_msg_data():
    rclpy.init()
    try:
        msg = FakeDataMsg()
        manager = ROS2Manager()
        result = manager.serialize_msg(msg)
        print(result)
        assert result == 42
    finally:
        rclpy.shutdown()

class WeirdMsg:
    def __str__(self):
        return "<weird>"

def test_serialize_msg_fallback():
    rclpy.init()
    try:
        msg = WeirdMsg()
        manager = ROS2Manager()
        result = manager.serialize_msg(msg)
        assert result == "<weird>"
    finally:
        rclpy.shutdown()

# Test for subscribe topic
@patch("server.ros2_manager.ServiceNode")
def test_subscribe_topic_topic_not_found(mock_node_cls):
    mock_node = MagicMock()
    mock_node.get_topic_names_and_types.return_value = [("/chatter", ["std_msgs/msg/String"])]
    mock_node_cls.return_value = mock_node

    manager = ROS2Manager()
    manager.node = mock_node

    result = manager.subscribe_topic("/not_here", "std_msgs/msg/String")
    assert "error" in result
    assert "not found" in result["error"]

@patch("server.ros2_manager.importlib.import_module", side_effect=ImportError("Boom"))
@patch("server.ros2_manager.ServiceNode")
def test_subscribe_topic_import_fail(mock_node_cls, mock_import):
    mock_node = MagicMock()
    mock_node.get_topic_names_and_types.return_value = [("/chatter", ["std_msgs/msg/String"])]
    mock_node_cls.return_value = mock_node

    manager = ROS2Manager()
    manager.node = mock_node

    result = manager.subscribe_topic("/chatter", "std_msgs/msg/String")
    assert "error" in result
    assert "Failed to import" in result["error"]

@patch("server.ros2_manager.get_service")
@patch("server.ros2_manager.get_message")
@patch("server.ros2_manager.deserialize_message")
def test_call_get_messages_service_any_success(
    mock_deserialize, mock_get_msg, mock_get_srv
):
    try:
        rclpy.init()

        mock_request = MagicMock()
        mock_service = MagicMock()
        mock_service.Request.return_value = mock_request
        mock_get_srv.return_value = mock_service

        full_datetime_cls = MagicMock()
        sensor_msg_cls = MagicMock()
        mock_get_msg.side_effect = [full_datetime_cls, sensor_msg_cls]

        fake_msg = MagicMock()
        fake_msg.get_fields_and_field_types.return_value = {"temp": "float"}
        setattr(fake_msg, "temp", 22.5)
        mock_deserialize.return_value = fake_msg

        payload = b"\x01"
        length_bytes = len(payload).to_bytes(4, byteorder="big")
        fake_response = MagicMock()
        fake_response.data = length_bytes + payload

        fake_ts = MagicMock()
        fake_ts.get_fields_and_field_types.return_value = {"sec": "int32"}
        setattr(fake_ts, "sec", 12345)
        fake_response.timestamps = [fake_ts]

        mock_client = MagicMock()
        mock_client.wait_for_service.return_value = True
        mock_client.call.return_value = fake_response

        manager = ROS2Manager()
        manager.node.create_client = MagicMock(return_value=mock_client)

        result = manager.call_get_messages_service_any(
            {
                "topic_name": "sensor_data",
                "message_type": "my_msgs/msg/SensorData",
                "number_of_msgs": 1,
            }
        )

        assert "messages" in result
        assert "timestamps" in result
        assert isinstance(result["messages"], list)
        assert isinstance(result["timestamps"], list)
        assert result["messages"][0] == {"temp": 22.5}
        assert result["timestamps"][0]["sec"] == 12345

    finally:
        rclpy.shutdown()

@patch("server.ros2_manager.ServiceNode")
def test_publish_to_topic_string_no_type_mocks(mock_node_cls):
    from std_msgs.msg import String

    try:
        if not rclpy.ok():
            rclpy.init()

        mock_node = MagicMock()
        mock_node_cls.return_value = mock_node

        manager = ROS2Manager()

        mock_publisher = MagicMock()
        mock_node.create_publisher.return_value = mock_publisher

        result = manager.publish_to_topic("/chatter", "std_msgs/msg/String", {"data": "Hello"})

        assert result["status"] == "published"
        mock_node.create_publisher.assert_called_once()
        mock_publisher.publish.assert_called_once()

        published_msg = mock_publisher.publish.call_args[0][0]
        assert isinstance(published_msg, String)
        assert published_msg.data == "Hello"
    finally:
        if rclpy.ok():
            rclpy.shutdown()

@patch("server.ros2_manager.ServiceNode")
def test_publish_to_topic_invalid_topic_name(mock_node_cls):
    manager = ROS2Manager()
    result = manager.publish_to_topic("", "std_msgs/msg/String", {"data": "Hello"})
    assert "error" in result
    assert result["error"] == "Invalid topic name. It must be a non-empty string."

@patch("server.ros2_manager.ServiceNode")
def test_publish_to_topic_invalid_message_type(mock_node_cls):
    manager = ROS2Manager()
    result = manager.publish_to_topic("/chatter", "invalidtype", {"data": "Hello"})
    assert "error" in result
    assert result["error"] == "Invalid message type. It must be a valid ROS2 message type string."

@patch("server.ros2_manager.ServiceNode")
def test_publish_to_topic_invalid_data(mock_node_cls):
    manager = ROS2Manager()
    result = manager.publish_to_topic("/chatter", "std_msgs/msg/String", "invalid_data")
    assert "error" in result
    assert result["error"] == "Invalid data. It must be a dictionary."


@patch("server.ros2_manager.ServiceNode")
@patch("server.ros2_manager.rclpy.spin_until_future_complete")
def test_mavros_waypoint_push_int_to_float_no_type_mocks(mock_spin, mock_node_cls):
    from mavros_msgs.srv import WaypointPush
    from mavros_msgs.msg import Waypoint

    try:
        if not rclpy.ok():
            rclpy.init()

        mock_node = MagicMock()
        mock_node_cls.return_value = mock_node

        manager = ROS2Manager()

        mock_client = MagicMock()
        mock_client.wait_for_service.return_value = True
        mock_future = MagicMock()

        resp = WaypointPush.Response()
        resp.success = True
        resp.wp_transfered = 3
        mock_future.result.return_value = resp
        mock_client.call_async.return_value = mock_future
        mock_node.create_client.return_value = mock_client

        fields = {
            "start_index": 0,
            "waypoints": [
                {
                    "frame": 3, "command": 22, "is_current": True, "autocontinue": True,
                    "param1": 0, "param2": 0, "param3": 0, "param4": 0,
                    "x_lat": 0, "y_long": 0, "z_alt": 7
                },
                {
                    "frame": 3, "command": 16, "is_current": False, "autocontinue": True,
                    "param1": 0, "param2": 0, "param3": 0, "param4": 0,
                    "x_lat": 52.43304071995481, "y_long": 20.72276917967427, "z_alt": 7
                },
                {
                    "frame": 3, "command": 20, "is_current": False, "autocontinue": True,
                    "param1": 0, "param2": 0, "param3": 0, "param4": 0,
                    "x_lat": 0, "y_long": 0, "z_alt": 0
                }
            ]
        }

        result = manager.call_service(
            service_name="/mavros/mission/push",
            service_type="mavros_msgs/srv/WaypointPush",
            fields=fields
        )

        assert "result" in result
        mock_client.wait_for_service.assert_called_once()
        mock_client.call_async.assert_called_once()

        sent_req = mock_client.call_async.call_args[0][0]
        assert isinstance(sent_req, WaypointPush.Request)
        assert isinstance(sent_req.start_index, int) and sent_req.start_index == 0
        assert isinstance(sent_req.waypoints, list) and len(sent_req.waypoints) == 3
        assert all(isinstance(w, Waypoint) for w in sent_req.waypoints)

        wp0 = sent_req.waypoints[0]
        assert isinstance(wp0.x_lat, float) and wp0.x_lat == 0.0
        assert isinstance(wp0.y_long, float) and wp0.y_long == 0.0
        assert isinstance(wp0.z_alt, float) and wp0.z_alt == 7.0
        assert isinstance(wp0.param1, float) and wp0.param1 == 0.0

        wp1 = sent_req.waypoints[1]
        assert isinstance(wp1.z_alt, float) and wp1.z_alt == 7.0


    finally:
        if rclpy.ok():
            rclpy.shutdown()

import types
from action_tutorials_interfaces.action import Fibonacci

ACTION_TYPE = "action_tutorials_interfaces/action/Fibonacci"
ACTION_NAME = "/fibonacci"


@patch("server.ros2_manager.ServiceNode")
@patch("server.ros2_manager.rclpy.spin_until_future_complete")
@patch("server.ros2_manager.ActionClient")
@patch("server.ros2_manager.importlib.import_module", return_value=types.SimpleNamespace(Fibonacci=Fibonacci))
def test_send_action_goal_no_wait_not_accepted(mock_import, mock_action_client, mock_spin, mock_node_cls):
    try:
        if not rclpy.ok():
            rclpy.init()

        mock_node = MagicMock()
        mock_node_cls.return_value = mock_node

        mgr = ROS2Manager()

        mock_client = MagicMock()
        mock_client.wait_for_server.return_value = True
        mock_action_client.return_value = mock_client

        goal_handle = MagicMock()
        goal_handle.accepted = False
        goal_handle.goal_id = types.SimpleNamespace(uuid=b"\x01"*16)

        send_future = MagicMock()
        send_future.result.return_value = goal_handle
        mock_client.send_goal_async.return_value = send_future

        resp = mgr.send_action_goal(
            action_name=ACTION_NAME,
            action_type=ACTION_TYPE,
            goal_fields={"order": 5},
            wait_for_result=False,
        )

        assert resp["accepted"] is False
        assert resp["status"] == "NOT_ACCEPTED"
        assert resp["result"] is None

    finally:
        if rclpy.ok():
            rclpy.shutdown()

@patch("server.ros2_manager.ServiceNode")
@patch("server.ros2_manager.rclpy.spin_until_future_complete")
@patch("server.ros2_manager.ActionClient")
@patch("server.ros2_manager.importlib.import_module", return_value=types.SimpleNamespace(Fibonacci=Fibonacci))
def test_send_action_goal_no_wait_accepted(mock_import, mock_action_client, mock_spin, mock_node_cls):
    try:
        if not rclpy.ok():
            rclpy.init()

        mock_node = MagicMock()
        mock_node_cls.return_value = mock_node

        mgr = ROS2Manager()

        mock_client = MagicMock()
        mock_client.wait_for_server.return_value = True
        mock_action_client.return_value = mock_client

        goal_handle = MagicMock()
        goal_handle.accepted = True
        goal_handle.goal_id = types.SimpleNamespace(uuid=b"\x02"*16)
        send_future = MagicMock()
        send_future.result.return_value = goal_handle
        mock_client.send_goal_async.return_value = send_future

        resp = mgr.send_action_goal(
            action_name=ACTION_NAME,
            action_type=ACTION_TYPE,
            goal_fields={"order": 5},
            wait_for_result=False,
        )

        assert resp["accepted"] is True
        assert resp["status"] == "ACCEPTED"
        assert resp["result"] is None

    finally:
        if rclpy.ok():
            rclpy.shutdown()


from action_tutorials_interfaces.action._fibonacci import Fibonacci_GetResult
from action_tutorials_interfaces import action as fib_action
@patch("server.ros2_manager.ServiceNode")
@patch("server.ros2_manager.rclpy.spin_until_future_complete")
@patch("server.ros2_manager.ActionClient")
@patch("server.ros2_manager.importlib.import_module", return_value=fib_action)
def test_send_action_goal_wait_for_result_succeeded(
    mock_import, mock_action_client, mock_spin, mock_node_cls
):
    try:
        if not rclpy.ok():
            rclpy.init()

        mock_node = MagicMock()
        mock_node_cls.return_value = mock_node

        mgr = ROS2Manager()

        mock_client = MagicMock()
        mock_client.wait_for_server.return_value = True
        mock_action_client.return_value = mock_client

        result_future = MagicMock()
        result_future.done.return_value = True

        resp_msg = Fibonacci_GetResult.Response()
        resp_msg.status = 4
        resp_msg.result = Fibonacci.Result(sequence=[0, 1, 1, 2, 3, 5])
        result_future.result.return_value = resp_msg

        goal_handle = MagicMock()
        goal_handle.accepted = True
        goal_handle.goal_id = types.SimpleNamespace(uuid=b"\x03" * 16)
        goal_handle.get_result_async.return_value = result_future

        send_future = MagicMock()
        send_future.result.return_value = goal_handle
        mock_client.send_goal_async.return_value = send_future

        resp = mgr.send_action_goal(
            action_name="/fibonacci",
            action_type="action_tutorials_interfaces/action/Fibonacci",
            goal_fields={"order": 5},
            wait_for_result=True,
            timeout_sec=5.0,
        )
        assert resp["accepted"] is True
        assert resp["status"] == "SUCCEEDED"
        assert resp["result"] is not None
        assert resp["result"]["sequence"][-1] == 5

    finally:
        if rclpy.ok():
            rclpy.shutdown()

@patch("server.ros2_manager.get_action_names_and_types")
@patch.object(ROS2Manager, "get_request_fields")
def test_list_actions_returns_actions_with_request_fields(mock_get_fields, mock_get_names):
    try:
        if not rclpy.ok():
            rclpy.init()
        mock_get_names.return_value = [
            ("/fibonacci", ["action_tutorials_interfaces/action/Fibonacci"]),
            ("/navigate_to_pose", ["nav2_msgs/action/NavigateToPose"]),
        ]

        def fake_get_fields(ros_type: str):
            if ros_type == "action_tutorials_interfaces/action/Fibonacci":
                return {"order": "int32"}
            elif ros_type == "nav2_msgs/action/NavigateToPose":
                return {"pose": "geometry_msgs/PoseStamped"}
            else:
                return {"error": "unknown type"}

        mock_get_fields.side_effect = fake_get_fields

        mgr = ROS2Manager()
        mgr.node = MagicMock()

        resp = mgr.list_actions()

        assert "actions" in resp
        actions = resp["actions"]
        assert any(a["name"] == "/fibonacci" and a["request_fields"]["order"] == "int32" for a in actions)
        assert any(a["name"] == "/navigate_to_pose" and "pose" in a["request_fields"] for a in actions)
    finally:
        if rclpy.ok():
            rclpy.shutdown()


@patch("rclpy.spin_until_future_complete")
@patch("server.ros2_manager.message_to_ordereddict")
@patch("importlib.import_module")
@patch("server.ros2_manager.GOAL_CANCEL_RET", new={0: "OK"})
def test_cancel_action_goal_cancel_all_success(
    mock_import_module, mock_msg_to_dict, mock_spin
):
    try:
        rclpy.init()

        class _Stamp:
            def __init__(self):
                self.sec = 0
                self.nanosec = 0

        class _GoalID:
            def __init__(self):
                self.uuid = b""

        class _GoalInfo:
            def __init__(self):
                self.goal_id = _GoalID()
                self.stamp = _Stamp()

        class _Request:
            def __init__(self):
                self.goal_info = _GoalInfo()

        CancelGoal = MagicMock()
        CancelGoal.Request = MagicMock(return_value=_Request())

        fake_srv_module = MagicMock()
        setattr(fake_srv_module, "CancelGoal", CancelGoal)
        mock_import_module.return_value = fake_srv_module

        mock_client = MagicMock()
        mock_client.wait_for_service.return_value = True

        fake_response = MagicMock()
        fake_response.return_code = 0

        mock_msg_to_dict.return_value = {
            "goals_canceling": [
                {
                    "goal_id": {"uuid": [0x12, 0x34, 0x56, 0x78] + [0] * 12},
                    "stamp": {"sec": 111, "nanosec": 222},
                }
            ]
        }

        fake_future = MagicMock()
        fake_future.result.return_value = fake_response
        mock_client.call_async.return_value = fake_future

        manager = ROS2Manager()
        node = MagicMock()
        node.create_client.return_value = mock_client

        result = manager.cancel_action_goal(
            node=node,
            action_name="/my_action",
            cancel_all=True,
            stamp_sec=111,
            stamp_nanosec=222,
            wait_timeout_sec=1.5,
        )

        assert "error" not in result
        assert result["service"] == "/my_action/_action/cancel_goal"
        assert result["return_code"] == 0
        assert result["return_code_text"] == "OK"
        assert isinstance(result["goals_canceling"], list)
        assert len(result["goals_canceling"]) == 1

        item = result["goals_canceling"][0]
        assert item["goal_id"].startswith("12345678")
        assert item["stamp"] == {"sec": 111, "nanosec": 222}

        sent_req = CancelGoal.Request.return_value
        uuid_field = getattr(sent_req.goal_info.goal_id, "uuid")
        if isinstance(uuid_field, (bytes, bytearray)):
            assert uuid_field == bytes([0] * 16)
        else:
            assert list(uuid_field) == [0] * 16

        mock_client.wait_for_service.assert_called_once()
        mock_client.call_async.assert_called_once()
        mock_spin.assert_called_once()

    finally:
        rclpy.shutdown()

@patch("rclpy.spin_until_future_complete")
@patch("server.ros2_manager.message_to_ordereddict")
@patch("importlib.import_module")
def test_action_request_result_success(mock_import_module, mock_msg_to_dict, mock_spin):
    try:
        rclpy.init()

        class _GetResultReq:
            def __init__(self):
                class GID:
                    def __init__(self):
                        self.uuid = None
                self.goal_id = GID()

        class _GetResultResp:
            def __init__(self):
                self.status = 4
                self.result = MagicMock()

        # >>> kluczowa zmiana: ActionCls ma atrybut GetResult z .Request
        class _GetResultType:
            Request = MagicMock(return_value=_GetResultReq())

        class _ActionCls:
            GetResult = _GetResultType

        fake_pkg_mod = MagicMock()
        setattr(fake_pkg_mod, "Fibonacci", _ActionCls)
        mock_import_module.return_value = fake_pkg_mod

        mock_msg_to_dict.return_value = {"sequence": [1, 1, 2, 3, 5]}

        mock_client = MagicMock()
        mock_client.wait_for_service.return_value = True

        fake_future = MagicMock()
        fake_future.done.return_value = True
        fake_future.result.return_value = _GetResultResp()
        mock_client.call_async.return_value = fake_future

        mgr = ROS2Manager()
        mgr.node.create_client = MagicMock(return_value=mock_client)
        # lepiej zwracać bytes (tak jak realne pole UUID oczekuje)
        mgr._hex_to_uuid_bytes = lambda h: bytes.fromhex(h)

        goal_id_hex = "0123456789abcdef0123456789abcdef"

        out = mgr.action_request_result(
            action_name="/fibonacci",
            action_type="example_interfaces/action/Fibonacci",
            goal_id_hex=goal_id_hex,
            timeout_sec=2.5,
        )

        assert "error" not in out
        assert out["service"] == "/fibonacci/_action/get_result"
        assert out["goal_id"] == goal_id_hex
        assert out["waited"] is True
        assert out["result_timeout_sec"] == 2.5
        assert isinstance(out["status_code"], int)
        assert out["status"] is not None
        assert out["result"] == {"sequence": [1, 1, 2, 3, 5]}

        sent_req = _GetResultType.Request.return_value
        assert isinstance(sent_req.goal_id.uuid, (bytes, bytearray))
        assert len(sent_req.goal_id.uuid) == 16

        mock_client.wait_for_service.assert_called_once()
        mock_client.call_async.assert_called_once()
        mock_spin.assert_called_once()

    finally:
        rclpy.shutdown()


@patch("rclpy.spin_until_future_complete")
@patch("importlib.import_module")
def test_action_request_result_timeout(mock_import_module, mock_spin):
    try:
        rclpy.init()

        class _GetResultReq:
            def __init__(self):
                class GID:
                    def __init__(self):
                        self.uuid = None
                self.goal_id = GID()

        # >>> tak samo jak wyżej: udajemy <Action>.GetResult
        class _GetResultType:
            Request = MagicMock(return_value=_GetResultReq())

        class _ActionCls:
            GetResult = _GetResultType

        fake_pkg_mod = MagicMock()
        setattr(fake_pkg_mod, "Fibonacci", _ActionCls)
        mock_import_module.return_value = fake_pkg_mod

        mock_client = MagicMock()
        mock_client.wait_for_service.return_value = True

        # future, który NIE kończy się w limicie
        fake_future = MagicMock()
        fake_future.done.return_value = False
        mock_client.call_async.return_value = fake_future

        mgr = ROS2Manager()
        mgr.node.create_client = MagicMock(return_value=mock_client)
        mgr._hex_to_uuid_bytes = lambda h: bytes.fromhex(h)

        goal_id_hex = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"

        out = mgr.action_request_result(
            action_name="/fibonacci",
            action_type="example_interfaces/action/Fibonacci",
            goal_id_hex=goal_id_hex,
            timeout_sec=0.1,
        )

        assert "error" not in out
        assert out["status"] == "TIMEOUT"
        assert out["status_code"] == 0
        assert out["result"] is None
        assert out["result_timeout_sec"] == 0.1

        mock_client.wait_for_service.assert_called_once()
        mock_client.call_async.assert_called_once()
        mock_spin.assert_called_once()

    finally:
        rclpy.shutdown()

class _FakeNow:
    def __init__(self):
        self.nanoseconds = 0

    def to_msg(self):
        class _Stamp:
            pass
        s = _Stamp()
        s.sec = int(self.nanoseconds // 1_000_000_000)
        s.nanosec = int(self.nanoseconds % 1_000_000_000)
        return s


class _FakeClock:
    def __init__(self, now):
        self._now = now

    def now(self):
        return self._now

@patch("server.ros2_manager.message_to_ordereddict")
@patch("importlib.import_module")
@patch("rclpy.spin_once")
def test_action_subscribe_feedback_filters_and_collects(mock_spin_once, mock_import_module, mock_msg_to_dict):
    try:
        rclpy.init()

        class _FeedbackMsg:
            def __init__(self):
                self.goal_id = types.SimpleNamespace(uuid=None)
            feedback = MagicMock()

        class _Impl:
            FeedbackMessage = _FeedbackMsg

        class _ActionCls:
            Impl = _Impl

        fake_pkg_mod = MagicMock()
        setattr(fake_pkg_mod, "Fibonacci", _ActionCls)
        mock_import_module.return_value = fake_pkg_mod

        mock_msg_to_dict.return_value = {"partial_sequence": [1, 1, 2, 3]}

        mgr = ROS2Manager()
        node = MagicMock()
        now = _FakeNow()
        node.get_clock.return_value = _FakeClock(now)
        callbacks = []

        def _create_subscription(_msg_type, _topic, cb, _qos):
            callbacks.append(cb)
            return MagicMock()

        node.create_subscription.side_effect = _create_subscription
        node.destroy_subscription = MagicMock()
        mgr.node = node

        goal_id_hex = "00112233445566778899aabbccddeeff"
        uuid_bytes = bytes.fromhex(goal_id_hex)

        def _spin_once_effect(_node, timeout_sec=0.1):
            if callbacks:
                cb = callbacks[0]
                msg = _FeedbackMsg()
                msg.goal_id.uuid = list(uuid_bytes)
                cb(msg)
                callbacks.clear()
            now.nanoseconds += 10_000_000_000

        mock_spin_once.side_effect = _spin_once_effect

        out = mgr.action_subscribe_feedback(
            action_name="/fibonacci",
            action_type="example_interfaces/action/Fibonacci",
            goal_id_hex=goal_id_hex,
            duration_sec=0.01,
            max_messages=10,
        )

        assert "error" not in out
        assert out["topic"] == "/fibonacci/_action/feedback"
        assert out["goal_id_filter"] == goal_id_hex
        assert isinstance(out["messages"], list)
        assert len(out["messages"]) == 1

        item = out["messages"][0]
        assert item["goal_id"] == goal_id_hex
        assert item["feedback"] == {"partial_sequence": [1, 1, 2, 3]}
        assert "recv_stamp" in item and "sec" in item["recv_stamp"]

        node.create_subscription.assert_called_once()
    finally:
        rclpy.shutdown()

@patch("rclpy.spin_once")
def test_action_subscribe_status_collects_frames(mock_spin_once, monkeypatch):
    try:
        rclpy.init()

        class _GoalInfo:
            def __init__(self, uuid_bytes, sec, nsec):
                self.goal_id = types.SimpleNamespace(uuid=list(uuid_bytes))
                self.stamp = types.SimpleNamespace(sec=sec, nanosec=nsec)

        class _GoalStatus:
            def __init__(self, uuid_bytes, status, sec=1, nsec=2):
                self.goal_info = _GoalInfo(uuid_bytes, sec, nsec)
                self.status = status

        class _GoalStatusArray:
            def __init__(self, statuses):
                self.status_list = statuses

        fake_action_msgs = types.ModuleType("action_msgs")
        fake_action_msgs_msg = types.ModuleType("action_msgs.msg")
        fake_action_msgs_msg.GoalStatusArray = _GoalStatusArray
        monkeypatch.setitem(__import__("sys").modules, "action_msgs", fake_action_msgs)
        monkeypatch.setitem(__import__("sys").modules, "action_msgs.msg", fake_action_msgs_msg)

        mgr = ROS2Manager()
        node = MagicMock()
        now = _FakeNow()
        node.get_clock.return_value = _FakeClock(now)
        callbacks = []

        def _create_subscription(_msg_type, _topic, cb, _qos):
            callbacks.append(cb)
            return MagicMock()

        node.create_subscription.side_effect = _create_subscription
        node.destroy_subscription = MagicMock()
        mgr.node = node

        uuid_hex = "deadbeefcafebabe0011223344556677"
        uuid_bytes = bytes.fromhex(uuid_hex)

        def _spin_once_effect(_node, timeout_sec=0.1):
            if callbacks:
                cb = callbacks[0]
                msg = _GoalStatusArray([_GoalStatus(uuid_bytes, 2, sec=5, nsec=6)])
                cb(msg)
                callbacks.clear()
            now.nanoseconds += 10_000_000_000

        mock_spin_once.side_effect = _spin_once_effect

        out = mgr.action_subscribe_status(
            action_name="/fibonacci",
            duration_sec=0.01,
            max_messages=10,
        )

        assert "error" not in out
        assert out["topic"] == "/fibonacci/_action/status"
        assert isinstance(out["frames"], list)
        assert len(out["frames"]) == 1

        frame = out["frames"][0]
        assert isinstance(frame["statuses"], list) and len(frame["statuses"]) == 1
        st = frame["statuses"][0]
        assert st["goal_id"] == uuid_hex
        assert st["accept_stamp"] == {"sec": 5, "nanosec": 6}
        assert isinstance(st["status_code"], int)
        assert st["status"] is not None

        node.create_subscription.assert_called_once()
    finally:
        rclpy.shutdown()

from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
)
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String
import time

def test_get_qos_for_publisher_topic_superset_real_subs():
    rclpy.init()
    try:
        topic = "/qos_integration_test"

        sub_a_node = rclpy.create_node("sub_a_node")
        sub_b_node = rclpy.create_node("sub_b_node")

        qos_a = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        qos_b = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=3,
        )

        sub_a = sub_a_node.create_subscription(String, topic, lambda m: None, qos_a)
        sub_b = sub_b_node.create_subscription(String, topic, lambda m: None, qos_b)

        inspector_node = rclpy.create_node("inspector_node")

        from server.ros2_manager import ROS2Manager
        manager = ROS2Manager()
        manager.node = inspector_node

        # Discovery
        exec = SingleThreadedExecutor()
        exec.add_node(sub_a_node)
        exec.add_node(sub_b_node)
        exec.add_node(inspector_node)

        for _ in range(10):
            exec.spin_once(timeout_sec=0.05)
            time.sleep(0.01)

        qos = manager.get_qos_for_publisher_topic(inspector_node, topic)

        assert qos.reliability == QoSReliabilityPolicy.RELIABLE
        assert qos.durability == QoSDurabilityPolicy.TRANSIENT_LOCAL
        assert qos.history == QoSHistoryPolicy.KEEP_LAST
        if qos.depth != 0:
            assert qos.depth >= 10

    finally:
        try:
            sub_a_node.destroy_subscription(sub_a)
            sub_b_node.destroy_subscription(sub_b)
        except Exception:
            pass
        try:
            sub_a_node.destroy_node()
            sub_b_node.destroy_node()
        except Exception:
            pass
        try:
            inspector_node.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()



from unittest.mock import patch, MagicMock

@patch("server.ros2_manager.ServiceNode")
def test_publish_to_topic_uses_selected_qos(mock_node_cls):
    from std_msgs.msg import String
    from server.ros2_manager import ROS2Manager

    rclpy.init()
    chosen_qos = QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=5,
    )

    mock_node = MagicMock()
    mock_publisher = MagicMock()
    mock_node.create_publisher.return_value = mock_publisher
    mock_node_cls.return_value = mock_node

    manager = ROS2Manager()
    manager.get_qos_for_publisher_topic = MagicMock(return_value=chosen_qos)

    result = manager.publish_to_topic("/chatter", "std_msgs/msg/String", {"data": "hello"})

    assert result["status"] == "published"
    args, kwargs = mock_node.create_publisher.call_args
    assert args[0] is String
    assert args[1] == "/chatter"
    passed_qos = args[2]
    assert isinstance(passed_qos, QoSProfile)
    assert passed_qos.reliability == QoSReliabilityPolicy.RELIABLE
    assert passed_qos.durability == QoSDurabilityPolicy.TRANSIENT_LOCAL
    assert passed_qos.history == QoSHistoryPolicy.KEEP_LAST
    assert passed_qos.depth == 5
