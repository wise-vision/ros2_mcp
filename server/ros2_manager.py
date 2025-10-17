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
from typing import Any, Optional
from rclpy.serialization import deserialize_message
from rosidl_runtime_py import get_interfaces
from rosidl_runtime_py.utilities import get_service, get_message
from rosidl_runtime_py.set_message import set_message_fields
from rosidl_runtime_py import message_to_ordereddict

from rclpy.action import get_action_names_and_types
from dateutil import parser
import numpy as np
import array
import time
from rclpy.task import Future
from builtin_interfaces.msg import Time, Duration
from std_msgs.msg import Header
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSPresetProfiles
from rclpy.qos import QoSReliabilityPolicy
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

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


class ServiceNode(Node):
    def __init__(self):
        super().__init__("mcp_service_lister")


class ROS2Manager:
    def __init__(self):
        self.node = ServiceNode()

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
            rclpy.spin_until_future_complete(self.node, future)
            if future.result() is not None:
                return {"result": str(future.result())}
            else:
                return {"error": "Service call failed."}
        except Exception as e:
            return {"error": str(e)}

    def serialize_msg(self, msg: Any) -> Any:
        try:
            if isinstance(msg, memoryview):
                try:
                    return list(msg.cast("d"))
                except TypeError:
                    return list(msg)

            elif isinstance(msg, (bytes, bytearray)):
                return list(msg)

            elif isinstance(msg, (int, float, str, bool)) or msg is None:
                return msg

            elif hasattr(msg, "data"):
                return self.serialize_msg(msg.data)

            elif isinstance(msg, (list, tuple)):
                return [self.serialize_msg(item) for item in msg]

            elif hasattr(msg, "__slots__"):
                return {
                    slot: self.serialize_msg(getattr(msg, slot))
                    for slot in msg.__slots__
                }

            elif isinstance(msg, dict):
                return {key: self.serialize_msg(value) for key, value in msg.items()}

            else:
                return str(msg)

        except Exception as e:
            return {"error": f"Failed to serialize message: {str(e)}"}

    def subscribe_topic(
        self,
        topic_name: str,
        duration: Optional[float] = None,
        message_limit: Optional[int] = None,
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
        try:
            qos = self.get_qos_profile_for_topic(tmp_node, topic_name)
        finally:
            tmp_node.destroy_node()

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

    def publish_to_topic(self, topic_name: str, message_type: str, data: dict) -> dict:
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
        
        tmp_node = Node("mcp_publish_tmp")
        try:
            qos = self.get_qos_for_publisher_topic(tmp_node, topic_name)
        finally:
            tmp_node.destroy_node()

        try:
            pub = self.node.create_publisher(msg_class, topic_name, qos)
            msg_instance = msg_class()
            set_message_fields(msg_instance, data)
            pub.publish(msg_instance)

            return {"status": "published", "data": data}
        except Exception as e:
            return {"error": "Failed to publish message due to an internal error."}

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
            rclpy.spin_until_future_complete(self.node, send_future)

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
            rclpy.spin_until_future_complete(
                self.node, result_future, timeout_sec=timeout_sec
            )

            if not result_future.done():
                try:
                    cancel_future = goal_handle.cancel_goal_async()
                    rclpy.spin_until_future_complete(
                        self.node, cancel_future, timeout_sec=3.0
                    )
                except Exception:
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
            rclpy.spin_until_future_complete(node, future, timeout_sec=wait_timeout_sec)
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
            if timeout_sec is None:
                rclpy.spin_until_future_complete(self.node, future)
            else:
                rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout_sec)

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
                rclpy.spin_once(self.node, timeout_sec=0.1)

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
                    out["frames"].append(frame)
                except Exception as e:
                    out.setdefault("warn", []).append(str(e))

            if not hasattr(self, "_tmp_subs"):
                self._tmp_subs = []
            sub = self.node.create_subscription(GoalStatusArray, topic, _cb, qos)
            self._tmp_subs.append(sub)

            end_time = self.node.get_clock().now().nanoseconds + int(duration_sec * 1e9)

            def _count_statuses():
                return sum(len(f.get("statuses", [])) for f in out["frames"])

            while self.node.get_clock().now().nanoseconds < end_time:
                if _count_statuses() >= max_messages:
                    break
                rclpy.spin_once(self.node, timeout_sec=0.1)

            # Clean up subscription
            self.node.destroy_subscription(sub)
            if hasattr(self, "_tmp_subs"):
                try:
                    self._tmp_subs.remove(sub)
                except ValueError:
                    pass

            return out

        except Exception as e:
            return {"error": str(e)}