#
#  Copyright (C) 2025 wisevision
#
#  SPDX-License-Identifier: MPL-2.0
#
#  This Source Code Form is subject to the terms of the Mozilla Public
#  License, v. 2.0. If a copy of the MPL was not distributed with this
#  file, You can obtain one at https://mozilla.org/MPL/2.0/.
#
"""Safety classification of every MCP tool this server registers.

A tool is *mutating* when calling it can change robot state or the ROS graph
in a way another node can observe: publishing on a topic, calling a service
(the server cannot know whether an arbitrary service has side effects),
sending or cancelling an action goal.

In read-only mode (``ROS2_MCP_READONLY=1`` or ``--read-only``) the mutating
tools are not registered at all, so ``list_tools`` does not show them and
calling one returns an unknown-tool error.

Every registered tool must appear in exactly one of the two sets below;
``tests/tool_safety_test.py`` fails when a new tool is added without being
reviewed here.
"""

# Tools that publish, call services, or send/cancel action goals.
MUTATING_TOOLS: frozenset[str] = frozenset(
    {
        # Publishes an arbitrary message on an arbitrary topic.
        "ros2_topic_publish",
        # Publishes to several topics at once, repeatedly (frequency x duration).
        "ros2_publish_multiple_topics",
        # Calls an arbitrary service; many services change state (set_bool, reset, arm, ...).
        "ros2_service_call",
        # Sends an action goal (navigate, move arm, take off, ...).
        "ros2_send_action_goal",
        # Cancels running goals; stopping a robot mid-task is a state change.
        "ros2_cancel_action_goal",
    }
)

# Tools reviewed as read-only: they list the graph, subscribe, or read
# stored data; none of them publishes or sends a goal.
READ_ONLY_TOOLS: frozenset[str] = frozenset(
    {
        "ros2_topic_list",
        "ros2_service_list",
        "ros2_interface_list",
        "ros2_topic_subscribe",
        "ros2_subscribe_multiple_topics",
        "ros2_get_message_fields",
        # Calls only the WiseVision Data Black Box query service (/get_messages),
        # which reads stored messages; it never calls a user-chosen service.
        "ros2_get_messages_stored_in_influx_data_base",
        "ros2_list_actions",
        # GetResult on an action server only waits for/reads the result of an existing goal.
        "ros2_action_request_result",
        "ros2_action_subscribe_feedback",
        "ros2_action_subscribe_status",
        "ros2_get_map_as_image",
        "ros2_get_pointcloud_as_bev",
        # Viewer app + its UI-only stream tools: they subscribe and render, never publish.
        "ros2_viewer_app",
        "ros2_viewer_config",
        "ros2_stream_start",
        "ros2_stream_next",
        "ros2_stream_next_image",
        "ros2_stream_stop",
    }
)


def is_mutating(name: str) -> bool:
    """Return True if the tool ``name`` can change robot or ROS graph state."""
    return name in MUTATING_TOOLS
