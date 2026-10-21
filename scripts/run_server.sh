#!/usr/bin/env bash
#
#  Copyright (C) 2025 wisevision
#
#  SPDX-License-Identifier: MPL-2.0
#
#  This Source Code Form is subject to the terms of the Mozilla Public
#  License, v. 2.0. If a copy of the MPL was not distributed with this
#  file, You can obtain one at https://mozilla.org/MPL/2.0/.
#
# The image's CMD: source ROS 2, the bundled interfaces and any mounted custom messages, then run
# the MCP server in place of this shell. scripts/smoke_client.py starts the server through this
# same script, so the smoke test exercises exactly what users run.

# shellcheck disable=SC1090,SC1091  # sourced from the image
source "/opt/ros/${ROS_DISTRO}/setup.bash" || exit 1
# shellcheck disable=SC1091
source /root/wisevision_ws/install/setup.bash || exit 1
if [ -f /app/custom_msgs/install/setup.bash ]; then
  # shellcheck disable=SC1091
  source /app/custom_msgs/install/setup.bash || exit 1
fi
cd /app || exit 1
exec uv run --locked mcp_ros_2_server "$@"
