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
# End-to-end smoke test of a built image against a live ROS 2 graph, for every RMW it ships.
#
#   scripts/smoke_e2e.sh <image> [rmw ...]
#
# For each RMW: a publisher container sends /chatter, and a second container from the same image
# runs scripts/smoke_client.py, which drives the real server over stdio (read-only mode) and checks
# that it discovers the topic and receives messages. rmw_zenoh_cpp also gets a router container.
# Containers use --network host, like a real install, and a random ROS_DOMAIN_ID so parallel runs
# on one machine don't see each other. Exits non-zero if any RMW fails.
set -u

IMAGE="${1:?usage: smoke_e2e.sh <image> [rmw ...]}"
shift
RMWS=("$@")
[ ${#RMWS[@]} -eq 0 ] && RMWS=(rmw_fastrtps_cpp rmw_cyclonedds_cpp rmw_zenoh_cpp)

HERE="$(cd "$(dirname "$0")" && pwd)"
RUN_ID="smoke$$"
failed=()

cleanup() { docker rm -f "$RUN_ID-pub" "$RUN_ID-router" >/dev/null 2>&1 || true; }
trap cleanup EXIT

for rmw in "${RMWS[@]}"; do
  cleanup
  domain=$(( (RANDOM % 100) + 100 ))
  common=(--network host -e "ROS_DOMAIN_ID=$domain" -e "RMW_IMPLEMENTATION=$rmw")
  # shellcheck disable=SC2016  # expanded inside the container, not here
  src='source /opt/ros/$ROS_DISTRO/setup.bash && source /root/wisevision_ws/install/setup.bash'

  if [ "$rmw" = rmw_zenoh_cpp ]; then
    docker run -d --name "$RUN_ID-router" "${common[@]}" --entrypoint bash "$IMAGE" \
      -c "$src && exec ros2 run rmw_zenoh_cpp rmw_zenohd" >/dev/null
    sleep 2
  fi
  docker run -d --name "$RUN_ID-pub" "${common[@]}" --entrypoint bash "$IMAGE" \
    -c "$src && exec ros2 topic pub -r 5 /chatter std_msgs/msg/String '{data: smoke}'" >/dev/null

  echo "== $IMAGE / $rmw (domain $domain)"
  log="$(mktemp)"
  if timeout 240 docker run --rm -i "${common[@]}" -v "$HERE:/smoke:ro" --entrypoint bash "$IMAGE" \
       -c "$src && /app/.venv/bin/python /smoke/smoke_client.py" >"$log" 2>&1; then
    grep -v '^\[' "$log" | grep -E '^(server|tools|topic_|SMOKE)' | sed 's/^/  /'
    echo "PASS $rmw"
  else
    echo "FAIL $rmw"
    tail -n 25 "$log" | sed 's/^/  client: /'
    docker logs --tail 5 "$RUN_ID-pub" 2>&1 | sed 's/^/  pub: /'
    failed+=("$rmw")
  fi
  rm -f "$log"
done

if [ ${#failed[@]} -gt 0 ]; then
  echo "smoke failed for: ${failed[*]}"
  exit 1
fi
echo "smoke passed: ${RMWS[*]}"
