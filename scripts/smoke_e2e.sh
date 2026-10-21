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
# Smoke tests of a built image.
#
#   scripts/smoke_e2e.sh <image> [rmw ...]          end-to-end, every shipped RMW by default
#   scripts/smoke_e2e.sh --type-support-only <image> ROS 2 type support loads (no graph needed)
#
# End-to-end, for each RMW: a publisher container publishes a payload on a topic whose name is
# unique to this run, and a second container from the same image runs the image's own
# /app/scripts/smoke_client.py, which drives the real server over stdio (read-only mode) and
# checks that it discovers that topic and receives that payload. A unique topic means another
# publisher on the same ROS domain (a parallel run, a robot on the network) can never make a run
# pass. rmw_zenoh_cpp also gets a router container. Containers use --network host, like a real
# install. Nothing is bind-mounted, so it also works from inside a container job.
# Exits non-zero if any check fails.
set -u

# shellcheck disable=SC2016  # expanded inside the container, not here
SRC='source /opt/ros/$ROS_DISTRO/setup.bash && source /root/wisevision_ws/install/setup.bash'

if [ "${1:-}" = --type-support-only ]; then
  IMAGE="${2:?usage: smoke_e2e.sh --type-support-only <image>}"
  exec docker run --rm --entrypoint bash "$IMAGE" -c "$SRC && /app/.venv/bin/python3 -c '
from builtin_interfaces.msg import Time
from rclpy.serialization import serialize_message
Time.__import_type_support__()
serialize_message(Time(sec=1, nanosec=2))
import lora_msgs.srv
print(\"ROS 2 type support smoke test passed\")'"
fi

IMAGE="${1:?usage: smoke_e2e.sh <image> [rmw ...]}"
shift
RMWS=("$@")
[ ${#RMWS[@]} -eq 0 ] && RMWS=(rmw_fastrtps_cpp rmw_cyclonedds_cpp rmw_zenoh_cpp)

RUN_ID="smoke$$x$RANDOM"   # letters, digits: valid in container names and ROS topic names
failed=()

cleanup() { docker rm -f "$RUN_ID-pub" "$RUN_ID-router" "$RUN_ID-client" >/dev/null 2>&1 || true; }
trap cleanup EXIT

# start <name> <command> <docker args...>: run detached and wait until it is running.
start() {
  local name=$1 cmd=$2 _
  shift 2
  if ! docker run -d --name "$name" "$@" --entrypoint bash "$IMAGE" -c "$SRC && exec $cmd" >/dev/null; then
    echo "  cannot start $name"
    return 1
  fi
  for _ in $(seq 20); do
    [ "$(docker inspect -f '{{.State.Running}}' "$name" 2>/dev/null)" = true ] && return 0
    sleep 0.5
  done
  echo "  $name did not stay up:"
  docker logs --tail 15 "$name" 2>&1 | sed 's/^/    /'
  return 1
}

for rmw in "${RMWS[@]}"; do
  cleanup
  # 10..99: ROS 2 domain IDs above 101 can collide with Linux ephemeral ports.
  domain=$(( (RANDOM % 90) + 10 ))
  topic="/${RUN_ID}_${rmw}"
  payload="${RUN_ID}_payload"
  common=(--network host -e "ROS_DOMAIN_ID=$domain" -e "RMW_IMPLEMENTATION=$rmw")
  echo "== $IMAGE / $rmw (domain $domain, topic $topic)"

  if [ "$rmw" = rmw_zenoh_cpp ] && ! start "$RUN_ID-router" "ros2 run rmw_zenoh_cpp rmw_zenohd" "${common[@]}"; then
    echo "FAIL $rmw"; failed+=("$rmw"); continue
  fi
  if ! start "$RUN_ID-pub" "ros2 topic pub -r 5 $topic std_msgs/msg/String '{data: $payload}'" "${common[@]}"; then
    echo "FAIL $rmw"; failed+=("$rmw"); continue
  fi

  log="$(mktemp)"
  # Named, so cleanup() also removes it when `timeout` kills only the docker CLI.
  # Read-only root (only /tmp and ROS's log dir writable): the server must not need to write
  # anywhere else, so `docker run --read-only` works for users.
  timeout 240 docker run --rm -i --name "$RUN_ID-client" "${common[@]}" \
    --read-only --tmpfs /tmp --tmpfs /root/.ros \
    -e "SMOKE_TOPIC=$topic" -e "SMOKE_PAYLOAD=$payload" --entrypoint bash "$IMAGE" \
    -c "$SRC && /app/.venv/bin/python /app/scripts/smoke_client.py" >"$log" 2>&1
  rc=$?
  # Pass needs both: exit 0 AND the client's final line (an early, silent exit 0 is not a pass).
  if [ "$rc" = 0 ] && grep -qx 'SMOKE OK' "$log"; then
    grep -E '^(server|tools|topic_|SMOKE)' "$log" | sed 's/^/  /'
    echo "PASS $rmw"
  else
    echo "FAIL $rmw (client exit $rc)"
    tail -n 25 "$log" | sed 's/^/  client: /'
    docker logs --tail 5 "$RUN_ID-pub" 2>&1 | sed 's/^/  pub: /'
    if [ "$rmw" = rmw_zenoh_cpp ]; then
      docker logs --tail 15 "$RUN_ID-router" 2>&1 | sed 's/^/  router: /'
    fi
    failed+=("$rmw")
  fi
  rm -f "$log"
done

if [ ${#failed[@]} -gt 0 ]; then
  echo "smoke failed for: ${failed[*]}"
  exit 1
fi
echo "smoke passed: ${RMWS[*]}"
