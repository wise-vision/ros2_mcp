ARG ROS_DISTRO=jazzy
# The viewer bundle is a single platform-independent index.html, so build it on
# the BUILD platform. Building it under QEMU for arm64 made esbuild's Go binary
# die with SIGBUS during `npm install` (docker_image.yml, main run 36641741446).
FROM --platform=$BUILDPLATFORM node:20-bookworm-slim AS ui_build
WORKDIR /ui
COPY server/ui/ros2_viewer_app/package.json server/ui/ros2_viewer_app/package-lock.json /ui/
COPY server/ui/ros2_viewer_app/build.mjs /ui/build.mjs
COPY server/ui/ros2_viewer_app/tsconfig.json /ui/tsconfig.json
COPY server/ui/ros2_viewer_app/index.template.html /ui/index.template.html
COPY server/ui/ros2_viewer_app/src /ui/src
RUN npm ci --no-audit --no-fund
RUN npm run build

# WiseVision message packages (lora_msgs is the Data Black Box /get_messages service type), built
# from a pinned commit for whichever distro this image targets. Building them here, instead of
# inheriting a prebuilt base image, is what lets every distro (including Lyrical and Rolling) build
# from the official ros:<distro>-ros-base image.
FROM ros:${ROS_DISTRO}-ros-base AS msgs_build
# wisevision_msgs release 2602.
ARG WISEVISION_MSGS_SHA=a6df52afd3a78a813b45923e4d7f1e04a36377ba
RUN git init -q /src/wisevision_msgs && \
    git -C /src/wisevision_msgs fetch -q --depth 1 https://github.com/wise-vision/wisevision_msgs.git "$WISEVISION_MSGS_SHA" && \
    git -C /src/wisevision_msgs checkout -q FETCH_HEAD
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build --base-paths /src --build-base /tmp/build \
      --install-base /root/wisevision_ws/install --merge-install \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

FROM ros:${ROS_DISTRO}-ros-base

LABEL io.modelcontextprotocol.server.name="io.github.wise-vision/ros2_mcp" \
      org.opencontainers.image.title="ROS2 MCP" \
      org.opencontainers.image.description="ROS2 MCP: open-source MCP server for ROS 2" \
      org.opencontainers.image.source="https://github.com/wise-vision/ros2_mcp" \
      org.opencontainers.image.documentation="https://wisevision.tech/docs" \
      org.opencontainers.image.licenses="MPL-2.0"

# FASTDDS_BUILTIN_TRANSPORTS=UDPv4: with the default RMW (Fast DDS), nodes on the same host
# exchange data over shared memory, which a container in its own IPC namespace cannot reach:
# topics are discovered but no messages arrive. UDP-only works without --ipc host. Other RMWs
# ignore it. (RMW_FASTRTPS_USE_SHM is not a Fast DDS setting and had no effect.)
#
# UV_PYTHON pins the venv to the distro's own interpreter: rclpy and every generated message
# package are C extensions built for exactly that Python.
ENV MCP_CUSTOM_PROMPTS="false" \
    MCP_PROMPTS_LOCAL="false" \
    MCP_PROMPTS_PATH="/app/ros2_mcp_prompts" \
    MCP_PROMPTS_MODULE="extension_prompts" \
    FASTDDS_BUILTIN_TRANSPORTS="UDPv4" \
    ROS2_MCP_READONLY="false" \
    UV_PYTHON="/usr/bin/python3" \
    UV_PYTHON_DOWNLOADS="never" \
    UV_LINK_MODE="copy"

COPY --from=ghcr.io/astral-sh/uv:0.9.27 /uv /uvx /usr/local/bin/

# REQUIRED must exist on every supported distro: a missing one fails the build.
# OPTIONAL are extra message types for common stacks (Nav2, MAVROS, MoveIt, ...). They are
# installed when the distro has them; a missing one fails the build too, unless it is listed in
# KNOWN_MISSING for that distro, so a package silently disappearing from a release is caught.
# The image records what it skipped in /etc/ros2_mcp/skipped-packages.
RUN set -eu; \
    REQUIRED="rclpy ros2cli rmw-zenoh-cpp rmw-cyclonedds-cpp common-interfaces action-msgs \
      builtin-interfaces example-interfaces tf2-msgs unique-identifier-msgs map-msgs"; \
    OPTIONAL="nav2-msgs mavros-msgs ackermann-msgs moveit-msgs control-msgs vision-msgs \
      tf2-geometry-msgs image-transport image-transport-plugins camera-info-manager diagnostic-updater"; \
    case "$ROS_DISTRO" in \
      rolling) KNOWN_MISSING="nav2-msgs" ;; \
      *) KNOWN_MISSING="" ;; \
    esac; \
    apt-get update; \
    pkgs=""; for p in $REQUIRED; do pkgs="$pkgs ros-$ROS_DISTRO-$p"; done; \
    mkdir -p /etc/ros2_mcp; : > /etc/ros2_mcp/skipped-packages; \
    for p in $OPTIONAL; do \
      if apt-cache show "ros-$ROS_DISTRO-$p" >/dev/null 2>&1; then pkgs="$pkgs ros-$ROS_DISTRO-$p"; \
      elif echo " $KNOWN_MISSING " | grep -q " $p "; then echo "ros-$ROS_DISTRO-$p" >> /etc/ros2_mcp/skipped-packages; \
      else echo "ros-$ROS_DISTRO-$p is not released for $ROS_DISTRO and not in KNOWN_MISSING" >&2; exit 1; fi; \
    done; \
    apt-get install -y --no-install-recommends ca-certificates $pkgs; \
    rm -rf /var/lib/apt/lists/*; \
    echo "skipped optional packages: $(cat /etc/ros2_mcp/skipped-packages | tr '\n' ' ')"

COPY --from=msgs_build /root/wisevision_ws/install /root/wisevision_ws/install

WORKDIR /app
COPY . /app
COPY --from=ui_build /ui/index.html /app/server/ui/ros2_viewer_app/index.html

# --system-site-packages: ROS's own Python dependencies (e.g. empy, which generated messages
# import on Lyrical and later) are Debian packages, invisible to an isolated venv.
RUN uv venv --system-site-packages && uv sync --locked --extra custom-prompts

RUN mkdir -p /ros2_mcp_prompts /app/custom_msgs
VOLUME ["/ros2_mcp_prompts"]

ENV RMW_IMPLEMENTATION=""

# Only used with ROS2_MCP_TRANSPORT=streamable-http (or sse); stdio needs no port.
EXPOSE 8123

ENTRYPOINT []
CMD ["bash", "-c", " \
  source /opt/ros/${ROS_DISTRO}/setup.bash && \
  source /root/wisevision_ws/install/setup.bash && \
  if [ -n \"$RMW_IMPLEMENTATION\" ]; then export RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION; fi && \
  if [ -f /app/custom_msgs/install/setup.bash ]; then \
  source /app/custom_msgs/install/setup.bash; \
  fi && \
  uv run --locked mcp_ros_2_server \
  "]
