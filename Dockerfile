ARG ROS_DISTRO=jazzy
FROM wisevision/ros_with_wisevision_msgs_and_wisevision_core:${ROS_DISTRO}

LABEL io.modelcontextprotocol.server.name="io.github.wise-vision/mcp_server_ros_2"

ENV MCP_CUSTOM_PROMPTS="false" \
    MCP_PROMPTS_LOCAL="false" \
    MCP_PROMPTS_PATH="/app/ros2_mcp_prompts" \
    MCP_PROMPTS_MODULE="extension_prompts"

RUN apt-get update && apt-get install -y \
    python3-pip \
    build-essential \
    ca-certificates \
    ros-${ROS_DISTRO}-std-msgs \
    ros-${ROS_DISTRO}-geometry-msgs \
    ros-${ROS_DISTRO}-sensor-msgs \
    ros-${ROS_DISTRO}-nav-msgs \
    ros-${ROS_DISTRO}-action-msgs \
    ros-${ROS_DISTRO}-diagnostic-msgs \
    ros-${ROS_DISTRO}-trajectory-msgs \
    ros-${ROS_DISTRO}-visualization-msgs \
    ros-${ROS_DISTRO}-example-interfaces \
    ros-${ROS_DISTRO}-rclpy \
    ros-${ROS_DISTRO}-ros2cli \
    ros-${ROS_DISTRO}-nav2-msgs \
    ros-${ROS_DISTRO}-mavros-msgs \
    ros-${ROS_DISTRO}-ackermann-msgs \
    ros-${ROS_DISTRO}-moveit-msgs \
    ros-${ROS_DISTRO}-action-tutorials-interfaces \
    ros-${ROS_DISTRO}-builtin-interfaces \
    ros-${ROS_DISTRO}-tf2-msgs \
    ros-${ROS_DISTRO}-tf2-geometry-msgs \
    ros-${ROS_DISTRO}-vision-msgs \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-image-transport-plugins \
    ros-${ROS_DISTRO}-camera-info-manager \
    ros-${ROS_DISTRO}-unique-identifier-msgs \
    ros-${ROS_DISTRO}-common-interfaces \
    ros-${ROS_DISTRO}-diagnostic-updater \
    ros-${ROS_DISTRO}-map-msgs \
    ros-${ROS_DISTRO}-control-msgs \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

RUN if [ "$ROS_DISTRO" = "humble" ]; then \
  pip install uv; \
  elif [ "$ROS_DISTRO" = "jazzy" ]; then \
  pip install uv --break-system-packages; \
  fi

WORKDIR /app
COPY . /app

RUN uv venv

RUN if [ "$ROS_DISTRO" = "jazzy" ]; then \
  uv python pin 3.12; \
  fi

RUN uv sync --extra custom-prompts

RUN mkdir -p /mcp/custom_msgs
VOLUME ["/mcp/custom_msgs"]

RUN mkdir -p /ros2_mcp_prompts
VOLUME ["/ros2_mcp_prompts"]

ENTRYPOINT []
CMD ["bash", "-c", " \
  source /opt/ros/${ROS_DISTRO}/setup.bash && \
  source /root/wisevision_ws/install/setup.bash && \
  if [ -f /app/custom_msgs/install/setup.bash ]; then \
  source /app/custom_msgs/install/setup.bash; \
  fi && \
  uv run mcp_ros_2_server \
  "]