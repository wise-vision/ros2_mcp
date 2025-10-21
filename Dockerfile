ARG ROS_DISTRO=jazzy
FROM wisevision/ros_with_wisevision_msgs_and_wisevision_core:${ROS_DISTRO}

LABEL io.modelcontextprotocol.server.name="io.github.wise-vision/mcp_server_ros_2"

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

RUN uv sync

ENTRYPOINT []
CMD ["bash", "-c", "source /opt/ros/${ROS_DISTRO}/setup.bash && source /root/wisevision_ws/install/setup.bash && source .venv/bin/activate && uv run mcp_ros_2_server"]