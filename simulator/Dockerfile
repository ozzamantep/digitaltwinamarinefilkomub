FROM osrf/ros:humble-desktop-full

# Install essential dependencies, Gazebo, and rosbridge
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-rosbridge-server \
    ros-humble-teleop-twist-keyboard \
    ros-humble-joint-state-publisher \
    ros-humble-robot-state-publisher \
    ros-humble-xacro \
    ros-humble-image-transport \
    ros-humble-cv-bridge \
    python3-pip \
    git \
    mesa-utils \
    && rm -rf /var/lib/apt/lists/*

# Create workspace for underwater simulation
WORKDIR /ros2_ws/src

# Setup entrypoint and telemetry node
WORKDIR /ros2_ws
COPY ./auv_subsea_publisher.py /ros2_ws/auv_subsea_publisher.py
COPY ./entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh /ros2_ws/auv_subsea_publisher.py

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "rosbridge_server", "rosbridge_websocket_launch.xml"]
