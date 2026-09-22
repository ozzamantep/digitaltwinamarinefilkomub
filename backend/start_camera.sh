#!/bin/bash
# =====================================================
# DIGITAL TWIN - Forward Camera Driver Launcher
# Publishes the real front camera to /camera/image_raw/compressed,
# which the Digital Twin dashboard already expects (TopicSubscriber.js).
# Uses the standard ROS2 v4l2_camera driver + image_transport republish -
# no custom vision code needed, just the correct topic wiring.
# =====================================================

set -e

if [ -z "$ROS_DISTRO" ]; then
    echo "❌ ROS2 not sourced. Please source your ROS2 setup first:"
    echo "   source /opt/ros/<distro>/setup.bash"
    exit 1
fi

DEVICE="${1:-/dev/video0}"

echo "📦 Installing v4l2_camera + image_transport (skips if already installed)..."
sudo apt update
sudo apt install -y ros-${ROS_DISTRO}-v4l2-camera ros-${ROS_DISTRO}-image-transport ros-${ROS_DISTRO}-compressed-image-transport

echo ""
echo "🎥 Starting camera driver on $DEVICE -> /camera/image_raw ..."
ros2 run v4l2_camera v4l2_camera_node --ros-args \
    -p video_device:="$DEVICE" \
    -p image_size:="[640,480]" \
    -r /image_raw:=/camera/image_raw &
CAMERA_PID=$!

echo "🗜️  Starting compressed republisher -> /camera/image_raw/compressed ..."
ros2 run image_transport republish raw compressed --ros-args \
    -r in:=/camera/image_raw \
    -r out/compressed:=/camera/image_raw/compressed &
REPUBLISH_PID=$!

trap "kill $CAMERA_PID $REPUBLISH_PID 2>/dev/null" EXIT
wait
