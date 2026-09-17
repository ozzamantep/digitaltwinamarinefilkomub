#!/bin/bash
# =====================================================
# DIGITAL TWIN - Jetson Orin Setup Script
# Installs rosbridge_server for WebSocket communication
# =====================================================

set -e

echo "=============================================="
echo " Digital Twin - Jetson Orin Setup"
echo "=============================================="

# Detect ROS2 distro
if [ -z "$ROS_DISTRO" ]; then
    echo "❌ ROS2 not sourced. Please source your ROS2 setup first:"
    echo "   source /opt/ros/<distro>/setup.bash"
    exit 1
fi

echo "✅ ROS2 Distro: $ROS_DISTRO"

# Install rosbridge_server
echo ""
echo "📦 Installing rosbridge_server..."
sudo apt update
sudo apt install -y ros-${ROS_DISTRO}-rosbridge-server

# Install compressed image transport (for camera)
echo ""
echo "📦 Installing image transport..."
sudo apt install -y ros-${ROS_DISTRO}-compressed-image-transport

echo ""
echo "=============================================="
echo " ✅ Setup Complete!"
echo "=============================================="
echo ""
echo " To start the rosbridge server, run:"
echo "   ros2 launch rosbridge_server rosbridge_websocket.launch.xml"
echo ""
echo " Or use the provided launch file:"
echo "   ros2 launch digitaltwin digitaltwin.launch.py"
echo ""
echo " Default WebSocket URL: ws://$(hostname -I | awk '{print $1}'):9090"
echo ""
