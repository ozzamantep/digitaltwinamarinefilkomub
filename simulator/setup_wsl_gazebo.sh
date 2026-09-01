#!/bin/bash
# ==============================================================================
# Launch SAUVC Pool World & Custom 6-Thruster AUV in Gazebo WSL2
# ==============================================================================
set -e

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Source ROS 2 environment
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source "/opt/ros/humble/setup.bash"
elif [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source "/opt/ros/jazzy/setup.bash"
fi

echo "======================================================================"
echo "   Launching SAUVC Subsea Pool & Custom AUV in Gazebo 3D"
echo "======================================================================"
echo ""

# 1. Kill any stale rosbridge / publisher processes
pkill -f "rosbridge_websocket" 2>/dev/null || true
pkill -f "auv_subsea_publisher" 2>/dev/null || true
sleep 1

# 2. Start Rosbridge WebSocket on ws://localhost:9090
echo "[1/3] Starting Rosbridge WebSocket on ws://localhost:9090..."
ros2 launch rosbridge_server rosbridge_websocket_launch.xml &
sleep 2

# 3. Start Subsea ROS 2 Telemetry Publisher
echo "[2/3] Starting AUV Subsea ROS 2 Telemetry Stream at 20 Hz..."
python3 "$DIR/auv_subsea_publisher.py" &
sleep 1

# 4. Launch Gazebo Classic with SAUVC Pool World (AUV embedded)
echo "[3/3] Opening Gazebo 3D Subsea Pool & AUV Robot..."
echo "      (Jendela 3D Gazebo akan terbuka di layar Windows kamu!)"
echo ""

export GAZEBO_RESOURCE_PATH=/usr/share/gazebo-11:$GAZEBO_RESOURCE_PATH
gazebo --verbose "$DIR/sauvc_pool.world"
