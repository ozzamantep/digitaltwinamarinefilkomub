#!/bin/bash
set -e

source /opt/ros/humble/setup.bash

if [ -f "/ros2_ws/install/setup.bash" ]; then
    source /ros2_ws/install/setup.bash
fi

# Run AUV subsea physics telemetry publisher in background
if [ -f "/ros2_ws/auv_subsea_publisher.py" ]; then
    echo "Starting AUV Subsea Publisher daemon..."
    nohup python3 /ros2_ws/auv_subsea_publisher.py > /tmp/auv_subsea.log 2>&1 &
fi

exec "$@"
