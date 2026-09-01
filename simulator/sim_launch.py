#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    rosbridge_share = get_package_share_directory('rosbridge_server')
    
    rosbridge_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(rosbridge_share, 'launch', 'rosbridge_websocket_launch.xml')
        )
    )
    
    auv_telemetry_node = Node(
        package='ros2cli',
        executable='ros2',
        arguments=['run', 'demo_nodes_py', 'listener'], # fallback placeholder
        name='auv_telemetry_placeholder',
    )
    
    return LaunchDescription([
        rosbridge_launch,
    ])
