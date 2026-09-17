"""
Digital Twin - ROS2 Launch File
Launches rosbridge_server and optional sensor simulators
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Arguments
    sim_mode = DeclareLaunchArgument(
        'sim_mode',
        default_value='false',
        description='Enable sensor simulation mode'
    )

    port = DeclareLaunchArgument(
        'port',
        default_value='9090',
        description='WebSocket port for rosbridge'
    )

    # Rosbridge WebSocket Server
    rosbridge_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'address': '',
            'retry_startup_delay': 5.0,
            'fragment_timeout': 600,
            'delay_between_messages': 0,
            'max_message_size': 10000000,
            'unregister_timeout': 10.0,
        }],
        output='screen',
    )

    return LaunchDescription([
        sim_mode,
        port,
        rosbridge_node,
    ])
