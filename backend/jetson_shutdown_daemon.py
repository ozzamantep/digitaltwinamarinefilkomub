#!/usr/bin/env python3
"""
Jetson System Shutdown Listener
Listens to ROS2 topic /system/command.
When receiving "shutdown", cleanly powers off the Jetson Orin.
"""

import os
import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class JetsonShutdownNode(Node):
    def __init__(self):
        super().__init__('jetson_shutdown_listener')
        self.subscription = self.create_subscription(
            String,
            '/system/command',
            self.command_callback,
            10
        )
        self.get_logger().info('✅ Jetson Shutdown Listener active on /system/command')

    def command_callback(self, msg):
        cmd = msg.data.strip().lower()
        if cmd == 'shutdown':
            self.get_logger().warn('⚠️ RECEIVED SHUTDOWN COMMAND FROM DASHBOARD! Shutting down...')
            # Execute poweroff
            os.system('sudo shutdown -h now || sudo poweroff || sudo systemctl poweroff')

def main(args=None):
    rclpy.init(args=args)
    node = JetsonShutdownNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
