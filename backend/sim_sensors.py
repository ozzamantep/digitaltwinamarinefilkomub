#!/usr/bin/env python3
"""
Digital Twin - BlueROV2 AUV Sensor Simulator for ROS2
Simulates SAUVC 2026 subsea competition world sensor telemetry.

Usage:
    ros2 run digitaltwin sim_sensors
    # or directly:
    python3 sim_sensors.py
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, BatteryState, Range, FluidPressure
from geometry_msgs.msg import Quaternion
import math
import random


class BlueROV2Simulator(Node):
    def __init__(self):
        super().__init__('bluerov2_simulator')
        self.get_logger().info('🌊 BlueROV2 SAUVC Subsea Simulator Started!')

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.depth_pub = self.create_publisher(FluidPressure, '/depth', 10)
        self.dvl_pub = self.create_publisher(Range, '/dvl/range', 10)
        self.battery_pub = self.create_publisher(BatteryState, '/battery_state', 10)

        # 30 Hz Telemetry loop
        self.timer = self.create_timer(1.0 / 30.0, self.publish_telemetry)
        self.t = 0.0
        self.battery_level = 0.95

        self.get_logger().info('📡 Publishing: /odom, /imu/data, /depth, /dvl/range, /battery_state')

    def publish_telemetry(self):
        self.t += 1.0 / 30.0
        t = self.t

        # SAUVC Pool Trajectory
        loop_time = (t * 0.25) % (math.pi * 2)
        x = math.sin(loop_time) * 9.5
        y = math.cos(loop_time * 2) * 2.8
        depth = 0.95 + math.sin(t * 0.4) * 0.35
        z = -depth # ROS Z is depth

        dx = math.cos(loop_time) * 9.5
        dy = -math.sin(loop_time * 2) * 5.6
        yaw = math.atan2(dy, dx)
        roll = math.sin(t * 0.8) * 0.04
        pitch = math.sin(t * 0.5) * 0.06

        # 1. Odometry
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = z
        odom.pose.pose.orientation = self._euler_to_quaternion(roll, pitch, yaw)

        surge_speed = math.sqrt(dx * dx + dy * dy) * 0.08 + 0.3
        odom.twist.twist.linear.x = surge_speed
        odom.twist.twist.linear.y = math.sin(t * 0.5) * 0.05
        odom.twist.twist.linear.z = math.cos(t * 0.4) * 0.08
        odom.twist.twist.angular.z = math.cos(loop_time * 2) * 0.2
        self.odom_pub.publish(odom)

        # 2. IMU
        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = 'imu_link'
        imu.orientation = self._euler_to_quaternion(roll, pitch, yaw)
        imu.angular_velocity.z = odom.twist.twist.angular.z
        imu.linear_acceleration.x = math.sin(t * 1.2) * 0.2
        imu.linear_acceleration.y = math.cos(t * 0.9) * 0.15
        imu.linear_acceleration.z = -9.81 + math.sin(t * 2) * 0.05
        self.imu_pub.publish(imu)

        # 3. Depth (FluidPressure)
        press = FluidPressure()
        press.header.stamp = self.get_clock().now().to_msg()
        press.fluid_pressure = 101325.0 + depth * 997.0 * 9.81
        self.depth_pub.publish(press)

        # 4. DVL Altitude to bottom
        dvl = Range()
        dvl.header.stamp = self.get_clock().now().to_msg()
        dvl.range = 2.0 - depth # distance to floor
        dvl.min_range = 0.1
        dvl.max_range = 10.0
        self.dvl_pub.publish(dvl)

        # 5. 4S LiPo Battery (1 Hz)
        if int(t * 30) % 30 == 0:
            self.battery_level = max(0.1, self.battery_level - 0.0001)
            bat = BatteryState()
            bat.header.stamp = self.get_clock().now().to_msg()
            bat.voltage = 14.4 + self.battery_level * 2.4 # 14.4V to 16.8V
            bat.current = 7.5 + random.random() * 3.5
            bat.percentage = self.battery_level
            bat.temperature = 28.0 + random.random() * 2.0
            bat.present = True
            self.battery_pub.publish(bat)

    def _euler_to_quaternion(self, roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = Quaternion()
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy
        q.w = cr * cp * cy + sr * sp * sy
        return q


def main(args=None):
    rclpy.init(args=args)
    node = BlueROV2Simulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
