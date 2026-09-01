#!/usr/bin/env python3
"""
AUV BlueROV2 Subsea ROS2 Node & Physics Publisher
Publishes real-time hydrodynamic telemetry to ROS2 topics:
 - /odom (nav_msgs/Odometry)
 - /imu/data (sensor_msgs/Imu)
 - /depth (sensor_msgs/FluidPressure)
 - /dvl/range (sensor_msgs/Range)
 - /battery_state (sensor_msgs/BatteryState)
"""

import time
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, FluidPressure, Range, BatteryState
from geometry_msgs.msg import Quaternion, Twist
from std_msgs.msg import Float64MultiArray

class AuvSubseaPublisher(Node):
    def __init__(self):
        super().__init__('auv_subsea_publisher')
        
        self.pub_odom = self.create_publisher(Odometry, '/odom', 10)
        self.pub_imu = self.create_publisher(Imu, '/imu/data', 10)
        self.pub_depth = self.create_publisher(FluidPressure, '/depth', 10)
        self.pub_dvl = self.create_publisher(Range, '/dvl/range', 10)
        self.pub_battery = self.create_publisher(BatteryState, '/battery_state', 10)
        self.pub_thrusters = self.create_publisher(Float64MultiArray, '/thruster_outputs', 10)

        # Velocity and Thruster Command Subscribers
        self.sub_cmd_vel = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        self.timer = self.create_timer(0.05, self.timer_callback) # 20 Hz
        self.start_time = time.time()
        self.sim_depth = 0.85
        self.x = -6.0
        self.y = 0.0
        self.surge_speed = 0.75
        self.sway_speed = 0.0
        self.heave_speed = 0.0
        self.yaw_rate = 0.0
        self.battery_level = 0.92
        self.thrusters = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.get_logger().info('🌊 BlueROV2 Subsea ROS2 Physics Node started with 6-Thruster TAM Allocation!')

    def cmd_vel_callback(self, msg: Twist):
        # Ingest joystick / pilot commands
        self.surge_speed = float(msg.linear.x)
        self.sway_speed = float(msg.linear.y)
        self.heave_speed = float(msg.linear.z)
        self.yaw_rate = float(msg.angular.z)

        # 6-Thruster Allocation Matrix (TAM)
        # T1..T4 = 45 deg Horizontal Corner Thrusters (Surge, Sway, Yaw)
        # T5..T6 = Vertical Thrusters in Yellow Nose/Tail Cowlings (Heave & Pitch)
        t_surge = self.surge_speed
        t_sway = self.sway_speed
        t_yaw = self.yaw_rate
        t_heave = self.heave_speed

        t1 = max(-1.0, min(1.0, t_surge + t_yaw - t_sway)) # Front-Left (45°)
        t2 = max(-1.0, min(1.0, t_surge - t_yaw + t_sway)) # Front-Right (-45°)
        t3 = max(-1.0, min(1.0, t_surge + t_yaw + t_sway)) # Rear-Left (135°)
        t4 = max(-1.0, min(1.0, t_surge - t_yaw - t_sway)) # Rear-Right (-135°)
        t5 = max(-1.0, min(1.0, t_heave))                  # Front-Vertical (Yellow Nose)
        t6 = max(-1.0, min(1.0, t_heave))                  # Rear-Vertical (Yellow Tail)

        self.thrusters = [t1, t2, t3, t4, t5, t6]

    def timer_callback(self):
        t = time.time() - self.start_time
        
        # 1. Odometry (/odom)
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        self.x += math.cos(t * 0.2) * self.surge_speed * 0.05
        self.y += math.sin(t * 0.2) * 0.2 * 0.05
        self.sim_depth = 0.85 + math.sin(t * 0.4) * 0.15
        
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = -self.sim_depth
        
        # Heading yaw quaternion
        yaw = t * 0.2
        odom.pose.pose.orientation.z = math.sin(yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        odom.twist.twist.linear.x = self.surge_speed
        odom.twist.twist.linear.y = math.sin(t) * 0.05
        odom.twist.twist.linear.z = math.cos(t * 0.4) * 0.06
        self.pub_odom.publish(odom)
        
        # 2. IMU (/imu/data)
        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = 'imu_link'
        imu.orientation = odom.pose.pose.orientation
        imu.linear_acceleration.x = 0.15 * math.sin(t * 2)
        imu.linear_acceleration.y = 0.05 * math.cos(t * 2)
        imu.linear_acceleration.z = -9.81
        self.pub_imu.publish(imu)
        
        # 3. Depth & Water Pressure (/depth)
        # MS5837 fluid pressure: 101325 Pa + rho * g * depth + bernoulli dynamic pressure
        v_total = math.sqrt(self.surge_speed**2 + 0.05**2)
        p_dyn = 0.5 * 1000.0 * (v_total**2)
        pressure_pa = 101325.0 + (997.0 * 9.80665 * self.sim_depth) + p_dyn + math.sin(t * 4.0) * 80.0
        
        depth_msg = FluidPressure()
        depth_msg.header.stamp = self.get_clock().now().to_msg()
        depth_msg.header.frame_id = 'pressure_sensor'
        depth_msg.fluid_pressure = float(pressure_pa)
        self.pub_depth.publish(depth_msg)
        
        # 4. DVL Altitude to bottom (/dvl/range)
        # SAUVC pool depth is ~2.0m, altitude = pool_depth - robot_depth
        dvl = Range()
        dvl.header.stamp = self.get_clock().now().to_msg()
        dvl.header.frame_id = 'dvl_link'
        dvl.range = float(max(0.1, 2.0 - self.sim_depth + math.sin(t * 5.0) * 0.005))
        self.pub_dvl.publish(dvl)
        
        # 5. 4S LiPo Battery (/battery_state)
        bat = BatteryState()
        bat.header.stamp = self.get_clock().now().to_msg()
        self.battery_level = max(0.05, self.battery_level - 0.00002)
        bat.percentage = float(self.battery_level)
        bat.voltage = float(14.6 + self.battery_level * 2.2)
        bat.current = float(4.5 + math.sin(t * 3.0) * 1.2)
        bat.temperature = float(28.0 + math.sin(t * 0.5) * 0.5)
        self.pub_battery.publish(bat)

        # 6. Live Thruster Outputs (/thruster_outputs)
        thruster_msg = Float64MultiArray()
        thruster_msg.data = [float(val) for val in self.thrusters]
        self.pub_thrusters.publish(thruster_msg)

def main(args=None):
    rclpy.init(args=args)
    node = AuvSubseaPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
