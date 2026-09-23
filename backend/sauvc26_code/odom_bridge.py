#!/usr/bin/env python3
"""
MAVROS Telemetry Bridge - Real Pixhawk Data -> Digital Twin

Without this node, /odom, /battery_state, /depth and /sonar/front/range have
NO publisher on the real vehicle (only the SITL simulators publish them) -
the dashboard's 3D position never moves, the real battery-critical
emergency-surface safety logic and depth/collision safety in
collision_safety.py never see real data. Republishes MAVROS topics the app
expects under different names/types:

Read-only telemetry bridge - safe to run alongside any other node
(mission, manual_bridge, etc), no control_lock needed.

Subscribes:
    /mavros/local_position/pose           (geometry_msgs/PoseStamped)
    /mavros/local_position/velocity_local (geometry_msgs/TwistStamped)
    /mavros/battery                       (sensor_msgs/BatteryState)
    /mavros/imu/static_pressure            (sensor_msgs/FluidPressure) - per schematic (2026-09-23),
        the water pressure sensor (MS5837/Bar30) is wired directly to Pixhawk over I2C, not the Jetson.
        ASSUMPTION - VERIFY on the real Jetson with `ros2 topic echo /mavros/imu/static_pressure`
        (should read ~101325 Pa in air, rising with depth underwater) before trusting depth safety
        checks on this bridge; if the FC reports it on a different baro index, adjust the topic name below.
    MAVROS_DISTANCE_SENSOR_TOPIC           (sensor_msgs/Range) - the single front sonar, per schematic
        wired into the Pixhawk rangefinder input (not a separate Jetson-attached driver).
        ASSUMPTION - VERIFY the actual topic name with `ros2 topic list | grep distance_sensor`
        on the real Jetson (depends on the mavros apm_config.yaml plugin id mapping) and adjust below.
Publishes:
    /odom (nav_msgs/Odometry)
    /battery_state (sensor_msgs/BatteryState) - same type as /mavros/battery, straight passthrough
    /depth (sensor_msgs/FluidPressure) - same type as /mavros/imu/static_pressure, straight passthrough
    /sonar/front/range (sensor_msgs/Range) - same type as the MAVROS distance sensor topic, straight passthrough
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import BatteryState, FluidPressure, Range

# Best-guess default MAVROS/ArduSub topic for a single configured rangefinder -
# VERIFY against the real Jetson (see module docstring) and correct if different.
MAVROS_DISTANCE_SENSOR_TOPIC = '/mavros/distance_sensor/rangefinder_pub'


class OdomBridge(Node):
    def __init__(self):
        super().__init__('odom_bridge')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.battery_pub = self.create_publisher(BatteryState, '/battery_state', 10)
        self.depth_pub = self.create_publisher(FluidPressure, '/depth', 10)
        self.sonar_front_pub = self.create_publisher(Range, '/sonar/front/range', 10)
        self.latest_twist = TwistStamped()

        self.pose_sub = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self.pose_callback, qos_profile
        )
        self.twist_sub = self.create_subscription(
            TwistStamped, '/mavros/local_position/velocity_local', self.twist_callback, qos_profile
        )
        self.battery_sub = self.create_subscription(
            BatteryState, '/mavros/battery', self.battery_callback, qos_profile
        )
        self.pressure_sub = self.create_subscription(
            FluidPressure, '/mavros/imu/static_pressure', self.pressure_callback, qos_profile
        )
        self.distance_sub = self.create_subscription(
            Range, MAVROS_DISTANCE_SENSOR_TOPIC, self.distance_callback, qos_profile
        )
        self.get_logger().info(
            '[MavrosTelemetryBridge] /mavros/local_position/{pose,velocity_local} -> /odom, '
            '/mavros/battery -> /battery_state, /mavros/imu/static_pressure -> /depth, '
            f'{MAVROS_DISTANCE_SENSOR_TOPIC} -> /sonar/front/range'
        )

    def twist_callback(self, msg):
        self.latest_twist = msg

    def battery_callback(self, msg):
        # Straight passthrough - same message type, just the topic name the app expects
        self.battery_pub.publish(msg)

    def pressure_callback(self, msg):
        # Straight passthrough - same message type, just the topic name the app expects
        self.depth_pub.publish(msg)

    def distance_callback(self, msg):
        # Straight passthrough - same message type, just the topic name the app expects
        self.sonar_front_pub.publish(msg)

    def pose_callback(self, msg):
        # Fires on every real pose update (typically 20-30Hz) - the real hull moving
        # (thruster-driven OR pushed by hand/current) is reflected here immediately.
        odom = Odometry()
        odom.header = msg.header
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose = msg.pose
        odom.twist.twist = self.latest_twist.twist
        self.odom_pub.publish(odom)


def main():
    rclpy.init()
    node = OdomBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
