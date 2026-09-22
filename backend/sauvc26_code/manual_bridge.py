#!/usr/bin/env python3
"""
Manual Control Bridge - Digital Twin Dashboard -> Real Vehicle (MAVROS/Pixhawk)

Translates dashboard teleop/gripper commands into MAVROS setpoints so the operator
can actually drive the physical AUV, not just the simulated twin.

Run this INSTEAD OF final.py/qualification.py - only one node should publish to
/mavros/setpoint_raw/local at a time, or their commands will fight each other.
Vehicle must be in GUIDED mode (set via arm.py or QGroundControl) for setpoints
to be obeyed.

Subscribes:
    /cmd_vel          (geometry_msgs/Twist)  - dashboard manual pilot: surge=linear.x, yaw=angular.z
    /gripper/command  (std_msgs/String)      - ball dropper servo: OPEN/RELEASE actuate, CLOSE/GRASP no-op

Publishes:
    /mavros/setpoint_raw/local (mavros_msgs/PositionTarget) - body-frame velocity setpoint, 20 Hz
Calls:
    /mavros/cmd/command (mavros_msgs/CommandLong) - servo dropper actuation
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist
from std_msgs.msg import String
from mavros_msgs.msg import PositionTarget
from mavros_msgs.srv import CommandLong

# Ball dropper servo (same channel/PWM as the autonomous mission nodes - keep in sync)
MAV_CMD_DO_SET_SERVO = 183
DROPPER_SERVO_CHANNEL = 9
DROPPER_SERVO_PWM_RELEASE = 1900

CMD_VEL_TIMEOUT = 1.0  # seconds - zero thrust if the dashboard link goes quiet
SETPOINT_RATE_HZ = 20.0


class ManualBridge(Node):
    def __init__(self):
        super().__init__('manual_bridge')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.vel_pub = self.create_publisher(PositionTarget, '/mavros/setpoint_raw/local', 10)

        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, qos_profile
        )
        self.gripper_sub = self.create_subscription(
            String, '/gripper/command', self.gripper_callback, qos_profile
        )
        self.servo_client = self.create_client(CommandLong, '/mavros/cmd/command')

        self.target_surge = 0.0
        self.target_yaw_rate = 0.0
        self.last_cmd_vel_time = self.get_clock().now()

        self.cmd = PositionTarget()
        self.cmd.coordinate_frame = PositionTarget.FRAME_BODY_NED
        self.cmd.type_mask = (
            PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ |
            PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW
        )

        self.timer = self.create_timer(1.0 / SETPOINT_RATE_HZ, self.send_cmd)
        self.get_logger().info('[ManualBridge] Ready: /cmd_vel + /gripper/command -> MAVROS setpoints')

    def cmd_vel_callback(self, msg):
        self.target_surge = float(msg.linear.x)
        self.target_yaw_rate = float(msg.angular.z)
        self.last_cmd_vel_time = self.get_clock().now()

    def send_cmd(self):
        elapsed = (self.get_clock().now() - self.last_cmd_vel_time).nanoseconds / 1e9
        if elapsed > CMD_VEL_TIMEOUT:
            # Fail-safe: dashboard link went quiet, stop the vehicle
            self.cmd.velocity.x = 0.0
            self.cmd.yaw_rate = 0.0
        else:
            self.cmd.velocity.x = self.target_surge
            self.cmd.velocity.y = 0.0
            self.cmd.yaw_rate = self.target_yaw_rate

        self.vel_pub.publish(self.cmd)

    def gripper_callback(self, msg):
        command = msg.data.strip().upper()
        if command in ('OPEN', 'RELEASE'):
            self._actuate_servo(DROPPER_SERVO_PWM_RELEASE)
        else:
            self.get_logger().warn(
                f'[ManualBridge] Gripper command "{command}" has no calibrated servo action, ignoring.'
            )

    def _actuate_servo(self, pwm):
        if not self.servo_client.service_is_ready():
            self.get_logger().warn('[ManualBridge] /mavros/cmd/command not ready, dropping servo command.')
            return
        request = CommandLong.Request()
        request.broadcast = False
        request.command = MAV_CMD_DO_SET_SERVO
        request.confirmation = 0
        request.param1 = float(DROPPER_SERVO_CHANNEL)
        request.param2 = float(pwm)
        self.servo_client.call_async(request)
        self.get_logger().info(f'[ManualBridge] Servo dropper actuated at PWM {pwm}')


def main():
    rclpy.init()
    node = ManualBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
