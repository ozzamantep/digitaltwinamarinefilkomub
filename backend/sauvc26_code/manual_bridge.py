#!/usr/bin/env python3
"""
Manual Control Bridge - Digital Twin Dashboard -> Real Vehicle (MAVROS/Pixhawk)

Translates dashboard teleop/gripper commands into MAVROS RC Override so the
operator can drive the physical AUV in MANUAL or STABILIZE mode — no GUIDED/EKF
position lock required.

RC Override channel mapping (standard ArduSub):
    Ch1 (idx 0): Pitch       - pass-through (0 = no override)
    Ch2 (idx 1): Roll        - pass-through
    Ch3 (idx 2): Throttle    - Heave  (linear.z)
    Ch4 (idx 3): Yaw         - angular.z
    Ch5 (idx 4): Forward     - Surge  (linear.x)
    Ch6 (idx 5): Lateral     - Sway   (linear.y)

PWM range: 1100 (full reverse) — 1500 (neutral) — 1900 (full forward)
Scale factor: ±1.0 input → ±400 PWM offset from neutral.

Run this INSTEAD OF final.py/qualification.py - control_lock.py enforces only
ONE control node at a time. Stop the other node first before running this.

Subscribes:
    /cmd_vel          (geometry_msgs/Twist)  - surge=linear.x, sway=linear.y,
                                               heave=linear.z, yaw=angular.z
    /gripper/command  (std_msgs/String)      - OPEN/RELEASE actuate servo
Publishes:
    /mavros/rc/override (mavros_msgs/OverrideRCIn) - RC channel override @ 20 Hz
Calls:
    /mavros/cmd/command (mavros_msgs/CommandLong)  - servo dropper actuation
"""
import sys
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import math
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float64MultiArray
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from mavros_msgs.msg import OverrideRCIn, RCOut
from mavros_msgs.srv import CommandLong, SetMode, StreamRate

# Ball dropper servo
MAV_CMD_DO_SET_SERVO   = 183
DROPPER_SERVO_CHANNEL  = 9
DROPPER_SERVO_PWM_RELEASE = 1900

# RC PWM constants
PWM_NEUTRAL = 1500
PWM_MIN     = 1100
PWM_MAX     = 1900
PWM_SCALE   = 400        # ±1.0 input → ±400 PWM offset
RC_PASSTHROUGH = 0       # 0 = do not override this channel

CMD_VEL_TIMEOUT  = 1.0   # seconds — zero thrust if dashboard link goes quiet
SETPOINT_RATE_HZ = 20.0


def to_pwm(value: float) -> int:
    """Convert normalised [-1, +1] to ArduSub PWM [1100, 1900]."""
    return max(PWM_MIN, min(PWM_MAX, int(PWM_NEUTRAL + value * PWM_SCALE)))


class ManualBridge(Node):
    def __init__(self):
        super().__init__('manual_bridge')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # RC override publisher (works in MANUAL / STABILIZE — no GUIDED needed)
        self.rc_pub = self.create_publisher(OverrideRCIn, '/mavros/rc/override', 10)
        # Thruster feedback publisher for Digital Twin 3D visualization and dashboard
        self.thruster_pub = self.create_publisher(Float64MultiArray, '/thruster_outputs', 10)
        # Odometry publisher for digital twin 3D arena positioning
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # Subscribe to actual motor PWM outputs from Pixhawk ESCs for 100% genuine thruster sync
        self.latest_rc_out = None
        self.rc_out_sub = self.create_subscription(
            RCOut, '/mavros/rc/out', self.rc_out_callback, qos
        )

        # Subscribe to Pixhawk IMU for Active Auto-Leveling (Self-Righting)
        self.current_roll = 0.0
        self.current_pitch = 0.0
        self.current_yaw = 0.0
        self.gyro_x = 0.0
        self.gyro_y = 0.0
        self.latest_imu_quat = None
        self.imu_sub = self.create_subscription(
            Imu, '/mavros/imu/data', self.imu_callback, qos
        )

        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, qos
        )
        self.gripper_sub = self.create_subscription(
            String, '/gripper/command', self.gripper_callback, qos
        )
        self.servo_client = self.create_client(CommandLong, '/mavros/cmd/command')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.stream_rate_client = self.create_client(StreamRate, '/mavros/set_stream_rate')

        self.target_surge = 0.0
        self.target_sway  = 0.0
        self.target_heave = 0.0
        self.target_yaw   = 0.0
        self.last_cmd_vel_time = self.get_clock().now()

        # Dead-reckoning position state for digital twin
        self.sim_x = -6.0
        self.sim_y = 0.0
        self.sim_depth = 0.85
        self.sim_heading = 0.0

        # Auto-leveling configuration
        self.auto_level_enabled = True
        self._stabilize_requested = False
        self._stream_requested = False
        self.create_timer(1.0, self._request_stream_rate)
        self.create_timer(1.5, self._request_stabilize_mode)

        self.timer = self.create_timer(1.0 / SETPOINT_RATE_HZ, self.send_cmd)
        self.get_logger().info(
            '[ManualBridge] Ready (RC Override + Active Auto-Leveling):\n'
            '  Auto-Leveling: ACTIVATED (Ch1=Pitch PID, Ch2=Roll PID -> Auto-Straighten Hull)\n'
            '  Thrusters: Ch3=Heave  Ch4=Yaw  Ch5=Surge  Ch6=Sway\n'
            '  Synchronizing Pixhawk IMU -> /odom & /thruster_outputs for Digital Twin'
        )

    # ------------------------------------------------------------------
    def _request_stream_rate(self):
        if self._stream_requested:
            return
        if self.stream_rate_client.service_is_ready():
            req = StreamRate.Request()
            req.stream_id = 2  # STREAM_RC_CHANNELS (SERVO_OUTPUT_RAW)
            req.message_rate = 30
            req.on_off = True
            self.stream_rate_client.call_async(req)
            self._stream_requested = True
            self.get_logger().info('[ManualBridge] Requested Pixhawk SERVO_OUTPUT_RAW @ 30 Hz via MAVROS')

    # ------------------------------------------------------------------
    def _request_stabilize_mode(self):
        if self._stabilize_requested:
            return
        if self.set_mode_client.service_is_ready():
            req = SetMode.Request()
            req.custom_mode = 'STABILIZE'
            self.set_mode_client.call_async(req)
            self._stabilize_requested = True
            self.get_logger().info('[ManualBridge] Requested ArduSub STABILIZE mode for native gyro auto-leveling')

    # ------------------------------------------------------------------
    def imu_callback(self, msg: Imu):
        self.latest_imu_quat = msg.orientation
        q = msg.orientation

        # Roll (X-axis)
        sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        self.current_roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (Y-axis)
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        self.current_pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)

        # Yaw (Z-axis)
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

        self.gyro_x = float(msg.angular_velocity.x)
        self.gyro_y = float(msg.angular_velocity.y)

    # ------------------------------------------------------------------
    def rc_out_callback(self, msg: RCOut):
        if msg.channels and len(msg.channels) >= 6:
            # Guard: Pixhawk outputs 0 for disabled channels — treat as neutral 1500
            raw_channels = [int(ch) if int(ch) > 0 else PWM_NEUTRAL for ch in msg.channels[:6]]
            self.latest_rc_out = raw_channels
            # Directly forward to /thruster_outputs for Digital Twin frontend
            efforts = []
            for pwm_val in self.latest_rc_out:
                val = (float(pwm_val) - float(PWM_NEUTRAL)) / float(PWM_SCALE)
                efforts.append(max(-1.0, min(1.0, val)))
            th_msg = Float64MultiArray()
            th_msg.data = efforts
            self.thruster_pub.publish(th_msg)
            self.get_logger().info(
                f'[ManualBridge] ⚡ ESC PWM={self.latest_rc_out} → efforts={[round(e,2) for e in efforts]}',
                throttle_duration_sec=1.0
            )

    # ------------------------------------------------------------------
    def cmd_vel_callback(self, msg: Twist):
        self.target_surge = float(msg.linear.x)
        self.target_sway  = float(msg.linear.y)
        self.target_heave = float(msg.linear.z)
        self.target_yaw   = float(msg.angular.z)
        self.last_cmd_vel_time = self.get_clock().now()

    # ------------------------------------------------------------------
    def send_cmd(self):
        elapsed = (self.get_clock().now() - self.last_cmd_vel_time).nanoseconds / 1e9
        timeout = elapsed > CMD_VEL_TIMEOUT
        dt = 1.0 / SETPOINT_RATE_HZ

        rc = OverrideRCIn()
        channels = [RC_PASSTHROUGH] * 18

        # Closed-Loop Active Auto-Leveling (Self-Righting PID Controller)
        # Calculates restoring moments to bring Pitch -> 0.0 and Roll -> 0.0
        pitch_effort = 0.0
        roll_effort  = 0.0
        if self.auto_level_enabled and self.latest_imu_quat is not None:
            # Tuned proportional + derivative attitude controller
            kp_pitch = 1.70
            kd_pitch = 0.22
            kp_roll  = 1.70
            kd_roll  = 0.22

            # Deadband 0.5 degrees
            if abs(self.current_pitch) > 0.008 or abs(self.gyro_y) > 0.02:
                pitch_effort = max(-0.85, min(0.85, -self.current_pitch * kp_pitch - self.gyro_y * kd_pitch))
            if abs(self.current_roll) > 0.008 or abs(self.gyro_x) > 0.02:
                roll_effort  = max(-0.85, min(0.85, -self.current_roll * kp_roll - self.gyro_x * kd_roll))

        if timeout:
            channels[0] = to_pwm(pitch_effort)  # Ch1 Pitch: Keep auto-leveling even when pilot is idle!
            channels[1] = to_pwm(roll_effort)   # Ch2 Roll:  Keep auto-leveling even when pilot is idle!
            channels[2] = PWM_NEUTRAL           # Ch3 Heave
            channels[3] = PWM_NEUTRAL           # Ch4 Yaw
            channels[4] = PWM_NEUTRAL           # Ch5 Surge
            channels[5] = PWM_NEUTRAL           # Ch6 Sway
            surge = 0.0
            sway  = 0.0
            heave = 0.0
            yaw   = 0.0
        else:
            channels[0] = to_pwm(pitch_effort)        # Ch1: Pitch auto-leveling
            channels[1] = to_pwm(roll_effort)         # Ch2: Roll auto-leveling
            channels[2] = to_pwm(self.target_heave)   # Ch3: Throttle / Heave
            channels[3] = to_pwm(self.target_yaw)     # Ch4: Yaw
            channels[4] = to_pwm(self.target_surge)   # Ch5: Forward / Surge
            channels[5] = to_pwm(self.target_sway)    # Ch6: Lateral / Sway
            surge = self.target_surge
            sway  = self.target_sway
            heave = self.target_heave
            yaw   = self.target_yaw

        rc.channels = channels
        self.rc_pub.publish(rc)

        # 1. Publish 6-thruster normalized output (-1.0 to 1.0)
        th_msg = Float64MultiArray()
        if self.latest_rc_out and len(self.latest_rc_out) >= 6:
            # 100% Genuine ESC PWM from Pixhawk Motor Matrix (/mavros/rc/out)
            efforts = []
            for pwm_val in self.latest_rc_out:
                val = (float(pwm_val) - float(PWM_NEUTRAL)) / float(PWM_SCALE)
                efforts.append(max(-1.0, min(1.0, val)))
            th_msg.data = efforts
        else:
            # High-precision TAM Fallback with Pitch & Roll Auto-Leveling
            t1 = max(-1.0, min(1.0, surge + yaw - sway + roll_effort * 0.4))  # Front-Left (45°)
            t2 = max(-1.0, min(1.0, surge - yaw + sway - roll_effort * 0.4))  # Front-Right (-45°)
            t3 = max(-1.0, min(1.0, surge + yaw + sway + roll_effort * 0.4))  # Rear-Left (135°)
            t4 = max(-1.0, min(1.0, surge - yaw - sway - roll_effort * 0.4))  # Rear-Right (-135°)
            t5 = max(-1.0, min(1.0, heave - pitch_effort))                    # Front-Vertical (Lifts/drops nose)
            t6 = max(-1.0, min(1.0, heave + pitch_effort))                    # Rear-Vertical (Drops/lifts tail)
            th_msg.data = [float(t1), float(t2), float(t3), float(t4), float(t5), float(t6)]
        self.thruster_pub.publish(th_msg)

        # 2. Dead-reckoning position integration for Digital Twin 3D view
        current_heading = self.current_yaw if self.latest_imu_quat is not None else self.sim_heading
        if not timeout and (abs(surge) > 0.02 or abs(sway) > 0.02 or abs(yaw) > 0.02 or abs(heave) > 0.02):
            self.sim_heading = (self.sim_heading + yaw * 0.85 * dt) % (math.pi * 2)
            cos_h = math.cos(current_heading)
            sin_h = math.sin(current_heading)
            dx = (surge * 1.1 * cos_h - sway * 0.6 * sin_h) * dt
            dy = (surge * 1.1 * sin_h + sway * 0.6 * cos_h) * dt
            self.sim_x = max(-12.0, min(12.0, self.sim_x + dx))
            self.sim_y = max(-5.5, min(5.5, self.sim_y + dy))
            self.sim_depth = max(0.1, min(2.0, self.sim_depth + heave * 0.4 * dt))

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = float(self.sim_x)
        odom.pose.pose.position.y = float(self.sim_y)
        odom.pose.pose.position.z = -float(self.sim_depth)
        if self.latest_imu_quat is not None:
            odom.pose.pose.orientation = self.latest_imu_quat
        else:
            odom.pose.pose.orientation.z = math.sin(self.sim_heading / 2.0)
            odom.pose.pose.orientation.w = math.cos(self.sim_heading / 2.0)
        odom.twist.twist.linear.x = float(surge * 1.1)
        odom.twist.twist.linear.y = float(sway * 0.6)
        odom.twist.twist.linear.z = float(heave * 0.4)
        odom.twist.twist.angular.z = float(yaw * 0.85)
        self.odom_pub.publish(odom)

    # ------------------------------------------------------------------
    def gripper_callback(self, msg: String):
        command = msg.data.strip().upper()
        if command in ('OPEN', 'RELEASE'):
            self._actuate_servo(DROPPER_SERVO_PWM_RELEASE)
        else:
            self.get_logger().warn(
                f'[ManualBridge] Gripper command "{command}" has no calibrated '
                f'servo action — ignoring (only OPEN/RELEASE actuate the servo).'
            )

    def _actuate_servo(self, pwm: int):
        if not self.servo_client.service_is_ready():
            self.get_logger().warn(
                '[ManualBridge] /mavros/cmd/command not ready, dropping servo command.'
            )
            return
        req = CommandLong.Request()
        req.broadcast     = False
        req.command       = MAV_CMD_DO_SET_SERVO
        req.confirmation  = 0
        req.param1        = float(DROPPER_SERVO_CHANNEL)
        req.param2        = float(pwm)
        self.servo_client.call_async(req)
        self.get_logger().info(f'[ManualBridge] Servo dropper → PWM {pwm}')


# ======================================================================
def main():
    # Add backend/ to path so sauvc26_code package is found — done here
    # (after ROS2 imports) so stub packages in backend/ don't shadow real modules.
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
    from sauvc26_code.control_lock import acquire_control_lock
    try:
        acquire_control_lock('manual_bridge')
    except RuntimeError as e:
        print(f'[FATAL] {e}')
        return

    rclpy.init()
    node = ManualBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
