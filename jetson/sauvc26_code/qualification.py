#!/usr/bin/env python3
"""
SAUVC 2026 High-Performance Racing Autonomous Qualification Node
Features:
- High-speed sprint & dive-on-the-fly (zero wasted startup delay)
- Fast-response PID controllers with derivative damping & anti-windup
- Aggressive 180° hairpin racing U-turn (2.2 rad/s with active counter-braking)
- Dynamic pure pursuit gate alignment
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import time
import math
import sys
import json

from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import String
from mavros_msgs.msg import PositionTarget
from sauvc26_code.pid import PID

# ═════════════════════════════════════════════════════════════════════════════
# CALIBRATED QUALIFICATION SPEED & STABILITY PROFILE
# ═════════════════════════════════════════════════════════════════════════════
ROTATE_SPEED = 1.05          # rad/s - Stable, controlled yaw rotation
FORWARD_SPEED_RACE = 0.90    # m/s   - Smooth cruise through gate
FORWARD_SPEED_SCAN = 0.75    # m/s   - Stable scan velocity
FORWARD_DURATION_GATE = 4.2  # s     - Optimized sprint duration across gate

# Tuned High-Stability PID Gains
KP_DEPTH = 0.75
KI_DEPTH = 0.12
KD_DEPTH = 0.30
TARGET_DEPTH = -0.85         # Centered in 1.5m gate opening (NED frame: negative or positive based on setup)

KP_GATE = 0.75
KI_GATE = 0.10
KD_GATE = 0.28

class GuidedMove(Node):
    def __init__(self):
        super().__init__('qualification_racing')

        # Publisher velocity use PositionTarget for body frame
        self.vel_pub = self.create_publisher(
            PositionTarget,
            '/mavros/setpoint_raw/local',
            10
        )
        
        # QoS profile for MAVROS
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.pose_callback,
            qos_profile
        )
        self.coord_sub = self.create_subscription(
            String,
            '/yolo_target_coord',
            self.coord_callback,
            qos_profile
        )
        
        # PositionTarget
        self.cmd = PositionTarget()
        self.cmd.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        self.cmd.type_mask = (
            PositionTarget.IGNORE_PX | 
            PositionTarget.IGNORE_PY | 
            PositionTarget.IGNORE_PZ |
            PositionTarget.IGNORE_AFX |
            PositionTarget.IGNORE_AFY |
            PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW
        )
                
        # Fast PID controllers with limits
        self.depth_pid = PID(kp=KP_DEPTH, ki=KI_DEPTH, kd=KD_DEPTH, setpoint=TARGET_DEPTH, output_limits=(-0.5, 0.5))
        self.gate_pid = PID(kp=KP_GATE, ki=KI_GATE, kd=KD_GATE, setpoint=0.0, output_limits=(-0.65, 0.65))
        
        # 20 Hz update loop for responsive racing control
        self.timer = self.create_timer(0.05, self.send_cmd)
        
        # State machine
        self.state = 1
        self.prev_state = 1
        self.state_start_time = self.get_clock().now()
        
        # Rotation & Trajectory tracking
        self.current_pose = None
        self.initial_yaw = None
        self.target_yaw = None
        self.previous_yaw_rate = 0.0
        self.max_yaw_acceleration = 0.35  # Smooth, stable acceleration limit
        
        # Gate tracking
        self.gate_coord = None
        self.last_gate_coord = None
        self.last_gate_time = None
        self.close_to_gate = False
        self.deadzone_gate = False
        self.forward_to_gate = False
        
        self.change_state(1)
        self.get_logger().info('🚀 RACING QUALIFICATION NODE INITIALIZED (Calibrated Smooth Mode)')

    def pose_callback(self, msg):
        self.current_pose = msg
    
    def coord_callback(self, msg):
        try:
            detections = json.loads(msg.data)
            current_time = self.get_clock().now()
            self.gate_coord = None
            
            for detection in detections:
                class_name = detection.get('class', '')
                point = Point()
                point.x = detection.get('x', 0.0)
                point.y = detection.get('y', 0.0)
                point.z = detection.get('z', 0.0)
                
                if class_name.lower() == 'gate':
                    self.gate_coord = point
                    self.last_gate_time = current_time
                    
        except Exception as e:
            self.get_logger().warn(f'Error parsing detections: {e}')

    def get_yaw(self):
        if self.current_pose is None:
            return 0.0
        q = self.current_pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def normalize_angle(self, angle):
        while angle > math.pi: angle -= 2 * math.pi
        while angle < -math.pi: angle += 2 * math.pi
        return angle
        
    def surface(self):
        self.cmd.velocity.x = 0.5  # Keep moving toward dock while surfacing
        self.cmd.velocity.y = 0.0
        self.cmd.velocity.z = 0.55 # Full upward heave speed
        self.cmd.yaw_rate = 0.0

    def rotate(self, yaw_rate):
        self.cmd.yaw_rate = yaw_rate
        
    def forward(self, speed, sway=0.0):
        if self.current_pose is None:
            return
        yaw = self.get_yaw()
        # Body to World velocity transformation
        self.cmd.velocity.x = speed * math.cos(yaw) - sway * math.sin(yaw)
        self.cmd.velocity.y = speed * math.sin(yaw) + sway * math.cos(yaw)

    def change_state(self, new_state):
        self.reset()
        self.initial_yaw = None
        self.previous_yaw_rate = 0.0
        self.close_to_gate = False
        self.deadzone_gate = False

        self.prev_state = self.state
        self.state = new_state
        self.state_start_time = self.get_clock().now()
        
        state_names = {
            1: '⚡ RACING DIVE & SPRINT',
            2: '🏁 FULL THROTTLE GATE PASS',
            3: '🔄 180° HANDBRAKE U-TURN',
            4: '🎯 VECTOR GATE TRACKING',
            5: '🏆 SURFACING AT DOCK'
        }
        self.get_logger().info(f'State Changed -> {state_names.get(new_state, new_state)}')
    
    def reset(self):
        self.cmd.velocity.x = 0.0
        self.cmd.velocity.y = 0.0
        self.cmd.velocity.z = 0.0
        self.cmd.yaw_rate = 0.0
        self.previous_yaw_rate = 0.0
        
    def maintain_depth(self):
        if self.current_pose is not None:
            current_z = self.current_pose.pose.position.z
            z_velocity = self.depth_pid.compute(current_z)
            self.cmd.velocity.z = max(-0.55, min(0.55, z_velocity))
            
    def track_gate(self):
        if self.gate_coord is not None:
            gate_x = self.gate_coord.x
        else:
            if self.last_gate_coord is None:
                return
            gate_x = self.last_gate_coord.x
            
        deadzone = 0.03
        if abs(gate_x) < deadzone:
            gate_x = 0.0
            self.deadzone_gate = True
        else:
            self.deadzone_gate = False
        
        desired_yaw_rate = self.gate_pid.compute(gate_x)
        
        # Snappy rate limiting
        yaw_rate_diff = desired_yaw_rate - self.previous_yaw_rate
        max_change = self.max_yaw_acceleration * 0.05
        
        if abs(yaw_rate_diff) > max_change:
            yaw_rate = self.previous_yaw_rate + (max_change if yaw_rate_diff > 0 else -max_change)
        else:
            yaw_rate = desired_yaw_rate
        
        self.cmd.yaw_rate = yaw_rate
        self.previous_yaw_rate = yaw_rate

    def send_cmd(self):
        current_time = self.get_clock().now()
        
        match self.state:                
            case 1: # Racing Dive & Launch
                self.maintain_depth()
                # Dive-on-the-fly: launch forward immediately!
                self.forward(FORWARD_SPEED_SCAN * 0.85)
                if self.current_pose is not None and abs(self.current_pose.pose.position.z - TARGET_DEPTH) < 0.15:
                    self.change_state(2)
                    
            case 2: # Full Throttle Sprint
                self.maintain_depth()
                
                if self.gate_coord is not None and not self.forward_to_gate and self.prev_state != 3:
                    self.change_state(4)
                    return
                
                if (self.prev_state == 4 or self.prev_state == 3) and self.forward_to_gate:
                    elapsed = (current_time - self.state_start_time).nanoseconds / 1e9
                    if elapsed < FORWARD_DURATION_GATE:
                        self.forward(FORWARD_SPEED_RACE)
                    else:
                        if self.prev_state == 3:
                            self.change_state(5)
                        else:
                            self.change_state(3)
                else:
                    self.forward(FORWARD_SPEED_RACE)

            case 3: # Fast 180° Hairpin U-Turn with Counter-Braking
                self.maintain_depth()
                if self.current_pose is None:
                    return
                
                current_yaw = self.get_yaw()
                if self.initial_yaw is None:
                    self.initial_yaw = current_yaw
                    self.target_yaw = self.normalize_angle(self.initial_yaw - math.pi)
                
                error = self.normalize_angle(self.target_yaw - current_yaw)
                
                if abs(error) < math.radians(4.0):
                    # Lock heading and launch immediately!
                    self.change_state(2)
                else:
                    # Dynamic brake & snap rotation
                    if abs(error) < math.radians(25.0):
                        yaw_rate = error * 3.5  # Snappy proportional lock-in
                    else:
                        yaw_rate = ROTATE_SPEED if error > 0 else -ROTATE_SPEED
                    
                    yaw_rate = max(-ROTATE_SPEED, min(ROTATE_SPEED, yaw_rate))
                    self.rotate(yaw_rate)
                    
            case 4: # Vector Gate Pursuit
                self.maintain_depth()
                if self.gate_coord is not None:
                    self.last_gate_coord = self.gate_coord

                time_since_last = (current_time - self.last_gate_time).nanoseconds / 1e9 if self.last_gate_time else 999
                if time_since_last > 2.5:
                    self.last_gate_coord = None
                    self.change_state(2)
                    return
                
                if self.close_to_gate:
                    self.get_logger().info('🚀 Gate centered! Full throttle blitz!')
                    self.forward_to_gate = True
                    self.change_state(2)
                else:
                    self.track_gate()
                    self.forward(FORWARD_SPEED_RACE)
                    
                    if self.gate_coord is not None:
                        if self.gate_coord.z > 0.18:
                            self.close_to_gate = True
                    elif self.last_gate_coord is not None:
                        if self.last_gate_coord.z > 0.18:
                            self.close_to_gate = True
                    
            case 5: # Racing Surface
                self.surface()
                
        self.cmd.header.stamp = current_time.to_msg()
        self.cmd.header.frame_id = 'base_link'
        self.vel_pub.publish(self.cmd)

def main():
    sleep_duration = 0
    if len(sys.argv) > 1:
        try:
            sleep_duration = int(sys.argv[1])
            time.sleep(sleep_duration)
        except ValueError:
            pass
    
    rclpy.init()
    node = GuidedMove()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
