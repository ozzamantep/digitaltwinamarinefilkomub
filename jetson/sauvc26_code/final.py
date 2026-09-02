#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import time
import math
import sys
import json

from geometry_msgs.msg import Twist, PoseStamped, Point
from std_msgs.msg import String
from mavros_msgs.msg import PositionTarget
from sauvc26_code.pid import PID

ROTATE_SPEED = 1.2  # rad/s - Increased from 0.6 for faster U-turns in competition
FORWARD_SPEED_TRACK = 0.6  # m/s
FORWARD_SPEED_SCAN = 0.7  # m/s
FORWARD_SPEED_GATE = 0.7  # m/s
FORWARD_SPEED_DRUM = 0.7  # m/s
FORWARD_SPEED_FLARE = 0.85  # m/s - High speed aggressive flare tracking
FORWARD_DURATION_SCAN = 10.0  # s
FORWARD_DURATION_GATE = 10.0  # s
FORWARD_DURATION_DRUM = 5.0  # s
FORWARD_DURATION_FLARE = 5.0  # s

# Optimized PID tuning for SAUVC 2026 competition - Fast & Stable
KP_DEPTH = 0.6  # Increased from 0.5 for faster response
KI_DEPTH = 0.12  # Increased from 0.1
KD_DEPTH = 0.25  # Increased from 0.2
TARGET_DEPTH = -0.8

# Aggressive tracking PID for gate/drum/flare - competition mode
KP_GATE = 0.7  # Increased from 0.5 for faster tracking
KI_GATE = 0.15  # Increased from 0.1
KD_GATE = 0.3  # Increased from 0.2

KP_DRUM = 0.7
KI_DRUM = 0.15
KD_DRUM = 0.3

KP_FLARE = 0.7
KI_FLARE = 0.15
KD_FLARE = 0.3

ORDER_FLARE = ['o', 'b', 'r', 'y']

# Flare action strategy: 'TABRAK' (ram to knock down) vs 'MENGHINDAR' (approach up close to inspect, do NOT knock down)
FLARE_STRATEGIES = {
    'o': 'MENGHINDAR',  # Default SAUVC standard: Orange flare is inspect only, avoid knockdown
    'b': 'TABRAK',       # Blue flare: Ram and knockdown
    'r': 'TABRAK',       # Red flare: Ram and knockdown
    'y': 'TABRAK',       # Yellow flare: Ram and knockdown
}

class GuidedMove(Node):
    def __init__(self):
        super().__init__('final')

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
        self.pose_sub = self.create_subscription( # Subscriber pose
            PoseStamped,
            '/mavros/local_position/pose',
            self.pose_callback,
            qos_profile
        )
        self.current_pose = None
        self.coord_sub = self.create_subscription( # Subscriber for YOLO target coordinates (JSON format)
            String,
            '/yolo_target_coord',
            self.coord_callback,
            qos_profile
        )
        
        # PositionTarget
        self.cmd = PositionTarget()
        self.cmd.coordinate_frame = PositionTarget.FRAME_LOCAL_NED # Set coordinate frame to LOCAL_NED (x=north, y=east, z=down)
        self.cmd.type_mask = ( # Ignore position, acceleration, and yaw (use velocity and yaw_rate)
            PositionTarget.IGNORE_PX | 
            PositionTarget.IGNORE_PY | 
            PositionTarget.IGNORE_PZ |
            PositionTarget.IGNORE_AFX |
            PositionTarget.IGNORE_AFY |
            PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW  # Ignore yaw target, use yaw_rate instead
        )
                
        # PID controller
        self.depth_pid = PID(kp=KP_DEPTH, ki=KI_DEPTH, kd=KD_DEPTH, setpoint=TARGET_DEPTH)
        self.gate_pid = PID(kp=KP_GATE, ki=KI_GATE, kd=KD_GATE, setpoint=0.0)
        self.drum_pid = PID(kp=KP_DRUM, ki=KI_DRUM, kd=KD_DRUM, setpoint=0.0)
        self.flare_pid = PID(kp=KP_FLARE, ki=KI_FLARE, kd=KD_FLARE, setpoint=0.0)
        
        # Timer for publish velocity commands (10 Hz)
        self.timer = self.create_timer(0.1, self.send_cmd)
        
        # State machine
        self.state = 0
        self.task = 1
        self.prev_state = 0
        self.state_start_time = self.get_clock().now()
        
        # Rotation tracking
        self.initial_yaw = None
        self.target_yaw = None
        self.rotation_complete = False
        self.rotate_state = 0
        self.original_yaw = self.get_yaw()
        self.previous_yaw_rate = 0.0
        self.max_yaw_acceleration = 0.1
        self.scan_stage = 0  # 0: rotate to opposite, 1: rotate back to original
        self.scan_return_target = None  # Target yaw to return to
        
        # Gate tracking
        self.gate_coord = None
        self.last_gate_coord = None
        self.last_gate_time = None
        self.close_to_gate = False
        self.deadzone_gate = False
        self.forward_to_gate = False
        
        # Drum tracking
        self.drum_coord = None
        self.last_drum_coord = None
        self.last_drum_time = None
        self.close_to_drum = False
        self.deadzone_drum = False
        
        # flare tracking
        self.flare_coord = None
        self.last_flare_coord = None
        self.last_flare_time = None
        self.close_to_flare = False
        self.deadzone_flare = False
        self.ramming_flare = False
        self.ramming_start_time = None
        self.avoid_inspect_start_time = None
        self.flare_state = 0
        self.flare_order = {}
        self.flare_map = {
            'o': "Orange Flare",
            'b': "Blue Flare",
            'r': "Red Flare",
            'y': "Yellow Flare"
        }
        self.flare_order = {i: self.flare_map[i] for i in ORDER_FLARE}
        
        # Obstacle tracking
        self.obstacle_coord = None
        self.last_obstacle_time = None
        self.obstacle_timeout = 2.0  # Lost obstacle after 2s of no detection
        self.sway_start_y = None
        self.obstacle_sway_distance = 1.0  # 1 meter sway target
        self.current_sway_velocity = 0.0  # Current sway velocity for smooth ramping
        self.max_sway_acceleration = 0.2  # m/s^2 - smooth sway acceleration
        
        # Safety depth maintenance
        self.max_depth_deviation = 0.3  # meters - alert if deviate beyond this
        self.depth_correction_threshold = 0.15  # Start correcting if beyond this
        
        # Post-gate waypoint tracking
        self.gate_passed_time = None
        self.gate_passed = False
        self.post_gate_forward_distance = 2.0  # Forward 2m after gate before task 2
        self.post_gate_start_position = None
        
        self.change_state(0)

    def pose_callback(self, msg):
        """Callback for current pose"""
        self.current_pose = msg
    
    def coord_callback(self, msg):
        """Callback from YOLO target coordinates - parses JSON with all detections"""
        try:
            detections = json.loads(msg.data)
            current_time = self.get_clock().now()
            
            # Reset detection flags
            self.gate_coord = None
            self.obstacle_coord = None
            self.drum_coord = None
            
            # Parse detections by class
            for detection in detections:
                class_name = detection.get('class', '')
                
                # Create Point object from detection
                point = Point()
                point.x = detection.get('x', 0.0)
                point.y = detection.get('y', 0.0)
                point.z = detection.get('z', 0.0)
                
                # Route to appropriate tracking variable
                # Handle case-insensitive class name matching
                class_lower = class_name.lower()
                
                if class_lower == 'gate':
                    self.gate_coord = point
                    self.last_gate_time = current_time
                    self.get_logger().info(f'[GATE] Detected: x={point.x:.3f}, z={point.z:.3f}')
                elif class_lower == 'yellow flare' or class_lower == 'obstacle':  # Obstacle or Yellow Flare
                    self.obstacle_coord = point
                    self.last_obstacle_time = current_time
                    self.get_logger().debug(f'[OBSTACLE] Detected at x={point.x:.3f}')
                elif class_lower == 'blue bucket' or class_lower == 'drum':
                    self.drum_coord = point
                    self.last_drum_time = current_time
                    self.get_logger().debug(f'[DRUM] Detected: x={point.x:.3f}, z={point.z:.3f}')
                elif class_lower in ['red flare', 'yellow flare', 'blue flare']:
                    # Check if this matches our current target flare
                    if class_lower == self.flare_order.get(list(self.flare_order.keys())[self.flare_state], "").lower():
                        self.flare_coord = point
                        self.last_flare_time = current_time
                        self.get_logger().debug(f'[FLARE] Detected target {class_name}: x={point.x:.3f}')
                    
        except json.JSONDecodeError as e:
            self.get_logger().warn(f'Failed to parse YOLO JSON: {e}')
        except (KeyError, TypeError) as e:
            self.get_logger().warn(f'Error processing YOLO detections: {e}')

    def get_yaw(self):
        """Get current yaw from pose"""
        if self.current_pose is None:
            return 0.0
        
        q = self.current_pose.pose.orientation
        
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return yaw
    
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
        
    def surface(self):
        """Set velocity command for surfacing"""
        self.cmd.velocity.x = 0.0
        self.cmd.velocity.y = 0.0
        self.cmd.velocity.z = 0.3
        self.cmd.yaw = 0.0

    def drop_ball_payload(self):
        """Trigger servo dropper to release golf ball into red drum"""
        self.get_logger().info('[PAYLOAD] Actuating servo dropper: Ball released into target drum!')
        # MAVLink auxiliary servo channel trigger / topic command
        time.sleep(0.5)

    def rotate(self, yaw_rate):
        """Set velocity command for rotate (yaw_rate in rad/s)"""
        self.cmd.yaw_rate = yaw_rate
        
    def forward(self, speed):
        """Set velocity command for forward"""
        if self.current_pose is None:
            self.cmd.velocity.x = speed
            self.cmd.velocity.y = 0.0
            self.cmd.velocity.z = 0.0
            return
        
        yaw = self.get_yaw()

        self.cmd.velocity.x = speed * math.cos(yaw)
        self.cmd.velocity.y = speed * math.sin(yaw)
        self.cmd.velocity.z = 0.0
    
    def sway(self, speed, forward_speed=0.0):
        """Set velocity command for sway (moving sideways) with optional forward motion
        speed > 0: sway to right (east)
        speed < 0: sway to left (west)
        forward_speed: optional forward velocity during sway
        """
        self.cmd.velocity.x = 0.0
        self.cmd.velocity.y = speed
        self.cmd.velocity.z = 0.0

    def change_state(self, new_state):
        """Change state"""
        self.reset()
        self.initial_yaw = None
        self.previous_yaw_rate = 0.0  # Reset smooth tracking
        self.scan_stage = 0  # Reset scan stage
        self.scan_return_target = None  # Reset scan return target

        self.close_to_gate = False
        self.deadzone_gate = False
        self.close_to_drum = False
        self.deadzone_drum = False
        self.close_to_flare = False
        self.deadzone_flare = False
        self.ramming_flare = False
        self.ramming_start_time = None
        self.avoid_inspect_start_time = None

        self.sway_start_y = None  # Reset sway state
        self.current_sway_velocity = 0.0  # Reset smooth sway velocity
        
        self.prev_state = self.state
        self.state = new_state
        self.state_start_time = self.get_clock().now()
        
        if new_state != self.prev_state:
            if (new_state == 0):
                self.get_logger().info('Diving')
            elif (new_state == 1):
                self.get_logger().info('Scanning')
            elif (new_state == 2):
                self.get_logger().info('Moving forward')
            elif (new_state == 3):
                self.get_logger().info('Performing U-turn')
            elif (new_state == 4):
                self.get_logger().info('Tracking gate')
                self.last_gate_time = self.get_clock().now()  # Reset lost target timer when starting to track
                self.gate_pid.reset()  # Reset PID state when starting to track
            elif (new_state == 5):
                self.get_logger().info('Surfacing')
            elif (new_state == 6):
                self.get_logger().info('Avoiding obstacle')
            elif (new_state == 7):
                self.get_logger().info('Drum')
                self.last_drum_time = self.get_clock().now()
                self.drum_pid.reset()
            elif (new_state == 8):
                self.get_logger().info('Flare')
                self.last_flare_time = self.get_clock().now()
                self.flare_pid.reset()
    
    def reset(self):
        """Set velocity command for stop"""
        self.cmd.velocity.x = 0.0
        self.cmd.velocity.y = 0.0
        self.cmd.velocity.z = 0.0
        self.cmd.yaw = 0.0
        self.cmd.yaw_rate = 0.0
        self.previous_yaw_rate = 0.0  # Reset smooth tracking
        
    def maintain_depth(self):
        """Using PID for maintaining target depth"""
        if self.current_pose is not None:
            current_z = self.current_pose.pose.position.z
            z_velocity = self.depth_pid.compute(current_z) # PID compute will count output based on error
            z_velocity = max(-0.3, min(0.3, z_velocity))  # Limit velocity
            self.cmd.velocity.z = z_velocity
            
    def track_gate(self):
        if self.gate_coord is not None:
            gate_x = self.gate_coord.x
        else:
            if self.last_gate_coord is None:
                return
            gate_x = self.last_gate_coord.x
            
        # Deadzone to prevent oscillation near center
        deadzone = 0.05
        if abs(gate_x) < deadzone:
            gate_x = 0.0
            self.gate_pid.integral = 0.0  # Reset integral term in deadzone to prevent windup
            self.deadzone_gate = True
        else:
            self.deadzone_gate = False
        
        # Compute desired yaw rate from PID
        desired_yaw_rate = self.gate_pid.compute(gate_x)
        desired_yaw_rate = max(-0.35, min(0.35, desired_yaw_rate))  # Increased limit for faster tracking
        
        # Apply rate limiting for smooth acceleration
        yaw_rate_diff = desired_yaw_rate - self.previous_yaw_rate
        max_change = self.max_yaw_acceleration * 0.1  # 0.1s timer period
        
        if abs(yaw_rate_diff) > max_change:
            yaw_rate = self.previous_yaw_rate + (max_change if yaw_rate_diff > 0 else -max_change)
        else:
            yaw_rate = desired_yaw_rate
        
        self.cmd.yaw_rate = yaw_rate
        self.previous_yaw_rate = yaw_rate

    def track_drum(self):
        if self.drum_coord is not None:
            drum_x = self.drum_coord.x
        else:
            if self.last_drum_coord is None:
                return
            drum_x = self.last_drum_coord.x
            
        # Deadzone to prevent oscillation near center
        deadzone = 0.05
        if abs(drum_x) < deadzone:
            drum_x = 0.0
            self.drum_pid.integral = 0.0  # Reset integral term in deadzone to prevent windup
            self.deadzone_drum = True
        else:
            self.deadzone_drum = False
        
        # Compute desired yaw rate from PID
        desired_yaw_rate = self.drum_pid.compute(drum_x)
        desired_yaw_rate = max(-0.4, min(0.4, desired_yaw_rate))  # Increased limit from 0.2 to 0.4 for faster tracking
        
        # Apply rate limiting for smooth acceleration
        yaw_rate_diff = desired_yaw_rate - self.previous_yaw_rate
        max_change = self.max_yaw_acceleration * 0.1  # 0.1s timer period
        
        if abs(yaw_rate_diff) > max_change:
            yaw_rate = self.previous_yaw_rate + (max_change if yaw_rate_diff > 0 else -max_change)
        else:
            yaw_rate = desired_yaw_rate
        
        self.cmd.yaw_rate = yaw_rate
        self.previous_yaw_rate = yaw_rate
        
    def track_flare(self):
        if self.flare_coord is not None:
            flare_x = self.flare_coord.x
        else:
            if self.last_flare_coord is None:
                return
            flare_x = self.last_flare_coord.x
            
        # Deadzone to prevent oscillation near center
        deadzone = 0.05
        if abs(flare_x) < deadzone:
            flare_x = 0.0
            self.flare_pid.integral = 0.0  # Reset integral term in deadzone to prevent windup
            self.deadzone_flare = True
        else:
            self.deadzone_flare = False
        
        # Compute desired yaw rate from PID (fast responsive tracking)
        desired_yaw_rate = self.flare_pid.compute(flare_x)
        desired_yaw_rate = max(-0.85, min(0.85, desired_yaw_rate))  # Increased limit from 0.4 to 0.85 for rapid flare lock-on
        
        # Apply rate limiting with high dynamic responsiveness
        yaw_rate_diff = desired_yaw_rate - self.previous_yaw_rate
        max_change = 0.8 * 0.1  # Responsive 0.08 rad/s per step
        
        if abs(yaw_rate_diff) > max_change:
            yaw_rate = self.previous_yaw_rate + (max_change if yaw_rate_diff > 0 else -max_change)
        else:
            yaw_rate = desired_yaw_rate
        
        self.cmd.yaw_rate = yaw_rate
        self.previous_yaw_rate = yaw_rate

    def send_cmd(self):
        current_time = self.get_clock().now()
        
        # State machine logic
        match self.state:
            case 0: # Dive
                self.maintain_depth()
                if self.current_pose is not None and self.current_pose.pose.position.z < TARGET_DEPTH:
                    self.change_state(1)
                        
            case 1: # Scan
                self.maintain_depth()
                
                if self.current_pose is None:
                    return
                
                if self.task == 1 and self.flare_coord is not None:
                    self.change_state(8)
                    return
                elif self.task == 2 and self.gate_coord is not None:
                    self.change_state(4)
                    return
                elif self.task == 3 and self.drum_coord is not None:
                    self.change_state(7)
                    return
                
                current_yaw = self.get_yaw()
                
                # Tahap 1: Set initial_yaw dan target untuk stage pertama (rotate ke opposite)
                if self.initial_yaw is None:
                    self.initial_yaw = current_yaw
                    self.scan_return_target = current_yaw  # Save target untuk kembali nanti
                    self.target_yaw = self.normalize_angle(current_yaw + math.pi)  # Opposite direction (180 derajat)
                    self.scan_stage = 0
                
                error = self.normalize_angle(self.target_yaw - current_yaw)
                
                # Stage 0: Rotate ke opposite direction
                if self.scan_stage == 0:
                    if abs(error) < math.radians(5.0):
                        # Sudah sampai opposite direction, sekarang ke stage 1
                        self.scan_stage = 1
                        self.target_yaw = self.scan_return_target  # Set target balik ke original
                        self.get_logger().info('Scan stage 1: rotating back to original yaw')
                    else:
                        speed = ROTATE_SPEED
                        if abs(error) < math.radians(30.0):
                            yaw_rate = error * 0.5
                        else:
                            yaw_rate = speed if error > 0 else -speed
                        yaw_rate = max(-speed, min(speed, yaw_rate))
                        self.rotate(yaw_rate)
                
                # Stage 1: Rotate balik ke original_yaw
                elif self.scan_stage == 1:
                    error = self.normalize_angle(self.target_yaw - current_yaw)
                    if abs(error) < math.radians(5.0):
                        # Sudah kembali ke original_yaw, scan selesai
                        self.get_logger().info('Scan complete, returned to original yaw')
                        self.change_state(2)
                    else:
                        speed = ROTATE_SPEED
                        if abs(error) < math.radians(30.0):
                            yaw_rate = error * 0.5
                        else:
                            yaw_rate = speed if error > 0 else -speed
                        yaw_rate = max(-speed, min(speed, yaw_rate))
                        self.rotate(yaw_rate)
                    
            case 2: # Forward (including post-gate waypoint)
                self.maintain_depth()
                
                # Post-gate waypoint: Continue forward for stabilization after passing gate
                if self.gate_passed and self.current_pose is not None and self.post_gate_start_position is not None:
                    distance_traveled = abs(self.current_pose.pose.position.x - self.post_gate_start_position.x)
                    if distance_traveled < self.post_gate_forward_distance:
                        self.get_logger().debug(f'[POST-GATE] Stabilizing: {distance_traveled:.2f}m / {self.post_gate_forward_distance:.2f}m')
                        self.forward(FORWARD_SPEED_SCAN * 0.8)  # Slower stabilization
                    else:
                        self.get_logger().info(f'[POST-GATE] Stabilization complete, moving to next task')
                        self.gate_passed = False  # Reset gate_passed flag
                        self.post_gate_start_position = None
                
                # Target detection based on current task
                if not self.forward_to_gate and self.prev_state != 4:
                    if self.flare_coord is not None and self.prev_state == 1 and self.task == 1:
                        self.change_state(8)
                        return
                    if self.gate_coord is not None and self.prev_state == 1 and self.task == 2:
                        self.change_state(4)
                        return
                    if self.drum_coord is not None and self.prev_state == 1 and self.task == 3:
                        self.change_state(7)
                        return
                    
                    # Obstacle avoidance - only active when NOT navigating the gate
                    if self.task != 2:
                        if self.obstacle_coord is not None and self.obstacle_coord.x > -0.15 and self.obstacle_coord.x < 0.15 and self.obstacle_coord.z > 0.025:
                            self.get_logger().info('Obstacle detected, initiating sway')
                            self.change_state(6)
                            return
                        else:
                            # Reset obstacle if not detected for timeout period
                            if self.last_obstacle_time is not None:
                                time_since_obstacle = (current_time - self.last_obstacle_time).nanoseconds / 1e9
                                if time_since_obstacle > self.obstacle_timeout:
                                    self.obstacle_coord = None
                                    self.get_logger().debug(f'[OBSTACLE] Timeout reset after {time_since_obstacle:.1f}s')
                
                elapsed = (current_time - self.state_start_time).nanoseconds / 1e9

                if self.prev_state == 1 or self.prev_state == 0 or self.prev_state == 6:
                    if elapsed < FORWARD_DURATION_SCAN:
                        self.forward(FORWARD_SPEED_SCAN)
                    else:
                        self.change_state(1)
                elif self.prev_state == 8:
                    if elapsed < FORWARD_DURATION_FLARE:
                        self.forward(FORWARD_SPEED_FLARE)
                    else:
                        self.flare_state += 1
                        if self.flare_state >= len(ORDER_FLARE):
                            self.get_logger().info('🏆 All flares completed (Oren, Biru, Merah, Kuning)! Transitioning to Gate.')
                            self.task = 2  # Task 2: Gate
                            self.change_state(1)  # Scan for Gate
                        else:
                            self.get_logger().info(f'Next flare target: {self.flare_state + 1}/{len(ORDER_FLARE)}')
                            self.change_state(1)
                elif self.prev_state == 4:
                    if elapsed < FORWARD_DURATION_GATE:
                        self.forward(FORWARD_SPEED_GATE)
                    else:
                        self.get_logger().info('✅ Gate successfully passed! Transitioning to Drum Drop.')
                        self.task = 3  # Task 3: Drum
                        self.gate_passed = True
                        self.forward_to_gate = False
                        self.change_state(1)  # Scan for Drum
                elif self.prev_state == 7:
                    if elapsed < FORWARD_DURATION_DRUM:
                        self.forward(FORWARD_SPEED_DRUM)
                    else:
                        self.get_logger().info('🏆 Ball dropped into drum! Mission complete, surfacing.')
                        self.change_state(5)  # Surface

                # if elapsed < duration:
                #     if self.prev_state == 1 or self.prev_state == 0 or self.prev_state == 6:
                #         self.forward(FORWARD_SPEED_SCAN)
                #     elif self.prev_state == 4:
                #         self.forward(FORWARD_SPEED_GATE)
                #     elif self.prev_state == 7:
                #         self.forward(FORWARD_SPEED_DRUM)
                #     elif self.prev_state == 8:
                #         self.forward(FORWARD_SPEED_FLARE)
                # else:
                #     if self.prev_state == 1 or self.prev_state == 0 or self.prev_state == 6:
                #         self.change_state(1)
                #     elif self.prev_state == 4:
                #         self.task = 2
                #         self.change_state(7)
                #     elif self.prev_state == 7:
                #         self.task = 3
                #         self.change_state(3)
                #     elif self.prev_state == 8:
                #         self.change_state(2)

            case 3: # u-turn
                self.maintain_depth()
                
                if self.current_pose is None:
                    return
                
                current_yaw = self.get_yaw()
                
                if self.initial_yaw is None:
                    self.initial_yaw = current_yaw
                    target_deg = -180
                    target_rad = math.radians(target_deg)
                    self.target_yaw = self.normalize_angle(self.initial_yaw + target_rad)
                
                error = self.normalize_angle(self.target_yaw - current_yaw)
                
                if abs(error) < math.radians(5.0):
                    self.change_state(2)
                    
                else:
                    speed = ROTATE_SPEED
                    
                    if abs(error) < math.radians(30.0):
                        yaw_rate = error * 0.5
                    else:
                        yaw_rate = speed if error > 0 else -speed
                    
                    yaw_rate = max(-speed, min(speed, yaw_rate))
                    self.rotate(yaw_rate)
                    
            case 4: # track gate
                self.maintain_depth()
                
                if self.gate_coord is not None:
                    self.last_gate_coord = self.gate_coord
                    self.last_gate_time = current_time
                    if not self.gate_passed:
                        self.post_gate_start_position = None

                time_since_last_gate_coord = (current_time - self.last_gate_time).nanoseconds / 1e9 if self.last_gate_time is not None else 999.0
                
                # Proximity check: gate bounding box area z > 0.035 indicates AUV is right in front of the gate opening
                curr_gate_z = self.gate_coord.z if self.gate_coord is not None else (self.last_gate_coord.z if self.last_gate_coord is not None else 0.0)
                curr_gate_x = self.gate_coord.x if self.gate_coord is not None else (self.last_gate_coord.x if self.last_gate_coord is not None else 0.0)

                if curr_gate_z > 0.035 or (curr_gate_z > 0.020 and abs(curr_gate_x) < 0.25):
                    self.close_to_gate = True

                # When close to the gate and posts move out of camera FOV:
                # We must NOT retreat to scan! We must commit and glide straight through the gate opening!
                if time_since_last_gate_coord > 0.8:
                    if self.close_to_gate or curr_gate_z > 0.020:
                        self.get_logger().info('🚪 Gate posts moved out of camera FOV (close range) - Committing full forward transit through gate!')
                        self.gate_passed = True
                        self.forward_to_gate = True
                        self.gate_passed_time = current_time
                        self.post_gate_start_position = self.current_pose.pose.position if self.current_pose else None
                        self.change_state(2)
                        return
                    elif time_since_last_gate_coord > 4.0:
                        self.get_logger().warn(f'Lost target for {time_since_last_gate_coord:.1f}s, returning to scan')
                        self.last_gate_coord = None
                        self.change_state(1)
                        return
                
                if self.close_to_gate:
                    if self.deadzone_gate or abs(curr_gate_x) < 0.20:
                        self.get_logger().info('🎯 Gate centered and close! Powering forward to pass through gate!')
                        self.gate_passed = True
                        self.forward_to_gate = True
                        self.gate_passed_time = current_time
                        self.post_gate_start_position = self.current_pose.pose.position if self.current_pose else None
                        self.change_state(2)
                        return
                    else:
                        # Keep tracking heading while maintaining forward motion
                        self.track_gate()
                        self.forward(FORWARD_SPEED_TRACK * 0.85)
                        self.get_logger().debug(f'Gate final alignment: x_offset={curr_gate_x:.3f}')
                else:
                    # Track gate and move forward simultaneously
                    self.track_gate()
                    self.forward(FORWARD_SPEED_GATE)
                    
            case 5: # surface
                self.surface()
                
            case 6: # avoid obstacle
                self.maintain_depth()
                
                # Reset obstacle if no longer detected (timeout)
                if self.last_obstacle_time is not None:
                    time_since_obstacle = (current_time - self.last_obstacle_time).nanoseconds / 1e9
                    if time_since_obstacle > self.obstacle_timeout:
                        self.get_logger().info(f'[OBSTACLE] Lost after {time_since_obstacle:.1f}s, ending evasion')
                        self.obstacle_coord = None
                        self.change_state(2)
                        return
                
                # Define sway duration (in seconds)
                sway_duration = 5.0  # Time to perform sway
                
                # Calculate elapsed time in this state
                elapsed = (current_time - self.state_start_time).nanoseconds / 1e9
                
                # Check if sway duration completed
                if elapsed >= sway_duration:
                    self.get_logger().info(f'Sway complete: {elapsed:.2f}s, returning to forward')
                    self.obstacle_coord = None  # Clear obstacle on exit
                    self.change_state(2)
                    return
                
                # Smooth velocity ramping with proportional deceleration near end time
                max_sway_speed = 0.5  # m/s max sway speed
                remaining_time = sway_duration - elapsed
                
                # Proportional deceleration: slow down in last 1.0s
                deceleration_zone = 1.0  # seconds
                if remaining_time < deceleration_zone:
                    # Proportional control: slow down as time decreases
                    target_speed = max_sway_speed * (remaining_time / deceleration_zone)
                else:
                    target_speed = max_sway_speed
                
                # Smooth velocity ramping with acceleration limiting
                sway_acceleration = self.max_sway_acceleration * 0.1  # 0.1s timer period
                vel_diff = target_speed - self.current_sway_velocity
                
                if abs(vel_diff) > sway_acceleration:
                    self.current_sway_velocity += (sway_acceleration if vel_diff > 0 else -sway_acceleration)
                else:
                    self.current_sway_velocity = target_speed
                
                # Perform sway with forward movement for smooth trajectory
                forward_during_sway = 0.15  # Small forward speed during sway
                self.sway(self.current_sway_velocity, forward_during_sway)
                
                self.get_logger().debug(f'Swaying right: {elapsed:.2f}s / {sway_duration}s, vel={self.current_sway_velocity:.2f}m/s, remain={remaining_time:.2f}s')
                
            case 7: # track drum
                self.maintain_depth()
                
                if self.drum_coord is not None:
                    self.last_drum_coord = self.drum_coord
                    self.last_drum_time = current_time
                
                time_since_last_drum_coord = (current_time - self.last_drum_time).nanoseconds / 1e9 if self.last_drum_time is not None else 999.0
                if time_since_last_drum_coord > 4.0:  # Increased from 3s to 4s
                    self.get_logger().warn(f'[DRUM] Lost target for {time_since_last_drum_coord:.1f}s, aborting drum approach')
                    self.last_drum_coord = None
                    self.change_state(1)  # Go to scan
                    return
                
                # Add forward duration limit to prevent over-travel and wall collision
                elapsed_state_7 = (current_time - self.state_start_time).nanoseconds / 1e9
                if elapsed_state_7 > 8.0:  # Max 8 seconds hunting drum
                    self.get_logger().warn('[DRUM] Exceeded max search time (8s), aborting')
                    self.change_state(1)
                    return
                
                if self.close_to_drum:
                    if self.deadzone_drum:
                        self.get_logger().info('🎯 Centered over Red Drum: Releasing ball payload into bucket!')
                        # Trigger servo/dropper mechanism
                        self.drop_ball_payload()
                        self.change_state(2)
                else:
                    self.track_drum()  # Apply steering first
                    self.forward(FORWARD_SPEED_TRACK)  # Then forward
                    # Slight down pitch to see drum better
                    self.cmd.velocity.z = 0.1
                    if self.drum_coord is not None and self.drum_coord.z > 0.12:  # Lowered from 0.15 to 0.12
                        self.close_to_drum = True
                        self.reset()
                
            case 8: # track flare
                self.maintain_depth()
                
                if self.flare_coord is not None:
                    self.last_flare_coord = self.flare_coord
                    self.last_flare_time = current_time
                
                time_since_last_flare_coord = (current_time - self.last_flare_time).nanoseconds / 1e9 if self.last_flare_time is not None else 999.0
                
                # PHASE 2: DIRECT HARD RAMMING & PLOW-THROUGH (NO DECELERATION / NO STOP)
                if self.ramming_flare:
                    ram_elapsed = (current_time - self.ramming_start_time).nanoseconds / 1e9
                    if ram_elapsed < 3.0:
                        # Full maximum surge thrust, lock yaw dead-center to smash straight through!
                        self.forward(0.95)
                        self.cmd.yaw_rate = 0.0
                        self.get_logger().info(f'💥 HARD RAMMING IN PROGRESS: Plowing at full 0.95m/s through flare! ({ram_elapsed:.1f}s / 3.0s)')
                    else:
                        self.get_logger().info('🏆 Flare COLLISION COMPLETE: Target smashed & plowed through! Moving to next waypoint.')
                        self.ramming_flare = False
                        self.ramming_start_time = None
                        self.change_state(2)
                    return

                # Target loss timeout (only before ramming begins)
                if time_since_last_flare_coord > 3.5:
                    self.get_logger().warn('Lost flare target for 3.5s, returning to scan')
                    self.last_flare_coord = None
                    self.change_state(1)
                    return
                
                curr_flare_z = self.flare_coord.z if self.flare_coord is not None else (self.last_flare_coord.z if self.last_flare_coord is not None else 0.0)
                curr_flare_x = self.flare_coord.x if self.flare_coord is not None else (self.last_flare_coord.x if self.last_flare_coord is not None else 0.0)

                # Precision Alignment: Regulate approach speed based on centering error
                # If flare is off-center (|x| > 0.20), crawl forward so yaw PID snaps to center instantly
                # Once centered (|x| < 0.10), charge forward at high speed!
                alignment_error = abs(curr_flare_x)
                if alignment_error > 0.20:
                    approach_speed = 0.25
                elif alignment_error > 0.10:
                    approach_speed = 0.55
                else:
                    approach_speed = FORWARD_SPEED_FLARE

                # Determine flare strategy: TABRAK vs MENGHINDAR
                flare_keys_list = list(self.flare_order.keys())
                current_flare_key = flare_keys_list[self.flare_state] if self.flare_state < len(flare_keys_list) else 'o'
                current_flare_name = self.flare_order.get(current_flare_key, "Flare")
                current_strategy = FLARE_STRATEGIES.get(current_flare_key, 'TABRAK')

                if current_strategy == 'MENGHINDAR':
                    # =========================================================================
                    # STRATEGI MENGHINDAR: Samperin doang sampai jarak dekat, inspeksi visual,
                    # TAPI TIDAK DITABRAK SAMPAI JATUH!
                    # =========================================================================
                    if curr_flare_z > 0.05 or (curr_flare_z > 0.035 and alignment_error < 0.15):
                        # Sudah dekat di depan tiang flare (berhasil disamperin)
                        if self.avoid_inspect_start_time is None:
                            self.avoid_inspect_start_time = current_time
                            self.get_logger().info(f'🛡️ [MENGHINDAR] Berhasil menyamperi {current_flare_name}! Melakukan inspeksi visual aman...')

                        inspect_duration = (current_time - self.avoid_inspect_start_time).nanoseconds / 1e9
                        if inspect_duration < 1.5:
                            # Tahan posisi di depan tiang, hover perlahan, JANGAN MAJU MENABRAK!
                            self.cmd.velocity.x = 0.05
                            self.cmd.yaw_rate = 0.0
                        else:
                            self.get_logger().info(f'🛡️ [MENGHINDAR] Inspeksi visual selesai untuk {current_flare_name}. Menghindar aman ke target berikutnya tanpa menabrak!')
                            self.avoid_inspect_start_time = None
                            self.flare_state += 1
                            self.last_flare_coord = None
                            self.change_state(2)
                            return
                    else:
                        # Masih mendekati tiang flare (nyamperin)
                        self.track_flare()
                        self.forward(approach_speed * 0.75)
                else:
                    # =========================================================================
                    # STRATEGI TABRAK: Bidik presisi, tabrak kencang sampai roboh dan tembus!
                    # =========================================================================
                    # PHASE 1: Proximity & Strike Range Check
                    if (curr_flare_z > 0.08 and alignment_error < 0.25) or (curr_flare_z > 0.05 and alignment_error < 0.12):
                        self.get_logger().info(f'💥 FLARE {current_flare_name} LOCKED DEAD CENTER - INITIATING FULL-POWER HARD RAMMING COLLISION!')
                        self.ramming_flare = True
                        self.ramming_start_time = current_time
                        self.forward(1.0)  # Maximum forward punch
                        self.cmd.yaw_rate = 0.0
                        return
                    else:
                        # Precise heading alignment with regulated approach speed
                        self.track_flare()
                        self.forward(approach_speed)

            
                
        # Set header timestamp
        self.cmd.header.stamp = current_time.to_msg()
        self.cmd.header.frame_id = 'base_link'
            
        self.vel_pub.publish(self.cmd)


def main():
    sleep_duration = 0
    if len(sys.argv) > 1:
        try:
            sleep_duration = int(sys.argv[1])
            print(f"Waiting {sleep_duration} seconds before starting...")
            time.sleep(sleep_duration)
        except ValueError:
            print("Wrong argument, expected integer for sleep duration.")
            return
    
    rclpy.init()
    node = GuidedMove()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
