#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import time
import math
import sys
import json

import os
# Add backend/ to path so sauvc26_code package is found — append (not insert)
# so real ROS2 packages from /opt/ros/humble always take priority over stubs.
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from geometry_msgs.msg import PoseStamped, Point
from sensor_msgs.msg import BatteryState, Imu, Range
from std_msgs.msg import Bool, String
from mavros_msgs.msg import PositionTarget
from mavros_msgs.srv import CommandLong
from sauvc26_code.collision_safety import CollisionSafety
from sauvc26_code.pid import PID

# ═════════════════════════════════════════════════════════════════════════════
# CALIBRATED RACING SPEED & STABLE MANEUVER CONSTANTS
# ═════════════════════════════════════════════════════════════════════════════
ROTATE_SPEED = 1.05          # rad/s - Stable, controlled yaw rotation
FORWARD_SPEED_TRACK = 0.75   # m/s   - High precision target tracking
FORWARD_SPEED_SCAN = 0.75    # m/s   - Smooth arena sweep
FORWARD_SPEED_GATE = 0.90    # m/s   - Controlled gate transit
FORWARD_SPEED_DRUM = 0.75    # m/s   - Smooth approach to target drum
FORWARD_SPEED_FLARE = 0.85   # m/s   - 💥 Direct hit ram into target flares
FORWARD_DURATION_SCAN = 3.5  # s     - Swift scan duration
FORWARD_DURATION_GATE = 4.0  # s     - Smooth gate transit
FORWARD_DURATION_DRUM = 1.5  # s     - Stable drum approach
FORWARD_DURATION_FLARE = 1.8 # s     - Reliable flare ramming

# Ball dropper servo (MAV_CMD_DO_SET_SERVO via /mavros/cmd/command)
MAV_CMD_DO_SET_SERVO = 183
DROPPER_SERVO_CHANNEL = 9     # AUX output channel - calibrate to actual wiring
DROPPER_SERVO_PWM_RELEASE = 1900  # PWM for open/release position - calibrate on bench

# Precision drum drop: must stay centered and fully stopped this long before releasing
DRUM_HOVER_CONFIRM_SEC = 0.8
# Post-drop inspection orbit: circles the drop point once so every side is visible on
# camera. There is no trained vision class for "ball in drum" yet, so a miss cannot be
# detected automatically - this only brings the drum into view for operator confirmation.
# The dropper servo is release-only (no calibrated grasp position), so automatic
# retrieval of a missed ball is not possible with the current hardware.
DRUM_ORBIT_RADIUS = 0.6       # m - inspection loop radius around the drop point
DRUM_ORBIT_SPEED = 0.35       # m/s - tangential speed while orbiting
DRUM_ORBIT_RADIUS_KP = 0.8    # proportional gain correcting radius drift

# Tuned High-Stability PID Gains
KP_DEPTH = 0.75
KI_DEPTH = 0.12
KD_DEPTH = 0.30
TARGET_DEPTH = -0.85

# Smooth Tracking PID Gains
KP_GATE = 0.75
KI_GATE = 0.10
KD_GATE = 0.28

KP_DRUM = 0.75
KI_DRUM = 0.10
KD_DRUM = 0.28

KP_FLARE = 0.80
KI_FLARE = 0.12
KD_FLARE = 0.30

ORDER_FLARE = ['o', 'b', 'r', 'y']

# Flare action strategy: 'TABRAK' (ram to knock down) vs 'MENGHINDAR' (approach up close to inspect, do NOT knock down)
FLARE_STRATEGIES = {
    'o': 'MENGHINDAR',  # SAUVC standard: Orange flare is inspect only, avoid knockdown
    'b': 'TABRAK',       # Blue flare: Ram and knockdown
    'r': 'TABRAK',       # Red flare: Ram and knockdown
    'y': 'TABRAK',       # Yellow flare: Ram and knockdown
}

# Human-readable state labels, mirrored to the Digital Twin via /mission_state for 1:1 FSM sync
STATE_LABELS = {
    0: 'Dive & Launch',
    1: 'Scanning',
    2: 'Forward Transit',
    3: 'U-Turn',
    4: 'Gate Transit',
    5: 'Surfacing',
    6: 'Obstacle Avoidance',
    7: 'Drum Approach',
    8: 'Flare Engagement',
    9: 'Drum Drop Inspection Orbit',
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
        self.order_sub = self.create_subscription( # Subscriber for Dynamic Obstacle Order from Digital Twin
            String,
            '/mission_obstacle_order',
            self.obstacle_order_callback,
            qos_profile
        )
        # Reports real mission events (flare hit, payload dropped) back to the Digital Twin
        self.mission_state_pub = self.create_publisher(String, '/mission_state', qos_profile)
        self.collision_safety = CollisionSafety()
        self.sonar_subs = [
            self.create_subscription(
                Range,
                f'/sonar/{direction}/range',
                lambda msg, direction=direction: self.sonar_callback(direction, msg),
                qos_profile
            )
            for direction in CollisionSafety.DIRECTIONS
        ]
        self.imu_sub = self.create_subscription(
            Imu,
            '/mavros/imu/data',
            self.imu_callback,
            qos_profile
        )
        self.dvl_floor_sub = self.create_subscription(
            Range,
            '/dvl/range',
            lambda msg: self.sonar_callback('floor', msg),
            qos_profile
        )
        self.battery_safety_sub = self.create_subscription(
            BatteryState, '/battery_state', self.battery_safety_callback, qos_profile
        )
        self.leak_safety_sub = self.create_subscription(
            Bool, '/leak_detected', self.leak_safety_callback, qos_profile
        )
        self.last_safety_reason = None

        # Servo dropper client (retry until MAVROS command service is up)
        self.servo_client = self.create_client(CommandLong, '/mavros/cmd/command')
        self.ball_drop_pending = False
        self.ball_dropped = False
        
        # PositionTarget
        self.cmd = PositionTarget()
        self.cmd.coordinate_frame = PositionTarget.FRAME_BODY_NED
        self.cmd.type_mask = ( # Ignore position, acceleration, and yaw (use velocity and yaw_rate)
            PositionTarget.IGNORE_PX | 
            PositionTarget.IGNORE_PY | 
            PositionTarget.IGNORE_PZ |
            PositionTarget.IGNORE_AFX |
            PositionTarget.IGNORE_AFY |
            PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW  # Ignore yaw target, use yaw_rate instead
        )
                
        # Fast PID controllers with smooth output saturation limits
        self.depth_pid = PID(kp=KP_DEPTH, ki=KI_DEPTH, kd=KD_DEPTH, setpoint=TARGET_DEPTH, output_limits=(-0.5, 0.5))
        self.gate_pid = PID(kp=KP_GATE, ki=KI_GATE, kd=KD_GATE, setpoint=0.0, output_limits=(-0.65, 0.65))
        self.drum_pid = PID(kp=KP_DRUM, ki=KI_DRUM, kd=KD_DRUM, setpoint=0.0, output_limits=(-0.65, 0.65))
        self.flare_pid = PID(kp=KP_FLARE, ki=KI_FLARE, kd=KD_FLARE, setpoint=0.0, output_limits=(-0.75, 0.75))
        
        # 20 Hz update timer for responsive control
        self.timer = self.create_timer(0.05, self.send_cmd)
        
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
        self.max_yaw_acceleration = 0.35  # Smooth, stable acceleration limit
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
        self.drum_hover_start_time = None  # how long we've held centered+stopped above the drum
        self.drum_world_position = None    # pose recorded at the moment of drop, orbit pivot
        self.orbit_last_bearing = None
        self.orbit_swept_angle = 0.0
        
        # flare tracking
        self.flare_coord = None
        self.last_flare_coord = None
        self.last_flare_time = None
        self.close_to_flare = False
        self.deadzone_flare = False
        self.ramming_flare = False
        self.ramming_start_time = None
        self.ramming_flare_color = None
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

    def sonar_callback(self, direction, msg):
        self.collision_safety.update_range(
            direction,
            msg.range,
            msg.min_range,
            msg.max_range
        )

    def imu_callback(self, msg):
        accel = msg.linear_acceleration
        gyro = msg.angular_velocity
        self.collision_safety.update_imu(
            accel.x, accel.y, accel.z,
            gyro.x, gyro.y, gyro.z
        )

    def battery_safety_callback(self, msg):
        self.collision_safety.update_battery(msg.voltage, msg.percentage)

    def leak_safety_callback(self, msg):
        self.collision_safety.update_leak(msg.data)

    def publish_mission_state(self, payload):
        """Sync a real mission event (flare hit, payload dropped, ...) back to the Digital Twin"""
        msg = String()
        msg.data = json.dumps(payload)
        self.mission_state_pub.publish(msg)

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
                elif class_lower == 'obstacle':  # Pure obstacle
                    self.obstacle_coord = point
                    self.last_obstacle_time = current_time
                    self.get_logger().debug(f'[OBSTACLE] Detected at x={point.x:.3f}')
                elif class_lower == 'blue bucket' or class_lower == 'drum' or class_lower == 'red drum':
                    self.drum_coord = point
                    self.last_drum_time = current_time
                    self.get_logger().debug(f'[DRUM] Detected: x={point.x:.3f}, z={point.z:.3f}')
                elif class_lower in ['orange flare', 'red flare', 'yellow flare', 'blue flare', 'flare']:
                    # Check if this matches our current target flare (bounds-checked: all flares may be done)
                    flare_keys = list(self.flare_order.keys())
                    if self.flare_state < len(flare_keys):
                        target_flare_name = self.flare_order.get(flare_keys[self.flare_state], "").lower()
                        if class_lower == target_flare_name or class_lower == 'flare':
                            self.flare_coord = point
                            self.last_flare_time = current_time
                            self.get_logger().debug(f'[FLARE] Detected target {class_name}: x={point.x:.3f}')
                    
        except json.JSONDecodeError as e:
            self.get_logger().warn(f'Failed to parse YOLO JSON: {e}')
        except (KeyError, TypeError, IndexError) as e:
            self.get_logger().warn(f'Error processing YOLO detections: {e}')

    def obstacle_order_callback(self, msg):
        """Callback to dynamically update mission obstacle/flare execution priority from Digital Twin"""
        try:
            data = json.loads(msg.data)
            if isinstance(data, list):
                flare_map_keys = {
                    'orange_flare': 'o',
                    'blue_flare': 'b',
                    'red_flare': 'r',
                    'yellow_flare': 'y'
                }
                new_flare_order = [flare_map_keys[k] for k in data if k in flare_map_keys]
                if new_flare_order:
                    global ORDER_FLARE
                    ORDER_FLARE = new_flare_order
                    self.flare_order = {i: self.flare_map[i] for i in ORDER_FLARE}
                    self.get_logger().info(f'🎯 Dynamic Flare Order Updated from Digital Twin: {ORDER_FLARE}')
        except Exception as e:
            self.get_logger().warn(f'Failed to parse obstacle order JSON: {e}')

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
        """Trigger servo dropper to release golf ball into red drum (non-blocking, retried in send_cmd)"""
        if self.ball_dropped or self.ball_drop_pending:
            return
        self.ball_drop_pending = True
        self._try_release_ball()

    def _try_release_ball(self):
        if not self.ball_drop_pending:
            return
        if not self.servo_client.service_is_ready():
            self.get_logger().warn('[PAYLOAD] /mavros/cmd/command not ready, retrying...', throttle_duration_sec=1.0)
            return
        request = CommandLong.Request()
        request.broadcast = False
        request.command = MAV_CMD_DO_SET_SERVO
        request.confirmation = 0
        request.param1 = float(DROPPER_SERVO_CHANNEL)
        request.param2 = float(DROPPER_SERVO_PWM_RELEASE)
        self.servo_client.call_async(request)
        self.ball_drop_pending = False
        self.ball_dropped = True
        self.get_logger().info('🎯 [PAYLOAD] Servo dropper actuated: Ball released into target drum!')
        pos = self.current_pose.pose.position if self.current_pose is not None else None
        self.publish_mission_state({
            'event': 'payload_dropped',
            'x': pos.x if pos is not None else None,
            'z': pos.y if pos is not None else None,
        })

    def rotate(self, yaw_rate):
        """Set velocity command for rotate (yaw_rate in rad/s)"""
        self.cmd.yaw_rate = yaw_rate
        
    def forward(self, speed):
        """Set velocity command for forward.
        FRAME_BODY_NED = body frame: +x is always vehicle-forward, no yaw
        rotation needed. Leaves velocity.z untouched so maintain_depth() holds.
        """
        self.cmd.velocity.x = speed
        self.cmd.velocity.y = 0.0
    
    def sway(self, speed, forward_speed=0.0):
        """Set velocity command for sway (moving sideways) with optional forward motion
        speed > 0: sway to right
        speed < 0: sway to left
        forward_speed: optional forward velocity during sway
        """
        self.cmd.velocity.x = forward_speed
        self.cmd.velocity.y = speed

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
        self.drum_hover_start_time = None
        self.close_to_flare = False
        self.deadzone_flare = False
        self.ramming_flare = False
        self.ramming_start_time = None
        self.ramming_flare_color = None
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
            elif (new_state == 9):
                self.get_logger().info('Drum drop inspection orbit')

            self.publish_mission_state({'activeTarget': f'[FINAL] {STATE_LABELS.get(new_state, new_state)}'})
    
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
        self._try_release_ball()  # Retry pending ball drop until service accepts it
        
        # State machine logic
        match self.state:
            case 0: # Dive-on-the-fly (Instant Launch)
                self.maintain_depth()
                self.forward(FORWARD_SPEED_SCAN * 0.85)  # Launch forward sprint immediately while diving
                if self.current_pose is not None and abs(self.current_pose.pose.position.z - TARGET_DEPTH) < 0.15:
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
                elif self.prev_state == 9:
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
                
            case 6: # avoid obstacle (High-Speed Racing Lateral Overtake)
                self.maintain_depth()
                
                # Reset obstacle if no longer detected (timeout)
                if self.last_obstacle_time is not None:
                    time_since_obstacle = (current_time - self.last_obstacle_time).nanoseconds / 1e9
                    if time_since_obstacle > 1.2:
                        self.get_logger().info(f'[OBSTACLE] Cleared after {time_since_obstacle:.1f}s, resuming sprint')
                        self.obstacle_coord = None
                        self.change_state(2)
                        return
                
                # Racing sway duration: snappy 1.8s lateral burst
                sway_duration = 1.8
                elapsed = (current_time - self.state_start_time).nanoseconds / 1e9
                
                if elapsed >= sway_duration:
                    self.get_logger().info(f'🚀 Lateral overtake complete: {elapsed:.2f}s, accelerating forward')
                    self.obstacle_coord = None
                    self.change_state(2)
                    return
                
                # High-speed agile sway with forward push
                max_sway_speed = 0.75  # m/s aggressive lateral speed
                remaining_time = sway_duration - elapsed
                
                deceleration_zone = 0.4  # seconds
                if remaining_time < deceleration_zone:
                    target_speed = max_sway_speed * (remaining_time / deceleration_zone)
                else:
                    target_speed = max_sway_speed
                
                self.current_sway_velocity = target_speed
                forward_during_sway = 0.45  # Forward push during sway for rapid arc overtake
                self.sway(self.current_sway_velocity, forward_during_sway)
                
            case 7: # track drum (Racing Precision Approach & Instant Drop)
                self.maintain_depth()
                
                if self.drum_coord is not None:
                    self.last_drum_coord = self.drum_coord
                    self.last_drum_time = current_time
                
                time_since_last_drum_coord = (current_time - self.last_drum_time).nanoseconds / 1e9 if self.last_drum_time is not None else 999.0
                if time_since_last_drum_coord > 3.0:
                    self.get_logger().warn(f'[DRUM] Lost target for {time_since_last_drum_coord:.1f}s, aborting drum approach')
                    self.last_drum_coord = None
                    self.change_state(1)
                    return
                
                elapsed_state_7 = (current_time - self.state_start_time).nanoseconds / 1e9
                if elapsed_state_7 > 6.0:
                    self.get_logger().warn('[DRUM] Search timeout (6s), accelerating forward')
                    self.change_state(1)
                    return
                
                if self.close_to_drum:
                    centered = self.deadzone_drum or (self.drum_coord is not None and abs(self.drum_coord.x) < 0.12)
                    if centered:
                        # Hold full stop directly above the drum - do not drop while still drifting
                        self.reset()
                        if self.drum_hover_start_time is None:
                            self.drum_hover_start_time = current_time
                            self.get_logger().info('🎯 Centered over Drum - holding still to confirm before drop...')
                        hover_elapsed = (current_time - self.drum_hover_start_time).nanoseconds / 1e9
                        if hover_elapsed >= DRUM_HOVER_CONFIRM_SEC:
                            self.get_logger().info(f'✅ Held steady for {hover_elapsed:.1f}s directly above drum: releasing payload!')
                            self.drop_ball_payload()
                            self.drum_world_position = self.current_pose.pose.position if self.current_pose is not None else None
                            self.orbit_last_bearing = None
                            self.orbit_swept_angle = 0.0
                            self.change_state(9)
                    else:
                        self.drum_hover_start_time = None  # drifted off-center, restart the hold timer
                        self.track_drum()
                        self.forward(0.35)
                else:
                    self.track_drum()
                    self.forward(FORWARD_SPEED_DRUM)
                    self.cmd.velocity.z = -0.12  # Descend toward drum (z-up convention)
                    if self.drum_coord is not None and self.drum_coord.z > 0.10:
                        self.close_to_drum = True
                        self.reset()
                
            case 8: # track flare (High-Velocity Collision & Quick-Inspect)
                self.maintain_depth()
                
                if self.flare_coord is not None:
                    self.last_flare_coord = self.flare_coord
                    self.last_flare_time = current_time
                
                time_since_last_flare_coord = (current_time - self.last_flare_time).nanoseconds / 1e9 if self.last_flare_time is not None else 999.0
                
                # PHASE 2: RACING FULL-THROTTLE RAMMING & PLOW-THROUGH
                if self.ramming_flare:
                    ram_elapsed = (current_time - self.ramming_start_time).nanoseconds / 1e9
                    if ram_elapsed < 1.6:
                        # Full maximum racing surge thrust!
                        self.forward(FORWARD_SPEED_FLARE)
                        self.cmd.yaw_rate = 0.0
                        self.get_logger().info(f'💥 RACING HARD RAMMING: Plowing at {FORWARD_SPEED_FLARE:.2f}m/s through flare! ({ram_elapsed:.1f}s / 1.6s)')
                    else:
                        self.get_logger().info('🏆 Flare KNOCKDOWN COMPLETE: Target smashed! Accelerating to next milestone.')
                        self.publish_mission_state({'event': 'flare_hit', 'color': self.ramming_flare_color})
                        self.ramming_flare = False
                        self.ramming_start_time = None
                        self.change_state(2)
                    return

                if time_since_last_flare_coord > 2.5:
                    self.get_logger().warn('Lost flare target for 2.5s, returning to scan')
                    self.last_flare_coord = None
                    self.change_state(1)
                    return
                
                curr_flare_z = self.flare_coord.z if self.flare_coord is not None else (self.last_flare_coord.z if self.last_flare_coord is not None else 0.0)
                curr_flare_x = self.flare_coord.x if self.flare_coord is not None else (self.last_flare_coord.x if self.last_flare_coord is not None else 0.0)

                alignment_error = abs(curr_flare_x)
                if alignment_error > 0.18:
                    approach_speed = 0.45
                elif alignment_error > 0.08:
                    approach_speed = 0.85
                else:
                    approach_speed = FORWARD_SPEED_FLARE

                # Determine flare strategy: TABRAK vs MENGHINDAR
                flare_keys_list = list(self.flare_order.keys())
                current_flare_key = flare_keys_list[self.flare_state] if self.flare_state < len(flare_keys_list) else 'o'
                current_flare_name = self.flare_order.get(current_flare_key, "Flare")
                current_strategy = FLARE_STRATEGIES.get(current_flare_key, 'TABRAK')

                if current_strategy == 'MENGHINDAR':
                    # =========================================================================
                    # STRATEGI MENGHINDAR: Samperin cepat, inspeksi sekejap (0.45s), lalu evasive sway!
                    # =========================================================================
                    if curr_flare_z > 0.04 or (curr_flare_z > 0.022 and alignment_error < 0.15):
                        if self.avoid_inspect_start_time is None:
                            self.avoid_inspect_start_time = current_time
                            self.get_logger().info(f'🛡️ [MENGHINDAR] Locked at inspection range for {current_flare_name}! Quick photo snapshot...')

                        inspect_duration = (current_time - self.avoid_inspect_start_time).nanoseconds / 1e9
                        if inspect_duration < 0.45:
                            # Brief counter-brake hover snapshot
                            self.cmd.velocity.x = 0.0
                            self.cmd.yaw_rate = 0.0
                        else:
                            self.get_logger().info(f'🛡️ [MENGHINDAR] Snapshot verified for {current_flare_name}! Instant lateral evasive boost.')
                            self.avoid_inspect_start_time = None
                            self.flare_state += 1
                            self.last_flare_coord = None
                            
                            if self.flare_state >= len(ORDER_FLARE):
                                self.get_logger().info('🏆 All flares finished! Blitzing to Task 2: Gate.')
                                self.task = 2
                                self.change_state(6)
                            else:
                                self.get_logger().info(f'🛡️ Lateral evasive sway to next flare ({self.flare_state + 1}/{len(ORDER_FLARE)})...')
                                self.change_state(6)
                            return
                    else:
                        self.track_flare()
                        if curr_flare_z > 0.015:
                            safe_spd = 0.35
                        else:
                            safe_spd = min(0.65, approach_speed * 0.75)
                        self.forward(safe_spd)
                else:
                    # =========================================================================
                    # STRATEGI TABRAK: Full-throttle racing smash & knockdown!
                    # =========================================================================
                    if (curr_flare_z > 0.07 and alignment_error < 0.25) or (curr_flare_z > 0.04 and alignment_error < 0.12):
                        self.get_logger().info(f'💥 FLARE {current_flare_name} LOCKED - CHARGING FULL 1.55 m/s SMASH!')
                        self.ramming_flare = True
                        self.ramming_start_time = current_time
                        self.ramming_flare_color = current_flare_name.split(' ')[0].lower()
                        self.forward(FORWARD_SPEED_FLARE)
                        self.cmd.yaw_rate = 0.0
                        return
                    else:
                        self.track_flare()
                        self.forward(approach_speed)

            case 9: # Orbit around the drop point for visual inspection of ball placement.
                # No trained vision class exists yet for "ball in drum", so success/failure
                # cannot be detected automatically - this maneuver only brings the drum into
                # camera view from every side; a human operator must confirm from the feed.
                # The dropper servo is release-only (no calibrated grasp position), so a
                # missed ball cannot be retrieved automatically with the current hardware.
                self.maintain_depth()

                if self.drum_world_position is None or self.current_pose is None:
                    self.get_logger().warn('[DRUM-ORBIT] No drop position recorded, skipping inspection orbit')
                    self.change_state(2)
                    return

                dx = self.current_pose.pose.position.x - self.drum_world_position.x
                dy = self.current_pose.pose.position.y - self.drum_world_position.y
                radius = math.hypot(dx, dy)
                if radius < 0.05:
                    # Too close to the pivot for a stable bearing - nudge outward first
                    dx, dy, radius = DRUM_ORBIT_RADIUS, 0.0, DRUM_ORBIT_RADIUS

                bearing = math.atan2(dy, dx)
                if self.orbit_last_bearing is not None:
                    self.orbit_swept_angle += self.normalize_angle(bearing - self.orbit_last_bearing)
                self.orbit_last_bearing = bearing

                if abs(self.orbit_swept_angle) >= 2 * math.pi:
                    self.get_logger().info('🔄 Inspection orbit complete - confirm ball placement from the camera feed (no automatic retrieval possible). Continuing mission.')
                    self.publish_mission_state({'event': 'drum_inspection_complete'})
                    self.change_state(2)
                    return

                outward_x, outward_y = dx / radius, dy / radius
                tangent_x, tangent_y = -outward_y, outward_x
                radius_error = radius - DRUM_ORBIT_RADIUS
                world_vx = tangent_x * DRUM_ORBIT_SPEED - outward_x * radius_error * DRUM_ORBIT_RADIUS_KP
                world_vy = tangent_y * DRUM_ORBIT_SPEED - outward_y * radius_error * DRUM_ORBIT_RADIUS_KP

                # Keep the camera trained on the drum throughout the loop
                yaw = self.get_yaw()
                desired_yaw = self.normalize_angle(bearing + math.pi)
                yaw_error = self.normalize_angle(desired_yaw - yaw)
                self.cmd.yaw_rate = max(-0.5, min(0.5, yaw_error * 1.2))

                # World-frame orbit velocity -> body-frame (FRAME_BODY_NED) command
                self.cmd.velocity.x = max(-DRUM_ORBIT_SPEED * 1.3, min(DRUM_ORBIT_SPEED * 1.3, math.cos(yaw) * world_vx + math.sin(yaw) * world_vy))
                self.cmd.velocity.y = max(-DRUM_ORBIT_SPEED * 1.3, min(DRUM_ORBIT_SPEED * 1.3, -math.sin(yaw) * world_vx + math.cos(yaw) * world_vy))

        safety_reason = self.collision_safety.apply(self.cmd)
        if safety_reason != self.last_safety_reason:
            if safety_reason:
                self.get_logger().warn(f'[COLLISION SAFETY] Command limited: {safety_reason}')
            else:
                self.get_logger().info('[COLLISION SAFETY] Path clear')
            self.last_safety_reason = safety_reason

        # Set header timestamp
        self.cmd.header.stamp = current_time.to_msg()
        self.cmd.header.frame_id = 'base_link'
            
        self.vel_pub.publish(self.cmd)


def main():
    from sauvc26_code.control_lock import acquire_control_lock
    try:
        acquire_control_lock('final')
    except RuntimeError as e:
        print(f'[FATAL] {e}')
        return

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
