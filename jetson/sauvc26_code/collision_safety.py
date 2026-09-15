import math
import time


class CollisionSafety:
    DIRECTIONS = ('front', 'rear', 'left', 'right')

    def __init__(self, stop_distance=0.55, slow_distance=1.20, stale_timeout=0.50,
                 floor_stop_distance=0.37, floor_slow_distance=0.70):
        self.stop_distance = stop_distance
        self.slow_distance = slow_distance
        self.floor_stop_distance = floor_stop_distance
        self.floor_slow_distance = floor_slow_distance
        self.stale_timeout = stale_timeout
        self.ranges = {direction: math.inf for direction in (*self.DIRECTIONS, 'floor')}
        self.range_times = {direction: 0.0 for direction in (*self.DIRECTIONS, 'floor')}
        self.blocked = {direction: False for direction in (*self.DIRECTIONS, 'floor')}
        self.impact_stop_until = 0.0
        self.emergency_surface_latched = False
        self.emergency_reason = None

    def update_leak(self, leak_detected):
        if leak_detected:
            self.emergency_surface_latched = True
            self.emergency_reason = 'hull_leak'

    def update_battery(self, voltage, percentage):
        if voltage <= 13.0 or percentage <= 0.08:
            self.emergency_surface_latched = True
            self.emergency_reason = 'battery_critical'

    def update_range(self, direction, distance, min_range=0.0, max_range=math.inf, now=None):
        if direction not in self.ranges or not math.isfinite(distance):
            return
        valid_min = max(0.0, min_range)
        valid_max = max_range if max_range > valid_min else math.inf
        if distance < valid_min or distance > valid_max:
            return
        timestamp = time.monotonic() if now is None else now
        self.ranges[direction] = max(0.0, distance)
        self.range_times[direction] = timestamp
        stop_distance = self.floor_stop_distance if direction == 'floor' else self.stop_distance
        release_distance = self.floor_slow_distance if direction == 'floor' else self.slow_distance
        if distance <= stop_distance:
            self.blocked[direction] = True
        elif distance >= release_distance:
            self.blocked[direction] = False

    def update_imu(self, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, now=None):
        timestamp = time.monotonic() if now is None else now
        acceleration = math.sqrt(accel_x ** 2 + accel_y ** 2 + accel_z ** 2)
        angular_rate = math.sqrt(gyro_x ** 2 + gyro_y ** 2 + gyro_z ** 2)
        if acceleration > 18.0 or angular_rate > 2.5:
            self.impact_stop_until = timestamp + 1.0

    def _scale(self, direction, now):
        if now - self.range_times[direction] > self.stale_timeout:
            self.blocked[direction] = True
            return 0.0
        if self.blocked[direction]:
            return 0.0
        distance = self.ranges[direction]
        if distance <= self.stop_distance:
            return 0.0
        if distance >= self.slow_distance:
            return 1.0
        return (distance - self.stop_distance) / (self.slow_distance - self.stop_distance)

    def _floor_scale(self, now):
        if now - self.range_times['floor'] > self.stale_timeout:
            self.blocked['floor'] = True
            return 0.0
        if self.blocked['floor']:
            return 0.0
        distance = self.ranges['floor']
        if distance <= self.floor_stop_distance:
            return 0.0
        if distance >= self.floor_slow_distance:
            return 1.0
        return ((distance - self.floor_stop_distance) /
                (self.floor_slow_distance - self.floor_stop_distance))

    def apply(self, command, now=None):
        timestamp = time.monotonic() if now is None else now
        reason = None

        if self.emergency_surface_latched:
            command.velocity.x = 0.0
            command.velocity.y = 0.0
            command.velocity.z = -0.35
            command.yaw_rate = 0.0
            return f'emergency_surface:{self.emergency_reason}'

        if timestamp < self.impact_stop_until:
            command.velocity.x = 0.0
            command.velocity.y = 0.0
            command.velocity.z = 0.0
            command.yaw_rate = 0.0
            return 'imu_emergency_stop'

        active_locks = []
        if self.blocked['front'] and self.blocked['rear']:
            command.velocity.x = 0.0
            active_locks.append('surge_sonar')
        elif self.blocked['front']:
            command.velocity.x = min(command.velocity.x, -0.20)
            active_locks.append('front_sonar')
        elif self.blocked['rear']:
            command.velocity.x = max(command.velocity.x, 0.20)
            active_locks.append('rear_sonar')

        if self.blocked['left'] and self.blocked['right']:
            command.velocity.y = 0.0
            active_locks.append('sway_sonar')
        elif self.blocked['left']:
            command.velocity.y = max(command.velocity.y, 0.20)
            active_locks.append('left_sonar')
        elif self.blocked['right']:
            command.velocity.y = min(command.velocity.y, -0.20)
            active_locks.append('right_sonar')

        if self.blocked['floor']:
            command.velocity.z = min(command.velocity.z, -0.20)
            active_locks.append('floor_dvl')

        if active_locks:
            return '+'.join(active_locks)

        if command.velocity.x > 0.0:
            scale = self._scale('front', timestamp)
            command.velocity.x *= scale
            if scale < 1.0:
                reason = 'front_sonar'
        elif command.velocity.x < 0.0:
            scale = self._scale('rear', timestamp)
            command.velocity.x *= scale
            if scale < 1.0:
                reason = 'rear_sonar'

        if command.velocity.y > 0.0:
            scale = self._scale('right', timestamp)
            command.velocity.y *= scale
            if scale < 1.0:
                reason = 'right_sonar'
        elif command.velocity.y < 0.0:
            scale = self._scale('left', timestamp)
            command.velocity.y *= scale
            if scale < 1.0:
                reason = 'left_sonar'

        if command.velocity.z > 0.0:
            scale = self._floor_scale(timestamp)
            command.velocity.z *= scale
            if scale < 1.0:
                reason = 'floor_dvl'

        return reason