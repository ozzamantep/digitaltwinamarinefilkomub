import json
import math
import os
import time

_DEFAULT_PARAMS = {
    'stop_distance': 0.55,
    'slow_distance': 1.2,
    'floor_stop_distance': 0.37,
    'floor_slow_distance': 0.7,
    'stale_timeout': 0.5,
    'backoff_speed': 0.2,
    'emergency_surface_speed': 0.35,
    'battery_voltage_critical': 13.0,
    'battery_fraction_critical': 0.08,
    'impact_accel_threshold': 18.0,
    'impact_gyro_threshold': 2.5,
    'impact_hold_seconds': 1.0,
}


def _load_params():
    """Shared with the JS digital twin (SafetySupervisor.js) - single source of truth"""
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'safety_params.json')
    try:
        with open(path, 'r', encoding='utf-8') as handle:
            data = json.load(handle)
    except (OSError, ValueError):
        data = {}
    return {**_DEFAULT_PARAMS, **data}


SAFETY_PARAMS = _load_params()


class CollisionSafety:
    DIRECTIONS = ('front', 'rear', 'left', 'right')

    def __init__(self, stop_distance=None, slow_distance=None, stale_timeout=None,
                 floor_stop_distance=None, floor_slow_distance=None):
        params = SAFETY_PARAMS
        self.stop_distance = params['stop_distance'] if stop_distance is None else stop_distance
        self.slow_distance = params['slow_distance'] if slow_distance is None else slow_distance
        self.floor_stop_distance = params['floor_stop_distance'] if floor_stop_distance is None else floor_stop_distance
        self.floor_slow_distance = params['floor_slow_distance'] if floor_slow_distance is None else floor_slow_distance
        self.stale_timeout = params['stale_timeout'] if stale_timeout is None else stale_timeout
        self.backoff_speed = params['backoff_speed']
        self.emergency_surface_speed = params['emergency_surface_speed']
        self.battery_voltage_critical = params['battery_voltage_critical']
        self.battery_fraction_critical = params['battery_fraction_critical']
        self.impact_accel_threshold = params['impact_accel_threshold']
        self.impact_gyro_threshold = params['impact_gyro_threshold']
        self.impact_hold_seconds = params['impact_hold_seconds']
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
        if voltage <= self.battery_voltage_critical or percentage <= self.battery_fraction_critical:
            self.emergency_surface_latched = True
            self.emergency_reason = 'battery_critical'

    def update_range(self, direction, distance, min_range=0.0, max_range=math.inf, now=None):
        if direction not in self.ranges or math.isnan(distance):
            return
        valid_min = max(0.0, min_range)
        valid_max = max_range if max_range > valid_min else math.inf
        timestamp = time.monotonic() if now is None else now
        if not math.isfinite(distance) or distance > valid_max:
            # No echo / beyond sensor reach = path clear; keep data fresh
            self.ranges[direction] = valid_max
            self.range_times[direction] = timestamp
            self.blocked[direction] = False
            return
        if distance < valid_min:
            return
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
        if acceleration > self.impact_accel_threshold or angular_rate > self.impact_gyro_threshold:
            self.impact_stop_until = timestamp + self.impact_hold_seconds

    def _is_stale(self, direction, now):
        return now - self.range_times[direction] > self.stale_timeout

    def _scale(self, direction, now):
        # Stale sensor = unknown clearance: stop, but never latch a back-off
        if self._is_stale(direction, now):
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
        if self._is_stale('floor', now):
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
            command.velocity.z = self.emergency_surface_speed  # z-up convention (matches surface()/maintain_depth)
            command.yaw_rate = 0.0
            return f'emergency_surface:{self.emergency_reason}'

        if timestamp < self.impact_stop_until:
            command.velocity.x = 0.0
            command.velocity.y = 0.0
            command.velocity.z = 0.0
            command.yaw_rate = 0.0
            return 'imu_emergency_stop'

        # Active back-off only on FRESH confirmed obstacles; stale sensors fall
        # through to the scaling section below which stops (never reverses)
        front_blocked = self.blocked['front'] and not self._is_stale('front', timestamp)
        rear_blocked = self.blocked['rear'] and not self._is_stale('rear', timestamp)
        left_blocked = self.blocked['left'] and not self._is_stale('left', timestamp)
        right_blocked = self.blocked['right'] and not self._is_stale('right', timestamp)
        floor_blocked = self.blocked['floor'] and not self._is_stale('floor', timestamp)

        active_locks = []
        if front_blocked and rear_blocked:
            command.velocity.x = 0.0
            active_locks.append('surge_sonar')
        elif front_blocked:
            command.velocity.x = min(command.velocity.x, -self.backoff_speed)
            active_locks.append('front_sonar')
        elif rear_blocked:
            command.velocity.x = max(command.velocity.x, self.backoff_speed)
            active_locks.append('rear_sonar')

        if left_blocked and right_blocked:
            command.velocity.y = 0.0
            active_locks.append('sway_sonar')
        elif left_blocked:
            command.velocity.y = max(command.velocity.y, self.backoff_speed)
            active_locks.append('left_sonar')
        elif right_blocked:
            command.velocity.y = min(command.velocity.y, -self.backoff_speed)
            active_locks.append('right_sonar')

        if floor_blocked:
            command.velocity.z = max(command.velocity.z, self.backoff_speed)  # force ascend away from floor (z-up)
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

        if command.velocity.z < 0.0:  # descending (z-up convention)
            scale = self._floor_scale(timestamp)
            command.velocity.z *= scale
            if scale < 1.0:
                reason = 'floor_dvl'

        return reason