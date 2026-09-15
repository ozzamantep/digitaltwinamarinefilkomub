import os
import sys
from types import SimpleNamespace


sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'jetson'))

from sauvc26_code.collision_safety import CollisionSafety


def command(x=0.0, y=0.0, z=0.0, yaw_rate=0.0):
    return SimpleNamespace(
        velocity=SimpleNamespace(x=x, y=y, z=z),
        yaw_rate=yaw_rate
    )


safety = CollisionSafety()
safety.update_range('front', 2.0, now=1.0)
clear = command(x=1.0)
assert safety.apply(clear, now=1.1) is None
assert clear.velocity.x == 1.0

safety.update_range('front', 0.4, now=2.0)
blocked = command(x=1.0)
assert safety.apply(blocked, now=2.1) == 'front_sonar'
assert blocked.velocity.x == -0.20

safety.update_range('front', 0.8, now=3.0)
still_blocked = command(x=1.0)
assert safety.apply(still_blocked, now=3.1) == 'front_sonar'
assert still_blocked.velocity.x == -0.20

safety.update_range('front', 1.3, now=3.2)
safety.update_range('front', 0.8, now=3.3)
slowed = command(x=1.0)
assert safety.apply(slowed, now=3.4) == 'front_sonar'
assert 0.0 < slowed.velocity.x < 1.0

rear_guard = CollisionSafety()
rear_guard.update_range('rear', 0.4, now=3.5)
rear_command = command(x=-0.8)
assert rear_guard.apply(rear_command, now=3.6) == 'rear_sonar'
assert rear_command.velocity.x == 0.20

right_guard = CollisionSafety()
right_guard.update_range('right', 0.4, now=3.5)
right_command = command(y=0.8)
assert right_guard.apply(right_command, now=3.6) == 'right_sonar'
assert right_command.velocity.y == -0.20

stale = command(x=1.0)
assert safety.apply(stale, now=4.0) == 'front_sonar'
assert stale.velocity.x == 0.0

safety.update_imu(20.0, 0.0, 0.0, 0.0, 0.0, 0.0, now=5.0)
impact = command(x=1.0, y=1.0, z=1.0, yaw_rate=1.0)
assert safety.apply(impact, now=5.1) == 'imu_emergency_stop'
assert impact.velocity.x == 0.0
assert impact.velocity.y == 0.0
assert impact.velocity.z == 0.0
assert impact.yaw_rate == 0.0

floor_guard = CollisionSafety()
floor_guard.update_range('floor', 0.30, now=5.0)
forced_dive = command(z=0.8)
assert floor_guard.apply(forced_dive, now=5.1) == 'floor_dvl'
assert forced_dive.velocity.z == -0.20

floor_guard.update_range('floor', 0.525, now=6.0)
latched_dive = command(z=0.8)
assert floor_guard.apply(latched_dive, now=6.1) == 'floor_dvl'
assert latched_dive.velocity.z == -0.20

floor_guard.update_range('floor', 0.75, now=6.2)
floor_guard.update_range('floor', 0.525, now=6.3)
slowed_dive = command(z=0.8)
assert floor_guard.apply(slowed_dive, now=6.4) == 'floor_dvl'
assert 0.0 < slowed_dive.velocity.z < 0.8

surface_command = command(z=-0.8)
assert floor_guard.apply(surface_command, now=7.0) is None
assert surface_command.velocity.z == -0.8

metadata_default = CollisionSafety()
metadata_default.update_range('front', 1.5, max_range=0.0, now=6.0)
assert metadata_default.ranges['front'] == 1.5

emergency = CollisionSafety()
emergency.update_leak(True)
emergency_command = command(x=1.0, y=1.0, z=1.0, yaw_rate=1.0)
assert emergency.apply(emergency_command, now=8.0) == 'emergency_surface:hull_leak'
assert emergency_command.velocity.x == 0.0
assert emergency_command.velocity.y == 0.0
assert emergency_command.velocity.z == -0.35
assert emergency_command.yaw_rate == 0.0

battery_emergency = CollisionSafety()
battery_emergency.update_battery(12.9, 0.5)
assert battery_emergency.apply(command(), now=9.0) == 'emergency_surface:battery_critical'

print('PASS: safety latches, floor guard, IMU stop, and emergency surface')