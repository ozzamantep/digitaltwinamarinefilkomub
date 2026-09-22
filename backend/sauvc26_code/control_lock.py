#!/usr/bin/env python3
"""
Control-authority lock: guarantees only ONE node publishes velocity setpoints to
/mavros/setpoint_raw/local at a time (manual_bridge.py XOR final.py XOR qualification.py).
Running two simultaneously means their setpoints fight each other on the real vehicle.
"""
import os
import atexit

LOCK_PATH = '/tmp/sauvc26_control.lock'


def _pid_is_alive(pid):
    try:
        os.kill(pid, 0)
    except (OSError, ValueError):
        return False
    return True


def acquire_control_lock(owner_name):
    """Claim exclusive control authority. Raises RuntimeError if another
    control node is already running (and alive)."""
    if os.path.exists(LOCK_PATH):
        existing_pid, existing_owner = None, 'unknown'
        try:
            with open(LOCK_PATH, 'r') as f:
                parts = f.read().strip().split(',', 1)
            existing_pid = int(parts[0])
            existing_owner = parts[1] if len(parts) > 1 else 'unknown'
        except (OSError, ValueError, IndexError):
            pass

        if existing_pid is not None and _pid_is_alive(existing_pid):
            raise RuntimeError(
                f"Control lock held by '{existing_owner}' (PID {existing_pid}). "
                f"Refusing to start '{owner_name}' - only ONE control node "
                f"(manual_bridge.py OR final.py OR qualification.py) may run at a "
                f"time, or their setpoints will fight on /mavros/setpoint_raw/local. "
                f"Stop the other node first."
            )
        # Stale lock left behind by a process that died without cleanup - reclaim it.

    with open(LOCK_PATH, 'w') as f:
        f.write(f"{os.getpid()},{owner_name}")
    atexit.register(release_control_lock)


def release_control_lock():
    try:
        if os.path.exists(LOCK_PATH):
            with open(LOCK_PATH, 'r') as f:
                held_pid = f.read().split(',', 1)[0].strip()
            if held_pid == str(os.getpid()):
                os.remove(LOCK_PATH)
    except OSError:
        pass
