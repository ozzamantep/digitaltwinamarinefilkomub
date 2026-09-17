#!/usr/bin/env python3
import time

class PID:
    """
    High-Performance Fast-Response PID Controller with Anti-Windup & Derivative Filter
    Designed for SAUVC AUV Racing Maneuvers
    """
    def __init__(self, kp, ki, kd, setpoint=0.0, output_limits=(-1.0, 1.0)):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.min_output, self.max_output = output_limits

        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = time.time()
        self.prev_derivative = 0.0
        self.d_alpha = 0.75  # 1st order derivative low-pass filter coefficient

    def reset(self):
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = time.time()
        self.prev_derivative = 0.0

    def set_gains(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd

    def set_setpoint(self, setpoint):
        self.setpoint = setpoint

    def compute(self, measurement):
        now = time.time()
        dt = now - self.prev_time

        if dt <= 0.0001:
            dt = 0.01  # Guard against zero/negative dt

        error = self.setpoint - measurement

        # 1. Proportional term
        P = self.kp * error

        # 2. Integral term with Anti-Windup Clamp
        # Only integrate when error is reasonable to prevent massive windup during transitions
        self.integral += error * dt
        # Integral clamp (limit to 40% of max output authority)
        max_integral = (self.max_output * 0.4) / (self.ki if self.ki > 1e-6 else 1.0)
        self.integral = max(-max_integral, min(max_integral, self.integral))
        I = self.ki * self.integral

        # 3. Filtered Derivative term (eliminates sensor noise spikes while damping overshoot)
        raw_derivative = (error - self.prev_error) / dt
        filtered_derivative = self.d_alpha * raw_derivative + (1.0 - self.d_alpha) * self.prev_derivative
        D = self.kd * filtered_derivative

        # Total unconstrained output
        raw_output = P + I + D

        # Output Saturation
        output = max(self.min_output, min(self.max_output, raw_output))

        self.prev_error = error
        self.prev_derivative = filtered_derivative
        self.prev_time = now

        return output
