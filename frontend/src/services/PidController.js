/**
 * PID Controller matching SAUVC 2026 Codebase (sauvc26_code/pid.py)
 * Implements Proportional, Integral, and Derivative control with anti-windup clamping.
 */
export class PIDController {
  constructor(kp = 0.5, ki = 0.1, kd = 0.2, setpoint = 0.0, minOutput = -1.0, maxOutput = 1.0) {
    this.kp = kp;
    this.ki = ki;
    this.kd = kd;
    this.setpoint = setpoint;
    this.minOutput = minOutput;
    this.maxOutput = maxOutput;

    this.prevError = 0.0;
    this.integral = 0.0;
    this.prevTime = performance.now() / 1000;
  }

  reset() {
    this.prevError = 0.0;
    this.integral = 0.0;
    this.prevTime = performance.now() / 1000;
  }

  setGains(kp, ki, kd) {
    this.kp = kp;
    this.ki = ki;
    this.kd = kd;
  }

  setSetpoint(setpoint) {
    this.setpoint = setpoint;
  }

  compute(measurement) {
    const now = performance.now() / 1000;
    const dt = now - this.prevTime;

    if (dt <= 0 || dt > 1.0 || isNaN(measurement)) {
      this.prevTime = now;
      return {
        output: 0.0,
        error: 0.0,
        P: 0.0,
        I: 0.0,
        D: 0.0,
      };
    }

    const error = this.setpoint - measurement;

    // Proportional
    const P = this.kp * error;

    // Integral with anti-windup clamping
    this.integral += error * dt;
    this.integral = Math.max(-2.0, Math.min(2.0, this.integral));
    const I = this.ki * this.integral;

    // Derivative
    const derivative = (error - this.prevError) / dt;
    const D = this.kd * derivative;

    let output = P + I + D;
    output = Math.max(this.minOutput, Math.min(this.maxOutput, output));

    this.prevError = error;
    this.prevTime = now;

    return {
      output: isNaN(output) ? 0.0 : output,
      error: isNaN(error) ? 0.0 : error,
      P: isNaN(P) ? 0.0 : P,
      I: isNaN(I) ? 0.0 : I,
      D: isNaN(D) ? 0.0 : D,
    };
  }
}

export default PIDController;
