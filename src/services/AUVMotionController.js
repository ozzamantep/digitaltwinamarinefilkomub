import PIDController from './PidController';

/**
 * 6-DOF Hydrodynamic Flight & PID Motion Controller for BlueROV2 AUV
 * Implements real fluid inertia, quadratic water drag, natural glide coasting,
 * dynamic hydrodynamic banking/pitching, and SAUVC 2026 closed-loop stabilization.
 */
class AUVMotionController {
  constructor() {
    // 1. Depth PID Controller (SAUVC tuned values: Kp=0.5, Ki=0.1, Kd=0.2, Target: 0.8m)
    this.depthPid = new PIDController(0.5, 0.1, 0.2, 0.8, -1.0, 1.0);

    // 2. Heading / Yaw Rate PID Controller (Kp=0.6, Ki=0.05, Kd=0.25)
    this.yawPid = new PIDController(0.6, 0.05, 0.25, 0.0, -1.0, 1.0);

    // 3. Roll & Pitch Attitude Stabilization PID
    this.pitchPid = new PIDController(0.8, 0.0, 0.3, 0.0, -0.8, 0.8);
    this.rollPid = new PIDController(0.8, 0.0, 0.3, 0.0, -0.8, 0.8);

    // Dynamic Physics State (Velocities & Accelerations for Fluid Inertia)
    this.velSurge = 0.0;
    this.velSway = 0.0;
    this.velHeave = 0.0;
    this.velYaw = 0.0;

    this.dynamicPitch = 0.0;
    this.dynamicRoll = 0.0;

    this.targetDepth = 0.8;
    this.targetHeading = 0;
    this.pidTelemetry = {
      depthError: 0,
      depthEffort: 0,
      yawError: 0,
      yawEffort: 0,
    };
  }

  setTargetDepth(depth) {
    this.targetDepth = Math.max(0.15, Math.min(1.85, depth));
    this.depthPid.setSetpoint(this.targetDepth);
  }

  setTargetHeading(rad) {
    this.targetHeading = (rad + Math.PI * 2) % (Math.PI * 2);
    this.yawPid.setSetpoint(this.targetHeading);
  }

  setDepthGains(kp, ki, kd) {
    this.depthPid.setGains(kp, ki, kd);
  }

  setYawGains(kp, ki, kd) {
    this.yawPid.setGains(kp, ki, kd);
  }

  reset() {
    this.depthPid.reset();
    this.yawPid.reset();
    this.pitchPid.reset();
    this.rollPid.reset();
    this.velSurge = 0;
    this.velSway = 0;
    this.velHeave = 0;
    this.velYaw = 0;
    this.dynamicPitch = 0;
    this.dynamicRoll = 0;
  }

  /**
   * Continuous 6-DOF Hydrodynamic integration with fluid inertia & drag
   */
  update(currentPose, userCmd, flightMode, armed, dt) {
    if (!armed) {
      // Fluid drag decelerates drifting AUV to stop forward motion
      this.velSurge *= Math.max(0, 1 - 3.5 * dt);
      this.velSway *= Math.max(0, 1 - 3.5 * dt);
      this.velYaw *= Math.max(0, 1 - 4.0 * dt);
      this.dynamicPitch *= Math.max(0, 1 - 2.0 * dt);
      this.dynamicRoll *= Math.max(0, 1 - 2.0 * dt);

      // Natural Positive Buoyancy (Fail-safe float to surface: depth ~ 0.15m)
      const currentDepth = currentPose.depth || 0.8;
      if (currentDepth > 0.15) {
        // Gently float upwards (-heave in depth coordinates)
        this.velHeave = Math.max(-0.12, this.velHeave - 0.25 * dt);
      } else {
        this.velHeave *= Math.max(0, 1 - 4.0 * dt);
      }

      return {
        surge: this.velSurge,
        sway: this.velSway,
        yaw: this.velYaw,
        heave: this.velHeave,
        pitch: this.dynamicPitch,
        roll: this.dynamicRoll,
        thrusters: [0, 0, 0, 0, 0, 0],
        telemetry: { depthError: 0, depthEffort: 0, yawError: 0, yawEffort: 0 },
      };
    }

    const targetSurge = (userCmd.surge || 0) * 1.3; // Max surge speed 1.3 m/s
    const targetSway = (userCmd.sway || 0) * 1.1;   // Max lateral speed 1.1 m/s
    let targetYawRate = (userCmd.yaw || 0) * 1.5;   // Max yaw rate 1.5 rad/s
    let targetHeave = 0;

    // 1. Closed-loop Depth PID Regulation
    if (flightMode === 'ALT_HOLD' || flightMode === 'STABILIZE' || flightMode === 'AUTO') {
      if (Math.abs(userCmd.heave) > 0.05) {
        this.targetDepth = Math.max(0.15, Math.min(1.85, currentPose.depth + userCmd.heave * dt * 0.7));
        this.depthPid.setSetpoint(this.targetDepth);
      }
      const depthRes = this.depthPid.compute(currentPose.depth);
      targetHeave = depthRes.output * 0.6;
      this.pidTelemetry.depthError = depthRes.error;
      this.pidTelemetry.depthEffort = depthRes.output;
    } else {
      targetHeave = (userCmd.heave || 0) * 0.6;
      this.targetDepth = currentPose.depth;
      this.depthPid.setSetpoint(this.targetDepth);
      this.pidTelemetry.depthError = 0;
      this.pidTelemetry.depthEffort = targetHeave;
    }

    // 2. Closed-loop Heading PID Regulation
    if (Math.abs(userCmd.yaw) > 0.05) {
      this.targetHeading = currentPose.heading;
      this.yawPid.setSetpoint(this.targetHeading);
      this.pidTelemetry.yawError = 0;
      this.pidTelemetry.yawEffort = userCmd.yaw;
    } else if (flightMode === 'STABILIZE' || flightMode === 'ALT_HOLD') {
      let headingErr = this.targetHeading - currentPose.heading;
      while (headingErr > Math.PI) headingErr -= Math.PI * 2;
      while (headingErr < -Math.PI) headingErr += Math.PI * 2;

      const yawRes = this.yawPid.compute(currentPose.heading);
      targetYawRate = yawRes.output * 1.2;
      this.pidTelemetry.yawError = headingErr;
      this.pidTelemetry.yawEffort = yawRes.output;
    }

    // 3. Fluid Inertia & Quadratic Drag Integration (Smooth physical glide)
    // dv/dt = (v_target - v) * accel_rate
    const accelRate = 4.2; // Smooth acceleration response
    this.velSurge += (targetSurge - this.velSurge) * Math.min(1, accelRate * dt);
    this.velSway += (targetSway - this.velSway) * Math.min(1, accelRate * dt);
    this.velHeave += (targetHeave - this.velHeave) * Math.min(1, 3.5 * dt);
    this.velYaw += (targetYawRate - this.velYaw) * Math.min(1, 5.0 * dt);

    // 4. Natural Hydrodynamic Tilt (Banking & Pitch on acceleration)
    const targetPitchTilt = -this.velSurge * 3.5 + (this.velHeave * 2.0);
    const targetRollTilt = this.velSway * 4.2 + (this.velYaw * 1.8);
    this.dynamicPitch += (targetPitchTilt - this.dynamicPitch) * Math.min(1, 3.0 * dt);
    this.dynamicRoll += (targetRollTilt - this.dynamicRoll) * Math.min(1, 3.0 * dt);

    // 5. BlueROV2 6-Thruster Allocation
    const tSurge = (this.velSurge / 1.3) * 65;
    const tSway = (this.velSway / 1.1) * 65;
    const tYaw = (this.velYaw / 1.5) * 45;
    const tHeave = (this.velHeave / 0.6) * 60;
    const tPitch = this.dynamicPitch * 4.0;

    const t1 = Math.round(Math.max(-100, Math.min(100, tSurge + tYaw - tSway)));
    const t2 = Math.round(Math.max(-100, Math.min(100, tSurge - tYaw + tSway)));
    const t3 = Math.round(Math.max(-100, Math.min(100, tSurge + tYaw + tSway)));
    const t4 = Math.round(Math.max(-100, Math.min(100, tSurge - tYaw - tSway)));
    const t5 = Math.round(Math.max(-100, Math.min(100, tHeave + tPitch)));
    const t6 = Math.round(Math.max(-100, Math.min(100, tHeave - tPitch)));

    return {
      surge: this.velSurge,
      sway: this.velSway,
      yaw: this.velYaw,
      heave: this.velHeave,
      pitch: this.dynamicPitch,
      roll: this.dynamicRoll,
      thrusters: [t1, t2, t3, t4, t5, t6],
      telemetry: { ...this.pidTelemetry },
    };
  }
}

const auvMotionController = new AUVMotionController();
export default auvMotionController;
