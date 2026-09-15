import PIDController from './PidController';
import hydrodynamicsEngine from './HydrodynamicsEngine';
import thrusterDynamics from './ThrusterDynamicsModel';
import vehicleConfig from '../dt-core/VehicleConfig';
import Kinematics from '../dt-core/Kinematics';

/**
 * 6-DOF Hydrodynamic Flight & PID Motion Controller for BlueROV2 AUV
 * 
 * Features:
 * - 6-DOF Fossen hydrodynamic model integration
 * - Realistic T200 thruster dynamics (voltage sag, ramp-up lag, motor time constant)
 * - Closed-loop Depth and Heading PID controllers with anti-windup
 * - True 6-DOF state representation
 */
class AUVMotionController {
  constructor() {
    // 1. Depth PID Controller (SAUVC tuned values: Kp=0.5, Ki=0.1, Kd=0.2, Target: 0.8m)
    this.depthPid = new PIDController(0.5, 0.1, 0.2, 0.8, -1.0, 1.0);

    // 2. Heading / Yaw Rate PID Controller — Conservative gains for straight-line stability
    this.yawPid = new PIDController(0.35, 0.03, 0.15, 0.0, -1.0, 1.0);

    // 3. Roll & Pitch Attitude Stabilization PID
    this.pitchPid = new PIDController(0.8, 0.0, 0.3, 0.0, -0.8, 0.8);
    this.rollPid = new PIDController(0.8, 0.0, 0.3, 0.0, -0.8, 0.8);

    // Dynamic Physics State (Velocities in body frame: u, v, w, p, q, r)
    this.velSurge = 0.0;
    this.velSway = 0.0;
    this.velHeave = 0.0;
    this.velRoll = 0.0;
    this.velPitch = 0.0;
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

    // Hydrodynamic debug telemetry
    this.hydroTelemetry = {
      currentDrift: { x: 0, z: 0, magnitude: 0 },
      dampingSurge: 0,
      dampingSway: 0,
      sagFactor: 1.0,
      totalCurrent: 0,
    };
  }

  setTargetDepth(depth) {
    this.targetDepth = Math.max(0.15, Math.min(1.85, depth));
    this.depthPid.setSetpoint(this.targetDepth);
  }

  setTargetHeading(rad) {
    this.targetHeading = Kinematics.wrapAngle(rad);
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
    this.velRoll = 0;
    this.velPitch = 0;
    this.velYaw = 0;
    this.dynamicPitch = 0;
    this.dynamicRoll = 0;
    hydrodynamicsEngine.reset();
    thrusterDynamics.reset();
  }

  /**
   * Continuous 6-DOF Hydrodynamic integration with full physics model
   */
  update(currentPose, userCmd, flightMode, armed, dt, batteryVoltage = 16.0, time = 0) {
    if (!armed) {
      // =========================================================================
      // DISARMED: No thruster force, only hydrodynamic drag + buoyancy
      // =========================================================================
      thrusterDynamics.update([0, 0, 0, 0, 0, 0], batteryVoltage, dt);

      const hydroResult = hydrodynamicsEngine.step(
        {
          velSurge: this.velSurge,
          velSway: this.velSway,
          velHeave: this.velHeave,
          velRoll: this.velRoll,
          velPitch: this.velPitch,
          velYaw: this.velYaw,
          heading: currentPose.heading || 0,
          roll: currentPose.roll || 0,
          pitch: currentPose.pitch || 0,
          depth: currentPose.depth || 0.8,
        },
        { surge: 0, sway: 0, heave: 0, yaw: 0, pitch: 0, roll: 0 },
        dt,
        time
      );

      this.velSurge = hydroResult.velSurge;
      this.velSway = hydroResult.velSway;
      this.velHeave = hydroResult.velHeave;
      this.velRoll = hydroResult.velRoll || 0;
      this.velPitch = hydroResult.velPitch || 0;
      this.velYaw = hydroResult.velYaw;

      // Dampen cosmetic attitude
      this.dynamicPitch *= Math.max(0, 1 - 2.0 * dt);
      this.dynamicRoll *= Math.max(0, 1 - 2.0 * dt);

      this.hydroTelemetry.currentDrift = hydroResult.currentDrift;

      return {
        surge: this.velSurge,
        sway: this.velSway,
        yaw: this.velYaw,
        heave: this.velHeave,
        rollRate: this.velRoll,
        pitchRate: this.velPitch,
        pitch: this.dynamicPitch,
        roll: this.dynamicRoll,
        thrusters: [0, 0, 0, 0, 0, 0],
        telemetry: { depthError: 0, depthEffort: 0, yawError: 0, yawEffort: 0 },
        hydroTelemetry: this.hydroTelemetry,
      };
    }

    // =========================================================================
    // ARMED: Full physics pipeline — CLEAN STRAIGHT-LINE PRIORITY ALLOCATION
    // All 4 horizontal thrusters fire at EQUAL power for straight travel.
    // Steering is applied as a tiny multiplicative differential (max ±12%)
    // so the vehicle maintains strong forward thrust even while turning.
    // =========================================================================
    const targetSurge = (userCmd.surge || 0) * 0.75; // Responsive surge 0.75 m/s
    const targetSway = (userCmd.sway || 0) * 0.40;   // Lateral 0.40 m/s
    let targetYawRate = (userCmd.yaw || 0) * 0.55;    // Calm yaw rate 0.55 rad/s
    let targetHeave = 0;

    // 1. Closed-loop Depth PID Regulation
    if (flightMode === 'ALT_HOLD' || flightMode === 'STABILIZE' || flightMode === 'AUTO') {
      if (Math.abs(userCmd.heave) > 0.05) {
        this.targetDepth = Math.max(0.15, Math.min(1.85, currentPose.depth + userCmd.heave * dt * 0.45));
        this.depthPid.setSetpoint(this.targetDepth);
      }
      const depthRes = this.depthPid.compute(currentPose.depth);
      targetHeave = depthRes.output * 0.45;
      this.pidTelemetry.depthError = depthRes.error;
      this.pidTelemetry.depthEffort = depthRes.output;
    } else {
      targetHeave = (userCmd.heave || 0) * 0.45;
      this.targetDepth = currentPose.depth;
      this.depthPid.setSetpoint(this.targetDepth);
      this.pidTelemetry.depthError = 0;
      this.pidTelemetry.depthEffort = targetHeave;
    }

    // 2. Closed-loop Heading PID Regulation (course lock for straight cruising)
    if (Math.abs(userCmd.yaw) > 0.05) {
      this.targetHeading = currentPose.heading;
      this.yawPid.setSetpoint(this.targetHeading);
      this.pidTelemetry.yawError = 0;
      this.pidTelemetry.yawEffort = userCmd.yaw;
    } else {
      // Whenever user/autopilot is NOT steering (e.g. driving forward with 'W'),
      // hold target heading locked so vehicle travels in a laser-straight line
      const headingErr = Kinematics.wrapAngle(this.targetHeading - currentPose.heading);
      const yawRes = this.yawPid.compute(currentPose.heading);
      targetYawRate = yawRes.output * 0.55;
      this.pidTelemetry.yawError = headingErr;
      this.pidTelemetry.yawEffort = yawRes.output;
    }

    // 3. CLEAN THRUSTER ALLOCATION — Straight-line priority
    // ---------------------------------------------------------------
    // Base effort: ALL 4 horizontal thrusters get the SAME power for pure forward thrust
    const surgeNorm = targetSurge / 0.75;           // -1..+1
    const basePower = surgeNorm * 78;                // All thrusters at 78% for full surge
    
    // Sway effort (lateral strafe)
    const swayNorm = targetSway / 0.40;
    const swayPower = swayNorm * 30;

    // Yaw differential: additive torque so steering direction is identical in forward, reverse, and spot turn
    const yawNorm = Math.max(-1, Math.min(1, targetYawRate / 0.55));
    // When surging fast, moderate yaw differential for high-speed straight-line stability
    const maxDifferential = Math.abs(basePower) > 20 ? 12 : 28;
    const yawDiff = yawNorm * maxDifferential;

    // Decoupled thruster commands:
    // Left pair  (T1, T3): base + yawDiff
    // Right pair (T2, T4): base - yawDiff
    // Sway is orthogonal: T2 and T3 push right, T1 and T4 push left
    let cmd1 = basePower + yawDiff - swayPower;  // Front-Left
    let cmd2 = basePower - yawDiff + swayPower;  // Front-Right
    let cmd3 = basePower + yawDiff + swayPower;  // Rear-Left
    let cmd4 = basePower - yawDiff - swayPower;  // Rear-Right

    // Desaturation: normalize if any thruster exceeds 100%
    const maxCmd = Math.max(Math.abs(cmd1), Math.abs(cmd2), Math.abs(cmd3), Math.abs(cmd4));
    if (maxCmd > 100) {
      const scale = 100 / maxCmd;
      cmd1 *= scale;
      cmd2 *= scale;
      cmd3 *= scale;
      cmd4 *= scale;
    }

    const thrusterCmd1 = Math.max(-100, Math.min(100, cmd1));
    const thrusterCmd2 = Math.max(-100, Math.min(100, cmd2));
    const thrusterCmd3 = Math.max(-100, Math.min(100, cmd3));
    const thrusterCmd4 = Math.max(-100, Math.min(100, cmd4));

    // Vertical thrusters (heave + pitch) — responsive authority for diving and firm depth hold
    const tHeave = (targetHeave / 0.45) * 75;
    const tPitch = this.dynamicPitch * 1.2;
    const cmd5 = Math.max(-100, Math.min(100, tHeave + tPitch));
    const cmd6 = Math.max(-100, Math.min(100, tHeave - tPitch));

    const thrusterCommands = [thrusterCmd1, thrusterCmd2, thrusterCmd3, thrusterCmd4, cmd5, cmd6];

    // 4. Pass through Thruster Dynamics Model (ramp-up lag, deadband, voltage sag)
    const thrusterResult = thrusterDynamics.update(thrusterCommands, batteryVoltage, dt);

    // 5. Convert actual thruster output to body-frame forces
    const bodyForces = thrusterDynamics.thrustToBodyForces(thrusterResult.thrusts);

    // 6. Full Hydrodynamic Step
    const hydroResult = hydrodynamicsEngine.step(
      {
        velSurge: this.velSurge,
        velSway: this.velSway,
        velHeave: this.velHeave,
        velRoll: this.velRoll,
        velPitch: this.velPitch,
        velYaw: this.velYaw,
        heading: currentPose.heading || 0,
        roll: currentPose.roll || 0,
        pitch: currentPose.pitch || 0,
        depth: currentPose.depth || 0.8,
      },
      bodyForces,
      dt,
      time
    );

    // 7. Update state velocities
    this.velSurge = hydroResult.velSurge;
    this.velSway = hydroResult.velSway;
    this.velHeave = hydroResult.velHeave;
    this.velRoll = hydroResult.velRoll || 0;
    this.velPitch = hydroResult.velPitch || 0;
    this.velYaw = hydroResult.velYaw;

    // 8. Natural Hydrodynamic Tilt (calm and smooth)
    const targetPitchTilt = -this.velSurge * 1.5 + (this.velHeave * 1.0) + (hydroResult.pitchAccel || 0) * 0.15;
    const targetRollTilt = this.velSway * 1.5 + (this.velYaw * 0.8) + (hydroResult.rollAccel || 0) * 0.15;
    this.dynamicPitch += (targetPitchTilt - this.dynamicPitch) * Math.min(1, 2.0 * dt);
    this.dynamicRoll += (targetRollTilt - this.dynamicRoll) * Math.min(1, 2.0 * dt);

    // 9. Store debug telemetry
    this.hydroTelemetry = {
      currentDrift: hydroResult.currentDrift,
      dampingSurge: hydroResult.debug.dampSurge,
      dampingSway: hydroResult.debug.dampSway,
      sagFactor: thrusterResult.sagFactor,
      totalCurrent: thrusterResult.totalCurrent,
    };

    return {
      surge: this.velSurge,
      sway: this.velSway,
      yaw: this.velYaw,
      heave: this.velHeave,
      rollRate: this.velRoll,
      pitchRate: this.velPitch,
      pitch: this.dynamicPitch,
      roll: this.dynamicRoll,
      thrusters: thrusterResult.efforts,
      telemetry: { ...this.pidTelemetry },
      hydroTelemetry: this.hydroTelemetry,
    };
  }
}

const auvMotionController = new AUVMotionController();
export default auvMotionController;
