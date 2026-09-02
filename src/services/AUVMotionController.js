import PIDController from './PidController';
import hydrodynamicsEngine from './HydrodynamicsEngine';
import thrusterDynamics from './ThrusterDynamicsModel';

/**
 * 6-DOF Hydrodynamic Flight & PID Motion Controller for BlueROV2 AUV
 * 
 * NOW WITH REALISTIC PHYSICS:
 * - Full hydrodynamic model (added mass, nonlinear damping, Coriolis, restoring forces)
 * - Accurate T200 thruster dynamics (ramp-up lag, voltage sag, cross-coupling)
 * - Underwater current disturbance
 * - Proper force → acceleration → velocity integration through effective mass
 * 
 * Previous version used simplified drag coefficients and instant thruster response.
 * This version integrates HydrodynamicsEngine and ThrusterDynamicsModel for
 * physically accurate underwater vehicle motion.
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

    // Dynamic Physics State (Velocities in body frame)
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

    // Hydrodynamic debug telemetry
    this.hydroTelemetry = {
      currentDrift: { x: 0, z: 0, magnitude: 0 },
      dampingSurge: 0,
      dampingSway: 0,
      sagFactor: 1.0,
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
      // AUV coasts to a stop due to water drag, then floats up (positive buoyancy)
      // =========================================================================
      
      // Zero thrust commands → thruster dynamics still need to wind down
      thrusterDynamics.update([0, 0, 0, 0, 0, 0], batteryVoltage, dt);

      // Hydrodynamic step with zero thrust force
      const hydroResult = hydrodynamicsEngine.step(
        {
          velSurge: this.velSurge,
          velSway: this.velSway,
          velHeave: this.velHeave,
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
      this.velYaw = hydroResult.velYaw;

      // Natural Positive Buoyancy (Fail-safe float to surface: depth ~ 0.15m)
      const currentDepth = currentPose.depth || 0.8;
      if (currentDepth > 0.15) {
        // Buoyancy force is already computed in hydro engine's restoring forces
        // but we add a small explicit upward bias for fail-safe behavior
        this.velHeave = Math.max(-0.12, this.velHeave - 0.08 * dt);
      }

      // Dampen attitude
      this.dynamicPitch *= Math.max(0, 1 - 2.0 * dt);
      this.dynamicRoll *= Math.max(0, 1 - 2.0 * dt);

      this.hydroTelemetry.currentDrift = hydroResult.currentDrift;

      return {
        surge: this.velSurge,
        sway: this.velSway,
        yaw: this.velYaw,
        heave: this.velHeave,
        pitch: this.dynamicPitch,
        roll: this.dynamicRoll,
        thrusters: [0, 0, 0, 0, 0, 0],
        telemetry: { depthError: 0, depthEffort: 0, yawError: 0, yawEffort: 0 },
        hydroTelemetry: this.hydroTelemetry,
      };
    }

    // =========================================================================
    // ARMED: Full physics pipeline
    // User command → PID → Thruster allocation → Thruster dynamics → 
    // Body forces → Hydrodynamics → Velocity integration
    // =========================================================================

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

    // 3. Compute thruster allocation commands (percentage: -100 to +100)
    //    Using improved 6-thruster vectored allocation with stability priority
    const tSurge = (targetSurge / 1.3) * 65;
    const tSway = (targetSway / 1.1) * 65;
    const tYaw = (targetYawRate / 1.5) * 45;
    const tHeave = (targetHeave / 0.6) * 60;
    const tPitch = this.dynamicPitch * 4.0;

    // 3. Compute thruster allocation commands (percentage: -100 to +100)
    //    6-thruster vectored allocation with proportional yaw/sway steering
    let cmd1 = tSurge + tYaw - tSway;
    let cmd2 = tSurge - tYaw + tSway;
    let cmd3 = tSurge + tYaw + tSway;
    let cmd4 = tSurge - tYaw - tSway;
    
    // Desaturation logic: if commands saturate unevenly, re-normalize to maintain forward priority
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
    
    const cmd5 = Math.max(-100, Math.min(100, tHeave + tPitch));
    const cmd6 = Math.max(-100, Math.min(100, tHeave - tPitch));

    const thrusterCommands = [thrusterCmd1, thrusterCmd2, thrusterCmd3, thrusterCmd4, cmd5, cmd6];

    // 4. Pass through Thruster Dynamics Model (ramp-up, deadband, sag, coupling)
    const thrusterResult = thrusterDynamics.update(thrusterCommands, batteryVoltage, dt);

    // 5. Convert actual thruster output to body-frame forces
    const bodyForces = thrusterDynamics.thrustToBodyForces(thrusterResult.thrusts);

    // 6. Full Hydrodynamic Step (added mass, damping, Coriolis, restoring, current)
    const hydroResult = hydrodynamicsEngine.step(
      {
        velSurge: this.velSurge,
        velSway: this.velSway,
        velHeave: this.velHeave,
        velYaw: this.velYaw,
        heading: currentPose.heading || 0,
        roll: (currentPose.roll || 0),
        pitch: (currentPose.pitch || 0),
        depth: currentPose.depth || 0.8,
      },
      bodyForces,
      dt,
      time
    );

    // 7. Update velocities from hydrodynamic integration
    this.velSurge = hydroResult.velSurge;
    this.velSway = hydroResult.velSway;
    this.velHeave = hydroResult.velHeave;
    this.velYaw = hydroResult.velYaw;

    // 8. Natural Hydrodynamic Tilt (Banking & Pitch from acceleration forces)
    //    Now modulated by restoring forces from hydro engine
    const targetPitchTilt = -this.velSurge * 3.5 + (this.velHeave * 2.0) + hydroResult.pitchAccel * 0.5;
    const targetRollTilt = this.velSway * 4.2 + (this.velYaw * 1.8) + hydroResult.rollAccel * 0.5;
    this.dynamicPitch += (targetPitchTilt - this.dynamicPitch) * Math.min(1, 3.0 * dt);
    this.dynamicRoll += (targetRollTilt - this.dynamicRoll) * Math.min(1, 3.0 * dt);

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
