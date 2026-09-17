/**
 * Hydrodynamics Engine Service Wrapper
 * 
 * Bridges the UI and legacy simulation loop to the full 6-DOF
 * DT-Core HydrodynamicsEngine.
 * 
 * References:
 *   - Fossen (2021) Marine Craft Hydrodynamics and Motion Control
 *   - Single Source of Truth: src/dt-core/VehicleConfig.js
 */

import { HydrodynamicsEngine as DTHydrodynamicsEngine } from '../dt-core/HydrodynamicsEngine';
import vehicleConfig from '../dt-core/VehicleConfig';
import defaultEnvironment from '../dt-core/EnvironmentModel';
import Kinematics from '../dt-core/Kinematics';

class HydrodynamicsEngineService {
  constructor() {
    this.coreEngine = new DTHydrodynamicsEngine({
      config: vehicleConfig,
      environment: defaultEnvironment,
      integrator: 'rk4',
    });

    // Expose effective mass getters for backward compatibility
    this.updateEffectiveMass();
  }

  updateEffectiveMass() {
    this.effectiveMass = this.coreEngine.getEffectiveMass();
  }

  get mass() { return vehicleConfig.mass; }
  get buoyancy() { return vehicleConfig.get('centers').zB - vehicleConfig.get('centers').zG; }
  get gravity() { return defaultEnvironment.gravity; }
  get waterDensity() { return defaultEnvironment.waterDensity; }

  get dampingLinear() { return vehicleConfig.get('linearDamping'); }
  get dampingQuadratic() { return vehicleConfig.get('quadraticDamping'); }

  get Ixx() { return vehicleConfig.get('rigidBody').Ixx; }
  get Iyy() { return vehicleConfig.get('rigidBody').Iyy; }
  get Izz() { return vehicleConfig.get('rigidBody').Izz; }

  /**
   * Compute total damping force/moment for a given velocity
   */
  computeDamping(dof, velocity) {
    const dLin = vehicleConfig.get('linearDamping')[dof] || 0;
    const dQuad = vehicleConfig.get('quadraticDamping')[dof] || 0;
    return -(dLin * velocity + dQuad * Math.abs(velocity) * velocity);
  }

  /**
   * Legacy step method compatible with AUVMotionController
   * Maps 4-DOF input to full 6-DOF state integration
   */
  step(state, thrustForces, dt, time) {
    // Step environment
    defaultEnvironment.update(dt);

    const phi = state.roll || 0;
    const theta = state.pitch || 0;
    const psi = state.heading || 0;
    const depth = state.depth || 0.8;

    const u = state.velSurge || 0;
    const v = state.velSway || 0;
    const w = state.velHeave || 0;
    const r = state.velYaw || 0;
    const p = state.velRoll || 0;
    const q = state.velPitch || 0;

    const eta = [0, 0, depth, phi, theta, psi];
    const nu = [u, v, w, p, q, r];
    const quat = Kinematics.eulerToQuaternion(phi, theta, psi);

    // Thruster forces in 6-DOF
    // Thrust scale factor: command [-100..100] -> Newtons
    const thrustScale = 1.0;
    const tau = [
      (thrustForces.surge || 0) * thrustScale,
      (thrustForces.sway || 0) * thrustScale,
      (thrustForces.heave || 0) * thrustScale,
      (thrustForces.roll || 0) * 0.1,
      (thrustForces.pitch || 0) * 0.1,
      (thrustForces.yaw || 0) * 1.4 * (thrustForces.yawBoost || 1.0),
    ];

    const result = this.coreEngine.step({ eta, nu, quat }, tau, dt);

    const currentDrift = this.getCurrentDrift();

    return {
      velSurge: result.nu[0],
      velSway: result.nu[1],
      velHeave: result.nu[2],
      velRoll: result.nu[3],
      velPitch: result.nu[4],
      velYaw: result.nu[5],
      pitchAccel: result.nu_dot[4],
      rollAccel: result.nu_dot[3],
      currentDrift,
      debug: {
        dampSurge: result.forces.damping[0],
        dampSway: result.forces.damping[1],
        dampHeave: result.forces.damping[2],
        coriolisSurge: result.forces.coriolis[0],
        coriolisSway: result.forces.coriolis[1],
        restoringHeave: result.forces.restoring[2],
        restoringRoll: result.forces.restoring[3],
        restoringPitch: result.forces.restoring[4],
        relVelSurge: result.nu[0],
        relVelSway: result.nu[1],
      },
    };
  }

  getCurrentDrift() {
    const c = defaultEnvironment.getCurrentVelocity(0, 0, 1.0);
    return { x: c.vx, z: c.vy, magnitude: c.speed };
  }

  reset() {
    // Reset environment turbulence
    defaultEnvironment.setPreset(defaultEnvironment.preset);
  }
}

const hydrodynamicsEngine = new HydrodynamicsEngineService();
export default hydrodynamicsEngine;
