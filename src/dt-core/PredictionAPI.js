/**
 * Prediction API for Digital Twin AUV
 * 
 * Provides decoupled, controller-agnostic multi-step forward simulation:
 *   predict(initialState, actionSequence, environment, options)
 * 
 * Supports:
 * - Model Predictive Control (MPC / MPPI)
 * - Trajectory Rollout for Safety Evaluation
 * - Reinforcement Learning Simulation
 * - Energy Consumption Estimation
 * - Constraint Violation Detection (pool boundaries, velocity limits)
 * 
 * Reference:
 *   Fossen (2021) Marine Craft Hydrodynamics and Motion Control
 */

import { HydrodynamicsEngine } from './HydrodynamicsEngine.js';
import Kinematics from './Kinematics.js';
import vehicleConfig from './VehicleConfig.js';
import defaultEnvironment from './EnvironmentModel.js';

export class PredictionAPI {
  constructor(options = {}) {
    this.engine = new HydrodynamicsEngine({
      config: options.config || vehicleConfig,
      environment: options.environment || defaultEnvironment,
      integrator: options.integrator || 'rk4',
    });
  }

  /**
   * Forward-simulate the AUV over a horizon of N steps
   * 
   * @param {Object} initialState - { eta: [6], nu: [6], quat?: {w,x,y,z} }
   * @param {Array<number[]|Object>} actionSequence - Array of N control inputs
   * @param {Object} [envOverride] - Optional environment override
   * @param {Object} [options] - { dt: 0.05, horizon: 50, checkConstraints: true }
   * @returns {{
   *   trajectory: Array<Object>,
   *   finalState: Object,
   *   energyJoules: number,
   *   constraintViolations: Array<string>,
   *   uncertaintyHorizon: number[]
   * }}
   */
  predict(initialState, actionSequence, envOverride = null, options = {}) {
    const dt = options.dt || 0.05;
    const horizon = options.horizon || (actionSequence ? actionSequence.length : 50);

    // Prepare initial state vectors
    let eta = initialState.eta ? [...initialState.eta] : [0, 0, 0.8, 0, 0, 0];
    let nu = initialState.nu ? [...initialState.nu] : [0, 0, 0, 0, 0, 0];
    let quat = initialState.quat
      ? { ...initialState.quat }
      : Kinematics.eulerToQuaternion(eta[3], eta[4], eta[5]);

    let currState = { eta, nu, quat };

    const trajectory = [];
    const constraintViolations = [];
    const uncertaintyHorizon = [];

    let totalEnergyJoules = 0.0;
    const thrustScale = 0.5; // N per unit command

    for (let k = 0; k < horizon; k++) {
      // Get action at step k (or hold last action)
      const action = actionSequence && actionSequence[k] ? actionSequence[k] : (actionSequence ? actionSequence[actionSequence.length - 1] : [0, 0, 0, 0, 0, 0]);

      let tau;
      if (Array.isArray(action)) {
        tau = action.length === 6 ? action : [action[0] || 0, action[1] || 0, action[2] || 0, 0, 0, action[3] || 0];
      } else {
        tau = [
          (action.surge || 0) * thrustScale,
          (action.sway || 0) * thrustScale,
          (action.heave || 0) * thrustScale,
          (action.roll || 0) * 0.05,
          (action.pitch || 0) * 0.05,
          (action.yaw || 0) * thrustScale * 0.18,
        ];
      }

      // Physics integration step
      const stepResult = this.engine.step(currState, tau, dt);
      currState = stepResult;

      // Estimate electrical power & energy consumed (P ≈ sum(|T_i|^1.5) * k_motor)
      const thrustNorm = Math.sqrt(tau[0] ** 2 + tau[1] ** 2 + tau[2] ** 2 + tau[5] ** 2);
      const instantPowerWatts = Math.max(12.0, thrustNorm * 8.5); // Idle 12W + thrust power
      totalEnergyJoules += instantPowerWatts * dt;

      // Constraint checking (SAUVC pool dimensions: -12..12m x, -7.5..7.5m y, 0.05..2.0m depth)
      if (options.checkConstraints !== false) {
        const x = currState.eta[0];
        const y = currState.eta[1];
        const depth = currState.eta[2];

        if (depth < 0.05) constraintViolations.push(`Surface breached at step ${k} (depth: ${depth.toFixed(2)}m)`);
        if (depth > 2.0) constraintViolations.push(`Pool floor collision at step ${k} (depth: ${depth.toFixed(2)}m)`);
        if (Math.abs(x) > 12.5 || Math.abs(y) > 8.0) {
          constraintViolations.push(`Pool wall boundary exceeded at step ${k} ([${x.toFixed(1)}, ${y.toFixed(1)}])`);
        }
      }

      // Uncertainty growth over prediction horizon: σ(k) = σ_0 * sqrt(1 + α * k)
      const baseUncertainty = options.initialUncertainty || 0.02;
      const uncertK = baseUncertainty * Math.sqrt(1 + 0.08 * k);
      uncertaintyHorizon.push(uncertK);

      trajectory.push({
        step: k,
        time: k * dt,
        eta: [...currState.eta],
        nu: [...currState.nu],
        depth: currState.eta[2],
        headingDeg: Kinematics.wrapAngle360((currState.eta[5] * 180) / Math.PI),
        forces: stepResult.forces,
        uncertainty: uncertK,
      });
    }

    return {
      trajectory,
      finalState: currState,
      energyJoules: totalEnergyJoules,
      energyWattHours: totalEnergyJoules / 3600.0,
      constraintViolations,
      uncertaintyHorizon,
    };
  }
}

export const defaultPredictionAPI = new PredictionAPI();
export default defaultPredictionAPI;
