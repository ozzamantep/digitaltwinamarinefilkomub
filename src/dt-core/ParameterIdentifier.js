/**
 * Evolutionary Hydrodynamic Parameter Identification Engine
 * 
 * Implements bounded Differential Evolution (DE/rand/1/bin) to systematically
 * identify 6-DOF hydrodynamic parameters from recorded vehicle trajectory data:
 * - Added Mass: [X_udot, Y_vdot, Z_wdot, K_pdot, M_qdot, N_rdot]
 * - Linear Drag: [Xu, Yv, Zw, Kp, Mq, Nr]
 * - Quadratic Drag: [Xuu, Yvv, Zww, Kpp, Mqq, Nrr]
 * - Restoring Arm: [zB - zG]
 * 
 * Objective Function:
 *   J(θ) = w_pos * RMSE_pos(θ) + w_vel * RMSE_vel(θ) + w_att * RMSE_att(θ) + λ_reg * ||θ - θ_0||²
 * 
 * Reference:
 *   Fossen (2021) & Storn & Price (1997) "Differential Evolution"
 */

import vehicleConfig from './VehicleConfig.js';
import { HydrodynamicsEngine } from './HydrodynamicsEngine.js';

export class ParameterIdentifier {
  constructor(options = {}) {
    this.populationSize = options.populationSize || 20;
    this.maxGenerations = options.maxGenerations || 30;
    this.F = options.F || 0.7; // Mutation factor
    this.CR = options.CR || 0.8; // Crossover probability
    this.lambdaReg = options.lambdaReg || 0.01;

    this.isOptimizing = false;
    this.bestFitness = Infinity;
    this.bestParameters = null;
    this.history = [];
  }

  /**
   * Run parameter identification on a dataset of recorded mission steps
   * 
   * @param {Array<Object>} dataset - Array of { time, dt, input: {surge, sway, heave, yaw}, measured: { u, v, w, yawRate, depth } }
   * @param {Function} [onProgress] - Optional callback(generation, bestFitness)
   * @returns {Promise<{ optimalParams: Object, initialRMSE: number, finalRMSE: number, improvement: number }>}
   */
  async identify(dataset, onProgress = null) {
    if (!dataset || dataset.length < 10) {
      throw new Error('Dataset too small for parameter identification (minimum 10 samples required)');
    }

    this.isOptimizing = true;
    const { names, values: initialValues, bounds } = vehicleConfig.getIdentifiableParameters();
    const D = names.length;

    // Evaluate baseline fitness with current configuration
    const initialFitness = this.evaluateFitness(initialValues, names, dataset);

    // 1. Initialize DE Population within physical bounds
    let population = [];
    let fitness = [];

    // Individual 0 is the current nominal parameter vector
    population.push([...initialValues]);
    fitness.push(initialFitness);

    for (let i = 1; i < this.populationSize; i++) {
      const ind = [];
      for (let j = 0; j < D; j++) {
        const [minB, maxB] = bounds[j];
        const val = minB + Math.random() * (maxB - minB);
        ind.push(val);
      }
      population.push(ind);
      fitness.push(this.evaluateFitness(ind, names, dataset));
    }

    let bestIdx = fitness.indexOf(Math.min(...fitness));
    this.bestFitness = fitness[bestIdx];
    this.bestParameters = [...population[bestIdx]];

    // 2. Differential Evolution Main Loop
    for (let gen = 0; gen < this.maxGenerations; gen++) {
      for (let i = 0; i < this.populationSize; i++) {
        // Pick 3 distinct random individuals different from i
        let r1, r2, r3;
        do { r1 = Math.floor(Math.random() * this.populationSize); } while (r1 === i);
        do { r2 = Math.floor(Math.random() * this.populationSize); } while (r2 === i || r2 === r1);
        do { r3 = Math.floor(Math.random() * this.populationSize); } while (r3 === i || r3 === r1 || r3 === r2);

        // Mutation & Crossover
        const trial = [];
        const jRand = Math.floor(Math.random() * D);

        for (let j = 0; j < D; j++) {
          if (Math.random() < this.CR || j === jRand) {
            let v = population[r1][j] + this.F * (population[r2][j] - population[r3][j]);
            // Enforce bounds
            const [minB, maxB] = bounds[j];
            v = Math.max(minB, Math.min(maxB, v));
            trial.push(v);
          } else {
            trial.push(population[i][j]);
          }
        }

        // Selection
        const trialFitness = this.evaluateFitness(trial, names, dataset);
        if (trialFitness < fitness[i]) {
          population[i] = trial;
          fitness[i] = trialFitness;

          if (trialFitness < this.bestFitness) {
            this.bestFitness = trialFitness;
            this.bestParameters = [...trial];
          }
        }
      }

      this.history.push({ generation: gen + 1, bestFitness: this.bestFitness });
      if (onProgress) onProgress(gen + 1, this.bestFitness);

      // Yield event loop briefly
      if (gen % 5 === 0) {
        await new Promise(r => setTimeout(r, 0));
      }
    }

    this.isOptimizing = false;

    // Map best vector back to named dictionary
    const optimalParams = {};
    names.forEach((name, idx) => {
      optimalParams[name] = this.bestParameters[idx];
    });

    const finalRMSE = Math.sqrt(this.bestFitness);
    const initialRMSE = Math.sqrt(initialFitness);
    const improvementPercent = Math.max(0, ((initialRMSE - finalRMSE) / initialRMSE) * 100);

    return {
      optimalParams,
      initialRMSE: Number(initialRMSE.toFixed(4)),
      finalRMSE: Number(finalRMSE.toFixed(4)),
      improvementPercent: Number(improvementPercent.toFixed(1)),
    };
  }

  /**
   * Evaluate simulation RMSE fitness of a candidate parameter vector θ
   */
  evaluateFitness(paramVector, names, dataset) {
    const tempConfig = vehicleConfig.clone();
    names.forEach((name, idx) => {
      tempConfig.updateParameter(name, paramVector[idx]);
    });

    const engine = new HydrodynamicsEngine({ config: tempConfig });

    let sseVel = 0;
    let sseDepth = 0;

    let currState = {
      eta: [0, 0, dataset[0].measured?.depth || 0.8, 0, 0, 0],
      nu: [
        dataset[0].measured?.u || 0,
        dataset[0].measured?.v || 0,
        dataset[0].measured?.w || 0,
        0, 0,
        dataset[0].measured?.yawRate || 0
      ],
    };

    const thrustScale = 0.5;

    for (let k = 1; k < dataset.length; k++) {
      const sample = dataset[k];
      const dt = sample.dt || 0.05;
      const inp = sample.input || {};

      const tau = [
        (inp.surge || 0) * thrustScale,
        (inp.sway || 0) * thrustScale,
        (inp.heave || 0) * thrustScale,
        0, 0,
        (inp.yaw || 0) * thrustScale * 0.18,
      ];

      currState = engine.step(currState, tau, dt);

      // Compute error against measured data
      const meas = sample.measured || {};
      const errU = (currState.nu[0] - (meas.u || 0));
      const errV = (currState.nu[1] - (meas.v || 0));
      const errW = (currState.nu[2] - (meas.w || 0));
      const errDepth = (currState.eta[2] - (meas.depth || 0.8));

      sseVel += errU * errU + errV * errV + errW * errW;
      sseDepth += errDepth * errDepth;
    }

    const n = dataset.length - 1;
    const mse = (sseVel + 2.0 * sseDepth) / n;

    return mse;
  }
}

export const defaultParameterIdentifier = new ParameterIdentifier();
export default defaultParameterIdentifier;
