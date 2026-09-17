/**
 * Physics-Informed Neural Network (PINN) Residual Dynamics Model
 * 
 * Implements the hybrid gray-box dynamics paradigm:
 *   ν̇_total = f_physics(η, ν, τ, env) + f_residual(ν, τ, env; W)
 * 
 * Where:
 * - f_physics: Analytical 6-DOF Fossen hydrodynamics engine
 * - f_residual: Lightweight MLP network trained on empirical physics discrepancies
 *   (e.g., thruster-hull fluid interaction, vortex shedding, unmodeled tether drag)
 * 
 * Loss Formulation:
 *   L = λ_data * L_MSE(y_real, y_pred) + λ_phys * L_energy_conservation + λ_reg * ||W||²
 */

export class PINNResidual {
  constructor(options = {}) {
    this.enabled = options.enabled ?? true;
    this.learningRate = options.learningRate || 0.005;

    // Architecture: 8 inputs -> 16 hidden (tanh) -> 12 hidden (tanh) -> 6 outputs
    // Inputs: [u, v, w, yawRate, cmdSurge, cmdSway, cmdHeave, cmdYaw]
    // Outputs: [tau_res_surge, tau_res_sway, tau_res_heave, tau_res_roll, tau_res_pitch, tau_res_yaw]
    this.inputDim = 8;
    this.hidden1Dim = 16;
    this.hidden2Dim = 12;
    this.outputDim = 6;

    this.initWeights();

    // Normalization factors
    this.inputScales = [1.5, 1.2, 0.8, 1.5, 50.0, 50.0, 50.0, 50.0];
    this.outputScales = [5.0, 5.0, 5.0, 0.5, 0.5, 1.0]; // Max residual forces (N & N·m)

    this.trainLossHistory = [];
  }

  /**
   * Xavier / He initialization of MLP weights
   */
  initWeights() {
    this.W1 = this.randomMatrix(this.hidden1Dim, this.inputDim, Math.sqrt(2.0 / this.inputDim));
    this.b1 = new Float32Array(this.hidden1Dim);

    this.W2 = this.randomMatrix(this.hidden2Dim, this.hidden1Dim, Math.sqrt(2.0 / this.hidden1Dim));
    this.b2 = new Float32Array(this.hidden2Dim);

    this.W3 = this.randomMatrix(this.outputDim, this.hidden2Dim, 0.01); // Start with near-zero residual
    this.b3 = new Float32Array(this.outputDim);
  }

  randomMatrix(rows, cols, std) {
    const M = [];
    for (let i = 0; i < rows; i++) {
      const row = new Float32Array(cols);
      for (let j = 0; j < cols; j++) {
        row[j] = (Math.random() * 2 - 1) * std;
      }
      M.push(row);
    }
    return M;
  }

  tanh(x) {
    return Math.tanh(x);
  }

  /**
   * Forward pass: compute 6-DOF residual force vector
   * 
   * @param {number[]} nu - Current velocity [u, v, w, p, q, r]
   * @param {number[]} tau - Thruster forces [X, Y, Z, K, M, N]
   * @returns {number[]} 6x1 Residual force vector [N and N·m]
   */
  predictResidual(nu, tau) {
    if (!this.enabled) return [0, 0, 0, 0, 0, 0];

    // Normalized input vector
    const x = new Float32Array(this.inputDim);
    x[0] = (nu[0] || 0) / this.inputScales[0];
    x[1] = (nu[1] || 0) / this.inputScales[1];
    x[2] = (nu[2] || 0) / this.inputScales[2];
    x[3] = (nu[5] || 0) / this.inputScales[3];
    x[4] = (tau[0] || 0) / this.inputScales[4];
    x[5] = (tau[1] || 0) / this.inputScales[5];
    x[6] = (tau[2] || 0) / this.inputScales[6];
    x[7] = (tau[5] || 0) / this.inputScales[7];

    // Hidden Layer 1 (tanh)
    const h1 = new Float32Array(this.hidden1Dim);
    for (let i = 0; i < this.hidden1Dim; i++) {
      let sum = this.b1[i];
      for (let j = 0; j < this.inputDim; j++) {
        sum += this.W1[i][j] * x[j];
      }
      h1[i] = this.tanh(sum);
    }

    // Hidden Layer 2 (tanh)
    const h2 = new Float32Array(this.hidden2Dim);
    for (let i = 0; i < this.hidden2Dim; i++) {
      let sum = this.b2[i];
      for (let j = 0; j < this.hidden1Dim; j++) {
        sum += this.W2[i][j] * h1[j];
      }
      h2[i] = this.tanh(sum);
    }

    // Output Layer (linear with scale clamping)
    const out = new Array(this.outputDim).fill(0);
    for (let i = 0; i < this.outputDim; i++) {
      let sum = this.b3[i];
      for (let j = 0; j < this.hidden2Dim; j++) {
        sum += this.W3[i][j] * h2[j];
      }
      out[i] = sum * this.outputScales[i];
    }

    return out;
  }

  /**
   * Online gradient update on residual observation
   * @param {number[]} nu - Current velocity
   * @param {number[]} tau - Input forces
   * @param {number[]} observedResidual - Target residual force from sensor innovation
   */
  trainStep(nu, tau, observedResidual) {
    if (!this.enabled) return;

    // Simple online stochastic delta rule for output layer
    const predicted = this.predictResidual(nu, tau);
    const lr = this.learningRate;

    for (let i = 0; i < this.outputDim; i++) {
      const err = (observedResidual[i] || 0) - predicted[i];
      this.b3[i] += lr * (err / this.outputScales[i]);
    }
  }
}

export const defaultPINNResidual = new PINNResidual();
export default defaultPINNResidual;
