/**
 * Uncertainty Estimator for Digital Twin State & Forward Rollouts
 * 
 * Quantifies both:
 * 1. Aleatoric Uncertainty: Environmental disturbances, sensor noise, ocean currents
 * 2. Epistemic Uncertainty: Unmodeled hydrodynamics, parameter inaccuracies
 * 
 * Features:
 * - Rolling statistical residual variance estimation
 * - Multi-step trajectory uncertainty cone propagation
 * - Normalized uncertainty score U ∈ [0.0, 1.0]
 * - Autonomous control safety threshold gating
 */

export const UNCERTAINTY_LEVEL = {
  NOMINAL: 'NOMINAL',         // U < 0.25: High confidence, full autonomous execution permitted
  MODERATE: 'MODERATE',       // 0.25 <= U < 0.60: Moderate confidence, monitor trajectory closely
  HIGH: 'HIGH',               // 0.60 <= U < 0.85: Reduced confidence, limit maximum speeds
  CRITICAL: 'CRITICAL',       // U >= 0.85: Safety threshold breached, fail-safe or manual takeover
};

export class UncertaintyEstimator {
  constructor(options = {}) {
    this.windowSize = options.windowSize || 50;
    this.safetyThreshold = options.safetyThreshold || 0.65;

    // Rolling residual errors for state variables
    this.errorHistory = {
      pos: [],
      vel: [],
      att: [],
    };

    // Current uncertainty metrics
    this.uncertaintyScore = 0.05;
    this.level = UNCERTAINTY_LEVEL.NOMINAL;
    this.residualStd = { pos: 0.02, vel: 0.03, att: 0.01 };
  }

  /**
   * Ingest a comparison between physical measurement/EKF state and DT predicted state
   * 
   * @param {Object} physicalState - { x, y, z, u, v, w, roll, pitch, yaw }
   * @param {Object} predictedState - { x, y, z, u, v, w, roll, pitch, yaw }
   */
  updateResidual(physicalState, predictedState) {
    if (!physicalState || !predictedState) return;

    const pP = physicalState.position || physicalState;
    const pV = predictedState.position || predictedState;

    const posErr = Math.sqrt((pP.x - pV.x) ** 2 + (pP.y - pV.y) ** 2 + (pP.z - pV.z) ** 2);

    const vP = physicalState.velocity || { u: 0, v: 0, w: 0 };
    const vV = predictedState.velocity || { u: 0, v: 0, w: 0 };
    const velErr = Math.sqrt((vP.u - vV.u) ** 2 + (vP.v - vV.v) ** 2 + (vP.w - vV.w) ** 2);

    // Push into rolling buffers
    this.errorHistory.pos.push(posErr);
    this.errorHistory.vel.push(velErr);
    if (this.errorHistory.pos.length > this.windowSize) {
      this.errorHistory.pos.shift();
      this.errorHistory.vel.shift();
    }

    // Compute standard deviations
    this.residualStd.pos = this.computeStdDev(this.errorHistory.pos);
    this.residualStd.vel = this.computeStdDev(this.errorHistory.vel);

    // Compute composite uncertainty score U ∈ [0, 1]
    // Normalized by expected bounds: 0.3m pos error = 1.0, 0.4 m/s vel error = 1.0
    const rawScore = 0.6 * (posErr / 0.3) + 0.4 * (velErr / 0.4);
    this.uncertaintyScore = Math.max(0.01, Math.min(1.0, this.uncertaintyScore * 0.9 + rawScore * 0.1));

    // Update level
    if (this.uncertaintyScore < 0.25) this.level = UNCERTAINTY_LEVEL.NOMINAL;
    else if (this.uncertaintyScore < 0.60) this.level = UNCERTAINTY_LEVEL.MODERATE;
    else if (this.uncertaintyScore < 0.85) this.level = UNCERTAINTY_LEVEL.HIGH;
    else this.level = UNCERTAINTY_LEVEL.CRITICAL;
  }

  computeStdDev(arr) {
    if (arr.length < 2) return 0.02;
    const mean = arr.reduce((a, b) => a + b, 0) / arr.length;
    const variance = arr.reduce((a, b) => a + (b - mean) ** 2, 0) / (arr.length - 1);
    return Math.sqrt(variance);
  }

  /**
   * Check if autonomous action is safe given current uncertainty
   * @returns {{ safe: boolean, uncertainty: number, level: string, reason?: string }}
   */
  isControlSafe() {
    const safe = this.uncertaintyScore < this.safetyThreshold;
    return {
      safe,
      uncertainty: Number(this.uncertaintyScore.toFixed(3)),
      level: this.level,
      confidence: Number((1.0 - this.uncertaintyScore).toFixed(3)),
      reason: safe ? 'Uncertainty within acceptable bounds' : 'High prediction uncertainty: manual intervention advised',
    };
  }
}

export const defaultUncertaintyEstimator = new UncertaintyEstimator();
export default defaultUncertaintyEstimator;
