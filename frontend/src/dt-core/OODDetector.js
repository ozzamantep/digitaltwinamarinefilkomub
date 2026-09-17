/**
 * Out-of-Distribution (OOD) & Distribution Shift Detector
 * 
 * Detects when the underwater vehicle operates in regimes far from the
 * calibrated training envelope (e.g. extreme ocean currents, tether entanglement,
 * sudden center-of-mass shift, or structural damage).
 * 
 * Algorithm:
 * - Multidimensional feature extraction: [speed, angular_rate, depth_rate, accel_norm, thrust_norm]
 * - Feature normalization using calibrated training set mean and standard deviation
 * - Standardized Mahalanobis distance metric: d_M = sqrt( sum( ((x_i - μ_i) / σ_i)^2 ) )
 * - Triggers OOD alerts and requests parameter recalibration
 */

export const OOD_STATE = {
  IN_DISTRIBUTION: 'IN_DISTRIBUTION',
  MARGINAL: 'MARGINAL_REGIME',
  OUT_OF_DISTRIBUTION: 'OUT_OF_DISTRIBUTION',
};

export class OODDetector {
  constructor(options = {}) {
    // Calibrated nominal training envelope (Mean μ and Std Dev σ for 5 key features)
    this.featureStats = {
      speedLinear: { mean: 0.35, std: 0.30, max: 1.6 },      // m/s
      speedAngular: { mean: 0.20, std: 0.25, max: 2.2 },     // rad/s
      accelMag: { mean: 9.85, std: 1.2, max: 18.0 },         // m/s²
      depth: { mean: 0.90, std: 0.45, max: 2.1 },            // m
      thrustEffort: { mean: 35.0, std: 25.0, max: 100.0 },   // %
    };

    this.oodThreshold = options.oodThreshold || 3.5; // Normalized Mahalanobis threshold
    this.currentScore = 0.0;
    this.state = OOD_STATE.IN_DISTRIBUTION;
    this.shiftEvents = [];
  }

  /**
   * Evaluate a telemetry state snapshot for OOD shift
   * 
   * @param {Object} state - { speed, imu, depth, thrusters }
   * @returns {{ oodScore: number, state: string, isOOD: boolean, outlierFeatures: string[] }}
   */
  evaluate(state) {
    const outlierFeatures = [];

    const u = state.speed?.surge || 0;
    const v = state.speed?.sway || 0;
    const w = state.speed?.heave || 0;
    const linSpeed = Math.sqrt(u * u + v * v + w * w);

    const angSpeed = Math.abs(state.speed?.angular || 0);

    const ax = state.imu?.accelX || 0;
    const ay = state.imu?.accelY || 0;
    const az = state.imu?.accelZ || -9.81;
    const accelMag = Math.sqrt(ax * ax + ay * ay + az * az);

    const depth = state.depth || 0.8;

    const thrusters = state.thrusters || [0, 0, 0, 0, 0, 0];
    const meanThrust = thrusters.reduce((a, b) => a + Math.abs(b), 0) / (thrusters.length || 1);

    // Compute normalized z-scores
    const zScores = {
      speedLinear: Math.abs(linSpeed - this.featureStats.speedLinear.mean) / this.featureStats.speedLinear.std,
      speedAngular: Math.abs(angSpeed - this.featureStats.speedAngular.mean) / this.featureStats.speedAngular.std,
      accelMag: Math.abs(accelMag - this.featureStats.accelMag.mean) / this.featureStats.accelMag.std,
      depth: Math.abs(depth - this.featureStats.depth.mean) / this.featureStats.depth.std,
      thrustEffort: Math.abs(meanThrust - this.featureStats.thrustEffort.mean) / this.featureStats.thrustEffort.std,
    };

    let sumSq = 0;
    for (const [key, z] of Object.entries(zScores)) {
      sumSq += z * z;
      if (z > 2.8) outlierFeatures.push(key);
    }

    const dMahalanobis = Math.sqrt(sumSq / Object.keys(zScores).length);
    this.currentScore = Number(dMahalanobis.toFixed(2));

    if (this.currentScore > this.oodThreshold) {
      this.state = OOD_STATE.OUT_OF_DISTRIBUTION;
      if (this.shiftEvents.length === 0 || Date.now() - this.shiftEvents[this.shiftEvents.length - 1].time > 5000) {
        this.shiftEvents.push({
          time: Date.now(),
          score: this.currentScore,
          outliers: [...outlierFeatures],
        });
      }
    } else if (this.currentScore > this.oodThreshold * 0.7) {
      this.state = OOD_STATE.MARGINAL;
    } else {
      this.state = OOD_STATE.IN_DISTRIBUTION;
    }

    return {
      oodScore: this.currentScore,
      state: this.state,
      isOOD: this.state === OOD_STATE.OUT_OF_DISTRIBUTION,
      outlierFeatures,
    };
  }
}

export const defaultOODDetector = new OODDetector();
export default defaultOODDetector;
