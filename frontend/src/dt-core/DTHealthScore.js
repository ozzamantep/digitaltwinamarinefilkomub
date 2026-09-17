/**
 * Digital Twin Composite Health Score Calculator
 * 
 * Computes unified Health Score H ∈ [0, 100] across 4 core operational pillars:
 * 1. Synchronization & Comm Health (30 pts): Latency, packet drop, clock jitter
 * 2. State Estimation & Sensor Quality (25 pts): EKF innovation residual, sensor sanity
 * 3. Physics & Prediction Fidelity (25 pts): 1-step & N-step validation RMSE, uncertainty
 * 4. Actuator & Parameter Health (20 pts): Motor voltage sag, OOD safety margin, bounds
 * 
 * Score Tiers:
 * - 90 - 100: OPTIMAL (High fidelity, bidirectional coupling safe)
 * - 75 - 89:  HEALTHY (Normal operational performance)
 * - 60 - 74:  DEGRADED (Minor sensor noise or latency, autonomy restricted)
 * - < 60:     CRITICAL (Out of distribution or desynced, manual pilot only)
 */

export const HEALTH_TIER = {
  OPTIMAL: 'OPTIMAL',
  HEALTHY: 'HEALTHY',
  DEGRADED: 'DEGRADED',
  CRITICAL: 'CRITICAL',
};

export class DTHealthScore {
  /**
   * Calculate overall health score from system subsystems
   * 
   * @param {Object} metrics
   * @param {Object} [metrics.sync] - { totalLatencyMs, packetRateHz, status }
   * @param {Object} [metrics.estimator] - { residuals: { depth, compass, dvl } }
   * @param {Object} [metrics.uncertainty] - { uncertainty, level }
   * @param {Object} [metrics.ood] - { isOOD, oodScore }
   * @param {Object} [metrics.battery] - { voltage, level }
   * @returns {{ totalScore: number, tier: string, breakdown: Object, recommendation: string }}
   */
  static evaluate(metrics = {}) {
    // 1. Synchronization Pillar (Max 30)
    let syncScore = 30.0;
    const latency = metrics.sync?.totalLatencyMs ?? 15.0;
    if (latency > 150) syncScore -= 18;
    else if (latency > 60) syncScore -= 8;

    const rate = metrics.sync?.packetRateHz ?? 20.0;
    if (rate < 5) syncScore -= 12;
    else if (rate < 15) syncScore -= 4;
    syncScore = Math.max(0, syncScore);

    // 2. Estimation Pillar (Max 25)
    let estScore = 25.0;
    const depthRes = Math.abs(metrics.estimator?.residuals?.depth ?? 0);
    if (depthRes > 0.3) estScore -= 12;
    else if (depthRes > 0.1) estScore -= 5;

    const compassRes = Math.abs(metrics.estimator?.residuals?.compass ?? 0);
    if (compassRes > 0.4) estScore -= 8;
    estScore = Math.max(0, estScore);

    // 3. Physics & Prediction Pillar (Max 25)
    let physScore = 25.0;
    const uncert = metrics.uncertainty?.uncertainty ?? 0.05;
    physScore -= uncert * 20.0;
    if (metrics.ood?.isOOD) physScore -= 10;
    physScore = Math.max(0, physScore);

    // 4. Actuator & Power Pillar (Max 20)
    let actScore = 20.0;
    const volt = metrics.battery?.voltage ?? 16.0;
    if (volt < 13.5) actScore -= 12;
    else if (volt < 14.8) actScore -= 5;
    actScore = Math.max(0, actScore);

    const total = Math.round(syncScore + estScore + physScore + actScore);

    let tier = HEALTH_TIER.OPTIMAL;
    let recommendation = 'Digital Twin fully synchronized and physically calibrated.';

    if (total < 60) {
      tier = HEALTH_TIER.CRITICAL;
      recommendation = 'Critical fidelity loss or packet dropout. Autonomy locked to fail-safe.';
    } else if (total < 75) {
      tier = HEALTH_TIER.DEGRADED;
      recommendation = 'Degraded sync or sensor residual detected. Speed limits enforced.';
    } else if (total < 90) {
      tier = HEALTH_TIER.HEALTHY;
      recommendation = 'Nominal subsea tracking performance.';
    }

    return {
      totalScore: total,
      tier,
      breakdown: {
        sync: Math.round(syncScore),
        estimation: Math.round(estScore),
        physics: Math.round(physScore),
        actuators: Math.round(actScore),
      },
      recommendation,
    };
  }
}

export default DTHealthScore;
