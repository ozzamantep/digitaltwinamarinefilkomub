/**
 * Blue Robotics T200 Single Thruster Digital Twin Engine
 * 
 * Implements physics-based and empirical benchmark characteristics for T200 thruster:
 * 1. PWM to RPM transfer function (Deadband: 1475-1525 µs)
 * 2. RPM to Thrust transfer function: T = k_t * omega * |omega|
 * 3. Power, Current, and Thermal modeling
 * 4. Online Recursive Least Squares (RLS) System Identification for adaptive k_t estimation
 * 
 * References:
 * - Blue Robotics T200 Performance Charts (12V, 16V, 20V)
 * - Fossen (2021) Actuator Dynamics
 */

export class ThrusterTwinEngine {
  constructor(options = {}) {
    this.voltage = options.voltage || 16.0; // Nominal 4S LiPo = 16.0V
    this.deadbandMin = 1475;
    this.deadbandMax = 1525;
    this.neutralPwm = 1500;

    // Nominal empirical coefficients for T200 @ 16V
    // Max forward thrust ~5.25 kgf (~51.5 N) @ 1900 µs (~3800 RPM)
    // Max reverse thrust ~4.10 kgf (~40.2 N) @ 1100 µs (~3300 RPM)
    this.nominalKtForward = 3.55e-6; // N / RPM^2
    this.nominalKtReverse = 3.70e-6; // N / RPM^2

    // Estimated parameter via RLS
    this.estimatedKt = this.nominalKtForward;

    // RLS algorithm state
    this.lambda = 0.985; // Forgetting factor
    this.P = 1.0;        // Covariance
    this.rlsHistory = [];

    // Internal dynamic state (for filtering / motor inertia lag)
    this.currentRpm = 0;
    this.motorTimeConstant = 0.12; // ~120ms mechanical time constant
  }

  setVoltage(v) {
    this.voltage = Math.max(8.0, Math.min(24.0, v));
  }

  /**
   * Compute theoretical steady-state RPM from PWM input signal
   * @param {number} pwm - Microseconds (1100 - 1900)
   * @returns {number} Signed RPM (+ = forward, - = reverse)
   */
  pwmToNominalRpm(pwm) {
    if (pwm >= this.deadbandMin && pwm <= this.deadbandMax) {
      return 0;
    }

    const vFactor = Math.sqrt(this.voltage / 16.0);

    if (pwm > this.deadbandMax) {
      // Forward direction: 1525 -> 1900 µs (delta: 0 -> 375 µs)
      const delta = pwm - this.deadbandMax;
      // Polynomial fit to Blue Robotics 16V curve (max ~3800 RPM)
      const rpm = (10.45 * delta - 0.00085 * Math.pow(delta, 2)) * vFactor;
      return Math.min(3900, Math.max(0, rpm));
    } else {
      // Reverse direction: 1475 -> 1100 µs (delta: 0 -> 375 µs)
      const delta = this.deadbandMin - pwm;
      // Polynomial fit to reverse curve (max ~3300 RPM)
      const rpm = (9.15 * delta - 0.00078 * Math.pow(delta, 2)) * vFactor;
      return -Math.min(3400, Math.max(0, rpm));
    }
  }

  /**
   * Compute theoretical steady-state thrust from RPM
   * @param {number} rpm - Signed RPM
   * @param {boolean} useEstimatedKt - Whether to use RLS identified kt
   * @returns {number} Signed Thrust in Newtons
   */
  rpmToThrust(rpm, useEstimatedKt = false) {
    const isForward = rpm >= 0;
    let kt;

    if (useEstimatedKt && isForward) {
      kt = this.estimatedKt;
    } else {
      kt = isForward ? this.nominalKtForward : this.nominalKtReverse;
    }

    return kt * rpm * Math.abs(rpm);
  }

  /**
   * Predict electrical current draw (Amperes)
   */
  estimateCurrent(rpm, thrust) {
    if (Math.abs(rpm) < 10) return 0.25; // Quiescent ESC power ~4W @ 16V
    // Power relation: P ~ k_p * RPM^2.8, I = P / V
    const absRpm = Math.abs(rpm);
    const mechPower = Math.abs(thrust) * (absRpm * 0.001); // approximate hydraulic power
    const electricalPower = 4.0 + 1.65 * mechPower + Math.pow(absRpm / 1000, 2.7) * 4.2;
    return Math.min(28.0, electricalPower / this.voltage);
  }

  /**
   * Step the Digital Twin forward in time by dt seconds
   * Incorporates first-order motor lag & produces full DT telemetry
   * 
   * @param {number} pwm - Commanded PWM (1100-1900 µs)
   * @param {number} dt - Time step in seconds
   * @param {number|null} measuredThrust - Real physical thrust if available (for RLS)
   */
  step(pwm, dt = 0.05, measuredThrust = null) {
    // 1. Target theoretical steady-state values
    const targetRpm = this.pwmToNominalRpm(pwm);

    // 2. Dynamic motor inertia filtering: d(RPM)/dt = (target - current) / tau
    const alpha = Math.min(1.0, dt / this.motorTimeConstant);
    this.currentRpm += alpha * (targetRpm - this.currentRpm);

    // 3. Digital Twin predicted thrust
    const thrustDT = this.rpmToThrust(this.currentRpm, false);
    const currentDT = this.estimateCurrent(this.currentRpm, thrustDT);
    const powerDT = currentDT * this.voltage;

    // 4. Online RLS Adaptation if physical feedback is provided
    let errorThrust = 0;
    let efficiency = 100.0;

    if (measuredThrust !== null && Math.abs(this.currentRpm) > 500) {
      errorThrust = measuredThrust - thrustDT;
      this.updateRLS(this.currentRpm, measuredThrust);
      efficiency = (this.estimatedKt / this.nominalKtForward) * 100.0;
    }

    return {
      pwm,
      rpmDT: this.currentRpm,
      targetRpm,
      thrustDT,
      currentDT,
      powerDT,
      voltage: this.voltage,
      estimatedKt: this.estimatedKt,
      nominalKt: this.nominalKtForward,
      errorThrust,
      efficiency: Math.max(10, Math.min(150, efficiency)),
    };
  }

  /**
   * Online Recursive Least Squares (RLS) parameter identification
   * Identifies model: y = phi * kt where:
   *   y = measuredThrust
   *   phi = rpm * |rpm|
   */
  updateRLS(rpm, measuredThrust) {
    if (rpm <= 0) return; // Only calibrate forward regime for simplicity

    const phi = rpm * Math.abs(rpm);
    if (phi < 1e5) return; // Skip near-zero regressor

    // Gain: K = (P * phi) / (lambda + phi * P * phi)
    const Pphi = this.P * phi;
    const denom = this.lambda + phi * Pphi;
    const K = Pphi / denom;

    // Innovation / Prediction error: e = y - phi * theta_hat
    const predicted = phi * this.estimatedKt;
    const error = measuredThrust - predicted;

    // Parameter update with gentle bounding to prevent divergence
    const newKt = this.estimatedKt + K * error;
    const minKt = this.nominalKtForward * 0.4; // Min 40% (severe damage/cavitation)
    const maxKt = this.nominalKtForward * 1.6; // Max 160%

    this.estimatedKt = Math.max(minKt, Math.min(maxKt, newKt));

    // Covariance update: P = (P - K * phi * P) / lambda
    this.P = (this.P - K * phi * this.P) / this.lambda;

    // Prevent covariance explosion or collapse
    this.P = Math.max(1e-4, Math.min(10.0, this.P));
  }

  /**
   * Reset RLS estimated parameter to nominal
   */
  resetRLS() {
    this.estimatedKt = this.nominalKtForward;
    this.P = 1.0;
  }
}

export default new ThrusterTwinEngine();
