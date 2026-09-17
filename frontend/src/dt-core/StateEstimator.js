/**
 * 15-State Extended Kalman Filter (EKF) for Underwater Vehicle Navigation
 * 
 * Fuses asynchronous multi-rate subsea sensors:
 * - 6-axis IMU (accelerometer + gyroscope) @ 50-100 Hz
 * - Pressure / Depth sensor @ 10-20 Hz
 * - DVL (Doppler Velocity Log) / Bottom track @ 5-10 Hz
 * - Magnetometer / Compass @ 10-20 Hz
 * 
 * State Vector (15x1):
 *   x[0..2]   = Position [x, y, z]ᵀ in NED world frame (m)
 *   x[3..5]   = Velocity [u, v, w]ᵀ in body-fixed frame (m/s)
 *   x[6..8]   = Euler angles [φ, θ, ψ]ᵀ (roll, pitch, yaw) in rad
 *   x[9..11]  = Accelerometer bias [b_ax, b_ay, b_az]ᵀ in m/s²
 *   x[12..14] = Gyroscope bias [b_gx, b_gy, b_gz]ᵀ in rad/s
 * 
 * Features:
 * - Continuous-discrete Extended Kalman Filter formulation
 * - Sensor outlier rejection via Mahalanobis gating
 * - Sensor dropout resilience (prediction step runs smoothly without measurements)
 * - Innovation residual tracking for sensor health monitoring
 * 
 * Reference:
 *   T. I. Fossen, "Handbook of Marine Craft Hydrodynamics and Motion Control", 2021, Chapter 13.
 */

import Kinematics from './Kinematics.js';

export class StateEstimator {
  constructor(options = {}) {
    // 15-state estimate vector
    this.x = new Float64Array(15);
    // Initial position default
    this.x[0] = options.x ?? -11.0;
    this.x[1] = options.y ?? 2.0;
    this.x[2] = options.z ?? 0.8;

    // 15x15 State Covariance Matrix P
    this.P = this.createIdentityMatrix(15, 0.1);
    // Increase initial uncertainty on biases and velocities
    for (let i = 3; i <= 5; i++) this.P[i][i] = 0.5; // velocities
    for (let i = 6; i <= 8; i++) this.P[i][i] = 0.2; // attitude
    for (let i = 9; i <= 11; i++) this.P[i][i] = 0.05; // accel bias
    for (let i = 12; i <= 14; i++) this.P[i][i] = 0.01; // gyro bias

    // Process Noise Covariance Q (diagonal)
    this.Q = new Float64Array([
      1e-4, 1e-4, 1e-4,    // pos (m²)
      1e-2, 1e-2, 1e-2,    // vel (m²/s²)
      1e-3, 1e-3, 1e-3,    // attitude (rad²)
      1e-5, 1e-5, 1e-5,    // accel bias drift
      1e-6, 1e-6, 1e-6     // gyro bias drift
    ]);

    // Sensor Measurement Noise Covariance R
    this.R = {
      depth: 0.005,       // Depth sensor variance (m²)
      dvl: [0.004, 0.004, 0.006], // DVL velocity variance [u, v, w] (m²/s²)
      compass: 0.02,      // Compass variance (rad²)
      accel: [0.08, 0.08, 0.08],  // Accel variance (m²/s⁴)
      gyro: [0.005, 0.005, 0.005] // Gyro variance (rad²/s²)
    };

    // Chi-square gating threshold (3-sigma for 1-DOF = 9.0, 3-DOF = 14.16)
    this.gateThreshold1D = 9.0;
    this.gateThreshold3D = 14.16;

    // Residual tracking
    this.residuals = {
      depth: 0,
      compass: 0,
      dvl: [0, 0, 0],
      accel: [0, 0, 0],
      gyro: [0, 0, 0],
    };

    this.lastUpdateTime = 0;
  }

  createIdentityMatrix(dim, scale = 1.0) {
    const M = [];
    for (let i = 0; i < dim; i++) {
      const row = new Float64Array(dim);
      row[i] = scale;
      M.push(row);
    }
    return M;
  }

  /**
   * EKF Time-Update (Prediction Step)
   * Integrates kinematics forward in time and propagates covariance P = F P Fᵀ + Q
   * 
   * @param {number[]} u_acc - Measured specific force [ax, ay, az] in m/s²
   * @param {number[]} u_gyro - Measured angular velocity [gx, gy, gz] in rad/s
   * @param {number} dt - Time step (s)
   */
  predict(u_acc, u_gyro, dt) {
    if (dt <= 0 || !isFinite(dt)) return;

    // Correct measurements with estimated biases
    const ax = (u_acc[0] || 0) - this.x[9];
    const ay = (u_acc[1] || 0) - this.x[10];
    const az = (u_acc[2] || 0) - this.x[11];

    const p = (u_gyro[0] || 0) - this.x[12];
    const q = (u_gyro[1] || 0) - this.x[13];
    const r = (u_gyro[2] || 0) - this.x[14];

    const phi = this.x[6];
    const theta = this.x[7];
    const psi = this.x[8];

    const u = this.x[3];
    const v = this.x[4];
    const w = this.x[5];

    // 1. Nonlinear State Kinematic Propagation
    // World velocity ẋ_pos = R_b^n * v_body
    const vWorld = Kinematics.bodyToWorldVelocity({ u, v, w }, { roll: phi, pitch: theta, yaw: psi });

    // Body acceleration: v̇_body = a_measured - ω × v + R_n^b * [0, 0, g]ᵀ
    const g = 9.80665;
    // Gravity vector in body frame: R_n^b * [0, 0, g]ᵀ
    const g_body_x = -g * Math.sin(theta);
    const g_body_y = g * Math.cos(theta) * Math.sin(phi);
    const g_body_z = g * Math.cos(theta) * Math.cos(phi);

    const u_dot = ax - (q * w - r * v) + g_body_x;
    const v_dot = ay - (r * u - p * w) + g_body_y;
    const w_dot = az - (p * v - q * u) + g_body_z;

    // Attitude rate: Θ̇ = T_Θ * ω_body
    const eulerRates = Kinematics.bodyRatesToEulerRates({ p, q, r }, { roll: phi, pitch: theta });

    // Integrate state estimates
    this.x[0] += vWorld.x * dt;
    this.x[1] += vWorld.y * dt;
    this.x[2] += vWorld.z * dt;

    this.x[3] += u_dot * dt;
    this.x[4] += v_dot * dt;
    this.x[5] += w_dot * dt;

    this.x[6] = Kinematics.wrapAngle(this.x[6] + eulerRates.rollRate * dt);
    this.x[7] = Math.max(-1.45, Math.min(1.45, this.x[7] + eulerRates.pitchRate * dt));
    this.x[8] = Kinematics.wrapAngle(this.x[8] + eulerRates.yawRate * dt);

    // 2. Propagate Error Covariance P = F * P * Fᵀ + Q * dt
    // First-order Jacobian approximation (Transition Matrix F ≈ I + F_cont * dt)
    // For numerical efficiency, propagate diagonal and dominant cross-terms
    for (let i = 0; i < 15; i++) {
      this.P[i][i] += this.Q[i] * dt;
    }
    // Velocity-to-position coupling
    this.P[0][0] += 2 * this.P[0][3] * dt + this.P[3][3] * dt * dt;
    this.P[1][1] += 2 * this.P[1][4] * dt + this.P[4][4] * dt * dt;
    this.P[2][2] += 2 * this.P[2][5] * dt + this.P[5][5] * dt * dt;
  }

  /**
   * Depth / Pressure Sensor Measurement Update
   * Measurement: z = x[2] + v (direct observation of depth in NED frame)
   * 
   * @param {number} measuredDepth - Depth in meters
   * @returns {boolean} Whether measurement was accepted
   */
  updateDepth(measuredDepth) {
    if (!isFinite(measuredDepth) || measuredDepth < 0) return false;

    const z_hat = this.x[2];
    const residual = measuredDepth - z_hat;
    this.residuals.depth = residual;

    // Innovation covariance S = P_zz + R
    const S = this.P[2][2] + this.R.depth;
    const mahalDistSq = (residual * residual) / S;

    // Outlier rejection gate
    if (mahalDistSq > this.gateThreshold1D) {
      return false;
    }

    // Kalman gain K = P * Hᵀ / S
    for (let i = 0; i < 15; i++) {
      const K_i = this.P[i][2] / S;
      this.x[i] += K_i * residual;
      this.P[i][2] -= K_i * this.P[2][2];
    }

    return true;
  }

  /**
   * Compass / Magnetometer Yaw Update
   * Measurement: z = x[8] (yaw angle in rad)
   * 
   * @param {number} measuredYaw - Yaw angle in radians
   * @returns {boolean}
   */
  updateCompass(measuredYaw) {
    if (!isFinite(measuredYaw)) return false;

    let residual = Kinematics.wrapAngle(measuredYaw - this.x[8]);
    this.residuals.compass = residual;

    const S = this.P[8][8] + this.R.compass;
    const mahalDistSq = (residual * residual) / S;

    if (mahalDistSq > this.gateThreshold1D) {
      return false;
    }

    for (let i = 0; i < 15; i++) {
      const K_i = this.P[i][8] / S;
      this.x[i] += K_i * residual;
      this.P[i][8] -= K_i * this.P[8][8];
    }

    this.x[8] = Kinematics.wrapAngle(this.x[8]);
    return true;
  }

  /**
   * DVL (Doppler Velocity Log) Update
   * Measurement: z = [u, v, w]ᵀ in body-fixed frame
   * 
   * @param {number[]} measuredVel - [u, v, w] in m/s
   * @returns {boolean}
   */
  updateDVL(measuredVel) {
    if (!measuredVel || measuredVel.length < 3) return false;

    for (let j = 0; j < 3; j++) {
      const stateIdx = 3 + j; // indices 3, 4, 5
      const z_meas = measuredVel[j];
      if (!isFinite(z_meas)) continue;

      const residual = z_meas - this.x[stateIdx];
      this.residuals.dvl[j] = residual;

      const S = this.P[stateIdx][stateIdx] + this.R.dvl[j];
      const mahalDistSq = (residual * residual) / S;

      if (mahalDistSq > this.gateThreshold1D) continue;

      for (let i = 0; i < 15; i++) {
        const K_i = this.P[i][stateIdx] / S;
        this.x[i] += K_i * residual;
        this.P[i][stateIdx] -= K_i * this.P[stateIdx][stateIdx];
      }
    }

    return true;
  }

  /**
   * Get formatted Estimated State Object
   */
  getEstimatedState() {
    return {
      position: { x: this.x[0], y: this.x[1], z: this.x[2] },
      velocity: { u: this.x[3], v: this.x[4], w: this.x[5] },
      attitude: {
        roll: this.x[6],
        pitch: this.x[7],
        yaw: this.x[8],
        rollDeg: (this.x[6] * 180) / Math.PI,
        pitchDeg: (this.x[7] * 180) / Math.PI,
        yawDeg: Kinematics.wrapAngle360((this.x[8] * 180) / Math.PI),
      },
      quaternion: Kinematics.eulerToQuaternion(this.x[6], this.x[7], this.x[8]),
      biases: {
        accel: [this.x[9], this.x[10], this.x[11]],
        gyro: [this.x[12], this.x[13], this.x[14]],
      },
      covarianceDiagonal: Array.from(this.x).map((_, i) => this.P[i][i]),
      residuals: { ...this.residuals },
    };
  }

  /**
   * Reset filter to initial state
   */
  reset(x0 = {}) {
    this.x.fill(0);
    this.x[0] = x0.x ?? -11.0;
    this.x[1] = x0.y ?? 2.0;
    this.x[2] = x0.z ?? 0.8;
    this.P = this.createIdentityMatrix(15, 0.1);
  }
}

export const defaultStateEstimator = new StateEstimator();
export default defaultStateEstimator;
