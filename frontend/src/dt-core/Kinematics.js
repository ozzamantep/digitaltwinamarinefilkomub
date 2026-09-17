/**
 * 6-DOF Marine Kinematics & Transformation Library
 * 
 * Implements standard Fossen (2011, 2021) kinematic transformations for
 * marine craft using both Euler angles (ZYX convention) and Unit Quaternions.
 * 
 * Coordinate Reference Frames:
 * 
 * 1. North-East-Down (NED) World Frame {n}:
 *    - x_n: North (forward along pool long axis)
 *    - y_n: East  (starboard along pool width)
 *    - z_n: Down  (positive downward, depth)
 * 
 * 2. Body-Fixed Frame {b}:
 *    - x_b: Forward (surge)
 *    - y_b: Starboard (sway)
 *    - z_b: Bottom/Down (heave)
 * 
 * 3. Three.js Render Frame {three}:
 *    - x_three = x_n
 *    - y_three = poolDepth - z_n  (upwards from pool floor)
 *    - z_three = y_n
 * 
 * State Vectors:
 *    η = [η_1ᵀ, η_2ᵀ]ᵀ = [x, y, z, φ, θ, ψ]ᵀ ∈ ℝ⁶
 *    ν = [ν_1ᵀ, ν_2ᵀ]ᵀ = [u, v, w, p, q, r]ᵀ ∈ ℝ⁶
 *    q = [qw, qx, qy, qz]ᵀ ∈ ℍ (unit quaternion, ||q|| = 1)
 * 
 * Kinematic Equations:
 *    η̇_1 = R_b^n(Θ) ν_1
 *    η̇_2 = T_Θ(Θ) ν_2
 *    q̇ = (1/2) q ⊗ [0, p, q, r]ᵀ
 * 
 * Reference:
 *    T. I. Fossen, "Handbook of Marine Craft Hydrodynamics and Motion Control",
 *    2nd ed., John Wiley & Sons, 2021, Chapters 2 & 3.
 */

export class Kinematics {
  /**
   * Wrap angle to range [-π, π]
   * @param {number} angle - Angle in radians
   * @returns {number} Normalized angle in [-π, π]
   */
  static wrapAngle(angle) {
    let a = angle % (2 * Math.PI);
    if (a > Math.PI) a -= 2 * Math.PI;
    if (a < -Math.PI) a += 2 * Math.PI;
    return a;
  }

  /**
   * Wrap angle in degrees to range [0, 360)
   * @param {number} deg - Angle in degrees
   * @returns {number} Angle in [0, 360)
   */
  static wrapAngle360(deg) {
    return ((deg % 360) + 360) % 360;
  }

  /**
   * Compute rotation matrix R_b^n(Θ) from body frame to NED frame
   * using Euler angles ZYX convention (yaw -> pitch -> roll)
   * 
   * @param {number} phi - Roll (rad)
   * @param {number} theta - Pitch (rad)
   * @param {number} psi - Yaw (rad)
   * @returns {number[][]} 3x3 rotation matrix
   */
  static rotationMatrixEuler(phi, theta, psi) {
    const cphi = Math.cos(phi), sphi = Math.sin(phi);
    const cth = Math.cos(theta), sth = Math.sin(theta);
    const cpsi = Math.cos(psi), spsi = Math.sin(psi);

    return [
      [cpsi * cth, -spsi * cphi + cpsi * sth * sphi,  spsi * sphi + cpsi * sth * cphi],
      [spsi * cth,  cpsi * cphi + spsi * sth * sphi, -cpsi * sphi + spsi * sth * cphi],
      [-sth,        cth * sphi,                       cth * cphi]
    ];
  }

  /**
   * Compute rotation matrix R(q) from unit quaternion
   * q = [qw, qx, qy, qz]
   * 
   * @param {{w: number, x: number, y: number, z: number} | number[]} q
   * @returns {number[][]} 3x3 rotation matrix
   */
  static rotationMatrixQuat(q) {
    const qw = q.w !== undefined ? q.w : q[0];
    const qx = q.x !== undefined ? q.x : q[1];
    const qy = q.y !== undefined ? q.y : q[2];
    const qz = q.z !== undefined ? q.z : q[3];

    const s = 2.0 / (qw * qw + qx * qx + qy * qy + qz * qz);

    const xs = qx * s,   ys = qy * s,   zs = qz * s;
    const wx = qw * xs,  wy = qw * ys,  wz = qw * zs;
    const xx = qx * xs,  xy = qx * ys,  xz = qx * zs;
    const yy = qy * ys,  yz = qy * zs,  zz = qz * zs;

    return [
      [1.0 - (yy + zz), xy - wz,          xz + wy],
      [xy + wz,          1.0 - (xx + zz), yz - wx],
      [xz - wy,          yz + wx,          1.0 - (xx + yy)]
    ];
  }

  /**
   * Compute Euler rate transformation matrix T_Θ(Θ)
   * η̇_2 = T_Θ(Θ) · ν_2
   * 
   * Note: Singular at θ = ±π/2 (pitch = ±90°)
   * 
   * @param {number} phi - Roll (rad)
   * @param {number} theta - Pitch (rad)
   * @returns {number[][]} 3x3 transformation matrix
   */
  static eulerRateMatrix(phi, theta) {
    const cphi = Math.cos(phi), sphi = Math.sin(phi);
    const cth = Math.cos(theta);
    // Protect against singularity at pitch ±90°
    const tanth = Math.abs(cth) > 1e-4 ? Math.tan(theta) : Math.sign(Math.sin(theta)) * 1e4;
    const secth = Math.abs(cth) > 1e-4 ? 1.0 / cth : 1e4;

    return [
      [1.0, sphi * tanth, cphi * tanth],
      [0.0, cphi,        -sphi],
      [0.0, sphi * secth, cphi * secth]
    ];
  }

  /**
   * Full 6x6 transformation matrix J(η)
   * η̇ = J(η) · ν
   * 
   * @param {number} phi - Roll (rad)
   * @param {number} theta - Pitch (rad)
   * @param {number} psi - Yaw (rad)
   * @returns {number[][]} 6x6 transformation matrix
   */
  static transformationMatrix6DOF(phi, theta, psi) {
    const R = this.rotationMatrixEuler(phi, theta, psi);
    const T = this.eulerRateMatrix(phi, theta);

    const J = Array(6).fill(0).map(() => Array(6).fill(0));
    for (let i = 0; i < 3; i++) {
      for (let j = 0; j < 3; j++) {
        J[i][j] = R[i][j];
        J[i + 3][j + 3] = T[i][j];
      }
    }
    return J;
  }

  /**
   * Transform body-fixed linear velocity [u, v, w] to NED rate of position [ẋ, ẏ, ż]
   * 
   * @param {{u: number, v: number, w: number} | number[]} vBody
   * @param {{roll: number, pitch: number, yaw: number} | {x: number, y: number, z: number, w: number}} att
   * @returns {{x: number, y: number, z: number}} World-frame velocity
   */
  static bodyToWorldVelocity(vBody, att) {
    const u = vBody.u !== undefined ? vBody.u : vBody[0] || 0;
    const v = vBody.v !== undefined ? vBody.v : vBody[1] || 0;
    const w = vBody.w !== undefined ? vBody.w : vBody[2] || 0;

    let R;
    if (att.w !== undefined) {
      R = this.rotationMatrixQuat(att);
    } else {
      R = this.rotationMatrixEuler(att.roll || 0, att.pitch || 0, att.yaw || 0);
    }

    return {
      x: R[0][0] * u + R[0][1] * v + R[0][2] * w,
      y: R[1][0] * u + R[1][1] * v + R[1][2] * w,
      z: R[2][0] * u + R[2][1] * v + R[2][2] * w,
    };
  }

  /**
   * Transform NED world velocity [ẋ, ẏ, ż] to body-fixed velocity [u, v, w]
   * v_body = R_b^n(Θ)ᵀ · v_world
   */
  static worldToBodyVelocity(vWorld, att) {
    const vx = vWorld.x !== undefined ? vWorld.x : vWorld[0] || 0;
    const vy = vWorld.y !== undefined ? vWorld.y : vWorld[1] || 0;
    const vz = vWorld.z !== undefined ? vWorld.z : vWorld[2] || 0;

    let R;
    if (att.w !== undefined) {
      R = this.rotationMatrixQuat(att);
    } else {
      R = this.rotationMatrixEuler(att.roll || 0, att.pitch || 0, att.yaw || 0);
    }

    // Transpose of R is Rᵀ = R⁻¹
    return {
      u: R[0][0] * vx + R[1][0] * vy + R[2][0] * vz,
      v: R[0][1] * vx + R[1][1] * vy + R[2][1] * vz,
      w: R[0][2] * vx + R[1][2] * vy + R[2][2] * vz,
    };
  }

  /**
   * Transform body angular rates [p, q, r] to Euler rates [φ̇, θ̇, ψ̇]
   */
  static bodyRatesToEulerRates(angRates, euler) {
    const p = angRates.p !== undefined ? angRates.p : angRates[0] || 0;
    const q = angRates.q !== undefined ? angRates.q : angRates[1] || 0;
    const r = angRates.r !== undefined ? angRates.r : angRates[2] || 0;

    const phi = euler.roll !== undefined ? euler.roll : euler[0] || 0;
    const theta = euler.pitch !== undefined ? euler.pitch : euler[1] || 0;

    const T = this.eulerRateMatrix(phi, theta);

    return {
      rollRate:  T[0][0] * p + T[0][1] * q + T[0][2] * r,
      pitchRate: T[1][0] * p + T[1][1] * q + T[1][2] * r,
      yawRate:   T[2][0] * p + T[2][1] * q + T[2][2] * r,
    };
  }

  /**
   * Convert Euler angles (roll, pitch, yaw in radians) to Unit Quaternion [w, x, y, z]
   * ZYX intrinsic rotation convention
   */
  static eulerToQuaternion(roll, pitch, yaw) {
    const cr = Math.cos(roll * 0.5),   sr = Math.sin(roll * 0.5);
    const cp = Math.cos(pitch * 0.5),  sp = Math.sin(pitch * 0.5);
    const cy = Math.cos(yaw * 0.5),    sy = Math.sin(yaw * 0.5);

    return {
      w: cr * cp * cy + sr * sp * sy,
      x: sr * cp * cy - cr * sp * sy,
      y: cr * sp * cy + sr * cp * sy,
      z: cr * cp * sy - sr * sp * cy,
    };
  }

  /**
   * Convert Unit Quaternion to Euler angles (radians, ZYX convention)
   */
  static quaternionToEuler(q) {
    const qw = q.w !== undefined ? q.w : q[0];
    const qx = q.x !== undefined ? q.x : q[1];
    const qy = q.y !== undefined ? q.y : q[2];
    const qz = q.z !== undefined ? q.z : q[3];

    // Roll (x-axis rotation)
    const sinr_cosp = 2.0 * (qw * qx + qy * qz);
    const cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
    const roll = Math.atan2(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    const sinp = 2.0 * (qw * qy - qz * qx);
    let pitch;
    if (Math.abs(sinp) >= 1) {
      pitch = Math.sign(sinp) * (Math.PI / 2.0); // Singularity clamp
    } else {
      pitch = Math.asin(sinp);
    }

    // Yaw (z-axis rotation)
    const siny_cosp = 2.0 * (qw * qz + qx * qy);
    const cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    const yaw = Math.atan2(siny_cosp, cosy_cosp);

    return { roll, pitch, yaw };
  }

  /**
   * Multiply two quaternions: q_out = q1 ⊗ q2
   */
  static quaternionMultiply(q1, q2) {
    const w1 = q1.w, x1 = q1.x, y1 = q1.y, z1 = q1.z;
    const w2 = q2.w, x2 = q2.x, y2 = q2.y, z2 = q2.z;

    return {
      w: w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
      x: w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
      y: w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
      z: w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    };
  }

  /**
   * Integrate unit quaternion forward in time using angular rates [p, q, r]
   * q̇ = (1/2) q ⊗ [0, p, q, r]ᵀ
   * 
   * Uses closed-form exponential map integration:
   * q(t + dt) = q(t) ⊗ exp(0.5 * ω * dt)
   * 
   * @param {{w: number, x: number, y: number, z: number}} q - Current quaternion
   * @param {{p: number, q: number, r: number}} rates - Angular velocities in rad/s
   * @param {number} dt - Time step in seconds
   * @returns {{w: number, x: number, y: number, z: number}} Updated unit quaternion
   */
  static integrateQuaternion(q, rates, dt) {
    const p = rates.p || 0;
    const qRate = rates.q || 0;
    const r = rates.r || 0;

    const omegaNorm = Math.sqrt(p * p + qRate * qRate + r * r);

    let dq;
    if (omegaNorm < 1e-8) {
      // First-order Taylor expansion for near-zero rotation
      const halfDt = 0.5 * dt;
      dq = {
        w: 1.0,
        x: p * halfDt,
        y: qRate * halfDt,
        z: r * halfDt
      };
    } else {
      // Exact exponential map
      const halfAngle = 0.5 * omegaNorm * dt;
      const sinc = Math.sin(halfAngle) / omegaNorm;
      dq = {
        w: Math.cos(halfAngle),
        x: p * sinc,
        y: qRate * sinc,
        z: r * sinc
      };
    }

    const qNew = this.quaternionMultiply(q, dq);
    return this.quaternionNormalize(qNew);
  }

  /**
   * Normalize a quaternion to enforce ||q|| = 1
   */
  static quaternionNormalize(q) {
    const norm = Math.sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (norm < 1e-12) return { w: 1, x: 0, y: 0, z: 0 };
    const inv = 1.0 / norm;
    return {
      w: q.w * inv,
      x: q.x * inv,
      y: q.y * inv,
      z: q.z * inv,
    };
  }

  /**
   * Convert NED world coordinates to Three.js coordinates
   * In SAUVC arena:
   * - NED x (North) -> Three.js x
   * - NED y (East)  -> Three.js z
   * - NED z (Depth) -> Three.js y = poolDepth - z_ned (default pool depth = 2.0m)
   * 
   * @param {number} xNed - North (m)
   * @param {number} yNed - East (m)
   * @param {number} zNed - Down/Depth (m)
   * @param {number} [poolDepth=2.0] - Total pool depth (m)
   * @returns {{x: number, y: number, z: number}} Three.js position
   */
  static nedToThree(xNed, yNed, zNed, poolDepth = 2.0) {
    return {
      x: xNed,
      y: poolDepth - zNed,
      z: yNed,
    };
  }

  /**
   * Convert Three.js coordinates to NED world coordinates
   */
  static threeToNed(xThree, yThree, zThree, poolDepth = 2.0) {
    return {
      x: xThree,
      y: zThree,
      z: poolDepth - yThree,
    };
  }

  /**
   * Convert NED Euler angles (roll, pitch, yaw) to Three.js Euler orientation
   * Three.js uses Y-up, right-handed.
   * Roll about x, Pitch about y (Three.js z), Yaw about z (Three.js -y).
   */
  static nedEulerToThreeEuler(roll, pitch, yaw) {
    return {
      rollThree: roll,
      pitchThree: pitch,
      yawThree: -yaw,
    };
  }
}

export default Kinematics;
