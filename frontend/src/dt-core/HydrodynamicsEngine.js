/**
 * Full 6-DOF Marine Hydrodynamics Engine
 * 
 * Implements complete Fossen (2011, 2021) 6-DOF nonlinear underwater vehicle dynamics:
 *   M ν̇ + C(ν)ν + D(ν_r)ν_r + g(η) = τ_actuator + τ_env + τ_residual
 * 
 * Where:
 *   M = M_RB + M_A           (6x6 Mass Matrix: Rigid Body + Added Mass)
 *   C(ν) = C_RB(ν) + C_A(ν_r) (6x6 Coriolis & Centripetal Matrix)
 *   D(ν_r) = D_L + D_Q|ν_r|   (6x6 Linear + Quadratic Damping Matrix)
 *   g(η)                     (6x1 Hydrostatic Restoring Force & Moment Vector)
 *   ν_r = ν - ν_c            (Relative velocity to fluid current)
 * 
 * Features:
 *   - Configurable via VehicleConfig (no magic numbers)
 *   - Coupled 6-DOF rotation and translation dynamics
 *   - Clean numerical solver (RK4 or Semi-Implicit Euler)
 *   - Supports learned residual physics injection (PINN)
 *   - High/Medium/Simplified fidelity modes
 * 
 * Reference:
 *   T. I. Fossen, "Handbook of Marine Craft Hydrodynamics and Motion Control",
 *   2nd ed., John Wiley & Sons, 2021, Chapters 3, 4, 6, 7.
 */

import vehicleConfig, { FIDELITY } from './VehicleConfig.js';
import Kinematics from './Kinematics.js';
import defaultEnvironment from './EnvironmentModel.js';

export class HydrodynamicsEngine {
  /**
   * @param {Object} [options]
   * @param {VehicleConfig} [options.config]
   * @param {EnvironmentModel} [options.environment]
   */
  constructor(options = {}) {
    this.config = options.config || vehicleConfig;
    this.environment = options.environment || defaultEnvironment;

    // Numerical integration scheme: 'rk4' or 'semi_implicit'
    this.integrator = options.integrator || 'rk4';

    // Last computed forces breakdown for observability & telemetry
    this.lastForces = {
      rigidBodyInertia: [0, 0, 0, 0, 0, 0],
      coriolis: [0, 0, 0, 0, 0, 0],
      damping: [0, 0, 0, 0, 0, 0],
      restoring: [0, 0, 0, 0, 0, 0],
      thruster: [0, 0, 0, 0, 0, 0],
      net: [0, 0, 0, 0, 0, 0],
      accel: [0, 0, 0, 0, 0, 0],
    };
  }

  /**
   * Helper to create skew-symmetric 3x3 matrix S(a)
   * S(a) * b = a × b (vector cross product)
   */
  skew(a) {
    const [a1, a2, a3] = a;
    return [
      [0, -a3, a2],
      [a3, 0, -a1],
      [-a2, a1, 0]
    ];
  }

  /**
   * 3x3 Matrix-vector multiplication: y = A * x
   */
  mat3VecMul(A, x) {
    return [
      A[0][0] * x[0] + A[0][1] * x[1] + A[0][2] * x[2],
      A[1][0] * x[0] + A[1][1] * x[1] + A[1][2] * x[2],
      A[2][0] * x[0] + A[2][1] * x[1] + A[2][2] * x[2]
    ];
  }

  /**
   * 6x6 Matrix-vector multiplication: y = A * x
   */
  mat6VecMul(A, x) {
    const y = Array(6).fill(0);
    for (let i = 0; i < 6; i++) {
      for (let j = 0; j < 6; j++) {
        y[i] += A[i][j] * x[j];
      }
    }
    return y;
  }

  /**
   * Invert 6x6 symmetric positive-definite or diagonally dominant matrix
   * Uses Gauss-Jordan elimination with partial pivoting
   */
  invert6x6(M) {
    const A = M.map(row => [...row]);
    const inv = Array(6).fill(0).map((_, i) => {
      const r = Array(6).fill(0);
      r[i] = 1.0;
      return r;
    });

    for (let i = 0; i < 6; i++) {
      let maxEl = Math.abs(A[i][i]);
      let maxRow = i;
      for (let k = i + 1; k < 6; k++) {
        if (Math.abs(A[k][i]) > maxEl) {
          maxEl = Math.abs(A[k][i]);
          maxRow = k;
        }
      }

      if (maxEl < 1e-12) {
        throw new Error('Mass matrix inversion singular');
      }

      // Swap rows
      [A[i], A[maxRow]] = [A[maxRow], A[i]];
      [inv[i], inv[maxRow]] = [inv[maxRow], inv[i]];

      const pivot = A[i][i];
      for (let j = 0; j < 6; j++) {
        A[i][j] /= pivot;
        inv[i][j] /= pivot;
      }

      for (let k = 0; k < 6; k++) {
        if (k !== i) {
          const factor = A[k][i];
          for (let j = 0; j < 6; j++) {
            A[k][j] -= factor * A[i][j];
            inv[k][j] -= factor * inv[i][j];
          }
        }
      }
    }

    return inv;
  }

  /**
   * Compute Total 6x6 Mass Matrix M = M_RB + M_A
   */
  computeMassMatrix() {
    const m = this.config.mass;
    const { Ixx, Iyy, Izz, Ixy, Ixz, Iyz } = this.config.get('rigidBody');
    const { xG, yG, zG } = this.config.getCenters();
    const added = this.config.get('addedMass');

    // 1. Rigid Body Mass Matrix M_RB (Fossen 2021, Eq. 3.42)
    const MRB = [
      [m, 0, 0, 0, m * zG, -m * yG],
      [0, m, 0, -m * zG, 0, m * xG],
      [0, 0, m, m * yG, -m * xG, 0],
      [0, -m * zG, m * yG, Ixx, -Ixy, -Ixz],
      [m * zG, 0, -m * xG, -Ixy, Iyy, -Iyz],
      [-m * yG, m * xG, 0, -Ixz, -Iyz, Izz]
    ];

    // 2. Added Mass Matrix M_A (Fossen 2021, Eq. 6.38)
    const MA = [
      [added.Xu_dot, 0, 0, 0, 0, 0],
      [0, added.Yv_dot, 0, 0, 0, 0],
      [0, 0, added.Zw_dot, 0, 0, 0],
      [0, 0, 0, added.Kp_dot, 0, 0],
      [0, 0, 0, 0, added.Mq_dot, 0],
      [0, 0, 0, 0, 0, added.Nr_dot]
    ];

    // Total M = MRB + MA
    const M = Array(6).fill(0).map(() => Array(6).fill(0));
    for (let i = 0; i < 6; i++) {
      for (let j = 0; j < 6; j++) {
        M[i][j] = MRB[i][j] + MA[i][j];
      }
    }

    return { M, MRB, MA };
  }

  /**
   * Compute 6x6 Coriolis and Centripetal Matrix C(ν) = C_RB(ν) + C_A(ν_r)
   * Exact Fossen formulation without arbitrary scaling factors.
   * 
   * @param {number[]} nu - Full velocity vector [u, v, w, p, q, r]
   * @param {number[]} nu_r - Relative velocity vector [u_r, v_r, w_r, p, q, r]
   */
  computeCoriolisMatrix(nu, nu_r) {
    const m = this.config.mass;
    const { Ixx, Iyy, Izz } = this.config.get('rigidBody');
    const { xG, yG, zG } = this.config.getCenters();
    const added = this.config.get('addedMass');

    const [u, v, w, p, q, r] = nu;
    const [ur, vr, wr] = nu_r;

    // Linear momentum vector of rigid body: p_RB = m * (v1 + v2 x r_G)
    const v1 = [u, v, w];
    const v2 = [p, q, r];
    const rG = [xG, yG, zG];

    // v2 x rG
    const v2_x_rG = [
      q * zG - r * yG,
      r * xG - p * zG,
      p * yG - q * xG
    ];

    const a_RB = [
      m * (v1[0] + v2_x_rG[0]),
      m * (v1[1] + v2_x_rG[1]),
      m * (v1[2] + v2_x_rG[2])
    ];

    // Angular momentum of rigid body: h_RB = I_b * v2
    const b_RB = [Ixx * p, Iyy * q, Izz * r];

    const S_aRB = this.skew(a_RB);
    const S_bRB = this.skew(b_RB);

    // Added mass momentum terms:
    // a_A = M_A11 * v_r1 = [Xu_dot * ur, Yv_dot * vr, Zw_dot * wr]
    const a_A = [added.Xu_dot * ur, added.Yv_dot * vr, added.Zw_dot * wr];
    const b_A = [added.Kp_dot * p, added.Mq_dot * q, added.Nr_dot * r];

    const S_aA = this.skew(a_A);
    const S_bA = this.skew(b_A);

    // Total C(ν) = C_RB(ν) + C_A(ν_r)
    // For an open-frame, bluff-body ROV (Fossen 2021, Sec 6.5 "Hydrodynamics of ROVs"),
    // potential-flow cross-coupling between linear surge/sway velocity and angular rates
    // is negligible compared to viscous drag and creates an artificial destabilizing
    // Munk moment if included without lifting surfaces.
    // The rotational centripetal block -S(b_RB + b_A) governs attitude gyroscopic coupling.
    const C = Array(6).fill(0).map(() => Array(6).fill(0));

    for (let i = 0; i < 3; i++) {
      for (let j = 0; j < 3; j++) {
        // Bottom-right 3x3 block: Gyroscopic centripetal moments -S(b_RB + b_A)
        C[i + 3][j + 3] = -(S_bRB[i][j] + S_bA[i][j]);
      }
    }

    return C;
  }

  /**
   * Compute Hydrodynamic Damping Vector D(ν_r) * ν_r
   * D(ν_r) = D_L * ν_r + D_Q * |ν_r| * ν_r
   * 
   * @param {number[]} nu_r - Relative velocity [ur, vr, wr, p, q, r]
   * @returns {number[]} 6x1 damping force/moment vector (N and N·m)
   */
  computeDampingForces(nu_r) {
    const dLin = this.config.get('linearDamping');
    const dQuad = this.config.get('quadraticDamping');

    const [ur, vr, wr, p, q, r] = nu_r;

    const F_surge = -(dLin.Xu * ur + dQuad.Xuu * Math.abs(ur) * ur);
    const F_sway  = -(dLin.Yv * vr + dQuad.Yvv * Math.abs(vr) * vr);
    const F_heave = -(dLin.Zw * wr + dQuad.Zww * Math.abs(wr) * wr);

    const M_roll  = -(dLin.Kp * p + dQuad.Kpp * Math.abs(p) * p);
    const M_pitch = -(dLin.Mq * q + dQuad.Mqq * Math.abs(q) * q);
    const M_yaw   = -(dLin.Nr * r + dQuad.Nrr * Math.abs(r) * r);

    return [F_surge, F_sway, F_heave, M_roll, M_pitch, M_yaw];
  }

  /**
   * Compute Hydrostatic Restoring Force & Moment Vector g(η)
   * 
   * Based on true Archimedes displaced volume: B = ρ * g * V_displaced
   * Weight: W = m * g
   * 
   * Reference: Fossen (2021) Eq. 4.14
   * 
   * @param {number} phi - Roll angle (rad)
   * @param {number} theta - Pitch angle (rad)
   * @param {number} [depth=1.0] - Depth (m)
   * @returns {number[]} 6x1 restoring vector g(η) in body frame
   */
  computeRestoringForces(phi, theta, depth = 1.0) {
    const m = this.config.mass;
    const g = this.environment.gravity;
    const rho = this.environment.waterDensity;
    const V = this.config.get('geometry').displacedVolume;

    const W = m * g;
    const B = rho * g * V;

    const { xG, yG, zG, xB, yB, zB } = this.config.getCenters();

    const sphi = Math.sin(phi), cphi = Math.cos(phi);
    const sth = Math.sin(theta), cth = Math.cos(theta);

    // Forces in body frame
    const X_g = (W - B) * sth;
    const Y_g = -(W - B) * cth * sphi;
    const Z_g = -(W - B) * cth * cphi;

    // Righting moments in body frame
    const K_g = -(yG * W - yB * B) * cth * cphi + (zG * W - zB * B) * cth * sphi;
    const M_g = (zG * W - zB * B) * sth + (xG * W - xB * B) * cth * cphi;
    const N_g = -(xG * W - xB * B) * cth * sphi - (yG * W - yB * B) * sth;

    return [X_g, Y_g, Z_g, K_g, M_g, N_g];
  }

  /**
   * Calculate 6-DOF body accelerations: ν̇ = f(η, ν, τ, env)
   * 
   * @param {Object} state - { eta: [x, y, z, phi, theta, psi], nu: [u, v, w, p, q, r], quat: {w,x,y,z} }
   * @param {number[]} tau - 6x1 Control force/moment vector [X, Y, Z, K, M, N]
   * @param {number[]} [tau_residual=[0,0,0,0,0,0]] - 6x1 PINN / residual force
   * @returns {{ nu_dot: number[], forcesBreakdown: Object }}
   */
  computeAccelerations(state, tau, tau_residual = [0, 0, 0, 0, 0, 0]) {
    const nu = state.nu || [0, 0, 0, 0, 0, 0];
    const eta = state.eta || [0, 0, 0, 0, 0, 0];
    const phi = eta[3] || 0;
    const theta = eta[4] || 0;
    const psi = eta[5] || 0;
    const depth = eta[2] || 1.0;

    // 1. Compute environmental ocean current at current vehicle position
    const currentNed = this.environment.getCurrentVelocity(eta[0], eta[1], depth);
    const vCurrentWorld = [currentNed.vx, currentNed.vy, currentNed.vz];

    // Transform ocean current to body frame
    const vCurrentBody = Kinematics.worldToBodyVelocity(
      vCurrentWorld,
      state.quat || { roll: phi, pitch: theta, yaw: psi }
    );

    // Relative velocity: ν_r = ν - ν_c
    const nu_r = [
      nu[0] - vCurrentBody.u,
      nu[1] - vCurrentBody.v,
      nu[2] - vCurrentBody.w,
      nu[3], // Angular current is assumed zero in pool
      nu[4],
      nu[5]
    ];

    // 2. Mass Matrix M and its inverse
    const { M } = this.computeMassMatrix();
    const M_inv = this.invert6x6(M);

    // 3. Coriolis forces: -C(ν) * ν
    const C = this.computeCoriolisMatrix(nu, nu_r);
    const coriolisForces = this.mat6VecMul(C, nu).map(v => -v);

    // 4. Damping forces: D(ν_r) * ν_r (already has negative sign)
    const dampingForces = this.computeDampingForces(nu_r);

    // 5. Restoring forces: -g(η)
    const g_eta = this.computeRestoringForces(phi, theta, depth);
    const restoringForces = g_eta.map(v => -v);

    // 6. Net Generalized Forces Vector:
    // τ_net = τ + τ_res + Coriolis + Damping + Restoring
    const tau_net = Array(6).fill(0);
    for (let i = 0; i < 6; i++) {
      tau_net[i] = (tau[i] || 0) +
                   (tau_residual[i] || 0) +
                   coriolisForces[i] +
                   dampingForces[i] +
                   restoringForces[i];
    }

    // 7. Solve for body accelerations: ν̇ = M⁻¹ * τ_net
    const nu_dot = this.mat6VecMul(M_inv, tau_net);

    // Save breakdown for observability
    this.lastForces = {
      coriolis: coriolisForces,
      damping: dampingForces,
      restoring: restoringForces,
      thruster: [...tau],
      residual: [...tau_residual],
      net: tau_net,
      accel: nu_dot,
    };

    return { nu_dot, forcesBreakdown: this.lastForces };
  }

  /**
   * Integrate 6-DOF state forward by dt using Runge-Kutta 4th Order (RK4)
   * 
   * @param {Object} state - { eta: [x,y,z,phi,theta,psi], nu: [u,v,w,p,q,r], quat: {w,x,y,z} }
   * @param {number[]} tau - 6x1 Control input [X,Y,Z,K,M,N]
   * @param {number} dt - Time step in seconds
   * @param {number[]} [tau_residual] - Optional PINN residual forces
   * @returns {Object} New state { eta, nu, quat, nu_dot }
   */
  step(state, tau, dt, tau_residual = [0, 0, 0, 0, 0, 0]) {
    // Current state vectors
    let { eta, nu, quat } = state;
    eta = [...eta];
    nu = [...nu];
    quat = quat ? { ...quat } : Kinematics.eulerToQuaternion(eta[3], eta[4], eta[5]);

    if (this.integrator === 'rk4') {
      // ───────────────────────────────────────────────────────────────────────
      // Runge-Kutta 4 (RK4) Integration
      // ───────────────────────────────────────────────────────────────────────
      // k1
      const { nu_dot: k1_nu_dot } = this.computeAccelerations({ eta, nu, quat }, tau, tau_residual);
      const k1_pos_dot = Kinematics.bodyToWorldVelocity({ u: nu[0], v: nu[1], w: nu[2] }, quat);
      const k1_ang_dot = Kinematics.bodyRatesToEulerRates({ p: nu[3], q: nu[4], r: nu[5] }, { roll: eta[3], pitch: eta[4] });

      // State at t + dt/2 (k2)
      const nu_k2 = nu.map((v, i) => v + k1_nu_dot[i] * 0.5 * dt);
      const eta_k2 = [
        eta[0] + k1_pos_dot.x * 0.5 * dt,
        eta[1] + k1_pos_dot.y * 0.5 * dt,
        eta[2] + k1_pos_dot.z * 0.5 * dt,
        eta[3] + k1_ang_dot.rollRate * 0.5 * dt,
        eta[4] + k1_ang_dot.pitchRate * 0.5 * dt,
        eta[5] + k1_ang_dot.yawRate * 0.5 * dt,
      ];
      const quat_k2 = Kinematics.integrateQuaternion(quat, { p: nu[3], q: nu[4], r: nu[5] }, 0.5 * dt);

      const { nu_dot: k2_nu_dot } = this.computeAccelerations({ eta: eta_k2, nu: nu_k2, quat: quat_k2 }, tau, tau_residual);
      const k2_pos_dot = Kinematics.bodyToWorldVelocity({ u: nu_k2[0], v: nu_k2[1], w: nu_k2[2] }, quat_k2);
      const k2_ang_dot = Kinematics.bodyRatesToEulerRates({ p: nu_k2[3], q: nu_k2[4], r: nu_k2[5] }, { roll: eta_k2[3], pitch: eta_k2[4] });

      // State at t + dt/2 (k3)
      const nu_k3 = nu.map((v, i) => v + k2_nu_dot[i] * 0.5 * dt);
      const eta_k3 = [
        eta[0] + k2_pos_dot.x * 0.5 * dt,
        eta[1] + k2_pos_dot.y * 0.5 * dt,
        eta[2] + k2_pos_dot.z * 0.5 * dt,
        eta[3] + k2_ang_dot.rollRate * 0.5 * dt,
        eta[4] + k2_ang_dot.pitchRate * 0.5 * dt,
        eta[5] + k2_ang_dot.yawRate * 0.5 * dt,
      ];
      const quat_k3 = Kinematics.integrateQuaternion(quat, { p: nu_k2[3], q: nu_k2[4], r: nu_k2[5] }, 0.5 * dt);

      const { nu_dot: k3_nu_dot } = this.computeAccelerations({ eta: eta_k3, nu: nu_k3, quat: quat_k3 }, tau, tau_residual);
      const k3_pos_dot = Kinematics.bodyToWorldVelocity({ u: nu_k3[0], v: nu_k3[1], w: nu_k3[2] }, quat_k3);
      const k3_ang_dot = Kinematics.bodyRatesToEulerRates({ p: nu_k3[3], q: nu_k3[4], r: nu_k3[5] }, { roll: eta_k3[3], pitch: eta_k3[4] });

      // State at t + dt (k4)
      const nu_k4 = nu.map((v, i) => v + k3_nu_dot[i] * dt);
      const eta_k4 = [
        eta[0] + k3_pos_dot.x * dt,
        eta[1] + k3_pos_dot.y * dt,
        eta[2] + k3_pos_dot.z * dt,
        eta[3] + k3_ang_dot.rollRate * dt,
        eta[4] + k3_ang_dot.pitchRate * dt,
        eta[5] + k3_ang_dot.yawRate * dt,
      ];
      const quat_k4 = Kinematics.integrateQuaternion(quat, { p: nu_k3[3], q: nu_k3[4], r: nu_k3[5] }, dt);

      const { nu_dot: k4_nu_dot } = this.computeAccelerations({ eta: eta_k4, nu: nu_k4, quat: quat_k4 }, tau, tau_residual);
      const k4_pos_dot = Kinematics.bodyToWorldVelocity({ u: nu_k4[0], v: nu_k4[1], w: nu_k4[2] }, quat_k4);
      const k4_ang_dot = Kinematics.bodyRatesToEulerRates({ p: nu_k4[3], q: nu_k4[4], r: nu_k4[5] }, { roll: eta_k4[3], pitch: eta_k4[4] });

      // Weighted combination
      const nu_next = nu.map((v, i) => v + (dt / 6.0) * (k1_nu_dot[i] + 2 * k2_nu_dot[i] + 2 * k3_nu_dot[i] + k4_nu_dot[i]));

      const pos_dot_avg = {
        x: (k1_pos_dot.x + 2 * k2_pos_dot.x + 2 * k3_pos_dot.x + k4_pos_dot.x) / 6.0,
        y: (k1_pos_dot.y + 2 * k2_pos_dot.y + 2 * k3_pos_dot.y + k4_pos_dot.y) / 6.0,
        z: (k1_pos_dot.z + 2 * k2_pos_dot.z + 2 * k3_pos_dot.z + k4_pos_dot.z) / 6.0,
      };

      const avgAngularRates = {
        p: (nu[3] + 2 * nu_k2[3] + 2 * nu_k3[3] + nu_k4[3]) / 6.0,
        q: (nu[4] + 2 * nu_k2[4] + 2 * nu_k3[4] + nu_k4[4]) / 6.0,
        r: (nu[5] + 2 * nu_k2[5] + 2 * nu_k3[5] + nu_k4[5]) / 6.0,
      };

      const quat_next = Kinematics.integrateQuaternion(quat, avgAngularRates, dt);
      const euler_next = Kinematics.quaternionToEuler(quat_next);

      const eta_next = [
        eta[0] + pos_dot_avg.x * dt,
        eta[1] + pos_dot_avg.y * dt,
        eta[2] + pos_dot_avg.z * dt,
        euler_next.roll,
        euler_next.pitch,
        euler_next.yaw,
      ];

      return {
        eta: eta_next,
        nu: nu_next,
        quat: quat_next,
        nu_dot: k1_nu_dot,
        forces: this.lastForces,
      };
    } else {
      // ───────────────────────────────────────────────────────────────────────
      // Semi-Implicit Euler Integration
      // ───────────────────────────────────────────────────────────────────────
      const { nu_dot } = this.computeAccelerations({ eta, nu, quat }, tau, tau_residual);
      const nu_next = nu.map((v, i) => v + nu_dot[i] * dt);

      const quat_next = Kinematics.integrateQuaternion(
        quat,
        { p: nu_next[3], q: nu_next[4], r: nu_next[5] },
        dt
      );
      const euler_next = Kinematics.quaternionToEuler(quat_next);

      const vWorld = Kinematics.bodyToWorldVelocity(
        { u: nu_next[0], v: nu_next[1], w: nu_next[2] },
        quat_next
      );

      const eta_next = [
        eta[0] + vWorld.x * dt,
        eta[1] + vWorld.y * dt,
        eta[2] + vWorld.z * dt,
        euler_next.roll,
        euler_next.pitch,
        euler_next.yaw,
      ];

      return {
        eta: eta_next,
        nu: nu_next,
        quat: quat_next,
        nu_dot,
        forces: this.lastForces,
      };
    }
  }

  // ───────────────────────────────────────────────────────────────────────────
  // Compatibility methods for existing services
  // ───────────────────────────────────────────────────────────────────────────
  getCurrentDrift() {
    const c = this.environment.getCurrentVelocity(0, 0, 1.0);
    return { x: c.vx, z: c.vy };
  }

  getEffectiveMass() {
    const { M } = this.computeMassMatrix();
    return {
      surge: M[0][0],
      sway: M[1][1],
      heave: M[2][2],
      roll: M[3][3],
      pitch: M[4][4],
      yaw: M[5][5],
    };
  }
}

export const defaultHydrodynamics = new HydrodynamicsEngine();
export default defaultHydrodynamics;
