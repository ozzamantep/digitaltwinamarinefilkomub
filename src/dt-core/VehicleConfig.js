/**
 * Centralized Vehicle Configuration for Digital Twin
 * 
 * All physical parameters that were previously hard-coded across
 * HydrodynamicsEngine, ThrusterDynamicsModel, AUVMotionController, etc.
 * are now extracted here with units, physical bounds, and descriptions.
 * 
 * This is the SINGLE SOURCE OF TRUTH for vehicle parameters.
 * Every parameter must have:
 *   - value
 *   - unit
 *   - description
 *   - physical bounds [min, max]
 *   - source (datasheet, URDF, measured, estimated, assumed)
 * 
 * Equation reference:
 *   M ν̇ + C(ν)ν + D(ν)ν + g(η) = τ + τ_env
 *   where M = M_RB + M_A
 * 
 * Coordinate convention: NED (North-East-Down) body frame internally.
 *   η = [x, y, z, φ, θ, ψ]ᵀ  (position in world NED + Euler angles)
 *   ν = [u, v, w, p, q, r]ᵀ  (body-frame linear + angular velocities)
 *   τ = [X, Y, Z, K, M, N]ᵀ  (body-frame forces + moments)
 * 
 * Units:
 *   position → m, velocity → m/s, angular velocity → rad/s
 *   angle → rad (internally), mass → kg, force → N, moment → N·m
 *   pressure → Pa, temperature → °C, density → kg/m³
 */

// ─────────────────────────────────────────────────────────────────────────────
// Fidelity Levels
// ─────────────────────────────────────────────────────────────────────────────
export const FIDELITY = {
  SIMPLIFIED: 1,  // Fast analytical: diagonal M, linear damping only, no cross-terms
  MEDIUM: 2,      // Physics + calibrated hydro: diagonal M, linear+quadratic damping, Coriolis
  HIGH: 3,        // Full 6×6 matrices, cross-coupling, learned residual
};

// ─────────────────────────────────────────────────────────────────────────────
// Parameter definition helper
// ─────────────────────────────────────────────────────────────────────────────
function param(value, unit, description, bounds, source) {
  return { value, unit, description, bounds, source };
}

// ─────────────────────────────────────────────────────────────────────────────
// Default Vehicle Configuration: Custom 6-Thruster BlueROV2-class AUV
// ─────────────────────────────────────────────────────────────────────────────
const defaultConfig = {

  // ═══════════════════════════════════════════════════════════════════════════
  // META
  // ═══════════════════════════════════════════════════════════════════════════
  meta: {
    name: 'Amarine FILKOM UB AUV',
    type: 'BlueROV2-class 6-thruster',
    version: '1.0.0',
    parameterVersion: '1.0.0',
    lastCalibrated: null,
    fidelityLevel: FIDELITY.MEDIUM,
  },

  // ═══════════════════════════════════════════════════════════════════════════
  // RIGID BODY PROPERTIES
  // Source: URDF (auv_model.urdf) and physical measurement
  // ═══════════════════════════════════════════════════════════════════════════
  rigidBody: {
    mass:   param(22.5, 'kg', 'Total dry mass', [20.0, 25.0], 'measured'),
    // Moments of inertia about CG (from URDF)
    Ixx:    param(0.12, 'kg·m²', 'Roll moment of inertia',  [0.05, 0.50], 'URDF'),
    Iyy:    param(0.22, 'kg·m²', 'Pitch moment of inertia', [0.10, 0.60], 'URDF'),
    Izz:    param(0.24, 'kg·m²', 'Yaw moment of inertia',   [0.10, 0.60], 'URDF'),
    Ixy:    param(0.0,  'kg·m²', 'Product of inertia XY',   [-0.05, 0.05], 'URDF'),
    Ixz:    param(0.0,  'kg·m²', 'Product of inertia XZ',   [-0.05, 0.05], 'URDF'),
    Iyz:    param(0.0,  'kg·m²', 'Product of inertia YZ',   [-0.05, 0.05], 'URDF'),
  },

  // ═══════════════════════════════════════════════════════════════════════════
  // GEOMETRY
  // Source: BlueROV2Model.jsx + URDF measurements
  // ═══════════════════════════════════════════════════════════════════════════
  geometry: {
    length: param(0.54, 'm', 'Overall length (x-axis)',  [0.3, 1.0], 'measured'),
    width:  param(0.28, 'm', 'Overall width (y-axis)',   [0.2, 0.6], 'measured'),
    height: param(0.24, 'm', 'Overall height (z-axis)',  [0.1, 0.5], 'measured'),
    // Displaced volume trimmed for a 22.5 kg vehicle in a 26°C freshwater pool.
    displacedVolume: param(0.02254345, 'm³', 'Displaced water volume (neutrally trimmed with ballast)',
      [0.020, 0.030], 'measured'),
    // fill_factor ≈ 0.34 for frame-type ROV
  },

  // ═══════════════════════════════════════════════════════════════════════════
  // CENTER OF GRAVITY & CENTER OF BUOYANCY
  // In body frame (NED convention: x-forward, y-starboard, z-down)
  // ═══════════════════════════════════════════════════════════════════════════
  centers: {
    // CG position relative to geometric center
    xG: param(0.0,    'm', 'CG x-offset (positive = forward)',  [-0.05, 0.05], 'assumed'),
    yG: param(0.0,    'm', 'CG y-offset (positive = starboard)',[-0.05, 0.05], 'assumed'),
    zG: param(0.0,    'm', 'CG z-offset (positive = down)',     [-0.05, 0.05], 'assumed'),
    // CB position relative to geometric center
    // In NED: CB above CG means zB < zG (more negative = higher)
    xB: param(0.0,    'm', 'CB x-offset', [-0.05, 0.05], 'estimated'),
    yB: param(0.0,    'm', 'CB y-offset', [-0.05, 0.05], 'estimated'),
    zB: param(-0.025, 'm', 'CB z-offset (negative = above CG in NED)',
      [-0.10, 0.0], 'estimated'),
  },

  // ═══════════════════════════════════════════════════════════════════════════
  // ADDED MASS COEFFICIENTS (M_A)
  // F_added = -M_A ν̇
  // Diagonal approximation for rectangular-frame ROV
  // Source: Berg (2012), Wu (2018) empirical BlueROV2 studies
  // ═══════════════════════════════════════════════════════════════════════════
  addedMass: {
    Xu_dot: param(5.5,  'kg',    'Added mass in surge',   [2.0, 15.0],  'empirical'),
    Yv_dot: param(12.7, 'kg',    'Added mass in sway',    [5.0, 25.0],  'empirical'),
    Zw_dot: param(14.6, 'kg',    'Added mass in heave',   [5.0, 30.0],  'empirical'),
    Kp_dot: param(0.12, 'kg·m²', 'Added inertia in roll', [0.01, 0.50], 'empirical'),
    Mq_dot: param(0.12, 'kg·m²', 'Added inertia in pitch',[0.01, 0.50], 'empirical'),
    Nr_dot: param(0.12, 'kg·m²', 'Added inertia in yaw',  [0.01, 0.50], 'empirical'),
  },

  // ═══════════════════════════════════════════════════════════════════════════
  // LINEAR DAMPING COEFFICIENTS (D_L)
  // F_linear = -D_L ν
  // ═══════════════════════════════════════════════════════════════════════════
  linearDamping: {
    Xu: param(4.03,  'N·s/m',     'Linear damping surge',   [1.0, 15.0],  'empirical'),
    Yv: param(6.22,  'N·s/m',     'Linear damping sway',    [2.0, 20.0],  'empirical'),
    Zw: param(5.18,  'N·s/m',     'Linear damping heave',   [2.0, 20.0],  'empirical'),
    Kp: param(0.07,  'N·m·s/rad', 'Linear damping roll',    [0.01, 1.0],  'empirical'),
    Mq: param(0.07,  'N·m·s/rad', 'Linear damping pitch',   [0.01, 1.0],  'empirical'),
    Nr: param(0.07,  'N·m·s/rad', 'Linear damping yaw',     [0.01, 1.0],  'empirical'),
  },

  // ═══════════════════════════════════════════════════════════════════════════
  // QUADRATIC DAMPING COEFFICIENTS (D_Q)
  // F_quad = -D_Q |ν| ν
  // ═══════════════════════════════════════════════════════════════════════════
  quadraticDamping: {
    Xuu: param(18.18, 'N·s²/m²',     'Quadratic damping surge',   [5.0, 50.0],  'empirical'),
    Yvv: param(21.66, 'N·s²/m²',     'Quadratic damping sway',    [5.0, 60.0],  'empirical'),
    Zww: param(36.99, 'N·s²/m²',     'Quadratic damping heave',   [10.0, 80.0], 'empirical'),
    Kpp: param(1.55,  'N·m·s²/rad²', 'Quadratic damping roll',    [0.1, 5.0],   'empirical'),
    Mqq: param(1.55,  'N·m·s²/rad²', 'Quadratic damping pitch',   [0.1, 5.0],   'empirical'),
    Nrr: param(1.55,  'N·m·s²/rad²', 'Quadratic damping yaw',     [0.1, 5.0],   'empirical'),
  },

  // ═══════════════════════════════════════════════════════════════════════════
  // THRUSTER CONFIGURATION
  // 6 thrusters: T1-T4 horizontal at 45° corners, T5-T6 vertical
  // Positions in body frame (NED: x-fwd, y-stbd, z-down)
  // Source: URDF joint origins
  // ═══════════════════════════════════════════════════════════════════════════
  thrusters: {
    count: 6,
    // Position of each thruster in body frame [x, y, z] (m)
    positions: [
      [0.14,  0.11, 0.0],   // T1: Front-Left  (port)
      [0.14, -0.11, 0.0],   // T2: Front-Right  (starboard)
      [-0.14, 0.11, 0.0],   // T3: Rear-Left   (port)
      [-0.14,-0.11, 0.0],   // T4: Rear-Right  (starboard)
      [0.18,  0.0,  0.0],   // T5: Front-Vertical (in front cowling)
      [-0.18, 0.0,  0.0],   // T6: Rear-Vertical  (in rear cowling)
    ],
    // Thrust direction unit vectors in body frame
    // Horizontal thrusters at 45° → thrust along [cos45, ±sin45, 0]
    // Vertical thrusters along [0, 0, -1] (thrust pushes water down → vehicle up in NED)
    directions: [
      [Math.cos(Math.PI / 4), -Math.sin(Math.PI / 4), 0],  // T1: 45° port-fwd
      [Math.cos(Math.PI / 4),  Math.sin(Math.PI / 4), 0],  // T2: -45° stbd-fwd
      [Math.cos(Math.PI / 4),  Math.sin(Math.PI / 4), 0],  // T3: 135° → same dir as T2 for fwd+yaw
      [Math.cos(Math.PI / 4), -Math.sin(Math.PI / 4), 0],  // T4: -135° → same dir as T1 for fwd+yaw
      [0, 0, -1],  // T5: vertical (thrust down in NED = upward force)
      [0, 0, -1],  // T6: vertical
    ],
    // Motor dynamics
    timeConstant: param(0.35, 's', 'First-order motor lag time constant',
      [0.10, 1.0], 'datasheet'),
    maxThrust_fwd: param(50.0, 'N', 'Max forward thrust per thruster at 16V',
      [30.0, 80.0], 'datasheet'),  // 5.1 kgf × 9.81
    maxThrust_rev: param(40.2, 'N', 'Max reverse thrust per thruster at 16V',
      [25.0, 60.0], 'datasheet'),  // 4.1 kgf × 9.81
    deadband: param([1470, 1530], 'μs', 'PWM deadband range', null, 'datasheet'),
    nominalVoltage: param(16.0, 'V', 'Nominal operating voltage (4S LiPo)',
      [12.0, 18.0], 'spec'),
  },

  // ═══════════════════════════════════════════════════════════════════════════
  // OPERATIONAL LIMITS
  // ═══════════════════════════════════════════════════════════════════════════
  limits: {
    maxVelocity: {
      u: param(2.0, 'm/s', 'Max surge velocity (simulation turbo limit)', [0.5, 3.0], 'simulation'),
      v: param(1.2, 'm/s', 'Max sway velocity',  [0.3, 2.0], 'spec'),
      w: param(0.8, 'm/s', 'Max heave velocity', [0.3, 1.5], 'spec'),
      p: param(1.5, 'rad/s', 'Max roll rate',    [0.5, 3.0], 'spec'),
      q: param(1.5, 'rad/s', 'Max pitch rate',   [0.5, 3.0], 'spec'),
      r: param(2.0, 'rad/s', 'Max yaw rate',     [0.5, 4.0], 'spec'),
    },
    maxDepth: param(2.0, 'm', 'Maximum operating depth', [1.0, 100.0], 'arena'),
    minDepth: param(0.08, 'm', 'Minimum depth (surface)', [0.0, 0.5], 'physical'),
  },
};

// ─────────────────────────────────────────────────────────────────────────────
// VehicleConfig class — provides typed access + parameter update + validation
// ─────────────────────────────────────────────────────────────────────────────
export class VehicleConfig {
  constructor(config = null) {
    this._config = JSON.parse(JSON.stringify(config || defaultConfig));
    this._version = 1;
  }

  get(group) {
    if (this._config[group]) {
      const res = {};
      for (const [k, v] of Object.entries(this._config[group])) {
        res[k] = v && typeof v === 'object' && 'value' in v ? v.value : v;
      }
      return res;
    }
    return {};
  }

  getCenters() {
    return {
      xG: this.xG,
      yG: this.yG,
      zG: this.zG,
      xB: this.xB,
      yB: this.yB,
      zB: this.zB,
    };
  }

  get mass()   { return this._config.rigidBody.mass.value; }
  get Ixx()    { return this._config.rigidBody.Ixx.value; }
  get Iyy()    { return this._config.rigidBody.Iyy.value; }
  get Izz()    { return this._config.rigidBody.Izz.value; }
  get Ixy()    { return this._config.rigidBody.Ixy.value; }
  get Ixz()    { return this._config.rigidBody.Ixz.value; }
  get Iyz()    { return this._config.rigidBody.Iyz.value; }

  get length() { return this._config.geometry.length.value; }
  get width()  { return this._config.geometry.width.value; }
  get height() { return this._config.geometry.height.value; }
  get displacedVolume() { return this._config.geometry.displacedVolume.value; }

  get xG() { return this._config.centers.xG.value; }
  get yG() { return this._config.centers.yG.value; }
  get zG() { return this._config.centers.zG.value; }
  get xB() { return this._config.centers.xB.value; }
  get yB() { return this._config.centers.yB.value; }
  get zB() { return this._config.centers.zB.value; }

  get fidelityLevel() { return this._config.meta.fidelityLevel; }
  set fidelityLevel(level) { this._config.meta.fidelityLevel = level; }

  // ─── Added mass accessors ──────────────────────────────────────────────

  get Xu_dot() { return this._config.addedMass.Xu_dot.value; }
  get Yv_dot() { return this._config.addedMass.Yv_dot.value; }
  get Zw_dot() { return this._config.addedMass.Zw_dot.value; }
  get Kp_dot() { return this._config.addedMass.Kp_dot.value; }
  get Mq_dot() { return this._config.addedMass.Mq_dot.value; }
  get Nr_dot() { return this._config.addedMass.Nr_dot.value; }

  // ─── Damping accessors ─────────────────────────────────────────────────

  get Xu()  { return this._config.linearDamping.Xu.value; }
  get Yv()  { return this._config.linearDamping.Yv.value; }
  get Zw()  { return this._config.linearDamping.Zw.value; }
  get Kp()  { return this._config.linearDamping.Kp.value; }
  get Mq()  { return this._config.linearDamping.Mq.value; }
  get Nr()  { return this._config.linearDamping.Nr.value; }

  get Xuu() { return this._config.quadraticDamping.Xuu.value; }
  get Yvv() { return this._config.quadraticDamping.Yvv.value; }
  get Zww() { return this._config.quadraticDamping.Zww.value; }
  get Kpp() { return this._config.quadraticDamping.Kpp.value; }
  get Mqq() { return this._config.quadraticDamping.Mqq.value; }
  get Nrr() { return this._config.quadraticDamping.Nrr.value; }

  // ─── Thruster accessors ────────────────────────────────────────────────

  get thrusterCount()     { return this._config.thrusters.count; }
  get thrusterPositions() { return this._config.thrusters.positions; }
  get thrusterDirections(){ return this._config.thrusters.directions; }
  get thrusterTimeConst() { return this._config.thrusters.timeConstant.value; }
  get maxThrust_fwd()     { return this._config.thrusters.maxThrust_fwd.value; }
  get maxThrust_rev()     { return this._config.thrusters.maxThrust_rev.value; }
  get nominalVoltage()    { return this._config.thrusters.nominalVoltage.value; }

  // ─── Velocity limits ───────────────────────────────────────────────────

  getMaxVelocity(dof) {
    const limits = this._config.limits.maxVelocity;
    return limits[dof] ? limits[dof].value : 2.0;
  }

  // ─── Effective mass (rigid body + added mass) per DOF ──────────────────

  /** M_RB + M_A for surge */
  get effectiveMassSurge() { return this.mass + this.Xu_dot; }
  /** M_RB + M_A for sway */
  get effectiveMassSway()  { return this.mass + this.Yv_dot; }
  /** M_RB + M_A for heave */
  get effectiveMassHeave() { return this.mass + this.Zw_dot; }
  /** I_xx + K_p_dot for roll */
  get effectiveInertiaRoll()  { return this.Ixx + this.Kp_dot; }
  /** I_yy + M_q_dot for pitch */
  get effectiveInertiaPitch() { return this.Iyy + this.Mq_dot; }
  /** I_zz + N_r_dot for yaw */
  get effectiveInertiaYaw()   { return this.Izz + this.Nr_dot; }

  /**
   * Get the 6-element diagonal of the effective mass matrix M = M_RB + M_A
   * Order: [surge, sway, heave, roll, pitch, yaw]
   * @returns {number[]}
   */
  getEffectiveMassDiagonal() {
    return [
      this.effectiveMassSurge,
      this.effectiveMassSway,
      this.effectiveMassHeave,
      this.effectiveInertiaRoll,
      this.effectiveInertiaPitch,
      this.effectiveInertiaYaw,
    ];
  }

  // ─── Thruster allocation matrix (6 thrusters → 6 DOF) ─────────────────

  /**
   * Compute the 6×6 thruster allocation matrix B
   * τ = B · f, where f = [f1, f2, f3, f4, f5, f6]ᵀ (thrust forces in N)
   * τ = [X, Y, Z, K, M, N]ᵀ (body-frame forces/moments)
   * 
   * Each column j: [fx_j, fy_j, fz_j, (r_j × f_j)]
   * @returns {number[][]} 6×6 matrix
   */
  computeAllocationMatrix() {
    const B = Array.from({ length: 6 }, () => new Array(6).fill(0));
    const pos = this.thrusterPositions;
    const dir = this.thrusterDirections;

    for (let j = 0; j < 6; j++) {
      const [dx, dy, dz] = dir[j];
      const [px, py, pz] = pos[j];

      // Force contribution
      B[0][j] = dx;  // X (surge)
      B[1][j] = dy;  // Y (sway)
      B[2][j] = dz;  // Z (heave)

      // Moment contribution: τ = r × F
      B[3][j] = py * dz - pz * dy;  // K (roll moment)
      B[4][j] = pz * dx - px * dz;  // M (pitch moment)
      B[5][j] = px * dy - py * dx;  // N (yaw moment)
    }

    return B;
  }

  // ─── Parameter update with bounds checking ─────────────────────────────

  /**
   * Update a parameter value with physical bounds enforcement
   * @param {string} path - Dot-separated path, e.g. 'addedMass.Xu_dot'
   * @param {number} newValue
   * @returns {boolean} true if value was within bounds and accepted
   */
  updateParameter(path, newValue) {
    const parts = path.split('.');
    let obj = this._config;
    for (let i = 0; i < parts.length; i++) {
      if (obj === undefined) return false;
      if (i === parts.length - 1) {
        if (obj[parts[i]] && obj[parts[i]].bounds) {
          const [min, max] = obj[parts[i]].bounds;
          if (newValue < min || newValue > max) {
            console.warn(
              `[VehicleConfig] Parameter ${path} = ${newValue} out of bounds [${min}, ${max}]. Clamping.`
            );
            newValue = Math.max(min, Math.min(max, newValue));
          }
        }
        if (obj[parts[i]] && typeof obj[parts[i]] === 'object' && 'value' in obj[parts[i]]) {
          obj[parts[i]].value = newValue;
        } else {
          obj[parts[i]] = newValue;
        }
        this._version++;
        return true;
      }
      obj = obj[parts[i]];
    }
    return false;
  }

  /**
   * Bulk update parameters from an optimization result
   * @param {Object} params - { 'addedMass.Xu_dot': 5.8, ... }
   */
  updateParameters(params) {
    for (const [path, value] of Object.entries(params)) {
      this.updateParameter(path, value);
    }
  }

  /**
   * Export the full parameter vector θ for identification
   * @returns {{ names: string[], values: number[], bounds: number[][] }}
   */
  getIdentifiableParameters() {
    const names = [];
    const values = [];
    const bounds = [];

    const sections = ['addedMass', 'linearDamping', 'quadraticDamping'];
    for (const section of sections) {
      const group = this._config[section];
      for (const [key, p] of Object.entries(group)) {
        names.push(`${section}.${key}`);
        values.push(p.value);
        bounds.push(p.bounds || [0, Infinity]);
      }
    }

    // CG/CB offsets
    for (const key of ['xG', 'yG', 'zG', 'xB', 'yB', 'zB']) {
      const p = this._config.centers[key];
      names.push(`centers.${key}`);
      values.push(p.value);
      bounds.push(p.bounds || [-0.1, 0.1]);
    }

    return { names, values, bounds };
  }

  /** Get model version for tracking */
  get version() { return this._version; }

  /** Get the raw config for serialization */
  toJSON() {
    return {
      ...this._config,
      _version: this._version,
    };
  }

  /** Deep clone this config */
  clone() {
    const c = new VehicleConfig();
    c._config = JSON.parse(JSON.stringify(this._config));
    c._version = this._version;
    return c;
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// Singleton default instance
// ─────────────────────────────────────────────────────────────────────────────
const vehicleConfig = new VehicleConfig();
export default vehicleConfig;
