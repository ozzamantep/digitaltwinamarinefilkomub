/**
 * 6-DOF Underwater Hydrodynamics Engine
 * 
 * Implements Fossen's marine vehicle dynamics framework for realistic
 * subsea motion simulation matching the real-world AUV behavior.
 * 
 * Equation of Motion:
 *   (M_RB + M_A) v̇ + C(v)v + D(v)v + g(η) = τ_thrust + τ_current
 * 
 * Where:
 *   M_RB = Rigid body inertia matrix (from robot mass & geometry)
 *   M_A  = Added mass matrix (virtual water mass moving with the vehicle)
 *   C(v) = Coriolis & centripetal matrix (gyroscopic coupling)
 *   D(v) = Damping matrix (linear + quadratic drag)
 *   g(η) = Restoring forces (buoyancy + gravity)
 *   τ    = Thruster force/moment inputs
 * 
 * Coefficients derived from BlueROV2 empirical data and Fossen (2011).
 * Reference: T.I. Fossen, "Handbook of Marine Craft Hydrodynamics and Motion Control"
 */

class HydrodynamicsEngine {
  constructor() {
    // =========================================================================
    // Physical Parameters (Custom AUV matching Amarine FILKOM specs)
    // =========================================================================
    this.mass = 11.5;         // Total dry mass (kg)
    this.buoyancy = 0.3;      // Positive buoyancy surplus (kg-force, ~300g excess)
    this.gravity = 9.81;      // m/s²
    this.waterDensity = 997;  // Fresh water density (kg/m³) — pool water

    // Vehicle geometry (from BlueROV2Model.jsx dimensions)
    this.length = 0.54;       // Overall length (m)
    this.width = 0.28;        // Overall width (m)
    this.height = 0.24;       // Overall height (m)

    // Center of Buoyancy relative to Center of Gravity (in body frame)
    // CB is above CG for positive stability (self-righting)
    this.zB = 0.025;  // CB is 25mm above CG (typical for BlueROV2-class AUV)
    this.zG = 0.0;    // CG at geometric center (reference point)

    // =========================================================================
    // Added Mass Coefficients (M_A diagonal approximation)
    // Empirical values for BlueROV2-class rectangular-frame AUV
    // Source: Berg (2012), Wu (2018) empirical ROV studies
    // =========================================================================
    this.Xudot = 5.5;    // Added mass in surge (kg) — ~48% of dry mass
    this.Yvdot = 12.7;   // Added mass in sway (kg) — ~110% (large flat area)
    this.Zwdot = 14.6;   // Added mass in heave (kg) — ~127% (large top/bottom plate)
    this.Kpdot = 0.12;   // Added inertia in roll (kg·m²)
    this.Mqdot = 0.12;   // Added inertia in pitch (kg·m²)
    this.Nrdot = 0.12;   // Added inertia in yaw (kg·m²)

    // Effective mass (rigid body + added mass) per DOF
    this.effectiveMass = {
      surge: this.mass + this.Xudot,    // 17.0 kg — feels 48% heavier
      sway: this.mass + this.Yvdot,     // 24.2 kg — feels 110% heavier laterally
      heave: this.mass + this.Zwdot,    // 26.1 kg — feels 127% heavier vertically
    };

    // =========================================================================
    // Damping Coefficients (Linear + Quadratic)
    // D(v) = D_linear * v + D_quadratic * |v| * v
    // =========================================================================
    this.dampingLinear = {
      surge: 4.03,    // Xu (N·s/m) — skin friction at low speed
      sway: 6.22,     // Yv — wider cross-section
      heave: 5.18,    // Zw — large deck plates
      roll: 0.07,     // Kp (N·m·s/rad)
      pitch: 0.07,    // Mq
      yaw: 0.07,      // Nr
    };

    this.dampingQuadratic = {
      surge: 18.18,   // X|u|u (N·s²/m²) — dominant at cruising speed
      sway: 21.66,    // Y|v|v — high drag broadside
      heave: 36.99,   // Z|w|w — very high drag top/bottom
      roll: 1.55,     // K|p|p (N·m·s²/rad²)
      pitch: 1.55,    // M|q|q
      yaw: 1.55,      // N|r|r
    };

    // =========================================================================
    // Moment of Inertia (Rigid Body, from URDF)
    // =========================================================================
    this.Ixx = 0.12;  // Roll moment of inertia (kg·m²)
    this.Iyy = 0.22;  // Pitch moment of inertia (kg·m²)
    this.Izz = 0.24;  // Yaw moment of inertia (kg·m²)

    // =========================================================================
    // Underwater Current Simulation
    // Time-varying random current that the AUV must fight against
    // =========================================================================
    this.current = {
      vx: 0,           // Current velocity in world X (m/s)
      vz: 0,           // Current velocity in world Z (m/s)
      targetVx: 0,     // Slowly changing target
      targetVz: 0,
      maxSpeed: 0.12,  // Max current speed (m/s) — typical pool conditions
      changeTimer: 0,  // Timer for direction changes
      changeInterval: 8.0, // Seconds between current direction changes
    };

    // State for Perlin-like smooth current evolution
    this._currentPhaseX = Math.random() * Math.PI * 2;
    this._currentPhaseZ = Math.random() * Math.PI * 2;
  }

  /**
   * Compute total damping force/moment for a given velocity
   * F_damp = -(D_linear * v + D_quadratic * |v| * v)
   * 
   * @param {string} dof - Degree of freedom name
   * @param {number} velocity - Current velocity in that DOF
   * @returns {number} Damping force/moment (opposes motion)
   */
  computeDamping(dof, velocity) {
    const dLin = this.dampingLinear[dof] || 0;
    const dQuad = this.dampingQuadratic[dof] || 0;

    // F = -(D_l * v + D_q * |v| * v)
    return -(dLin * velocity + dQuad * Math.abs(velocity) * velocity);
  }

  /**
   * Compute Coriolis & centripetal coupling forces
   * These create cross-DOF forces when rotating:
   *   - Yaw rotation creates lateral sway force (centrifugal)
   *   - Surge motion during yaw creates "skid" force
   * 
   * @param {object} vel - { surge, sway, heave, yawRate }
   * @returns {object} Cross-coupling forces { surgeCoriolis, swayCoriolis, heaveCoriolis }
   */
  computeCoriolis(vel) {
    const u = vel.surge || 0;
    const v = vel.sway || 0;
    const w = vel.heave || 0;
    const r = vel.yawRate || 0;

    // Simplified Coriolis terms (dominant cross-coupling only)
    // From Fossen: C_A(v) for added mass terms
    const mEff_surge = this.effectiveMass.surge;
    const mEff_sway = this.effectiveMass.sway;
    const mEff_heave = this.effectiveMass.heave;

    return {
      // Yaw rotation induces lateral force on surge (centripetal)
      surgeCoriolis: mEff_sway * v * r * 0.15,
      // Surge velocity during yaw creates sway force
      swayCoriolis: -mEff_surge * u * r * 0.15,
      // Minimal heave coupling (pitch rate would matter more)
      heaveCoriolis: 0,
    };
  }

  /**
   * Compute hydrostatic restoring forces & moments
   * Gravity pulls CG down, buoyancy pushes CB up.
   * When tilted, this creates a righting moment.
   * 
   * @param {object} attitude - { roll, pitch } in radians
   * @param {number} depth - Current depth in meters
   * @returns {object} Restoring forces { heaveRestore, rollRestore, pitchRestore }
   */
  computeRestoringForces(attitude, depth) {
    const roll = attitude.roll || 0;
    const pitch = attitude.pitch || 0;

    // Net buoyancy force (positive = upward)
    const W = this.mass * this.gravity;
    const B = (this.mass + this.buoyancy) * this.gravity;

    // Heave restoring: net buoyancy always pushes up
    // (B - W) is the buoyancy surplus
    const heaveRestore = (B - W) * Math.cos(roll) * Math.cos(pitch);

    // Roll restoring moment: BG_z * B * sin(roll)
    // CB above CG creates a righting moment opposing roll
    const BG_z = this.zB - this.zG;  // Distance between CB and CG
    const rollRestore = -BG_z * B * Math.sin(roll);

    // Pitch restoring moment: BG_z * B * sin(pitch)
    const pitchRestore = -BG_z * B * Math.sin(pitch);

    return { heaveRestore, rollRestore, pitchRestore };
  }

  /**
   * Integrate velocity using effective mass (rigid body + added mass)
   * 
   * a = F_net / m_effective
   * v_new = v_old + a * dt
   * 
   * @param {string} dof - Degree of freedom
   * @param {number} currentVel - Current velocity
   * @param {number} thrustForce - Applied thrust force
   * @param {number} dampingForce - Damping force (from computeDamping)
   * @param {number} coriolisForce - Coriolis force
   * @param {number} restoringForce - Restoring force
   * @param {number} dt - Time step
   * @returns {number} New velocity
   */
  integrateVelocity(dof, currentVel, thrustForce, dampingForce, coriolisForce, restoringForce, dt) {
    const mEff = this.effectiveMass[dof] || this.mass;
    const totalForce = thrustForce + dampingForce + coriolisForce + restoringForce;
    const accel = totalForce / mEff;

    let newVel = currentVel + accel * dt;

    // Velocity clamping (physical limits of T200 at 16V)
    const maxVel = {
      surge: 1.5,
      sway: 1.2,
      heave: 0.8,
    };
    const limit = maxVel[dof] || 2.0;
    newVel = Math.max(-limit, Math.min(limit, newVel));

    // NaN safety
    if (!isFinite(newVel)) return 0;

    return newVel;
  }

  /**
   * Integrate angular velocity for rotational DOFs
   * 
   * @param {string} dof - 'roll', 'pitch', or 'yaw'
   * @param {number} currentRate - Current angular rate (rad/s)
   * @param {number} moment - Applied moment (N·m)
   * @param {number} dt - Time step
   * @returns {number} New angular rate
   */
  integrateAngularVelocity(dof, currentRate, moment, dt) {
    const I = {
      roll: this.Ixx + this.Kpdot,
      pitch: this.Iyy + this.Mqdot,
      yaw: this.Izz + this.Nrdot,
    };

    const dampForce = this.computeDamping(dof, currentRate);
    const totalMoment = moment + dampForce;
    const angAccel = totalMoment / (I[dof] || 0.2);

    let newRate = currentRate + angAccel * dt;

    // Angular rate limits
    const maxRate = { roll: 1.5, pitch: 1.5, yaw: 2.0 };
    const limit = maxRate[dof] || 2.0;
    newRate = Math.max(-limit, Math.min(limit, newRate));

    if (!isFinite(newRate)) return 0;
    return newRate;
  }

  /**
   * Update underwater current simulation
   * Produces slowly-varying current that changes direction and magnitude
   * over time, simulating real pool conditions.
   * 
   * @param {number} dt - Time step (seconds)
   * @param {number} time - Total elapsed time
   */
  updateCurrent(dt, time) {
    this._currentPhaseX += dt * 0.13;
    this._currentPhaseZ += dt * 0.11;

    // Smooth sinusoidal base current (simulates large-scale pool circulation)
    const baseX = Math.sin(this._currentPhaseX) * 0.06
      + Math.sin(this._currentPhaseX * 2.3 + 1.7) * 0.03;
    const baseZ = Math.cos(this._currentPhaseZ) * 0.05
      + Math.cos(this._currentPhaseZ * 1.8 + 0.9) * 0.025;

    // Add intermittent gusts (simulates thruster wash from nearby activities)
    const gustFactor = Math.max(0, Math.sin(time * 0.4) * Math.sin(time * 0.17));
    const gustX = gustFactor * Math.sin(time * 1.3) * 0.04;
    const gustZ = gustFactor * Math.cos(time * 1.1) * 0.03;

    // Smooth interpolation toward target
    this.current.targetVx = Math.max(-this.current.maxSpeed,
      Math.min(this.current.maxSpeed, baseX + gustX));
    this.current.targetVz = Math.max(-this.current.maxSpeed,
      Math.min(this.current.maxSpeed, baseZ + gustZ));

    // First-order low-pass filter (slow response, like real water)
    const alpha = 1 - Math.exp(-dt / 3.0); // τ = 3 seconds
    this.current.vx += (this.current.targetVx - this.current.vx) * alpha;
    this.current.vz += (this.current.targetVz - this.current.vz) * alpha;
  }

  /**
   * Get current drift velocity in world frame
   * @returns {{ x: number, z: number, magnitude: number }}
   */
  getCurrentDrift() {
    return {
      x: this.current.vx,
      z: this.current.vz,
      magnitude: Math.sqrt(this.current.vx ** 2 + this.current.vz ** 2),
    };
  }

  /**
   * Transform current velocity from world frame to body frame
   * (needed for computing relative velocity & drag)
   * 
   * @param {number} heading - AUV heading in radians
   * @returns {{ surge: number, sway: number }}
   */
  getCurrentInBodyFrame(heading) {
    const cos = Math.cos(heading);
    const sin = Math.sin(heading);

    return {
      surge: this.current.vx * cos + this.current.vz * sin,
      sway: -this.current.vx * sin + this.current.vz * cos,
    };
  }

  /**
   * Full 6-DOF physics step
   * Computes all hydrodynamic effects and returns updated velocities
   * 
   * @param {object} state - Current vehicle state
   * @param {object} thrustForces - Forces from thruster allocation { surge, sway, heave, yaw, pitch, roll }
   * @param {number} dt - Time step
   * @param {number} time - Total elapsed time
   * @returns {object} Updated velocities and debug info
   */
  step(state, thrustForces, dt, time) {
    // Update underwater current
    this.updateCurrent(dt, time);

    // Get current in body frame for relative velocity computation
    const currentBody = this.getCurrentInBodyFrame(state.heading || 0);

    // Relative velocities (vehicle velocity minus current)
    const relSurge = (state.velSurge || 0) - currentBody.surge;
    const relSway = (state.velSway || 0) - currentBody.sway;
    const relHeave = state.velHeave || 0;
    const relYaw = state.velYaw || 0;

    // 1. Damping forces (computed on relative velocity)
    const dampSurge = this.computeDamping('surge', relSurge);
    const dampSway = this.computeDamping('sway', relSway);
    const dampHeave = this.computeDamping('heave', relHeave);

    // 2. Coriolis forces
    const coriolis = this.computeCoriolis({
      surge: relSurge,
      sway: relSway,
      heave: relHeave,
      yawRate: relYaw,
    });

    // 3. Restoring forces
    const restoring = this.computeRestoringForces(
      { roll: state.roll || 0, pitch: state.pitch || 0 },
      state.depth || 0.8
    );

    // 4. Convert thrust commands to forces (N)
    // Thrust force scale: command range [-100, 100] → [-50N, 50N] (T200 max ~50N at 16V)
    const thrustScale = 0.5; // N per unit command
    const fSurge = (thrustForces.surge || 0) * thrustScale;
    const fSway = (thrustForces.sway || 0) * thrustScale;
    const fHeave = (thrustForces.heave || 0) * thrustScale;

    // 5. Integrate translational velocities
    const newVelSurge = this.integrateVelocity(
      'surge', state.velSurge || 0,
      fSurge, dampSurge, coriolis.surgeCoriolis, 0, dt
    );

    const newVelSway = this.integrateVelocity(
      'sway', state.velSway || 0,
      fSway, dampSway, coriolis.swayCoriolis, 0, dt
    );

    const newVelHeave = this.integrateVelocity(
      'heave', state.velHeave || 0,
      fHeave, dampHeave, coriolis.heaveCoriolis,
      -restoring.heaveRestore / this.effectiveMass.heave, // Buoyancy opposes depth
      dt
    );

    // 6. Integrate angular velocities
    const yawMoment = (thrustForces.yaw || 0) * thrustScale * 0.15; // moment arm ~0.15m
    const newVelYaw = this.integrateAngularVelocity(
      'yaw', state.velYaw || 0, yawMoment, dt
    );

    // 7. Dynamic pitch & roll from acceleration + restoring moments
    const pitchMoment = (thrustForces.pitch || 0) * 0.02 + restoring.pitchRestore;
    const rollMoment = (thrustForces.roll || 0) * 0.02 + restoring.rollRestore;

    // Pitch/roll as spring-damper response (not integrated rates)
    const pitchAccel = pitchMoment / (this.Iyy + this.Mqdot);
    const rollAccel = rollMoment / (this.Ixx + this.Kpdot);

    return {
      velSurge: newVelSurge,
      velSway: newVelSway,
      velHeave: newVelHeave,
      velYaw: newVelYaw,
      pitchAccel,
      rollAccel,
      currentDrift: this.getCurrentDrift(),
      debug: {
        dampSurge,
        dampSway,
        dampHeave,
        coriolisSurge: coriolis.surgeCoriolis,
        coriolisSway: coriolis.swayCoriolis,
        restoringHeave: restoring.heaveRestore,
        restoringRoll: restoring.rollRestore,
        restoringPitch: restoring.pitchRestore,
        relVelSurge: relSurge,
        relVelSway: relSway,
      },
    };
  }

  /**
   * Reset all dynamic state
   */
  reset() {
    this.current.vx = 0;
    this.current.vz = 0;
    this.current.targetVx = 0;
    this.current.targetVz = 0;
    this._currentPhaseX = Math.random() * Math.PI * 2;
    this._currentPhaseZ = Math.random() * Math.PI * 2;
  }
}

const hydrodynamicsEngine = new HydrodynamicsEngine();
export default hydrodynamicsEngine;
