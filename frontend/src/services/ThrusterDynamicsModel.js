/**
 * Blue Robotics T200 Thruster Dynamics Model
 * 
 * Simulates realistic motor behavior including:
 * - First-order motor ramp-up lag (τ = 0.35s)
 * - PWM deadband (1470-1530μs)
 * - Voltage sag under load (LiPo internal resistance)
 * - Non-linear thrust curve interpolation from T200 datasheet
 * - Cross-coupling efficiency loss between adjacent thrusters
 * 
 * Reference: Blue Robotics T200 Performance Data (16V, September 2019)
 * https://bluerobotics.com/store/thrusters/t100-t200-thrusters/t200-thruster-r2-rp/
 */

class ThrusterDynamicsModel {
  constructor() {
    this.numThrusters = 6;

    // Time constant for motor spin-up (seconds)
    // T200 takes approximately 0.3-0.5s to reach commanded thrust
    this.motorTimeConstant = 0.35;

    // PWM deadband range (microseconds)
    this.deadbandLow = 1470;
    this.deadbandHigh = 1530;
    this.neutralPWM = 1500;
    // SAFETY: command limits 1300-1600 µs (full range pernah memecahkan propeller)
    this.minPWM = 1300;
    this.maxPWM = 1600;

    // T200 Performance at 16V (4S LiPo nominal)
    this.nominalVoltage = 16.0;
    this.maxThrust_fwd = 5.1;  // kgf at full forward
    this.maxThrust_rev = 4.1;  // kgf at full reverse
    this.maxRPM = 3600;
    this.maxCurrent_per_thruster = 25.0; // Amps at full thrust

    // Thrust curve lookup table (PWM → kgf at 16V)
    // Source: Official Blue Robotics T200 Performance Data
    this.thrustCurve = [
      { pwm: 1100, thrust: -4.10, rpm: -3200, current: 22.0 },
      { pwm: 1150, thrust: -3.20, rpm: -2800, current: 16.5 },
      { pwm: 1200, thrust: -2.40, rpm: -2400, current: 12.0 },
      { pwm: 1250, thrust: -1.60, rpm: -2000, current: 8.0 },
      { pwm: 1300, thrust: -0.95, rpm: -1600, current: 5.0 },
      { pwm: 1350, thrust: -0.50, rpm: -1200, current: 3.0 },
      { pwm: 1400, thrust: -0.18, rpm: -800, current: 1.5 },
      { pwm: 1450, thrust: -0.04, rpm: -300, current: 0.8 },
      { pwm: 1470, thrust: 0.00, rpm: 0, current: 0.5 },
      { pwm: 1500, thrust: 0.00, rpm: 0, current: 0.5 },
      { pwm: 1530, thrust: 0.00, rpm: 0, current: 0.5 },
      { pwm: 1550, thrust: 0.05, rpm: 350, current: 0.9 },
      { pwm: 1600, thrust: 0.22, rpm: 900, current: 1.8 },
      { pwm: 1650, thrust: 0.55, rpm: 1300, current: 3.5 },
      { pwm: 1700, thrust: 1.10, rpm: 1700, current: 5.5 },
      { pwm: 1750, thrust: 1.80, rpm: 2100, current: 8.5 },
      { pwm: 1800, thrust: 2.70, rpm: 2500, current: 13.0 },
      { pwm: 1850, thrust: 3.60, rpm: 2900, current: 17.5 },
      { pwm: 1900, thrust: 5.10, rpm: 3600, current: 25.0 },
    ];

    // Cross-coupling efficiency matrix
    // Adjacent thrusters reduce each other's efficiency by 5-10%
    // Format: couplingMatrix[i][j] = efficiency loss of thruster i due to thruster j
    // Only significant for physically adjacent thrusters
    this.couplingPairs = [
      // [thruster_a, thruster_b, coupling_factor]
      [0, 1, 0.05],  // Front-Left ↔ Front-Right (share front frame)
      [2, 3, 0.05],  // Rear-Left ↔ Rear-Right (share rear frame)
      [0, 2, 0.03],  // Front-Left ↔ Rear-Left (same side, less coupling)
      [1, 3, 0.03],  // Front-Right ↔ Rear-Right (same side)
      [4, 5, 0.08],  // Front-Vert ↔ Rear-Vert (both push water vertically, more coupling)
    ];

    // Internal state: actual thrust per thruster (with ramp-up lag)
    this.actualThrust = new Array(this.numThrusters).fill(0);   // kgf
    this.actualRPM = new Array(this.numThrusters).fill(0);      // RPM
    this.actualCurrent = new Array(this.numThrusters).fill(0.5); // Amps

    // Commanded values (target that actual ramps toward)
    this.commandedThrust = new Array(this.numThrusters).fill(0);
    this.failedThrusters = new Set();
  }

  /**
   * Convert normalized command [-100, +100] to PWM microseconds
   * @param {number} cmd - Normalized command (-100 to +100)
   * @returns {number} PWM in microseconds
   */
  commandToPWM(cmd) {
    const clamped = Math.max(-100, Math.min(100, cmd));

    // Deadband: commands within ±5% produce neutral PWM
    if (Math.abs(clamped) < 5) return this.neutralPWM;

    if (clamped > 0) {
      // Forward: map 5..100 → 1530..1600 (safety-limited)
      const normalized = (clamped - 5) / 95;
      return this.deadbandHigh + normalized * (this.maxPWM - this.deadbandHigh);
    } else {
      // Reverse: map -5..-100 → 1470..1300 (safety-limited)
      const normalized = (-clamped - 5) / 95;
      return this.deadbandLow - normalized * (this.deadbandLow - this.minPWM);
    }
  }

  /**
   * Look up thrust, RPM, and current from PWM using T200 datasheet interpolation
   * @param {number} pwm - PWM signal in microseconds
   * @returns {{ thrust: number, rpm: number, current: number }}
   */
  lookupT200(pwm) {
    const clamped = Math.max(this.minPWM, Math.min(this.maxPWM, pwm));

    for (let i = 0; i < this.thrustCurve.length - 1; i++) {
      if (clamped >= this.thrustCurve[i].pwm && clamped <= this.thrustCurve[i + 1].pwm) {
        const t = (clamped - this.thrustCurve[i].pwm) /
          (this.thrustCurve[i + 1].pwm - this.thrustCurve[i].pwm);
        return {
          thrust: this.thrustCurve[i].thrust + t * (this.thrustCurve[i + 1].thrust - this.thrustCurve[i].thrust),
          rpm: this.thrustCurve[i].rpm + t * (this.thrustCurve[i + 1].rpm - this.thrustCurve[i].rpm),
          current: this.thrustCurve[i].current + t * (this.thrustCurve[i + 1].current - this.thrustCurve[i].current),
        };
      }
    }

    return { thrust: 0, rpm: 0, current: 0.5 };
  }

  /**
   * Compute voltage sag factor based on total current draw
   * Under heavy load, LiPo voltage drops → reduced thrust
   * 
   * V_sag = V_nominal - I_total * R_internal
   * thrust_factor = (V_actual / V_nominal) ^ 1.5
   * 
   * @param {number} totalCurrent - Total current draw from all thrusters (Amps)
   * @param {number} batteryVoltage - Current battery voltage
   * @returns {number} Thrust efficiency factor (0.7 to 1.0)
   */
  computeVoltageSagFactor(totalCurrent, batteryVoltage) {
    // 4S LiPo internal resistance: ~40-80 mΩ total
    const internalResistance = 0.06; // 60 mΩ typical for 4S pack
    const voltageDrop = totalCurrent * internalResistance;
    const actualVoltage = Math.max(13.0, batteryVoltage - voltageDrop);
    const sagFactor = Math.pow(actualVoltage / this.nominalVoltage, 1.5);

    return Math.max(0.65, Math.min(1.0, sagFactor));
  }

  /**
   * Compute cross-coupling efficiency loss
   * Adjacent thrusters disturb each other's flow → reduced efficiency
   * 
   * IMPROVED: Only apply coupling loss when thrusters have DIFFERENT commands
   * For symmetric forward/straight motion, paired thrusters get NO coupling loss
   * This ensures stable forward motion with all thrusters equally responsive
   * 
   * @param {number} thrusterIdx - Index of the thruster (0-5)
   * @param {number[]} allCommands - All 6 thruster commands
   * @returns {number} Efficiency factor (0.90 to 1.0)
   */
  computeCrossCouplingFactor(thrusterIdx, allCommands) {
    let efficiencyLoss = 0;

    for (const [a, b, factor] of this.couplingPairs) {
      // Only apply coupling loss if paired thrusters have DIFFERENT commands
      // If commands are within 5%, treat as symmetric motion (no coupling loss)
      const cmdA = Math.abs(allCommands[a] || 0);
      const cmdB = Math.abs(allCommands[b] || 0);
      const commandDifference = Math.abs(cmdA - cmdB);
      
      if (a === thrusterIdx && commandDifference > 5) {  // Tolerance: 5% difference
        efficiencyLoss += factor * (cmdB / 100) * (commandDifference / 100);
      }
      if (b === thrusterIdx && commandDifference > 5) {
        efficiencyLoss += factor * (cmdA / 100) * (commandDifference / 100);
      }
    }

    // Allow higher efficiency (up to 1.0) for symmetric configurations
    return Math.max(0.90, Math.min(1.0, 1.0 - efficiencyLoss));
  }

  /**
   * Main update step: simulate all 6 thrusters for one time step
   * 
   * @param {number[]} commands - 6 thruster commands [-100, +100]
   * @param {number} batteryVoltage - Current battery voltage
   * @param {number} dt - Time step (seconds)
   * @returns {{ thrusts: number[], rpms: number[], currents: number[], totalCurrent: number, efforts: number[] }}
   */
  update(commands, batteryVoltage, dt) {
    const thrusts = [];
    const rpms = [];
    const currents = [];
    const efforts = [];

    // First pass: compute target thrust for each thruster
    for (let i = 0; i < this.numThrusters; i++) {
      const cmd = this.failedThrusters.has(i) ? 0 : commands[i] || 0;
      const pwm = this.commandToPWM(cmd);
      const lookup = this.lookupT200(pwm);
      this.commandedThrust[i] = lookup.thrust;
    }

    // Compute total current for voltage sag
    let totalCurrentEstimate = 0;
    for (let i = 0; i < this.numThrusters; i++) {
      const pwm = this.commandToPWM(this.failedThrusters.has(i) ? 0 : commands[i] || 0);
      const lookup = this.lookupT200(pwm);
      totalCurrentEstimate += Math.abs(lookup.current);
    }

    const sagFactor = this.computeVoltageSagFactor(totalCurrentEstimate, batteryVoltage);

    // Second pass: apply dynamics per thruster
    for (let i = 0; i < this.numThrusters; i++) {
      const targetThrust = this.commandedThrust[i];

      // 1. First-order motor ramp-up lag
      // thrust_actual += (thrust_target - thrust_actual) * (1 - e^(-dt/τ))
      const alpha = 1 - Math.exp(-dt / this.motorTimeConstant);
      this.actualThrust[i] += (targetThrust - this.actualThrust[i]) * alpha;

      // 2. Apply voltage sag
      const saggedThrust = this.actualThrust[i] * sagFactor;

      // 3. Apply cross-coupling efficiency loss
      const couplingFactor = this.computeCrossCouplingFactor(i, commands);
      const finalThrust = saggedThrust * couplingFactor;

      // 4. Compute actual RPM and current from final thrust
      // Reverse-lookup from thrust → RPM/current (approximate linear mapping)
      const thrustRatio = finalThrust / this.maxThrust_fwd;
      const rpm = thrustRatio * this.maxRPM;
      const current = Math.abs(thrustRatio) * this.maxCurrent_per_thruster + 0.5;

      this.actualRPM[i] += (rpm - this.actualRPM[i]) * alpha;
      this.actualCurrent[i] += (current - this.actualCurrent[i]) * alpha;

      thrusts.push(finalThrust);
      rpms.push(Math.round(this.actualRPM[i]));
      currents.push(this.actualCurrent[i]);

      // Effort in percentage for display (scaled to ±100%)
      const effortPct = (finalThrust / this.maxThrust_fwd) * 100;
      efforts.push(Math.round(Math.max(-100, Math.min(100, effortPct))));
    }

    const totalCurrent = currents.reduce((sum, c) => sum + c, 0);

    return {
      thrusts,    // kgf per thruster
      rpms,       // RPM per thruster
      currents,   // Amps per thruster
      totalCurrent,
      efforts,    // Percentage output per thruster for display
      sagFactor,  // How much voltage sag reduced thrust (0.65-1.0)
    };
  }

  /**
   * Convert thruster outputs to body-frame forces/moments
   * Uses the 6-thruster allocation matrix from the thesis document
   * 
   * 4x Horizontal thrusters at 45° corners → surge, sway, yaw
   * 2x Vertical thrusters → heave, pitch
   * 
   * @param {number[]} thrusts - 6 thrust values in kgf
   * @returns {{ surge: number, sway: number, heave: number, yaw: number, pitch: number, roll: number }}
   */
  thrustToBodyForces(thrusts) {
    const [t1, t2, t3, t4, t5, t6] = thrusts;
    const cos45 = Math.cos(Math.PI / 4); // 0.7071
    const kgfToN = 9.81; // Convert kgf to Newtons

    // Thruster layout (from BlueROV2Model.jsx):
    // T1: Front-Left  (+45°) → pushes forward-right
    // T2: Front-Right (-45°) → pushes forward-left
    // T3: Rear-Left   (+135°) → pushes forward-right
    // T4: Rear-Right  (-135°) → pushes forward-left
    // T5: Front-Vertical → heave + pitch forward
    // T6: Rear-Vertical  → heave + pitch rear

    const surge = (t1 + t2 + t3 + t4) * cos45 * kgfToN;
    const sway = (-t1 + t2 + t3 - t4) * cos45 * kgfToN;
    const heave = (t5 + t6) * kgfToN;
    const yaw = (t1 - t2 + t3 - t4) * cos45 * 0.20 * kgfToN; // moment arm ~0.20m
    const pitch = (t5 - t6) * 0.18 * kgfToN; // moment arm ~0.18m (front-rear distance)
    const roll = 0; // No direct roll control in this configuration

    return { surge, sway, heave, yaw, pitch, roll };
  }

  /**
   * Reset all thruster states to zero
   */
  reset() {
    this.actualThrust.fill(0);
    this.actualRPM.fill(0);
    this.actualCurrent.fill(0.5);
    this.commandedThrust.fill(0);
    this.failedThrusters.clear();
  }

  setFailedThrusters(indices = []) {
    this.failedThrusters = new Set(indices.filter((index) => Number.isInteger(index) && index >= 0 && index < this.numThrusters));
  }

  forceStop(indices) {
    for (const index of indices) {
      if (index < 0 || index >= this.numThrusters) continue;
      this.actualThrust[index] = 0;
      this.actualRPM[index] = 0;
      this.actualCurrent[index] = 0.5;
      this.commandedThrust[index] = 0;
    }
  }
}

const thrusterDynamics = new ThrusterDynamicsModel();
export default thrusterDynamics;
