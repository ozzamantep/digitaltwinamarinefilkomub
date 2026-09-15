/**
 * Environment Model for Underwater Digital Twin
 * 
 * Simulates physical oceanographic and pool environmental properties:
 * - Water density as a function of temperature & salinity (UNESCO equation)
 * - Hydrostatic pressure calculation (depth -> bar/Pa)
 * - 3D current velocity field with depth profiles, shear, and Gauss-Markov turbulence
 * - Scenario presets (calm pool, circulation currents, ocean currents)
 * 
 * Reference:
 *   - Fossen (2021) Marine Craft Hydrodynamics, Section 8.3 (Environmental Disturbances)
 *   - UNESCO 1980 Equation of State for Seawater
 */

export const CURRENT_PRESETS = {
  CALM: 'calm',
  POOL_CIRCULATION: 'pool_circulation',
  MILD_CROSS_CURRENT: 'mild_cross_current',
  STRONG_CURRENT: 'strong_current',
  TURBULENT_WAVE: 'turbulent_wave',
  CUSTOM: 'custom',
};

export class EnvironmentModel {
  constructor(config = {}) {
    // Basic physical constants
    this.gravity = config.gravity ?? 9.80665; // m/s²
    this.atmosphericPressure = config.atmosphericPressure ?? 101325; // Pa (1 atm)

    // Fluid properties
    this.temperature = config.temperature ?? 26.0; // °C (pool temperature)
    this.salinity = config.salinity ?? 0.0;        // PSU (0 = fresh water pool, 35 = sea water)
    this.waterDensity = this.computeWaterDensity(this.temperature, this.salinity);

    // Current generation configuration
    this.preset = config.preset ?? CURRENT_PRESETS.POOL_CIRCULATION;

    this.current = {
      speed: 0.05,       // Mean horizontal speed (m/s)
      direction: Math.PI / 4, // Direction in NED frame (rad, 0 = North, π/2 = East)
      verticalSpeed: 0.0, // Up/down current (m/s)
      turbulenceIntensity: 0.02, // RMS turbulence speed (m/s)
      correlationTime: 4.0,     // Gauss-Markov correlation time (s)
    };

    // Internal state for colored noise (1st order Gauss-Markov process)
    this._turbX = 0.0;
    this._turbY = 0.0;
    this._turbZ = 0.0;
    this._phase = Math.random() * Math.PI * 2;
    this._time = 0.0;

    // Apply initial preset
    this.setPreset(this.preset);
  }

  /**
   * Set environmental preset
   */
  setPreset(presetName) {
    this.preset = presetName;
    switch (presetName) {
      case CURRENT_PRESETS.CALM:
        this.current.speed = 0.0;
        this.current.turbulenceIntensity = 0.002;
        this.salinity = 0.0;
        break;
      case CURRENT_PRESETS.POOL_CIRCULATION:
        this.current.speed = 0.0;  // Swimming pool: still water
        this.current.direction = 0.0;
        this.current.turbulenceIntensity = 0.0; // No turbulence in indoor pool
        this.salinity = 0.0;
        break;
      case CURRENT_PRESETS.MILD_CROSS_CURRENT:
        this.current.speed = 0.12;
        this.current.direction = Math.PI * 0.5; // Starboard cross current
        this.current.turbulenceIntensity = 0.03;
        this.salinity = 0.0;
        break;
      case CURRENT_PRESETS.STRONG_CURRENT:
        this.current.speed = 0.25;
        this.current.direction = Math.PI * 0.75;
        this.current.turbulenceIntensity = 0.06;
        this.salinity = 35.0; // Sea water
        break;
      case CURRENT_PRESETS.TURBULENT_WAVE:
        this.current.speed = 0.10;
        this.current.direction = 0.0;
        this.current.turbulenceIntensity = 0.08;
        this.salinity = 0.0;
        break;
      default:
        break;
    }
    this.waterDensity = this.computeWaterDensity(this.temperature, this.salinity);
  }

  /**
   * Compute water density using standard empirical relationship
   * @param {number} T - Temperature in °C
   * @param {number} S - Salinity in PSU (g/kg)
   * @returns {number} Density in kg/m³
   */
  computeWaterDensity(T, S) {
    // Standard pure water density formula (Kell, 1975)
    const rhoPure = 999.842594 +
      6.793952e-2 * T -
      9.095290e-3 * T * T +
      1.001685e-4 * T * T * T -
      1.120083e-6 * T * T * T * T +
      6.536332e-9 * T * T * T * T * T;

    // Salinity correction (UNESCO 1980 polynomial simplified)
    const A = 0.824493 - 4.0899e-3 * T + 7.6438e-5 * T * T - 8.2467e-7 * T * T * T;
    const B = -5.72466e-3 + 1.0227e-4 * T - 1.6546e-6 * T * T;
    const C = 4.8314e-4;

    return rhoPure + A * S + B * Math.pow(S, 1.5) + C * S * S;
  }

  /**
   * Compute absolute hydrostatic pressure at depth z (NED frame, z >= 0)
   * P(z) = P_atm + ρ * g * z
   * 
   * @param {number} depth - Depth below surface in meters
   * @returns {{ pressurePa: number, pressureBar: number, pressureDbar: number }}
   */
  getHydrostaticPressure(depth) {
    const d = Math.max(0, depth);
    const pWater = this.waterDensity * this.gravity * d;
    const pressurePa = this.atmosphericPressure + pWater;
    return {
      pressurePa,
      pressureBar: pressurePa * 1e-5,
      pressureDbar: pressurePa * 1e-4,
    };
  }

  /**
   * Step the environment simulation forward by dt
   * Updates turbulence via 1st order Gauss-Markov:
   *   d(turb)/dt = -1/T * turb + σ * sqrt(2/T) * w(t)
   */
  update(dt) {
    this._time += dt;

    const T = Math.max(0.5, this.current.correlationTime);
    const sigma = this.current.turbulenceIntensity;
    const decay = Math.exp(-dt / T);
    const noiseGain = sigma * Math.sqrt(Math.max(0, 1 - decay * decay));

    // Box-Muller Gaussian random values
    const u1 = Math.max(1e-7, Math.random());
    const u2 = Math.random();
    const u3 = Math.max(1e-7, Math.random());
    const u4 = Math.random();

    const g1 = Math.sqrt(-2 * Math.log(u1)) * Math.cos(2 * Math.PI * u2);
    const g2 = Math.sqrt(-2 * Math.log(u1)) * Math.sin(2 * Math.PI * u2);
    const g3 = Math.sqrt(-2 * Math.log(u3)) * Math.cos(2 * Math.PI * u4);

    this._turbX = this._turbX * decay + noiseGain * g1;
    this._turbY = this._turbY * decay + noiseGain * g2;
    this._turbZ = this._turbZ * decay + noiseGain * g3;
  }

  /**
   * Get 3D ocean/pool current velocity at vehicle position in NED frame
   * [v_cx, v_cy, v_cz] (m/s)
   * 
   * @param {number} x - North position (m)
   * @param {number} y - East position (m)
   * @param {number} depth - Depth (m, NED z >= 0)
   * @returns {{vx: number, vy: number, vz: number, speed: number, direction: number}}
   */
  getCurrentVelocity(x = 0, y = 0, depth = 1.0) {
    // Base horizontal current
    const baseSpeed = this.current.speed;
    const dir = this.current.direction;

    // Depth shear profile: in shallow pool, circulation is slightly higher near surface
    // Profile factor: 1.0 at surface, decaying to 0.7 near floor (depth ~ 2.0m)
    const depthFactor = Math.max(0.6, 1.0 - 0.15 * Math.min(depth, 2.0));

    // Spatial circulation wave (sinusoidal eddy in pool)
    const spatialEddy = 0.015 * Math.sin(x * 0.3 + this._time * 0.2);

    const vx = (baseSpeed * Math.cos(dir) + spatialEddy) * depthFactor + this._turbX;
    const vy = (baseSpeed * Math.sin(dir) + spatialEddy) * depthFactor + this._turbY;
    const vz = this.current.verticalSpeed + this._turbZ * 0.3; // Less vertical turbulence in pool

    const totalSpeed = Math.sqrt(vx * vx + vy * vy + vz * vz);
    const effectiveDir = Math.atan2(vy, vx);

    return {
      vx,
      vy,
      vz,
      speed: totalSpeed,
      direction: effectiveDir,
    };
  }

  /**
   * Export snapshot of environment for Digital Twin state
   */
  getSnapshot(depth = 1.0) {
    const press = this.getHydrostaticPressure(depth);
    const curr = this.getCurrentVelocity(0, 0, depth);
    return {
      waterDensity: this.waterDensity,
      gravity: this.gravity,
      temperature: this.temperature,
      salinity: this.salinity,
      pressureBar: press.pressureBar,
      current: curr,
      preset: this.preset,
    };
  }
}

export const defaultEnvironment = new EnvironmentModel();
export default defaultEnvironment;
