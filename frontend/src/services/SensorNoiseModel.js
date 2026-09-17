/**
 * Realistic Underwater Sensor Noise Model
 * 
 * Replaces simplistic sin() + random() noise with proper statistical
 * noise models matching real sensor behavior:
 * 
 * - IMU (BNO055/ICM-20948): Gaussian noise + gyro bias drift + accel cross-axis
 * - Depth (MS5837-30BA): Quantization noise + thermal drift + first-order lag
 * - DVL (Acoustic Ranger): Gaussian + occasional multipath spikes
 * - Magnetometer/Compass: Hard-iron offset + soft-iron distortion + Gaussian
 * - Battery ADC: 12-bit quantization + switching ripple
 * 
 * Uses Box-Muller transform for proper Gaussian distribution.
 */

class SensorNoiseModel {
  constructor() {
    // =========================================================================
    // IMU Noise Parameters (typical MEMS IMU: BNO055 / ICM-20948)
    // =========================================================================
    this.imu = {
      // Gyroscope
      gyroNoiseDensity: 0.1,      // °/s/√Hz — angular random walk
      gyroBiasStability: 0.01,    // °/s — bias instability
      gyroBias: { x: 0, y: 0, z: 0 }, // Current bias state (random walk)
      gyroBiasWalkRate: 0.005,    // °/s per second — how fast bias drifts

      // Accelerometer
      accelNoiseDensity: 0.02,    // m/s²/√Hz
      accelCrossAxis: 0.02,       // 2% cross-axis sensitivity
      accelBias: { x: 0, y: 0, z: 0 },
      accelBiasWalkRate: 0.001,

      // Magnetometer / Compass
      magHardIronOffset: 2.5,     // ° — constant offset from nearby motors/batteries
      magSoftIronScale: 0.97,     // Scale factor from soft-iron distortion
      magNoiseStd: 0.8,           // ° — Gaussian noise standard deviation
    };

    // =========================================================================
    // Depth Sensor Parameters (Blue Robotics Bar30 / MS5837-30BA)
    // =========================================================================
    this.depth = {
      resolution: 0.002,           // 0.2 mbar ≈ 0.002m depth resolution
      noiseStd: 0.003,             // 3mm Gaussian noise standard deviation
      thermalCoeff: 0.0005,        // 0.5 mbar/°C thermal drift
      filterTau: 0.08,             // 80ms first-order filter time constant
      filteredValue: null,         // Internal filter state
      pressureNoiseStd: 0.15,      // kPa noise on pressure reading
    };

    // =========================================================================
    // DVL / Acoustic Altimeter Parameters
    // =========================================================================
    this.dvl = {
      noiseStd: 0.005,             // 5mm Gaussian noise
      spikeProb: 0.005,            // 0.5% chance of multipath spike per reading
      spikeAmplitude: 0.15,        // 15cm spike magnitude
      minRange: 0.1,               // Below this → bottom-lock loss
      maxRange: 5.0,               // Above this → signal too weak
    };

    // =========================================================================
    // Battery ADC Parameters (12-bit ADC + switching regulator ripple)
    // =========================================================================
    this.battery = {
      adcBits: 12,
      adcRange: 20.0,              // Full-scale voltage range (0-20V)
      adcStep: 0,                  // Computed in constructor
      rippleAmplitude: 0.05,       // 50mV switching ripple
      rippleFrequency: 200,        // Hz — typical buck converter switching
      currentNoiseStd: 0.15,       // Amps — current sensor noise
    };

    // Compute ADC step size
    this.battery.adcStep = this.battery.adcRange / Math.pow(2, this.battery.adcBits);

    // Pre-allocate Gaussian pair cache for Box-Muller
    this._gaussianSpare = null;
    this._hasSpare = false;
  }

  /**
   * Generate a Gaussian random number using Box-Muller transform
   * Produces proper normal distribution N(0, 1), much better than Math.random()
   * 
   * @param {number} mean - Mean of distribution (default 0)
   * @param {number} std - Standard deviation (default 1)
   * @returns {number} Gaussian random value
   */
  gaussian(mean = 0, std = 1) {
    if (this._hasSpare) {
      this._hasSpare = false;
      return mean + std * this._gaussianSpare;
    }

    let u, v, s;
    do {
      u = Math.random() * 2 - 1;
      v = Math.random() * 2 - 1;
      s = u * u + v * v;
    } while (s >= 1 || s === 0);

    const mul = Math.sqrt(-2.0 * Math.log(s) / s);
    this._gaussianSpare = v * mul;
    this._hasSpare = true;

    return mean + std * u * mul;
  }

  /**
   * Apply IMU gyroscope noise with bias drift
   * 
   * @param {object} rawIMU - { roll, pitch, yaw, accelX, accelY, accelZ }
   * @param {number} dt - Time step for bias walk integration
   * @returns {object} Noisy IMU readings
   */
  applyIMUNoise(rawIMU, dt) {
    // Update gyroscope bias random walk
    this.imu.gyroBias.x += this.gaussian(0, this.imu.gyroBiasWalkRate * Math.sqrt(dt));
    this.imu.gyroBias.y += this.gaussian(0, this.imu.gyroBiasWalkRate * Math.sqrt(dt));
    this.imu.gyroBias.z += this.gaussian(0, this.imu.gyroBiasWalkRate * Math.sqrt(dt));

    // Clamp bias to reasonable bounds (auto-reset if sensor would recalibrate)
    const maxBias = 0.5; // °/s max bias before auto-correction
    this.imu.gyroBias.x = Math.max(-maxBias, Math.min(maxBias, this.imu.gyroBias.x));
    this.imu.gyroBias.y = Math.max(-maxBias, Math.min(maxBias, this.imu.gyroBias.y));
    this.imu.gyroBias.z = Math.max(-maxBias, Math.min(maxBias, this.imu.gyroBias.z));

    // Apply Gaussian noise + bias to angular readings
    const rollNoise = this.gaussian(0, this.imu.gyroNoiseDensity) + this.imu.gyroBias.x;
    const pitchNoise = this.gaussian(0, this.imu.gyroNoiseDensity) + this.imu.gyroBias.y;
    const yawNoise = this.gaussian(0, this.imu.gyroNoiseDensity) + this.imu.gyroBias.z;

    // Apply accelerometer noise + cross-axis coupling
    const accelNoiseX = this.gaussian(0, this.imu.accelNoiseDensity);
    const accelNoiseY = this.gaussian(0, this.imu.accelNoiseDensity);
    const accelNoiseZ = this.gaussian(0, this.imu.accelNoiseDensity);

    // Cross-axis: X accel leaks into Y and Z by ~2%
    const crossX = (rawIMU.accelY || 0) * this.imu.accelCrossAxis;
    const crossY = (rawIMU.accelX || 0) * this.imu.accelCrossAxis;

    return {
      roll: (rawIMU.roll || 0) + rollNoise,
      pitch: (rawIMU.pitch || 0) + pitchNoise,
      yaw: (rawIMU.yaw || 0) + yawNoise,
      accelX: (rawIMU.accelX || 0) + accelNoiseX + crossX,
      accelY: (rawIMU.accelY || 0) + accelNoiseY + crossY,
      accelZ: (rawIMU.accelZ || 0) + accelNoiseZ,
    };
  }

  /**
   * Apply depth sensor noise (MS5837-30BA)
   * Includes quantization, Gaussian noise, thermal drift, and first-order filter lag
   * 
   * @param {number} rawDepth - True depth in meters
   * @param {number} temperature - Water temperature (for thermal drift)
   * @param {number} dt - Time step
   * @returns {{ depth: number, pressure: number }}
   */
  applyDepthNoise(rawDepth, temperature, dt) {
    // Initialize filter
    if (this.depth.filteredValue === null) {
      this.depth.filteredValue = rawDepth;
    }

    // 1. Add Gaussian measurement noise
    const noisyDepth = rawDepth + this.gaussian(0, this.depth.noiseStd);

    // 2. Quantize to sensor resolution (0.2 mbar → ~2mm depth)
    const quantized = Math.round(noisyDepth / this.depth.resolution) * this.depth.resolution;

    // 3. Thermal drift (MS5837 has slight temperature sensitivity)
    const tempRef = 25.0; // Reference calibration temperature
    const thermalDrift = (temperature - tempRef) * this.depth.thermalCoeff;

    // 4. First-order low-pass filter (sensor response time)
    const alpha = 1 - Math.exp(-dt / this.depth.filterTau);
    this.depth.filteredValue += (quantized + thermalDrift - this.depth.filteredValue) * alpha;

    // Ensure non-negative depth
    const finalDepth = Math.max(0, this.depth.filteredValue);

    // Compute noisy pressure reading
    // P = P_atm + ρgh
    const pAtm = 101.325 + this.gaussian(0, 0.02); // Atmospheric with barometric noise
    const pHydrostatic = finalDepth * 9.80665;
    const pressureNoise = this.gaussian(0, this.depth.pressureNoiseStd);
    const pressure = pAtm + pHydrostatic + pressureNoise;

    return {
      depth: finalDepth,
      pressure,
    };
  }

  /**
   * Apply DVL / acoustic altimeter noise
   * Includes Gaussian noise and occasional multipath spikes
   * 
   * @param {number} rawAltitude - True altitude above bottom (meters)
   * @returns {number} Noisy altitude reading
   */
  applyDVLNoise(rawAltitude) {
    // 1. Gaussian noise
    let noisy = rawAltitude + this.gaussian(0, this.dvl.noiseStd);

    // 2. Occasional multipath spike (acoustic reflection off walls)
    if (Math.random() < this.dvl.spikeProb) {
      noisy += (Math.random() > 0.5 ? 1 : -1) * this.dvl.spikeAmplitude * Math.random();
    }

    // 3. Bottom-lock loss (too close or too far)
    if (rawAltitude < this.dvl.minRange) {
      // Very close → increased noise (near-field acoustics)
      noisy += this.gaussian(0, this.dvl.noiseStd * 5);
    }
    if (rawAltitude > this.dvl.maxRange) {
      // Too far → signal loss, return last known + large noise
      noisy += this.gaussian(0, 0.5);
    }

    return Math.max(0.02, noisy);
  }

  /**
   * Apply magnetometer / compass heading noise
   * Includes hard-iron offset, soft-iron distortion, and Gaussian noise
   * 
   * @param {number} rawHeading - True heading in degrees
   * @returns {number} Noisy heading in degrees
   */
  applyCompassNoise(rawHeading) {
    // 1. Hard-iron offset (constant bias from nearby magnets/motors)
    let noisy = rawHeading + this.imu.magHardIronOffset;

    // 2. Soft-iron distortion (heading-dependent error)
    const headingRad = rawHeading * Math.PI / 180;
    noisy += Math.sin(2 * headingRad) * 1.2; // 2nd harmonic error typical of soft-iron

    // 3. Gaussian noise
    noisy += this.gaussian(0, this.imu.magNoiseStd);

    // Wrap to 0-360°
    noisy = ((noisy % 360) + 360) % 360;

    return noisy;
  }

  /**
   * Apply battery ADC noise (12-bit quantization + switching ripple)
   * 
   * @param {number} rawVoltage - True battery voltage
   * @param {number} rawCurrent - True current draw
   * @param {number} time - Elapsed time for ripple phase
   * @returns {{ voltage: number, current: number }}
   */
  applyBatteryNoise(rawVoltage, rawCurrent, time) {
    // 1. ADC quantization
    const quantizedVoltage = Math.round(rawVoltage / this.battery.adcStep) * this.battery.adcStep;

    // 2. Switching ripple (buck converter noise at ~200Hz)
    const ripple = this.battery.rippleAmplitude *
      Math.sin(time * this.battery.rippleFrequency * 2 * Math.PI) *
      (0.7 + Math.random() * 0.3); // Slightly random amplitude

    // 3. ADC noise floor
    const adcNoise = this.gaussian(0, this.battery.adcStep * 0.5);

    const finalVoltage = quantizedVoltage + ripple + adcNoise;

    // Current sensor noise (Hall-effect based, inherently noisy)
    const currentNoise = this.gaussian(0, this.battery.currentNoiseStd);
    const finalCurrent = Math.max(0, rawCurrent + currentNoise);

    return {
      voltage: Math.max(0, finalVoltage),
      current: finalCurrent,
    };
  }

  /**
   * Apply water temperature sensor noise
   * NTC thermistor with slow response and quantization
   * 
   * @param {number} rawTemp - True temperature in °C
   * @returns {number} Noisy temperature
   */
  applyTemperatureNoise(rawTemp) {
    // NTC thermistor: ±0.1°C accuracy, 10-bit ADC
    const quantStep = 0.1;
    const quantized = Math.round(rawTemp / quantStep) * quantStep;
    const noise = this.gaussian(0, 0.04); // 40mK noise floor
    return quantized + noise;
  }

  /**
   * Reset all internal state (bias drifts, filter states)
   */
  reset() {
    this.imu.gyroBias = { x: 0, y: 0, z: 0 };
    this.imu.accelBias = { x: 0, y: 0, z: 0 };
    this.depth.filteredValue = null;
    this._hasSpare = false;
  }
}

const sensorNoiseModel = new SensorNoiseModel();
export default sensorNoiseModel;
