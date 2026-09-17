/**
 * Formal Digital Twin State Data Object (DTState)
 * 
 * Unifies all facets of the physical-virtual marine system into a single
 * coherent, versioned state snapshot:
 * - Physical Raw Sensor State
 * - EKF Fused Estimated State
 * - Forward-Simulated Predicted State
 * - Hydrodynamic Parameter Snapshot & Model Version
 * - Environment State
 * - Uncertainty Bounds
 * - Sync & Communication Health
 * - Model Residuals & Health Score
 */

import vehicleConfig from './VehicleConfig.js';

export class DTState {
  constructor(data = {}) {
    this.timestamp = data.timestamp || Date.now();
    this.modelVersion = data.modelVersion || vehicleConfig.version;

    // 1. Physical State (Raw telemetry from robot sensors)
    this.physicalState = data.physicalState || {
      position: { x: -11.0, y: 2.0, z: 0.8 },
      depth: 0.8,
      headingDeg: 0,
      imu: { accel: [0, 0, -9.81], gyro: [0, 0, 0] },
      thrusters: [0, 0, 0, 0, 0, 0],
      battery: { voltage: 16.0, current: 4.0, percentage: 90 },
    };

    // 2. Estimated State (From 15-state EKF)
    this.estimatedState = data.estimatedState || {
      position: { x: -11.0, y: 2.0, z: 0.8 },
      velocity: { u: 0, v: 0, w: 0 },
      attitude: { roll: 0, pitch: 0, yaw: 0 },
      biases: { accel: [0, 0, 0], gyro: [0, 0, 0] },
    };

    // 3. Predicted State (From N-step forward prediction)
    this.predictedState = data.predictedState || null;

    // 4. Uncertainty & Health Metrics
    this.uncertainty = data.uncertainty ?? 0.05; // 0.0 (high confidence) to 1.0 (unsafe)
    this.confidence = data.confidence ?? 0.95;   // 1.0 - uncertainty
    this.oodScore = data.oodScore ?? 0.0;       // Out-of-Distribution distance

    // 5. Environmental Snapshot
    this.environment = data.environment || {
      waterDensity: 997,
      currentSpeed: 0.05,
      pressureBar: 1.09,
    };

    // 6. Synchronization & Pipeline Metrics
    this.sync = data.sync || {
      status: 'SYNCHRONIZED',
      totalLatencyMs: 15.0,
      packetRateHz: 20.0,
    };

    // 7. Physics Residuals
    this.residuals = data.residuals || {
      depthError: 0.0,
      velocityError: 0.0,
      headingError: 0.0,
    };
  }

  /**
   * Export comprehensive state snapshot for dashboard & logging
   */
  toJSON() {
    return {
      timestamp: this.timestamp,
      modelVersion: this.modelVersion,
      physicalState: this.physicalState,
      estimatedState: this.estimatedState,
      predictedState: this.predictedState,
      uncertainty: this.uncertainty,
      confidence: this.confidence,
      oodScore: this.oodScore,
      environment: this.environment,
      sync: this.sync,
      residuals: this.residuals,
    };
  }
}

export default DTState;
