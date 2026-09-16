const clamp = (value, min, max) => Math.max(min, Math.min(max, value));

/**
 * Safety-critical autonomy decisions shared by manual and autonomous modes.
 * Positive heave/depth rate is down in the NED convention.
 */
export class AdvancedAutonomyEngine {
  constructor() {
    this.currentEstimate = { x: 0, y: 0, z: 0 };
    this.faultCounters = { depth: 0, dvl: 0 };
    this.thrusterFaultCounters = new Array(6).fill(0);
    this.thrusterCommandTime = new Array(6).fill(0);
    this.lastAssessment = null;
  }

  evaluate({ floorAltitude, depthRate = 0, estimated = {}, dvlVelocity = [], thrusters = [], measuredCurrents = [], dt = 0.05 }) {
    const downwardSpeed = Math.max(0, depthRate);
    const brakingDeceleration = 1.2;
    const reactionTime = 0.10;
    const stoppingDistance = downwardSpeed * reactionTime + (downwardSpeed ** 2) / (2 * brakingDeceleration);
    const brakingAltitude = clamp(0.37 + stoppingDistance + 0.12, 0.49, 1.10);
    const floorBrake = Number.isFinite(floorAltitude) && floorAltitude <= brakingAltitude && downwardSpeed > 0.015;

    const residuals = estimated.residuals || {};
    this.faultCounters.depth = Math.abs(residuals.depth || 0) > 0.30 ? this.faultCounters.depth + 1 : 0;
    this.faultCounters.dvl = (residuals.dvl || []).some((value) => Math.abs(value || 0) > 0.40)
      ? this.faultCounters.dvl + 1 : 0;

    const thrusterFaults = [];
    for (let index = 0; index < 6; index++) {
      const command = Math.abs(thrusters[index] || 0);
      const current = measuredCurrents[index];
      const hasMeasurement = Number.isFinite(current);
      const expectedCurrent = command * 0.25;
      this.thrusterCommandTime[index] = command > 35
        ? this.thrusterCommandTime[index] + dt
        : 0;
      // T200 thrust and current are first-order dynamics (tau ~= 0.35 s).
      // Do not isolate a healthy thruster while it is still spooling up.
      const underCurrent = hasMeasurement && this.thrusterCommandTime[index] >= 1.0 - dt && current < expectedCurrent * 0.45;
      this.thrusterFaultCounters[index] = underCurrent ? this.thrusterFaultCounters[index] + 1 : 0;
      if (this.thrusterFaultCounters[index] >= 3) thrusterFaults.push(index);
    }

    const depthSensorFault = this.faultCounters.depth >= 3;
    const dvlFault = this.faultCounters.dvl >= 3;
    const measuredVelocity = dvlVelocity.length >= 3 ? dvlVelocity : [0, 0, 0];
    const estimatedVelocity = estimated.velocity || {};
    const alpha = 0.08;
    this.currentEstimate.x += alpha * ((measuredVelocity[0] || 0) - (estimatedVelocity.u || 0) - this.currentEstimate.x);
    this.currentEstimate.y += alpha * ((measuredVelocity[1] || 0) - (estimatedVelocity.v || 0) - this.currentEstimate.y);
    this.currentEstimate.z += alpha * ((measuredVelocity[2] || 0) - (estimatedVelocity.w || 0) - this.currentEstimate.z);

    const events = [];
    if (floorBrake) events.push('PREDICTIVE_FLOOR_BRAKE');
    if (depthSensorFault) events.push('DEPTH_SENSOR_FALLBACK');
    if (dvlFault) events.push('DVL_DEAD_RECKONING');
    for (const index of thrusterFaults) events.push(`THRUSTER_${index + 1}_FAULT`);

    this.lastAssessment = {
      floorBrake,
      stoppingDistance,
      brakingAltitude,
      depthSensorFault,
      dvlFault,
      thrusterFaults,
      currentEstimate: { ...this.currentEstimate },
      events,
      autonomyRestricted: dvlFault || thrusterFaults.length > 0,
    };
    return this.lastAssessment;
  }

  apply(command, assessment = this.lastAssessment) {
    if (!assessment) return { ...command };
    const safe = { ...command };
    if (assessment.floorBrake) {
      safe.heave = -clamp(0.35 + assessment.stoppingDistance * 1.8, 0.35, 0.80);
    }
    if (assessment.autonomyRestricted) {
      safe.surge = clamp(safe.surge || 0, -0.35, 0.35);
      safe.sway = clamp(safe.sway || 0, -0.25, 0.25);
    }
    return safe;
  }

  reset() {
    this.currentEstimate = { x: 0, y: 0, z: 0 };
    this.faultCounters = { depth: 0, dvl: 0 };
    this.thrusterFaultCounters.fill(0);
    this.thrusterCommandTime.fill(0);
    this.lastAssessment = null;
  }
}

const advancedAutonomyEngine = new AdvancedAutonomyEngine();
export default advancedAutonomyEngine;