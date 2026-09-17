// Single source of truth shared with the vehicle backend code (collision_safety.py)
import SAFETY_PARAMS from '../../../backend/sauvc26_code/safety_params.json' with { type: 'json' };

export { SAFETY_PARAMS };

export const SAFETY_STATE = Object.freeze({
  NORMAL: 'NORMAL',
  CAUTION: 'CAUTION',
  LOCKED: 'LOCKED',
  EMERGENCY_SURFACE: 'EMERGENCY_SURFACE',
});

export class SafetySupervisor {
  constructor() {
    this.blocked = { front: false, rear: false, left: false, right: false, floor: false };
  }

  reset() {
    this.blocked = { front: false, rear: false, left: false, right: false, floor: false };
  }

  updateLatch(direction, range, stopDistance, releaseDistance) {
    if (!Number.isFinite(range)) {
      this.blocked[direction] = true;
    } else if (range <= stopDistance) {
      this.blocked[direction] = true;
    } else if (range >= releaseDistance) {
      this.blocked[direction] = false;
    }
  }

  evaluate({ sonarRanges = {}, floorAltitude, leakDetected, battery = {}, imuDetection = {}, oodStatus = {} }) {
    const P = SAFETY_PARAMS;
    this.updateLatch('front', sonarRanges.front, P.stop_distance, P.slow_distance);
    this.updateLatch('rear', sonarRanges.rear, P.stop_distance, P.slow_distance);
    this.updateLatch('left', sonarRanges.left, P.stop_distance, P.slow_distance);
    this.updateLatch('right', sonarRanges.right, P.stop_distance, P.slow_distance);
    this.updateLatch('floor', floorAltitude, P.floor_stop_distance, P.floor_slow_distance);

    const emergencyReasons = [];
    if (leakDetected) emergencyReasons.push('HULL_LEAK');
    if (imuDetection.impactDetected) emergencyReasons.push('IMU_IMPACT');
    if ((battery.level ?? 100) <= P.battery_fraction_critical * 100 || (battery.voltage ?? 16) <= P.battery_voltage_critical) emergencyReasons.push('BATTERY_CRITICAL');

    let state = SAFETY_STATE.NORMAL;
    const reasons = [];
    if (emergencyReasons.length > 0) {
      state = SAFETY_STATE.EMERGENCY_SURFACE;
      reasons.push(...emergencyReasons);
    } else if (Object.values(this.blocked).some(Boolean) || oodStatus.isOOD) {
      state = SAFETY_STATE.LOCKED;
      reasons.push(...Object.entries(this.blocked).filter(([, active]) => active).map(([direction]) => `${direction.toUpperCase()}_LOCK`));
      if (oodStatus.isOOD) reasons.push('OOD_LOCK');
    } else {
      const nearestWall = Math.min(...Object.values(sonarRanges).filter(Number.isFinite));
      if (nearestWall < SAFETY_PARAMS.slow_distance || floorAltitude < SAFETY_PARAMS.floor_slow_distance || (battery.level ?? 100) < 20) {
        state = SAFETY_STATE.CAUTION;
        reasons.push('REDUCED_SPEED');
      }
    }

    return { state, reasons, blocked: { ...this.blocked }, emergency: state === SAFETY_STATE.EMERGENCY_SURFACE };
  }

  applyCommand(command, status) {
    const P = SAFETY_PARAMS;
    if (status.state === SAFETY_STATE.EMERGENCY_SURFACE) {
      // Twin heave convention: negative = up (jetson z-up uses positive = up)
      return { surge: 0, sway: 0, yaw: 0, heave: -P.emergency_surface_speed };
    }

    const safe = { ...command };
    if (status.blocked.front && status.blocked.rear) safe.surge = 0;
    else if (status.blocked.front) safe.surge = Math.min(safe.surge, -P.backoff_speed);
    else if (status.blocked.rear) safe.surge = Math.max(safe.surge, P.backoff_speed);

    if (status.blocked.left && status.blocked.right) safe.sway = 0;
    else if (status.blocked.left) safe.sway = Math.max(safe.sway, P.backoff_speed);
    else if (status.blocked.right) safe.sway = Math.min(safe.sway, -P.backoff_speed);

    if (status.blocked.floor) safe.heave = Math.min(safe.heave, -P.backoff_speed);

    if (status.state === SAFETY_STATE.CAUTION) {
      safe.surge *= 0.5;
      safe.sway *= 0.5;
      safe.yaw *= 0.5;
      safe.heave *= 0.5;
    }
    return safe;
  }
}

export default new SafetySupervisor();