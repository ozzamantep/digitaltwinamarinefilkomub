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
    this.updateLatch('front', sonarRanges.front, 0.55, 1.2);
    this.updateLatch('rear', sonarRanges.rear, 0.55, 1.2);
    this.updateLatch('left', sonarRanges.left, 0.55, 1.2);
    this.updateLatch('right', sonarRanges.right, 0.55, 1.2);
    this.updateLatch('floor', floorAltitude, 0.37, 0.7);

    const emergencyReasons = [];
    if (leakDetected) emergencyReasons.push('HULL_LEAK');
    if (imuDetection.impactDetected) emergencyReasons.push('IMU_IMPACT');
    if ((battery.level ?? 100) <= 8 || (battery.voltage ?? 16) <= 13.0) emergencyReasons.push('BATTERY_CRITICAL');

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
      if (nearestWall < 1.2 || floorAltitude < 0.7 || (battery.level ?? 100) < 20) {
        state = SAFETY_STATE.CAUTION;
        reasons.push('REDUCED_SPEED');
      }
    }

    return { state, reasons, blocked: { ...this.blocked }, emergency: state === SAFETY_STATE.EMERGENCY_SURFACE };
  }

  applyCommand(command, status) {
    if (status.state === SAFETY_STATE.EMERGENCY_SURFACE) {
      return { surge: 0, sway: 0, yaw: 0, heave: -0.65 };
    }

    const safe = { ...command };
    if (status.blocked.front && status.blocked.rear) safe.surge = 0;
    else if (status.blocked.front) safe.surge = Math.min(safe.surge, -0.30);
    else if (status.blocked.rear) safe.surge = Math.max(safe.surge, 0.30);

    if (status.blocked.left && status.blocked.right) safe.sway = 0;
    else if (status.blocked.left) safe.sway = Math.max(safe.sway, 0.30);
    else if (status.blocked.right) safe.sway = Math.min(safe.sway, -0.30);

    if (status.blocked.floor) safe.heave = Math.min(safe.heave, -0.25);

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