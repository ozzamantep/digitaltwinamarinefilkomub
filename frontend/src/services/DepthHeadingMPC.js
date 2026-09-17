import Kinematics from '../dt-core/Kinematics.js';

const clamp = (value, min, max) => Math.max(min, Math.min(max, value));

export class DepthHeadingMPC {
  constructor(options = {}) {
    this.horizon = options.horizon || 12;
    this.dt = options.dt || 0.05;
    this.heaveActions = [-0.80, -0.45, 0, 0.45, 0.80];
    this.yawActions = [-0.60, -0.30, 0, 0.30, 0.60];
  }

  solve({ depth, depthRate = 0, heading = 0, yawRate = 0, targetDepth, targetHeading, floorDepth = 1.65 }) {
    targetDepth ??= depth;
    targetHeading ??= heading;
    let best = { heave: 0, yaw: 0, cost: Infinity };
    for (const heave of this.heaveActions) {
      for (const yaw of this.yawActions) {
        let predictedDepth = depth;
        let predictedDepthRate = depthRate;
        let predictedHeading = heading;
        let predictedYawRate = yawRate;
        let cost = 0;
        for (let step = 0; step < this.horizon; step++) {
          predictedDepthRate = clamp(predictedDepthRate + (heave * 1.4 - predictedDepthRate * 0.8) * this.dt, -0.8, 0.8);
          predictedDepth += predictedDepthRate * this.dt;
          predictedYawRate = clamp(predictedYawRate + (yaw * 1.6 - predictedYawRate) * this.dt, -0.8, 0.8);
          predictedHeading = Kinematics.wrapAngle(predictedHeading + predictedYawRate * this.dt);
          const depthError = predictedDepth - targetDepth;
          const headingError = Kinematics.wrapAngle(predictedHeading - targetHeading);
          const floorPenalty = predictedDepth > floorDepth ? 10000 * (predictedDepth - floorDepth + 1) : 0;
          const surfacePenalty = predictedDepth < 0.10 ? 10000 * (0.10 - predictedDepth + 1) : 0;
          cost += depthError ** 2 * 12 + headingError ** 2 * 4 + heave ** 2 * 0.15 + yaw ** 2 * 0.08 + floorPenalty + surfacePenalty;
        }
        if (cost < best.cost) best = { heave, yaw, cost };
      }
    }
    return best;
  }
}

const depthHeadingMPC = new DepthHeadingMPC();
export default depthHeadingMPC;