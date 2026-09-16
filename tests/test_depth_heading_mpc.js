import assert from 'node:assert/strict';
import { DepthHeadingMPC } from '../src/services/DepthHeadingMPC.js';

const mpc = new DepthHeadingMPC();
const floorApproach = mpc.solve({ depth: 1.55, depthRate: 0.45, heading: 0, yawRate: 0, targetDepth: 0.85, targetHeading: 0 });
assert.ok(floorApproach.heave < 0, 'MPC must command ascent while approaching the floor');
const headingCorrection = mpc.solve({ depth: 0.85, heading: 0.6, yawRate: 0, targetDepth: 0.85, targetHeading: 0 });
assert.ok(headingCorrection.yaw < 0, 'MPC must turn toward the target heading');
console.log('PASS: constrained depth-heading MPC selects safe corrective control');