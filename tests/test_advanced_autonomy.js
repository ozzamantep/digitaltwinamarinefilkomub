import assert from 'node:assert/strict';
import { AdvancedAutonomyEngine } from '../frontend/src/services/AdvancedAutonomyEngine.js';

const engine = new AdvancedAutonomyEngine();
const descending = engine.evaluate({ floorAltitude: 0.60, depthRate: 0.50 });
assert.equal(descending.floorBrake, true, 'Descending vehicle must brake before the floor lock boundary');
assert.ok(descending.brakingAltitude > 0.60, 'Braking envelope must include stopping distance');
assert.ok(engine.apply({ surge: 0, sway: 0, yaw: 0, heave: 1 }, descending).heave < 0, 'Floor brake must command upward heave');

let assessment;
for (let index = 0; index < 19; index++) {
  assessment = engine.evaluate({
    floorAltitude: 1.2,
    estimated: { residuals: { depth: 0.5, dvl: [0.5, 0, 0] }, velocity: { u: 0, v: 0, w: 0 } },
    dvlVelocity: [0.2, 0, 0],
    thrusters: [60, 0, 0, 0, 0, 0],
    measuredCurrents: [1, 0.5, 0.5, 0.5, 0.5, 0.5],
  });
}
assert.deepEqual(assessment.thrusterFaults, [], 'Normal startup current lag must not isolate a healthy thruster');
assessment = engine.evaluate({
  floorAltitude: 1.2,
  estimated: { residuals: { depth: 0.5, dvl: [0.5, 0, 0] }, velocity: { u: 0, v: 0, w: 0 } },
  dvlVelocity: [0.2, 0, 0],
  thrusters: [60, 0, 0, 0, 0, 0],
  measuredCurrents: [1, 0.5, 0.5, 0.5, 0.5, 0.5],
});
assessment = engine.evaluate({
  floorAltitude: 1.2,
  estimated: { residuals: { depth: 0.5, dvl: [0.5, 0, 0] }, velocity: { u: 0, v: 0, w: 0 } },
  dvlVelocity: [0.2, 0, 0],
  thrusters: [60, 0, 0, 0, 0, 0],
  measuredCurrents: [1, 0.5, 0.5, 0.5, 0.5, 0.5],
});
assert.equal(assessment.depthSensorFault, true, 'Persistent depth residual must isolate depth sensor');
assert.equal(assessment.dvlFault, true, 'Persistent DVL residual must activate dead reckoning');
assert.deepEqual(assessment.thrusterFaults, [0], 'Persistent under-current must isolate failed thruster');
assert.ok(engine.apply({ surge: 1, sway: 1, yaw: 0, heave: 0 }, assessment).surge <= 0.35, 'Fault recovery must restrict speed');
assert.equal(engine.apply({ surge: 0, sway: 0.2, yaw: 0, heave: 0 }, assessment).sway, 0.2, 'Unvalidated current estimate must not steer the vehicle');

console.log('PASS: predictive braking, FDIR, and current estimation');