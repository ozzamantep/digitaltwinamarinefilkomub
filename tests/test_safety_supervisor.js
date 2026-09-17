import assert from 'node:assert/strict';
import { SafetySupervisor, SAFETY_STATE } from '../frontend/src/services/SafetySupervisor.js';

const supervisor = new SafetySupervisor();
const nominal = {
  sonarRanges: { front: 3, rear: 3, left: 3, right: 3 },
  floorAltitude: 1,
  battery: { level: 80, voltage: 16 },
};

let status = supervisor.evaluate(nominal);
assert.equal(status.state, SAFETY_STATE.NORMAL);

status = supervisor.evaluate({ ...nominal, sonarRanges: { ...nominal.sonarRanges, front: 0.4 } });
assert.equal(status.state, SAFETY_STATE.LOCKED);
assert.equal(supervisor.applyCommand({ surge: 1, sway: 0, yaw: 0, heave: 0 }, status).surge, -0.2);
assert.equal(supervisor.applyCommand({ surge: -1, sway: 0, yaw: 0, heave: 0 }, status).surge, -1);

status = supervisor.evaluate({ ...nominal, sonarRanges: { ...nominal.sonarRanges, front: 0.8 } });
assert.equal(status.blocked.front, true, 'Front lock released without hysteresis clearance');
status = supervisor.evaluate(nominal);
assert.equal(status.blocked.front, false);

const directionalCases = [
  { direction: 'rear', command: { surge: -1, sway: 0, yaw: 0, heave: 0 }, axis: 'surge', expected: 0.2 },
  { direction: 'left', command: { surge: 0, sway: -1, yaw: 0, heave: 0 }, axis: 'sway', expected: 0.2 },
  { direction: 'right', command: { surge: 0, sway: 1, yaw: 0, heave: 0 }, axis: 'sway', expected: -0.2 },
];
for (const testCase of directionalCases) {
  status = supervisor.evaluate({
    ...nominal,
    sonarRanges: { ...nominal.sonarRanges, [testCase.direction]: 0.4 },
  });
  assert.equal(supervisor.applyCommand(testCase.command, status)[testCase.axis], testCase.expected);
  status = supervisor.evaluate(nominal);
}

status = supervisor.evaluate({ ...nominal, floorAltitude: 0.3 });
assert.equal(supervisor.applyCommand({ surge: 0, sway: 0, yaw: 0, heave: 1 }, status).heave, -0.2);
assert.equal(supervisor.applyCommand({ surge: 0, sway: 0, yaw: 0, heave: -1 }, status).heave, -1);

status = supervisor.evaluate({ ...nominal, floorAltitude: 0.36 });
assert.equal(status.blocked.floor, true, 'Floor lock missed the sensor-noise buffer');

status = supervisor.evaluate({ ...nominal, leakDetected: true });
assert.equal(status.state, SAFETY_STATE.EMERGENCY_SURFACE);
assert.deepEqual(
  supervisor.applyCommand({ surge: 1, sway: 1, yaw: 1, heave: 1 }, status),
  { surge: 0, sway: 0, yaw: 0, heave: -0.35 }
);

console.log('PASS: directional safety locks and emergency surface override');