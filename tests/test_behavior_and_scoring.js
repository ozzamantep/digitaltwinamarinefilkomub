import assert from 'node:assert/strict';
import { BT_STATUS, createSafetyRecoveryTree } from '../frontend/src/services/BehaviorTree.js';
import { CompetitionScoringEngine } from '../frontend/src/dt-core/CompetitionScoringEngine.js';

const tree = createSafetyRecoveryTree();
const command = { surge: 0.8, sway: 0, yaw: 0, heave: 1 };
const status = tree.tick({ command, safety: { floorBrake: true }, autonomy: { apply: () => ({ heave: -0.6 }) } });
assert.equal(status, BT_STATUS.RUNNING);
assert.equal(command.heave, -0.6);
assert.equal(tree.tick({ command, safety: {}, autonomy: { apply: () => ({ heave: -0.6 }) } }), BT_STATUS.FAILURE);

const score = new CompetitionScoringEngine().scoreMission([
  { t: 1, events: ['GATE_PASSED', 'FLARE_RED_HIT'] },
  { t: 20, events: ['PAYLOAD_DELIVERED', 'PREDICTIVE_FLOOR_BRAKE'] },
]);
assert.equal(score.totalScore, 100 + 50 + 200 + 280 - 10);
console.log('PASS: behavior-tree recovery and replay scoring');