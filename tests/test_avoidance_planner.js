import assert from 'node:assert/strict';
import { planFlareAvoidance } from '../frontend/src/services/AvoidancePlanner.js';

const flare = { x: 0, z: 0 };
const fromLeft = { x: -4, z: 0 };
const towardPositiveZ = planFlareAvoidance({ position: fromLeft, flare, nextTarget: { x: 4, z: 5 } });
const towardNegativeZ = planFlareAvoidance({ position: fromLeft, flare, nextTarget: { x: 4, z: -5 } });

assert.ok(towardPositiveZ.clearZ > 0, 'Planner must choose the positive-Z side for a positive-Z continuation target');
assert.ok(towardNegativeZ.clearZ < 0, 'Planner must choose the negative-Z side for a negative-Z continuation target');
assert.deepEqual(towardPositiveZ.approach, { x: -1.6, z: 0 }, 'Planner must create a forward standoff before lateral clearance');
console.log('PASS: flexible avoidance selects the shortest continuation side');