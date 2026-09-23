import assert from 'node:assert/strict';
import { computeFastestOrder } from '../frontend/src/services/RouteOptimizer.js';

// Vehicle starts at origin; B is closest, then A, then C is farthest -
// a naive fixed order would visit A, B, C (declaration order) which is longer.
const start = { x: 0, z: 0 };
const targets = [
  { key: 'A', x: 5, z: 0 },
  { key: 'B', x: 1, z: 0 },
  { key: 'C', x: 10, z: 0 },
];

const order = computeFastestOrder(start, targets);
assert.deepEqual(order, ['B', 'A', 'C'], 'Must visit collinear targets nearest-first for the shortest total path');

// A MENGHINDAR flare directly on the way should still be visited before a farther
// TABRAK target, but the detour penalty should not flip an obviously shorter route.
const withStrategy = [
  { key: 'near_menghindar', x: 1, z: 0, strategy: 'MENGHINDAR' },
  { key: 'far_tabrak', x: 5, z: 0, strategy: 'TABRAK' },
];
const strategyOrder = computeFastestOrder(start, withStrategy);
assert.deepEqual(strategyOrder, ['near_menghindar', 'far_tabrak'], 'Nearest target first even when it carries a detour penalty');

// Single target and empty list edge cases
assert.deepEqual(computeFastestOrder(start, []), []);
assert.deepEqual(computeFastestOrder(start, [{ key: 'only', x: 3, z: 3 }]), ['only']);

console.log('PASS: computeFastestOrder finds the shortest total-distance visiting order');
