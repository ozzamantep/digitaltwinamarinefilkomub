const distance = (a, b) => Math.hypot(b.x - a.x, b.z - a.z);

// Extra distance charged to a MENGHINDAR flare during ordering only - it must be bypassed
// (approach -> lateral clearance -> pass) instead of a straight line, so a pure straight-line
// TSP would otherwise underestimate its true cost and misorder it. The exact detour geometry
// is still computed live by planFlareAvoidance() at mission time; this is only a planning bias.
const MENGHINDAR_DETOUR_PENALTY = 1.4;

function permutations(arr) {
  if (arr.length <= 1) return [arr];
  const result = [];
  for (let i = 0; i < arr.length; i++) {
    const rest = [...arr.slice(0, i), ...arr.slice(i + 1)];
    for (const perm of permutations(rest)) {
      result.push([arr[i], ...perm]);
    }
  }
  return result;
}

function targetCost(target) {
  return target.strategy === 'MENGHINDAR' ? MENGHINDAR_DETOUR_PENALTY : 0;
}

function routeCost(start, order) {
  let cost = 0;
  let from = start;
  for (const target of order) {
    cost += distance(from, target) + targetCost(target);
    from = target;
  }
  return cost;
}

function nearestNeighborOrder(start, targets) {
  const remaining = [...targets];
  const order = [];
  let from = start;
  while (remaining.length) {
    let bestIdx = 0;
    let bestCost = Infinity;
    remaining.forEach((t, i) => {
      const cost = distance(from, t) + targetCost(t);
      if (cost < bestCost) {
        bestCost = cost;
        bestIdx = i;
      }
    });
    const [chosen] = remaining.splice(bestIdx, 1);
    order.push(chosen);
    from = chosen;
  }
  return order;
}

/**
 * Fastest-route ordering of mission targets from the vehicle's current position.
 * Brute-force Traveling-Salesman search (exact optimum) - mission sizes here are
 * small (<=6 waypoints: gate + 4 flares + drum), so exhaustive search over every
 * permutation (<=720) is instant. Falls back to a greedy nearest-neighbor heuristic
 * if an unexpectedly large target list is ever passed in.
 *
 * @param {{x:number, z:number}} start - current vehicle position
 * @param {Array<{key:string, x:number, z:number, strategy?:string}>} targets
 * @returns {string[]} target keys in the fastest order found
 */
export function computeFastestOrder(start, targets) {
  if (!targets || targets.length === 0) return [];
  if (targets.length === 1) return [targets[0].key];

  if (targets.length > 8) {
    return nearestNeighborOrder(start, targets).map((t) => t.key);
  }

  let best = null;
  let bestCost = Infinity;
  for (const perm of permutations(targets)) {
    const cost = routeCost(start, perm);
    if (cost < bestCost) {
      bestCost = cost;
      best = perm;
    }
  }
  return best.map((t) => t.key);
}
