const distance = (from, to) => Math.hypot(to.x - from.x, to.z - from.z);

/** Select the left or right flare bypass with the shortest safe continuation. */
export function planFlareAvoidance({ position, flare, nextTarget, standoff = 1.6, clearance = 1.4, zLimit = 7.1 }) {
  const stop = { x: flare.x - standoff, z: flare.z };
  const candidates = [-1, 1].map((side) => {
    const clearZ = Math.max(-zLimit, Math.min(zLimit, flare.z + side * clearance));
    const clearancePoint = { x: stop.x, z: clearZ };
    const pass = { x: flare.x + 1.5, z: clearZ };
    const cost = distance(position, stop) + distance(stop, clearancePoint) + distance(clearancePoint, pass) + distance(pass, nextTarget);
    return { side, clearZ, approach: stop, clearancePoint, pass, cost };
  });
  return candidates.reduce((best, candidate) => candidate.cost < best.cost ? candidate : best);
}