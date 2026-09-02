import useVehicleStore from '../store/vehicleStore';

/**
 * Subsea 3D Physical Collision Engine for Official SAUVC 2026 Arena
 */

const DEFAULT_POOL_OBSTACLES = [
  // A. Orange Flare (Approached closely, solid obstacle)
  {
    id: 'flare_orange',
    type: 'cylinder',
    x: -6.0,
    z: 2.0,
    radius: 0.24,
    minY: 0.0,
    maxY: 1.65,
  },

  // B. Gate (X = 4.0m) with Red & Green Markers
  {
    id: 'gate_left_post',
    type: 'cylinder',
    x: 4.0,
    z: -0.9,
    radius: 0.12,
    minY: 0.0,
    maxY: 1.55,
  },
  {
    id: 'gate_right_post',
    type: 'cylinder',
    x: 4.0,
    z: 0.9,
    radius: 0.12,
    minY: 0.0,
    maxY: 1.55,
  },
  {
    id: 'gate_top_crossbar',
    type: 'box',
    minX: 3.90,
    maxX: 4.10,
    minZ: -0.9,
    maxZ: 0.9,
    minY: 1.48,
    maxY: 1.55,
  },

  // C. Target Drums
  {
    id: 'drum_red_1',
    type: 'cylinder',
    x: 10.5,
    z: 1.5,
    radius: 0.42,
    minY: 0.0,
    maxY: 0.48,
  },
];

class SubseaCollisionEngine {
  constructor() {
    this.subRadius = 0.24;
    this.subHeight = 0.26;
    this.lastCollision = null;
  }

  getDynamicObstacles() {
    const store = useVehicleStore.getState ? useVehicleStore.getState() : null;
    const obs = store?.obstacles;
    const flaresFallen = store?.flaresFallen || {};
    const flareStrategies = store?.flareStrategies || {};
    if (!obs) return DEFAULT_POOL_OBSTACLES;

    const orangeX = obs.orange_flare?.x ?? -6.0;
    const orangeZ = obs.orange_flare?.z ?? 2.0;
    const gateX = obs.gate?.x ?? 4.0;
    const gateZ = obs.gate?.z ?? 0.0;
    const drumX = obs.drum_red_tgt?.x ?? 10.5;
    const drumZ = obs.drum_red_tgt?.z ?? 1.5;

    const list = [
      {
        id: 'gate_left_post',
        type: 'cylinder',
        x: gateX,
        z: gateZ - 0.9,
        radius: 0.12,
        minY: 0.0,
        maxY: 1.55,
      },
      {
        id: 'gate_right_post',
        type: 'cylinder',
        x: gateX,
        z: gateZ + 0.9,
        radius: 0.12,
        minY: 0.0,
        maxY: 1.55,
      },
      {
        id: 'gate_top_crossbar',
        type: 'box',
        minX: gateX - 0.10,
        maxX: gateX + 0.10,
        minZ: gateZ - 0.9,
        maxZ: gateZ + 0.9,
        minY: 1.48,
        maxY: 1.55,
      },
      {
        id: 'drum_red_1',
        type: 'cylinder',
        x: drumX,
        z: drumZ,
        radius: 0.42,
        minY: 0.0,
        maxY: 0.48,
      },
    ];

    const flareKeys = [
      { key: 'orange_flare', color: 'orange', x: orangeX, z: orangeZ },
      { key: 'blue_flare',   color: 'blue',   x: obs.blue_flare?.x ?? -2.0,   z: obs.blue_flare?.z ?? 2.2 },
      { key: 'red_flare',    color: 'red',    x: obs.red_flare?.x ?? 0.5,    z: obs.red_flare?.z ?? 4.0 },
      { key: 'yellow_flare', color: 'yellow', x: obs.yellow_flare?.x ?? -0.5, z: obs.yellow_flare?.z ?? -4.5 },
    ];

    for (const fl of flareKeys) {
      if (!flaresFallen[fl.color] && flareStrategies[fl.key] === 'MENGHINDAR') {
        list.push({
          id: `flare_${fl.color}`,
          type: 'cylinder',
          x: fl.x,
          z: fl.z,
          radius: 0.24,
          minY: 0.0,
          maxY: 1.65,
        });
      }
    }

    return list;
  }

  resolveCollision(posX, posZ, depth, velX = 0, velZ = 0) {
    const store = useVehicleStore.getState ? useVehicleStore.getState() : null;
    const obstacleDict = store?.obstacles;
    const flaresFallen = store?.flaresFallen || {};
    const flareStrategies = store?.flareStrategies || {};

    let correctedX = posX;
    let correctedZ = posZ;
    let correctedDepth = depth;
    let isCollided = false;
    let hitObstacle = null;
    let recoilX = 0;
    let recoilZ = 0;

    // 1. POOL WALL BOUNDARIES (25m x 16m)
    const wallMarginX = 12.0 - this.subRadius;
    const wallMarginZ = 7.5 - this.subRadius;

    if (correctedX > wallMarginX) {
      correctedX = wallMarginX;
      isCollided = true;
      hitObstacle = 'East Pool Wall';
      recoilX = -0.3;
    } else if (correctedX < -wallMarginX) {
      correctedX = -wallMarginX;
      isCollided = true;
      hitObstacle = 'West Pool Wall';
      recoilX = 0.3;
    }

    if (correctedZ > wallMarginZ) {
      correctedZ = wallMarginZ;
      isCollided = true;
      hitObstacle = 'North Pool Wall';
      recoilZ = -0.3;
    } else if (correctedZ < -wallMarginZ) {
      correctedZ = -wallMarginZ;
      isCollided = true;
      hitObstacle = 'South Pool Wall';
      recoilZ = 0.3;
    }

    // 2. FLOOR & SURFACE
    if (correctedDepth > 1.84) {
      correctedDepth = 1.84;
      isCollided = true;
      hitObstacle = 'Pool Floor Tiles';
    } else if (correctedDepth < 0.1) {
      correctedDepth = 0.1;
    }

    const subY = 2.0 - correctedDepth;

    // 3. DYNAMIC OBSTACLES RESOLUTION
    const activeObstacles = this.getDynamicObstacles();
    for (const obstacleItem of activeObstacles) {
      if (subY + this.subHeight * 0.5 < obstacleItem.minY || subY - this.subHeight * 0.5 > obstacleItem.maxY) {
        continue;
      }

      if (obstacleItem.type === 'cylinder') {
        const dx = correctedX - obstacleItem.x;
        const dz = correctedZ - obstacleItem.z;
        const dist = Math.sqrt(dx * dx + dz * dz);
        const minDist = obstacleItem.radius + this.subRadius;

        if (dist < minDist && dist > 0.0001) {
          const overlap = minDist - dist;
          const nx = dx / dist;
          const nz = dz / dist;

          correctedX += nx * overlap;
          correctedZ += nz * overlap;

          isCollided = true;
          hitObstacle = obstacleItem.id;
          if (obstacleItem.id.startsWith('gate_')) {
            recoilX = 0; // Don't block forward gate passage
            recoilZ = nz * 0.35; // Nudge laterally away from post
          } else {
            recoilX = nx * 0.25;
            recoilZ = nz * 0.25;
          }
        }
      } else if (obstacleItem.type === 'box') {
        const inX = correctedX + this.subRadius > obstacleItem.minX && correctedX - this.subRadius < obstacleItem.maxX;
        const inZ = correctedZ + this.subRadius > obstacleItem.minZ && correctedZ - this.subRadius < obstacleItem.maxZ;
        const inY = subY + this.subHeight * 0.5 > obstacleItem.minY && subY - this.subHeight * 0.5 < obstacleItem.maxY;

        if (inX && inZ && inY) {
          // Push horizontally away from the crossbar
          const midX = (obstacleItem.minX + obstacleItem.maxX) / 2;
          const pushDir = correctedX < midX ? -1 : 1;
          correctedX += pushDir * 0.05;
          isCollided = true;
          hitObstacle = obstacleItem.id;
          recoilX = pushDir * 0.2;
        }
      }
    }

    // 4. Physical Flare Contact & Knockdown Check (ONLY for flares marked TABRAK)
    if (obstacleDict) {
      const flareColors = ['orange', 'blue', 'red', 'yellow'];
      for (const color of flareColors) {
        const flareKey = `${color}_flare`;
        const isTabrak = flareStrategies[flareKey] === 'TABRAK';
        const fObs = obstacleDict[flareKey];
        if (fObs && !flaresFallen[color] && isTabrak) {
          const dX = correctedX - fObs.x;
          const dZ = correctedZ - fObs.z;
          const dDist = Math.sqrt(dX * dX + dZ * dZ);
          if (dDist < (0.28 + this.subRadius) && subY > 0.05 && subY < 1.65) {
            store?.knockdownFlare(color);
          }
        }
      }
    }

    return {
      x: correctedX,
      z: correctedZ,
      depth: correctedDepth,
      collided: isCollided,
      obstacle: hitObstacle,
      recoilX,
      recoilZ,
    };
  }
}

const subseaCollisionEngine = new SubseaCollisionEngine();
export default subseaCollisionEngine;
