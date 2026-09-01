/**
 * Subsea 3D Physical Collision Engine for Official SAUVC 2026 Arena
 */

const POOL_OBSTACLES = [
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

  // C. Target Drums (X = 10.5m)
  {
    id: 'drum_blue',
    type: 'cylinder',
    x: 10.5,
    z: 4.5,
    radius: 0.42,
    minY: 0.0,
    maxY: 0.48,
  },
  {
    id: 'drum_red_1',
    type: 'cylinder',
    x: 10.5,
    z: 1.5,
    radius: 0.42,
    minY: 0.0,
    maxY: 0.48,
  },
  {
    id: 'drum_red_2',
    type: 'cylinder',
    x: 10.5,
    z: -1.5,
    radius: 0.42,
    minY: 0.0,
    maxY: 0.48,
  },
  {
    id: 'drum_red_3',
    type: 'cylinder',
    x: 10.5,
    z: -4.5,
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

  resolveCollision(posX, posZ, depth, velX = 0, velZ = 0) {
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

    // 3. OBSTACLES RESOLUTION
    for (const obs of POOL_OBSTACLES) {
      if (subY + this.subHeight * 0.5 < obs.minY || subY - this.subHeight * 0.5 > obs.maxY) {
        continue;
      }

      if (obs.type === 'cylinder') {
        const dx = correctedX - obs.x;
        const dz = correctedZ - obs.z;
        const dist = Math.sqrt(dx * dx + dz * dz);
        const minDist = obs.radius + this.subRadius;

        if (dist < minDist && dist > 0.0001) {
          const overlap = minDist - dist;
          const nx = dx / dist;
          const nz = dz / dist;

          correctedX += nx * overlap;
          correctedZ += nz * overlap;

          isCollided = true;
          hitObstacle = obs.id;
          recoilX = nx * 0.25;
          recoilZ = nz * 0.25;
        }
      } else if (obs.type === 'box') {
        const inX = correctedX + this.subRadius > obs.minX && correctedX - this.subRadius < obs.maxX;
        const inZ = correctedZ + this.subRadius > obs.minZ && correctedZ - this.subRadius < obs.maxZ;
        const inY = subY + this.subHeight * 0.5 > obs.minY && subY - this.subHeight * 0.5 < obs.maxY;

        if (inX && inZ && inY) {
          // Push horizontally away from the crossbar
          const midX = (obs.minX + obs.maxX) / 2;
          const pushDir = correctedX < midX ? -1 : 1;
          correctedX += pushDir * 0.05;
          isCollided = true;
          hitObstacle = obs.id;
          recoilX = pushDir * 0.2;
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
