import useVehicleStore from '../store/vehicleStore.js';
import vehicleConfig from '../dt-core/VehicleConfig.js';

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
    this.halfLength = vehicleConfig.length * 0.5 + 0.02;
    this.halfWidth = vehicleConfig.width * 0.5 + 0.02;
    this.topOffset = vehicleConfig.height * 0.5 + 0.02;
    this.bottomOffset = 0.26;
    this.subRadius = Math.max(this.halfLength, this.halfWidth);
    this.subHeight = this.topOffset + this.bottomOffset;
    this.poolDepth = 2.0;
    this.minFloorClearance = 0.35;
    this.maxSafeDepth = this.poolDepth - this.minFloorClearance;
    this.lastCollision = null;
  }

  getHorizontalExtents(heading) {
    const cosHeading = Math.abs(Math.cos(heading));
    const sinHeading = Math.abs(Math.sin(heading));
    return {
      x: cosHeading * this.halfLength + sinHeading * this.halfWidth,
      z: sinHeading * this.halfLength + cosHeading * this.halfWidth,
    };
  }

  getCylinderContact(posX, posZ, heading, cylinderX, cylinderZ, radius) {
    const deltaX = cylinderX - posX;
    const deltaZ = cylinderZ - posZ;
    const cosHeading = Math.cos(heading);
    const sinHeading = Math.sin(heading);
    const localForward = deltaX * cosHeading + deltaZ * sinHeading;
    const localRight = -deltaX * sinHeading + deltaZ * cosHeading;
    const closestForward = Math.max(-this.halfLength, Math.min(this.halfLength, localForward));
    const closestRight = Math.max(-this.halfWidth, Math.min(this.halfWidth, localRight));
    const separationForward = closestForward - localForward;
    const separationRight = closestRight - localRight;
    const distance = Math.sqrt(separationForward ** 2 + separationRight ** 2);

    if (distance >= radius) return { collided: false, penetration: 0, nx: 0, nz: 0 };

    let normalForward;
    let normalRight;
    let penetration;
    if (distance > 1e-8) {
      normalForward = separationForward / distance;
      normalRight = separationRight / distance;
      penetration = radius - distance;
    } else {
      const forwardExit = this.halfLength - Math.abs(localForward);
      const rightExit = this.halfWidth - Math.abs(localRight);
      if (forwardExit <= rightExit) {
        normalForward = localForward >= 0 ? -1 : 1;
        normalRight = 0;
        penetration = radius + forwardExit;
      } else {
        normalForward = 0;
        normalRight = localRight >= 0 ? -1 : 1;
        penetration = radius + rightExit;
      }
    }

    return {
      collided: true,
      penetration,
      nx: normalForward * cosHeading - normalRight * sinHeading,
      nz: normalForward * sinHeading + normalRight * cosHeading,
    };
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

    return list;
  }

  resolveCollision(posX, posZ, depth, velX = 0, velZ = 0, heading = 0, previousX = posX, previousZ = posZ) {
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
    const extents = this.getHorizontalExtents(heading);
    const wallMarginX = 12.0 - extents.x;
    const wallMarginZ = 7.5 - extents.z;

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
    if (correctedDepth > this.maxSafeDepth) {
      correctedDepth = this.maxSafeDepth;
      isCollided = true;
      hitObstacle = 'Floor Safety Boundary';
    } else if (correctedDepth < 0.1) {
      correctedDepth = 0.1;
    }

    const subY = 2.0 - correctedDepth;

    // 3. DYNAMIC OBSTACLES RESOLUTION
    const activeObstacles = this.getDynamicObstacles();
    for (const obstacleItem of activeObstacles) {
      if (subY + this.topOffset < obstacleItem.minY || subY - this.bottomOffset > obstacleItem.maxY) {
        continue;
      }

      if (obstacleItem.type === 'cylinder') {
        const contact = this.getCylinderContact(
          correctedX,
          correctedZ,
          heading,
          obstacleItem.x,
          obstacleItem.z,
          obstacleItem.radius
        );

        if (contact.collided) {
          correctedX += contact.nx * contact.penetration;
          correctedZ += contact.nz * contact.penetration;

          isCollided = true;
          hitObstacle = obstacleItem.id;
          if (obstacleItem.id.startsWith('gate_')) {
            recoilX = 0; // Don't block forward gate passage
            recoilZ = contact.nz * 0.35; // Nudge laterally away from post
          } else {
            recoilX = contact.nx * 0.25;
            recoilZ = contact.nz * 0.25;
          }
        }
      } else if (obstacleItem.type === 'box') {
        const inX = correctedX + extents.x > obstacleItem.minX && correctedX - extents.x < obstacleItem.maxX;
        const inZ = correctedZ + extents.z > obstacleItem.minZ && correctedZ - extents.z < obstacleItem.maxZ;
        const inY = subY + this.topOffset > obstacleItem.minY && subY - this.bottomOffset < obstacleItem.maxY;

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
          const flareRadius = Math.max(0.12, (fObs.width || 0.35) * 0.5);
          const flareMinY = (fObs.y || 0) - (fObs.height || 1.5) * 0.5;
          const flareMaxY = (fObs.y || 0) + (fObs.height || 1.5) * 0.5;
          const verticalContact = subY + this.topOffset >= flareMinY && subY - this.bottomOffset <= flareMaxY;
          let hitFlare = false;
          // Sweep the oriented hull over this integration step. This prevents a
          // high-speed ram from tunneling through a narrow flare between ticks.
          for (let sample = 0; sample <= 4; sample++) {
            const fraction = sample / 4;
            const contact = this.getCylinderContact(
              previousX + (correctedX - previousX) * fraction,
              previousZ + (correctedZ - previousZ) * fraction,
              heading,
              fObs.x,
              fObs.z,
              flareRadius
            );
            if (contact.collided) {
              hitFlare = true;
              break;
            }
          }
          if (verticalContact && hitFlare) {
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
