import { useRef, useMemo } from 'react';
import { useFrame } from '@react-three/fiber';
import * as THREE from 'three';
import useVehicleStore from '../../store/vehicleStore';

/**
 * Per-Thruster Bubble Emission System
 * 
 * Each of the 6 T200 thrusters emits realistic bubble streams
 * only when that individual thruster is active. Bubble intensity
 * scales with thruster effort — more thrust = more bubbles & faster stream.
 * 
 * Thruster layout (from BlueROV2Model.jsx):
 *  0: Front-Left  Horizontal (+45°)  pos [+0.20, 0, +0.105]
 *  1: Front-Right Horizontal (-45°)  pos [+0.20, 0, -0.105]
 *  2: Rear-Left   Horizontal (+135°) pos [-0.20, 0, +0.105]
 *  3: Rear-Right  Horizontal (-135°) pos [-0.20, 0, -0.105]
 *  4: Front Vertical                 pos [+0.185, 0, 0]
 *  5: Rear  Vertical                 pos [-0.185, 0, 0]
 */

// Thruster local offsets relative to vehicle center & their exhaust direction vectors
const THRUSTER_CONFIG = [
  { // 0: Front-Left Horizontal (45°)
    offset: [0.20, 0, 0.105],
    direction: new THREE.Vector3(-0.707, 0, 0.707), // exhaust blows backward-outward
    isVertical: false,
  },
  { // 1: Front-Right Horizontal (-45°)
    offset: [0.20, 0, -0.105],
    direction: new THREE.Vector3(-0.707, 0, -0.707),
    isVertical: false,
  },
  { // 2: Rear-Left Horizontal (135°)
    offset: [-0.20, 0, 0.105],
    direction: new THREE.Vector3(0.707, 0, 0.707),
    isVertical: false,
  },
  { // 3: Rear-Right Horizontal (-135°)
    offset: [-0.20, 0, -0.105],
    direction: new THREE.Vector3(0.707, 0, -0.707),
    isVertical: false,
  },
  { // 4: Front Vertical
    offset: [0.185, 0, 0],
    direction: new THREE.Vector3(0, -1, 0), // exhaust blows downward (thrust pushes up)
    isVertical: true,
  },
  { // 5: Rear Vertical
    offset: [-0.185, 0, 0],
    direction: new THREE.Vector3(0, -1, 0),
    isVertical: true,
  },
];

const PARTICLES_PER_THRUSTER = 24;
const TOTAL_PARTICLES = PARTICLES_PER_THRUSTER * 6;

export default function ThrusterBubbles() {
  const pointsRef = useRef();
  const position = useVehicleStore((s) => s.position);
  const orientation = useVehicleStore((s) => s.orientation);
  const armed = useVehicleStore((s) => s.armed);
  const thrusters = useVehicleStore((s) => s.thrusters);

  const [posArr, velArr, lifeArr, sizeArr, thrusterIdx] = useMemo(() => {
    const pos = new Float32Array(TOTAL_PARTICLES * 3);
    const vel = new Array(TOTAL_PARTICLES);
    const life = new Float32Array(TOTAL_PARTICLES);
    const sizes = new Float32Array(TOTAL_PARTICLES);
    const tIdx = new Uint8Array(TOTAL_PARTICLES);

    for (let i = 0; i < TOTAL_PARTICLES; i++) {
      pos[i * 3] = 0;
      pos[i * 3 + 1] = -20; // hidden below world
      pos[i * 3 + 2] = 0;
      vel[i] = new THREE.Vector3();
      life[i] = Math.random() * 1.5; // stagger initial spawns
      sizes[i] = 0.02;
      tIdx[i] = Math.floor(i / PARTICLES_PER_THRUSTER);
    }
    return [pos, vel, life, sizes, tIdx];
  }, []);

  useFrame((state, delta) => {
    if (!pointsRef.current) return;
    const posAttr = pointsRef.current.geometry.attributes.position;
    const sizeAttr = pointsRef.current.geometry.attributes.size;
    const arr = posAttr.array;

    const quat = new THREE.Quaternion(
      orientation?.x || 0,
      orientation?.y || 0,
      orientation?.z || 0,
      orientation?.w || 1
    );

    const vPos = new THREE.Vector3(
      position?.x ?? -6,
      position?.y ?? 1.1,
      position?.z ?? 0
    );

    // Reusable vectors
    const localOffset = new THREE.Vector3();
    const exhaustDir = new THREE.Vector3();

    for (let i = 0; i < TOTAL_PARTICLES; i++) {
      const tIdx_i = thrusterIdx[i];
      const effort = Math.abs(thrusters[tIdx_i] || 0);
      const effortSign = Math.sign(thrusters[tIdx_i] || 0);
      const config = THRUSTER_CONFIG[tIdx_i];
      const isActive = armed && effort > 5;

      // Advance life
      // Faster decay for higher effort (more rapid cycling = denser stream)
      const decayRate = 1.2 + (effort / 100) * 2.0;
      lifeArr[i] += delta * decayRate;

      if (lifeArr[i] > 1.0) {
        // Respawn if thruster is active
        if (isActive && Math.random() < 0.3 + (effort / 100) * 0.5) {
          lifeArr[i] = 0;

          // Spawn at thruster nozzle position (in local vehicle space)
          localOffset.set(
            config.offset[0] + (Math.random() - 0.5) * 0.025,
            config.offset[1] + (Math.random() - 0.5) * 0.025,
            config.offset[2] + (Math.random() - 0.5) * 0.025
          );
          localOffset.applyQuaternion(quat);

          arr[i * 3]     = vPos.x + localOffset.x;
          arr[i * 3 + 1] = vPos.y + localOffset.y;
          arr[i * 3 + 2] = vPos.z + localOffset.z;

          // Exhaust direction (in world frame) — flipped for reverse thrust
          if (config.isVertical) {
            // Vertical thrusters: positive effort = upward thrust = exhaust downward
            // Negative effort = downward thrust = exhaust upward
            exhaustDir.set(0, effortSign > 0 ? -1 : 1, 0);
          } else {
            exhaustDir.copy(config.direction);
          }
          exhaustDir.applyQuaternion(quat);

          // Bubble velocity scales with effort (faster thrust = faster bubbles)
          const speed = 0.08 + (effort / 100) * 0.35;
          velArr[i].set(
            exhaustDir.x * speed + (Math.random() - 0.5) * 0.04,
            exhaustDir.y * speed + Math.random() * 0.06 + 0.02, // natural buoyancy
            exhaustDir.z * speed + (Math.random() - 0.5) * 0.04
          );

          // Bubble size scales with effort
          sizeArr[i] = 0.015 + (effort / 100) * 0.035 + Math.random() * 0.01;
        } else {
          // Hide inactive particles
          arr[i * 3 + 1] = -20;
          sizeArr[i] = 0;
        }
      } else {
        // Animate living bubble
        const age = lifeArr[i]; // 0 to 1

        // Move
        arr[i * 3]     += velArr[i].x * delta;
        arr[i * 3 + 1] += velArr[i].y * delta;
        arr[i * 3 + 2] += velArr[i].z * delta;

        // Buoyancy increases as bubble rises (decelerate exhaust, accelerate upward)
        velArr[i].y += delta * 0.12;

        // Slight random wobble for realism
        velArr[i].x += (Math.random() - 0.5) * delta * 0.15;
        velArr[i].z += (Math.random() - 0.5) * delta * 0.15;

        // Slow down over time (water drag)
        velArr[i].multiplyScalar(1 - delta * 0.8);

        // Fade size near end of life
        if (age > 0.7) {
          sizeArr[i] *= (1 - (age - 0.7) / 0.3) * 0.97;
        }
      }
    }

    posAttr.needsUpdate = true;
    sizeAttr.needsUpdate = true;
  });

  return (
    <points ref={pointsRef}>
      <bufferGeometry>
        <bufferAttribute
          attach="attributes-position"
          count={TOTAL_PARTICLES}
          array={posArr}
          itemSize={3}
        />
        <bufferAttribute
          attach="attributes-size"
          count={TOTAL_PARTICLES}
          array={sizeArr}
          itemSize={1}
        />
      </bufferGeometry>
      <pointsMaterial
        size={0.04}
        color="#e0f2fe"
        transparent
        opacity={0.8}
        blending={THREE.AdditiveBlending}
        depthWrite={false}
        sizeAttenuation={true}
      />
    </points>
  );
}
