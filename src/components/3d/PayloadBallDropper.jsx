import { useRef, useMemo } from 'react';
import { useFrame } from '@react-three/fiber';
import * as THREE from 'three';
import useVehicleStore from '../../store/vehicleStore';

export default function PayloadBallDropper() {
  const ballGroupRef = useRef();
  const ballMeshRef = useRef();

  const position = useVehicleStore((s) => s.position);
  const rotation = useVehicleStore((s) => s.rotation);
  const payloadState = useVehicleStore(
    (s) => s.payloadState || { loaded: true, dropped: false, onFloor: false, grasped: true, inDrum: false, x: 10.5, y: 0.20, z: 1.5 }
  );

  // Hydrodynamic Simulation Internal State
  const sim = useRef({
    initialized: false,
    prevGrasped: true,
    prevDropped: false,
    isSinking: false,
    hasSettled: false,
    elapsed: 0,
    currPos: new THREE.Vector3(10.5, 0.20, 1.5),
    startPos: new THREE.Vector3(10.5, 0.95, 1.5),
    targetPos: new THREE.Vector3(10.5, 0.20, 1.5),
    velocity: new THREE.Vector3(0, 0, 0),
    rot: new THREE.Euler(0, 0, 0),
    rotVel: new THREE.Vector3(0, 0, 0),
  });

  useFrame((state, rawDelta) => {
    if (!ballGroupRef.current) return;
    const delta = Math.min(rawDelta, 0.045); // Clamp to prevent physics step spikes
    const time = state.clock.getElapsedTime();
    const s = sim.current;

    const isGrasped = !!payloadState.grasped;
    const isDropped = !!payloadState.dropped || !!payloadState.inDrum || !!payloadState.onFloor;

    // Detect Release Trigger: was grasped/loaded, now dropped/released
    const releaseTriggered =
      (s.prevGrasped && !isGrasped && isDropped) ||
      (!s.prevDropped && isDropped && !s.isSinking && !s.hasSettled);

    s.prevGrasped = isGrasped;
    s.prevDropped = isDropped;

    if (releaseTriggered) {
      // 1. Capture exact release origin from BlueROV2 gripper jaws in world coordinates
      const subX = position?.x ?? 10.5;
      const subY = position?.y ?? 1.1;
      const subZ = position?.z ?? 1.5;
      const yaw = rotation?.yaw ?? (typeof rotation?.y === 'number' ? rotation.y : 0);

      // Gripper jaws are mounted at front (+0.32m forward, -0.10m lower)
      const releaseX = subX + Math.cos(yaw) * 0.32;
      const releaseY = Math.max(0.35, subY - 0.10);
      const releaseZ = subZ + Math.sin(yaw) * 0.32;

      s.startPos.set(releaseX, releaseY, releaseZ);
      s.currPos.copy(s.startPos);

      // Target landing destination (drum bucket interior or floor)
      const destX = payloadState.x ?? 10.5;
      const destZ = payloadState.z ?? 1.5;
      const destY = payloadState.onFloor ? 0.08 : 0.20;
      s.targetPos.set(destX, destY, destZ);

      // Initial impulse: vehicle forward momentum + release nudge
      s.velocity.set(
        Math.cos(yaw) * 0.10,
        -0.08,
        Math.sin(yaw) * 0.10 + (Math.random() - 0.5) * 0.05
      );

      // Initial random 3D angular tumble velocity (rad/s)
      s.rotVel.set(
        (Math.random() - 0.5) * 8.0,
        (Math.random() - 0.5) * 6.0,
        (Math.random() - 0.5) * 8.0
      );

      s.isSinking = true;
      s.hasSettled = false;
      s.elapsed = 0;
      s.nextBubbleTimer = 0;
    }

    // STATE 1: Clamped inside gripper jaws (visible via BlueROV2Model directly)
    if (isGrasped) {
      ballGroupRef.current.visible = false;
      s.isSinking = false;
      s.hasSettled = false;

      // Update position to stay tethered to ROV gripper tip
      const subX = position?.x ?? -10;
      const subY = position?.y ?? 1.1;
      const subZ = position?.z ?? 0;
      const yaw = rotation?.yaw ?? (typeof rotation?.y === 'number' ? rotation.y : 0);
      s.currPos.set(
        subX + Math.cos(yaw) * 0.32,
        subY - 0.10,
        subZ + Math.sin(yaw) * 0.32
      );
      ballGroupRef.current.position.copy(s.currPos);
      return;
    }

    // Ball is freed: ensure dropper model is visible
    ballGroupRef.current.visible = true;

    // STATE 2: Actively sinking through water column with Hydrodynamic Distortion
    if (s.isSinking) {
      s.elapsed += delta;
      const t = s.elapsed;

      // --- Vertical Hydrodynamics (Negative Buoyancy + Quadratic Water Drag) ---
      const effectiveGravity = -1.90; // m/s^2 (submerged payload downward pull)
      const waterDragCoeff = 2.45;    // drag coefficient in pool water
      const dragAccelY = -waterDragCoeff * s.velocity.y * Math.abs(s.velocity.y);
      s.velocity.y += (effectiveGravity + dragAccelY) * delta;
      
      // Step vertical descent
      s.currPos.y += s.velocity.y * delta;

      // Vertical descent progress ratio: 0.0 at release -> 1.0 at touchdown
      const totalSpanY = Math.max(0.1, s.startPos.y - s.targetPos.y);
      const progress = Math.max(0.0, Math.min(1.0, (s.startPos.y - s.currPos.y) / totalSpanY));

      // --- Horizontal Wave Distortion & Pool Eddy Currents (Distorsi Ombak Kolam) ---
      // 1. Primary low-frequency pool wave orbital currents (surface swell decaying with depth)
      const waveOrbitalX = Math.sin(time * 3.2 + 0.6) * 0.042;
      const waveOrbitalZ = Math.cos(time * 2.7 + 1.1) * 0.045;

      // 2. Turbulent subsea eddy / thruster downwash swirling
      const eddyX = Math.cos(time * 6.5 + 2.1) * 0.024;
      const eddyZ = Math.sin(time * 7.1 + 0.3) * 0.026;

      // 3. Von Kármán vortex-shedding high frequency cross-flow flutter (wobble)
      const flutterX = Math.sin(time * 17.5) * 0.012;
      const flutterZ = Math.cos(time * 19.0) * 0.012;

      // Wave distortion factor: strong near surface, gently dampens inside drum bucket
      const waveDamping = 1.0 - Math.pow(progress, 1.8) * 0.75;
      const totalDistortionX = (waveOrbitalX + eddyX + flutterX) * waveDamping;
      const totalDistortionZ = (waveOrbitalZ + eddyZ + flutterZ) * waveDamping;

      // Base smooth interpolation from release X/Z to target drum X/Z
      const baseX = THREE.MathUtils.lerp(s.startPos.x, s.targetPos.x, progress);
      const baseZ = THREE.MathUtils.lerp(s.startPos.z, s.targetPos.z, progress);

      s.currPos.x = baseX + totalDistortionX;
      s.currPos.z = baseZ + totalDistortionZ;

      // --- 3D Angular Tumbling & Rotational Distortion ---
      // Water eddies exert torque causing the spherical payload to tumble organically
      s.rot.x += (s.rotVel.x + Math.sin(time * 5.5) * 4.2) * delta;
      s.rot.y += (s.rotVel.y + Math.cos(time * 4.0) * 3.5) * delta;
      s.rot.z += (s.rotVel.z + Math.sin(time * 6.2) * 4.0) * delta;
      // Fluid viscosity slows down spinning
      s.rotVel.multiplyScalar(1.0 - 0.85 * delta);

      // --- Check Bottom Touchdown (Drum floor or Pool floor) ---
      if (s.currPos.y <= s.targetPos.y) {
        s.currPos.y = s.targetPos.y;

        // Subtle soft hydrodynamic cushioned bounce
        if (Math.abs(s.velocity.y) > 0.07) {
          s.velocity.y = -s.velocity.y * 0.22; // High damping in water
        } else {
          // Fully settled at rest inside drum
          s.velocity.set(0, 0, 0);
          s.isSinking = false;
          s.hasSettled = true;
          s.currPos.copy(s.targetPos);
        }
      }
    } else {
      // STATE 3: Settled Resting inside Drum or on Floor
      const destX = payloadState.x ?? 10.5;
      const destZ = payloadState.z ?? 1.5;
      const destY = payloadState.onFloor ? 0.08 : 0.20;

      // Gentle subsea idle water breathing micro-sway (±1.5mm)
      const idleSwayY = Math.sin(time * 1.8) * 0.0018;
      const idleSwayX = Math.cos(time * 1.2) * 0.0012;
      s.currPos.set(destX + idleSwayX, destY + idleSwayY, destZ);
      s.rot.y += delta * 0.15; // Slow ambient rotation
    }

    // Apply computed position & rotation to the 3D group
    ballGroupRef.current.position.copy(s.currPos);
    if (ballMeshRef.current) {
      ballMeshRef.current.rotation.copy(s.rot);
    }
  });

  return (
    <group ref={ballGroupRef}>
      {/* Official SAUVC Red Competition Ball Payload with Hydrodynamic Gloss */}
      <mesh ref={ballMeshRef} castShadow receiveShadow>
        <sphereGeometry args={[0.042, 32, 32]} />
        <meshStandardMaterial
          color="#ef4444"
          roughness={0.22}
          metalness={0.20}
          emissive="#b91c1c"
          emissiveIntensity={0.45}
        />
        {/* Subtle Equatorial Seam / Dimpled Golf Ball Accent */}
        <mesh rotation={[Math.PI / 2, 0, 0]}>
          <torusGeometry args={[0.0422, 0.0018, 12, 32]} />
          <meshStandardMaterial
            color="#ff6b6b"
            emissive="#f87171"
            emissiveIntensity={0.6}
            roughness={0.3}
          />
        </mesh>
      </mesh>

      {/* Ambient Subsea Luminous Hydrodynamic Beacon */}
      <pointLight color="#ef4444" intensity={0.65} distance={1.2} />

      {/* Subtle Water Disturbance Glow Aura */}
      <mesh scale={[1.18, 1.18, 1.18]}>
        <sphereGeometry args={[0.042, 16, 16]} />
        <meshBasicMaterial
          color="#ef4444"
          transparent
          opacity={0.12}
          side={THREE.BackSide}
        />
      </mesh>
    </group>
  );
}
