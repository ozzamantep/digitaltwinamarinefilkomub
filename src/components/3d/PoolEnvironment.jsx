import { useMemo, useRef } from 'react';
import { useFrame, useLoader } from '@react-three/fiber';
import * as THREE from 'three';
import { TextureLoader } from 'three';
import PoolCaustics from './PoolCaustics';
import useVehicleStore from '../../store/vehicleStore';

export default function PoolEnvironment() {
  const waterRef = useRef();
  const flaresFallen = useVehicleStore((s) => s.flaresFallen || { red: false, blue: false, yellow: false, orange: false });
  const obstacles = useVehicleStore((s) => s.obstacles) || {
    orange_flare: { x: -6.0, z: 2.0 },
    blue_flare: { x: -2.0, z: 2.2 },
    red_flare: { x: 0.5, z: 4.0 },
    yellow_flare: { x: -0.5, z: -4.5 },
    gate: { x: 4.0, z: 0.0 },
    drum_red_tgt: { x: 10.5, z: 1.5 },
    drum_blue: { x: 10.5, z: 4.5 },
  };

  // Refs for smooth toppling / falling animation
  const blueFlareGroupRef = useRef();
  const redFlareGroupRef = useRef();
  const yellowFlareGroupRef = useRef();

  // Load pool floor tile texture
  const poolTexture = useLoader(TextureLoader, '/assets/pool/pool_bottom_tiles_highres.png');
  useMemo(() => {
    if (poolTexture) {
      poolTexture.wrapS = THREE.RepeatWrapping;
      poolTexture.wrapT = THREE.RepeatWrapping;
      poolTexture.repeat.set(10, 6);
      poolTexture.anisotropy = 8;
    }
  }, [poolTexture]);

  // Animate water surface ripples & physical flare toppling
  useFrame((state, delta) => {
    if (waterRef.current) {
      waterRef.current.material.opacity = 0.18 + Math.sin(state.clock.elapsedTime * 1.5) * 0.02;
    }

    // Blue Flare Topple Animation (Rotates 90 deg down to pool floor)
    if (blueFlareGroupRef.current) {
      const targetRot = flaresFallen.blue ? Math.PI / 2.05 : 0;
      blueFlareGroupRef.current.rotation.z = THREE.MathUtils.damp(
        blueFlareGroupRef.current.rotation.z,
        targetRot,
        6.0,
        delta
      );
    }

    // Red Flare Topple Animation
    if (redFlareGroupRef.current) {
      const targetRot = flaresFallen.red ? Math.PI / 2.05 : 0;
      redFlareGroupRef.current.rotation.z = THREE.MathUtils.damp(
        redFlareGroupRef.current.rotation.z,
        targetRot,
        6.0,
        delta
      );
    }

    // Yellow Flare Topple Animation
    if (yellowFlareGroupRef.current) {
      const targetRot = flaresFallen.yellow ? -Math.PI / 2.05 : 0;
      yellowFlareGroupRef.current.rotation.z = THREE.MathUtils.damp(
        yellowFlareGroupRef.current.rotation.z,
        targetRot,
        6.0,
        delta
      );
    }
  });

  const poolLength = 25;
  const poolWidth = 16;
  const poolDepth = 2.0;

  return (
    <group>
      {/* 1. POOL FLOOR (Y = 0) */}
      <mesh rotation={[-Math.PI / 2, 0, 0]} position={[0, 0, 0]} receiveShadow>
        <planeGeometry args={[poolLength, poolWidth]} />
        <meshStandardMaterial
          map={poolTexture}
          roughness={0.25}
          metalness={0.15}
          color="#93c5fd"
        />
      </mesh>

      {/* Dynamic Animated Swimming Pool Caustics Layer */}
      <PoolCaustics />

      {/* Swim Lane Markings on Floor */}
      {[-5, 0, 5].map((zPos, idx) => (
        <group key={idx}>
          <mesh rotation={[-Math.PI / 2, 0, 0]} position={[0, 0.002, zPos]}>
            <planeGeometry args={[22, 0.25]} />
            <meshBasicMaterial color="#0f172a" />
          </mesh>
          <mesh rotation={[-Math.PI / 2, 0, 0]} position={[-9.5, 0.003, zPos]}>
            <planeGeometry args={[0.25, 1.2]} />
            <meshBasicMaterial color="#0f172a" />
          </mesh>
          <mesh rotation={[-Math.PI / 2, 0, 0]} position={[9.5, 0.003, zPos]}>
            <planeGeometry args={[0.25, 1.2]} />
            <meshBasicMaterial color="#0f172a" />
          </mesh>
        </group>
      ))}

      {/* Zone Floor Delineation Stripes matching official diagram */}
      {/* Orange Flare Zone Backdrop (-7m to -4.5m) */}
      <mesh rotation={[-Math.PI / 2, 0, 0]} position={[-5.75, 0.003, 0]}>
        <planeGeometry args={[2.5, 15.6]} />
        <meshBasicMaterial color="#38bdf8" transparent opacity={0.15} />
      </mesh>
      {/* Multi-flare Zone Backdrop (-4.5m to 3.5m) */}
      <mesh rotation={[-Math.PI / 2, 0, 0]} position={[-0.5, 0.003, 0]}>
        <planeGeometry args={[8.0, 15.6]} />
        <meshBasicMaterial color="#fef08a" transparent opacity={0.12} />
      </mesh>

      {/* 2. STARTING ZONE SQUARE (140cm x 140cm at X = -11.0m, Z = 2.0m) */}
      <group position={[-11.0, 0, 2.0]}>
        <mesh rotation={[-Math.PI / 2, 0, 0]} position={[0, 0.004, 0]}>
          <planeGeometry args={[1.4, 1.4]} />
          <meshBasicMaterial color="#ffffff" transparent opacity={0.4} />
        </mesh>
        <mesh rotation={[-Math.PI / 2, 0, 0]} position={[0, 0.005, 0]}>
          <ringGeometry args={[0.65, 0.7, 32]} />
          <meshBasicMaterial color="#00ff88" />
        </mesh>
      </group>

      {/* 3. POOL WALLS */}
      {/* North Wall (+Z = 8) */}
      <mesh position={[0, poolDepth / 2, poolWidth / 2]} receiveShadow>
        <boxGeometry args={[poolLength, poolDepth, 0.2]} />
        <meshStandardMaterial color="#0369a1" roughness={0.3} metalness={0.2} />
      </mesh>
      {/* South Wall (-Z = -8) */}
      <mesh position={[0, poolDepth / 2, -poolWidth / 2]} receiveShadow>
        <boxGeometry args={[poolLength, poolDepth, 0.2]} />
        <meshStandardMaterial color="#0369a1" roughness={0.3} metalness={0.2} />
      </mesh>
      {/* East Wall (+X = 12.5) */}
      <mesh position={[poolLength / 2, poolDepth / 2, 0]} receiveShadow>
        <boxGeometry args={[0.2, poolDepth, poolWidth]} />
        <meshStandardMaterial color="#0284c7" roughness={0.3} metalness={0.2} />
      </mesh>
      {/* West Wall (-X = -12.5) */}
      <mesh position={[-poolLength / 2, poolDepth / 2, 0]} receiveShadow>
        <boxGeometry args={[0.2, poolDepth, poolWidth]} />
        <meshStandardMaterial color="#0284c7" roughness={0.3} metalness={0.2} />
      </mesh>

      {/* Pool Deck Coping Rim */}
      <mesh position={[0, poolDepth + 0.05, poolWidth / 2 + 0.3]}>
        <boxGeometry args={[poolLength + 1.2, 0.1, 0.6]} />
        <meshStandardMaterial color="#f8fafc" roughness={0.5} />
      </mesh>
      <mesh position={[0, poolDepth + 0.05, -poolWidth / 2 - 0.3]}>
        <boxGeometry args={[poolLength + 1.2, 0.1, 0.6]} />
        <meshStandardMaterial color="#f8fafc" roughness={0.5} />
      </mesh>
      <mesh position={[poolLength / 2 + 0.3, poolDepth + 0.05, 0]}>
        <boxGeometry args={[0.6, 0.1, poolWidth + 1.2]} />
        <meshStandardMaterial color="#f8fafc" roughness={0.5} />
      </mesh>
      <mesh position={[-poolLength / 2 - 0.3, poolDepth + 0.05, 0]}>
        <boxGeometry args={[0.6, 0.1, poolWidth + 1.2]} />
        <meshStandardMaterial color="#f8fafc" roughness={0.5} />
      </mesh>

      {/* Swimming Pool Racing Black Guidelines on Floor */}
      <mesh rotation={[-Math.PI / 2, 0, 0]} position={[0, 0.005, 0]} receiveShadow>
        <planeGeometry args={[23, 0.25]} />
        <meshBasicMaterial color="#0f172a" />
      </mesh>
      <mesh rotation={[-Math.PI / 2, 0, 0]} position={[0, 0.005, -4.0]} receiveShadow>
        <planeGeometry args={[23, 0.25]} />
        <meshBasicMaterial color="#0f172a" />
      </mesh>
      <mesh rotation={[-Math.PI / 2, 0, 0]} position={[0, 0.005, 4.0]} receiveShadow>
        <planeGeometry args={[23, 0.25]} />
        <meshBasicMaterial color="#0f172a" />
      </mesh>

      {/* Cross-lines on Start & Finish */}
      <mesh rotation={[-Math.PI / 2, 0, 0]} position={[-10.0, 0.006, 0]} receiveShadow>
        <planeGeometry args={[0.25, 1.5]} />
        <meshBasicMaterial color="#0f172a" />
      </mesh>
      <mesh rotation={[-Math.PI / 2, 0, 0]} position={[10.0, 0.006, 0]} receiveShadow>
        <planeGeometry args={[0.25, 1.5]} />
        <meshBasicMaterial color="#0f172a" />
      </mesh>

      {/* 4. Crystal-Clear Transparent Water Surface (Zero Ghosting / Zero Double-Image Refraction) */}
      <mesh
        ref={waterRef}
        rotation={[-Math.PI / 2, 0, 0]}
        position={[0, 1.95, 0]}
      >
        <planeGeometry args={[25, 16]} />
        <meshStandardMaterial
          color="#0284c7"
          transparent
          opacity={0.18}
          roughness={0.08}
          metalness={0.1}
          depthWrite={false}
          side={THREE.DoubleSide}
        />
      </mesh>

      {/* ========================================================= */}
      {/* 5. OFFICIAL SAUVC 2026 COMPETITION MISSION OBJECTS        */}
      {/* ========================================================= */}

      {/* === ZONE 1: ORANGE FLARE === */}
      <group position={[obstacles.orange_flare?.x ?? -6.0, 0, obstacles.orange_flare?.z ?? 2.0]}>
        <mesh position={[0, 0.75, 0]} castShadow>
          <cylinderGeometry args={[0.07, 0.07, 1.5, 16]} />
          <meshStandardMaterial color="#ea580c" emissive="#c2410c" emissiveIntensity={0.35} roughness={0.25} />
        </mesh>
        <mesh position={[0, 0.05, 0]}>
          <cylinderGeometry args={[0.26, 0.26, 0.1, 16]} />
          <meshStandardMaterial color="#1e293b" />
        </mesh>
      </group>

      {/* === ZONE 2: FLARES (Blue, Red, Yellow) THAT FALL WHEN STRUCK === */}

      {/* Blue Flare */}
      <group position={[obstacles.blue_flare?.x ?? -2.0, 0, obstacles.blue_flare?.z ?? 2.2]}>
        <mesh position={[0, 0.05, 0]}>
          <cylinderGeometry args={[0.26, 0.26, 0.1, 16]} />
          <meshStandardMaterial color="#1e293b" />
        </mesh>
        <group ref={blueFlareGroupRef} position={[0, 0.1, 0]}>
          <mesh position={[0, 0.7, 0]} castShadow>
            <cylinderGeometry args={[0.07, 0.07, 1.4, 16]} />
            <meshStandardMaterial color="#0284c7" emissive="#0369a1" emissiveIntensity={0.35} roughness={0.25} />
          </mesh>
        </group>
      </group>

      {/* Red Flare */}
      <group position={[obstacles.red_flare?.x ?? 0.5, 0, obstacles.red_flare?.z ?? 4.0]}>
        <mesh position={[0, 0.05, 0]}>
          <cylinderGeometry args={[0.26, 0.26, 0.1, 16]} />
          <meshStandardMaterial color="#1e293b" />
        </mesh>
        <group ref={redFlareGroupRef} position={[0, 0.1, 0]}>
          <mesh position={[0, 0.7, 0]} castShadow>
            <cylinderGeometry args={[0.07, 0.07, 1.4, 16]} />
            <meshStandardMaterial color="#dc2626" emissive="#b91c1c" emissiveIntensity={0.35} roughness={0.25} />
          </mesh>
        </group>
      </group>

      {/* Yellow Flare */}
      <group position={[obstacles.yellow_flare?.x ?? -0.5, 0, obstacles.yellow_flare?.z ?? -4.5]}>
        <mesh position={[0, 0.05, 0]}>
          <cylinderGeometry args={[0.26, 0.26, 0.1, 16]} />
          <meshStandardMaterial color="#1e293b" />
        </mesh>
        <group ref={yellowFlareGroupRef} position={[0, 0.1, 0]}>
          <mesh position={[0, 0.7, 0]} castShadow>
            <cylinderGeometry args={[0.07, 0.07, 1.4, 16]} />
            <meshStandardMaterial color="#eab308" emissive="#ca8a04" emissiveIntensity={0.35} roughness={0.25} />
          </mesh>
        </group>
      </group>

      {/* === GATE with Red & Green Markers === */}
      <group position={[obstacles.gate?.x ?? 4.0, 0, obstacles.gate?.z ?? 0.0]}>
        {/* Left Post */}
        <mesh position={[0, 0.75, -0.9]} castShadow>
          <cylinderGeometry args={[0.045, 0.045, 1.5, 16]} />
          <meshStandardMaterial color="#f59e0b" roughness={0.3} metalness={0.2} />
        </mesh>
        {/* Right Post */}
        <mesh position={[0, 0.75, 0.9]} castShadow>
          <cylinderGeometry args={[0.045, 0.045, 1.5, 16]} />
          <meshStandardMaterial color="#f59e0b" roughness={0.3} metalness={0.2} />
        </mesh>
        {/* Top Crossbar */}
        <mesh position={[0, 1.5, 0]} rotation={[Math.PI / 2, 0, 0]} castShadow>
          <cylinderGeometry args={[0.045, 0.045, 1.8, 16]} />
          <meshStandardMaterial color="#f97316" roughness={0.3} metalness={0.2} />
        </mesh>
        {/* Red Marker Top-Left */}
        <mesh position={[0, 1.5, -0.9]}>
          <boxGeometry args={[0.15, 0.15, 0.15]} />
          <meshStandardMaterial color="#ef4444" emissive="#b91c1c" emissiveIntensity={0.4} />
        </mesh>
        {/* Green Marker Top-Right */}
        <mesh position={[0, 1.5, 0.9]}>
          <boxGeometry args={[0.15, 0.15, 0.15]} />
          <meshStandardMaterial color="#22c55e" emissive="#15803d" emissiveIntensity={0.4} />
        </mesh>
        {/* Post Bases */}
        <mesh position={[0, 0.05, -0.9]}>
          <boxGeometry args={[0.35, 0.1, 0.35]} />
          <meshStandardMaterial color="#334155" roughness={0.8} />
        </mesh>
        <mesh position={[0, 0.05, 0.9]}>
          <boxGeometry args={[0.35, 0.1, 0.35]} />
          <meshStandardMaterial color="#334155" roughness={0.8} />
        </mesh>
      </group>

      {/* === TARGET ZONE: 4 DRUMS (1 Blue Drum + 3 Red Drums along X = 10.5m) === */}
      {/* 1. Blue Drum */}
      <group position={[obstacles.drum_blue?.x ?? 10.5, 0.18, obstacles.drum_blue?.z ?? 4.5]}>
        <mesh castShadow>
          <cylinderGeometry args={[0.35, 0.3, 0.45, 24, 1, true]} />
          <meshStandardMaterial color="#0284c7" side={THREE.DoubleSide} roughness={0.4} />
        </mesh>
        <mesh position={[0, -0.21, 0]}>
          <cylinderGeometry args={[0.3, 0.3, 0.02, 24]} />
          <meshStandardMaterial color="#0369a1" />
        </mesh>
      </group>

      {/* 2. Red Drum Target Bucket 1 - Primary Target Drop Zone */}
      <group position={[obstacles.drum_red_tgt?.x ?? 10.5, 0.18, obstacles.drum_red_tgt?.z ?? 1.5]}>
        <mesh castShadow>
          <cylinderGeometry args={[0.35, 0.3, 0.45, 24, 1, true]} />
          <meshStandardMaterial color="#ef4444" side={THREE.DoubleSide} roughness={0.4} />
        </mesh>
        <mesh position={[0, -0.21, 0]}>
          <cylinderGeometry args={[0.3, 0.3, 0.02, 24]} />
          <meshStandardMaterial color="#991b1b" />
        </mesh>
      </group>

      {/* 3. Red Drum 2 (Z = -1.5m) */}
      <group position={[10.5, 0.18, -1.5]}>
        <mesh castShadow>
          <cylinderGeometry args={[0.35, 0.3, 0.45, 24, 1, true]} />
          <meshStandardMaterial color="#ef4444" side={THREE.DoubleSide} roughness={0.4} />
        </mesh>
        <mesh position={[0, -0.21, 0]}>
          <cylinderGeometry args={[0.3, 0.3, 0.02, 24]} />
          <meshStandardMaterial color="#991b1b" />
        </mesh>
      </group>

      {/* 4. Red Drum 3 (Z = -4.5m) */}
      <group position={[10.5, 0.18, -4.5]}>
        <mesh castShadow>
          <cylinderGeometry args={[0.35, 0.3, 0.45, 24, 1, true]} />
          <meshStandardMaterial color="#ef4444" side={THREE.DoubleSide} roughness={0.4} />
        </mesh>
        <mesh position={[0, -0.21, 0]}>
          <cylinderGeometry args={[0.3, 0.3, 0.02, 24]} />
          <meshStandardMaterial color="#991b1b" />
        </mesh>
      </group>
    </group>
  );
}
