import { useRef, useMemo } from 'react';
import { Canvas, useFrame } from '@react-three/fiber';
import { OrbitControls, PerspectiveCamera, Grid, Center } from '@react-three/drei';
import * as THREE from 'three';

/**
 * Procedural High-Fidelity Blue Robotics T200 Subsea Thruster Model
 */
function T200PhysicalThruster({ rpm = 0, thrust = 0 }) {
  const propRef = useRef();

  // Rotate propeller blades around central shaft axis based on RPM
  useFrame((_, delta) => {
    if (propRef.current) {
      // RPM to rad/s: (rpm * 2 * PI) / 60
      const radPerSec = (rpm * 2 * Math.PI) / 60;
      propRef.current.rotation.z += radPerSec * delta;
    }
  });

  // 3x Pitch-Cambered Hydrodynamic Propeller Blades Geometry
  const bladeGeometry = useMemo(() => {
    const geom = new THREE.BufferGeometry();
    const vertices = new Float32Array([
      0.0, 0.008, 0.004,
      0.016, 0.046, -0.007,
      -0.012, 0.048, 0.007,

      0.0, 0.008, -0.004,
      -0.012, 0.048, 0.007,
      0.016, 0.046, -0.007,
    ]);
    geom.setAttribute('position', new THREE.BufferAttribute(vertices, 3));
    geom.computeVertexNormals();
    return geom;
  }, []);

  return (
    <group position={[0, 0.14, 0]}>
      {/* 1. Outer Cylindrical Kort Nozzle Shroud (Black Polycarbonate) */}
      <mesh rotation={[Math.PI / 2, 0, 0]}>
        <cylinderGeometry args={[0.062, 0.062, 0.054, 36, 1, true]} />
        <meshStandardMaterial
          color="#0f172a"
          roughness={0.35}
          metalness={0.7}
          side={THREE.DoubleSide}
        />
      </mesh>

      {/* Aerodynamic Duct Lips (Chamfered Inlet & Outlet Rings) */}
      <mesh position={[0, 0, 0.027]} rotation={[0, 0, 0]}>
        <torusGeometry args={[0.062, 0.004, 16, 36]} />
        <meshStandardMaterial color="#1e293b" roughness={0.3} metalness={0.8} />
      </mesh>
      <mesh position={[0, 0, -0.027]} rotation={[0, 0, 0]}>
        <torusGeometry args={[0.062, 0.004, 16, 36]} />
        <meshStandardMaterial color="#1e293b" roughness={0.3} metalness={0.8} />
      </mesh>

      {/* Blue Robotics Accent Ring Striping */}
      <mesh position={[0, 0, 0]} rotation={[Math.PI / 2, 0, 0]}>
        <cylinderGeometry args={[0.063, 0.063, 0.008, 36, 1, true]} />
        <meshStandardMaterial
          color="#00f0ff"
          emissive="#00f0ff"
          emissiveIntensity={0.6}
          roughness={0.2}
          side={THREE.DoubleSide}
        />
      </mesh>

      {/* 3x Stator Vanes / Internal Support Brackets */}
      {[0, Math.PI * 0.666, Math.PI * 1.333].map((angle, idx) => (
        <mesh
          key={idx}
          position={[0, 0, 0]}
          rotation={[Math.PI / 2, angle, 0]}
        >
          <boxGeometry args={[0.005, 0.038, 0.052]} />
          <meshStandardMaterial color="#1e293b" roughness={0.4} metalness={0.7} />
        </mesh>
      ))}

      {/* Mounting Foot Bracket with Cyan Accents */}
      <group position={[0, -0.066, 0]}>
        <mesh position={[0, -0.006, 0]}>
          <boxGeometry args={[0.032, 0.016, 0.044]} />
          <meshStandardMaterial color="#0f172a" roughness={0.3} metalness={0.85} />
        </mesh>
        <mesh position={[0, -0.012, 0]}>
          <boxGeometry args={[0.026, 0.004, 0.036]} />
          <meshStandardMaterial color="#00f0ff" emissive="#00f0ff" emissiveIntensity={0.8} />
        </mesh>
      </group>

      {/* 2. Rotating Propeller Rotor & Bullet Hub Assembly */}
      <group ref={propRef}>
        {/* Central Cylindrical Hub */}
        <mesh rotation={[Math.PI / 2, 0, 0]}>
          <cylinderGeometry args={[0.016, 0.018, 0.038, 24]} />
          <meshStandardMaterial color="#0f172a" roughness={0.25} metalness={0.8} />
        </mesh>

        {/* Bullet Nosecone Tip */}
        <mesh position={[0, 0, 0.024]} rotation={[Math.PI / 2, 0, 0]}>
          <coneGeometry args={[0.016, 0.022, 24]} />
          <meshStandardMaterial color="#0284c7" roughness={0.2} metalness={0.7} />
        </mesh>

        {/* 3x Hydrofoil Pitch-Cambered Blades */}
        {[0, (Math.PI * 2) / 3, (Math.PI * 4) / 3].map((angle, i) => (
          <group key={i} rotation={[0, 0, angle]}>
            <mesh geometry={bladeGeometry}>
              <meshStandardMaterial
                color="#0284c7"
                emissive="#00f0ff"
                emissiveIntensity={Math.min(1.2, Math.abs(rpm) / 3500)}
                roughness={0.2}
                metalness={0.65}
                side={THREE.DoubleSide}
              />
            </mesh>
          </group>
        ))}
      </group>
    </group>
  );
}

/**
 * Testbed Rig: Aluminum Profile Stand & Load Cell Sensor Body
 */
function TestStandBench({ thrust = 0 }) {
  const isForward = thrust >= 0;
  const absThrust = Math.abs(thrust);
  const vectorLength = Math.min(0.35, (absThrust / 50) * 0.35);

  return (
    <group position={[0, 0, 0]}>
      {/* Heavy Base Plate */}
      <mesh position={[0, -0.04, 0]}>
        <boxGeometry args={[0.32, 0.02, 0.45]} />
        <meshStandardMaterial color="#111827" metalness={0.8} roughness={0.2} />
      </mesh>

      {/* Rubber anti-vibration feet */}
      {[
        [-0.14, -0.055, -0.2],
        [0.14, -0.055, -0.2],
        [-0.14, -0.055, 0.2],
        [0.14, -0.055, 0.2],
      ].map((pos, i) => (
        <mesh key={i} position={pos}>
          <cylinderGeometry args={[0.018, 0.018, 0.01, 16]} />
          <meshStandardMaterial color="#030712" roughness={0.9} />
        </mesh>
      ))}

      {/* Vertical Aluminum Extrusion Support Mast (2020 / 4040 profile) */}
      <mesh position={[0, 0.03, 0]}>
        <boxGeometry args={[0.04, 0.12, 0.04]} />
        <meshStandardMaterial color="#334155" metalness={0.9} roughness={0.25} />
      </mesh>

      {/* S-Beam / Parallel Beam Load Cell (Strain Gauge Sensor) */}
      <group position={[0, 0.065, 0]}>
        <mesh position={[0, 0, 0]}>
          <boxGeometry args={[0.028, 0.036, 0.075]} />
          <meshStandardMaterial color="#94a3b8" metalness={0.95} roughness={0.15} />
        </mesh>
        {/* Load cell strain cutouts */}
        <mesh position={[0, 0.008, 0]}>
          <boxGeometry args={[0.03, 0.006, 0.04]} />
          <meshStandardMaterial color="#1e293b" />
        </mesh>
        {/* Strain Gauge Wire (HX711 connection) */}
        <mesh position={[0.016, 0, 0]} rotation={[0, 0, Math.PI / 2]}>
          <cylinderGeometry args={[0.003, 0.003, 0.012, 8]} />
          <meshStandardMaterial color="#ef4444" />
        </mesh>
      </group>

      {/* Dynamic 3D Force Vector Arrow */}
      {absThrust > 0.5 && (
        <group position={[0, 0.14, isForward ? 0.06 : -0.06]}>
          {/* Arrow Cylinder Shaft */}
          <mesh
            position={[0, 0, isForward ? vectorLength / 2 : -vectorLength / 2]}
            rotation={[Math.PI / 2, 0, 0]}
          >
            <cylinderGeometry args={[0.004, 0.004, vectorLength, 12]} />
            <meshStandardMaterial
              color={isForward ? '#00f0ff' : '#ff8c00'}
              emissive={isForward ? '#00f0ff' : '#ff8c00'}
              emissiveIntensity={1.0}
            />
          </mesh>
          {/* Arrow Cone Head */}
          <mesh
            position={[0, 0, isForward ? vectorLength : -vectorLength]}
            rotation={[isForward ? Math.PI / 2 : -Math.PI / 2, 0, 0]}
          >
            <coneGeometry args={[0.012, 0.028, 16]} />
            <meshStandardMaterial
              color={isForward ? '#00f0ff' : '#ff8c00'}
              emissive={isForward ? '#00f0ff' : '#ff8c00'}
              emissiveIntensity={1.2}
            />
          </mesh>
        </group>
      )}
    </group>
  );
}

/**
 * Cavitation / Water Flow Particle Stream traveling through the thruster
 */
function FluidCavitationStream({ rpm = 0 }) {
  const particlesRef = useRef();
  const count = 75;

  const [positions, speeds] = useMemo(() => {
    const pos = new Float32Array(count * 3);
    const spd = new Float32Array(count);
    for (let i = 0; i < count; i++) {
      pos[i * 3] = (Math.random() - 0.5) * 0.09; // X
      pos[i * 3 + 1] = 0.14 + (Math.random() - 0.5) * 0.09; // Y (thruster height)
      pos[i * 3 + 2] = (Math.random() - 0.5) * 0.45; // Z
      spd[i] = 0.5 + Math.random() * 0.5;
    }
    return [pos, spd];
  }, [count]);

  useFrame((_, delta) => {
    if (!particlesRef.current || Math.abs(rpm) < 100) return;

    const posAttr = particlesRef.current.geometry.attributes.position;
    const array = posAttr.array;
    const flowDirection = rpm >= 0 ? 1 : -1;
    const flowVelocity = (Math.abs(rpm) / 3800) * 1.5;

    for (let i = 0; i < count; i++) {
      array[i * 3 + 2] += flowDirection * flowVelocity * speeds[i] * delta;

      // Wrap around
      if (flowDirection > 0 && array[i * 3 + 2] > 0.3) {
        array[i * 3 + 2] = -0.25;
      } else if (flowDirection < 0 && array[i * 3 + 2] < -0.3) {
        array[i * 3 + 2] = 0.25;
      }
    }
    posAttr.needsUpdate = true;
  });

  if (Math.abs(rpm) < 100) return null;

  return (
    <points ref={particlesRef}>
      <bufferGeometry>
        <bufferAttribute
          attach="attributes-position"
          args={[positions, 3]}
        />
      </bufferGeometry>
      <pointsMaterial
        size={0.007}
        color={rpm >= 0 ? '#38bdf8' : '#fb923c'}
        transparent
        opacity={Math.min(0.85, Math.abs(rpm) / 2500)}
        blending={THREE.AdditiveBlending}
      />
    </points>
  );
}

export default function SingleThrusterScene({ rpm = 0, thrust = 0, targetRpm = 0 }) {
  return (
    <div style={{ width: '100%', height: '100%', position: 'relative', background: '#0a0e17' }}>
      <Canvas shadows antialias="true">
        <PerspectiveCamera makeDefault position={[0.35, 0.28, 0.42]} fov={45} />
        <OrbitControls
          enableDamping
          dampingFactor={0.08}
          minDistance={0.15}
          maxDistance={1.2}
          target={[0, 0.11, 0]}
        />

        {/* Studio Subsea Lighting */}
        <ambientLight intensity={0.4} />
        <directionalLight position={[0.5, 0.8, 0.6]} intensity={1.2} castShadow />
        <pointLight position={[-0.4, 0.3, -0.3]} intensity={0.6} color="#00f0ff" />
        <pointLight position={[0, -0.1, 0.3]} intensity={0.5} color="#3b82f6" />

        <Center top>
          <T200PhysicalThruster rpm={rpm} thrust={thrust} />
          <TestStandBench thrust={thrust} />
          <FluidCavitationStream rpm={rpm} />
        </Center>

        {/* Precision Subsea Testbed Grid Floor */}
        <Grid
          position={[0, -0.06, 0]}
          args={[2.0, 2.0]}
          cellSize={0.05}
          cellThickness={0.8}
          cellColor="#1e293b"
          sectionSize={0.2}
          sectionThickness={1.2}
          sectionColor="#00f0ff"
          fadeDistance={1.5}
        />
      </Canvas>

      {/* Floating 3D HUD Badges */}
      <div
        style={{
          position: 'absolute',
          top: '12px',
          left: '14px',
          display: 'flex',
          gap: '8px',
          pointerEvents: 'none',
        }}
      >
        <div
          style={{
            background: 'rgba(15, 23, 42, 0.75)',
            backdropFilter: 'blur(8px)',
            border: '1px solid rgba(0, 240, 255, 0.3)',
            borderRadius: '6px',
            padding: '4px 10px',
            fontSize: '0.72rem',
            color: '#00f0ff',
            fontFamily: 'monospace',
            fontWeight: 'bold',
          }}
        >
          RPM: {Math.round(rpm)}
        </div>

        <div
          style={{
            background: 'rgba(15, 23, 42, 0.75)',
            backdropFilter: 'blur(8px)',
            border: `1px solid ${thrust >= 0 ? 'rgba(0, 255, 136, 0.3)' : 'rgba(255, 140, 0, 0.3)'}`,
            borderRadius: '6px',
            padding: '4px 10px',
            fontSize: '0.72rem',
            color: thrust >= 0 ? '#00ff88' : '#ff8c00',
            fontFamily: 'monospace',
            fontWeight: 'bold',
          }}
        >
          THRUST: {thrust.toFixed(2)} N ({((thrust / 9.81) * 1000).toFixed(0)} gf)
        </div>
      </div>
    </div>
  );
}
