import { useRef, useMemo } from 'react';
import { useFrame } from '@react-three/fiber';
import * as THREE from 'three';
import useVehicleStore from '../../store/vehicleStore';

export default function VehicleModel({ onFrame }) {
  const groupRef = useRef();
  const wheelsRef = useRef([]);
  const headlightsRef = useRef();
  const bodyGlowRef = useRef();

  const position = useVehicleStore((s) => s.position);
  const orientation = useVehicleStore((s) => s.orientation);
  const speed = useVehicleStore((s) => s.speed);
  const battery = useVehicleStore((s) => s.battery);

  // Color based on battery level
  const bodyColor = useMemo(() => {
    if (battery.level > 60) return '#00f0ff';
    if (battery.level > 30) return '#ff8c00';
    return '#ff3b5c';
  }, [battery.level]);

  useFrame((state, delta) => {
    if (onFrame) onFrame();

    if (groupRef.current) {
      // Smooth position interpolation
      groupRef.current.position.lerp(
        new THREE.Vector3(position.x, 0.35, position.y),
        0.1
      );

      // Apply orientation from quaternion
      const targetQuat = new THREE.Quaternion(
        orientation.x,
        orientation.z,  // swap y/z for ROS->Three.js
        -orientation.y,
        orientation.w
      );
      groupRef.current.quaternion.slerp(targetQuat, 0.1);
    }

    // Rotate wheels based on speed
    wheelsRef.current.forEach((wheel) => {
      if (wheel) {
        wheel.rotation.x += speed.linear * delta * 3;
      }
    });

    // Pulsing headlights
    if (headlightsRef.current) {
      const pulse = 0.8 + Math.sin(state.clock.elapsedTime * 2) * 0.2;
      headlightsRef.current.intensity = pulse;
    }
  });

  const wheelGeometry = useMemo(() => new THREE.CylinderGeometry(0.25, 0.25, 0.15, 16), []);
  const wheelMaterial = useMemo(() => new THREE.MeshStandardMaterial({
    color: '#1a1a2e',
    roughness: 0.3,
    metalness: 0.8,
  }), []);

  const tireMaterial = useMemo(() => new THREE.MeshStandardMaterial({
    color: '#222222',
    roughness: 0.9,
    metalness: 0.1,
  }), []);

  return (
    <group ref={groupRef}>
      {/* Car Body - Main */}
      <mesh castShadow position={[0, 0, 0]}>
        <boxGeometry args={[1.2, 0.35, 2.4]} />
        <meshStandardMaterial
          color="#161b2e"
          roughness={0.2}
          metalness={0.9}
        />
      </mesh>

      {/* Car Body - Top/Cabin */}
      <mesh castShadow position={[0, 0.28, -0.1]}>
        <boxGeometry args={[1.0, 0.3, 1.2]} />
        <meshStandardMaterial
          color="#0d1117"
          roughness={0.1}
          metalness={0.95}
          transparent
          opacity={0.85}
        />
      </mesh>

      {/* Accent Stripe */}
      <mesh position={[0, 0.18, 0]}>
        <boxGeometry args={[1.22, 0.02, 2.42]} />
        <meshStandardMaterial
          color={bodyColor}
          emissive={bodyColor}
          emissiveIntensity={0.5}
          roughness={0.3}
          metalness={0.7}
        />
      </mesh>

      {/* Roof sensors (LiDAR dome) */}
      <mesh position={[0, 0.55, -0.1]}>
        <cylinderGeometry args={[0.15, 0.2, 0.12, 16]} />
        <meshStandardMaterial
          color="#00f0ff"
          emissive="#00f0ff"
          emissiveIntensity={0.6}
          roughness={0.2}
          metalness={0.8}
        />
      </mesh>

      {/* LiDAR spinning ring */}
      <mesh position={[0, 0.62, -0.1]} rotation={[0, 0, 0]}>
        <torusGeometry args={[0.12, 0.015, 8, 24]} />
        <meshStandardMaterial
          color="#00f0ff"
          emissive="#00f0ff"
          emissiveIntensity={1}
        />
      </mesh>

      {/* Front bumper */}
      <mesh position={[0, -0.05, 1.25]}>
        <boxGeometry args={[1.15, 0.2, 0.1]} />
        <meshStandardMaterial color="#1a1a2e" roughness={0.4} metalness={0.6} />
      </mesh>

      {/* Rear bumper */}
      <mesh position={[0, -0.05, -1.25]}>
        <boxGeometry args={[1.15, 0.2, 0.1]} />
        <meshStandardMaterial color="#1a1a2e" roughness={0.4} metalness={0.6} />
      </mesh>

      {/* Headlights */}
      <mesh position={[0.4, 0.05, 1.21]}>
        <boxGeometry args={[0.2, 0.08, 0.05]} />
        <meshStandardMaterial
          color="#ffffff"
          emissive="#ffffff"
          emissiveIntensity={1.5}
        />
      </mesh>
      <mesh position={[-0.4, 0.05, 1.21]}>
        <boxGeometry args={[0.2, 0.08, 0.05]} />
        <meshStandardMaterial
          color="#ffffff"
          emissive="#ffffff"
          emissiveIntensity={1.5}
        />
      </mesh>

      {/* Headlight beams */}
      <spotLight
        ref={headlightsRef}
        position={[0, 0.1, 1.3]}
        angle={0.6}
        penumbra={0.5}
        intensity={0.8}
        color="#f0f4ff"
        distance={15}
        target-position={[0, -0.5, 5]}
      />

      {/* Tail lights */}
      <mesh position={[0.4, 0.05, -1.21]}>
        <boxGeometry args={[0.2, 0.08, 0.05]} />
        <meshStandardMaterial
          color="#ff3b5c"
          emissive="#ff3b5c"
          emissiveIntensity={1}
        />
      </mesh>
      <mesh position={[-0.4, 0.05, -1.21]}>
        <boxGeometry args={[0.2, 0.08, 0.05]} />
        <meshStandardMaterial
          color="#ff3b5c"
          emissive="#ff3b5c"
          emissiveIntensity={1}
        />
      </mesh>

      {/* Wheels */}
      {[
        [0.7, -0.1, 0.7],    // Front-right
        [-0.7, -0.1, 0.7],   // Front-left
        [0.7, -0.1, -0.7],   // Rear-right
        [-0.7, -0.1, -0.7],  // Rear-left
      ].map((pos, i) => (
        <group key={i} position={pos}>
          {/* Tire */}
          <mesh
            ref={(el) => { wheelsRef.current[i] = el; }}
            rotation={[0, 0, Math.PI / 2]}
            castShadow
          >
            <cylinderGeometry args={[0.28, 0.28, 0.18, 20]} />
            <meshStandardMaterial color="#222222" roughness={0.9} metalness={0.1} />
          </mesh>
          {/* Rim */}
          <mesh rotation={[0, 0, Math.PI / 2]}>
            <cylinderGeometry args={[0.16, 0.16, 0.19, 8]} />
            <meshStandardMaterial color="#2a2a3e" roughness={0.2} metalness={0.9} />
          </mesh>
        </group>
      ))}

      {/* Side cameras */}
      <mesh position={[0.62, 0.25, 0.3]}>
        <boxGeometry args={[0.06, 0.06, 0.1]} />
        <meshStandardMaterial color="#00ff88" emissive="#00ff88" emissiveIntensity={0.5} />
      </mesh>
      <mesh position={[-0.62, 0.25, 0.3]}>
        <boxGeometry args={[0.06, 0.06, 0.1]} />
        <meshStandardMaterial color="#00ff88" emissive="#00ff88" emissiveIntensity={0.5} />
      </mesh>

      {/* Underglow */}
      <pointLight
        position={[0, -0.1, 0]}
        color={bodyColor}
        intensity={0.3}
        distance={3}
      />
    </group>
  );
}
