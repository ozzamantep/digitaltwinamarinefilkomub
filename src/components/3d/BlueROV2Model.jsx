import { useRef, useMemo } from 'react';
import { useFrame } from '@react-three/fiber';
import * as THREE from 'three';
import useVehicleStore from '../../store/vehicleStore';

/**
 * Authentic BlueRobotics T200 3-Blade Subsea Propeller Assembly
 * Rotates strictly around its central shaft axis.
 */
function T200ThrusterUnit({ propRef, color = '#0284c7', isVertical = false }) {
  // Generate 3 Pitch-Cambered Hydrodynamic Propeller Blades
  const bladeGeometry = useMemo(() => {
    const geom = new THREE.BufferGeometry();
    const vertices = new Float32Array([
      0.0, 0.005, 0.003,
      0.010, 0.028, -0.004,
      -0.007, 0.030, 0.004,

      0.0, 0.005, -0.003,
      -0.007, 0.030, 0.004,
      0.010, 0.028, -0.004,
    ]);
    geom.setAttribute('position', new THREE.BufferAttribute(vertices, 3));
    geom.computeVertexNormals();
    return geom;
  }, []);

  return (
    <group>
      {/* 1. Outer Cylindrical Duct Shroud (Black Polycarbonate) */}
      <mesh rotation={isVertical ? [0, 0, 0] : [Math.PI / 2, 0, 0]}>
        <cylinderGeometry args={[0.038, 0.038, 0.032, 24, 1, true]} />
        <meshStandardMaterial color="#0f172a" roughness={0.4} metalness={0.6} side={THREE.DoubleSide} />
      </mesh>

      {/* Aerodynamic Duct Lip Ring */}
      <mesh rotation={isVertical ? [0, 0, 0] : [Math.PI / 2, 0, 0]}>
        <torusGeometry args={[0.038, 0.003, 12, 24]} />
        <meshStandardMaterial color="#1e293b" roughness={0.3} metalness={0.7} />
      </mesh>

      {/* Stator Vanes / Support Brackets inside duct */}
      {[0, Math.PI * 0.66, Math.PI * 1.33].map((angle, idx) => (
        <mesh
          key={idx}
          position={[0, 0, 0]}
          rotation={isVertical ? [0, angle, 0] : [Math.PI / 2, angle, 0]}
        >
          <boxGeometry args={[0.003, 0.020, 0.036]} />
          <meshStandardMaterial color="#1e293b" metalness={0.7} />
        </mesh>
      ))}

      {/* Base Mounting Foot with Teal Accent Bracket */}
      <group position={[0, isVertical ? 0 : -0.041, 0]}>
        <mesh position={[0, -0.004, 0]}>
          <boxGeometry args={[0.018, 0.008, 0.022]} />
          <meshStandardMaterial color="#0f172a" roughness={0.3} metalness={0.8} />
        </mesh>
        <mesh position={[0, -0.006, 0]}>
          <boxGeometry args={[0.014, 0.003, 0.016]} />
          <meshStandardMaterial color="#00f0ff" emissive="#00f0ff" emissiveIntensity={0.6} />
        </mesh>
      </group>

      {/* 2. Rotating Propeller & Hub Assembly */}
      <group ref={propRef}>
        {/* Central Bullet Nosecone Hub */}
        <mesh rotation={isVertical ? [0, 0, 0] : [Math.PI / 2, 0, 0]}>
          <cylinderGeometry args={[0.008, 0.010, 0.022, 16]} />
          <meshStandardMaterial color="#0f172a" roughness={0.25} metalness={0.8} />
        </mesh>
        {/* Aerodynamic Bullet Tip */}
        <mesh
          position={isVertical ? [0, 0.012, 0] : [0, 0, 0.012]}
          rotation={isVertical ? [0, 0, 0] : [Math.PI / 2, 0, 0]}
        >
          <coneGeometry args={[0.008, 0.012, 16]} />
          <meshStandardMaterial color="#0284c7" roughness={0.3} metalness={0.7} />
        </mesh>

        {/* 3x Pitch-Cambered Hydrofoil Blades */}
        {[0, (Math.PI * 2) / 3, (Math.PI * 4) / 3].map((angle, i) => (
          <group key={i} rotation={isVertical ? [0, angle, 0] : [0, 0, angle]}>
            <mesh geometry={bladeGeometry} rotation={isVertical ? [0, 0, Math.PI / 2] : [0, 0, 0]}>
              <meshStandardMaterial
                color={color}
                roughness={0.2}
                metalness={0.6}
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
 * Custom Competition AUV 3D CAD Model
 * Exact match to user CAD orthographic and perspective diagrams:
 * - 4x Horizontal 45° Vectored Thrusters at corners
 * - 2x Vertical Ducted Thrusters in Front/Rear Yellow Cowlings
 * - Dual Stacked Carbon Fiber Battery Cylinders on Port and Starboard (4 tubes total)
 * - Clear Acrylic Electronics Enclosure with Internal Motherboard & Brackets
 * - Aerodynamic High-Visibility Yellow Front & Rear Nose Fairings
 * - Translucent Polycarbonate Upper and Lower Deck Plates
 */
export default function BlueROV2Model({ onFrame }) {
  const groupRef = useRef();

  // 6 Thruster Propeller Refs (4 Horizontal + 2 Vertical)
  const propRefs = [
    useRef(), // 0: Front-Left Horiz (45°)
    useRef(), // 1: Front-Right Horiz (-45°)
    useRef(), // 2: Rear-Left Horiz (135°)
    useRef(), // 3: Rear-Right Horiz (-135°)
    useRef(), // 4: Front-Vertical Ducted
    useRef(), // 5: Rear-Vertical Ducted
  ];

  const position = useVehicleStore((s) => s.position);
  const orientation = useVehicleStore((s) => s.orientation);
  const lightsIntensity = useVehicleStore((s) => s.lightsIntensity);
  const speed = useVehicleStore((s) => s.speed);
  const thrusters = useVehicleStore((s) => s.thrusters);
  const armed = useVehicleStore((s) => s.armed);

  // Carbon fiber weave texture simulation material
  const carbonMaterial = useMemo(() => {
    return new THREE.MeshStandardMaterial({
      color: '#18181b',
      roughness: 0.35,
      metalness: 0.65,
      bumpScale: 0.05,
    });
  }, []);

  // Translucent Acrylic / Polycarbonate Material
  const acrylicMaterial = useMemo(() => {
    return new THREE.MeshPhysicalMaterial({
      color: '#e2e8f0',
      transparent: true,
      opacity: 0.45,
      roughness: 0.12,
      metalness: 0.15,
      transmission: 0.65,
      ior: 1.49,
      side: THREE.DoubleSide,
      depthWrite: false,
    });
  }, []);

  // Marine Competition Yellow High-Gloss Cowling Material
  const yellowCowlMaterial = useMemo(() => {
    return new THREE.MeshStandardMaterial({
      color: '#facc15',
      emissive: '#ca8a04',
      emissiveIntensity: 0.25,
      roughness: 0.22,
      metalness: 0.35,
    });
  }, []);

  // Thin Sleek Aerodynamic Yellow Cowl Visor (Thickness: 8mm)
  const cowlVisorGeometry = useMemo(() => {
    const shape = new THREE.Shape();
    const baseW = 0.062;
    const noseL = 0.095;

    shape.moveTo(0, -baseW);
    shape.bezierCurveTo(noseL * 0.4, -baseW * 0.9, noseL * 0.8, -baseW * 0.4, noseL, 0);
    shape.bezierCurveTo(noseL * 0.8, baseW * 0.4, noseL * 0.4, baseW * 0.9, 0, baseW);
    shape.lineTo(0, -baseW);

    // Circular through-duct cutout
    const hole = new THREE.Path();
    const ductCenterX = 0.036;
    const ductRadius = 0.036;
    hole.absarc(ductCenterX, 0, ductRadius, 0, Math.PI * 2, true);
    shape.holes.push(hole);

    const extrudeSettings = {
      steps: 1,
      depth: 0.007, // Thin 7mm sheet visor
      bevelEnabled: true,
      bevelThickness: 0.003,
      bevelSize: 0.003,
      bevelSegments: 4,
    };

    const geom = new THREE.ExtrudeGeometry(shape, extrudeSettings);
    geom.center();
    geom.rotateX(Math.PI / 2);
    geom.computeVertexNormals();
    return geom;
  }, []);

  useFrame((state, delta) => {
    if (onFrame) onFrame();

    const t = state.clock.elapsedTime;
    const bobY = Math.sin(t * 1.5) * 0.006;

    if (groupRef.current) {
      groupRef.current.position.lerp(
        new THREE.Vector3(position.x, position.y + bobY, position.z),
        0.18
      );

      const targetQuat = new THREE.Quaternion(
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w
      );
      groupRef.current.quaternion.slerp(targetQuat, 0.18);
    }

    // Dynamic Propeller Spins - strictly driven by active thruster efforts
    propRefs.forEach((ref, idx) => {
      if (ref.current) {
        if (armed) {
          const effort = thrusters[idx] || 0;
          if (Math.abs(effort) > 2) {
            // Spin speed strictly proportional to actual thruster effort (-100% to +100%)
            const rpm = (effort / 100) * 32.0 * delta;
            
            if (idx >= 4) {
              // 4 & 5: Vertical thrusters in yellow cowls rotate around vertical Y-axis
              ref.current.rotation.y += rpm;
            } else {
              // 0, 1, 2, 3: Horizontal corner thrusters rotate around local shaft Z-axis
              ref.current.rotation.z += rpm;
            }
          }
        }
      }
    });
  });

  const lightPower = lightsIntensity / 100;

  return (
    <group ref={groupRef} position={[position.x, position.y, position.z]}>
      {/* ================================================================= */}
      {/* 1. CENTRAL MAIN PRESSURE HULL & INTERNAL ELECTRONICS BOX         */}
      {/* ================================================================= */}
      {/* White Main Cylindrical Core Hull */}
      <mesh position={[0, 0, 0]} rotation={[0, 0, Math.PI / 2]} castShadow receiveShadow>
        <cylinderGeometry args={[0.075, 0.075, 0.28, 32]} />
        <meshStandardMaterial color="#f8fafc" roughness={0.3} metalness={0.2} />
      </mesh>
      {/* Front & Rear Core Endcaps */}
      <mesh position={[0.14, 0, 0]} rotation={[0, 0, -Math.PI / 2]}>
        <cylinderGeometry args={[0.076, 0.074, 0.012, 32]} />
        <meshStandardMaterial color="#334155" metalness={0.8} />
      </mesh>
      <mesh position={[-0.14, 0, 0]} rotation={[0, 0, Math.PI / 2]}>
        <cylinderGeometry args={[0.076, 0.074, 0.012, 32]} />
        <meshStandardMaterial color="#334155" metalness={0.8} />
      </mesh>

      {/* Top Rectangular Electronics Enclosure (Acrylic / Glass Casing) */}
      <group position={[0, 0.065, 0]}>
        {/* Transparent Outer Box */}
        <mesh material={acrylicMaterial} castShadow>
          <boxGeometry args={[0.22, 0.062, 0.19]} />
        </mesh>
        <mesh>
          <boxGeometry args={[0.222, 0.064, 0.192]} />
          <meshBasicMaterial color="#38bdf8" wireframe transparent opacity={0.35} />
        </mesh>

        {/* Internal Jetson / Flight Controller Motherboard (Green PCB) */}
        <mesh position={[0, -0.012, 0]}>
          <boxGeometry args={[0.16, 0.006, 0.14]} />
          <meshStandardMaterial color="#065f46" roughness={0.4} metalness={0.5} />
        </mesh>
        {/* Jetson Aluminum Heatsink Block */}
        <mesh position={[-0.02, 0.008, 0]}>
          <boxGeometry args={[0.06, 0.018, 0.06]} />
          <meshStandardMaterial color="#0f172a" metalness={0.9} roughness={0.2} />
        </mesh>
        {/* Blinking / Glowing Circuit Status LEDs */}
        <mesh position={[0.05, 0.002, 0.04]}>
          <boxGeometry args={[0.008, 0.006, 0.008]} />
          <meshBasicMaterial color="#00ff88" />
        </mesh>
        <mesh position={[0.05, 0.002, -0.04]}>
          <boxGeometry args={[0.008, 0.006, 0.008]} />
          <meshBasicMaterial color="#00f0ff" />
        </mesh>
        <mesh position={[0.02, 0.002, 0.05]}>
          <boxGeometry args={[0.006, 0.006, 0.006]} />
          <meshBasicMaterial color="#f59e0b" />
        </mesh>

        {/* Internal Wiring Bus (Red/Black Subsea Power Lines) */}
        <mesh position={[0, -0.004, 0.03]} rotation={[0, 0, Math.PI / 2]}>
          <cylinderGeometry args={[0.0025, 0.0025, 0.18, 12]} />
          <meshStandardMaterial color="#ef4444" roughness={0.5} />
        </mesh>
        <mesh position={[0, -0.004, -0.03]} rotation={[0, 0, Math.PI / 2]}>
          <cylinderGeometry args={[0.0025, 0.0025, 0.18, 12]} />
          <meshStandardMaterial color="#0f172a" roughness={0.5} />
        </mesh>

        {/* Angled Mounting Brackets */}
        <mesh position={[0.07, 0.01, 0.08]} rotation={[0, 0, -0.3]}>
          <boxGeometry args={[0.008, 0.045, 0.012]} />
          <meshStandardMaterial color="#cbd5e1" metalness={0.7} />
        </mesh>
        <mesh position={[-0.07, 0.01, 0.08]} rotation={[0, 0, 0.3]}>
          <boxGeometry args={[0.008, 0.045, 0.012]} />
          <meshStandardMaterial color="#cbd5e1" metalness={0.7} />
        </mesh>
      </group>

      {/* Top Acrylic Clear Mounting Plate */}
      <mesh position={[0, 0.10, 0]} material={acrylicMaterial}>
        <boxGeometry args={[0.30, 0.006, 0.22]} />
      </mesh>

      {/* ================================================================= */}
      {/* 2. DUAL STACKED CARBON FIBER BATTERY TUBES (4 TUBES TOTAL)        */}
      {/* ================================================================= */}
      {/* Port Side (Left: +Z) Stacked Tubes */}
      <group position={[0, 0, 0.118]}>
        <mesh position={[0, 0.026, 0]} rotation={[0, 0, Math.PI / 2]} material={carbonMaterial} castShadow>
          <cylinderGeometry args={[0.024, 0.024, 0.30, 24]} />
        </mesh>
        <mesh position={[0.15, 0.026, 0]} rotation={[0, 0, -Math.PI / 2]}>
          <cylinderGeometry args={[0.025, 0.023, 0.01, 24]} />
          <meshStandardMaterial color="#334155" metalness={0.8} />
        </mesh>
        <mesh position={[-0.15, 0.026, 0]} rotation={[0, 0, Math.PI / 2]}>
          <cylinderGeometry args={[0.025, 0.023, 0.01, 24]} />
          <meshStandardMaterial color="#334155" metalness={0.8} />
        </mesh>

        <mesh position={[0, -0.026, 0]} rotation={[0, 0, Math.PI / 2]} material={carbonMaterial} castShadow>
          <cylinderGeometry args={[0.024, 0.024, 0.30, 24]} />
        </mesh>
        <mesh position={[0.15, -0.026, 0]} rotation={[0, 0, -Math.PI / 2]}>
          <cylinderGeometry args={[0.025, 0.023, 0.01, 24]} />
          <meshStandardMaterial color="#334155" metalness={0.8} />
        </mesh>
        <mesh position={[-0.15, -0.026, 0]} rotation={[0, 0, Math.PI / 2]}>
          <cylinderGeometry args={[0.025, 0.023, 0.01, 24]} />
          <meshStandardMaterial color="#334155" metalness={0.8} />
        </mesh>
      </group>

      {/* Starboard Side (Right: -Z) Stacked Tubes */}
      <group position={[0, 0, -0.118]}>
        <mesh position={[0, 0.026, 0]} rotation={[0, 0, Math.PI / 2]} material={carbonMaterial} castShadow>
          <cylinderGeometry args={[0.024, 0.024, 0.30, 24]} />
        </mesh>
        <mesh position={[0.15, 0.026, 0]} rotation={[0, 0, -Math.PI / 2]}>
          <cylinderGeometry args={[0.025, 0.023, 0.01, 24]} />
          <meshStandardMaterial color="#334155" metalness={0.8} />
        </mesh>
        <mesh position={[-0.15, 0.026, 0]} rotation={[0, 0, Math.PI / 2]}>
          <cylinderGeometry args={[0.025, 0.023, 0.01, 24]} />
          <meshStandardMaterial color="#334155" metalness={0.8} />
        </mesh>

        <mesh position={[0, -0.026, 0]} rotation={[0, 0, Math.PI / 2]} material={carbonMaterial} castShadow>
          <cylinderGeometry args={[0.024, 0.024, 0.30, 24]} />
        </mesh>
        <mesh position={[0.15, -0.026, 0]} rotation={[0, 0, -Math.PI / 2]}>
          <cylinderGeometry args={[0.025, 0.023, 0.01, 24]} />
          <meshStandardMaterial color="#334155" metalness={0.8} />
        </mesh>
        <mesh position={[-0.15, -0.026, 0]} rotation={[0, 0, Math.PI / 2]}>
          <cylinderGeometry args={[0.025, 0.023, 0.01, 24]} />
          <meshStandardMaterial color="#334155" metalness={0.8} />
        </mesh>
      </group>

      {/* ================================================================= */}
      {/* 3. FRONT & REAR AERODYNAMIC YELLOW VISORS & VERTICAL THRUSTERS    */}
      {/* ================================================================= */}
      {/* FRONT NOSE SECTION (+X = 0.18) */}
      <group position={[0.18, 0, 0]}>
        {/* Top Yellow Aerodynamic Visor Hood (Thin 8mm plate) */}
        <mesh position={[0, 0.054, 0]} geometry={cowlVisorGeometry} material={yellowCowlMaterial} castShadow />

        {/* Bottom Yellow Skid Lip (Thin 8mm plate) */}
        <mesh position={[0, -0.054, 0]} geometry={cowlVisorGeometry} material={yellowCowlMaterial} castShadow />

        {/* Central Black Vertical Duct Cylinder */}
        <mesh position={[0.005, 0, 0]}>
          <cylinderGeometry args={[0.036, 0.036, 0.108, 32, 1, true]} />
          <meshStandardMaterial color="#0f172a" roughness={0.3} metalness={0.8} side={THREE.DoubleSide} />
        </mesh>
        {/* Upper & Lower Duct Lip Torus Rings */}
        <mesh position={[0.005, 0.054, 0]}>
          <torusGeometry args={[0.036, 0.003, 16, 32]} />
          <meshStandardMaterial color="#0f172a" roughness={0.2} metalness={0.8} />
        </mesh>
        <mesh position={[0.005, -0.054, 0]}>
          <torusGeometry args={[0.036, 0.003, 16, 32]} />
          <meshStandardMaterial color="#0f172a" roughness={0.2} metalness={0.8} />
        </mesh>

        {/* Front Acrylic Dome Lens at Nose Tip */}
        <mesh position={[0.054, 0, 0]} rotation={[0, 0, -Math.PI / 2]}>
          <sphereGeometry args={[0.016, 16, 16, 0, Math.PI * 2, 0, Math.PI * 0.5]} />
          <meshPhysicalMaterial
            color="#bae6fd"
            transparent
            opacity={0.85}
            roughness={0.05}
            transmission={0.9}
            ior={1.49}
          />
        </mesh>

        {/* 5. FRONT VERTICAL HEAVE THRUSTER (Inside vertical duct facing UPWARDS) */}
        <group position={[0.005, 0, 0]}>
          <T200ThrusterUnit propRef={propRefs[4]} color="#0284c7" isVertical={true} />
        </group>
      </group>

      {/* REAR TAIL SECTION (-X = -0.18) */}
      <group position={[-0.18, 0, 0]} rotation={[0, Math.PI, 0]}>
        {/* Top Yellow Aerodynamic Visor Hood (Thin 8mm plate) */}
        <mesh position={[0, 0.054, 0]} geometry={cowlVisorGeometry} material={yellowCowlMaterial} castShadow />

        {/* Bottom Yellow Skid Lip (Thin 8mm plate) */}
        <mesh position={[0, -0.054, 0]} geometry={cowlVisorGeometry} material={yellowCowlMaterial} castShadow />

        {/* Central Black Vertical Duct Cylinder */}
        <mesh position={[0.005, 0, 0]}>
          <cylinderGeometry args={[0.036, 0.036, 0.108, 32, 1, true]} />
          <meshStandardMaterial color="#0f172a" roughness={0.3} metalness={0.8} side={THREE.DoubleSide} />
        </mesh>
        {/* Upper & Lower Duct Lip Torus Rings */}
        <mesh position={[0.005, 0.054, 0]}>
          <torusGeometry args={[0.036, 0.003, 16, 32]} />
          <meshStandardMaterial color="#0f172a" roughness={0.2} metalness={0.8} />
        </mesh>
        <mesh position={[0.005, -0.054, 0]}>
          <torusGeometry args={[0.036, 0.003, 16, 32]} />
          <meshStandardMaterial color="#0f172a" roughness={0.2} metalness={0.8} />
        </mesh>

        {/* 6. REAR VERTICAL HEAVE THRUSTER (Inside vertical duct facing UPWARDS) */}
        <group position={[0.005, 0, 0]}>
          <T200ThrusterUnit propRef={propRefs[5]} color="#0284c7" isVertical={true} />
        </group>
      </group>

      {/* ================================================================= */}
      {/* 4. TRANSPARENT ACRYLIC OUTER FAIRING PLATES (TOP & BOTTOM DECKS) */}
      {/* ================================================================= */}
      {/* Upper Translucent Deck Plate with Rounded Corner Fillets */}
      <group position={[0, 0.06, 0]}>
        <mesh material={acrylicMaterial}>
          <boxGeometry args={[0.54, 0.008, 0.28]} />
        </mesh>
      </group>
      {/* Lower Translucent Deck Plate */}
      <group position={[0, -0.06, 0]}>
        <mesh material={acrylicMaterial}>
          <boxGeometry args={[0.54, 0.008, 0.28]} />
        </mesh>
      </group>

      {/* Translucent Side Vertical Hull Panels */}
      <mesh position={[0, 0, 0.142]} material={acrylicMaterial}>
        <boxGeometry args={[0.52, 0.118, 0.006]} />
      </mesh>
      <mesh position={[0, 0, -0.142]} material={acrylicMaterial}>
        <boxGeometry args={[0.52, 0.118, 0.006]} />
      </mesh>

      {/* ================================================================= */}
      {/* 5. 4x VECTORED HORIZONTAL THRUSTERS AT 45° CORNERS               */}
      {/* ================================================================= */}
      {/* 0. Front-Left Thruster (X = +0.20, Z = +0.10, Angled at +45°) */}
      <group position={[0.20, 0, 0.105]} rotation={[0, -Math.PI / 4, 0]}>
        <T200ThrusterUnit propRef={propRefs[0]} color="#0284c7" />
      </group>

      {/* 1. Front-Right Thruster (X = +0.20, Z = -0.10, Angled at -45°) */}
      <group position={[0.20, 0, -0.105]} rotation={[0, Math.PI / 4, 0]}>
        <T200ThrusterUnit propRef={propRefs[1]} color="#0284c7" />
      </group>

      {/* 2. Rear-Left Thruster (X = -0.20, Z = +0.10, Angled at +135°) */}
      <group position={[-0.20, 0, 0.105]} rotation={[0, (-Math.PI * 3) / 4, 0]}>
        <T200ThrusterUnit propRef={propRefs[2]} color="#0284c7" />
      </group>

      {/* 3. Rear-Right Thruster (X = -0.20, Z = -0.10, Angled at -135°) */}
      <group position={[-0.20, 0, -0.105]} rotation={[0, (Math.PI * 3) / 4, 0]}>
        <T200ThrusterUnit propRef={propRefs[3]} color="#0284c7" />
      </group>

      {/* ================================================================= */}
      {/* 6. FORWARD SUBSEA HEADLIGHTS & ILLUMINATION                       */}
      {/* ================================================================= */}
      <group position={[0.26, 0.03, 0]}>
        {/* Twin Mini Lumen Lights */}
        <mesh position={[0, 0.02, 0.045]} rotation={[0, 0, Math.PI / 2]}>
          <cylinderGeometry args={[0.012, 0.012, 0.016, 16]} />
          <meshStandardMaterial color="#00f0ff" emissive="#00f0ff" emissiveIntensity={2.0 * lightPower} />
        </mesh>
        <mesh position={[0, 0.02, -0.045]} rotation={[0, 0, Math.PI / 2]}>
          <cylinderGeometry args={[0.012, 0.012, 0.016, 16]} />
          <meshStandardMaterial color="#00f0ff" emissive="#00f0ff" emissiveIntensity={2.0 * lightPower} />
        </mesh>

        {/* Forward Volumetric Spotlights */}
        <spotLight
          position={[0, 0.02, 0.045]}
          target-position={[5, -0.1, 0.1]}
          angle={0.65}
          penumbra={0.4}
          intensity={lightPower * 3.0}
          color="#a5f3fc"
          distance={12}
        />
        <spotLight
          position={[0, 0.02, -0.045]}
          target-position={[5, -0.1, -0.1]}
          angle={0.65}
          penumbra={0.4}
          intensity={lightPower * 3.0}
          color="#a5f3fc"
          distance={12}
        />

        {/* Volumetric Light Beams */}
        {lightPower > 0.05 && (
          <group>
            <mesh position={[0.9, 0, 0.045]} rotation={[0, 0, -Math.PI / 2]}>
              <coneGeometry args={[0.45, 1.8, 24, 1, true]} />
              <meshBasicMaterial
                color="#38bdf8"
                transparent
                opacity={0.12 * lightPower}
                blending={THREE.AdditiveBlending}
                side={THREE.DoubleSide}
                depthWrite={false}
              />
            </mesh>
            <mesh position={[0.9, 0, -0.045]} rotation={[0, 0, -Math.PI / 2]}>
              <coneGeometry args={[0.45, 1.8, 24, 1, true]} />
              <meshBasicMaterial
                color="#38bdf8"
                transparent
                opacity={0.12 * lightPower}
                blending={THREE.AdditiveBlending}
                side={THREE.DoubleSide}
                depthWrite={false}
              />
            </mesh>
          </group>
        )}
      </group>

      {/* Internal Hull Glow / Underglow */}
      <pointLight position={[0, 0.02, 0]} color="#38bdf8" intensity={0.8} distance={3.5} />
      <pointLight position={[0.22, 0, 0]} color="#facc15" intensity={0.4} distance={1.5} />
      <pointLight position={[-0.22, 0, 0]} color="#facc15" intensity={0.4} distance={1.5} />
    </group>
  );
}
