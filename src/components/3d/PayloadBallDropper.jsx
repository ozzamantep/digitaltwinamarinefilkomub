import { useRef, useEffect } from 'react';
import { useFrame } from '@react-three/fiber';
import useVehicleStore from '../../store/vehicleStore';

export default function PayloadBallDropper() {
  const ballRef = useRef();
  const position = useVehicleStore((s) => s.position);
  const payloadState = useVehicleStore((s) => s.payloadState || { dropped: false, x: 8.0, y: 1.2, z: 2.0, vy: 0 });

  useFrame((_, delta) => {
    if (!ballRef.current) return;

    if (!payloadState.dropped) {
      // Attached to bottom skid rails of BlueROV2
      const subX = position?.x ?? -10;
      const subY = position?.y ?? 1.1;
      const subZ = position?.z ?? 0;

      ballRef.current.position.set(subX, subY - 0.12, subZ);
      ballRef.current.visible = true;
    } else {
      // Physically sinking under water gravity into Red Drum (Floor at Y = 0.22)
      if (ballRef.current.position.y > 0.22) {
        ballRef.current.position.y -= Math.max(0.2, delta * 0.85);
      } else {
        ballRef.current.position.set(payloadState.x, 0.22, payloadState.z);
      }
    }
  });

  return (
    <group ref={ballRef}>
      {/* High-visibility Fluorescent Yellow Competition Golf Ball Payload */}
      <mesh castShadow receiveShadow>
        <sphereGeometry args={[0.045, 24, 24]} />
        <meshStandardMaterial
          color="#eab308"
          roughness={0.2}
          metalness={0.3}
          emissive="#ca8a04"
          emissiveIntensity={0.4}
        />
      </mesh>
      {/* Subtle glowing halo */}
      <pointLight color="#facc15" intensity={0.4} distance={0.8} />
    </group>
  );
}
