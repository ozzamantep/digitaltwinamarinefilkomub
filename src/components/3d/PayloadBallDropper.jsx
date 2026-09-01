import { useRef, useEffect } from 'react';
import { useFrame } from '@react-three/fiber';
import useVehicleStore from '../../store/vehicleStore';

export default function PayloadBallDropper() {
  const ballRef = useRef();
  const position = useVehicleStore((s) => s.position);
  const payloadState = useVehicleStore((s) => s.payloadState || { loaded: true, dropped: false, onFloor: false, grasped: false, inDrum: false, x: 10.5, y: 0.22, z: 1.5 });

  useFrame((_, delta) => {
    if (!ballRef.current) return;

    if (payloadState.grasped) {
      // Clamped in gripper jaws (rendered attached in BlueROV2Model)
      ballRef.current.visible = false;
    } else if (payloadState.onFloor) {
      // Resting on pool floor next to target drum
      ballRef.current.position.set(payloadState.x, 0.08, payloadState.z);
      ballRef.current.visible = true;
    } else if (payloadState.inDrum) {
      // Inside red drum bucket
      ballRef.current.position.set(payloadState.x, 0.20, payloadState.z);
      ballRef.current.visible = true;
    } else if (payloadState.dropped) {
      // Sinking down
      if (ballRef.current.position.y > (payloadState.onFloor ? 0.08 : 0.20)) {
        ballRef.current.position.y -= Math.max(0.1, delta * 0.85);
      } else {
        ballRef.current.position.set(payloadState.x, payloadState.onFloor ? 0.08 : 0.20, payloadState.z);
      }
      ballRef.current.visible = true;
    } else {
      // Loaded under AUV chassis
      const subX = position?.x ?? -10;
      const subY = position?.y ?? 1.1;
      const subZ = position?.z ?? 0;
      ballRef.current.position.set(subX + 0.15, subY - 0.11, subZ);
      ballRef.current.visible = true;
    }
  });

  return (
    <group ref={ballRef}>
      {/* Official SAUVC Red Competition Ball Payload */}
      <mesh castShadow receiveShadow>
        <sphereGeometry args={[0.042, 24, 24]} />
        <meshStandardMaterial
          color="#ef4444"
          roughness={0.25}
          metalness={0.15}
          emissive="#b91c1c"
          emissiveIntensity={0.4}
        />
      </mesh>
      {/* Subtle glowing beacon */}
      <pointLight color="#ef4444" intensity={0.5} distance={0.9} />
    </group>
  );
}
