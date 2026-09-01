import { useRef, useMemo } from 'react';
import { useFrame } from '@react-three/fiber';
import * as THREE from 'three';
import useVehicleStore from '../../store/vehicleStore';

export default function ThrusterBubbles() {
  const pointsRef = useRef();
  const speed = useVehicleStore((s) => s.speed);
  const position = useVehicleStore((s) => s.position);
  const orientation = useVehicleStore((s) => s.orientation);
  const armed = useVehicleStore((s) => s.armed);

  const particleCount = 120;

  const [positions, velocities, life] = useMemo(() => {
    const pos = new Float32Array(particleCount * 3);
    const vel = [];
    const lifeArr = new Float32Array(particleCount);

    for (let i = 0; i < particleCount; i++) {
      pos[i * 3] = 0;
      pos[i * 3 + 1] = -10;
      pos[i * 3 + 2] = 0;
      vel.push({ x: 0, y: 0, z: 0 });
      lifeArr[i] = Math.random();
    }
    return [pos, vel, lifeArr];
  }, []);

  useFrame((state, delta) => {
    if (!pointsRef.current) return;
    const posAttr = pointsRef.current.geometry.attributes.position;
    const arr = posAttr.array;

    const totalSpeed = Math.sqrt(
      (speed?.surge || 0) ** 2 + (speed?.sway || 0) ** 2 + (speed?.heave || 0) ** 2
    );
    const isEmitting = armed && totalSpeed > 0.08;

    const ox = isFinite(orientation?.x) ? orientation.x : 0;
    const oy = isFinite(orientation?.y) ? orientation.y : 0;
    const oz = isFinite(orientation?.z) ? orientation.z : 0;
    const ow = isFinite(orientation?.w) ? orientation.w : 1;

    const quat = new THREE.Quaternion(ox, oy, oz, ow);

    const px = isFinite(position?.x) ? position.x : -11;
    const py = isFinite(position?.y) ? position.y : 1.2;
    const pz = isFinite(position?.z) ? position.z : 2;

    for (let i = 0; i < particleCount; i++) {
      life[i] += delta * 1.8;

      if (life[i] > 1.0) {
        if (isEmitting && Math.random() < 0.65) {
          life[i] = 0;
          // Spawn at rear (-X in local frame)
          const localOffset = new THREE.Vector3(
            -0.26 + (Math.random() - 0.5) * 0.08,
            (Math.random() - 0.5) * 0.1,
            (Math.random() - 0.5) * 0.28
          ).applyQuaternion(quat);

          arr[i * 3] = px + localOffset.x;
          arr[i * 3 + 1] = py + localOffset.y;
          arr[i * 3 + 2] = pz + localOffset.z;

          const streamVel = new THREE.Vector3(
            -(speed?.surge || 0) * 0.5 + (Math.random() - 0.5) * 0.08,
            Math.random() * 0.08 + 0.04,
            -(speed?.sway || 0) * 0.5 + (Math.random() - 0.5) * 0.08
          );

          velocities[i] = streamVel;
        } else {
          arr[i * 3 + 1] = -10;
        }
      } else {
        arr[i * 3] += (velocities[i]?.x || 0) * delta;
        arr[i * 3 + 1] += (velocities[i]?.y || 0) * delta;
        arr[i * 3 + 2] += (velocities[i]?.z || 0) * delta;
      }
    }

    posAttr.needsUpdate = true;
  });

  return (
    <points ref={pointsRef}>
      <bufferGeometry>
        <bufferAttribute
          attach="attributes-position"
          count={particleCount}
          array={positions}
          itemSize={3}
        />
      </bufferGeometry>
      <pointsMaterial
        size={0.04}
        color="#e0f2fe"
        transparent
        opacity={0.75}
        blending={THREE.AdditiveBlending}
        depthWrite={false}
      />
    </points>
  );
}
