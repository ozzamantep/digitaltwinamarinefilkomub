import { useRef, useMemo } from 'react';
import { useFrame } from '@react-three/fiber';
import * as THREE from 'three';

export default function UnderwaterParticles({ count = 250 }) {
  const pointsRef = useRef();

  const [positions, velocities] = useMemo(() => {
    const pos = new Float32Array(count * 3);
    const vel = [];
    for (let i = 0; i < count; i++) {
      // distribute inside pool: X: -12..12, Y: 0.1..1.9, Z: -7.5..7.5
      pos[i * 3] = (Math.random() - 0.5) * 24;
      pos[i * 3 + 1] = Math.random() * 1.8 + 0.1;
      pos[i * 3 + 2] = (Math.random() - 0.5) * 15;

      vel.push({
        y: Math.random() * 0.005 + 0.002, // slow rise like micro bubbles
        x: (Math.random() - 0.5) * 0.002,
        z: (Math.random() - 0.5) * 0.002,
      });
    }
    return [pos, vel];
  }, [count]);

  useFrame(() => {
    if (!pointsRef.current) return;
    const posAttr = pointsRef.current.geometry.attributes.position;
    const arr = posAttr.array;

    for (let i = 0; i < count; i++) {
      arr[i * 3 + 1] += velocities[i].y;
      arr[i * 3] += velocities[i].x;
      arr[i * 3 + 2] += velocities[i].z;

      // Reset when reaching water surface (Y=1.95)
      if (arr[i * 3 + 1] > 1.95) {
        arr[i * 3 + 1] = 0.1;
        arr[i * 3] = (Math.random() - 0.5) * 24;
        arr[i * 3 + 2] = (Math.random() - 0.5) * 15;
      }
    }
    posAttr.needsUpdate = true;
  });

  return (
    <points ref={pointsRef}>
      <bufferGeometry>
        <bufferAttribute
          attach="attributes-position"
          count={count}
          array={positions}
          itemSize={3}
        />
      </bufferGeometry>
      <pointsMaterial
        size={0.035}
        color="#7dd3fc"
        transparent
        opacity={0.6}
        blending={THREE.AdditiveBlending}
        depthWrite={false}
      />
    </points>
  );
}
