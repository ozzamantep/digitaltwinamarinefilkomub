import { useRef, useMemo } from 'react';
import { useFrame } from '@react-three/fiber';
import * as THREE from 'three';
import useVehicleStore from '../../store/vehicleStore';

export default function LidarPoints() {
  const pointsRef = useRef();
  const lidarPoints = useVehicleStore((s) => s.lidarPoints);
  const position = useVehicleStore((s) => s.position);

  const { positions, colors } = useMemo(() => {
    if (!lidarPoints.length) {
      return {
        positions: new Float32Array(0),
        colors: new Float32Array(0),
      };
    }

    const posArr = new Float32Array(lidarPoints.length * 3);
    const colArr = new Float32Array(lidarPoints.length * 3);

    const maxDist = 10;
    const cyan = new THREE.Color('#00f0ff');
    const green = new THREE.Color('#00ff88');
    const red = new THREE.Color('#ff3b5c');

    lidarPoints.forEach((p, i) => {
      posArr[i * 3] = p.x + position.x;
      posArr[i * 3 + 1] = p.z;
      posArr[i * 3 + 2] = p.y + position.y;

      // Color based on distance
      const ratio = Math.min(p.distance / maxDist, 1);
      const color = new THREE.Color();
      if (ratio < 0.3) {
        color.lerpColors(red, green, ratio / 0.3);
      } else {
        color.lerpColors(green, cyan, (ratio - 0.3) / 0.7);
      }

      colArr[i * 3] = color.r;
      colArr[i * 3 + 1] = color.g;
      colArr[i * 3 + 2] = color.b;
    });

    return { positions: posArr, colors: colArr };
  }, [lidarPoints, position]);

  useFrame(() => {
    if (pointsRef.current && pointsRef.current.geometry) {
      pointsRef.current.geometry.attributes.position.needsUpdate = true;
      if (pointsRef.current.geometry.attributes.color) {
        pointsRef.current.geometry.attributes.color.needsUpdate = true;
      }
    }
  });

  if (!lidarPoints.length) return null;

  return (
    <points ref={pointsRef}>
      <bufferGeometry>
        <bufferAttribute
          attach="attributes-position"
          count={positions.length / 3}
          array={positions}
          itemSize={3}
        />
        <bufferAttribute
          attach="attributes-color"
          count={colors.length / 3}
          array={colors}
          itemSize={3}
        />
      </bufferGeometry>
      <pointsMaterial
        size={0.06}
        vertexColors
        transparent
        opacity={0.8}
        sizeAttenuation
        depthWrite={false}
      />
    </points>
  );
}
