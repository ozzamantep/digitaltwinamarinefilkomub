import { useMemo } from 'react';
import * as THREE from 'three';
import useVehicleStore from '../../store/vehicleStore';

export default function TrajectoryPath() {
  const positionHistory = useVehicleStore((s) => s.positionHistory);

  const { positions, colors } = useMemo(() => {
    if (!positionHistory || positionHistory.length < 2) {
      return { positions: new Float32Array(0), colors: new Float32Array(0) };
    }

    const maxPoints = 250;
    let points = positionHistory.filter((p) => p && isFinite(p.x) && isFinite(p.z));
    if (points.length < 2) {
      return { positions: new Float32Array(0), colors: new Float32Array(0) };
    }

    if (points.length > maxPoints) {
      const step = Math.floor(points.length / maxPoints);
      points = points.filter((_, i) => i % step === 0);
    }

    const posArr = new Float32Array(points.length * 3);
    const colArr = new Float32Array(points.length * 3);

    const startColor = new THREE.Color('#0284c7');
    const endColor = new THREE.Color('#00ff88');

    points.forEach((p, i) => {
      posArr[i * 3] = isFinite(p.x) ? p.x : -11;
      posArr[i * 3 + 1] = isFinite(p.y) ? p.y : 1.2;
      posArr[i * 3 + 2] = isFinite(p.z) ? p.z : 2;

      const ratio = points.length > 1 ? i / (points.length - 1) : 0;
      const color = new THREE.Color().lerpColors(startColor, endColor, ratio);
      colArr[i * 3] = color.r;
      colArr[i * 3 + 1] = color.g;
      colArr[i * 3 + 2] = color.b;
    });

    return { positions: posArr, colors: colArr };
  }, [positionHistory]);

  if (!positions || positions.length < 6) return null;

  return (
    <line>
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
      <lineBasicMaterial
        vertexColors
        transparent
        opacity={0.8}
        linewidth={2}
      />
    </line>
  );
}
