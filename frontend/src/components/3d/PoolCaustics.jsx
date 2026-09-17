import { useRef, useMemo } from 'react';
import { useFrame } from '@react-three/fiber';
import * as THREE from 'three';

/**
 * Creates dynamic animated swimming pool water caustics texture
 * that projects shimmering sunlight patterns onto the pool floor and walls.
 */
export default function PoolCaustics() {
  const meshRef = useRef();

  // Create caustic shader canvas / texture
  const { canvas, ctx, texture } = useMemo(() => {
    const canvas = document.createElement('canvas');
    canvas.width = 512;
    canvas.height = 512;
    const ctx = canvas.getContext('2d');
    const texture = new THREE.CanvasTexture(canvas);
    texture.wrapS = THREE.RepeatWrapping;
    texture.wrapT = THREE.RepeatWrapping;
    texture.repeat.set(6, 4);
    return { canvas, ctx, texture };
  }, []);

  useFrame((state) => {
    const t = state.clock.elapsedTime * 0.8;
    if (!ctx) return;

    // Draw procedural Voronoi-like caustic wave networks
    ctx.fillStyle = 'rgba(0, 40, 80, 0.2)';
    ctx.fillRect(0, 0, 512, 512);

    ctx.strokeStyle = 'rgba(180, 245, 255, 0.45)';
    ctx.lineWidth = 3.5;
    ctx.shadowBlur = 12;
    ctx.shadowColor = '#00f0ff';

    const count = 16;
    for (let i = 0; i < count; i++) {
      ctx.beginPath();
      const x1 = Math.sin(t + i) * 200 + 256;
      const y1 = Math.cos(t * 0.8 + i * 1.5) * 200 + 256;
      const x2 = Math.sin(t * 1.2 + i * 2) * 200 + 256;
      const y2 = Math.cos(t + i * 2.5) * 200 + 256;
      const cpX = Math.sin(t * 0.5 + i) * 150 + 256;
      const cpY = Math.cos(t * 0.7 + i) * 150 + 256;

      ctx.moveTo(x1, y1);
      ctx.quadraticCurveTo(cpX, cpY, x2, y2);
      ctx.stroke();
    }

    texture.needsUpdate = true;
  });

  return (
    <mesh
      ref={meshRef}
      rotation={[-Math.PI / 2, 0, 0]}
      position={[0, 0.015, 0]}
    >
      <planeGeometry args={[25, 16]} />
      <meshBasicMaterial
        map={texture}
        transparent
        opacity={0.16}
        blending={THREE.AdditiveBlending}
        depthWrite={false}
      />
    </mesh>
  );
}
