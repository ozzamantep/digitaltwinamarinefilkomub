import { Canvas, useFrame } from '@react-three/fiber';
import { Suspense, useRef, useEffect, useCallback } from 'react';
import * as THREE from 'three';
import PoolEnvironment from '../3d/PoolEnvironment';
import UnderwaterParticles from '../3d/UnderwaterParticles';
import PayloadBallDropper from '../3d/PayloadBallDropper';
import useVehicleStore from '../../store/vehicleStore';

// Official SAUVC 2026 Arena Target Coordinates (World 3D positions)
const MISSION_TARGETS = [
  { id: 'flare_orange', name: 'FLARE_ORG', classColor: '#ea580c', pos: [-6.0, 0.75, 2.0], width: 0.35, height: 1.5, baseConf: 96.5 },
  { id: 'flare_blue', name: 'FLARE_BLU', classColor: '#0284c7', pos: [-2.0, 0.75, 2.2], width: 0.35, height: 1.5, baseConf: 94.2 },
  { id: 'flare_red', name: 'FLARE_RED', classColor: '#ef4444', pos: [0.5, 0.75, 4.0], width: 0.35, height: 1.5, baseConf: 95.8 },
  { id: 'flare_yellow', name: 'FLARE_YEL', classColor: '#eab308', pos: [-0.5, 0.75, -4.5], width: 0.35, height: 1.5, baseConf: 93.4 },
  { id: 'gate', name: 'SAUVC_GATE', classColor: '#f59e0b', pos: [4.0, 0.85, 0.0], width: 1.9, height: 1.6, baseConf: 98.2 },
  { id: 'drum_blue', name: 'DRUM_BLU', classColor: '#0284c7', pos: [10.5, 0.25, 4.5], width: 0.7, height: 0.5, baseConf: 92.1 },
  { id: 'drum_red_1', name: 'DRUM_RED_TGT', classColor: '#ef4444', pos: [10.5, 0.25, 1.5], width: 0.7, height: 0.5, baseConf: 97.4 },
  { id: 'drum_red_2', name: 'DRUM_RED_2', classColor: '#ef4444', pos: [10.5, 0.25, -1.5], width: 0.7, height: 0.5, baseConf: 91.5 },
  { id: 'drum_red_3', name: 'DRUM_RED_3', classColor: '#ef4444', pos: [10.5, 0.25, -4.5], width: 0.7, height: 0.5, baseConf: 90.8 },
];

function RealOnboardCameraRig({ onRenderFrame }) {
  const position = useVehicleStore((s) => s.position);
  const orientation = useVehicleStore((s) => s.orientation);

  useFrame(({ camera }) => {
    const pos = position || { x: -11, y: 1.1, z: 2.0 };
    const quat = new THREE.Quaternion(
      orientation.x || 0,
      orientation.y || 0,
      orientation.z || 0,
      orientation.w ?? 1
    );

    // Front acrylic dome offset in local BlueROV2 body coordinates (+X = front)
    const localDomeOffset = new THREE.Vector3(0.25, 0.03, 0.0).applyQuaternion(quat);
    camera.position.set(
      pos.x + localDomeOffset.x,
      pos.y + localDomeOffset.y,
      pos.z + localDomeOffset.z
    );

    // Camera orientation: copy vehicle 6-DOF quaternion, then rotate -90° around Y
    // so Three.js camera (-Z default) looks directly along vehicle nose (+X)
    camera.quaternion.copy(quat);
    camera.rotateY(-Math.PI / 2);

    if (onRenderFrame) {
      onRenderFrame(camera);
    }
  });

  return null;
}

export default function SubseaRealCameraView() {
  const overlayCanvasRef = useRef(null);
  const lastCameraRef = useRef(null);

  const handleCameraUpdate = useCallback((camera) => {
    lastCameraRef.current = camera;
    const canvas = overlayCanvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    const w = canvas.width;
    const h = canvas.height;
    ctx.clearRect(0, 0, w, h);

    const camPos = camera.position;
    const projVec = new THREE.Vector3();

    // Mathematically project all SAUVC mission targets through the actual onboard 3D camera lens
    MISSION_TARGETS.forEach((target) => {
      const dx = target.pos[0] - camPos.x;
      const dy = target.pos[1] - camPos.y;
      const dz = target.pos[2] - camPos.z;
      const dist = Math.sqrt(dx * dx + dy * dy + dz * dz);

      if (dist > 0.4 && dist < 16.0) {
        projVec.set(target.pos[0], target.pos[1], target.pos[2]);
        projVec.project(camera);

        // Check if object is in front of camera frustum (-1 to +1 in NDC)
        if (projVec.z > -1.0 && projVec.z < 1.0 && Math.abs(projVec.x) < 1.15 && Math.abs(projVec.y) < 1.15) {
          const screenX = (projVec.x * 0.5 + 0.5) * w;
          const screenY = (-projVec.y * 0.5 + 0.5) * h;

          const boxW = Math.max(24, Math.min(130, (target.width / dist) * w * 0.85));
          const boxH = Math.max(28, Math.min(150, (target.height / dist) * h * 0.9));

          const left = screenX - boxW / 2;
          const top = screenY - boxH / 2;

          // YOLO Bounding Box Rectangle
          ctx.strokeStyle = target.classColor;
          ctx.lineWidth = 2;
          ctx.strokeRect(left, top, boxW, boxH);

          // Modern Corner Accents
          const cornerSize = 6;
          ctx.strokeStyle = '#ffffff';
          ctx.lineWidth = 2;
          ctx.beginPath();
          ctx.moveTo(left, top + cornerSize); ctx.lineTo(left, top); ctx.lineTo(left + cornerSize, top);
          ctx.stroke();
          ctx.beginPath();
          ctx.moveTo(left + boxW - cornerSize, top); ctx.lineTo(left + boxW, top); ctx.lineTo(left + boxW, top + cornerSize);
          ctx.stroke();
          ctx.beginPath();
          ctx.moveTo(left, top + boxH - cornerSize); ctx.lineTo(left, top + boxH); ctx.lineTo(left + cornerSize, top + boxH);
          ctx.stroke();
          ctx.beginPath();
          ctx.moveTo(left + boxW - cornerSize, top + boxH); ctx.lineTo(left + boxW, top + boxH); ctx.lineTo(left + boxW, top + boxH - cornerSize);
          ctx.stroke();

          // Label Tag with Distance & Confidence
          const conf = Math.max(78, (target.baseConf - dist * 1.2)).toFixed(1);
          const labelText = `${target.name} ${conf}% ${dist.toFixed(1)}m`;

          ctx.font = 'bold 9px monospace';
          const textWidth = ctx.measureText(labelText).width;
          ctx.fillStyle = target.classColor;
          ctx.fillRect(left, Math.max(10, top - 15), textWidth + 8, 14);

          ctx.fillStyle = '#000000';
          ctx.fillText(labelText, left + 4, Math.max(21, top - 4));
        }
      }
    });
  }, []);

  return (
    <div style={{ width: '100%', height: '100%', position: 'relative' }}>
      <Canvas
        camera={{ fov: 72, near: 0.05, far: 60 }}
        gl={{ antialias: true, alpha: false, powerPreference: 'high-performance' }}
        onCreated={({ gl }) => {
          gl.setClearColor('#022033');
        }}
      >
        <ambientLight intensity={0.7} color="#bae6fd" />
        <directionalLight position={[6, 14, 6]} intensity={1.5} color="#f0f9ff" />
        <pointLight position={[0, 1.8, 0]} intensity={1.2} color="#00f0ff" distance={18} />
        <fog attach="fog" args={['#022033', 1, 28]} />

        <Suspense fallback={null}>
          <PoolEnvironment />
          <PayloadBallDropper />
          <UnderwaterParticles count={80} />
        </Suspense>

        <RealOnboardCameraRig onRenderFrame={handleCameraUpdate} />
      </Canvas>

      <canvas
        ref={overlayCanvasRef}
        width={340}
        height={185}
        style={{
          position: 'absolute',
          inset: 0,
          width: '100%',
          height: '100%',
          pointerEvents: 'none',
        }}
      />
    </div>
  );
}
