import { Canvas } from '@react-three/fiber';
import { OrbitControls } from '@react-three/drei';
import { Suspense, useRef, useState, useEffect } from 'react';
import PoolEnvironment from './PoolEnvironment';
import BlueROV2Model from './BlueROV2Model';
import UnderwaterParticles from './UnderwaterParticles';
import ThrusterBubbles from './ThrusterBubbles';
import TrajectoryPath from './TrajectoryPath';
import CameraController from './CameraController';
import PayloadBallDropper from './PayloadBallDropper';
import useVehicleStore from '../../store/vehicleStore';

function LoadingFallback() {
  return (
    <group position={[0, 1.2, 0]}>
      <mesh>
        <boxGeometry args={[0.6, 0.3, 0.45]} />
        <meshStandardMaterial color="#00f0ff" wireframe />
      </mesh>
    </group>
  );
}

export default function VehicleScene() {
  const [fps, setFps] = useState(60);
  const frameCount = useRef(0);
  const lastTime = useRef(performance.now());
  const controlsRef = useRef();

  const depth = useVehicleStore((s) => s.depth);
  const activeTarget = useVehicleStore((s) => s.activeTarget);
  const cameraViewMode = useVehicleStore((s) => s.cameraViewMode);
  const setCameraViewMode = useVehicleStore((s) => s.setCameraViewMode);
  const triggerCameraReset = useVehicleStore((s) => s.triggerCameraReset);
  const headingRad = useVehicleStore((s) => s.headingRad);
  const headingDeg = Math.round(((((headingRad || 0) * 180) / Math.PI) % 360 + 360) % 360);

  // Stable FPS meter (Runs once on mount without re-binding to position)
  useEffect(() => {
    const interval = setInterval(() => {
      const now = performance.now();
      const delta = now - lastTime.current;
      if (delta > 0) {
        setFps(Math.round((frameCount.current / delta) * 1000));
      }
      frameCount.current = 0;
      lastTime.current = now;
    }, 500);
    return () => clearInterval(interval);
  }, []);

  return (
    <div className="viewport-3d" style={{ position: 'relative' }}>
      <Canvas
        camera={{ position: [-11, 6.5, 9.5], fov: 48, near: 0.05, far: 200 }}
        shadows
        dpr={[1, 2]}
        gl={{ antialias: true, alpha: false, powerPreference: 'high-performance' }}
        onCreated={({ gl }) => {
          gl.setClearColor('#031926');
        }}
      >
        {/* Underwater Cinematic Lighting */}
        <ambientLight intensity={0.65} color="#bae6fd" />
        <directionalLight
          position={[6, 14, 6]}
          intensity={1.6}
          castShadow
          shadow-mapSize={[2048, 2048]}
          shadow-camera-far={40}
          shadow-camera-left={-15}
          shadow-camera-right={15}
          shadow-camera-top={15}
          shadow-camera-bottom={-15}
          color="#f0f9ff"
        />
        {/* Pool Subsea Halogen Floodlights */}
        <pointLight position={[-6, 1.9, 0]} intensity={1.0} color="#38bdf8" distance={16} />
        <pointLight position={[4, 1.9, 0]} intensity={1.0} color="#00f0ff" distance={16} />
        <pointLight position={[10, 1.9, 0]} intensity={0.8} color="#34d399" distance={16} />

        {/* Realistic Subsea Atmospheric Fog */}
        <fog attach="fog" args={['#022033', 6, 36]} />

        {/* SAUVC Pool Environment, Submarine & Hydrodynamic Particle Effects */}
        <Suspense fallback={<LoadingFallback />}>
          <PoolEnvironment />
          <BlueROV2Model onFrame={() => { frameCount.current++; }} />
          <PayloadBallDropper />
          <ThrusterBubbles />
          <UnderwaterParticles count={160} />
          <TrajectoryPath />
        </Suspense>

        {/* Dynamic Camera Controller (FPV / Chase / Orbit) */}
        <CameraController controlsRef={controlsRef} />

        {/* Smooth Subsea Camera Controls */}
        <OrbitControls
          ref={controlsRef}
          makeDefault
          minPolarAngle={0.05}
          maxPolarAngle={Math.PI / 2.05}
          minDistance={0.5}
          maxDistance={35}
          enableDamping
          dampingFactor={0.07}
          target={[-3, 1.1, 0]}
        />
      </Canvas>

      {/* FPV Subsea Tactical Reticle & Compass Overlay */}
      {cameraViewMode === 'fpv' && (
        <div
          style={{
            position: 'absolute',
            inset: 0,
            pointerEvents: 'none',
            display: 'flex',
            flexDirection: 'column',
            justifyContent: 'space-between',
            padding: '24px',
            boxShadow: 'inset 0 0 80px rgba(0, 240, 255, 0.15)',
          }}
        >
          {/* Top Compass Heading Tape */}
          <div style={{ textAlign: 'center' }}>
            <div
              style={{
                display: 'inline-block',
                background: 'rgba(3, 25, 38, 0.75)',
                border: '1px solid rgba(0, 240, 255, 0.4)',
                borderRadius: '6px',
                padding: '4px 16px',
                fontFamily: 'monospace',
                fontSize: '13px',
                color: 'var(--accent-cyan)',
                backdropFilter: 'blur(8px)',
              }}
            >
              HDG: {headingDeg.toString().padStart(3, '0')}° · ONBOARD FORWARD CAMERA (110° FOV)
            </div>
          </div>

          {/* Center Tactical Crosshair */}
          <div
            style={{
              position: 'absolute',
              top: '50%',
              left: '50%',
              transform: 'translate(-50%, -50%)',
              width: '120px',
              height: '120px',
              border: '1px dashed rgba(0, 240, 255, 0.35)',
              borderRadius: '50%',
              display: 'flex',
              alignItems: 'center',
              justifyContent: 'center',
            }}
          >
            <div style={{ width: '16px', height: '1px', background: 'var(--accent-cyan)' }} />
            <div style={{ width: '1px', height: '16px', background: 'var(--accent-cyan)', position: 'absolute' }} />
            <div style={{ width: '6px', height: '6px', borderRadius: '50%', background: 'rgba(0, 255, 136, 0.8)' }} />
          </div>

          {/* Bottom Telemetry Bar */}
          <div style={{ display: 'flex', justifyContent: 'space-between', fontSize: '12px', fontFamily: 'monospace' }}>
            <span style={{ color: 'var(--accent-green)', background: 'rgba(0,0,0,0.6)', padding: '2px 8px', borderRadius: '4px' }}>
              DEPTH: {(depth || 0.8).toFixed(2)}m
            </span>
            <span style={{ color: 'var(--accent-cyan)', background: 'rgba(0,0,0,0.6)', padding: '2px 8px', borderRadius: '4px' }}>
              SAUVC AUTONOMOUS VISION RIG
            </span>
          </div>
        </div>
      )}

      {/* Viewport Overlay & Quick Camera Selector */}
      <div className="viewport-overlay">
        <div className="viewport-badge" style={{ display: 'flex', gap: '8px', alignItems: 'center' }}>
          <span>🌊 SAUVC 2026</span>
          <span
            style={{
              color: 'var(--accent-green)',
              borderLeft: '1px solid rgba(255,255,255,0.2)',
              paddingLeft: '8px',
            }}
          >
            {activeTarget}
          </span>
        </div>

        {/* Floating Camera Selector Buttons */}
        <div
          style={{
            display: 'flex',
            gap: '6px',
            background: 'rgba(10, 22, 34, 0.85)',
            border: '1px solid rgba(0, 240, 255, 0.3)',
            borderRadius: '8px',
            padding: '4px 6px',
            backdropFilter: 'blur(10px)',
            pointerEvents: 'auto',
          }}
        >
          <button
            id="btn-cam-tpp"
            onClick={() => setCameraViewMode('tpp')}
            title="Kamera TPP (Third-Person Perspective): Terkunci presisi tepat di belakang kapal dan selalu menghadap ke depan tanpa rotasi orbit bebas"
            style={{
              background: cameraViewMode === 'tpp' ? 'var(--accent-cyan)' : 'transparent',
              color: cameraViewMode === 'tpp' ? '#000' : 'var(--text-secondary)',
              border: 'none',
              borderRadius: '4px',
              padding: '4px 10px',
              fontSize: '11px',
              fontWeight: '600',
              cursor: 'pointer',
              display: 'flex',
              alignItems: 'center',
              gap: '4px',
              transition: 'all 0.2s ease',
            }}
          >
            🎯 TPP (Belakang)
          </button>
          <button
            id="btn-cam-chase"
            onClick={() => {
              setCameraViewMode('chase');
              triggerCameraReset();
            }}
            title="Kamera mengikuti pergerakan kapal & dapat di-orbit bebas 360° dengan mouse drag"
            style={{
              background: cameraViewMode === 'chase' ? 'var(--accent-cyan)' : 'transparent',
              color: cameraViewMode === 'chase' ? '#000' : 'var(--text-secondary)',
              border: 'none',
              borderRadius: '4px',
              padding: '4px 10px',
              fontSize: '11px',
              fontWeight: '600',
              cursor: 'pointer',
              display: 'flex',
              alignItems: 'center',
              gap: '4px',
              transition: 'all 0.2s ease',
            }}
          >
            🚁 Chase (Orbit)
          </button>
          <button
            id="btn-cam-fpv"
            onClick={() => setCameraViewMode('fpv')}
            style={{
              background: cameraViewMode === 'fpv' ? 'var(--accent-cyan)' : 'transparent',
              color: cameraViewMode === 'fpv' ? '#000' : 'var(--text-secondary)',
              border: 'none',
              borderRadius: '4px',
              padding: '4px 10px',
              fontSize: '11px',
              fontWeight: '600',
              cursor: 'pointer',
              display: 'flex',
              alignItems: 'center',
              gap: '4px',
              transition: 'all 0.2s ease',
            }}
          >
            🎥 FPV (Depan)
          </button>
          <button
            id="btn-cam-orbit"
            onClick={() => setCameraViewMode('orbit')}
            style={{
              background: cameraViewMode === 'orbit' ? 'var(--accent-cyan)' : 'transparent',
              color: cameraViewMode === 'orbit' ? '#000' : 'var(--text-secondary)',
              border: 'none',
              borderRadius: '4px',
              padding: '4px 10px',
              fontSize: '11px',
              fontWeight: '600',
              cursor: 'pointer',
              display: 'flex',
              alignItems: 'center',
              gap: '4px',
              transition: 'all 0.2s ease',
            }}
          >
            🌐 Orbit (Bebas)
          </button>

          {(cameraViewMode === 'chase' || cameraViewMode === 'orbit') && (
            <button
              id="btn-cam-focus"
              onClick={() => triggerCameraReset()}
              title="Kunci & fokuskan kamera tepat ke tengah kapal (Shortcut Gamepad: R3 atau D-Pad Bawah)"
              style={{
                background: 'rgba(0, 255, 136, 0.15)',
                color: 'var(--accent-green)',
                border: '1px solid rgba(0, 255, 136, 0.35)',
                borderRadius: '4px',
                padding: '4px 8px',
                fontSize: '11px',
                fontWeight: '700',
                cursor: 'pointer',
                display: 'flex',
                alignItems: 'center',
                gap: '4px',
                transition: 'all 0.2s ease',
              }}
            >
              🎯 Fokus Kapal
            </button>
          )}
        </div>

        <div className="viewport-fps">{fps} FPS</div>
      </div>
    </div>
  );
}
