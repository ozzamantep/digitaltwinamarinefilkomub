import { useState } from 'react';
import { motion } from 'framer-motion';
import SubseaRealCameraView from './SubseaRealCameraView';
import useVehicleStore from '../../store/vehicleStore';

export default function CameraFeed() {
  const cameraFrame = useVehicleStore((s) => s.cameraFrame);
  const depth = useVehicleStore((s) => s.depth);
  const headingRad = useVehicleStore((s) => s.headingRad);
  const activeTarget = useVehicleStore((s) => s.activeTarget);
  const setCameraViewMode = useVehicleStore((s) => s.setCameraViewMode);
  const cameraViewMode = useVehicleStore((s) => s.cameraViewMode);
  const connectionStatus = useVehicleStore((s) => s.connectionStatus);

  const headingDeg = Math.round((((headingRad * 180) / Math.PI) % 360 + 360) % 360);

  // Expanded view toggles: 'split' (both side-by-side) | 'real' (real camera fullscreen) | 'digital' (digital twin fullscreen)
  const [viewLayout, setViewLayout] = useState('split');

  const isRealCameraOnline = !!cameraFrame;

  return (
    <motion.div
      className="glass-panel"
      initial={{ opacity: 0, x: 30 }}
      animate={{ opacity: 1, x: 0 }}
      transition={{ duration: 0.5, delay: 0.2 }}
    >
      {/* Panel Header */}
      <div className="glass-panel-header">
        <div className="glass-panel-title">
          <span className="icon">📷</span>
          Dual Camera View
        </div>
        <div style={{ display: 'flex', alignItems: 'center', gap: '4px' }}>
          {/* Layout Toggle Buttons */}
          {['split', 'real', 'digital'].map((mode) => (
            <button
              key={mode}
              id={`btn-cam-layout-${mode}`}
              onClick={() => setViewLayout(mode)}
              style={{
                background: viewLayout === mode ? 'var(--accent-cyan)' : 'rgba(0, 240, 255, 0.1)',
                color: viewLayout === mode ? '#000' : 'var(--accent-cyan)',
                border: '1px solid rgba(0, 240, 255, 0.35)',
                borderRadius: '3px',
                padding: '1px 6px',
                fontSize: '0.58rem',
                fontWeight: 600,
                cursor: 'pointer',
                transition: 'all 0.2s ease',
                textTransform: 'uppercase',
                letterSpacing: '0.03em',
              }}
            >
              {mode === 'split' ? '⬜⬜' : mode === 'real' ? '🎥 REAL' : '🤖 DT'}
            </button>
          ))}

          {/* FPV Sync Button */}
          <button
            id="btn-switch-fpv"
            onClick={() => setCameraViewMode(cameraViewMode === 'fpv' ? 'orbit' : 'fpv')}
            style={{
              background: cameraViewMode === 'fpv' ? 'var(--accent-cyan)' : 'rgba(0, 240, 255, 0.15)',
              color: cameraViewMode === 'fpv' ? '#000' : 'var(--accent-cyan)',
              border: '1px solid var(--accent-cyan)',
              borderRadius: '3px',
              padding: '1px 6px',
              fontSize: '0.58rem',
              fontWeight: 600,
              cursor: 'pointer',
              transition: 'all 0.2s ease',
            }}
          >
            {cameraViewMode === 'fpv' ? '✓ FPV SYNC' : 'FPV SYNC'}
          </button>
        </div>
      </div>

      {/* Dual Camera Container */}
      <div
        className="dual-camera-wrapper"
        style={{
          display: 'flex',
          gap: viewLayout === 'split' ? '3px' : '0',
          height: '185px',
          position: 'relative',
        }}
      >
        {/* ═══════════════════════════════════════════════════════════ */}
        {/* LEFT: REAL CAMERA (Physical Ship / ROS2 Stream)           */}
        {/* ═══════════════════════════════════════════════════════════ */}
        {(viewLayout === 'split' || viewLayout === 'real') && (
          <div
            className="camera-pane"
            style={{
              flex: viewLayout === 'split' ? 1 : '1 1 100%',
              position: 'relative',
              overflow: 'hidden',
              borderRadius: '4px',
              background: '#020e18',
              border: '1px solid rgba(0, 240, 255, 0.15)',
            }}
          >
            {isRealCameraOnline ? (
              <>
                {/* Live ROS2 Camera Stream */}
                <img
                  className="camera-feed"
                  src={cameraFrame}
                  alt="Real ship camera"
                  style={{ width: '100%', height: '100%', objectFit: 'cover' }}
                />

                {/* Live Feed Overlay */}
                <div style={{ position: 'absolute', inset: 0, pointerEvents: 'none' }}>
                  {/* REC Indicator */}
                  <div
                    style={{
                      position: 'absolute',
                      top: '5px',
                      left: '5px',
                      display: 'flex',
                      alignItems: 'center',
                      gap: '4px',
                      background: 'rgba(255, 59, 92, 0.85)',
                      padding: '1px 6px',
                      borderRadius: '3px',
                      fontSize: '0.55rem',
                      fontWeight: 700,
                      letterSpacing: '0.04em',
                    }}
                  >
                    <span
                      style={{
                        width: '5px',
                        height: '5px',
                        background: '#fff',
                        borderRadius: '50%',
                        animation: 'pulse-dot 1s ease-in-out infinite',
                      }}
                    />
                    ROS2 LIVE
                  </div>

                  {/* Label Badge */}
                  <div
                    style={{
                      position: 'absolute',
                      top: '5px',
                      right: '5px',
                      background: 'rgba(0, 0, 0, 0.7)',
                      padding: '1px 5px',
                      borderRadius: '3px',
                      fontSize: '0.52rem',
                      fontWeight: 600,
                      color: '#00ff88',
                      border: '1px solid rgba(0, 255, 136, 0.3)',
                    }}
                  >
                    🎥 REAL CAM
                  </div>

                  {/* Bottom Telemetry */}
                  <div
                    style={{
                      position: 'absolute',
                      bottom: '4px',
                      left: '4px',
                      right: '4px',
                      display: 'flex',
                      justifyContent: 'space-between',
                      fontSize: '0.5rem',
                      fontFamily: 'monospace',
                    }}
                  >
                    <span style={{ background: 'rgba(0,0,0,0.65)', padding: '1px 5px', borderRadius: '2px', color: 'var(--accent-cyan)' }}>
                      HDG: {headingDeg.toString().padStart(3, '0')}°
                    </span>
                    <span style={{ background: 'rgba(0,0,0,0.65)', padding: '1px 5px', borderRadius: '2px', color: '#00ff88' }}>
                      DEP: {depth.toFixed(2)}m
                    </span>
                  </div>
                </div>
              </>
            ) : (
              /* Offline / Waiting for ROS2 Camera Stream */
              <div
                style={{
                  width: '100%',
                  height: '100%',
                  display: 'flex',
                  flexDirection: 'column',
                  alignItems: 'center',
                  justifyContent: 'center',
                  background: 'linear-gradient(180deg, #020e18 0%, #071a2b 50%, #0a1e30 100%)',
                  position: 'relative',
                }}
              >
                {/* Scanning Animation Lines */}
                <div
                  style={{
                    position: 'absolute',
                    inset: 0,
                    overflow: 'hidden',
                    opacity: 0.12,
                  }}
                >
                  {Array.from({ length: 12 }).map((_, i) => (
                    <div
                      key={i}
                      style={{
                        position: 'absolute',
                        left: 0,
                        right: 0,
                        height: '1px',
                        background: 'var(--accent-cyan)',
                        top: `${(i + 1) * 8}%`,
                      }}
                    />
                  ))}
                </div>

                {/* Camera Icon + Status */}
                <div style={{ fontSize: '1.5rem', marginBottom: '4px', opacity: 0.5 }}>🎥</div>
                <div
                  style={{
                    fontSize: '0.6rem',
                    fontWeight: 700,
                    color: 'var(--accent-cyan)',
                    letterSpacing: '0.06em',
                    marginBottom: '2px',
                  }}
                >
                  REAL CAMERA
                </div>
                <div
                  style={{
                    fontSize: '0.5rem',
                    color: connectionStatus === 'connected' ? '#facc15' : 'var(--text-tertiary)',
                    fontFamily: 'monospace',
                  }}
                >
                  {connectionStatus === 'connected'
                    ? '⏳ WAITING FOR /camera/image_raw...'
                    : '⚡ CONNECT TO JETSON TO VIEW'}
                </div>

                {/* Corner brackets */}
                <div style={{ position: 'absolute', top: '8px', left: '8px', width: '12px', height: '12px', borderTop: '1.5px solid rgba(0, 240, 255, 0.4)', borderLeft: '1.5px solid rgba(0, 240, 255, 0.4)' }} />
                <div style={{ position: 'absolute', top: '8px', right: '8px', width: '12px', height: '12px', borderTop: '1.5px solid rgba(0, 240, 255, 0.4)', borderRight: '1.5px solid rgba(0, 240, 255, 0.4)' }} />
                <div style={{ position: 'absolute', bottom: '8px', left: '8px', width: '12px', height: '12px', borderBottom: '1.5px solid rgba(0, 240, 255, 0.4)', borderLeft: '1.5px solid rgba(0, 240, 255, 0.4)' }} />
                <div style={{ position: 'absolute', bottom: '8px', right: '8px', width: '12px', height: '12px', borderBottom: '1.5px solid rgba(0, 240, 255, 0.4)', borderRight: '1.5px solid rgba(0, 240, 255, 0.4)' }} />

                {/* Label Badge */}
                <div
                  style={{
                    position: 'absolute',
                    top: '5px',
                    right: '5px',
                    background: 'rgba(0, 0, 0, 0.7)',
                    padding: '1px 5px',
                    borderRadius: '3px',
                    fontSize: '0.48rem',
                    fontWeight: 600,
                    color: 'rgba(0, 240, 255, 0.6)',
                    border: '1px solid rgba(0, 240, 255, 0.2)',
                  }}
                >
                  🎥 REAL CAM
                </div>
              </div>
            )}
          </div>
        )}

        {/* ═══════════════════════════════════════════════════════════ */}
        {/* RIGHT: DIGITAL TWIN CAMERA (3D Simulated / Synced View)   */}
        {/* ═══════════════════════════════════════════════════════════ */}
        {(viewLayout === 'split' || viewLayout === 'digital') && (
          <div
            className="camera-pane"
            style={{
              flex: viewLayout === 'split' ? 1 : '1 1 100%',
              position: 'relative',
              overflow: 'hidden',
              borderRadius: '4px',
              border: '1px solid rgba(0, 240, 255, 0.15)',
            }}
          >
            {/* Physical 3D Subsea Camera View - Same Pose as Real Ship */}
            <SubseaRealCameraView />

            {/* Tactical HUD Overlay */}
            <div
              style={{
                position: 'absolute',
                inset: 0,
                pointerEvents: 'none',
                display: 'flex',
                flexDirection: 'column',
                justifyContent: 'space-between',
                padding: '5px 6px',
                boxShadow: 'inset 0 0 25px rgba(0, 30, 50, 0.7)',
              }}
            >
              {/* Top OSD Bar */}
              <div style={{ display: 'flex', justifyContent: 'space-between', fontSize: '0.5rem', fontFamily: 'monospace', color: 'var(--accent-cyan)' }}>
                <span style={{ background: 'rgba(0,0,0,0.6)', padding: '1px 5px', borderRadius: '2px' }}>
                  DOME FWD 110° FOV
                </span>
                <span
                  style={{
                    background: 'rgba(0,0,0,0.7)',
                    padding: '1px 5px',
                    borderRadius: '3px',
                    fontSize: '0.48rem',
                    fontWeight: 600,
                    color: '#38bdf8',
                    border: '1px solid rgba(56, 189, 248, 0.3)',
                  }}
                >
                  🤖 DIGITAL TWIN
                </span>
              </div>

              {/* Center Crosshair */}
              <div
                style={{
                  position: 'absolute',
                  top: '50%',
                  left: '50%',
                  transform: 'translate(-50%, -50%)',
                  width: '36px',
                  height: '36px',
                  border: '1px dashed rgba(0, 240, 255, 0.35)',
                  borderRadius: '50%',
                  display: 'flex',
                  alignItems: 'center',
                  justifyContent: 'center',
                }}
              >
                <div style={{ width: '8px', height: '1px', background: 'var(--accent-cyan)' }} />
                <div style={{ width: '1px', height: '8px', background: 'var(--accent-cyan)', position: 'absolute' }} />
                <div style={{ width: '3px', height: '3px', borderRadius: '50%', background: '#00ff88' }} />
              </div>

              {/* Bottom OSD Bar */}
              <div style={{ display: 'flex', justifyContent: 'space-between', fontSize: '0.5rem', fontFamily: 'monospace', color: '#fff' }}>
                <span style={{ background: 'rgba(0,0,0,0.65)', padding: '1px 5px', borderRadius: '2px' }}>
                  HDG: {headingDeg.toString().padStart(3, '0')}°
                </span>
                <span style={{ background: 'rgba(0,0,0,0.65)', padding: '1px 5px', borderRadius: '2px', color: 'var(--accent-cyan)' }}>
                  DEP: {depth.toFixed(2)}m
                </span>
              </div>
            </div>

            {/* Sync Status Indicator */}
            {isRealCameraOnline && (
              <div
                style={{
                  position: 'absolute',
                  bottom: '18px',
                  left: '50%',
                  transform: 'translateX(-50%)',
                  background: 'rgba(0, 255, 136, 0.15)',
                  border: '1px solid rgba(0, 255, 136, 0.4)',
                  borderRadius: '3px',
                  padding: '0px 5px',
                  fontSize: '0.46rem',
                  fontWeight: 700,
                  color: '#00ff88',
                  letterSpacing: '0.04em',
                  display: 'flex',
                  alignItems: 'center',
                  gap: '3px',
                }}
              >
                <span style={{ width: '4px', height: '4px', borderRadius: '50%', background: '#00ff88', animation: 'pulse-dot 1.5s ease-in-out infinite' }} />
                SYNCED WITH REAL CAM
              </div>
            )}
          </div>
        )}
      </div>

      {/* Sync Status Footer Bar */}
      <div
        style={{
          display: 'flex',
          justifyContent: 'space-between',
          alignItems: 'center',
          padding: '3px 8px',
          fontSize: '0.52rem',
          fontFamily: 'monospace',
          color: 'var(--text-tertiary)',
          borderTop: '1px solid rgba(0, 240, 255, 0.08)',
          marginTop: '2px',
        }}
      >
        <span style={{ display: 'flex', alignItems: 'center', gap: '4px' }}>
          <span
            style={{
              width: '5px',
              height: '5px',
              borderRadius: '50%',
              background: isRealCameraOnline ? '#00ff88' : '#64748b',
              boxShadow: isRealCameraOnline ? '0 0 6px #00ff88' : 'none',
            }}
          />
          {isRealCameraOnline ? 'REAL CAM: ONLINE' : 'REAL CAM: OFFLINE'}
        </span>
        <span style={{ color: 'var(--accent-cyan)', fontWeight: 600 }}>
          POSE SYNC: {isRealCameraOnline ? 'ACTIVE' : 'DT ONLY'}
        </span>
        <span>
          {isRealCameraOnline ? 'ROS2 STREAM' : '3D FPV'} · 15 FPS
        </span>
      </div>
    </motion.div>
  );
}
