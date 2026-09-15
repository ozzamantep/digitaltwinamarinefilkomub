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

  const headingDeg = Math.round((((headingRad * 180) / Math.PI) % 360 + 360) % 360);

  return (
    <motion.div
      className="glass-panel"
      initial={{ opacity: 0, x: 30 }}
      animate={{ opacity: 1, x: 0 }}
      transition={{ duration: 0.5, delay: 0.2 }}
    >
      <div className="glass-panel-header">
        <div className="glass-panel-title">
          <span className="icon">📷</span>
          Subsea Forward Camera (Real Pool View)
        </div>
        <div
          style={{
            display: 'flex',
            alignItems: 'center',
            gap: '8px',
          }}
        >
          <button
            id="btn-switch-fpv"
            onClick={() => setCameraViewMode(cameraViewMode === 'fpv' ? 'orbit' : 'fpv')}
            style={{
              background: cameraViewMode === 'fpv' ? 'var(--accent-cyan)' : 'rgba(0, 240, 255, 0.15)',
              color: cameraViewMode === 'fpv' ? '#000' : 'var(--accent-cyan)',
              border: '1px solid var(--accent-cyan)',
              borderRadius: '4px',
              padding: '2px 8px',
              fontSize: '0.65rem',
              fontWeight: 600,
              cursor: 'pointer',
              transition: 'all 0.2s ease',
            }}
          >
            {cameraViewMode === 'fpv' ? '✓ FPV VIEW ACTIVE' : 'FULLSCREEN FPV'}
          </button>
          <div
            style={{
              fontSize: '0.65rem',
              color: 'var(--accent-green)',
              fontWeight: 600,
            }}
          >
            {cameraFrame ? 'ROS2 STREAM' : '3D FPV LIVE'}
          </div>
        </div>
      </div>

      <div className="camera-container" style={{ position: 'relative', overflow: 'hidden', height: '185px' }}>
        {cameraFrame ? (
          <>
            <img className="camera-feed" src={cameraFrame} alt="Camera feed" />
            <div className="camera-overlay">
              <div className="camera-rec">
                <span className="camera-rec-dot"></span>
                ROS2 REC
              </div>
            </div>
          </>
        ) : (
          <>
            {/* Real 3D Physical Subsea Camera View from Front Dome */}
            <SubseaRealCameraView />

            {/* Tactical HUD Overlay over the Real 3D Camera Feed */}
            <div
              style={{
                position: 'absolute',
                inset: 0,
                pointerEvents: 'none',
                display: 'flex',
                flexDirection: 'column',
                justifyContent: 'space-between',
                padding: '8px 12px',
                boxShadow: 'inset 0 0 35px rgba(0, 30, 50, 0.75)',
              }}
            >
              {/* Top OSD Bar */}
              <div style={{ display: 'flex', justifyContent: 'space-between', fontSize: '10px', fontFamily: 'monospace', color: 'var(--accent-cyan)' }}>
                <span style={{ background: 'rgba(0,0,0,0.6)', padding: '2px 6px', borderRadius: '3px' }}>
                  CAM: FWD ACRYLIC DOME (110° FOV)
                </span>
                <span style={{ background: 'rgba(0,0,0,0.6)', padding: '2px 6px', borderRadius: '3px', color: 'var(--accent-green)' }}>
                  CV ● 15 FPS
                </span>
              </div>

              {/* Center Crosshair Target Sight */}
              <div
                style={{
                  position: 'absolute',
                  top: '50%',
                  left: '50%',
                  transform: 'translate(-50%, -50%)',
                  width: '50px',
                  height: '50px',
                  border: '1px dashed rgba(0, 240, 255, 0.45)',
                  borderRadius: '50%',
                  display: 'flex',
                  alignItems: 'center',
                  justifyContent: 'center',
                }}
              >
                <div style={{ width: '12px', height: '1px', background: 'var(--accent-cyan)' }} />
                <div style={{ width: '1px', height: '12px', background: 'var(--accent-cyan)', position: 'absolute' }} />
                <div style={{ width: '4px', height: '4px', borderRadius: '50%', background: '#00ff88' }} />
              </div>

              {/* Bottom OSD Bar */}
              <div style={{ display: 'flex', justifyContent: 'space-between', fontSize: '10px', fontFamily: 'monospace', color: '#fff' }}>
                <span style={{ background: 'rgba(0,0,0,0.65)', padding: '2px 6px', borderRadius: '3px' }}>
                  HDG: {headingDeg.toString().padStart(3, '0')}°
                </span>
                <span style={{ background: 'rgba(0,0,0,0.65)', padding: '2px 6px', borderRadius: '3px', color: 'var(--accent-cyan)' }}>
                  DEP: {depth.toFixed(2)}m
                </span>
              </div>
            </div>
          </>
        )}
      </div>
    </motion.div>
  );
}
