import { useState, useRef, useCallback, useEffect } from 'react';
import { motion } from 'framer-motion';
import useVehicleStore from '../../store/vehicleStore';
import topicPublisher from '../../services/TopicPublisher';
import auvMotionController from '../../services/AUVMotionController';
import mockRos from '../../services/MockRosConnection';
import sysIdEngine from '../../services/SystemIdentificationEngine';

function VirtualSubseaJoystick() {
  const areaRef = useRef(null);
  const [knobPos, setKnobPos] = useState({ x: 0, y: 0 });
  const [isDragging, setIsDragging] = useState(false);
  const setControlInput = useVehicleStore((s) => s.setControlInput);
  const armed = useVehicleStore((s) => s.armed);

  const maxRadius = 40;

  const handleMove = useCallback(
    (clientX, clientY) => {
      if (!areaRef.current) return;
      const rect = areaRef.current.getBoundingClientRect();
      const centerX = rect.left + rect.width / 2;
      const centerY = rect.top + rect.height / 2;

      let dx = clientX - centerX;
      let dy = clientY - centerY;

      const distance = Math.sqrt(dx * dx + dy * dy);
      if (distance > maxRadius) {
        dx = (dx / distance) * maxRadius;
        dy = (dy / distance) * maxRadius;
      }

      setKnobPos({ x: dx, y: dy });

      // Up/Down = Surge (Maju/Mundur), Left/Right = Lateral Sway (Geser Kiri/Kanan)
      const surge = -dy / maxRadius;
      const sway = dx / maxRadius;

      setControlInput({ surge, sway });
      topicPublisher.publishVelocity(surge, 0);
    },
    [setControlInput]
  );

  const handleStart = (e) => {
    if (!armed) return;
    setIsDragging(true);
    const event = e.touches ? e.touches[0] : e;
    handleMove(event.clientX, event.clientY);
  };

  const handleMoveEvent = (e) => {
    if (!isDragging) return;
    const event = e.touches ? e.touches[0] : e;
    handleMove(event.clientX, event.clientY);
  };

  const handleEnd = () => {
    setIsDragging(false);
    setKnobPos({ x: 0, y: 0 });
    setControlInput({ surge: 0, sway: 0 });
    topicPublisher.publishVelocity(0, 0);
  };

  return (
    <div className="joystick-container" style={{ padding: '8px 0', flexDirection: 'column', alignItems: 'center' }}>
      <div
        className="joystick-area"
        ref={areaRef}
        onMouseDown={handleStart}
        onMouseMove={handleMoveEvent}
        onMouseUp={handleEnd}
        onMouseLeave={handleEnd}
        onTouchStart={handleStart}
        onTouchMove={handleMoveEvent}
        onTouchEnd={handleEnd}
        style={{
          opacity: armed ? 1 : 0.4,
          cursor: armed ? 'grab' : 'not-allowed',
        }}
      >
        <div className="joystick-crosshair" />
        <div
          className="joystick-knob"
          style={{
            transform: `translate(calc(-50% + ${knobPos.x}px), calc(-50% + ${knobPos.y}px))`,
          }}
        />
      </div>
      <div style={{ fontSize: '0.6rem', color: 'var(--text-tertiary)', marginTop: '4px' }}>
        ↕️ Maju/Mundur · ↔️ <b>Lateral Kiri/Kanan</b>
      </div>
    </div>
  );
}

export default function ControlPanel() {
  const armed = useVehicleStore((s) => s.armed);
  const setArmed = useVehicleStore((s) => s.setArmed);
  const flightMode = useVehicleStore((s) => s.flightMode);
  const setFlightMode = useVehicleStore((s) => s.setFlightMode);
  const lightsIntensity = useVehicleStore((s) => s.lightsIntensity);
  const setLightsIntensity = useVehicleStore((s) => s.setLightsIntensity);
  const setControlInput = useVehicleStore((s) => s.setControlInput);
  const activeTarget = useVehicleStore((s) => s.activeTarget);
  const depth = useVehicleStore((s) => s.depth);

  const [activeTab, setActiveTab] = useState('pilot'); // 'pilot' | 'pid' | 'sysid'

  // System Identification Model Selection
  const [selectedSysIdModel, setSelectedSysIdModel] = useState('ARMAX');

  const handleSelectSysIdModel = (model) => {
    setSelectedSysIdModel(model);
    sysIdEngine.setModel(model);
  };

  // Live PID Gain States
  const [kpDepth, setKpDepth] = useState(0.5);
  const [kiDepth, setKiDepth] = useState(0.1);
  const [kdDepth, setKdDepth] = useState(0.2);
  const [targetDepth, setTargetDepth] = useState(0.8);

  const [kpYaw, setKpYaw] = useState(0.6);
  const [kiYaw, setKiYaw] = useState(0.05);
  const [kdYaw, setKdYaw] = useState(0.25);

  const handleUpdateDepthPID = (kp, ki, kd) => {
    setKpDepth(kp);
    setKiDepth(ki);
    setKdDepth(kd);
    auvMotionController.setDepthGains(kp, ki, kd);
  };

  const handleUpdateYawPID = (kp, ki, kd) => {
    setKpYaw(kp);
    setKiYaw(ki);
    setKdYaw(kd);
    auvMotionController.setYawGains(kp, ki, kd);
  };

  const handleSetTargetDepth = (val) => {
    setTargetDepth(val);
    auvMotionController.setTargetDepth(val);
  };

  // Keyboard navigation controls
  const keysPressed = useRef({});

  const handleKeyDown = useCallback(
    (e) => {
      if (!armed || activeTab !== 'pilot' || flightMode === 'QUALIFIKASI' || flightMode === 'FINAL') return;
      if (['KeyW', 'KeyS', 'KeyA', 'KeyD', 'KeyQ', 'KeyE', 'Space', 'ShiftLeft'].includes(e.code)) {
        e.preventDefault();
      }

      keysPressed.current[e.code] = true;

      let surge = 0;
      let sway = 0;
      let yaw = 0;
      let heave = 0;

      if (keysPressed.current['KeyW']) surge += 1;
      if (keysPressed.current['KeyS']) surge -= 1;
      if (keysPressed.current['KeyD']) sway += 1; // Lateral Kanan
      if (keysPressed.current['KeyA']) sway -= 1; // Lateral Kiri
      if (keysPressed.current['KeyE']) yaw += 1;  // Putar Kanan (Yaw)
      if (keysPressed.current['KeyQ']) yaw -= 1;  // Putar Kiri (Yaw)
      if (keysPressed.current['Space']) heave -= 1; // Surface (Naik)
      if (keysPressed.current['ShiftLeft']) heave += 1; // Dive (Selam)

      setControlInput({ surge, sway, yaw, heave });
      topicPublisher.publishVelocity(surge, yaw);
    },
    [armed, activeTab, flightMode, setControlInput]
  );

  const handleKeyUp = useCallback(
    (e) => {
      keysPressed.current[e.code] = false;

      let surge = 0;
      let sway = 0;
      let yaw = 0;
      let heave = 0;

      if (keysPressed.current['KeyW']) surge += 1;
      if (keysPressed.current['KeyS']) surge -= 1;
      if (keysPressed.current['KeyD']) sway += 1;
      if (keysPressed.current['KeyA']) sway -= 1;
      if (keysPressed.current['KeyE']) yaw += 1;
      if (keysPressed.current['KeyQ']) yaw += 1;
      if (keysPressed.current['Space']) heave -= 1;
      if (keysPressed.current['ShiftLeft']) heave += 1;

      setControlInput({ surge, sway, yaw, heave });
      topicPublisher.publishVelocity(surge, yaw);
    },
    [setControlInput]
  );

  useEffect(() => {
    window.addEventListener('keydown', handleKeyDown);
    window.addEventListener('keyup', handleKeyUp);
    return () => {
      window.removeEventListener('keydown', handleKeyDown);
      window.removeEventListener('keyup', handleKeyUp);
    };
  }, [handleKeyDown, handleKeyUp]);

  // Start Autonomous Mission Handlers
  const handleStartQualification = () => {
    setArmed(true);
    setFlightMode('QUALIFIKASI');
    mockRos.resetQualification();
  };

  const handleStartFinal = () => {
    setArmed(true);
    setFlightMode('FINAL');
    mockRos.resetFinal();
  };

  return (
    <motion.div
      className="glass-panel"
      initial={{ opacity: 0, x: 30 }}
      animate={{ opacity: 1, x: 0 }}
      transition={{ duration: 0.5, delay: 0.3 }}
    >
      <div className="glass-panel-header">
        <div className="glass-panel-title">
          <span className="icon">🎮</span>
          AUV Pilot & System ID Engine
        </div>
        <button
          onClick={() => setArmed(!armed)}
          style={{
            padding: '3px 8px',
            fontSize: '0.65rem',
            fontWeight: '700',
            borderRadius: '4px',
            cursor: 'pointer',
            border: 'none',
            background: armed ? 'var(--accent-green)' : 'var(--accent-red)',
            color: '#000',
            transition: 'all 0.2s ease',
          }}
        >
          {armed ? 'ARMED' : 'DISARMED'}
        </button>
      </div>

      {/* 3 Tabs: Pilot, PID Tuner, System ID */}
      <div style={{ display: 'flex', gap: '4px', marginBottom: '8px' }}>
        <button
          className={`chart-tab ${activeTab === 'pilot' ? 'active' : ''}`}
          onClick={() => setActiveTab('pilot')}
          style={{ flex: 1, textAlign: 'center', fontSize: '0.65rem' }}
        >
          🕹️ Pilot
        </button>
        <button
          className={`chart-tab ${activeTab === 'pid' ? 'active' : ''}`}
          onClick={() => setActiveTab('pid')}
          style={{ flex: 1, textAlign: 'center', fontSize: '0.65rem' }}
        >
          ⚙️ PID Tuner
        </button>
        <button
          className={`chart-tab ${activeTab === 'sysid' ? 'active' : ''}`}
          onClick={() => setActiveTab('sysid')}
          style={{ flex: 1, textAlign: 'center', fontSize: '0.65rem' }}
        >
          📊 System ID
        </button>
      </div>

      {activeTab === 'pilot' && (
        <>
          {/* Standard Flight Modes */}
          <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr 1fr', gap: '4px', marginBottom: '6px' }}>
            {['MANUAL', 'STABILIZE', 'ALT_HOLD'].map((m) => (
              <button
                key={m}
                className={`control-btn ${flightMode === m ? 'active' : ''}`}
                onClick={() => setFlightMode(m)}
                style={{ padding: '6px 2px', fontSize: '0.62rem' }}
              >
                {m}
              </button>
            ))}
          </div>

          {/* Autonomous SAUVC Mission Buttons (QUALIFIKASI & FINAL) */}
          <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: '6px', marginBottom: '10px' }}>
            <button
              id="btn-mission-qual"
              onClick={handleStartQualification}
              style={{
                padding: '8px 4px',
                fontSize: '0.68rem',
                fontWeight: '700',
                borderRadius: '6px',
                cursor: 'pointer',
                border: flightMode === 'QUALIFIKASI' ? '2px solid #22c55e' : '1px solid rgba(34, 197, 94, 0.4)',
                background: flightMode === 'QUALIFIKASI' ? 'rgba(34, 197, 94, 0.3)' : 'rgba(10, 30, 20, 0.8)',
                color: '#4ade80',
                display: 'flex',
                alignItems: 'center',
                justifyContent: 'center',
                gap: '4px',
                boxShadow: flightMode === 'QUALIFIKASI' ? '0 0 12px rgba(34, 197, 94, 0.5)' : 'none',
                transition: 'all 0.2s ease',
              }}
            >
              🏁 QUALIFIKASI
            </button>

            <button
              id="btn-mission-final"
              onClick={handleStartFinal}
              style={{
                padding: '8px 4px',
                fontSize: '0.68rem',
                fontWeight: '700',
                borderRadius: '6px',
                cursor: 'pointer',
                border: flightMode === 'FINAL' ? '2px solid #00f0ff' : '1px solid rgba(0, 240, 255, 0.4)',
                background: flightMode === 'FINAL' ? 'rgba(0, 240, 255, 0.3)' : 'rgba(5, 25, 45, 0.8)',
                color: '#38bdf8',
                display: 'flex',
                alignItems: 'center',
                justifyContent: 'center',
                gap: '4px',
                boxShadow: flightMode === 'FINAL' ? '0 0 12px rgba(0, 240, 255, 0.5)' : 'none',
                transition: 'all 0.2s ease',
              }}
            >
              🏆 FINAL MISSION
            </button>
          </div>

          {/* Subsea Lumen Lights Dimmer */}
          <div
            style={{
              background: 'var(--bg-tertiary)',
              padding: '6px 10px',
              borderRadius: '6px',
              marginBottom: '8px',
            }}
          >
            <div style={{ display: 'flex', justifyContent: 'space-between', fontSize: '0.65rem', marginBottom: '4px' }}>
              <span style={{ color: 'var(--text-secondary)' }}>💡 Lumen Subsea Lights</span>
              <span style={{ color: 'var(--accent-cyan)', fontWeight: '600' }}>{lightsIntensity}%</span>
            </div>
            <input
              type="range"
              min="0"
              max="100"
              value={lightsIntensity}
              onChange={(e) => setLightsIntensity(Number(e.target.value))}
              style={{ width: '100%', accentColor: 'var(--accent-cyan)' }}
            />
          </div>

          {/* Controls or Active Mission Status */}
          {flightMode === 'QUALIFIKASI' || flightMode === 'FINAL' ? (
            <div
              style={{
                textAlign: 'center',
                padding: '10px',
                background: flightMode === 'QUALIFIKASI' ? 'rgba(34, 197, 94, 0.15)' : 'rgba(0, 240, 255, 0.15)',
                border: flightMode === 'QUALIFIKASI' ? '1px solid #22c55e' : '1px solid #00f0ff',
                borderRadius: '8px',
                marginBottom: '8px',
              }}
            >
              <div style={{ fontSize: '1.2rem', marginBottom: '2px' }}>
                {flightMode === 'QUALIFIKASI' ? '🏁' : '🏆'}
              </div>
              <div style={{ fontSize: '0.75rem', fontWeight: 'bold', color: '#fff', marginBottom: '4px' }}>
                {flightMode === 'QUALIFIKASI' ? 'Autonomous Qualification Active' : 'SAUVC Final Autonomous Mission Active'}
              </div>
              <div style={{ fontSize: '0.65rem', color: flightMode === 'QUALIFIKASI' ? '#4ade80' : '#38bdf8', fontFamily: 'monospace' }}>
                {activeTarget}
              </div>
              <button
                onClick={() => setFlightMode('MANUAL')}
                style={{
                  marginTop: '8px',
                  background: 'rgba(255, 255, 255, 0.1)',
                  border: '1px solid rgba(255, 255, 255, 0.3)',
                  color: '#fff',
                  padding: '3px 10px',
                  fontSize: '0.6rem',
                  borderRadius: '4px',
                  cursor: 'pointer',
                }}
              >
                🛑 Take Over Manual Control
              </button>
            </div>
          ) : (
            <div style={{ opacity: armed ? 1 : 0.4 }}>
              {/* Virtual Joystick */}
              <VirtualSubseaJoystick />

              {/* Action Buttons: Turn & Depth */}
              <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: '6px', marginTop: '6px' }}>
                <button
                  className="control-btn"
                  onMouseDown={() => { if (armed) setControlInput({ yaw: -1 }); }}
                  onMouseUp={() => { if (armed) setControlInput({ yaw: 0 }); }}
                  onTouchStart={() => { if (armed) setControlInput({ yaw: -1 }); }}
                  onTouchEnd={() => { if (armed) setControlInput({ yaw: 0 }); }}
                  style={{ padding: '6px 2px', fontSize: '0.65rem' }}
                >
                  ↶ Putar Kiri (Q)
                </button>
                <button
                  className="control-btn"
                  onMouseDown={() => { if (armed) setControlInput({ yaw: 1 }); }}
                  onMouseUp={() => { if (armed) setControlInput({ yaw: 0 }); }}
                  onTouchStart={() => { if (armed) setControlInput({ yaw: 1 }); }}
                  onTouchEnd={() => { if (armed) setControlInput({ yaw: 0 }); }}
                  style={{ padding: '6px 2px', fontSize: '0.65rem' }}
                >
                  ↷ Putar Kanan (E)
                </button>
                <button
                  className="control-btn"
                  onMouseDown={() => { if (armed) setControlInput({ heave: -1 }); }}
                  onMouseUp={() => { if (armed) setControlInput({ heave: 0 }); }}
                  onTouchStart={() => { if (armed) setControlInput({ heave: -1 }); }}
                  onTouchEnd={() => { if (armed) setControlInput({ heave: 0 }); }}
                  style={{ padding: '6px 2px', fontSize: '0.65rem' }}
                >
                  ⬆️ Surface (Space)
                </button>
                <button
                  className="control-btn"
                  onMouseDown={() => { if (armed) setControlInput({ heave: 1 }); }}
                  onMouseUp={() => { if (armed) setControlInput({ heave: 0 }); }}
                  onTouchStart={() => { if (armed) setControlInput({ heave: 1 }); }}
                  onTouchEnd={() => { if (armed) setControlInput({ heave: 0 }); }}
                  style={{ padding: '6px 2px', fontSize: '0.65rem' }}
                >
                  ⬇️ Dive (Shift)
                </button>
              </div>

              {/* Keyboard Guide */}
              <div
                style={{
                  textAlign: 'center',
                  fontSize: '0.6rem',
                  color: 'var(--text-tertiary)',
                  marginTop: '6px',
                  lineHeight: '1.4',
                }}
              >
                <b>W/S</b>: Maju/Mundur · <b>A/D</b>: <span style={{ color: 'var(--accent-cyan)' }}>Lateral Kiri/Kanan</span><br />
                <b>Q/E</b>: Putar Haluan · <b>Space/Shift</b>: Naik/Selam
              </div>
            </div>
          )}
        </>
      )}

      {/* TAB 2: PID TUNER */}
      {activeTab === 'pid' && (
        <div style={{ background: 'var(--bg-tertiary)', padding: '10px', borderRadius: '8px', marginBottom: '8px' }}>
          <div style={{ fontSize: '0.65rem', fontWeight: '700', color: 'var(--accent-cyan)', marginBottom: '6px' }}>
            🎯 DEPTH PID (KP={kpDepth}, KI={kiDepth}, KD={kdDepth})
          </div>
          <div style={{ display: 'flex', justifyContent: 'space-between', fontSize: '0.6rem', color: 'var(--text-secondary)', marginBottom: '4px' }}>
            <span>Target Depth: {targetDepth.toFixed(2)}m</span>
            <span>Current: {depth.toFixed(2)}m</span>
          </div>
          <input
            type="range"
            min="0.2"
            max="1.8"
            step="0.05"
            value={targetDepth}
            onChange={(e) => handleSetTargetDepth(Number(e.target.value))}
            style={{ width: '100%', accentColor: 'var(--accent-cyan)', marginBottom: '8px' }}
          />

          <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr 1fr', gap: '4px', marginBottom: '10px' }}>
            <div>
              <span style={{ fontSize: '0.55rem', color: 'var(--text-tertiary)' }}>Kp (0.5)</span>
              <input
                type="number"
                step="0.05"
                value={kpDepth}
                onChange={(e) => handleUpdateDepthPID(Number(e.target.value), kiDepth, kdDepth)}
                style={{ width: '100%', padding: '2px', fontSize: '0.65rem', background: '#000', color: '#fff', border: '1px solid #333' }}
              />
            </div>
            <div>
              <span style={{ fontSize: '0.55rem', color: 'var(--text-tertiary)' }}>Ki (0.1)</span>
              <input
                type="number"
                step="0.02"
                value={kiDepth}
                onChange={(e) => handleUpdateDepthPID(kpDepth, Number(e.target.value), kdDepth)}
                style={{ width: '100%', padding: '2px', fontSize: '0.65rem', background: '#000', color: '#fff', border: '1px solid #333' }}
              />
            </div>
            <div>
              <span style={{ fontSize: '0.55rem', color: 'var(--text-tertiary)' }}>Kd (0.2)</span>
              <input
                type="number"
                step="0.05"
                value={kdDepth}
                onChange={(e) => handleUpdateDepthPID(kpDepth, kiDepth, Number(e.target.value))}
                style={{ width: '100%', padding: '2px', fontSize: '0.65rem', background: '#000', color: '#fff', border: '1px solid #333' }}
              />
            </div>
          </div>

          <div style={{ fontSize: '0.65rem', fontWeight: '700', color: 'var(--accent-green)', marginBottom: '6px' }}>
            🧭 HEADING PID (KP={kpYaw}, KI={kiYaw}, KD={kdYaw})
          </div>
          <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr 1fr', gap: '4px' }}>
            <div>
              <span style={{ fontSize: '0.55rem', color: 'var(--text-tertiary)' }}>Kp (0.6)</span>
              <input
                type="number"
                step="0.05"
                value={kpYaw}
                onChange={(e) => handleUpdateYawPID(Number(e.target.value), kiYaw, kdYaw)}
                style={{ width: '100%', padding: '2px', fontSize: '0.65rem', background: '#000', color: '#fff', border: '1px solid #333' }}
              />
            </div>
            <div>
              <span style={{ fontSize: '0.55rem', color: 'var(--text-tertiary)' }}>Ki (0.05)</span>
              <input
                type="number"
                step="0.01"
                value={kiYaw}
                onChange={(e) => handleUpdateYawPID(kpYaw, Number(e.target.value), kdYaw)}
                style={{ width: '100%', padding: '2px', fontSize: '0.65rem', background: '#000', color: '#fff', border: '1px solid #333' }}
              />
            </div>
            <div>
              <span style={{ fontSize: '0.55rem', color: 'var(--text-tertiary)' }}>Kd (0.25)</span>
              <input
                type="number"
                step="0.05"
                value={kdYaw}
                onChange={(e) => handleUpdateYawPID(kpYaw, kiYaw, Number(e.target.value))}
                style={{ width: '100%', padding: '2px', fontSize: '0.65rem', background: '#000', color: '#fff', border: '1px solid #333' }}
              />
            </div>
          </div>
        </div>
      )}

      {/* TAB 3: SYSTEM IDENTIFICATION (ARX, ARMAX, OE, BJ) */}
      {activeTab === 'sysid' && (
        <div style={{ background: 'var(--bg-tertiary)', padding: '10px', borderRadius: '8px', marginBottom: '8px' }}>
          <div style={{ fontSize: '0.7rem', fontWeight: '700', color: 'var(--accent-cyan)', marginBottom: '4px' }}>
            🔬 System Identification Model Selection
          </div>
          <div style={{ fontSize: '0.58rem', color: 'var(--text-tertiary)', marginBottom: '8px', lineHeight: '1.3' }}>
            Discrete Polynomial Models matching real BlueROV2 T200 thruster dynamics & hydrodynamic fluid drag.
          </div>

          {/* Model Selection Buttons */}
          <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr 1fr 1fr', gap: '4px', marginBottom: '8px' }}>
            {['ARX', 'ARMAX', 'OE', 'BJ'].map((m) => (
              <button
                key={m}
                onClick={() => handleSelectSysIdModel(m)}
                style={{
                  padding: '6px 2px',
                  fontSize: '0.65rem',
                  fontWeight: '700',
                  borderRadius: '4px',
                  cursor: 'pointer',
                  border: selectedSysIdModel === m ? '1px solid var(--accent-cyan)' : '1px solid rgba(255,255,255,0.1)',
                  background: selectedSysIdModel === m ? 'rgba(0, 240, 255, 0.25)' : 'rgba(0,0,0,0.5)',
                  color: selectedSysIdModel === m ? '#fff' : 'var(--text-secondary)',
                  transition: 'all 0.2s ease',
                }}
              >
                {m}
              </button>
            ))}
          </div>

          {/* Model Equation Details */}
          <div style={{ background: 'rgba(0,0,0,0.4)', padding: '8px', borderRadius: '6px', fontSize: '0.6rem', fontFamily: 'monospace' }}>
            <div style={{ display: 'flex', justifyContent: 'space-between', marginBottom: '4px' }}>
              <span style={{ color: 'var(--accent-cyan)', fontWeight: 'bold' }}>{sysIdEngine.models[selectedSysIdModel].name}</span>
              <span style={{ color: 'var(--accent-green)', fontWeight: 'bold' }}>FIT: {sysIdEngine.models[selectedSysIdModel].fitRate}%</span>
            </div>
            <div style={{ color: 'var(--text-secondary)', fontSize: '0.55rem', marginBottom: '6px' }}>
              {sysIdEngine.models[selectedSysIdModel].description}
            </div>

            {selectedSysIdModel === 'ARX' && (
              <div style={{ color: '#bae6fd' }}>
                A(q) = 1 - 1.42q⁻¹ + 0.48q⁻²<br />
                B(q) = 0.085q⁻¹ + 0.045q⁻²
              </div>
            )}
            {selectedSysIdModel === 'ARMAX' && (
              <div style={{ color: '#bae6fd' }}>
                A(q) = 1 - 1.38q⁻¹ + 0.44q⁻²<br />
                B(q) = 0.092q⁻¹ + 0.038q⁻²<br />
                C(q) = 1 + 0.22q⁻¹
              </div>
            )}
            {selectedSysIdModel === 'OE' && (
              <div style={{ color: '#bae6fd' }}>
                B(q) = 0.096q⁻¹ + 0.034q⁻²<br />
                F(q) = 1 - 1.35q⁻¹ + 0.42q⁻²
              </div>
            )}
            {selectedSysIdModel === 'BJ' && (
              <div style={{ color: '#bae6fd' }}>
                B(q)/F(q) = (0.094q⁻¹ + 0.036q⁻²) / (1 - 1.36q⁻¹ + 0.43q⁻²)<br />
                C(q)/D(q) = (1 + 0.18q⁻¹) / (1 - 0.65q⁻¹)
              </div>
            )}
          </div>
        </div>
      )}
    </motion.div>
  );
}
