import { useState, useRef, useCallback, useEffect } from 'react';
import { motion } from 'framer-motion';
import useVehicleStore from '../../store/vehicleStore';
import topicPublisher from '../../services/TopicPublisher';
import auvMotionController from '../../services/AUVMotionController';
import mockRos from '../../services/MockRosConnection';
import sysIdEngine from '../../services/SystemIdentificationEngine';
import DigitalTwinPanel from './DigitalTwinPanel';

function computeThrustersFromVelocity(surge = 0, sway = 0, heave = 0, yaw = 0) {
  const t1 = Math.max(-100, Math.min(100, Math.round((surge + yaw - sway) * 100)));
  const t2 = Math.max(-100, Math.min(100, Math.round((surge - yaw + sway) * 100)));
  const t3 = Math.max(-100, Math.min(100, Math.round((surge + yaw + sway) * 100)));
  const t4 = Math.max(-100, Math.min(100, Math.round((surge - yaw - sway) * 100)));
  const t5 = Math.max(-100, Math.min(100, Math.round(heave * 100)));
  const t6 = Math.max(-100, Math.min(100, Math.round(heave * 100)));
  const thrusters = [t1, t2, t3, t4, t5, t6];
  const rpms = thrusters.map((v) => Math.round(Math.abs(v) * 32));
  return { thrusters, rpms };
}

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
      const { thrusters, rpms } = computeThrustersFromVelocity(surge, sway, 0, 0);
      useVehicleStore.getState().updateThrusters(thrusters, rpms);
      topicPublisher.publishVelocity(surge, sway, 0, 0);
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
    useVehicleStore.getState().updateThrusters([0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0]);
    topicPublisher.publishVelocity(0, 0, 0, 0);
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
  const obstacles = useVehicleStore((s) => s.obstacles) || {};
  const setObstaclePos = useVehicleStore((s) => s.setObstaclePos);
  const resetObstacles = useVehicleStore((s) => s.resetObstacles);
  const applyPresetLayout = useVehicleStore((s) => s.applyPresetLayout);
  const flaresFallen = useVehicleStore((s) => s.flaresFallen || { red: false, blue: false, yellow: false, orange: false });
  const flareStrategies = useVehicleStore((s) => s.flareStrategies || {
    orange_flare: 'MENGHINDAR',
    blue_flare: 'TABRAK',
    red_flare: 'TABRAK',
    yellow_flare: 'TABRAK',
  });
  const setFlareStrategy = useVehicleStore((s) => s.setFlareStrategy);
  const obstacleOrder = useVehicleStore((s) => s.obstacleOrder || [
    'orange_flare',
    'blue_flare',
    'red_flare',
    'yellow_flare',
    'gate',
    'drum_red_tgt',
  ]);
  const obstacleEnabled = useVehicleStore((s) => s.obstacleEnabled || {
    orange_flare: true,
    blue_flare: true,
    red_flare: true,
    yellow_flare: true,
    gate: true,
    drum_red_tgt: true,
  });
  const moveObstacleOrder = useVehicleStore((s) => s.moveObstacleOrder);
  const setObstaclePriority = useVehicleStore((s) => s.setObstaclePriority);
  const toggleObstacleEnabled = useVehicleStore((s) => s.toggleObstacleEnabled);
  const applyOrderPreset = useVehicleStore((s) => s.applyOrderPreset);
  const payloadState = useVehicleStore((s) => s.payloadState);
  const gripperState = useVehicleStore((s) => s.gripperState || 'CLOSED');
  const connectionStatus = useVehicleStore((s) => s.connectionStatus);
  const safetyInterlocks = useVehicleStore((s) => s.safetyInterlocks);

  const [activeTab, setActiveTab] = useState('pilot'); // 'pilot' | 'pid' | 'sysid' | 'sync'
  const [selectedLayoutPreset, setSelectedLayoutPreset] = useState('standard');
  const [selectedOrderPreset, setSelectedOrderPreset] = useState('standard');
  const [gamepadConnected, setGamepadConnected] = useState(false);
  const [gamepadName, setGamepadName] = useState('');
  const gamepadPrevButtons = useRef({});

  const handleToggleGripper = useCallback(() => {
    const current = useVehicleStore.getState();
    if (current.gripperState === 'OPEN') {
      current.setGripperState('CLOSED');
      topicPublisher.publishGripperCommand('CLOSE');
    } else if (current.payloadState.grasped) {
      current.releaseBall();
      topicPublisher.publishGripperCommand('RELEASE');
    } else {
      current.setGripperState('OPEN');
      topicPublisher.publishGripperCommand('OPEN');
    }
  }, []);

  const handleReleaseOrGrasp = useCallback(() => {
    const current = useVehicleStore.getState();
    if (current.payloadState.grasped) {
      current.releaseBall();
      topicPublisher.publishGripperCommand('RELEASE');
    } else {
      current.graspBallWithGripper();
      topicPublisher.publishGripperCommand('GRASP');
    }
  }, []);

  // =========================================================================
  // GAMEPAD / CONTROLLER SUPPORT (PS4 / Xbox / Generic HID)
  // Left Stick  → Surge (Y) + Sway (X)
  // Right Stick → Yaw (X) + Heave (Y)
  // L1/LB       → Disarm
  // R1/RB       → Arm
  // Triangle/Y  → Cycle flight mode
  // Circle/B    → Toggle Subsea Gripper (Open/Close)
  // Cross/A     → Release Ball / Grasp Ball
  // =========================================================================
  useEffect(() => {
    const handleGamepadConnected = (e) => {
      setGamepadConnected(true);
      setGamepadName(e.gamepad.id.split('(')[0].trim());
      console.log(`[Gamepad] Connected: ${e.gamepad.id}`);
    };
    const handleGamepadDisconnected = () => {
      setGamepadConnected(false);
      setGamepadName('');
      console.log('[Gamepad] Disconnected');
    };

    window.addEventListener('gamepadconnected', handleGamepadConnected);
    window.addEventListener('gamepaddisconnected', handleGamepadDisconnected);

    // 60Hz gamepad polling loop
    let animFrame;
    const DEADZONE = 0.12;
    const applyDeadzone = (val) => Math.abs(val) < DEADZONE ? 0 : (val - Math.sign(val) * DEADZONE) / (1 - DEADZONE);
    const flightModes = ['MANUAL', 'STABILIZE', 'ALT_HOLD', 'GUIDED'];

    const pollGamepad = () => {
      const gamepads = navigator.getGamepads ? navigator.getGamepads() : [];
      const gp = gamepads[0] || gamepads[1] || gamepads[2] || gamepads[3];

      if (gp) {
        if (!gamepadConnected) {
          setGamepadConnected(true);
          setGamepadName(gp.id.split('(')[0].trim());
        }

        const store = useVehicleStore.getState();
        const currentArmed = store.armed;
        const currentMode = store.flightMode;

        // Skip autonomous modes
        if (currentMode !== 'QUALIFIKASI' && currentMode !== 'FINAL') {
          // Left stick: axes[0] = X (sway), axes[1] = Y (surge, inverted)
          const surge = applyDeadzone(-gp.axes[1]);
          const sway = applyDeadzone(gp.axes[0]);

          // Right stick: axes[2] = X (yaw), axes[3] = Y (heave, inverted)
          const yaw = applyDeadzone(gp.axes[2] || gp.axes[3] || 0);
          const heave = applyDeadzone(gp.axes[3] !== undefined ? -gp.axes[3] : 0);

          // Only update if any stick is active
          if (currentArmed && (Math.abs(surge) > 0 || Math.abs(sway) > 0 || Math.abs(yaw) > 0 || Math.abs(heave) > 0)) {
            store.setControlInput({ surge, sway, yaw, heave });
            const { thrusters, rpms } = computeThrustersFromVelocity(surge, sway, heave, yaw);
            store.updateThrusters(thrusters, rpms);
            topicPublisher.publishVelocity(surge, sway, heave, yaw);
          } else if (currentArmed && gp.axes.some((a) => Math.abs(a) > 0.05)) {
            // Neutral stick release
            const { thrusters, rpms } = computeThrustersFromVelocity(0, 0, 0, 0);
            store.updateThrusters(thrusters, rpms);
          }
        }

        // Button handling (with edge detection to prevent repeat triggers)
        const prevBtns = gamepadPrevButtons.current;

        // L1/LB (button 4) → Disarm
        if (gp.buttons[4]?.pressed && !prevBtns[4]) {
          store.setArmed(false);
        }
        // R1/RB (button 5) → Arm
        if (gp.buttons[5]?.pressed && !prevBtns[5]) {
          store.setArmed(true);
        }
        // Triangle/Y (button 3) → Cycle flight mode
        if (gp.buttons[3]?.pressed && !prevBtns[3]) {
          const currentIdx = flightModes.indexOf(currentMode);
          const nextIdx = (currentIdx + 1) % flightModes.length;
          store.setFlightMode(flightModes[nextIdx]);
        }
        // Circle/B (button 1) → Toggle Subsea Robotic Gripper (Open/Close)
        if (gp.buttons[1]?.pressed && !prevBtns[1]) {
          handleToggleGripper();
        }
        // Cross/A (button 0) → Release payload, or grasp again in simulation
        if (gp.buttons[0]?.pressed && !prevBtns[0]) {
          handleReleaseOrGrasp();
        }

        // =====================================================================
        // CAMERA CONTROLS ON GAMEPAD / STICK
        // 1. Cycle Camera Mode: Select/Back (Button 8), Square/X (Button 2), L3 (Button 10)
        // 2. Snap / Re-center Focus on Sub: R3 (Button 11) or D-Pad Down tap (Button 13)
        // 3. Free Directional Camera Orbit: D-Pad Up / Down / Left / Right (Buttons 12-15)
        // =====================================================================
        if (
          (gp.buttons[8]?.pressed && !prevBtns[8]) ||
          (gp.buttons[2]?.pressed && !prevBtns[2]) ||
          (gp.buttons[10]?.pressed && !prevBtns[10])
        ) {
          store.cycleCameraViewMode(1);
        }

        // R3 (Right Stick Click) or D-Pad Down Tap -> Instant Focus & Snap to Submarine
        if (
          (gp.buttons[11]?.pressed && !prevBtns[11]) ||
          (gp.buttons[13]?.pressed && !prevBtns[13])
        ) {
          store.triggerCameraReset();
        }

        // Directional Free Camera Orbiting via D-Pad (Up, Down, Left, Right)
        let deltaAzimuth = 0;
        let deltaElevation = 0;

        // D-Pad Left (Button 14) -> Orbit Left around sub
        if (gp.buttons[14]?.pressed) {
          deltaAzimuth -= 1.0;
        }
        // D-Pad Right (Button 15) -> Orbit Right around sub
        if (gp.buttons[15]?.pressed) {
          deltaAzimuth += 1.0;
        }
        // D-Pad Up (Button 12) -> Pitch Camera Up (Elevate view)
        if (gp.buttons[12]?.pressed) {
          deltaElevation -= 1.0;
        }
        // D-Pad Down (Button 13) -> Pitch Camera Down
        if (gp.buttons[13]?.pressed) {
          deltaElevation += 0.8;
        }

        store.setGamepadCameraOrbit(deltaAzimuth, deltaElevation);

        // Record button state for edge detection
        gamepadPrevButtons.current = {};
        for (let i = 0; i < gp.buttons.length; i++) {
          gamepadPrevButtons.current[i] = gp.buttons[i]?.pressed || false;
        }
      }

      animFrame = requestAnimationFrame(pollGamepad);
    };

    animFrame = requestAnimationFrame(pollGamepad);

    return () => {
      cancelAnimationFrame(animFrame);
      window.removeEventListener('gamepadconnected', handleGamepadConnected);
      window.removeEventListener('gamepaddisconnected', handleGamepadDisconnected);
    };
  }, [gamepadConnected, handleReleaseOrGrasp, handleToggleGripper]);

  // System Identification Model Selection
  const [selectedSysIdModel, setSelectedSysIdModel] = useState('BJ');

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
      if (keysPressed.current['Space'])     heave += 1; // Dive (Selam/Turun) — cocok dengan fisik
      if (keysPressed.current['ShiftLeft']) heave -= 1; // Surface (Naik) — cocok dengan fisik

      setControlInput({ surge, sway, yaw, heave });
      const { thrusters, rpms } = computeThrustersFromVelocity(surge, sway, heave, yaw);
      useVehicleStore.getState().updateThrusters(thrusters, rpms);
      topicPublisher.publishVelocity(surge, sway, heave, yaw);
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
      if (keysPressed.current['KeyQ']) yaw -= 1;
      if (keysPressed.current['Space'])     heave += 1; // Dive
      if (keysPressed.current['ShiftLeft']) heave -= 1; // Surface

      setControlInput({ surge, sway, yaw, heave });
      const { thrusters, rpms } = computeThrustersFromVelocity(surge, sway, heave, yaw);
      useVehicleStore.getState().updateThrusters(thrusters, rpms);
      topicPublisher.publishVelocity(surge, sway, heave, yaw);
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
          Flight Control
        </div>
        <button
          onClick={() => {
            const next = !armed;
            setArmed(next);
            if (connectionStatus === 'connected') topicPublisher.armDisarm(next);
          }}
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

      {/* Gamepad Connection Indicator */}
      {gamepadConnected && (
        <div
          style={{
            display: 'flex',
            flexDirection: 'column',
            gap: '2px',
            padding: '4px 10px',
            marginBottom: '6px',
            background: 'rgba(0, 255, 136, 0.08)',
            border: '1px solid rgba(0, 255, 136, 0.25)',
            borderRadius: '6px',
            fontSize: '0.60rem',
            color: 'var(--accent-green)',
          }}
        >
          <div style={{ display: 'flex', alignItems: 'center', gap: '6px' }}>
            <span style={{ fontSize: '0.8rem' }}>🎮</span>
            <span style={{ fontWeight: '700' }}>{gamepadName || 'Controller'}</span>
            <span style={{ color: 'var(--text-secondary)', marginLeft: 'auto', fontSize: '0.55rem' }}>
              LB/RB: Disarm/Arm · △: Mode
            </span>
          </div>
          <div style={{ color: 'var(--text-tertiary)', fontSize: '0.55rem' }}>
            L-Stick: Gerak · R-Stick: Yaw/Selam · D-Pad: 🔄 Orbit Kamera · D-Pad ↓/R3: 🎯 Fokus Kapal · ▢/Select: 🎥 Ganti Kamera
          </div>
        </div>
      )}

      <div style={{ display: 'flex', gap: '3px', marginBottom: '8px' }}>
        <button
          className={`chart-tab ${activeTab === 'pilot' ? 'active' : ''}`}
          onClick={() => setActiveTab('pilot')}
          style={{ flex: 1, textAlign: 'center', fontSize: '0.62rem', padding: '4px 2px' }}
        >
          🕹️ Pilot
        </button>
        <button
          className={`chart-tab ${activeTab === 'pid' ? 'active' : ''}`}
          onClick={() => setActiveTab('pid')}
          style={{ flex: 1, textAlign: 'center', fontSize: '0.62rem', padding: '4px 2px' }}
        >
          ⚙️ PID
        </button>
        <button
          className={`chart-tab ${activeTab === 'sysid' ? 'active' : ''}`}
          onClick={() => setActiveTab('sysid')}
          style={{ flex: 1, textAlign: 'center', fontSize: '0.62rem', padding: '4px 2px' }}
        >
          📊 SysID
        </button>
        <button
          className={`chart-tab ${activeTab === 'sync' ? 'active' : ''}`}
          onClick={() => setActiveTab('sync')}
          style={{
            flex: 1.25,
            textAlign: 'center',
            fontSize: '0.62rem',
            padding: '4px 2px',
            background: activeTab === 'sync' ? 'rgba(0, 255, 136, 0.2)' : undefined,
            borderColor: activeTab === 'sync' ? 'var(--accent-green)' : undefined,
            color: activeTab === 'sync' ? 'var(--accent-green)' : undefined,
          }}
        >
          🌐 Twin Sync
        </button>
      </div>

      {activeTab === 'pilot' && (
        <>
          {/* Standard Flight Modes */}
          <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: '4px', marginBottom: '6px' }}>
            {['MANUAL', 'STABILIZE', 'ALT_HOLD', 'GUIDED'].map((m) => (
              <button
                key={m}
                className={`control-btn ${flightMode === m ? 'active' : ''}`}
                onClick={() => {
                  setFlightMode(m);
                  if (connectionStatus === 'connected') topicPublisher.setFlightMode(m);
                }}
                style={{
                  padding: '6px 2px',
                  fontSize: '0.62rem',
                  ...(m === 'GUIDED' && {
                    background: flightMode === 'GUIDED'
                      ? 'linear-gradient(135deg, rgba(0,240,255,0.35), rgba(0,128,255,0.35))'
                      : 'rgba(0,240,255,0.05)',
                    border: flightMode === 'GUIDED'
                      ? '1px solid #00f0ff'
                      : '1px solid rgba(0,240,255,0.3)',
                    color: '#00f0ff',
                    fontWeight: 700,
                    boxShadow: flightMode === 'GUIDED' ? '0 0 10px rgba(0,240,255,0.4)' : 'none',
                  }),
                }}
              >
                {m === 'GUIDED' ? '⚡ GUIDED' : m}
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

          {/* Subsea Robotic Gripper Control Widget */}
          <div
            style={{
              padding: '6px 8px',
              background: 'rgba(0, 240, 255, 0.05)',
              border: '1px solid rgba(0, 240, 255, 0.25)',
              borderRadius: '6px',
              marginBottom: '8px',
            }}
          >
            <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: '4px' }}>
              <span style={{ fontSize: '0.65rem', color: 'var(--text-secondary)', fontWeight: '600' }}>
                🗜️ Capit Robotik (Gripper)
              </span>
              <span
                style={{
                  fontSize: '0.60rem',
                  fontWeight: '700',
                  padding: '2px 6px',
                  borderRadius: '4px',
                  background:
                    gripperState === 'OPEN'
                      ? 'rgba(234, 179, 8, 0.2)'
                      : payloadState?.grasped
                      ? 'rgba(239, 68, 68, 0.2)'
                      : 'rgba(0, 255, 136, 0.2)',
                  color:
                    gripperState === 'OPEN'
                      ? '#facc15'
                      : payloadState?.grasped
                      ? '#ef4444'
                      : 'var(--accent-green)',
                }}
              >
                {payloadState?.grasped ? '🔴 MENCAPIT BOLA' : gripperState === 'OPEN' ? 'TERBUKA' : 'TERTUTUP'}
              </span>
            </div>
            <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: '4px' }}>
              <button
                id="btn-toggle-gripper"
                onClick={handleToggleGripper}
                style={{
                  padding: '5px 4px',
                  fontSize: '0.62rem',
                  fontWeight: '700',
                  borderRadius: '4px',
                  cursor: 'pointer',
                  border: '1px solid rgba(0, 240, 255, 0.4)',
                  background: gripperState === 'OPEN' ? 'rgba(0, 240, 255, 0.25)' : 'rgba(15, 23, 42, 0.8)',
                  color: '#38bdf8',
                }}
              >
                {gripperState === 'OPEN' ? '🔒 Tutup Gripper' : '🔓 Buka Gripper'} (◯/B)
              </button>
              <button
                id="btn-test-grasp"
                onClick={handleReleaseOrGrasp}
                style={{
                  padding: '5px 4px',
                  fontSize: '0.62rem',
                  fontWeight: '700',
                  borderRadius: '4px',
                  cursor: 'pointer',
                  border: '1px solid rgba(239, 68, 68, 0.4)',
                  background: payloadState?.grasped ? 'rgba(239, 68, 68, 0.3)' : 'rgba(15, 23, 42, 0.8)',
                  color: '#f87171',
                }}
              >
                {payloadState?.grasped ? 'Lepas Bola (✕/A)' : 'Capit Bola (✕/A)'}
              </button>
            </div>
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
                onClick={() => {
                  setFlightMode('MANUAL');
                  if (connectionStatus === 'connected') topicPublisher.setFlightMode('MANUAL');
                }}
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
                  disabled={safetyInterlocks.floor}
                  onMouseDown={() => { if (armed && !safetyInterlocks.floor) setControlInput({ heave: 1 }); }}
                  onMouseUp={() => { if (armed) setControlInput({ heave: 0 }); }}
                  onTouchStart={() => { if (armed && !safetyInterlocks.floor) setControlInput({ heave: 1 }); }}
                  onTouchEnd={() => { if (armed) setControlInput({ heave: 0 }); }}
                  style={{ padding: '6px 2px', fontSize: '0.65rem', opacity: safetyInterlocks.floor ? 0.45 : 1 }}
                >
                  {safetyInterlocks.floor ? 'FLOOR LOCK' : '⬇️ Dive (Shift)'}
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

      {/* 4. TWIN SYNC & DYNAMIC ARENA CONFIGURATION */}
      {activeTab === 'sync' && (
        <div style={{ fontSize: '0.62rem' }}>
          {/* Digital Twin Core Health & Intelligence Center */}
          <div style={{ marginBottom: '10px' }}>
            <DigitalTwinPanel />
          </div>

          {/* Link Status */}
          <div
            style={{
              padding: '6px 8px',
              borderRadius: '6px',
              background: connectionStatus === 'connected' ? 'rgba(0,255,136,0.12)' : 'rgba(0,240,255,0.08)',
              border: `1px solid ${connectionStatus === 'connected' ? 'rgba(0,255,136,0.3)' : 'rgba(0,240,255,0.2)'}`,
              marginBottom: '8px',
              display: 'flex',
              alignItems: 'center',
              justifyContent: 'space-between',
            }}
          >
            <div>
              <div style={{ fontWeight: 'bold', color: connectionStatus === 'connected' ? 'var(--accent-green)' : 'var(--accent-cyan)' }}>
                {connectionStatus === 'connected' ? '🟢 REAL-WORLD JETSON (LIVE ROS2)' : '🔵 DIGITAL TWIN SHADOW / SITL'}
              </div>
              <div style={{ fontSize: '0.55rem', color: 'var(--text-tertiary)' }}>
                Sync Topics: /odom, /yolo_target_coord, /mission_state
              </div>
            </div>
            <span style={{ fontSize: '0.75rem' }}>{connectionStatus === 'connected' ? '⚡ SYNCED' : '🧪 SITL'}</span>
          </div>

          {/* Real vs Twin Milestone Achievements */}
          <div style={{ marginBottom: '8px' }}>
            <div style={{ fontWeight: 'bold', color: '#bae6fd', marginBottom: '4px' }}>
              🎯 Live Mission Milestone Synchronization
            </div>
            <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: '4px' }}>
              <div style={{ background: 'rgba(0,0,0,0.4)', padding: '4px 6px', borderRadius: '4px' }}>
                <span style={{ color: '#ea580c' }}>🟠 Orange Flare:</span>{' '}
                <span style={{ color: 'var(--text-primary)', fontWeight: '600' }}>Safe Stand-off</span>
              </div>
              <div style={{ background: 'rgba(0,0,0,0.4)', padding: '4px 6px', borderRadius: '4px' }}>
                <span style={{ color: '#0284c7' }}>🔵 Blue Flare:</span>{' '}
                <span style={{ color: flaresFallen.blue ? 'var(--accent-green)' : 'var(--text-tertiary)', fontWeight: '600' }}>
                  {flaresFallen.blue ? '💥 Fallen' : '⚪ Upright'}
                </span>
              </div>
              <div style={{ background: 'rgba(0,0,0,0.4)', padding: '4px 6px', borderRadius: '4px' }}>
                <span style={{ color: '#ef4444' }}>🔴 Red Flare:</span>{' '}
                <span style={{ color: flaresFallen.red ? 'var(--accent-green)' : 'var(--text-tertiary)', fontWeight: '600' }}>
                  {flaresFallen.red ? '💥 Fallen' : '⚪ Upright'}
                </span>
              </div>
              <div style={{ background: 'rgba(0,0,0,0.4)', padding: '4px 6px', borderRadius: '4px' }}>
                <span style={{ color: '#eab308' }}>🟡 Yellow Flare:</span>{' '}
                <span style={{ color: flaresFallen.yellow ? 'var(--accent-green)' : 'var(--text-tertiary)', fontWeight: '600' }}>
                  {flaresFallen.yellow ? '💥 Fallen' : '⚪ Upright'}
                </span>
              </div>
              <div style={{ background: 'rgba(0,0,0,0.4)', padding: '4px 6px', borderRadius: '4px' }}>
                <span style={{ color: '#f59e0b' }}>🚪 Gate Transit:</span>{' '}
                <span style={{ color: obstacles.gate?.passed ? 'var(--accent-green)' : 'var(--text-tertiary)', fontWeight: '600' }}>
                  {obstacles.gate?.passed ? '✓ Passed' : '⚪ Standby'}
                </span>
              </div>
              <div style={{ background: 'rgba(0,0,0,0.4)', padding: '4px 6px', borderRadius: '4px' }}>
                <span style={{ color: '#ef4444' }}>🪣 Red Drum:</span>{' '}
                <span style={{ color: payloadState.dropped ? 'var(--accent-green)' : 'var(--text-tertiary)', fontWeight: '600' }}>
                  {payloadState.dropped ? '🎯 Dropped' : '⚪ Loaded'}
                </span>
              </div>
            </div>
          </div>

          {/* Arena Layout Preset Selector */}
          <div style={{ marginBottom: '8px' }}>
            <div style={{ fontWeight: 'bold', color: '#bae6fd', marginBottom: '4px' }}>
              🔀 Dynamic Obstacle Layout Presets
            </div>
            <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: '4px', marginBottom: '4px' }}>
              <button
                className={`control-btn ${selectedLayoutPreset === 'standard' ? 'active' : ''}`}
                onClick={() => {
                  setSelectedLayoutPreset('standard');
                  applyPresetLayout('standard');
                }}
                style={{ padding: '5px 2px', fontSize: '0.58rem' }}
              >
                📍 Standard SAUVC
              </button>
              <button
                className={`control-btn ${selectedLayoutPreset === 'offset' ? 'active' : ''}`}
                onClick={() => {
                  setSelectedLayoutPreset('offset');
                  applyPresetLayout('offset_layout');
                }}
                style={{ padding: '5px 2px', fontSize: '0.58rem' }}
              >
                🔀 Shifted Layout
              </button>
            </div>
            <button
              onClick={() => {
                resetObstacles();
                useVehicleStore.getState().resetPayload();
                useVehicleStore.getState().resetFlares();
              }}
              style={{
                width: '100%',
                padding: '4px',
                fontSize: '0.58rem',
                background: 'rgba(255,255,255,0.05)',
                border: '1px solid rgba(255,255,255,0.15)',
                borderRadius: '4px',
                color: 'var(--text-secondary)',
                cursor: 'pointer',
              }}
            >
              ↺ Reset All Obstacle Coordinates & Milestones
            </button>
          </div>

          {/* Mission Execution Order & Task Priority Roadmap */}
          <div style={{ marginBottom: '8px', background: 'rgba(0,240,255,0.03)', padding: '6px 8px', borderRadius: '6px', border: '1px solid rgba(0,240,255,0.15)' }}>
            <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: '4px' }}>
              <div style={{ fontWeight: 'bold', color: '#38bdf8', fontSize: '0.62rem', display: 'flex', alignItems: 'center', gap: '4px' }}>
                <span>🎯</span>
                <span>Urutan Eksekusi Target (Mission Queue)</span>
              </div>
              <span style={{ fontSize: '0.52rem', color: 'var(--text-tertiary)' }}>
                {obstacleOrder.filter(k => obstacleEnabled[k] !== false).length} / {obstacleOrder.length} Target Aktif
              </span>
            </div>

            <div style={{ fontSize: '0.54rem', color: 'var(--text-tertiary)', marginBottom: '6px', lineHeight: 1.3 }}>
              Pilih target mana yang dikerjakan duluan. AUV akan mengejar target urutan <b>#1</b> hingga selesai.
            </div>

            {/* Visual Queue Flow Breadcrumb */}
            <div style={{ display: 'flex', alignItems: 'center', gap: '3px', overflowX: 'auto', paddingBottom: '4px', marginBottom: '6px' }}>
              {obstacleOrder.map((key, idx) => {
                const meta = {
                  orange_flare: { label: 'Oren', color: '#ea580c', icon: '🟠' },
                  blue_flare:   { label: 'Biru', color: '#0284c7', icon: '🔵' },
                  red_flare:    { label: 'Merah', color: '#ef4444', icon: '🔴' },
                  yellow_flare: { label: 'Kuning', color: '#eab308', icon: '🟡' },
                  gate:         { label: 'Gate', color: '#f59e0b', icon: '🚪' },
                  drum_red_tgt: { label: 'Ember', color: '#ef4444', icon: '🪣' },
                }[key] || { label: key, color: '#94a3b8', icon: '📍' };
                const isEnabled = obstacleEnabled[key] !== false;

                return (
                  <div key={key} style={{ display: 'flex', alignItems: 'center', gap: '2px', flexShrink: 0 }}>
                    <div
                      style={{
                        display: 'flex',
                        alignItems: 'center',
                        gap: '2px',
                        padding: '2px 5px',
                        borderRadius: '4px',
                        background: isEnabled ? 'rgba(0,0,0,0.5)' : 'rgba(255,255,255,0.02)',
                        border: `1px solid ${isEnabled ? meta.color : 'rgba(255,255,255,0.08)'}`,
                        opacity: isEnabled ? 1 : 0.4,
                        fontSize: '0.52rem',
                        color: isEnabled ? '#f8fafc' : 'var(--text-tertiary)',
                        textDecoration: isEnabled ? 'none' : 'line-through',
                      }}
                    >
                      <span style={{ fontWeight: 'bold', color: meta.color }}>#{idx + 1}</span>
                      <span>{meta.icon}</span>
                      <span>{meta.label}</span>
                    </div>
                    {idx < obstacleOrder.length - 1 && (
                      <span style={{ fontSize: '0.50rem', color: 'rgba(255,255,255,0.25)' }}>➔</span>
                    )}
                  </div>
                );
              })}
            </div>

            {/* Quick Order Strategy Presets */}
            <div style={{ display: 'grid', gridTemplateColumns: 'repeat(4, 1fr)', gap: '3px' }}>
              <button
                className={`control-btn ${selectedOrderPreset === 'standard' ? 'active' : ''}`}
                onClick={() => {
                  setSelectedOrderPreset('standard');
                  applyOrderPreset('standard');
                  topicPublisher.publishObstacleOrder(['orange_flare', 'blue_flare', 'red_flare', 'yellow_flare', 'gate', 'drum_red_tgt']);
                }}
                title="Urutan Standar SAUVC: Oren ➔ Biru ➔ Merah ➔ Kuning ➔ Gate ➔ Ember"
                style={{ padding: '3px 2px', fontSize: '0.52rem' }}
              >
                📍 Standard
              </button>
              <button
                className={`control-btn ${selectedOrderPreset === 'reverse_flares' ? 'active' : ''}`}
                onClick={() => {
                  setSelectedOrderPreset('reverse_flares');
                  applyOrderPreset('reverse_flares');
                  topicPublisher.publishObstacleOrder(['yellow_flare', 'red_flare', 'blue_flare', 'orange_flare', 'gate', 'drum_red_tgt']);
                }}
                title="Urutan Terbalik: Kuning ➔ Merah ➔ Biru ➔ Oren ➔ Gate ➔ Ember"
                style={{ padding: '3px 2px', fontSize: '0.52rem' }}
              >
                🔄 Reverse
              </button>
              <button
                className={`control-btn ${selectedOrderPreset === 'tabrak_first' ? 'active' : ''}`}
                onClick={() => {
                  setSelectedOrderPreset('tabrak_first');
                  applyOrderPreset('tabrak_first');
                }}
                title="Prioritaskan semua flare TABRAK terlebih dahulu sebelum MENGHINDAR"
                style={{ padding: '3px 2px', fontSize: '0.52rem' }}
              >
                💥 Tabrak Dulu
              </button>
              <button
                className={`control-btn ${selectedOrderPreset === 'gate_first' ? 'active' : ''}`}
                onClick={() => {
                  setSelectedOrderPreset('gate_first');
                  applyOrderPreset('gate_first');
                  topicPublisher.publishObstacleOrder(['gate', 'orange_flare', 'blue_flare', 'red_flare', 'yellow_flare', 'drum_red_tgt']);
                }}
                title="Lolos Gate terlebih dahulu sebelum menargetkan flare dan ember"
                style={{ padding: '3px 2px', fontSize: '0.52rem' }}
              >
                🚪 Gate Dulu
              </button>
            </div>
            <button
              className="control-btn"
              onClick={() => {
                setSelectedOrderPreset(null);
                useVehicleStore.getState().optimizeObstacleOrder();
                topicPublisher.publishObstacleOrder(useVehicleStore.getState().obstacleOrder);
              }}
              title="Hitung urutan tercepat (jarak tempuh total terpendek) dari posisi kapal saat ini menggunakan pencarian TSP"
              style={{ padding: '3px 2px', fontSize: '0.52rem', marginTop: '3px', width: '100%' }}
            >
              ⚡ Optimalkan (Tercepat)
            </button>
          </div>

          {/* Dynamic Coordinate Sliders, Flare Action Mode & Task Reordering */}
          <div style={{ maxHeight: '230px', overflowY: 'auto', paddingRight: '4px' }}>
            {obstacleOrder.map((key, index) => {
              const metaMap = {
                orange_flare: { label: '🟠 Orange Flare', shortLabel: 'Orange', color: '#ea580c', isFlare: true },
                blue_flare:   { label: '🔵 Blue Flare',   shortLabel: 'Blue',   color: '#0284c7', isFlare: true },
                red_flare:    { label: '🔴 Red Flare',    shortLabel: 'Red',    color: '#ef4444', isFlare: true },
                yellow_flare: { label: '🟡 Yellow Flare', shortLabel: 'Yellow', color: '#eab308', isFlare: true },
                gate:         { label: '🚪 Gate Center',  shortLabel: 'Gate',   color: '#f59e0b', isFlare: false },
                drum_red_tgt: { label: '🪣 Target Red Drum', shortLabel: 'Drum', color: '#ef4444', isFlare: false },
              };
              const meta = metaMap[key] || { label: key, shortLabel: key, color: '#38bdf8', isFlare: false };
              const obs = obstacles[key] || { x: 0, z: 0 };
              const currentStrategy = flareStrategies[key] || (key === 'orange_flare' ? 'MENGHINDAR' : 'TABRAK');
              const isEnabled = obstacleEnabled[key] !== false;

              return (
                <div
                  key={key}
                  style={{
                    background: isEnabled ? 'rgba(0,0,0,0.35)' : 'rgba(0,0,0,0.2)',
                    padding: '6px 8px',
                    borderRadius: '6px',
                    marginBottom: '6px',
                    border: isEnabled ? `1px solid ${index === 0 ? 'rgba(0,255,136,0.3)' : 'rgba(255,255,255,0.08)'}` : '1px dashed rgba(255,255,255,0.08)',
                    opacity: isEnabled ? 1 : 0.65,
                    transition: 'all 0.2s ease',
                  }}
                >
                  {/* Card Top Header: Priority Badge + Name + Enable Toggle + Coords */}
                  <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: '4px' }}>
                    <div style={{ display: 'flex', alignItems: 'center', gap: '5px' }}>
                      <span
                        style={{
                          fontSize: '0.52rem',
                          fontWeight: 'bold',
                          padding: '1px 5px',
                          borderRadius: '4px',
                          background: index === 0
                            ? 'linear-gradient(135deg, rgba(0,255,136,0.3), rgba(0,240,255,0.3))'
                            : 'rgba(255,255,255,0.08)',
                          color: index === 0 ? '#00ff88' : '#cbd5e1',
                          border: `1px solid ${index === 0 ? 'rgba(0,255,136,0.5)' : 'rgba(255,255,255,0.15)'}`,
                          boxShadow: index === 0 ? '0 0 8px rgba(0,255,136,0.3)' : 'none',
                        }}
                      >
                        {index === 0 ? '🌟 #1 PRIORITAS' : `#${index + 1}`}
                      </span>
                      <span style={{ color: meta.color, fontWeight: 'bold', fontSize: '0.58rem' }}>
                        {meta.label}
                      </span>
                    </div>

                    <div style={{ display: 'flex', alignItems: 'center', gap: '4px' }}>
                      <button
                        onClick={() => {
                          toggleObstacleEnabled(key);
                        }}
                        title={isEnabled ? "Klik untuk melewati obstacle ini dalam misi" : "Klik untuk mengaktifkan obstacle ini"}
                        style={{
                          padding: '1px 4px',
                          fontSize: '0.48rem',
                          fontWeight: '600',
                          borderRadius: '3px',
                          cursor: 'pointer',
                          background: isEnabled ? 'rgba(0,255,136,0.15)' : 'rgba(239,68,68,0.15)',
                          border: `1px solid ${isEnabled ? 'rgba(0,255,136,0.4)' : 'rgba(239,68,68,0.4)'}`,
                          color: isEnabled ? '#86efac' : '#fca5a5',
                        }}
                      >
                        {isEnabled ? '✓ AKTIF' : '⏭️ LEWATI'}
                      </button>
                      <span style={{ fontSize: '0.50rem', color: 'var(--text-tertiary)', fontFamily: 'monospace' }}>
                        X:{obs.x?.toFixed(1)}m | Z:{obs.z?.toFixed(1)}m
                      </span>
                    </div>
                  </div>

                  {/* Priority & Reordering Controls Toolbar */}
                  <div style={{ display: 'flex', alignItems: 'center', justifyContent: 'space-between', gap: '4px', marginBottom: '5px', background: 'rgba(255,255,255,0.02)', padding: '2px 4px', borderRadius: '4px' }}>
                    <div style={{ display: 'flex', alignItems: 'center', gap: '3px' }}>
                      <button
                        onClick={() => {
                          moveObstacleOrder(key, 'up');
                          const updated = [...obstacleOrder];
                          const idx = updated.indexOf(key);
                          if (idx > 0) {
                            const [m] = updated.splice(idx, 1);
                            updated.splice(idx - 1, 0, m);
                            topicPublisher.publishObstacleOrder(updated);
                          }
                        }}
                        disabled={index === 0}
                        title={`Pindahkan ${meta.label} ke urutan lebih awal`}
                        style={{
                          padding: '2px 5px',
                          fontSize: '0.50rem',
                          borderRadius: '3px',
                          background: 'rgba(255,255,255,0.06)',
                          border: '1px solid rgba(255,255,255,0.15)',
                          color: index === 0 ? 'rgba(255,255,255,0.2)' : 'var(--text-secondary)',
                          cursor: index === 0 ? 'not-allowed' : 'pointer',
                          fontWeight: 'bold',
                        }}
                      >
                        ▲ Naik
                      </button>
                      <button
                        onClick={() => {
                          moveObstacleOrder(key, 'down');
                          const updated = [...obstacleOrder];
                          const idx = updated.indexOf(key);
                          if (idx < updated.length - 1) {
                            const [m] = updated.splice(idx, 1);
                            updated.splice(idx + 1, 0, m);
                            topicPublisher.publishObstacleOrder(updated);
                          }
                        }}
                        disabled={index === obstacleOrder.length - 1}
                        title={`Pindahkan ${meta.label} ke urutan setelahnya`}
                        style={{
                          padding: '2px 5px',
                          fontSize: '0.50rem',
                          borderRadius: '3px',
                          background: 'rgba(255,255,255,0.06)',
                          border: '1px solid rgba(255,255,255,0.15)',
                          color: index === obstacleOrder.length - 1 ? 'rgba(255,255,255,0.2)' : 'var(--text-secondary)',
                          cursor: index === obstacleOrder.length - 1 ? 'not-allowed' : 'pointer',
                          fontWeight: 'bold',
                        }}
                      >
                        ▼ Turun
                      </button>
                      {index !== 0 && (
                        <button
                          onClick={() => {
                            setObstaclePriority(key, 0);
                            const updated = [...obstacleOrder];
                            const idx = updated.indexOf(key);
                            const [m] = updated.splice(idx, 1);
                            updated.unshift(m);
                            topicPublisher.publishObstacleOrder(updated);
                          }}
                          title={`Jadikan ${meta.label} sebagai target pertama (#1)`}
                          style={{
                            padding: '2px 5px',
                            fontSize: '0.50rem',
                            borderRadius: '3px',
                            background: 'rgba(0,255,136,0.12)',
                            border: '1px solid rgba(0,255,136,0.3)',
                            color: '#86efac',
                            cursor: 'pointer',
                            fontWeight: '600',
                          }}
                        >
                          ⭐ Jadikan #1
                        </button>
                      )}
                    </div>

                    <div style={{ display: 'flex', alignItems: 'center', gap: '3px' }}>
                      <span style={{ fontSize: '0.48rem', color: 'var(--text-tertiary)' }}>Posisi:</span>
                      <select
                        value={index}
                        onChange={(e) => {
                          const targetIdx = parseInt(e.target.value, 10);
                          setObstaclePriority(key, targetIdx);
                          const updated = [...obstacleOrder];
                          const curIdx = updated.indexOf(key);
                          const [m] = updated.splice(curIdx, 1);
                          updated.splice(targetIdx, 0, m);
                          topicPublisher.publishObstacleOrder(updated);
                        }}
                        style={{
                          background: '#0f172a',
                          border: '1px solid rgba(255,255,255,0.2)',
                          color: '#f8fafc',
                          fontSize: '0.50rem',
                          borderRadius: '3px',
                          padding: '1px 3px',
                          cursor: 'pointer',
                        }}
                      >
                        {obstacleOrder.map((_, i) => (
                          <option key={i} value={i}>
                            #{i + 1} {i === 0 ? '(Prioritas Utama)' : ''}
                          </option>
                        ))}
                      </select>
                    </div>
                  </div>

                  {/* TABRAK / MENGHINDAR Strategy Selector Buttons for Flares */}
                  {meta.isFlare && (
                    <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: '4px', marginBottom: '5px' }}>
                      <button
                        onClick={() => setFlareStrategy(key, 'TABRAK')}
                        title={`Pilih mode TABRAK untuk ${meta.label} (AUV akan menabrak hingga roboh)`}
                        style={{
                          padding: '3px 4px',
                          fontSize: '0.54rem',
                          fontWeight: 'bold',
                          borderRadius: '4px',
                          cursor: 'pointer',
                          display: 'flex',
                          alignItems: 'center',
                          justifyContent: 'center',
                          gap: '3px',
                          transition: 'all 0.2s ease',
                          border: currentStrategy === 'TABRAK'
                            ? '1px solid #ef4444'
                            : '1px solid rgba(255,255,255,0.12)',
                          background: currentStrategy === 'TABRAK'
                            ? 'linear-gradient(135deg, rgba(239, 68, 68, 0.4), rgba(185, 28, 28, 0.7))'
                            : 'rgba(255,255,255,0.03)',
                          color: currentStrategy === 'TABRAK' ? '#fecaca' : 'var(--text-tertiary)',
                          boxShadow: currentStrategy === 'TABRAK' ? '0 0 10px rgba(239, 68, 68, 0.45)' : 'none',
                        }}
                      >
                        💥 TABRAK
                      </button>
                      <button
                        onClick={() => setFlareStrategy(key, 'MENGHINDAR')}
                        title={`Pilih mode MENGHINDAR untuk ${meta.label} (AUV akan inspeksi / melewati aman tanpa kontak)`}
                        style={{
                          padding: '3px 4px',
                          fontSize: '0.54rem',
                          fontWeight: 'bold',
                          borderRadius: '4px',
                          cursor: 'pointer',
                          display: 'flex',
                          alignItems: 'center',
                          justifyContent: 'center',
                          gap: '3px',
                          transition: 'all 0.2s ease',
                          border: currentStrategy === 'MENGHINDAR'
                            ? '1px solid #00f0ff'
                            : '1px solid rgba(255,255,255,0.12)',
                          background: currentStrategy === 'MENGHINDAR'
                            ? 'linear-gradient(135deg, rgba(0, 240, 255, 0.35), rgba(2, 132, 199, 0.6))'
                            : 'rgba(255,255,255,0.03)',
                          color: currentStrategy === 'MENGHINDAR' ? '#a5f3fc' : 'var(--text-tertiary)',
                          boxShadow: currentStrategy === 'MENGHINDAR' ? '0 0 10px rgba(0, 240, 255, 0.45)' : 'none',
                        }}
                      >
                        🛡️ MENGHINDAR
                      </button>
                    </div>
                  )}

                  {/* Coordinate Sliders */}
                  <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: '6px' }}>
                    <div style={{ display: 'flex', alignItems: 'center', gap: '3px' }}>
                      <span style={{ fontSize: '0.52rem', color: 'var(--text-tertiary)' }}>X:</span>
                      <input
                        type="range"
                        min="-10.0"
                        max="12.0"
                        step="0.1"
                        value={obs.x ?? 0}
                        onChange={(e) => setObstaclePos(key, e.target.value, obs.z ?? 0)}
                        style={{ width: '100%', accentColor: meta.color }}
                      />
                    </div>
                    <div style={{ display: 'flex', alignItems: 'center', gap: '3px' }}>
                      <span style={{ fontSize: '0.52rem', color: 'var(--text-tertiary)' }}>Z:</span>
                      <input
                        type="range"
                        min="-6.5"
                        max="6.5"
                        step="0.1"
                        value={obs.z ?? 0}
                        onChange={(e) => setObstaclePos(key, obs.x ?? 0, e.target.value)}
                        style={{ width: '100%', accentColor: meta.color }}
                      />
                    </div>
                  </div>
                </div>
              );
            })}
          </div>
        </div>
      )}
    </motion.div>
  );
}
