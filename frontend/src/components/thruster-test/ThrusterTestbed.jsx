import { useState, useEffect, useRef } from 'react';
import {
  LineChart,
  Line,
  XAxis,
  YAxis,
  Tooltip,
  ResponsiveContainer,
  Legend,
  CartesianGrid,
} from 'recharts';
import SingleThrusterScene from '../3d/SingleThrusterScene';
import thrusterTwinEngine from '../../dt-core/ThrusterTwinEngine';
import webSerialManager from '../../services/WebSerialManager';
import ArduinoFirmwareModal from './ArduinoFirmwareModal';

export default function ThrusterTestbed() {
  // Actuator PWM State
  const [pwm, setPwm] = useState(1500);
  const [armed, setArmed] = useState(true);
  const [voltage, setVoltage] = useState(16.0);
  const [isFirmwareModalOpen, setFirmwareModalOpen] = useState(false);

  // Connection State
  const [isSerialConnected, setIsSerialConnected] = useState(false);
  const [serialError, setSerialError] = useState(null);
  const [hardwareMode, setHardwareMode] = useState('simulated'); // 'usb' | 'simulated'
  const [simDefect, setSimDefect] = useState(0.0); // 0% to 30% propeller wear

  // Real-Time Telemetry & Comparison State
  const [telemetry, setTelemetry] = useState({
    rpmDT: 0,
    rpmReal: 0,
    thrustDT: 0,
    thrustReal: 0,
    currentDT: 0.25,
    currentReal: 0.25,
    voltageReal: 16.0,
    errorThrust: 0,
    estimatedKt: 3.55e-6,
    efficiency: 100.0,
  });

  // Oscilloscope History Buffer
  const [history, setHistory] = useState([]);
  const historyRef = useRef([]);

  // Auto-Test Routine
  const [testRoutine, setTestRoutine] = useState('none'); // 'none' | 'step' | 'sweep'
  const routineTimerRef = useRef(null);

  // Sync Voltage with DT Engine
  useEffect(() => {
    thrusterTwinEngine.setVoltage(voltage);
  }, [voltage]);

  // Subscribe to Telemetry from WebSerial / Simulator
  useEffect(() => {
    const unsubscribe = webSerialManager.subscribe((msg) => {
      if (msg.type === 'status') {
        setIsSerialConnected(msg.connected);
        if (msg.connected) {
          setHardwareMode('usb');
          setSerialError(null);
        }
      } else if (msg.type === 'telemetry') {
        // Run Digital Twin Model Step
        const activePwm = armed ? pwm : 1500;
        const dtResult = thrusterTwinEngine.step(activePwm, 0.05, msg.thrustReal, msg.rpmReal);

        const currentTelemetry = {
          rpmDT: dtResult.rpmDT,
          rpmReal: msg.rpmReal,
          thrustDT: dtResult.thrustDT,
          thrustReal: msg.thrustReal,
          currentDT: dtResult.currentDT,
          currentReal: msg.currentReal,
          voltageReal: msg.voltageReal || voltage,
          errorThrust: msg.thrustReal - dtResult.thrustDT,
          estimatedKt: dtResult.estimatedKt,
          efficiency: dtResult.efficiency,
        };

        setTelemetry(currentTelemetry);

        // Update Oscilloscope Buffer (keep last 40 samples)
        const timeLabel = new Date().toLocaleTimeString('en-US', {
          minute: '2-digit',
          second: '2-digit',
        });

        const newPoint = {
          time: timeLabel,
          ThrustReal: Number(msg.thrustReal.toFixed(2)),
          ThrustDT: Number(dtResult.thrustDT.toFixed(2)),
          Error: Number((msg.thrustReal - dtResult.thrustDT).toFixed(2)),
          RpmReal: Math.round(msg.rpmReal),
          RpmDT: Math.round(dtResult.rpmDT),
        };

        const updated = [...historyRef.current.slice(-35), newPoint];
        historyRef.current = updated;
        setHistory(updated);
      }
    });

    return () => unsubscribe();
  }, [pwm, armed, voltage]);

  // Handle PWM Change & Send Command to Physical Serial
  // SAFETY: limit 1300-1600 µs — full range (1100-1900) pernah memecahkan propeller di bench test
  const handlePwmChange = (newPwm) => {
    const clamped = Math.max(1300, Math.min(1600, Number(newPwm)));
    setPwm(clamped);
    if (armed) {
      webSerialManager.sendPwm(clamped);
    }
  };

  // Connect Web Serial
  const handleConnectUsb = async () => {
    try {
      setSerialError(null);
      await webSerialManager.connect(115200);
    } catch (err) {
      setSerialError(err.message || 'Gagal terhubung ke USB COM Port');
    }
  };

  // Disconnect Web Serial
  const handleDisconnectUsb = async () => {
    await webSerialManager.disconnect();
    setHardwareMode('simulated');
    webSerialManager.toggleSimulator(true);
  };

  // Switch Hardware Mode
  const handleToggleHardwareMode = (mode) => {
    setHardwareMode(mode);
    if (mode === 'simulated') {
      webSerialManager.toggleSimulator(true);
    } else {
      if (!isSerialConnected) {
        handleConnectUsb();
      }
    }
  };

  // Reset System ID / RLS
  const handleResetRLS = () => {
    thrusterTwinEngine.resetRLS();
  };

  // Automated Step Response Routine
  const runStepTest = () => {
    if (testRoutine !== 'none') return;
    setTestRoutine('step');
    handlePwmChange(1500);

    setTimeout(() => {
      handlePwmChange(1550); // 50% forward step (safe range)
      setTimeout(() => {
        handlePwmChange(1500); // Return to neutral
        setTestRoutine('none');
      }, 3500);
    }, 1500);
  };

  // Automated Ramp Sweep Routine
  const runSweepTest = () => {
    if (testRoutine !== 'none') return;
    setTestRoutine('sweep');
    let currentStep = 1500;
    const interval = setInterval(() => {
      currentStep += 10;
      if (currentStep > 1600) {
        clearInterval(interval);
        setTimeout(() => {
          handlePwmChange(1500);
          setTestRoutine('none');
        }, 1000);
      } else {
        handlePwmChange(currentStep);
      }
    }, 100);
  };

  return (
    <div
      style={{
        display: 'flex',
        flexDirection: 'column',
        height: 'calc(100vh - var(--header-height, 56px))',
        background: 'var(--bg-primary, #0a0e17)',
        color: 'var(--text-primary, #f0f4f8)',
        overflow: 'hidden',
      }}
    >
      {/* 1. TOP SUB-HEADER: Hardware Link & Telemetry Status Bar */}
      <div
        style={{
          display: 'flex',
          alignItems: 'center',
          justifyContent: 'space-between',
          padding: '8px 20px',
          background: 'var(--bg-secondary, #111827)',
          borderBottom: '1px solid rgba(148, 163, 184, 0.12)',
          flexShrink: 0,
        }}
      >
        <div style={{ display: 'flex', alignItems: 'center', gap: '12px' }}>
          <div
            style={{
              display: 'flex',
              alignItems: 'center',
              gap: '8px',
              fontSize: '0.9rem',
              fontWeight: 700,
              color: 'var(--accent-cyan, #00f0ff)',
            }}
          >
            <span>⚡</span>
            <span>T200 HIL TESTBED DIGITAL TWIN</span>
          </div>

          <span
            style={{
              fontSize: '0.65rem',
              padding: '3px 8px',
              borderRadius: '4px',
              background: 'rgba(0, 240, 255, 0.12)',
              color: '#00f0ff',
              border: '1px solid rgba(0, 240, 255, 0.3)',
              fontFamily: 'monospace',
            }}
          >
            1x Physical Propeller Channel
          </span>
        </div>

        {/* Center: Hardware Link Buttons & Selectors */}
        <div style={{ display: 'flex', alignItems: 'center', gap: '10px' }}>
          {/* Mode Switcher */}
          <div
            style={{
              display: 'flex',
              background: 'rgba(15, 23, 42, 0.8)',
              padding: '2px',
              borderRadius: '6px',
              border: '1px solid rgba(148, 163, 184, 0.15)',
            }}
          >
            <button
              onClick={() => handleToggleHardwareMode('simulated')}
              style={{
                background: hardwareMode === 'simulated' ? 'rgba(0, 240, 255, 0.2)' : 'transparent',
                color: hardwareMode === 'simulated' ? '#00f0ff' : '#94a3b8',
                border: 'none',
                padding: '4px 10px',
                borderRadius: '4px',
                fontSize: '0.7rem',
                fontWeight: 600,
                cursor: 'pointer',
              }}
            >
              💻 Hardware Simulator
            </button>
            <button
              onClick={() => handleToggleHardwareMode('usb')}
              style={{
                background: hardwareMode === 'usb' ? 'rgba(0, 255, 136, 0.2)' : 'transparent',
                color: hardwareMode === 'usb' ? '#00ff88' : '#94a3b8',
                border: 'none',
                padding: '4px 10px',
                borderRadius: '4px',
                fontSize: '0.7rem',
                fontWeight: 600,
                cursor: 'pointer',
              }}
            >
              🔌 USB Serial (COM)
            </button>
          </div>

          {/* Connect USB Button */}
          {hardwareMode === 'usb' && !isSerialConnected && (
            <button
              onClick={handleConnectUsb}
              style={{
                background: 'linear-gradient(135deg, #00f0ff 0%, #0080ff 100%)',
                color: '#000',
                border: 'none',
                padding: '5px 12px',
                borderRadius: '6px',
                fontSize: '0.72rem',
                fontWeight: 700,
                cursor: 'pointer',
                display: 'flex',
                alignItems: 'center',
                gap: '6px',
              }}
            >
              🔌 Connect USB Port
            </button>
          )}

          {hardwareMode === 'usb' && isSerialConnected && (
            <button
              onClick={handleDisconnectUsb}
              style={{
                background: 'rgba(255, 59, 92, 0.2)',
                color: '#ff3b5c',
                border: '1px solid rgba(255, 59, 92, 0.4)',
                padding: '5px 12px',
                borderRadius: '6px',
                fontSize: '0.72rem',
                fontWeight: 700,
                cursor: 'pointer',
              }}
            >
              Disconnect USB
            </button>
          )}

          {/* Safety Arm / Disarm */}
          <button
            onClick={() => {
              const next = !armed;
              setArmed(next);
              if (!next) {
                handlePwmChange(1500);
              }
            }}
            style={{
              background: armed ? 'rgba(0, 255, 136, 0.15)' : 'rgba(255, 59, 92, 0.15)',
              border: `1px solid ${armed ? 'rgba(0, 255, 136, 0.4)' : 'rgba(255, 59, 92, 0.4)'}`,
              color: armed ? '#00ff88' : '#ff3b5c',
              padding: '5px 12px',
              borderRadius: '6px',
              fontSize: '0.72rem',
              fontWeight: 700,
              cursor: 'pointer',
            }}
          >
            {armed ? '⚡ MOTOR ARMED' : '⛔ DISARMED'}
          </button>

          {/* Wiring Guide Button */}
          <button
            onClick={() => setFirmwareModalOpen(true)}
            style={{
              background: 'rgba(168, 85, 247, 0.15)',
              border: '1px solid rgba(168, 85, 247, 0.4)',
              color: '#c084fc',
              padding: '5px 12px',
              borderRadius: '6px',
              fontSize: '0.72rem',
              fontWeight: 700,
              cursor: 'pointer',
              display: 'flex',
              alignItems: 'center',
              gap: '6px',
            }}
          >
            📖 Skema Kabel & Firmware
          </button>
        </div>

        {/* Right Voltage Selector */}
        <div style={{ display: 'flex', alignItems: 'center', gap: '8px', fontSize: '0.72rem' }}>
          <span style={{ color: '#94a3b8' }}>PSU Voltage:</span>
          <select
            value={voltage}
            onChange={(e) => setVoltage(Number(e.target.value))}
            style={{
              background: '#0f172a',
              border: '1px solid rgba(148, 163, 184, 0.2)',
              borderRadius: '4px',
              color: '#00f0ff',
              padding: '3px 8px',
              fontSize: '0.72rem',
              fontWeight: 600,
            }}
          >
            <option value={12.0}>12.0 V (3S LiPo)</option>
            <option value={16.0}>16.0 V (4S Nominal)</option>
            <option value={20.0}>20.0 V (Max PSU)</option>
          </select>
        </div>
      </div>

      {serialError && (
        <div
          style={{
            background: 'rgba(255, 59, 92, 0.15)',
            borderBottom: '1px solid rgba(255, 59, 92, 0.4)',
            color: '#ff3b5c',
            padding: '6px 20px',
            fontSize: '0.75rem',
          }}
        >
          ⚠️ Error Serial: {serialError} (Gunakan Google Chrome / Microsoft Edge untuk fitur Web Serial API)
        </div>
      )}

      {/* 2. MAIN BODY: Split Grid (Left Controls, Center 3D + KPIs, Bottom Chart) */}
      <div
        style={{
          display: 'grid',
          gridTemplateColumns: '320px 1fr',
          gridTemplateRows: '1fr 220px',
          flex: 1,
          overflow: 'hidden',
          gap: '8px',
          padding: '8px',
        }}
      >
        {/* LEFT COLUMN: Actuator Command & Automated Tests */}
        <div
          style={{
            background: 'var(--bg-secondary, #111827)',
            border: '1px solid rgba(148, 163, 184, 0.12)',
            borderRadius: '12px',
            padding: '16px',
            display: 'flex',
            flexDirection: 'column',
            gap: '14px',
            overflowY: 'auto',
          }}
        >
          <div style={{ borderBottom: '1px solid rgba(148, 163, 184, 0.1)', paddingBottom: '8px' }}>
            <h4 style={{ margin: 0, fontSize: '0.85rem', color: '#00f0ff', fontWeight: 700 }}>
              🎮 ACTUATOR COMMAND (PWM)
            </h4>
            <span style={{ fontSize: '0.7rem', color: '#94a3b8' }}>Digital Twin → Physical ESC</span>
          </div>

          {/* Big PWM Display & Slider */}
          <div
            style={{
              background: '#0a0e17',
              border: '1px solid rgba(0, 240, 255, 0.25)',
              borderRadius: '8px',
              padding: '14px',
              textAlign: 'center',
            }}
          >
            <div style={{ fontSize: '0.7rem', color: '#94a3b8', marginBottom: '4px' }}>
              TARGET PULSE WIDTH
            </div>
            <div
              style={{
                fontSize: '2rem',
                fontWeight: 800,
                fontFamily: 'monospace',
                color: pwm === 1500 ? '#94a3b8' : pwm > 1500 ? '#00ff88' : '#ff8c00',
              }}
            >
              {pwm} <span style={{ fontSize: '0.9rem' }}>µs</span>
            </div>
            <div style={{ fontSize: '0.72rem', color: '#cbd5e1', marginTop: '2px' }}>
              {pwm === 1500
                ? 'NEUTRAL (STOP)'
                : pwm > 1500
                ? `FORWARD (+${(((pwm - 1500) / 100) * 100).toFixed(0)}%)`
                : `REVERSE (-${(((1500 - pwm) / 200) * 100).toFixed(0)}%)`}
            </div>

            {/* Range Slider */}
            <input
              type="range"
              min="1300"
              max="1600"
              step="5"
              value={pwm}
              disabled={!armed || testRoutine !== 'none'}
              onChange={(e) => handlePwmChange(e.target.value)}
              style={{
                width: '100%',
                marginTop: '12px',
                accentColor: '#00f0ff',
                cursor: armed ? 'pointer' : 'not-allowed',
              }}
            />

            <div
              style={{
                display: 'flex',
                justifyContent: 'space-between',
                fontSize: '0.65rem',
                color: '#64748b',
                marginTop: '4px',
                fontFamily: 'monospace',
              }}
            >
              <span>1300 µs (Rev)</span>
              <span>1500 µs</span>
              <span>1600 µs (Fwd)</span>
            </div>
          </div>

          {/* Quick Presets */}
          <div>
            <div style={{ fontSize: '0.7rem', color: '#94a3b8', marginBottom: '6px', fontWeight: 600 }}>
              QUICK COMMAND PRESETS
            </div>
            <div style={{ display: 'grid', gridTemplateColumns: 'repeat(3, 1fr)', gap: '6px' }}>
              <button
                onClick={() => handlePwmChange(1525)}
                disabled={!armed}
                style={presetButtonStyle('#00ff88')}
              >
                +25% (1525)
              </button>
              <button
                onClick={() => handlePwmChange(1550)}
                disabled={!armed}
                style={presetButtonStyle('#00ff88')}
              >
                +50% (1550)
              </button>
              <button
                onClick={() => handlePwmChange(1600)}
                disabled={!armed}
                style={presetButtonStyle('#00ff88')}
              >
                +100% (1600)
              </button>
              <button
                onClick={() => handlePwmChange(1450)}
                disabled={!armed}
                style={presetButtonStyle('#ff8c00')}
              >
                -25% (1450)
              </button>
              <button
                onClick={() => handlePwmChange(1400)}
                disabled={!armed}
                style={presetButtonStyle('#ff8c00')}
              >
                -50% (1400)
              </button>
              <button
                onClick={() => handlePwmChange(1300)}
                disabled={!armed}
                style={presetButtonStyle('#ff8c00')}
              >
                -100% (1300)
              </button>
            </div>
            <button
              onClick={() => handlePwmChange(1500)}
              style={{
                width: '100%',
                marginTop: '6px',
                padding: '8px',
                background: 'rgba(255, 59, 92, 0.2)',
                border: '1px solid rgba(255, 59, 92, 0.4)',
                color: '#ff3b5c',
                borderRadius: '6px',
                fontSize: '0.75rem',
                fontWeight: 700,
                cursor: 'pointer',
              }}
            >
              🛑 EMERGENCY STOP (1500 µs)
            </button>
          </div>

          {/* Automated Test Routines */}
          <div style={{ borderTop: '1px solid rgba(148, 163, 184, 0.1)', paddingTop: '10px' }}>
            <div style={{ fontSize: '0.7rem', color: '#94a3b8', marginBottom: '6px', fontWeight: 600 }}>
              AUTOMATED DYNAMIC TESTS
            </div>
            <div style={{ display: 'flex', flexDirection: 'column', gap: '6px' }}>
              <button
                onClick={runStepTest}
                disabled={!armed || testRoutine !== 'none'}
                style={{
                  padding: '7px',
                  background: testRoutine === 'step' ? '#00f0ff' : 'rgba(0, 240, 255, 0.15)',
                  color: testRoutine === 'step' ? '#000' : '#00f0ff',
                  border: '1px solid rgba(0, 240, 255, 0.3)',
                  borderRadius: '6px',
                  fontSize: '0.72rem',
                  fontWeight: 600,
                  cursor: 'pointer',
                }}
              >
                {testRoutine === 'step' ? '⏳ Running Step Test...' : '📈 Step Response Test (1500 → 1700 µs)'}
              </button>

              <button
                onClick={runSweepTest}
                disabled={!armed || testRoutine !== 'none'}
                style={{
                  padding: '7px',
                  background: testRoutine === 'sweep' ? '#00f0ff' : 'rgba(0, 240, 255, 0.15)',
                  color: testRoutine === 'sweep' ? '#000' : '#00f0ff',
                  border: '1px solid rgba(0, 240, 255, 0.3)',
                  borderRadius: '6px',
                  fontSize: '0.72rem',
                  fontWeight: 600,
                  cursor: 'pointer',
                }}
              >
                {testRoutine === 'sweep' ? '⏳ Sweeping PWM...' : '🚀 Ramp Curve Sweep (1500 → 1850 µs)'}
              </button>
            </div>
          </div>

          {/* Physical Simulation Tweaks (if in simulator mode) */}
          {hardwareMode === 'simulated' && (
            <div style={{ borderTop: '1px solid rgba(148, 163, 184, 0.1)', paddingTop: '10px' }}>
              <div style={{ fontSize: '0.7rem', color: '#94a3b8', marginBottom: '4px', fontWeight: 600 }}>
                SIMULASI DEFEK / KAVITASI FISIK
              </div>
              <div style={{ display: 'flex', alignItems: 'center', justifyContent: 'space-between' }}>
                <span style={{ fontSize: '0.68rem', color: '#cbd5e1' }}>Propeller Fouling:</span>
                <span style={{ fontSize: '0.68rem', color: '#ff8c00', fontWeight: 'bold' }}>
                  {(simDefect * 100).toFixed(0)}% loss
                </span>
              </div>
              <input
                type="range"
                min="0.0"
                max="0.4"
                step="0.05"
                value={simDefect}
                onChange={(e) => {
                  const val = Number(e.target.value);
                  setSimDefect(val);
                  webSerialManager.setSimulatorDefect(val);
                }}
                style={{ width: '100%', marginTop: '4px', accentColor: '#ff8c00' }}
              />
            </div>
          )}
        </div>

        {/* CENTER COLUMN: 3D Thruster Viewport + Real-Time Telemetry Comparison Cards */}
        <div
          style={{
            display: 'flex',
            flexDirection: 'column',
            gap: '8px',
            overflow: 'hidden',
          }}
        >
          {/* 3D Thruster Viewport */}
          <div
            style={{
              flex: 1,
              background: '#0a0e17',
              border: '1px solid rgba(148, 163, 184, 0.12)',
              borderRadius: '12px',
              position: 'relative',
              overflow: 'hidden',
            }}
          >
            <SingleThrusterScene
              rpm={telemetry.rpmReal}
              thrust={telemetry.thrustReal}
              targetRpm={telemetry.rpmDT}
            />
          </div>

          {/* Side-by-Side KPI Cards: Digital Twin vs Physical Reality */}
          <div
            style={{
              display: 'grid',
              gridTemplateColumns: 'repeat(5, 1fr)',
              gap: '8px',
              flexShrink: 0,
            }}
          >
            {/* KPI 1: Thrust (N) */}
            <div style={kpiCardStyle}>
              <div style={{ fontSize: '0.65rem', color: '#94a3b8' }}>THRUST FORCE</div>
              <div style={{ display: 'flex', alignItems: 'baseline', gap: '6px', marginTop: '2px' }}>
                <span style={{ fontSize: '1.2rem', fontWeight: 800, color: '#00f0ff' }}>
                  {telemetry.thrustReal.toFixed(2)}
                </span>
                <span style={{ fontSize: '0.7rem', color: '#64748b' }}>N (Real)</span>
              </div>
              <div style={{ fontSize: '0.65rem', color: '#cbd5e1', marginTop: '2px' }}>
                DT Pred: <strong style={{ color: '#ffd700' }}>{telemetry.thrustDT.toFixed(2)} N</strong>
              </div>
              <div
                style={{
                  fontSize: '0.62rem',
                  fontWeight: 600,
                  color: Math.abs(telemetry.errorThrust) < 1.0 ? '#00ff88' : '#ff3b5c',
                  marginTop: '2px',
                }}
              >
                Error: {telemetry.errorThrust > 0 ? '+' : ''}
                {telemetry.errorThrust.toFixed(2)} N
              </div>
            </div>

            {/* KPI 2: RPM */}
            <div style={kpiCardStyle}>
              <div style={{ fontSize: '0.65rem', color: '#94a3b8' }}>ROTATION SPEED</div>
              <div style={{ display: 'flex', alignItems: 'baseline', gap: '6px', marginTop: '2px' }}>
                <span style={{ fontSize: '1.2rem', fontWeight: 800, color: '#00ff88' }}>
                  {Math.round(telemetry.rpmReal)}
                </span>
                <span style={{ fontSize: '0.7rem', color: '#64748b' }}>RPM</span>
              </div>
              <div style={{ fontSize: '0.65rem', color: '#cbd5e1', marginTop: '2px' }}>
                DT: <strong style={{ color: '#ffd700' }}>{Math.round(telemetry.rpmDT)} RPM</strong>
              </div>
              <div style={{ fontSize: '0.62rem', color: '#94a3b8', marginTop: '2px' }}>
                Diff: {Math.round(telemetry.rpmReal - telemetry.rpmDT)} RPM
              </div>
            </div>

            {/* KPI 3: Current & Power */}
            <div style={kpiCardStyle}>
              <div style={{ fontSize: '0.65rem', color: '#94a3b8' }}>CURRENT & POWER</div>
              <div style={{ display: 'flex', alignItems: 'baseline', gap: '6px', marginTop: '2px' }}>
                <span style={{ fontSize: '1.2rem', fontWeight: 800, color: '#f59e0b' }}>
                  {telemetry.currentReal.toFixed(2)}
                </span>
                <span style={{ fontSize: '0.7rem', color: '#64748b' }}>A</span>
              </div>
              <div style={{ fontSize: '0.65rem', color: '#cbd5e1', marginTop: '2px' }}>
                Power: {(telemetry.currentReal * telemetry.voltageReal).toFixed(1)} W
              </div>
              <div style={{ fontSize: '0.62rem', color: '#94a3b8', marginTop: '2px' }}>
                @ {telemetry.voltageReal.toFixed(1)} V
              </div>
            </div>

            {/* KPI 4: Thrust Coefficient Kt (RLS) */}
            <div style={kpiCardStyle}>
              <div style={{ fontSize: '0.65rem', color: '#94a3b8' }}>COEFF IDENT (RLS)</div>
              <div style={{ display: 'flex', alignItems: 'baseline', gap: '6px', marginTop: '2px' }}>
                <span style={{ fontSize: '1.1rem', fontWeight: 800, color: '#c084fc', fontFamily: 'monospace' }}>
                  {(telemetry.estimatedKt * 1e6).toFixed(2)}
                </span>
                <span style={{ fontSize: '0.65rem', color: '#64748b' }}>×10⁻⁶</span>
              </div>
              <div style={{ fontSize: '0.65rem', color: '#cbd5e1', marginTop: '2px' }}>
                Nominal: 3.55 ×10⁻⁶
              </div>
              <button
                onClick={handleResetRLS}
                style={{
                  marginTop: '3px',
                  background: 'transparent',
                  border: 'none',
                  color: '#00f0ff',
                  fontSize: '0.62rem',
                  textDecoration: 'underline',
                  cursor: 'pointer',
                  padding: 0,
                }}
              >
                Reset RLS
              </button>
            </div>

            {/* KPI 5: Efficiency & Health Score */}
            <div style={kpiCardStyle}>
              <div style={{ fontSize: '0.65rem', color: '#94a3b8' }}>HEALTH / EFFICIENCY</div>
              <div style={{ display: 'flex', alignItems: 'baseline', gap: '6px', marginTop: '2px' }}>
                <span
                  style={{
                    fontSize: '1.2rem',
                    fontWeight: 800,
                    color:
                      telemetry.efficiency >= 90
                        ? '#00ff88'
                        : telemetry.efficiency >= 75
                        ? '#f59e0b'
                        : '#ff3b5c',
                  }}
                >
                  {telemetry.efficiency.toFixed(1)}%
                </span>
              </div>
              <div style={{ fontSize: '0.65rem', color: '#cbd5e1', marginTop: '2px' }}>
                Status:{' '}
                <strong
                  style={{
                    color: telemetry.efficiency >= 85 ? '#00ff88' : '#ff8c00',
                  }}
                >
                  {telemetry.efficiency >= 85 ? 'OPTIMAL' : 'DEGRADED'}
                </strong>
              </div>
              <div style={{ fontSize: '0.62rem', color: '#94a3b8', marginTop: '2px' }}>
                {hardwareMode === 'usb' ? 'Live Telemetry' : 'Simulated'}
              </div>
            </div>
          </div>
        </div>

        {/* 3. BOTTOM ROW: Live Real-Time Oscilloscope Chart */}
        <div
          style={{
            gridColumn: '1 / -1',
            background: 'var(--bg-secondary, #111827)',
            border: '1px solid rgba(148, 163, 184, 0.12)',
            borderRadius: '12px',
            padding: '10px 16px',
            display: 'flex',
            flexDirection: 'column',
            overflow: 'hidden',
          }}
        >
          <div
            style={{
              display: 'flex',
              justifyContent: 'space-between',
              alignItems: 'center',
              marginBottom: '6px',
            }}
          >
            <div style={{ display: 'flex', alignItems: 'center', gap: '8px' }}>
              <span style={{ fontSize: '0.8rem', fontWeight: 700, color: '#00f0ff' }}>
                📊 LIVE TELEMETRY OSCILLOSCOPE
              </span>
              <span style={{ fontSize: '0.68rem', color: '#94a3b8' }}>
                Perbandingan Gaya Dorong: Fisik Real (Cyan) vs Prediksi Digital Twin (Gold) vs Error (Merah)
              </span>
            </div>

            <div style={{ fontSize: '0.68rem', color: '#64748b', fontFamily: 'monospace' }}>
              Samples: {history.length} pts (20 Hz)
            </div>
          </div>

          <div style={{ flex: 1, width: '100%', minHeight: 0 }}>
            <ResponsiveContainer width="100%" height="100%">
              <LineChart data={history}>
                <CartesianGrid strokeDasharray="3 3" stroke="rgba(148, 163, 184, 0.08)" />
                <XAxis dataKey="time" stroke="#64748b" fontSize={10} tickLine={false} />
                <YAxis stroke="#64748b" fontSize={10} domain={['auto', 'auto']} unit=" N" />
                <Tooltip
                  contentStyle={{
                    background: '#0f172a',
                    borderColor: 'rgba(0, 240, 255, 0.3)',
                    fontSize: '0.72rem',
                  }}
                />
                <Legend wrapperStyle={{ fontSize: '0.72rem' }} />
                <Line
                  type="monotone"
                  dataKey="ThrustReal"
                  name="T_real (Fisik)"
                  stroke="#00f0ff"
                  strokeWidth={2}
                  dot={false}
                  isAnimationActive={false}
                />
                <Line
                  type="monotone"
                  dataKey="ThrustDT"
                  name="T_DT (Digital Twin)"
                  stroke="#ffd700"
                  strokeWidth={2}
                  strokeDasharray="4 4"
                  dot={false}
                  isAnimationActive={false}
                />
                <Line
                  type="monotone"
                  dataKey="Error"
                  name="Error (e_T)"
                  stroke="#ff3b5c"
                  strokeWidth={1.5}
                  dot={false}
                  isAnimationActive={false}
                />
              </LineChart>
            </ResponsiveContainer>
          </div>
        </div>
      </div>

      {/* Arduino Firmware & Wiring Diagram Modal */}
      <ArduinoFirmwareModal
        isOpen={isFirmwareModalOpen}
        onClose={() => setFirmwareModalOpen(false)}
      />
    </div>
  );
}

const presetButtonStyle = (color) => ({
  background: 'rgba(15, 23, 42, 0.8)',
  border: '1px solid rgba(148, 163, 184, 0.15)',
  color,
  borderRadius: '6px',
  padding: '6px 4px',
  fontSize: '0.68rem',
  fontWeight: 600,
  cursor: 'pointer',
  transition: 'all 0.15s ease',
});

const kpiCardStyle = {
  background: 'rgba(15, 23, 42, 0.65)',
  border: '1px solid rgba(148, 163, 184, 0.1)',
  borderRadius: '8px',
  padding: '8px 12px',
  display: 'flex',
  flexDirection: 'column',
  justifyContent: 'center',
};
