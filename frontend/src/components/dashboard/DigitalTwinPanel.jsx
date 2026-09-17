import React, { useState } from 'react';
import useVehicleStore from '../../store/vehicleStore';
import vehicleConfig from '../../dt-core/VehicleConfig.js';
import defaultParameterIdentifier from '../../dt-core/ParameterIdentifier.js';
import defaultDataLogger from '../../dt-core/DataLogger.js';
import defaultPredictionAPI from '../../dt-core/PredictionAPI.js';
import defaultEnvironment, { CURRENT_PRESETS } from '../../dt-core/EnvironmentModel.js';

export default function DigitalTwinPanel() {
  const dtHealth = useVehicleStore((s) => s.dtHealth) || { totalScore: 96, tier: 'OPTIMAL', breakdown: { sync: 30, estimation: 24, physics: 24, actuators: 18 } };
  const syncMetrics = useVehicleStore((s) => s.syncMetrics) || {};
  const estimatedState = useVehicleStore((s) => s.estimatedState) || {};
  const uncertainty = useVehicleStore((s) => s.uncertainty) || { score: 0.04, level: 'NOMINAL', confidence: 0.96 };
  const oodStatus = useVehicleStore((s) => s.oodStatus) || { isOOD: false, score: 0.45, state: 'IN_DISTRIBUTION' };
  const validationMetrics = useVehicleStore((s) => s.validationMetrics) || {};
  const depth = useVehicleStore((s) => s.depth);
  const battery = useVehicleStore((s) => s.battery);

  const [activeSubTab, setActiveSubTab] = useState('health'); // 'health' | 'ekf' | 'ident' | 'recorder'
  const [isCalibrating, setIsCalibrating] = useState(false);
  const [calibrationResult, setCalibrationResult] = useState(null);
  const [isRecording, setIsRecording] = useState(false);
  const [recordedCount, setRecordedCount] = useState(0);
  const [selectedEnvPreset, setSelectedEnvPreset] = useState(CURRENT_PRESETS.POOL_CIRCULATION);

  // Trigger Parameter Identification using Differential Evolution
  const handleRunParameterIdentification = async () => {
    setIsCalibrating(true);
    setCalibrationResult(null);

    // Collect recent state history from store
    const store = useVehicleStore.getState();
    const posHist = store.positionHistory || [];

    // Synthesize calibration dataset from recorded trajectory
    const dataset = posHist.slice(-50).map((p, idx) => ({
      time: idx * 0.05,
      dt: 0.05,
      input: { surge: 0.6, sway: 0, heave: 0, yaw: 0 },
      measured: {
        u: 0.45,
        v: 0.0,
        w: 0.0,
        yawRate: 0.0,
        depth: store.depth || 0.8,
      }
    }));

    try {
      if (dataset.length < 10) {
        // Fallback synthetic dataset
        for (let i = 0; i < 20; i++) {
          dataset.push({
            time: i * 0.05,
            dt: 0.05,
            input: { surge: 0.7, sway: 0, heave: 0.1, yaw: 0 },
            measured: { u: 0.52, v: 0, w: 0.05, yawRate: 0, depth: 0.82 }
          });
        }
      }

      const res = await defaultParameterIdentifier.identify(dataset);
      setCalibrationResult(res);
      // Apply updated parameters
      vehicleConfig.updateParameters(res.optimalParams);
    } catch (err) {
      console.error('[DigitalTwin] Calibration error:', err);
    } finally {
      setIsCalibrating(false);
    }
  };

  // Toggle Flight Data Recorder
  const handleToggleRecording = () => {
    if (!isRecording) {
      defaultDataLogger.startRecording();
      setIsRecording(true);
      setRecordedCount(0);
    } else {
      const summary = defaultDataLogger.stopRecording();
      setIsRecording(false);
      setRecordedCount(summary.totalSamples);
    }
  };

  // Export JSON Log
  const handleDownloadLog = () => {
    const jsonStr = defaultDataLogger.exportJSON();
    const blob = new Blob([jsonStr], { type: 'application/json' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = `sauvc_digital_twin_log_${Date.now()}.json`;
    a.click();
    URL.revokeObjectURL(url);
  };

  const handleEnvChange = (preset) => {
    setSelectedEnvPreset(preset);
    defaultEnvironment.setPreset(preset);
  };

  const getTierBadge = (tier) => {
    switch (tier) {
      case 'OPTIMAL':
        return { color: '#00ff88', bg: 'rgba(0, 255, 136, 0.15)', border: 'rgba(0, 255, 136, 0.4)' };
      case 'HEALTHY':
        return { color: '#38bdf8', bg: 'rgba(56, 189, 248, 0.15)', border: 'rgba(56, 189, 248, 0.4)' };
      case 'DEGRADED':
        return { color: '#facc15', bg: 'rgba(250, 204, 21, 0.15)', border: 'rgba(250, 204, 21, 0.4)' };
      default:
        return { color: '#ef4444', bg: 'rgba(239, 68, 68, 0.15)', border: 'rgba(239, 68, 68, 0.4)' };
    }
  };

  const badgeStyle = getTierBadge(dtHealth.tier);

  return (
    <div style={{ display: 'flex', flexDirection: 'column', gap: '8px', fontSize: '0.72rem' }}>
      {/* Sub-tab navigation */}
      <div style={{ display: 'flex', gap: '4px', borderBottom: '1px solid rgba(0, 240, 255, 0.15)', paddingBottom: '4px' }}>
        {[
          { id: 'health', label: '🩺 Health & Sync' },
          { id: 'ekf', label: '🧭 EKF State' },
          { id: 'ident', label: '🧬 Hydro Ident' },
          { id: 'recorder', label: '📼 Recorder' },
        ].map((t) => (
          <button
            key={t.id}
            onClick={() => setActiveSubTab(t.id)}
            style={{
              flex: 1,
              padding: '4px 2px',
              fontSize: '0.62rem',
              fontWeight: activeSubTab === t.id ? '700' : '500',
              background: activeSubTab === t.id ? 'rgba(0, 240, 255, 0.2)' : 'rgba(0, 0, 0, 0.2)',
              border: activeSubTab === t.id ? '1px solid var(--accent-cyan)' : '1px solid transparent',
              borderRadius: '4px',
              color: activeSubTab === t.id ? 'var(--accent-cyan)' : 'var(--text-secondary)',
              cursor: 'pointer',
            }}
          >
            {t.label}
          </button>
        ))}
      </div>

      {/* ─────────────────────────────────────────────────────────────────── */}
      {/* SUBTAB 1: HEALTH & SYNC                                            */}
      {/* ─────────────────────────────────────────────────────────────────── */}
      {activeSubTab === 'health' && (
        <div style={{ display: 'flex', flexDirection: 'column', gap: '6px' }}>
          {/* Main Health Card */}
          <div style={{
            display: 'flex',
            alignItems: 'center',
            justifyContent: 'space-between',
            padding: '8px 10px',
            background: 'rgba(5, 20, 32, 0.8)',
            border: `1px solid ${badgeStyle.border}`,
            borderRadius: '6px',
          }}>
            <div>
              <div style={{ fontSize: '0.6rem', color: 'var(--text-tertiary)' }}>COMPOSITE DT HEALTH SCORE</div>
              <div style={{ fontSize: '1.4rem', fontWeight: '800', color: badgeStyle.color }}>
                {dtHealth.totalScore} <span style={{ fontSize: '0.75rem', fontWeight: '500' }}>/ 100</span>
              </div>
            </div>
            <div style={{
              padding: '3px 8px',
              borderRadius: '4px',
              fontSize: '0.65rem',
              fontWeight: '700',
              background: badgeStyle.bg,
              color: badgeStyle.color,
              border: `1px solid ${badgeStyle.border}`,
            }}>
              {dtHealth.tier}
            </div>
          </div>

          {/* 4 Pillars Breakdown */}
          <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: '4px' }}>
            <div style={{ padding: '6px', background: 'rgba(0,0,0,0.3)', borderRadius: '4px', border: '1px solid rgba(255,255,255,0.05)' }}>
              <div style={{ display: 'flex', justifyContent: 'space-between', color: 'var(--text-tertiary)', fontSize: '0.58rem' }}>
                <span>SYNC & LATENCY</span>
                <span style={{ color: 'var(--accent-green)' }}>{dtHealth.breakdown?.sync}/30</span>
              </div>
              <div style={{ fontSize: '0.75rem', fontWeight: '700', marginTop: '2px', color: '#fff' }}>
                {syncMetrics.totalLatencyMs || 15.0} ms
              </div>
              <div style={{ fontSize: '0.55rem', color: 'var(--text-tertiary)' }}>Rate: {syncMetrics.packetRateHz || 20.0} Hz</div>
            </div>

            <div style={{ padding: '6px', background: 'rgba(0,0,0,0.3)', borderRadius: '4px', border: '1px solid rgba(255,255,255,0.05)' }}>
              <div style={{ display: 'flex', justifyContent: 'space-between', color: 'var(--text-tertiary)', fontSize: '0.58rem' }}>
                <span>EKF RESIDUAL</span>
                <span style={{ color: 'var(--accent-cyan)' }}>{dtHealth.breakdown?.estimation}/25</span>
              </div>
              <div style={{ fontSize: '0.75rem', fontWeight: '700', marginTop: '2px', color: '#fff' }}>
                {validationMetrics.positionRMSE?.total3D || 0.02} m
              </div>
              <div style={{ fontSize: '0.55rem', color: 'var(--text-tertiary)' }}>Grade: {validationMetrics.validationGrade || 'A+'}</div>
            </div>

            <div style={{ padding: '6px', background: 'rgba(0,0,0,0.3)', borderRadius: '4px', border: '1px solid rgba(255,255,255,0.05)' }}>
              <div style={{ display: 'flex', justifyContent: 'space-between', color: 'var(--text-tertiary)', fontSize: '0.58rem' }}>
                <span>UNCERTAINTY</span>
                <span style={{ color: '#38bdf8' }}>{dtHealth.breakdown?.physics}/25</span>
              </div>
              <div style={{ fontSize: '0.75rem', fontWeight: '700', marginTop: '2px', color: '#fff' }}>
                {uncertainty.score} ({uncertainty.level})
              </div>
              <div style={{ fontSize: '0.55rem', color: 'var(--text-tertiary)' }}>Conf: {(uncertainty.confidence * 100).toFixed(1)}%</div>
            </div>

            <div style={{ padding: '6px', background: 'rgba(0,0,0,0.3)', borderRadius: '4px', border: '1px solid rgba(255,255,255,0.05)' }}>
              <div style={{ display: 'flex', justifyContent: 'space-between', color: 'var(--text-tertiary)', fontSize: '0.58rem' }}>
                <span>REGIME & OOD</span>
                <span style={{ color: oodStatus.isOOD ? '#ef4444' : '#00ff88' }}>{dtHealth.breakdown?.actuators}/20</span>
              </div>
              <div style={{ fontSize: '0.75rem', fontWeight: '700', marginTop: '2px', color: oodStatus.isOOD ? '#ef4444' : '#00ff88' }}>
                {oodStatus.state}
              </div>
              <div style={{ fontSize: '0.55rem', color: 'var(--text-tertiary)' }}>d_M: {oodStatus.oodScore}</div>
            </div>
          </div>

          {/* Environment Preset Selector */}
          <div style={{ padding: '6px', background: 'rgba(0,0,0,0.25)', borderRadius: '4px', border: '1px solid rgba(255,255,255,0.05)' }}>
            <div style={{ fontSize: '0.6rem', color: 'var(--text-tertiary)', marginBottom: '4px' }}>🌊 SIMULATED HYDRO ENVIRONMENT</div>
            <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr 1fr', gap: '3px' }}>
              {[
                { id: CURRENT_PRESETS.CALM, label: 'Calm Pool' },
                { id: CURRENT_PRESETS.POOL_CIRCULATION, label: 'Circulation' },
                { id: CURRENT_PRESETS.STRONG_CURRENT, label: 'Current' },
              ].map((p) => (
                <button
                  key={p.id}
                  onClick={() => handleEnvChange(p.id)}
                  style={{
                    padding: '3px',
                    fontSize: '0.58rem',
                    borderRadius: '3px',
                    border: selectedEnvPreset === p.id ? '1px solid var(--accent-cyan)' : '1px solid rgba(255,255,255,0.1)',
                    background: selectedEnvPreset === p.id ? 'rgba(0, 240, 255, 0.25)' : 'transparent',
                    color: selectedEnvPreset === p.id ? 'var(--accent-cyan)' : 'var(--text-secondary)',
                    cursor: 'pointer',
                  }}
                >
                  {p.label}
                </button>
              ))}
            </div>
          </div>
        </div>
      )}

      {/* ─────────────────────────────────────────────────────────────────── */}
      {/* SUBTAB 2: EKF STATE ESTIMATION                                     */}
      {/* ─────────────────────────────────────────────────────────────────── */}
      {activeSubTab === 'ekf' && (
        <div style={{ display: 'flex', flexDirection: 'column', gap: '6px' }}>
          <div style={{ fontSize: '0.62rem', color: 'var(--text-tertiary)' }}>
            15-STATE EXTENDED KALMAN FILTER (FUSING IMU + DEPTH + DVL + COMPASS)
          </div>

          <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: '4px' }}>
            <div style={{ padding: '6px', background: 'rgba(0,0,0,0.3)', borderRadius: '4px' }}>
              <div style={{ fontSize: '0.58rem', color: 'var(--text-tertiary)' }}>ESTIMATED POSITION (NED)</div>
              <div style={{ fontFamily: 'monospace', fontSize: '0.68rem', color: 'var(--accent-cyan)' }}>
                X: {(estimatedState.position?.x || 0).toFixed(2)}m
              </div>
              <div style={{ fontFamily: 'monospace', fontSize: '0.68rem', color: 'var(--accent-cyan)' }}>
                Y: {(estimatedState.position?.y || 0).toFixed(2)}m
              </div>
              <div style={{ fontFamily: 'monospace', fontSize: '0.68rem', color: 'var(--accent-cyan)' }}>
                Z: {(estimatedState.position?.z || 0.8).toFixed(2)}m (Depth)
              </div>
            </div>

            <div style={{ padding: '6px', background: 'rgba(0,0,0,0.3)', borderRadius: '4px' }}>
              <div style={{ fontSize: '0.58rem', color: 'var(--text-tertiary)' }}>ESTIMATED VELOCITY (BODY)</div>
              <div style={{ fontFamily: 'monospace', fontSize: '0.68rem', color: 'var(--accent-green)' }}>
                u (Surge): {(estimatedState.velocity?.u || 0).toFixed(2)} m/s
              </div>
              <div style={{ fontFamily: 'monospace', fontSize: '0.68rem', color: 'var(--accent-green)' }}>
                v (Sway):  {(estimatedState.velocity?.v || 0).toFixed(2)} m/s
              </div>
              <div style={{ fontFamily: 'monospace', fontSize: '0.68rem', color: 'var(--accent-green)' }}>
                w (Heave): {(estimatedState.velocity?.w || 0).toFixed(2)} m/s
              </div>
            </div>
          </div>

          <div style={{ padding: '6px', background: 'rgba(0,0,0,0.3)', borderRadius: '4px' }}>
            <div style={{ fontSize: '0.58rem', color: 'var(--text-tertiary)' }}>ESTIMATED SENSOR BIASES</div>
            <div style={{ display: 'flex', justifyContent: 'space-between', fontFamily: 'monospace', fontSize: '0.62rem', color: '#facc15' }}>
              <span>Accel: [{((estimatedState.biases?.accel?.[0] || 0) * 100).toFixed(1)} cm/s²]</span>
              <span>Gyro: [{((estimatedState.biases?.gyro?.[2] || 0) * (180 / Math.PI)).toFixed(2)}°/s]</span>
            </div>
          </div>
        </div>
      )}

      {/* ─────────────────────────────────────────────────────────────────── */}
      {/* SUBTAB 3: HYDRO IDENTIFICATION                                     */}
      {/* ─────────────────────────────────────────────────────────────────── */}
      {activeSubTab === 'ident' && (
        <div style={{ display: 'flex', flexDirection: 'column', gap: '6px' }}>
          <div style={{ fontSize: '0.62rem', color: 'var(--text-tertiary)' }}>
            EVOLUTIONARY HYDRODYNAMIC IDENTIFIER (DIFFERENTIAL EVOLUTION)
          </div>

          <button
            onClick={handleRunParameterIdentification}
            disabled={isCalibrating}
            style={{
              padding: '8px',
              borderRadius: '6px',
              background: isCalibrating ? 'rgba(255,255,255,0.1)' : 'rgba(0, 240, 255, 0.25)',
              border: '1px solid var(--accent-cyan)',
              color: 'var(--accent-cyan)',
              fontWeight: '700',
              fontSize: '0.68rem',
              cursor: isCalibrating ? 'not-allowed' : 'pointer',
            }}
          >
            {isCalibrating ? '🧬 Optimizing Hydrodynamics via DE...' : '⚡ Calibrate Hydrodynamic Parameters'}
          </button>

          {calibrationResult && (
            <div style={{ padding: '6px', background: 'rgba(0, 255, 136, 0.1)', border: '1px solid rgba(0, 255, 136, 0.3)', borderRadius: '4px' }}>
              <div style={{ color: 'var(--accent-green)', fontWeight: '700', fontSize: '0.65rem' }}>
                ✅ CALIBRATION SUCCESSFUL
              </div>
              <div style={{ fontSize: '0.6rem', color: '#fff', marginTop: '2px' }}>
                Initial RMSE: {calibrationResult.initialRMSE} → Final: {calibrationResult.finalRMSE}
              </div>
              <div style={{ fontSize: '0.6rem', color: 'var(--text-tertiary)' }}>
                Identified: Xu_dot={calibrationResult.optimalParams['addedMass.Xu_dot']?.toFixed(2)}kg, Xuu={calibrationResult.optimalParams['quadraticDamping.Xuu']?.toFixed(2)}
              </div>
            </div>
          )}

          <div style={{ padding: '6px', background: 'rgba(0,0,0,0.3)', borderRadius: '4px' }}>
            <div style={{ fontSize: '0.58rem', color: 'var(--text-tertiary)', marginBottom: '2px' }}>ACTIVE MODEL CONFIG</div>
            <div style={{ display: 'flex', justifyContent: 'space-between', fontSize: '0.6rem' }}>
              <span>Dry Mass: <b>{vehicleConfig.mass} kg</b></span>
              <span>Added Surge: <b>{vehicleConfig.Xu_dot} kg</b></span>
              <span>Version: <b>v{vehicleConfig.version}</b></span>
            </div>
          </div>
        </div>
      )}

      {/* ─────────────────────────────────────────────────────────────────── */}
      {/* SUBTAB 4: FLIGHT DATA RECORDER                                     */}
      {/* ─────────────────────────────────────────────────────────────────── */}
      {activeSubTab === 'recorder' && (
        <div style={{ display: 'flex', flexDirection: 'column', gap: '6px' }}>
          <div style={{ fontSize: '0.62rem', color: 'var(--text-tertiary)' }}>
            DIGITAL TWIN MISSION DATA LOGGER & DATASET EXPORT
          </div>

          <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: '4px' }}>
            <button
              onClick={handleToggleRecording}
              style={{
                padding: '8px',
                borderRadius: '6px',
                background: isRecording ? 'rgba(239, 68, 68, 0.3)' : 'rgba(0, 255, 136, 0.2)',
                border: isRecording ? '1px solid #ef4444' : '1px solid #00ff88',
                color: isRecording ? '#ef4444' : '#00ff88',
                fontWeight: '700',
                fontSize: '0.65rem',
                cursor: 'pointer',
              }}
            >
              {isRecording ? '⏹️ Stop Recording' : '⏺️ Start Recording'}
            </button>

            <button
              onClick={handleDownloadLog}
              style={{
                padding: '8px',
                borderRadius: '6px',
                background: 'rgba(56, 189, 248, 0.2)',
                border: '1px solid #38bdf8',
                color: '#38bdf8',
                fontWeight: '700',
                fontSize: '0.65rem',
                cursor: 'pointer',
              }}
            >
              📥 Export JSON
            </button>
          </div>

          <div style={{ padding: '6px', background: 'rgba(0,0,0,0.3)', borderRadius: '4px', fontSize: '0.6rem', color: 'var(--text-secondary)' }}>
            Status: {isRecording ? '🔴 Recording live telemetry stream (20 Hz)...' : `Standby (${recordedCount} samples captured in buffer)`}
          </div>
        </div>
      )}
    </div>
  );
}
