import { motion } from 'framer-motion';
import useVehicleStore from '../../store/vehicleStore';

function formatUptime(seconds) {
  const h = Math.floor(seconds / 3600);
  const m = Math.floor((seconds % 3600) / 60);
  const s = Math.floor(seconds % 60);
  return `${h.toString().padStart(2, '0')}:${m.toString().padStart(2, '0')}:${s.toString().padStart(2, '0')}`;
}

function UsageBar({ value, color }) {
  return (
    <div
      style={{
        width: '60px',
        height: '6px',
        background: 'var(--bg-primary)',
        borderRadius: '3px',
        overflow: 'hidden',
      }}
    >
      <div
        style={{
          width: `${Math.min(value, 100)}%`,
          height: '100%',
          background: color,
          borderRadius: '3px',
          transition: 'width 0.3s ease',
        }}
      />
    </div>
  );
}

export default function DiagnosticsPanel() {
  const diagnostics = useVehicleStore((s) => s.diagnostics);
  const connectionStatus = useVehicleStore((s) => s.connectionStatus);
  const leakDetected = useVehicleStore((s) => s.leakDetected);
  const safety = useVehicleStore((s) => s.safetySupervisor);
  const monitorState = useVehicleStore((s) => s.monitorState);
  const setMonitorState = useVehicleStore((s) => s.setMonitorState);
  const jetsonIp = useVehicleStore((s) => s.jetsonIp);

  const handleToggleMonitor = async () => {
    if (monitorState === 'running') {
      if (window.jetsonSsh) {
        await window.jetsonSsh.stopMonitor();
      }
      setMonitorState('idle');
      return;
    }

    if (!window.jetsonSsh) {
      const cmd = 'python3 ~/digitaltwin/backend/sauvc26_code/jetson_monitor_node.py';
      try {
        await navigator.clipboard.writeText(cmd);
        alert(`📋 Perintah Jetson Monitor telah disalin ke clipboard:\n${cmd}\n\nJalankan di Jetson Orin Anda!`);
      } catch {
        alert(`Jalankan di Jetson:\n${cmd}`);
      }
      return;
    }

    setMonitorState('starting');
    const res = await window.jetsonSsh.startMonitor(jetsonIp || 'localhost', 'amarine');
    setMonitorState(res.ok ? 'running' : 'error');
    if (!res.ok && res.error) {
      alert(`❌ Gagal menyalakan jetson_monitor_node via SSH:\n${res.error}`);
    }
  };

  const getStatusClass = (value, warnThreshold = 70, errorThreshold = 90) => {
    if (value >= errorThreshold) return 'error';
    if (value >= warnThreshold) return 'warn';
    return 'ok';
  };

  const items = [
    {
      label: 'Safety Supervisor',
      value: safety.state,
      status: safety.emergency ? 'error' : safety.state === 'NORMAL' ? 'ok' : 'warn',
    },
    {
      label: '🖥️ Jetson CPU',
      value: `${diagnostics.cpuUsage.toFixed(0)}%`,
      status: getStatusClass(diagnostics.cpuUsage),
      bar: diagnostics.cpuUsage,
      color: diagnostics.cpuUsage > 70 ? 'var(--accent-orange)' : 'var(--accent-cyan)',
    },
    {
      label: '⚡ Jetson GPU (CUDA)',
      value: `${diagnostics.gpuUsage.toFixed(0)}%`,
      status: getStatusClass(diagnostics.gpuUsage),
      bar: diagnostics.gpuUsage,
      color: diagnostics.gpuUsage > 70 ? 'var(--accent-orange)' : 'var(--accent-green)',
    },
    {
      label: '💾 LPDDR5 RAM',
      value: `${diagnostics.memoryUsage.toFixed(0)}%`,
      status: getStatusClass(diagnostics.memoryUsage),
      bar: diagnostics.memoryUsage,
      color: diagnostics.memoryUsage > 80 ? 'var(--accent-red)' : 'var(--accent-blue)',
    },
    {
      label: '📡 DVL Acoustic Lock',
      value: diagnostics.dvlStatus || 'LOCKED',
      status: 'ok',
    },
    {
      label: '💧 Hull Leak Sensor',
      value: leakDetected ? 'ALARM' : 'SECURE',
      status: leakDetected ? 'error' : 'ok',
    },
    {
      label: '🔗 ROS2 Active Nodes',
      value: diagnostics.rosNodes.toString(),
      status: 'ok',
    },
    {
      label: '📊 Bridge Topic Rate',
      value: `${diagnostics.topicRate.toFixed(0)} Hz`,
      status: diagnostics.topicRate > 20 ? 'ok' : 'warn',
    },
    {
      label: '⏱️ Mission Uptime',
      value: formatUptime(diagnostics.uptime),
      status: 'ok',
    },
  ];

  return (
    <motion.div
      className="glass-panel"
      initial={{ opacity: 0, x: 30 }}
      animate={{ opacity: 1, x: 0 }}
      transition={{ duration: 0.5, delay: 0.4 }}
    >
      <div className="glass-panel-header">
        <div className="glass-panel-title">
          <span className="icon">⚙️</span>
          System Health
        </div>
        <div
          style={{
            fontSize: '0.6rem',
            fontFamily: 'var(--font-mono)',
            color:
              connectionStatus === 'connected'
                ? 'var(--accent-green)'
                : connectionStatus === 'demo'
                ? 'var(--accent-orange)'
                : 'var(--accent-red)',
          }}
        >
          {connectionStatus === 'demo' ? 'SIMULATION' : 'JETSON ORIN'}
        </div>
      </div>

      {/* Quick Jetson Monitor Node Button */}
      <div
        style={{
          display: 'flex',
          alignItems: 'center',
          justifyContent: 'space-between',
          background: 'rgba(0, 240, 255, 0.05)',
          border: '1px solid rgba(0, 240, 255, 0.18)',
          borderRadius: '6px',
          padding: '5px 8px',
          margin: '0 0 10px 0',
          fontSize: '0.65rem',
        }}
      >
        <div style={{ display: 'flex', alignItems: 'center', gap: '6px' }}>
          <span
            style={{
              width: '7px',
              height: '7px',
              borderRadius: '50%',
              background:
                monitorState === 'running'
                  ? '#00ff88'
                  : monitorState === 'starting'
                  ? '#ffaa00'
                  : 'rgba(255,255,255,0.3)',
              boxShadow:
                monitorState === 'running'
                  ? '0 0 6px #00ff88'
                  : 'none',
            }}
          />
          <span style={{ color: 'var(--text-secondary)', fontWeight: 600 }}>
            Node: jetson_monitor
          </span>
        </div>
        <button
          onClick={handleToggleMonitor}
          title="Klik untuk menyalakan/mematikan node pemantau CPU/GPU/RAM di Jetson"
          style={{
            background:
              monitorState === 'running'
                ? 'rgba(255, 59, 92, 0.2)'
                : 'linear-gradient(135deg, rgba(0, 240, 255, 0.25) 0%, rgba(0, 128, 255, 0.25) 100%)',
            border:
              monitorState === 'running'
                ? '1px solid rgba(255, 59, 92, 0.5)'
                : '1px solid rgba(0, 240, 255, 0.4)',
            color: monitorState === 'running' ? '#ff3b5c' : '#00f0ff',
            borderRadius: '4px',
            padding: '3px 8px',
            fontSize: '0.63rem',
            fontWeight: 700,
            cursor: 'pointer',
            transition: 'all 0.2s',
          }}
        >
          {monitorState === 'running'
            ? '⏹ Stop Node'
            : monitorState === 'starting'
            ? '⏳ Starting...'
            : '▶️ Run Monitor'}
        </button>
      </div>

      <div className="diag-list">
        {items.map((item, i) => (
          <div className="diag-item" key={i}>
            <div className="diag-item-label">
              <span className={`diag-item-status ${item.status}`} />
              {item.label}
            </div>
            <div style={{ display: 'flex', alignItems: 'center', gap: '8px' }}>
              {item.bar !== undefined && <UsageBar value={item.bar} color={item.color} />}
              <div className="diag-item-value">{item.value}</div>
            </div>
          </div>
        ))}
      </div>
    </motion.div>
  );
}
