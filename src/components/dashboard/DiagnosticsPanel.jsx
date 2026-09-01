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

  const getStatusClass = (value, warnThreshold = 70, errorThreshold = 90) => {
    if (value >= errorThreshold) return 'error';
    if (value >= warnThreshold) return 'warn';
    return 'ok';
  };

  const items = [
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
          Subsea & Compute Health
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
