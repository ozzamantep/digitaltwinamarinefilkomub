import { useState, useMemo } from 'react';
import { motion } from 'framer-motion';
import {
  LineChart, Line, XAxis, YAxis, CartesianGrid, Tooltip,
  ResponsiveContainer, Area, AreaChart,
} from 'recharts';
import useVehicleStore from '../../store/vehicleStore';

const CustomTooltip = ({ active, payload }) => {
  if (active && payload && payload.length) {
    return (
      <div
        style={{
          background: 'rgba(3, 25, 38, 0.95)',
          border: '1px solid rgba(0, 240, 255, 0.3)',
          borderRadius: '8px',
          padding: '8px 12px',
          fontSize: '0.7rem',
          fontFamily: 'var(--font-mono)',
        }}
      >
        {payload.map((p, i) => (
          <div key={i} style={{ color: p.color, marginBottom: '2px' }}>
            {p.name}: {typeof p.value === 'number' ? p.value.toFixed(2) : p.value}
          </div>
        ))}
      </div>
    );
  }
  return null;
};

export default function ChartPanel() {
  const [activeTab, setActiveTab] = useState('depth');
  const depthHistory = useVehicleStore((s) => s.depthHistory);
  const speedHistory = useVehicleStore((s) => s.speedHistory);
  const batteryHistory = useVehicleStore((s) => s.batteryHistory);

  const depthSpeedData = useMemo(() => {
    return depthHistory.slice(-50).map((d, i) => {
      const s = speedHistory[i] || { surge: 0 };
      return {
        t: i,
        depth: d.depth,
        surge: (s.surge || s.linear || 0),
      };
    });
  }, [depthHistory, speedHistory]);

  const batteryData = useMemo(() => {
    return batteryHistory.slice(-50).map((b, i) => ({
      t: i,
      level: b.level,
      voltage: b.voltage,
      current: b.current,
    }));
  }, [batteryHistory]);

  const tabs = [
    { id: 'depth', label: 'Depth & Surge' },
    { id: 'battery', label: '4S LiPo Power' },
  ];

  return (
    <motion.div
      className="glass-panel chart-panel bottom-charts"
      initial={{ opacity: 0, y: 20 }}
      animate={{ opacity: 1, y: 0 }}
      transition={{ duration: 0.5, delay: 0.4 }}
    >
      <div style={{ display: 'flex', alignItems: 'center', justifyContent: 'space-between', marginBottom: '8px' }}>
        <div className="glass-panel-title">
          <span className="icon">📈</span>
          Subsea Telemetry Streams
        </div>
        <div className="chart-tabs">
          {tabs.map((tab) => (
            <button
              key={tab.id}
              className={`chart-tab ${activeTab === tab.id ? 'active' : ''}`}
              onClick={() => setActiveTab(tab.id)}
            >
              {tab.label}
            </button>
          ))}
        </div>
      </div>

      <div className="chart-content">
        <ResponsiveContainer width="100%" height="100%">
          {activeTab === 'depth' ? (
            <AreaChart data={depthSpeedData}>
              <defs>
                <linearGradient id="depthFill" x1="0" y1="0" x2="0" y2="1">
                  <stop offset="0%" stopColor="#00f0ff" stopOpacity={0.35} />
                  <stop offset="100%" stopColor="#0284c7" stopOpacity={0} />
                </linearGradient>
                <linearGradient id="surgeFill" x1="0" y1="0" x2="0" y2="1">
                  <stop offset="0%" stopColor="#00ff88" stopOpacity={0.3} />
                  <stop offset="100%" stopColor="#00ff88" stopOpacity={0} />
                </linearGradient>
              </defs>
              <CartesianGrid strokeDasharray="3 3" stroke="rgba(148,163,184,0.08)" />
              <XAxis dataKey="t" hide />
              <YAxis
                stroke="rgba(148,163,184,0.2)"
                tick={{ fill: '#64748b', fontSize: 10 }}
                width={35}
              />
              <Tooltip content={<CustomTooltip />} />
              <Area
                type="monotone"
                dataKey="depth"
                stroke="#00f0ff"
                fill="url(#depthFill)"
                strokeWidth={2}
                name="Depth (m)"
                dot={false}
                animationDuration={0}
              />
              <Area
                type="monotone"
                dataKey="surge"
                stroke="#00ff88"
                fill="url(#surgeFill)"
                strokeWidth={1.5}
                name="Surge (m/s)"
                dot={false}
                animationDuration={0}
              />
            </AreaChart>
          ) : (
            <AreaChart data={batteryData}>
              <defs>
                <linearGradient id="batteryFill" x1="0" y1="0" x2="0" y2="1">
                  <stop offset="0%" stopColor="#00ff88" stopOpacity={0.3} />
                  <stop offset="100%" stopColor="#00ff88" stopOpacity={0} />
                </linearGradient>
              </defs>
              <CartesianGrid strokeDasharray="3 3" stroke="rgba(148,163,184,0.08)" />
              <XAxis dataKey="t" hide />
              <YAxis
                stroke="rgba(148,163,184,0.2)"
                tick={{ fill: '#64748b', fontSize: 10 }}
                width={35}
              />
              <Tooltip content={<CustomTooltip />} />
              <Area
                type="monotone"
                dataKey="level"
                stroke="#00ff88"
                fill="url(#batteryFill)"
                strokeWidth={2}
                name="Battery (%)"
                dot={false}
                animationDuration={0}
              />
              <Line
                type="monotone"
                dataKey="voltage"
                stroke="#ff8c00"
                strokeWidth={1.5}
                name="4S Voltage (V)"
                dot={false}
                animationDuration={0}
              />
              <Line
                type="monotone"
                dataKey="current"
                stroke="#a855f7"
                strokeWidth={1.5}
                name="Current Draw (A)"
                dot={false}
                animationDuration={0}
              />
            </AreaChart>
          )}
        </ResponsiveContainer>
      </div>
    </motion.div>
  );
}
