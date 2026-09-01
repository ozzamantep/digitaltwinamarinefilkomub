import { motion } from 'framer-motion';
import useVehicleStore from '../../store/vehicleStore';

function DepthMeter() {
  const depth = useVehicleStore((s) => s.depth);
  const maxDepth = 2.5; // SAUVC pool depth is ~2.0m - 2.5m
  const percentage = Math.min(depth / maxDepth, 1);

  const radius = 58;
  const circumference = 2 * Math.PI * radius;
  const arcLength = circumference * 0.75; // 270 degrees
  const dashoffset = arcLength - arcLength * percentage;

  return (
    <div className="speedometer">
      <div className="speed-ring">
        <svg viewBox="0 0 140 140">
          <defs>
            <linearGradient id="depthGradient" x1="0%" y1="0%" x2="100%" y2="100%">
              <stop offset="0%" stopColor="#00f0ff" />
              <stop offset="100%" stopColor="#0284c7" />
            </linearGradient>
          </defs>
          <circle
            className="speed-ring-bg"
            cx="70"
            cy="70"
            r={radius}
            strokeDasharray={`${arcLength} ${circumference}`}
            strokeDashoffset={0}
            transform="rotate(135 70 70)"
          />
          <circle
            className="speed-ring-fill"
            cx="70"
            cy="70"
            r={radius}
            stroke="url(#depthGradient)"
            strokeDasharray={`${arcLength} ${circumference}`}
            strokeDashoffset={dashoffset}
            transform="rotate(135 70 70)"
            style={{ transition: 'stroke-dashoffset 0.2s ease' }}
          />
        </svg>
        <div className="speed-value">
          <div className="speed-number" style={{ color: '#00f0ff' }}>
            {depth.toFixed(2)}
          </div>
          <div className="speed-label">DEPTH (m)</div>
        </div>
      </div>
    </div>
  );
}

function Inclinometer() {
  const euler = useVehicleStore((s) => s.euler);

  return (
    <div>
      <div className="sensor-label" style={{ marginBottom: '6px' }}>
        📐 6-DOF Attitude & Heading
      </div>
      <div className="imu-axes">
        <div className="imu-axis">
          <div className="imu-axis-label">Roll</div>
          <div className="imu-axis-value accent-cyan">{euler.roll.toFixed(1)}°</div>
        </div>
        <div className="imu-axis">
          <div className="imu-axis-label">Pitch</div>
          <div className="imu-axis-value accent-green">{euler.pitch.toFixed(1)}°</div>
        </div>
        <div className="imu-axis">
          <div className="imu-axis-label">Yaw / Compass</div>
          <div className="imu-axis-value accent-orange">{euler.yaw.toFixed(0)}°</div>
        </div>
      </div>
    </div>
  );
}

function SubseaBattery() {
  const battery = useVehicleStore((s) => s.battery);
  const level = battery.level;

  let color;
  if (level > 50) color = 'linear-gradient(90deg, #00ff88, #00f0ff)';
  else if (level > 25) color = 'linear-gradient(90deg, #ff8c00, #ffb347)';
  else color = 'linear-gradient(90deg, #ff3b5c, #ff6b8a)';

  return (
    <div>
      <div className="sensor-label" style={{ marginBottom: '6px' }}>
        🔋 4S LiPo Power Supply
      </div>
      <div className="battery-bar">
        <div className="battery-fill" style={{ width: `${level}%`, background: color }} />
        <span className="battery-text">{level.toFixed(1)}%</span>
      </div>
      <div className="sensor-grid" style={{ marginTop: '8px' }}>
        <div className="sensor-item">
          <div className="sensor-label">Voltage</div>
          <div className="sensor-value accent-cyan" style={{ fontSize: '1rem' }}>
            {battery.voltage.toFixed(2)}
            <span className="sensor-unit">V</span>
          </div>
        </div>
        <div className="sensor-item">
          <div className="sensor-label">Current Draw</div>
          <div className="sensor-value accent-green" style={{ fontSize: '1rem' }}>
            {battery.current.toFixed(1)}
            <span className="sensor-unit">A</span>
          </div>
        </div>
      </div>
    </div>
  );
}

export default function SensorPanel() {
  const speed = useVehicleStore((s) => s.speed);
  const depthSensor = useVehicleStore((s) => s.depthSensor);
  const altitudeDVL = useVehicleStore((s) => s.altitudeDVL);
  const leakDetected = useVehicleStore((s) => s.leakDetected);

  return (
    <motion.div
      className="glass-panel animate-slide-left"
      initial={{ opacity: 0, x: -30 }}
      animate={{ opacity: 1, x: 0 }}
      transition={{ duration: 0.5, delay: 0.1 }}
    >
      <div className="glass-panel-header">
        <div className="glass-panel-title">
          <span className="icon">🌊</span>
          AUV Subsea Telemetry
        </div>
        <div
          style={{
            fontSize: '0.65rem',
            fontWeight: '700',
            padding: '2px 8px',
            borderRadius: '4px',
            background: leakDetected ? 'rgba(255,59,92,0.2)' : 'rgba(0,255,136,0.15)',
            color: leakDetected ? 'var(--accent-red)' : 'var(--accent-green)',
            border: `1px solid ${leakDetected ? 'rgba(255,59,92,0.4)' : 'rgba(0,255,136,0.3)'}`,
          }}
        >
          {leakDetected ? '⚠️ LEAK ALERT' : '🛡️ HULL SEALED'}
        </div>
      </div>

      <DepthMeter />

      <div className="sensor-grid" style={{ marginBottom: '12px' }}>
        <div className="sensor-item">
          <div className="sensor-label">Surge Velocity</div>
          <div className="sensor-value accent-cyan">
            {speed.surge.toFixed(2)}
            <span className="sensor-unit">m/s</span>
          </div>
        </div>
        <div className="sensor-item">
          <div className="sensor-label">DVL Altitude</div>
          <div className="sensor-value accent-purple">
            {altitudeDVL.toFixed(2)}
            <span className="sensor-unit">m</span>
          </div>
        </div>
        <div className="sensor-item">
          <div className="sensor-label">Water Pressure</div>
          <div className="sensor-value accent-blue">
            {depthSensor.pressure.toFixed(2)}
            <span className="sensor-unit">kPa</span>
          </div>
        </div>
        <div className="sensor-item">
          <div className="sensor-label">Water Temp</div>
          <div className="sensor-value accent-green">
            {depthSensor.temperature.toFixed(1)}
            <span className="sensor-unit">°C</span>
          </div>
        </div>
      </div>

      <SubseaBattery />

      <div style={{ marginTop: '12px' }}>
        <Inclinometer />
      </div>
    </motion.div>
  );
}
