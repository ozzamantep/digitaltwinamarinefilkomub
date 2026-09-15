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

function ProximityRadar() {
  const sonarRanges = useVehicleStore((s) => s.sonarRanges);
  const sonarDetections = useVehicleStore((s) => s.sonarDetections);
  const detection = useVehicleStore((s) => s.imuDetection);
  const maxRange = 8;
  const center = 100;
  const radarRadius = 78;
  const entries = [
    { direction: 'front', label: 'F', x: 0, y: -1, bearing: 0 },
    { direction: 'right', label: 'R', x: 1, y: 0, bearing: Math.PI / 2 },
    { direction: 'rear', label: 'B', x: 0, y: 1, bearing: Math.PI },
    { direction: 'left', label: 'L', x: -1, y: 0, bearing: -Math.PI / 2 },
  ];
  const sweepDelay = (bearing) => {
    const normalized = ((bearing % (Math.PI * 2)) + Math.PI * 2) % (Math.PI * 2);
    return `${(normalized / (Math.PI * 2)) * 3.2}s`;
  };
  const nearest = Math.min(...Object.values(sonarRanges));
  const imuColor = detection.impactDetected ? '#ff3b5c' : detection.motionDetected ? '#ffb347' : '#00ff88';
  const rangeColor = (range) => range <= 0.55 ? '#ff3b5c' : range < 1.2 ? '#ffb347' : '#7cff6b';

  return (
    <div className="proximity-radar-panel">
      <div className="proximity-radar-heading">
        <span>SONAR PROXIMITY</span>
        <span style={{ color: rangeColor(nearest) }}>{nearest.toFixed(2)} m</span>
      </div>
      <svg className="proximity-radar" viewBox="0 0 200 200" role="img" aria-label="Four direction sonar proximity radar">
        <defs>
          <radialGradient id="radarGlow">
            <stop offset="0%" stopColor="#063d2a" />
            <stop offset="100%" stopColor="#010b08" />
          </radialGradient>
          <linearGradient id="radarSweep" x1="0" y1="1" x2="1" y2="0">
            <stop offset="0%" stopColor="#00ff88" stopOpacity="0" />
            <stop offset="100%" stopColor="#00ff88" stopOpacity="0.55" />
          </linearGradient>
        </defs>
        <circle cx={center} cy={center} r="88" fill="url(#radarGlow)" stroke="#00ff88" strokeWidth="1.5" />
        {[22, 44, 66].map((radius) => (
          <circle key={radius} cx={center} cy={center} r={radius} fill="none" stroke="#16a34a" strokeOpacity="0.45" />
        ))}
        <path d="M100 12V188M12 100H188" stroke="#16a34a" strokeOpacity="0.38" />
        <g className="radar-sweep">
          <path d="M100 100 L100 12 A88 88 0 0 1 162 38 Z" fill="url(#radarSweep)" />
          <line x1="100" y1="100" x2="100" y2="12" stroke="#7cff6b" strokeWidth="1.5" />
        </g>
        {entries.map(({ direction, label, x, y, bearing }) => {
          const range = Math.min(maxRange, sonarRanges[direction]);
          const radius = (range / maxRange) * radarRadius;
          return (
            <g key={direction} className="radar-swept-contact" style={{ animationDelay: sweepDelay(bearing) }}>
              <circle
                cx={center + x * radius}
                cy={center + y * radius}
                r={range < 1.2 ? 5 : 3.5}
                fill={rangeColor(range)}
                className={range < 1.2 ? 'radar-alert-blip' : ''}
              />
              <text x={center + x * 91} y={center + y * 91 + 3} textAnchor="middle" fill="#8ddcab" fontSize="9">{label}</text>
              <text x={center + x * (radius + 10)} y={center + y * (radius + 10) + 3} textAnchor="middle" fill={rangeColor(range)} fontSize="7">
                {sonarRanges[direction].toFixed(1)}
              </text>
            </g>
          );
        })}
        {sonarDetections.map((detection) => {
          const radius = (Math.min(maxRange, detection.range) / maxRange) * radarRadius;
          const x = center + Math.sin(detection.bearing) * radius;
          const y = center - Math.cos(detection.bearing) * radius;
          return (
            <g key={detection.id} className="radar-swept-contact" style={{ animationDelay: sweepDelay(detection.bearing) }}>
              <circle cx={x} cy={y} r="4" fill={rangeColor(detection.range)} className="radar-object-blip" />
              <circle cx={x} cy={y} r="7" fill="none" stroke={rangeColor(detection.range)} strokeOpacity="0.45" />
              <title>{`${detection.label}: ${detection.range.toFixed(2)} m`}</title>
            </g>
          );
        })}
        <circle cx={center} cy={center} r="9" fill="#071b15" stroke={imuColor} strokeWidth="2" />
        <path d="M100 92 L106 106 L100 103 L94 106 Z" fill={imuColor} />
      </svg>
      <div className="proximity-radar-footer">
        <span>IMU {detection.status}</span>
        <span>{sonarDetections.length} TRACK · MAX {maxRange} m</span>
      </div>
    </div>
  );
}

function IMUDetection() {
  const imu = useVehicleStore((s) => s.imu);
  const detection = useVehicleStore((s) => s.imuDetection);
  const altitudeDVL = useVehicleStore((s) => s.altitudeDVL);
  const floorState = altitudeDVL <= 0.37 ? 'TERKUNCI' : altitudeDVL < 0.70 ? 'MELAMBAT' : 'AMAN';
  const statusColor = detection.impactDetected
    ? 'var(--accent-red)'
    : detection.motionDetected
      ? 'var(--accent-orange)'
      : 'var(--accent-green)';
  const floorColor = floorState === 'TERKUNCI'
    ? 'var(--accent-red)'
    : floorState === 'MELAMBAT'
      ? 'var(--accent-orange)'
      : 'var(--accent-green)';

  return (
    <div style={{ marginTop: '12px' }}>
      <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: '6px' }}>
        <div className="sensor-label">IMU Motion Detection</div>
        <div style={{ color: statusColor, fontSize: '0.65rem', fontWeight: '700' }}>
          {detection.status}
        </div>
      </div>
      <div className="sensor-grid">
        <div className="sensor-item">
          <div className="sensor-label">Accel X</div>
          <div className="sensor-value accent-cyan" style={{ fontSize: '0.9rem' }}>{imu.accelX.toFixed(2)} m/s²</div>
        </div>
        <div className="sensor-item">
          <div className="sensor-label">Accel Y</div>
          <div className="sensor-value accent-green" style={{ fontSize: '0.9rem' }}>{imu.accelY.toFixed(2)} m/s²</div>
        </div>
        <div className="sensor-item">
          <div className="sensor-label">Accel Z</div>
          <div className="sensor-value accent-blue" style={{ fontSize: '0.9rem' }}>{imu.accelZ.toFixed(2)} m/s²</div>
        </div>
        <div className="sensor-item">
          <div className="sensor-label">Resultan</div>
          <div className="sensor-value" style={{ color: statusColor, fontSize: '0.9rem' }}>
            {detection.accelerationMagnitude.toFixed(2)} m/s²
          </div>
        </div>
      </div>
      <div style={{ display: 'flex', justifyContent: 'space-between', marginTop: '7px', fontSize: '0.65rem' }}>
        <span style={{ color: 'var(--text-tertiary)' }}>DVL Floor Guard · {altitudeDVL.toFixed(2)} m</span>
        <span style={{ color: floorColor, fontWeight: '700' }}>{floorState}</span>
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

      <ProximityRadar />

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

      <IMUDetection />
    </motion.div>
  );
}
