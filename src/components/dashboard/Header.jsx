import { useState, useEffect } from 'react';
import { motion } from 'framer-motion';
import useVehicleStore from '../../store/vehicleStore';
import rosConnection from '../../services/RosConnection';
import topicSubscriber from '../../services/TopicSubscriber';
import topicPublisher from '../../services/TopicPublisher';
import mockRos from '../../services/MockRosConnection';

export default function Header() {
  const connectionStatus = useVehicleStore((s) => s.connectionStatus);
  const mode = useVehicleStore((s) => s.mode);
  const jetsonIp = useVehicleStore((s) => s.jetsonIp);
  const armed = useVehicleStore((s) => s.armed);
  const flightMode = useVehicleStore((s) => s.flightMode);
  const setJetsonIp = useVehicleStore((s) => s.setJetsonIp);
  const setMode = useVehicleStore((s) => s.setMode);
  const setArmed = useVehicleStore((s) => s.setArmed);

  const [ipInput, setIpInput] = useState(jetsonIp);
  const [time, setTime] = useState(new Date());

  useEffect(() => {
    const timer = setInterval(() => setTime(new Date()), 1000);
    return () => clearInterval(timer);
  }, []);

  // Start demo mode by default
  useEffect(() => {
    mockRos.start();
    return () => mockRos.stop();
  }, []);

  const handleConnect = () => {
    if (mode === 'demo') {
      mockRos.stop();
    }
    setJetsonIp(ipInput);
    setMode('live');
    const ros = rosConnection.connect(ipInput);
    topicSubscriber.subscribeAll(ros);
    topicPublisher.init(ros);
  };

  const handleSwitchMode = (newMode) => {
    if (newMode === mode) return;

    if (newMode === 'demo') {
      rosConnection.disconnect();
      topicSubscriber.unsubscribeAll();
      topicPublisher.cleanup();
      mockRos.start();
      setMode('demo');
    } else {
      mockRos.stop();
      setMode('live');
      handleConnect();
    }
  };

  const statusLabel = {
    connected: 'JETSON ORIN LINKED',
    disconnected: 'DISCONNECTED',
    demo: 'SAUVC SIMULATION',
  };

  return (
    <motion.header
      className="header"
      initial={{ opacity: 0, y: -20 }}
      animate={{ opacity: 1, y: 0 }}
      transition={{ duration: 0.5 }}
    >
      <div className="header-left">
        <div className="header-logo">
          <div className="header-logo-icon">🌊</div>
          <span className="header-logo-text">BLUEROV2 TWIN</span>
          <span className="header-logo-sub">SAUVC 2026 Competition</span>
        </div>
      </div>

      <div className="header-center">
        {/* AUV Arm/Disarm Status Badge */}
        <div
          onClick={() => setArmed(!armed)}
          style={{
            cursor: 'pointer',
            display: 'flex',
            alignItems: 'center',
            gap: '6px',
            padding: '4px 12px',
            borderRadius: '9999px',
            fontSize: '0.7rem',
            fontWeight: '700',
            letterSpacing: '0.05em',
            background: armed ? 'rgba(0, 255, 136, 0.15)' : 'rgba(255, 59, 92, 0.15)',
            border: `1px solid ${armed ? 'rgba(0, 255, 136, 0.3)' : 'rgba(255, 59, 92, 0.3)'}`,
            color: armed ? 'var(--accent-green)' : 'var(--accent-red)',
          }}
          title="Click to toggle Motor Arm/Disarm"
        >
          <span>{armed ? '⚡ MOTORS ARMED' : '⛔ DISARMED'}</span>
        </div>

        {/* Flight Mode Tag */}
        <div
          style={{
            padding: '4px 10px',
            borderRadius: '6px',
            fontSize: '0.7rem',
            fontFamily: 'var(--font-mono)',
            fontWeight: '600',
            background: 'var(--bg-tertiary)',
            border: '1px solid var(--border-glass)',
            color: 'var(--accent-cyan)',
          }}
        >
          MODE: {flightMode}
        </div>

        {/* Mode Toggle */}
        <div className="mode-toggle">
          <button
            className={`mode-toggle-btn ${mode === 'demo' ? 'active' : ''}`}
            onClick={() => handleSwitchMode('demo')}
          >
            Demo
          </button>
          <button
            className={`mode-toggle-btn ${mode === 'live' ? 'active' : ''}`}
            onClick={() => handleSwitchMode('live')}
          >
            Live ROS2
          </button>
        </div>

        {/* IP Input & Connection Target Presets */}
        {mode === 'live' && (
          <div className="ip-input-group">
            <button
              style={{
                background: ipInput === 'localhost' ? 'var(--accent-cyan)' : 'var(--bg-tertiary)',
                color: ipInput === 'localhost' ? '#000' : 'var(--text-secondary)',
                border: '1px solid var(--border-glass)',
                borderRadius: '4px',
                padding: '2px 8px',
                fontSize: '0.65rem',
                fontWeight: '700',
                cursor: 'pointer',
              }}
              onClick={() => {
                setIpInput('localhost');
                setJetsonIp('localhost');
                const ros = rosConnection.connect('localhost');
                topicSubscriber.subscribeAll(ros);
                topicPublisher.init(ros);
              }}
            >
              🖥️ Local Sim
            </button>
            <input
              type="text"
              className="ip-input"
              value={ipInput}
              onChange={(e) => setIpInput(e.target.value)}
              placeholder="localhost / 192.168.1.100"
              style={{ width: '110px' }}
            />
            <button className="ip-connect-btn" onClick={handleConnect}>
              Link
            </button>
          </div>
        )}
      </div>

      <div className="header-right">
        <div className={`connection-status ${connectionStatus}`}>
          <span className="status-dot"></span>
          {statusLabel[connectionStatus] || 'UNKNOWN'}
        </div>
        <div className="header-time">
          {time.toLocaleTimeString('en-US', { hour12: false })}
        </div>
      </div>
    </motion.header>
  );
}
