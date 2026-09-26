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
  const activePage = useVehicleStore((s) => s.activePage);
  const setActivePage = useVehicleStore((s) => s.setActivePage);
  const setJetsonIp = useVehicleStore((s) => s.setJetsonIp);
  const setMode = useVehicleStore((s) => s.setMode);
  const setArmed = useVehicleStore((s) => s.setArmed);
  const monitorState = useVehicleStore((s) => s.monitorState);
  const setMonitorState = useVehicleStore((s) => s.setMonitorState);

  const [ipInput, setIpInput] = useState(jetsonIp);
  const [time, setTime] = useState(new Date());
  const [sshUser, setSshUser] = useState('amarine');
  const [sshState, setSshState] = useState('idle');
  const [odomState, setOdomState] = useState('idle');
  const [cameraState, setCameraState] = useState('idle');

  useEffect(() => {
    const timer = setInterval(() => setTime(new Date()), 1000);
    return () => clearInterval(timer);
  }, []);

  // Start demo mode by default
  useEffect(() => {
    mockRos.start();
    return () => mockRos.stop();
  }, []);

  const handleConnect = async () => {
    if (mode === 'demo') {
      mockRos.stop();
    }
    setJetsonIp(ipInput);
    setMode('live');
    rosConnection.isManualDisconnect = false;

    // If Electron desktop app, start rosbridge on Jetson via SSH first
    if (window.jetsonSsh && sshState !== 'running') {
      setSshState('starting');
      const result = await window.jetsonSsh.startRosbridge(ipInput, sshUser);
      setSshState(result.ok ? 'running' : 'error');
      if (result.ok) {
        // Wait for rosbridge to fully start before connecting WebSocket
        await new Promise((r) => setTimeout(r, 2000));
      } else {
        alert(`❌ Gagal menyalakan ROS di Jetson via SSH:\n${result.error || 'Pastikan SSH key sudah terpasang atau jalankan rosbridge manual di Jetson.'}`);
      }
    }

    // Connect WebSocket to rosbridge
    const ros = rosConnection.connect(ipInput);
    topicSubscriber.subscribeAll(ros);
    topicPublisher.init(ros);
  };

  const handleStartJetsonBridge = async () => {
    if (!window.jetsonSsh) {
      setSshState('desktop-only');
      return;
    }
    setSshState('starting');
    const result = await window.jetsonSsh.startRosbridge(ipInput, sshUser);
    setSshState(result.ok ? 'running' : 'error');
  };

  const handleStopJetsonBridge = async () => {
    if (!window.jetsonSsh) return;
    await window.jetsonSsh.stopRosbridge();
    setSshState('idle');
  };

  const handleStartOdomBridge = async () => {
    if (!window.jetsonSsh) {
      setOdomState('desktop-only');
      return;
    }
    setOdomState('starting');
    const result = await window.jetsonSsh.startOdomBridge(ipInput, sshUser);
    setOdomState(result.ok ? 'running' : 'error');
  };

  const handleStopOdomBridge = async () => {
    if (!window.jetsonSsh) return;
    await window.jetsonSsh.stopOdomBridge();
    setOdomState('idle');
  };

  const handleStartCamera = async () => {
    if (!window.jetsonSsh) {
      setCameraState('desktop-only');
      return;
    }
    setCameraState('starting');
    const result = await window.jetsonSsh.startCamera(ipInput, sshUser, '/dev/video0');
    setCameraState(result.ok ? 'running' : 'error');
  };

  const handleStopCamera = async () => {
    if (!window.jetsonSsh) return;
    await window.jetsonSsh.stopCamera();
    setCameraState('idle');
  };

  const handleStartMonitor = async () => {
    if (!window.jetsonSsh) {
      setMonitorState('desktop-only');
      const cmd = 'python3 ~/digitaltwin/backend/sauvc26_code/jetson_monitor_node.py';
      try {
        await navigator.clipboard.writeText(cmd);
        alert(`📋 Mode Browser: Perintah Jetson Monitor telah disalin ke clipboard:\n${cmd}\n\nJalankan di terminal Jetson (atau gunakan aplikasi desktop untuk eksekusi langsung 1-klik).`);
      } catch {
        alert(`Jalankan di terminal Jetson:\n${cmd}`);
      }
      setTimeout(() => setMonitorState('idle'), 3500);
      return;
    }
    setMonitorState('starting');
    const result = await window.jetsonSsh.startMonitor(ipInput, sshUser);
    setMonitorState(result.ok ? 'running' : 'error');
    if (!result.ok && result.error) {
      alert(`❌ Gagal menjalankan Jetson Monitor Node via SSH:\n${result.error}`);
    }
  };

  const handleStopMonitor = async () => {
    if (!window.jetsonSsh) {
      setMonitorState('idle');
      return;
    }
    await window.jetsonSsh.stopMonitor();
    setMonitorState('idle');
  };

  const handleDisconnectJetson = async () => {
    // Stop all SSH processes
    if (window.jetsonSsh) {
      await window.jetsonSsh.disconnectAll();
    }
    setSshState('idle');
    setOdomState('idle');
    setCameraState('idle');
    setMonitorState('idle');
    // Disconnect ROS WebSocket
    rosConnection.disconnect();
    topicSubscriber.unsubscribeAll();
    topicPublisher.cleanup();
    setMode('demo');
    mockRos.start();
  };

  const handleShutdownJetson = async () => {
    const confirmed = window.confirm(
      '⚠️ SHUTDOWN JETSON?\n\nJetson Orin akan dimatikan sepenuhnya.\nKamu perlu nyalakan ulang secara fisik.\n\nLanjutkan?'
    );
    if (!confirmed) return;

    // 1. Send shutdown command via active ROS2 WebSocket link
    topicPublisher.shutdownVehicle();

    // 2. Also attempt SSH shutdown if running inside Electron desktop app
    if (window.jetsonSsh) {
      window.jetsonSsh.shutdown(ipInput, sshUser).catch(() => {});
    }

    // 3. Cleanup local state & switch to demo
    setSshState('idle');
    setOdomState('idle');
    setCameraState('idle');
    setMonitorState('idle');
    setTimeout(() => {
      rosConnection.disconnect();
      topicSubscriber.unsubscribeAll();
      topicPublisher.cleanup();
      setMode('demo');
      mockRos.start();
    }, 1500);

    alert('✅ Sinyal shutdown terkirim ke Jetson!\nJetson akan mati dalam beberapa detik.');
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
      initial={{ opacity: 0 }}
      animate={{ opacity: 1, y: 0 }}
      transition={{ duration: 0.2 }}
    >
      <div className="header-left">
        <div className="header-logo">
          <div className="header-logo-icon">A</div>
          <span className="header-logo-text">AMARINE DT</span>
          <span className="header-logo-sub">AUV OPERATIONS</span>
        </div>

        {/* View Switcher Tabs */}
        <div
          style={{
            display: 'flex',
            background: 'rgba(15, 23, 42, 0.85)',
            border: '1px solid rgba(0, 240, 255, 0.25)',
            borderRadius: '8px',
            padding: '2px',
            gap: '3px',
            marginLeft: '12px',
          }}
        >
          <button
            onClick={() => setActivePage('mission')}
            style={{
              display: 'flex',
              alignItems: 'center',
              gap: '6px',
              padding: '4px 10px',
              borderRadius: '6px',
              fontSize: '0.72rem',
              fontWeight: 700,
              cursor: 'pointer',
              border: 'none',
              background:
                activePage === 'mission'
                  ? 'linear-gradient(135deg, rgba(0, 240, 255, 0.25) 0%, rgba(0, 128, 255, 0.25) 100%)'
                  : 'transparent',
              color: activePage === 'mission' ? '#00f0ff' : '#94a3b8',
              boxShadow: activePage === 'mission' ? '0 0 10px rgba(0, 240, 255, 0.2)' : 'none',
              transition: 'all 0.2s ease',
            }}
          >
            <span>Mission Control</span>
          </button>

          <button
            onClick={() => setActivePage('thruster_test')}
            style={{
              display: 'flex',
              alignItems: 'center',
              gap: '6px',
              padding: '4px 10px',
              borderRadius: '6px',
              fontSize: '0.72rem',
              fontWeight: 700,
              cursor: 'pointer',
              border: 'none',
              background:
                activePage === 'thruster_test'
                  ? 'linear-gradient(135deg, rgba(0, 255, 136, 0.25) 0%, rgba(0, 180, 216, 0.25) 100%)'
                  : 'transparent',
              color: activePage === 'thruster_test' ? '#00ff88' : '#94a3b8',
              boxShadow: activePage === 'thruster_test' ? '0 0 10px rgba(0, 255, 136, 0.2)' : 'none',
              transition: 'all 0.2s ease',
            }}
          >
            <span>Thruster HIL</span>
          </button>
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
          <span>{armed ? 'MOTORS ARMED' : 'DISARMED'}</span>
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
            SIM
          </button>
          <button
            className={`mode-toggle-btn ${mode === 'live' ? 'active' : ''}`}
            onClick={() => handleSwitchMode('live')}
          >
            ROS 2
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
              Local
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
            <input
              type="text"
              className="ip-input"
              value={sshUser}
              onChange={(e) => setSshUser(e.target.value)}
              placeholder="jetson user"
              style={{ width: '78px' }}
              title="Jetson SSH username"
            />
            <button
              className="ip-connect-btn"
              onClick={sshState === 'running' ? handleStopJetsonBridge : handleStartJetsonBridge}
              title="Start or stop rosbridge on Jetson through SSH key authentication"
            >
              {sshState === 'running' ? 'Stop ROS' : sshState === 'starting' ? 'Starting' : 'Start ROS'}
            </button>
            <button
              className="ip-connect-btn"
              onClick={odomState === 'running' ? handleStopOdomBridge : handleStartOdomBridge}
              title="Start or stop the MAVROS telemetry bridge (odom/battery/depth/front sonar) on Jetson via SSH"
            >
              {odomState === 'running' ? 'Stop Bridge' : odomState === 'starting' ? 'Starting' : 'Start Bridge'}
            </button>
            <button
              className="ip-connect-btn"
              onClick={cameraState === 'running' ? handleStopCamera : handleStartCamera}
              title="Start or stop the camera driver (/dev/video0) on Jetson via SSH"
            >
              {cameraState === 'running' ? 'Stop Cam' : cameraState === 'starting' ? 'Starting' : 'Start Cam'}
            </button>
            <button
              className="ip-connect-btn"
              onClick={monitorState === 'running' ? handleStopMonitor : handleStartMonitor}
              title="Start or stop Jetson Monitor Node (jetson_monitor_node.py) via SSH untuk telemetri CPU/GPU/RAM asli"
              style={{
                background: monitorState === 'running' ? 'rgba(0, 255, 200, 0.2)' : undefined,
                borderColor: monitorState === 'running' ? '#00ffc8' : undefined,
                color: monitorState === 'running' ? '#00ffc8' : undefined,
              }}
            >
              {monitorState === 'running' ? 'Stop Mon' : monitorState === 'starting' ? 'Starting' : 'Start Mon'}
            </button>

            {/* Separator */}
            <div style={{
              width: '1px',
              height: '18px',
              background: 'rgba(255, 255, 255, 0.15)',
              margin: '0 2px',
            }} />

            {/* Disconnect Jetson */}
            <button
              className="ip-connect-btn"
              onClick={handleDisconnectJetson}
              title="Disconnect semua: matikan ROS bridge, odom bridge, camera, dan putus koneksi WebSocket"
              style={{
                background: 'rgba(255, 170, 0, 0.15)',
                border: '1px solid rgba(255, 170, 0, 0.4)',
                color: '#ffaa00',
              }}
            >
              ⛓️‍💥 Disconnect
            </button>

            {/* Shutdown Jetson */}
            <button
              className="ip-connect-btn"
              onClick={handleShutdownJetson}
              title="Shutdown Jetson Orin sepenuhnya (perlu nyalakan ulang secara fisik)"
              style={{
                background: 'rgba(255, 59, 92, 0.15)',
                border: '1px solid rgba(255, 59, 92, 0.4)',
                color: '#ff3b5c',
              }}
            >
              ⏻ Shutdown
            </button>
          </div>
        )}
      </div>

      <div className="header-right">
        <div className={`connection-status ${mode === 'demo' ? 'demo' : connectionStatus}`}>
          <span className="status-dot"></span>
          {mode === 'demo' ? 'SAUVC SIMULATION' : (statusLabel[connectionStatus] || 'UNKNOWN')}
        </div>
        <div className="header-time">
          {time.toLocaleTimeString('en-US', { hour12: false })}
        </div>
      </div>
    </motion.header>
  );
}
