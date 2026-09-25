import { useState, useEffect, useRef } from 'react';
import useVehicleStore from '../../store/vehicleStore';
import rosConnection from '../../services/RosConnection';
import topicSubscriber from '../../services/TopicSubscriber';
import topicPublisher from '../../services/TopicPublisher';
import mockRos from '../../services/MockRosConnection';

export default function JetsonTerminalBox() {
  const jetsonIp = useVehicleStore((s) => s.jetsonIp);
  const setJetsonIp = useVehicleStore((s) => s.setJetsonIp);
  const setMode = useVehicleStore((s) => s.setMode);
  const mode = useVehicleStore((s) => s.mode);
  const connectionStatus = useVehicleStore((s) => s.connectionStatus);

  const defaultCmd = 'source /opt/ros/humble/setup.bash && ros2 launch rosbridge_server rosbridge_websocket_launch.xml';
  const [command, setCommand] = useState(defaultCmd);
  const [user, setUser] = useState('amarine');
  const [copied, setCopied] = useState(false);
  const [isExecuting, setIsExecuting] = useState(false);
  const [logs, setLogs] = useState([
    { time: new Date().toLocaleTimeString(), text: 'System ready. Enter command or press "Jalankan & Konek".', type: 'info' }
  ]);
  const logEndRef = useRef(null);

  const appendLog = (text, type = 'info') => {
    setLogs((prev) => [...prev.slice(-30), { time: new Date().toLocaleTimeString(), text, type }]);
  };

  useEffect(() => {
    if (logEndRef.current) {
      logEndRef.current.scrollIntoView({ behavior: 'smooth' });
    }
  }, [logs]);

  const handleCopy = () => {
    navigator.clipboard.writeText(command);
    setCopied(true);
    appendLog(`📋 Command copied to clipboard: ${command}`, 'success');
    setTimeout(() => setCopied(false), 2000);
  };

  const handleExecuteAndLink = async () => {
    setIsExecuting(true);
    appendLog(`🚀 Running: ${command}`, 'warn');

    // Switch to live mode
    if (mode === 'demo') {
      mockRos.stop();
    }
    setMode('live');
    setJetsonIp(jetsonIp);
    rosConnection.isManualDisconnect = false;

    // Check if running inside Electron Desktop App
    if (window.jetsonSsh) {
      appendLog(`[SSH] Connecting to ${user}@${jetsonIp} via native SSH...`, 'info');
      try {
        const result = await window.jetsonSsh.startRosbridge(jetsonIp, user);
        if (result.ok) {
          appendLog(`[SSH] ✅ Rosbridge launched on Jetson! Waiting 2s for port 9090...`, 'success');
          await new Promise((r) => setTimeout(r, 2000));
        } else {
          appendLog(`[SSH] ⚠️ SSH warning: ${result.error || 'Failed to launch automatically'}`, 'error');
        }
      } catch (err) {
        appendLog(`[SSH] ❌ Error: ${err.message}`, 'error');
      }
    } else {
      // In Browser: copy command automatically for convenience
      try {
        await navigator.clipboard.writeText(command);
        appendLog(`[Browser] 📋 Command auto-copied to clipboard! Pastikan sudah dijalankan di terminal Jetson.`, 'info');
      } catch {
        // Ignore clipboard failure
      }
    }

    // Connect WebSocket
    appendLog(`[WS] Connecting WebSocket to ws://${jetsonIp}:9090 ...`, 'info');
    const ros = rosConnection.connect(jetsonIp);
    topicSubscriber.subscribeAll(ros);
    topicPublisher.init(ros);

    // Watch status
    setTimeout(() => {
      const isConn = rosConnection.isConnected();
      if (isConn) {
        appendLog(`[WS] 🟢 CONNECTED! Jetson Orin Digital Twin Link Active!`, 'success');
      } else {
        appendLog(`[WS] ⏳ Connecting in progress... Pastikan terminal Jetson sudah menjalankan rosbridge di port 9090.`, 'warn');
      }
      setIsExecuting(false);
    }, 1500);
  };

  const handleShutdownDaemon = () => {
    const daemonCmd = 'python3 ~/digitaltwinamarinefilkomub/backend/jetson_shutdown_daemon.py &';
    setCommand(daemonCmd);
    navigator.clipboard.writeText(daemonCmd);
    appendLog(`📋 Loaded & Copied Shutdown Daemon command: ${daemonCmd}`, 'success');
  };

  const handleBoxShutdown = async () => {
    const ok = window.confirm('⚠️ SHUTDOWN JETSON?\n\nJetson Orin akan dimatikan sepenuhnya.\nKamu perlu nyalakan fisik secara manual jika mau hidup lagi.\n\nLanjutkan?');
    if (!ok) return;

    appendLog('⏻ Mengirim perintah shutdown ke Jetson...', 'warn');

    // 1. ROS topic
    topicPublisher.shutdownVehicle();

    // 2. SSH shutdown in Electron
    if (window.jetsonSsh) {
      appendLog(`[SSH] Executing remote poweroff on ${user}@${jetsonIp}...`, 'info');
      try {
        const res = await window.jetsonSsh.shutdown(jetsonIp, user);
        if (res.ok) {
          appendLog('✅ [SSH] Shutdown sukses! Jetson sedang dimatikan.', 'success');
        } else {
          appendLog(`⚠️ [SSH] Respon: ${res.error || 'Perintah terkirim'}`, 'warn');
        }
      } catch (err) {
        appendLog(`❌ Error: ${err.message}`, 'error');
      }
    } else {
      appendLog('📋 Sinyal ROS2 terkirim. Jika di browser, pastikan listener aktif atau ketik: sudo shutdown now', 'info');
    }
  };

  return (
    <div
      style={{
        background: 'rgba(10, 15, 29, 0.95)',
        border: '1px solid rgba(0, 240, 255, 0.25)',
        borderRadius: '10px',
        padding: '12px',
        color: '#e2e8f0',
        fontFamily: 'var(--font-mono, monospace)',
        fontSize: '0.75rem',
        boxShadow: '0 4px 20px rgba(0, 0, 0, 0.5)',
        marginTop: '8px',
      }}
    >
      {/* Box Header */}
      <div
        style={{
          display: 'flex',
          alignItems: 'center',
          justifyContent: 'space-between',
          marginBottom: '8px',
          borderBottom: '1px solid rgba(0, 240, 255, 0.15)',
          paddingBottom: '6px',
        }}
      >
        <div style={{ display: 'flex', alignItems: 'center', gap: '6px', fontWeight: '700', color: '#00f0ff' }}>
          <span>💻</span>
          <span>JETSON COMMAND LINE</span>
        </div>
        <div
          style={{
            fontSize: '0.65rem',
            padding: '2px 6px',
            borderRadius: '4px',
            background: connectionStatus === 'connected' ? 'rgba(0, 255, 136, 0.2)' : 'rgba(255, 170, 0, 0.2)',
            color: connectionStatus === 'connected' ? '#00ff88' : '#ffaa00',
            fontWeight: '600',
          }}
        >
          {connectionStatus === 'connected' ? '● PORT 9090 LINKED' : '○ DISCONNECTED'}
        </div>
      </div>

      {/* Target Info */}
      <div style={{ display: 'flex', gap: '6px', marginBottom: '8px' }}>
        <input
          type="text"
          value={user}
          onChange={(e) => setUser(e.target.value)}
          placeholder="user"
          style={{
            width: '68px',
            background: 'rgba(15, 23, 42, 0.8)',
            border: '1px solid rgba(255, 255, 255, 0.15)',
            borderRadius: '4px',
            color: '#94a3b8',
            padding: '3px 6px',
            fontSize: '0.7rem',
          }}
          title="Jetson SSH User"
        />
        <input
          type="text"
          value={jetsonIp}
          onChange={(e) => setJetsonIp(e.target.value)}
          placeholder="Jetson IP"
          style={{
            flex: 1,
            background: 'rgba(15, 23, 42, 0.8)',
            border: '1px solid rgba(0, 240, 255, 0.2)',
            borderRadius: '4px',
            color: '#00f0ff',
            padding: '3px 6px',
            fontSize: '0.7rem',
            fontWeight: '600',
          }}
          title="Jetson IP Address"
        />
      </div>

      {/* Command Input Box */}
      <div style={{ position: 'relative', marginBottom: '8px' }}>
        <div
          style={{
            position: 'absolute',
            left: '8px',
            top: '6px',
            color: '#00f0ff',
            fontWeight: 'bold',
            userSelect: 'none',
          }}
        >
          $
        </div>
        <textarea
          rows={3}
          value={command}
          onChange={(e) => setCommand(e.target.value)}
          style={{
            width: '100%',
            background: 'rgba(5, 10, 20, 0.9)',
            border: '1px solid rgba(0, 240, 255, 0.3)',
            borderRadius: '6px',
            color: '#00ff88',
            fontFamily: 'monospace',
            fontSize: '0.68rem',
            lineHeight: '1.3',
            padding: '6px 8px 6px 20px',
            resize: 'none',
            outline: 'none',
            boxSizing: 'border-box',
          }}
        />
      </div>

      {/* Action Buttons */}
      <div style={{ display: 'flex', gap: '6px', marginBottom: '8px' }}>
        <button
          onClick={handleExecuteAndLink}
          disabled={isExecuting}
          style={{
            flex: 2,
            background: isExecuting
              ? 'rgba(0, 240, 255, 0.2)'
              : 'linear-gradient(135deg, rgba(0, 240, 255, 0.3) 0%, rgba(0, 128, 255, 0.3) 100%)',
            border: '1px solid #00f0ff',
            color: '#00f0ff',
            borderRadius: '6px',
            padding: '6px 8px',
            fontWeight: '700',
            fontSize: '0.72rem',
            cursor: isExecuting ? 'wait' : 'pointer',
            boxShadow: '0 0 10px rgba(0, 240, 255, 0.2)',
            transition: 'all 0.2s ease',
          }}
        >
          {isExecuting ? '⏳ Menghubungkan...' : '⚡ Jalankan & Konek'}
        </button>

        <button
          onClick={handleCopy}
          style={{
            flex: 1,
            background: copied ? 'rgba(0, 255, 136, 0.25)' : 'rgba(255, 255, 255, 0.08)',
            border: `1px solid ${copied ? '#00ff88' : 'rgba(255, 255, 255, 0.2)'}`,
            color: copied ? '#00ff88' : '#e2e8f0',
            borderRadius: '6px',
            padding: '6px 8px',
            fontSize: '0.7rem',
            fontWeight: '600',
            cursor: 'pointer',
          }}
        >
          {copied ? '✓ Tersalin!' : '📋 Salin'}
        </button>

        <button
          onClick={handleBoxShutdown}
          style={{
            flex: 1,
            background: 'rgba(255, 59, 92, 0.2)',
            border: '1px solid rgba(255, 59, 92, 0.5)',
            color: '#ff3b5c',
            borderRadius: '6px',
            padding: '6px 8px',
            fontSize: '0.7rem',
            fontWeight: '700',
            cursor: 'pointer',
          }}
          title="Matikan Jetson Orin"
        >
          ⏻ Mati
        </button>
      </div>

      {/* Quick Command Presets */}
      <div style={{ marginBottom: '8px' }}>
        <div style={{ fontSize: '0.62rem', color: 'rgba(255,255,255,0.35)', marginBottom: '4px', letterSpacing: '0.05em' }}>
          ⚡ QUICK COMMANDS
        </div>
        <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: '4px' }}>

          {/* 1. Restart manual_bridge */}
          <button
            id="btn-restart-bridge"
            onClick={() => {
              const cmd = 'pkill -f manual_bridge.py; sleep 2; cd ~/digitaltwinamarinefilkomub && source /opt/ros/humble/setup.bash && python3 backend/sauvc26_code/manual_bridge.py';
              setCommand(cmd);
              navigator.clipboard.writeText(cmd).catch(() => {});
              appendLog('📋 Loaded: Restart manual_bridge.py', 'info');
            }}
            style={{
              background: 'linear-gradient(135deg, rgba(0,240,255,0.12) 0%, rgba(0,80,180,0.15) 100%)',
              border: '1px solid rgba(0,240,255,0.3)',
              borderRadius: '5px',
              padding: '5px 4px',
              color: '#00f0ff',
              fontSize: '0.62rem',
              fontWeight: '600',
              cursor: 'pointer',
              textAlign: 'left',
              lineHeight: '1.3',
              transition: 'all 0.15s',
            }}
            onMouseEnter={e => e.currentTarget.style.background = 'rgba(0,240,255,0.2)'}
            onMouseLeave={e => e.currentTarget.style.background = 'linear-gradient(135deg, rgba(0,240,255,0.12) 0%, rgba(0,80,180,0.15) 100%)'}
            title="pkill -f manual_bridge.py && python3 backend/sauvc26_code/manual_bridge.py"
          >
            🔄 Restart Bridge
            <div style={{ fontSize: '0.55rem', color: 'rgba(0,240,255,0.6)', fontWeight: '400' }}>manual_bridge.py</div>
          </button>

          {/* 2. Launch backend digital twin */}
          <button
            id="btn-launch-backend"
            onClick={() => {
              const cmd = 'cd ~/digitaltwinamarinefilkomub && source /opt/ros/humble/setup.bash && python3 backend/launch_digitaltwin.launch.py';
              setCommand(cmd);
              navigator.clipboard.writeText(cmd).catch(() => {});
              appendLog('📋 Loaded: Launch Digital Twin backend', 'info');
            }}
            style={{
              background: 'linear-gradient(135deg, rgba(0,255,136,0.1) 0%, rgba(0,120,80,0.15) 100%)',
              border: '1px solid rgba(0,255,136,0.3)',
              borderRadius: '5px',
              padding: '5px 4px',
              color: '#00ff88',
              fontSize: '0.62rem',
              fontWeight: '600',
              cursor: 'pointer',
              textAlign: 'left',
              lineHeight: '1.3',
              transition: 'all 0.15s',
            }}
            onMouseEnter={e => e.currentTarget.style.background = 'rgba(0,255,136,0.2)'}
            onMouseLeave={e => e.currentTarget.style.background = 'linear-gradient(135deg, rgba(0,255,136,0.1) 0%, rgba(0,120,80,0.15) 100%)'}
            title="Launch digital twin backend + rosbridge"
          >
            🚀 Launch Backend
            <div style={{ fontSize: '0.55rem', color: 'rgba(0,255,136,0.6)', fontWeight: '400' }}>launch_digitaltwin</div>
          </button>

          {/* 3. Echo /cmd_vel */}
          <button
            id="btn-echo-cmdvel"
            onClick={() => {
              const cmd = 'ros2 topic echo /cmd_vel';
              setCommand(cmd);
              navigator.clipboard.writeText(cmd).catch(() => {});
              appendLog('📋 Loaded: ros2 topic echo /cmd_vel', 'info');
            }}
            style={{
              background: 'linear-gradient(135deg, rgba(250,204,21,0.1) 0%, rgba(150,100,0,0.15) 100%)',
              border: '1px solid rgba(250,204,21,0.3)',
              borderRadius: '5px',
              padding: '5px 4px',
              color: '#facc15',
              fontSize: '0.62rem',
              fontWeight: '600',
              cursor: 'pointer',
              textAlign: 'left',
              lineHeight: '1.3',
              transition: 'all 0.15s',
            }}
            onMouseEnter={e => e.currentTarget.style.background = 'rgba(250,204,21,0.2)'}
            onMouseLeave={e => e.currentTarget.style.background = 'linear-gradient(135deg, rgba(250,204,21,0.1) 0%, rgba(150,100,0,0.15) 100%)'}
            title="ros2 topic echo /cmd_vel"
          >
            📡 Echo cmd_vel
            <div style={{ fontSize: '0.55rem', color: 'rgba(250,204,21,0.6)', fontWeight: '400' }}>/cmd_vel monitor</div>
          </button>

          {/* 4. Check MAVROS nodes */}
          <button
            id="btn-check-mavros"
            onClick={() => {
              const cmd = 'ros2 node list | grep mavros';
              setCommand(cmd);
              navigator.clipboard.writeText(cmd).catch(() => {});
              appendLog('📋 Loaded: ros2 node list | grep mavros', 'info');
            }}
            style={{
              background: 'linear-gradient(135deg, rgba(168,85,247,0.1) 0%, rgba(80,0,150,0.15) 100%)',
              border: '1px solid rgba(168,85,247,0.3)',
              borderRadius: '5px',
              padding: '5px 4px',
              color: '#a855f7',
              fontSize: '0.62rem',
              fontWeight: '600',
              cursor: 'pointer',
              textAlign: 'left',
              lineHeight: '1.3',
              transition: 'all 0.15s',
            }}
            onMouseEnter={e => e.currentTarget.style.background = 'rgba(168,85,247,0.2)'}
            onMouseLeave={e => e.currentTarget.style.background = 'linear-gradient(135deg, rgba(168,85,247,0.1) 0%, rgba(80,0,150,0.15) 100%)'}
            title="ros2 node list | grep mavros"
          >
            🔍 Cek MAVROS
            <div style={{ fontSize: '0.55rem', color: 'rgba(168,85,247,0.6)', fontWeight: '400' }}>node list mavros</div>
          </button>

        </div>
      </div>

      {/* Preset Classic Tabs */}
      <div style={{ display: 'flex', gap: '4px', marginBottom: '8px' }}>
        <button
          onClick={() => setCommand(defaultCmd)}
          style={{
            flex: 1,
            background: 'rgba(15, 23, 42, 0.7)',
            border: '1px solid rgba(255, 255, 255, 0.1)',
            borderRadius: '4px',
            padding: '3px 4px',
            color: '#94a3b8',
            fontSize: '0.62rem',
            cursor: 'pointer',
          }}
          title="Reset ke perintah rosbridge launch"
        >
          Rosbridge
        </button>
        <button
          onClick={handleShutdownDaemon}
          style={{
            flex: 1,
            background: 'rgba(15, 23, 42, 0.7)',
            border: '1px solid rgba(255, 255, 255, 0.1)',
            borderRadius: '4px',
            padding: '3px 4px',
            color: '#94a3b8',
            fontSize: '0.62rem',
            cursor: 'pointer',
          }}
          title="Salin perintah listener shutdown untuk Jetson"
        >
          Shutdown Daemon
        </button>
      </div>

      {/* Mini Terminal Log Screen */}
      <div
        style={{
          background: 'rgba(0, 0, 0, 0.75)',
          border: '1px solid rgba(255, 255, 255, 0.1)',
          borderRadius: '4px',
          padding: '6px',
          maxHeight: '85px',
          overflowY: 'auto',
          fontSize: '0.64rem',
          lineHeight: '1.35',
        }}
      >
        {logs.map((log, i) => (
          <div
            key={i}
            style={{
              color:
                log.type === 'error'
                  ? '#ff3b5c'
                  : log.type === 'warn'
                  ? '#ffaa00'
                  : log.type === 'success'
                  ? '#00ff88'
                  : '#94a3b8',
              whiteSpace: 'pre-wrap',
              wordBreak: 'break-all',
            }}
          >
            <span style={{ color: 'rgba(255, 255, 255, 0.35)', marginRight: '4px' }}>[{log.time}]</span>
            {log.text}
          </div>
        ))}
        <div ref={logEndRef} />
      </div>
    </div>
  );
}
