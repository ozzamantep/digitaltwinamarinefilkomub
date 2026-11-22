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
  const monitorState = useVehicleStore((s) => s.monitorState);
  const setMonitorState = useVehicleStore((s) => s.setMonitorState);

  const defaultCmd = 'source /opt/ros/humble/setup.bash && ros2 launch rosbridge_server rosbridge_websocket_launch.xml';
  const monitorCmd = 'source /opt/ros/humble/setup.bash 2>/dev/null; (cd ~/digitaltwin 2>/dev/null || cd ~/digitaltwinamarinefilkomub 2>/dev/null); python3 backend/sauvc26_code/jetson_monitor_node.py';
  const [command, setCommand] = useState(defaultCmd);
  const [user, setUser] = useState('amarine');
  const [copied, setCopied] = useState(false);
  const [isExecuting, setIsExecuting] = useState(false);
  const [logs, setLogs] = useState([
    { time: new Date().toLocaleTimeString(), text: 'System ready. Enter command or press "Jalankan & Konek".', type: 'info' }
  ]);
  const logEndRef = useRef(null);

  const handleRunMonitorNode = async () => {
    setCommand(monitorCmd);
    if (window.jetsonSsh) {
      if (monitorState === 'running') {
        appendLog('[SSH] Menghentikan jetson_monitor_node.py...', 'warn');
        await window.jetsonSsh.stopMonitor();
        setMonitorState('idle');
        appendLog('[SSH] ✅ Jetson Monitor Node dihentikan.', 'info');
      } else {
        appendLog(`[SSH] Menjalankan jetson_monitor_node.py ke ${user}@${jetsonIp}...`, 'info');
        setMonitorState('starting');
        const res = await window.jetsonSsh.startMonitor(jetsonIp, user);
        setMonitorState(res.ok ? 'running' : 'error');
        if (res.ok) {
          appendLog('✅ [SSH] Jetson Monitor Node BERJALAN! Telemetri aktif di /jetson/diagnostics', 'success');
        } else {
          appendLog(`❌ [SSH] Gagal: ${res.error || 'Periksa koneksi SSH'}`, 'error');
        }
      }
    } else {
      navigator.clipboard.writeText(monitorCmd).catch(() => {});
      appendLog('📋 [Browser] Perintah Jetson Monitor disalin ke clipboard! Jalankan di terminal Jetson.', 'success');
    }
  };

  const appendLog = (text, type = 'info') => {
    setLogs((prev) => [...prev.slice(-30), { time: new Date().toLocaleTimeString(), text, type }]);
  };

  useEffect(() => {
    if (logEndRef.current) {
      logEndRef.current.scrollIntoView({ behavior: 'smooth' });
    }
  }, [logs]);

  useEffect(() => {
    if (connectionStatus === 'connected') {
      appendLog('🟢 [WS] TERHUBUNG! Jetson Digital Twin Link Aktif di port 9090!', 'success');
    }
  }, [connectionStatus]);

  const handleCopy = () => {
    navigator.clipboard.writeText(command);
    setCopied(true);
    appendLog(`📋 Command copied to clipboard: ${command}`, 'success');
    setTimeout(() => setCopied(false), 2000);
  };

  const handleExecuteAndLink = async () => {
    setIsExecuting(true);
    appendLog(`🚀 Menjalankan: ${command}`, 'warn');

    // Switch to live mode
    if (mode === 'demo') {
      mockRos.stop();
    }
    setMode('live');
    setJetsonIp(jetsonIp);
    rosConnection.isManualDisconnect = false;

    // Check if running inside Electron Desktop App
    if (window.jetsonSsh) {
      appendLog(`[SSH] Menghubungkan ke ${user}@${jetsonIp} via SSH...`, 'info');
      try {
        const result = await window.jetsonSsh.startRosbridge(jetsonIp, user);
        if (result.ok) {
          appendLog(`[SSH] ✅ Rosbridge aktif di Jetson! Menunggu port 9090...`, 'success');
          await new Promise((r) => setTimeout(r, 2000));
        } else {
          appendLog(`[SSH] ⚠️ SSH: ${result.error || 'Gagal otomatis. Jalankan rosbridge manual di Jetson.'}`, 'error');
        }
      } catch (err) {
        appendLog(`[SSH] ❌ Error: ${err.message}`, 'error');
      }
    } else {
      // In Browser: copy command automatically for convenience
      try {
        await navigator.clipboard.writeText(command);
        appendLog(`[Browser] 📋 Perintah disalin ke clipboard. Pastikan rosbridge sudah jalan di Jetson.`, 'info');
      } catch {
        // Ignore clipboard failure
      }
    }

    // Connect WebSocket
    appendLog(`[WS] Menghubungkan WebSocket ke ws://${jetsonIp}:9090 ...`, 'info');
    const ros = rosConnection.connect(jetsonIp);
    topicSubscriber.subscribeAll(ros);
    topicPublisher.init(ros);

    ros.on('connection', () => {
      appendLog(`[WS] 🟢 TERHUBUNG! Jetson Digital Twin Link Aktif!`, 'success');
      setIsExecuting(false);
    });

    ros.on('error', (err) => {
      appendLog(`[WS] ❌ WebSocket Error ke ws://${jetsonIp}:9090`, 'error');
    });

    // Fallback status check
    const checkConnection = (attempt) => {
      const isConn = rosConnection.isConnected();
      if (isConn) {
        setIsExecuting(false);
      } else if (attempt < 2) {
        setTimeout(() => checkConnection(attempt + 1), 3000);
      } else {
        if (!rosConnection.isConnected()) {
          appendLog(`[WS] ⚠️ Masih mencoba menghubungkan ke ws://${jetsonIp}:9090. Auto-reconnect aktif di latar belakang.`, 'warn');
        }
        setIsExecuting(false);
      }
    };
    setTimeout(() => checkConnection(0), 2000);
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

      {!window.jetsonSsh && (
        <div
          style={{
            background: 'var(--bg-card)',
            border: '1px solid var(--accent-orange)',
            borderRadius: '2px',
            padding: '8px 10px',
            marginBottom: '8px',
            fontSize: '0.68rem',
            lineHeight: '1.45',
            color: 'var(--accent-orange)',
          }}
        >
          <strong>MOBILE / BROWSER:</strong> SSH otomatis hanya berjalan di Desktop. Jalankan node Jetson (<code>rosbridge</code>, <code>odom_bridge.py</code>, <code>manual_bridge.py</code>) via SSH/Termius lalu tekan <strong>CONNECT</strong>.
        </div>
      )}

      {/* Target Info */}
      <div style={{ display: 'flex', gap: '6px', marginBottom: '8px' }}>
        <input
          type="text"
          value={user}
          onChange={(e) => setUser(e.target.value)}
          placeholder="user"
          style={{
            width: '72px',
            background: 'var(--bg-primary)',
            border: '1px solid var(--border-glass)',
            borderRadius: '2px',
            color: 'var(--text-secondary)',
            padding: '4px 6px',
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
            background: 'var(--bg-primary)',
            border: '1px solid var(--border-glass)',
            borderRadius: '2px',
            color: 'var(--text-primary)',
            padding: '4px 8px',
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
            color: 'var(--accent-green)',
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
            background: 'var(--bg-primary)',
            border: '1px solid var(--border-glass)',
            borderRadius: '2px',
            color: 'var(--accent-green)',
            fontFamily: 'var(--font-mono)',
            fontSize: '0.68rem',
            lineHeight: '1.35',
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
            background: isExecuting ? 'var(--bg-tertiary)' : 'var(--accent-green-dim)',
            border: '1px solid var(--accent-green)',
            color: 'var(--accent-green)',
            borderRadius: '2px',
            padding: '6px 8px',
            fontWeight: '700',
            fontSize: '0.7rem',
            letterSpacing: '0.04em',
            cursor: isExecuting ? 'wait' : 'pointer',
          }}
        >
          {isExecuting ? 'CONNECTING...' : '[2] RUN & LINK'}
        </button>

        <button
          onClick={handleCopy}
          style={{
            flex: 1,
            background: copied ? 'var(--accent-green-dim)' : 'var(--bg-tertiary)',
            border: `1px solid ${copied ? 'var(--accent-green)' : 'var(--border-glass)'}`,
            color: copied ? 'var(--accent-green)' : 'var(--text-primary)',
            borderRadius: '2px',
            padding: '6px 8px',
            fontSize: '0.68rem',
            fontWeight: '600',
            cursor: 'pointer',
          }}
        >
          {copied ? 'COPIED' : 'COPY'}
        </button>

        <button
          onClick={handleBoxShutdown}
          style={{
            flex: 1,
            background: 'var(--accent-red-dim)',
            border: '1px solid var(--accent-red)',
            color: 'var(--accent-red)',
            borderRadius: '2px',
            padding: '6px 8px',
            fontSize: '0.68rem',
            fontWeight: '700',
            cursor: 'pointer',
          }}
          title="Matikan Jetson Orin"
        >
          POWEROFF
        </button>
      </div>

      {/* Quick Command Presets */}
      <div style={{ marginBottom: '8px' }}>
        <div style={{ fontSize: '0.62rem', color: 'var(--text-tertiary)', marginBottom: '4px', letterSpacing: '0.06em' }}>
          SEQUENCE: 1 LAUNCH -&gt; 2 LINK -&gt; 3 MONITOR -&gt; 4 PILOT
        </div>
        <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: '4px' }}>

          {/* LANGKAH 1: Launch Backend */}
          <button
            id="btn-launch-backend"
            onClick={() => {
              const cmd = 'source /opt/ros/humble/setup.bash && ros2 launch rosbridge_server rosbridge_websocket_launch.xml';
              setCommand(cmd);
              navigator.clipboard.writeText(cmd).catch(() => {});
              appendLog('[1] Loaded: rosbridge port 9090 launch command', 'info');
            }}
            className="control-btn"
            style={{
              padding: '6px 8px',
              textAlign: 'left',
              lineHeight: '1.35',
              gridColumn: 'span 2',
              justifyContent: 'flex-start',
              flexDirection: 'column',
              alignItems: 'flex-start',
            }}
            title="Langkah 1 (Wajib Pertama): Menyalakan backend ROS 2 & Rosbridge server port 9090 di Jetson"
          >
            <div style={{ fontSize: '0.64rem', fontWeight: '700', color: 'var(--text-primary)' }}>
              [1] LAUNCH BACKEND (ROSBRIDGE :9090)
            </div>
            <div style={{ fontSize: '0.56rem', color: 'var(--text-tertiary)', fontWeight: '400' }}>
              launch_digitaltwin.launch.py
            </div>
          </button>

          {/* LANGKAH 3: Jetson Monitor Node */}
          <button
            id="btn-jetson-monitor"
            onClick={handleRunMonitorNode}
            className="control-btn"
            style={{
              padding: '6px 8px',
              textAlign: 'left',
              lineHeight: '1.35',
              gridColumn: 'span 2',
              justifyContent: 'flex-start',
              flexDirection: 'column',
              alignItems: 'flex-start',
              borderColor: monitorState === 'running' ? 'var(--accent-green)' : 'var(--border-glass)',
              background: monitorState === 'running' ? 'var(--accent-green-dim)' : undefined,
            }}
            title="Langkah 3: Jalankan jetson_monitor_node.py untuk streaming live CPU/GPU/RAM asli"
          >
            <div style={{ fontSize: '0.64rem', fontWeight: '700', color: monitorState === 'running' ? 'var(--accent-green)' : 'var(--text-primary)' }}>
              [3] {monitorState === 'running' ? 'STOP JETSON MONITOR' : monitorState === 'starting' ? 'STARTING MONITOR...' : 'JETSON MONITOR NODE'}
            </div>
            <div style={{ fontSize: '0.56rem', color: monitorState === 'running' ? 'var(--accent-green)' : 'var(--text-tertiary)', fontWeight: '400' }}>
              jetson_monitor_node.py {monitorState === 'running' ? '[RUNNING]' : ''}
            </div>
          </button>

          {/* LANGKAH 4: Restart manual_bridge */}
          <button
            id="btn-restart-bridge"
            onClick={() => {
              const cmd = 'pkill -f manual_bridge.py; sleep 2; cd ~/digitaltwinamarinefilkomub && source /opt/ros/humble/setup.bash && python3 backend/sauvc26_code/manual_bridge.py';
              setCommand(cmd);
              navigator.clipboard.writeText(cmd).catch(() => {});
              appendLog('[4] Loaded: manual_bridge.py', 'info');
            }}
            className="control-btn"
            style={{
              padding: '6px 8px',
              textAlign: 'left',
              lineHeight: '1.35',
              gridColumn: 'span 2',
              justifyContent: 'flex-start',
              flexDirection: 'column',
              alignItems: 'flex-start',
            }}
            title="Langkah 4: Jalankan bridge kontrol manual (cmd_vel & gripper ke MAVROS)"
          >
            <div style={{ fontSize: '0.64rem', fontWeight: '700', color: 'var(--text-primary)' }}>
              [4] PILOT BRIDGE (manual_bridge.py)
            </div>
            <div style={{ fontSize: '0.56rem', color: 'var(--text-tertiary)', fontWeight: '400' }}>
              /cmd_vel &amp; /gripper/command -&gt; MAVROS
            </div>
          </button>

          {/* Alat Cek 1: Echo /cmd_vel */}
          <button
            id="btn-echo-cmdvel"
            onClick={() => {
              const cmd = 'ros2 topic echo /cmd_vel';
              setCommand(cmd);
              navigator.clipboard.writeText(cmd).catch(() => {});
              appendLog('Loaded: ros2 topic echo /cmd_vel', 'info');
            }}
            className="control-btn"
            style={{
              padding: '5px 6px',
              textAlign: 'left',
              lineHeight: '1.3',
              flexDirection: 'column',
              alignItems: 'flex-start',
            }}
            title="Cek apakah perintah joystick /cmd_vel terkirim ke Jetson"
          >
            <div style={{ fontSize: '0.62rem', fontWeight: '700', color: 'var(--text-primary)' }}>ECHO /cmd_vel</div>
            <div style={{ fontSize: '0.54rem', color: 'var(--text-tertiary)', fontWeight: '400' }}>Monitor gerak</div>
          </button>

          {/* Alat Cek 2: Check MAVROS nodes */}
          <button
            id="btn-check-mavros"
            onClick={() => {
              const cmd = 'ros2 node list | grep mavros';
              setCommand(cmd);
              navigator.clipboard.writeText(cmd).catch(() => {});
              appendLog('Loaded: ros2 node list | grep mavros', 'info');
            }}
            className="control-btn"
            style={{
              padding: '5px 6px',
              textAlign: 'left',
              lineHeight: '1.3',
              flexDirection: 'column',
              alignItems: 'flex-start',
            }}
            title="Cek apakah MAVROS aktif di Jetson"
          >
            <div style={{ fontSize: '0.62rem', fontWeight: '700', color: 'var(--text-primary)' }}>CHECK MAVROS</div>
            <div style={{ fontSize: '0.54rem', color: 'var(--text-tertiary)', fontWeight: '400' }}>Node status</div>
          </button>

        </div>
      </div>

      {/* Preset Classic Tabs */}
      <div style={{ display: 'flex', gap: '4px', marginBottom: '8px' }}>
        <button
          onClick={() => {
            setCommand(defaultCmd);
            appendLog('📋 [1] Dimuat perintah Rosbridge websocket', 'info');
          }}
          style={{
            flex: 1,
            background: 'rgba(15, 23, 42, 0.7)',
            border: '1px solid rgba(0, 255, 136, 0.3)',
            borderRadius: '4px',
            padding: '3px 4px',
            color: '#00ff88',
            fontSize: '0.62rem',
            cursor: 'pointer',
            fontWeight: '600',
          }}
          title="Reset ke perintah rosbridge launch"
        >
          [1] Rosbridge
        </button>
        <button
          onClick={() => {
            const cmd = 'python3 ~/digitaltwin/backend/sauvc26_code/jetson_monitor_node.py';
            setCommand(cmd);
            navigator.clipboard.writeText(cmd).catch(() => {});
            appendLog(`📋 [3] Dimuat perintah: ${cmd}`, 'info');
          }}
          style={{
            flex: 1,
            background: 'rgba(15, 23, 42, 0.7)',
            border: '1px solid rgba(0, 240, 255, 0.3)',
            borderRadius: '4px',
            padding: '3px 4px',
            color: '#00f0ff',
            fontSize: '0.62rem',
            cursor: 'pointer',
            fontWeight: '600',
          }}
          title="Salin perintah jetson_monitor_node.py"
        >
          [3] Monitor Node
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
