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

      {/* Visual Step Guide Banner */}
      <div
        style={{
          background: 'rgba(0, 240, 255, 0.08)',
          border: '1px solid rgba(0, 240, 255, 0.25)',
          borderRadius: '6px',
          padding: '6px 8px',
          marginBottom: '8px',
          fontSize: '0.64rem',
          lineHeight: '1.4',
        }}
      >
        <div style={{ color: '#00f0ff', fontWeight: '700', marginBottom: '2px', display: 'flex', alignItems: 'center', gap: '4px' }}>
          <span>🧭</span>
          <span>URUTAN KLIK DARI AWAL:</span>
        </div>
        <div style={{ color: '#94a3b8', fontSize: '0.6rem', display: 'flex', flexWrap: 'wrap', gap: '4px', alignItems: 'center' }}>
          <span style={{ background: 'rgba(0,255,136,0.15)', color: '#00ff88', padding: '1px 5px', borderRadius: '3px', fontWeight: '700' }}>① Launch Backend</span>
          <span>➔</span>
          <span style={{ background: 'rgba(0,240,255,0.15)', color: '#00f0ff', padding: '1px 5px', borderRadius: '3px', fontWeight: '700' }}>② Jalankan & Konek</span>
          <span>➔</span>
          <span style={{ background: 'rgba(0,255,200,0.15)', color: '#00ffc8', padding: '1px 5px', borderRadius: '3px', fontWeight: '700' }}>③ Monitor Node</span>
          <span>➔</span>
          <span style={{ background: 'rgba(250,204,21,0.15)', color: '#facc15', padding: '1px 5px', borderRadius: '3px', fontWeight: '700' }}>④ Pilot Bridge</span>
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
          {isExecuting ? '⏳ Menghubungkan...' : '⚡ [2] Jalankan & Konek'}
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
          ⚡ QUICK COMMANDS (KLIK BERURUTAN)
        </div>
        <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: '4px' }}>

          {/* LANGKAH 1: Launch Backend */}
          <button
            id="btn-launch-backend"
            onClick={() => {
              const cmd = 'cd ~/digitaltwinamarinefilkomub && source /opt/ros/humble/setup.bash && python3 backend/launch_digitaltwin.launch.py';
              setCommand(cmd);
              navigator.clipboard.writeText(cmd).catch(() => {});
              appendLog('📋 [Langkah 1] Dimuat: Launch Digital Twin backend & Rosbridge', 'info');
            }}
            style={{
              background: 'linear-gradient(135deg, rgba(0,255,136,0.15) 0%, rgba(0,120,80,0.2) 100%)',
              border: '1px solid rgba(0,255,136,0.4)',
              borderRadius: '5px',
              padding: '6px 6px',
              color: '#00ff88',
              fontSize: '0.62rem',
              fontWeight: '700',
              cursor: 'pointer',
              textAlign: 'left',
              lineHeight: '1.3',
              transition: 'all 0.15s',
              gridColumn: 'span 2',
            }}
            onMouseEnter={e => e.currentTarget.style.background = 'rgba(0,255,136,0.25)'}
            onMouseLeave={e => e.currentTarget.style.background = 'linear-gradient(135deg, rgba(0,255,136,0.15) 0%, rgba(0,120,80,0.2) 100%)'}
            title="Langkah 1 (Wajib Pertama): Menyalakan backend ROS 2 & Rosbridge server port 9090 di Jetson"
          >
            🚀 [1] Launch Backend (KLIK PERTAMA)
            <div style={{ fontSize: '0.55rem', color: 'rgba(0,255,136,0.7)', fontWeight: '400' }}>
              Buka gerbang port 9090 Jetson (launch_digitaltwin.launch.py)
            </div>
          </button>

          {/* LANGKAH 3: Jetson Monitor Node */}
          <button
            id="btn-jetson-monitor"
            onClick={handleRunMonitorNode}
            style={{
              background: monitorState === 'running'
                ? 'linear-gradient(135deg, rgba(0,255,136,0.2) 0%, rgba(0,180,120,0.25) 100%)'
                : 'linear-gradient(135deg, rgba(0,240,255,0.14) 0%, rgba(0,150,220,0.18) 100%)',
              border: monitorState === 'running' ? '1px solid #00ff88' : '1px solid rgba(0,240,255,0.4)',
              borderRadius: '5px',
              padding: '6px 6px',
              color: monitorState === 'running' ? '#00ff88' : '#00f0ff',
              fontSize: '0.62rem',
              fontWeight: '600',
              cursor: 'pointer',
              textAlign: 'left',
              lineHeight: '1.3',
              transition: 'all 0.15s',
              gridColumn: 'span 2',
            }}
            onMouseEnter={e => e.currentTarget.style.background = monitorState === 'running' ? 'rgba(0,255,136,0.3)' : 'rgba(0,240,255,0.25)'}
            onMouseLeave={e => e.currentTarget.style.background = monitorState === 'running' ? 'linear-gradient(135deg, rgba(0,255,136,0.2) 0%, rgba(0,180,120,0.25) 100%)' : 'linear-gradient(135deg, rgba(0,240,255,0.14) 0%, rgba(0,150,220,0.18) 100%)'}
            title="Langkah 3: Jalankan jetson_monitor_node.py untuk streaming live CPU/GPU/RAM asli"
          >
            📊 [3] {monitorState === 'running' ? '⏹ Stop Jetson Monitor Node' : monitorState === 'starting' ? '⏳ Starting Monitor Node...' : 'Jalankan Jetson Monitor Node'}
            <div style={{ fontSize: '0.55rem', color: monitorState === 'running' ? '#00ff88' : 'rgba(0,240,255,0.7)', fontWeight: '400' }}>
              python3 ~/digitaltwin/backend/sauvc26_code/jetson_monitor_node.py {monitorState === 'running' ? '● RUNNING' : ''}
            </div>
          </button>

          {/* LANGKAH 4: Restart manual_bridge */}
          <button
            id="btn-restart-bridge"
            onClick={() => {
              const cmd = 'pkill -f manual_bridge.py; sleep 2; cd ~/digitaltwinamarinefilkomub && source /opt/ros/humble/setup.bash && python3 backend/sauvc26_code/manual_bridge.py';
              setCommand(cmd);
              navigator.clipboard.writeText(cmd).catch(() => {});
              appendLog('📋 [Langkah 4] Dimuat: Restart manual_bridge.py', 'info');
            }}
            style={{
              background: 'linear-gradient(135deg, rgba(250,204,21,0.12) 0%, rgba(150,100,0,0.15) 100%)',
              border: '1px solid rgba(250,204,21,0.35)',
              borderRadius: '5px',
              padding: '6px 6px',
              color: '#facc15',
              fontSize: '0.62rem',
              fontWeight: '600',
              cursor: 'pointer',
              textAlign: 'left',
              lineHeight: '1.3',
              transition: 'all 0.15s',
              gridColumn: 'span 2',
            }}
            onMouseEnter={e => e.currentTarget.style.background = 'rgba(250,204,21,0.22)'}
            onMouseLeave={e => e.currentTarget.style.background = 'linear-gradient(135deg, rgba(250,204,21,0.12) 0%, rgba(150,100,0,0.15) 100%)'}
            title="Langkah 4: Jalankan bridge kontrol manual (cmd_vel & gripper ke MAVROS)"
          >
            🔄 [4] Pilot Bridge (manual_bridge.py)
            <div style={{ fontSize: '0.55rem', color: 'rgba(250,204,21,0.7)', fontWeight: '400' }}>Aktifkan kontrol joystick AUV</div>
          </button>

          {/* Alat Cek 1: Echo /cmd_vel */}
          <button
            id="btn-echo-cmdvel"
            onClick={() => {
              const cmd = 'ros2 topic echo /cmd_vel';
              setCommand(cmd);
              navigator.clipboard.writeText(cmd).catch(() => {});
              appendLog('📋 Dimuat alat cek: ros2 topic echo /cmd_vel', 'info');
            }}
            style={{
              background: 'linear-gradient(135deg, rgba(0,240,255,0.1) 0%, rgba(0,80,180,0.15) 100%)',
              border: '1px solid rgba(0,240,255,0.25)',
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
            onMouseLeave={e => e.currentTarget.style.background = 'linear-gradient(135deg, rgba(0,240,255,0.1) 0%, rgba(0,80,180,0.15) 100%)'}
            title="Cek apakah perintah joystick /cmd_vel terkirim ke Jetson"
          >
            📡 Echo cmd_vel
            <div style={{ fontSize: '0.55rem', color: 'rgba(0,240,255,0.6)', fontWeight: '400' }}>Monitor gerak</div>
          </button>

          {/* Alat Cek 2: Check MAVROS nodes */}
          <button
            id="btn-check-mavros"
            onClick={() => {
              const cmd = 'ros2 node list | grep mavros';
              setCommand(cmd);
              navigator.clipboard.writeText(cmd).catch(() => {});
              appendLog('📋 Dimuat alat cek: ros2 node list | grep mavros', 'info');
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
            title="Cek apakah MAVROS aktif di Jetson"
          >
            🔍 Cek MAVROS
            <div style={{ fontSize: '0.55rem', color: 'rgba(168,85,247,0.6)', fontWeight: '400' }}>Status MAVROS</div>
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
