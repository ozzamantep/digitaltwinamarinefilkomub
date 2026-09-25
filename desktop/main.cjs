const { app, BrowserWindow, ipcMain } = require('electron');
const { spawn } = require('child_process');
const path = require('path');
const fs = require('fs');
const http = require('http');

// 1. Performance and GPU optimization flags for smooth 60fps WebGL / Three.js
app.commandLine.appendSwitch('ignore-gpu-blocklist');
app.commandLine.appendSwitch('enable-gpu-rasterization');
app.commandLine.appendSwitch('enable-zero-copy');

let mainWindow = null;
let jetsonSshProcess = null;

function validateSshTarget(host, user) {
  return /^[a-zA-Z0-9.-]+$/.test(host) && /^[a-zA-Z0-9_-]+$/.test(user);
}

ipcMain.handle('jetson:ssh-status', () => ({ running: Boolean(jetsonSshProcess && !jetsonSshProcess.killed) }));

ipcMain.handle('jetson:start-rosbridge', async (_, { host, user }) => {
  if (!validateSshTarget(host, user)) return { ok: false, error: 'Invalid Jetson host or username.' };
  if (jetsonSshProcess && !jetsonSshProcess.killed) return { ok: true, alreadyRunning: true };

  const command = 'bash -c "source /opt/ros/humble/setup.bash && ros2 launch rosbridge_server rosbridge_websocket_launch.xml"';
  jetsonSshProcess = spawn('ssh', ['-o', 'BatchMode=yes', '-o', 'ConnectTimeout=8', `${user}@${host}`, command], {
    windowsHide: true,
  });
  let error = '';
  jetsonSshProcess.stderr.on('data', (chunk) => { error += chunk.toString(); });
  jetsonSshProcess.on('error', (spawnError) => { error = spawnError.message; });
  jetsonSshProcess.on('close', () => { jetsonSshProcess = null; });

  return await new Promise((resolve) => setTimeout(() => {
    if (error) resolve({ ok: false, error: error.trim() });
    else resolve({ ok: Boolean(jetsonSshProcess), error: '' });
  }, 900));
});

ipcMain.handle('jetson:stop-rosbridge', () => {
  if (jetsonSshProcess && !jetsonSshProcess.killed) jetsonSshProcess.kill();
  jetsonSshProcess = null;
  return { ok: true };
});

let jetsonOdomProcess = null;

ipcMain.handle('jetson:odom-status', () => ({ running: Boolean(jetsonOdomProcess && !jetsonOdomProcess.killed) }));

ipcMain.handle('jetson:start-odom-bridge', async (_, { host, user }) => {
  if (!validateSshTarget(host, user)) return { ok: false, error: 'Invalid Jetson host or username.' };
  if (jetsonOdomProcess && !jetsonOdomProcess.killed) return { ok: true, alreadyRunning: true };

  const command = 'bash -c "source /opt/ros/humble/setup.bash && cd ~/digitaltwinamarinefilkomub && python3 backend/sauvc26_code/odom_bridge.py"';
  jetsonOdomProcess = spawn('ssh', ['-o', 'BatchMode=yes', '-o', 'ConnectTimeout=8', `${user}@${host}`, command], {
    windowsHide: true,
  });
  let error = '';
  jetsonOdomProcess.stderr.on('data', (chunk) => { error += chunk.toString(); });
  jetsonOdomProcess.on('error', (spawnError) => { error = spawnError.message; });
  jetsonOdomProcess.on('close', () => { jetsonOdomProcess = null; });

  return await new Promise((resolve) => setTimeout(() => {
    if (error) resolve({ ok: false, error: error.trim() });
    else resolve({ ok: Boolean(jetsonOdomProcess), error: '' });
  }, 900));
});

ipcMain.handle('jetson:stop-odom-bridge', () => {
  if (jetsonOdomProcess && !jetsonOdomProcess.killed) jetsonOdomProcess.kill();
  jetsonOdomProcess = null;
  return { ok: true };
});

let jetsonCameraProcess = null;

ipcMain.handle('jetson:camera-status', () => ({ running: Boolean(jetsonCameraProcess && !jetsonCameraProcess.killed) }));

ipcMain.handle('jetson:start-camera', async (_, { host, user, device }) => {
  if (!validateSshTarget(host, user)) return { ok: false, error: 'Invalid Jetson host or username.' };
  if (device && !/^\/dev\/[a-zA-Z0-9_-]+$/.test(device)) return { ok: false, error: 'Invalid camera device path.' };
  if (jetsonCameraProcess && !jetsonCameraProcess.killed) return { ok: true, alreadyRunning: true };

  const command = `cd ~/digitaltwin && bash backend/start_camera.sh ${device || '/dev/video0'}`;
  jetsonCameraProcess = spawn('ssh', ['-o', 'BatchMode=yes', '-o', 'ConnectTimeout=8', `${user}@${host}`, command], {
    windowsHide: true,
  });
  let error = '';
  jetsonCameraProcess.stderr.on('data', (chunk) => { error += chunk.toString(); });
  jetsonCameraProcess.on('error', (spawnError) => { error = spawnError.message; });
  jetsonCameraProcess.on('close', () => { jetsonCameraProcess = null; });

  return await new Promise((resolve) => setTimeout(() => {
    if (error) resolve({ ok: false, error: error.trim() });
    else resolve({ ok: Boolean(jetsonCameraProcess), error: '' });
  }, 900));
});

ipcMain.handle('jetson:stop-camera', () => {
  if (jetsonCameraProcess && !jetsonCameraProcess.killed) jetsonCameraProcess.kill();
  jetsonCameraProcess = null;
  return { ok: true };
});

// --- Disconnect All: kill every active SSH child process at once ---
ipcMain.handle('jetson:disconnect-all', () => {
  if (jetsonSshProcess && !jetsonSshProcess.killed) jetsonSshProcess.kill();
  jetsonSshProcess = null;
  if (jetsonOdomProcess && !jetsonOdomProcess.killed) jetsonOdomProcess.kill();
  jetsonOdomProcess = null;
  if (jetsonCameraProcess && !jetsonCameraProcess.killed) jetsonCameraProcess.kill();
  jetsonCameraProcess = null;
  return { ok: true };
});

// --- Shutdown Jetson via SSH ---
ipcMain.handle('jetson:shutdown', async (_, { host, user }) => {
  if (!validateSshTarget(host, user)) return { ok: false, error: 'Invalid Jetson host or username.' };

  // First kill all running processes
  if (jetsonSshProcess && !jetsonSshProcess.killed) jetsonSshProcess.kill();
  jetsonSshProcess = null;
  if (jetsonOdomProcess && !jetsonOdomProcess.killed) jetsonOdomProcess.kill();
  jetsonOdomProcess = null;
  if (jetsonCameraProcess && !jetsonCameraProcess.killed) jetsonCameraProcess.kill();
  jetsonCameraProcess = null;

  return new Promise((resolve) => {
    const proc = spawn('ssh', [
      '-o', 'BatchMode=yes',
      '-o', 'ConnectTimeout=8',
      `${user}@${host}`,
      'sudo /sbin/shutdown -h now || sudo /sbin/poweroff || sudo shutdown -h now',
    ], { windowsHide: true });

    let error = '';
    proc.stderr.on('data', (chunk) => { error += chunk.toString(); });
    proc.on('error', (spawnError) => { error = spawnError.message; });
    proc.on('close', (code) => {
      // code 255 is expected because SSH disconnects when remote shuts down
      if (code === 0 || code === 255) resolve({ ok: true });
      else resolve({ ok: false, error: error.trim() || `Exit code ${code}` });
    });

    // Timeout fallback - shutdown usually cuts the connection quickly
    setTimeout(() => resolve({ ok: true }), 5000);
  });
});

// Helper to quickly check if Vite dev server is running on port 5173
function checkDevServer(url, timeoutMs = 800) {
  return new Promise((resolve) => {
    try {
      const parsed = new URL(url);
      const req = http.request(
        {
          hostname: parsed.hostname || 'localhost',
          port: parsed.port || 5173,
          path: '/',
          method: 'GET',
          timeout: timeoutMs,
        },
        (res) => {
          resolve(res.statusCode >= 200 && res.statusCode < 400);
        }
      );
      req.on('error', () => resolve(false));
      req.on('timeout', () => {
        req.destroy();
        resolve(false);
      });
      req.end();
    } catch {
      resolve(false);
    }
  });
}

async function createWindow() {
  mainWindow = new BrowserWindow({
    width: 1440,
    height: 900,
    minWidth: 1024,
    minHeight: 700,
    backgroundColor: '#0a0e17',
    title: 'BlueROV2 AUV Digital Twin & Telemetry Dashboard',
    autoHideMenuBar: true,
    show: false, // Show gracefully after load
    webPreferences: {
      nodeIntegration: false,
      contextIsolation: true,
      preload: path.join(__dirname, 'preload.cjs'),
      webSecurity: false,
    },
  });

  const showWindow = () => {
    if (mainWindow && !mainWindow.isDestroyed() && !mainWindow.isVisible()) {
      mainWindow.show();
      mainWindow.focus();
    }
  };

  // Register show listeners BEFORE loading the URL/file
  mainWindow.once('ready-to-show', showWindow);
  mainWindow.webContents.once('did-finish-load', showWindow);

  // Safety fallback to guarantee the window is revealed even if load events resolve early
  const fallbackTimer = setTimeout(showWindow, 1500);

  const devUrl = process.env.ELECTRON_START_URL || 'http://localhost:5173';
  const isDevRunning = await checkDevServer(devUrl);
  const distHtmlPath = path.join(__dirname, '../frontend/dist/index.html');

  if (isDevRunning) {
    mainWindow.loadURL(devUrl).catch(() => {
      if (fs.existsSync(distHtmlPath)) {
        mainWindow.loadFile(distHtmlPath);
      }
    });
  } else if (fs.existsSync(distHtmlPath)) {
    mainWindow.loadFile(distHtmlPath);
  } else {
    mainWindow.loadURL(devUrl).catch((err) => {
      console.error('Failed to load URL:', err);
    });
  }

  mainWindow.on('closed', () => {
    clearTimeout(fallbackTimer);
    mainWindow = null;
  });
}

app.whenReady().then(() => {
  createWindow();

  app.on('activate', () => {
    if (BrowserWindow.getAllWindows().length === 0) createWindow();
  });
});

app.on('window-all-closed', () => {
  if (process.platform !== 'darwin') {
    app.quit();
  }
});
