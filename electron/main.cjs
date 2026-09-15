const { app, BrowserWindow } = require('electron');
const path = require('path');
const fs = require('fs');
const http = require('http');

// 1. Performance and GPU optimization flags for smooth 60fps WebGL / Three.js
app.commandLine.appendSwitch('ignore-gpu-blocklist');
app.commandLine.appendSwitch('enable-gpu-rasterization');
app.commandLine.appendSwitch('enable-zero-copy');

let mainWindow = null;

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
  const distHtmlPath = path.join(__dirname, '../dist/index.html');

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
