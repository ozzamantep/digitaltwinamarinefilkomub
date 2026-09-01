const { app, BrowserWindow } = require('electron');
const path = require('path');
const os = require('os');
const fs = require('fs');

// 1. Configure dedicated writable cache directory to prevent Windows Access Denied warnings
const customUserData = path.join(os.tmpdir(), 'auv_digitaltwin_desktop_cache');
try {
  if (!fs.existsSync(customUserData)) {
    fs.mkdirSync(customUserData, { recursive: true });
  }
  app.setPath('userData', customUserData);
} catch (e) {
  // Fallback gracefully
}

// 2. Performance and Cache flags for smooth WebGL / Three.js
app.commandLine.appendSwitch('disable-gpu-shader-disk-cache');
app.commandLine.appendSwitch('ignore-gpu-blocklist');
app.commandLine.appendSwitch('enable-gpu-rasterization');
app.commandLine.appendSwitch('enable-zero-copy');

let mainWindow;

function createWindow() {
  mainWindow = new BrowserWindow({
    width: 1440,
    height: 900,
    minWidth: 1024,
    minHeight: 700,
    backgroundColor: '#0a0e17',
    title: 'BlueROV2 AUV Digital Twin & Telemetry Dashboard',
    autoHideMenuBar: true,
    webPreferences: {
      nodeIntegration: false,
      contextIsolation: true,
      webSecurity: false,
    },
  });

  // Load local dev server or production build
  const startUrl = process.env.ELECTRON_START_URL || 'http://localhost:5173';
  
  mainWindow.loadURL(startUrl).catch(() => {
    // If dev server not yet ready, load built dist
    mainWindow.loadFile(path.join(__dirname, '../dist/index.html'));
  });

  mainWindow.on('closed', () => {
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
  if (process.platform !== 'darwin') app.quit();
});
