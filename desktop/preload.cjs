const { contextBridge, ipcRenderer } = require('electron');

contextBridge.exposeInMainWorld('jetsonSsh', {
  status: () => ipcRenderer.invoke('jetson:ssh-status'),
  startRosbridge: (host, user) => ipcRenderer.invoke('jetson:start-rosbridge', { host, user }),
  stopRosbridge: () => ipcRenderer.invoke('jetson:stop-rosbridge'),
  odomStatus: () => ipcRenderer.invoke('jetson:odom-status'),
  startOdomBridge: (host, user) => ipcRenderer.invoke('jetson:start-odom-bridge', { host, user }),
  stopOdomBridge: () => ipcRenderer.invoke('jetson:stop-odom-bridge'),
  cameraStatus: () => ipcRenderer.invoke('jetson:camera-status'),
  startCamera: (host, user, device) => ipcRenderer.invoke('jetson:start-camera', { host, user, device }),
  stopCamera: () => ipcRenderer.invoke('jetson:stop-camera'),
  monitorStatus: () => ipcRenderer.invoke('jetson:monitor-status'),
  startMonitor: (host, user) => ipcRenderer.invoke('jetson:start-monitor', { host, user }),
  stopMonitor: () => ipcRenderer.invoke('jetson:stop-monitor'),
  disconnectAll: () => ipcRenderer.invoke('jetson:disconnect-all'),
  shutdown: (host, user) => ipcRenderer.invoke('jetson:shutdown', { host, user }),
});