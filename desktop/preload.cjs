const { contextBridge, ipcRenderer } = require('electron');

contextBridge.exposeInMainWorld('jetsonSsh', {
  status: () => ipcRenderer.invoke('jetson:ssh-status'),
  startRosbridge: (host, user) => ipcRenderer.invoke('jetson:start-rosbridge', { host, user }),
  stopRosbridge: () => ipcRenderer.invoke('jetson:stop-rosbridge'),
});