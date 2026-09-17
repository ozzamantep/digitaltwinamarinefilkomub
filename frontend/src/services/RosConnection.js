import { Ros } from 'roslib';
import useVehicleStore from '../store/vehicleStore';

class RosConnection {
  constructor() {
    this.ros = null;
    this.reconnectTimer = null;
    this.reconnectInterval = 3000;
  }

  connect(jetsonIp = '192.168.1.100', port = 9090) {
    if (this.ros) {
      this.disconnect();
    }

    const url = `ws://${jetsonIp}:${port}`;
    console.log(`[ROS] Connecting to ${url}...`);

    this.ros = new Ros({ url });

    this.ros.on('connection', () => {
      console.log('[ROS] Connected!');
      useVehicleStore.getState().setConnectionStatus('connected');
      if (this.reconnectTimer) {
        clearTimeout(this.reconnectTimer);
        this.reconnectTimer = null;
      }
    });

    this.ros.on('error', (error) => {
      console.error('[ROS] Error:', error);
    });

    this.ros.on('close', () => {
      console.log('[ROS] Connection closed.');
      useVehicleStore.getState().setConnectionStatus('disconnected');
      this.scheduleReconnect(jetsonIp, port);
    });

    return this.ros;
  }

  disconnect() {
    if (this.reconnectTimer) {
      clearTimeout(this.reconnectTimer);
      this.reconnectTimer = null;
    }
    if (this.ros) {
      try {
        this.ros.close();
      } catch (e) {
        // Ignore
      }
      this.ros = null;
    }
    useVehicleStore.getState().setConnectionStatus('disconnected');
  }

  scheduleReconnect(jetsonIp, port) {
    if (!this.reconnectTimer) {
      this.reconnectTimer = setTimeout(() => {
        this.reconnectTimer = null;
        console.log('[ROS] Attempting reconnect...');
        this.connect(jetsonIp, port);
      }, this.reconnectInterval);
    }
  }

  getRos() {
    return this.ros;
  }

  isConnected() {
    return this.ros && this.ros.isConnected;
  }
}

// Singleton
const rosConnection = new RosConnection();
export default rosConnection;
