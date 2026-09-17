/**
 * Synchronization Manager for Physical-Virtual Digital Twin Coupling
 * 
 * Tracks temporal synchronization, latency, clock drift, and jitter between
 * the real-world Jetson/ROS2 node and the virtual Digital Twin.
 * 
 * Key Metrics:
 * - Δt_sync: Time discrepancy between physical state timestamp and DT clock
 * - Network Latency: Round-trip and one-way communication delay
 * - Processing Latency: EKF estimation + Physics step computation time
 * - Total Pipeline Latency: T_total = T_network + T_estimation + T_DT
 * - Sync Health: SYNCHRONIZED | DEGRADED | DESYNCHRONIZED
 */

export const SYNC_STATUS = {
  SYNCHRONIZED: 'SYNCHRONIZED', // < 50ms latency, high packet rate
  DEGRADED: 'DEGRADED',         // 50ms - 200ms latency or minor packet loss
  DESYNCHRONIZED: 'DESYNCED',   // > 200ms latency or stream stalled
};

export class SyncManager {
  constructor(options = {}) {
    this.maxHistory = options.maxHistory || 100;

    // Timing stats (in ms)
    this.networkLatency = 12.0;
    this.estimationLatency = 1.5;
    this.simulationLatency = 2.0;
    this.totalLatency = 15.5;

    this.deltaTSync = 0.0; // ms
    this.packetRateHz = 20.0;
    this.jitterMs = 1.2;

    this.clockOffset = 0.0; // Estimated physical clock - local clock offset (ms)

    // Packet tracking
    this.lastPacketTimestamp = 0;
    this.lastLocalReceiveTime = 0;
    this.packetsReceived = 0;
    this.droppedPackets = 0;

    this.latencyHistory = [];
    this.status = SYNC_STATUS.SYNCHRONIZED;
  }

  /**
   * Process incoming packet timestamp for clock sync and latency estimation
   * 
   * @param {number} physicalTimestamp - Epoch timestamp from Jetson (ms)
   * @param {number} [localReceiveTime] - Local clock timestamp (ms)
   */
  recordPacket(physicalTimestamp, localReceiveTime = performance.now()) {
    this.packetsReceived++;

    if (this.lastLocalReceiveTime > 0) {
      const dtLocal = localReceiveTime - this.lastLocalReceiveTime;
      if (dtLocal > 0) {
        const instantRate = 1000.0 / dtLocal;
        this.packetRateHz = this.packetRateHz * 0.9 + instantRate * 0.1;
      }

      // Detect missed packets (expected ~50ms for 20 Hz)
      if (dtLocal > 120) {
        const missed = Math.floor(dtLocal / 50) - 1;
        if (missed > 0) this.droppedPackets += missed;
      }
    }

    // Rough one-way delay calculation if clocks are synchronized
    const rawDelay = Math.max(1.0, Date.now() - physicalTimestamp);
    if (rawDelay < 5000) { // Reject extreme clock offsets
      this.networkLatency = this.networkLatency * 0.85 + rawDelay * 0.15;
    }

    this.lastPacketTimestamp = physicalTimestamp;
    this.lastLocalReceiveTime = localReceiveTime;

    this.updateHealth();
  }

  /**
   * Record computation times for internal DT stages
   * @param {number} tEst - State estimation duration (ms)
   * @param {number} tSim - Dynamics simulation duration (ms)
   */
  recordCompute(tEst, tSim) {
    this.estimationLatency = this.estimationLatency * 0.8 + (tEst || 0) * 0.2;
    this.simulationLatency = this.simulationLatency * 0.8 + (tSim || 0) * 0.2;
    this.totalLatency = this.networkLatency + this.estimationLatency + this.simulationLatency;
  }

  /**
   * Evaluate overall synchronization health
   */
  updateHealth() {
    const timeSinceLast = performance.now() - this.lastLocalReceiveTime;

    if (timeSinceLast > 1500) {
      this.status = SYNC_STATUS.DESYNCHRONIZED;
    } else if (this.totalLatency > 150 || this.packetRateHz < 8.0) {
      this.status = SYNC_STATUS.DEGRADED;
    } else {
      this.status = SYNC_STATUS.SYNCHRONIZED;
    }
  }

  /**
   * Get sync metrics for DT state and dashboard
   */
  getMetrics() {
    this.updateHealth();
    return {
      status: this.status,
      networkLatencyMs: Number(this.networkLatency.toFixed(1)),
      estimationLatencyMs: Number(this.estimationLatency.toFixed(2)),
      simulationLatencyMs: Number(this.simulationLatency.toFixed(2)),
      totalLatencyMs: Number(this.totalLatency.toFixed(1)),
      packetRateHz: Number(this.packetRateHz.toFixed(1)),
      packetsReceived: this.packetsReceived,
      droppedPackets: this.droppedPackets,
    };
  }

  reset() {
    this.packetsReceived = 0;
    this.droppedPackets = 0;
    this.lastLocalReceiveTime = 0;
    this.status = SYNC_STATUS.SYNCHRONIZED;
  }
}

export const defaultSyncManager = new SyncManager();
export default defaultSyncManager;
