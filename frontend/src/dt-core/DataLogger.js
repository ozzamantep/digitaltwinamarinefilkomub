/**
 * Data Logger & Mission Flight Recorder for Marine Digital Twin
 * 
 * Captures synchronized time-series data of:
 * - Actuator control commands & thruster PWMs
 * - Raw physical sensor measurements (IMU, Depth, DVL, Battery)
 * - Fused EKF state estimates
 * - Digital Twin forward predictions & uncertainty
 * - Environmental state & events
 * 
 * Export formats: Structured JSON & CSV
 */

export class DataLogger {
  constructor(options = {}) {
    this.maxSamples = options.maxSamples || 10000;
    this.recording = false;
    this.logs = [];
    this.sessionStartTime = 0;
  }

  startRecording() {
    this.recording = true;
    this.logs = [];
    this.sessionStartTime = Date.now();
  }

  stopRecording() {
    this.recording = false;
    return this.getSummary();
  }

  /**
   * Record a single time step
   */
  logStep(entry) {
    if (!this.recording) return;

    const sample = {
      t: (Date.now() - this.sessionStartTime) / 1000.0,
      timestamp: Date.now(),
      input: entry.input || { surge: 0, sway: 0, heave: 0, yaw: 0 },
      thrusters: entry.thrusters || [0, 0, 0, 0, 0, 0],
      sensors: entry.sensors || {},
      estimated: entry.estimated || {},
      predicted: entry.predicted || {},
      uncertainty: entry.uncertainty ?? 0.05,
      events: entry.events || [],
    };

    this.logs.push(sample);
    if (this.logs.length > this.maxSamples) {
      this.logs.shift();
    }
  }

  getSummary() {
    return {
      totalSamples: this.logs.length,
      durationSec: this.logs.length > 0 ? this.logs[this.logs.length - 1].t : 0,
      startTime: this.sessionStartTime,
    };
  }

  /**
   * Export recorded mission as formatted JSON string
   */
  exportJSON() {
    return JSON.stringify({
      meta: {
        sessionStartTime: this.sessionStartTime,
        sampleCount: this.logs.length,
        version: '1.0.0',
      },
      data: this.logs,
    }, null, 2);
  }

  /**
   * Export recorded mission as CSV string
   */
  exportCSV() {
    if (this.logs.length === 0) return '';

    const headers = [
      'time_s',
      'cmd_surge', 'cmd_sway', 'cmd_heave', 'cmd_yaw',
      't1', 't2', 't3', 't4', 't5', 't6',
      'est_x', 'est_y', 'est_z', 'est_u', 'est_v', 'est_w',
      'est_roll', 'est_pitch', 'est_yaw',
      'uncertainty'
    ];

    const rows = [headers.join(',')];

    for (const sample of this.logs) {
      const inp = sample.input || {};
      const th = sample.thrusters || [0, 0, 0, 0, 0, 0];
      const est = sample.estimated || {};
      const pos = est.position || {};
      const vel = est.velocity || {};
      const att = est.attitude || {};

      const row = [
        sample.t.toFixed(3),
        (inp.surge || 0).toFixed(2), (inp.sway || 0).toFixed(2), (inp.heave || 0).toFixed(2), (inp.yaw || 0).toFixed(2),
        th[0] || 0, th[1] || 0, th[2] || 0, th[3] || 0, th[4] || 0, th[5] || 0,
        (pos.x || 0).toFixed(3), (pos.y || 0).toFixed(3), (pos.z || 0).toFixed(3),
        (vel.u || 0).toFixed(3), (vel.v || 0).toFixed(3), (vel.w || 0).toFixed(3),
        (att.roll || 0).toFixed(3), (att.pitch || 0).toFixed(3), (att.yaw || 0).toFixed(3),
        (sample.uncertainty || 0).toFixed(3)
      ];

      rows.push(row.join(','));
    }

    return rows.join('\n');
  }

  clear() {
    this.logs = [];
  }
}

export const defaultDataLogger = new DataLogger();
export default defaultDataLogger;
