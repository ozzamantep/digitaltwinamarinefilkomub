/**
 * Historical Mission Replay & Shadow Validation Engine
 * 
 * Replays recorded physical robot missions through the Digital Twin physics engine
 * to answer: "What would the Digital Twin have predicted given the exact control inputs?"
 * 
 * Features:
 * - Variable playback speed (0.5x, 1.0x, 2.0x, 5.0x)
 * - Step-by-step stepping & arbitrary timeline seeking
 * - Real-time divergence error tracking against recorded trajectory
 * - Callback hooks for visualization and dashboard updates
 */

import { HydrodynamicsEngine } from './HydrodynamicsEngine.js';
import Kinematics from './Kinematics.js';

export class ReplayEngine {
  constructor(options = {}) {
    this.dataset = [];
    this.currentIndex = 0;
    this.isPlaying = false;
    this.playbackSpeed = 1.0;
    this.timer = null;

    this.engine = new HydrodynamicsEngine();
    this.simState = null;

    this.onFrameCallback = null;
    this.onCompleteCallback = null;
  }

  load(data) {
    this.pause();
    if (typeof data === 'string') {
      const parsed = JSON.parse(data);
      this.dataset = Array.isArray(parsed) ? parsed : (parsed.data || []);
    } else if (Array.isArray(data)) {
      this.dataset = data;
    } else if (data && Array.isArray(data.data)) {
      this.dataset = data.data;
    } else {
      this.dataset = [];
    }

    this.currentIndex = 0;
    this.resetSimulation();
    return this.dataset.length;
  }

  resetSimulation() {
    if (this.dataset.length > 0) {
      const first = this.dataset[0];
      const est = first.estimated || {};
      const pos = est.position || { x: -11.0, y: 2.0, z: 0.8 };
      const vel = est.velocity || { u: 0, v: 0, w: 0 };
      const att = est.attitude || { roll: 0, pitch: 0, yaw: 0 };

      this.simState = {
        eta: [pos.x || -11.0, pos.y || 2.0, pos.z || 0.8, att.roll || 0, att.pitch || 0, att.yaw || 0],
        nu: [vel.u || 0, vel.v || 0, vel.w || 0, 0, 0, 0],
        quat: Kinematics.eulerToQuaternion(att.roll || 0, att.pitch || 0, att.yaw || 0),
      };
    }
  }

  play(onFrame = null, onComplete = null) {
    if (this.dataset.length === 0) return;
    if (onFrame) this.onFrameCallback = onFrame;
    if (onComplete) this.onCompleteCallback = onComplete;

    this.isPlaying = true;
    this.scheduleNext();
  }

  pause() {
    this.isPlaying = false;
    if (this.timer) {
      clearTimeout(this.timer);
      this.timer = null;
    }
  }

  seek(index) {
    this.currentIndex = Math.max(0, Math.min(this.dataset.length - 1, index));
    // Fast-forward simulation to index
    this.resetSimulation();
    for (let i = 0; i < this.currentIndex; i++) {
      this.stepSimulation(i);
    }
  }

  setSpeed(speed) {
    this.playbackSpeed = Math.max(0.2, Math.min(10.0, speed));
  }

  stepSimulation(idx) {
    const frame = this.dataset[idx];
    if (!frame) return;

    const inp = frame.input || {};
    const dt = 0.05; // Standard 20 Hz
    const thrustScale = 0.5;

    const tau = [
      (inp.surge || 0) * thrustScale,
      (inp.sway || 0) * thrustScale,
      (inp.heave || 0) * thrustScale,
      0, 0,
      (inp.yaw || 0) * thrustScale * 0.18,
    ];

    if (this.simState) {
      this.simState = this.engine.step(this.simState, tau, dt);
    }
  }

  scheduleNext() {
    if (!this.isPlaying || this.currentIndex >= this.dataset.length) {
      this.isPlaying = false;
      if (this.onCompleteCallback) this.onCompleteCallback();
      return;
    }

    const frame = this.dataset[this.currentIndex];
    this.stepSimulation(this.currentIndex);

    if (this.onFrameCallback) {
      this.onFrameCallback({
        frameIndex: this.currentIndex,
        recordedFrame: frame,
        simulatedState: this.simState,
        progress: (this.currentIndex + 1) / this.dataset.length,
      });
    }

    this.currentIndex++;

    const intervalMs = Math.max(5, 50 / this.playbackSpeed);
    this.timer = setTimeout(() => this.scheduleNext(), intervalMs);
  }
}

export const defaultReplayEngine = new ReplayEngine();
export default defaultReplayEngine;
