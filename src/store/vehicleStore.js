import { create } from 'zustand';

const useVehicleStore = create((set, get) => ({
  // Connection
  connectionStatus: 'disconnected', // 'connected' | 'disconnected' | 'demo'
  jetsonIp: '192.168.1.100',
  mode: 'demo', // 'live' | 'demo'

  // Camera View Mode for 3D Viewport
  cameraViewMode: 'orbit', // 'orbit' | 'fpv' | 'chase'
  cameraActive: true,
  cameraFrame: null,

  // Vehicle Type & Metadata
  vehicleName: 'BlueROV2 AUV',
  competition: 'SAUVC 2026',
  armed: true,
  flightMode: 'MANUAL', // 'MANUAL' | 'STABILIZE' | 'ALT_HOLD' | 'AUTO'

  // Manual Control Inputs (Normalized -1 to +1)
  controlInput: {
    surge: 0, // forward (+1) / backward (-1)
    yaw: 0,   // turn right (+1) / turn left (-1)
    heave: 0, // dive (+1) / surface (-1)
    sway: 0,  // strafe right (+1) / left (-1)
  },

  // 6-DOF Underwater Pose
  position: { x: -6, y: 1.1, z: 0 }, // X: -12..12, Y: 0..2.0 (depth), Z: -7.5..7.5
  depth: 0.9, // in meters below surface (0.0m to 2.0m)
  headingRad: 0, // yaw in radians
  orientation: { x: 0, y: 0, z: 0, w: 1 },
  euler: { roll: 0, pitch: 0, yaw: 0 },
  speed: { linear: 0, surge: 0, sway: 0, heave: 0, angular: 0 },

  // Thrusters (T200 Thrusters 1-6 in % output)
  thrusters: [0, 0, 0, 0, 0, 0],

  // Subsea Sensors
  imu: { roll: 0, pitch: 0, yaw: 0, accelX: 0, accelY: 0, accelZ: -9.81 },
  depthSensor: { depth: 0.9, pressure: 110.1, temperature: 26.4 },
  altitudeDVL: 1.1,
  leakDetected: false,
  battery: { level: 92, voltage: 16.2, current: 4.2, temperature: 28.5 },
  lightsIntensity: 80, // % 0-100

  // Active Target & Competition Payload
  activeTarget: 'Manual Pilot Control',
  payloadState: { dropped: false, x: 10.5, y: 1.1, z: 1.5 },
  dropPayload: (x = 10.5, z = 1.5) => set({ payloadState: { dropped: true, x, y: 1.1, z } }),
  resetPayload: () => set({ payloadState: { dropped: false, x: 10.5, y: 1.1, z: 1.5 } }),

  // Flares Status (Red, Blue, Yellow can be knocked down; Orange is inspected)
  flaresFallen: { red: false, blue: false, yellow: false, orange: false },
  knockdownFlare: (color) => set((s) => ({ flaresFallen: { ...s.flaresFallen, [color]: true } })),
  resetFlares: () => set({ flaresFallen: { red: false, blue: false, yellow: false, orange: false } }),

  // Telemetry History
  positionHistory: [],
  speedHistory: [],
  depthHistory: [],
  batteryHistory: [],

  // Jetson Diagnostics
  diagnostics: {
    cpuUsage: 35,
    gpuUsage: 42,
    memoryUsage: 45,
    diskUsage: 35,
    rosNodes: 12,
    topicRate: 50,
    uptime: 0,
    dvlStatus: 'LOCKED',
    errors: [],
  },

  // Actions
  setConnectionStatus: (status) => set({ connectionStatus: status }),
  setJetsonIp: (ip) => set({ jetsonIp: ip }),
  setMode: (mode) => set({ mode }),
  setCameraViewMode: (cameraViewMode) => set({ cameraViewMode }),
  setCameraFrame: (cameraFrame) => set({ cameraFrame, cameraActive: true }),
  setCameraActive: (cameraActive) => set({ cameraActive }),

  setArmed: (armed) => {
    set({ armed });
    if (!armed) {
      // Zero thrusters immediately on disarm
      set({
        thrusters: [0, 0, 0, 0, 0, 0],
        speed: { linear: 0, surge: 0, sway: 0, heave: 0, angular: 0 },
      });
    }
  },
  setFlightMode: (flightMode) => {
    set({
      flightMode,
      activeTarget: flightMode === 'AUTO' ? 'Gate Qualification' : `Pilot Mode (${flightMode})`,
    });
  },
  setLightsIntensity: (intensity) => set({ lightsIntensity: intensity }),

  setControlInput: (input) => {
    const prev = get().controlInput;
    set({ controlInput: { ...prev, ...input } });
  },

  updatePose: ({ position, orientation, euler, depth, speed, headingRad }) => {
    const history = get().positionHistory;
    const newHistory = [...history, { ...position, timestamp: Date.now() }];
    if (newHistory.length > 500) newHistory.shift();

    const dHistory = get().depthHistory;
    const newDHistory = [...dHistory, { depth: depth ?? get().depth, timestamp: Date.now() }];
    if (newDHistory.length > 100) newDHistory.shift();

    const sHistory = get().speedHistory;
    const newSHistory = [...sHistory, { ...(speed ?? get().speed), timestamp: Date.now() }];
    if (newSHistory.length > 100) newSHistory.shift();

    set({
      position: position ?? get().position,
      orientation: orientation ?? get().orientation,
      euler: euler ?? get().euler,
      depth: depth ?? get().depth,
      headingRad: headingRad ?? get().headingRad,
      speed: speed ?? get().speed,
      positionHistory: newHistory,
      depthHistory: newDHistory,
      speedHistory: newSHistory,
    });
  },

  updateThrusters: (thrusters) => set({ thrusters }),
  updateIMU: (imu) => set({ imu, euler: { roll: imu.roll, pitch: imu.pitch, yaw: imu.yaw } }),
  updateDepthSensor: (depthSensor) => set({ depthSensor, depth: depthSensor.depth }),
  updateDVL: (altitudeDVL) => set({ altitudeDVL }),
  setLeakDetected: (leakDetected) => set({ leakDetected }),

  updateBattery: (bat) => {
    const history = get().batteryHistory;
    const newHistory = [...history, { ...bat, timestamp: Date.now() }];
    if (newHistory.length > 100) newHistory.shift();
    set({ battery: bat, batteryHistory: newHistory });
  },

  updateDiagnostics: (diag) => set({ diagnostics: { ...get().diagnostics, ...diag } }),
}));

export default useVehicleStore;
