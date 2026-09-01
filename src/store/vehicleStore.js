import { create } from 'zustand';

const useVehicleStore = create((set, get) => ({
  // Connection
  connectionStatus: 'disconnected', // 'connected' | 'disconnected' | 'demo'
  jetsonIp: '192.168.1.100',
  mode: 'demo', // 'live' | 'demo'

  // Camera View Mode for 3D Viewport
  cameraViewMode: 'orbit', // 'orbit' | 'fpv' | 'chase'
  cameraResetTrigger: 0,
  triggerCameraReset: () => set((s) => ({ cameraResetTrigger: s.cameraResetTrigger + 1 })),
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

  // Active Target & Competition Payload + Robotic Gripper
  activeTarget: 'Manual Pilot Control',
  gripperState: 'HOLDING', // 'OPEN' | 'CLOSED' | 'GRASPING' | 'HOLDING' (Holds ball from start)
  setGripperState: (gripperState) => set({ gripperState }),
  toggleGripper: () => set((s) => ({
    gripperState: s.gripperState === 'OPEN' ? 'CLOSED' : 'OPEN',
    payloadState: s.payloadState.grasped && s.gripperState !== 'OPEN'
      ? { ...s.payloadState, grasped: false, dropped: true, onFloor: true, x: s.position.x + 0.2, z: s.position.z }
      : s.payloadState,
  })),

  payloadState: {
    loaded: true,
    dropped: false,
    onFloor: false,
    grasped: true, // Ball is held by the gripper right from the start!
    inDrum: false,
    retrievalActive: false,
    x: 10.5,
    y: 0.22,
    z: 1.5,
  },
  dropPayloadOnFloor: (x = 10.6, z = 1.2) => set({
    gripperState: 'OPEN',
    payloadState: {
      loaded: false,
      dropped: true,
      onFloor: true,
      grasped: false,
      inDrum: false,
      retrievalActive: true,
      x,
      y: 0.08,
      z,
    },
    obstacles: {
      ...get().obstacles,
      ball_red_floor: { id: 'ball_red', name: 'BALL_RED', x, z, y: 0.15, width: 0.2, height: 0.2, color: '#ef4444', detected: true },
    },
  }),
  graspBallWithGripper: () => set((s) => ({
    gripperState: 'HOLDING',
    payloadState: {
      ...s.payloadState,
      onFloor: false,
      grasped: true,
    },
    obstacles: {
      ...s.obstacles,
      ball_red_floor: undefined,
    },
  })),
  dropBallIntoDrum: (x = 10.5, z = 1.5) => set((s) => ({
    gripperState: 'OPEN',
    payloadState: {
      ...s.payloadState,
      grasped: false,
      inDrum: true,
      retrievalActive: false,
      x,
      y: 0.20,
      z,
    },
    obstacles: {
      ...s.obstacles,
      drum_red_tgt: s.obstacles.drum_red_tgt ? { ...s.obstacles.drum_red_tgt, dropped: true } : s.obstacles.drum_red_tgt,
    },
  })),
  dropPayload: (x = 10.5, z = 1.5) => set((s) => ({
    gripperState: 'OPEN',
    payloadState: { ...s.payloadState, loaded: false, dropped: true, inDrum: true, grasped: false, x, y: 0.20, z },
    obstacles: {
      ...s.obstacles,
      drum_red_tgt: s.obstacles.drum_red_tgt ? { ...s.obstacles.drum_red_tgt, dropped: true } : s.obstacles.drum_red_tgt,
    },
  })),
  resetPayload: () => set((s) => ({
    gripperState: 'HOLDING',
    payloadState: {
      loaded: true,
      dropped: false,
      onFloor: false,
      grasped: true, // Clamped from the start
      inDrum: false,
      retrievalActive: false,
      x: 10.5,
      y: 0.22,
      z: 1.5,
    },
    obstacles: {
      ...s.obstacles,
      ball_red_floor: undefined,
      drum_red_tgt: s.obstacles.drum_red_tgt ? { ...s.obstacles.drum_red_tgt, dropped: false } : s.obstacles.drum_red_tgt,
    },
  })),

  // Flares Status (Red, Blue, Yellow can be knocked down; Orange is inspected)
  flaresFallen: { red: false, blue: false, yellow: false, orange: false },
  knockdownFlare: (color) => set((s) => ({
    flaresFallen: { ...s.flaresFallen, [color]: true },
    obstacles: {
      ...s.obstacles,
      [`${color}_flare`]: s.obstacles[`${color}_flare`]
        ? { ...s.obstacles[`${color}_flare`], fallen: true }
        : undefined,
    },
  })),
  resetFlares: () => set((s) => ({
    flaresFallen: { red: false, blue: false, yellow: false, orange: false },
  })),

  // Dynamic Obstacle Map (World 3D Positions & Real-Time Sync)
  // Coordinates are updated live from ROS 2 (/yolo_target_coord, /obstacle_positions) or Arena Configurator
  obstacles: {
    orange_flare: { id: 'flare_orange', name: 'FLARE_ORG', x: -6.0, z: 2.0, y: 0.75, width: 0.35, height: 1.5, color: '#ea580c', detected: false },
    blue_flare:   { id: 'flare_blue',   name: 'FLARE_BLU', x: -2.0, z: 2.2, y: 0.75, width: 0.35, height: 1.5, color: '#0284c7', detected: false, fallen: false },
    red_flare:    { id: 'flare_red',    name: 'FLARE_RED', x: 0.5,  z: 4.0, y: 0.75, width: 0.35, height: 1.5, color: '#dc2626', detected: false, fallen: false },
    yellow_flare: { id: 'flare_yellow', name: 'FLARE_YEL', x: -0.5, z: -4.5, y: 0.75, width: 0.35, height: 1.5, color: '#eab308', detected: false, fallen: false },
    gate:         { id: 'gate',         name: 'SAUVC_GATE',x: 4.0,  z: 0.0, y: 0.85, width: 1.9,  height: 1.6, color: '#f59e0b', detected: false, passed: false },
    drum_red_tgt: { id: 'drum_red_1',   name: 'DRUM_RED_TGT', x: 10.5, z: 1.5, y: 0.25, width: 0.7, height: 0.5, color: '#ef4444', detected: false, dropped: false },
    drum_blue:    { id: 'drum_blue',    name: 'DRUM_BLU',  x: 10.5, z: 4.5, y: 0.25, width: 0.7, height: 0.5, color: '#0284c7', detected: false },
  },

  setObstaclePos: (key, x, z) => set((s) => ({
    obstacles: {
      ...s.obstacles,
      [key]: s.obstacles[key] ? { ...s.obstacles[key], x: parseFloat(x), z: parseFloat(z) } : s.obstacles[key],
    },
  })),

  setObstacleDetected: (key, detected) => set((s) => ({
    obstacles: {
      ...s.obstacles,
      [key]: s.obstacles[key] ? { ...s.obstacles[key], detected } : s.obstacles[key],
    },
  })),

  resetObstacles: () => set({
    obstacles: {
      orange_flare: { id: 'flare_orange', name: 'FLARE_ORG', x: -6.0, z: 2.0, y: 0.75, width: 0.35, height: 1.5, color: '#ea580c', detected: false },
      blue_flare:   { id: 'flare_blue',   name: 'FLARE_BLU', x: -2.0, z: 2.2, y: 0.75, width: 0.35, height: 1.5, color: '#0284c7', detected: false, fallen: false },
      red_flare:    { id: 'flare_red',    name: 'FLARE_RED', x: 0.5,  z: 4.0, y: 0.75, width: 0.35, height: 1.5, color: '#dc2626', detected: false, fallen: false },
      yellow_flare: { id: 'flare_yellow', name: 'FLARE_YEL', x: -0.5, z: -4.5, y: 0.75, width: 0.35, height: 1.5, color: '#eab308', detected: false, fallen: false },
      gate:         { id: 'gate',         name: 'SAUVC_GATE',x: 4.0,  z: 0.0, y: 0.85, width: 1.9,  height: 1.6, color: '#f59e0b', detected: false, passed: false },
      drum_red_tgt: { id: 'drum_red_1',   name: 'DRUM_RED_TGT', x: 10.5, z: 1.5, y: 0.25, width: 0.7, height: 0.5, color: '#ef4444', detected: false, dropped: false },
      drum_blue:    { id: 'drum_blue',    name: 'DRUM_BLU',  x: 10.5, z: 4.5, y: 0.25, width: 0.7, height: 0.5, color: '#0284c7', detected: false },
    },
    flaresFallen: { red: false, blue: false, yellow: false, orange: false },
  }),

  applyPresetLayout: (presetName) => {
    if (presetName === 'offset_layout') {
      // Challenging shifted layout
      set({
        obstacles: {
          orange_flare: { id: 'flare_orange', name: 'FLARE_ORG', x: -6.5, z: 2.5, y: 0.75, width: 0.35, height: 1.5, color: '#ea580c', detected: false },
          blue_flare:   { id: 'flare_blue',   name: 'FLARE_BLU', x: -2.5, z: 1.5, y: 0.75, width: 0.35, height: 1.5, color: '#0284c7', detected: false, fallen: false },
          red_flare:    { id: 'flare_red',    name: 'FLARE_RED', x: 1.0,  z: 3.5, y: 0.75, width: 0.35, height: 1.5, color: '#dc2626', detected: false, fallen: false },
          yellow_flare: { id: 'flare_yellow', name: 'FLARE_YEL', x: -1.0, z: -4.0, y: 0.75, width: 0.35, height: 1.5, color: '#eab308', detected: false, fallen: false },
          gate:         { id: 'gate',         name: 'SAUVC_GATE',x: 4.5,  z: 0.5, y: 0.85, width: 1.9,  height: 1.6, color: '#f59e0b', detected: false, passed: false },
          drum_red_tgt: { id: 'drum_red_1',   name: 'DRUM_RED_TGT', x: 10.0, z: 2.0, y: 0.25, width: 0.7, height: 0.5, color: '#ef4444', detected: false, dropped: false },
          drum_blue:    { id: 'drum_blue',    name: 'DRUM_BLU',  x: 10.0, z: 5.0, y: 0.25, width: 0.7, height: 0.5, color: '#0284c7', detected: false },
        },
      });
    } else {
      get().resetObstacles();
    }
  },

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

  cameraViewMode: 'orbit', // 'fpv' | 'chase' | 'orbit'
  cameraResetTrigger: 0,
  triggerCameraReset: () => set((s) => ({ cameraResetTrigger: s.cameraResetTrigger + 1 })),
  gamepadCameraOrbit: { deltaAzimuth: 0, deltaElevation: 0 },
  setGamepadCameraOrbit: (deltaAzimuth, deltaElevation) => set({ gamepadCameraOrbit: { deltaAzimuth, deltaElevation } }),

  cycleCameraViewMode: (direction = 1) => {
    const modes = ['fpv', 'chase', 'orbit'];
    const current = get().cameraViewMode;
    const currentIdx = modes.indexOf(current);
    const nextIdx = (currentIdx + direction + modes.length) % modes.length;
    set({ cameraViewMode: modes[nextIdx] });
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
