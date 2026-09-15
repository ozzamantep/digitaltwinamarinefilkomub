import { create } from 'zustand';

const useVehicleStore = create((set, get) => ({
  // Connection
  connectionStatus: 'disconnected', // 'connected' | 'disconnected' | 'demo'
  jetsonIp: '192.168.1.100',
  mode: 'demo', // 'live' | 'demo'
  activePage: 'mission', // 'mission' | 'thruster_test'
  setActivePage: (page) => set({ activePage: page }),

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
  imuDetection: {
    status: 'STABIL',
    accelerationMagnitude: 9.81,
    horizontalAcceleration: 0,
    gravityDeviation: 0,
    motionDetected: false,
    impactDetected: false,
  },
  depthSensor: { depth: 0.9, pressure: 110.1, temperature: 26.4 },
  altitudeDVL: 1.1,
  sonarRanges: { front: 8, rear: 8, left: 8, right: 8 },
  sonarDetections: [],
  safetyInterlocks: { front: false, rear: false, left: false, right: false, floor: false },
  safetySupervisor: { state: 'NORMAL', reasons: [], blocked: { front: false, rear: false, left: false, right: false, floor: false }, emergency: false },
  leakDetected: false,
  battery: { level: 92, voltage: 16.2, current: 4.2, temperature: 28.5 },
  lightsIntensity: 80, // % 0-100

  // Digital Twin Core & Physical-Virtual Shadow Telemetry
  dtHealth: {
    totalScore: 96,
    tier: 'OPTIMAL',
    breakdown: { sync: 30, estimation: 24, physics: 24, actuators: 18 },
    recommendation: 'Digital Twin fully synchronized and physically calibrated.',
  },
  syncMetrics: {
    status: 'SYNCHRONIZED',
    totalLatencyMs: 14.8,
    networkLatencyMs: 11.2,
    estimationLatencyMs: 1.5,
    simulationLatencyMs: 2.1,
    packetRateHz: 20.0,
    packetsReceived: 0,
    droppedPackets: 0,
  },
  estimatedState: {
    position: { x: -11.0, y: 1.2, z: 2.0 },
    velocity: { u: 0, v: 0, w: 0 },
    attitude: { roll: 0, pitch: 0, yaw: 0, rollDeg: 0, pitchDeg: 0, yawDeg: 0 },
  },
  uncertainty: { score: 0.04, level: 'NOMINAL', confidence: 0.96 },
  oodStatus: { isOOD: false, oodScore: 0.45, state: 'IN_DISTRIBUTION' },
  validationMetrics: {
    sampleCount: 120,
    positionRMSE: { x: 0.015, y: 0.018, z: 0.012, total3D: 0.026 },
    velocityRMSE: { total: 0.032 },
    validationGrade: 'A+ (Excellent)',
  },
  fidelityLevel: 2,
  modelVersion: 1,

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
  releaseBall: () => set((s) => {
    if (!s.payloadState.grasped) return s;
    const drum = s.obstacles.drum_red_tgt;
    const distanceToDrum = drum
      ? Math.sqrt((s.position.x - drum.x) ** 2 + (s.position.z - drum.z) ** 2)
      : Infinity;
    const inDrum = distanceToDrum <= 0.55;
    const targetX = inDrum ? drum.x : s.position.x;
    const targetZ = inDrum ? drum.z : s.position.z;

    return {
      gripperState: 'OPEN',
      payloadState: {
        ...s.payloadState,
        loaded: false,
        grasped: false,
        dropped: true,
        inDrum,
        onFloor: !inDrum,
        retrievalActive: !inDrum,
        x: targetX,
        y: inDrum ? 0.20 : 0.08,
        z: targetZ,
      },
      obstacles: {
        ...s.obstacles,
        drum_red_tgt: inDrum && drum ? { ...drum, dropped: true } : drum,
      },
    };
  }),
  dropBallIntoDrum: (x = 10.5, z = 1.5) => set((s) => ({
    gripperState: 'OPEN',
    payloadState: {
      ...s.payloadState,
      loaded: false,
      grasped: false,
      dropped: true,
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

  // Flares Status (Red, Blue, Yellow, Orange can be knocked down if TABRAK is selected)
  flaresFallen: { red: false, blue: false, yellow: false, orange: false },
  
  // Flare Navigation Strategy: 'TABRAK' (Ram/Knockdown) vs 'MENGHINDAR' (Avoid/Bypass)
  flareStrategies: {
    orange_flare: 'MENGHINDAR', // Default SAUVC standard: inspect and avoid
    blue_flare: 'TABRAK',       // Default SAUVC standard: knockdown
    red_flare: 'TABRAK',        // Default SAUVC standard: knockdown
    yellow_flare: 'TABRAK',     // Default SAUVC standard: knockdown
  },

  setFlareStrategy: (flareKey, strategy) => set((s) => ({
    flareStrategies: { ...s.flareStrategies, [flareKey]: strategy },
    // If set to MENGHINDAR, unfall the flare if it was fallen
    flaresFallen: strategy === 'MENGHINDAR' 
      ? { ...s.flaresFallen, [flareKey.replace('_flare', '')]: false }
      : s.flaresFallen,
  })),

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

  // Execution Order & Priority of Obstacles for Autonomous Mission
  obstacleOrder: [
    'orange_flare',
    'blue_flare',
    'red_flare',
    'yellow_flare',
    'gate',
    'drum_red_tgt',
  ],

  // Whether the obstacle is enabled for autonomous execution
  obstacleEnabled: {
    orange_flare: true,
    blue_flare: true,
    red_flare: true,
    yellow_flare: true,
    gate: true,
    drum_red_tgt: true,
  },

  setObstacleOrder: (newOrder) => set({ obstacleOrder: newOrder }),

  moveObstacleOrder: (key, direction) => set((s) => {
    const list = [...(s.obstacleOrder || ['orange_flare', 'blue_flare', 'red_flare', 'yellow_flare', 'gate', 'drum_red_tgt'])];
    const currentIndex = list.indexOf(key);
    if (currentIndex === -1) return s;
    const targetIndex = direction === 'up' ? currentIndex - 1 : currentIndex + 1;
    if (targetIndex < 0 || targetIndex >= list.length) return s;
    const [moved] = list.splice(currentIndex, 1);
    list.splice(targetIndex, 0, moved);
    return { obstacleOrder: list };
  }),

  setObstaclePriority: (key, targetIndex) => set((s) => {
    const list = [...(s.obstacleOrder || ['orange_flare', 'blue_flare', 'red_flare', 'yellow_flare', 'gate', 'drum_red_tgt'])];
    const currentIndex = list.indexOf(key);
    if (currentIndex === -1 || targetIndex < 0 || targetIndex >= list.length) return s;
    const [moved] = list.splice(currentIndex, 1);
    list.splice(targetIndex, 0, moved);
    return { obstacleOrder: list };
  }),

  toggleObstacleEnabled: (key) => set((s) => ({
    obstacleEnabled: {
      ...s.obstacleEnabled,
      [key]: s.obstacleEnabled?.[key] === false ? true : false,
    },
  })),

  applyOrderPreset: (preset) => set((s) => {
    if (preset === 'standard') {
      return {
        obstacleOrder: ['orange_flare', 'blue_flare', 'red_flare', 'yellow_flare', 'gate', 'drum_red_tgt'],
      };
    } else if (preset === 'reverse_flares') {
      return {
        obstacleOrder: ['yellow_flare', 'red_flare', 'blue_flare', 'orange_flare', 'gate', 'drum_red_tgt'],
      };
    } else if (preset === 'tabrak_first') {
      const flares = ['orange_flare', 'blue_flare', 'red_flare', 'yellow_flare'];
      const tabraks = flares.filter((k) => s.flareStrategies[k] === 'TABRAK');
      const menghindars = flares.filter((k) => s.flareStrategies[k] !== 'TABRAK');
      return {
        obstacleOrder: [...tabraks, ...menghindars, 'gate', 'drum_red_tgt'],
      };
    } else if (preset === 'gate_first') {
      return {
        obstacleOrder: ['gate', 'orange_flare', 'blue_flare', 'red_flare', 'yellow_flare', 'drum_red_tgt'],
      };
    }
    return s;
  }),

  // Dynamic Obstacle Map (World 3D Positions & Real-Time Sync)
  // Coordinates are updated live from ROS 2 (/yolo_target_coord, /obstacle_positions) or Arena Configurator
  obstacles: {
    orange_flare: { id: 'flare_orange', name: 'FLARE_ORG', x: -6.0, z: 2.0, y: 0.75, width: 0.35, height: 1.5, color: '#ea580c', detected: false },
    blue_flare:   { id: 'flare_blue',   name: 'FLARE_BLU', x: -2.0, z: 2.2, y: 0.75, width: 0.35, height: 1.5, color: '#0284c7', detected: false, fallen: false },
    red_flare:    { id: 'flare_red',    name: 'FLARE_RED', x: 0.5,  z: 4.0, y: 0.75, width: 0.35, height: 1.5, color: '#dc2626', detected: false, fallen: false },
    yellow_flare: { id: 'flare_yellow', name: 'FLARE_YEL', x: -0.5, z: -4.5, y: 0.75, width: 0.35, height: 1.5, color: '#eab308', detected: false, fallen: false },
    gate:         { id: 'gate',         name: 'SAUVC_GATE',x: 4.0,  z: 0.0, y: 0.85, width: 1.9,  height: 1.6, color: '#f59e0b', detected: false, passed: false },
    drum_red_tgt: { id: 'drum_red_1',   name: 'DRUM_RED_TGT', x: 10.5, z: 1.5, y: 0.25, width: 0.7, height: 0.5, color: '#ef4444', detected: false, dropped: false },
    drum_red_2:   { id: 'drum_red_2',   name: 'DRUM_RED_SIGNAL_2', x: 10.5, z: -1.5, y: 0.25, width: 0.7, height: 0.5, color: '#ef4444', detected: false, signalOnly: true, sensorSource: 'vision' },
    drum_red_3:   { id: 'drum_red_3',   name: 'DRUM_RED_SIGNAL_3', x: 10.5, z: -4.5, y: 0.25, width: 0.7, height: 0.5, color: '#ef4444', detected: false, signalOnly: true, sensorSource: 'vision' },
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
    obstacleOrder: ['orange_flare', 'blue_flare', 'red_flare', 'yellow_flare', 'gate', 'drum_red_tgt'],
    obstacleEnabled: {
      orange_flare: true,
      blue_flare: true,
      red_flare: true,
      yellow_flare: true,
      gate: true,
      drum_red_tgt: true,
    },
    obstacles: {
      orange_flare: { id: 'flare_orange', name: 'FLARE_ORG', x: -6.0, z: 2.0, y: 0.75, width: 0.35, height: 1.5, color: '#ea580c', detected: false },
      blue_flare:   { id: 'flare_blue',   name: 'FLARE_BLU', x: -2.0, z: 2.2, y: 0.75, width: 0.35, height: 1.5, color: '#0284c7', detected: false, fallen: false },
      red_flare:    { id: 'flare_red',    name: 'FLARE_RED', x: 0.5,  z: 4.0, y: 0.75, width: 0.35, height: 1.5, color: '#dc2626', detected: false, fallen: false },
      yellow_flare: { id: 'flare_yellow', name: 'FLARE_YEL', x: -0.5, z: -4.5, y: 0.75, width: 0.35, height: 1.5, color: '#eab308', detected: false, fallen: false },
      gate:         { id: 'gate',         name: 'SAUVC_GATE',x: 4.0,  z: 0.0, y: 0.85, width: 1.9,  height: 1.6, color: '#f59e0b', detected: false, passed: false },
      drum_red_tgt: { id: 'drum_red_1',   name: 'DRUM_RED_TGT', x: 10.5, z: 1.5, y: 0.25, width: 0.7, height: 0.5, color: '#ef4444', detected: false, dropped: false },
      drum_red_2:   { id: 'drum_red_2',   name: 'DRUM_RED_SIGNAL_2', x: 10.5, z: -1.5, y: 0.25, width: 0.7, height: 0.5, color: '#ef4444', detected: false, signalOnly: true, sensorSource: 'vision' },
      drum_red_3:   { id: 'drum_red_3',   name: 'DRUM_RED_SIGNAL_3', x: 10.5, z: -4.5, y: 0.25, width: 0.7, height: 0.5, color: '#ef4444', detected: false, signalOnly: true, sensorSource: 'vision' },
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

  cameraViewMode: 'tpp', // 'tpp' | 'chase' | 'fpv' | 'orbit'
  cameraResetTrigger: 0,
  triggerCameraReset: () => set((s) => ({ cameraResetTrigger: s.cameraResetTrigger + 1 })),
  gamepadCameraOrbit: { deltaAzimuth: 0, deltaElevation: 0 },
  setGamepadCameraOrbit: (deltaAzimuth, deltaElevation) => set({ gamepadCameraOrbit: { deltaAzimuth, deltaElevation } }),

  cycleCameraViewMode: (direction = 1) => {
    const modes = ['tpp', 'chase', 'fpv', 'orbit'];
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
    const now = Date.now();
    const history = get().positionHistory;
    const lastPositionSample = history[history.length - 1];
    const shouldSamplePosition = position && (
      !lastPositionSample ||
      now - lastPositionSample.timestamp >= 200
    );
    const newHistory = shouldSamplePosition
      ? [...history.slice(-499), { ...position, timestamp: now }]
      : history;

    const dHistory = get().depthHistory;
    const lastDepthSample = dHistory[dHistory.length - 1];
    const shouldSampleDepth = !lastDepthSample || now - lastDepthSample.timestamp >= 200;
    const newDHistory = shouldSampleDepth
      ? [...dHistory.slice(-99), { depth: depth ?? get().depth, timestamp: now }]
      : dHistory;

    const sHistory = get().speedHistory;
    const lastSpeedSample = sHistory[sHistory.length - 1];
    const shouldSampleSpeed = !lastSpeedSample || now - lastSpeedSample.timestamp >= 200;
    const newSHistory = shouldSampleSpeed
      ? [...sHistory.slice(-99), { ...(speed ?? get().speed), timestamp: now }]
      : sHistory;

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
  updateIMU: (imu) => {
    const accelX = Number(imu.accelX) || 0;
    const accelY = Number(imu.accelY) || 0;
    const accelZ = Number(imu.accelZ) || 0;
    const accelerationMagnitude = Math.sqrt(accelX ** 2 + accelY ** 2 + accelZ ** 2);
    const horizontalAcceleration = Math.sqrt(accelX ** 2 + accelY ** 2);
    const gravityDeviation = Math.abs(accelerationMagnitude - 9.81);
    const impactDetected = accelerationMagnitude > 18.0 || gravityDeviation > 6.0;
    const motionDetected = impactDetected || horizontalAcceleration > 0.08 || gravityDeviation > 0.12;

    set({
      imu,
      imuDetection: {
        status: impactDetected ? 'IMPACT' : motionDetected ? 'GERAK' : 'STABIL',
        accelerationMagnitude,
        horizontalAcceleration,
        gravityDeviation,
        motionDetected,
        impactDetected,
      },
      euler: { roll: imu.roll, pitch: imu.pitch, yaw: imu.yaw },
    });
  },
  updateDepthSensor: (depthSensor) => set({ depthSensor, depth: depthSensor.depth }),
  updateDVL: (altitudeDVL) => set({ altitudeDVL }),
  updateSonarRange: (direction, range) => set((state) => ({
    sonarRanges: { ...state.sonarRanges, [direction]: Math.max(0, Number(range) || 0) },
  })),
  updateSonarRanges: (sonarRanges) => set((state) => ({
    sonarRanges: { ...state.sonarRanges, ...sonarRanges },
  })),
  updateSonarDetections: (sonarDetections) => set({ sonarDetections }),
  updateSafetyInterlocks: (safetyInterlocks) => set({ safetyInterlocks }),
  setSafetySupervisor: (safetySupervisor) => set({ safetySupervisor }),
  setLeakDetected: (leakDetected) => set({ leakDetected }),

  updateBattery: (bat) => {
    const history = get().batteryHistory;
    const newHistory = [...history, { ...bat, timestamp: Date.now() }];
    if (newHistory.length > 100) newHistory.shift();
    set({ battery: bat, batteryHistory: newHistory });
  },

  updateDiagnostics: (diag) => set({ diagnostics: { ...get().diagnostics, ...diag } }),

  setDtHealth: (dtHealth) => set({ dtHealth }),
  setSyncMetrics: (syncMetrics) => set({ syncMetrics }),
  setEstimatedState: (estimatedState) => set({ estimatedState }),
  setUncertainty: (uncertainty) => set({ uncertainty }),
  setOODStatus: (oodStatus) => set({ oodStatus }),
  setValidationMetrics: (validationMetrics) => set({ validationMetrics }),
  setFidelityLevel: (fidelityLevel) => set({ fidelityLevel }),
}));

export default useVehicleStore;
