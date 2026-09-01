import * as THREE from 'three';
import useVehicleStore from '../store/vehicleStore';
import auvMotionController from './AUVMotionController';
import subseaCollisionEngine from './SubseaCollisionEngine';
import sysIdEngine from './SystemIdentificationEngine';

class MockRosConnection {
  constructor() {
    this.interval = null;
    this.time = 0;

    this.simX = -11.0;
    this.simZ = 2.0;
    this.simDepth = 0.8;
    this.simHeading = 0;

    // Qualification State Machine
    this.qualState = 1;
    this.qualPrevState = 1;
    this.qualStateStartTime = 0;
    this.qualUTurnTargetYaw = 0;
    this.qualReturnPass = false;

    // Final Mission State Machine
    this.missionStage = 0;
    this.missionTime = 0;
    this.drumDropTriggered = false;
    this.drumWaitTime = 0;
  }

  start() {
    console.log('[MockROS] Starting SAUVC 2026 Simulation matching official arena layout...');
    const store = useVehicleStore.getState();
    store.setConnectionStatus('demo');
    store.setMode('demo');

    this.time = 0;
    this.simX = store.position?.x ?? -11.0;
    this.simZ = store.position?.z ?? 2.0;
    this.simDepth = store.depth ?? 0.8;
    this.simHeading = store.headingRad ?? 0;
    this.resetQualification();
    this.missionStage = 0;
    this.missionTime = 0;
    this.drumDropTriggered = false;
    this.drumWaitTime = 0;
    auvMotionController.reset();
    sysIdEngine.reset();

    // 20 Hz simulation loop (50ms)
    this.interval = setInterval(() => {
      this.time += 0.05;
      this.stepPhysics(0.05);
    }, 50);
  }

  stop() {
    if (this.interval) {
      clearInterval(this.interval);
      this.interval = null;
    }
    useVehicleStore.getState().setConnectionStatus('disconnected');
  }

  resetQualification() {
    this.simX = -11.0;
    this.simZ = 2.0;
    this.simDepth = 0.15; // Start at pool surface
    this.simHeading = 0.0;
    this.qualState = 1;
    this.qualPrevState = 1;
    this.qualStateStartTime = 0;
    this.qualUTurnTargetYaw = 0;
    this.qualReturnPass = false;
    useVehicleStore.getState().resetPayload();
    useVehicleStore.getState().resetFlares();
    auvMotionController.reset();
    sysIdEngine.reset();
  }

  resetFinal() {
    this.simX = -11.0;
    this.simZ = 2.0;
    this.simDepth = 0.8;
    this.simHeading = 0.0;
    this.missionStage = 0;
    this.missionTime = 0;
    this.drumDropTriggered = false;
    this.drumWaitTime = 0;
    useVehicleStore.getState().resetPayload();
    useVehicleStore.getState().resetFlares();
    auvMotionController.reset();
    sysIdEngine.reset();
  }

  stepPhysics(dt) {
    const store = useVehicleStore.getState();
    const t = this.time;
    const isArmed = store.armed;
    const flightMode = store.flightMode;
    const input = store.controlInput || { surge: 0, sway: 0, yaw: 0, heave: 0 };

    // Guard against NaN
    if (!isFinite(this.simX)) this.simX = -11.0;
    if (!isFinite(this.simZ)) this.simZ = 2.0;
    if (!isFinite(this.simDepth)) this.simDepth = 0.8;
    if (!isFinite(this.simHeading)) this.simHeading = 0.0;

    let targetPitchAngle = 0; // Dynamic nose & camera tilt angle (-35 deg when inspecting drum)

    const currentPose = {
      x: this.simX,
      z: this.simZ,
      depth: this.simDepth,
      heading: this.simHeading,
      roll: ((store.euler?.roll || 0) * Math.PI) / 180,
      pitch: ((store.euler?.pitch || 0) * Math.PI) / 180,
    };

    let ctrl;
    if (!isArmed) {
      ctrl = auvMotionController.update(currentPose, input, flightMode, false, dt);
    } else if (flightMode === 'QUALIFIKASI') {
      // =========================================================================
      // AUTONOMOUS QUALIFICATION STATE MACHINE (from qualification.py)
      // =========================================================================
      const TARGET_DEPTH = 0.75; // Optimal depth (Y = 1.25m): directly through gate center
      const GATE_X = 4.0;
      const GATE_Z = 0.0;

      let cmdSurge = 0;
      let cmdYaw = 0;
      let cmdHeave = 0;
      let statusText = '';

      switch (this.qualState) {
        case 1: // State 1: Dive smoothly to target depth
          statusText = 'State 1/5: Diving to Target Gate Depth 0.75m';
          cmdHeave = Math.max(-0.4, Math.min(0.4, (TARGET_DEPTH - this.simDepth) * 2.0));
          cmdSurge = 0.15; // Slow gentle forward drift while diving
          if (Math.abs(this.simDepth - TARGET_DEPTH) < 0.06) {
            this.qualPrevState = 1;
            this.qualState = 4;
            this.qualStateStartTime = t;
          }
          break;

        case 4: // State 4: Track Gate (YOLO Gate PID Tracking)
          statusText = 'State 2/5: YOLO Gate PID Tracking & Center Align';
          cmdHeave = Math.max(-0.35, Math.min(0.35, (TARGET_DEPTH - this.simDepth) * 2.0));
          cmdSurge = 0.70;

          // Target point is center of the gate (X = 4.0, Z = 0.0)
          const targetHeading = Math.atan2(GATE_Z - this.simZ, GATE_X - this.simX);
          let headingErr = targetHeading - this.simHeading;
          while (headingErr > Math.PI) headingErr -= Math.PI * 2;
          while (headingErr < -Math.PI) headingErr += Math.PI * 2;
          cmdYaw = Math.max(-0.5, Math.min(0.5, headingErr * 2.2));

          if (this.simX >= GATE_X - 0.4) {
            this.qualPrevState = 4;
            this.qualState = 2;
            this.qualStateStartTime = t;
          }
          break;

        case 2: // State 2: Forward through Gate
          cmdHeave = Math.max(-0.35, Math.min(0.35, (TARGET_DEPTH - this.simDepth) * 2.0));
          const elapsed = t - this.qualStateStartTime;

          if (!this.qualReturnPass) {
            statusText = `State 3/5: Passing Forward Through Gate (${elapsed.toFixed(1)}s / 6.0s)`;
            cmdSurge = 0.75;
            // Keep straight trajectory through gate center
            let straightErr = 0.0 - this.simHeading;
            while (straightErr > Math.PI) straightErr -= Math.PI * 2;
            while (straightErr < -Math.PI) straightErr += Math.PI * 2;
            cmdYaw = Math.max(-0.3, Math.min(0.3, straightErr * 1.5));

            if (elapsed >= 6.0 || this.simX >= 7.2) {
              this.qualPrevState = 2;
              this.qualState = 3;
              this.qualStateStartTime = t;
              this.qualUTurnTargetYaw = Math.PI; // Exact 180° facing back
            }
          } else {
            statusText = `State 5/5: Returning Back Through Gate to Start (${elapsed.toFixed(1)}s / 6.0s)`;
            cmdSurge = 0.75;
            // Heading towards start zone (-11.0, 2.0)
            const returnHeading = Math.atan2(2.0 - this.simZ, -11.0 - this.simX);
            let retErr = returnHeading - this.simHeading;
            while (retErr > Math.PI) retErr -= Math.PI * 2;
            while (retErr < -Math.PI) retErr += Math.PI * 2;
            cmdYaw = Math.max(-0.5, Math.min(0.5, retErr * 2.0));

            if (elapsed >= 7.0 || this.simX <= -9.8) {
              this.qualPrevState = 2;
              this.qualState = 5;
              this.qualStateStartTime = t;
            }
          }
          break;

        case 3: // State 3: 180° U-Turn
          statusText = 'State 4/5: Performing 180° U-Turn (Putar Balik)';
          cmdHeave = Math.max(-0.35, Math.min(0.35, (TARGET_DEPTH - this.simDepth) * 2.0));
          cmdSurge = 0.08;

          let uTurnErr = this.qualUTurnTargetYaw - this.simHeading;
          while (uTurnErr > Math.PI) uTurnErr -= Math.PI * 2;
          while (uTurnErr < -Math.PI) uTurnErr += Math.PI * 2;

          if (Math.abs(uTurnErr) < 0.08) {
            this.qualPrevState = 3;
            this.qualState = 2;
            this.qualReturnPass = true;
            this.qualStateStartTime = t;
          } else {
            cmdYaw = uTurnErr > 0 ? 0.7 : -0.7;
          }
          break;

        case 5: // State 5: Surface
          statusText = 'QUALIFICATION COMPLETE! 🏆 Surfacing at Start Zone';
          cmdSurge = 0.05;
          cmdHeave = -0.35; // Ascend smoothly to surface
          break;
      }

      const qualTargetMsg = `[QUAL] ${statusText}`;
      if (store.activeTarget !== qualTargetMsg) {
        useVehicleStore.setState({ activeTarget: qualTargetMsg });
      }

      ctrl = auvMotionController.update(
        currentPose,
        { surge: cmdSurge, sway: 0, yaw: cmdYaw, heave: cmdHeave },
        'MANUAL',
        true,
        dt
      );
    } else if (flightMode === 'FINAL') {
      // =========================================================================
      // AUTONOMOUS SAUVC FINAL MISSION
      // =========================================================================
      this.missionTime += dt;

      const waypoints = [
        { name: '1/8: Inspect Orange Flare (No Collide)', x: -6.0, z: 2.0, targetDepth: 0.8, speed: 0.7, threshold: 0.9 },
        { name: '2/8: Strike & Knockdown Blue Flare', x: -2.0, z: 2.2, targetDepth: 0.8, speed: 0.75, threshold: 0.45, hitFlare: 'blue' },
        { name: '3/8: Strike & Knockdown Red Flare', x: 0.5, z: 4.0, targetDepth: 0.8, speed: 0.75, threshold: 0.45, hitFlare: 'red' },
        { name: '4/8: Strike & Knockdown Yellow Flare', x: -0.5, z: -4.5, targetDepth: 0.8, speed: 0.8, threshold: 0.45, hitFlare: 'yellow' },
        { name: '5/8: Pass Through Gate', x: 4.0, z: 0.0, targetDepth: 0.8, speed: 0.75, threshold: 0.8 },
        { name: '6/8: Search Red Drum (Nose & Camera Tilt Down -35°)', x: 10.5, z: 1.5, targetDepth: 0.95, speed: 0.45, threshold: 0.5 },
        { name: '7/8: Drop Ball Payload into Red Drum', x: 10.5, z: 1.5, targetDepth: 0.95, speed: 0.05, threshold: 0.4 },
        { name: '8/8: Mission Complete! 🏆 (Surfacing)', x: 11.5, z: 0.0, targetDepth: 0.1, speed: 0.2, threshold: 0.5 },
      ];

      const currentWp = waypoints[this.missionStage] || waypoints[waypoints.length - 1];
      const distToWp = Math.sqrt((currentWp.x - this.simX) ** 2 + (currentWp.z - this.simZ) ** 2);

      let cmdSurge = currentWp.speed;

      if (currentWp.hitFlare && distToWp < 0.6) {
        store.knockdownFlare(currentWp.hitFlare);
      }

      if (this.missionStage === 5 || this.missionStage === 6) {
        if (distToWp < 2.5) {
          const tiltFactor = Math.min(1.0, (2.5 - distToWp) / 1.2);
          targetPitchAngle = -35.0 * tiltFactor;
        }

        if (this.missionStage === 6 && distToWp < 0.5) {
          cmdSurge = 0.05;
          if (!this.drumDropTriggered) {
            this.drumDropTriggered = true;
            this.drumWaitTime = t;
            store.dropPayload(10.5, 1.5);
          }

          if (t - this.drumWaitTime > 3.0) {
            this.missionStage = 7;
          }
        }
      }

      if (this.missionStage !== 6 && distToWp < currentWp.threshold && this.missionStage < waypoints.length - 1) {
        this.missionStage++;
      }

      const depthErr = currentWp.targetDepth - this.simDepth;
      const cmdHeave = Math.max(-0.6, Math.min(0.6, depthErr * 1.5));

      const targetHeading = Math.atan2(currentWp.z - this.simZ, currentWp.x - this.simX);
      let headingErr = targetHeading - this.simHeading;
      while (headingErr > Math.PI) headingErr -= Math.PI * 2;
      while (headingErr < -Math.PI) headingErr += Math.PI * 2;
      const cmdYaw = Math.max(-0.85, Math.min(0.85, headingErr * 2.2));

      let finalTarget = `[FINAL] ${currentWp.name}`;
      if (this.missionStage === 6 && this.drumDropTriggered) {
        finalTarget = `[FINAL] 6/8: 🎯 BALL DROPPED INTO RED DRUM!`;
      }

      if (store.activeTarget !== finalTarget) {
        useVehicleStore.setState({ activeTarget: finalTarget });
      }

      ctrl = auvMotionController.update(
        currentPose,
        { surge: cmdSurge, sway: 0, yaw: cmdYaw, heave: cmdHeave },
        'MANUAL',
        true,
        dt
      );
    } else {
      ctrl = auvMotionController.update(currentPose, input, flightMode, true, dt);
    }

    // Process Thruster Dynamics through the Identified Polynomial Model (ARX / ARMAX / OE / BJ)
    const rawSurge = ctrl?.surge || 0;
    const sysIdRes = sysIdEngine.step(input.surge || (rawSurge / 1.3), rawSurge, dt);
    const effectiveSurge = Math.max(-1.35, Math.min(1.35, rawSurge * 0.7 + (sysIdRes.velocity || 0) * 0.3));

    // Advance proposed position & orientation
    this.simHeading += (ctrl?.yaw || 0) * dt;

    let proposedX = this.simX + (Math.cos(this.simHeading) * effectiveSurge - Math.sin(this.simHeading) * (ctrl?.sway || 0)) * dt;
    let proposedZ = this.simZ + (Math.sin(this.simHeading) * effectiveSurge + Math.cos(this.simHeading) * (ctrl?.sway || 0)) * dt;
    let proposedDepth = this.simDepth + (ctrl?.heave || 0) * dt;

    // 3D Physical Obstacle Collision Resolution
    const collision = subseaCollisionEngine.resolveCollision(
      proposedX,
      proposedZ,
      proposedDepth,
      effectiveSurge,
      ctrl?.sway || 0
    );

    this.simX = isFinite(collision.x) ? collision.x : -11.0;
    this.simZ = isFinite(collision.z) ? collision.z : 2.0;
    this.simDepth = isFinite(collision.depth) ? Math.max(0.08, collision.depth) : 0.8;

    if (collision.collided) {
      this.simX += (collision.recoilX || 0) * dt * 2.0;
      this.simZ += (collision.recoilZ || 0) * dt * 2.0;
    }

    // Three.js coordinates (Y is Up: Y = 2.0 - depth)
    const y3D = 2.0 - this.simDepth;
    const yawDeg = (((this.simHeading * 180) / Math.PI) % 360 + 360) % 360;

    // Dynamic Nose-Down Pitch & Camera Tilt (-35 deg when inspecting bucket)
    const pitchDeg = (ctrl?.pitch || 0) + targetPitchAngle + (isArmed ? Math.sin(t * 1.5) * 0.4 : Math.sin(t * 0.5) * 0.8);
    const rollDeg = (ctrl?.roll || 0) + (isArmed ? Math.sin(t * 1.2) * 0.4 : Math.sin(t * 0.4) * 0.6);

    // Three.js 6-DOF Quaternion Orientation
    const eulerThree = new THREE.Euler(
      ((rollDeg || 0) * Math.PI) / 180,
      -this.simHeading,
      ((pitchDeg || 0) * Math.PI) / 180,
      'YXZ'
    );
    const quat = new THREE.Quaternion().setFromEuler(eulerThree);

    // Update Pose in Store with strict NaN guards
    store.updatePose({
      position: { x: this.simX, y: y3D, z: this.simZ },
      orientation: {
        x: isFinite(quat.x) ? quat.x : 0,
        y: isFinite(quat.y) ? quat.y : 0,
        z: isFinite(quat.z) ? quat.z : 0,
        w: isFinite(quat.w) ? quat.w : 1,
      },
      euler: { roll: rollDeg || 0, pitch: pitchDeg || 0, yaw: yawDeg || 0 },
      depth: this.simDepth,
      headingRad: this.simHeading,
      speed: {
        linear: Math.sqrt(effectiveSurge ** 2 + ((ctrl?.sway || 0)) ** 2),
        surge: effectiveSurge,
        sway: ctrl?.sway || 0,
        heave: ctrl?.heave || 0,
        angular: Math.abs(ctrl?.yaw || 0),
      },
    });

    store.updateThrusters(ctrl?.thrusters || [0, 0, 0, 0, 0, 0]);

    // Dynamic Hydrodynamic Water Pressure & Subsea Thermal Gradient Physics
    const vTotal = Math.sqrt(effectiveSurge ** 2 + ((ctrl?.sway || 0)) ** 2 + ((ctrl?.heave || 0)) ** 2);
    const th = ctrl?.thrusters || [0, 0, 0, 0, 0, 0];
    const totalThrusterLoad =
      (Math.abs(th[0]) + Math.abs(th[1]) + Math.abs(th[2]) + Math.abs(th[3]) + Math.abs(th[4]) + Math.abs(th[5])) / 600;

    // 1. Hydrostatic pressure: P_atm (101.325 kPa) + rho * g * depth
    const pAtm = 101.325 + Math.sin(t * 0.15) * 0.03;
    const pHydrostatic = this.simDepth * 9.80665;
    // 2. Dynamic Bernoulli Ram Pressure from forward/heave velocity past sensor
    const pDynamic = 0.5 * (vTotal ** 2) * 1.0; 
    // 3. Realistic water turbulence & wave ripple fluctuations
    const pTurbulence = Math.sin(t * 3.8 + this.simX * 0.5) * 0.08 * (1 + totalThrusterLoad * 1.5) + (Math.sin(t * 11.2) * 0.03);
    const dynamicWaterPressure = pAtm + pHydrostatic + pDynamic + pTurbulence;

    // Subsea Temperature: surface ~26.4°C, deeper is cooler, motor heat dissipation & high-res ADC noise
    const tempSurface = 26.4;
    const tempDepthGradient = -0.32 * this.simDepth;
    const tempThruster = totalThrusterLoad * 0.28;
    const tempNoise = Math.sin(t * 0.7) * 0.04 + Math.sin(t * 2.3) * 0.02;
    const dynamicWaterTemp = tempSurface + tempDepthGradient + tempThruster + tempNoise;

    // Sensor Telemetry Update
    store.updateDepthSensor({
      depth: this.simDepth,
      pressure: dynamicWaterPressure,
      temperature: dynamicWaterTemp,
    });

    // DVL with subtle acoustic ranging micro-variance
    const dvlNoise = (Math.sin(t * 5.0) * 0.004) + (Math.cos(t * 8.7) * 0.003);
    store.updateDVL(Math.max(0.05, y3D + dvlNoise));

    store.updateIMU({
      roll: rollDeg || 0,
      pitch: pitchDeg || 0,
      yaw: yawDeg || 0,
      accelX: effectiveSurge * 0.25 + (Math.random() - 0.5) * 0.02,
      accelY: (ctrl?.sway || 0) * 0.25 + (Math.random() - 0.5) * 0.02,
      accelZ: -9.81 + (ctrl?.heave || 0) * 0.3,
    });

    // 4S LiPo Battery smooth real-time telemetry
    const bat = store.battery;
    const currentDraw = isArmed 
      ? 1.4 + totalThrusterLoad * 16.5 + (Math.sin(t * 4) * 0.15)
      : 0.65 + (Math.sin(t * 2) * 0.05);
    const voltageSag = totalThrusterLoad * 0.65; // Voltage sag under heavy thruster load
    const currentBatLevel = Math.max(5, bat.level - (isArmed ? 0.00015 : 0.00003));
    
    store.updateBattery({
      level: currentBatLevel,
      voltage: Math.max(13.8, 14.6 + (currentBatLevel / 100) * 2.2 - voltageSag),
      current: Math.max(0.4, currentDraw),
      temperature: 27.5 + totalThrusterLoad * 6.5 + Math.sin(t * 0.3) * 0.2,
    });
  }
}

const mockRos = new MockRosConnection();
export default mockRos;
