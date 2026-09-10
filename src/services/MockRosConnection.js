import * as THREE from 'three';
import useVehicleStore from '../store/vehicleStore';
import auvMotionController from './AUVMotionController';
import subseaCollisionEngine from './SubseaCollisionEngine';
import sysIdEngine from './SystemIdentificationEngine';
import sensorNoiseModel from './SensorNoiseModel';
import hydrodynamicsEngine from './HydrodynamicsEngine';
import Kinematics from '../dt-core/Kinematics.js';
import defaultEnvironment from '../dt-core/EnvironmentModel.js';
import defaultStateEstimator from '../dt-core/StateEstimator.js';
import defaultSyncManager from '../dt-core/SyncManager.js';
import defaultUncertaintyEstimator from '../dt-core/UncertaintyEstimator.js';
import defaultOODDetector from '../dt-core/OODDetector.js';
import defaultValidationEngine from '../dt-core/ValidationEngine.js';
import DTHealthScore from '../dt-core/DTHealthScore.js';

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
    this.orangeInspectStartTime = 0;
    this.lastFlareHitTime = 0;
    this.avoidInspectStartTime = null;
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
    this.orangeInspectStartTime = 0;
    this.lastFlareHitTime = 0;
    this.avoidInspectStartTime = null;
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
    this.orangeInspectStartTime = 0;
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
      ctrl = auvMotionController.update(currentPose, input, flightMode, false, dt, store.battery.voltage || 16.0, t);
    } else if (flightMode === 'QUALIFIKASI') {
      // =========================================================================
      // AUTONOMOUS QUALIFICATION RACING STATE MACHINE (Smooth, Calibrated Pure Pursuit)
      // =========================================================================
      const obs = store.obstacles || { gate: { x: 4.0, z: 0.0 } };
      const TARGET_DEPTH = 0.85; // Centered in gate opening
      const GATE_X = obs.gate?.x ?? 4.0;
      const GATE_Z = obs.gate?.z ?? 0.0;

      let cmdSurge = 0;
      let cmdSway = 0;
      let cmdYaw = 0;
      let cmdHeave = 0;
      let statusText = '';

      const depthErr = TARGET_DEPTH - this.simDepth;
      const holdDepthHeave = Math.max(-0.70, Math.min(0.70, depthErr * 2.5));

      switch (this.qualState) {
        case 1: // State 1: Dive-on-the-fly Launch
          statusText = 'State 1/5: ⚡ Smooth Dive-on-the-Fly Launch (0.85m)';
          cmdHeave = holdDepthHeave;
          cmdSurge = 0.80;

          // Aim directly towards gate while diving
          const diveTargetHeading = Math.atan2(GATE_Z - this.simZ, GATE_X - this.simX);
          let diveErr = diveTargetHeading - this.simHeading;
          while (diveErr > Math.PI) diveErr -= Math.PI * 2;
          while (diveErr < -Math.PI) diveErr += Math.PI * 2;
          cmdYaw = Math.max(-0.90, Math.min(0.90, diveErr * 2.0));

          if (Math.abs(this.simDepth - TARGET_DEPTH) < 0.15 || this.simX > -5.5) {
            this.qualPrevState = 1;
            this.qualState = 4;
            this.qualStateStartTime = t;
          }
          break;

        case 4: // State 4: Pure Pursuit Gate Lock-in
          statusText = 'State 2/5: 🎯 Vector Pure Pursuit Gate Approach';
          cmdHeave = holdDepthHeave;
          cmdSurge = 0.85;

          const lookaheadX = Math.max(GATE_X + 2.0, this.simX + 2.5);
          const targetHeading = Math.atan2(GATE_Z - this.simZ, lookaheadX - this.simX);
          let headingErr = targetHeading - this.simHeading;
          while (headingErr > Math.PI) headingErr -= Math.PI * 2;
          while (headingErr < -Math.PI) headingErr += Math.PI * 2;
          cmdYaw = Math.max(-0.95, Math.min(0.95, headingErr * 2.2));

          if (this.simX >= GATE_X - 0.5) {
            this.qualPrevState = 4;
            this.qualState = 2;
            this.qualStateStartTime = t;
          }
          break;

        case 2: // State 2: Stable Gate Pass & Return Sprint
          cmdHeave = holdDepthHeave;
          const elapsed = t - this.qualStateStartTime;

          if (!this.qualReturnPass) {
            statusText = `State 3/5: 🏁 Stable Gate Pass (${elapsed.toFixed(1)}s / 4.0s @ 0.90 m/s)`;
            cmdSurge = 0.90;
            
            // Lock straight heading 0 rad
            let straightErr = 0.0 - this.simHeading;
            while (straightErr > Math.PI) straightErr -= Math.PI * 2;
            while (straightErr < -Math.PI) straightErr += Math.PI * 2;
            cmdYaw = Math.max(-0.70, Math.min(0.70, straightErr * 2.0));

            if (elapsed >= 4.0 || this.simX >= GATE_X + 3.0) {
              this.qualPrevState = 2;
              this.qualState = 3;
              this.qualStateStartTime = t;
            }
          } else {
            // Return pass
            if (this.simX > GATE_X - 2.5) {
              statusText = `State 5/5: ⚡ Return Transit Through Gate (Z = ${GATE_Z.toFixed(1)}m)`;
              cmdSurge = 0.90;
              
              const retTargetYaw = Math.atan2(GATE_Z - this.simZ, -10.0 - this.simX);
              let returnErr = retTargetYaw - this.simHeading;
              while (returnErr > Math.PI) returnErr -= Math.PI * 2;
              while (returnErr < -Math.PI) returnErr += Math.PI * 2;
              cmdYaw = Math.max(-0.85, Math.min(0.85, returnErr * 2.0));
            } else {
              statusText = `State 5/5: 🚀 CRUISE TO DOCK (${elapsed.toFixed(1)}s)`;
              cmdSurge = 0.85;
              const returnHeading = Math.atan2(2.0 - this.simZ, -11.0 - this.simX);
              let retErr = returnHeading - this.simHeading;
              while (retErr > Math.PI) retErr -= Math.PI * 2;
              while (retErr < -Math.PI) retErr += Math.PI * 2;
              cmdYaw = Math.max(-0.85, Math.min(0.85, retErr * 2.0));

              if (elapsed >= 6.0 || this.simX <= -10.2) {
                this.qualPrevState = 2;
                this.qualState = 5;
                this.qualStateStartTime = t;
              }
            }
          }
          break;

        case 3: // State 3: Clean Hairpin 180° U-Turn (Target heading: Math.PI)
          statusText = 'State 4/5: 🔄 Controlled 180° Hairpin U-Turn';
          cmdHeave = holdDepthHeave;
          cmdSurge = 0.15; // Smooth controlled arc

          let targetUTurn = Math.PI;
          let uTurnErr = targetUTurn - this.simHeading;
          while (uTurnErr > Math.PI) uTurnErr -= Math.PI * 2;
          while (uTurnErr < -Math.PI) uTurnErr += Math.PI * 2;

          if (Math.abs(uTurnErr) < 0.20) {
            this.qualPrevState = 3;
            this.qualState = 2;
            this.qualReturnPass = true;
            this.qualStateStartTime = t;
          } else {
            cmdYaw = Math.max(-1.10, Math.min(1.10, uTurnErr * 2.2));
          }
          break;

        case 5: // State 5: Rapid Surface
          statusText = 'QUALIFICATION COMPLETE! 🏆 Surfacing at Start Zone';
          cmdSurge = 0.20;
          cmdHeave = -0.55;
          break;
      }

      const qualTargetMsg = `[QUAL] ${statusText}`;
      if (store.activeTarget !== qualTargetMsg) {
        useVehicleStore.setState({ activeTarget: qualTargetMsg });
      }

      ctrl = auvMotionController.update(
        currentPose,
        { surge: cmdSurge, sway: cmdSway, yaw: cmdYaw, heave: cmdHeave },
        'MANUAL',
        true,
        dt,
        store.battery.voltage || 16.0,
        t
      );
    } else if (flightMode === 'FINAL') {
      // =========================================================================
      // AUTONOMOUS SAUVC FINAL MISSION — SMOOTH, CALIBRATED RACING EXECUTION
      // =========================================================================
      this.missionTime += dt;

      const obs = store.obstacles || {
        orange_flare: { x: -6.0, z: 2.0 },
        blue_flare: { x: -2.0, z: 2.2 },
        red_flare: { x: 0.5, z: 4.0 },
        yellow_flare: { x: -0.5, z: -4.5 },
        gate: { x: 4.0, z: 0.0 },
        drum_red_tgt: { x: 10.5, z: 1.5 },
      };

      const orangeX = obs.orange_flare?.x ?? -6.0;
      const orangeZ = obs.orange_flare?.z ?? 2.0;
      const blueX = obs.blue_flare?.x ?? -2.0;
      const blueZ = obs.blue_flare?.z ?? 2.2;
      const redX = obs.red_flare?.x ?? 0.5;
      const redZ = obs.red_flare?.z ?? 4.0;
      const yellowX = obs.yellow_flare?.x ?? -0.5;
      const yellowZ = obs.yellow_flare?.z ?? -4.5;
      const gateX = obs.gate?.x ?? 4.0;
      const gateZ = obs.gate?.z ?? 0.0;
      const drumX = obs.drum_red_tgt?.x ?? 10.5;
      const drumZ = obs.drum_red_tgt?.z ?? 1.5;

      const flareStrategies = store.flareStrategies || {
        orange_flare: 'MENGHINDAR',
        blue_flare: 'TABRAK',
        red_flare: 'TABRAK',
        yellow_flare: 'TABRAK',
      };
      const flaresFallen = store.flaresFallen || {
        red: false,
        blue: false,
        yellow: false,
        orange: false,
      };
      const obstacleOrder = store.obstacleOrder || [
        'orange_flare',
        'blue_flare',
        'red_flare',
        'yellow_flare',
        'gate',
        'drum_red_tgt',
      ];
      const obstacleEnabled = store.obstacleEnabled || {
        orange_flare: true,
        blue_flare: true,
        red_flare: true,
        yellow_flare: true,
        gate: true,
        drum_red_tgt: true,
      };

      const rawWaypoints = [];

      // Milestone 0: Immediate Calm Dive Launch to Arena Target Depth (0.85m)
      rawWaypoints.push({
        id: 'initial_dive',
        title: '[#0] ⚡ Smooth Dive Launch (0.85m)',
        x: -9.0,
        z: 2.0,
        targetDepth: 0.85,
        speed: 0.80,
        threshold: 1.10,
      });

      // Helper function to build clean sequential waypoints
      const addFlareWaypoint = (key, color, label, flX, flZ, stepNum) => {
        const strategy = flareStrategies[key] || 'TABRAK';
        const isTabrak = strategy === 'TABRAK';

        if (isTabrak) {
          rawWaypoints.push({
            id: key,
            title: `[#${stepNum}] 💥 Ram ${label}`,
            strategy: 'TABRAK',
            x: flX,
            z: flZ,
            targetDepth: 0.85,
            speed: 0.85,
            threshold: 0.80,
            hitFlare: color,
            flareX: flX,
            flareZ: flZ,
          });
        } else {
          // 1. Waypoint Inspeksi Visual (Standoff at 1.4m)
          rawWaypoints.push({
            id: `${key}_inspect`,
            title: `[#${stepNum}] 🛡️ Quick Inspect ${label}`,
            strategy: 'MENGHINDAR',
            x: flX - 1.40,
            z: flZ,
            targetDepth: 0.85,
            speed: 0.70,
            threshold: 0.75,
            hitFlare: null,
            isAvoidInspect: true,
            inspectColor: color,
            flareX: flX,
            flareZ: flZ,
          });

          // 2. Bypass lateral waypoint to smoothly clear the pole
          const clearZ = (flZ >= 0) ? flZ - 1.20 : flZ + 1.20;
          rawWaypoints.push({
            id: `${key}_bypass`,
            title: `[#${stepNum}] 🛡️ Bypass ${label}`,
            strategy: 'MENGHINDAR',
            x: flX + 1.20,
            z: clearZ,
            targetDepth: 0.85,
            speed: 0.80,
            threshold: 0.85,
            hitFlare: null,
            isBypass: true,
          });
        }
      };

      const flareConfig = {
        orange_flare: { color: 'orange', label: 'Orange Flare', flX: orangeX, flZ: orangeZ },
        blue_flare:   { color: 'blue',   label: 'Blue Flare',   flX: blueX,   flZ: blueZ },
        red_flare:    { color: 'red',    label: 'Red Flare',    flX: redX,    flZ: redZ },
        yellow_flare: { color: 'yellow', label: 'Yellow Flare', flX: yellowX, flZ: yellowZ },
      };

      let stepNum = 1;
      obstacleOrder.forEach((key) => {
        if (obstacleEnabled[key] === false) return;

        if (flareConfig[key]) {
          const c = flareConfig[key];
          addFlareWaypoint(key, c.color, c.label, c.flX, c.flZ, stepNum++);
        } else if (key === 'gate') {
          rawWaypoints.push({
            id: 'gate_runway',
            title: `[#${stepNum}] 🚪 Approach Gate`,
            x: gateX - 1.50,
            z: gateZ,
            targetDepth: 0.85,
            speed: 0.80,
            threshold: 0.80,
          });
          rawWaypoints.push({
            id: 'gate_pass',
            title: `[#${stepNum}] 🏁 Gate Transit`,
            x: gateX + 2.80,
            z: gateZ,
            targetDepth: 0.85,
            speed: 0.90,
            threshold: 0.85,
          });
          stepNum++;
        } else if (key === 'drum_red_tgt' || key === 'drum') {
          rawWaypoints.push({
            id: 'drum_search',
            title: `[#${stepNum}] 🎯 Approach Red Drum`,
            x: drumX - 0.35,
            z: drumZ,
            targetDepth: 0.85,
            speed: 0.75,
            threshold: 0.80,
          });
          rawWaypoints.push({
            id: 'drum_drop',
            title: `[#${stepNum}] ⛳ Drop Ball Payload`,
            x: drumX,
            z: drumZ,
            targetDepth: 0.85,
            speed: 0.30,
            threshold: 0.65,
          });
          stepNum++;
        }
      });

      // Final milestone: Surface
      rawWaypoints.push({
        id: 'surface_dock',
        title: `[#${stepNum}] 🏆 Surface at Dock`,
        x: -11.0,
        z: 2.0,
        targetDepth: 0.15,
        speed: 0.75,
        threshold: 0.90,
      });

      const waypoints = rawWaypoints.map((wp, idx) => ({
        ...wp,
        name: `${idx + 1}/${rawWaypoints.length}: ${wp.title}`,
      }));

      const currentWp = waypoints[this.missionStage] || waypoints[waypoints.length - 1];
      const distToWp = Math.sqrt((currentWp.x - this.simX) ** 2 + (currentWp.z - this.simZ) ** 2);

      // 1. Direct Physical Flare Contact & Knockdown
      if (currentWp.hitFlare && !flaresFallen[currentWp.hitFlare]) {
        const flX = currentWp.flareX ?? currentWp.x;
        const flZ = currentWp.flareZ ?? currentWp.z;
        const contactDist = Math.sqrt((flX - this.simX) ** 2 + (flZ - this.simZ) ** 2);

        if (contactDist <= 0.85) {
          store.knockdownFlare(currentWp.hitFlare);
          this.lastFlareHitTime = t;
          console.log(`[MockROS] 💥 DIRECT HIT! Knocked down ${currentWp.hitFlare} flare!`);
        }
      }

      // 2. Drum Approach & Instant Ball Drop
      if (currentWp.id === 'drum_search' || currentWp.id === 'drum_drop') {
        if (distToWp < 2.0) {
          const tiltFactor = Math.min(1.0, (2.0 - distToWp) / 1.0);
          targetPitchAngle = -20.0 * tiltFactor;
        }

        if (currentWp.id === 'drum_drop') {
          if (!this.drumDropTriggered) {
            this.drumDropTriggered = true;
            this.drumWaitTime = t;
            store.setGripperState('OPEN');
            store.dropBallIntoDrum(drumX, drumZ);
          }

          if (t - this.drumWaitTime > 0.8) {
            this.missionStage++;
          }
        }
      }

      // 3. Waypoint Advancement Logic
      let canAdvance = distToWp < currentWp.threshold;
      if (currentWp.hitFlare) {
        canAdvance = flaresFallen[currentWp.hitFlare] || distToWp < currentWp.threshold;
      } else if (currentWp.isAvoidInspect) {
        const flX = currentWp.flareX ?? currentWp.x;
        const flZ = currentWp.flareZ ?? currentWp.z;
        const distToFlare = Math.sqrt((this.simX - flX) ** 2 + (this.simZ - flZ) ** 2);
        
        if (distToFlare <= 1.60 || distToWp < currentWp.threshold) {
          if (!this.avoidInspectStartTime) {
            this.avoidInspectStartTime = t;
          }
          if (t - this.avoidInspectStartTime >= 0.45) {
            canAdvance = true;
            this.avoidInspectStartTime = null;
          } else {
            canAdvance = false;
          }
        }
      }

      if (
        currentWp.id !== 'drum_drop' &&
        canAdvance &&
        this.missionStage < waypoints.length - 1
      ) {
        this.missionStage++;
      }

      // 4. Depth Regulation (Robust Proportional Control to Target Depth)
      const depthErr = currentWp.targetDepth - this.simDepth;
      const cmdHeave = Math.max(-0.70, Math.min(0.70, depthErr * 2.5));

      // 5. Smooth Pure Pursuit Guidance
      const toWpX = currentWp.x - this.simX;
      const toWpZ = currentWp.z - this.simZ;
      const targetHeading = Math.atan2(toWpZ, toWpX);

      let headingErr = targetHeading - this.simHeading;
      while (headingErr > Math.PI) headingErr -= Math.PI * 2;
      while (headingErr < -Math.PI) headingErr += Math.PI * 2;

      // Proportional Smooth Steering (gentle gain, no overshoots)
      let cmdYaw = Math.max(-1.0, Math.min(1.0, headingErr * 2.0));

      // Continuous Smooth Forward Surge modulated by heading alignment
      const alignment = Math.max(0, Math.cos(headingErr));
      const forwardDrive = Math.max(0.20, alignment * alignment);
      let cmdSurge = currentWp.speed * forwardDrive;

      if (currentWp.isAvoidInspect && (this.avoidInspectStartTime !== null)) {
        cmdSurge = 0.0;
        cmdYaw = 0.0;
      }

      let finalTarget = `[FINAL] ${currentWp.name}`;
      if (currentWp.id === 'drum_drop' && this.drumDropTriggered) {
        finalTarget = `[FINAL] ${waypoints.length}/${waypoints.length}: 🎯 BALL DROPPED INTO RED DRUM!`;
      }

      if (store.activeTarget !== finalTarget) {
        useVehicleStore.setState({ activeTarget: finalTarget });
      }

      ctrl = auvMotionController.update(
        currentPose,
        { surge: cmdSurge, sway: 0, yaw: cmdYaw, heave: cmdHeave },
        'MANUAL',
        true,
        dt,
        store.battery.voltage || 16.0,
        t
      );
    } else {
      ctrl = auvMotionController.update(currentPose, input, flightMode, true, dt, store.battery.voltage || 16.0, t);
    }

    // Process Thruster Dynamics through the Identified Polynomial Model (ARX / ARMAX / OE / BJ)
    const rawSurge = ctrl?.surge || 0;
    const sysIdRes = sysIdEngine.step(input.surge || (rawSurge / 1.0), rawSurge, dt);
    const effectiveSurge = Math.max(-1.10, Math.min(1.10, rawSurge * 0.75 + (sysIdRes.velocity || 0) * 0.25));

    // Advance dynamic pitch and roll angles
    const pitchDeg = (ctrl?.pitch || 0) + targetPitchAngle + (isArmed ? Math.sin(t * 1.5) * 0.4 : Math.sin(t * 0.5) * 0.8);
    const rollDeg = (ctrl?.roll || 0) + (isArmed ? Math.sin(t * 1.2) * 0.4 : Math.sin(t * 0.4) * 0.6);
    const pitchRad = (pitchDeg * Math.PI) / 180;
    const rollRad = (rollDeg * Math.PI) / 180;

    // Advance proposed heading with angle wrapping
    this.simHeading = Kinematics.wrapAngle(this.simHeading + (ctrl?.yaw || 0) * dt);

    // Compute 3D velocity in NED world frame using full 6-DOF Kinematic transformation
    const vWorld = Kinematics.bodyToWorldVelocity(
      { u: effectiveSurge, v: ctrl?.sway || 0, w: ctrl?.heave || 0 },
      { roll: rollRad, pitch: pitchRad, yaw: this.simHeading }
    );

    // Get 3D ocean/pool current from EnvironmentModel
    const currentNed = defaultEnvironment.getCurrentVelocity(this.simX, this.simZ, this.simDepth);

    let proposedX = this.simX + (vWorld.x + currentNed.vx) * dt;
    let proposedZ = this.simZ + (vWorld.y + currentNed.vy) * dt;
    // Stable, decoupled vertical heave depth integration
    let proposedDepth = Math.max(0.08, Math.min(1.84, this.simDepth + ((ctrl?.heave || 0) + currentNed.vz) * dt));

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
    const yawDeg = Kinematics.wrapAngle360((this.simHeading * 180) / Math.PI);

    // Three.js 6-DOF Quaternion Orientation
    const eulerThree = new THREE.Euler(
      rollRad,
      -this.simHeading,
      pitchRad,
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

    // Subsea Temperature (ground truth before sensor noise)
    const tempSurface = 26.4;
    const tempDepthGradient = -0.32 * this.simDepth;
    const tempThruster = totalThrusterLoad * 0.28;
    const dynamicWaterTemp = tempSurface + tempDepthGradient + tempThruster;

    // === SENSOR NOISE MODEL: Depth (MS5837-30BA) ===
    const depthNoisy = sensorNoiseModel.applyDepthNoise(this.simDepth, dynamicWaterTemp, dt);
    const noisyTemp = sensorNoiseModel.applyTemperatureNoise(dynamicWaterTemp);

    store.updateDepthSensor({
      depth: depthNoisy.depth,
      pressure: depthNoisy.pressure,
      temperature: noisyTemp,
    });

    // === SENSOR NOISE MODEL: DVL (Acoustic Altimeter) ===
    const dvlNoisy = sensorNoiseModel.applyDVLNoise(y3D);
    store.updateDVL(dvlNoisy);

    // === SENSOR NOISE MODEL: IMU (BNO055 / ICM-20948) ===
    const rawIMU = {
      roll: rollDeg || 0,
      pitch: pitchDeg || 0,
      yaw: yawDeg || 0,
      accelX: effectiveSurge * 0.25,
      accelY: (ctrl?.sway || 0) * 0.25,
      accelZ: -9.81 + (ctrl?.heave || 0) * 0.3,
    };
    const imuNoisy = sensorNoiseModel.applyIMUNoise(rawIMU, dt);
    store.updateIMU(imuNoisy);

    // 4S LiPo Battery smooth real-time telemetry
    const bat = store.battery;
    const rawCurrentDraw = isArmed 
      ? 1.4 + totalThrusterLoad * 16.5
      : 0.65;
    const voltageSag = totalThrusterLoad * 0.65;
    const currentBatLevel = Math.max(5, bat.level - (isArmed ? 0.00015 : 0.00003));
    const rawVoltage = Math.max(13.8, 14.6 + (currentBatLevel / 100) * 2.2 - voltageSag);

    // === SENSOR NOISE MODEL: Battery ADC (12-bit + switching ripple) ===
    const batNoisy = sensorNoiseModel.applyBatteryNoise(rawVoltage, rawCurrentDraw, t);
    
    store.updateBattery({
      level: currentBatLevel,
      voltage: batNoisy.voltage,
      current: Math.max(0.4, batNoisy.current),
      temperature: 27.5 + totalThrusterLoad * 6.5 + sensorNoiseModel.gaussian(0, 0.15),
    });

    // =========================================================================
    // DIGITAL TWIN CORE REAL-TIME PIPELINE
    // Fuses EKF, tracks synchronization, evaluates uncertainty, OOD & Health
    // =========================================================================
    // 1. 15-State Extended Kalman Filter (EKF) State Estimation
    defaultStateEstimator.predict(
      [rawIMU.accelX, rawIMU.accelY, rawIMU.accelZ],
      [ctrl?.rollRate || 0, ctrl?.pitchRate || 0, ctrl?.yaw || 0],
      dt
    );
    defaultStateEstimator.updateDepth(depthNoisy.depth);
    defaultStateEstimator.updateCompass(this.simHeading);
    defaultStateEstimator.updateDVL([effectiveSurge, ctrl?.sway || 0, ctrl?.heave || 0]);
    const estimated = defaultStateEstimator.getEstimatedState();
    store.setEstimatedState(estimated);

    // 2. Synchronization & Latency Tracking
    defaultSyncManager.recordPacket(Date.now());
    defaultSyncManager.recordCompute(0.8, 1.4);
    const syncMetrics = defaultSyncManager.getMetrics();
    store.setSyncMetrics(syncMetrics);

    // 3. Uncertainty & OOD Evaluation
    defaultUncertaintyEstimator.updateResidual(
      { position: { x: this.simX, y: this.simZ, z: this.simDepth }, velocity: { u: effectiveSurge, v: ctrl?.sway || 0, w: ctrl?.heave || 0 } },
      estimated
    );
    const uncertaintyInfo = defaultUncertaintyEstimator.isControlSafe();
    store.setUncertainty({
      score: uncertaintyInfo.uncertainty,
      level: uncertaintyInfo.level,
      confidence: uncertaintyInfo.confidence,
    });

    const oodResult = defaultOODDetector.evaluate(store);
    store.setOODStatus(oodResult);

    // 4. Real-time Validation Engine & Multi-Pillar DT Health Score
    defaultValidationEngine.addSample(
      { position: { x: this.simX, y: this.simZ, z: this.simDepth }, velocity: { u: effectiveSurge, v: ctrl?.sway || 0, w: ctrl?.heave || 0 }, attitude: { roll: rollRad, pitch: pitchRad, yaw: this.simHeading } },
      estimated,
      1
    );
    const validationMetrics = defaultValidationEngine.computeMetrics();
    store.setValidationMetrics(validationMetrics);

    const dtHealth = DTHealthScore.evaluate({
      sync: syncMetrics,
      estimator: estimated,
      uncertainty: uncertaintyInfo,
      ood: oodResult,
      battery: batNoisy,
    });
    store.setDtHealth(dtHealth);
  }
}

const mockRos = new MockRosConnection();
export default mockRos;
