import * as THREE from 'three';
import useVehicleStore from '../store/vehicleStore';
import auvMotionController from './AUVMotionController';
import subseaCollisionEngine from './SubseaCollisionEngine';
import sysIdEngine from './SystemIdentificationEngine';
import sensorNoiseModel from './SensorNoiseModel';
import hydrodynamicsEngine from './HydrodynamicsEngine';

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
      // AUTONOMOUS QUALIFICATION STATE MACHINE (from qualification.py)
      // =========================================================================
      const obs = store.obstacles || { gate: { x: 4.0, z: 0.0 } };
      const TARGET_DEPTH = 0.85; // Optimal depth (Y = 1.15m): vertically centered in 1.5m gate opening
      const GATE_X = obs.gate?.x ?? 4.0;
      const GATE_Z = obs.gate?.z ?? 0.0;

      let cmdSurge = 0;
      let cmdYaw = 0;
      let cmdHeave = 0;
      let statusText = '';

      switch (this.qualState) {
        case 1: // State 1: Dive smoothly to target depth
          statusText = 'State 1/5: Diving to Target Gate Depth 0.85m';
          cmdHeave = Math.max(-0.45, Math.min(0.45, (TARGET_DEPTH - this.simDepth) * 2.2));
          cmdSurge = 0.20; // Gentle forward glide while diving
          if (Math.abs(this.simDepth - TARGET_DEPTH) < 0.06) {
            this.qualPrevState = 1;
            this.qualState = 4;
            this.qualStateStartTime = t;
          }
          break;

        case 4: // State 4: Track Gate (YOLO Gate PID Vector Field Alignment)
          statusText = 'State 2/5: YOLO Gate PID Tracking & Centerline Alignment';
          cmdHeave = Math.max(-0.4, Math.min(0.4, (TARGET_DEPTH - this.simDepth) * 2.2));
          cmdSurge = 0.70;

          // Vector Field Pure Pursuit: Aim forward through gate centerline with zero singularity
          const lookaheadX = Math.max(GATE_X + 1.2, this.simX + 2.2);
          const targetHeading = Math.atan2((GATE_Z - this.simZ) * 1.8, lookaheadX - this.simX);
          let headingErr = targetHeading - this.simHeading;
          while (headingErr > Math.PI) headingErr -= Math.PI * 2;
          while (headingErr < -Math.PI) headingErr += Math.PI * 2;
          cmdYaw = Math.max(-0.65, Math.min(0.65, headingErr * 2.8));

          // When aligned in front of gate, transition to State 2 gate pass
          if ((this.simX >= GATE_X - 1.2 && Math.abs(this.simZ - GATE_Z) < 0.35) || this.simX >= GATE_X - 0.2) {
            this.qualPrevState = 4;
            this.qualState = 2;
            this.qualStateStartTime = t;
          }
          break;

        case 2: // State 2: Forward / Return through Gate
          cmdHeave = Math.max(-0.4, Math.min(0.4, (TARGET_DEPTH - this.simDepth) * 2.2));
          const elapsed = t - this.qualStateStartTime;

          if (!this.qualReturnPass) {
            statusText = `State 3/5: Passing Forward Through Gate (${elapsed.toFixed(1)}s / 6.0s)`;
            cmdSurge = 0.75;
            
            // Vector Field Pure Pursuit Line-Lock to GATE_Z
            const desiredYaw = Math.atan2((GATE_Z - this.simZ) * 2.4, 2.0);
            let straightErr = desiredYaw - this.simHeading;
            while (straightErr > Math.PI) straightErr -= Math.PI * 2;
            while (straightErr < -Math.PI) straightErr += Math.PI * 2;
            cmdYaw = Math.max(-0.65, Math.min(0.65, straightErr * 3.0));

            if (elapsed >= 5.5 || this.simX >= GATE_X + 2.8) {
              this.qualPrevState = 2;
              this.qualState = 3;
              this.qualStateStartTime = t;
              this.qualUTurnTargetYaw = Math.PI; // Exact 180° facing back
            }
          } else {
            // Return pass: First pass back through gate to GATE_X - 2.8m before heading to start dock!
            if (this.simX > GATE_X - 2.8) {
              statusText = `State 5/5: Gliding Back Through Gate (Z = ${GATE_Z.toFixed(1)}m)`;
              cmdSurge = 0.70;
              
              // Return lookahead in -X direction towards gate center
              const returnLookaheadX = Math.min(GATE_X - 2.8, this.simX - 2.0);
              const returnDesiredYaw = Math.atan2((GATE_Z - this.simZ) * 1.8, returnLookaheadX - this.simX);
              let returnErr = returnDesiredYaw - this.simHeading;
              while (returnErr > Math.PI) returnErr -= Math.PI * 2;
              while (returnErr < -Math.PI) returnErr += Math.PI * 2;
              cmdYaw = Math.max(-0.65, Math.min(0.65, returnErr * 3.0));
            } else {
              statusText = `State 5/5: Returning to Start Zone (${elapsed.toFixed(1)}s / 8.0s)`;
              cmdSurge = 0.75;
              const returnHeading = Math.atan2(2.0 - this.simZ, -11.0 - this.simX);
              let retErr = returnHeading - this.simHeading;
              while (retErr > Math.PI) retErr -= Math.PI * 2;
              while (retErr < -Math.PI) retErr += Math.PI * 2;
              cmdYaw = Math.max(-0.55, Math.min(0.55, retErr * 2.2));

              if (elapsed >= 8.0 || this.simX <= -10.0) {
                this.qualPrevState = 2;
                this.qualState = 5;
                this.qualStateStartTime = t;
              }
            }
          }
          break;

        case 3: // State 3: 180° In-Place U-Turn (Zero Surge)
          statusText = 'State 4/5: Performing In-Place 180° U-Turn (Putar Balik)';
          cmdHeave = Math.max(-0.35, Math.min(0.35, (TARGET_DEPTH - this.simDepth) * 2.0));
          cmdSurge = 0.0; // In-place rotation, zero lateral drift!

          let uTurnErr = this.qualUTurnTargetYaw - Math.abs(this.simHeading);
          if (Math.abs(uTurnErr) < 0.12) {
            this.qualPrevState = 3;
            this.qualState = 2;
            this.qualReturnPass = true;
            this.qualStateStartTime = t;
          } else {
            cmdYaw = 0.75;
          }
          break;

        case 5: // State 5: Surface
          statusText = 'QUALIFICATION COMPLETE! 🏆 Surfacing at Start Zone';
          cmdSurge = 0.05;
          cmdHeave = -0.35; // Ascend smoothly to surface
          break;
      }

      // Active Wall Safety Repulsion Barrier
      let wallSafetyYaw = 0;
      if (this.simZ < -5.0) {
        wallSafetyYaw = ((-5.0 - this.simZ) / 2.0) * 0.90;
      } else if (this.simZ > 5.0) {
        wallSafetyYaw = -((this.simZ - 5.0) / 2.0) * 0.90;
      }
      cmdYaw = Math.max(-0.95, Math.min(0.95, cmdYaw + wallSafetyYaw));

      const qualTargetMsg = `[QUAL] ${statusText}`;
      if (store.activeTarget !== qualTargetMsg) {
        useVehicleStore.setState({ activeTarget: qualTargetMsg });
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
    } else if (flightMode === 'FINAL') {
      // =========================================================================
      // AUTONOMOUS SAUVC FINAL MISSION (High-Speed Direct Milestone Execution)
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

      const rawWaypoints = [];

      // 1. TARGET 1: ORANGE FLARE ZONE
      if (flareStrategies.orange_flare === 'TABRAK') {
        rawWaypoints.push({
          id: 'orange_flare',
          title: '💥 Ram Orange Flare',
          x: orangeX,
          z: orangeZ,
          targetDepth: 0.85,
          speed: 1.20,
          threshold: 0.85,
          hitFlare: 'orange',
        });
      } else {
        const evadeSide = orangeZ >= 0 ? -1.5 : 1.5;
        rawWaypoints.push({
          id: 'orange_bypass',
          title: '🛡️ Bypass Orange Flare',
          x: orangeX + 0.6,
          z: orangeZ + evadeSide,
          targetDepth: 0.85,
          speed: 1.15,
          threshold: 0.90,
        });
      }

      // 2. TARGET 2: BLUE FLARE ZONE
      if (flareStrategies.blue_flare === 'TABRAK') {
        rawWaypoints.push({
          id: 'blue_flare',
          title: '💥 Ram Blue Flare',
          x: blueX,
          z: blueZ,
          targetDepth: 0.85,
          speed: 1.20,
          threshold: 0.85,
          hitFlare: 'blue',
        });
      } else {
        const evadeSide = blueZ >= 0 ? -1.5 : 1.5;
        rawWaypoints.push({
          id: 'blue_bypass',
          title: '🛡️ Bypass Blue Flare',
          x: blueX + 0.6,
          z: blueZ + evadeSide,
          targetDepth: 0.85,
          speed: 1.15,
          threshold: 0.90,
        });
      }

      // 3. TARGET 3: RED FLARE ZONE
      if (flareStrategies.red_flare === 'TABRAK') {
        rawWaypoints.push({
          id: 'red_flare',
          title: '💥 Ram Red Flare',
          x: redX,
          z: redZ,
          targetDepth: 0.85,
          speed: 1.20,
          threshold: 0.85,
          hitFlare: 'red',
        });
      } else {
        const evadeSide = redZ >= 0 ? -1.5 : 1.5;
        rawWaypoints.push({
          id: 'red_bypass',
          title: '🛡️ Bypass Red Flare',
          x: redX + 0.6,
          z: redZ + evadeSide,
          targetDepth: 0.85,
          speed: 1.15,
          threshold: 0.90,
        });
      }

      // 4. TARGET 4: YELLOW FLARE ZONE
      if (flareStrategies.yellow_flare === 'TABRAK') {
        rawWaypoints.push({
          id: 'yellow_flare',
          title: '💥 Ram Yellow Flare',
          x: yellowX,
          z: yellowZ,
          targetDepth: 0.85,
          speed: 1.20,
          threshold: 0.85,
          hitFlare: 'yellow',
        });
      } else {
        const evadeSide = yellowZ >= 0 ? -1.5 : 1.5;
        rawWaypoints.push({
          id: 'yellow_bypass',
          title: '🛡️ Bypass Yellow Flare',
          x: yellowX + 0.6,
          z: yellowZ + evadeSide,
          targetDepth: 0.85,
          speed: 1.15,
          threshold: 0.90,
        });
      }

      // 5. TARGET 5: GATE APPROACH & RUNWAY ALIGNMENT
      rawWaypoints.push({
        id: 'gate_align',
        title: '🎯 Align with Gate Runway',
        x: gateX - 1.6,
        z: gateZ,
        targetDepth: 0.85,
        speed: 1.05,
        threshold: 0.85,
        gateAlign: true,
      });

      // 6. TARGET 6: FAST STRAIGHT GATE TRANSIT
      rawWaypoints.push({
        id: 'gate_pass',
        title: '🚪 Straight Gate Transit',
        x: gateX + 2.5,
        z: gateZ,
        targetDepth: 0.85,
        speed: 1.25,
        threshold: 1.0,
        gateTransit: true,
        gateTargetZ: gateZ,
      });

      // 7. TARGET 7: APPROACH TARGET DRUM
      rawWaypoints.push({
        id: 'drum_search',
        title: '🎯 Approach Target Drum',
        x: drumX - 0.4,
        z: drumZ,
        targetDepth: 0.95,
        speed: 0.90,
        threshold: 0.70,
      });

      // 8. TARGET 8: DIRECT PAYLOAD DROP
      rawWaypoints.push({
        id: 'drum_drop',
        title: '🔴 Release Ball into Drum',
        x: drumX,
        z: drumZ,
        targetDepth: 0.88,
        speed: 0.10,
        threshold: 0.50,
      });

      // 9. TARGET 9: MISSION COMPLETE & SURFACE
      rawWaypoints.push({
        id: 'mission_complete',
        title: '🏆 MISSION COMPLETE! Surfacing',
        x: drumX + 1.2,
        z: 0.0,
        targetDepth: 0.15,
        speed: 0.35,
        threshold: 0.80,
      });

      const waypoints = rawWaypoints.map((wp, idx) => ({
        ...wp,
        name: `${idx + 1}/${rawWaypoints.length}: ${wp.title}`,
      }));

      const currentWp = waypoints[this.missionStage] || waypoints[waypoints.length - 1];
      const distToWp = Math.sqrt((currentWp.x - this.simX) ** 2 + (currentWp.z - this.simZ) ** 2);

      let cmdSurge = currentWp.speed;
      let cmdYaw = 0;

      // 1. Direct Flare Knockdown (Strict Proximity Only)
      if (currentWp.hitFlare && distToWp < 0.65) {
        store.knockdownFlare(currentWp.hitFlare);
      }

      // 2. Drum Approach Pitch Tilt & Instant Ball Drop
      if (currentWp.id === 'drum_search' || currentWp.id === 'drum_drop') {
        if (distToWp < 2.2) {
          const tiltFactor = Math.min(1.0, (2.2 - distToWp) / 1.0);
          targetPitchAngle = -30.0 * tiltFactor;
        }

        if (currentWp.id === 'drum_drop') {
          if (!this.drumDropTriggered) {
            this.drumDropTriggered = true;
            this.drumWaitTime = t;
            store.setGripperState('OPEN');
            store.dropBallIntoDrum(drumX, drumZ);
          }

          // Quick 0.6s release, then immediately surface!
          if (t - this.drumWaitTime > 0.6) {
            this.missionStage++;
          }
        }
      }

      // 3. Fast Deterministic Waypoint Advancement
      let canAdvance = distToWp < currentWp.threshold;
      if (currentWp.hitFlare && flaresFallen[currentWp.hitFlare]) {
        canAdvance = true;
      }

      if (
        currentWp.id !== 'drum_drop' &&
        canAdvance &&
        this.missionStage < waypoints.length - 1
      ) {
        this.missionStage++;
      }

      // 4. Depth PID Command
      const depthErr = currentWp.targetDepth - this.simDepth;
      const cmdHeave = Math.max(-0.65, Math.min(0.65, depthErr * 2.0));

      // 5. Yaw Heading Steering (Fast Pure Pursuit with Local Obstacle Guard)
      if (currentWp.gateTransit) {
        const tgtZ = currentWp.gateTargetZ ?? gateZ;
        const desiredGateYaw = Math.atan2((tgtZ - this.simZ) * 2.5, 2.0);
        let trackErr = desiredGateYaw - this.simHeading;
        while (trackErr > Math.PI) trackErr -= Math.PI * 2;
        while (trackErr < -Math.PI) trackErr += Math.PI * 2;
        cmdYaw = Math.max(-0.85, Math.min(0.85, trackErr * 3.5));
        cmdSurge = currentWp.speed;
      } else {
        let toWpX = currentWp.x - this.simX;
        let toWpZ = currentWp.z - this.simZ;
        const toWpDist = Math.sqrt(toWpX * toWpX + toWpZ * toWpZ) || 1.0;

        let attX = toWpX / toWpDist;
        let attZ = toWpZ / toWpDist;

        // Reactive Obstacle Avoidance for nearby non-target standing flares (tight 1.2m radius)
        let repX = 0;
        let repZ = 0;

        const allFlares = [
          { key: 'orange_flare', color: 'orange', x: orangeX, z: orangeZ },
          { key: 'blue_flare',   color: 'blue',   x: blueX,   z: blueZ },
          { key: 'red_flare',    color: 'red',    x: redX,    z: redZ },
          { key: 'yellow_flare', color: 'yellow', x: yellowX, z: yellowZ },
        ];

        for (const fl of allFlares) {
          const isFallen = flaresFallen[fl.color];
          const isCurrentRamTarget = currentWp.hitFlare === fl.color;
          const isMarkedMenghindar = flareStrategies[fl.key] === 'MENGHINDAR';

          if (!isFallen && !isCurrentRamTarget && isMarkedMenghindar) {
            const dx = this.simX - fl.x;
            const dz = this.simZ - fl.z;
            const dist = Math.sqrt(dx * dx + dz * dz);
            const AVOID_RADIUS = 1.2;

            if (dist < AVOID_RADIUS && dist > 0.001) {
              const force = ((AVOID_RADIUS - dist) / AVOID_RADIUS) * 2.0;
              const sideSign = fl.z >= 0 ? -1 : 1;
              repX += (dx / dist) * force;
              repZ += ((dz / dist) + ((-dz / dist) * sideSign * 0.8)) * force;
            }
          }
        }

        const totalDirX = attX + repX;
        const totalDirZ = attZ + repZ;
        const targetHeading = Math.atan2(totalDirZ, totalDirX);

        let headingErr = targetHeading - this.simHeading;
        while (headingErr > Math.PI) headingErr -= Math.PI * 2;
        while (headingErr < -Math.PI) headingErr += Math.PI * 2;
        cmdYaw = Math.max(-0.95, Math.min(0.95, headingErr * 3.2));

        // High sustained forward cruise speed
        const alignmentFactor = Math.max(0.60, Math.cos(headingErr));
        cmdSurge = currentWp.speed * alignmentFactor;
      }

      // Active Gate Post Guard in FINAL mode
      if (this.simX >= gateX - 2.0 && this.simX <= gateX + 1.2) {
        const lateralOffset = this.simZ - gateZ;
        if (lateralOffset < -0.25) {
          cmdYaw += Math.min(0.45, (-0.25 - lateralOffset) * 2.5);
        } else if (lateralOffset > 0.25) {
          cmdYaw -= Math.min(0.45, (lateralOffset - 0.25) * 2.5);
        }
      }

      // Active Wall Repulsion Guard
      let wallSafetyYaw = 0;
      if (this.simZ < -5.5) {
        wallSafetyYaw = ((-5.5 - this.simZ) / 1.5) * 0.90;
      } else if (this.simZ > 5.5) {
        wallSafetyYaw = -((this.simZ - 5.5) / 1.5) * 0.90;
      }
      cmdYaw = Math.max(-0.95, Math.min(0.95, cmdYaw + wallSafetyYaw));

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
    const sysIdRes = sysIdEngine.step(input.surge || (rawSurge / 1.3), rawSurge, dt);
    const effectiveSurge = Math.max(-1.35, Math.min(1.35, rawSurge * 0.7 + (sysIdRes.velocity || 0) * 0.3));

    // Advance proposed position & orientation
    this.simHeading += (ctrl?.yaw || 0) * dt;

    // Get underwater current drift from hydrodynamics engine
    const currentDrift = hydrodynamicsEngine.getCurrentDrift();

    let proposedX = this.simX + (Math.cos(this.simHeading) * effectiveSurge - Math.sin(this.simHeading) * (ctrl?.sway || 0) + currentDrift.x) * dt;
    let proposedZ = this.simZ + (Math.sin(this.simHeading) * effectiveSurge + Math.cos(this.simHeading) * (ctrl?.sway || 0) + currentDrift.z) * dt;
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
  }
}

const mockRos = new MockRosConnection();
export default mockRos;
