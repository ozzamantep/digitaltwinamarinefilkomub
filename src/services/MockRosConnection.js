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
    } else    if (flightMode === 'QUALIFIKASI') {
      // =========================================================================
      // AUTONOMOUS QUALIFICATION STATE MACHINE (from qualification.py)
      // =========================================================================
      const obs = store.obstacles || { gate: { x: 4.0, z: 0.0 } };
      const TARGET_DEPTH = 0.75; // Optimal depth (Y = 1.25m): directly through gate center
      const GATE_X = obs.gate?.x ?? 4.0;
      const GATE_Z = obs.gate?.z ?? 0.0;

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

          // Target point is pre-gate window (GATE_X - 1.5, GATE_Z)
          const targetHeading = Math.atan2(GATE_Z - this.simZ, (GATE_X - 1.5) - this.simX);
          let headingErr = targetHeading - this.simHeading;
          while (headingErr > Math.PI) headingErr -= Math.PI * 2;
          while (headingErr < -Math.PI) headingErr += Math.PI * 2;
          cmdYaw = Math.max(-0.55, Math.min(0.55, headingErr * 2.2));

          if (this.simX >= GATE_X - 0.4) {
            this.qualPrevState = 4;
            this.qualState = 2;
            this.qualStateStartTime = t;
          }
          break;

        case 2: // State 2: Forward / Return through Gate
          cmdHeave = Math.max(-0.35, Math.min(0.35, (TARGET_DEPTH - this.simDepth) * 2.0));
          const elapsed = t - this.qualStateStartTime;

          if (!this.qualReturnPass) {
            statusText = `State 3/5: Passing Forward Through Gate (${elapsed.toFixed(1)}s / 6.0s)`;
            cmdSurge = 0.75;
            // Locked 0.0° heading through gate center window
            let straightErr = 0.0 - this.simHeading;
            while (straightErr > Math.PI) straightErr -= Math.PI * 2;
            while (straightErr < -Math.PI) straightErr += Math.PI * 2;
            const crossTrack = (GATE_Z - this.simZ) * 0.40;
            cmdYaw = Math.max(-0.35, Math.min(0.35, (straightErr + crossTrack) * 2.0));

            if (elapsed >= 5.5 || this.simX >= GATE_X + 2.5) {
              this.qualPrevState = 2;
              this.qualState = 3;
              this.qualStateStartTime = t;
              this.qualUTurnTargetYaw = Math.PI; // Exact 180° facing back
            }
          } else {
            // Return pass: First pass back through gate to GATE_X - 2.5m before heading to start dock!
            if (this.simX > GATE_X - 2.5) {
              statusText = `State 5/5: Gliding Back Through Gate (Z = ${GATE_Z.toFixed(1)}m)`;
              cmdSurge = 0.70;
              let backStraightErr = Math.PI - Math.abs(this.simHeading);
              let backErr = this.simHeading > 0 ? backStraightErr : -backStraightErr;
              const backCrossTrack = (this.simZ - GATE_Z) * 0.40;
              cmdYaw = Math.max(-0.4, Math.min(0.4, (backErr + backCrossTrack) * 2.0));
            } else {
              statusText = `State 5/5: Returning to Start Zone (${elapsed.toFixed(1)}s / 8.0s)`;
              cmdSurge = 0.75;
              const returnHeading = Math.atan2(2.0 - this.simZ, -11.0 - this.simX);
              let retErr = returnHeading - this.simHeading;
              while (retErr > Math.PI) retErr -= Math.PI * 2;
              while (retErr < -Math.PI) retErr += Math.PI * 2;
              cmdYaw = Math.max(-0.5, Math.min(0.5, retErr * 2.0));

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
      // AUTONOMOUS SAUVC FINAL MISSION (Dynamic Pure-Pursuit from Reactive Obstacle Map)
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

      const waypoints = [
        // 1. Approach Orange Flare (Safe distance 1.8m before obstacle)
        {
          id: 'orange_approach',
          name: '1/12: 🚀 Approach Orange Flare',
          x: orangeX - 1.8,
          z: orangeZ,
          targetDepth: 0.8,
          speed: 0.65,
          threshold: 0.70,
        },
        // 2. Safe Stand-off Hover / Inspection (1.8s hover, ZERO collision)
        {
          id: 'orange_inspect',
          name: '2/12: 🔍 Inspecting Orange Flare (Safe Stand-off: 1.8m)',
          x: orangeX - 1.8,
          z: orangeZ,
          targetDepth: 0.8,
          speed: 0.03,
          threshold: 0.60,
          inspectDuration: 1.8,
        },
        // 3. Smooth Starboard Bypass around Orange Flare
        {
          id: 'orange_bypass',
          name: '3/12: ↪️ Smooth Bypass around Orange Flare',
          x: orangeX + 1.5,
          z: orangeZ - 1.4,
          targetDepth: 0.8,
          speed: 0.70,
          threshold: 0.80,
        },
        // 4. Full-Body Ramming & Strike Blue Flare
        {
          id: 'blue_flare',
          name: '4/12: 💥 Full-Body Ram Blue Flare (Sebadan-badannya)',
          x: blueX,
          z: blueZ,
          targetDepth: 0.8,
          speed: 0.85,
          threshold: 0.35,
          hitFlare: 'blue',
        },
        // 5. Full-Body Ramming & Strike Red Flare
        {
          id: 'red_flare',
          name: '5/12: 💥 Full-Body Ram Red Flare (Sebadan-badannya)',
          x: redX,
          z: redZ,
          targetDepth: 0.8,
          speed: 0.85,
          threshold: 0.35,
          hitFlare: 'red',
        },
        // 6. Full-Body Ramming & Strike Yellow Flare
        {
          id: 'yellow_flare',
          name: '6/12: 💥 Full-Body Ram Yellow Flare (Sebadan-badannya)',
          x: yellowX,
          z: yellowZ,
          targetDepth: 0.8,
          speed: 0.85,
          threshold: 0.35,
          hitFlare: 'yellow',
        },
        // 7. Safe Arc Steering North Away from South Wall
        {
          id: 'gate_arc',
          name: '7/12: 🔄 Aligning Trajectory towards Gate',
          x: yellowX + 1.4,
          z: yellowZ + 2.2,
          targetDepth: 0.75,
          speed: 0.70,
          threshold: 0.80,
        },
        // 8. Pre-Gate Centerline Alignment Window (Locks onto Z = gateZ)
        {
          id: 'gate_align',
          name: `8/12: 🎯 Pre-Gate Alignment (Z = ${gateZ.toFixed(1)}m)`,
          x: gateX - 1.8,
          z: gateZ,
          targetDepth: 0.75,
          speed: 0.70,
          threshold: 0.80,
        },
        // 9. Straight & Confident Gate Transit (Locked 0.0° heading through center window)
        {
          id: 'gate_pass',
          name: `9/12: 🚪 Smooth Straight Gate Transit (${gateZ.toFixed(1)}m Aligned)`,
          x: gateX + 2.5,
          z: gateZ,
          targetDepth: 0.75,
          speed: 0.75,
          threshold: 0.85,
          gateTransit: true,
          gateTargetZ: gateZ,
        },
        // 10. Approach Red Target Drum (Carrying Ball with Gripper, Nose & Cam Tilt -35°)
        {
          id: 'drum_search',
          name: '10/12: 🎯 Approach Target Drum (Carrying Ball with Gripper)',
          x: drumX,
          z: drumZ,
          targetDepth: 0.95,
          speed: 0.50,
          threshold: 0.55,
        },
        // 11. Direct Payload Drop: Gripper opens smoothly and releases ball directly into drum!
        {
          id: 'drum_drop',
          name: '11/12: 🔴 Gripper Opens: Ball Dropped Directly into Drum!',
          x: drumX,
          z: drumZ,
          targetDepth: 0.88,
          speed: 0.02,
          threshold: 0.40,
        },
        // 12. Mission Complete - Ascend to Surface
        {
          id: 'mission_complete',
          name: '🏆 MISSION COMPLETE! Surfacing at End Zone',
          x: drumX + 1.0,
          z: 0.0,
          targetDepth: 0.15,
          speed: 0.25,
          threshold: 0.70,
        },
      ];

      const currentWp = waypoints[this.missionStage] || waypoints[waypoints.length - 1];
      const distToWp = Math.sqrt((currentWp.x - this.simX) ** 2 + (currentWp.z - this.simZ) ** 2);

      let cmdSurge = currentWp.speed;
      let cmdYaw = 0;

      // 1. Full-Body Flare Direct Ramming Knockdown (< 0.40m full hull contact)
      if (currentWp.hitFlare && distToWp < 0.40) {
        store.knockdownFlare(currentWp.hitFlare);
      }

      // 2. Orange Flare Safe Stand-off Hover/Inspect
      if (currentWp.id === 'orange_inspect') {
        if (!this.orangeInspectStartTime) {
          this.orangeInspectStartTime = t;
        }
        cmdSurge = 0.03; // Gentle hover holding position
        const elapsedInspect = t - this.orangeInspectStartTime;
        if (elapsedInspect >= (currentWp.inspectDuration || 1.8)) {
          this.missionStage++;
          this.orangeInspectStartTime = 0;
        }
      }

      // 3. Drum Approach Pitch Tilt & Direct Ball Drop
      if (currentWp.id === 'drum_search' || currentWp.id === 'drum_drop') {
        if (distToWp < 2.5) {
          const tiltFactor = Math.min(1.0, (2.5 - distToWp) / 1.2);
          targetPitchAngle = -30.0 * tiltFactor;
        }

        if (currentWp.id === 'drum_drop' && distToWp < 0.50) {
          cmdSurge = 0.02;
          if (!this.drumDropTriggered) {
            this.drumDropTriggered = true;
            this.drumWaitTime = t;
            store.setGripperState('OPEN');
            store.dropBallIntoDrum(drumX, drumZ);
          }

          if (t - this.drumWaitTime > 2.5) {
            this.missionStage++;
          }
        }
      }

      // 4. Normal Waypoint Advance
      if (
        currentWp.id !== 'orange_inspect' &&
        currentWp.id !== 'drum_drop' &&
        distToWp < currentWp.threshold &&
        this.missionStage < waypoints.length - 1
      ) {
        this.missionStage++;
      }

      // 5. Depth PID Command
      const depthErr = currentWp.targetDepth - this.simDepth;
      const cmdHeave = Math.max(-0.65, Math.min(0.65, depthErr * 1.6));

      // 6. Yaw Heading Steering & Speed Modulation
      if (currentWp.gateTransit) {
        // Special Gate Centerline Lock: keep heading strictly 0.0 rad with gentle cross-track correction towards gateTargetZ
        const tgtZ = currentWp.gateTargetZ ?? gateZ;
        let straightErr = 0.0 - this.simHeading;
        while (straightErr > Math.PI) straightErr -= Math.PI * 2;
        while (straightErr < -Math.PI) straightErr += Math.PI * 2;
        const crossTrackCorrection = (tgtZ - this.simZ) * 0.40;
        const combinedHeadingErr = straightErr + Math.max(-0.15, Math.min(0.15, crossTrackCorrection));
        cmdYaw = Math.max(-0.45, Math.min(0.45, combinedHeadingErr * 2.2));
        cmdSurge = currentWp.speed;
      } else {
        // Standard waypoint tracking with heading-alignment speed scaling
        const targetHeading = Math.atan2(currentWp.z - this.simZ, currentWp.x - this.simX);
        let headingErr = targetHeading - this.simHeading;
        while (headingErr > Math.PI) headingErr -= Math.PI * 2;
        while (headingErr < -Math.PI) headingErr += Math.PI * 2;
        cmdYaw = Math.max(-0.95, Math.min(0.95, headingErr * 2.6));

        // When heading error is large, slow down to turn sharply without overshooting!
        const alignmentFactor = Math.max(0.18, Math.cos(headingErr));
        cmdSurge = currentWp.speed * (alignmentFactor ** 2);
      }

      // Active Wall Safety Repulsion Barrier in FINAL mode (guarantees zero wall collisions)
      let wallSafetyYaw = 0;
      if (this.simZ < -5.0) {
        wallSafetyYaw = ((-5.0 - this.simZ) / 2.0) * 0.90;
      } else if (this.simZ > 5.0) {
        wallSafetyYaw = -((this.simZ - 5.0) / 2.0) * 0.90;
      }
      cmdYaw = Math.max(-0.95, Math.min(0.95, cmdYaw + wallSafetyYaw));

      let finalTarget = `[FINAL] ${currentWp.name}`;
      if (currentWp.id === 'drum_drop' && this.drumDropTriggered) {
        finalTarget = `[FINAL] 11/12: 🎯 BALL DROPPED INTO RED DRUM!`;
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
