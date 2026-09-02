import { useFrame, useThree } from '@react-three/fiber';
import * as THREE from 'three';
import { useRef } from 'react';
import useVehicleStore from '../../store/vehicleStore';

/**
 * Dynamic Camera Controller
 * Supports:
 * 1. 🌐 Orbit (Free camera view in the pool arena)
 * 2. 🚁 Chase (Tracks the AUV in 3D while allowing full 360° mouse drag orbit & zoom around the vehicle)
 * 3. 🎥 FPV (First-Person View from front dome with compass & HUD)
 */
export default function CameraController({ controlsRef }) {
  const { camera } = useThree();
  const position = useVehicleStore((s) => s.position);
  const headingRad = useVehicleStore((s) => s.headingRad);
  const cameraViewMode = useVehicleStore((s) => s.cameraViewMode);
  const cameraResetTrigger = useVehicleStore((s) => s.cameraResetTrigger);

  const prevMode = useRef(cameraViewMode);
  const prevTrigger = useRef(cameraResetTrigger);
  const lastSubPos = useRef(new THREE.Vector3(-11, 1.2, 2));
  const initializedChase = useRef(false);

  // FPV Smooth Lerp tracking vectors
  const currentFpvPos = useRef(new THREE.Vector3(-11, 1.2, 2));
  const currentFpvLook = useRef(new THREE.Vector3(-10, 1.2, 2));

  // TPP Third-Person Perspective (Strict Rear Follow - NO ORBIT)
  const currentTppPos = useRef(new THREE.Vector3(-13, 1.8, 2));
  const currentTppLook = useRef(new THREE.Vector3(-7, 1.2, 2));
  const initializedTpp = useRef(false);

  // Initialize or re-center Chase Camera directly behind the moving vehicle
  const resetChaseCamera = (pos, hRad) => {
    if (!controlsRef.current) return;
    const cosH = Math.cos(hRad);
    const sinH = Math.sin(hRad);

    const targetPos = new THREE.Vector3(
      pos.x - cosH * 1.8,
      pos.y + 0.65,
      pos.z - sinH * 1.8
    );
    const targetLook = new THREE.Vector3(pos.x, pos.y + 0.05, pos.z);

    camera.position.copy(targetPos);
    controlsRef.current.target.copy(targetLook);
    controlsRef.current.update();
    lastSubPos.current.set(pos.x, pos.y, pos.z);
    initializedChase.current = true;
  };

  useFrame((_, delta) => {
    const pos = position || { x: -11, y: 1.2, z: 2 };
    const hRad = headingRad || 0;
    const controls = controlsRef.current;

    const modeChanged = prevMode.current !== cameraViewMode;
    const triggerChanged = prevTrigger.current !== cameraResetTrigger;
    prevMode.current = cameraViewMode;
    prevTrigger.current = cameraResetTrigger;

    // =========================================================================
    // 1. FREE ORBIT MODE (Arena View)
    // =========================================================================
    if (cameraViewMode === 'orbit') {
      if (controls) {
        controls.enabled = true;
        controls.minDistance = 1.2;
        controls.maxDistance = 35;

        // Instant Focus on Vehicle when Reset/Focus triggered (D-Pad Down / R3)
        if (triggerChanged) {
          controls.target.set(pos.x, pos.y, pos.z);
          camera.position.set(pos.x - 3.5, pos.y + 2.5, pos.z + 3.5);
          controls.update();
        }

        // Apply Gamepad Continuous Orbit Rotation
        const gpOrbit = useVehicleStore.getState().gamepadCameraOrbit || { deltaAzimuth: 0, deltaElevation: 0 };
        if (gpOrbit.deltaAzimuth !== 0 || gpOrbit.deltaElevation !== 0) {
          controls.rotateLeft(gpOrbit.deltaAzimuth * delta * 2.2);
          controls.rotateUp(gpOrbit.deltaElevation * delta * 2.2);
          controls.update();
        }
      }
      initializedChase.current = false;
      return;
    }

    // =========================================================================
    // 2. FPV FIRST-PERSON VIEW MODE (Front Camera Dome)
    // =========================================================================
    if (cameraViewMode === 'fpv') {
      if (controls) {
        controls.enabled = false;
      }
      initializedChase.current = false;

      const cosH = Math.cos(hRad);
      const sinH = Math.sin(hRad);
      const store = useVehicleStore.getState();
      const euler = store.euler || { pitch: 0, roll: 0, yaw: 0 };
      const pitchRad = ((euler.pitch || 0) * Math.PI) / 180;

      const targetPos = new THREE.Vector3(
        pos.x + cosH * 0.22,
        pos.y + 0.05,
        pos.z + sinH * 0.22
      );

      const forwardDist = 8.0 * Math.cos(pitchRad);
      const verticalOffset = 8.0 * Math.sin(pitchRad);

      const targetLook = new THREE.Vector3(
        targetPos.x + cosH * forwardDist,
        targetPos.y + verticalOffset,
        targetPos.z + sinH * forwardDist
      );

      const lerpFactor = Math.min(1, 14.0 * delta);
      currentFpvPos.current.lerp(targetPos, lerpFactor);
      currentFpvLook.current.lerp(targetLook, lerpFactor);

      camera.position.copy(currentFpvPos.current);
      camera.lookAt(currentFpvLook.current);
      return;
    }

    // =========================================================================
    // 3. TPP THIRD-PERSON PERSPECTIVE (Locked Rear View - Strictly NO ORBIT)
    // =========================================================================
    if (cameraViewMode === 'tpp') {
      if (controls) {
        controls.enabled = false; // Disable orbit mouse/touch drag completely
      }
      initializedChase.current = false;

      const cosH = Math.cos(hRad);
      const sinH = Math.sin(hRad);
      const store = useVehicleStore.getState();
      const euler = store.euler || { pitch: 0, roll: 0, yaw: 0 };
      const pitchRad = ((euler.pitch || 0) * Math.PI) / 180;

      // 1.95m directly behind the stern along current heading, 0.58m elevated
      const chaseDistance = 1.95;
      const chaseHeight = 0.58;

      const targetCamPos = new THREE.Vector3(
        pos.x - cosH * chaseDistance,
        pos.y + chaseHeight - Math.sin(pitchRad) * 0.4,
        pos.z - sinH * chaseDistance
      );

      // Look target: 3.5m ahead of the ship in the forward heading direction
      const lookDistance = 3.5;
      const targetLook = new THREE.Vector3(
        pos.x + cosH * lookDistance,
        pos.y + 0.12 + Math.sin(pitchRad) * 0.8,
        pos.z + sinH * lookDistance
      );

      // Snap on first activation to prevent camera gliding from across the pool
      if (modeChanged || triggerChanged || !initializedTpp.current) {
        currentTppPos.current.copy(targetCamPos);
        currentTppLook.current.copy(targetLook);
        initializedTpp.current = true;
      } else {
        const lerpFactor = Math.min(1, 10.0 * delta);
        currentTppPos.current.lerp(targetCamPos, lerpFactor);
        currentTppLook.current.lerp(targetLook, lerpFactor);
      }

      camera.position.copy(currentTppPos.current);
      camera.lookAt(currentTppLook.current);
      return;
    }

    // Reset TPP initialized state when outside TPP mode
    initializedTpp.current = false;

    // =========================================================================
    // 4. CHASE ORBIT MODE (Follows AUV + full 360° Gamepad / Mouse Orbit & Zoom)
    // =========================================================================
    if (cameraViewMode === 'chase') {
      if (!controls) return;

      controls.enabled = true;
      controls.minDistance = 0.5;
      controls.maxDistance = 20;

      // On entering chase mode or on clicking Reset Focus (D-Pad Down / R3)
      if (modeChanged || triggerChanged || !initializedChase.current) {
        resetChaseCamera(pos, hRad);
        return;
      }

      // Apply Gamepad Continuous Orbit Rotation around moving AUV
      const gpOrbit = useVehicleStore.getState().gamepadCameraOrbit || { deltaAzimuth: 0, deltaElevation: 0 };
      if (gpOrbit.deltaAzimuth !== 0 || gpOrbit.deltaElevation !== 0) {
        controls.rotateLeft(gpOrbit.deltaAzimuth * delta * 2.2);
        controls.rotateUp(gpOrbit.deltaElevation * delta * 2.2);
        controls.update();
      }

      // Delta translation: seamlessly moves camera and OrbitControls target
      // with the submarine, while preserving the user's manual orbit angle & zoom distance!
      const deltaX = pos.x - lastSubPos.current.x;
      const deltaY = pos.y - lastSubPos.current.y;
      const deltaZ = pos.z - lastSubPos.current.z;

      if (deltaX !== 0 || deltaY !== 0 || deltaZ !== 0) {
        camera.position.x += deltaX;
        camera.position.y += deltaY;
        camera.position.z += deltaZ;

        controls.target.x += deltaX;
        controls.target.y += deltaY;
        controls.target.z += deltaZ;

        controls.update();
      }

      lastSubPos.current.set(pos.x, pos.y, pos.z);
    }
  });

  return null;
}
