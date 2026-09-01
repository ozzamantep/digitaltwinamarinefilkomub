import { useFrame, useThree } from '@react-three/fiber';
import * as THREE from 'three';
import { useRef } from 'react';
import useVehicleStore from '../../store/vehicleStore';

export default function CameraController({ controlsRef }) {
  const { camera } = useThree();
  const position = useVehicleStore((s) => s.position);
  const headingRad = useVehicleStore((s) => s.headingRad);
  const cameraViewMode = useVehicleStore((s) => s.cameraViewMode);

  const currentCamPos = useRef(new THREE.Vector3(-11, 6.5, 9.5));
  const currentLookAt = useRef(new THREE.Vector3(-3, 1.1, 0));

  useFrame((_, delta) => {
    if (cameraViewMode === 'orbit') {
      if (controlsRef.current) {
        controlsRef.current.enabled = true;
      }
      return;
    }

    if (controlsRef.current) {
      controlsRef.current.enabled = false;
    }

    const pos = position || { x: -10, y: 1.1, z: 0 };
    const hRad = headingRad || 0;
    const cosH = Math.cos(hRad);
    const sinH = Math.sin(hRad);

    const store = useVehicleStore.getState();
    const euler = store.euler || { pitch: 0, roll: 0, yaw: 0 };
    const pitchRad = ((euler.pitch || 0) * Math.PI) / 180;

    let targetPos = new THREE.Vector3();
    let targetLook = new THREE.Vector3();

    if (cameraViewMode === 'fpv') {
      // FPV: Camera mounted directly at front camera dome (+X forward)
      targetPos.set(
        pos.x + cosH * 0.22,
        pos.y + 0.05,
        pos.z + sinH * 0.22
      );

      // Pitch down (negative pitch) tilts look-at vector DOWN towards the floor
      const forwardDist = 8.0 * Math.cos(pitchRad);
      const verticalOffset = 8.0 * Math.sin(pitchRad); // Correct downward look-at

      targetLook.set(
        targetPos.x + cosH * forwardDist,
        targetPos.y + verticalOffset,
        targetPos.z + sinH * forwardDist
      );
    } else if (cameraViewMode === 'chase') {
      // Chase: Camera follows behind and slightly above the submarine
      targetPos.set(
        pos.x - cosH * 1.6,
        pos.y + 0.55,
        pos.z - sinH * 1.6
      );

      targetLook.set(
        pos.x + cosH * 1.2,
        pos.y + 0.02,
        pos.z + sinH * 1.2
      );
    }

    // Smooth camera damping lerp
    const lerpFactor = Math.min(1, 8.0 * delta);
    currentCamPos.current.lerp(targetPos, lerpFactor);
    currentLookAt.current.lerp(targetLook, lerpFactor);

    camera.position.copy(currentCamPos.current);
    camera.lookAt(currentLookAt.current);
  });

  return null;
}
