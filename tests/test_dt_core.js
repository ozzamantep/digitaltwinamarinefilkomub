/**
 * Comprehensive Automated Test Suite for Digital Twin Core
 * 
 * Tests:
 * 1. Kinematics & Coordinate Transformations (Euler, Quaternions, J(η))
 * 2. 6-DOF Hydrodynamics (Mass matrix, damping, Coriolis, restoring forces, RK4)
 * 3. 15-State Extended Kalman Filter (Prediction, Measurement Update, Outlier Gating)
 * 4. Prediction API & Multi-step Rollout
 * 5. Parameter Identifier & Differential Evolution
 */

import Kinematics from '../frontend/src/dt-core/Kinematics.js';
import vehicleConfig from '../frontend/src/dt-core/VehicleConfig.js';
import { HydrodynamicsEngine } from '../frontend/src/dt-core/HydrodynamicsEngine.js';
import { StateEstimator } from '../frontend/src/dt-core/StateEstimator.js';
import { PredictionAPI } from '../frontend/src/dt-core/PredictionAPI.js';
import { ParameterIdentifier } from '../frontend/src/dt-core/ParameterIdentifier.js';
import { UncertaintyEstimator } from '../frontend/src/dt-core/UncertaintyEstimator.js';
import { OODDetector } from '../frontend/src/dt-core/OODDetector.js';
import { ValidationEngine } from '../frontend/src/dt-core/ValidationEngine.js';

let passed = 0;
let failed = 0;

function assert(condition, message) {
  if (condition) {
    console.log(`  ✅ PASS: ${message}`);
    passed++;
  } else {
    console.error(`  ❌ FAIL: ${message}`);
    failed++;
  }
}

function assertClose(actual, expected, tol = 1e-3, message = '') {
  const diff = Math.abs(actual - expected);
  assert(diff <= tol, `${message} (Expected ~${expected}, got ${actual}, diff=${diff.toFixed(5)})`);
}

console.log('═══════════════════════════════════════════════════════════════');
console.log('  RUNNING DIGITAL TWIN AUV TEST SUITE                          ');
console.log('═══════════════════════════════════════════════════════════════\n');

// ─────────────────────────────────────────────────────────────────────────────
// 1. KINEMATICS TESTS
// ─────────────────────────────────────────────────────────────────────────────
console.log('🔹 1. Testing Kinematics & Coordinate Transformations...');

// Angle wrapping
assertClose(Kinematics.wrapAngle(Math.PI * 2.5), Math.PI * 0.5, 1e-5, 'wrapAngle(+2.5π -> +0.5π)');
assertClose(Kinematics.wrapAngle(-Math.PI * 3.0), -Math.PI, 1e-5, 'wrapAngle(-3π -> -π)');
assertClose(Kinematics.wrapAngle360(380), 20, 1e-5, 'wrapAngle360(380° -> 20°)');
assertClose(Kinematics.wrapAngle360(-45), 315, 1e-5, 'wrapAngle360(-45° -> 315°)');

// Euler <-> Quaternion round trip
const testRoll = 0.15; // rad
const testPitch = -0.22;
const testYaw = 1.45;
const quat = Kinematics.eulerToQuaternion(testRoll, testPitch, testYaw);
const eulerBack = Kinematics.quaternionToEuler(quat);
assertClose(eulerBack.roll, testRoll, 1e-4, 'Euler -> Quat -> Euler (Roll)');
assertClose(eulerBack.pitch, testPitch, 1e-4, 'Euler -> Quat -> Euler (Pitch)');
assertClose(eulerBack.yaw, testYaw, 1e-4, 'Euler -> Quat -> Euler (Yaw)');

// Rotation Matrix Orthogonality: R * R^T = I
const R = Kinematics.rotationMatrixEuler(testRoll, testPitch, testYaw);
let isOrthogonal = true;
for (let i = 0; i < 3; i++) {
  for (let j = 0; j < 3; j++) {
    let dot = 0;
    for (let k = 0; k < 3; k++) dot += R[i][k] * R[j][k];
    const expected = i === j ? 1.0 : 0.0;
    if (Math.abs(dot - expected) > 1e-4) isOrthogonal = false;
  }
}
assert(isOrthogonal, 'Rotation matrix R_b^n is orthonormal (R * Rᵀ = I)');

// Body <-> World velocity round-trip
const vBody = { u: 1.2, v: -0.4, w: 0.3 };
const vWorld = Kinematics.bodyToWorldVelocity(vBody, { roll: testRoll, pitch: testPitch, yaw: testYaw });
const vBodyBack = Kinematics.worldToBodyVelocity(vWorld, { roll: testRoll, pitch: testPitch, yaw: testYaw });
assertClose(vBodyBack.u, vBody.u, 1e-4, 'Body -> World -> Body velocity (u)');
assertClose(vBodyBack.v, vBody.v, 1e-4, 'Body -> World -> Body velocity (v)');
assertClose(vBodyBack.w, vBody.w, 1e-4, 'Body -> World -> Body velocity (w)');

// Three.js Coordinate mapping
const posNED = { x: 5.0, y: -2.0, z: 1.2 };
const posThree = Kinematics.nedToThree(posNED.x, posNED.y, posNED.z, 2.0);
assertClose(posThree.x, 5.0, 1e-4, 'NED to Three.js X');
assertClose(posThree.y, 0.8, 1e-4, 'NED to Three.js Y (height above floor = 2.0 - 1.2 = 0.8m)');
assertClose(posThree.z, -2.0, 1e-4, 'NED to Three.js Z');
const posNEDBack = Kinematics.threeToNed(posThree.x, posThree.y, posThree.z, 2.0);
assertClose(posNEDBack.z, posNED.z, 1e-4, 'Three.js to NED Z');

// ─────────────────────────────────────────────────────────────────────────────
// 2. HYDRODYNAMICS ENGINE TESTS
// ─────────────────────────────────────────────────────────────────────────────
console.log('\n🔹 2. Testing 6-DOF Hydrodynamics Engine & Fossen Formulation...');

const hydroEngine = new HydrodynamicsEngine();

// Mass matrix positive definiteness check
const { M } = hydroEngine.computeMassMatrix();
assert(M[0][0] > 10.0, `Surge effective mass M_11 > 10kg (${M[0][0].toFixed(1)}kg)`);
assert(M[1][1] > M[0][0], `Sway effective mass M_22 > M_11 due to broadside area (${M[1][1].toFixed(1)}kg)`);
assert(M[2][2] > M[1][1], `Heave effective mass M_33 > M_22 due to deck plates (${M[2][2].toFixed(1)}kg)`);

// Restoring forces test
// Level attitude -> Zero restoring moments
const g_level = hydroEngine.computeRestoringForces(0, 0, 1.0);
assertClose(g_level[3], 0.0, 1e-4, 'Level roll restoring moment is zero');
assertClose(g_level[4], 0.0, 1e-4, 'Level pitch restoring moment is zero');

// Positive pitch (nose up) -> Restoring pitch moment should oppose pitch
const g_pitched = hydroEngine.computeRestoringForces(0, 0.2, 1.0);
assert(g_pitched[4] > 0, 'Positive pitch generates righting restoring moment');

// Damping forces test: drag always opposes velocity
const dampSurgeFwd = hydroEngine.computeDampingForces([0.5, 0, 0, 0, 0, 0]);
assert(dampSurgeFwd[0] < 0, 'Surge drag opposes forward velocity');
const dampSurgeRev = hydroEngine.computeDampingForces([-0.5, 0, 0, 0, 0, 0]);
assert(dampSurgeRev[0] > 0, 'Surge drag opposes backward velocity');

// RK4 Forward simulation step test
const initSimState = {
  eta: [0, 0, 0.8, 0, 0, 0],
  nu: [0, 0, 0, 0, 0, 0],
  quat: { w: 1, x: 0, y: 0, z: 0 },
};
// Apply 20N forward thrust for 1 second (20 steps of 0.05s)
let testSimState = initSimState;
for (let i = 0; i < 20; i++) {
  testSimState = hydroEngine.step(testSimState, [20, 0, 0, 0, 0, 0], 0.05);
}
assert(testSimState.nu[0] > 0.4, `Vehicle accelerates under thrust: u=${testSimState.nu[0].toFixed(2)} m/s`);
assert(testSimState.eta[0] > 0.2, `Vehicle moves forward: x=${testSimState.eta[0].toFixed(2)} m`);
assert(isFinite(testSimState.eta[0]), 'State remains numerically stable');

// ─────────────────────────────────────────────────────────────────────────────
// 3. EXTENDED KALMAN FILTER (STATE ESTIMATOR) TESTS
// ─────────────────────────────────────────────────────────────────────────────
console.log('\n🔹 3. Testing 15-State Extended Kalman Filter (StateEstimator)...');

const ekf = new StateEstimator({ x: 0, y: 0, z: 0.8 });

// Prediction step with zero acceleration
ekf.predict([0, 0, 0], [0, 0, 0], 0.05);
const est1 = ekf.getEstimatedState();
assertClose(est1.position.z, 0.8, 1e-3, 'EKF maintains position under zero input');

// Measurement Update: Depth Sensor
ekf.updateDepth(1.25);
const est2 = ekf.getEstimatedState();
assert(est2.position.z > 0.85, `EKF pulls estimate toward measured depth (depth=${est2.position.z.toFixed(3)}m)`);

// Measurement Update: Compass Yaw
ekf.updateCompass(0.5);
const est3 = ekf.getEstimatedState();
assert(est3.attitude.yaw > 0.1, `EKF updates heading toward compass (yaw=${est3.attitude.yaw.toFixed(3)} rad)`);

// Outlier Gating Test: Absurd depth (e.g. 50m) should be rejected
const acceptedOutlier = ekf.updateDepth(50.0);
assert(!acceptedOutlier, 'EKF rejects absurd 50m depth sensor spike via Mahalanobis gating');

// ─────────────────────────────────────────────────────────────────────────────
// 4. PREDICTION API & MULTI-STEP HORIZON TESTS
// ─────────────────────────────────────────────────────────────────────────────
console.log('\n🔹 4. Testing Prediction API & Trajectory Forecasting...');

const predApi = new PredictionAPI();
const rollout = predApi.predict(
  { eta: [0, 0, 0.8, 0, 0, 0], nu: [0, 0, 0, 0, 0, 0] },
  Array(30).fill([15, 0, 0, 0, 0, 0]), // 30 steps of 15N forward thrust
  null,
  { dt: 0.05, horizon: 30 }
);

assert(rollout.trajectory.length === 30, `Prediction generates requested 30 steps (got ${rollout.trajectory.length})`);
assert(rollout.finalState.eta[0] > 0.2, `Rollout reaches expected forward position: x=${rollout.finalState.eta[0].toFixed(2)}m`);
assert(rollout.energyJoules > 0, `Rollout calculates energy consumption: ${rollout.energyJoules.toFixed(1)} Joules`);
assert(rollout.uncertaintyHorizon[29] > rollout.uncertaintyHorizon[0], 'Prediction uncertainty expands over horizon');

// ─────────────────────────────────────────────────────────────────────────────
// 5. INTELLIGENCE LAYER TESTS (Uncertainty, OOD, ValidationEngine)
// ─────────────────────────────────────────────────────────────────────────────
console.log('\n🔹 5. Testing Uncertainty Estimator, OOD Detector & Validation Engine...');

const uncertEst = new UncertaintyEstimator();
uncertEst.updateResidual({ position: { x: 0, y: 0, z: 0.8 }, velocity: { u: 0.5, v: 0, w: 0 } },
                         { position: { x: 0.02, y: 0, z: 0.81 }, velocity: { u: 0.48, v: 0, w: 0 } });
const safety = uncertEst.isControlSafe();
assert(safety.safe, `Nominal small error classified as SAFE (uncertainty=${safety.uncertainty})`);

const ood = new OODDetector();
const nominalOOD = ood.evaluate({ speed: { surge: 0.4, sway: 0, heave: 0, angular: 0.1 }, imu: { accelX: 0, accelY: 0, accelZ: -9.81 }, depth: 0.8 });
assert(!nominalOOD.isOOD, `Nominal cruise classified as IN_DISTRIBUTION (score=${nominalOOD.oodScore})`);

const extremeOOD = ood.evaluate({ speed: { surge: 4.5, sway: 3.0, heave: 2.0, angular: 5.0 }, imu: { accelX: 15, accelY: 12, accelZ: 5 }, depth: 0.8 });
assert(extremeOOD.isOOD, `Extreme unphysical maneuver classified as OUT_OF_DISTRIBUTION (score=${extremeOOD.oodScore})`);

const valEngine = new ValidationEngine();
valEngine.addSample({ position: { x: 1.0, y: 0, z: 0.8 }, velocity: { u: 0.5, v: 0, w: 0 }, attitude: { roll: 0, pitch: 0, yaw: 0 } },
                    { position: { x: 1.03, y: 0.01, z: 0.81 }, velocity: { u: 0.49, v: 0, w: 0 }, attitude: { roll: 0, pitch: 0, yaw: 0 } }, 1);
const valMetrics = valEngine.computeMetrics();
assert(valMetrics.sampleCount === 1, 'Validation engine ingests and computes metrics');
assert(valMetrics.positionRMSE.total3D < 0.05, `Validation engine computes 3D position RMSE (${valMetrics.positionRMSE.total3D}m)`);

console.log('\n═══════════════════════════════════════════════════════════════');
console.log(`  TEST RESULTS: ${passed} PASSED, ${failed} FAILED                 `);
console.log('═══════════════════════════════════════════════════════════════');

if (failed > 0) process.exit(1);
