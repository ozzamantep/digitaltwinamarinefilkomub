import { ParameterIdentifier } from '../src/dt-core/ParameterIdentifier.js';
import { HydrodynamicsEngine } from '../src/dt-core/HydrodynamicsEngine.js';

console.log('Testing ParameterIdentifier with Differential Evolution...');

// Generate 25 steps of synthetic flight data
const engine = new HydrodynamicsEngine();
let simState = { eta: [0, 0, 0.8, 0, 0, 0], nu: [0, 0, 0, 0, 0, 0] };
const dataset = [];

for (let i = 0; i < 25; i++) {
  const input = { surge: 0.8, sway: 0, heave: 0.2, yaw: 0.1 };
  const thrustScale = 0.5;
  const tau = [input.surge * thrustScale, input.sway * thrustScale, input.heave * thrustScale, 0, 0, input.yaw * thrustScale * 0.18];
  
  simState = engine.step(simState, tau, 0.05);
  
  dataset.push({
    time: i * 0.05,
    dt: 0.05,
    input,
    measured: {
      u: simState.nu[0] + (Math.random() - 0.5) * 0.01,
      v: simState.nu[1],
      w: simState.nu[2],
      yawRate: simState.nu[5],
      depth: simState.eta[2],
    }
  });
}

const pid = new ParameterIdentifier({ populationSize: 10, maxGenerations: 6 });
const res = await pid.identify(dataset);
console.log('Identification result:', res);
if (res.finalRMSE <= res.initialRMSE) {
  console.log('✅ PASS: Differential Evolution converged and reduced or maintained RMSE.');
} else {
  console.error('❌ FAIL: DE error increased.');
  process.exit(1);
}
