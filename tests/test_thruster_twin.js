import { ThrusterTwinEngine } from '../frontend/src/dt-core/ThrusterTwinEngine.js';

console.log('Testing ThrusterTwinEngine...');

const engine = new ThrusterTwinEngine({ voltage: 16.0 });

// 1. Test Neutral deadband
const deadbandPwm = engine.pwmToNominalRpm(1500);
console.assert(deadbandPwm === 0, `Expected 0 RPM at 1500 PWM, got ${deadbandPwm}`);
const deadbandThrust = engine.rpmToThrust(deadbandPwm);
console.assert(deadbandThrust === 0, `Expected 0 N at 0 RPM, got ${deadbandThrust}`);

// 2. Test Forward 1700 µs
const fwdRpm = engine.pwmToNominalRpm(1700);
const fwdThrust = engine.rpmToThrust(fwdRpm);
console.log(`Forward 1700 µs: RPM = ${fwdRpm.toFixed(1)}, Thrust = ${fwdThrust.toFixed(2)} N`);
console.assert(fwdRpm > 1500 && fwdRpm < 2200, `Forward RPM out of range: ${fwdRpm}`);
console.assert(fwdThrust > 10.0 && fwdThrust < 20.0, `Forward thrust out of range: ${fwdThrust}`);

// 3. Test Reverse 1300 µs
const revRpm = engine.pwmToNominalRpm(1300);
const revThrust = engine.rpmToThrust(revRpm);
console.log(`Reverse 1300 µs: RPM = ${revRpm.toFixed(1)}, Thrust = ${revThrust.toFixed(2)} N`);
console.assert(revRpm < -1000 && revRpm > -2000, `Reverse RPM out of range: ${revRpm}`);
console.assert(revThrust < -5.0 && revThrust > -15.0, `Reverse thrust out of range: ${revThrust}`);

// 4. Test safety clamp: step() must limit commands to 1300-1600 µs
const clampedResult = engine.step(1900, 0.05);
console.assert(clampedResult.pwm === 1600, `Expected step(1900) clamped to 1600, got ${clampedResult.pwm}`);
const clampedRev = engine.step(1100, 0.05);
console.assert(clampedRev.pwm === 1300, `Expected step(1100) clamped to 1300, got ${clampedRev.pwm}`);

// 5. Test RLS online adaptation (within safe PWM range)
const safeRpm = engine.pwmToNominalRpm(1600);
const safeThrust = engine.rpmToThrust(safeRpm);
const initialKt = engine.estimatedKt;
// Simulate measured physical thrust is 15% lower due to propeller blade wear
const wornPhysicalThrust = safeThrust * 0.85;

for (let i = 0; i < 30; i++) {
  engine.step(1600, 0.05, wornPhysicalThrust);
}

console.log(`Initial Kt: ${(initialKt * 1e6).toFixed(3)} x 10^-6`);
console.log(`Adapted Kt after 30 steps of 85% thrust: ${(engine.estimatedKt * 1e6).toFixed(3)} x 10^-6`);
console.assert(engine.estimatedKt < initialKt, `RLS failed to adapt downward for lower thrust`);
console.log('✅ ThrusterTwinEngine tests PASSED!');
