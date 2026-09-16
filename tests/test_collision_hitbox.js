import assert from 'node:assert/strict';
import useVehicleStore from '../src/store/vehicleStore.js';
import collisionEngine from '../src/services/SubseaCollisionEngine.js';

const store = useVehicleStore.getState();
const blueFlare = { ...store.obstacles.blue_flare, x: 0, z: 0, y: 0.75, width: 0.35, height: 1.5 };

useVehicleStore.setState({
  obstacles: { ...store.obstacles, blue_flare: blueFlare },
  flareStrategies: { ...store.flareStrategies, blue_flare: 'TABRAK' },
  flaresFallen: { ...store.flaresFallen, blue: false },
});

collisionEngine.resolveCollision(-0.50, 0, 0.85, 0, 0, 0);
assert.equal(useVehicleStore.getState().flaresFallen.blue, false, 'Flare fell before hull contact');

collisionEngine.resolveCollision(-0.46, 0, 0.85, 0, 0, 0);
assert.equal(useVehicleStore.getState().flaresFallen.blue, true, 'Flare did not fall on bow contact');

useVehicleStore.setState({
  flaresFallen: { ...useVehicleStore.getState().flaresFallen, blue: false },
});
collisionEngine.resolveCollision(0.30, 0, 0.85, 0, 0, 0, -0.80, 0);
assert.equal(useVehicleStore.getState().flaresFallen.blue, true, 'High-speed hull sweep missed flare contact');

useVehicleStore.setState({
  flaresFallen: { ...useVehicleStore.getState().flaresFallen, blue: false },
});
collisionEngine.resolveCollision(-0.40, 0, 0.85, 0, 0, Math.PI / 2);
assert.equal(useVehicleStore.getState().flaresFallen.blue, false, 'Rotated narrow side used the wrong extent');

collisionEngine.resolveCollision(-0.46, 0, 0.10, 0, 0, 0);
assert.equal(useVehicleStore.getState().flaresFallen.blue, false, 'Flare fell without vertical overlap');

const wallContact = collisionEngine.resolveCollision(11.9, 0, 0.85, 0, 0, 0);
assert.ok(wallContact.x <= 12 - collisionEngine.halfLength, 'Bow crossed pool wall');

console.log('PASS: oriented hull hitbox requires real horizontal and vertical contact.');