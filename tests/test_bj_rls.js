import { SystemIdentificationEngine } from '../frontend/src/services/SystemIdentificationEngine.js';

const engine = new SystemIdentificationEngine();
const armaxBefore = JSON.stringify(engine.models.ARMAX);
const bjBefore = JSON.stringify(engine.models.BJ);

engine.setRLSEnabled(true);
engine.w_hist = [0.55, 0.50, 0.30, 0];
engine.u_hist = [0.90, 0.80, 0.60, 0];
engine.e_hist = [0, 0.02, 0, 0];
engine.v_hist = [0.05, 0.04, 0, 0];
engine.rlsUpdate(0.78);

console.assert(engine.selectedModel === 'BJ', 'Box-Jenkins must be the default model');
console.assert(engine.rls.theta.length === 6, 'BJ RLS must estimate six parameters');
console.assert(engine.rls.P.length === 6, 'BJ RLS covariance must be 6x6');
console.assert(JSON.stringify(engine.models.BJ) !== bjBefore, 'BJ parameters must adapt');
console.assert(JSON.stringify(engine.models.ARMAX) === armaxBefore, 'ARMAX must not be adapted');
console.assert(engine.rls.sampleCount === 1, 'One measurement must produce one update');

console.log('PASS: Box-Jenkins pseudo-linear RLS updated all model state consistently.');