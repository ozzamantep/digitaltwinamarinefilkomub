export class CompetitionScoringEngine {
  scoreMission(samples = []) {
    const events = samples.flatMap((sample) => sample.events || []);
    const count = (event) => events.filter((value) => value === event).length;
    const gatePasses = count('GATE_PASSED');
    const flareHits = events.filter((event) => event.startsWith('FLARE_') && event.endsWith('_HIT')).length;
    const payloadDrops = count('PAYLOAD_DELIVERED');
    const collisions = events.filter((event) => event.includes('COLLISION')).length;
    const safetyEvents = events.filter((event) => event.includes('FLOOR_BRAKE') || event.includes('FAULT')).length;
    const durationSec = samples.length ? samples[samples.length - 1].t || 0 : 0;
    const taskPoints = gatePasses * 100 + flareHits * 50 + payloadDrops * 200;
    const timeBonus = Math.max(0, Math.round(300 - durationSec));
    const penalties = collisions * 50 + safetyEvents * 10;
    return { durationSec, gatePasses, flareHits, payloadDrops, collisions, safetyEvents, taskPoints, timeBonus, penalties, totalScore: Math.max(0, taskPoints + timeBonus - penalties) };
  }
}

const competitionScoringEngine = new CompetitionScoringEngine();
export default competitionScoringEngine;