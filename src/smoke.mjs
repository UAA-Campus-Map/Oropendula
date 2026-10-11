/**
 * Smoke test for the tiered campus router, run against the real building
 * JSONs in this directory.  Usage:  node smoke.mjs
 */
import { readFileSync } from 'fs';
import { Worker } from 'node:worker_threads';
import { availableParallelism } from 'node:os';
import { buildGraph, buildGraphState, graphFromState } from './navgraph.mjs';
import { extractPortals, buildCampusRouter, buildOutdoorGraph, buildObstacleIndex, shapeSegments, cachePortals, portalsFromCache } from './campus.mjs';
import { loadOsmShapes } from './osm.mjs';
import { pmtilesGetZxy } from './pmtiles-node.mjs';

import { DXF_SCALE, metresPerDegree, makeToLL } from './geo.mjs';
import { StepDetector, HeadingTracker, StepLengthModel, WalkCalibrator, wrapAngle, headingFromOrientation, createMotion } from './motion.mjs';
import { ParticleFilter, corridorAxis } from './locate.mjs';
import { PositionArbiter } from './arbiter.mjs';

// Registrations are the shared buildings-config.json (produced by
// import.html); smoke reads the same file index.html fetches.
const CONFIG = JSON.parse(readFileSync(new URL('./buildings-config.json', import.meta.url), 'utf8'));
// Flat per-building shape the tests below use; registration itself is
// resolved per floor through geo.mjs makeToLL.
const BUILDING_CONFIGS = CONFIG.map((b) => {
  const f = b.floors[0];
  const off = f.offset ?? b.offset ?? { x: 0, y: 0, rot: 0 };
  return {
    id: b.id,
    file: f.url.replace(/^\//, ''),
    groundFloor: b.groundFloor,
    origin: f.origin ?? b.origin,
    offset: { x: off.x, y: off.y },
    rot: off.rot,
    floor: f,
    building: b,
  };
});

function makeTransform(cfg) {
  return makeToLL(cfg.building, cfg.floor);
}

let failures = 0;
function check(name, cond, detail = '') {
  const ok = cond ? 'PASS' : 'FAIL';
  if (!cond) failures++;
  console.log(`  [${ok}] ${name}${detail ? ' - ' + detail : ''}`);
}

// SMOKE_TIMING=1 prefixes every line with elapsed seconds - run it when
// hunting for the slow section. Off by default so the gate's output stays
// clean.
if (process.env.SMOKE_TIMING) {
  const T0 = performance.now();
  const origLog = console.log.bind(console);
  console.log = (...a) =>
    origLog(`[${((performance.now() - T0) / 1000).toFixed(1).padStart(5)}s]`, ...a);
}

/**
 * Per-floor basis: JSON parse, wall segments, navGraph, fresh portal set.
 * These are independent per floor and dominated by CPU (buildGraph,
 * extractPortals), so they run across worker threads - this was ~9 s of the
 * suite's ~40 s single-threaded. Set SMOKE_NO_WORKERS=1 to run inline.
 */
function basisFromJson(cfg, floor, data) {
  const toLL = makeToLL(cfg, floor);
  // Same wall filter as index.html's loadFloor() (case-insensitive: AS106
  // writes WINDOWS in caps).
  const walls = data.shapes.filter(s => ['walls', 'windows'].includes(s.layer.toLowerCase()));
  const lines = walls.flatMap(s =>
    shapeSegments(s).map(([a, b]) => [toLL(a[0], a[1]), toLL(b[0], b[1])]));
  return {
    floor, toLL, lines,
    navGraph: buildGraph(lines),
    freshPortals: extractPortals(data.shapes, toLL),
    textLines: data.text_lines ?? [],
  };
}

async function buildFloorBasis() {
  const jobs = [];
  for (let i = 0; i < CONFIG.length; i++) {
    for (let k = 0; k < CONFIG[i].floors.length; k++) {
      jobs.push({ i, k, cfg: CONFIG[i], floor: CONFIG[i].floors[k] });
    }
  }
  if (process.env.SMOKE_NO_WORKERS) {
    return CONFIG.map(cfg => cfg.floors.map(floor => {
      const data = JSON.parse(readFileSync(new URL(floor.url.replace(/^\//, ''), import.meta.url), 'utf8'));
      return basisFromJson(cfg, floor, data);
    }));
  }
  const basis = CONFIG.map(cfg => cfg.floors.map(() => null));
  const concurrency = Math.max(1, Math.min(8, (availableParallelism?.() ?? 4) - 1));
  let next = 0;
  await Promise.all(Array.from({ length: concurrency }, () => (async () => {
    const worker = new Worker(new URL('./basis-worker.mjs', import.meta.url));
    try {
      for (;;) {
        const job = jobs[next++];
        if (!job) break;
        const result = await new Promise((resolve, reject) => {
          const onMessage = m => { cleanup(); resolve(m); };
          const onError = e => { cleanup(); reject(e); };
          const cleanup = () => {
            worker.off('message', onMessage);
            worker.off('error', onError);
          };
          worker.on('message', onMessage);
          worker.on('error', onError);
          worker.postMessage({ cfg: job.cfg, floor: job.floor });
        });
        if (!result.ok) throw new Error(`basis worker: ${result.error}`);
        basis[job.i][job.k] = {
          floor: job.floor,
          // toLL is a function - rebuild it here rather than serialising it.
          toLL: makeToLL(job.cfg, job.floor),
          lines: result.lines,
          navGraph: graphFromState(result.state),
          freshPortals: result.freshPortals,
          textLines: result.textLines,
        };
      }
    } finally {
      await worker.terminate();
    }
  })()));
  return basis;
}

console.log('Loading buildings...');
const t0 = performance.now();
// ONE JSON parse + wall pass + buildGraph + fresh portal extraction per floor,
// shared by every test group below (built in parallel workers). The suite used
// to redo this per group - the multi-floor block rebuilt every navGraph a
// second time for its cached-router comparison.
const floorBasis = await buildFloorBasis();

// The older tests deliberately work on a ground-floor-only campus.
const buildings = CONFIG.map((cfg, i) => {
  const { navGraph, lines, freshPortals } = floorBasis[i][0];
  console.log(`  ${cfg.id}: ${navGraph.stats.nodes} nodes, ${navGraph.stats.edges} edges, ` +
    `portals: ${freshPortals.elevators.length} elevators, ${freshPortals.stairs.length} stairs, ${freshPortals.exits.length} exits`);
  return {
    id: cfg.id,
    groundFloor: cfg.groundFloor,
    floors: [{
      floorId: cfg.groundFloor,
      navGraph,
      // lon/lat wall segments - the router keeps outdoor edges out of the
      // building footprint with these.
      obstacleLines: lines,
      elevators: freshPortals.elevators,
      stairs: freshPortals.stairs,
      exits: freshPortals.exits,
    }],
  };
});
console.log(`Loaded in ${Math.round(performance.now() - t0)}ms\n`);

// The shipped OSM path cache. index.html loads this (or decodes pmtiles live)
// and the production router walks the network; the full-campus router below
// uses the same shapes so its multi-floor routes model the deployed app
// rather than straight-line outdoor legs.
let osmCache = null;
try { osmCache = JSON.parse(readFileSync(new URL('./osm-paths.json', import.meta.url), 'utf8')); }
catch { /* optional; the OSM cache tests below report it */ }

// shapeSegments must read the true vertices in `items`, never the bbox
// diagonal: a rotated wall run drawn as its bbox is a diagonal slash across
// the corner it really turns.
{
  const line2 = shapeSegments({
    bbox: [0, 0, 3, 4], items: [{op: 'line', points: [[3, 0], [0, 4]]}],
  });
  check('shapeSegments: line keeps true endpoints',
    line2.length === 1 && line2[0][0][0] === 3 && line2[0][1][1] === 4);
  const multi = shapeSegments({
    bbox: [0, 0, 4, 2], items: [{op: 'line', points: [[0, 0], [2, 1], [4, 2]]}],
  });
  check('shapeSegments: multi-point item chains segments', multi.length === 2);
  const open = shapeSegments({
    bbox: [0, 0, 2, 2], items: [{op: 'polyline', points: [[0, 0], [2, 0], [2, 2]]}],
  });
  check('shapeSegments: open polyline does not wrap', open.length === 2);
  const closed = shapeSegments({
    bbox: [0, 0, 2, 2], closed: true,
    items: [{op: 'polyline', points: [[0, 0], [2, 0], [2, 2]]}],
  });
  check('shapeSegments: closed polyline wraps',
    closed.length === 3 && closed[2][1][0] === 0 && closed[2][1][1] === 0);
  const arc = shapeSegments({
    bbox: [0, 0, 2, 2], items: [{op: 'arc', center: [1, 1], radius: 1, start_angle: 0, end_angle: 90}],
  });
  check('shapeSegments: arc items are skipped, not flattened', arc.length === 0);
  check('shapeSegments: empty shape yields nothing', shapeSegments({bbox: [0, 0, 1, 1]}).length === 0);
}

// The PDR particle filter relies on these O(1) accessors staying consistent
// with the router's clearance raster.
console.log('\nWalkability accessor:');
{
  let falseOnWall = 0, freeRecovered = 0, tested = 0, maxRecoverM = 0;
  for (const b of buildings) {
    const g = b.floors[0].navGraph;
    const [lon, lat] = b.floors[0].obstacleLines[0][0];   // a vertex on a wall centreline
    const mpd = metresPerDegree(lat);
    tested++;
    if (!g.isWalkable([lon, lat])) falseOnWall++;
    const near = g.nearestWalkable([lon, lat]);
    if (near && g.isWalkable(near)) {
      freeRecovered++;
      maxRecoverM = Math.max(maxRecoverM, Math.hypot((near[0] - lon) * mpd.lon, (near[1] - lat) * mpd.lat));
    }
  }
  check('wall points are reported blocked', falseOnWall === tested, `${falseOnWall}/${tested}`);
  check('nearestWalkable recovers a free point', freeRecovered === tested, `${freeRecovered}/${tested}`);
  check('nearestWalkable stays within a metre', maxRecoverM < 1.0, `max ${maxRecoverM.toFixed(2)} m`);

  const g0 = buildings[0].floors[0].navGraph;
  const [wlon, wlat] = buildings[0].floors[0].obstacleLines[0][0];
  check('a point far outside is not walkable', g0.isWalkable([wlon + 1, wlat + 1]) === false);
  check('nearestWalkable returns null far from any floor', g0.nearestWalkable([wlon + 1, wlat + 1]) === null);
}

// ---------------------------------------------------------------------------
// PDR particle filter (locate.mjs) - synthetic maps, no sensors required.
// ---------------------------------------------------------------------------
console.log('\nPDR particle filter:');
{
  const lon0 = -149.83, lat0 = 61.18;
  const mpd = metresPerDegree(lat0);
  const toLL = (e, n) => [lon0 + e / mpd.lon, lat0 + n / mpd.lat];

  // A long east-west corridor: walkable only within 1 m of the centreline.
  const corridorWalkable = (p) => Math.abs((p[1] - lat0) * mpd.lat) < 1.0;
  const corridorNearest = (p) => corridorWalkable(p) ? p : toLL((p[0] - lon0) * mpd.lon, 0);

  const ax = corridorAxis(corridorWalkable, [lon0, lat0]);
  check('corridorAxis finds the corridor bearing',
    ax != null && Math.abs(wrapAngle(ax - Math.PI / 2)) < 1e-6,
    ax == null ? 'null' : `${(ax * 180 / Math.PI).toFixed(0)} deg`);
  check('corridorAxis returns null in an open room', corridorAxis(() => true, [lon0, lat0]) === null);

  // Free-space kinematics: 10 x 0.7 m north should integrate to ~7 m north.
  const free = new ParticleFilter({ isWalkable: () => true, count: 400, headingNoiseDeg: 5, distanceNoise: 0.05, lateralNoiseM: 0.02 });
  free.seed([lon0, lat0], { spreadM: 0.5, heading: 0, headingSpreadDeg: 5 });
  for (let k = 0; k < 10; k++) free.predict(0.7, 0);
  let est = free.estimate();
  check('filter integrates steps along the heading',
    Math.abs((est.lat - lat0) * mpd.lat - 7) < 0.6, `${((est.lat - lat0) * mpd.lat).toFixed(2)} m north`);

  // Map constraint: walking east, lateral error stays bounded by the walls.
  const filt = new ParticleFilter({ isWalkable: corridorWalkable, nearestWalkable: corridorNearest, count: 400, headingNoiseDeg: 6, distanceNoise: 0.08 });
  filt.seed(toLL(-5, 0), { spreadM: 1.5, heading: Math.PI / 2, headingSpreadDeg: 20 });
  for (let k = 0; k < 14; k++) filt.predict(0.7, Math.PI / 2);
  est = filt.estimate();
  const offAxis = Math.abs((est.lat - lat0) * mpd.lat);
  check('corridor keeps lateral error bounded',
    offAxis < 2.0 && filt.isWalkable([est.lon, est.lat]), `${offAxis.toFixed(2)} m off-axis`);

  // Anchoring re-seeds a tight cloud around the tapped point.
  filt.seed(toLL(3, 0), { spreadM: 0.5, heading: Math.PI / 2, headingSpreadDeg: 10 });
  est = filt.estimate();
  const anchorErr = Math.hypot((est.lon - lon0) * mpd.lon - 3, (est.lat - lat0) * mpd.lat);
  check('anchor re-seeds near the tapped point', anchorErr < 1.0, `${anchorErr.toFixed(2)} m err`);
}

// ---------------------------------------------------------------------------
// GPS/PDR arbiter (arbiter.mjs) - synthetic map, no GPS or sensors required.
// ---------------------------------------------------------------------------
console.log('\nGPS/PDR arbiter:');
{
  const lon0 = -149.83, lat0 = 61.18;
  const mpd = metresPerDegree(lat0);
  const toLL = (e, n) => [lon0 + e / mpd.lon, lat0 + n / mpd.lat];
  const off = (p) => [(p[0] - lon0) * mpd.lon, (p[1] - lat0) * mpd.lat];
  const isWalkable = (p) => Math.abs(off(p)[1]) < 1.0;
  const nearestWalkable = (p, cells = 150) => {
    const [e, n] = off(p);
    if (Math.abs(n) < 1.0) return p;
    return Math.abs(n) > cells * 0.1 ? null : toLL(e, 0);   // null = nowhere near a building
  };

  const filter = new ParticleFilter({ isWalkable, nearestWalkable, count: 300 });
  const arb = new PositionArbiter({ filter, isWalkable, nearestWalkable });

  // A good fix far from any building is plausible; GPS wins.
  const [glon, glat] = toLL(0, 40);
  arb.gps({ lon: glon, lat: glat, accuracy: 2, t: 0 });
  arb.estimate(0);
  let est = arb.estimate(4000);
  check('good outdoor GPS is used', est.source === 'gps' && est.accuracyM === 2, `${est.source} @${est.accuracyM} m`);

  // A fix inside a wall right next to a building is implausible.
  const [wlon, wlat] = toLL(0, 5);
  check('a fix inside a wall is rejected', arb.isPlausible({ lon: wlon, lat: wlat, accuracy: 5 }) === false);
  check('a far, accurate fix is plausible', arb.isPlausible({ lon: glon, lat: glat, accuracy: 5 }) === true);

  // Anchor, walk 21 m east, then only implausible indoor fixes arrive: PDR takes over.
  arb.anchor(toLL(0, 0), { spreadM: 1.0 });
  for (let k = 0; k < 30; k++) arb.step({ stepLengthM: 0.7, headingRad: Math.PI / 2 });
  arb.gps({ lon: wlon, lat: wlat, accuracy: 5, t: 10000 });
  arb.estimate(10000);
  est = arb.estimate(14000);
  check('PDR takes over when indoor GPS is implausible', est.source === 'pdr', est.source);
  check('PDR uncertainty grows with distance walked', est.accuracyM > 3.5, `${est.accuracyM.toFixed(1)} m after 21 m`);
  check('PDR estimate stays on the corridor', isWalkable([est.lon, est.lat]));

  // A good fix on the corridor re-seeds the cloud and resets the drift.
  const [plon, plat] = toLL(12, 0);
  arb.gps({ lon: plon, lat: plat, accuracy: 3, t: 20000 });
  check('a good fix resets the PDR drift counter', arb.metresSinceCorrection === 0, `${arb.metresSinceCorrection} m`);

  // A first plausible fix (even a coarse one) must seed the filter so PDR is
  // live - otherwise steps are counted but never move anything (the index.html
  // "PDR not working" bug).
  const coarseFilter = new ParticleFilter({ isWalkable, nearestWalkable, count: 100 });
  const coarseArb = new PositionArbiter({ filter: coarseFilter, isWalkable, nearestWalkable });
  check('filter starts unseeded', coarseFilter.initialized === false);
  coarseArb.gps({ lon: glon, lat: glat, accuracy: 40, t: 0 });
  check('a coarse first fix seeds the filter', coarseFilter.initialized === true);

  // Hysteresis: switching source needs the winner to stay ahead.
  arb.anchor(toLL(0, 0), { spreadM: 1.0 });
  arb.gps({ lon: glon, lat: glat, accuracy: 1, t: 30000 });   // clearly better than PDR
  arb.estimate(30000);
  arb.estimate(34000);                                  // commit GPS
  arb.gps({ lon: wlon, lat: wlat, accuracy: 5, t: 34000 }); // now implausible
  const midway = arb.estimate(35400);                   // < hysteresis
  const settled = arb.estimate(39000);                  // > hysteresis
  check('source switch is hysteretic', midway.source === 'gps' && settled.source === 'pdr',
    `${midway.source} -> ${settled.source}`);
}

// Even-odd ray cast (mirrors campus.mjs's insideWalls) for exit placement.
function insideWalls(px, py, segs) {
  let crossings = 0;
  for (const [[x0, y0], [x1, y1]] of segs) {
    if ((y0 > py) !== (y1 > py)) {
      const xint = x0 + ((py - y0) / (y1 - y0)) * (x1 - x0);
      if (px < xint) crossings++;
    }
  }
  return crossings % 2 === 1;
}

console.log('Portal sanity:');
check('every building has exits', buildings.every(b => b.floors[0].exits.length > 0));
check('AS103 has elevators', buildings.find(b => b.id === 'AS103').floors[0].elevators.length > 0);
check('AS102 has no elevator layer (expected)', buildings.find(b => b.id === 'AS102').floors[0].elevators.length === 0);
check('every building has stairs', buildings.every(b => b.floors[0].stairs.length > 0));

// Exits must sit OUTSIDE the footprint (walls + windows + solid partitions,
// the same layer set campus.mjs uses); the rare fallback is allowed only
// inside the doorway itself. A double door's centre sits up to ~1.2m from
// either wall, hence the allowance.
{
  let offenders = [];
  const FOOT = new Set(['wall', 'walls', 'window', 'windows', 'panels', 'partition', 'partitions']);
  for (const cfg of BUILDING_CONFIGS) {
    const data = JSON.parse(readFileSync(new URL(cfg.file, import.meta.url), 'utf8'));
    const wallSegs = data.shapes
      .filter(s => FOOT.has(s.layer.toLowerCase()))
      .flatMap(shapeSegments);
    const b = buildings.find(x => x.id === cfg.id);
    const distToSegs = (px, py) => {
      let best = Infinity;
      for (const [[x0, y0], [x1, y1]] of wallSegs) {
        const dx = x1 - x0, dy = y1 - y0, len2 = dx * dx + dy * dy;
        let t = len2 ? ((px - x0) * dx + (py - y0) * dy) / len2 : 0;
        t = Math.max(0, Math.min(1, t));
        best = Math.min(best, Math.hypot(px - (x0 + t * dx), py - (y0 + t * dy)));
      }
      return best * 0.0254;
    };
    for (const e of b.floors[0].exits) {
      if (insideWalls(e.dxf[0], e.dxf[1], wallSegs) && distToSegs(e.dxf[0], e.dxf[1]) > 1.25) {
        offenders.push(`${cfg.id} ${e.id}`);
      }
    }
  }
  check('exits are outside (or in the doorway)', offenders.length === 0, offenders.join(', '));
}

// Every exit must sit near a real DOORS shape (kills phantom/misplaced exits
// that the old 2m clustering produced far from any door).
{
  let offenders = [];
  for (const cfg of BUILDING_CONFIGS) {
    const data = JSON.parse(readFileSync(new URL(cfg.file, import.meta.url), 'utf8'));
    const doorSegs = data.shapes
      .filter(s => ['door', 'doors'].includes(s.layer.toLowerCase()))
      .map(s => [[s.bbox[0], s.bbox[1]], [s.bbox[2], s.bbox[3]]]);
    const b = buildings.find(x => x.id === cfg.id);
    for (const e of b.floors[0].exits) {
      let min = Infinity;
      for (const [[x0, y0], [x1, y1]] of doorSegs) {
        const dx = x1 - x0, dy = y1 - y0, len2 = dx * dx + dy * dy;
        let t = len2 ? ((e.dxf[0] - x0) * dx + (e.dxf[1] - y0) * dy) / len2 : 0;
        t = Math.max(0, Math.min(1, t));
        min = Math.min(min, Math.hypot(e.dxf[0] - (x0 + t * dx), e.dxf[1] - (y0 + t * dy)));
      }
      if (min * DXF_SCALE > 1.0) offenders.push(`${cfg.id} ${e.id} ${Math.round(min * DXF_SCALE * 100) / 100}m`);
    }
  }
  check('exits near a real door shape', offenders.length === 0, offenders.join(', '));
}

// The portal cache (buildings-config.json) must reproduce a fresh extraction
// exactly - the app trusts it instead of re-running the doorway clustering and
// flood fill. DXF coords are cached, so the check applies the floor's own
// registration to both sides.
console.log('\nPortal cache:');
{
  let floors = 0, missing = 0, mismatches = [];
  for (let i = 0; i < CONFIG.length; i++) {
    const cfg = CONFIG[i];
    for (let k = 0; k < cfg.floors.length; k++) {
      const floor = cfg.floors[k];
      if (!floor.portals) { missing++; continue; }
      // Reuse the shared fresh extraction from floorBasis (no second pass).
      const { toLL, freshPortals: fresh } = floorBasis[i][k];
      const cached = portalsFromCache(floor.portals, toLL);
      // A fresh extraction is the source of truth; the cache must equal it.
      const round = cachePortals(fresh);
      const same = (a, b) =>
        a.length === b.length && a.every((p, i) =>
          p.id === b[i].id &&
          Math.hypot(p.dxf[0] - b[i].dxf[0], p.dxf[1] - b[i].dxf[1]) < 1e-9);
      const ok = ['elevators', 'stairs', 'exits'].every(k =>
        same(round[k], floor.portals[k]) &&
        fresh[k].length === cached[k].length &&
        fresh[k].every((p, i) =>
          Math.abs(p.ll[0] - cached[k][i].ll[0]) < 1e-9 &&
          Math.abs(p.ll[1] - cached[k][i].ll[1]) < 1e-9));
      if (!ok) mismatches.push(`${cfg.id}-${floor.floorId}`);
      floors++;
    }
  }
  check('every configured floor has a portal cache', missing === 0, `${missing} missing`);
  check('cached portals match a fresh extraction', mismatches.length === 0,
    mismatches.length ? mismatches.join(', ') : `${floors} floors checked`);
}

console.log('\nBuilding router...');
const router = buildCampusRouter(buildings);
console.log('  stats:', router.stats, '\n');

console.log('Exit reachability (regression for clearance snapping):');
for (const b of buildings) {
  for (const f of b.floors) {
    for (const e of f.exits) {
      check(`${b.id} ${e.id} reachable`, !!router.buildingAt(e.ll, f.floorId));
    }
  }
}
console.log('');

const as103 = buildings.find(b => b.id === 'AS103');
const as102 = buildings.find(b => b.id === 'AS102');
const as101 = buildings.find(b => b.id === 'AS101');

function showRoute(name, r) {
  if (!r) { console.log(`  ${name}: null`); return; }
  console.log(`  ${name}: ${r.summary} | ${Math.round(r.totalMeters)}m, ${Math.round(r.totalSeconds)}s`);
  for (const s of r.segments) {
    const geom = s.polyline ? `${s.polyline.length} pts` : 'vertical';
    console.log(`    - ${s.tier.padEnd(8)} ${s.building ?? ''} ${s.floor ?? (s.fromFloor ? s.fromFloor + '->' + s.toFloor : '')} ${geom} ${Math.round(s.meters)}m ${Math.round(s.seconds)}s`);
  }
}

console.log('Test 1: same building, same floor (AS103 exit -> AS103 exit)');
const a = as103.floors[0].exits[0].ll, b = as103.floors[0].exits[3].ll;
const r1 = router.route(a, b);
showRoute('AS103->AS103', r1);
check('routes', !!r1);
check('single indoor segment', r1 && r1.segments.length === 1 && r1.segments[0].tier === 'indoor');

console.log('\nTest 2: cross-building (AS103 room -> AS102 room)');
const room103 = { x: 0, y: 0 }; // will use a text_line below
const data103 = JSON.parse(readFileSync(new URL('AS103-01.json', import.meta.url), 'utf8'));
const data102 = JSON.parse(readFileSync(new URL('AS102-01.json', import.meta.url), 'utf8'));
const t103 = makeTransform(BUILDING_CONFIGS[0]);
const t102 = makeTransform(BUILDING_CONFIGS[2]);
const startLL = t103(data103.text_lines[0].x, data103.text_lines[0].y);
const endLL = t102(data102.text_lines[0].x, data102.text_lines[0].y);
console.log(`  start room ${data103.text_lines[0].text}, end room ${data102.text_lines[0].text}`);
const r2 = router.route(startLL, endLL);
showRoute('AS103->AS102', r2);
check('routes', !!r2);
check('has outdoor segment', r2 && r2.segments.some(s => s.tier === 'outdoor'));
check('has indoor segments at both ends', r2 && r2.segments.filter(s => s.tier === 'indoor').length >= 2);

console.log('\nTest 3: start outdoors (a point between two buildings)');
// Find a sample along the exitA->exitB line that is genuinely outdoors.
const exitA = as103.floors[0].exits[0].ll, exitB = as102.floors[0].exits[0].ll;
let midLL = null;
for (let t = 0.1; t <= 0.9 && !midLL; t += 0.05) {
  const p = [exitA[0] + (exitB[0] - exitA[0]) * t, exitA[1] + (exitB[1] - exitA[1]) * t];
  if (!router.buildingAt(p, '01')) midLL = p;
}
check('found an outdoor sample point', !!midLL);
const r3 = midLL ? router.route(midLL, endLL) : null;
showRoute('outdoor->AS102', r3);
check('routes', !!r3);
check('starts outdoor', r3 && r3.segments[0].tier === 'outdoor');

console.log('\nTest 4: synthetic second floor of AS103 (elevator/stairs test)');
const as103b = buildings.find(x => x.id === 'AS103');
const floor2 = { ...as103b.floors[0], floorId: '02' };
as103b.floors.push(floor2);
const router2 = buildCampusRouter(buildings);
const r4 = router2.route(startLL, endLL, { startFloor: '02' });
showRoute('AS103 f02 -> AS102 f01', r4);
check('routes', !!r4);
check('uses a vertical connector', r4 && r4.segments.some(s => s.tier === 'elevator' || s.tier === 'stairs'));
check('vertical is first-building leg', r4 && (() => {
  const vIdx = r4.segments.findIndex(s => s.tier === 'elevator' || s.tier === 'stairs');
  return vIdx >= 0 && r4.segments.slice(0, vIdx).every(s => s.tier === 'indoor' && s.floor === '02');
})());

console.log('\nTest 5: same building, cross floor (AS103 f02 -> AS103 f01)');
const endLL103 = as103.floors[0].exits[3].ll;
const r5 = router2.route(startLL, endLL103, { startFloor: '02', endFloor: '01' });
showRoute('AS103 f02 -> f01', r5);
check('routes', !!r5);
check('contains vertical', r5 && r5.segments.some(s => s.tier === 'elevator' || s.tier === 'stairs'));
check('no outdoor', r5 && !r5.segments.some(s => s.tier === 'outdoor'));

console.log('\nTest 6: cross-campus chain (AS105 room -> AS101 room)');
const data105 = JSON.parse(readFileSync(new URL('AS105-01.json', import.meta.url), 'utf8'));
const data101 = JSON.parse(readFileSync(new URL('AS101-01.json', import.meta.url), 'utf8'));
const t105 = makeTransform(BUILDING_CONFIGS[3]);
const t101 = makeTransform(BUILDING_CONFIGS[1]);
const s6 = t105(data105.text_lines[0].x, data105.text_lines[0].y);
const e6 = t101(data101.text_lines[0].x, data101.text_lines[0].y);
const r6 = router.route(s6, e6);
showRoute('AS105->AS101', r6);
check('routes', !!r6);
check('has outdoor', r6 && r6.segments.some(s => s.tier === 'outdoor'));
check('starts in AS105, ends in AS101', r6 && /AS105/.test(r6.summary) && /AS101/.test(r6.summary));

console.log('\nTest 7: outdoor path network (synthetic grid)');
// A lon/lat grid over the campus. Every exit snaps to it, so outdoor legs
// must bend along grid lines instead of cutting straight through buildings.
{
  const lat = as103.floors[0].exits[0].ll[1];
  const mPerDeg = metresPerDegree(lat);
  let minLon = Infinity, minLat = Infinity, maxLon = -Infinity, maxLat = -Infinity;
  for (const bld of buildings) for (const f of bld.floors) for (const p of [...f.elevators, ...f.stairs, ...f.exits]) {
    minLon = Math.min(minLon, p.ll[0]); maxLon = Math.max(maxLon, p.ll[0]);
    minLat = Math.min(minLat, p.ll[1]); maxLat = Math.max(maxLat, p.ll[1]);
  }
  const pad = 150;
  const lon0 = minLon - pad / mPerDeg.lon, lat0 = minLat - pad / mPerDeg.lat;
  const lon1 = maxLon + pad / mPerDeg.lon, lat1 = maxLat + pad / mPerDeg.lat;
  const stepM = 25;
  const cols = Math.round((lon1 - lon0) * mPerDeg.lon / stepM), rows = Math.round((lat1 - lat0) * mPerDeg.lat / stepM);
  const at = (i, which) => which === 0 ? lon0 + (lon1 - lon0) * i / cols : lat0 + (lat1 - lat0) * i / rows;
  const shapes = [];
  for (let r = 0; r <= rows; r++) {
    const pts = []; for (let c = 0; c <= cols; c++) pts.push([at(c, 0), at(r, 1)]);
    shapes.push({bbox: [pts[0][0], pts[0][1], pts[cols][0], pts[cols][1]], stroke: null, fill: null, width: null, closed: false, layer: 'PATHS', items: [{op: 'line', points: pts}]});
  }
  for (let c = 0; c <= cols; c++) {
    const pts = []; for (let r = 0; r <= rows; r++) pts.push([at(c, 0), at(r, 1)]);
    shapes.push({bbox: [pts[0][0], pts[0][1], pts[pts.length - 1][0], pts[pts.length - 1][1]], stroke: null, fill: null, width: null, closed: false, layer: 'PATHS', items: [{op: 'line', points: pts}]});
  }

  const obstacleSegs = buildings.flatMap(bld => bld.floors.flatMap(f => f.obstacleLines ?? []));
  const graph = buildOutdoorGraph(shapes, lat, obstacleSegs);
  check('grid graph built', !!graph && graph.nodeCount > 100, `${graph?.nodeCount} nodes`);
  // Every exit must reach the network without cutting through a building;
  // walled-in corners (e.g. an AS105 exit) attach further away, ~40m.
  check('grid exits snap to the network', !!graph && buildings.every(bld =>
    bld.floors[0].exits.every(e => graph.snapClear(e.ll)?.metres < 40)));

  const routerGrid = buildCampusRouter(buildings, shapes);
  const r7 = routerGrid.route(startLL, endLL);
  showRoute('AS103->AS102 (grid network)', r7);
  check('routes', !!r7);
  const outdoorSeg = r7 && r7.segments.find(s => s.tier === 'outdoor');
  // any detour along the network is longer than the chord between its ends
  const chord7 = outdoorSeg && Math.hypot(
    (outdoorSeg.polyline[0][0] - outdoorSeg.polyline[outdoorSeg.polyline.length - 1][0]) * mPerDeg.lon,
    (outdoorSeg.polyline[0][1] - outdoorSeg.polyline[outdoorSeg.polyline.length - 1][1]) * mPerDeg.lat);
  check('outdoor leg follows the network', !!outdoorSeg && outdoorSeg.polyline.length > 2,
    outdoorSeg ? `${outdoorSeg.polyline.length} pts` : 'no outdoor leg');
  check('outdoor leg detours past the straight line',
    !!outdoorSeg && outdoorSeg.meters >= chord7 - 1,
    `${Math.round(outdoorSeg?.meters ?? 0)}m vs ${Math.round(chord7 ?? 0)}m straight`);

  const r7b = midLL ? routerGrid.route(midLL, endLL) : null;
  showRoute('outdoor click->AS102 (grid network)', r7b);
  check('outdoor click attaches via network', !!r7b && !!r7b.segments.find(s => s.tier === 'outdoor' && s.polyline.length > 2));

  // Without shapes the router must fall back to straight-line outdoor legs.
  const routerPlain = buildCampusRouter(buildings);
  const r7c = routerPlain.route(startLL, endLL);
  check('no network -> straight-line outdoor fallback', !!r7c &&
    r7c.segments.filter(s => s.tier === 'outdoor').every(s => s.polyline.length === 2));
}

console.log('\nTest 7c: outdoor attachment projects onto edges');
// A tap or exit that faces the middle of a long way must attach to the edge
// itself, not to the way's nearest vertex tens of metres away: vertex-only
// snapping left a path that visibly ran under the finger unused.
{
  const lat = 61.19;
  const mPerDeg = metresPerDegree(lat);
  const o = [-149.84, 61.19];
  const ll = (x, y) => [o[0] + x / mPerDeg.lon, o[1] + y / mPerDeg.lat];
  const pts = [ll(-50, 0), ll(50, 0)];
  const graph = buildOutdoorGraph([{
    bbox: [pts[0][0], pts[0][1], pts[1][0], pts[1][1]],
    stroke: null, fill: null, width: null, closed: false, layer: 'PATHS',
    items: [{op: 'line', points: pts}],
  }], lat);

  const mid = graph.snapClear(ll(0, 0));
  check('mid-edge tap attaches on the edge', !!mid && mid.metres < 0.5,
    `${mid ? mid.metres.toFixed(2) : '-'}m (nearest vertex would be 50m)`);
  const off = graph.snapClear(ll(0, 3));
  check('off-edge tap attaches with a short stub', !!off && Math.abs(off.metres - 3) < 0.5,
    `${off ? off.metres.toFixed(2) : '-'}m`);
  const a = graph.snapClear(ll(-10, 0));
  const b = graph.snapClear(ll(10, 0));
  const net = graph.dijkstraFrom(a.node).dist[b.node];
  check('walk along the edge is direct', Math.abs(net - 20) < 1,
    `${Number.isFinite(net) ? net.toFixed(1) : '-'}m for a 20m span`);
  const end = graph.snapClear(ll(-50, 0));
  check('snap at a vertex reuses the node',
    end.node === graph.snap(ll(-50, 0)).node && end.metres < 0.05,
    `${end.metres.toFixed(2)}m`);
}

console.log('\nTest 8: live OpenStreetMap network (scak.pmtiles roads layer)');
{
  const OSM_URL = 'https://navigator.cheeseverse.net/scak.pmtiles';
  const lat8 = as103.floors[0].exits[0].ll[1];
  const mPerDeg8 = metresPerDegree(lat8);
  const obstacleSegs = buildings.flatMap(bld => bld.floors.flatMap(f => f.obstacleLines ?? []));
  let minLon = Infinity, minLat = Infinity, maxLon = -Infinity, maxLat = -Infinity;
  for (const bld of buildings) for (const f of bld.floors) for (const p of f.exits) {
    minLon = Math.min(minLon, p.ll[0]); maxLon = Math.max(maxLon, p.ll[0]);
    minLat = Math.min(minLat, p.ll[1]); maxLat = Math.max(maxLat, p.ll[1]);
  }
  let osm = null;
  try {
    const pad = 200;
    osm = await loadOsmShapes({
      url: OSM_URL,
      bbox: {minLon: minLon - pad / mPerDeg8.lon, maxLon: maxLon + pad / mPerDeg8.lon,
             minLat: minLat - pad / mPerDeg8.lat, maxLat: maxLat + pad / mPerDeg8.lat},
      fetchTile: (z, x, y) => pmtilesGetZxy(OSM_URL, z, x, y),
    });
  } catch (e) {
    console.log(`  (skipped: ${e.message})`);
  }
  if (osm) {
    check('fetched tiles', osm.stats.tiles >= 4, `${osm.stats.tiles} tiles`);
    check('kept ways', osm.stats.ways > 50, `${osm.stats.ways} ways (${osm.stats.duplicateWays} dupes dropped)`);
    check('shapes are PATHS polylines', osm.shapes.length > 50 &&
      osm.shapes.every(s => s.layer === 'PATHS' && s.items[0].op === 'line' && s.items[0].points.length >= 2 &&
        Number.isFinite(s.items[0].points[0][0])));
    check('coordinates look like lon/lat near campus', osm.shapes.every(s =>
      s.bbox[0] > -149.9 && s.bbox[2] < -149.7 && s.bbox[1] > 61.1 && s.bbox[3] < 61.3));

    const t8 = performance.now();
    const routerOsm = buildCampusRouter(buildings, osm.shapes);
    const r8 = routerOsm.route(startLL, endLL);
    showRoute('AS103->AS102 (OSM network)', r8);
    console.log(`  routed in ${Math.round(performance.now() - t8)}ms, graph:`,
      routerOsm.stats.outdoor.nodes, 'nodes,', routerOsm.stats.outdoor.networkEdges, 'of',
      routerOsm.stats.macroEdges, 'macro edges via network');
    check('routes', !!r8);
    const seg8 = r8 && r8.segments.find(s => s.tier === 'outdoor');
    check('outdoor leg follows the OSM network', !!seg8 && seg8.polyline.length > 2,
      seg8 ? `${seg8.polyline.length} pts` : 'no outdoor leg');
    const outdoorSegs = r8 ? r8.segments.filter(s => s.tier === 'outdoor') : [];
    const straight8 = outdoorSegs.reduce((a, s) => a + Math.hypot(
      (s.polyline[0][0] - s.polyline[s.polyline.length - 1][0]) * mPerDeg8.lon,
      (s.polyline[0][1] - s.polyline[s.polyline.length - 1][1]) * mPerDeg8.lat), 0);
    const meters8 = outdoorSegs.reduce((a, s) => a + s.meters, 0);
    check('outdoor distance sane (>= straight, <= 3x)',
      outdoorSegs.length > 0 && meters8 >= straight8 - 1 && meters8 <= straight8 * 3,
      `${Math.round(meters8)}m vs ${Math.round(straight8)}m straight`);
    const r8b = midLL ? routerOsm.route(midLL, endLL) : null;
    showRoute('outdoor click->AS102 (OSM network)', r8b);
    check('outdoor click attaches via OSM network',
      !!r8b && !!r8b.segments.find(s => s.tier === 'outdoor' && s.polyline.length > 2));

    // Regression for "a displayed path the route ignores": tapping two points
    // on one way must walk that way, not detour to a building entrance and
    // back. Pick a long way whose quarter points are far from every exit, so
    // no building shortcut could legitimately win.
    const allExits = buildings.flatMap(bld => bld.floors.flatMap(f => f.exits.map(e => e.ll)));
    const farFromExits = ll => allExits.every(e =>
      Math.hypot((ll[0] - e[0]) * mPerDeg8.lon, (ll[1] - e[1]) * mPerDeg8.lat) > 100);
    const onWay = (p, t) => {
      const lens = [];
      let total = 0;
      for (let k = 1; k < p.length; k++) {
        const d = Math.hypot((p[k][0] - p[k - 1][0]) * mPerDeg8.lon,
                             (p[k][1] - p[k - 1][1]) * mPerDeg8.lat);
        lens.push(d); total += d;
      }
      let want = total * t;
      for (let k = 0; k < lens.length; k++) {
        if (want <= lens[k]) {
          const f = lens[k] ? want / lens[k] : 0;
          return [p[k][0] + (p[k + 1][0] - p[k][0]) * f, p[k][1] + (p[k + 1][1] - p[k][1]) * f];
        }
        want -= lens[k];
      }
      return p[p.length - 1];
    };
    let tapCheck = null;
    for (const s of osm.shapes) {
      const p = s.items[0].points;
      const A2 = onWay(p, 0.25), B2 = onWay(p, 0.75);
      const straightAB = Math.hypot((A2[0] - B2[0]) * mPerDeg8.lon, (A2[1] - B2[1]) * mPerDeg8.lat);
      if (straightAB < 50 || !farFromExits(A2) || !farFromExits(B2)) continue;
      const rAB = routerOsm.route(A2, B2);
      tapCheck = {rAB, straightAB, A2, B2};
      break;
    }
    check('tap two points on one path -> walk it',
      !!tapCheck && !!tapCheck.rAB && tapCheck.rAB.totalMeters <= tapCheck.straightAB * 1.6,
      tapCheck?.rAB
        ? `${Math.round(tapCheck.rAB.totalMeters)}m for ${Math.round(tapCheck.straightAB)}m of path`
        : 'no long way found');

    // Regression for "the route cuts through another building": no segment of
    // any outdoor polyline may cross a wall of any building (the exit/click
    // endpoints sit on the wall, hence a 2m endpoint margin).
    const obstacleIndex = buildObstacleIndex(obstacleSegs, mPerDeg8);
    const wallCuts = (r) => {
      let cuts = 0, segs = 0;
      for (const s of r.segments) {
        if (s.tier !== 'outdoor') continue;
        segs += s.polyline.length - 1;
        for (let k = 1; k < s.polyline.length; k++) {
          const a = s.polyline[k - 1], b = s.polyline[k];
          if (obstacleIndex.crosses(a[0] * mPerDeg8.lon, a[1] * mPerDeg8.lat,
                                    b[0] * mPerDeg8.lon, b[1] * mPerDeg8.lat, 2)) cuts++;
        }
      }
      return {cuts, segs};
    };
    const c8 = wallCuts(r8);
    check('outdoor legs never cross a wall', c8.cuts === 0, `${c8.segs} outdoor segments checked`);

    // every building pair must route, with no outdoor segment crossing a wall
    const rooms = BUILDING_CONFIGS.map(cfg => {
      const data = JSON.parse(readFileSync(new URL(cfg.file, import.meta.url), 'utf8'));
      return makeTransform(cfg)(data.text_lines[0].x, data.text_lines[0].y);
    });
    const badPairs = [];
    for (let i = 0; i < BUILDING_CONFIGS.length; i++) {
      for (let j = 0; j < BUILDING_CONFIGS.length; j++) {
        if (i === j) continue;
        const rp = routerOsm.route(rooms[i], rooms[j]);
        if (!rp) { badPairs.push(`${BUILDING_CONFIGS[i].id}->${BUILDING_CONFIGS[j].id} no route`); continue; }
        const c = wallCuts(rp);
        if (c.cuts > 0) badPairs.push(`${BUILDING_CONFIGS[i].id}->${BUILDING_CONFIGS[j].id} ${c.cuts} cuts`);
      }
    }
    check('all 12 building pairs route without cutting walls', badPairs.length === 0, badPairs.join('; '));
  }
}

// ---------------------------------------------------------------------------
// OSM paths cache (gen-osm.mjs) - the app loads this instead of decoding the
// pmtiles archive when it covers the campus.
// ---------------------------------------------------------------------------
console.log('\nOSM paths cache:');
{
  const cache = osmCache;
  check('osm-paths.json exists', !!cache);
  if (cache) {
    const shapesOk = Array.isArray(cache.shapes) && cache.shapes.length > 0 &&
      cache.shapes.every(s => s.layer === 'PATHS' && s.items?.[0]?.op === 'line' &&
        s.items[0].points.length >= 2);
    check('osm-paths.json shapes are well formed', shapesOk, `${cache.shapes?.length} shapes`);
    // It must cover the campus bbox the app computes: effective exits (manual
    // doors win), plus stairs/elevators, across every floor.
    let minLon = Infinity, minLat = Infinity, maxLon = -Infinity, maxLat = -Infinity;
    const bump = ll => {
      minLon = Math.min(minLon, ll[0]); maxLon = Math.max(maxLon, ll[0]);
      minLat = Math.min(minLat, ll[1]); maxLat = Math.max(maxLat, ll[1]);
    };
    for (const cfg of CONFIG) for (const f of cfg.floors) {
      const toLL = makeToLL(cfg, f);
      const p = f.portals ? portalsFromCache(f.portals, toLL) : {elevators: [], stairs: [], exits: []};
      const exits = f.exits?.length ? f.exits : p.exits;
      for (const q of [...p.elevators, ...p.stairs, ...exits]) bump(toLL(q.dxf[0], q.dxf[1]));
    }
    const m = metresPerDegree((minLat + maxLat) / 2);
    const bb = {
      minLon: minLon - 200 / m.lon, maxLon: maxLon + 200 / m.lon,
      minLat: minLat - 200 / m.lat, maxLat: maxLat + 200 / m.lat,
    };
    const c = cache.bbox;
    check('osm-paths.json covers the campus bbox',
      !!c && bb.minLon >= c.minLon && bb.maxLon <= c.maxLon &&
      bb.minLat >= c.minLat && bb.maxLat <= c.maxLat);
    // The cached shapes must build the same outdoor graph as a live decode.
    const graph = buildOutdoorGraph(cache.shapes, 61.19,
      buildings.flatMap(b => b.floors[0]?.obstacleLines ?? []));
    check('cached shapes build an outdoor graph', !!graph && graph.nodeCount > 0,
      `${graph?.nodeCount} nodes`);
  }
}

// ---------------------------------------------------------------------------
// Multi-floor routing (real second floors) and auto-detected skybridges.
// ---------------------------------------------------------------------------
console.log('\nMulti-floor routing (all real floors):');
{
  // A second full campus, this time with every registered floor (the `buildings`
  // above deliberately keep only the ground floor for the older tests). All
  // per-floor work comes from the shared `floorBasis`; this block only wraps
  // it. Construction mirrors index.html: portals come from the shipped cache
  // with a fresh-extraction fallback, manual doors win, skybridges are
  // explicit, and the router is fed the shipped OSM path cache (the app's
  // outdoor network). The portal-equality check above proves the cache equals
  // a fresh extraction, so a second full router to compare the two was pure
  // duplication (it cost ~10 s per run).
  const mkBuilding = (cfg, basics) => {
    const b = { id: cfg.id, groundFloor: cfg.groundFloor, floors: [] };
    for (const { floor, toLL, lines, navGraph, freshPortals } of basics) {
      const portals = floor.portals ? portalsFromCache(floor.portals, toLL) : freshPortals;
      // Mirror index.html: manual doors win; skybridges are explicit portals.
      const doorList = floor.exits?.length ? floor.exits : portals.exits;
      const exits = doorList.map((e, i) => ({
        id: e.id ?? `exits-${i}`, ll: toLL(e.dxf[0], e.dxf[1]), dxf: e.dxf,
      }));
      const skybridges = (floor.skybridges ?? []).map((e, i) => ({
        id: e.id ?? `bridge-${i}`, ll: toLL(e.dxf[0], e.dxf[1]), dxf: e.dxf,
        to: e.to ?? null,
      }));
      b.floors.push({
        floorId: floor.floorId, navGraph, obstacleLines: lines,
        elevators: portals.elevators, stairs: portals.stairs, exits, skybridges,
      });
    }
    return b;
  };
  const allBuildings = CONFIG.map((c, i) => mkBuilding(c, floorBasis[i]));
  const routerAll = buildCampusRouter(allBuildings, osmCache?.shapes ?? null);

  const A103 = allBuildings.find(b => b.id === 'AS103');
  const A102 = allBuildings.find(b => b.id === 'AS102');
  const A106 = allBuildings.find(b => b.id === 'AS106');
  // Both helpers read from the shared floorBasis (no re-parse, no re-extract).
  const basisOf = (id, floorId) => {
    const i = CONFIG.findIndex(c => c.id === id);
    const k = CONFIG[i].floors.findIndex(f => f.floorId === floorId);
    return floorBasis[i][k];
  };
  const roomLL = (id, floorId) => {
    const { textLines, toLL } = basisOf(id, floorId);
    return toLL(textLines[0].x, textLines[0].y);
  };

  const sF2 = roomLL('AS103', '02');
  const eAS102 = roomLL('AS102', '01');
  const eF2 = roomLL('AS106', '02');

  // The floor hint (what the picker shows) resolves the real second floor.
  check('AS103 second floor resolves with a hint',
    routerAll.buildingAt(sF2, '02')?.building.id === 'AS103' &&
    routerAll.buildingAt(sF2, '02')?.floor.floorId === '02');

  const r9 = routerAll.route(sF2, eAS102, { startFloor: '02' });
  showRoute('AS103 f02 -> AS102 f01', r9);
  check('floor-2 cross-building routes', !!r9);
  check('descends to the ground floor first', r9 && (() => {
    const vIdx = r9.segments.findIndex(s => s.tier === 'elevator' || s.tier === 'stairs');
    const oIdx = r9.segments.findIndex(s => s.tier === 'outdoor');
    return vIdx >= 0 && oIdx >= 0 && vIdx < oIdx;
  })());
  check('ends in AS102', r9 && /AS102/.test(r9.summary));
  // The display filters the drawn line by segment floor, so every drawn
  // indoor leg must name its floor.
  check('indoor segments carry their floor',
    r9 && r9.segments.filter(s => s.tier === 'indoor').every(s => s.floor));
  // The 3D view draws a riser at each vertical leg's connector.
  check('vertical segments carry a location (3D riser)',
    r9 && r9.segments.filter(s => s.tier === 'elevator' || s.tier === 'stairs').every(s => Array.isArray(s.ll)));

  // No separate cached-vs-fresh router comparison here: `routerAll` itself is
  // fed the shipped cache (the production path) with a fresh fallback, and the
  // portal-equality check above proves the cache is bit-identical to a fresh
  // extraction, so routing cannot differ.

  // Two buildings with no skybridge: the f2 -> f2 trip still goes down first.
  const r9b = routerAll.route(sF2, eF2, { startFloor: '02', endFloor: '02' });
  showRoute('AS103 f02 -> AS106 f02', r9b);
  check('f2 -> f2 without a skybridge still descends',
    r9b && r9b.segments.some(s => s.tier === 'elevator' || s.tier === 'stairs') &&
    !r9b.segments.some(s => s.tier === 'skybridge'));

  // The two taps may be on different floors: start on the ground, finish
  // upstairs in the other building.
  const sF1 = roomLL('AS103', '01');
  const r9c = routerAll.route(sF1, eF2, { startFloor: '01', endFloor: '02' });
  showRoute('AS103 f01 -> AS106 f02', r9c);
  check('mixed floors: start 01, end 02 in another building',
    !!r9c && r9c.segments.some(s => s.tier === 'elevator' || s.tier === 'stairs') &&
    /AS106 02/.test(r9c.summary));

  // Winter: outdoor travel is penalised heavily, so a skybridge (or any indoor
  // detour) beats going outside. AS105-02 and AS117-02 are joined by a bridge.
  const walkableRoom = (id, floorId) => {
    const { textLines, toLL } = basisOf(id, floorId);
    for (const t of textLines) {
      const ll = toLL(t.x, t.y);
      const hit = routerAll.buildingAt(ll, floorId);
      if (hit && hit.building.id === id) return ll;
    }
    return null;
  };
  const hasBridge = (id, floorId) =>
    (CONFIG.find(c => c.id === id).floors.find(f => f.floorId === floorId)?.skybridges?.length ?? 0) > 0;
  if (hasBridge('AS105', '02') && hasBridge('AS117', '02')) {
    const s105 = walkableRoom('AS105', '02'), s117 = walkableRoom('AS117', '02');
    const rW = routerAll.route(s105, s117, { startFloor: '02', endFloor: '02' });
    showRoute('AS105 f02 -> AS117 f02 (skybridge)', rW);
    check('skybridge preferred over outdoor in winter',
      !!rW && rW.segments.some(s => s.tier === 'skybridge') &&
      !rW.segments.some(s => s.tier === 'outdoor'));

    // Regression: the route must be able to change floors BEFORE the bridge -
    // starting a floor below the bridge should go up first, not outside.
    const g105 = walkableRoom('AS105', '01');
    const rUp = g105 ? routerAll.route(g105, s117, { startFloor: '01', endFloor: '02' }) : null;
    showRoute('AS105 f01 -> AS117 f02 (up first)', rUp);
    check('ascends to the bridge first, then skybridges (no outdoor)',
      !!rUp && rUp.segments.some(s => s.tier === 'skybridge') &&
      rUp.segments.some(s => s.tier === 'elevator' || s.tier === 'stairs') &&
      !rUp.segments.some(s => s.tier === 'outdoor'));
  }

  // ECB (AS121) <-> SU (AS119): the mutual explicit pair (~125 m, and it has
  // lived on different floors while being placed). Explicit marks must link it
  // regardless of floor number and the SKYBRIDGE_MAX_M cap, so find the pair
  // from the config rather than hard-coding a floor.
  {
    const e121 = CONFIG.find(c => c.id === 'AS121')?.floors.find(f =>
      f.skybridges?.some(s => s.to === 'AS119'));
    const e119 = CONFIG.find(c => c.id === 'AS119')?.floors.find(f =>
      f.skybridges?.some(s => s.to === 'AS121'));
    if (e121 && e119) {
      const s121 = walkableRoom('AS121', e121.floorId);
      const s119 = walkableRoom('AS119', e119.floorId);
      const rE = s121 && s119
        ? routerAll.route(s121, s119, { startFloor: e121.floorId, endFloor: e119.floorId })
        : null;
      showRoute(`AS121 f${e121.floorId} -> AS119 f${e119.floorId} (explicit skybridge)`, rE);
      check('explicit bridge links AS121 to AS119 (no outdoor)',
        !!rE && rE.segments.some(s => s.tier === 'skybridge') &&
        !rE.segments.some(s => s.tier === 'outdoor'));
    }
  }
}

console.log('\nAuto-detected skybridge (synthetic):');
{
  // Two small square buildings ~3m apart at level 02, each with an exit on the
  // facing wall. Those two same-level exits within SKYBRIDGE_MAX_M must be
  // detected as a bridge and used instead of descending to the ground.
  const lat0 = 61.19, lon0 = -149.84;
  const mpd = metresPerDegree(lat0);
  const half = 6;                       // square half-width, metres
  const gap = 26;                       // centre-to-centre, metres: bridge span ~15m
  const dx = half / mpd.lon, dy = half / mpd.lat;
  const yx = lon0 + gap / mpd.lon;
  const square = (cx, cy) => {
    const c = [[cx - dx, cy - dy], [cx + dx, cy - dy], [cx + dx, cy + dy], [cx - dx, cy + dy]];
    const lines = c.map((p, i) => [p, c[(i + 1) % 4]]);
    return { navGraph: buildGraph(lines), obstacleLines: lines };
  };
  const mkTwin = (cx, west) => ({
    groundFloor: '01',
    floors: [
      { floorId: '01', ...square(cx, lat0), elevators: [],
        stairs: [{ id: 'gs', ll: [cx, lat0] }],
        exits: [{ id: 'g', ll: [cx + (west ? -1 : 1) * dx * 0.9, lat0] }] },
      { floorId: '02', ...square(cx, lat0), elevators: [],
        stairs: [{ id: 's2', ll: [cx, lat0] }],
        // A plain upper-floor exit facing the neighbour must NOT connect ...
        exits: [{ id: 'roof', ll: [cx + (west ? -1 : 1) * dx * 0.9, lat0 + 1 / mpd.lat] }],
        // ... only the explicit skybridge portals link.
        skybridges: [{ id: 'bridge', ll: [cx + (west ? -1 : 1) * dx * 0.9, lat0] }] },
    ],
  });
  const X = { id: 'X', ...mkTwin(lon0, false) };
  const Y = { id: 'Y', ...mkTwin(yx, true) };
  const routerS = buildCampusRouter([X, Y]);
  check('only the skybridge portals link (roof exits stay isolated)',
    routerS.stats.skybridgeEdges === 1, `${routerS.stats.skybridgeEdges} edge(s)`);

  const rS = routerS.route([lon0, lat0], [yx, lat0], { startFloor: '02', endFloor: '02' });
  showRoute('X f02 -> Y f02 via bridge', rS);
  check('routes over the skybridge', !!rS &&
    rS.segments.some(s => s.tier === 'skybridge') &&
    !rS.segments.some(s => s.tier === 'elevator' || s.tier === 'stairs'));
  check('skybridge segment carries its floor',
    rS && rS.segments.find(s => s.tier === 'skybridge')?.floor === '02');
}

console.log('\nExplicit skybridge destination (synthetic):');
{
  // X and Z are 40 m apart (auto-matchable), Y is 170 m away (over the cap).
  // Without a destination X auto-pairs with Z; marking X's bridge `to: 'Y'`
  // (the ECB case) must link X-Y instead and never auto-pair X with Z.
  const lat0 = 61.18, lon0 = -149.85;
  const mpd = metresPerDegree(lat0);
  const half = 6;
  const dx = half / mpd.lon, dy = half / mpd.lat;
  const square = (cx, cy) => {
    const c = [[cx - dx, cy - dy], [cx + dx, cy - dy], [cx + dx, cy + dy], [cx - dx, cy + dy]];
    const lines = c.map((p, i) => [p, c[(i + 1) % 4]]);
    return { navGraph: buildGraph(lines), obstacleLines: lines };
  };
  const mkStack = (id, cx, bridge, bridgeFloor = '02') => {
    const f1 = { floorId: '01', ...square(cx, lat0), elevators: [],
      stairs: [{ id: 'gs', ll: [cx, lat0] }],
      exits: [{ id: 'g', ll: [cx + dx * 0.9, lat0] }] };
    const f2 = { floorId: '02', ...square(cx, lat0), elevators: [],
      stairs: [{ id: 's2', ll: [cx, lat0] }],
      exits: [] };
    const floors = [f1, f2];
    if (bridge) floors.find(f => f.floorId === bridgeFloor).skybridges = [bridge];
    return { id, groundFloor: '01', floors };
  };
  const xBridge = to => [{ id: 'bx', ll: [lon0 + dx * 0.9, lat0], ...(to ? { to } : {}) }];
  const zBridge = [{ id: 'bz', ll: [lon0 + 40 / mpd.lon - dx * 0.9, lat0] }];
  const yBridge = [{ id: 'by', ll: [lon0 + 170 / mpd.lon - dx * 0.9, lat0] }];
  const mkWorld = xTo => [
    mkStack('X', lon0, xBridge(xTo)[0]),
    mkStack('Z', lon0 + 40 / mpd.lon, zBridge[0]),
    mkStack('Y', lon0 + 170 / mpd.lon, yBridge[0]),
  ];

  const routerAuto = buildCampusRouter(mkWorld(null));
  check('no destination: X auto-pairs with the nearer Z',
    routerAuto.stats.skybridgeEdges === 1 &&
    !!routerAuto.route([lon0, lat0], [lon0 + 40 / mpd.lon, lat0], { startFloor: '02', endFloor: '02' })
      ?.segments.some(s => s.tier === 'skybridge'));

  const routerExp = buildCampusRouter(mkWorld('Y'));
  const rXY = routerExp.route([lon0, lat0], [lon0 + 170 / mpd.lon, lat0],
    { startFloor: '02', endFloor: '02' });
  showRoute('X f02 -> Y f02 (explicit destination)', rXY);
  const bridgeSeg = rXY?.segments.find(s => s.tier === 'skybridge');
  check('explicit destination links X-Y despite the 170 m gap and cap',
    routerExp.stats.skybridgeEdges === 1 && !!bridgeSeg && bridgeSeg.meters > 100);
  const rXZ = routerExp.route([lon0, lat0], [lon0 + 40 / mpd.lon, lat0],
    { startFloor: '02', endFloor: '02' });
  check('an explicit bridge never auto-pairs elsewhere (X-Z has no bridge)',
    !rXZ?.segments.some(s => s.tier === 'skybridge'));

  // Cross-floor mutual marks (a grade change, the ECB-01 <-> SU-02 case):
  // X floor 01 <-> Y floor 02 must link even though the numbers differ.
  const routerXF = buildCampusRouter([
    mkStack('X', lon0, { id: 'bx', ll: [lon0 + dx * 0.9, lat0], to: 'Y' }, '01'),
    mkStack('Y', lon0 + 40 / mpd.lon,
      { id: 'by', ll: [lon0 + 40 / mpd.lon - dx * 0.9, lat0], to: 'X' }, '02'),
  ]);
  const rXF = routerXF.route([lon0, lat0], [lon0 + 40 / mpd.lon, lat0],
    { startFloor: '01', endFloor: '02' });
  showRoute('X f01 -> Y f02 (cross-floor explicit)', rXF);
  check('mutual marks link across floor numbers (X-01 <-> Y-02)',
    routerXF.stats.skybridgeEdges === 1 &&
    !!rXF && rXF.segments.some(s => s.tier === 'skybridge') &&
    !rXF.segments.some(s => s.tier === 'outdoor'));
}

// ---------------------------------------------------------------------------
// PDR signal processing (motion.mjs) - synthetic gait, no sensors required.
// ---------------------------------------------------------------------------
console.log('\nMotion / PDR signal processing:');

// One Gaussian vertical-acceleration bump per step (a realistic "one peak per
// step" gait signal, unlike a pure sinusoid whose magnitude doubles).
function synthWalk({ steps = 18, cadenceHz = 1.8, sampleHz = 50, amp = 2.5, sigma = 0.08, noise = 0.05, leadMs = 700 } = {}) {
  const dt = 1000 / sampleHz;
  const period = 1000 / cadenceHz;
  const totalMs = leadMs + steps * period + 500;
  const events = [];
  for (let k = 0; k < steps; k++) events.push(leadMs + k * period + period / 2);
  const samples = [];
  for (let t = 0; t <= totalMs; t += dt) {
    let az = 9.80665;
    for (const e of events) {
      const d = (t - e) / 1000;
      az += amp * Math.exp(-(d * d) / (2 * sigma * sigma));
    }
    samples.push({
      t,
      ax: (Math.random() - 0.5) * noise,
      ay: (Math.random() - 0.5) * noise,
      az: az + (Math.random() - 0.5) * noise,
    });
  }
  return { samples, steps };
}

// Feed a sample stream through a fresh detector, returning the emitted step
// timestamps (a walk is only "confirmed" after qualifySteps rhythmic peaks).
function runSteps(det, samples) {
  const times = [];
  for (const s of samples) for (const st of det.push(s)) times.push(st);
  return times;
}

{
  // Clean walk: count should match truth (buffered steps flush on confirmation).
  const det = new StepDetector();
  const { samples, steps } = synthWalk({ steps: 18, noise: 0 });
  const times = runSteps(det, samples);
  let maxCadence = 0;
  for (let i = 1; i < times.length; i++) maxCadence = Math.max(maxCadence, 1000 / (times[i] - times[i - 1]));
  check('step detector counts a clean walk', Math.abs(times.length - steps) <= 2, `detected ${times.length}/${steps}`);
  check('cadence is in a plausible walking range', maxCadence > 1.2 && maxCadence < 2.6, `max ${maxCadence.toFixed(2)} Hz`);

  // Noisy walk: still within ~20%.
  const noisy = synthWalk({ steps: 20, noise: 0.5 });
  const n = runSteps(new StepDetector(), noisy.samples).length;
  check('step detector tolerates sensor noise', Math.abs(n - noisy.steps) <= noisy.steps * 0.2, `detected ${n}/${noisy.steps}`);

  // A standing pause must not be counted as a slow step.
  const detP = new StepDetector();
  const a = synthWalk({ steps: 8, leadMs: 700 });
  const rest = [];
  const gap = a.samples[a.samples.length - 1].t;
  for (let t = gap + 50; t < gap + 3500; t += 20) rest.push({ t, ax: 0, ay: 0, az: 9.80665 });
  const b = synthWalk({ steps: 8, leadMs: 0 });
  const shift = gap + 3500 - b.samples[0].t;
  const merged = [...a.samples, ...rest, ...b.samples.map((s) => ({ ...s, t: s.t + shift }))];
  const p = runSteps(detP, merged).length;
  check('a standing pause is not counted as steps', Math.abs(p - 16) <= 2, `detected ${p}/16`);

  // The whole point of the gate: isolated movements ("moved a bit") must not
  // register as steps.
  const isolated = [];
  let t = 0;
  for (let k = 0; k < 6; k++) {
    // one Gaussian bump, then 2.5 s of stillness, six times
    for (let s = 0; s < 4; s++, t += 20) {
      const d = (s - 1.5) * 0.02;
      isolated.push({ t, ax: 0, ay: 0, az: 9.80665 + 2.5 * Math.exp(-(d * d) / (2 * 0.04 * 0.04)) });
    }
    for (let s = 0; s < 120; s++, t += 20) isolated.push({ t, ax: 0, ay: 0, az: 9.80665 });
  }
  const iso = runSteps(new StepDetector(), isolated).length;
  check('isolated movements do not count as steps', iso === 0, `detected ${iso}`);

  // Light non-rhythmic jitter (e.g. fidgeting) must not count either.
  const jitter = [];
  for (let i = 0, tt = 0; i < 600; i++, tt += 20) {
    jitter.push({ t: tt, ax: (Math.random() - 0.5) * 0.8, ay: (Math.random() - 0.5) * 0.8, az: 9.80665 + (Math.random() - 0.5) * 0.8 });
  }
  const jit = runSteps(new StepDetector(), jitter).length;
  check('non-rhythmic jitter does not count as steps', jit === 0, `detected ${jit}`);
}

{
  // Time-based smoothing (tau seconds) is frame-rate independent.
  const h = new HeadingTracker({ tau: 0.5 });
  let out = 0;
  for (let i = 0; i < 60; i++) out = h.pushCompass(0, i * 20);
  check('heading converges toward its compass input', Math.abs(wrapAngle(out)) < 0.02, `${out.toFixed(4)} rad`);

  // A +90 deg compass step smooths to ~+90 deg without crossing the seam.
  for (let i = 0; i < 200; i++) out = h.pushCompass(Math.PI / 2, 2000 + i * 20);
  check('heading tracks a 90 deg turn', Math.abs(wrapAngle(out - Math.PI / 2)) < 0.05, `${(out * 180 / Math.PI).toFixed(1)} deg`);

  // A one-sample jitter spike must barely move the smoothed heading.
  const j = new HeadingTracker({ tau: 0.5 });
  for (let i = 0; i < 100; i++) j.pushCompass(0, i * 20);
  const spike = j.pushCompass(10 * Math.PI / 180, 100 * 20);
  check('compass jitter is attenuated', Math.abs(wrapAngle(spike)) < 0.5 * Math.PI / 180,
    `${(spike * 180 / Math.PI).toFixed(2)} deg`);

  // Gyro bridging: turning right increases the bearing when the compass is stale.
  const g = new HeadingTracker({ gyroSign: -1, gyroBridgeAfterMs: 0 });
  g.pushCompass(0, 0);
  g.pushGyro(0, 0);
  const before = g.heading;
  g.pushGyro(-Math.PI, 100);   // 180 deg/s about +z for 100 ms
  check('gyro bridges a right turn upward', wrapAngle(g.heading - before) > 0.2, `+${((g.heading - before) * 180 / Math.PI).toFixed(1)} deg`);

  // ...but not while the compass is fresh (that would only add jitter).
  const g2 = new HeadingTracker({ gyroSign: -1, gyroBridgeAfterMs: 250 });
  g2.pushCompass(0, 1000);
  g2.pushGyro(0, 1000);
  g2.pushGyro(-Math.PI, 1100);   // compass only 100 ms old
  check('gyro does not perturb a fresh compass', Math.abs(wrapAngle(g2.heading)) < 1e-9, `${(g2.heading * 180 / Math.PI).toFixed(2)} deg`);

  // Compass bias calibration shifts the reported heading.
  const c = new HeadingTracker();
  c.pushCompass(0, 0);
  c.calibrate(10 * Math.PI / 180);
  let biased = 0;
  for (let i = 0; i < 200; i++) biased = c.pushCompass(0, i * 20);
  check('compass bias calibration applies', Math.abs(wrapAngle(biased - 10 * Math.PI / 180)) < 0.02, `${(biased * 180 / Math.PI).toFixed(1)} deg`);
}

{
  // Step length scales with cadence and clamps.
  const m = new StepLengthModel({ base: 0.72, cadenceRef: 1.8, min: 0.5, max: 0.9 });
  check('step length at reference cadence', Math.abs(m.lengthFor(1.8) - 0.72) < 1e-9);
  check('step length grows with cadence', m.lengthFor(2.4) > m.lengthFor(1.2));
  check('step length clamps at the maximum', m.lengthFor(10) === 0.9);
  check('step length clamps at the minimum', m.lengthFor(0.01) === 0.5);

  const cal = new WalkCalibrator();
  cal.start();
  for (let i = 0; i < 20; i++) cal.addStep();
  check('walk calibration divides distance by steps', Math.abs(cal.finish(14.4) - 0.72) < 1e-9);
  cal.start();
  check('walk calibration rejects a zero-step walk', cal.finish(10) === null);
}

{
  // Orientation normalisation for both platforms.
  const ios = headingFromOrientation({ webkitCompassHeading: 90 });
  check('iOS webkitCompassHeading -> north-cw bearing', ios && ios.absolute && Math.abs(ios.heading - Math.PI / 2) < 1e-9);
  const north = headingFromOrientation({ alpha: 0, absolute: true, type: 'deviceorientationabsolute' });
  check('alpha=0 is north', north && Math.abs(north.heading) < 1e-9, `${north.heading}`);
  const east = headingFromOrientation({ alpha: 270, absolute: true, type: 'deviceorientationabsolute' });
  check('alpha=270 is east (90 deg cw)', east && Math.abs(wrapAngle(east.heading - Math.PI / 2)) < 1e-9, `${(east.heading * 180 / Math.PI).toFixed(1)} deg`);
  check('event with no heading returns null', headingFromOrientation({}) === null);

  // createMotion must prefer the absolute orientation event: Chrome fires both
  // deviceorientationabsolute and a non-compass deviceorientation, and the
  // latter must not overwrite the good heading.
  const listeners = {};
  const win = {
    addEventListener: (type, fn) => { (listeners[type] ||= []).push(fn); },
    removeEventListener: () => {},
    performance: { now: () => 0 },
  };
  let last = null;
  const m = createMotion({ win, onHeading: (h) => { last = h; } });
  m.start();   // no permission needed, listeners registered synchronously
  const fire = (type, ev) => (listeners[type] || []).forEach((fn) => fn(ev));
  fire('deviceorientationabsolute', { type: 'deviceorientationabsolute', absolute: true, alpha: 90, timeStamp: 0 });
  const absHeading = last && last.heading;
  check('absolute orientation is used', last && last.absolute === true && Math.abs(wrapAngle(absHeading - wrapAngle(270 * Math.PI / 180))) < 1e-6,
    last ? `${(absHeading * 180 / Math.PI).toFixed(1)} deg` : 'none');
  // The plain deviceorientation event is ignored entirely - it is relative,
  // and index.html never used it for the compass.
  fire('deviceorientation', { type: 'deviceorientation', absolute: false, alpha: 10, timeStamp: 1 });
  check('plain deviceorientation is ignored',
    last && last.absolute === true && Math.abs(wrapAngle(last.heading - absHeading)) < 1e-9);
  fire('deviceorientation', { type: 'deviceorientation', absolute: true, alpha: 200, timeStamp: 6000 });
  check('plain deviceorientation is ignored even if it claims absolute',
    last && last.absolute === true && Math.abs(wrapAngle(last.heading - absHeading)) < 1e-9);

  // Only deviceorientationabsolute and (iOS) webkitCompassHeading are trusted
  // by default; the plain event is tagged relative.
  check('headingFromOrientation marks a plain event relative',
    headingFromOrientation({ type: 'deviceorientation', alpha: 45, absolute: true })?.absolute === false);

  // In relative mode the plain event drives the heading (calibrated later).
  const relListeners = {};
  const relWin = {
    addEventListener: (type, fn) => { (relListeners[type] ||= []).push(fn); },
    removeEventListener: () => {},
    performance: { now: () => 0 },
  };
  let relLast = null;
  const relMotion = createMotion({ win: relWin, headingMode: 'relative', onHeading: (h) => { relLast = h; } });
  relMotion.start();
  (relListeners.deviceorientationabsolute || []).forEach((fn) =>
    fn({ type: 'deviceorientationabsolute', alpha: 90, absolute: true, timeStamp: 0 }));
  check('relative mode ignores the absolute event', relLast === null);
  (relListeners.deviceorientation || []).forEach((fn) =>
    fn({ type: 'deviceorientation', alpha: 90, timeStamp: 10 }));
  check('relative mode uses the plain event', relLast != null && relLast.absolute === false,
    relLast ? `${(relLast.heading * 180 / Math.PI).toFixed(1)} deg` : 'none');
}

console.log(`\n${failures === 0 ? 'ALL TESTS PASSED' : failures + ' FAILURES'}`);process.exit(failures === 0 ? 0 : 1);
