/**
 * Smoke test for the tiered campus router, run against the real building
 * JSONs in this directory.
 *
 *   node smoke.mjs                        # everything (the gate)
 *   node smoke.mjs motion pdr arbiter autocal   # a fast subset (no floor load)
 *   node smoke.mjs router                 # one category
 *   node smoke.mjs --list                 # categories
 *
 * Categories only gate which sections RUN; selected groups still share the
 * one-time building load, which is skipped entirely when no selected category
 * needs the real floors.
 */
import { readFileSync } from 'fs';
import { Worker } from 'node:worker_threads';
import { availableParallelism } from 'node:os';
import { buildGraph, buildGraphState, graphFromState } from './navgraph.mjs';
import { navGraphInputs } from './navgraph-source.mjs';
import { extractPortals, buildCampusRouter, buildOutdoorGraph, buildObstacleIndex, shapeSegments, cachePortals, portalsFromCache, isWallLayer, isDoorLayer } from './campus.mjs';
import { loadOsmShapes } from './osm.mjs';
import { pmtilesGetZxy } from './pmtiles-node.mjs';
import { convexHull, hullSamples, bestFitRigid, bestFitTranslation, meanPoint } from './align.mjs';

import { DXF_SCALE, metresPerDegree, makeToLL } from './geo.mjs';
import { StepDetector, HeadingTracker, StepLengthModel, WalkCalibrator, wrapAngle, headingFromOrientation, createMotion } from './motion.mjs';
import { GpsStepCalibrator, calibrationConfidence } from './autocal.mjs';
import { AnchorStepCalibrator } from './anchorcal.mjs';
import { ParticleFilter, corridorAxis, HeadingDriftCalibrator } from './locate.mjs';
import { buildPassages, makeHeadingField, CompassSnapper, distPi } from './headingfield.mjs';
import { PositionArbiter } from './arbiter.mjs';
import { normName, placeTarget, parseTarget, buildPlaceIndex, searchPlaces, splitRoomQuery, findRoom, findRoomExact, resolvePlace, validatePlaces } from './places.mjs';
import { doorwaysFromShapes, barrierCrossesPolyline } from './doorways.mjs';
import { resolveRoomPin } from './roompin.mjs';

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

/** Pretty-print a route (used by the router and multi-floor groups). */
function showRoute(name, r) {
  if (!r) { console.log(`  ${name}: null`); return; }
  console.log(`  ${name}: ${r.summary} | ${Math.round(r.totalMeters)}m, ${Math.round(r.totalSeconds)}s`);
  for (const s of r.segments) {
    const geom = s.polyline ? `${s.polyline.length} pts` : 'vertical';
    console.log(`    - ${s.tier.padEnd(8)} ${s.building ?? ''} ${s.floor ?? (s.fromFloor ? s.fromFloor + '->' + s.toFloor : '')} ${geom} ${Math.round(s.meters)}m ${Math.round(s.seconds)}s`);
  }
}

// --- test categories --------------------------------------------------------
// Run a subset to avoid the full ~40 s suite while iterating:
//   node smoke.mjs motion autocal pdr      (fast, no building load)
//   node smoke.mjs router                  (building-loading groups)
//   node smoke.mjs                         (everything, the gate)
const ALL_CATEGORIES = [
  'align',       // floor auto-align (synthetic)
  'walkability', // navgraph isWalkable/nearestWalkable over real floors
  'pdr',         // particle filter (synthetic maps)
  'corridor',    // hallway lock trigger rate on the real floors
  'arbiter',     // GPS/PDR arbiter (synthetic map)
  'portals',     // portal sanity + portal cache (real floors)
  'router',      // building router + exit reachability + tests 1-8
  'osm',         // osm-paths cache + live OSM network
  'multifloor',  // multi-floor routing, skybridges, connector anchors
  'motion',      // StepDetector / HeadingTracker / StepLengthModel
  'autocal',     // GPS step-length auto-calibration
  'anchorcal',   // step-length refinement from manual anchors
  'headingfield', // allowed compass directions + compass snapping
  'doorways',    // DOOR-layer clustering and wall-aligned barriers
  'roompin',     // strict hallway-door resolution (real AS121-02)
  'places',      // place nicknames, places.json, favorites
];
const _args = process.argv.slice(2).filter((a) => !a.startsWith('-'));
if (process.argv.includes('--list') || process.argv.includes('-h') || process.argv.includes('--help')) {
  console.log('Usage: node smoke.mjs [category ...]   (no args = run everything)\n');
  console.log('Categories:');
  for (const c of ALL_CATEGORIES) console.log(`  ${c}`);
  console.log('\nExamples:');
  console.log('  node smoke.mjs motion pdr arbiter autocal   # fast, no building load');
  console.log('  node smoke.mjs router                        # building-loading groups');
  process.exit(0);
}
const _unknown = _args.filter((a) => !ALL_CATEGORIES.includes(a));
if (_unknown.length) {
  console.error(`Unknown category: ${_unknown.join(', ')}`);
  console.error(`Categories: ${ALL_CATEGORIES.join(', ')}`);
  process.exit(2);
}
const ACTIVE = _args.length ? new Set(_args) : new Set(ALL_CATEGORIES);
const want = (c) => ACTIVE.has(c);
// Which groups need the (slow) real-floor load: wall raster + portals.
const NEEDS_FLOORS = ['walkability', 'portals', 'router', 'osm', 'multifloor', 'corridor'];
const needsFloors = NEEDS_FLOORS.some(want);
if (_args.length) console.log(`Running categories: ${[...ACTIVE].join(', ')}\n`);

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
  const walls = data.shapes.filter(s => isWallLayer(s.layer));
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

const t0 = performance.now();
// ONE JSON parse + wall pass + buildGraph + fresh portal extraction per floor,
// shared by every test group below (built in parallel workers). The suite used
// to redo this per group - the multi-floor block rebuilt every navGraph a
// second time for its cached-router comparison.
// SKIPPED entirely when no selected category needs the real floors (so
// `node smoke.mjs motion pdr arbiter autocal` runs in a couple of seconds).
let floorBasis = [];
if (needsFloors) {
  console.log('Loading buildings...');
  floorBasis = await buildFloorBasis();
}

// The older tests deliberately work on a ground-floor-only campus.
const buildings = !needsFloors ? [] : CONFIG.map((cfg, i) => {
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
if (needsFloors) console.log(`Loaded in ${Math.round(performance.now() - t0)}ms\n`);

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

// ---------------------------------------------------------------------------
// Floor auto-align (align.mjs) - synthetic rigid recovery for the import
// tool's "align to ground".
// ---------------------------------------------------------------------------
if (want('align')) {
console.log('\nFloor auto-align:');
  const corners = [[0, 0], [100, 0], [100, 60], [0, 60]];
  const hull = convexHull(corners.concat([[50, 30], [10, 10]]));
  check('convexHull drops interior points', hull.length === 4, `${hull.length} pts`);
  const ring = hullSamples(corners, 10);
  check('hullSamples walks the perimeter', ring.length >= 30 && ring.length <= 34,
    `${ring.length} points`);
  check('hullSamples of a degenerate set is empty', hullSamples([[0, 0], [1, 1]], 10).length === 0);

  // Asymmetric pentagon so the recovered rotation is unambiguous (a plain
  // rectangle has a 180° twin).
  const shape = [[0, 0], [100, 0], [100, 40], [55, 65], [0, 60]];
  const reference = hullSamples(shape, 2);
  const alpha = (-35 * Math.PI) / 180;
  const shift = [123, -47];
  const ca = Math.cos(alpha), sa = Math.sin(alpha);
  const current = reference.map(([x, y]) =>
    [ca * x - sa * y + shift[0], sa * x + ca * y + shift[1]]);

  const norm = (a) => {
    a = a % (2 * Math.PI);
    if (a > Math.PI) a -= 2 * Math.PI;
    if (a < -Math.PI) a += 2 * Math.PI;
    return a;
  };
  const fit = bestFitRigid(current, reference);
  check('bestFitRigid recovers the rotation (scan seed)',
    !!fit && Math.abs(norm(fit.rotation - 35 * Math.PI / 180)) < 0.01,
    fit ? `${(fit.rotation * 180 / Math.PI).toFixed(2)}°` : 'null');
  check('bestFitRigid residual ~ 0 on exact data', !!fit && fit.residual < 0.05,
    fit ? `${fit.residual.toFixed(4)} m` : 'null');
  // Applying the fit to the current points must land on the reference.
  const c2 = Math.cos(fit.rotation), s2 = Math.sin(fit.rotation);
  const moved = current.map(([x, y]) =>
    [c2 * x - s2 * y + fit.translation[0], s2 * x + c2 * y + fit.translation[1]]);
  let maxNN = 0;
  for (const m of moved) {
    let nn = Infinity;
    for (const r of reference) nn = Math.min(nn, Math.hypot(m[0] - r[0], m[1] - r[1]));
    maxNN = Math.max(maxNN, nn);
  }
  check('bestFitRigid maps current onto reference', maxNN < 0.2, `max nn ${maxNN.toFixed(3)} m`);

  const seeded = bestFitRigid(current, reference, { rotation: 0.4 });
  check('bestFitRigid seeded path converges too',
    !!seeded && Math.abs(norm(seeded.rotation - 35 * Math.PI / 180)) < 0.01 &&
    seeded.residual < 0.05,
    seeded ? `${(seeded.rotation * 180 / Math.PI).toFixed(2)}°, ${seeded.residual.toFixed(4)} m` : 'null');
  check('bestFitRigid handles empty input', bestFitRigid([], reference) === null &&
    bestFitRigid(current, []) === null);

  // The import tool creates a building before it has been placed. makeToLL
  // must still yield finite coordinates (it used to throw on undefined origin,
  // which silently blanked the first render of a new building).
  const bare = makeToLL({ id: 'NEW' }, { floorId: '01' });
  const bareP = bare(1000, 500);
  check('makeToLL defaults an unplaced building', bareP.every(Number.isFinite),
    bareP.map(v => v.toFixed(6)).join(', '));
  const placed = makeToLL(
    { origin: { lon: -149.83, lat: 61.19 }, offset: { x: 400, y: 300, rot: 0 } }, {});
  const [cx, cy] = placed(400, 300);
  check('makeToLL maps the plan datum onto its origin',
    Math.abs(cx + 149.83) < 1e-12 && Math.abs(cy - 61.19) < 1e-12);

  // Translation-only alignment: the import tool locks a floor's rotation to
  // its parent, then solves position. No rotation may be introduced.
  const shifted = reference.map(([x, y]) => [x + 37, y - 19]);
  const tfit = bestFitTranslation(shifted, reference);
  check('bestFitTranslation recovers a pure translation',
    !!tfit && Math.abs(tfit.translation[0] + 37) < 0.05 &&
    Math.abs(tfit.translation[1] - 19) < 0.05 && tfit.residual < 0.05,
    tfit ? `t ${tfit.translation.map(v => v.toFixed(2)).join(', ')}; ` +
      `residual ${tfit.residual.toFixed(4)} m` : 'null');

  const c0 = meanPoint(reference);
  const cth = Math.cos(0.35), sth = Math.sin(0.35);
  const spun = reference.map(([x, y]) => {
    const dx = x - c0[0], dy = y - c0[1];
    return [cth * dx - sth * dy + c0[0], sth * dx + cth * dy + c0[1]];
  });
  const rfit = bestFitTranslation(spun, reference);
  check('bestFitTranslation leaves rotation alone',
    !!rfit && Math.hypot(rfit.translation[0], rfit.translation[1]) < 10 &&
    rfit.residual > 1,
    rfit ? `t ${Math.hypot(rfit.translation[0], rfit.translation[1]).toFixed(2)} m, ` +
      `residual ${rfit.residual.toFixed(2)} m` : 'null');
  check('bestFitTranslation handles empty input',
    bestFitTranslation([], reference) === null && bestFitTranslation(current, []) === null);
}

// The PDR particle filter relies on these O(1) accessors staying consistent
// with the router's clearance raster.
if (want('walkability')) {
console.log('\nWalkability accessor:');
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
// Hallway lock (corridorAxis) on the real floor plans. The aspect gate is what
// decides how often the lock can fire; validate a lower gate fires it more
// often across real walkable space without becoming trigger-happy.
// ---------------------------------------------------------------------------
if (want('corridor')) {
console.log('\nHallway lock on real floors (corridorAxis trigger rate):');
  const OLD_RATIO = 9, NEW_RATIO = 6, MAXM = 17, STRIDE_M = 2.5, CAP = 120;
  let oldHit = 0, newHit = 0, total = 0;
  for (const floors of floorBasis) {
    for (const fb of floors) {
      const g = fb.navGraph;
      let x0 = Infinity, y0 = Infinity, x1 = -Infinity, y1 = -Infinity;
      for (const [[ax, ay], [bx, by]] of fb.lines) {
        x0 = Math.min(x0, ax, bx); x1 = Math.max(x1, ax, bx);
        y0 = Math.min(y0, ay, by); y1 = Math.max(y1, ay, by);
      }
      if (!isFinite(x0)) continue;
      const mpd = metresPerDegree((y0 + y1) / 2);
      const dLon = STRIDE_M / mpd.lon, dLat = STRIDE_M / mpd.lat;
      let n = 0;
      for (let lon = x0; lon <= x1 && n < CAP; lon += dLon) {
        for (let lat = y0; lat <= y1 && n < CAP; lat += dLat) {
          const p = [lon, lat];
          if (!g.isWalkable(p)) continue;
          n++; total++;
          if (corridorAxis(g.isWalkable, p, { aspectRatio: OLD_RATIO, maxM: MAXM }) != null) oldHit++;
          if (corridorAxis(g.isWalkable, p, { aspectRatio: NEW_RATIO, maxM: MAXM }) != null) newHit++;
        }
      }
    }
  }
  const rateOld = oldHit / Math.max(1, total), rateNew = newHit / Math.max(1, total);
  check('the lower aspect gate fires the hallway lock more often on real floors',
    rateNew > rateOld * 1.3 && rateNew < 0.6,
    `ratio ${OLD_RATIO}: ${(rateOld * 100).toFixed(1)}% -> ${NEW_RATIO}: ${(rateNew * 100).toFixed(1)}% (${total} samples)`);

  // The precomputed heading field (corridors + junctions + doorways) must exist
  // on every real floor and offer allowed directions along real walkable space.
  {
    let floors = 0, withField = 0, tested = 0, snaps = 0;
    for (const fbs of floorBasis) for (const fb of fbs) {
      floors++;
      if (fb.navGraph && fb.navGraph.headingField) withField++;
    }
    check('every real floor builds a heading field',
      floors > 0 && withField === floors, `${withField}/${floors}`);
    for (const fbs of floorBasis) for (const fb of fbs) {
      const g = fb.navGraph;
      if (!g || !g.headingField) continue;
      let x0 = Infinity, y0 = Infinity, x1 = -Infinity, y1 = -Infinity;
      for (const [[ax, ay], [bx, by]] of fb.lines) {
        x0 = Math.min(x0, ax, bx); x1 = Math.max(x1, ax, bx);
        y0 = Math.min(y0, ay, by); y1 = Math.max(y1, ay, by);
      }
      if (!isFinite(x0)) continue;
      const mpd = metresPerDegree((y0 + y1) / 2);
      const dLon = STRIDE_M / mpd.lon, dLat = STRIDE_M / mpd.lat;
      let n = 0;
      for (let lon = x0; lon <= x1 && n < CAP; lon += dLon) {
        for (let lat = y0; lat <= y1 && n < CAP; lat += dLat) {
          const p = [lon, lat];
          if (!g.isWalkable(p)) continue;
          n++; tested++;
          const q = g.headingField.query(p);
          if (q && q.confidence >= 0.4) snaps++;
        }
      }
    }
    check('real walkable space offers allowed directions to snap to',
      tested > 0 && snaps / Math.max(1, tested) > 0.15,
      `${(100 * snaps / Math.max(1, tested)).toFixed(1)}% of ${tested}`);
  }
}

// ---------------------------------------------------------------------------
// PDR particle filter (locate.mjs) - synthetic maps, no sensors required.
// ---------------------------------------------------------------------------
if (want('pdr')) {
console.log('\nPDR particle filter:');
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

  // The 9:1 aspect gate rejects a merely squarish space.
  const squat = (p) => Math.abs((p[0] - lon0) * mpd.lon) < 2 && Math.abs((p[1] - lat0) * mpd.lat) < 1;
  check('corridorAxis rejects a 2:1 space', corridorAxis(squat, [lon0, lat0]) === null);

  // A 1.5 m wide corridor whose axis is 20 degrees off north resolves to it.
  const rot = 20 * Math.PI / 180;
  const rotWalkable = (p) => {
    const e = (p[0] - lon0) * mpd.lon, n = (p[1] - lat0) * mpd.lat;
    return Math.abs(e * Math.cos(rot) - n * Math.sin(rot)) < 0.75;
  };
  const axr = corridorAxis(rotWalkable, [lon0, lat0]);
  check('corridorAxis resolves a rotated corridor',
    axr != null && Math.abs(wrapAngle(axr - rot)) < 3 * Math.PI / 180,
    axr == null ? 'null' : `${(axr * 180 / Math.PI).toFixed(1)} deg`);

  // Hallway heading-drift corrector: arm after 6 straight steps, pull a biased
  // heading back to the corridor axis, and stop on an intentional turn.
  {
    const DEG = Math.PI / 180;
    const axis = corridorAxis(corridorWalkable, [lon0, lat0]);
    let bias = 12 * DEG;
    const cal = new HeadingDriftCalibrator();
    let firstActive = -1;
    for (let k = 0; k < 20; k++) {
      const heading = wrapAngle(Math.PI / 2 + bias);
      const r = cal.update(heading, axis);
      if (r.active && firstActive < 0) firstActive = k;
      bias = wrapAngle(bias + r.correctionRad);
    }
    check('hallway correction arms only after 6 straight steps', firstActive === 5, `first active @${firstActive}`);
    check('hallway correction converges the heading bias', Math.abs(bias) < 3 * DEG,
      `${(bias / DEG).toFixed(1)} deg left`);

    const noAxis = new HeadingDriftCalibrator();
    let activeNoAxis = false;
    for (let k = 0; k < 12; k++) {
      if (noAxis.update(wrapAngle(Math.PI / 2 + 10 * DEG), null).active) activeNoAxis = true;
    }
    check('no hallway correction without a corridor axis', !activeNoAxis);

    const turnCal = new HeadingDriftCalibrator();
    for (let k = 0; k < 8; k++) turnCal.update(Math.PI / 2, axis);
    const turned = turnCal.update(Math.PI, axis);            // 90 deg step
    check('an intentional turn disarms hallway correction',
      !turned.active && turned.reason === 'turn', turned.reason);
    let rearmed = false;
    for (let k = 0; k < 7; k++) { if (turnCal.update(Math.PI, 0).active) rearmed = true; }
    check('hallway correction re-arms after a turn', rearmed);

    const mismatch = new HeadingDriftCalibrator();
    for (let k = 0; k < 6; k++) mismatch.update(0, 0);
    const bad = mismatch.update(0, 80 * DEG);
    check('a large heading/axis mismatch is distrusted',
      !bad.active && bad.reason === 'mismatch', bad.reason);

    const curve = new HeadingDriftCalibrator();
    for (let k = 0; k < 8; k++) curve.update(k * 4 * DEG, 0);
    check('a slow curve resets the straight streak', curve.streak < 6, `streak ${curve.streak}`);
  }

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

  // The cloud is exposed for on-map display, weight-normalised.
  {
    const pts = filt.points();
    const wsum = pts.reduce((s, p) => s + p.w, 0);
    check('particle cloud is exposed for display',
      pts.length === filt.count && Math.abs(wsum - 1) < 1e-9, `${pts.length} pts`);
    check('an unseeded filter exposes no points',
      new ParticleFilter({ isWalkable: () => true }).points().length === 0);
  }

  // Free roam (outside a building): the floor-plan constraint is lifted, so
  // the cloud walks freely off the corridor - a skybridge or the gap between
  // buildings leaves the raster entirely.
  const roam = new ParticleFilter({ isWalkable: corridorWalkable, nearestWalkable: corridorNearest,
    count: 400, headingNoiseDeg: 5, distanceNoise: 0.05, lateralNoiseM: 0.02 });
  roam.setFreeRoam(true);
  roam.seed(toLL(0, 8), { spreadM: 0.5, heading: 0, headingSpreadDeg: 5 });   // 8 m off the corridor
  for (let k = 0; k < 10; k++) roam.predict(0.7, 0);                          // walk further north
  est = roam.estimate();
  const roamN = (est.lat - lat0) * mpd.lat;
  check('free roam moves PDR freely outside the floor plan',
    roamN > 12, `${roamN.toFixed(1)} m north (seeded at 8, +7 walked)`);
  const pinned = new ParticleFilter({ isWalkable: corridorWalkable, nearestWalkable: corridorNearest,
    count: 400, headingNoiseDeg: 5, distanceNoise: 0.05, lateralNoiseM: 0.02 });
  pinned.seed(toLL(0, 8), { spreadM: 0.5, heading: 0, headingSpreadDeg: 5 });
  for (let k = 0; k < 10; k++) pinned.predict(0.7, 0);
  est = pinned.estimate();
  const pinnedN = Math.abs((est.lat - lat0) * mpd.lat);
  check('without free roam the cloud is pinned to the plan',
    pinnedN < 2, `${pinnedN.toFixed(1)} m off the corridor`);

  // Constrained free roam ("outside all buildings on this floor"): forbidden
  // footprints are down-weighted, so a cloud straddling one is pushed to the
  // allowed side instead of walking through it.
  const allowedNorth = (p) => (p[1] - lat0) * mpd.lat >= 0;   // forbid n < 0
  const mkRoam = (allow) => {
    const f = new ParticleFilter({ isWalkable: corridorWalkable, nearestWalkable: corridorNearest,
      count: 400, headingNoiseDeg: 5, distanceNoise: 0.05, lateralNoiseM: 0.02 });
    f.setFreeRoam(true, allow);
    f.seed(toLL(0, 0), { spreadM: 3, heading: Math.PI / 2, headingSpreadDeg: 5 });
    for (let k = 0; k < 4; k++) f.predict(0.7, Math.PI / 2);
    return (f.estimate().lat - lat0) * mpd.lat;
  };
  const constrainedN = mkRoam(allowedNorth);
  check('constrained free roam pushes PDR off a forbidden footprint',
    constrainedN > 0.6, `${constrainedN.toFixed(2)} m north`);
  const looseN = Math.abs(mkRoam(null));
  check('unconstrained free roam stays symmetric', looseN < 0.6, `${looseN.toFixed(2)} m off`);

  // --- wall / doorway behaviour ------------------------------------------
  // Two rooms split by a north-south wall at e=0 with a 0.9 m doorway at n=0.
  // The wall blocks a 0.6 m band (thickness + body clearance), mirroring the
  // navgraph raster. Rooms span |e| < 6, |n| < 3.
  const roomWalkable = (p) => {
    const e = (p[0] - lon0) * mpd.lon, n = (p[1] - lat0) * mpd.lat;
    if (Math.abs(e) >= 6 || Math.abs(n) >= 3) return false;
    if (Math.abs(e) <= 0.3 && Math.abs(n) > 0.45) return false;   // wall; doorway open
    return true;
  };
  const roomNearest = (p) => {
    const e = (p[0] - lon0) * mpd.lon, n = (p[1] - lat0) * mpd.lat;
    if (roomWalkable(p)) return p;
    if (Math.abs(e) <= 0.3) return toLL(e < 0 ? -0.31 : 0.31, n);
    return toLL(Math.max(-5.9, Math.min(5.9, e)), Math.max(-2.9, Math.min(2.9, n)));
  };
  const mkRoom = (opts = {}) => new ParticleFilter({
    isWalkable: roomWalkable, nearestWalkable: roomNearest,
    count: 120, headingNoiseDeg: 4, distanceNoise: 0.05, lateralNoiseM: 0.03, ...opts,
  });

  // Tunnelling: a single step aimed at the solid wall (away from the door)
  // must not cross it. (Respawning pinned to the mean here, so the check is
  // about the swept STEP, not where a killed particle is re-spawned.)
  {
    const f = mkRoom({ respawnSpreadM: 0 });
    f.seed(toLL(-1.0, 2.0), { spreadM: 0.05, heading: Math.PI / 2, headingSpreadDeg: 2 });
    f.predict(0.7, Math.PI / 2, { accuracyM: 10 });
    let crossed = 0;
    for (let i = 0; i < f.count; i++) {
      const e = (f.lon[i] - lon0) * mpd.lon, n = (f.lat[i] - lat0) * mpd.lat;
      if (e > 0.3 && Math.abs(n) > 0.45) crossed++;   // the far side of the solid wall
    }
    check('particles do not tunnel through a wall', crossed === 0, `${crossed} crossed`);
  }
  // A real doorway is walkable: aimed at the gap, the cloud crosses.
  {
    const f = mkRoom();
    f.seed(toLL(-1.0, 0.0), { spreadM: 0.1, heading: Math.PI / 2, headingSpreadDeg: 5 });
    for (let k = 0; k < 8; k++) f.predict(0.7, Math.PI / 2, { accuracyM: 12 });
    const e = (f.estimate().lon - lon0) * mpd.lon;
    check('a doorway lets the cloud cross', e > 1.0, `e=${e.toFixed(2)} m`);
  }
  // A particle blocked by a wall is destroyed and re-spawned from a surviving
  // particle (reachable), never a fresh random point across a wall.
  {
    const f = mkRoom();
    f.seed(toLL(-1.5, 1.4), { spreadM: 0.1, heading: Math.PI / 2, headingSpreadDeg: 5 });
    for (let k = 0; k < 12; k++) f.predict(0.7, Math.PI / 2, { accuracyM: 20 });
    let offPlan = 0;
    for (let i = 0; i < f.count; i++) if (!roomWalkable([f.lon[i], f.lat[i]])) offPlan++;
    check('re-spawned particles stay on walkable ground (no cross-wall spawns)',
      offPlan === 0, `${offPlan} off-plan`);
  }
  // The reported estimate is walkable even with the cloud split across the wall.
  {
    const f = new ParticleFilter({ isWalkable: roomWalkable, nearestWalkable: roomNearest, count: 200 });
    f.seed([lon0, lat0], { spreadM: 1.5, heading: 0 });
    check('estimate is never inside a wall', roomWalkable([f.estimate().lon, f.estimate().lat]));
  }
  // Stall guard: a SEALED room (no opening for wall-slide to reach) leaves the
  // cloud unable to advance, so the guard must arm after a few steps.
  {
    const boxWalkable = (p) => {
      const e = (p[0] - lon0) * mpd.lon, n = (p[1] - lat0) * mpd.lat;
      return Math.abs(e) < 0.05 && Math.abs(n) < 0.05;
    };
    const boxNearest = (p) => {
      const e = (p[0] - lon0) * mpd.lon, n = (p[1] - lat0) * mpd.lat;
      return toLL(Math.max(-0.049, Math.min(0.049, e)), Math.max(-0.049, Math.min(0.049, n)));
    };
    const f = new ParticleFilter({ isWalkable: boxWalkable, nearestWalkable: boxNearest,
      count: 120, stallSteps: 3, relaxSteps: 5 });
    f.seed(toLL(0, 0), { spreadM: 0.1, heading: Math.PI / 2, headingSpreadDeg: 5 });
    for (let k = 0; k < 6; k++) f.predict(0.7, Math.PI / 2, { accuracyM: 10 });
    check('a sealed room arms the stall guard', f.relaxLeft > 0, `relaxLeft=${f.relaxLeft}`);
  }
  // Heading prior: a known heading seeds within +/-90 deg.
  {
    const f = new ParticleFilter({ isWalkable: () => true, count: 300 });
    f.seed([lon0, lat0], { spreadM: 1, heading: 0 });
    let maxDev = 0;
    for (let i = 0; i < f.count; i++) {
      maxDev = Math.max(maxDev, Math.abs(wrapAngle(f.heading[i])));
    }
    check('known heading seeds within +/-90 deg', maxDev <= Math.PI / 2 + 1e-9,
      `${(maxDev * 180 / Math.PI).toFixed(1)} deg`);
  }
  // Accuracy radius: the trusted radius bounds the cloud spread.
  {
    const f = new ParticleFilter({ isWalkable: () => true, count: 300 });
    f.seed([lon0, lat0], { spreadM: 5, heading: Math.PI / 2, headingSpreadDeg: 90 });
    f.predict(0.7, Math.PI / 2, { accuracyM: 1.0 });
    const spread = f.estimate().spreadM;
    check('accuracy radius bounds the cloud spread', spread < 2.0, `${spread.toFixed(2)} m`);
  }
  // Rear-hemisphere prior: particles behind the mean lose weight.
  {
    const f = new ParticleFilter({ isWalkable: () => true, count: 400, behindWeight: 0.05 });
    f.seed([lon0, lat0], { spreadM: 2, heading: Math.PI / 2, headingSpreadDeg: 30 });
    f.predict(0.7, Math.PI / 2, { accuracyM: Infinity });
    const mean = f._weightedMean();
    let wRear = 0, nRear = 0, wFront = 0, nFront = 0;
    for (let i = 0; i < f.count; i++) {
      const de = (f.lon[i] - mean.lon) * mpd.lon;
      if (de < 0) { wRear += f.weight[i]; nRear++; } else { wFront += f.weight[i]; nFront++; }
    }
    check('particles behind the heading are down-weighted',
      wRear / Math.max(1, nRear) < wFront / Math.max(1, nFront),
      `rear ${(wRear / Math.max(1, nRear)).toExponential(1)} vs front ${(wFront / Math.max(1, nFront)).toExponential(1)}`);
  }
  // U-turn: a ~180 deg heading change is flagged (priors lifted that step).
  {
    const f = new ParticleFilter({ isWalkable: () => true, count: 300 });
    f.seed([lon0, lat0], { spreadM: 1, heading: Math.PI / 2, headingSpreadDeg: 30 });
    for (let k = 0; k < 5; k++) f.predict(0.7, Math.PI / 2, { accuracyM: 10 });
    f.predict(0.7, -Math.PI / 2, { accuracyM: 10 });
    check('a U-turn is flagged for the priors', f.reversed === true);
  }

}

// ---------------------------------------------------------------------------
// GPS/PDR arbiter (arbiter.mjs) - synthetic map, no GPS or sensors required.
// ---------------------------------------------------------------------------
if (want('arbiter')) {
console.log('\nGPS/PDR arbiter:');
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

  // An indoor fix must NOT re-seed the cloud (even a "good" one can be metres
  // wrong inside and would drag every particle to the wrong place); an outdoor
  // good fix still resets the drift.
  const [ilon, ilat] = toLL(12, 0);          // on the corridor: indoor, walkable
  const driftBefore = arb.metresSinceCorrection;
  arb.gps({ lon: ilon, lat: ilat, accuracy: 3, t: 20000 });
  check('an indoor fix does not re-seed the cloud',
    arb.metresSinceCorrection === driftBefore && driftBefore > 0,
    `${driftBefore} -> ${arb.metresSinceCorrection} m`);
  arb.gps({ lon: glon, lat: glat, accuracy: 3, t: 21000 });
  check('an outdoor good fix resets the PDR drift counter',
    arb.metresSinceCorrection === 0, `${arb.metresSinceCorrection} m`);

  // Outdoors PDR drives the estimate (the cloud is free, not tied to the
  // floor plan); only a VERY good fix (<= 3 m) overrides it - and re-seeds.
  arb.anchor(toLL(0, 0), { spreadM: 1.0 });
  for (let k = 0; k < 5; k++) arb.step({ stepLengthM: 0.7, headingRad: 0 });
  const drift5 = arb.metresSinceCorrection;
  arb.gps({ lon: glon, lat: glat, accuracy: 5, t: 22000 });   // mediocre outdoor fix
  arb.estimate(22000);
  const mediocre = arb.estimate(26000);
  check('a mediocre outdoor fix does not override free-roaming PDR',
    mediocre.source === 'pdr', `${mediocre.source} @${mediocre.gpsAccuracyM} m`);
  check('...and does not re-seed the cloud',
    arb.metresSinceCorrection === drift5, `${drift5} -> ${arb.metresSinceCorrection} m`);
  arb.gps({ lon: glon, lat: glat, accuracy: 2, t: 27000 });   // very good outdoor fix
  arb.estimate(27000);
  const goodAfter = arb.estimate(31000);                      // > hysteresis
  check('a very good outdoor fix overrides PDR',
    goodAfter.source === 'gps', `${goodAfter.source}`);
  check('...and re-seeds the cloud', arb.metresSinceCorrection === 0,
    `${arb.metresSinceCorrection} m`);

  // Free roam follows indoor/outdoor: outdoor fixes lift the floor-plan
  // constraint (skybridges, gaps between buildings), indoor fixes restore it.
  const roamArb = new PositionArbiter({
    filter: new ParticleFilter({ isWalkable, nearestWalkable, count: 100 }),
    isWalkable, nearestWalkable,
  });
  roamArb.gps({ lon: glon, lat: glat, accuracy: 5, t: 0 });      // outdoor
  check('outdoor fixes free the cloud', roamArb.filter.freeRoam === true);
  roamArb.gps({ lon: ilon, lat: ilat, accuracy: 5, t: 1000 });   // on the corridor
  check('indoor fixes re-constrain the cloud', roamArb.filter.freeRoam === false);

  // Going outdoors re-anchors ONCE (the app may have snapped the cloud into a
  // building); after that only a very good fix corrects it.
  const outArb = new PositionArbiter({
    filter: new ParticleFilter({ isWalkable, nearestWalkable, count: 100 }),
    isWalkable, nearestWalkable,
  });
  outArb.anchor(toLL(0, 0), { spreadM: 1 });
  for (let k = 0; k < 5; k++) outArb.step({ stepLengthM: 0.7, headingRad: 0 });
  const beforeOut = outArb.metresSinceCorrection;
  outArb.gps({ lon: glon, lat: glat, accuracy: 12, t: 0 });      // first outdoor fix (mediocre)
  check('the first outdoor fix re-anchors the cloud',
    beforeOut > 0 && outArb.metresSinceCorrection === 0,
    `${beforeOut} -> ${outArb.metresSinceCorrection} m`);
  for (let k = 0; k < 5; k++) outArb.step({ stepLengthM: 0.7, headingRad: 0 });
  outArb.gps({ lon: glon, lat: glat, accuracy: 12, t: 1000 });   // later mediocre fix
  check('later mediocre outdoor fixes do not re-anchor',
    outArb.metresSinceCorrection > 0, `${outArb.metresSinceCorrection} m`);

  // Outdoors, a fix MORE ACCURATE THAN THE PDR ESTIMATE re-seeds the cloud
  // even if it is not "very good" (over 3 m); a fix worse than PDR does not.
  const rsArb = new PositionArbiter({
    filter: new ParticleFilter({ isWalkable, nearestWalkable, count: 100 }),
    isWalkable, nearestWalkable,
  });
  rsArb.gps({ lon: glon, lat: glat, accuracy: 2, t: 0 });        // outdoor mode established
  rsArb.anchor(toLL(0, 0), { spreadM: 1 });
  for (let k = 0; k < 40; k++) rsArb.step({ stepLengthM: 0.7, headingRad: 0 });   // ~28 m
  const rsPdrAcc = rsArb.pdrAccuracy();                          // ~4.7 m
  rsArb.gps({ lon: glon, lat: glat, accuracy: rsPdrAcc + 2, t: 1000 });
  check('an outdoor fix worse than the PDR estimate does not re-seed',
    rsArb.metresSinceCorrection > 0, `${rsArb.metresSinceCorrection.toFixed(1)} m`);
  rsArb.gps({ lon: glon, lat: glat, accuracy: rsPdrAcc - 1, t: 2000 });
  check('an outdoor fix better than the PDR estimate re-seeds',
    rsArb.metresSinceCorrection === 0, `PDR ±${rsPdrAcc.toFixed(1)} m`);

  // A fix flickering indoor<->outdoor near a doorway must not re-anchor the
  // cloud every time it reads outdoors (the "jumps 20 ft" report).
  const flArb = new PositionArbiter({
    filter: new ParticleFilter({ isWalkable, nearestWalkable, count: 100 }),
    isWalkable, nearestWalkable,
  });
  flArb.gps({ lon: glon, lat: glat, accuracy: 4, t: 0 });      // first outdoor fix: seeds
  flArb.anchor(toLL(0, 0), { spreadM: 1 });
  for (let k = 0; k < 5; k++) flArb.step({ stepLengthM: 0.7, headingRad: 0 });
  const driftFlicker = flArb.metresSinceCorrection;
  flArb.gps({ lon: ilon, lat: ilat, accuracy: 4, t: 1000 });   // indoor flicker
  flArb.gps({ lon: glon, lat: glat, accuracy: 12, t: 2000 });  // outdoors again (mediocre)
  check('an indoor/outdoor fix flicker does not re-anchor the cloud',
    flArb.metresSinceCorrection === driftFlicker,
    `${driftFlicker} -> ${flArb.metresSinceCorrection} m`);

  // A first plausible fix (even a coarse one) must seed the filter so PDR is
  // live - otherwise steps are counted but never move anything (the index.html
  // "PDR not working" bug).
  const coarseFilter = new ParticleFilter({ isWalkable, nearestWalkable, count: 100 });
  const coarseArb = new PositionArbiter({ filter: coarseFilter, isWalkable, nearestWalkable });
  check('filter starts unseeded', coarseFilter.initialized === false);
  coarseArb.gps({ lon: glon, lat: glat, accuracy: 40, t: 0 });
  check('a coarse first fix seeds the filter', coarseFilter.initialized === true);

  // Entering a building is IMMEDIATE: an indoor/implausible fix hands over to
  // PDR on the spot, with no hysteresis window.
  arb.anchor(toLL(0, 0), { spreadM: 1.0 });
  arb.gps({ lon: glon, lat: glat, accuracy: 1, t: 30000 });   // outdoor, clearly better
  arb.estimate(30000);
  arb.estimate(34000);                                        // commit GPS
  arb.gps({ lon: wlon, lat: wlat, accuracy: 5, t: 34000 });   // implausible (in a wall)
  const indoorNow = arb.estimate(34400);                      // 0.4 s later
  check('an indoor fix hands over to PDR immediately',
    indoorNow.source === 'pdr', indoorNow.source);

  // ...but LEAVING a building still switches back through normal hysteresis.
  arb.gps({ lon: glon, lat: glat, accuracy: 1, t: 35000 });   // outdoor, clearly better
  const leaveMid = arb.estimate(35400);                       // < hysteresis
  const leaveAfter = arb.estimate(39000);                     // > hysteresis
  check('leaving a building still switches back through hysteresis',
    leaveMid.source === 'pdr' && leaveAfter.source === 'gps',
    `${leaveMid.source} -> ${leaveAfter.source}`);

  // "I'm Here" cancels GPS: clearGps must forget the fix (and its held copy)
  // so a stale, wrong fix cannot win the arbitration back after the anchor.
  arb.gps({ lon: glon, lat: glat, accuracy: 1, t: 40000 });
  arb.estimate(40000);
  arb.clearGps();
  const cleared = arb.estimate(40500);
  check('clearGps forgets the fix and yields to PDR',
    cleared.source === 'pdr' && cleared.gpsAccuracyM === null,
    `${cleared.source} gpsAcc ${cleared.gpsAccuracyM}`);
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

if (want('portals')) {
console.log('Portal sanity:');
check('every building has exits', buildings.every(b => b.floors[0].exits.length > 0));
check('AS103 has elevators', buildings.find(b => b.id === 'AS103').floors[0].elevators.length > 0);
check('AS102 has no elevator layer (expected)', buildings.find(b => b.id === 'AS102').floors[0].elevators.length === 0);
check('every multi-floor building has a vertical connector',
  buildings.every(b => b.floors.length < 2 ||
    b.floors.some(f => (f.stairs?.length ?? 0) + (f.elevators?.length ?? 0) > 0)));
  const as105Idx = CONFIG.findIndex(c => c.id === 'AS105');
  const as105Floors = as105Idx >= 0 ? floorBasis[as105Idx] : [];
  check('AS105 splits the two-car elevator bank (floors 01-03)',
    as105Floors.length >= 3 &&
    as105Floors.slice(0, 3).every(f => f.freshPortals.elevators.length === 2),
    as105Floors.map(f => f.freshPortals.elevators.length).join(','));
  const as105cfg = CONFIG.find(c => c.id === 'AS105');
  check('AS105 cached portals carry both cars',
    !!as105cfg && as105cfg.floors.slice(0, 3).every(f =>
      (f.portals?.elevators?.length ?? 0) === 2));
  const as111Idx = CONFIG.findIndex(c => c.id === 'AS111');
  const as111Floors = as111Idx >= 0 ? floorBasis[as111Idx] : [];
  check('AS111 detects its plural "Elevators" layer (floor 01)',
    as111Floors.length >= 1 && as111Floors[0].freshPortals.elevators.length === 2,
    as111Floors.map(f => f.freshPortals.elevators.length).join(','));
  const as111cfg = CONFIG.find(c => c.id === 'AS111');
  check('AS111 cached portals include the floor-01 elevators',
    !!as111cfg && (as111cfg.floors[0].portals?.elevators?.length ?? 0) === 2);
check('wall layers accept singular and plural forms',
  ['wall', 'walls', 'WALL', 'Walls', 'window', 'windows', 'WINDOWS'].every(isWallLayer) &&
  !isWallLayer('doors') && !isWallLayer(undefined));
check('door layers accept DOOR and DOORS',
  ['door', 'doors', 'DOOR', 'Doors'].every(isDoorLayer) &&
  !isDoorLayer('wall') && !isDoorLayer(undefined));

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
      .filter(s => isDoorLayer(s.layer))
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
}
if (want('portals')) {
console.log('\nPortal cache:');
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

if (want('router')) {
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
  // 10 m keeps at least one line in the narrow gaps between buildings (25 m
  // could leave a 23 m gap uncovered after the campus bbox changed, making the
  // router legitimately fall back to a clear straight outdoor leg).
  const stepM = 10;
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
  const OSM_URL = 'https://uaa-campus-map.net/scak.pmtiles';
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
}
if (want('osm')) {
console.log('\nOSM paths cache:');
  // Named-road crossings add a heavy cost; following a road or crossing an
  // unnamed minor road (parking aisle) does not.
  {
    const lat = 61.19, m = metresPerDegree(lat);
    const A = [-149.84, lat], B = [-149.839, lat];
    const roadPts = [[-149.8395, lat - 0.001], [-149.8395, lat + 0.001]];
    const cross = { layer: 'PATHS', kind: 'path', name: null,
      items: [{ op: 'line', points: [A, B] }] };
    const roadShape = (kind, name) => ({ layer: 'PATHS', kind, name,
      items: [{ op: 'line', points: roadPts }] });
    const cost = shape => {
      const g = buildOutdoorGraph([cross, shape], lat, null);
      return { w: g.dijkstraFrom(g.snap(A).node).dist[g.snap(B).node],
               along: g.roadPenaltyLL(roadPts[0], roadPts[1]) };
    };
    const named = cost(roadShape('major_road', 'Providence Drive'));
    const unnamedMinor = cost(roadShape('minor_road', null));
    const piper = cost(roadShape('minor_road', 'Piper Street'));
    const seawolf = cost(roadShape('minor_road', 'Seawolf Drive'));
    const spirit = cost(roadShape('minor_road', 'Spirit Way'));
    const plain = cost({ layer: 'PATHS', kind: 'path', name: null,
      items: [{ op: 'line', points: roadPts }] });
    check('crossing a named road adds a heavy outdoor cost',
      named.w - plain.w >= 600, `${plain.w.toFixed(0)} -> ${named.w.toFixed(0)}`);
    check('a named minor road (Piper Street) is penalised',
      piper.w - plain.w >= 600, `${plain.w.toFixed(0)} -> ${piper.w.toFixed(0)}`);
    check('Spirit Way is exempt from the crossing penalty',
      spirit.w - plain.w < 1, `${plain.w.toFixed(0)} -> ${spirit.w.toFixed(0)}`);
    check('Seawolf Drive is exempt from the crossing penalty',
      seawolf.w - plain.w < 1, `${plain.w.toFixed(0)} -> ${seawolf.w.toFixed(0)}`);
    check('walking along a named road is not penalised', named.along === 0);
    check('an unnamed minor road is not penalised',
      unnamedMinor.w - plain.w < 1,
      `${plain.w.toFixed(0)} -> ${unnamedMinor.w.toFixed(0)}`);
  }

  // CAD walls are split into sub-metre fragments. The obstacle index must see
  // them, or a straight outdoor leg can cut through a building whose exterior
  // wall is drawn in small pieces (the AS162 bug).
  {
    const scale = metresPerDegree(61.19);
    const lon0 = -149.83, lat0 = 61.19;
    const segs = [];
    for (let i = 0; i < 20; i++) {
      segs.push([[lon0 + i / scale.lon, lat0],
                 [lon0 + (i + 1) / scale.lon, lat0]]);
    }
    const idx = buildObstacleIndex(segs, scale);
    const mid = lon0 + 10.5 / scale.lon;
    const north = lat0 + 5 / scale.lat, south = lat0 - 5 / scale.lat;
    check('a fragmented wall blocks a crossing',
      idx.crosses(mid * scale.lon, north * scale.lat,
                  mid * scale.lon, south * scale.lat, 1));
    check('a crossing near the query end is still allowed',
      !idx.crosses(mid * scale.lon, north * scale.lat,
                   mid * scale.lon, (lat0 - 0.5 / scale.lat) * scale.lat, 2));
  }
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
    // Generated bboxes are rounded in JSON; the app allows a sub-metre
    // boundary tolerance so a portal exactly on the pad does not force a live
    // PMTiles decode. Mirror it here.
    const eps = 1e-7;
    check('osm-paths.json covers the campus bbox',
      !!c && bb.minLon >= c.minLon - eps && bb.maxLon <= c.maxLon + eps &&
      bb.minLat >= c.minLat - eps && bb.maxLat <= c.maxLat + eps);
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
if (want('multifloor')) {
console.log('\nMulti-floor routing (all real floors):');
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

  // HSB (AS156) lands on floor 02 while EIB (AS162) lands on floor 03, via
  // Parrish Bridge (AS164) on floor 03. The route must change floors on the
  // bridge, not descend to the ground.
  {
    const sHSB = walkableRoom('AS156', '02');
    const eEIB = walkableRoom('AS162', '03');
    const rHB = sHSB && eEIB
      ? routerAll.route(sHSB, eEIB, { startFloor: '02', endFloor: '03' })
      : null;
    showRoute('AS156 f02 -> AS162 f03 (cross-floor skybridge)', rHB);
    check('HSB f02 -> EIB f03 uses cross-floor skybridge',
      !!rHB && rHB.segments.some(s => s.tier === 'skybridge') &&
      rHB.segments.some(s => s.tier === 'skybridge' &&
        s.fromFloor === '02' && s.toFloor === '03') &&
      !rHB.segments.some(s => s.tier === 'outdoor'));
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

if (want('multifloor')) {
console.log('\nAuto-detected skybridge (synthetic):');
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

  // Cross-floor auto pairing: X's bridge is on floor 02, Y's on floor 03.
  // They are still the only pair within the cap, so auto-detection must link
  // them and the route must carry the floor change across the bridge.
  const mkCross = (cx, west, bridgeFloor) => {
    const floors = ['01', '02', '03'].map(fid => ({
      floorId: fid, ...square(cx, lat0), elevators: [],
      stairs: [{ id: `s${fid}`, ll: [cx, lat0] }], exits: [], skybridges: [],
    }));
    floors.find(f => f.floorId === bridgeFloor).skybridges = [{
      id: 'bridge', ll: [cx + (west ? -1 : 1) * dx * 0.9, lat0],
    }];
    return { id: west ? 'XC' : 'YC', groundFloor: '01', floors };
  };
  const XC = mkCross(lon0, false, '02');
  const YC = mkCross(yx, true, '03');
  const routerC = buildCampusRouter([XC, YC]);
  const rC = routerC.route([lon0, lat0], [yx, lat0],
    { startFloor: '02', endFloor: '03' });
  showRoute('XC f02 -> YC f03 via auto cross-floor bridge', rC);
  check('auto-detected bridge links across floor numbers',
    routerC.stats.skybridgeEdges === 1 &&
    rC?.segments.some(s => s.tier === 'skybridge' &&
      s.fromFloor === '02' && s.toFloor === '03') &&
    !rC.segments.some(s => s.tier === 'elevator' || s.tier === 'stairs' || s.tier === 'outdoor'));
}

if (want('multifloor')) {
console.log('\nExplicit skybridge destination (synthetic):');
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

if (want('multifloor')) {
console.log('\nConnector anchors without a ground-floor stop (synthetic):');
  // A stairwell that exists on floors 02 and 03 but not 01: the first stop
  // anchors the connector and the other floor links to it. Before the anchor
  // fix, the second floor's lookup of the missing ground-floor stop passed
  // `undefined` into distLL and crashed buildCampusRouter.
  const lat0 = 61.17, lon0 = -149.86;
  const mpd = metresPerDegree(lat0);
  const half = 6;
  const dx = half / mpd.lon, dy = half / mpd.lat;
  const square = (cx, cy) => {
    const c = [[cx - dx, cy - dy], [cx + dx, cy - dy], [cx + dx, cy + dy], [cx - dx, cy + dy]];
    const lines = c.map((p, i) => [p, c[(i + 1) % 4]]);
    return { navGraph: buildGraph(lines), obstacleLines: lines };
  };
  const mk = (floorId, stairs) => ({
    floorId, ...square(lon0, lat0), elevators: [], stairs, exits: [],
  });
  const b = {
    id: 'X', groundFloor: '01',
    floors: [
      mk('01', []),
      mk('02', [{ id: 's2', ll: [lon0, lat0] }]),
      mk('03', [{ id: 's3', ll: [lon0, lat0] }]),
    ],
  };
  let router = null, threw = null;
  try { router = buildCampusRouter([b]); } catch (e) { threw = e; }
  check('builds when the ground floor lacks that connector kind', !threw,
    threw ? String(threw) : 'ok');
  const r = router && router.route([lon0, lat0], [lon0, lat0],
    { startFloor: '02', endFloor: '03' });
  showRoute('X f02 -> X f03 (anchored stairwell)', r);
  check('upper-floor stops still link vertically',
    !!r && r.segments.some(s => s.tier === 'stairs') &&
    !r.segments.some(s => s.tier === 'outdoor'));
}

if (want('multifloor')) {
console.log('\nConnector keys stay unique within a floor:');
  // Two elevator cars on both floors. Blocking one car must not remove the
  // other; the old key was kind+floor, so both shared one key.
  const latC = 61.155, lonC = -149.875;
  const mpdC = metresPerDegree(latC);
  const halfC = 8;
  const dxC = halfC / mpdC.lon, dyC = halfC / mpdC.lat;
  const squareC = (cx, cy) => {
    const c = [[cx - dxC, cy - dyC], [cx + dxC, cy - dyC], [cx + dxC, cy + dyC], [cx - dxC, cy + dyC]];
    const lines = c.map((p, i) => [p, c[(i + 1) % 4]]);
    return { navGraph: buildGraph(lines), obstacleLines: lines };
  };
  const carA = [lonC - 1.5 / mpdC.lon, latC];
  const carB = [lonC + 1.5 / mpdC.lon, latC];
  const carFloor = (floorId) => ({
    floorId, ...squareC(lonC, latC), elevators: [
      { id: 'car-a-' + floorId, ll: carA }, { id: 'car-b-' + floorId, ll: carB },
    ], stairs: [], exits: [],
  });
  const carRouter = buildCampusRouter([{ id: 'CARS', groundFloor: '01',
    floors: [carFloor('01'), carFloor('02')] }]);
  const carRoute = carRouter.route([lonC, latC], [lonC, latC],
    { startFloor: '01', endFloor: '02' });
  const carPortals = (carRoute?.portals ?? []).filter((p) => p.kind === 'connector');
  check('two elevator cars on a floor have distinct portal keys',
    carPortals.length >= 2 && new Set(carPortals.map((p) => p.key)).size === carPortals.length,
    carPortals.map((p) => p.key).join(', '));
  const blockedCar = carPortals[0];
  const carAlt = blockedCar && carRouter.route([lonC, latC], [lonC, latC],
    { startFloor: '01', endFloor: '02', avoidPortals: [blockedCar.key] });
  check('blocking one car leaves the other usable',
    !!carAlt && carAlt.segments.some((s) => s.tier === 'elevator') &&
    !(carAlt.portals ?? []).some((p) => p.key === blockedCar.key));
}

if (want('multifloor')) {
console.log('\nPortal preference (force stairs / elevators):');
  const lat0 = 61.16, lon0 = -149.87;
  const mpd = metresPerDegree(lat0);
  const half = 6;
  const dx = half / mpd.lon, dy = half / mpd.lat;
  const square = (cx, cy) => {
    const c = [[cx - dx, cy - dy], [cx + dx, cy - dy], [cx + dx, cy + dy], [cx - dx, cy + dy]];
    const lines = c.map((p, i) => [p, c[(i + 1) % 4]]);
    return { navGraph: buildGraph(lines), obstacleLines: lines };
  };
  const mkCF = (floorId) => ({
    floorId, ...square(lon0, lat0),
    elevators: [{ id: 'e', ll: [lon0, lat0] }],
    stairs: [{ id: 's', ll: [lon0, lat0] }],
    exits: [],
  });
  const mkTall = (n) => ({
    id: 'X', groundFloor: '01',
    floors: Array.from({ length: n }, (_, i) => mkCF(String(i + 1).padStart(2, '0'))),
  });
  const rung = (n, ignore) => buildCampusRouter([mkTall(n)], null, ignore)
    .route([lon0, lat0], [lon0, lat0], { startFloor: '01', endFloor: String(n).padStart(2, '0') });

  // 2 floors: stairs (6 s) beat the elevator (15 + 4 s wait). Forcing the
  // elevator must still deliver an elevator leg.
  const rBoth = rung(2, null);
  showRoute('X f01 -> X f02 (both available)', rBoth);
  check('with both available the cheaper connector wins (stairs)',
    !!rBoth && rBoth.segments.some(s => s.tier === 'stairs') &&
    !rBoth.segments.some(s => s.tier === 'elevator'));
  const rElev = rung(2, 'stairs');
  showRoute('X f01 -> X f02 (elevator forced)', rElev);
  check('ignoring stairs forces the elevator',
    !!rElev && rElev.segments.some(s => s.tier === 'elevator') &&
    !rElev.segments.some(s => s.tier === 'stairs'));

  // 12 floors: the elevator (15 + 4/f) beats stairs (6/f) from 8 floors up.
  // Forcing stairs must override that preference.
  const rE = rung(12, null);
  showRoute('X f01 -> X f12 (both available)', rE);
  check('with both available the elevator wins on a tall building',
    !!rE && rE.segments.some(s => s.tier === 'elevator') &&
    !rE.segments.some(s => s.tier === 'stairs'));
  const rS = rung(12, 'elevator');
  showRoute('X f01 -> X f12 (stairs forced)', rS);
  check('ignoring elevators forces stairs',
    !!rS && rS.segments.some(s => s.tier === 'stairs') &&
    !rS.segments.some(s => s.tier === 'elevator'));
}

if (want('multifloor')) {
console.log('\nBasement vertical travel and explicit bridge conflicts:');
  const latB = 61.15, lonB = -149.88;
  const mpdB = metresPerDegree(latB);
  const halfB = 5;
  const dxB = halfB / mpdB.lon, dyB = halfB / mpdB.lat;
  const sqB = (cx, cy) => {
    const c = [[cx - dxB, cy - dyB], [cx + dxB, cy - dyB], [cx + dxB, cy + dyB], [cx - dxB, cy + dyB]];
    const lines = c.map((p, i) => [p, c[(i + 1) % 4]]);
    return { navGraph: buildGraph(lines), obstacleLines: lines };
  };
  const stairB = [lonB, latB];
  const baseFloors = ['B2', 'B1', '01'].map((floorId) => ({
    floorId, ...sqB(lonB, latB), elevators: [],
    stairs: [{ id: 'st-' + floorId, ll: stairB }], exits: [],
  }));
  const baseRouter = buildCampusRouter([{ id: 'BASE', groundFloor: '01', floors: baseFloors }]);
  const baseRoute = baseRouter.route(stairB, stairB, { startFloor: 'B2', endFloor: '01' });
  const baseStairs = (baseRoute?.segments ?? []).find((s) => s.tier === 'stairs');
  check('B2 -> ground counts two floors of stairs', !!baseStairs && baseStairs.seconds === 12,
    baseStairs ? `${baseStairs.seconds}s` : 'no route');

  // A says B, B says C: A-B must not pair just because A wants B.
  const edge = (id, to, x) => ([{
    floorId: '01', ...sqB(x, latB), elevators: [], stairs: [], exits: [],
    skybridges: [{ id: 'br-' + id, ll: [x, latB + dyB], to }],
  }]);
  const brRouter = buildCampusRouter([
    { id: 'A', groundFloor: '01', floors: edge('A', 'B', lonB - 0.002) },
    { id: 'B', groundFloor: '01', floors: edge('B', 'C', lonB) },
    { id: 'C', groundFloor: '01', floors: edge('C', null, lonB + 0.002) },
  ]);
  check('conflicting explicit bridge destinations do not pair',
    brRouter.stats.skybridgeEdges === 1, `${brRouter.stats.skybridgeEdges} bridge edge(s)`);
}

if (want('multifloor')) {
console.log('\nElevated-only structures are not ground doors:');
  // AS164 (Parrish Bridge) exists only on floor 03. Its doors must never
  // become outdoor doors, or a route can step from an outdoor leg straight
  // onto floor 03 with no vertical connector (the HSB 301 bug).
  const lat0 = 61.15, lon0 = -149.88;
  const mpd = metresPerDegree(lat0);
  const half = 6;
  const dx = half / mpd.lon, dy = half / mpd.lat;
  const square = (cx, cy) => {
    const c = [[cx - dx, cy - dy], [cx + dx, cy - dy], [cx + dx, cy + dy], [cx - dx, cy + dy]];
    const lines = c.map((p, i) => [p, c[(i + 1) % 4]]);
    return { navGraph: buildGraph(lines), obstacleLines: lines };
  };
  const mk = (id, floorId, cx) => ({
    id, groundFloor: floorId,
    floors: [{ floorId, ...square(cx, lat0), elevators: [], stairs: [],
               exits: [{ id: 'd', ll: [cx, lat0] }] }],
  });
  const A = mk('A', '01', lon0);
  const B = mk('B', '03', lon0 + 40 / mpd.lon);
  const r = buildCampusRouter([A, B]).route(
    [lon0, lat0], [lon0 + 40 / mpd.lon, lat0],
    { startFloor: '01', endFloor: '03' });
  check('a level-3-only building is not reachable by an outdoor door', !r);
}

// ---------------------------------------------------------------------------
// PDR signal processing (motion.mjs) - synthetic gait, no sensors required.
// ---------------------------------------------------------------------------
if (want('motion')) {
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

  // Holding still with light RHYTHMIC jostling (hand tremor, a foot tap on the
  // phone, a shaky table): peaks below the absolute intensity gate must not
  // register even though the spacing is walk-like. This is the "too sensitive
  // when holding still" bug - spurious steps drift PDR while standing.
  const jostle = (amp) => {
    const out = [];
    for (let i = 0, tt = 0; i < 400; i++, tt += 20) {
      const ph = (i % 25) / 25;
      out.push({ t: tt, ax: 0, ay: 0,
        az: 9.80665 + amp * Math.exp(-((ph - 0.5) ** 2) / 0.02) });
    }
    return out;
  };
  const lightJostle = runSteps(new StepDetector(), jostle(0.5)).length;
  check('light rhythmic jostling does not count as steps', lightJostle === 0,
    `detected ${lightJostle}`);
  const strongWalk = runSteps(new StepDetector(), jostle(2.5)).length;
  check('...but the same rhythm at step intensity still counts', strongWalk > 5,
    `detected ${strongWalk}`);

  // Only movement ALONG gravity counts. Rotating/tilting the phone sweeps
  // gravity across the device axes; a rhythmic horizontal shove moves |a| but
  // not the vertical component. Neither is walking.
  const roll = [];
  for (let i = 0, tt = 0; i < 250; i++, tt += 20) {
    const ang = (i / 250) * Math.PI * 2;             // one slow full roll
    roll.push({ t: tt, ax: 9.80665 * Math.sin(ang), ay: 0,
      az: 9.80665 * Math.cos(ang) });
  }
  check('rotating the phone is not counted as steps',
    runSteps(new StepDetector(), roll).length === 0,
    `detected ${runSteps(new StepDetector(), roll).length}`);

  const shove = (onZ) => {
    const out = [];
    for (let i = 0, tt = 0; i < 500; i++, tt += 20) {
      const s = 5 * Math.sin((i / 25) * 2 * Math.PI);   // 2 Hz, |a| swings hard
      out.push({ t: tt, ax: onZ ? 0 : s, ay: 0, az: 9.80665 + (onZ ? s : 0) });
    }
    return out;
  };
  const horiz = runSteps(new StepDetector(), shove(false)).length;
  check('rhythmic horizontal shoving is not counted as steps', horiz === 0,
    `detected ${horiz}`);
  // Sanity: the OLD magnitude signal WOULD fire on this shove - so the
  // projection onto gravity is what rejects it, not the intensity gate.
  {
    let base = null, oldPeaks = 0;
    for (const { ax, ay, az } of shove(false)) {
      const mag = Math.hypot(ax, ay, az);
      if (base == null) { base = mag; continue; }
      base += 0.02 * (mag - base);
      if (mag - base > 0.8) oldPeaks++;
    }
    check('the old magnitude signal would have fired here (regression guard)',
      oldPeaks > 3, `${oldPeaks} peaks`);
  }
  const vert = runSteps(new StepDetector(), shove(true)).length;
  check('...but the same rhythm along gravity still counts', vert > 5,
    `detected ${vert}`);

  // Band-pass: high-frequency jitter (hand vibration) riding on a real walk is
  // rejected, so the count tracks the gait, not the vibration.
  {
    const hfWalk = [];
    const dtMs = 20, cadence = 1.8, period = 1000 / cadence, steps = 16, leadMs = 700;
    const total = leadMs + steps * period + 500;
    for (let t = 0; t <= total; t += dtMs) {
      let az = 9.80665;
      for (let k = 0; k < steps; k++) {
        const d = (t - (leadMs + k * period + period / 2)) / 1000;
        az += 2.5 * Math.exp(-(d * d) / (2 * 0.08 * 0.08));
      }
      az += 1.0 * Math.sin(2 * Math.PI * 8 * (t / 1000));   // 8 Hz vibration
      hfWalk.push({ t, ax: (Math.random() - 0.5) * 0.05, ay: (Math.random() - 0.5) * 0.05, az });
    }
    const c = runSteps(new StepDetector(), hfWalk).length;
    check('high-frequency jitter on a walk is filtered out', Math.abs(c - steps) <= 3,
      `detected ${c}/${steps}`);
  }

  // Hysteresis/envelope: a step with a secondary bump 300 ms later (a "double
  // bump" from footfall + hand) counts once, not twice.
  {
    const doubleWalk = [];
    const dtMs = 20, cadence = 1.8, period = 1000 / cadence, steps = 16, leadMs = 700;
    const total = leadMs + steps * period + 500;
    for (let t = 0; t <= total; t += dtMs) {
      let az = 9.80665;
      for (let k = 0; k < steps; k++) {
        const e = leadMs + k * period + period / 2;
        const d1 = (t - e) / 1000;
        az += 2.5 * Math.exp(-(d1 * d1) / (2 * 0.08 * 0.08));
        const d2 = (t - (e + 300)) / 1000;
        az += 1.1 * Math.exp(-(d2 * d2) / (2 * 0.05 * 0.05));   // 44% secondary bump
      }
      doubleWalk.push({ t, ax: (Math.random() - 0.5) * 0.05, ay: (Math.random() - 0.5) * 0.05, az });
    }
    const c = runSteps(new StepDetector(), doubleWalk).length;
    check('a double-bump counts as one step', Math.abs(c - steps) <= 3, `detected ${c}/${steps}`);
  }

  // A held movement with ripple is not a walk.
  {
    const plateau = [];
    for (let i = 0, tt = 0; i < 150; i++, tt += 20) {
      const hold = tt > 600 && tt < 1400 ? 2.5 : 0;
      plateau.push({ t: tt, ax: 0, ay: 0,
        az: 9.80665 + hold + 0.6 * Math.sin(2 * Math.PI * 10 * (tt / 1000)) });
    }
    const c = runSteps(new StepDetector(), plateau).length;
    check('a held movement with ripple is not counted', c <= 1, `detected ${c}`);
  }
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

  // The raw (uncorrected) compass is exposed next to the corrected heading, so
  // the readout can show both. Same smoothing, before the bias/trim.
  {
    const r = new HeadingTracker({ tau: 0.3 });
    for (let i = 0; i < 200; i++) r.pushCompass(20 * Math.PI / 180, i * 20);
    check('raw heading tracks the compass with no bias',
      Math.abs(wrapAngle(r.rawHeading - 20 * Math.PI / 180)) < 0.02,
      `${(r.rawHeading * 180 / Math.PI).toFixed(1)} deg`);
    r.calibrate(30 * Math.PI / 180);
    for (let i = 200; i < 400; i++) r.pushCompass(20 * Math.PI / 180, i * 20);
    check('raw heading ignores the calibrated bias',
      Math.abs(wrapAngle(r.heading - 50 * Math.PI / 180)) < 0.02 &&
      Math.abs(wrapAngle(r.rawHeading - 20 * Math.PI / 180)) < 0.02,
      `corrected ${(r.heading * 180 / Math.PI).toFixed(1)}, raw ${(r.rawHeading * 180 / Math.PI).toFixed(1)}`);
  }
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

  // iOS permission: ONE request is made (the shared prompt enables both APIs).
  // The old code awaited DeviceMotion then called DeviceOrientation, whose
  // request rejects once the user activation is spent - the catch bailed
  // without registering listeners ("granted but the walking model does
  // nothing"). Firing both in one tick lets the second cancel the first.
  {
    const permListeners = {};
    let motionRequests = 0, orientRequests = 0;
    let resolveMotion;
    let motionStarted = false;
    const permWin = {
      addEventListener: (t, fn) => { (permListeners[t] ||= []).push(fn); },
      removeEventListener: () => {},
      performance: { now: () => 0 },
      DeviceMotionEvent: {
        requestPermission: () => {
          motionRequests++;
          return new Promise((res) => { resolveMotion = () => res('granted'); });
        },
      },
      DeviceOrientationEvent: {
        requestPermission: () => { orientRequests++; return Promise.reject(new Error('NotAllowedError')); },
      },
    };
    const pm = createMotion({ win: permWin, onStatus: (s) => { if (s.type === 'started') motionStarted = true; } });
    const startingP = pm.start();
    check('only the accelerometer permission is requested', motionRequests === 1 && orientRequests === 0,
      `motion=${motionRequests} orient=${orientRequests}`);
    resolveMotion();
    await startingP;
    check('sensors start after the grant', motionStarted);
    check('both listeners register', (permListeners.devicemotion || []).length === 1 &&
      (permListeners.deviceorientation || []).length === 1);
  }

  // Falls back to the orientation permission when no motion API exists.
  {
    let orientRequests = 0, started = false;
    const fallbackWin = {
      addEventListener: () => {}, removeEventListener: () => {},
      performance: { now: () => 0 },
      DeviceOrientationEvent: { requestPermission: () => { orientRequests++; return Promise.resolve('granted'); } },
    };
    const fm = createMotion({ win: fallbackWin, onStatus: (s) => { if (s.type === 'started') started = true; } });
    await fm.start();
    check('orientation permission used when motion API is absent', orientRequests === 1 && started);
  }

  // A second start() while one is in flight must not re-request permission.
  {
    const joinListeners = {};
    let requests = 0;
    let resolvePerm;
    const joinWin = {
      addEventListener: (t, fn) => { (joinListeners[t] ||= []).push(fn); },
      removeEventListener: () => {},
      performance: { now: () => 0 },
      DeviceMotionEvent: { requestPermission: () => { requests++; return new Promise((res) => { resolvePerm = () => res('granted'); }); } },
    };
    const jm = createMotion({ win: joinWin });
    const a = jm.start();
    const b = jm.start();
    check('a concurrent start() joins the in-flight request', requests === 1, `${requests} requests`);
    resolvePerm();
    await Promise.all([a, b]);
    check('joined start() leaves the motion running once', jm.running);
  }

  // Explicit denial reports permission-denied and registers no listeners.
  {
    const deniedListeners = {};
    const deniedWin = {
      addEventListener: (t, fn) => { (deniedListeners[t] ||= []).push(fn); },
      removeEventListener: () => {},
      performance: { now: () => 0 },
      DeviceMotionEvent: { requestPermission: () => Promise.resolve('denied') },
    };
    let denied = false;
    const dm = createMotion({ win: deniedWin, onStatus: (s) => { if (s.type === 'permission-denied') denied = true; } });
    await dm.start();
    check('denied permission reports and registers no listeners',
      denied && !deniedListeners.devicemotion, denied ? 'denied' : 'no status');
  }
}

// ---------------------------------------------------------------------------
// GPS step length auto-calibration (autocal.mjs) - synthetic GPS walks.
// ---------------------------------------------------------------------------
}
if (want('autocal')) {
console.log('\nGPS step length auto-calibration (autocal.mjs):');

// Seeded uniforms/Gaussians so a certification test that passes once keeps
// passing (Math.random would make a borderline threshold flaky).
function mulberry32(seed) {
  let a = seed >>> 0;
  return () => {
    a |= 0; a = (a + 0x6D2B79F5) | 0;
    let t = Math.imul(a ^ (a >>> 15), 1 | a);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}
function gauss(rng) {
  const u = Math.max(rng(), 1e-9), v = rng();
  return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v);
}

/**
 * Feed a synthetic straight walk: steps at `cadenceHz` with ground-truth step
 * length base * cadenceFactor, one GPS fix per second (position noise on both
 * axes, Doppler-like speed noise, course along the walk). Returns the end
 * state so blocks can be chained (corners, pace changes). `turnRateDegS`
 * bends the path continuously (a curve).
 */
function feedCalWalk(cal, opts = {}) {
  const {
    base = 0.75, cadenceHz = 1.85, seconds = 60, startT = 0,
    startLon = -149.83, startLat = 61.19, bearingDeg = 90,
    acc = 4, posNoiseM = 2.5, speedNoise = 0.15, doppler = true,
    courseNoiseDeg = 0, turnRateDegS = 0, accuracyAt = null, rng = mulberry32(1),
  } = opts;
  const stepLen = base * (1 + 0.35 * (cadenceHz - 1.8) / 1.8);
  const groundSpeed = stepLen * cadenceHz;
  const stepMs = 1000 / cadenceHz;
  const endT = startT + seconds * 1000;
  const mpd = metresPerDegree(startLat);
  let bearing = bearingDeg * Math.PI / 180;
  let east = 0, north = 0, curT = startT;
  let nextStepT = startT + stepMs * 0.5;
  let nextFixT = startT + 1000;
  const advance = (toT) => {
    const dt = (toT - curT) / 1000;
    if (dt <= 0) return;
    const dd = groundSpeed * dt;
    east += Math.sin(bearing) * dd;
    north += Math.cos(bearing) * dd;
    bearing += turnRateDegS * Math.PI / 180 * dt;
    curT = toT;
  };
  for (;;) {
    const stepNext = nextStepT <= endT ? nextStepT : Infinity;
    const fixNext = nextFixT <= endT ? nextFixT : Infinity;
    const t = Math.min(stepNext, fixNext);
    if (!isFinite(t)) break;
    advance(t);
    if (t === stepNext) {
      cal.addStep({ t });
      nextStepT += stepMs;
    } else {
      cal.addFix({
        t,
        lon: startLon + (east + gauss(rng) * posNoiseM) / mpd.lon,
        lat: startLat + (north + gauss(rng) * posNoiseM) / mpd.lat,
        accuracyM: typeof accuracyAt === 'function' ? accuracyAt((t - startT) / 1000) : acc,
        speedMps: doppler ? groundSpeed + gauss(rng) * speedNoise : null,
        courseDeg: bearing * 180 / Math.PI + gauss(rng) * courseNoiseDeg,
      });
      nextFixT += 1000;
    }
  }
  advance(endT);
  return {
    t: endT,
    lon: startLon + east / mpd.lon,
    lat: startLat + north / mpd.lat,
    bearingDeg: bearing * 180 / Math.PI,
  };
}

{
  // Two minutes of straight walking with good GPS certify the estimate.
  const cal = new GpsStepCalibrator();
  feedCalWalk(cal, { base: 0.75, cadenceHz: 1.9, seconds: 120, startT: 1000, rng: mulberry32(7) });
  const s = cal.stats();
  check('straight walk certifies', s.calibrated,
    s.confidencePct != null ? `conf ${s.confidencePct.toFixed(1)}%, CI ±${s.ci90RelPct.toFixed(1)}%` : 'no estimate');
  check('certified base is within 1.5% of truth',
    s.baseM != null && Math.abs(s.baseM - 0.75) / 0.75 < 0.015,
    s.baseM != null ? `${s.baseM.toFixed(3)} m vs 0.750` : 'none');
  check('the speed method dominates when available',
    s.samples.speedSharePct > 50, `${s.samples.speedSharePct?.toFixed(0)}% speed`);
  check('a claim always implies ≥90% within ±6%',
    !s.calibrated || (s.confidencePct >= 90 && s.ci90RelPct <= 6 + 1e-9),
    `${s.confidencePct?.toFixed(1)}% / ±${s.ci90RelPct?.toFixed(1)}%`);

  // A short stroll yields at most a provisional estimate - never a claim.
  const shortCal = new GpsStepCalibrator();
  feedCalWalk(shortCal, { seconds: 12, startT: 1000, rng: mulberry32(8) });
  check('a short stroll never certifies', !shortCal.stats().calibrated);

  // ...but a single ~25 s walk is enough under the 20-step / 10 m gates.
  const strideCal = new GpsStepCalibrator();
  feedCalWalk(strideCal, { seconds: 25, startT: 1000, rng: mulberry32(70) });
  const sts = strideCal.stats();
  check('a ~25 s walk certifies (20 steps / 10 m gates)',
    sts.calibrated,
    sts.confidencePct != null
      ? `${sts.samples.steps} steps, ${sts.samples.distanceM.toFixed(0)} m, conf ${sts.confidencePct.toFixed(1)}%`
      : 'none');

  // Real GPS courses are noisy even walking straight; the straightness gate
  // must not reject a genuinely straight walk because of that.
  const jitterCal = new GpsStepCalibrator();
  feedCalWalk(jitterCal, { seconds: 120, courseNoiseDeg: 20, startT: 1000, rng: mulberry32(60) });
  check('a straight walk with noisy GPS course still certifies',
    jitterCal.stats().calibrated,
    jitterCal.stats().confidencePct != null ? `conf ${jitterCal.stats().confidencePct.toFixed(1)}%` : 'none');

  // Poor fixes are unusable evidence, however long the walk.
  const badCal = new GpsStepCalibrator();
  feedCalWalk(badCal, { seconds: 120, acc: 25, startT: 1000, rng: mulberry32(9) });
  check('poor GPS fixes are ignored entirely',
    badCal.stats().baseM == null && badCal.stats().status === 'idle');

  // Coarse-but-usable fixes: up to the 15 m floor they are accepted, the
  // uncertainties scale with accuracy, and the confidence gate - not a hard
  // cutoff - decides how much extra walking they need.
  const pastCal = new GpsStepCalibrator();
  feedCalWalk(pastCal, { seconds: 120, acc: 16, startT: 1000, rng: mulberry32(110) });
  check('fixes past the 15 m floor are ignored entirely', pastCal.stats().baseM == null);

  const coarseCal = new GpsStepCalibrator();
  feedCalWalk(coarseCal, { seconds: 120, acc: 15, startT: 1000, rng: mulberry32(110) });
  const ccs = coarseCal.stats();
  check('15 m fixes certify given more walking', ccs.calibrated,
    ccs.confidencePct != null ? `conf ${ccs.confidencePct.toFixed(1)}%` : 'none');
  check('coarse-fix base within 3% of truth',
    ccs.baseM != null && Math.abs(ccs.baseM - 0.75) / 0.75 < 0.03,
    ccs.baseM != null ? `${ccs.baseM.toFixed(3)} m` : 'none');

  // Same physical walk, only the reported accuracy differs: the coarse run
  // must score strictly lower confidence (that is the dynamism).
  const c10 = new GpsStepCalibrator();
  feedCalWalk(c10, { seconds: 60, acc: 10, startT: 1000, rng: mulberry32(111) });
  const c15 = new GpsStepCalibrator();
  feedCalWalk(c15, { seconds: 60, acc: 15, startT: 1000, rng: mulberry32(111) });
  check('the same walk is less confident at 15 m than at 10 m',
    c10.stats().confidencePct > c15.stats().confidencePct,
    `10 m: ${c10.stats().confidencePct.toFixed(1)}%, 15 m: ${c15.stats().confidencePct.toFixed(1)}%`);

  // A GPS dropout (the arbiter is on PDR indoors) must not let two good fixes
  // be stitched into a position-only sample across the gap: only raw GPS
  // evidence that is continuously present may be used.
  const gapCal = new GpsStepCalibrator();
  feedCalWalk(gapCal, {
    seconds: 20, startT: 1000, rng: mulberry32(120),
    accuracyAt: (s) => (s >= 5 && s <= 16 ? 25 : 4),
  });
  const gs = gapCal.stats();
  check('a GPS dropout cannot be stitched into a sample',
    gs.samples.total === 0, `${gs.samples.total} samples`);
  check('live stats: the dropout is reported as a GPS-gap rejection',
    gs.live.lastAttempt != null && gs.live.lastAttempt.reason === 'coverage',
    gs.live.lastAttempt ? String(gs.live.lastAttempt.reason) : 'none');

  // Control: the same walk with continuous good fixes samples normally.
  const connectedCal = new GpsStepCalibrator();
  feedCalWalk(connectedCal, { seconds: 20, startT: 1000, rng: mulberry32(120) });
  check('...while the same walk with continuous GPS samples fine',
    connectedCal.stats().samples.total > 0, `${connectedCal.stats().samples.total} samples`);

  // A stop in the middle: fixes keep arriving, steps do not. No segment may
  // span the pause, and calibration resumes when walking does.
  const pauseCal = new GpsStepCalibrator();
  const pa = feedCalWalk(pauseCal, { seconds: 10, startT: 1000, rng: mulberry32(10) });
  const beforePause = pauseCal.stats().samples.total;
  for (let t = pa.t + 1000; t <= pa.t + 10000; t += 1000) {
    pauseCal.addFix({ t, lon: pa.lon, lat: pa.lat, accuracyM: 4, speedMps: 0, courseDeg: pa.bearingDeg });
  }
  const pb = feedCalWalk(pauseCal, {
    seconds: 6, startT: pa.t + 10000, startLon: pa.lon, startLat: pa.lat,
    bearingDeg: pa.bearingDeg, rng: mulberry32(11),
  });
  check('a pause is not counted as walking',
    pauseCal.stats().samples.total === beforePause,
    `${beforePause} -> ${pauseCal.stats().samples.total}`);
  feedCalWalk(pauseCal, {
    seconds: 60, startT: pb.t, startLon: pb.lon, startLat: pb.lat,
    bearingDeg: pb.bearingDeg, rng: mulberry32(12),
  });
  check('walking resumes the calibration after the pause',
    pauseCal.stats().samples.total > beforePause);

  // Gentler than a right angle every few metres: a continuous sharp curve
  // must not qualify as a straight calibration stretch.
  const curveCal = new GpsStepCalibrator();
  feedCalWalk(curveCal, { seconds: 120, turnRateDegS: 12, startT: 1000, rng: mulberry32(13) });
  check('a sharp continuous curve is rejected',
    curveCal.stats().samples.total === 0, `${curveCal.stats().samples.total} samples`);
  check('live stats: a curve reports "turn" as the reject reason',
    curveCal.stats().live.lastAttempt != null
      && curveCal.stats().live.lastAttempt.ok === false
      && curveCal.stats().live.lastAttempt.reason === 'turn',
    curveCal.stats().live.lastAttempt ? String(curveCal.stats().live.lastAttempt.reason) : 'none');

  // Right-angle corners are fine: the straight blocks between them calibrate.
  const cornerCal = new GpsStepCalibrator();
  let cst = { t: 1000, lon: -149.83, lat: 61.19, bearingDeg: 90 };
  for (let k = 0; k < 4; k++) {
    const end = feedCalWalk(cornerCal, {
      base: 0.75, cadenceHz: 1.9, seconds: 30, startT: cst.t,
      startLon: cst.lon, startLat: cst.lat, bearingDeg: cst.bearingDeg,
      rng: mulberry32(20 + k),
    });
    cst = { t: end.t, lon: end.lon, lat: end.lat, bearingDeg: cst.bearingDeg + 90 };
  }
  const cs2 = cornerCal.stats();
  check('straight blocks between right-angle corners certify',
    cs2.calibrated, cs2.confidencePct != null ? `conf ${cs2.confidencePct.toFixed(1)}%` : 'none');
  check('corner-walk base within 3% of truth',
    cs2.baseM != null && Math.abs(cs2.baseM - 0.75) / 0.75 < 0.03,
    cs2.baseM != null ? `${cs2.baseM.toFixed(3)} m` : 'none');

  // Constrained indoor space: short legs (7 s) with a pause at every
  // turn-around. The pause closes each leg, so the U-turn courses never mix
  // into a segment - and the boundary speed handling must not bias the base
  // low (a stationary fix averaged into the leg's first pair would).
  const turnCal = new GpsStepCalibrator();
  let tst = { t: 1000, lon: -149.83, lat: 61.19, bearingDeg: 90 };
  for (let k = 0; k < 6; k++) {
    tst = feedCalWalk(turnCal, {
      base: 0.75, cadenceHz: 1.9, seconds: 7, startT: tst.t,
      startLon: tst.lon, startLat: tst.lat, bearingDeg: tst.bearingDeg,
      rng: mulberry32(90 + k),
    });
    for (let t = tst.t + 1000; t <= tst.t + 3000; t += 1000) {
      turnCal.addFix({ t, lon: tst.lon, lat: tst.lat, accuracyM: 4, speedMps: 0, courseDeg: null });
    }
    tst = { t: tst.t + 3000, lon: tst.lon, lat: tst.lat, bearingDeg: tst.bearingDeg + 180 };
  }
  const tns = turnCal.stats();
  check('pause-and-turn walking certifies in a constrained space',
    tns.calibrated,
    tns.confidencePct != null
      ? `${tns.samples.total} legs, ${tns.samples.steps} steps, ${tns.samples.distanceM.toFixed(0)} m, conf ${tns.confidencePct.toFixed(1)}%`
      : 'none');
  check('pause-and-turn base within 3% (no pause bias)',
    tns.baseM != null && Math.abs(tns.baseM - 0.75) / 0.75 < 0.03,
    tns.baseM != null ? `${tns.baseM.toFixed(3)} m` : 'none');

  // Live progress: the panel can show the walk as it happens - latest fix,
  // steps in the current stretch, and the newest attempt's outcome.
  const liveCal = new GpsStepCalibrator();
  feedCalWalk(liveCal, { seconds: 30, startT: 1000, rng: mulberry32(100) });
  const ls = liveCal.stats();
  check('live stats: latest fix visible',
    ls.live.fixAccuracyM === 4 && ls.live.fixSpeedMps != null,
    `±${ls.live.fixAccuracyM} m, v=${ls.live.fixSpeedMps?.toFixed(2)}`);
  check('live stats: an accepted leg is reported',
    ls.live.lastAttempt != null && ls.live.lastAttempt.ok === true
      && ls.live.lastAttempt.baseM > 0.6,
    ls.live.lastAttempt ? `ok=${ls.live.lastAttempt.ok}, ${ls.live.lastAttempt.baseM?.toFixed(3)} m` : 'none');
  check('live stats: the current stretch restarts after an accepted leg',
    ls.samples.total >= 2 && ls.live.stretch.steps < 30,
    `${ls.samples.total} legs, ${ls.live.stretch.steps} steps since`);

  // ...and a stretch that is too short to sample yet says so.
  const shortLive = new GpsStepCalibrator();
  feedCalWalk(shortLive, { seconds: 7, cadenceHz: 1.5, startT: 1000, rng: mulberry32(101) });
  const sl = shortLive.stats().live;
  check('live stats: stretch grows before any sample',
    sl.stretch.steps >= 3 && sl.stretch.seconds > 1,
    `${sl.stretch.steps} steps, ${sl.stretch.seconds.toFixed(1)} s`);
  check('live stats: a too-short stretch is reported as such',
    sl.lastAttempt != null && !sl.lastAttempt.ok
      && ['steps', 'span', 'distance'].includes(sl.lastAttempt.reason),
    sl.lastAttempt ? String(sl.lastAttempt.reason) : 'none');

  // Devices that report no speed: displacement/step geometry still certifies,
  // just more slowly (position noise is the limiting factor).
  const posCal = new GpsStepCalibrator();
  feedCalWalk(posCal, {
    seconds: 360, doppler: false, acc: 2.5, posNoiseM: 1.2,
    startT: 1000, rng: mulberry32(30),
  });
  const ps = posCal.stats();
  check('position-only walking still certifies', ps.calibrated,
    ps.confidencePct != null ? `conf ${ps.confidencePct.toFixed(1)}%` : 'none');
  check('position-only base within 4% of truth',
    ps.baseM != null && Math.abs(ps.baseM - 0.75) / 0.75 < 0.04,
    ps.baseM != null ? `${ps.baseM.toFixed(3)} m` : 'none');

  // Cadence normalisation: the same person walking at two paces must produce
  // the same base, not a blend of their step lengths.
  const cadCal = new GpsStepCalibrator();
  const ca = feedCalWalk(cadCal, { base: 0.75, cadenceHz: 1.6, seconds: 60, startT: 1000, rng: mulberry32(50) });
  feedCalWalk(cadCal, {
    base: 0.75, cadenceHz: 2.2, seconds: 60, startT: ca.t, startLon: ca.lon,
    startLat: ca.lat, bearingDeg: ca.bearingDeg, rng: mulberry32(51),
  });
  const cds = cadCal.stats();
  check('cadence changes are normalised away',
    cds.baseM != null && Math.abs(cds.baseM - 0.75) / 0.75 < 0.03,
    cds.baseM != null ? `${cds.baseM.toFixed(3)} m` : 'none');

  // If the first and second halves disagree, the pooled mean is a fiction:
  // the claim must be withheld even though the pooled confidence looks good.
  const splitCal = new GpsStepCalibrator();
  const sa = feedCalWalk(splitCal, { base: 0.65, seconds: 60, startT: 1000, rng: mulberry32(40) });
  feedCalWalk(splitCal, {
    base: 0.9, seconds: 60, startT: sa.t, startLon: sa.lon,
    startLat: sa.lat, bearingDeg: sa.bearingDeg, rng: mulberry32(41),
  });
  const ss = splitCal.stats();
  check('inconsistent halves block the claim',
    !ss.calibrated && ss.missing.includes('consistency'), ss.missing.join(','));
  check('...while the provisional estimate stays between the halves',
    ss.baseM > 0.6 && ss.baseM < 0.95, ss.baseM != null ? `${ss.baseM.toFixed(3)} m` : 'none');

  // The confidence function is the gate itself: 90% exactly at sigma =
  // tolerance / z90, and quickly small for a sloppy estimate.
  check('confidence is 90% at the z90 boundary',
    Math.abs(calibrationConfidence(0.06, 0.06 / 1.6448536269514722) - 0.9) < 0.002);
  check('confidence is low for a sloppy estimate',
    calibrationConfidence(0.06, 0.06) < 0.7);

  // Fresh state: idle until GPS, collecting once fixes arrive.
  const fresh = new GpsStepCalibrator();
  check('idle before any GPS',
    fresh.stats().status === 'idle' && fresh.stats().baseM == null);
  fresh.addFix({ t: 1000, lon: -149.83, lat: 61.19, accuracyM: 5, speedMps: null, courseDeg: null });
  check('a fix alone is collecting, not calibrating', fresh.stats().status === 'collecting');
}

}

// ---------------------------------------------------------------------------
// Anchor step-length calibration (anchorcal.mjs) - synthetic straight walks.
// ---------------------------------------------------------------------------
if (want('anchorcal')) {
console.log('\nAnchor step-length calibration (anchorcal.mjs):');
  const lon0 = -149.83, lat0 = 61.19;
  const mpd = metresPerDegree(lat0);
  const toLL = (e, n) => [lon0 + e / mpd.lon, lat0 + n / mpd.lat];
  const BASE = 0.72, CAD = 1.8, N = 30;
  const dist = N * BASE;                          // 30 steps at the reference cadence
  const anchor = (t, e, n, floor = '01') =>
    ({ t, ll: toLL(e, n), buildingId: 'B', floorId: floor });
  // One straight leg: mark A, walk N steps at `heading`, mark B `dist` along it.
  const feed = (cal, { t0 = 0, heading = 0, n = N, gapMs = 0, gapAt = n,
    floor = '01', clear = () => true } = {}) => {
    cal.addAnchor(anchor(t0, 0, 0, floor), clear);
    for (let i = 1; i <= n; i++) {
      cal.addStep({ t: t0 + i * 1000 + (i > gapAt ? gapMs : 0), headingRad: heading, cadenceHz: CAD });
    }
    const B = anchor(t0 + n * 1000 + gapMs + 1,
      Math.sin(heading) * dist, Math.cos(heading) * dist, floor);
    return cal.addAnchor(B, clear);
  };

  // Straight legs from exact marks recover the base and certify.
  {
    const cal = new AnchorStepCalibrator();
    cal.setEnabled(true);
    feed(cal, { t0: 0 });
    feed(cal, { t0: 60000 });
    const s = cal.stats();
    check('straight anchor legs recover the base step length',
      s.baseM != null && Math.abs(s.baseM - BASE) < 0.01,
      s.baseM != null ? `${s.baseM.toFixed(3)} m` : 'none');
    check('two straight legs certify (within +-6% @ 90%)',
      s.calibrated, s.missing.join(',') || `conf ${s.confidencePct.toFixed(0)}%`);
  }

  // A curve (headings spanning 180 deg) is not a straight walk.
  {
    const cal = new AnchorStepCalibrator();
    cal.setEnabled(true);
    cal.addAnchor(anchor(0, 0, 0), () => true);
    for (let i = 1; i <= N; i++) {
      cal.addStep({ t: i * 1000, headingRad: (i / N) * Math.PI, cadenceHz: CAD });
    }
    const r = cal.addAnchor(anchor(31000, 0, dist), () => true);
    check('a curved walk is rejected', r === null && cal.lastAttempt.reason === 'curve',
      cal.lastAttempt.reason);
  }

  // The mean heading must point from A to B.
  {
    const cal = new AnchorStepCalibrator();
    cal.setEnabled(true);
    cal.addAnchor(anchor(0, 0, 0), () => true);
    for (let i = 1; i <= N; i++) cal.addStep({ t: i * 1000, headingRad: Math.PI / 2, cadenceHz: CAD });
    const r = cal.addAnchor(anchor(31000, 0, dist), () => true);   // walked east, B is north
    check('a walk not pointing at the second mark is rejected',
      r === null && cal.lastAttempt.reason === 'heading', cal.lastAttempt.reason);
  }

  // A straight span across a wall is not the walked distance.
  {
    const cal = new AnchorStepCalibrator();
    cal.setEnabled(true);
    const r = feed(cal, { clear: () => false });
    check('a mark pair across a wall is rejected',
      r === null && cal.lastAttempt.reason === 'wall', cal.lastAttempt.reason);
  }

  // Changing floor between the marks is never a straight leg.
  {
    const cal = new AnchorStepCalibrator();
    cal.setEnabled(true);
    cal.addAnchor(anchor(0, 0, 0, '01'), () => true);
    for (let i = 1; i <= N; i++) cal.addStep({ t: i * 1000, headingRad: 0, cadenceHz: CAD });
    const r = cal.addAnchor(anchor(31000, 0, dist, '02'), () => true);
    check('a floor change is rejected', r === null && cal.lastAttempt.reason === 'floor',
      cal.lastAttempt.reason);
  }

  // A pause inside the leg breaks it into two walks.
  {
    const cal = new AnchorStepCalibrator();
    cal.setEnabled(true);
    const r = feed(cal, { gapMs: 2000, gapAt: 15 });
    check('a pause mid-walk is rejected', r === null && cal.lastAttempt.reason === 'pause',
      cal.lastAttempt.reason);
  }

  // Off by default: nothing is collected, and toggling off clears the leg.
  {
    const cal = new AnchorStepCalibrator();
    check('the anchor calibrator is off by default',
      !cal.enabled && cal.stats().status === 'idle' && cal.stats().samples.total === 0);
    cal.setEnabled(true);
    cal.addAnchor(anchor(0, 0, 0), () => true);
    cal.addStep({ t: 1000, headingRad: 0, cadenceHz: CAD });
    check('steps since the last pin are tracked',
      cal.stats().sincePinSteps === 1, `${cal.stats().sincePinSteps}`);
    cal.setEnabled(false);
    check('turning it off discards the in-progress leg',
      cal.lastAnchor === null && cal.steps.length === 0 && cal.stats().samples.total === 0);
  }

  // Opportunistic: a non-straight stretch is skipped, and the next straight
  // stretch (starting from that pin) still calibrates.
  {
    const cal = new AnchorStepCalibrator();
    cal.setEnabled(true);
    // leg 1: curved -> rejected
    cal.addAnchor(anchor(0, 0, 0), () => true);
    for (let i = 1; i <= N; i++) {
      cal.addStep({ t: i * 1000, headingRad: (i / N) * Math.PI, cadenceHz: CAD });
    }
    const r1 = cal.addAnchor(anchor(31000, 0, dist), () => true);
    // leg 2: straight from the new pin -> accepted
    const r2 = feed(cal, { t0: 40000 });
    check('a non-straight stretch is skipped, the next straight one calibrates',
      r1 === null && r2 != null && Math.abs(r2.baseM - BASE) < 0.01,
      r2 != null ? `${r2.baseM.toFixed(3)} m` : 'none');
  }
}

if (want('headingfield')) {
  // The allowed-compass-directions map and the snapper. Synthetic geometry so
  // it needs no floor load: metres ARE the coordinates (toM = identity).
  const DEG = Math.PI / 180;
  const ident = (ll) => ll;
  /** Fully-connect points closer than `linkM` (mimics the visibility graph). */
  const link = (points, linkM = 6) => {
    const nodes = points.map((p) => [p[0], p[1]]);
    const adj = nodes.map(() => []);
    for (let i = 0; i < nodes.length; i++) {
      for (let j = i + 1; j < nodes.length; j++) {
        const d = Math.hypot(nodes[j][0] - nodes[i][0], nodes[j][1] - nodes[i][1]);
        if (d <= linkM) { adj[i].push([j, d]); adj[j].push([i, d]); }
      }
    }
    return { nodes, adj };
  };
  const fieldOf = (g) => makeHeadingField(buildPassages(g.nodes, g.adj), { toM: ident });

  // --- a straight, east-west hallway -> one dominant (undirected) axis ------
  {
    const corridor = fieldOf(link([[0, 0], [3, 0], [6, 0], [9, 0]]));
    const q = corridor.query([4.5, 0]);
    check('a corridor has one dominant east-west axis',
      q && distPi(q.axis, Math.PI / 2) < 5 * DEG && q.confidence > 0.6,
      q ? `${(q.axis / DEG).toFixed(0)} deg, conf ${q.confidence.toFixed(2)}` : 'null');
  }

  // --- a T junction -> two axes (the through corridor + the branch) ---------
  {
    const t = fieldOf(link([[0, 0], [3, 0], [6, 0], [9, 0], [6, 3], [6, 6]]));
    const q = t.query([6, 0]);
    const hasNorth = q && q.candidates.some((c) => distPi(c, 0) < 8 * DEG);
    const hasEast = q && q.candidates.some((c) => distPi(c, Math.PI / 2) < 8 * DEG);
    check('a T junction exposes both the through axis and the branch',
      !!(hasNorth && hasEast),
      q ? q.candidates.map((c) => (c / DEG).toFixed(0)).join(',') : 'null');
  }

  // --- an open room -> bearings fan out, no dominant direction --------------
  {
    const room = fieldOf(link([[0, 0], [4, 0], [0, 4], [4, 4], [2, 2]]));
    const q = room.query([2, 2]);
    check('an open room has no dominant direction to snap to',
      !q || q.confidence < 0.4, q ? `conf ${q.confidence.toFixed(2)}` : 'null');
  }

  // --- a doorway points across the wall it sits in --------------------------
  {
    const passages = buildPassages([], [], [[5, 0]], [[[0, 0], [10, 0]]]);
    const q = makeHeadingField(passages, { toM: ident }).query([5, 0.6]);
    check('a doorway points across its wall (north, not along it)',
      q && distPi(q.axis, 0) < 8 * DEG,
      q ? `${(q.axis / DEG).toFixed(0)} deg` : 'null');
  }

  // --- the snapper: locks, pulls, releases on a turn, re-locks --------------
  {
    const corridorQ = fieldOf(link([[0, 0], [3, 0], [6, 0], [9, 0]])).query([4.5, 0]);
    const s = new CompassSnapper();
    const r1 = s.update(Math.PI / 2 + 6 * DEG, corridorQ);
    check('the snapper locks to the nearest corridor direction',
      r1.locked && r1.dir != null && distPi(r1.dir, Math.PI / 2) < 3 * DEG,
      `${r1.reason}`);
    check('the snapper pulls toward the locked axis (no hard quantise)',
      r1.correctionRad < 0 && r1.correctionRad >= -3 * DEG,
      `${(r1.correctionRad / DEG).toFixed(1)} deg`);

    const tq = fieldOf(link([[0, 0], [3, 0], [6, 0], [9, 0], [6, 3], [6, 6]])).query([6, 0]);
    const s2 = new CompassSnapper();
    s2.update(Math.PI / 2, tq);                        // lock east
    let last;
    for (const h of [Math.PI / 2, Math.PI / 2 - 20 * DEG, Math.PI / 2 - 45 * DEG, 0]) {
      last = s2.update(h, tq);                          // an obvious 90 deg turn
    }
    check('an obvious turn releases the lock', s2.locked === false, `${last.reason}`);
    const after = s2.update(0, tq);
    check('after the turn it re-locks to the new direction',
      after.locked && distPi(after.dir, 0) < 3 * DEG, `${after.reason}`);
  }

  // --- no field (outdoors / wide lobby) -> never corrects -------------------
  {
    const s = new CompassSnapper();
    const r = s.update(1.0, null);
    check('with no field the snapper does nothing', !r.locked && r.correctionRad === 0);
  }
}
// ---------------------------------------------------------------------------
// Doorways: DOOR-layer clustering and wall-aligned barriers.
// ---------------------------------------------------------------------------
if (want('doorways')) {
console.log('\nDoorways (doorways.mjs):');

// A closed leaf inside a wall gap is its own barrier; an open leaf with a
// swing arc must use the closed direction, not the drawn leaf.
const wallShapes = [
  { layer: 'WALL', items: [{ op: 'line', points: [[0, 0], [40, 0]] }] },
  { layer: 'WALL', items: [{ op: 'line', points: [[60, 0], [100, 0]] }] },
  { layer: 'WALL', items: [{ op: 'line', points: [[0, 0], [0, 40]] }] },
  { layer: 'WALL', items: [{ op: 'line', points: [[0, 40], [100, 40]] }] },
];
const closedLeaf = { layer: 'DOORS', items: [{ op: 'polyline', closed: true,
  points: [[40, -1], [60, -1], [60, 1], [40, 1]] }] };
const openLeaf = { layer: 'DOORS', items: [
  { op: 'polyline', closed: true, points: [[60, 0], [60, 20], [62, 20], [62, 0]] },
  { op: 'arc', center: [60, 0], radius: 20, start_angle: 0, end_angle: 90 },
] };
const ds = doorwaysFromShapes([...wallShapes, closedLeaf, openLeaf], (x, y) => [x, y]);
check('closed and open leaves of one opening produce one doorway', ds.length === 1, `${ds.length} doorways`);
const synthetic = ds[0];
check('doorway width is the 20-unit opening', !!synthetic && Math.abs(synthetic.width - 20 * 0.0254) < 0.02,
  synthetic ? `${synthetic.width.toFixed(3)} m` : 'none');
check('barrier follows the wall, not the open leaf', !!synthetic &&
  Math.abs(synthetic.dxfBarrier[0][0] - synthetic.dxfBarrier[1][0]) > 15 &&
  Math.abs(synthetic.dxfBarrier[0][1]) < 2,
  synthetic ? JSON.stringify(synthetic.dxfBarrier) : 'none');

// A route through a barrier crosses it; a nearby parallel run does not.
const door = { barrier: [[0, 0], [2, 0]] };
check('barrier crossing is detected', barrierCrossesPolyline(door, [[1, -1], [1, 1]]));
check('a near miss does not cross the barrier', !barrierCrossesPolyline(door, [[1, 0.5], [3, 0.5]]));

// Real floor: the target doorway must be a wall opening, not an open leaf.
const cfg = CONFIG.find((b) => b.id === 'AS121');
const fl = cfg.floors.find((f) => f.floorId === '02');
const data = JSON.parse(readFileSync(new URL('.' + fl.url, import.meta.url), 'utf8'));
const toLL = makeToLL(cfg, fl);
const real = doorwaysFromShapes(data.shapes, toLL);
const room = (data.text_lines ?? []).find((t) => String(t.text ?? '').trim() === '208');
const labelLL = toLL(room.x, room.y);
const labelM = metresPerDegree(labelLL[1]);
const near = real
  .map((d) => ({ d, m: Math.hypot((d.ll[0] - labelLL[0]) * labelM.lon, (d.ll[1] - labelLL[1]) * labelM.lat) }))
  .filter((x) => x.m < 2)
  .sort((a, b) => a.m - b.m)[0];
check('ECB 208 has a nearby doorway', !!near, `${real.length} doorways`);
check('ECB 208 doorway barrier is a sane opening', !!near && near.d.width > 0.5 && near.d.width < 1.5,
  near ? `${near.d.width.toFixed(2)} m` : 'none');
check('ECB 208 barrier is the wall opening, not the 0.9 m leaf', !!near &&
  Math.abs(near.d.dxfBarrier[0][0] - near.d.dxfBarrier[1][0]) > 15 ||
  !!near && Math.abs(near.d.dxfBarrier[0][1] - near.d.dxfBarrier[1][1]) > 15,
  near ? JSON.stringify(near.d.dxfBarrier) : 'none');
}

// ---------------------------------------------------------------------------
// Room Pin: an exact room's one hallway door on the real ECB plan.
// ---------------------------------------------------------------------------
if (want('roompin')) {
console.log('\nRoom Pin (roompin.mjs, real AS121-02):');
const cfg = CONFIG.find((b) => b.id === 'AS121');
const fl = cfg.floors.find((f) => f.floorId === '02');
const data = JSON.parse(readFileSync(new URL('.' + fl.url, import.meta.url), 'utf8'));
const toLL = makeToLL(cfg, fl);
const { lines, doors: doorPoints } = navGraphInputs(cfg, fl, data, toLL);
const graph = graphFromState(buildGraphState(lines, doorPoints, { passages: false }));
const portals = portalsFromCache(fl.portals, toLL);
const exits = (fl.exits?.length ? fl.exits : portals.exits)
  .map((e, i) => ({ id: e.id ?? `exits-${i}`, ll: toLL(e.dxf[0], e.dxf[1]) }));
const doorways = doorwaysFromShapes(data.shapes, toLL);
const roomLabels = (data.text_lines ?? []).map((t) => ({
  type: 'Feature',
  geometry: { type: 'Point', coordinates: toLL(t.x, t.y) },
  properties: { text: String(t.text ?? '').trim() },
})).filter((f) => f.properties.text);
const floor = { floorId: fl.floorId, navGraph: graph, obstacleLines: lines,
  elevators: portals.elevators, stairs: portals.stairs, exits, skybridges: [],
  roomLabels, doorways, doorPoints };
const building = { id: cfg.id, sourceId: data.ID, groundFloor: cfg.groundFloor, floors: [floor] };
const router = buildCampusRouter([building]);
const hit = findRoomExact(building, '208');
check('exact room lookup finds ECB 208', !hit.error && hit.floor.floorId === '02', hit.error ?? '');
const pin = hit.error ? hit : resolveRoomPin(building, hit, router);
check('ECB 208 resolves to one hallway door', !pin.error, pin.error ?? '');
check('pin point is outside the doorway', !pin.error &&
  Math.hypot((pin.ll[0] - pin.doorLL[0]) * metresPerDegree(pin.ll[1]).lon,
             (pin.ll[1] - pin.doorLL[1]) * metresPerDegree(pin.ll[1]).lat) > 0.8,
  pin.error ?? '');
check('unknown exact room is rejected', !!findRoomExact(building, 'ZZZ').error);

// A route crossing an interior door lists it and can avoid it.
const room209 = findRoomExact(building, '209');
const cross = room209.error ? null : router.route(hit.labelLL, room209.labelLL,
  { startFloor: '02', endFloor: '02' });
const crossDoor = (cross?.portals ?? []).find((p) => p.interior);
check('a route through an interior door lists a blockable portal', !!crossDoor,
  crossDoor ? crossDoor.key : 'none');
if (crossDoor) {
  const alt = router.route(hit.labelLL, room209.labelLL,
    { startFloor: '02', endFloor: '02', avoidPortals: [crossDoor.key] });
  check('blocked interior door is not used again',
    !alt || !(alt.portals ?? []).some((p) => p.key === crossDoor.key));
  const blockedDoor = doorways.find((d) => crossDoor.id === `interior-${d.id}`);
  const crossesBlocked = !!alt && !!blockedDoor && (alt.segments ?? []).some((s) =>
    s.tier === 'indoor' && s.polyline &&
    barrierCrossesPolyline(blockedDoor, s.polyline, (p) => graph.toM(p)));
  check('alternate route avoids the blocked barrier', !crossesBlocked,
    blockedDoor ? blockedDoor.id : 'none');
}
}

// ---------------------------------------------------------------------------
// Places of interest (places.mjs + places.json). Fast: reads floor JSON
// labels only, no navGraph build.
// ---------------------------------------------------------------------------
if (want('places')) {
console.log('\nPlaces of interest (places.mjs, places.json):');

// Name matching: punctuation, case and stop words never decide a match.
check('normName drops case, punctuation and stop words',
  normName('AI & Robotics Lab') === 'ai robotics lab' && normName('  the  POOL ') === 'pool');
const demo = [
  {id: 'eib-ai', building: 'EIB', room: '303', names: ['AI & Robotics Lab', 'Robotics Club', 'AI Club']},
  {id: 'su-subway', building: 'SU', room: 'Subway', names: ['Subway']},
  {id: 'statue', ll: [-149.82, 61.19], floor: '01', names: ['Seawolf Statue']},
];
const demoIndex = buildPlaceIndex(demo, [{name: 'robo lab', building: 'EIB', room: '303'}]);
const top = (q) => searchPlaces(demoIndex, q, 3);
check('exact nickname ranks first', top('robotics club')[0]?.name === 'Robotics Club' && top('robotics club')[0].score === 0);
check('ampersand and case ignored', top('ai robotics lab')[0]?.name === 'AI & Robotics Lab');
check('name prefix beats word match', top('robotics')[0]?.name === 'Robotics Club');
check('every query word must start a name word',
  top('ai club')[0]?.name === 'AI Club' && top('club ai')[0]?.name === 'AI Club' && top('ai pizza').length === 0);
check('substring match as a last resort', top('awolf')[0]?.name === 'Seawolf Statue');
check('favorites are searchable and win ties', top('robo')[0]?.favorite === true, top('robo').map((e) => e.name).join(', '));
check('one result per place and name',
  searchPlaces(buildPlaceIndex([...demo, demo[0]]), 'ai', 10).filter((e) => e.name === 'AI Club').length === 1);
check('empty query matches nothing', top('  &  ').length === 0);

// Targets are the vote key: stable and reversible.
check('room target', placeTarget({building: 'su', room: 'Subway'}) === 'SU Subway');
const pt = placeTarget({ll: [-149.8231234, 61.19], floor: '02'});
check('point target round-trips',
  pt === 'll:-149.823123,61.190000@02' && parseTarget(pt).floor === '02' && parseTarget(pt).ll[0] === -149.823123);
check('room target round-trips', parseTarget('EIB 303A').building === 'EIB' && parseTarget('EIB 303A').room === '303A');

// Schema checks the admin page and this gate share.
check('valid places pass', validatePlaces(demo).length === 0, validatePlaces(demo).join('; '));
const bad = validatePlaces([
  {id: 'a', names: ['Lab'], building: 'EIB', room: '1'},
  {id: 'a', names: ['x'], building: 'EIB', room: '2'},
  {id: 'b', names: [], building: 'EIB', room: '3'},
  {id: 'c', names: ['THE lab!'], building: 'EIB', room: '4'},
  {id: 'd', names: ['Somewhere']},
]);
check('duplicate id, no names, shared nickname and no location are all caught',
  bad.some((p) => p.includes('duplicate id')) && bad.some((p) => p.includes('at least one name')) &&
  bad.some((p) => p.includes('names both a and c')) && bad.some((p) => p.includes('building + room')),
  bad.join('; '));

// Room lookup on a synthetic two-floor building (the app's search path).
const label = (text, lon, lat) => ({type: 'Feature', geometry: {type: 'Point', coordinates: [lon, lat]}, properties: {text}});
const fake = [{id: 'AS103', sourceId: 'BMH', floors: [
  {floorId: '01', roomLabels: [label('114A', 1, 1), label('213', 9, 9), label('Student Lounge', 2, 2)]},
  {floorId: '02', roomLabels: [label('213', 3, 3), label('114A1', 4, 4)]},
]}];
check('longest building code wins the split',
  splitRoomQuery(fake, 'AS103 114A')?.roomText === '114A' && splitRoomQuery(fake, 'bmh114a')?.roomText === '114a' &&
  splitRoomQuery(fake, 'Subway') === null);
check('room on its numbered floor wins', findRoom(fake[0], '213')?.floorId === '02');
check('exact beats prefix', findRoom(fake[0], '114A')?.labelLL[0] === 1 && findRoom(fake[0], '114A').text === '114A');
check('named rooms match too', findRoom(fake[0], 'student lounge')?.floorId === '01');
check('resolvePlace finds rooms by UAA code', resolvePlace(fake, {building: 'bmh', room: '213'})?.floorId === '02');
check('resolvePlace: unregistered building is null', resolvePlace(fake, {building: 'EIB', room: '303'}) === null);

// The shipped places.json: valid, and every entry resolves on the campus
// registered in buildings-config.json (same label/registration path as the
// app's loadFloor). A room the extractor does not draw fails here.
let shipped = null;
try { shipped = JSON.parse(readFileSync(new URL('./places.json', import.meta.url), 'utf8')); }
catch (e) { check('places.json parses', false, e.message); }
if (shipped) {
  const problems = validatePlaces(shipped);
  check(`places.json is valid (${shipped.length} places)`, problems.length === 0, problems.join('; '));
  const labelCampus = CONFIG.map((cfg) => {
    const b = {id: cfg.id, sourceId: null, floors: []};
    for (const floor of cfg.floors) {
      const data = JSON.parse(readFileSync(new URL(floor.url.replace(/^\//, ''), import.meta.url), 'utf8'));
      const toLL = makeToLL(cfg, floor);
      b.sourceId ??= data.ID ?? null;
      b.floors.push({floorId: floor.floorId, roomLabels: (data.text_lines ?? []).map((t) =>
        label(String(t.text ?? '').trim(), ...toLL(t.x, t.y))).filter((f) => f.properties.text)});
    }
    return b;
  });
  for (const p of shipped) {
    const hit = resolvePlace(labelCampus, p);
    const floors = hit?.building?.floors.map((f) => f.floorId) ?? [];
    check(`place "${p.id}" resolves`, hit && (!p.ll || floors.includes(p.floor)),
      hit ? `${hit.building?.sourceId ?? ''} floor ${hit.floorId}` : `${placeTarget(p)} not found`);
  }
}
}

console.log(`\n${failures === 0 ? 'ALL TESTS PASSED' : failures + ' FAILURES'}`);process.exit(failures === 0 ? 0 : 1);