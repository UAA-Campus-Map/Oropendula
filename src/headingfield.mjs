/**
 * `headingfield.mjs` - the floor's "allowed compass directions".
 *
 * Dead reckoning indoors goes wrong when the compass drifts, and the floor plan
 * knows better: in a hallway you are either facing one way along the axis or the
 * other. At a junction the passages point along two axes. A doorway points into
 * the room. So the plan defines a *set* of plausible headings at every point.
 *
 * This module builds that set once per floor (inside the navgraph worker, from
 * the same graph the router uses) and exposes:
 *
 *   buildPassages({nodes, adj, doorsM, segments}) -> Float32Array
 *       Every graph edge and every doorway as a short segment with a bearing and
 *       a weight, packed [x1,y1,x2,y2,w, ...]. Serialisable/transferable.
 *
 *   makeHeadingField(passages, {toM}) -> {query(ll)}
 *       A spatial index + bearing histogram. `query(ll)` returns the distinct
 *       candidate bearings near a point, each with a weight, plus a `confidence`
 *       (how concentrated the bearings are - a corridor is concentrated, an open
 *       room is not).
 *
 *   class CompassSnapper
 *       Stateful: snaps the live heading to the nearest candidate, HOLDS it, and
 *       releases only on a deliberate turn. Pure and testable.
 *
 * Bearings are radians, clockwise from north (north = +y metres, east = +x).
 * Directions are UNDIRECTED: every bearing is folded into [0, PI).
 */

const PI = Math.PI;
const DEG = PI / 180;

/** Fold a bearing into [0, PI) - direction has no sign. */
export function foldPi(a) {
  a %= PI;
  if (a < 0) a += PI;
  return a;
}

/** Circular distance between two folded bearings, in [0, PI/2]. */
export function distPi(a, b) {
  const d = Math.abs(foldPi(a) - foldPi(b));
  return d > PI / 2 ? PI - d : d;
}

function bearingOf(x1, y1, x2, y2) {
  return Math.atan2(x2 - x1, y2 - y1);   // atan2(east, north)
}

function distToSegment(px, py, x1, y1, x2, y2) {
  const vx = x2 - x1, vy = y2 - y1;
  const len2 = vx * vx + vy * vy;
  let t = len2 ? ((px - x1) * vx + (py - y1) * vy) / len2 : 0;
  t = t < 0 ? 0 : t > 1 ? 1 : t;
  return Math.hypot(px - (x1 + t * vx), py - (y1 + t * vy));
}

/**
 * Build the passage segments for a floor.
 *
 * @param {number[][]} nodes     graph node centres, local metres
 * @param {Array<Array<[number,number]>>} adj  adjacency (node -> [neighbour, dist])
 * @param {number[][]} doorsM    doorway points, local metres
 * @param {number[][][]} segments wall segments, local metres (for door normals)
 * @param {object} [opts]
 * @param {number} [opts.doorWeight=3] a doorway is a stronger cue than an edge
 * @param {number} [opts.doorLenM=1.5] length of the door-normal stub
 * @returns {Float32Array} packed [x1,y1,x2,y2,w, ...]
 */
export function buildPassages(nodes, adj, doorsM = [], segments = [], {
  doorWeight = 3, doorLenM = 1.5,
} = {}) {
  const out = [];
  // Graph edges: the visibility graph already runs along hallways and out to
  // each branch at a junction, so a bearing histogram of nearby edges is the
  // corridor/junction direction set.
  for (let i = 0; i < nodes.length; i++) {
    const [x1, y1] = nodes[i];
    for (const [j] of adj[i]) {
      if (j < i) continue;                    // each undirected edge once
      const [x2, y2] = nodes[j];
      out.push(x1, y1, x2, y2, 1);
    }
  }
  // Doorways: a short stub along the door normal (perpendicular to the wall it
  // sits in), weighted up because "through the door" is a real option.
  if (doorsM.length && segments.length) {
    for (const [dx, dy] of doorsM) {
      let best = null, bestD = Infinity;
      for (const [[ax, ay], [bx, by]] of segments) {
        const d = distToSegment(dx, dy, ax, ay, bx, by);
        if (d < bestD) { bestD = d; best = [ax, ay, bx, by]; }
      }
      if (!best || bestD > 2.0) continue;     // no wall within 2 m -> not a door
      const wallDir = bearingOf(best[0], best[1], best[2], best[3]);
      const normal = wallDir + PI / 2;        // passage is across the wall
      const ex = dx + Math.sin(normal) * doorLenM;
      const ey = dy + Math.cos(normal) * doorLenM;
      out.push(dx, dy, ex, ey, doorWeight);
    }
  }
  return Float32Array.from(out);
}

/**
 * Turn a packed passage array into a queryable heading field.
 *
 * @param {Float32Array|number[]} passages  packed [x1,y1,x2,y2,w, ...]
 * @param {object} opts
 * @param {Function} opts.toM  ([lon,lat]) -> [x,y] metres
 * @param {number} [opts.cellM=2]     spatial-grid cell, metres
 * @param {number} [opts.radiusM=4.5] passages further than this are ignored
 * @returns {{query:Function, count:number}}
 */
export function makeHeadingField(passages, { toM, cellM = 2, radiusM = 4.5 } = {}) {
  const n = Math.floor(passages.length / 5);
  // Bounding box of every segment endpoint.
  let minX = Infinity, minY = Infinity, maxX = -Infinity, maxY = -Infinity;
  for (let i = 0; i < n; i++) {
    const o = i * 5;
    for (const [x, y] of [[passages[o], passages[o + 1]], [passages[o + 2], passages[o + 3]]]) {
      if (x < minX) minX = x; if (x > maxX) maxX = x;
      if (y < minY) minY = y; if (y > maxY) maxY = y;
    }
  }
  const GW = n ? Math.max(1, Math.ceil((maxX - minX) / cellM) + 1) : 1;
  const GH = n ? Math.max(1, Math.ceil((maxY - minY) / cellM) + 1) : 1;
  const grid = new Array(GW * GH);
  const cellOf = (x, y) => {
    const c = Math.min(GW - 1, Math.max(0, Math.floor((x - minX) / cellM)));
    const r = Math.min(GH - 1, Math.max(0, Math.floor((y - minY) / cellM)));
    return r * GW + c;
  };
  // Rasterise each segment into the cells it passes through (half-cell samples)
  // so a query cell finds segments that merely cross it, not just its midpoint.
  for (let i = 0; i < n; i++) {
    const o = i * 5;
    const x1 = passages[o], y1 = passages[o + 1], x2 = passages[o + 2], y2 = passages[o + 3];
    const len = Math.hypot(x2 - x1, y2 - y1);
    const steps = Math.max(1, Math.ceil(len / (cellM / 2)));
    for (let s = 0; s <= steps; s++) {
      const t = s / steps;
      const idx = cellOf(x1 + (x2 - x1) * t, y1 + (y2 - y1) * t);
      (grid[idx] ?? (grid[idx] = [])).push(i);
    }
  }

  const B = 72;                 // bearing bins (2.5 deg)
  const binw = PI / B;
  const acc = new Float64Array(B);
  const smooth = new Float64Array(B);
  const seen = new Set();

  /**
   * @param {number[]} ll
   * @returns {{candidates:number[], weights:number[], confidence:number,
   *            axis:number|null}|null}
   */
  function query(ll) {
    const [px, py] = toM(ll);
    const cr = Math.ceil(radiusM / cellM);
    const c0 = Math.floor((px - minX) / cellM), r0 = Math.floor((py - minY) / cellM);
    seen.clear();
    acc.fill(0);
    let total = 0, count = 0;
    for (let dr = -cr; dr <= cr; dr++) {
      for (let dc = -cr; dc <= cr; dc++) {
        const c = c0 + dc, r = r0 + dr;
        if (c < 0 || r < 0 || c >= GW || r >= GH) continue;
        const bucket = grid[r * GW + c];
        if (!bucket) continue;
        for (const i of bucket) {
          if (seen.has(i)) continue;
          seen.add(i);
          const o = i * 5;
          const x1 = passages[o], y1 = passages[o + 1], x2 = passages[o + 2], y2 = passages[o + 3];
          if (distToSegment(px, py, x1, y1, x2, y2) > radiusM) continue;
          const w = passages[o + 4];
          const b = Math.min(B - 1, Math.floor(foldPi(bearingOf(x1, y1, x2, y2)) / binw));
          acc[b] += w;
          total += w;
          count++;
        }
      }
    }
    if (!count) return null;
    // Triangular smoothing over +-K bins (circular) so a bearing split across a
    // bin edge is still one peak.
    const K = 3;
    for (let b = 0; b < B; b++) {
      let s = 0;
      for (let k = -K; k <= K; k++) {
        s += acc[((b + k) % B + B) % B] * (1 - Math.abs(k) / (K + 1));
      }
      smooth[b] = s;
    }
    let maxw = 0;
    for (let b = 0; b < B; b++) if (smooth[b] > maxw) maxw = smooth[b];
    const thresh = Math.max(maxw * 0.5, total * 0.12);
    const candidates = [], weights = [];
    for (let b = 0; b < B; b++) {
      const prev = smooth[(b - 1 + B) % B], next = smooth[(b + 1) % B];
      if (smooth[b] < thresh || smooth[b] < prev || smooth[b] < next) continue;
      // Reject a peak that is part of the same lobe as a stronger neighbour.
      candidates.push((b + 0.5) * binw);
      weights.push(smooth[b]);
    }
    if (!candidates.length) return null;
    // Merge peaks closer than a bin apart, keep the strongest, sort desc.
    const order = weights.map((w, i) => i).sort((a, c) => weights[c] - weights[a]);
    const kept = [];
    for (const i of order) {
      if (kept.some((k) => distPi(candidates[i], candidates[k]) < binw * 1.5)) continue;
      kept.push(i);
    }
    const conf = maxw / total;
    return {
      candidates: kept.map((i) => foldPi(candidates[i])),
      weights: kept.map((i) => weights[i]),
      confidence: conf,
      axis: foldPi(candidates[kept[0]]),
    };
  }

  return { query, count: n };
}

/**
 * Snap the live heading to the nearest allowed direction and hold it.
 *
 * State machine: SEARCHING -> (nearest candidate within `snapTol`) LOCKED(dir)
 * -> (a deliberate turn) SEARCHING. While LOCKED it pulls the heading toward the
 * direction with a small bounded correction (never a hard quantise); while
 * SEARCHING it does nothing. A turn is detected from the heading stream itself,
 * so a real turn releases the lock *before* it can fight the turn.
 */
export class CompassSnapper {
  constructor({
    snapTolDeg = 35,
    releaseTolDeg = 55,
    deadbandDeg = 3,
    maxErrDeg = 50,
    gain = 0.5,
    maxCorrDegPerStep = 3,
    minConfidence = 0.4,
    turnStepDeg = 25,
    turnWindowSteps = 3,
    turnWindowDeg = 35,
  } = {}) {
    Object.assign(this, {
      snapTol: snapTolDeg * DEG,
      releaseTol: releaseTolDeg * DEG,
      deadband: deadbandDeg * DEG,
      maxErr: maxErrDeg * DEG,
      gain,
      maxCorr: maxCorrDegPerStep * DEG,
      minConfidence,
      turnStep: turnStepDeg * DEG,
      turnWindowSteps,
      turnWindow: turnWindowDeg * DEG,
    });
    this.reset();
  }

  reset() {
    this.locked = false;
    this.dir = null;
    this.confidence = 0;
    this.prevHeading = null;
    this.window = [];
    this.lastTurn = false;
  }

  /**
   * @param {number} headingRad smoothed heading
   * @param {object|null} field result of `makeHeadingField().query(ll)`
   * @returns {{locked:boolean, dir:number|null, correctionRad:number,
   *            errRad:number|null, confidence:number, reason:string}}
   */
  update(headingRad, field) {
    const idle = (reason) => ({ locked: false, dir: this.dir, correctionRad: 0,
      errRad: null, confidence: this.confidence, reason });
    if (headingRad == null || !Number.isFinite(headingRad)) return idle('no-heading');

    // --- deliberate-turn test (heading stream; releases before fighting) ---
    const delta = this.prevHeading == null ? 0 : shortestDelta(this.prevHeading, headingRad);
    this.prevHeading = headingRad;
    this.window.push(headingRad);
    if (this.window.length > this.turnWindowSteps) this.window.shift();
    const net = this.window.length >= 2
      ? Math.abs(shortestDelta(this.window[0], headingRad)) : 0;
    const turning = Math.abs(delta) > this.turnStep || net > this.turnWindow;
    this.lastTurn = turning;
    if (turning) { this.locked = false; this.dir = null; this.window = [headingRad]; }

    const conf = field ? field.confidence : 0;
    this.confidence = conf;
    const candidates = field ? field.candidates : null;

    if (!turning && candidates && candidates.length && conf >= this.minConfidence) {
      if (!this.locked) {
        // Nearest candidate to the current heading, within snapTol.
        let best = null, bestD = Infinity;
        for (const c of candidates) {
          const d = distPi(headingRad, c);
          if (d < bestD) { bestD = d; best = c; }
        }
        if (best != null && bestD <= this.snapTol) { this.locked = true; this.dir = best; }
      } else if (distPi(headingRad, this.dir) > this.releaseTol) {
        this.locked = false; this.dir = null;
      }
    } else {
      this.locked = false; this.dir = null;
    }

    if (!this.locked || this.dir == null) return idle(turning ? 'turn' : 'searching');

    // Fold the (undirected) direction to the representative nearest the heading,
    // then correct a bounded fraction of the error.
    let err = shortestDelta(headingRad, this.dir);
    if (err > PI / 2) err -= PI; else if (err <= -PI / 2) err += PI;
    if (Math.abs(err) > this.maxErr) { this.locked = false; this.dir = null; return idle('mismatch'); }
    const correctionRad = Math.abs(err) > this.deadband
      ? clamp(this.gain * err, -this.maxCorr, this.maxCorr) : 0;
    return { locked: true, dir: this.dir, correctionRad, errRad: err,
      confidence: conf, reason: correctionRad ? 'correcting' : 'locked' };
  }
}

function clamp(v, lo, hi) { return v < lo ? lo : v > hi ? hi : v; }

/** Shortest signed angular difference a -> b, in (-PI, PI]. */
export function shortestDelta(a, b) {
  let d = (b - a) % (2 * PI);
  if (d > PI) d -= 2 * PI; else if (d <= -PI) d += 2 * PI;
  return d;
}
