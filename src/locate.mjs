/**
 * PDR map-matching: a particle filter constrained to the walkable floor plan.
 *
 * The motion model is deliberately simple - on each detected step, every
 * particle advances `stepLength` metres along its heading with Gaussian noise.
 * The floor plan does the heavy lifting: particles that drift into a wall are
 * weighted out, so the cloud is squeezed into corridors and rooms. This is
 * what makes a modest step/heading error good enough for a few metres of
 * accuracy indoors.
 *
 * Walkability comes from navgraph.mjs's `isWalkable`/`nearestWalkable` (the
 * same clearance raster the router uses), so the filter and the router can
 * never disagree about where a body fits.
 *
 * Positions are lon/lat; displacements are metres converted through
 * geo.mjs's metresPerDegree. The filter never mixes the two.
 */

import { metresPerDegree } from './geo.mjs';
import { wrapAngle, shortestDelta, clamp } from './motion.mjs';

const DEFAULT_COUNT = 1000;
const DEG = Math.PI / 180;

function gaussian() {
  // Box-Muller; enough for particle noise.
  let u = 0, v = 0;
  while (u === 0) u = Math.random();
  while (v === 0) v = Math.random();
  return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v);
}

/** Gaussian draw clamped to +/- `maxAbs` (a bounded prior, e.g. heading noise). */
function gaussianClamped(sigma, maxAbs) {
  return Math.max(-maxAbs, Math.min(maxAbs, gaussian() * sigma));
}

/** Distance in metres between two lon/lat points. */
function distM(lon1, lat1, lon2, lat2) {
  const mpd = metresPerDegree((lat1 + lat2) / 2);
  return Math.hypot((lon2 - lon1) * mpd.lon, (lat2 - lat1) * mpd.lat);
}

/** Offset a lon/lat by (east, north) metres. */
function offsetLL(lon, lat, eastM, northM) {
  const mpd = metresPerDegree(lat);
  return [lon + eastM / mpd.lon, lat + northM / mpd.lat];
}

/**
 * Estimate the local corridor axis at a point by ray-casting the walkable
 * area with `isWalkable`.
 *
 * A corridor is a *narrow* free space, so we require a large aspect ratio: the
 * free run ALONG the axis must be at least `aspectRatio` times the free run
 * ACROSS it. We measure both as the full chord through the point (the two
 * opposite rays summed), which makes the ratio honest - a corridor narrower
 * than `2*maxM/aspectRatio` is the widest space that can ever qualify.
 *
 * This "very elongated" gate is what keeps the caller from snapping the heading
 * in an open room, a wide lobby, at an intersection (the cross-corridor blows
 * up the across-run) or at an L-corner (both chords are long). An elongated
 * space that passes is unambiguous: its axis is parallel to the walls.
 *
 * Runtime cost is bounded: a coarse sweep finds the rough axis, then a fine
 * sweep refines it to ~1 deg. Rays are memoised, so opposite/perpendicular
 * directions - which the coarse sweep reuses - are cast once.
 *
 * Returns the bearing (radians, cw-from-north, in [0, PI)) of the corridor if
 * one is found, else null.
 */
export function corridorAxis(isWalkable, [lon, lat], {
  stepM = 0.25,
  maxM = 15.0,
  aspectRatio = 9.0,
  minLengthM = 3.0,
  minWidthM = 0.4,
  coarseSteps = 24,
  refineStepDeg = 1,
  refineSpanDeg = 8,
} = {}) {
  const cache = new Map();
  // Distance to the first blocked cell along `bearing`, capped at maxM.
  const cast = (bearing) => {
    const key = bearing.toFixed(6);
    let len = cache.get(key);
    if (len !== undefined) return len;
    const east = Math.sin(bearing), north = Math.cos(bearing);
    len = 0;
    for (let d = stepM; d <= maxM; d += stepM) {
      if (!isWalkable(offsetLL(lon, lat, east * d, north * d))) break;
      len = d;
    }
    cache.set(key, len);
    return len;
  };
  // Full free chord along `bearing` and the full chord across it.
  const measure = (bearing) => ({
    length: cast(bearing) + cast(wrapAngle(bearing + Math.PI)),
    width: cast(wrapAngle(bearing + Math.PI / 2)) + cast(wrapAngle(bearing - Math.PI / 2)),
  });
  const passes = (m) => m.length >= minLengthM && m.width >= minWidthM &&
    m.length / m.width >= aspectRatio;

  let best = null;
  for (let i = 0; i < coarseSteps; i++) {
    const bearing = (i / coarseSteps) * 2 * Math.PI;
    const m = measure(bearing);
    if (passes(m) && (!best || m.length > best.length)) best = { bearing, length: m.length };
  }
  if (!best) return null;

  // Refine around the coarse winner at ~1 deg, keeping the longest qualifying
  // chord (the ratio is flat near the true axis, so length picks the centre).
  let axis = best.bearing, axisLen = best.length;
  for (let k = -refineSpanDeg; k <= refineSpanDeg; k += refineStepDeg) {
    if (k === 0) continue;
    const bearing = wrapAngle(best.bearing + k * DEG);
    const m = measure(bearing);
    if (passes(m) && m.length > axisLen) { axisLen = m.length; axis = bearing; }
  }
  return wrapAngle(axis);
}

/**
 * Walks-straight-down-a-hallway compass-drift corrector.
 *
 * The floor plan knows the hallway axis (from `corridorAxis`); a walking human
 * knows which way they are going. When the two say different things, the
 * compass has drifted. This class watches the heading stream and, once the
 * walker has gone 6 consecutive near-straight steps in a clearly elongated
 * (corridor) space, emits a small proportional correction toward the axis each
 * step - and keeps emitting until an *intentional* turn (a sharp step, or a
 * quick net rotation) ends the run.
 *
 * It is pure and frame-agnostic: the caller feeds the smoothed heading and the
 * axis (or null) and applies the returned `correctionRad` to whatever bias it
 * owns. `wantsAxis` lets the caller ray-cast only once arming is imminent.
 *
 * @param {number} [straightSteps=6]   consecutive low-turn steps to arm
 * @param {number} [straightTurnDeg=6] per-step change still counted as straight
 * @param {number} [netTurnDeg=15]     net change over the streak that breaks it
 * @param {number} [turnStepDeg=25]    one-step change = intentional turn
 * @param {number} [turnWindowSteps=3] window for the net-turn test
 * @param {number} [turnWindowDeg=35]  net change over that window = turn
 * @param {number} [deadbandDeg=3]     ignore errors smaller than this
 * @param {number} [maxErrDeg=50]      a larger mismatch = distrust the axis
 * @param {number} [gain=0.4]          fraction of the error corrected per step
 * @param {number} [maxCorrDegPerStep=3] per-step correction clamp
 */
export class HeadingDriftCalibrator {
  constructor({
    straightSteps = 6,
    straightTurnDeg = 6,
    netTurnDeg = 15,
    turnStepDeg = 25,
    turnWindowSteps = 3,
    turnWindowDeg = 35,
    deadbandDeg = 3,
    maxErrDeg = 50,
    gain = 0.4,
    maxCorrDegPerStep = 3,
  } = {}) {
    Object.assign(this, {
      straightSteps,
      straightTurn: straightTurnDeg * DEG,
      netTurn: netTurnDeg * DEG,
      turnStep: turnStepDeg * DEG,
      turnWindowSteps,
      turnWindow: turnWindowDeg * DEG,
      deadband: deadbandDeg * DEG,
      maxErr: maxErrDeg * DEG,
      gain,
      maxCorr: maxCorrDegPerStep * DEG,
    });
    this.reset();
  }

  reset() {
    this.prevHeading = null;
    this.streak = 0;
    this.streakHeading = null;   // heading at the start of the current streak
    this.window = [];            // recent headings (intentional-turn test)
    this.active = false;
  }

  /** True once a corridor axis for the NEXT step is worth ray-casting. */
  get wantsAxis() {
    return this.streak >= this.straightSteps - 1;
  }

  /**
   * @param {number} headingRad current smoothed heading
   * @param {number|null} axisRad corridor axis from `corridorAxis`, or null
   * @returns {{active:boolean, correctionRad:number, errRad:number|null,
   *            streak:number, reason:string}}
   */
  update(headingRad, axisRad) {
    const idle = (reason) => ({ active: false, correctionRad: 0, errRad: null,
      streak: this.streak, reason });
    if (headingRad == null || !isFinite(headingRad)) return idle('no-heading');

    const delta = this.prevHeading == null ? 0 : shortestDelta(this.prevHeading, headingRad);
    this.prevHeading = headingRad;

    this.window.push(headingRad);
    if (this.window.length > this.turnWindowSteps) this.window.shift();
    const windowNet = this.window.length >= 2
      ? Math.abs(shortestDelta(this.window[0], headingRad)) : 0;
    if (Math.abs(delta) > this.turnStep || windowNet > this.turnWindow) {
      const wasActive = this.active;
      this._newStreak(headingRad);
      this.active = false;
      return idle(wasActive ? 'turn' : 'turn-early');
    }

    if (this.streakHeading == null) {
      this._newStreak(headingRad);
    } else {
      const net = Math.abs(shortestDelta(this.streakHeading, headingRad));
      if (Math.abs(delta) <= this.straightTurn && net <= this.netTurn) this.streak++;
      else this._newStreak(headingRad);
    }

    // No corridor here (open room, wide lobby, intersection, corner): pause,
    // but keep counting straightness so walking into a corridor arms quickly.
    if (axisRad == null) { this.active = false; return idle('no-axis'); }
    if (this.streak < this.straightSteps) { this.active = false; return idle('arming'); }

    // The axis is undirected, so fold the error into (-90, 90]: never flip the
    // walker around, only trim drift. `err` is (axis - heading); adding
    // `gain*err` to the heading turns it toward the axis.
    let err = shortestDelta(headingRad, axisRad);
    if (err > Math.PI / 2) err -= Math.PI;
    else if (err <= -Math.PI / 2) err += Math.PI;

    if (Math.abs(err) > this.maxErr) { this.active = false; return idle('mismatch'); }

    this.active = true;
    const correctionRad = Math.abs(err) > this.deadband
      ? clamp(this.gain * err, -this.maxCorr, this.maxCorr) : 0;
    return { active: true, correctionRad, errRad: err, streak: this.streak,
      reason: correctionRad ? 'correcting' : 'locked' };
  }

  _newStreak(headingRad) {
    this.streak = 1;
    this.streakHeading = headingRad;
    this.window = [headingRad];
  }
}

/**
 * The particle filter.
 *
 * @param {object}   opts
 * @param {Function} opts.isWalkable       (lonLat) -> bool, from navgraph
 * @param {Function} [opts.nearestWalkable] (lonLat) -> lonLat|null, for recovery
 * @param {number}   [opts.count=200]
 * @param {number}   [opts.headingNoiseDeg=8]   per-step heading jitter
 * @param {number}   [opts.distanceNoise=0.12]  per-step distance jitter, fraction of step
 * @param {number}   [opts.lateralNoiseM=0.05]  per-step lateral jitter, metres
 * @param {number}   [opts.roughenM=0.05]       post-resample roughening
 */
export class ParticleFilter {
  constructor({ isWalkable, nearestWalkable, count = DEFAULT_COUNT, headingNoiseDeg = 8,
    headingJitterDeg = 35, distanceNoise = 0.22, lateralNoiseM = 0.20, roughenM = 0.20,
    segmentStepM = 0.1, behindWeight = 0.05, reversalDeg = 120,
    stallEpsM = 0.2, stallSteps = 3, relaxSteps = 5, respawnSpreadM = 0.3,
    persistentHeadingDeg = 0 } = {}) {
    this.isWalkable = isWalkable;
    this.nearestWalkable = nearestWalkable;
    // Fixed particle count (arrays are sized to it).
    this.count = count;
    this.headingNoise = headingNoiseDeg * Math.PI / 180;
    // Uniform random heading jitter applied to every particle each step.
    this.headingJitter = headingJitterDeg * Math.PI / 180;
    this.distanceNoise = distanceNoise;
    this.lateralNoiseM = lateralNoiseM;
    this.roughenM = roughenM;
    // Where a wall-killed particle's replacement is spawned (spread around the
    // cloud mean).
    this.respawnSpreadM = respawnSpreadM;
    // Segment-collision sampling pitch. Matches navgraph's 0.1 m cell so a
    // ~0.3 m doorway channel is never falsely blocked.
    this.segmentStepM = segmentStepM;
    // Rear-hemisphere particles are down-weighted (not killed) this much.
    this.behindWeight = behindWeight;
    // A measured heading swing this large is an intentional U-turn.
    this.reversal = reversalDeg * Math.PI / 180;
    // Stall guard: if the mean won't advance for this many steps while the
    // walker keeps stepping, relax the priors briefly so the cloud re-diffuses.
    this.stallEpsM = stallEpsM;
    this.stallSteps = stallSteps;
    this.relaxSteps = relaxSteps;
    const cap = count;
    this.lon = new Float64Array(cap);
    this.lat = new Float64Array(cap);
    this.heading = new Float64Array(cap);
    this.weight = new Float64Array(cap);
    // Scratch buffers for the allocation-free ping-pong resample.
    this._lon2 = new Float64Array(cap);
    this._lat2 = new Float64Array(cap);
    this._h2 = new Float64Array(cap);
    // Persistent per-particle heading OFFSET. With the default i.i.d. per-step
    // jitter (persistentHeadingDeg = 0) every particle re-randomises its heading
    // each step, so a trajectory's SHAPE carries no information. With a
    // persistent offset the particle keeps one coherent heading error, so the
    // measured turns become informative.
    this.persistentHeading = persistentHeadingDeg * DEG;
    this._hOff = new Float64Array(cap);
    this._hOff2 = new Float64Array(cap);
    this._pt = [0, 0];   // scratch lon/lat (avoids per-sample allocation)
    this._mpd = metresPerDegree(0);   // per-step metres-per-degree cache
    // Steps at most this long may use the 2-sample fast path in `_advance`
    // (a short step cannot jump the ~0.6 m clearance band).
    this._fastAdvanceMaxM = 1.0;
    this.headingBias = 0;
    // When the walker is outside the bound building, PDR must NOT be tied to
    // the floor-plan raster (skybridges, the gaps between buildings and any
    // unmapped walkway are all off it). Free roam lifts the walkability
    // weighting and the snap-back recovery, so the cloud moves freely.
    this.freeRoam = false;
    this.freeRoamAllowed = null;
    this.initialized = false;
    this.lastHeading = null;   // measured heading at the previous step
    this.reversed = false;     // this step followed a ~180 deg U-turn
    this.accuracyM = Infinity; // the arbiter's trusted accuracy radius
    this.stallCount = 0;
    this.relaxLeft = 0;
    this._prevMean = null;
    // Bumped whenever the cloud actually changes (seed / step). The renderer
    // uses it to skip rebuilding the particle overlay for GPS-only estimates
    // that leave the PDR cloud untouched.
    this.version = 0;
  }

  /**
   * Outside a building: move freely (no floor-plan constraint). An optional
   * `isAllowed(ll)` predicate constrains that freedom - the app passes
   * "outside every building that has a plan on this floor", so PDR cannot cut
   * through a building while still working over one (a skybridge above a
   * building with no floor at this level stays free).
   */
  setFreeRoam(on, isAllowed = null) {
    this.freeRoam = !!on;
    this.freeRoamAllowed = this.freeRoam ? isAllowed : null;
  }

  /** Reset the cloud around a known point (a manual anchor). */
  seed([lon, lat], { spreadM = 2.5, heading = null, headingSpreadDeg = 180 } = {}) {
    // The compass cannot be more than ~90 deg off, so a KNOWN heading bounds
    // the spread to +/-90 deg; only an unknown heading keeps the full circle.
    const spreadDeg = heading == null ? null : Math.min(headingSpreadDeg, 90);
    for (let i = 0; i < this.count; i++) {
      const e = gaussian() * spreadM, n = gaussian() * spreadM;
      const [plon, plat] = offsetLL(lon, lat, e, n);
      const fixed = this.freeRoam ? [plon, plat]
        : this.isWalkable([plon, plat]) ? [plon, plat]
          : (this.nearestWalkable?.([plon, plat]) ?? [plon, plat]);
      this.lon[i] = fixed[0];
      this.lat[i] = fixed[1];
      const h = spreadDeg == null ? Math.random() * 2 * Math.PI
        : heading + gaussianClamped(spreadDeg * DEG, Math.PI / 2);
      this.heading[i] = h;
      this._hOff[i] = this.persistentHeading
        ? gaussianClamped(this.persistentHeading, Math.PI / 2) : 0;
    }
    this.weight.fill(1 / this.count);
    this.initialized = true;
    this._lastStepM = 0;
    this.lastHeading = null;
    this.reversed = false;
    this.stallCount = 0;
    this.relaxLeft = 0;
    this._prevMean = null;
    this.version++;
  }

  predict(stepLengthM, headingRad, { accuracyM = Infinity } = {}) {
    if (!this.initialized) return;
    this.accuracyM = accuracyM;

    // A ~180 deg swing in the measured heading is an intentional U-turn. This
    // step must not punish the (now-behind) particles or the cloud would
    // starve, so flag it - the next _update lifts the rear penalty and cap.
    this.reversed = this.lastHeading != null
      && Math.abs(shortestDelta(this.lastHeading, headingRad)) > this.reversal;
    this.lastHeading = headingRad;

    // One metres-per-degree for the whole step: latitude varies by <0.001 deg
    // across a floor, so recomputing it per particle (40k times) was pure waste.
    const mpd = metresPerDegree(this.lat[0]);
    this._mpd = mpd;
    const jitter = this.headingJitter, distNoise = this.distanceNoise, latNoise = this.lateralNoiseM;
    const bias = this.headingBias, hOff = this._hOff;
    for (let i = 0; i < this.count; i++) {
      // Uniform random heading jitter in +/- headingJitter (default +/-35 deg).
      const h = headingRad + bias + hOff[i] + (Math.random() * 2 - 1) * jitter;
      const d = Math.max(0.05, stepLengthM * (1 + gaussian() * distNoise));
      // Lateral jitter PERPENDICULAR to the travel direction (not on the east
      // axis), so the cloud spreads across the corridor rather than along it.
      const l = gaussian() * latNoise;
      const sh = Math.sin(h), ch = Math.cos(h);
      const e = sh * d + ch * l;
      const n = ch * d - sh * l;
      if (this.freeRoam) {
        this.lon[i] += e / mpd.lon;
        this.lat[i] += n / mpd.lat;
      } else {
        this._move(i, e, n, d, h);
      }
      this.heading[i] = h;
    }
    this._update(headingRad);
    this._trackStall();
    this.version++;
  }

  /**
   * Move particle `i` by (eastM, northM). The intended step is swept for wall
   * collisions (`_advance`); if a wall blocks most of it, the particle fans out
   * at increasing angles and takes the alternative that advances furthest.
   * Every alternative is swept too, so none can cross a wall - this does NOT
   * reopen tunnelling. It is what lets a particle pressed against a wall slide
   * along it to the doorway and spread to the next room, instead of freezing.
   */
  /**
   * Move particle `i` by (eastM, northM). The step is swept for wall
   * collisions (`_advance`); a particle whose step a wall blocks is a wrong
   * guess, so it is destroyed immediately and replaced by a fresh random one
   * (`_respawn`) - it never piles up against the wall.
   */
  _move(i, eastM, northM, dist, heading) {
    const fromLon = this.lon[i], fromLat = this.lat[i];
    const mpd = this._mpd;
    const toLon = fromLon + eastM / mpd.lon, toLat = fromLat + northM / mpd.lat;
    const p = this._advance(fromLon, fromLat, toLon, toLat);
    const ddx = (p[0] - fromLon) * mpd.lon, ddy = (p[1] - fromLat) * mpd.lat;
    const moved = Math.sqrt(ddx * ddx + ddy * ddy);
    if (moved < dist * 0.9) { this._respawn(i, heading); return; }
    this.lon[i] = p[0];
    this.lat[i] = p[1];
  }

  /**
   * Replace particle `i` with a fresh one. Its POSITION is inherited from a
   * random SURVIVING particle (plus a small jitter) - never a fresh random
   * point, which could sit in another room behind a wall the walker could not
   * have reached. Its heading is the step heading (measured +/- jitter).
   */
  _respawn(i, heading) {
    const src = this._pickSurvivor();
    const bx = src >= 0 ? this.lon[src] : this.lon[i];
    const by = src >= 0 ? this.lat[src] : this.lat[i];
    const [plon, plat] = offsetLL(bx, by,
      gaussian() * this.respawnSpreadM, gaussian() * this.respawnSpreadM);
    if (this.freeRoam || this.isWalkable([plon, plat])) { this.lon[i] = plon; this.lat[i] = plat; }
    else { this.lon[i] = bx; this.lat[i] = by; }
    this.heading[i] = heading;
    this._hOff[i] = src >= 0 ? this._hOff[src] : 0;
  }

  /** Index of a random particle that still carries weight, or -1. */
  _pickSurvivor() {
    for (let t = 0; t < 12; t++) {
      const j = (Math.random() * this.count) | 0;
      if (this.weight[j] > 0) return j;
    }
    return -1;
  }

  /**
   * Walk from a (walkable) point toward (toLon,toLat), stopping at the last
   * walkable sample. Prevents stepping across a wall. Writes into the reusable
   * `_pt` scratch array (no per-sample allocation) and returns it.
   */
  _advance(fromLon, fromLat, toLon, toLat) {
    const pt = this._pt;
    const mpd = this._mpd;
    const dx = (toLon - fromLon) * mpd.lon, dy = (toLat - fromLat) * mpd.lat;
    const len = Math.sqrt(dx * dx + dy * dy);
    if (len <= this.segmentStepM) {
      pt[0] = toLon; pt[1] = toLat;
      if (!this.isWalkable(pt)) { pt[0] = fromLon; pt[1] = fromLat; }
      return pt;
    }
    // FAST PATH. The clearance raster inflates every wall by ~0.3 m each side
    // (a ~0.6 m blocked band), so a short step whose midpoint and endpoint are
    // both clear cannot have jumped a wall - test those two instead of sweeping
    // every 0.1 m. Only fall through to the sweep when a sample is blocked.
    if (len <= this._fastAdvanceMaxM) {
      const mx = fromLon + (toLon - fromLon) * 0.5, my = fromLat + (toLat - fromLat) * 0.5;
      pt[0] = mx; pt[1] = my;
      if (this.isWalkable(pt)) {
        pt[0] = toLon; pt[1] = toLat;
        if (this.isWalkable(pt)) return pt;
      }
      // blocked somewhere on the segment -> full sweep below
    }
    const steps = Math.ceil(len / this.segmentStepM);
    let lastLon = fromLon, lastLat = fromLat;
    for (let s = 1; s <= steps; s++) {
      const t = s / steps;
      pt[0] = fromLon + (toLon - fromLon) * t;
      pt[1] = fromLat + (toLat - fromLat) * t;
      if (!this.isWalkable(pt)) break;
      lastLon = pt[0]; lastLat = pt[1];
    }
    pt[0] = lastLon; pt[1] = lastLat;
    return pt;
  }

  /** Weight particles by walkability, apply the priors, then resample. */
  _update(forwardHeading = null) {
    const relaxed = this.relaxLeft > 0;
    // Free roam (outdoors). With no predicate every particle is equally
    // plausible; with one (outside all buildings on this floor) the forbidden
    // footprints are down-weighted and resampled away, so the cloud flows
    // around them instead of walking through.
    if (this.freeRoam) {
      const allow = this.freeRoamAllowed;
      if (!allow) {
        this.weight.fill(1 / this.count);
        return;
      }
      let total = 0;
      for (let i = 0; i < this.count; i++) {
        const w = allow([this.lon[i], this.lat[i]]) ? 1 : 0.02;
        this.weight[i] = w;
        total += w;
      }
      if (total <= 0) return;
      for (let i = 0; i < this.count; i++) this.weight[i] /= total;
      this._applyPriors(forwardHeading, relaxed);
      let sumSq = 0;
      for (let i = 0; i < this.count; i++) sumSq += this.weight[i] * this.weight[i];
      if (sumSq > 2 / this.count) this._resample();
      if (relaxed) this.relaxLeft--;
      return;
    }
    let total = 0, live = 0;
    for (let i = 0; i < this.count; i++) {
      const w = this.isWalkable([this.lon[i], this.lat[i]]) ? 1 : 0.02;
      this.weight[i] = w;
      total += w;
      if (w > 0.5) live++;
    }
    if (total <= 0) return;
    for (let i = 0; i < this.count; i++) this.weight[i] /= total;

    // Nearly everything died: recover by snapping to the nearest free cell.
    if (live < 0.1 * this.count && this.nearestWalkable) {
      this._recover();
      if (relaxed) this.relaxLeft--;
      return;
    }

    this._applyPriors(forwardHeading, relaxed);

    // Resample when the effective sample size (1/sum(w^2)) drops below half.
    let sumSq = 0;
    for (let i = 0; i < this.count; i++) sumSq += this.weight[i] * this.weight[i];
    if (sumSq > 2 / this.count) this._resample();
    if (relaxed) this.relaxLeft--;
  }

  /**
   * The two directional/spatial priors, applied after walkability weighting:
   *   - forward hemisphere: particles behind the measured heading are
   *     implausible while walking, so they are down-weighted (kept, weakly, so
   *     a real U-turn is still detectable);
   *   - accuracy radius: the arbiter's accuracy is trusted, so nothing lives
   *     farther than `accuracyM` from the cloud's own mean.
   * A U-turn step or a stall-recovery step lifts both so the cloud can move.
   */
  _applyPriors(forwardHeading, relaxed) {
    const mean = this._weightedMean();
    if (!mean) return;
    const mpd = metresPerDegree(mean.lat);
    const fe = forwardHeading == null ? 0 : Math.sin(forwardHeading);
    const fn = forwardHeading == null ? 0 : Math.cos(forwardHeading);
    const useRear = forwardHeading != null && !relaxed && !this.reversed;
    const useCap = !relaxed && isFinite(this.accuracyM);
    const cap2 = this.accuracyM * this.accuracyM;
    for (let i = 0; i < this.count; i++) {
      const de = (this.lon[i] - mean.lon) * mpd.lon;
      const dn = (this.lat[i] - mean.lat) * mpd.lat;
      if (useCap && de * de + dn * dn > cap2) { this.weight[i] = 0; continue; }
      if (useRear && de * fe + dn * fn < 0) this.weight[i] *= this.behindWeight;
    }
    let total = 0;
    for (let i = 0; i < this.count; i++) total += this.weight[i];
    if (total <= 0) { this.weight.fill(1 / this.count); return; }
    for (let i = 0; i < this.count; i++) this.weight[i] /= total;
  }

  /** Weighted-mean position of the cloud, or null if it carries no weight. */
  _weightedMean() {
    let lon = 0, lat = 0, wsum = 0;
    for (let i = 0; i < this.count; i++) {
      const w = this.weight[i];
      wsum += w; lon += this.lon[i] * w; lat += this.lat[i] * w;
    }
    if (wsum <= 0) return null;
    return { lon: lon / wsum, lat: lat / wsum };
  }

  /**
   * Stall guard. The walker keeps stepping but the mean will not advance (e.g.
   * the cloud is misplaced and every step is wall-blocked): relax the priors
   * for a few steps so the natural noise re-diffuses the cloud until a particle
   * finds the doorway. Never lets particles cross a wall - collision stays on.
   */
  _trackStall() {
    const mean = this._weightedMean();
    if (mean && this._prevMean) {
      const moved = distM(this._prevMean.lon, this._prevMean.lat, mean.lon, mean.lat);
      if (moved < this.stallEpsM) this.stallCount++;
      else this.stallCount = 0;
      if (this.stallCount >= this.stallSteps) {
        this.relaxLeft = this.relaxSteps;
        this.stallCount = 0;
      }
    }
    this._prevMean = mean;
  }

  _resample() {
    const n = this.count;
    const { weight } = this;
    const sLon = this._lon2, sLat = this._lat2, sH = this._h2, sHO = this._hOff2;
    const step = 1 / n;
    let r = Math.random() * step, c = weight[0], idx = 0;
    const mpd = metresPerDegree(this.lat[0]);
    for (let i = 0; i < n; i++) {
      const u = r + i * step;
      while (u > c && idx < n - 1) { idx++; c += weight[idx]; }
      // Roughening so copies do not stay identical forever.
      const re = gaussian() * this.roughenM, rn = gaussian() * this.roughenM;
      const plon = this.lon[idx] + re / mpd.lon, plat = this.lat[idx] + rn / mpd.lat;
      const h = this.heading[idx] + gaussian() * this.headingNoise * 0.5;
      sLon[i] = plon; sLat[i] = plat; sH[i] = h; sHO[i] = this._hOff[idx];
    }
    // Ping-pong: swap the scratch buffers in as the live ones.
    let tmp;
    tmp = this.lon; this.lon = this._lon2; this._lon2 = tmp;
    tmp = this.lat; this.lat = this._lat2; this._lat2 = tmp;
    tmp = this.heading; this.heading = this._h2; this._h2 = tmp;
    tmp = this._hOff; this._hOff = this._hOff2; this._hOff2 = tmp;
    const w = 1 / this.count;
    for (let i = 0; i < this.count; i++) this.weight[i] = w;
  }

  _recover() {
    for (let i = 0; i < this.count; i++) {
      const near = this.nearestWalkable([this.lon[i], this.lat[i]]);
      if (near) { this.lon[i] = near[0]; this.lat[i] = near[1]; }
    }
    this.weight.fill(1 / this.count);
  }

  /**
   * Weighted estimate: mean position, circular-mean heading, and the RMS
   * spread (position uncertainty) in metres.
   */
  estimate() {
    if (!this.initialized) return null;
    // With the floor plan in play the reported point must be walkable: a cloud
    // straddling a thin wall averages INTO the wall, so average only the
    // walkable particles. If even that mean lands in a wall, report the
    // heaviest walkable particle (never a spiral snap that could cross one).
    // Free roam skips the constraint (off-raster spaces).
    const snap = !this.freeRoam && typeof this.isWalkable === 'function';
    let lon = 0, lat = 0, wsum = 0, sin = 0, cos = 0;
    let bestW = -1, bestLon = 0, bestLat = 0, bestHeading = 0;
    for (let i = 0; i < this.count; i++) {
      const w = this.weight[i];
      if (snap && !this.isWalkable([this.lon[i], this.lat[i]])) continue;
      wsum += w;
      lon += this.lon[i] * w;
      lat += this.lat[i] * w;
      sin += Math.sin(this.heading[i]) * w;
      cos += Math.cos(this.heading[i]) * w;
      if (w > bestW) { bestW = w; bestLon = this.lon[i]; bestLat = this.lat[i]; bestHeading = this.heading[i]; }
    }
    let denom = wsum;
    let offPlan = false;
    if (wsum <= 0) {
      // Everything is off-plan: fall back to the raw weighted mean so there is
      // still a report (an anchor / GPS fix / the stall guard recovers).
      offPlan = true;
      for (let i = 0; i < this.count; i++) {
        const w = this.weight[i];
        denom += w;
        lon += this.lon[i] * w;
        lat += this.lat[i] * w;
        sin += Math.sin(this.heading[i]) * w;
        cos += Math.cos(this.heading[i]) * w;
      }
      if (denom <= 0) return null;
      lon /= denom; lat /= denom;
    } else {
      lon /= wsum; lat /= wsum;
      if (snap && !this.isWalkable([lon, lat])) {
        lon = bestLon; lat = bestLat;
        sin = Math.sin(bestHeading); cos = Math.cos(bestHeading);
      }
    }
    const mpd = metresPerDegree(lat);
    let varSum = 0;
    for (let i = 0; i < this.count; i++) {
      // In the off-plan fallback every particle is off-plan, so the walkability
      // filter would discard them all and report a false spread of 0.
      if (snap && !offPlan && !this.isWalkable([this.lon[i], this.lat[i]])) continue;
      const e = (this.lon[i] - lon) * mpd.lon;
      const n = (this.lat[i] - lat) * mpd.lat;
      varSum += this.weight[i] * (e * e + n * n);
    }
    return {
      lon, lat,
      heading: Math.atan2(sin, cos),
      spreadM: Math.sqrt(varSum / denom),
    };
  }

  /** Snap an arbitrary lon/lat to the nearest walkable point (for display). */
  snap(point) {
    if (this.isWalkable(point)) return point;
    return this.nearestWalkable?.(point) ?? point;
  }

  /**
   * The cloud as plain `{lon, lat, w}` records (w = normalised weight), for
   * on-map debug rendering. Empty until the filter is seeded.
   */
  points() {
    if (!this.initialized) return [];
    const out = [];
    for (let i = 0; i < this.count; i++) {
      out.push({ lon: this.lon[i], lat: this.lat[i], w: this.weight[i] });
    }
    return out;
  }

  /** Nudge the shared heading bias (e.g. from a corridor axis observation). */
  correctHeadingBias(deltaRad) {
    this.headingBias = wrapAngle(this.headingBias + deltaRad);
  }
}
