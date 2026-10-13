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

const DEFAULT_COUNT = 200;
const DEG = Math.PI / 180;

function gaussian() {
  // Box-Muller; enough for particle noise.
  let u = 0, v = 0;
  while (u === 0) u = Math.random();
  while (v === 0) v = Math.random();
  return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v);
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
    distanceNoise = 0.12, lateralNoiseM = 0.05, roughenM = 0.05 } = {}) {
    this.isWalkable = isWalkable;
    this.nearestWalkable = nearestWalkable;
    this.count = count;
    this.headingNoise = headingNoiseDeg * Math.PI / 180;
    this.distanceNoise = distanceNoise;
    this.lateralNoiseM = lateralNoiseM;
    this.roughenM = roughenM;
    this.lon = new Float64Array(count);
    this.lat = new Float64Array(count);
    this.heading = new Float64Array(count);
    this.weight = new Float64Array(count);
    this.headingBias = 0;
    // When the walker is outside the bound building, PDR must NOT be tied to
    // the floor-plan raster (skybridges, the gaps between buildings and any
    // unmapped walkway are all off it). Free roam lifts the walkability
    // weighting and the snap-back recovery, so the cloud moves freely.
    this.freeRoam = false;
    this.freeRoamAllowed = null;
    this.initialized = false;
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
    for (let i = 0; i < this.count; i++) {
      const e = gaussian() * spreadM, n = gaussian() * spreadM;
      const [plon, plat] = offsetLL(lon, lat, e, n);
      const fixed = this.freeRoam ? [plon, plat]
        : this.isWalkable([plon, plat]) ? [plon, plat]
          : (this.nearestWalkable?.([plon, plat]) ?? [plon, plat]);
      this.lon[i] = fixed[0];
      this.lat[i] = fixed[1];
      const h = heading == null ? Math.random() * 2 * Math.PI
        : heading + gaussian() * headingSpreadDeg * Math.PI / 180;
      this.heading[i] = h;
    }
    this.weight.fill(1 / this.count);
    this.initialized = true;
    this._lastStepM = 0;
  }

  /** Advance every particle one step along its heading. */
  predict(stepLengthM, headingRad) {
    if (!this.initialized) return;
    for (let i = 0; i < this.count; i++) {
      const h = headingRad + this.headingBias + gaussian() * this.headingNoise;
      const d = Math.max(0.05, stepLengthM * (1 + gaussian() * this.distanceNoise));
      const e = Math.sin(h) * d + gaussian() * this.lateralNoiseM;
      const n = Math.cos(h) * d;
      const [plon, plat] = offsetLL(this.lon[i], this.lat[i], e, n);
      this.lon[i] = plon;
      this.lat[i] = plat;
      this.heading[i] = h;
    }
    this._update();
  }

  /** Weight particles by walkability, then resample if they are collapsing. */
  _update() {
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
      let sumSq = 0;
      for (let i = 0; i < this.count; i++) sumSq += this.weight[i] * this.weight[i];
      if (sumSq > 2 / this.count) this._resample();
      return;
    }
    let total = 0, live = 0;
    for (let i = 0; i < this.count; i++) {
      const w = this.isWalkable([this.lon[i], this.lat[i]]) ? 1 : 0.02;  // hard-ish floor
      this.weight[i] = w;
      total += w;
      if (w > 0.5) live++;
    }
    if (total <= 0) return;
    for (let i = 0; i < this.count; i++) this.weight[i] /= total;

    // Nearly everything died: recover by snapping to the nearest free cell.
    if (live < 0.1 * this.count && this.nearestWalkable) { this._recover(); return; }

    // Resample when the effective sample size (1/sum(w^2)) drops below half.
    let sumSq = 0;
    for (let i = 0; i < this.count; i++) sumSq += this.weight[i] * this.weight[i];
    if (sumSq > 2 / this.count) this._resample();
  }

  _resample() {
    const { count, weight } = this;
    const outLon = new Float64Array(count), outLat = new Float64Array(count);
    const outH = new Float64Array(count);
    const step = 1 / count;
    let r = Math.random() * step, c = weight[0], idx = 0;
    for (let i = 0; i < count; i++) {
      const u = r + i * step;
      while (u > c && idx < count - 1) { idx++; c += weight[idx]; }
      outLon[i] = this.lon[idx]; outLat[i] = this.lat[idx]; outH[i] = this.heading[idx];
    }
    // Roughening so copies do not stay identical forever.
    for (let i = 0; i < count; i++) {
      const [plon, plat] = offsetLL(outLon[i], outLat[i],
        gaussian() * this.roughenM, gaussian() * this.roughenM);
      this.lon[i] = plon; this.lat[i] = plat;
      this.heading[i] = outH[i] + gaussian() * this.headingNoise * 0.5;
    }
    this.weight.fill(1 / count);
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
    let lon = 0, lat = 0, wsum = 0, sin = 0, cos = 0;
    for (let i = 0; i < this.count; i++) {
      const w = this.weight[i];
      wsum += w;
      lon += this.lon[i] * w;
      lat += this.lat[i] * w;
      sin += Math.sin(this.heading[i]) * w;
      cos += Math.cos(this.heading[i]) * w;
    }
    if (wsum <= 0) return null;
    lon /= wsum; lat /= wsum;
    const mpd = metresPerDegree(lat);
    let varSum = 0;
    for (let i = 0; i < this.count; i++) {
      const e = (this.lon[i] - lon) * mpd.lon;
      const n = (this.lat[i] - lat) * mpd.lat;
      varSum += this.weight[i] * (e * e + n * n);
    }
    return {
      lon, lat,
      heading: Math.atan2(sin, cos),
      spreadM: Math.sqrt(varSum / wsum),
    };
  }

  /** Snap an arbitrary lon/lat to the nearest walkable point (for display). */
  snap(point) {
    if (this.isWalkable(point)) return point;
    return this.nearestWalkable?.(point) ?? point;
  }

  /** Nudge the shared heading bias (e.g. from a corridor axis observation). */
  correctHeadingBias(deltaRad) {
    this.headingBias = wrapAngle(this.headingBias + deltaRad);
  }
}
