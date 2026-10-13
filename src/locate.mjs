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
import { wrapAngle } from './motion.mjs';

const DEFAULT_COUNT = 200;

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
 * area with `isWalkable`. Returns the bearing (radians, cw-from-north) of the
 * corridor if the space is clearly elongated, else null (an open room has no
 * meaningful axis and must not snap the heading).
 */
export function corridorAxis(isWalkable, [lon, lat], {
  directions = 16,
  stepM = 0.25,
  maxM = 6.0,
  minClearM = 1.5,
  elongation = 1.8,
} = {}) {
  const n = directions;
  const clear = new Float64Array(n);
  for (let i = 0; i < n; i++) {
    const bearing = (i / n) * 2 * Math.PI;
    const east = Math.sin(bearing), north = Math.cos(bearing);
    let len = 0;
    for (let d = stepM; d <= maxM; d += stepM) {
      const p = offsetLL(lon, lat, east * d, north * d);
      if (!isWalkable(p)) break;
      len = d;
    }
    clear[i] = len;
  }
  const half = n / 2;
  let best = null;
  for (let i = 0; i < half; i++) {
    const along = Math.min(clear[i], clear[i + half]);   // the two opposite rays
    const perp = Math.max(clear[(i + half / 2) % n], clear[(i + (3 * half) / 2) % n]);
    if (along >= minClearM && along >= elongation * Math.max(perp, 0.01)) {
      const score = along;
      if (!best || score > best.score) best = { score, axis: (i / n) * 2 * Math.PI };
    }
  }
  return best ? best.axis : null;
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
