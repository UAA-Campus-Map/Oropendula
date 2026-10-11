/**
 * GPS / PDR position arbiter.
 *
 * Rather than switching navigation modes, this runs both sources continuously
 * and picks the one with the smaller uncertainty. GPS reports its own accuracy;
 * PDR's uncertainty is modelled as a base error that grows with the distance
 * walked since the last correction. Outdoors GPS wins; as it degrades inside a
 * building the PDR estimate takes over - automatically, with no mode flag.
 *
 * Two guards make that work:
 *   - a validity gate: a fix that lands inside a wall (when near a known
 *     building) is implausible and is discarded, because indoor GPS accuracy
 *     is often optimistically small *and wrong*;
 *   - hysteresis: the source only changes after the winner stays ahead for a
 *     few seconds, so the display does not flap at the boundary.
 *
 * A plausible, accurate-enough GPS fix also re-seeds the PDR cloud, so GPS
 * acts as a free automatic anchor and the two sources correct each other.
 */

const EARTH = 6371000;

function distanceM([lon1, lat1], [lon2, lat2]) {
  const r = Math.PI / 180;
  const dLat = (lat2 - lat1) * r;
  const dLon = (lon2 - lon1) * r;
  const a = Math.sin(dLat / 2) ** 2 +
    Math.cos(lat1 * r) * Math.cos(lat2 * r) * Math.sin(dLon / 2) ** 2;
  return 2 * EARTH * Math.asin(Math.min(1, Math.sqrt(a)));
}

export class PositionArbiter {
  /**
   * @param {object}   opts
   * @param {object}   opts.filter            a locate.mjs ParticleFilter
   * @param {Function} opts.isWalkable        (lonLat) -> bool (navgraph)
   * @param {Function} [opts.nearestWalkable] (lonLat, radius) -> lonLat|null
   * @param {number}   [opts.gpsStalenessS=6]     GPS older than this is ignored
   * @param {number}   [opts.gpsMaxAccuracyM=60]  fixes worse than this are ignored
   * @param {number}   [opts.gpsReseedAccuracyM=20] fixes better than this re-seed PDR
   * @param {number}   [opts.switchFactor=0.8]    how much better to force a switch
   * @param {number}   [opts.hysteresisS=3]       seconds before a switch commits
   * @param {number}   [opts.pdrBaseAccM=2.5]     PDR accuracy at zero distance
   * @param {number}   [opts.pdrDriftPerM=0.08]   added PDR error per metre walked
   * @param {number}   [opts.buildingZoneM=15]    within this of a wall = "at a building"
   */
  constructor({ filter, isWalkable, nearestWalkable,
    gpsStalenessS = 6, gpsMaxAccuracyM = 60, gpsReseedAccuracyM = 20,
    gpsHoldS = 10,
    switchFactor = 0.8, hysteresisS = 3,
    pdrBaseAccM = 2.5, pdrDriftPerM = 0.08, buildingZoneM = 15 } = {}) {
    Object.assign(this, {
      filter, isWalkable, nearestWalkable,
      gpsStalenessS, gpsMaxAccuracyM, gpsReseedAccuracyM, gpsHoldS,
      switchFactor, hysteresisS, pdrBaseAccM, pdrDriftPerM, buildingZoneM,
    });
    this.gpsFix = null;
    this.heldGps = null;         // last good fix, held through a source switch
    this.metresSinceCorrection = 0;
    this.source = null;          // committed source
    this._candidate = null;      // source currently ahead
    this._candidateSince = null; // when it started being ahead
    this.heading = null;
    this.lastAnchor = null;
  }

  /** A geolocation fix. `accuracy` in metres. */
  gps(fix) {
    this.gpsFix = fix;
    const plausible = this.isPlausible(fix);
    // Seed on the first plausible fix so PDR is live, and re-seed whenever a
    // good fix arrives (GPS as a free anchor).
    if (plausible && (!this.filter.initialized || fix.accuracy <= this.gpsReseedAccuracyM)) {
      this.filter.seed([fix.lon, fix.lat], {
        spreadM: Math.max(1.0, Math.min(fix.accuracy, 8)),
        heading: this.heading,
        headingSpreadDeg: this.heading == null ? 180 : 25,
      });
      this.metresSinceCorrection = 0;
    }
  }

  /** One detected step: advance the filter and grow PDR uncertainty. */
  step({ stepLengthM, headingRad }) {
    if (headingRad != null) this.heading = headingRad;
    if (!this.filter.initialized) return;
    this.filter.predict(stepLengthM, headingRad ?? this.heading ?? 0);
    this.metresSinceCorrection += stepLengthM;
  }

  /** A manual anchor (map tap): re-seed and treat as a correction. */
  anchor([lon, lat], { spreadM = 2.5 } = {}) {
    this.filter.seed([lon, lat], { spreadM, heading: this.heading, headingSpreadDeg: 30 });
    this.metresSinceCorrection = 0;
    this.lastAnchor = [lon, lat];
    this.heldGps = null;
    this.source = 'pdr';       // the estimate now comes straight from the seeded filter
    this._candidate = null;
    this._candidateSince = null;
  }

  /** A heading update from the compass (used for re-seeding direction). */
  setHeading(headingRad) {
    if (headingRad != null) this.heading = headingRad;
  }

  /**
   * Is a GPS fix believable? Off-building (outdoors) fixes are accepted; a fix
   * near/at a building must be inside the walkable area, not in a wall.
   */
  isPlausible(fix) {
    if (!(fix.accuracy > 0) || fix.accuracy > this.gpsMaxAccuracyM) return false;
    const p = [fix.lon, fix.lat];
    if (this.isWalkable?.(p)) return true;
    const near = this.nearestWalkable?.(p, Math.round(this.buildingZoneM / 0.1));
    return near == null;   // nowhere near a building -> outdoors -> fine
  }

  /** PDR position uncertainty in metres at the current walked distance. */
  pdrAccuracy() {
    return this.pdrBaseAccM + this.pdrDriftPerM * this.metresSinceCorrection;
  }

  /** The live GPS candidate, or null if stale/implausible. */
  gpsCandidate(t) {
    if (!this.gpsFix || t - this.gpsFix.t > this.gpsStalenessS * 1000) return null;
    return this.isPlausible(this.gpsFix) ? this.gpsFix : null;
  }

  /**
   * Committed estimate.
   * @returns {{lon:number, lat:number, source:'gps'|'pdr'|'anchor',
   *            accuracyM:number, heading:number|null, spreadM:number}|null}
   */
  estimate(t) {
    const g = this.gpsCandidate(t);
    if (g) this.heldGps = { ...g, t };
    const pdr = this.filter.initialized ? this.filter.estimate() : null;
    const pdrAcc = this.pdrAccuracy();

    let winner;
    if (g && !pdr) winner = 'gps';
    else if (!g && pdr) winner = 'pdr';
    else if (g && pdr) {
      if (g.accuracy < pdrAcc * this.switchFactor) winner = 'gps';
      else if (pdrAcc < g.accuracy * this.switchFactor) winner = 'pdr';
      else winner = this.source === 'pdr' ? 'pdr' : 'gps';   // too close: keep, default absolute GPS
    } else {
      return null;
    }

    // Hysteresis: only commit a change after it persists.
    if (winner !== this.source) {
      if (winner !== this._candidate) { this._candidate = winner; this._candidateSince = t; }
      else if (t - this._candidateSince >= this.hysteresisS * 1000) {
        this.source = winner;
        this._candidate = null;
        this._candidateSince = null;
      }
    } else {
      this._candidate = null;
      this._candidateSince = null;
    }

    // While committed to GPS, hold the last good fix through the switch window
    // so the display neither jumps nor flaps before hysteresis commits to PDR.
    const held = this.heldGps && t - this.heldGps.t < this.gpsHoldS * 1000 ? this.heldGps : null;
    const useGps = this.source === 'gps' ? (g || held) : null;
    const out = useGps
      ? { lon: useGps.lon, lat: useGps.lat, source: 'gps' }
      : (pdr
        ? { lon: pdr.lon, lat: pdr.lat, source: 'pdr' }
        : (g ? { lon: g.lon, lat: g.lat, source: 'gps' } : null));
    if (!out) return null;

    return {
      ...out,
      accuracyM: useGps ? useGps.accuracy : pdrAcc,
      heading: this.heading,
      spreadM: pdr ? pdr.spreadM : (useGps ? useGps.accuracy : null),
      gpsAccuracyM: g ? g.accuracy : null,
    };
  }
}
