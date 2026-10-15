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
   * @param {Function} [opts.isOutsideBuildings] (lonLat) -> bool: allowed
   *        free-roam area outdoors - "outside every building with a plan on
   *        this floor" (null = freely anywhere)
   * @param {number}   [opts.gpsStalenessS=6]     GPS older than this is ignored
   * @param {number}   [opts.gpsMaxAccuracyM=60]  fixes worse than this are ignored
   * @param {number}   [opts.gpsSeedAccuracyM=20] the first fix of an outdoor
   *        episode re-anchors the cloud up to this accuracy (the app may have
   *        snapped it inside a building); later corrections need
   *        `gpsOverrideAccuracyM`
   * @param {number}   [opts.gpsOverrideAccuracyM=3] outdoors, GPS only overrides
   *        free-roaming PDR (and re-seeds the cloud) with a fix this good
   * @param {number}   [opts.hysteresisS=3]       seconds before a switch commits
   * @param {number}   [opts.pdrBaseAccM=2.5]     PDR accuracy at zero distance
   * @param {number}   [opts.pdrDriftPerM=0.20]   added PDR error per metre walked
   * @param {number}   [opts.buildingZoneM=15]    within this of a wall = "at a building"
   */
  constructor({ filter, isWalkable, nearestWalkable, isOutsideBuildings = null,
    gpsStalenessS = 6, gpsMaxAccuracyM = 60,
    gpsSeedAccuracyM = 20, gpsOverrideAccuracyM = 3,
    gpsHoldS = 10, hysteresisS = 3,
    pdrBaseAccM = 2.5, pdrDriftPerM = 0.20, buildingZoneM = 15 } = {}) {
    Object.assign(this, {
      filter, isWalkable, nearestWalkable, isOutsideBuildings,
      gpsStalenessS, gpsMaxAccuracyM, gpsSeedAccuracyM, gpsOverrideAccuracyM, gpsHoldS,
      hysteresisS, pdrBaseAccM, pdrDriftPerM, buildingZoneM,
    });
    this._seededOutdoors = false;
    this.gpsFix = null;
    // A heuristic re-seed (floor change / anchor) is NOT a correction: the
    // trusted radius must start at least at the seed's own spread, or the first
    // step's accuracy cap would crush the fresh cloud to a point. The floor
    // becomes irrelevant once walked distance dominates the drift term.
    this.accuracyFloorM = 0;
    this.heldGps = null;         // last good fix, held through a source switch
    this.metresSinceCorrection = 0;
    this.source = null;          // committed source
    this._candidate = null;      // source currently ahead
    this._candidateSince = null; // when it started being ahead
    this.heading = null;
    this.lastAnchor = null;
  }

  /**
   * Does this fix land inside the bound building? `isWalkable` is the bound
   * floor's own clearance raster, so a fix on it is inside that building (or
   * in its doorway band), while an outdoor fix is not.
   */
  insideBuilding(fix) {
    if (!fix || !this.isWalkable) return false;
    return !!this.isWalkable([fix.lon, fix.lat]);
  }

  /** A geolocation fix. `accuracy` in metres. */
  gps(fix) {
    this.gpsFix = fix;
    const plausible = this.isPlausible(fix);
    const indoor = this.insideBuilding(fix);
    const outdoor = plausible && !indoor;
    // Outdoors the cloud must not be tied to the floor plan (skybridges, the
    // gaps between buildings, unmapped walkways are all off the raster), so
    // free-roam on every believable outdoor fix - constrained to outside every
    // building that has a plan on this floor, so PDR cannot cut through one.
    this.filter.setFreeRoam?.(outdoor, this.isOutsideBuildings);
    // Outdoors the cloud is re-anchored from a fix when it has never been
    // seeded outdoors (the app may have snapped it into a building; up to
    // gpsSeedAccuracyM) or whenever the fix is more accurate than the PDR
    // estimate itself. Only ONCE per session for the first case: re-anchoring
    // on every indoor<->outdoor flicker near a doorway teleported the estimate
    // (the reported "jumps 20 ft" bug).
    if (outdoor) {
      const firstOutdoor = !this._seededOutdoors;
      const betterThanPdr = fix.accuracy < this.pdrAccuracy();
      if (!this.filter.initialized ||
          (firstOutdoor && fix.accuracy <= this.gpsSeedAccuracyM) ||
          betterThanPdr) {
        this.filter.seed([fix.lon, fix.lat], {
          spreadM: Math.max(1.0, Math.min(fix.accuracy, 8)),
          heading: this.heading,
          headingSpreadDeg: this.heading == null ? 180 : 25,
        });
        this.metresSinceCorrection = 0;
        this._seededOutdoors = true;
      }
    }
  }

  /** One detected step: advance the filter and grow PDR uncertainty. */
  step({ stepLengthM, headingRad }) {
    if (headingRad != null) this.heading = headingRad;
    if (!this.filter.initialized) return;
    // The PDR accuracy is the trusted radius the filter confines its cloud to
    // (particles never live farther than this from their own mean).
    this.filter.predict(stepLengthM, headingRad ?? this.heading ?? 0,
      { accuracyM: this.pdrAccuracy() });
    this.metresSinceCorrection += stepLengthM;
  }

  /**
   * Forget the current GPS fix (and its held copy). Used when the user says
   * "I'm Here" because the fix is wrong: without this the stale fix stays a
   * candidate - or is held through a source switch - and wins the
   * arbitration again seconds after the anchor. Normal GPS/PDR arbitration
   * resumes when a new fix arrives.
   */
  clearGps() {
    this.gpsFix = null;
    this.heldGps = null;
    if (this.source === 'gps') {
      this.source = null;
      this._candidate = null;
      this._candidateSince = null;
    }
  }

  /** A manual anchor (map tap): re-seed and treat as a correction. */
  anchor([lon, lat], { spreadM = 2.5 } = {}) {
    this.filter.seed([lon, lat], { spreadM, heading: this.heading, headingSpreadDeg: 30 });
    this.metresSinceCorrection = 0;
    // Keep the trusted radius at the seed's spread so the anchor is not crushed
    // on the first step (then it grows with walked distance).
    this.setAccuracyFloor(spreadM * 2);
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
    return Math.max(this.accuracyFloorM,
      this.pdrBaseAccM + this.pdrDriftPerM * this.metresSinceCorrection);
  }

  /**
   * Set the trusted-radius floor after a re-seed. Call with ~2x the seed
   * spread so the freshly seeded cloud (roughly gaussian, so a hard radius at
   * 1x spread would discard most of it) is not crushed on the first step.
   */
  setAccuracyFloor(m) {
    if (m > this.accuracyFloorM) this.accuracyFloorM = m;
    return this;
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
    // Global (no-georeference) mode ignores GPS entirely: the estimate is the
    // PDR cloud's own mean, with nothing to seed or override it.
    const g = this.gpsCandidate(t);
    if (g) this.heldGps = { ...g, t };
    const pdr = this.filter.initialized ? this.filter.estimate() : null;
    const pdrAcc = this.pdrAccuracy();

    // Entering a building hand over to dead reckoning IMMEDIATELY: an indoor
    // fix (lands on the bound floor's walkable area) or any implausible fix is
    // poison outdoors-grade arbitration cares about, so it must not hold on
    // through the hysteresis window or win on a fake small accuracy.
    // OUTDOORS the opposite holds: PDR is not tied to the floor plan at all
    // (skybridge, gap between buildings, unmapped walkway all leave the
    // raster), so it drives the estimate - GPS only overrides it with a very
    // good fix (<= gpsOverrideAccuracyM).
    const raw = this.gpsFix;
    const rawFresh = raw && t - raw.t <= this.gpsStalenessS * 1000;
    const handOver = !!(rawFresh && pdr && (this.insideBuilding(raw) || !this.isPlausible(raw)));

    if (handOver) {
      this.source = 'pdr';
      this._candidate = null;
      this._candidateSince = null;
    } else {
      let winner = 'pdr';
      if (g && !pdr) winner = 'gps';                       // nothing to dead-reckon from yet
      else if (g && g.accuracy <= this.gpsOverrideAccuracyM) winner = 'gps';
      else if (!pdr && !g) return null;

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
