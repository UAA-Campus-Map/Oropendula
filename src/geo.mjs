/**
 * Shared geo registration math for DXF floor plans.
 *
 * A floor is placed by (origin, offset):
 *   origin {lon, lat}  - lon/lat of the DXF datum point below
 *   offset {x, y, rot} - the datum point in DXF units (inches) and a
 *                        rotation (radians) about it
 *
 * makeToLL(building, floor) applies both, with the floor's values winning
 * when present (AS106 is registered at floor level). This is the single
 * source of truth: index.html, smoke.mjs and import.html all import from
 * here so the app and the import tool can never disagree.
 */

/** DXF units are inches; metres per inch. */
export const DXF_SCALE = 0.0254;

/** Metres per degree of lon/lat at a latitude (USGS Snyder series). */
export function metresPerDegree(lat) {
  const r = (lat * Math.PI) / 180;
  const c = (k) => Math.cos(k * r);
  return {
    lat: 111132.92 - 559.82 * c(2) + 1.175 * c(4) - 0.0023 * c(6),
    lon: 111412.84 * c(1) - 93.5 * c(3) + 0.118 * c(5),
  };
}

/**
 * DXF (inches) -> [lon, lat], rotated about the DXF origin by `rot` radians.
 * @param {{lon:number,lat:number}} origin - lon/lat of DXF (0,0)
 * @param {number} scale - metres per DXF unit
 */
export function dxfCoordsToLonLat(origin, x, y, rot, scale = DXF_SCALE) {
  const mPerDeg = metresPerDegree(origin.lat);
  const east = x * scale * Math.cos(rot) - y * scale * Math.sin(rot);
  const north = x * scale * Math.sin(rot) + y * scale * Math.cos(rot);
  const lat = origin.lat + north / mPerDeg.lat;
  const lon = origin.lon + east / mPerDeg.lon;
  return [lon, lat];
}

/** Inverse of dxfCoordsToLonLat: [lon,lat] -> DXF (inches). */
export function lonLatToDxf(origin, lon, lat, rot, scale = DXF_SCALE) {
  const mPerDeg = metresPerDegree(origin.lat);
  const east = (lon - origin.lon) * mPerDeg.lon;
  const north = (lat - origin.lat) * mPerDeg.lat;
  const c = Math.cos(-rot), s = Math.sin(-rot);
  return [
    c * east - s * north,
    s * east + c * north,
  ].map((v) => v / scale);
}

/**
 * The toLL transform for one building+floor: a floor may carry its own
 * origin/offset (AS106 does), otherwise the building-level ones apply.
 * @param {object} building - a buildingsConfig entry
 * @param {object} floor - one of its floors entries
 * @returns {(x:number,y:number)=>[number,number]}
 */
export function makeToLL(building, floor) {
  // Match placementOf(): a building created before it was placed has no
  // origin/offset yet. Defaulting here keeps a render from throwing (which
  // used to leave a newly imported building's plan blank).
  const origin = floor.origin ?? building.origin ?? { lon: 0, lat: 0 };
  const offset = floor.offset ?? building.offset ?? { x: 0, y: 0, rot: 0 };
  return (x, y) =>
    dxfCoordsToLonLat(origin, x - offset.x, y - offset.y, offset.rot);
}

/**
 * True when two 2-D segments properly intersect (including endpoints).
 * Points must be in the same coordinate system; callers convert lon/lat to
 * a local metre frame first when accuracy matters.
 */
export function segmentsIntersect(a, b, c, d) {
  const rx = b[0] - a[0], ry = b[1] - a[1];
  const sx = d[0] - c[0], sy = d[1] - c[1];
  const denom = rx * sy - ry * sx;
  if (Math.abs(denom) < 1e-12) return false;   // parallel/collinear: no crossing
  const qpx = c[0] - a[0], qpy = c[1] - a[1];
  const t = (qpx * sy - qpy * sx) / denom;
  const u = (qpx * ry - qpy * rx) / denom;
  return t >= 0 && t <= 1 && u >= 0 && u <= 1;
}

/** True when any segment of a polyline crosses the segment a->b. */
export function polylineCrossesSegment(polyline, a, b) {
  for (let i = 1; i < polyline.length; i++) {
    if (segmentsIntersect(polyline[i - 1], polyline[i], a, b)) return true;
  }
  return false;
}

/**
 * First place a polyline crosses a segment, in route order (all metres).
 *
 * Doorways store the wall gap as their barrier, but the navGraph passage is
 * carved at the DOOR leaf midpoint, so on real plans a route often threads the
 * opening a few decimetres to the side of the barrier (measured ~0.3-0.5 m on
 * SU-02). A strict in-segment intersection therefore misses such doors, so
 * `toleranceM` extends the barrier at both ends before intersecting.
 *
 * @returns {{at:number, point:number[], dir:number[], u:number}|null}
 *   `at` is arc length from the polyline start; `dir` the forward unit
 *   direction at the crossing; `u` the barrier parameter (may be outside [0,1]
 *   within the tolerance).
 */
export function crossBarrier(polylineM, a, b, toleranceM = 0.4) {
  const rx = b[0] - a[0], ry = b[1] - a[1];
  const bl = Math.hypot(rx, ry);
  if (!(bl > 0)) return null;
  const ex = toleranceM / bl;   // barrier parameter domain [-ex, 1+ex]
  let walked = 0;
  for (let i = 1; i < polylineM.length; i++) {
    const p0 = polylineM[i - 1], p1 = polylineM[i];
    const dx = p1[0] - p0[0], dy = p1[1] - p0[1];
    const segLen = Math.hypot(dx, dy);
    const denom = dx * ry - dy * rx;
    if (Math.abs(denom) >= 1e-12) {
      const qpx = a[0] - p0[0], qpy = a[1] - p0[1];
      const t = (qpx * ry - qpy * rx) / denom;   // along the polyline
      const u = (qpx * dy - qpy * dx) / denom;   // along the barrier
      if (t >= 0 && t <= 1 && u >= -ex && u <= 1 + ex) {
        const l = segLen || 1;
        return {
          at: walked + t * segLen,
          point: [p0[0] + dx * t, p0[1] + dy * t],
          dir: [dx / l, dy / l],
          u,
        };
      }
    }
    walked += segLen;
  }
  return null;
}

/** Closest point on segment s0->s1 to point p (metres). */
function closestOnSegment(p, s0, s1) {
  const dx = s1[0] - s0[0], dy = s1[1] - s0[1];
  const l2 = dx * dx + dy * dy;
  let t = l2 ? ((p[0] - s0[0]) * dx + (p[1] - s0[1]) * dy) / l2 : 0;
  t = t < 0 ? 0 : t > 1 ? 1 : t;
  return {t, d: Math.hypot(p[0] - (s0[0] + t * dx), p[1] - (s0[1] + t * dy))};
}

/**
 * Closest approach of a polyline to a segment, in route order (metres).
 *
 * The navGraph passage is carved at the DOOR leaf midpoint while the barrier is
 * the wall gap, so a route can thread the opening a few decimetres *beside* the
 * barrier without crossing its line (`crossBarrier` then misses). This reports
 * the point of nearest approach so the caller can treat "passed through the
 * opening" as a crossing.
 *
 * @returns {{at:number, point:number[], dir:number[], dist:number}|null}
 */
export function nearestApproach(polylineM, a, b) {
  if (!polylineM || polylineM.length < 2) return null;
  let best = null, walked = 0;
  for (let i = 1; i < polylineM.length; i++) {
    const p0 = polylineM[i - 1], p1 = polylineM[i];
    const dx = p1[0] - p0[0], dy = p1[1] - p0[1];
    const len = Math.hypot(dx, dy), l = len || 1;
    const cands = [
      closestOnSegment(a, p0, p1),
      closestOnSegment(b, p0, p1),
      {t: 0, d: closestOnSegment(p0, a, b).d},
      {t: 1, d: closestOnSegment(p1, a, b).d},
    ];
    for (const c of cands) {
      if (!best || c.d < best.dist) {
        best = {
          dist: c.d,
          at: walked + c.t * len,
          point: [p0[0] + dx * c.t, p0[1] + dy * c.t],
          dir: [dx / l, dy / l],
        };
      }
    }
    walked += len;
  }
  return best;
}

/** One registry entry, normalised: {origin, offset} with defaults applied. */
export function placementOf(building, floor) {
  const origin = floor.origin ?? building.origin ?? {lon: 0, lat: 0};
  const offset = floor.offset ?? building.offset ?? {x: 0, y: 0, rot: 0};
  return {origin, offset};
}
