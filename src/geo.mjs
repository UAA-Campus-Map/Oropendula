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

/** One registry entry, normalised: {origin, offset} with defaults applied. */
export function placementOf(building, floor) {
  const origin = floor.origin ?? building.origin ?? {lon: 0, lat: 0};
  const offset = floor.offset ?? building.offset ?? {x: 0, y: 0, rot: 0};
  return {origin, offset};
}
