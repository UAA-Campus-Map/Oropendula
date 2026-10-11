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
  const origin = floor.origin ?? building.origin;
  const offset = floor.offset ?? building.offset;
  return (x, y) =>
    dxfCoordsToLonLat(origin, x - offset.x, y - offset.y, offset.rot);
}

/** One registry entry, normalised: {origin, offset} with defaults applied. */
export function placementOf(building, floor) {
  const origin = floor.origin ?? building.origin ?? {lon: 0, lat: 0};
  const offset = floor.offset ?? building.offset ?? {x: 0, y: 0, rot: 0};
  return {origin, offset};
}
