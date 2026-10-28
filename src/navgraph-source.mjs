/**
 * Shared extraction of the inputs to a floor navGraph.
 *
 * The app, the import-tool bundle generator, and the test harness must all
 * derive the same wall/door geometry from a floor JSON. Keeping this in one
 * place prevents a generated bundle from silently describing a different graph
 * than the fallback on-device build.
 */
import { shapeSegments, isWallLayer, isDoorLayer } from './campus.mjs';

function fnv(text, seed = 2166136261) {
  let h = seed;
  for (let i = 0; i < text.length; i++) {
    h ^= text.charCodeAt(i);
    h = Math.imul(h, 16777619);
  }
  return h >>> 0;
}

const num = (v) => (Math.round(Number(v) * 1e9) / 1e9).toFixed(9);

/**
 * Deterministic hash of the geometry that feeds `buildGraphState()`.
 * Placement changes already change `lines`/`doors` (they are lon/lat), so
 * this catches floor JSON, wall and door changes without hashing the raw
 * file (dashed-linetype refreshes must not force a rebuild).
 */
export function graphInputHash({ lines = [], doors = [] } = {}) {
  const parts = ['L', String(lines.length)];
  for (const line of lines) {
    parts.push(String(line.length));
    for (const [lon, lat] of line) parts.push(num(lon), num(lat));
  }
  parts.push('D', String(doors.length));
  for (const [lon, lat] of doors) parts.push(num(lon), num(lat));
  return fnv(parts.join(';')).toString(36);
}

/** Hash of one building's per-floor hashes, in floor order. */
export function graphFingerprint(floors) {
  const parts = [...(floors ?? [])]
    .sort((a, b) => String(a.floorId).localeCompare(String(b.floorId)))
    .map((f) => `${f.floorId}:${f.hash ?? ''}`);
  return fnv(parts.join('|')).toString(36);
}

/** Lon/lat bbox of wall lines, or null. */
export function linesBBox(lines) {
  let minLon = Infinity, minLat = Infinity, maxLon = -Infinity, maxLat = -Infinity;
  for (const line of lines ?? []) {
    for (const [lon, lat] of line) {
      if (!Number.isFinite(lon) || !Number.isFinite(lat)) continue;
      if (lon < minLon) minLon = lon;
      if (lon > maxLon) maxLon = lon;
      if (lat < minLat) minLat = lat;
      if (lat > maxLat) maxLat = lat;
    }
  }
  return Number.isFinite(minLon) ? [minLon, minLat, maxLon, maxLat] : null;
}

/**
 * @param {object} buildingConfig - buildings-config.json building entry
 * @param {object} floorConfig - that building's floor entry
 * @param {object} data - parsed floor JSON
 * @param {(x:number,y:number)=>number[]} toLL - DXF -> lon/lat for this floor
 * @returns {{lines: number[][][], doors: number[][]}}
 */
export function navGraphInputs(buildingConfig, floorConfig, data, toLL) {
  const shapes = data?.shapes ?? [];
  const walls = shapes.filter((s) => isWallLayer(s.layer));
  const lines = walls.flatMap((wall) =>
    shapeSegments(wall).map(([a, b]) => [toLL(a[0], a[1]), toLL(b[0], b[1])]));

  const doors = shapes
    .filter((s) => isDoorLayer(s.layer))
    .map((s) => {
      const leaf = (s.items ?? []).find((it) => it.op === 'line' && it.points?.length >= 2);
      if (leaf) {
        const a = leaf.points[0];
        const b = leaf.points[leaf.points.length - 1];
        return toLL((a[0] + b[0]) / 2, (a[1] + b[1]) / 2);
      }
      if (s.bbox) return toLL((s.bbox[0] + s.bbox[2]) / 2, (s.bbox[1] + s.bbox[3]) / 2);
      return null;
    })
    .filter(Boolean);

  return { lines, doors };
}
