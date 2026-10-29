/**
 * Indoor walkability from a two-region flood fill (import tool / bundle build).
 *
 * The navGraph raster treats every free cell inside the wall bbox as walkable,
 * so courtyards, gaps and the surroundings become part of the indoor graph.
 * This pass computes an indoor mask *before* navGraph generation:
 *
 *   - flood the outside from a seed in the ring outside the wall bbox;
 *   - seal the floor's doorways (and any manual seals / neighbour walls) so the
 *     flood cannot enter through an opening;
 *   - everything free inside the bbox that the outdoor flood cannot reach is
 *     indoor (plus the doorway thresholds).
 *
 * The frame is the same as `navGraph.mjs` (origin = min lon/lat of the walls,
 * scale = metresPerDegree(origin.lat)), so the returned mask samples 1:1 into
 * `buildGraphState(..., { mask })`.
 */

import { metresPerDegree } from './geo.mjs';

const DIRS = [[1, 0], [-1, 0], [0, 1], [0, -1]];
const WALL_HALF_M = 0.06;    // thin wall raster
const DOOR_SEAL_M = 0.5;     // flood seal band around a doorway barrier
const DOOR_KEEP_M = 0.4;     // threshold cells kept walkable

/**
 * @param {object} opts
 * @param {number[][][]} opts.wallsLL - this floor's wall lines (lon/lat)
 * @param {number[][][]} [opts.extraWallsLL] - neighbour walls (same level) + manual seals
 * @param {Array<{barrier?:number[][], ll?:number[]}>} [opts.doorways]
 * @param {number[][][]} [opts.seals] - extra seal segments (lon/lat)
 * @param {number[]} [opts.outdoorSeed] - override (lon/lat)
 * @param {number[]} [opts.indoorSeed] - override/validation label (lon/lat)
 * @param {number} [opts.cell=0.1]
 * @param {number} [opts.padM=0.5]
 * @returns {object} mask `{x0,y0,cell,W,H,data}` plus stats; or `{error}`
 */
export function floodIndoor({
  wallsLL, extraWallsLL = [], doorways = [], seals = [], sealPoints = [],
  outdoorSeed = null, indoorSeed = null, cell = 0.1, padM = 0.5,
} = {}) {
  const t0 = performance.now();
  if (!wallsLL?.length) return { error: 'floor has no walls' };

  // --- local metre frame (identical to navGraph.mjs) -----------------------
  let minLon = Infinity, maxLon = -Infinity, minLat = Infinity, maxLat = -Infinity;
  for (const line of wallsLL) for (const [lon, lat] of line) {
    if (lon < minLon) minLon = lon; if (lon > maxLon) maxLon = lon;
    if (lat < minLat) minLat = lat; if (lat > maxLat) maxLat = lat;
  }
  const origin = { lon: minLon, lat: minLat };
  const scale = metresPerDegree(origin.lat);
  const toM = ([lon, lat]) => [(lon - origin.lon) * scale.lon, (lat - origin.lat) * scale.lat];
  const toLL = ([x, y]) => [origin.lon + x / scale.lon, origin.lat + y / scale.lat];

  let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
  for (const line of wallsLL) for (const [x, y] of line.map(toM)) {
    if (x < minX) minX = x; if (x > maxX) maxX = x;
    if (y < minY) minY = y; if (y > maxY) maxY = y;
  }
  const gridPadM = padM + 1.5; // extra outside margin (2.0 m) so the flood can
                               // go around exit seals and stay contiguous
  const x0 = minX - gridPadM, y0 = minY - gridPadM;
  const x1 = maxX + gridPadM, y1 = maxY + gridPadM;
  const W = Math.max(1, Math.ceil((x1 - x0) / cell));
  const H = Math.max(1, Math.ceil((y1 - y0) / cell));
  const N = W * H;
  const blocked = new Uint8Array(N);
  const wall = new Uint8Array(N);     // real walls only (mask/preview)

  const markSeg = (a, b, half, target = blocked) => {
    const dx = b[0] - a[0], dy = b[1] - a[1];
    const len = Math.hypot(dx, dy);
    const steps = Math.max(1, Math.ceil(len / (cell * 0.25)));
    const rad = Math.max(0, Math.ceil(half / cell));
    for (let s = 0; s <= steps; s++) {
      const t = s / steps;
      const x = a[0] + dx * t, y = a[1] + dy * t;
      const ci = Math.floor((x - x0) / cell), cj = Math.floor((y - y0) / cell);
      for (let dj = -rad; dj <= rad; dj++) for (let di = -rad; di <= rad; di++) {
        const i = ci + di, j = cj + dj;
        if (i < 0 || j < 0 || i >= W || j >= H) continue;
        const px = x0 + (i + 0.5) * cell, py = y0 + (j + 0.5) * cell;
        if (Math.hypot(px - x, py - y) <= half + cell * 0.5) target[j * W + i] = 1;
      }
    }
  };
  const markCircle = (c, radius) => {
    const rad = Math.max(1, Math.ceil(radius / cell));
    const ci = Math.floor((c[0] - x0) / cell), cj = Math.floor((c[1] - y0) / cell);
    for (let dj = -rad; dj <= rad; dj++) for (let di = -rad; di <= rad; di++) {
      const i = ci + di, j = cj + dj;
      if (i < 0 || j < 0 || i >= W || j >= H) continue;
      const px = x0 + (i + 0.5) * cell, py = y0 + (j + 0.5) * cell;
      if (Math.hypot(px - c[0], py - c[1]) <= radius) blocked[j * W + i] = 1;
    }
  };

  const markLine = (line, half, target = blocked) => {
    for (let i = 1; i < line.length; i++) markSeg(toM(line[i - 1]), toM(line[i]), half, target);
  };
  for (const line of wallsLL) markLine(line, WALL_HALF_M, wall);
  for (const line of extraWallsLL) markLine(line, WALL_HALF_M, wall);
  blocked.set(wall);
  for (const s of seals) markSeg(toM(s[0]), toM(s[1]), WALL_HALF_M);
  // Exterior doors / skybridges: a 1.2 m disc seals the opening even when the
  // DXF leaf is drawn beside it. The grid margin (2.0 m) leaves room outside
  // the disc, so the outdoor band stays contiguous around the building.
  for (const [ll, radius = 1.2] of sealPoints) markCircle(toM(ll), radius);
  for (const d of doorways) {
    if (d.barrier) markSeg(toM(d.barrier[0]), toM(d.barrier[1]), DOOR_SEAL_M);
    // The DXF leaf/barrier can sit beside the actual opening; a disc around
    // the doorway centre covers the gap whatever the offset.
    if (d.ll) markCircle(toM(d.ll), Math.max(DOOR_SEAL_M, (d.width ?? 0.9) / 2 + 0.3));
  }

  // --- distance transform (distance to walls/seals) ------------------------
  const DT = new Int32Array(N).fill(-1);
  const q = [];
  for (let k = 0; k < N; k++) if (blocked[k]) { DT[k] = 0; q.push(k); }
  for (let head = 0; head < q.length; head++) {
    const k = q[head], i = k % W, j = (k - i) / W;
    for (const [di, dj] of DIRS) {
      const ni = i + di, nj = j + dj;
      if (ni < 0 || nj < 0 || ni >= W || nj >= H) continue;
      const n = nj * W + ni;
      if (DT[n] !== -1) continue;
      DT[n] = DT[k] + 1;
      q.push(n);
    }
  }

  const cellAt = (ll) => {
    const [x, y] = toM(ll);
    let i = Math.floor((x - x0) / cell), j = Math.floor((y - y0) / cell);
    if (i >= 0 && j >= 0 && i < W && j < H && !blocked[j * W + i]) return j * W + i;
    const i0 = i, j0 = j;
    for (let rad = 1; rad <= 40; rad++) {
      for (let dj = -rad; dj <= rad; dj++) for (let di = -rad; di <= rad; di++) {
        if (Math.max(Math.abs(di), Math.abs(dj)) !== rad) continue;
        const ni = i0 + di, nj = j0 + dj;
        if (ni < 0 || nj < 0 || ni >= W || nj >= H) continue;
        if (!blocked[nj * W + ni]) return nj * W + ni;
      }
    }
    return -1;
  };
  const llOf = (k) => {
    const i = k % W, j = (k - i) / W;
    return toLL([x0 + (i + 0.5) * cell, y0 + (j + 0.5) * cell]);
  };

  // --- outdoor seed: 0.5 m ring outside the building bbox ------------------
  // The grid has an extra margin beyond the ring so the outdoor flood is a
  // contiguous band around the building, not a clipped 0.5 m frame.
  const isRing = (k) => {
    const i = k % W, j = (k - i) / W;
    const x = x0 + (i + 0.5) * cell, y = y0 + (j + 0.5) * cell;
    return x < minX || x > maxX || y < minY || y > maxY;
  };
  const isSeedRing = (k) => {
    const i = k % W, j = (k - i) / W;
    const x = x0 + (i + 0.5) * cell, y = y0 + (j + 0.5) * cell;
    return (x < minX || x > maxX || y < minY || y > maxY) &&
      x >= minX - padM && x <= maxX + padM &&
      y >= minY - padM && y <= maxY + padM;
  };
  const isOuterMargin = (k) => {
    const i = k % W, j = (k - i) / W;
    const x = x0 + (i + 0.5) * cell, y = y0 + (j + 0.5) * cell;
    return x < minX - padM || x > maxX + padM ||
      y < minY - padM || y > maxY + padM;
  };
  let outSeed = outdoorSeed ? cellAt(outdoorSeed) : -1;
  if (outSeed < 0) {
    // Prefer the outer margin: it is outside the exit-seal discs, so the
    // flood can travel around the building instead of being cut off in the
    // 0.5 m seed ring.
    let best = -1, bestDT = -1;
    for (let k = 0; k < N; k++) {
      if (blocked[k] || !isOuterMargin(k)) continue;
      if (DT[k] > bestDT) { bestDT = DT[k]; best = k; }
    }
    if (best < 0) {
      for (let k = 0; k < N; k++) {
        if (blocked[k] || !isSeedRing(k)) continue;
        if (DT[k] > bestDT) { bestDT = DT[k]; best = k; }
      }
    }
    if (best < 0) {   // ring blocked by neighbour walls: any free outside cell
      for (let k = 0; k < N; k++) if (!blocked[k] && isRing(k)) { best = k; break; }
    }
    outSeed = best;
  }
  if (outSeed < 0) return { error: 'no free outdoor cell (bbox fully blocked)' };

  // --- outdoor flood -------------------------------------------------------
  const outdoor = new Uint8Array(N);
  {
    const qq = [outSeed];
    outdoor[outSeed] = 1;
    for (let head = 0; head < qq.length; head++) {
      const k = qq[head], i = k % W, j = (k - i) / W;
      for (const [di, dj] of DIRS) {
        const ni = i + di, nj = j + dj;
        if (ni < 0 || nj < 0 || ni >= W || nj >= H) continue;
        const n = nj * W + ni;
        if (outdoor[n] || blocked[n]) continue;
        outdoor[n] = 1;
        qq.push(n);
      }
    }
  }
  // The whole ring is outdoor regardless of whether one seed reached it
  // (neighbour walls can split the ring).
  for (let k = 0; k < N; k++) if (!blocked[k] && isRing(k)) outdoor[k] = 1;

  // --- indoor seed (auto: deepest inside, not outdoor) ---------------------
  let inSeed = indoorSeed ? cellAt(indoorSeed) : -1;
  if (inSeed < 0) {
    let best = -1, bestDT = -1;
    for (let k = 0; k < N; k++) {
      if (blocked[k] || outdoor[k] || isRing(k)) continue;
      if (DT[k] > bestDT) { bestDT = DT[k]; best = k; }
    }
    inSeed = best;
  }
  if (inSeed < 0) return { error: 'no free indoor cell (all cells are outdoor)' };
  const leak = !!outdoor[inSeed];

  // --- mask ----------------------------------------------------------------
  // Artificial seals are only for the flood: a sealed threshold is indoor, so
  // the navGraph can still cross the door. Real walls are excluded (the
  // navGraph raster blocks them independently).
  const data = new Uint8Array(N);
  for (let k = 0; k < N; k++) {
    if (wall[k] || outdoor[k] || isRing(k)) continue;
    data[k] = 1;
  }
  // Keep doorway thresholds walkable (they were sealed for the flood).
  for (const d of doorways) {
    if (!d.ll) continue;
    const [cx, cy] = toM(d.ll);
    const rad = Math.ceil(DOOR_KEEP_M / cell);
    const ci = Math.floor((cx - x0) / cell), cj = Math.floor((cy - y0) / cell);
    for (let dj = -rad; dj <= rad; dj++) for (let di = -rad; di <= rad; di++) {
      const i = ci + di, j = cj + dj;
      if (i < 0 || j < 0 || i >= W || j >= H) continue;
      const px = x0 + (i + 0.5) * cell, py = y0 + (j + 0.5) * cell;
      if (Math.hypot(px - cx, py - cy) <= DOOR_KEEP_M) data[j * W + i] = 1;
    }
  }

  let indoorCells = 0, outdoorCells = 0;
  for (let k = 0; k < N; k++) { if (data[k]) indoorCells++; if (outdoor[k]) outdoorCells++; }

  return {
    mask: { x0, y0, cell, W, H, data },
    blocked,          // preview/debug: walls + seals (flood obstacles)
    wall,             // real walls only (preview)
    outdoor,          // preview/debug: outdoor flood
    bbox: { x0, y0, x1: x0 + W * cell, y1: y0 + H * cell },  // expanded grid
    wallBBox: { minX, minY, maxX, maxY },                    // wall extents
    toLL,             // mask metre frame -> lon/lat (preview)
    outdoorSeed: llOf(outSeed),
    indoorSeed: llOf(inSeed),
    leak,
    indoorCells,
    outdoorCells,
    ms: performance.now() - t0,
  };
}
