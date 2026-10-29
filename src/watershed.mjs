/**
 * Watershed room segmentation over one floor's walls (test-only).
 *
 * Pure, DOM-free. The point is to partition a floor into room/corridor
 * territories from the room labels, so we can see whether rooms really are
 * separated and where the doors (throats) are.
 *
 * Model:
 *   - rasterise the raw WALL lines (no navGraph clearance inflation) at `cell`
 *     metres, so door openings stay open;
 *   - distance transform (DT) = distance from each free cell to the nearest wall;
 *   - marker watershed: grow every seed into the highest-DT available cell
 *     first, so fronts meet at narrow throats rather than in open space;
 *   - adjacency + throat width = 2 * max DT along a shared boundary.
 *
 * Deterministic (max-heap with a cell-index tie-break); no annealing.
 */

const DIRS = [[1, 0], [-1, 0], [0, 1], [0, -1]];

/** Max-heap of (dt, cell); ties prefer the smaller cell index. */
function makeHeap() {
  const a = [];
  const better = (x, y) => x[0] > y[0] || (x[0] === y[0] && x[1] < y[1]);
  return {
    get size() { return a.length; },
    push(dt, cell) {
      a.push([dt, cell]);
      let i = a.length - 1;
      while (i > 0) {
        const p = (i - 1) >> 1;
        if (!better(a[i], a[p])) break;
        [a[p], a[i]] = [a[i], a[p]];
        i = p;
      }
    },
    pop() {
      const top = a[0];
      const last = a.pop();
      if (a.length) {
        a[0] = last;
        let i = 0;
        for (;;) {
          const l = 2 * i + 1, r = l + 1;
          let m = i;
          if (l < a.length && better(a[l], a[m])) m = l;
          if (r < a.length && better(a[r], a[m])) m = r;
          if (m === i) break;
          [a[i], a[m]] = [a[m], a[i]];
          i = m;
        }
      }
      return top;
    },
  };
}

function distToSeg(px, py, ax, ay, bx, by) {
  const dx = bx - ax, dy = by - ay;
  const l2 = dx * dx + dy * dy;
  let t = l2 ? ((px - ax) * dx + (py - ay) * dy) / l2 : 0;
  t = t < 0 ? 0 : t > 1 ? 1 : t;
  return Math.hypot(px - (ax + t * dx), py - (ay + t * dy));
}

/**
 * @param {object} opts
 * @param {number[][][]} opts.wallsLL - wall segments as [[lon,lat],[lon,lat]]
 * @param {object} opts.graph - navGraph (toM/toLL + raster bbox)
 * @param {Array<{text:string, ll:number[]}>} [opts.roomLabels]
 * @param {Array<{name:string, ll:number[]}>} [opts.backgroundSeeds]
 * @param {number} [opts.cell=0.2] - grid pitch, metres
 * @param {number} [opts.maxColors=4]
 * @returns {{W,H,cell,x0,y0,x1,y1,labels,blocked,DT,regions,adjacency,colors,colorCount,ms,seedCount}}
 */
export function watershed({
  wallsLL, graph, roomLabels = [], backgroundSeeds = [], cell = 0.2, maxColors = 4,
}) {
  const t0 = performance.now();
  const r = graph.raster;
  const x0 = r.OX, y0 = r.OY;
  const x1 = r.OX + r.W * r.cell, y1 = r.OY + r.H * r.cell;
  const W = Math.max(1, Math.ceil((x1 - x0) / cell));
  const H = Math.max(1, Math.ceil((y1 - y0) / cell));
  const N = W * H;
  const blocked = new Uint8Array(N);

  // --- rasterise the raw walls (one cell thick) -----------------------------
  const toM = graph.toM;
  for (const seg of wallsLL) {
    const [a, b] = [toM(seg[0]), toM(seg[seg.length - 1])];
    const len = Math.hypot(b[0] - a[0], b[1] - a[1]);
    const steps = Math.max(1, Math.ceil(len / (cell * 0.25)));
    for (let s = 0; s <= steps; s++) {
      const t = s / steps;
      const x = a[0] + (b[0] - a[0]) * t;
      const y = a[1] + (b[1] - a[1]) * t;
      const i = Math.floor((x - x0) / cell);
      const j = Math.floor((y1 - y) / cell);
      if (i >= 0 && j >= 0 && i < W && j < H) blocked[j * W + i] = 1;
    }
  }

  // --- distance transform (BFS from wall cells) -----------------------------
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

  // --- seeds ----------------------------------------------------------------
  const cellOf = (ll) => {
    const [x, y] = toM(ll);
    let i = Math.floor((x - x0) / cell), j = Math.floor((y1 - y) / cell);
    if (i >= 0 && j >= 0 && i < W && j < H && !blocked[j * W + i]) return j * W + i;
    // spiral out to the nearest free cell
    const i0 = i, j0 = j;
    for (let rad = 1; rad <= 20; rad++) {
      for (let dj = -rad; dj <= rad; dj++) for (let di = -rad; di <= rad; di++) {
        if (Math.max(Math.abs(di), Math.abs(dj)) !== rad) continue;
        const ni = i0 + di, nj = j0 + dj;
        if (ni < 0 || nj < 0 || ni >= W || nj >= H) continue;
        if (!blocked[nj * W + ni]) return nj * W + ni;
      }
    }
    return -1;
  };

  const seeds = [];
  for (const l of roomLabels) seeds.push({ id: seeds.length + 1, name: l.text, kind: 'room', ll: l.ll });
  const bgSeeds = [...backgroundSeeds];
  if (!bgSeeds.length) {
    // No connector/explicit corridor seed on this floor: use the widest open
    // cell so the corridor/background region still exists.
    let best = -1, bestDT = -1;
    for (let k = 0; k < N; k++) if (!blocked[k] && DT[k] > bestDT) { bestDT = DT[k]; best = k; }
    if (best >= 0) {
      const i = best % W, j = (best - i) / W;
      bgSeeds.push({ name: 'corridor', ll: graph.toLL([x0 + (i + 0.5) * cell, y1 - (j + 0.5) * cell]) });
    }
  }
  for (const b of bgSeeds) seeds.push({ id: seeds.length + 1, name: b.name, kind: 'background', ll: b.ll });
  const labels = new Int32Array(N);
  const regions = new Map();
  const seedCells = [];
  const heap = makeHeap();
  for (const s of seeds) {
    const c = cellOf(s.ll);
    if (c < 0 || labels[c]) continue;
    labels[c] = s.id;
    regions.set(s.id, { id: s.id, name: s.name, kind: s.kind, area: 0, color: 0 });
    seedCells.push({ id: s.id, cell: c, ll: s.ll, kind: s.kind, name: s.name });
    heap.push(DT[c], c);
  }
  const seedCount = regions.size;

  // --- marker watershed (highest DT first) ----------------------------------
  while (heap.size) {
    const [, k] = heap.pop();
    const i = k % W, j = (k - i) / W;
    const id = labels[k];
    for (const [di, dj] of DIRS) {
      const ni = i + di, nj = j + dj;
      if (ni < 0 || nj < 0 || ni >= W || nj >= H) continue;
      const n = nj * W + ni;
      if (blocked[n] || labels[n]) continue;
      labels[n] = id;
      heap.push(DT[n], n);
    }
  }

  // --- region stats + adjacency/throats -------------------------------------
  for (let k = 0; k < N; k++) {
    const id = labels[k];
    if (id > 0) regions.get(id).area++;
  }
  const adjacency = new Map();   // "a|b#n" -> {a,b,maxDT,boundaryCell,widthM}
  const boundary = new Uint8Array(N);
  const byPair = new Map();      // "a|b" -> Set(boundary cell)
  const pair = (a, b, k) => {
    if (!a || !b || a === b) return;
    const lo = Math.min(a, b), hi = Math.max(a, b);
    const key = lo + '|' + hi;
    let cells = byPair.get(key);
    if (!cells) { cells = new Set(); byPair.set(key, cells); }
    cells.add(k);
  };
  for (let j = 0; j < H; j++) for (let i = 0; i < W; i++) {
    const k = j * W + i;
    if (blocked[k]) continue;
    const id = labels[k];
    if (i + 1 < W) { const n = k + 1; if (!blocked[n] && labels[n] !== id) { boundary[k] = 1; pair(id, labels[n], k); } }
    if (j + 1 < H) { const n = k + W; if (!blocked[n] && labels[n] !== id) { boundary[k] = 1; pair(id, labels[n], k); } }
  }
  // Split each region pair's boundary into connected components: two separate
  // openings between the same two regions are two doors, not one.
  let edgeSeq = 0;
  for (const [key, cells] of byPair) {
    const [a, b] = key.split('|').map(Number);
    const seen = new Set();
    for (const start of cells) {
      if (seen.has(start)) continue;
      const comp = [];
      const q = [start];
      seen.add(start);
      while (q.length) {
        const k = q.pop();
        comp.push(k);
        const i = k % W, j = (k - i) / W;
        for (const [di, dj] of DIRS) {
          const ni = i + di, nj = j + dj;
          if (ni < 0 || nj < 0 || ni >= W || nj >= H) continue;
          const n = nj * W + ni;
          if (cells.has(n) && !seen.has(n)) { seen.add(n); q.push(n); }
        }
      }
      let best = comp[0];
      for (const k of comp) if (DT[k] > DT[best]) best = k;
      adjacency.set(`${key}#${edgeSeq++}`, {
        a, b, maxDT: DT[best], boundaryCell: best, widthM: DT[best] * 2 * cell,
      });
    }
  }

  // --- greedy colouring (4 by default) --------------------------------------
  const deg = new Map();
  for (const id of regions.keys()) deg.set(id, 0);
  for (const e of adjacency.values()) { deg.set(e.a, deg.get(e.a) + 1); deg.set(e.b, deg.get(e.b) + 1); }
  const order = [...regions.keys()].sort((a, b) => deg.get(b) - deg.get(a) || a - b);
  const nbr = new Map();
  for (const id of regions.keys()) nbr.set(id, []);
  for (const e of adjacency.values()) { nbr.get(e.a).push(e.b); nbr.get(e.b).push(e.a); }
  let colorCount = 0;
  for (const id of order) {
    const used = new Set(nbr.get(id).map((n) => regions.get(n).color));
    let c = 1;
    while (used.has(c)) c++;
    regions.get(id).color = c;
    if (c > colorCount) colorCount = c;
  }
  if (colorCount < maxColors) colorCount = maxColors;

  return {
    W, H, cell, x0, y0, x1, y1, labels, blocked, DT, boundary,
    regions, adjacency, colorCount, seedCount, seedCells,
    ms: performance.now() - t0,
  };
}

/** 4-colour palette (RGB), muted so the plan shows through. */
export const REGION_COLORS = [
  [59, 130, 246],    // blue
  [239, 68, 68],     // red
  [34, 197, 94],     // green
  [234, 179, 8],     // amber
  [168, 85, 247],    // purple (only if >4 forced)
  [20, 184, 166],    // teal
];
export const BACKGROUND_COLOR = [148, 163, 184];   // grey
