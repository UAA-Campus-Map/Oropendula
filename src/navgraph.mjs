/**
 * Indoor pathfinding on a precomputed navigation graph.
 *
 * The expensive part — deciding where a body cannot go — happens ONCE at
 * load. Each click afterwards is a Dijkstra over a few hundred nodes rather
 * than a fresh search across a quarter of a million cells.
 *
 *   buildGraph()  1. paint a clearance bitmask (rasterise the walls)
 *                 2. merge the free cells into rectangles, one node each
 *                 3. link nodes that can see each other
 *   findPath()    4. attach both clicks to the nodes they can see
 *                 5. Dijkstra
 *
 * All geometry is done in a local metre frame derived from the data itself.
 * Mixing degrees with metre-sized lengths is the classic trap: a "0.1 m
 * cell" becomes ~1e-6 degrees wide and the grid silently collapses to
 * nothing while still reporting success.
 */

/** Body clearance from walls, metres. */
const CLEARANCE = 0.30;
/** Bitmask cell size, metres. Smaller = safer, slower to build. */
const CELL = 0.10;
/** Longest edge added to the graph, metres. Bounds build time and memory. */
const MAX_LINK = 14.0;

/** Metres per degree of longitude / latitude at a given latitude. */
function metresPerDeg(lat) {
  const r = lat * Math.PI / 180, c = k => Math.cos(k * r);
  return {
    lat: 111132.92 - 559.82 * c(2) + 1.175 * c(4) - 0.0023 * c(6),
    lon: 111412.84 * c(1) - 93.5 * c(3) + 0.118 * c(5),
  };
}

/**
 * Build the navigation graph for a set of wall centrelines.
 *
 * @param {object|object[]|number[][][]} walls - wall centrelines, as any of:
 *        a GeoJSON FeatureCollection, an array of Features, or bare
 *        [[lon,lat],[lon,lat]] arrays. Pass the raw wall *lines* - this
 *        module does its own clearance buffering, so do not pre-buffer.
 *        The DXF-to-lon/lat conversion stays in the caller.
 * @returns {{findPath: Function, nodes: number[][][], stats: object}}
 */
export function buildGraph(walls) {
  const t0 = performance.now();

  // Normalise the three shapes callers have in circulation. Guessing wrong
  // otherwise fails deep inside with an unhelpful "not iterable".
  const list = walls.features ? walls.features : walls;   // FC -> features
  const linesLL = list.map(f => (f && f.geometry ? f.geometry.coordinates : f));

  // --- pick an origin and convert everything to metres -------------------
  let minLon = Infinity, maxLon = -Infinity, minLat = Infinity, maxLat = -Infinity;
  for (const line of linesLL) for (const [lon, lat] of line) {
    if (lon < minLon) minLon = lon; if (lon > maxLon) maxLon = lon;
    if (lat < minLat) minLat = lat; if (lat > maxLat) maxLat = lat;
  }
  const origin = {lon: minLon, lat: minLat};
  const scale = metresPerDeg(origin.lat);
  const toM = ([lon, lat]) => [(lon - origin.lon) * scale.lon, (lat - origin.lat) * scale.lat];
  const toLL = ([x, y]) => [origin.lon + x / scale.lon, origin.lat + y / scale.lat];
  const segments = linesLL.map(line => line.map(toM));

  let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
  for (const s of segments) for (const [x, y] of s) {
    if (x < minX) minX = x; if (x > maxX) maxX = x;
    if (y < minY) minY = y; if (y > maxY) maxY = y;
  }

  // --- 1. rasterise the clearance zone ------------------------------------
  // Grid origin, padded by one clearance radius so walls on the edge of the
  // data still get a solid border.
  const OX = minX - CLEARANCE - CELL, OY = minY - CLEARANCE - CELL;
  const W = Math.ceil((maxX - minX + 2 * (CLEARANCE + CELL)) / CELL) + 1;
  const H = Math.ceil((maxY - minY + 2 * (CLEARANCE + CELL)) / CELL) + 1;
  const blocked = new Uint8Array(W * H);
  const reach2 = CLEARANCE * CLEARANCE;

  for (const [[ax, ay], [bx, by]] of segments) {
    // Walls are short, so testing the cells in each wall's padded box is
    // cheap enough that no spatial index is needed at all.
    const c0 = Math.max(0, Math.floor((Math.min(ax, bx) - CLEARANCE - OX) / CELL));
    const c1 = Math.min(W - 1, Math.ceil((Math.max(ax, bx) + CLEARANCE - OX) / CELL));
    const r0 = Math.max(0, Math.floor((Math.min(ay, by) - CLEARANCE - OY) / CELL));
    const r1 = Math.min(H - 1, Math.ceil((Math.max(ay, by) + CLEARANCE - OY) / CELL));
    const vx = bx - ax, vy = by - ay, len2 = vx * vx + vy * vy;

    for (let r = r0; r <= r1; r++) {
      const wy = OY + (r + 0.5) * CELL - ay;
      for (let c = c0; c <= c1; c++) {
        const i = r * W + c;
        if (blocked[i]) continue;                       // already solid
        const wx = OX + (c + 0.5) * CELL - ax;
        // Project the cell centre onto the segment and clamp to its ends,
        // which handles walls and zero-length segments in one expression.
        let t = len2 ? (wx * vx + wy * vy) / len2 : 0;
        t = t < 0 ? 0 : t > 1 ? 1 : t;
        const dx = wx - t * vx, dy = wy - t * vy;
        if (dx * dx + dy * dy <= reach2) blocked[i] = 1;
      }
    }
  }

  /** Is the straight line a->b clear of walls? Sampled at half a cell. */
  function clearLine(a, b) {
    const dx = b[0] - a[0], dy = b[1] - a[1];
    const steps = Math.ceil(Math.sqrt(dx * dx + dy * dy) / (CELL * 0.5));
    for (let s = 1; s < steps; s++) {
      const t = s / steps;
      const c = Math.round((a[0] + dx * t - OX) / CELL - 0.5);
      const r = Math.round((a[1] + dy * t - OY) / CELL - 0.5);
      if (c < 0 || r < 0 || c >= W || r >= H || blocked[r * W + c]) return false;
    }
    return true;
  }

  // --- 2. nodes: one per merged rectangle of free space ---------------------
  // A plain fixed lattice does not work indoors: a doorway narrower than the
  // lattice pitch can contain no node at all, which silently disconnects
  // whole rooms. Merging free cells into maximal rectangles adapts to the
  // shape the free space actually has, so a doorway always becomes its own
  // (thin) rectangle with a node in the middle of it.
  const used = new Uint8Array(W * H);
  const nodes = [];
  for (let r = 0; r < H; r++) for (let c = 0; c < W; c++) {
    const i = r * W + c;
    if (blocked[i] || used[i]) continue;
    let c1 = c;
    while (c1 + 1 < W && !blocked[r*W + c1 + 1] && !used[r*W + c1 + 1]) c1++;
    let r1 = r;
    grow: while (r1 + 1 < H) {
      for (let cc = c; cc <= c1; cc++) {
        const j = (r1 + 1) * W + cc;
        if (blocked[j] || used[j]) break grow;
      }
      r1++;
    }
    for (let rr = r; rr <= r1; rr++) for (let cc = c; cc <= c1; cc++) used[rr*W + cc] = 1;
    nodes.push([OX + ((c + c1 + 1) / 2) * CELL, OY + ((r + r1 + 1) / 2) * CELL]);
  }

  // --- 3. edges between mutually visible nodes ----------------------------
  const adj = nodes.map(() => []);
  const link2 = MAX_LINK * MAX_LINK;
  for (let i = 0; i < nodes.length; i++) {
    for (let j = i + 1; j < nodes.length; j++) {
      const dx = nodes[j][0] - nodes[i][0], dy = nodes[j][1] - nodes[i][1];
      const d2 = dx * dx + dy * dy;
      if (d2 > link2 || !clearLine(nodes[i], nodes[j])) continue;
      const d = Math.sqrt(d2);
      adj[i].push([j, d]);
      adj[j].push([i, d]);
    }
  }

  // The bitmask must have no holes or the whole graph is fiction. Cheap to
  // assert: every point along every wall centreline has to be solid.
  let missed = 0, sampled = 0;
  for (const [[ax, ay], [bx, by]] of segments) for (let u = 0; u <= 1; u += 0.25) {
    const px = ax + (bx - ax) * u, py = ay + (by - ay) * u;
    const c = Math.round((px - OX) / CELL - 0.5), r = Math.round((py - OY) / CELL - 0.5);
    sampled++;
    if (!(c >= 0 && r >= 0 && c < W && r < H && blocked[r * W + c])) missed++;
  }

  // --- 4 & 5. attach the clicks, then Dijkstra ----------------------------
  /**
   * @param {number[]} startLL [lon,lat]
   * @param {number[]} endLL   [lon,lat]
   * @returns {number[][]|null} path as [[lon,lat],...], or null if unreachable
   */
  function findPath(startLL, endLL) {
    const p0 = toM(startLL), p1 = toM(endLL);
    const n = nodes.length;

    // A click is not a node, so treat it as one and link it to every node it
    // can actually see. Skipping this test is the one omission that silently
    // produces paths straight through walls.
    const from = [];
    const seesGoal = new Uint8Array(n), goalCost = new Float64Array(n);
    for (let i = 0; i < n; i++) {
      const d0 = Math.hypot(nodes[i][0] - p0[0], nodes[i][1] - p0[1]);
      if (clearLine(p0, nodes[i])) from.push([i, d0]);
      const d1 = Math.hypot(nodes[i][0] - p1[0], nodes[i][1] - p1[1]);
      if (clearLine(p1, nodes[i])) { seesGoal[i] = 1; goalCost[i] = d1; }
    }
    if (!from.length) return null;               // click buried in a wall

    const cost = new Float64Array(n).fill(Infinity);
    const prev = new Int32Array(n).fill(-1);
    const done = new Uint8Array(n);
    const open = new MinHeap();
    for (const [i, w] of from) { cost[i] = w; open.push(i, w); }

    let goal = -1;
    while (open.size) {
      const u = open.pop();
      if (done[u]) continue;
      done[u] = 1;
      if (seesGoal[u]) { goal = u; break; }     // cheapest node that sees the target
      for (const [v, w] of adj[u]) {
        const nd = cost[u] + w;
        if (nd < cost[v]) { cost[v] = nd; prev[v] = u; open.push(v, nd); }
      }
    }
    if (goal < 0) return null;

    // prev[] walks backwards from the goal, so flip it before attaching the
    // clicks - otherwise path[1] is the *goal* node and the first hop is an
    // unverified segment straight through whatever is in between.
    const chain = [];
    for (let v = goal; v >= 0; v = prev[v]) chain.push(toLL(nodes[v]));
    chain.reverse();
    return [startLL, ...chain, endLL];
  }

  const stats = {
    cells: W * H, nodes: nodes.length,
    edges: adj.reduce((a, l) => a + l.length, 0) / 2,
    blockedPercent: +(100 * blocked.reduce((a, v) => a + v, 0) / (W * H)).toFixed(1),
    buildMs: +(performance.now() - t0).toFixed(0),
    rasterHoles: `${missed}/${sampled}`,
  };
  if (missed) console.warn('navgraph: raster has holes', stats.rasterHoles);

  return {findPath, nodes, stats};;
}

/** Binary min-heap keyed on cost. */
class MinHeap {
  constructor() { this.cost = []; this.node = []; }
  get size() { return this.cost.length; }

  push(node, cost) {
    const {cost: c, node: n} = this;
    let i = c.length;
    c.push(cost); n.push(node);
    while (i > 0) {                                   // sift up
      const p = (i - 1) >> 1;
      if (c[p] <= c[i]) break;
      [c[p], c[i]] = [c[i], c[p]]; [n[p], n[i]] = [n[i], n[p]];
      i = p;
    }
  }

  pop() {
    const {cost: c, node: n} = this, top = n[0];
    const lc = c.pop(), ln = n.pop();
    if (c.length) {                                   // sift down
      c[0] = lc; n[0] = ln;
      let i = 0;
      for (;;) {
        let l = 2 * i + 1, r = l + 1, m = i;
        if (l < c.length && c[l] < c[m]) m = l;
        if (r < c.length && c[r] < c[m]) m = r;
        if (m === i) break;
        [c[m], c[i]] = [c[i], c[m]]; [n[m], n[i]] = [n[i], n[m]];
        i = m;
      }
    }
    return top;
  }
}