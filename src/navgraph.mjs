/**
 * Indoor pathfinding on a precomputed navigation graph.
 *
 * The expensive part — deciding where a body cannot go — happens ONCE at
 * load. Each click afterwards is a Dijkstra over a few hundred nodes rather
 * than a fresh search across a quarter of a million cells.
 *
 * Algorithm Overview:
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

import { buildPassages, makeHeadingField } from './headingfield.mjs';
import { segmentsIntersect } from './geo.mjs';

/** Body clearance from walls, metres. */
const CLEARANCE = 0.30;
/** Bitmask cell size, metres. Smaller = safer, slower to build. */
const CELL = 0.10;
/** Longest edge added to the graph, metres. Bounds build time and memory. */
const MAX_LINK = 14.0;

/** 
 * Metres per degree of longitude / latitude at a given latitude.
 * 
 * This is essential to avoid the trap of treating degrees as metres.
 * The Earth is not flat - latitude affects the length of a degree.
 * At a given latitude:
 * - 1 degree latitude ≈ 111 km (varies slightly due to ellipsoid)
 * - 1 degree longitude ≈ 111 km * cos(latitude)
 * 
 * This function returns scaling factors to convert geographic coordinates
 * (lon, lat in degrees) to local Cartesian coordinates (x, y in metres)
 * for easier distance calculations and grid operations.
 * 
 * @param {number} lat - The reference latitude in degrees
 * @returns {{lat: number, lon: number}} Metres per degree for lat and lon
 */
function metresPerDeg(lat) {
  const r = lat * Math.PI / 180, c = k => Math.cos(k * r);
  return {
    lat: 111132.92 - 559.82 * c(2) + 1.175 * c(4) - 0.0023 * c(6), // latitude scaling
    lon: 111412.84 * c(1) - 93.5 * c(3) + 0.118 * c(5),            // longitude scaling
  };
}

/**
 * Build the navigation graph for a set of wall centrelines.
 *
 * This function precomputes an indoor pathfinding graph by:
 * 1. Converting geographic walls to local metre coordinates
 * 2. Creating a 2D grid bitmask with wall clearance zones
 * 3. Clustering free space into rectangle nodes (doorways/rooms adaptively)
 * 4. Connecting mutually visible nodes with edges (max link distance limited)
 *
 * Returns only the serializable state (raster, nodes, adjacency, statistics);
 * call graphFromState() to get the live graph with its query closures. This
 * split lets the graph be built in a Worker and rehydrated on the main thread.
 *
 * @param {object|object[]|number[][][]} walls - wall centrelines, as any of:
 *        a GeoJSON FeatureCollection, an array of Features, or bare
 *        [[lon,lat],[lon,lat]] arrays. Pass the raw wall *lines* - this
 *        module does its own clearance buffering, so do not pre-buffer.
 *        The DXF-to-lon/lat conversion stays in the caller.
 * @param {number[][]} [doors] - doorway midpoints in lon/lat.
 * @param {{passages?: boolean}} [opts] - set `passages: false` when the
 *        serialised bundle does not need the heading-field passages. Defaults
 *        to true, preserving the existing app/worker behaviour.
 * @returns {{findPath: Function, nodes: number[][][], stats: object,
 *            isWalkable: Function, nearestWalkable: Function}}
 *          - findPath: Function to compute path between two (lon,lat) points
 *          - nodes: Array of [x,y] node positions in local metres (for debugging/visualization)
 *          - stats: Object with build statistics
 *          - isWalkable: O(1) test of whether a (lon,lat) is body-clear
 *          - nearestWalkable: closest body-clear (lon,lat), or null
 */
/**
 * Line-of-sight test over a clearance raster: true when the straight run a->b
 * crosses no blocked cell. Sampling is at half-cell intervals. Shared by
 * buildGraphState (node linking) and graphFromState (clicks, visibility).
 */
function crossesBarrier(a, b, barriers) {
  if (!barriers || !barriers.length) return false;
  for (const [c, d] of barriers) {
    if (segmentsIntersect(a, b, c, d)) return true;
  }
  return false;
}

function makeClearLine(blocked, W, H, OX, OY) {
  return function clearLine(a, b, barriers = null) {
    if (crossesBarrier(a, b, barriers)) return false;
    const dx = b[0] - a[0], dy = b[1] - a[1];
    const dist = Math.sqrt(dx * dx + dy * dy);
    const steps = Math.ceil(dist / (CELL * 0.5)); // sample interval = half cell
    for (let s = 1; s < steps; s++) {
      const t = s / steps;
      const px = a[0] + dx * t;
      const py = a[1] + dy * t;
      // Convert sample point to grid coordinates
      const c = Math.round((px - OX) / CELL - 0.5);
      const r = Math.round((py - OY) / CELL - 0.5);
      if (c < 0 || r < 0 || c >= W || r >= H || blocked[r * W + c]) {
        return false;
      }
    }
    return true;
  };
}

export function buildGraphState(walls, doors = [], { passages: buildPassagesEnabled = true } = {}) {
  const t0 = performance.now();

  // Normalise the three shapes callers have in circulation. Guessing wrong
  // otherwise fails deep inside with an unhelpful "not iterable".
  // Handles: GeoJSON FeatureCollection, array of Features, or bare line arrays
  const list = walls.features ? walls.features : walls;   // FC -> features
  const linesLL = list.map(f => (f && f.geometry ? f.geometry.coordinates : f))
    .filter((line) => Array.isArray(line) && line.length >= 2);
  if (!linesLL.length) throw new Error('navgraph: floor has no wall segments');

  // --- pick an origin and convert everything to metres -------------------
  // Find the bounding box in degrees to establish a local coordinate system
  // Use minimum lon/lat as origin to keep coordinates small (avoid floating point issues)
  let minLon = Infinity, maxLon = -Infinity, minLat = Infinity, maxLat = -Infinity;
  for (const line of linesLL) {
    for (const [lon, lat] of line) {
      if (lon < minLon) minLon = lon; if (lon > maxLon) maxLon = lon;
      if (lat < minLat) minLat = lat; if (lat > maxLat) maxLat = lat;
    }
  }
  const origin = {lon: minLon, lat: minLat};
  const scale = metresPerDeg(origin.lat);
  
  // Conversion functions between lat/lon and local metre coordinates
  const toM = ([lon, lat]) => [(lon - origin.lon) * scale.lon, (lat - origin.lat) * scale.lat];
  const toLL = ([x, y]) => [origin.lon + x / scale.lon, origin.lat + y / scale.lat];
  
  // Convert all wall segments to local metre frame
  const segments = linesLL.map(line => line.map(toM));

  // Find bounding box in metres
  let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
  for (const s of segments) {
    for (const [x, y] of s) {
      if (x < minX) minX = x; if (x > maxX) maxX = x;
      if (y < minY) minY = y; if (y > maxY) maxY = y;
    }
  }

  // --- 1. rasterise the clearance zone ------------------------------------
  // Grid origin, padded by one clearance radius so walls on the edge of the
  // data still get a solid border. This ensures agent (body) can't "slip out"
  // between wall and grid edge.
  const OX = minX - CLEARANCE - CELL;
  const OY = minY - CLEARANCE - CELL;
  const W = Math.ceil((maxX - minX + 2 * (CLEARANCE + CELL)) / CELL) + 1;
  const H = Math.ceil((maxY - minY + 2 * (CLEARANCE + CELL)) / CELL) + 1;
  const blocked = new Uint8Array(W * H); // 1 = blocked by wall+clearance, 0 = free
  const reach2 = CLEARANCE * CLEARANCE;  // squared clearance distance for faster comparison

  // For each wall segment, mark all grid cells within CLEARANCE distance as blocked
  for (const [[ax, ay], [bx, by]] of segments) {
    // Walls are short, so testing the cells in each wall's padded box is
    // cheap enough that no spatial index is needed at all.
    // Compute the axis-aligned bounding box of the wall plus clearance
    const c0 = Math.max(0, Math.floor((Math.min(ax, bx) - CLEARANCE - OX) / CELL));
    const c1 = Math.min(W - 1, Math.ceil((Math.max(ax, bx) + CLEARANCE - OX) / CELL));
    const r0 = Math.max(0, Math.floor((Math.min(ay, by) - CLEARANCE - OY) / CELL));
    const r1 = Math.min(H - 1, Math.ceil((Math.max(ay, by) + CLEARANCE - OY) / CELL));
    const vx = bx - ax, vy = by - ay;
    const len2 = vx * vx + vy * vy;

    // Check each cell in the bounding box - is it within clearance of the segment?
    for (let r = r0; r <= r1; r++) {
      const wy = OY + (r + 0.5) * CELL - ay; // cell center relative to segment start
      for (let c = c0; c <= c1; c++) {
        const i = r * W + c;
        if (blocked[i]) continue;                       // already solid, skip
        const wx = OX + (c + 0.5) * CELL - ax;          // cell center relative to segment start
        // Project the cell centre onto the segment and clamp to its ends,
        // which handles walls and zero-length segments in one expression.
        let t = len2 ? (wx * vx + wy * vy) / len2 : 0;
        t = t < 0 ? 0 : t > 1 ? 1 : t;
        const dx = wx - t * vx, dy = wy - t * vy;
        const dist2 = dx * dx + dy * dy;
        if (dist2 <= reach2) {
          blocked[i] = 1;
        }
      }
    }
  }

  const clearLine = makeClearLine(blocked, W, H, OX, OY);

  // --- 2. nodes: one per merged rectangle of free space ---------------------
  // A plain fixed lattice does not work indoors: a doorway narrower than the
  // lattice pitch can contain no node at all, which silently disconnects
  // whole rooms. Merging free cells into maximal rectangles adapts to the
  // shape the free space actually has, so a doorway always becomes its own
  // (thin) rectangle with a node in the middle of it.
  const used = new Uint8Array(W * H);
  const nodes = [];
  for (let r = 0; r < H; r++) {
    for (let c = 0; c < W; c++) {
      const i = r * W + c;
      if (blocked[i] || used[i]) continue;
      // Find the maximal horizontal span starting from (r,c)
      let c1 = c;
      while (c1 + 1 < W && !blocked[r * W + c1 + 1] && !used[r * W + c1 + 1]) {
        c1++;
      }
      // Find maximal vertical extent for this horizontal span
      let r1 = r;
      let canGrow = true;
      while (r1 + 1 < H && canGrow) {
        // Check if the entire row (r1+1) from c to c1 is free and unused
        for (let cc = c; cc <= c1; cc++) {
          const j = (r1 + 1) * W + cc;
          if (blocked[j] || used[j]) {
            canGrow = false;
            break;
          }
        }
        if (canGrow) r1++;
      }
      // Mark all cells in this rectangle as used
      for (let rr = r; rr <= r1; rr++) {
        for (let cc = c; cc <= c1; cc++) {
          used[rr * W + cc] = 1;
        }
      }
      // Create a node at the center of the rectangle
      const nodeX = OX + ((c + c1 + 1) / 2) * CELL;
      const nodeY = OY + ((r + r1 + 1) / 2) * CELL;
      nodes.push([nodeX, nodeY]);
    }
  }

  // --- 3. edges between mutually visible nodes ----------------------------
  const adj = nodes.map(() => []); // adjacency list: adj[i] contains [j, distance]
  const link2 = MAX_LINK * MAX_LINK;
  
  // Build the visibility graph - connect all pairs of nodes that can see each other
  // within MAX_LINK distance. This is O(n² * cost of clearLine) - n is small (~hundreds)
  for (let i = 0; i < nodes.length; i++) {
    for (let j = i + 1; j < nodes.length; j++) {
      const dx = nodes[j][0] - nodes[i][0];
      const dy = nodes[j][1] - nodes[i][1];
      const d2 = dx * dx + dy * dy;
      if (d2 > link2) continue; // too far apart, skip
      if (!clearLine(nodes[i], nodes[j])) continue; // blocked by walls
      const d = Math.sqrt(d2);
      // Add undirected edge
      adj[i].push([j, d]);
      adj[j].push([i, d]);
    }
  }

  // The bitmask must have no holes or the whole graph is fiction. Cheap to
  // assert: every point along every wall centreline has to be solid.
  let missed = 0, sampled = 0;
  for (const [[ax, ay], [bx, by]] of segments) {
    for (let u = 0; u <= 1; u += 0.25) {
      const px = ax + (bx - ax) * u;
      const py = ay + (by - ay) * u;
      const c = Math.round((px - OX) / CELL - 0.5);
      const r = Math.round((py - OY) / CELL - 0.5);
      sampled++;
      if (!(c >= 0 && r >= 0 && c < W && r < H && blocked[r * W + c])) {
        missed++;
      }
    }
  }

  // Build statistics
  const blockedCount = blocked.reduce((a, v) => a + v, 0);
  const totalCells = W * H;
  const edgeCount = adj.reduce((a, l) => a + l.length, 0) / 2; // undirected edges
  const stats = {
    cells: totalCells,
    nodes: nodes.length,
    edges: edgeCount,
    blockedPercent: +(100 * blockedCount / totalCells).toFixed(1),
    buildMs: +(performance.now() - t0).toFixed(0),
    rasterHoles: `${missed}/${sampled}`,
  };
  if (missed) console.warn('navgraph: raster has holes', stats.rasterHoles);

  // --- allowed compass directions (the "heading field") -------------------
  // Every graph edge and every interior doorway becomes a bearing-bearing
  // segment. Built here so it rides the same worker pass as the graph; see
  // headingfield.mjs for how it is queried and snapped.
  const doorsM = (doors || []).map(toM);
  const passages = buildPassagesEnabled ? buildPassages(nodes, adj, doorsM, segments) : null;

  return {origin, scale, OX, OY, W, H, nodes, adj, blocked, stats, passages};
}

/**
 * Rebuild the live navigation graph (query closures included) from the
 * serializable state returned by buildGraphState. Output is exactly what
 * buildGraph() returns for the same walls.
 */
export function graphFromState({origin, scale, OX, OY, W, H, nodes, adj, blocked, stats, passages}) {
  const mLon = scale.lon, mLat = scale.lat;
  const toM = ([lon, lat]) => [(lon - origin.lon) * mLon, (lat - origin.lat) * mLat];
  const toLL = ([x, y]) => [origin.lon + x / mLon, origin.lat + y / mLat];
  const clearLine = makeClearLine(blocked, W, H, OX, OY);
  const toBarriers = (barriers) => (barriers ?? []).map(([a, b]) => [toM(a), toM(b)]);

  // --- 4 & 5. attach the clicks, then Dijkstra ----------------------------
  /**
   * Find shortest path between two geographic points using Dijkstra on the
   * precomputed navigation graph. 
   * 
   * Process:
   * 1. Convert start/end from lat/lon to local metres
   * 2. Find all graph nodes visible from start and from end (treat start as temporary node)
   * 3. Run Dijkstra from start's visible nodes until we reach any node visible from end
   * 4. Reconstruct path by backtracking through prev array
   * 5. Return path as array of [lon,lat] points (including start and end)
   * 
   * Early termination: the final hop to the goal click is a free straight line,
   * not a graph edge, so the path minimises graph cost + goal stub cost. The
   * search keeps the best total seen and stops once the popped node's graph
   * cost alone exceeds it (then no later node can improve).
   *
   * @param {number[]} startLL - [lon, lat] of start point
   * @param {number[]} endLL - [lon, lat] of end point
   * @returns {number[][]|null} Path as [[lon,lat],...], or null if unreachable
   */
  function findPath(startLL, endLL, { barriers = [] } = {}) {
    if (!isWalkable(startLL) || !isWalkable(endLL)) return null;
    const bsegs = toBarriers(barriers);
    const p0 = toM(startLL); // start in metres
    const p1 = toM(endLL);   // end in metres
    const n = nodes.length;

    // A click is not a node, so treat it as one and link it to every node it
    // can actually see. Skipping this test is the one omission that silently
    // produces paths straight through walls.
    const from = []; // list of [nodeIndex, edgeCost] for nodes visible from start
    const seesGoal = new Uint8Array(n);
    const goalCost = new Float64Array(n);
    
    // Check visibility from start and to end for all nodes
    for (let i = 0; i < n; i++) {
      // Check if node i is visible from start
      const d0 = Math.hypot(nodes[i][0] - p0[0], nodes[i][1] - p0[1]);
      if (clearLine(p0, nodes[i], bsegs)) {
        from.push([i, d0]);
      }
      // Check if node i is visible from end (goal)
      const d1 = Math.hypot(nodes[i][0] - p1[0], nodes[i][1] - p1[1]);
      if (clearLine(p1, nodes[i], bsegs)) {
        seesGoal[i] = 1;
        goalCost[i] = d1;
      }
    }
    if (!from.length) return null;               // click buried in a wall

    // Dijkstra's algorithm
    const cost = new Float64Array(n).fill(Infinity); // shortest path cost to each node
    const prev = new Int32Array(n).fill(-1);         // previous node in shortest path
    const done = new Uint8Array(n);                  // visited flag
    const open = new MinHeap();                      // priority queue (min-heap by cost)

    // Initialize with start's visible nodes
    for (const [i, w] of from) {
      cost[i] = w;
      open.push(i, w);
    }

    let goal = -1;
    let bestTotal = Infinity;
    while (open.size) {
      const u = open.pop();
      if (done[u]) continue;
      done[u] = 1;
      // The last hop to the goal click is a free straight line (not a graph
      // edge), so the first goal-visible node popped is NOT necessarily the
      // best total: keep the minimum of graph cost + stub cost, and stop once
      // no remaining node can beat it (the heap pops in increasing cost order).
      if (seesGoal[u]) {
        const total = cost[u] + goalCost[u];
        if (total < bestTotal) { bestTotal = total; goal = u; }
      }
      if (cost[u] >= bestTotal) break;
      // Relax adjacent edges
      for (const [v, w] of adj[u]) {
        if (crossesBarrier(nodes[u], nodes[v], bsegs)) continue;
        const nd = cost[u] + w;
        if (nd < cost[v]) {
          cost[v] = nd;
          prev[v] = u;
          open.push(v, nd);
        }
      }
    }
    if (goal < 0) return null;

    // prev[] walks backwards from the goal, so flip it before attaching the
    // clicks - otherwise path[1] is the *goal* node and the first hop is an
    // unverified segment straight through whatever is in between.
    const chain = [];
    for (let v = goal; v >= 0; v = prev[v]) {
      chain.push(toLL(nodes[v]));
    }
    chain.reverse();
    // Return path: start -> ... intermediate nodes ... -> end
    return [startLL, ...chain, endLL];
  }

  /**
   * The nodes visible from a lon/lat point, with straight-line distances.
   *
   * This is the attachment half of findPath() exposed on its own: a caller
   * that needs one point -> many targets can attach once and run ONE Dijkstra
   * (dijkstraFromAttach) instead of a full findPath() per target - which is
   * what makes the campus router's exit-to-exit distance matrix O(k) searches
   * rather than O(k^2). The visibility test is exactly findPath's (no distance
   * cutoff), so results are identical to the old per-pair findPath().
   *
   * @param {number[]} ll - [lon,lat]
   * @returns {{node:number, dist:number}[]}
   */
  function attach(ll, { barriers = [] } = {}) {
    if (!isWalkable(ll)) return [];
    const bsegs = toBarriers(barriers);
    const p = toM(ll);
    const out = [];
    for (let i = 0; i < nodes.length; i++) {
      const d = Math.hypot(nodes[i][0] - p[0], nodes[i][1] - p[1]);
      if (clearLine(p, nodes[i], bsegs)) out.push({node: i, dist: d});
    }
    return out;
  }

  /**
   * Dijkstra from an attachment set (see attach). Each source node starts at
   * its stub distance, so dist[b] is the whole point -> node b distance.
   * @param {{node:number, dist:number}[]} attached
   * @returns {{dist:Float64Array, prev:Int32Array}}
   */
  function dijkstraFromAttach(attached, { barriers = [] } = {}) {
    const bsegs = toBarriers(barriers);
    const n = nodes.length;
    const dist = new Float64Array(n).fill(Infinity);
    const prev = new Int32Array(n).fill(-1);
    const done = new Uint8Array(n);
    const open = new MinHeap();
    for (const {node, dist: w} of attached) {
      if (w < dist[node]) { dist[node] = w; open.push(node, w); }
    }
    while (open.size) {
      const u = open.pop();
      if (done[u]) continue;
      done[u] = 1;
      const du = dist[u];
      for (const [v, w] of adj[u]) {
        if (crossesBarrier(nodes[u], nodes[v], bsegs)) continue;
        const nd = du + w;
        if (nd < dist[v]) { dist[v] = nd; prev[v] = u; open.push(v, nd); }
      }
    }
    return {dist, prev};
  }

  /**
   * Cheapest whole distance from the point behind `run` to a target
   * attachment, and the target node that achieves it.
   */
  function distToAttach(run, attached) {
    let best = Infinity, node = -1;
    for (const {node: i, dist} of attached) {
      const t = run.dist[i] + dist;
      if (t < best) { best = t; node = i; }
    }
    return {dist: best, node};
  }

  /**
   * Graph-node path (lon/lat) from the point behind `run` to the nearest node
   * of a target attachment, or null. The caller adds its own click endpoints.
   */
  function pathToAttach(run, attached) {
    const {dist, node} = distToAttach(run, attached);
    if (!isFinite(dist) || node < 0) return null;
    const chain = [];
    for (let v = node; v !== -1; v = run.prev[v]) chain.push(toLL(nodes[v]));
    return chain.reverse();
  }

  /**
   * Is this (lon,lat) inside the body-clear area?
   *
   * Pure lookup into the clearance raster findPath() already builds, so the
   * two can never disagree. This is the accessor the PDR particle filter uses
   * to weight particles (and the GPS-validity gate uses to reject fixes that
   * land inside a wall) without paying for a Dijkstra per query.
   */
  function isWalkable(ll) {
    const [x, y] = toM(ll);
    const c = Math.floor((x - OX) / CELL);
    const r = Math.floor((y - OY) / CELL);
    if (c < 0 || c >= W || r < 0 || r >= H) return false;
    return blocked[r * W + c] === 0;
  }

  /**
   * Is this (lon,lat) inside the raster, optionally within `marginM` metres of
   * it? The raster is bounded by the wall bbox (padded by one clearance), and
   * `isWalkable` reports false for everything outside it. That outside is NOT a
   * wall - it is simply unmapped - so the PDR particle filter uses this to tell
   * "blocked by a wall" apart from "past the edge of the plan". A margin lets
   * the cloud spill a bounded distance past the edge (no pile-up on the
   * invisible bbox line) without dead-reckoning away into the void.
   */
  function inBounds(ll, marginM = 0) {
    const [x, y] = toM(ll);
    const c = Math.floor((x - OX) / CELL);
    const r = Math.floor((y - OY) / CELL);
    const m = marginM > 0 ? Math.ceil(marginM / CELL) : 0;
    return c >= -m && c < W + m && r >= -m && r < H + m;
  }

  /**
   * Nearest body-clear (lon,lat) to `ll`, found by an outward spiral search in
   * whole cells (the centre of the first free cell wins). Returns null if
   * nothing free is found within `maxRadius` cells.
   */
  function nearestWalkable(ll, maxRadius = 60) {
    const [x, y] = toM(ll);
    const c0 = Math.floor((x - OX) / CELL);
    const r0 = Math.floor((y - OY) / CELL);
    for (let rad = 0; rad <= maxRadius; rad++) {
      for (let dr = -rad; dr <= rad; dr++) {
        for (let dc = -rad; dc <= rad; dc++) {
          if (Math.max(Math.abs(dr), Math.abs(dc)) !== rad) continue; // ring only
          const c = c0 + dc, r = r0 + dr;
          if (c < 0 || c >= W || r < 0 || r >= H) continue;
          if (blocked[r * W + c] === 0) {
            return toLL([OX + (c + 0.5) * CELL, OY + (r + 0.5) * CELL]);
          }
        }
      }
    }
    return null;
  }

  // Allowed compass directions (see headingfield.mjs). Rehydrated here as a
  // spatial index over the serialised passages so queries stay cheap.
  const headingField = passages && passages.length
    ? makeHeadingField(passages, { toM })
    : null;

  return {findPath, nodes, stats, isWalkable, nearestWalkable, inBounds,
          attach, dijkstraFromAttach, distToAttach, pathToAttach, headingField,
          // Read-only raster access for room/space analysis (import tool/app).
          raster: {blocked, W, H, OX, OY, cell: CELL}, toM, toLL};
}

/** Build the navigation graph for a set of wall centrelines (see buildGraphState). */
export function buildGraph(walls, doors = []) {
  return graphFromState(buildGraphState(walls, doors));
}

class MinHeap {
  constructor() {
    this.cost = [];
    this.node = [];
  }
  
  get size() {
    return this.cost.length;
  }

  push(node, cost) {
    const {cost: c, node: n} = this;
    let i = c.length;
    c.push(cost);
    n.push(node);
    while (i > 0) {
      // sift up
      const p = Math.floor((i - 1) / 2);
      if (c[p] <= c[i]) break;
      // Swap
      [c[p], c[i]] = [c[i], c[p]];
      [n[p], n[i]] = [n[i], n[p]];
      i = p;
    }
  }

  pop() {
    const {cost: c, node: n} = this;
    const top = n[0];
    const lc = c.pop();
    const ln = n.pop();
    if (c.length) {
      // sift down
      c[0] = lc;
      n[0] = ln;
      let i = 0;
      for (;;) {
        let l = 2 * i + 1;
        let r = l + 1;
        let m = i;
        if (l < c.length && c[l] < c[m]) m = l;
        if (r < c.length && c[r] < c[m]) m = r;
        if (m === i) break;
        // Swap
        [c[m], c[i]] = [c[i], c[m]];
        [n[m], n[i]] = [n[i], n[m]];
        i = m;
      }
    }
    return top;
  }
}
