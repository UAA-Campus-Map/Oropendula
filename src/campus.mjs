/**
 * Tiered campus router: multi-building, multi-floor wayfinding.
 *
 * navgraph.mjs provides one visibility graph per floor. This module adds the
 * tiers above it:
 *
 *   1. indoor leg      start -> vertical connector on the start floor
 *   2. vertical leg    elevator / stairs down to the ground floor
 *   3. indoor leg      connector -> building exit
 *   4. outdoor leg     exit -> entrance of the next building (follows roads)
 *      ... any number of intermediate buildings ...
 *   5. mirror of 1-3 into the destination building and floor
 *
 * The chain of buildings is chosen by Dijkstra over a small "macro graph"
 * whose nodes are building exits: same-building edges are indoor ground-floor
 * path lengths, cross-building edges are outdoor path lengths. That yields
 * the optimal chain of any length - the recursive lookup for "the fastest
 * path might go through another building".
 *
 * Portals (elevators, stairs, doors) are extracted from tagged DXF layers.
 * Layer names vary in case between buildings, so matching is case-insensitive:
 *   elevators: ELEVATOR / elevator
 *   stairs:    STAIR / STAIRS / stairs / Stairs
 *   doors:     DOORS / Doors
 * Door *segments* are clustered into doorways. A doorway is an exit when a
 * coarse flood fill of the free space around the building - with every drawn
 * door opening sealed, so the fill cannot slip indoors - shows the outdoors
 * reachable through that doorway.
 *
 * Outdoor legs walk an OpenStreetMap path network when one is supplied: a
 * PATHS layer of shapes in the same format as the DXF files, translated from
 * OSM by osm.mjs. The graph is real OSM ways only - edges that would cut
 * through a building (stale ways, tile-buffer stitching) are dropped against
 * the buildings' wall segments - plus very short stitches re-joining halves
 * of a way that a tile edge clipped apart. Exits and outdoor clicks attach at
 * the nearest point along the network (splitting an edge when it beats the
 * nearest vertex), and an outdoor-to-outdoor trip also offers the direct
 * network walk, not just a chain through building entrances. Without a path
 * network the router falls back to straight-line outdoor legs.
 *
 * All routing costs are time in seconds. Distances are metres.
 */

/** Walking speeds, metres per second. */
const INDOOR_SPEED = 1.3;
const OUTDOOR_SPEED = 1.4;
/**
 * Outdoor legs are heavily penalised for ROUTE SELECTION (never for the
 * reported time): Fairbanks hits -40C, where an outdoor walk is impractical, so
 * the router should take a skybridge (or any indoor detour) over going outside
 * whenever one exists. The multiplier only inflates the Dijkstra cost of
 * outdoor edges; segment seconds are always recomputed from the real speed, so
 * ETAs stay honest. Outdoor is still used when it is the only option.
 */
const OUTDOOR_COST_MULT = 15;
/** Elevator cost model. */
const ELEVATOR_WAIT_S = 15;
const ELEVATOR_PER_FLOOR_S = 4;
/** Stairs cost model. */
const STAIRS_PER_FLOOR_S = 6;
/** DXF shapes are inches; metres per DXF unit. */
const M_PER_DXF_UNIT = 0.0254;
/** Clustering radius for portal shapes, metres. */
const CLUSTER_RADIUS_M = 2.0;
/** Door clustering radius - one cluster should be one physical door, metres. */
const DOOR_CLUSTER_RADIUS_M = 0.8;
/** Door swing-arc boxes are ~1.8m across; drop anything this big or larger
 *  from door clustering so arcs can't drag the cluster centre off the leaf. */
const DOOR_ARC_MAX_M = 1.5;
/** Coarse flood fill used for the exterior-door test. The grid, block radius
 *  and margin are tuned so that gaps narrower than ~1.9m - i.e. every ordinary
 *  interior doorway - stay sealed and only genuinely outdoor space floods. */
const FLOOD_CELL_M = 0.5;         // grid resolution
const FLOOD_WALL_M = 0.45;        // block radius around wall segments
const FLOOD_SEAL_M = 0.35;        // how far past a door leaf the seal reaches
const FLOOD_MARGIN_M = 4;         // outdoor seed band around the footprint
const FLOOD_LEAD_CELLS = 7;       // ~3.5m allowed from a door to the outdoors
/** An exit's leaf must sit this close to the building outline, metres. */
const OUTLINE_MAX_M = 1.0;
/** Max distance between same-connector stops on different floors, metres. */
const CONNECTOR_LINK_M = 3.0;
/**
 * Max distance between two skybridge portals that may pair, metres. Bridges
 * are explicit manual portals, and a real one can be long (an ~80 m enclosed
 * link exists), so this is only a sanity cap - pairing itself is mutual
 * nearest, see buildCampusRouter.
 */
const SKYBRIDGE_MAX_M = 120.0;
/** Retry-ring radius when a portal point lands in a wall, metres. */
const RETRY_RADIUS_M = 1.2;

/** Metres per degree of lon/lat at a latitude (same series as navgraph.mjs). */
function metresPerDeg(lat) {
  const r = lat * Math.PI / 180, c = k => Math.cos(k * r);
  return {
    lat: 111132.92 - 559.82 * c(2) + 1.175 * c(4) - 0.0023 * c(6),
    lon: 111412.84 * c(1) - 93.5 * c(3) + 0.118 * c(5),
  };
}

/** Great-circle-ish distance in metres between two [lon,lat] points. */
function distLL(a, b, scale) {
  const dx = (a[0] - b[0]) * scale.lon, dy = (a[1] - b[1]) * scale.lat;
  return Math.hypot(dx, dy);
}

/** Length in metres of a [[lon,lat],...] polyline. */
function polyLength(p, scale) {
  let d = 0;
  for (let i = 1; i < p.length; i++) d += distLL(p[i - 1], p[i], scale);
  return d;
}

// ---------------------------------------------------------------------------
// Portal extraction
// ---------------------------------------------------------------------------

const PORTAL_LAYERS = {
  elevator: new Set(['elevator']),
  stairs: new Set(['stair', 'stairs']),
  doors: new Set(['door', 'doors']),
};

/**
 * Wall/window layers, singular or plural, any case. Plans are inconsistent:
 * AS106 writes WINDOWS, while HSB (AS156) uses WALL/window on floor 01 and
 * Walls/WINDOWS on floor 02. The app, import tool and tests share this so a
 * floor can never render in one and be blank in the other.
 */
export function isWallLayer(layer) {
  const l = String(layer ?? '').toLowerCase();
  return l === 'wall' || l === 'walls' || l === 'window' || l === 'windows';
}

/** Door-leaf layers, singular or plural, any case (DOOR / DOORS). */
export function isDoorLayer(layer) {
  const l = String(layer ?? '').toLowerCase();
  return l === 'door' || l === 'doors';
}

/**
 * Union-find clustering of shapes by expanded-bbox overlap.
 * Two shapes join when their bboxes come within `thresh` DXF units.
 * @returns {number[][]} clusters as lists of shape indices
 */
function clusterShapes(shapes, thresh) {
  const n = shapes.length;
  const parent = Array.from({length: n}, (_, i) => i);
  const find = i => { while (parent[i] !== i) { parent[i] = parent[parent[i]]; i = parent[i]; } return i; };
  const boxes = shapes.map(s => {
    const [x0, y0, x1, y1] = s.bbox;
    const h = thresh / 2;
    return [x0 - h, y0 - h, x1 + h, y1 + h];
  });
  for (let i = 0; i < n; i++) {
    for (let j = i + 1; j < n; j++) {
      const a = boxes[i], b = boxes[j];
      if (a[0] <= b[2] && b[0] <= a[2] && a[1] <= b[3] && b[1] <= a[3]) {
        parent[find(i)] = find(j);
      }
    }
  }
  const groups = new Map();
  for (let i = 0; i < n; i++) {
    const r = find(i);
    if (!groups.has(r)) groups.set(r, []);
    groups.get(r).push(i);
  }
  return [...groups.values()];
}

/** Point-to-segment distance in DXF units. */
function distToSegU(px, py, x0, y0, x1, y1) {
  const dx = x1 - x0, dy = y1 - y0;
  const len2 = dx * dx + dy * dy;
  let t = len2 ? ((px - x0) * dx + (py - y0) * dy) / len2 : 0;
  t = t < 0 ? 0 : t > 1 ? 1 : t;
  const ddx = px - (x0 + t * dx), ddy = py - (y0 + t * dy);
  return Math.sqrt(ddx * ddx + ddy * ddy);
}

/**
 * True geometry of a DXF shape as line segments, in DXF units.
 *
 * The extractor keeps every shape's real vertices in `items`; a shape's bbox
 * is only its extent, so using the bbox diagonal as the geometry silently
 * corrupts every slanted wall and every multi-vertex polyline - a rotated
 * wall run comes out as a diagonal slash across the corner it really turns,
 * and a "missing" stretch of wall is often just a rotated one drawn wrong.
 * Arc items carry no vertices and are skipped (flattening was rejected).
 *
 * @param {object} shape - one entry of a building JSON's `shapes` array
 * @returns {[number,number][][]} [[x,y],[x,y]] segment pairs, [] if none
 */
export function shapeSegments(shape) {
  const segs = [];
  for (const item of shape.items || []) {
    const pts = item.points;
    if (!pts || pts.length < 2) continue;
    for (let i = 1; i < pts.length; i++) segs.push([pts[i - 1], pts[i]]);
    if (shape.closed && pts.length > 2) segs.push([pts[pts.length - 1], pts[0]]);
  }
  return segs;
}

/**
 * Extract portals from raw DXF shapes.
 *
 * @param {object[]} shapes - the `shapes` array of a building JSON
 * @param {(x:number,y:number)=>number[]} toLL - DXF -> [lon,lat] transform,
 *        must be the SAME registration used for the walls of this floor
 * @returns {{elevators:object[], stairs:object[], exits:object[]}}
 *          each portal: {id, ll:[lon,lat]}
 */
export function extractPortals(shapes, toLL) {
  const isLayer = (s, names) => names.has(s.layer.toLowerCase());
  // Window layers are part of the floor's wall graph in the viewer, so they
  // count toward the clearance check too.
  const CLEAR_NAMES = new Set(['wall', 'walls', 'window', 'windows']);
  // Extra solid layers that enclose space without being in the navgraph
  // (AS102 draws part of its envelope on 'Panels').
  const FLOOD_NAMES = new Set(['wall', 'walls', 'window', 'windows', 'panels', 'partition', 'partitions']);
  const clearSegs = shapes
    .filter(s => isLayer(s, CLEAR_NAMES))
    .flatMap(shapeSegments);
  const floodSegs = shapes
    .filter(s => isLayer(s, FLOOD_NAMES))
    .flatMap(shapeSegments);
  const doorShapes = shapes.filter(s => isDoorLayer(s.layer));
  if (floodSegs.length === 0) return {elevators: [], stairs: [], exits: []};

  // A "doorway" is the group of door shapes at one physical opening: the
  // leaf/leaves (dozens of tiny polyline segments each) plus the swing arcs
  // hanging off them. Leaves are clustered together; arcs are NOT (an arc
  // spans ~1.8m and would chain neighbouring doors - including the outer and
  // inner rows of an arctic entry - into one meaningless group), instead each
  // arc is attached to the doorway whose leaves it roots at. Each doorway
  // seals its own opening, and its exterior test may pass through its own
  // seal - arcs included - but never through any other doorway's.
  const doorIsLeaf = s => {
    const w = Math.abs(s.bbox[2] - s.bbox[0]);
    const h = Math.abs(s.bbox[3] - s.bbox[1]);
    return Math.hypot(w, h) * M_PER_DXF_UNIT <= DOOR_ARC_MAX_M;
  };
  const bboxOf = list => {
    let x0 = Infinity, y0 = Infinity, x1 = -Infinity, y1 = -Infinity;
    for (const s of list) {
      const [a, b, c, d] = s.bbox;
      x0 = Math.min(x0, a, c); y0 = Math.min(y0, b, d);
      x1 = Math.max(x1, a, c); y1 = Math.max(y1, b, d);
    }
    return [x0, y0, x1, y1];
  };
  const bboxGap = (a, b) => {
    const dx = Math.max(a[0] - b[2], b[0] - a[2], 0);
    const dy = Math.max(a[1] - b[3], b[1] - a[3], 0);
    return Math.hypot(dx, dy);
  };
  const doorLeaves = doorShapes.filter(doorIsLeaf);
  const doorArcs = doorShapes.filter(s => !doorIsLeaf(s));
  const doorways = [];
  for (const idxs of clusterShapes(doorLeaves, DOOR_CLUSTER_RADIUS_M / M_PER_DXF_UNIT)) {
    doorways.push({leaves: idxs.map(i => doorLeaves[i]), arcs: []});
  }
  if (doorways.length) {
    const boxes = doorways.map(w => bboxOf(w.leaves));
    for (const s of doorArcs) {
      let best = 0, bestD = Infinity;
      for (let g = 0; g < boxes.length; g++) {
        const d = bboxGap(bboxOf([s]), boxes[g]);
        if (d < bestD) { bestD = d; best = g; }
      }
      doorways[best].arcs.push(s);
    }
  } else {
    for (const s of doorArcs) doorways.push({leaves: [], arcs: [s]});
  }

  // --- coarse flood fill: which space connects to the outdoors -------------
  // Cell size and block radius are chosen so gaps narrower than ~1.9m - i.e.
  // every ordinary interior doorway - stay sealed; only genuinely outdoor
  // space floods.
  const CELL = FLOOD_CELL_M / M_PER_DXF_UNIT;
  const wallR = FLOOD_WALL_M / M_PER_DXF_UNIT;
  const sealR = FLOOD_SEAL_M / M_PER_DXF_UNIT;
  const marchin = FLOOD_MARGIN_M / M_PER_DXF_UNIT;
  let fx0 = Infinity, fy0 = Infinity, fx1 = -Infinity, fy1 = -Infinity;
  for (const [[x0, y0], [x1, y1]] of floodSegs) {
    fx0 = Math.min(fx0, x0, x1); fx1 = Math.max(fx1, x0, x1);
    fy0 = Math.min(fy0, y0, y1); fy1 = Math.max(fy1, y0, y1);
  }
  const gW = Math.ceil((fx1 - fx0 + 2 * marchin) / CELL) + 1;
  const gH = Math.ceil((fy1 - fy0 + 2 * marchin) / CELL) + 1;
  const orgx = fx0 - marchin, orgy = fy0 - marchin;
  const idxAt = (c, r) => c * gH + r;
  const col = x => Math.floor((x - orgx) / CELL);
  const row = y => Math.floor((y - orgy) / CELL);
  const inGridCell = (c, r) => c >= 0 && r >= 0 && c < gW && r < gH;
  const blocked = new Uint8Array(gW * gH);
  // Cells a wall's continuous run blocks (as opposed to the halo its END
  // throws), plus the run's direction: a doorway's seal may punch through a
  // run that continues its own wall, but not through one that crosses it -
  // AS102's room-113 door stands ~0.15m from a window wall, and sealing
  // through that run let the door's exterior test walk out through the glass.
  const wallRun = new Uint8Array(gW * gH);
  const wallRunDir = new Float32Array(gW * gH);
  // Which doorway sealed each cell (-1 = not door-sealed). A door's exterior
  // test may walk through its own seal; every other door's seal still blocks.
  const sealOwner = new Int32Array(gW * gH).fill(-1);

  // Walls: walk each segment, block cells whose centre is within wallR.
  const rad = Math.ceil(wallR / CELL);
  for (const [[x0, y0], [x1, y1]] of floodSegs) {
    const dx = x1 - x0, dy = y1 - y0;
    const len2 = dx * dx + dy * dy;
    const len = Math.hypot(dx, dy);
    if (len < 1e-9) continue;
    const ang = Math.atan2(dy, dx);
    const steps = Math.max(1, Math.ceil(len / (CELL * 0.4)));
    for (let i = 0; i <= steps; i++) {
      const t = i / steps;
      const c = col(x0 + dx * t), r = row(y0 + dy * t);
      for (let dc = -rad; dc <= rad; dc++) for (let dr = -rad; dr <= rad; dr++) {
        const cc = c + dc, rr = r + dr;
        if (!inGridCell(cc, rr)) continue;
        const cxu = orgx + (cc + 0.5) * CELL, cyu = orgy + (rr + 0.5) * CELL;
        if (distToSegU(cxu, cyu, x0, y0, x1, y1) > wallR) continue;
        const i = idxAt(cc, rr);
        blocked[i] = 1;
        // The cell's projection along the segment: strictly inside the span
        // means the cell sits on the wall's run, not just near its end.
        if (len2 > 0) {
          const tRaw = ((cxu - x0) * dx + (cyu - y0) * dy) / len2;
          if (tRaw > 0 && tRaw < 1) {
            wallRun[i] = 1;
            if (wallRunDir[i] === 0) wallRunDir[i] = ang;
          }
        }
      }
    }
  }
  // Door openings: seal them, remembering which doorway each sealed cell
  // belongs to, otherwise the fill slips indoors. The seal must override the
  // halo a wall's END throws across the doorway's own opening - that is what
  // makes the opening passable at all - but must NOT cut through a wall or
  // window that merely runs past the door. So a run-blocked cell may only be
  // sealed when the run continues the doorway's own wall: either the door's
  // drawn geometry touches the cell, or the run is parallel to the wall the
  // doorway sits in. AS102's room-113 door meets its own (east-west) wall at
  // the jambs but stands ~0.15m from a north-south window wall - sealing
  // through that run let the door's exterior test walk out through the glass.
  const RUN_EPS = 0.1 / M_PER_DXF_UNIT;    // DXF units
  const PARALLEL_TOL = Math.PI / 6;        // 30 degrees
  const arcFlatten = item => {
    const [cx, cy] = item.center;
    const r = item.radius;
    if (!(r > 0) || item.start_angle === undefined || item.end_angle === undefined) return [];
    let span = (item.end_angle - item.start_angle) % 360;
    if (span <= 0) span += 360;
    const n = Math.max(2, Math.ceil(span / 30));
    const pts = [];
    for (let i = 0; i <= n; i++) {
      const t = (item.start_angle + span * i / n) * Math.PI / 180;
      pts.push([cx + r * Math.cos(t), cy + r * Math.sin(t)]);
    }
    return pts;
  };
  const doorwaySegs = g => {
    const out = [];
    for (const s of [...doorways[g].leaves, ...doorways[g].arcs]) {
      for (const it of s.items || []) {
        const pts = it.points && it.points.length >= 2 ? it.points : arcFlatten(it);
        for (let i = 1; i < pts.length; i++) out.push([pts[i - 1], pts[i]]);
      }
    }
    return out;
  };
  for (let g = 0; g < doorways.length; g++) {
    const own = doorwaySegs(g);
    // Direction of the wall this doorway sits in: the wall segment closest
    // to the doorway's own drawn geometry.
    let wallAng = null, wallD = Infinity;
    for (const [[x0, y0], [x1, y1]] of own) {
      for (const [[wx0, wy0], [wx1, wy1]] of floodSegs) {
        const d = Math.min(distToSegU(x0, y0, wx0, wy0, wx1, wy1),
                           distToSegU(x1, y1, wx0, wy0, wx1, wy1));
        if (d < wallD) { wallD = d; wallAng = Math.atan2(wy1 - wy0, wx1 - wx0); }
      }
    }
    if (wallD > 1.2 / M_PER_DXF_UNIT) wallAng = null;   // no wall found: allow all
    const canSealRun = (px, py, cellAng) => {
      if (wallAng !== null) {
        let diff = Math.abs(cellAng - wallAng) % Math.PI;   // runs are undirected
        if (diff <= PARALLEL_TOL) return true;
      }
      for (const [[x0, y0], [x1, y1]] of own) {
        if (distToSegU(px, py, x0, y0, x1, y1) <= RUN_EPS) return true;
      }
      return false;
    };
    for (const s of [...doorways[g].leaves, ...doorways[g].arcs]) {
      const [a0, b0, a1, b1] = s.bbox;
      const c0 = Math.max(0, col(Math.min(a0, a1) - sealR));
      const c1 = Math.min(gW - 1, col(Math.max(a0, a1) + sealR));
      const r0 = Math.max(0, row(Math.min(b0, b1) - sealR));
      const r1 = Math.min(gH - 1, row(Math.max(b0, b1) + sealR));
      for (let c = c0; c <= c1; c++) for (let r = r0; r <= r1; r++) {
        const i = idxAt(c, r);
        if (wallRun[i] &&
            !canSealRun(orgx + (c + 0.5) * CELL, orgy + (r + 0.5) * CELL, wallRunDir[i])) continue;
        blocked[i] = 1;
        sealOwner[i] = g;
      }
    }
  }
  // Flood outward from the border.
  const seen = new Uint8Array(gW * gH);
  const stack = [];
  const seed = i => { if (!blocked[i] && !seen[i]) { seen[i] = 1; stack.push(i); } };
  for (let c = 0; c < gW; c++) { seed(idxAt(c, 0)); seed(idxAt(c, gH - 1)); }
  for (let r = 0; r < gH; r++) { seed(idxAt(0, r)); seed(idxAt(gW - 1, r)); }
  while (stack.length) {
    const i = stack.pop();
    const c = Math.floor(i / gH), r = i % gH;
    for (const [dc, dr] of [[1, 0], [-1, 0], [0, 1], [0, -1]]) {
      const cc = c + dc, rr = r + dr;
      if (!inGridCell(cc, rr)) continue;
      const ni = idxAt(cc, rr);
      if (blocked[ni] || seen[ni]) continue;
      seen[ni] = 1;
      stack.push(ni);
    }
  }
  const LAYER_KIND = {elevators: 'elevator', stairs: 'stairs', exits: 'doors'};
  const byKind = {elevators: [], stairs: [], exits: []};
  const kindFor = layer => {
    const l = layer.toLowerCase();
    for (const [kind, names] of Object.entries(PORTAL_LAYERS)) {
      if (names.has(l)) return kind;
    }
    return null;
  };

  // A stop point must sit at least the graph's clearance (0.30m) away from
  // every wall segment, plus a safety margin, or navgraph can never reach it.
  const SAFE_DIST = 0.45 / M_PER_DXF_UNIT;   // DXF units
  // navgraph's grid only extends ~0.45m beyond the walls' bounding box;
  // anything further out is out-of-grid and unreachable.
  const GRID_MARGIN = 0.45 / M_PER_DXF_UNIT;
  let bx0 = Infinity, by0 = Infinity, bx1 = -Infinity, by1 = -Infinity;
  for (const [[x0, y0], [x1, y1]] of clearSegs) {
    bx0 = Math.min(bx0, x0, x1); by0 = Math.min(by0, y0, y1);
    bx1 = Math.max(bx1, x0, x1); by1 = Math.max(by1, y0, y1);
  }
  const inGrid = (x, y) =>
    x >= bx0 - GRID_MARGIN && x <= bx1 + GRID_MARGIN &&
    y >= by0 - GRID_MARGIN && y <= by1 + GRID_MARGIN;
  const minWallDist = (px, py) => {
    let best = Infinity;
    for (const [[x0, y0], [x1, y1]] of clearSegs) {
      const d = distToSegU(px, py, x0, y0, x1, y1);
      if (d < best) best = d;
    }
    return best;
  };
  const outlineDist = (px, py) => {
    let best = Infinity;
    for (const [[x0, y0], [x1, y1]] of floodSegs) {
      const d = distToSegU(px, py, x0, y0, x1, y1);
      if (d < best) best = d;
    }
    return best * M_PER_DXF_UNIT;                    // metres
  };

  // BFS outward from a door through its own sealed opening. Only this
  // doorway's seal is passable, so the search cannot slip through any other
  // doorway; walls stay blocking everywhere.
  const vstamp = new Int32Array(gW * gH);
  let tag = 0;
  const leadsOutdoors = (groupId, cx, cy) => {
    const start = idxAt(col(cx), row(cy));
    if (!inGridCell(col(cx), row(cy))) return false;
    if (blocked[start] && sealOwner[start] !== groupId) return false;
    if (seen[start]) return true;
    const myTag = ++tag;
    vstamp[start] = myTag;
    const q = [start];
    const qd = [0];
    for (let head = 0; head < q.length; head++) {
      const i = q[head], d = qd[head];
      if (d >= FLOOD_LEAD_CELLS) continue;
      const c = Math.floor(i / gH), r = i % gH;
      for (const [dc, dr] of [[1, 0], [-1, 0], [0, 1], [0, -1]]) {
        const cc = c + dc, rr = r + dr;
        if (!inGridCell(cc, rr)) continue;
        const ni = idxAt(cc, rr);
        if (vstamp[ni] === myTag) continue;
        if (blocked[ni] && sealOwner[ni] !== groupId) continue;
        if (seen[ni]) return true;
        vstamp[ni] = myTag;
        q.push(ni); qd.push(d + 1);
      }
    }
    return false;
  };

  // Move a stop point into free, in-grid space, staying near the original
  // cluster (elevators, stairs and exit stops alike).
  const snapStop = (cx, cy) => {
    const ok = (x, y) => minWallDist(x, y) >= SAFE_DIST && inGrid(x, y);
    if (ok(cx, cy)) return [cx, cy];
    for (const rU of [0.25, 0.4, 0.55, 0.7, 0.85, 1.0].map(m => m / M_PER_DXF_UNIT)) {
      for (let k = 0; k < 16; k++) {
        const th = (k / 16) * 2 * Math.PI;
        const x = cx + Math.cos(th) * rU, y = cy + Math.sin(th) * rU;
        if (ok(x, y)) return [x, y];
      }
    }
    return null;
  };

  for (const [outKey, layerKind] of Object.entries(LAYER_KIND)) {
    const out = byKind[outKey];
    let id = 0;
    if (outKey === 'exits') {
      for (let g = 0; g < doorways.length; g++) {
        // Centre on the leaf/leaves; arcs would drag it onto the hinge side.
        const anchors = doorways[g].leaves.length ? doorways[g].leaves : doorways[g].arcs;
        const [x0, y0, x1, y1] = bboxOf(anchors);
        let cx = (x0 + x1) / 2, cy = (y0 + y1) / 2;
        // Exterior doors only: the leaf must hug the outline, and the
        // outdoors must be reachable through this doorway's own opening.
        // Arctic entries have an inner leaf row behind the outer one; the
        // outline test keeps the exit on the row that actually faces out.
        if (outlineDist(cx, cy) > OUTLINE_MAX_M) continue;
        if (!leadsOutdoors(g, cx, cy)) continue;
        const snapped = snapStop(cx, cy);
        if (!snapped) continue;     // can't make it reachable
        [cx, cy] = snapped;
        out.push({id: `exits-${id++}`, ll: toLL(cx, cy), dxf: [cx, cy]});
      }
      continue;
    }
    const layerShapes = shapes.filter(s => kindFor(s.layer) === layerKind);
    for (const idxs of clusterShapes(layerShapes, CLUSTER_RADIUS_M / M_PER_DXF_UNIT)) {
      let x0 = Infinity, y0 = Infinity, x1 = -Infinity, y1 = -Infinity;
      for (const i of idxs) {
        const [bx0, by0, bx1, by1] = layerShapes[i].bbox;
        x0 = Math.min(x0, bx0); y0 = Math.min(y0, by0);
        x1 = Math.max(x1, bx1); y1 = Math.max(y1, by1);
      }
      let cx = (x0 + x1) / 2, cy = (y0 + y1) / 2;
      const snapped = snapStop(cx, cy);
      if (!snapped) continue;       // can't make it reachable
      [cx, cy] = snapped;
      out.push({id: `${outKey}-${id++}`, ll: toLL(cx, cy), dxf: [cx, cy]});
    }
  }
  return byKind;
}

/**
 * Snapshot a portal set to just what is placement-independent: id + DXF
 * position. This is what the import tool writes into buildings-config.json so
 * the app can skip extractPortals (doorway clustering + coarse flood fill,
 * which costs seconds per building). DXF coords are used deliberately: the
 * extraction runs in DXF space and only touches lon/lat at the very end, so
 * re-aligning or rotating a plan never invalidates the cache.
 *
 * @param {{elevators:object[],stairs:object[],exits:object[]}} portals
 * @returns {{elevators:object[],stairs:object[],exits:object[]}}
 */
export function cachePortals(portals) {
  const strip = list => (list ?? []).map(({id, dxf}) => ({id, dxf}));
  return {
    elevators: strip(portals.elevators),
    stairs: strip(portals.stairs),
    exits: strip(portals.exits),
  };
}

/**
 * Rebuild a full portal set (with lon/lat) from a cached one, applying the
 * floor's current registration. Inverse of cachePortals.
 *
 * @param {{elevators:object[],stairs:object[],exits:object[]}} cached - {id,dxf}
 * @param {(x:number,y:number)=>number[]} toLL - the floor's DXF -> lon/lat
 * @returns {{elevators:object[],stairs:object[],exits:object[]}}
 */
export function portalsFromCache(cached, toLL) {
  const build = (list, kind) => (list ?? []).map((p, i) => ({
    id: p.id ?? `${kind}-${i}`,
    ll: toLL(p.dxf[0], p.dxf[1]),
    dxf: p.dxf,
  }));
  return {
    elevators: build(cached.elevators, 'elevators'),
    stairs: build(cached.stairs, 'stairs'),
    exits: build(cached.exits, 'exits'),
  };
}

// ---------------------------------------------------------------------------
// Tiered router
// ---------------------------------------------------------------------------

/** Binary min-heap keyed on cost (same structure as navgraph.mjs's). */
class MinHeap {
  constructor() { this.cost = []; this.node = []; }
  get size() { return this.cost.length; }
  push(node, cost) {
    const {cost: c, node: n} = this;
    let i = c.length;
    c.push(cost); n.push(node);
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (c[p] <= c[i]) break;
      [c[p], c[i]] = [c[i], c[p]]; [n[p], n[i]] = [n[i], n[p]];
      i = p;
    }
  }
  pop() {
    const {cost: c, node: n} = this, top = n[0];
    const lc = c.pop(), ln = n.pop();
    if (c.length) {
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

/** Vertex snap tolerance for the outdoor path graph, metres. */
const OUTDOOR_SNAP_M = 0.6;
/**
 * Max distance when stitching dangling fragment ends, metres. Ways crossing a
 * tile edge are clipped to each tile's 64px buffer (~11.6m at z15 here), so
 * the two halves of one way sit ~23m apart: only that genuine overlap counts
 * as a "very short connection". Longer jumps would invent paths across open
 * ground that OSM never mapped.
 */
const OUTDOOR_BRIDGE_M = 25;
/** Max straight-line distance from an exit (or outdoor click) to a node. */
const OUTDOOR_MAX_SNAP_M = 150;

/**
 * Spatial index over wall/obstacle line segments answering "does this segment
 * cut through a building?". Used to keep outdoor path edges out of
 * footprints: OSM ways can be stale, and tile-buffer stitching can join
 * dangling ends across a building, and either way the drawn route would run
 * straight through a wall.
 *
 * Segments live in metres; the index buckets them into a grid so a query only
 * tests the handful of segments near the tested segment.
 *
 * @param {number[][][]} llSegments - [[lon,lat],[lon,lat]], ...
 * @param {{lon:number,lat:number}} scale - metres-per-degree at that latitude
 * @returns {{crosses:(ax,ay,bx,by,margin)=>boolean, segCount:number}}
 */
export function buildObstacleIndex(llSegments, scale) {
  const CELL = 10;   // metres
  const segs = llSegments.map(([[x0, y0], [x1, y1]]) =>
    [x0 * scale.lon, y0 * scale.lat, x1 * scale.lon, y1 * scale.lat]);
  const grid = new Map();
  for (let i = 0; i < segs.length; i++) {
    const [ax, ay, bx, by] = segs[i];
    for (let cx = Math.floor(Math.min(ax, bx) / CELL); cx <= Math.floor(Math.max(ax, bx) / CELL); cx++) {
      for (let cy = Math.floor(Math.min(ay, by) / CELL); cy <= Math.floor(Math.max(ay, by) / CELL); cy++) {
        const k = cx + ':' + cy;
        let list = grid.get(k);
        if (!list) grid.set(k, list = []);
        list.push(i);
      }
    }
  }
  // Stamped so a segment shared by several query cells is tested once.
  let stamp = 0;
  const stamped = new Int32Array(segs.length);

  /**
   * Proper segment intersection test with an endpoint margin on the QUERY
   * segment only: crossings within `margin` metres of the query's ends don't
   * count, so a path/stub that merely meets a wall at a doorway passes.
   *
   * The margin is deliberately NOT applied to the obstacle segment. Plans are
   * drawn with walls fragmented into many sub-metre pieces (AS162 has 821 of
   * 1026 wall segments under 2 m); skipping crossings near an obstacle end
   * made every such fragment invisible, so a straight outdoor line could cut
   * through a building. Only a crossing exactly at an obstacle vertex is
   * ignored (where two fragments meet).
   */
  const crossings = (ax, ay, bx, by, margin) => {
    const query = ++stamp;
    let count = 0;
    for (let cx = Math.floor(Math.min(ax, bx) / CELL); cx <= Math.floor(Math.max(ax, bx) / CELL); cx++) {
      for (let cy = Math.floor(Math.min(ay, by) / CELL); cy <= Math.floor(Math.max(ay, by) / CELL); cy++) {
        const list = grid.get(cx + ':' + cy);
        if (!list) continue;
        for (const i of list) {
          if (stamped[i] === query) continue;
          stamped[i] = query;
          const [, , dx0, dy0, dx1, dy1] = [0, 0, ...segs[i]];
          const rx = bx - ax, ry = by - ay;
          const sx = dx1 - dx0, sy = dy1 - dy0;
          const denom = rx * sy - ry * sx;
          if (Math.abs(denom) < 1e-12) continue;   // parallel
          const t = ((dx0 - ax) * sy - (dy0 - ay) * sx) / denom;
          const u = ((dx0 - ax) * ry - (dy0 - ay) * rx) / denom;
          if (t < 0 || t > 1 || u < 0 || u > 1) continue;
          const lenAB = Math.hypot(rx, ry), lenCD = Math.hypot(sx, sy);
          const mAB = t * lenAB, mCD = u * lenCD;
          if (mAB <= margin || mAB >= lenAB - margin) continue;
          // Ignore only a crossing on an obstacle vertex, never inside a
          // short wall fragment.
          if (mCD < 0.02 || mCD > lenCD - 0.02) continue;
          count++;
        }
      }
    }
    return count;
  };
  const crosses = (ax, ay, bx, by, margin) => crossings(ax, ay, bx, by, margin) > 0;

  return {crosses, crossings, segCount: segs.length};
}

/**
 * Build a walkable path graph from PATHS-layer shapes (see osm.mjs).
 *
 * Polyline vertices within ~OUTDOOR_SNAP_M quantise to the same node, which
 * merges junctions. Ways that straddle a tile edge are clipped to each tile's
 * buffer, leaving the two halves with dangling ends tens of metres apart and
 * nothing shared between them; those ends are stitched back together
 * afterwards. Edges whose segment cuts through a building (see
 * buildObstacleIndex) are dropped - a route may not run through a wall.
 * Edges carry their length in metres; the graph is undirected (pedestrians
 * are not bound by one-way rules).
 *
 * @param {object[]} shapes - {layer:'PATHS', items:[{op:'line', points}]}
 * @param {number} [lat] - latitude the shape coordinates are around, for the
 *        metres-per-degree scale.
 * @param {number[][][]} [obstacleSegs] - wall/obstacle segments as
 *        [[lon,lat],[lon,lat]], used to keep edges out of buildings.
 * @param {number} [bridgeM] - max dangling-end stitch distance, metres.
 * @returns {{nodes, nodeCount, edgeCount, scale, snap, snapClear,
 *            segmentClear, dijkstraFrom, pathOf}|null}
 */
export function buildOutdoorGraph(shapes, lat = 61.19, obstacleSegs = null,
                                  bridgeM = OUTDOOR_BRIDGE_M) {
  const scale = metresPerDeg(lat);
  const cellLon = OUTDOOR_SNAP_M / scale.lon, cellLat = OUTDOOR_SNAP_M / scale.lat;
  const key = ll => Math.round(ll[0] / cellLon) + ',' + Math.round(ll[1] / cellLat);

  const nodes = [];              // [[lon,lat],...]
  const index = new Map();       // cell key -> node index
  const adj = [];                // per node: [[neighbour, metres],...]
  const nodeOf = ll => {
    const k = key(ll);
    let i = index.get(k);
    if (i === undefined) {
      i = nodes.length;
      index.set(k, i);
      nodes.push(ll);
      adj.push([]);
    }
    return i;
  };
  const addEdge = (a, b, metres) => {
    if (a === b || metres < 0.05) return;   // duplicated vertices at joins
    const w = metres + roadPenaltyLL(nodes[a], nodes[b]);
    adj[a].push([b, w]);
    adj[b].push([a, w]);
  };

  let edgeCount = 0;
  const obstacles = obstacleSegs?.length ? buildObstacleIndex(obstacleSegs, scale) : null;

  // Roads are walkable too, but CROSSING a named road (Providence Dr, Piper
  // St, Seawolf Dr, ...) is penalised heavily in route selection so an indoor
  // or skybridge route wins wherever one exists. Following a road (parallel /
  // collinear) is not a crossing and stays free. Reported time/distance still
  // come from the real geometry, so ETAs stay honest.
  const ROAD_CROSS_PENALTY_M = 600;
  // Internal campus ways that OSM draws as roads but that function as
  // walkways. Crossing these should not force a long skybridge detour.
  // Case-insensitive; add names here if another internal way needs the same.
  const ROAD_PENALTY_EXEMPT = new Set(['spirit way', 'seawolf drive']);
  const isPenalisedRoad = (s) => {
    const isRoad = s.kind === 'major_road' ||
      (s.kind === 'minor_road' && !!s.name);
    if (!isRoad) return false;
    return !ROAD_PENALTY_EXEMPT.has(String(s.name ?? '').trim().toLowerCase());
  };
  const roadSegs = [];
  for (const s of shapes) {
    if (!isPenalisedRoad(s)) continue;
    for (const [a, b] of shapeSegments(s)) roadSegs.push([a, b]);
  }
  const roads = roadSegs.length ? buildObstacleIndex(roadSegs, scale) : null;
  /** Metres-equivalent penalty for a segment crossing one or more roads. */
  const roadPenaltyLL = (a, b) => roads
    ? roads.crossings(a[0] * scale.lon, a[1] * scale.lat,
                      b[0] * scale.lon, b[1] * scale.lat, 1.0) * ROAD_CROSS_PENALTY_M
    : 0;

  /** True when the straight run node a -> node b passes through a building. */
  const edgeBlocked = (a, b) => {
    if (!obstacles) return false;
    const A = nodes[a], B = nodes[b];
    return obstacles.crosses(
      A[0] * scale.lon, A[1] * scale.lat, B[0] * scale.lon, B[1] * scale.lat, 1.0);
  };
  for (const s of shapes) {
    for (const item of s.items || []) {
      if (item.op !== 'line' || !item.points) continue;
      for (let k = 1; k < item.points.length; k++) {
        const a = nodeOf(item.points[k - 1]);
        const b = nodeOf(item.points[k]);
        if (a === b) continue;
        const metres = distLL(nodes[a], nodes[b], scale);
        if (metres < 0.05) continue;
        if (edgeBlocked(a, b)) continue;
        addEdge(a, b, metres);
        edgeCount++;
      }
    }
  }
  if (!nodes.length) return null;

  // --- stitch tile-buffer fragments ---------------------------------------
  // A way crossing a tile edge is clipped in both tiles, so its two halves
  // end at different buffer-edge points ~23m apart with nothing shared
  // between them. Join only nearby dangling ends - "very short connections"
  // that rejoin the halves of one way. Anything longer would invent a path
  // across ground OSM never mapped.
  {
    const dangling = [];
    for (let i = 0; i < adj.length; i++) if (adj[i].length === 1) dangling.push(i);
    for (let i = 0; i < dangling.length; i++) {
      for (let j = i + 1; j < dangling.length; j++) {
        const a = dangling[i], b = dangling[j];
        const metres = distLL(nodes[a], nodes[b], scale);
        if (metres > bridgeM) continue;          // cheap test first
        if (edgeBlocked(a, b)) continue;          // may not cut a building
        addEdge(a, b, metres);
        edgeCount++;
      }
    }
  }

  /** Nearest graph node to a lon/lat point. Graphs here are small; scan. */
  const snap = ll => {
    let best = 0, bestD = Infinity;
    for (let i = 0; i < nodes.length; i++) {
      const d = distLL(ll, nodes[i], scale);
      if (d < bestD) { bestD = d; best = i; }
    }
    return {node: best, metres: bestD};
  };

  /**
   * Attach a point to the path network at the nearest place whose straight
   * stub does not cut through a building, within `maxM` metres; null when
   * there is none. Used to attach exits and outdoor clicks.
   *
   * The nearest place is the nearest node OR the nearest point along an edge.
   * Edges matter: a tap or exit that faces the middle of a long way sees no
   * vertex for tens of metres, and vertex-only snapping dragged the attachment
   * out to that far vertex - so a path that visibly runs under the finger was
   * never used, and the route detoured around it. Projecting onto the edge
   * instead splits it in place and attaches there, so the stub is a short
   * straight hop onto the path.
   *
   * @returns {{node:number, metres:number, ll:number[]}|null}
   */
  const snapClear = (ll, maxM = OUTDOOR_MAX_SNAP_M) => {
    const x = ll[0] * scale.lon, y = ll[1] * scale.lat;
    // nearest clear node
    let nodeBest = -1, nodeBestD = Infinity;
    for (let i = 0; i < nodes.length; i++) {
      const d = distLL(ll, nodes[i], scale);
      if (d >= nodeBestD || d > maxM) continue;
      const N = nodes[i];
      if (obstacles && obstacles.crosses(x, y, N[0] * scale.lon, N[1] * scale.lat, 2.0)) continue;
      nodeBestD = d;
      nodeBest = i;
    }
    // nearest clear point along any edge
    let eBest = null;
    for (let i = 0; i < adj.length; i++) {
      for (const [j] of adj[i]) {
        if (i > j) continue;
        const A = nodes[i], B = nodes[j];
        const ax = A[0] * scale.lon, ay = A[1] * scale.lat;
        const bx = B[0] * scale.lon, by = B[1] * scale.lat;
        const dx = bx - ax, dy = by - ay, l2 = dx * dx + dy * dy;
        if (l2 < 1e-9) continue;
        let t = ((x - ax) * dx + (y - ay) * dy) / l2;
        t = t < 0 ? 0 : t > 1 ? 1 : t;
        const qx = ax + t * dx, qy = ay + t * dy;
        const d = Math.hypot(x - qx, y - qy);
        if (d > maxM || (eBest && d >= eBest.d)) continue;
        if (obstacles && obstacles.crosses(x, y, qx, qy, 2.0)) continue;
        eBest = {d, i, j, qx, qy, t};
      }
    }
    if (nodeBest !== -1 && (!eBest || nodeBestD <= eBest.d)) {
      return {node: nodeBest, metres: nodeBestD, ll: nodes[nodeBest]};
    }
    if (!eBest) return null;
    // Reuse an endpoint when the projection lands on it, otherwise split the
    // edge at the projection and attach to the new node.
    if (eBest.t <= 0.001) return {node: eBest.i, metres: eBest.d, ll: nodes[eBest.i]};
    if (eBest.t >= 0.999) return {node: eBest.j, metres: eBest.d, ll: nodes[eBest.j]};
    const qll = [eBest.qx / scale.lon, eBest.qy / scale.lat];
    const p = nodeOf(qll);
    if (p !== eBest.i && p !== eBest.j) {
      adj[eBest.i] = adj[eBest.i].filter(([v]) => v !== eBest.j);
      adj[eBest.j] = adj[eBest.j].filter(([v]) => v !== eBest.i);
      edgeCount--;
      const mi = distLL(nodes[eBest.i], nodes[p], scale);
      const mj = distLL(nodes[p], nodes[eBest.j], scale);
      if (mi >= 0.05) { addEdge(eBest.i, p, mi); edgeCount++; }
      if (mj >= 0.05) { addEdge(p, eBest.j, mj); edgeCount++; }
    }
    return {node: p, metres: eBest.d, ll: qll};
  };

  /** True when the straight lon/lat run a -> b stays clear of buildings. */
  const segmentClear = (a, b, margin = 1) => !obstacles?.crosses(
    a[0] * scale.lon, a[1] * scale.lat, b[0] * scale.lon, b[1] * scale.lat, margin);

  /** Dijkstra from one node -> {dist, prev} over the whole graph. */
  const dijkstraFrom = src => {
    const dist = new Float64Array(nodes.length).fill(Infinity);
    const prev = new Int32Array(nodes.length).fill(-1);
    const done = new Uint8Array(nodes.length);
    const open = new MinHeap();
    dist[src] = 0;
    open.push(src, 0);
    while (open.size) {
      const u = open.pop();
      if (done[u]) continue;
      done[u] = 1;
      const du = dist[u];
      for (const [v, metres] of adj[u]) {
        const nd = du + metres;
        if (nd < dist[v]) { dist[v] = nd; prev[v] = u; open.push(v, nd); }
      }
    }
    return {dist, prev};
  };

  /** Node polyline src -> dst from a dijkstraFrom result, or null. */
  const pathOf = ({dist, prev}, dst) => {
    if (!isFinite(dist[dst])) return null;
    const out = [];
    for (let v = dst; v !== -1; v = prev[v]) out.push(nodes[v]);
    return out.reverse();
  };

  return {
    nodes, nodeCount: nodes.length, edgeCount, scale,
    snap, snapClear, segmentClear, dijkstraFrom, pathOf,
    roadPenaltyLL,
  };
}

/**
 * Build the campus router.
 *
 * @param {object[]} buildings - per building:
 *   { id, groundFloor, floors: [{ floorId, navGraph, elevators, stairs, exits }] }
 *   where elevators/stairs/exits are as returned by extractPortals, and
 *   navGraph is the result of navgraph.mjs buildGraph() for that floor's walls.
 * @param {object[]} [outdoorShapes] - shapes on the PATHS layer (osm.mjs),
 *   polylines of [[lon,lat],...]; the walkable outdoor network.
 * @param {string} [ignorePortals] - 'stairs' to ignore stair connectors,
 *   'elevator' to ignore elevator connectors, or null/undefined to use both.
 * @returns {{route: Function, stats: object}}
 */
export function buildCampusRouter(buildings, outdoorShapes = null, ignorePortals = null) {
  const t0 = performance.now();

  // --- link vertical connectors across floors ------------------------------
  // The same physical elevator/stairwell appears on every floor at (nearly)
  // the same plan position; match stops by proximity.
  for (const b of buildings) {
    const ground = b.floors.find(f => f.floorId === b.groundFloor);
    if (!ground) continue;
    const anyLL = ground.elevators[0]?.ll ?? ground.stairs[0]?.ll ?? ground.exits[0]?.ll;
    const scale = metresPerDeg(anyLL?.[1] ?? 61.19);
    const byKind = {elevator: new Map(), stairs: new Map()};
    // Seed with the ground floor's connectors. Every connector carries an
    // explicit `anchor` (its own first stop): a connector kind that is missing
    // on the ground floor (bad data, or a stairwell that only exists on upper
    // floors) seeds from the first floor that has it, and later floors link to
    // that anchor by proximity. Looking up the ground-floor stop directly
    // crashed (`distLL(undefined, ...)`) when no such stop existed.
    const activeKinds = ignorePortals === 'stairs' ? ['elevator'] :
                        ignorePortals === 'elevator' ? ['stairs'] :
                        ['elevator', 'stairs'];
    for (const kind of activeKinds) {
      const list = kind === 'elevator' ? ground.elevators : ground.stairs;
      for (const p of list ?? []) {
        byKind[kind].set(p.id, {
          kind, building: b.id, anchor: p.ll,
          stops: new Map([[ground.floorId, p.ll]]),
        });
      }
    }
    // attach stops from other floors to the nearest anchor of the same kind
    for (const f of b.floors) {
      if (f.floorId === b.groundFloor) continue;
      for (const kind of activeKinds) {
        const list = f[kind === 'elevator' ? 'elevators' : 'stairs'] ?? [];
        for (const p of list) {
          let best = null, bestD = Infinity;
          for (const c of byKind[kind].values()) {
            const d = distLL(c.anchor, p.ll, scale);
            if (d < bestD) { bestD = d; best = c; }
          }
          if (best && bestD <= CONNECTOR_LINK_M) best.stops.set(f.floorId, p.ll);
          else byKind[kind].set(`${kind}-x${byKind[kind].size}`, {
            kind, building: b.id, anchor: p.ll,
            stops: new Map([[f.floorId, p.ll]]),
          });
        }
      }
    }
    b._connectors = [...byKind.elevator.values(), ...byKind.stairs.values()];
  }

  // Lon/lat bounding box per floor, so buildingAt can reject a floor without
  // paying for an attachment scan (the campus has many floors now).
  for (const b of buildings) {
    for (const f of b.floors) {
      let x0 = Infinity, y0 = Infinity, x1 = -Infinity, y1 = -Infinity;
      for (const [[sx0, sy0], [sx1, sy1]] of f.obstacleLines ?? []) {
        x0 = Math.min(x0, sx0, sx1); x1 = Math.max(x1, sx0, sx1);
        y0 = Math.min(y0, sy0, sy1); y1 = Math.max(y1, sy0, sy1);
      }
      f._bbox = isFinite(x0) ? {x0, y0, x1, y1} : null;
    }
  }

  // --- macro graph nodes ---------------------------------------------------
  // A node is an exterior door, a skybridge portal, OR a connector stop (an
  // elevator/stairs landing on a floor). Including connector stops lets the
  // route change floors ANYWHERE - at the start, the destination, or an
  // intermediate building - instead of only descending at the start and
  // ascending at the end (which forced bizarre go-outside-and-around routes).
  // Edges:
  //   same building, same floor -> indoor walk between any two nodes
  //   same connector, different floors -> vertical ride (elevator/stairs)
  //   two bridges, same level, different buildings -> skybridge link
  //   cross-building ground-floor doors -> outdoor path-network length
  const nodes = [];
  for (const b of buildings) {
    for (const f of b.floors) {
      for (const e of f.exits) nodes.push({building: b, floor: f, ll: e.ll, kind: 'door', id: e.id});
      for (const s of f.skybridges ?? []) nodes.push({building: b, floor: f, ll: s.ll, kind: 'bridge', id: s.id, to: s.to ?? null});
      for (const c of b._connectors ?? []) {
        const stop = c.stops.get(f.floorId);
        if (stop) nodes.push({building: b, floor: f, ll: stop, kind: 'connector', connector: c, id: `${c.kind}-${f.floorId}`});
      }
    }
  }
  const scale = metresPerDeg(nodes[0]?.ll[1] ?? 61.19);
  const N = nodes.length;
  // Outdoor legs exist at ground level only. Using the building's configured
  // groundFloor was wrong for level-3-only structures: AS164 (Parrish Bridge)
  // is a skybridge landing with groundFloor "03", so its doors became outdoor
  // doors and a route stepped from an outdoor leg straight onto floor 03 with
  // no vertical connector. Floor level 1 is the campus ground plane.
  const atGround = n => (parseInt(n.floor.floorId, 10) || 0) === 1;
  const floorNum = id => parseInt(id, 10) || 0;

  // Attach every node to its floor's graph once. A manually placed portal often
  // sits on (or a hair outside) the wall, where the exact point sees no nodes;
  // fall back to the nearest walkable cell so the floor can still reach it.
  const attachPortal = (g, ll) => {
    const a = g.attach(ll);
    if (a.length) return a;
    const nw = g.nearestWalkable(ll, 120);
    return nw ? g.attach(nw) : [];
  };
  const nodeAttach = nodes.map(n => attachPortal(n.floor.navGraph, n.ll));

  // Same-floor distance matrix among all nodes on a floor. One attach per node
  // (above) plus one Dijkstra per node gives every intra-floor edge in O(k),
  // not the O(k^2) findPaths the macro graph used to run.
  const sameFloorSec = new Map();   // `${i}:${j}` (i<j) -> seconds
  {
    const byFloor = new Map();
    for (let i = 0; i < N; i++) {
      let list = byFloor.get(nodes[i].floor);
      if (!list) byFloor.set(nodes[i].floor, list = []);
      list.push(i);
    }
    for (const [, idxs] of byFloor) {
      if (idxs.length < 2) continue;
      const g = nodes[idxs[0]].floor.navGraph;
      for (const i of idxs) {
        if (!nodeAttach[i].length) continue;
        const run = g.dijkstraFromAttach(nodeAttach[i]);
        for (const j of idxs) {
          if (j <= i) continue;
          const {dist} = g.distToAttach(run, nodeAttach[j]);
          if (isFinite(dist)) sameFloorSec.set(`${i}:${j}`, dist / INDOOR_SPEED);
        }
      }
    }
  }

  // Outdoor network: one snap + one Dijkstra run per ground-floor door. Every
  // cross-building outdoor edge reads its cost off those runs, and the same runs
  // reconstruct the polyline for whichever chain the route picks. Snaps use
  // snapClear so the door -> node stub cannot cut through a building.
  const obstacleSegs = [];
  for (const b of buildings) {
    const ground = b.floors.find(f => f.floorId === b.groundFloor);
    for (const seg of ground?.obstacleLines ?? []) obstacleSegs.push(seg);
  }
  const outdoor = outdoorShapes?.length
    ? buildOutdoorGraph(outdoorShapes, nodes[0]?.ll[1] ?? 61.19, obstacleSegs)
    : null;
  /** Extra cost (metres-equivalent) for a polyline crossing named roads. */
  const roadPenaltyFor = (poly) => {
    if (!outdoor?.roadPenaltyLL || poly.length < 2) return 0;
    let p = 0;
    for (let i = 1; i < poly.length; i++) p += outdoor.roadPenaltyLL(poly[i - 1], poly[i]);
    return p;
  };
  const outdoorRuns = nodes.map(n => {
    if (!outdoor || n.kind !== 'door' || !atGround(n)) return null;
    const s = outdoor.snapClear(n.ll);
    return s ? {snap: s, stubPenalty: roadPenaltyFor([n.ll, s.ll]), run: null} : null;
  });
  for (const r of outdoorRuns) if (r) r.run = outdoor.dijkstraFrom(r.snap.node);

  const macroAdj = Array.from({length: N}, () => []);
  // Descriptor for each macro edge, so route() can rebuild the right geometry.
  const macroEdge = new Map();
  const link = (i, j, sec, desc) => {
    macroAdj[i].push([j, sec]);
    macroAdj[j].push([i, sec]);
    macroEdge.set(`${i}:${j}`, desc);
    macroEdge.set(`${j}:${i}`, desc);
  };

  let sameFloorEdges = 0, verticalEdges = 0, outdoorEdges = 0, outdoorNetworkEdges = 0, skybridgeEdges = 0;

  // Intra-floor edges (all node pairs on a floor).
  for (const [key, sec] of sameFloorSec) {
    const [i, j] = key.split(':').map(Number);
    link(i, j, sec, {kind: 'indoor'});
    sameFloorEdges++;
  }

  // Vertical edges: the stops of one connector, across floors.
  {
    const byConn = new Map();
    for (let i = 0; i < N; i++) {
      const n = nodes[i];
      if (n.kind !== 'connector') continue;
      let l = byConn.get(n.connector);
      if (!l) byConn.set(n.connector, l = []);
      l.push(i);
    }
    for (const [c, idxs] of byConn) {
      for (let a = 0; a < idxs.length; a++) for (let b = a + 1; b < idxs.length; b++) {
        const i = idxs[a], j = idxs[b];
        const f1 = nodes[i].floor.floorId, f2 = nodes[j].floor.floorId;
        if (f1 === f2) continue;
        link(i, j, verticalSeconds(c, f1, f2), {kind: 'vertical', tier: c.kind, fromFloor: f1, toFloor: f2});
        verticalEdges++;
      }
    }
  }

  // --- skybridge pairing --------------------------------------------------
  // A skybridge portal may carry an explicit destination (`to`, the other
  // building's id - set in the import tool). Explicit pairs win: mutual marks
  // first, then one-sided, both shortest-first; they ignore both
  // SKYBRIDGE_MAX_M and the floor number, because the marks say exactly which
  // bridge goes where (ECB-01 <-> SU-02 crosses a grade change; ECB-02 <->
  // SU-02 was ~125 m, over the old cap). Unmarked bridges then fall back to
  // greedy min-distance matching: same floor *number* ("2" vs "02" still
  // link) and within the cap. A bridge with an explicit destination never
  // auto-pairs elsewhere, so a wrong auto-link cannot happen to it. Only
  // explicit bridges participate - an ordinary (rooftop) exit never connects a
  // building - and ground-level bridges are allowed (a shared first floor).
  const bridgeIdx = [];
  for (let i = 0; i < N; i++) if (nodes[i].kind === 'bridge') bridgeIdx.push(i);
  const bridgeCandidates = [];
  for (let a = 0; a < bridgeIdx.length; a++) {
    for (let b = a + 1; b < bridgeIdx.length; b++) {
      const i = bridgeIdx[a], j = bridgeIdx[b];
      const A = nodes[i], B = nodes[j];
      if (A.building === B.building) continue;
      const aWants = A.to === B.building.id, bWants = B.to === A.building.id;
      if (!aWants && !bWants) {
        // Auto fallback: both unmarked, same floor number, within the cap.
        if (A.to || B.to) continue;
        if (floorNum(B.floor.floorId) !== floorNum(A.floor.floorId)) continue;
        const d = distLL(A.ll, B.ll, scale);
        if (d > SKYBRIDGE_MAX_M) continue;
        bridgeCandidates.push({i, j, d, rank: 2});
      } else {
        bridgeCandidates.push({i, j, d: distLL(A.ll, B.ll, scale), rank: aWants && bWants ? 0 : 1});
      }
    }
  }
  bridgeCandidates.sort((x, y) => x.rank - y.rank || x.d - y.d);
  const bridgeUsed = new Set();
  for (const {i, j, d} of bridgeCandidates) {
    if (bridgeUsed.has(i) || bridgeUsed.has(j)) continue;
    bridgeUsed.add(i); bridgeUsed.add(j);
    link(i, j, d / INDOOR_SPEED, {kind: 'skybridge'});
    skybridgeEdges++;
  }

  // Cross-building outdoor edges between ground-floor doors.
  for (let i = 0; i < N; i++) {
    const A = nodes[i];
    if (A.kind !== 'door' || !atGround(A)) continue;
    for (let j = i + 1; j < N; j++) {
      const B = nodes[j];
      if (B.building === A.building) continue;
      if (B.kind !== 'door' || !atGround(B)) continue;
      let sec = null;
      const a = outdoorRuns[i], b = outdoorRuns[j];
      if (a && b) {
        const metres = a.run.dist[b.snap.node] + a.snap.metres + b.snap.metres +
          (a.stubPenalty ?? 0) + (b.stubPenalty ?? 0);
        if (isFinite(metres)) { sec = (metres / OUTDOOR_SPEED) * OUTDOOR_COST_MULT; outdoorNetworkEdges++; }
      }
      if (sec === null && (!outdoor || outdoor.segmentClear(A.ll, B.ll, 2))) {
        const metres = distLL(A.ll, B.ll, scale) + roadPenaltyFor([A.ll, B.ll]);
        sec = (metres / OUTDOOR_SPEED) * OUTDOOR_COST_MULT;
      }
      if (sec !== null) { link(i, j, sec, {kind: 'outdoor'}); outdoorEdges++; }
    }
  }

  // --- helpers -------------------------------------------------------------

  /** findPath with deterministic retry ring when a portal sits in a wall. */
  function indoorPath(graph, a, b) {
    let p = graph.findPath(a, b);
    if (p) return p;
    const offs = [];
    for (let k = 0; k < 8; k++) {
      const th = (k / 8) * 2 * Math.PI;
      offs.push([Math.cos(th), Math.sin(th)]);
    }
    const mpr = metresPerDeg(a[1]);
    const jitter = (pt, fx, fy) => [pt[0] + (fx * RETRY_RADIUS_M) / mpr.lon, pt[1] + (fy * RETRY_RADIUS_M) / mpr.lat];
    for (const [fx, fy] of offs) {
      p = graph.findPath(jitter(a, fx, fy), b);
      if (p) return [a, ...p.slice(1)];
    }
    for (const [fx, fy] of offs) {
      p = graph.findPath(a, jitter(b, fx, fy));
      if (p) return [...p.slice(0, -1), b];
    }
    return null;
  }

  /** Vertical travel time between two floors via one connector, seconds. */
  function verticalSeconds(connector, fromFloor, toFloor) {
    const df = Math.abs(floorNum(fromFloor) - floorNum(toFloor));
    if (df === 0) return 0;
    return connector.kind === 'elevator'
      ? ELEVATOR_WAIT_S + ELEVATOR_PER_FLOOR_S * df
      : STAIRS_PER_FLOOR_S * df;
  }

  /**
   * Attach a point on a floor to every node on that floor (doors, bridges and
   * connector stops): one attach + one Dijkstra, read off the reusable node
   * attachments. Returns Map<nodeIndex, {sec}>, or null if none reachable.
   * Vertical travel to other floors is then just the connector edges in the
   * macro graph, so the route may ascend or descend anywhere.
   */
  function attachToFloor(floor, point) {
    const g = floor.navGraph;
    const pAtt = g.attach(point);
    if (!pAtt.length) return null;
    const run = g.dijkstraFromAttach(pAtt);
    const res = new Map();
    for (let i = 0; i < N; i++) {
      if (nodes[i].floor !== floor) continue;
      const {dist} = g.distToAttach(run, nodeAttach[i]);
      if (isFinite(dist)) res.set(i, {sec: dist / INDOOR_SPEED});
    }
    return res.size ? res : null;
  }

  /**
   * Attachment for a click that lands outside every building: walk the network
   * to each ground-floor door, fall back to a straight line when there is no
   * network or no connection. Results per door are {sec, polyline} where
   * polyline runs click -> door.
   * @returns {Map<number, {sec:number, polyline:number[][]}>}
   */
  function outdoorAttachment(ll, clickSnap, from) {
    const res = new Map();
    for (let i = 0; i < N; i++) {
      const n = nodes[i];
      if (n.kind !== 'door' || !atGround(n)) continue;
      const r = outdoorRuns[i];
      if (from && r) {
        const p = outdoor.pathOf(from, r.snap.node);
        if (p) {
          const poly = [ll, ...p, n.ll];
          const metres = polyLength(poly, scale) + roadPenaltyFor(poly);
          res.set(i, {sec: (metres / OUTDOOR_SPEED) * OUTDOOR_COST_MULT, polyline: poly});
          continue;
        }
      }
      if (outdoor && !outdoor.segmentClear(ll, n.ll, 2)) continue;
      const poly = [ll, n.ll];
      const metres = polyLength(poly, scale) + roadPenaltyFor(poly);
      res.set(i, {sec: (metres / OUTDOOR_SPEED) * OUTDOOR_COST_MULT, polyline: poly});
    }
    return res;
  }

  /**
   * Which building (if any) contains this point, on the given floor. With a
   * `floorId` hint that floor wins (falling back to the ground floor when the
   * building lacks it); without one, the ground floor is tried first and any
   * upper floor after - geometry alone can't tell overlapping footprints
   * apart, so the hint (the floor the user is viewing) is authoritative.
   */
  function buildingAt(ll, floorId) {
    const mpr = metresPerDeg(ll[1]);
    // A point marginally inside a wall (door jamb, portal point) must still
    // classify indoors, so fall back to the same retry ring indoorPath uses.
    const ring = [];
    for (let k = 0; k < 8; k++) {
      const th = (k / 8) * 2 * Math.PI;
      ring.push([ll[0] + (Math.cos(th) * RETRY_RADIUS_M) / mpr.lon,
                 ll[1] + (Math.sin(th) * RETRY_RADIUS_M) / mpr.lat]);
    }
    const hit = (b, f) => {
      if (!f) return null;
      // Reject by bounding box first: an attachment scan is O(nodes).
      if (f._bbox) {
        const pad = 1.5;
        if (ll[0] < f._bbox.x0 - pad / mpr.lon || ll[0] > f._bbox.x1 + pad / mpr.lon ||
            ll[1] < f._bbox.y0 - pad / mpr.lat || ll[1] > f._bbox.y1 + pad / mpr.lat) return null;
      }
      if (f.navGraph.attach(ll).length) return {building: b, floor: f};
      for (const q of ring) {
        if (f.navGraph.attach(q).length) return {building: b, floor: f};
      }
      return null;
    };
    if (floorId !== undefined) {
      for (const b of buildings) {
        const f = b.floors.find(x => x.floorId === floorId) ??
                  b.floors.find(x => x.floorId === b.groundFloor);
        const r = hit(b, f);
        if (r) return r;
      }
      return null;
    }
    for (const b of buildings) {
      const r = hit(b, b.floors.find(x => x.floorId === b.groundFloor));
      if (r) return r;
    }
    for (const b of buildings) {
      for (const f of b.floors) { const r = hit(b, f); if (r) return r; }
    }
    return null;
  }

  /**
   * Route between two [lon,lat] points across buildings and floors.
   *
   * @param {number[]} startLL
   * @param {number[]} endLL
   * @param {{startFloor?:string, endFloor?:string}} [opts] floor hints - the
   *        floor the user is viewing. Omitted, the point's floor is detected
   *        (ground first). Supplied, it wins (a building without that floor
   *        uses its ground floor).
   * @returns {{segments:object[], totalSeconds:number, totalMeters:number,
   *            summary:string}|null}
   */
  function route(startLL, endLL, opts = {}) {
    const startHit = buildingAt(startLL, opts.startFloor);
    const endHit = buildingAt(endLL, opts.endFloor);

    // Same building, same floor: a single indoor leg is always at least as
    // good as anything routed through exits.
    if (startHit && endHit &&
        startHit.building === endHit.building && startHit.floor === endHit.floor) {
      const p = indoorPath(startHit.floor.navGraph, startLL, endLL);
      if (p) {
        const meters = polyLength(p, scale);
        return {
          segments: [{
            tier: 'indoor', building: startHit.building.id, floor: startHit.floor.floorId,
            polyline: p, meters, seconds: meters / INDOOR_SPEED,
          }],
          totalSeconds: meters / INDOOR_SPEED,
          totalMeters: meters,
          summary: `${startHit.building.id} ${startHit.floor.floorId} (indoor)`,
        };
      }
      // fall through: same floor but no direct line - route via exits
    }

    // --- origin / destination attachment to macro nodes --------------------
    // Snap both outdoor ends BEFORE any Dijkstra: a snap may split an edge and
    // add a node, and a run started earlier would not contain that node.
    let origin = null, dest = null;
    let startSnap = null, endSnap = null, startRun = null, endRun = null;
    if (startHit) {
      origin = attachToFloor(startHit.floor, startLL);
    } else {
      startSnap = outdoor ? outdoor.snapClear(startLL) : null;
    }
    if (endHit) {
      dest = attachToFloor(endHit.floor, endLL);
    } else {
      endSnap = outdoor ? outdoor.snapClear(endLL) : null;
    }
    if (startSnap) startRun = outdoor.dijkstraFrom(startSnap.node);
    if (endSnap) endRun = outdoor.dijkstraFrom(endSnap.node);
    // outdoors: walk the OSM network from the click to every exit
    if (!startHit) origin = outdoorAttachment(startLL, startSnap, startRun);
    if (!endHit) dest = outdoorAttachment(endLL, endSnap, endRun);
    if (!origin || !dest) return null;

    // --- direct outdoor walk, when neither end is inside a building --------
    // The exit-chain model can only express an outdoor trip as exit -> exit,
    // so a walk straight along a path that passes no entrance was invisible
    // and the route detoured through the nearest exit. The network path
    // between the two clicks is a path in the same graph, so offer it too and
    // take whichever is quicker (cutting through a building can still win).
    let direct = null;
    if (!startHit && !endHit && startRun && endRun) {
      const p = outdoor.pathOf(startRun, endSnap.node);
      if (p) {
        const poly = [startLL, ...p, endLL];
        const metres = polyLength(poly, scale) + roadPenaltyFor(poly);
        direct = {sec: (metres / OUTDOOR_SPEED) * OUTDOOR_COST_MULT, polyline: poly};
      }
    }

    // --- Dijkstra over the macro graph with virtual source/sink -----------
    // Node ids 0..N-1 are exits; N = virtual source, N+1 = virtual sink.
    const SRC = N, SNK = N + 1;
    const adj = macroAdj.map(l => l.slice());
    adj.push([]); adj.push([]);
    for (const [ei, {sec}] of origin) adj[SRC].push([ei, sec]);
    for (const [ei, {sec}] of dest) adj[ei].push([SNK, sec]);

    const cost = new Float64Array(N + 2).fill(Infinity);
    const prev = new Int32Array(N + 2).fill(-1);
    const done = new Uint8Array(N + 2);
    const open = new MinHeap();
    cost[SRC] = 0;
    open.push(SRC, 0);
    while (open.size) {
      const u = open.pop();
      if (done[u]) continue;
      done[u] = 1;
      if (u === SNK) break;
      for (const [v, w] of adj[u]) {
        const nd = cost[u] + w;
        if (nd < cost[v]) { cost[v] = nd; prev[v] = u; open.push(v, nd); }
      }
    }
    // Prefer the direct outdoor walk when the exit chain is infeasible or no
    // quicker (the chain can beat it only by cutting through a building).
    if (!done[SNK] || (direct && direct.sec <= cost[SNK])) {
      if (!direct) return null;
      const meters = polyLength(direct.polyline, scale);
      return {
        segments: [{tier: 'outdoor', polyline: direct.polyline,
                    meters, seconds: meters / OUTDOOR_SPEED}],
        totalSeconds: meters / OUTDOOR_SPEED,
        totalMeters: meters,
        summary: 'outside',
      };
    }

    // --- expand the chain into segments ------------------------------------
    const chain = [];
    for (let v = SNK; v >= 0; v = prev[v]) chain.push(v);
    chain.reverse(); // SRC, exit..., SNK

    const segments = [];
    let totalMeters = 0;

    /** One outdoor segment from a full polyline (click -> exit legs). */
    const addOutdoorPoly = poly => {
      const meters = polyLength(poly, scale);
      totalMeters += meters;
      segments.push({tier: 'outdoor', polyline: poly, meters, seconds: meters / OUTDOOR_SPEED});
    };
    const addPoly = (seg) => {
      const meters = seg.polyline ? polyLength(seg.polyline, scale) : 0;
      totalMeters += meters;
      const speed = seg.tier === 'outdoor' ? OUTDOOR_SPEED : INDOOR_SPEED;
      segments.push({...seg, meters, seconds: meters / speed});
    };
    const addVertical = (seg) => {
      const seconds = verticalSeconds({kind: seg.tier}, seg.fromFloor, seg.toFloor);
      segments.push({...seg, meters: 0, seconds});
    };

    // prefix: start point -> first node
    const firstIdx = chain[1];
    if (startHit) {
      const p = indoorPath(startHit.floor.navGraph, startLL, nodes[firstIdx].ll);
      if (p) addPoly({tier: 'indoor', building: startHit.building.id, floor: startHit.floor.floorId, polyline: p});
    } else {
      addOutdoorPoly(origin.get(firstIdx).polyline);
    }

    // macro chain legs
    for (let k = 1; k < chain.length - 2; k++) {
      const a = chain[k], b = chain[k + 1];
      const desc = macroEdge.get(`${a}:${b}`);
      if (desc?.kind === 'vertical') {
        addVertical({tier: desc.tier, building: nodes[a].building.id,
                     fromFloor: desc.fromFloor, toFloor: desc.toFloor,
                     // The connector landing, so a 3D view can draw the riser.
                     ll: nodes[a].ll});
      } else if (desc?.kind === 'skybridge') {
        addPoly({tier: 'skybridge', building: nodes[b].building.id,
                 floor: nodes[a].floor.floorId,
                 // Explicit bridges may join different floors (a grade
                 // change); carry both ends so the view can place the line.
                 fromFloor: nodes[a].floor.floorId, toFloor: nodes[b].floor.floorId,
                 polyline: [nodes[a].ll, nodes[b].ll]});
      } else if (desc?.kind === 'indoor') {
        const p = indoorPath(nodes[a].floor.navGraph, nodes[a].ll, nodes[b].ll);
        if (p) addPoly({tier: 'indoor', building: nodes[a].building.id, floor: nodes[a].floor.floorId, polyline: p});
      } else {
        // outdoor: door stub -> path network -> door stub
        const ra = outdoorRuns[a], rb = outdoorRuns[b];
        let poly = [nodes[a].ll, nodes[b].ll];
        if (ra && rb) {
          const p = outdoor.pathOf(ra.run, rb.snap.node);
          if (p) poly = [nodes[a].ll, ...p, nodes[b].ll];
        }
        addOutdoorPoly(poly);
      }
    }

    // suffix: last node -> end point
    const lastIdx = chain[chain.length - 2];
    if (endHit) {
      const p = indoorPath(endHit.floor.navGraph, nodes[lastIdx].ll, endLL);
      if (p) addPoly({tier: 'indoor', building: endHit.building.id, floor: endHit.floor.floorId, polyline: p});
    } else {
      addOutdoorPoly(dest.get(lastIdx).polyline);
    }

    const totalSeconds = segments.reduce((a, s) => a + (s.seconds || 0), 0);
    const parts = [];
    let lastBuilding = null;
    if (startHit) {
      parts.push(`${startHit.building.id} ${startHit.floor.floorId}`);
      lastBuilding = startHit.building.id;
    }
    for (const s of segments) {
      if (s.tier === 'outdoor') {
        if (parts[parts.length - 1] !== 'outside') parts.push('outside');
        lastBuilding = null;
      } else if (s.tier === 'elevator' || s.tier === 'stairs') {
        parts.push(s.tier);
      } else if (s.tier === 'skybridge') {
        parts.push('skybridge');
        lastBuilding = s.building ?? lastBuilding;
      } else if (s.tier === 'indoor' && s.building && s.building !== lastBuilding) {
        parts.push(s.building);
        lastBuilding = s.building;
      }
    }
    const endTag = endHit ? `${endHit.building.id} ${endHit.floor.floorId}` : null;
    if (endTag && parts[parts.length - 1] !== endTag) parts.push(endTag);
    return {
      segments,
      totalSeconds,
      totalMeters,
      summary: parts.join(' → '),
    };
  }

  const doorCount = nodes.reduce((a, n) => a + (n.kind === 'door' ? 1 : 0), 0);
  const stats = {
    buildings: buildings.length,
    nodes: N,
    exits: doorCount,
    macroEdges: sameFloorEdges + verticalEdges + outdoorEdges + skybridgeEdges,
    skybridgeEdges,
    outdoor: outdoor
      ? {nodes: outdoor.nodeCount, edges: outdoor.edgeCount,
         networkEdges: outdoorNetworkEdges, sameBuildingEdges: sameFloorEdges}
      : {nodes: 0, edges: 0, networkEdges: 0, sameBuildingEdges: sameFloorEdges},
    buildMs: +(performance.now() - t0).toFixed(0),
  };
  return {route, buildingAt, stats};
}
