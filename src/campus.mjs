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
 * of a way that a tile edge clipped apart. Each building exit snaps to the
 * nearest node it can actually walk to. Without a path network the router
 * falls back to straight-line outdoor legs.
 *
 * All routing costs are time in seconds. Distances are metres.
 */

/** Walking speeds, metres per second. */
const INDOOR_SPEED = 1.3;
const OUTDOOR_SPEED = 1.4;
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
  const doorShapes = shapes.filter(s => PORTAL_LAYERS.doors.has(s.layer.toLowerCase()));
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
   * Proper segment intersection test with an endpoint margin: crossings
   * within `margin` metres of either segment's end don't count, so edges
   * that merely meet a wall at a doorway or a corner pass through.
   */
  const crosses = (ax, ay, bx, by, margin) => {
    const query = ++stamp;
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
          if (mCD <= margin || mCD >= lenCD - margin) continue;
          return true;
        }
      }
    }
    return false;
  };

  return {crosses, segCount: segs.length};
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
    adj[a].push([b, metres]);
    adj[b].push([a, metres]);
  };

  let edgeCount = 0;
  const obstacles = obstacleSegs?.length ? buildObstacleIndex(obstacleSegs, scale) : null;
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
        adj[a].push([b, metres]);
        adj[b].push([a, metres]);
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
   * Nearest graph node reachable from `ll` without cuttting through a
   * building, within `maxM` metres; null when there is none. Used to attach
   * exits and outdoor clicks - the connecting stub must stay outdoors too.
   */
  const snapClear = (ll, maxM = OUTDOOR_MAX_SNAP_M) => {
    const x = ll[0] * scale.lon, y = ll[1] * scale.lat;
    let best = -1, bestD = Infinity;
    for (let i = 0; i < nodes.length; i++) {
      const d = distLL(ll, nodes[i], scale);
      if (d >= bestD || d > maxM) continue;
      const N = nodes[i];
      if (obstacles && obstacles.crosses(x, y, N[0] * scale.lon, N[1] * scale.lat, 2.0)) continue;
      bestD = d;
      best = i;
    }
    return best === -1 ? null : {node: best, metres: bestD};
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
 * @returns {{route: Function, stats: object}}
 */
export function buildCampusRouter(buildings, outdoorShapes = null) {
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
    // seed with ground-floor connectors
    for (const p of ground.elevators) {
      byKind.elevator.set(p.id, {kind: 'elevator', building: b.id, stops: new Map([[ground.floorId, p.ll]])});
    }
    for (const p of ground.stairs) {
      byKind.stairs.set(p.id, {kind: 'stairs', building: b.id, stops: new Map([[ground.floorId, p.ll]])});
    }
    // attach stops from other floors to the nearest ground-floor connector
    for (const f of b.floors) {
      if (f.floorId === b.groundFloor) continue;
      for (const kind of ['elevator', 'stairs']) {
        const list = f[kind === 'elevator' ? 'elevators' : 'stairs'];
        for (const p of list) {
          let best = null, bestD = Infinity;
          for (const c of byKind[kind].values()) {
            const gll = c.stops.get(b.groundFloor);
            const d = distLL(gll, p.ll, scale);
            if (d < bestD) { bestD = d; best = c; }
          }
          if (best && bestD <= CONNECTOR_LINK_M) best.stops.set(f.floorId, p.ll);
          else byKind[kind].set(`${kind}-x${byKind[kind].size}`, {kind, building: b.id, stops: new Map([[f.floorId, p.ll]])});
        }
      }
    }
    b._connectors = [...byKind.elevator.values(), ...byKind.stairs.values()];
  }

  // --- global exit list, outdoor path network and macro graph ---------------
  // Macro node: one building exit. Static edges:
  //   same building  -> indoor ground-floor path length (seconds)
  //   other building -> outdoor path-network length (seconds), straight line
  //                     when no network was supplied
  const exits = [];
  for (const b of buildings) {
    const ground = b.floors.find(f => f.floorId === b.groundFloor);
    for (const e of ground.exits) exits.push({building: b, floor: ground, ll: e.ll, id: e.id});
  }
  const scale = metresPerDeg(exits[0]?.ll[1] ?? 61.19);
  const N = exits.length;

  // Outdoor network: one snap + one Dijkstra run per exit. Every macro
  // cross-building edge reads its cost off those runs, and the same runs
  // reconstruct the polyline for whichever chain the route picks. Snaps use
  // snapClear so the exit -> node stub cannot cut through a building.
  // obstacleLines are the ground-floor wall segments (WALLS + Windows +
  // partitions, in lon/lat) each floor provides.
  const obstacleSegs = [];
  for (const b of buildings) {
    const ground = b.floors.find(f => f.floorId === b.groundFloor);
    for (const seg of ground?.obstacleLines ?? []) obstacleSegs.push(seg);
  }
  const outdoor = outdoorShapes?.length
    ? buildOutdoorGraph(outdoorShapes, exits[0]?.ll[1] ?? 61.19, obstacleSegs)
    : null;
  const outdoorRuns = exits.map(e => {
    if (!outdoor) return null;
    const s = outdoor.snapClear(e.ll);
    return s ? {snap: s, run: outdoor.dijkstraFrom(s.node)} : null;
  });

  const macroAdj = Array.from({length: N}, () => []);
  let sameBuildingEdges = 0, outdoorEdges = 0, outdoorNetworkEdges = 0;
  for (let i = 0; i < N; i++) {
    for (let j = i + 1; j < N; j++) {
      let sec = null;
      if (exits[i].building === exits[j].building) {
        const p = exits[i].floor.navGraph.findPath(exits[i].ll, exits[j].ll);
        if (p) { sec = polyLength(p, scale) / INDOOR_SPEED; sameBuildingEdges++; }
      } else {
        outdoorEdges++;
        const a = outdoorRuns[i], b = outdoorRuns[j];
        if (a && b) {
          const metres = a.run.dist[b.snap.node] + a.snap.metres + b.snap.metres;
          if (isFinite(metres)) {
            sec = metres / OUTDOOR_SPEED;
            outdoorNetworkEdges++;
          }
        }
        if (sec === null) {
          // straight-line fallback only when it stays clear of buildings
          if (!outdoor || outdoor.segmentClear(exits[i].ll, exits[j].ll, 2)) {
            sec = distLL(exits[i].ll, exits[j].ll, scale) / OUTDOOR_SPEED;
          }
        }
      }
      if (sec !== null) {
        macroAdj[i].push([j, sec]);
        macroAdj[j].push([i, sec]);
      }
    }
  }

  // --- helpers -------------------------------------------------------------
  const floorNum = id => parseInt(id, 10) || 0;

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
   * Cost in seconds from `point` on `floor` to every exit of `building`,
   * going through any connector reachable on that floor. Ground-floor
   * starts can also walk straight to an exit without a vertical leg.
   * @returns {Map<exitIndex, {sec, prefix: segment[]}>} | null if none reachable
   */
  function originCosts(building, floor, point) {
    const ground = building.floors.find(f => f.floorId === building.groundFloor);
    if (!ground) return null;
    const res = new Map();
    const pushExit = (eIdx, sec, prefix) => {
      const cur = res.get(eIdx);
      if (!cur || sec < cur.sec) res.set(eIdx, {sec, prefix});
    };

    for (let ei = 0; ei < exits.length; ei++) {
      const e = exits[ei];
      if (e.building !== building) continue;
      // direct walk (only when starting on the ground floor)
      if (floor.floorId === building.groundFloor) {
        const p = indoorPath(floor.navGraph, point, e.ll);
        if (p) pushExit(ei, polyLength(p, scale) / INDOOR_SPEED, [
          {tier: 'indoor', building: building.id, floor: floor.floorId, polyline: p},
        ]);
      }
    }

    for (const c of building._connectors) {
      const stopHere = c.stops.get(floor.floorId);
      const stopGround = c.stops.get(building.groundFloor);
      if (!stopHere || !stopGround) continue;
      const leg1 = indoorPath(floor.navGraph, point, stopHere);
      if (!leg1) continue;
      const sec1 = polyLength(leg1, scale) / INDOOR_SPEED;
      const vSec = verticalSeconds(c, floor.floorId, building.groundFloor);
      const prefix = [{tier: 'indoor', building: building.id, floor: floor.floorId, polyline: leg1}];
      if (floor.floorId !== building.groundFloor) {
        prefix.push({tier: c.kind, building: building.id, fromFloor: floor.floorId, toFloor: building.groundFloor});
      }
      for (let ei = 0; ei < exits.length; ei++) {
        const e = exits[ei];
        if (e.building !== building) continue;
        const leg2 = indoorPath(ground.navGraph, stopGround, e.ll);
        if (!leg2) continue;
        const sec2 = polyLength(leg2, scale) / INDOOR_SPEED;
        pushExit(ei, sec1 + vSec + sec2, [
          ...prefix,
          {tier: 'indoor', building: building.id, floor: building.groundFloor, polyline: leg2},
        ]);
      }
    }
    return res.size ? res : null;
  }

  /**
   * Attachment for a click that lands outside every building: snap to the
   * nearest outdoor path node, walk the network to each exit, and fall back
   * to a straight line when there is no network or no connection.
   * One Dijkstra run serves every exit; results per exit are
   * {sec, polyline} where polyline runs click -> exit.
   * @returns {Map<number, {sec:number, polyline:number[][]}>}
   */
  function outdoorAttachment(ll) {
    const res = new Map();
    const clickSnap = outdoor ? outdoor.snapClear(ll) : null;
    const from = clickSnap ? outdoor.dijkstraFrom(clickSnap.node) : null;
    for (let ei = 0; ei < N; ei++) {
      const r = outdoorRuns[ei];
      if (from && r) {
        const p = outdoor.pathOf(from, r.snap.node);
        if (p) {
          const poly = [ll, ...p, exits[ei].ll];
          res.set(ei, {sec: polyLength(poly, scale) / OUTDOOR_SPEED, polyline: poly});
          continue;
        }
      }
      // straight-line fallback, unless it would cut through a building
      if (outdoor && !outdoor.segmentClear(ll, exits[ei].ll, 2)) continue;
      const poly = [ll, exits[ei].ll];
      res.set(ei, {sec: polyLength(poly, scale) / OUTDOOR_SPEED, polyline: poly});
    }
    return res;
  }

  /** Mirror of originCosts: exits of `building` -> `point` on `floor`. */
  function destinationCosts(building, floor, point) {
    const ground = building.floors.find(f => f.floorId === building.groundFloor);
    if (!ground) return null;
    const res = new Map();
    const pushExit = (eIdx, sec, suffix) => {
      const cur = res.get(eIdx);
      if (!cur || sec < cur.sec) res.set(eIdx, {sec, suffix});
    };

    for (let ei = 0; ei < exits.length; ei++) {
      const e = exits[ei];
      if (e.building !== building) continue;
      if (floor.floorId === building.groundFloor) {
        const p = indoorPath(floor.navGraph, e.ll, point);
        if (p) pushExit(ei, polyLength(p, scale) / INDOOR_SPEED, [
          {tier: 'indoor', building: building.id, floor: floor.floorId, polyline: p},
        ]);
      }
    }

    for (const c of building._connectors) {
      const stopHere = c.stops.get(floor.floorId);
      const stopGround = c.stops.get(building.groundFloor);
      if (!stopHere || !stopGround) continue;
      const leg2 = indoorPath(floor.navGraph, stopHere, point);
      if (!leg2) continue;
      const sec2 = polyLength(leg2, scale) / INDOOR_SPEED;
      const vSec = verticalSeconds(c, building.groundFloor, floor.floorId);
      const suffixTail = [];
      if (floor.floorId !== building.groundFloor) {
        suffixTail.push({tier: c.kind, building: building.id, fromFloor: building.groundFloor, toFloor: floor.floorId});
      }
      suffixTail.push({tier: 'indoor', building: building.id, floor: floor.floorId, polyline: leg2});
      for (let ei = 0; ei < exits.length; ei++) {
        const e = exits[ei];
        if (e.building !== building) continue;
        const leg1 = indoorPath(ground.navGraph, e.ll, stopGround);
        if (!leg1) continue;
        const sec1 = polyLength(leg1, scale) / INDOOR_SPEED;
        pushExit(ei, sec1 + vSec + sec2, [
          {tier: 'indoor', building: building.id, floor: building.groundFloor, polyline: leg1},
          ...suffixTail,
        ]);
      }
    }
    return res.size ? res : null;
  }

  /** Which building (if any) contains this point on this floor? */
  function buildingAt(ll, floorId) {
    for (const b of buildings) {
      const f = b.floors.find(x => x.floorId === floorId) ??
                b.floors.find(x => x.floorId === b.groundFloor);
      // indoorPath's retry ring lets points that sit marginally inside a
      // wall (door jambs, portal points) still classify as indoor.
      if (f && indoorPath(f.navGraph, ll, ll)) return {building: b, floor: f};
    }
    return null;
  }

  /**
   * Route between two [lon,lat] points across buildings and floors.
   *
   * @param {number[]} startLL
   * @param {number[]} endLL
   * @param {{startFloor?:string, endFloor?:string}} [opts] floor hints;
   *        default to the ground floor of whichever building the point is in.
   * @returns {{segments:object[], totalSeconds:number, totalMeters:number,
   *            summary:string}|null}
   */
  function route(startLL, endLL, opts = {}) {
    const startHit = buildingAt(startLL, opts.startFloor ?? buildings[0].groundFloor);
    const endHit = buildingAt(endLL, opts.endFloor ?? buildings[0].groundFloor);

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
    let origin = null, dest = null;

    if (startHit) {
      origin = originCosts(startHit.building, startHit.floor, startLL);
    } else {
      // outdoors: walk the OSM network from the click to every exit
      origin = outdoorAttachment(startLL);
    }
    if (endHit) {
      dest = destinationCosts(endHit.building, endHit.floor, endLL);
    } else {
      dest = outdoorAttachment(endLL);
    }
    if (!origin || !dest) return null;

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
    if (!done[SNK]) return null;

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

    // prefix: start point -> first exit
    const firstExitIdx = chain[1];
    const o = origin.get(firstExitIdx);
    if (startHit) {
      for (const seg of o.prefix) seg.tier === 'elevator' || seg.tier === 'stairs' ? addVertical(seg) : addPoly(seg);
    } else {
      addOutdoorPoly(o.polyline);
    }

    // macro chain legs
    for (let k = 1; k < chain.length - 2; k++) {
      const a = chain[k], b = chain[k + 1];
      if (exits[a].building === exits[b].building) {
        const p = exits[a].floor.navGraph.findPath(exits[a].ll, exits[b].ll);
        addPoly({tier: 'indoor', building: exits[a].building.id, floor: exits[b].floor.floorId, polyline: p});
      } else {
        // outdoor: exit stub -> path network -> exit stub
        const ra = outdoorRuns[a], rb = outdoorRuns[b];
        let poly = [exits[a].ll, exits[b].ll];
        if (ra && rb) {
          const p = outdoor.pathOf(ra.run, rb.snap.node);
          if (p) poly = [exits[a].ll, ...p, exits[b].ll];
        }
        addOutdoorPoly(poly);
      }
    }

    // suffix: last exit -> end point
    const lastExitIdx = chain[chain.length - 2];
    const d = dest.get(lastExitIdx);
    if (endHit) {
      for (const seg of d.suffix) seg.tier === 'elevator' || seg.tier === 'stairs' ? addVertical(seg) : addPoly(seg);
    } else {
      addOutdoorPoly(d.polyline);
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

  const stats = {
    buildings: buildings.length,
    exits: N,
    macroEdges: sameBuildingEdges + outdoorEdges,
    outdoor: outdoor
      ? {nodes: outdoor.nodeCount, edges: outdoor.edgeCount,
         networkEdges: outdoorNetworkEdges, sameBuildingEdges}
      : {nodes: 0, edges: 0, networkEdges: 0, sameBuildingEdges},
    buildMs: +(performance.now() - t0).toFixed(0),
  };
  return {route, buildingAt, stats};
}
