/**
 * Doorway geometry shared by Room Pin, route blocking and place lookup.
 *
 * A DOOR-layer shape is one leaf. The extractor draws leaves either closed in
 * the wall or swung open. The opening itself is the wall gap, so a route
 * barrier must follow the wall, not the open leaf. This module turns each leaf
 * into a wall-aligned barrier (using its swing arc when present), clusters
 * adjacent leaves (double doors) into one doorway, and gives every doorway a
 * stable geometry-derived id.
 */
import { shapeSegments, isWallLayer, isDoorLayer } from './campus.mjs';
import { segmentsIntersect } from './geo.mjs';

const M = 0.0254;              // DXF inches -> metres
const HINGE_M = 0.30;          // leaf endpoint this close to a wall = hinge
const CONTACT_M = 0.12;        // a wall "through" the hinge
const JAMB_MIN_M = 0.45;       // shortest believable door opening
const JAMB_MAX_M = 2.0;        // longest believable door opening
const CLUSTER_M = 1.4;         // leaf-centre distance for one doorway
const CLUSTER_GAP_M = 0.4;     // gap between leaf barriers in one doorway
const PARALLEL_COS = Math.cos(30 * Math.PI / 180);
const WALL_CELL_U = 100;       // ~2.54 m wall index cell (DXF inches)

function dist(a, b) {
  return Math.hypot(a[0] - b[0], a[1] - b[1]);
}

function distToSeg(p, a, b) {
  const dx = b[0] - a[0], dy = b[1] - a[1];
  const len2 = dx * dx + dy * dy;
  let t = len2 ? ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / len2 : 0;
  t = t < 0 ? 0 : t > 1 ? 1 : t;
  return Math.hypot(p[0] - (a[0] + t * dx), p[1] - (a[1] + t * dy));
}

function unit(a, b) {
  const dx = b[0] - a[0], dy = b[1] - a[1];
  const l = Math.hypot(dx, dy);
  return l > 0 ? [dx / l, dy / l] : null;
}

function longestEdge(shape) {
  let best = null, bestLen = 0;
  for (const [a, b] of shapeSegments(shape)) {
    const len = dist(a, b);
    if (len > bestLen) { bestLen = len; best = [a, b]; }
  }
  return best ? { a: best[0], b: best[1], len: bestLen } : null;
}

/** Uniform grid over wall segments, so a leaf only looks at nearby walls. */
function wallIndex(walls) {
  const cells = new Map();
  const key = (cx, cy) => `${cx}:${cy}`;
  for (let i = 0; i < walls.length; i++) {
    const [a, b] = walls[i];
    const x0 = Math.floor((Math.min(a[0], b[0]) - 1) / WALL_CELL_U);
    const x1 = Math.floor((Math.max(a[0], b[0]) + 1) / WALL_CELL_U);
    const y0 = Math.floor((Math.min(a[1], b[1]) - 1) / WALL_CELL_U);
    const y1 = Math.floor((Math.max(a[1], b[1]) + 1) / WALL_CELL_U);
    for (let cx = x0; cx <= x1; cx++) for (let cy = y0; cy <= y1; cy++) {
      const k = key(cx, cy);
      let list = cells.get(k);
      if (!list) cells.set(k, list = []);
      list.push(i);
    }
  }
  return {
    near(p, radiusU) {
      const x0 = Math.floor((p[0] - radiusU) / WALL_CELL_U);
      const x1 = Math.floor((p[0] + radiusU) / WALL_CELL_U);
      const y0 = Math.floor((p[1] - radiusU) / WALL_CELL_U);
      const y1 = Math.floor((p[1] + radiusU) / WALL_CELL_U);
      const seen = new Set();
      const out = [];
      for (let cx = x0; cx <= x1; cx++) for (let cy = y0; cy <= y1; cy++) {
        for (const i of cells.get(key(cx, cy)) ?? []) {
          if (seen.has(i)) continue;
          seen.add(i);
          const d = distToSeg(p, walls[i][0], walls[i][1]);
          if (d <= radiusU) out.push({ i, d, seg: walls[i] });
        }
      }
      return out.sort((a, b) => a.d - b.d);
    },
  };
}

function nearestWall(p, index) {
  return index.near(p, HINGE_M / M)[0] ?? null;
}

/** Unique endpoints of nearby wall segments. */
function endpointsOf(near) {
  const seen = new Set();
  const out = [];
  for (const { seg } of near) {
    for (const q of [seg[0], seg[1]]) {
      const k = `${q[0].toFixed(2)}:${q[1].toFixed(2)}`;
      if (seen.has(k)) continue;
      seen.add(k);
      out.push(q);
    }
  }
  return out;
}

function leafBarrier(shape, walls, index) {
  const leaf = longestEdge(shape);
  if (!leaf) return null;
  const { a, b, len } = leaf;
  if (len < JAMB_MIN_M / M || len > JAMB_MAX_M / M) return null;
  const leafU = unit(a, b);

  const arcs = (shape.items ?? []).filter((it) => it.op === 'arc' && it.radius > 0
    && it.start_angle !== undefined && it.end_angle !== undefined);

  // A swing arc names both the closed and open directions; the wall is the
  // arc direction the leaf is NOT drawn along.
  if (arcs.length && leafU) {
    const arc = arcs[0];
    const dirs = [arc.start_angle, arc.end_angle].map((deg) => {
      const r = deg * Math.PI / 180;
      return [Math.cos(r), Math.sin(r)];
    });
    const wallDir = dirs.sort((p, q) =>
      Math.abs(leafU[0] * p[0] + leafU[1] * p[1]) - Math.abs(leafU[0] * q[0] + leafU[1] * q[1]))[0];
    const hinge = arc.center;
    const lenU = arc.radius;
    // The arc gives the wall line but not which jamb; use the closer wall
    // endpoint (the other side of the opening) when one is in range.
    const near = index.near(hinge, (JAMB_MAX_M + 0.2) / M);
    let best = null, bestErr = Infinity;
    for (const sign of [1, -1]) {
      const u = [wallDir[0] * sign, wallDir[1] * sign];
      for (const q of endpointsOf(near)) {
        const d = Math.hypot(q[0] - hinge[0], q[1] - hinge[1]);
        if (d < 0.45 / M || d > 2.0 / M) continue;
        const vx = (q[0] - hinge[0]) / d, vy = (q[1] - hinge[1]) / d;
        if (Math.abs(u[0] * vx + u[1] * vy) < 0.98) continue;
        const score = Math.abs(d - lenU);
        if (score < bestErr) { bestErr = score; best = q; }
      }
    }
    return {
      a: hinge,
      b: best ?? [hinge[0] + wallDir[0] * lenU, hinge[1] + wallDir[1] * lenU],
      source: 'arc',
    };
  }

  const wa = nearestWall(a, index);
  const wb = nearestWall(b, index);
  const aHinge = !!wa, bHinge = !!wb;

  // Closed leaf: both ends are in the wall; the leaf itself spans the opening.
  if (aHinge && bHinge) return { a, b, source: 'leaf' };

  if (aHinge || bHinge) {
    const hinge = aHinge ? a : b;
    // Choose the wall direction most perpendicular to the open leaf.
    const near = index.near(hinge, 1.2 / M);
    let wallDir = null, bestPerp = -1;
    for (const { d, seg } of near) {
      if (d > CONTACT_M / M) continue;
      const u = unit(seg[0], seg[1]);
      if (!u) continue;
      const perp = Math.abs(leafU[0] * u[1] - leafU[1] * u[0]);
      if (perp > bestPerp) { bestPerp = perp; wallDir = u; }
    }
    if (wallDir) {
      // Try both directions along the wall for the opposite jamb.
      let best = null, bestErr = Infinity;
      for (const sign of [1, -1]) {
        const u = [wallDir[0] * sign, wallDir[1] * sign];
        for (const q of endpointsOf(near)) {
          if (dist(hinge, q) < JAMB_MIN_M / M || dist(hinge, q) > JAMB_MAX_M / M) continue;
          const v = unit(hinge, q);
          if (!v) continue;
          if (Math.abs(u[0] * v[0] + u[1] * v[1]) < PARALLEL_COS) continue;
          const err = Math.abs(dist(hinge, q) - len);
          if (err < bestErr) { bestErr = err; best = q; }
        }
      }
      if (best) return { a: hinge, b: best, source: 'jamb' };
      return {
        a: hinge,
        b: [hinge[0] + wallDir[0] * len, hinge[1] + wallDir[1] * len],
        source: 'wall',
      };
    }
  }

  // No hinge contact: project the leaf centre onto the nearest wall and make
  // a barrier of the leaf's length, aligned with the wall.
  const mid = [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2];
  const near = nearestWall(mid, index);
  if (!near) return { a, b, source: 'leaf' };
  const [w0, w1] = near.seg;
  const u = unit(w0, w1);
  if (!u) return { a, b, source: 'leaf' };
  const t = (mid[0] - w0[0]) * u[0] + (mid[1] - w0[1]) * u[1];
  const cx = w0[0] + u[0] * t, cy = w0[1] + u[1] * t;
  return {
    a: [cx - u[0] * len / 2, cy - u[1] * len / 2],
    b: [cx + u[0] * len / 2, cy + u[1] * len / 2],
    source: 'projected',
  };
}

function barrierGap(x, y) {
  return Math.min(
    dist(x.a, y.a), dist(x.a, y.b),
    dist(x.b, y.a), dist(x.b, y.b),
  );
}

function parallel(x, y) {
  const ax = x.b[0] - x.a[0], ay = x.b[1] - x.a[1];
  const bx = y.b[0] - y.a[0], by = y.b[1] - y.a[1];
  const al = Math.hypot(ax, ay) || 1, bl = Math.hypot(bx, by) || 1;
  return Math.abs((ax * bx + ay * by) / (al * bl)) >= PARALLEL_COS;
}

function combine(group) {
  const first = group[0];
  const u = unit(first.a, first.b);
  if (!u) return first;
  const n = [-u[1], u[0]];
  let minU = Infinity, maxU = -Infinity, sumV = 0, count = 0;
  for (const g of group) {
    for (const p of [g.a, g.b]) {
      const along = p[0] * u[0] + p[1] * u[1];
      const across = p[0] * n[0] + p[1] * n[1];
      minU = Math.min(minU, along); maxU = Math.max(maxU, along);
      sumV += across; count++;
    }
  }
  const across = sumV / count;
  return {
    a: [u[0] * minU + n[0] * across, u[1] * minU + n[1] * across],
    b: [u[0] * maxU + n[0] * across, u[1] * maxU + n[1] * across],
    source: group.map((g) => g.source).join('+'),
  };
}

function stableId(barrier) {
  const s = [barrier.a, barrier.b]
    .flat()
    .map((v) => Math.round(v * 100))
    .join(',');
  let h = 2166136261;
  for (let i = 0; i < s.length; i++) {
    h ^= s.charCodeAt(i);
    h = Math.imul(h, 16777619);
  }
  return `d${(h >>> 0).toString(36)}`;
}

/**
 * Extract doorways from one floor's raw shapes.
 *
 * @param {object[]} shapes - parsed floor JSON shapes
 * @param {(x:number,y:number)=>number[]} toLL - DXF -> lon/lat
 * @returns {Array<{id,ll,barrier,width,leaves,dxfBarrier}>}
 */
export function doorwaysFromShapes(shapes, toLL) {
  const walls = (shapes ?? []).filter((s) => isWallLayer(s.layer)).flatMap(shapeSegments);
  const index = wallIndex(walls);
  const leaves = (shapes ?? [])
    .filter((s) => isDoorLayer(s.layer))
    .map((s) => leafBarrier(s, walls, index))
    .filter(Boolean);

  // Union-find over leaf barriers: adjacent leaves of one double door belong
  // to the same doorway; two separate doors along a corridor do not.
  const parent = leaves.map((_, i) => i);
  const find = (i) => { while (parent[i] !== i) { parent[i] = parent[parent[i]]; i = parent[i]; } return i; };
  for (let i = 0; i < leaves.length; i++) {
    const mi = [(leaves[i].a[0] + leaves[i].b[0]) / 2, (leaves[i].a[1] + leaves[i].b[1]) / 2];
    for (let j = i + 1; j < leaves.length; j++) {
      const mj = [(leaves[j].a[0] + leaves[j].b[0]) / 2, (leaves[j].a[1] + leaves[j].b[1]) / 2];
      if (dist(mi, mj) > CLUSTER_M / M) continue;
      if (!parallel(leaves[i], leaves[j])) continue;
      if (barrierGap(leaves[i], leaves[j]) > CLUSTER_GAP_M / M) continue;
      parent[find(i)] = find(j);
    }
  }
  const groups = new Map();
  for (let i = 0; i < leaves.length; i++) {
    const r = find(i);
    if (!groups.has(r)) groups.set(r, []);
    groups.get(r).push(leaves[i]);
  }

  return [...groups.values()].map((group) => {
    const b = combine(group);
    const width = dist(b.a, b.b) * M;
    return {
      id: stableId(b),
      ll: toLL((b.a[0] + b.b[0]) / 2, (b.a[1] + b.b[1]) / 2),
      barrier: [toLL(b.a[0], b.a[1]), toLL(b.b[0], b.b[1])],
      width,
      leaves: group.length,
      dxfBarrier: [b.a, b.b],
    };
  });
}

/** True when a route polyline crosses a doorway's barrier. */
export function barrierCrossesPolyline(doorway, polyline, toM = (p) => p) {
  if (!doorway?.barrier || !polyline || polyline.length < 2) return false;
  const [a, b] = doorway.barrier.map(toM);
  const pts = polyline.map(toM);
  for (let i = 1; i < pts.length; i++) {
    if (segmentsIntersect(pts[i - 1], pts[i], a, b)) return true;
  }
  return false;
}
