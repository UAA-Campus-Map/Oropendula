/**
 * Best-fit rigid alignment for stacking floors in the import tool.
 *
 * A floor is aligned onto a reference floor (the building's ground floor) by
 * matching their exterior perimeters: the convex hull of each floor's wall
 * endpoints, sampled into points. The transform is rigid (rotation +
 * translation; scale is fixed at 1, both floors are inches), found by
 *
 *   1. seed   - elevator anchors when both floors have two (the shaft
 *               positions are vertically true), else a coarse rotation scan;
 *   2. refine - a few trimmed nearest-neighbour/Procrustes passes.
 *
 * Pure and unit-agnostic: points are [x, y] pairs in any common frame.
 */

/** Mean of a point set. */
export function meanPoint(pts) {
  let x = 0, y = 0;
  for (const p of pts) { x += p[0]; y += p[1]; }
  return [x / pts.length, y / pts.length];
}

/** Convex hull (monotone chain) of a point set. Returns [] for < 3 points. */
export function convexHull(points) {
  if (points.length < 3) return [];
  const pts = points.slice().sort((a, b) => a[0] - b[0] || a[1] - b[1]);
  const cross = (o, a, b) =>
    (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
  const lower = [];
  for (const p of pts) {
    while (lower.length >= 2 && cross(lower[lower.length - 2], lower[lower.length - 1], p) <= 0) lower.pop();
    lower.push(p);
  }
  const upper = [];
  for (let i = pts.length - 1; i >= 0; i--) {
    const p = pts[i];
    while (upper.length >= 2 && cross(upper[upper.length - 2], upper[upper.length - 1], p) <= 0) upper.pop();
    upper.push(p);
  }
  const hull = lower.slice(0, -1).concat(upper.slice(0, -1));
  return hull.length >= 3 ? hull : [];
}

/** Sample a closed polygon ring every ~`step` units. */
export function ringSamples(ring, step) {
  const out = [];
  for (let i = 0; i < ring.length; i++) {
    const [x0, y0] = ring[i], [x1, y1] = ring[(i + 1) % ring.length];
    const len = Math.hypot(x1 - x0, y1 - y0);
    const n = Math.max(1, Math.round(len / step));
    for (let k = 0; k < n; k++) {
      const t = k / n;
      out.push([x0 + (x1 - x0) * t, y0 + (y1 - y0) * t]);
    }
  }
  return out;
}

/** Perimeter samples of a point cloud: its convex hull, sampled every `step`. */
export function hullSamples(points, step) {
  const hull = convexHull(points);
  return hull.length ? ringSamples(hull, step) : [];
}

const apply = ([x, y], theta, [tx, ty]) => {
  const c = Math.cos(theta), s = Math.sin(theta);
  return [c * x - s * y + tx, s * x + c * y + ty];
};

/** Mean nearest-neighbour distance from `pts` to `target` (squared search). */
function meanNearest(pts, target) {
  let sum = 0;
  for (const p of pts) {
    let best = Infinity;
    for (const q of target) {
      const dx = p[0] - q[0], dy = p[1] - q[1];
      const d2 = dx * dx + dy * dy;
      if (d2 < best) best = d2;
    }
    sum += Math.sqrt(best);
  }
  return sum / pts.length;
}

/**
 * Best-fit rigid transform T(p) = R(theta) p + t mapping `current` onto
 * `reference`.
 *
 * @param {[number,number][]} current
 * @param {[number,number][]} reference
 * @param {{rotation?:number, currentAnchor?:number[], referenceAnchor?:number[]}|null} seed
 *        Optional starting rotation and anchor pair. Without it, a coarse 5°
 *        rotation scan with centroid matching picks the start.
 * @returns {{rotation:number, translation:number[], residual:number}|null}
 */
export function bestFitRigid(current, reference, seed = null) {
  if (!current?.length || !reference?.length) return null;
  const cCur = meanPoint(current), cRef = meanPoint(reference);

  let best = null;
  const tryTheta = (theta, anchorCur = cCur, anchorRef = cRef) => {
    // Translation that brings the anchor pair together under `theta`.
    const c = Math.cos(theta), s = Math.sin(theta);
    const t = [anchorRef[0] - (c * anchorCur[0] - s * anchorCur[1]),
               anchorRef[1] - (s * anchorCur[0] + c * anchorCur[1])];
    const score = meanNearest(current.map((p) => apply(p, theta, t)), reference);
    if (!best || score < best.score) best = { theta, t, score };
  };

  // Candidates: the optional seed first, then a coarse 5° scan. Taking the
  // better start matters - nearest-neighbour refinement can stall when a long
  // edge's correspondences slide along it, so the scan is always included even
  // when a seed exists.
  if (seed) {
    tryTheta(seed.rotation ?? 0, seed.currentAnchor ?? cCur, seed.referenceAnchor ?? cRef);
  }
  for (let deg = 0; deg < 360; deg += 5) tryTheta((deg * Math.PI) / 180);
  if (!best) return null;

  // Trimmed nearest-neighbour refinement: pair each current point with its
  // nearest reference point, drop the worst 20%, and re-fit.
  let { theta, t } = best;
  for (let pass = 0; pass < 5; pass++) {
    const pairs = current
      .map((p) => {
        const q = apply(p, theta, t);
        let bestQ = null, bestD = Infinity;
        for (const r of reference) {
          const dx = q[0] - r[0], dy = q[1] - r[1];
          const d2 = dx * dx + dy * dy;
          if (d2 < bestD) { bestD = d2; bestQ = r; }
        }
        return { q, r: bestQ, d: Math.sqrt(bestD) };
      })
      .sort((a, b) => a.d - b.d)
      .slice(0, Math.max(3, Math.ceil(current.length * 0.8)));

    const cq = meanPoint(pairs.map((p) => p.q));
    const cr = meanPoint(pairs.map((p) => p.r));
    let dot = 0, cross = 0;
    for (const { q, r } of pairs) {
      const qx = q[0] - cq[0], qy = q[1] - cq[1];
      const rx = r[0] - cr[0], ry = r[1] - cr[1];
      dot += qx * rx + qy * ry;
      cross += qx * ry - qy * rx;
    }
    const dTheta = Math.atan2(cross, dot);
    const c = Math.cos(dTheta), s = Math.sin(dTheta);
    const dT = [cr[0] - (c * cq[0] - s * cq[1]), cr[1] - (s * cq[0] + c * cq[1])];
    // Compose: T' = shift-after(T).
    theta += dTheta;
    t = [c * t[0] - s * t[1] + dT[0], s * t[0] + c * t[1] + dT[1]];
    if (Math.hypot(dT[0], dT[1]) < 0.05) break;
  }

  return {
    rotation: theta,
    translation: t,
    residual: meanNearest(current.map((p) => apply(p, theta, t)), reference),
  };
}
