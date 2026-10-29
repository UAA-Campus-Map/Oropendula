/**
 * Room Pin: find the door of an exact room and return a point just outside it.
 *
 * Door selection is route-based, not geometric. We route from the room's label
 * to a set of seeded-random other rooms on the same floor and read off the FIRST
 * doorway each route crosses. A door is picked only when every probe that left
 * through a nearby door agrees on it; probes through no door (same open space)
 * are ignored. There is no "does this open onto a hallway" proof any more: a
 * door is trusted once it is the first crossing AND lies within DOOR_MAX_M of
 * the label. The old approach (probe a far door, prove no other doorway is
 * crossed) rejected most rooms because it demanded an 8 m probe and a
 * barrier-free route; every door is now assumed to open onto circulation space.
 *
 * Crossing is tolerant (see `crossBarrier`): the navGraph passage is carved at
 * the door leaf midpoint, a few decimetres off the wall-gap barrier, so an exact
 * intersection misses many doors.
 */
import { metresPerDegree, crossBarrier, nearestApproach } from './geo.mjs';

const DOOR_CLUSTER_M = 1.4;   // fallback clustering when a floor has no doorways
const DOOR_MAX_M = 15;        // label -> door proximity guard
const DOOR_PASS_M = 0.815;    // route this close to a barrier "goes through" it (ADA 32 in clear)
const DOOR_APART_M = 2;       // distinct first-doors this far apart = multiple doors
const PLACE_M = 0.5;          // how far past the door to place the user
const PROBES = 6;             // random same-floor probe rooms per lookup

function distM(a, b) {
  const m = metresPerDegree((a[1] + b[1]) / 2);
  return Math.hypot((a[0] - b[0]) * m.lon, (a[1] - b[1]) * m.lat);
}

/** Deterministic 32-bit hash, so a room's probe set is stable across runs. */
function hashSeed(str) {
  let h = 2166136261 >>> 0;
  for (let i = 0; i < str.length; i++) { h ^= str.charCodeAt(i); h = Math.imul(h, 16777619) >>> 0; }
  return h >>> 0;
}

/** Fisher-Yates shuffle driven by `seed`; returns a new array. */
function shuffled(arr, seed) {
  const a = arr.slice();
  let s = seed >>> 0;
  for (let i = a.length - 1; i > 0; i--) {
    s = (Math.imul(s, 1664525) + 1013904223) >>> 0;
    const j = s % (i + 1);
    [a[i], a[j]] = [a[j], a[i]];
  }
  return a;
}

/** Doorways when the loader supplied barrier geometry, else clustered leaves. */
export function floorDoorways(floor) {
  if (floor?.doorways?.length) return floor.doorways;
  const pts = floor?.doorPoints ?? [];
  if (!pts.length) return [];
  const m = metresPerDegree(pts[0][1]);
  const clusters = [];
  for (const p of pts) {
    let best = null, bestD = DOOR_CLUSTER_M;
    for (const c of clusters) {
      const d = Math.hypot((p[0] - c.ll[0]) * m.lon, (p[1] - c.ll[1]) * m.lat);
      if (d <= bestD) { bestD = d; best = c; }
    }
    if (best) {
      best.sumLon += p[0]; best.sumLat += p[1]; best.n++;
      best.ll = [best.sumLon / best.n, best.sumLat / best.n];
    } else {
      clusters.push({sumLon: p[0], sumLat: p[1], n: 1, ll: p});
    }
  }
  return clusters.map((c, i) => ({
    id: `legacy-${i}`, ll: c.ll, barrier: null, leaves: c.n, width: 0,
  }));
}

/**
 * The first doorway a route crosses on `floor`, in route order, with the
 * crossing point (`ll`) and the forward travel direction there (`dir`, metres).
 * Barriers use the tolerant `crossBarrier`; a legacy cluster without geometry
 * falls back to "the route passed within 0.6 m".
 */
export function firstDoorCrossing(route, floor) {
  const g = floor?.navGraph;
  if (!g) return null;
  const doors = floorDoorways(floor);
  let best = null;
  let walked = 0;
  for (const s of route?.segments ?? []) {
    if (!s.polyline?.length) continue;
    const pts = s.polyline.map(g.toM);
    const onFloor = s.tier === 'indoor' && s.floor === floor.floorId;
    if (onFloor) {
      for (const d of doors) {
        if (d.barrier) {
          const ba = g.toM(d.barrier[0]), bb = g.toM(d.barrier[1]);
          // Crossed either by crossing the barrier's line, or by passing through
          // the opening so close to the barrier that the route is beside it
          // (the navGraph passage is offset from the wall-gap barrier).
          let cand = null;
          const c = crossBarrier(pts, ba, bb);
          if (c) cand = {at: walked + c.at, dir: c.dir, ll: g.toLL(c.point)};
          const n = nearestApproach(pts, ba, bb);
          if (n && n.dist <= DOOR_PASS_M && (!cand || walked + n.at < cand.at)) {
            cand = {at: walked + n.at, dir: n.dir, ll: g.toLL(n.point)};
          }
          if (!cand) continue;
          if (!best || cand.at < best.at) best = {door: d, ...cand};
        } else {
          // Legacy cluster with no barrier geometry: nearest approach along the
          // segment (a small back-off counts as crossing).
          const pm = g.toM(d.ll);
          let ba = Infinity, bat = 0, bdir = null, bpt = null, w = 0;
          for (let i = 1; i < pts.length; i++) {
            const p0 = pts[i - 1], p1 = pts[i];
            const dx = p1[0] - p0[0], dy = p1[1] - p0[1];
            const l = Math.hypot(dx, dy), l2 = l * l;
            let t = l2 ? ((pm[0] - p0[0]) * dx + (pm[1] - p0[1]) * dy) / l2 : 0;
            t = Math.max(0, Math.min(1, t));
            const dist = Math.hypot(pm[0] - (p0[0] + t * dx), pm[1] - (p0[1] + t * dy));
            if (dist < ba) { ba = dist; bat = w + t * l; bdir = [dx / (l || 1), dy / (l || 1)]; bpt = [p0[0] + t * dx, p0[1] + t * dy]; }
            w += l;
          }
          if (ba <= 0.6) {
            const at = walked + bat;
            if (!best || at < best.at) best = {door: d, at, dir: bdir, ll: g.toLL(bpt)};
          }
        }
      }
    }
    for (let i = 1; i < pts.length; i++) walked += Math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1]);
  }
  return best;
}

/**
 * Probe targets for a room: up to `max` other room labels on the same floor,
 * drawn by a seeded shuffle so the set is spread across the floor and stable
 * across runs (unlike the old nearest-per-quadrant set, which clustered on the
 * room's own suite and produced interior-door false positives). Reachability is
 * checked by the caller, which is the only place a router is available.
 */
export function sampleTargets(floor, labelLL, seedText, max = PROBES) {
  const others = [];
  for (const l of floor?.roomLabels ?? []) {
    const ll = l.geometry.coordinates;
    if (!String(l.properties?.text ?? '').trim()) continue;
    if (Math.abs(ll[0] - labelLL[0]) < 1e-9 && Math.abs(ll[1] - labelLL[1]) < 1e-9) continue;
    others.push(ll);
  }
  return shuffled(others, hashSeed(seedText)).slice(0, max);
}

/**
 * Place the user just outside the doorway. Primary: continue the line from the
 * room's label THROUGH the door (the label is a stable inside-the-room
 * reference). If that line runs into a wall (a far, off-axis door - RH 210),
 * fall back to the barrier normal (away from the label), then the route
 * direction, then the nearest walkable point to the door.
 */
export function placementOutside(floor, door, dir, labelLL = null) {
  const g = floor.navGraph;
  const p = g.toM(door.ll);
  const step = (u) => {
    for (const d of [PLACE_M, 0.4, 0.3, 0.2]) {
      const ll = g.toLL([p[0] + u[0] * d, p[1] + u[1] * d]);
      if (g.isWalkable(ll)) return ll;
    }
    return null;
  };
  const lb = labelLL ? g.toM(labelLL) : null;
  if (lb) {
    const dx = p[0] - lb[0], dy = p[1] - lb[1];
    const l = Math.hypot(dx, dy);
    if (l > 1e-6) { const r = step([dx / l, dy / l]); if (r) return r; }
  }
  if (door.barrier) {
    const [a, b] = door.barrier.map(g.toM);
    const tx = b[0] - a[0], ty = b[1] - a[1];
    const ln = Math.hypot(tx, ty);
    if (ln > 0) {
      let n = [-ty / ln, tx / ln];
      if (lb && n[0] * (lb[0] - p[0]) + n[1] * (lb[1] - p[1]) >= 0) n = [-n[0], -n[1]];
      const r = step(n) ?? step([-n[0], -n[1]]);
      if (r) return r;
    }
  }
  if (dir) { const r = step(dir); if (r) return r; }
  return g.isWalkable(door.ll) ? door.ll : (g.nearestWalkable(door.ll) ?? null);
}

/**
 * @param {object} building
 * @param {{text, floor, labelLL}|{error}} hit - from `findRoomExact`
 * @param {object} campusRouter
 * @returns {{building, floor, room, doorLL, ll}|{error}}
 */
export function resolveRoomPin(building, hit, campusRouter) {
  if (hit.error) return hit;
  const floor = hit.floor;
  const candidates = sampleTargets(floor, hit.labelLL,
    `${building?.sourceId ?? building?.id ?? ''}|${floor.floorId}|${hit.text}`);
  if (!candidates.length) return {error: `Room ${hit.text} is the only room on its floor.`};
  if (!campusRouter) return {error: `Room ${hit.text} has no nearby doors.`};
  // Each probe's first nearby door. A room's own door is adjacent to it, so the
  // nearest distinct door wins; "multiple doors" is reported only when two
  // distinct doors are comparably close (i.e. genuinely ambiguous). Probes that
  // cross no door (same open space) are uninformative and ignored.
  const found = new Map();   // door id -> {cross, dist, votes}
  let probes = 0;
  for (const ll of candidates) {
    const route = campusRouter.route(hit.labelLL, ll, {
      startFloor: floor.floorId, endFloor: floor.floorId,
    });
    if (!route || !(route.segments ?? []).some((s) => s.tier === 'indoor' && s.floor === floor.floorId)) continue;
    probes++;
    const cross = firstDoorCrossing(route, floor);
    if (!cross) continue;                                             // same open space
    const dist = distM(cross.door.ll, hit.labelLL);
    if (dist > DOOR_MAX_M) continue;                                  // not this room's door
    const prev = found.get(cross.door.id);
    if (prev) { prev.votes++; if (dist < prev.dist) prev.dist = dist; }
    else found.set(cross.door.id, {cross, dist, votes: 1});
  }
  if (!probes) return {error: `Room ${hit.text} is the only room on its floor.`};
  if (!found.size) return {error: `Room ${hit.text} has no nearby doors.`};
  // Most probes leave through the majority first door; ties break on distance.
  const ranked = [...found.values()].sort((a, b) => b.votes - a.votes || a.dist - b.dist);
  // Compare every probe's first door. If two probes leave through genuinely
  // different openings - first doors at least DOOR_APART_M apart - the room is
  // ambiguous. Doors closer than that are one entrance (double door / two
  // leaves), so the majority door is used.
  for (let i = 0; i < ranked.length; i++) {
    for (let j = i + 1; j < ranked.length; j++) {
      if (distM(ranked[i].cross.door.ll, ranked[j].cross.door.ll) >= DOOR_APART_M) {
        return {error: `Room ${hit.text} has multiple doors.`};
      }
    }
  }
  const {door, dir} = ranked[0].cross;
  const ll = placementOutside(floor, door, dir, hit.labelLL);
  if (!ll) return {error: `Room ${hit.text}: couldn't find a spot outside the door.`};
  return {building, floor, room: hit.text, doorLL: door.ll, ll};
}
