/**
 * Room Pin: find the one door of an exact room that opens onto a hallway and
 * return the hallway point just outside it.
 *
 * The hallway test is geometric: a corridor point across the door's wall
 * barrier must route to a probe door without crossing any *other* doorway.
 * Merely passing near a different door in the corridor does not count, which
 * is what the old 0.6 m proximity test got wrong (ECB 208).
 */
import { metresPerDegree } from './geo.mjs';
import { barrierCrossesPolyline } from './doorways.mjs';

const DOOR_CLUSTER_M = 1.4;   // fallback clustering when a floor has no doorways
const DOOR_MAX_M = 15;        // label -> candidate door
const HALL_SCAN_M = 4;        // how far across the corridor to measure a centre
const PROBE_MIN_M = 8;        // a "far enough" different door to prove a corridor
const ENDPOINT_M = 1.2;       // a door this close to a route end is an endpoint

function distM(a, b) {
  const m = metresPerDegree((a[1] + b[1]) / 2);
  return Math.hypot((a[0] - b[0]) * m.lon, (a[1] - b[1]) * m.lat);
}

function distToSegment(p, a, b) {
  const m = metresPerDegree(p[1]);
  const px = p[0] * m.lon, py = p[1] * m.lat;
  const ax = a[0] * m.lon, ay = a[1] * m.lat;
  const bx = b[0] * m.lon, by = b[1] * m.lat;
  const dx = bx - ax, dy = by - ay;
  const len2 = dx * dx + dy * dy;
  const t = len2 > 0 ? Math.max(0, Math.min(1, ((px - ax) * dx + (py - ay) * dy) / len2)) : 0;
  return Math.hypot(px - (ax + t * dx), py - (ay + t * dy));
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

export function roomDoorCandidates(floor, labelLL) {
  const g = floor.navGraph;
  if (!g) return [];
  const from = g.isWalkable(labelLL) ? labelLL : g.nearestWalkable(labelLL);
  if (!from) return [];
  const m = metresPerDegree(from[1]);
  const out = [];
  for (const c of floorDoorways(floor)) {
    const door = g.isWalkable(c.ll) ? c.ll : g.nearestWalkable(c.ll);
    if (!door) continue;
    const dist = Math.hypot((door[0] - from[0]) * m.lon, (door[1] - from[1]) * m.lat);
    if (dist > DOOR_MAX_M) continue;
    let clear = true;
    const steps = Math.max(1, Math.ceil(dist / 0.2));
    for (let s = 1; s <= steps; s++) {
      const t = s / steps;
      if (!g.isWalkable([from[0] + (door[0] - from[0]) * t,
                         from[1] + (door[1] - from[1]) * t])) { clear = false; break; }
    }
    if (clear) out.push({...c, ll: door, dist});
  }
  return out;
}

export function corridorPoint(floor, labelLL, doorway, distance, dirOverride = null) {
  const g = floor.navGraph;
  const from = g.toM(labelLL), door = g.toM(doorway.ll);
  let nx, ny, px, py;
  if (doorway.barrier) {
    const ba = g.toM(doorway.barrier[0]), bb = g.toM(doorway.barrier[1]);
    let tx = bb[0] - ba[0], ty = bb[1] - ba[1];
    const tl = Math.hypot(tx, ty);
    if (!(tl > 0)) return null;
    tx /= tl; ty /= tl;
    nx = -ty; ny = tx;           // wall normal
    px = tx; py = ty;            // along the wall / corridor
  } else {
    let ux = door[0] - from[0], uy = door[1] - from[1];
    const len = Math.hypot(ux, uy);
    if (!(len > 0)) return null;
    ux /= len; uy /= len;
    nx = ux; ny = uy;
    px = -uy; py = ux;
  }
  // The room label is on the room side; the hallway is the other side.
  const labelSide = (from[0] - door[0]) * nx + (from[1] - door[1]) * ny;
  const dir = dirOverride ?? (labelSide >= 0 ? -1 : 1);
  const x = door[0] + nx * distance * dir;
  const y = door[1] + ny * distance * dir;
  const ll = g.toLL([x, y]);
  if (!g.isWalkable(ll)) return null;
  // Centre across the corridor.
  let neg = 0, pos = 0;
  for (let d = 0.25; d <= HALL_SCAN_M; d += 0.25) {
    const q = g.toLL([x + px * d, y + py * d]);
    if (!g.isWalkable(q)) break;
    pos = d;
  }
  for (let d = 0.25; d <= HALL_SCAN_M; d += 0.25) {
    const q = g.toLL([x - px * d, y - py * d]);
    if (!g.isWalkable(q)) break;
    neg = d;
  }
  const mid = (pos - neg) / 2;
  const centre = g.toLL([x + px * mid, y + py * mid]);
  return g.isWalkable(centre) ? centre : ll;
}

/** Doorways a route actually crosses. `except` holds doorways to ignore. */
export function crossedDoorways(route, floor, except = []) {
  const lines = (route?.segments ?? [])
    .filter((s) => s.tier === 'indoor' && s.polyline?.length)
    .map((s) => s.polyline);
  if (!lines.length) return null;
  const ignore = new Set(except.map((d) => d.id));
  const crossed = [];
  for (const c of floorDoorways(floor)) {
    if (ignore.has(c.id)) continue;
    let hit = false;
    if (c.barrier) {
      for (const line of lines) {
        if (barrierCrossesPolyline(c, line, floor.navGraph.toM)) { hit = true; break; }
      }
    } else {
      for (const line of lines) {
        for (let i = 1; i < line.length; i++) {
          if (distToSegment(c.ll, line[i - 1], line[i]) <= 0.6) { hit = true; break; }
        }
        if (hit) break;
      }
    }
    if (hit) crossed.push(c);
  }
  return crossed;
}

export function probeDoor(floor, fromLL, targetDoorway) {
  let best = null, bestD = Infinity;
  for (const c of floorDoorways(floor)) {
    if (distM(c.ll, targetDoorway.ll) < PROBE_MIN_M) continue;
    const dd = distM(c.ll, fromLL);
    if (dd < bestD) { bestD = dd; best = c; }
  }
  return best;
}

export function hallwayPointForDoor(floor, labelLL, doorway, campusRouter) {
  if (!campusRouter) return null;
  const probe = probeDoor(floor, labelLL, doorway);
  if (!probe) return null;
  const g = floor.navGraph;
  const probeLL = g.isWalkable(probe.ll) ? probe.ll : g.nearestWalkable(probe.ll);
  if (!probeLL) return null;
  const indoor = (r) => r && !(r.segments ?? []).some((s) => s.tier !== 'indoor');
  for (const distance of [1.0, 1.4, 1.8, 2.2, 2.6, 3.0]) {
    for (const dir of [null, 1, -1]) {
      const ll = corridorPoint(floor, labelLL, doorway, distance, dir);
      if (!ll) continue;
      // 1. The point must be outside the room: walking from the label to it
      //    crosses this door exactly once, and no other doorway.
      const fromLabel = campusRouter.route(labelLL, ll, {
        startFloor: floor.floorId, endFloor: floor.floorId,
      });
      if (!indoor(fromLabel)) continue;
      const crossed = crossedDoorways(fromLabel, floor);
      if (crossed?.length !== 1 || crossed[0].id !== doorway.id) continue;
      // 2. From there it must reach a different door without crossing any
      //    doorway (the target or otherwise).
      const toProbe = campusRouter.route(ll, probeLL, {
        startFloor: floor.floorId, endFloor: floor.floorId,
      });
      if (!indoor(toProbe)) continue;
      if ((crossedDoorways(toProbe, floor, [probe]) ?? []).length) continue;
      return ll;
    }
  }
  return null;
}

/**
 * @param {object} building
 * @param {{text, floor, labelLL}|{error}} hit - from `findRoomExact`
 * @param {object} campusRouter
 * @returns {{building, floor, room, doorLL, ll}|{error}}
 */
export function resolveRoomPin(building, hit, campusRouter) {
  if (hit.error) return hit;
  const doors = roomDoorCandidates(hit.floor, hit.labelLL);
  if (!doors.length) return {error: `Room ${hit.text} has no door connected to its space.`};
  const hallway = [];
  for (const d of doors) {
    const ll = hallwayPointForDoor(hit.floor, hit.labelLL, d, campusRouter);
    if (ll) hallway.push({...d, hallwayLL: ll});
  }
  if (!hallway.length) {
    return {error: `Room ${hit.text} has no door that opens onto a hallway.`};
  }
  if (hallway.length > 1) {
    return {error: `Room ${hit.text} has multiple doors that open onto a hallway.`};
  }
  return {
    building, floor: hit.floor, room: hit.text,
    doorLL: hallway[0].ll, ll: hallway[0].hallwayLL,
  };
}
