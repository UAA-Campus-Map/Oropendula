/**
 * Places of interest: room lookup and nickname search. No DOM code, so
 * app.html, places.html and smoke.mjs share it.
 *
 * A place is one entry of places.json:
 *   { id, names: [...], category?, building, room }   a room on a floor plan
 *   { id, names: [...], category?, ll: [lon, lat], floor }   any other point
 * `building` is a search code (the floor JSON `ID`, e.g. SU, or the
 * registration id, e.g. AS119); `room` is a room label exactly as drawn.
 * Student favorites have the same shape with a single `name`.
 */
import { metresPerDegree } from './geo.mjs';

const floorLevelOf = (floorId) => parseInt(floorId, 10) || 1;

// --- rooms ------------------------------------------------------------------

/** Room-label normalisation (case and spaces/dashes ignored). */
export const normRoom = (s) => String(s).toLowerCase().replace(/[\s\-]+/g, '');

/** The building whose UAA code (`sourceId`, BMH) or registration id (AS103) is `code`. */
export function findBuilding(buildings, code) {
  const c = String(code ?? '').toLowerCase();
  if (!c) return null;
  return buildings.find((b) => b.sourceId?.toLowerCase() === c || b.id.toLowerCase() === c) ?? null;
}

/**
 * Split "BMH 114A" into { building, roomText }. The longest known code at the
 * start of the query wins, so "BMH 114A", "BMH114A" and "AS103114" all split
 * correctly. Returns null when no building code starts the query.
 */
export function splitRoomQuery(buildings, q) {
  const ql = q.toLowerCase();
  const candidates = [];
  for (const b of buildings) {
    if (b.sourceId) candidates.push({b, key: b.sourceId.toLowerCase()});
    candidates.push({b, key: b.id.toLowerCase()});
  }
  candidates.sort((a, c) => c.key.length - a.key.length);
  const hit = candidates.find((c) => ql.startsWith(c.key));
  if (!hit) return null;
  const roomText = q.slice(hit.key.length).replace(/^[\s,:;-]+/, '').trim();
  return {building: hit.b, roomText};
}

/**
 * The room's DOORWAY: the nearest door (DOORS-layer midpoint) that a
 * straight walk from the label reaches without crossing a wall. A segment
 * label->door that stays walkable means they share one open space, so the
 * door belongs to this room (not an adjacent one across a wall). Falls back
 * to the label when the floor has no such door.
 */
export function doorwayFor(floor, labelLL) {
  if (!floor || !floor.navGraph || !floor.doorPoints?.length) return labelLL;
  const g = floor.navGraph;
  const from = g.isWalkable(labelLL) ? labelLL : (g.nearestWalkable?.(labelLL) ?? labelLL);
  const mpd = metresPerDegree(from[1]);
  let best = null, bestD = Infinity;
  for (const d of floor.doorPoints) {
    const dd = g.isWalkable(d) ? d : (g.nearestWalkable?.(d) ?? d);
    const dist = Math.hypot((dd[0] - from[0]) * mpd.lon, (dd[1] - from[1]) * mpd.lat);
    if (dist >= bestD || dist > 15) continue;
    const steps = Math.max(1, Math.ceil(dist / 0.2));
    let clear = true;
    for (let s = 1; s <= steps; s++) {
      const t = s / steps;
      if (!g.isWalkable([from[0] + (dd[0] - from[0]) * t, from[1] + (dd[1] - from[1]) * t])) { clear = false; break; }
    }
    if (clear) { best = dd; bestD = dist; }
  }
  return best ?? labelLL;
}

/**
 * A room by its label in one building: { ll (doorway), labelLL, floorId,
 * text (the label as drawn) }. Exact match first, then prefix.
 */
export function findRoom(building, roomText) {
  const want = normRoom(roomText);
  if (!want) return null;
  // Room numbering usually starts with the floor (114A is on 1), so an
  // exact hit on that floor beats an exact hit elsewhere.
  const wantLevel = parseInt(want[0], 10);
  const hitFor = (f, label) => ({
    ll: doorwayFor(f, label.geometry.coordinates), labelLL: label.geometry.coordinates,
    floorId: f.floorId, text: label.properties.text,
  });
  let exact = null, prefixed = null;
  for (const f of building.floors) {
    for (const label of f.roomLabels ?? []) {
      const text = normRoom(label.properties.text);
      if (text === want) {
        const hit = hitFor(f, label);
        if (!isNaN(wantLevel) && floorLevelOf(f.floorId) === wantLevel) return hit;
        exact ??= hit;
      } else if (!prefixed && text.startsWith(want)) {
        prefixed = hitFor(f, label);
      }
    }
  }
  return exact ?? prefixed;
}

// --- nicknames --------------------------------------------------------------

// Words that never decide a match ("AI & Robotics Lab" vs "ai robotics lab").
const STOP_WORDS = new Set(['and', 'the', 'of', 'a', 'an']);

/** Name tokens: lowercase, accents and punctuation dropped, stop words removed. */
export function nameTokens(s) {
  return String(s ?? '').normalize('NFKD').replace(/[̀-ͯ]/g, '').toLowerCase()
    .split(/[^a-z0-9]+/).filter((t) => t && !STOP_WORDS.has(t));
}

/** One string per name, for equality and counting (the vote key). */
export const normName = (s) => nameTokens(s).join(' ');

/** The canonical "where" of a place: "SU Subway" or "ll:-149.8,61.19@01". */
export function placeTarget(place) {
  if (place.ll) return `ll:${(+place.ll[0]).toFixed(6)},${(+place.ll[1]).toFixed(6)}@${place.floor}`;
  return `${String(place.building).toUpperCase()} ${place.room}`;
}

/** Parse a target back into place fields ({building, room} or {ll, floor}). */
export function parseTarget(target) {
  const m = /^ll:(-?[\d.]+),(-?[\d.]+)@(.+)$/.exec(target);
  if (m) return {ll: [+m[1], +m[2]], floor: m[3]};
  const i = target.indexOf(' ');
  if (i <= 0) return null;
  return {building: target.slice(0, i), room: target.slice(i + 1)};
}

/**
 * Flatten places.json + the student's favorites into searchable entries,
 * one per name: { name, norm, tokens, place, target, favorite }.
 */
export function buildPlaceIndex(places, favorites = []) {
  const entries = [];
  const add = (name, place, favorite) => {
    const tokens = nameTokens(name);
    if (tokens.length) entries.push({name, norm: tokens.join(' '), tokens, place, target: placeTarget(place), favorite});
  };
  for (const fav of favorites) add(fav.name, fav, true);
  for (const p of places) for (const n of p.names ?? []) add(n, p, false);
  return entries;
}

/**
 * Rank the entries for a query: 0 same name, 1 name starts with the query,
 * 2 every query word starts a word of the name, 3 the query appears inside
 * the name. Favorites win ties, then shorter names. One result per
 * (place, name).
 */
export function searchPlaces(index, q, limit = 6) {
  const qTokens = nameTokens(q);
  if (!qTokens.length) return [];
  const qNorm = qTokens.join(' ');
  const qFlat = qTokens.join('');
  const scored = [];
  for (const e of index) {
    let score;
    if (e.norm === qNorm) score = 0;
    else if (e.norm.startsWith(qNorm)) score = 1;
    else if (qTokens.every((t) => e.tokens.some((w) => w.startsWith(t)))) score = 2;
    else if (e.tokens.join('').includes(qFlat)) score = 3;
    else continue;
    scored.push({e, score});
  }
  scored.sort((a, b) => a.score - b.score || (b.e.favorite - a.e.favorite) || a.e.name.length - b.e.name.length);
  const seen = new Set();
  const out = [];
  for (const {e, score} of scored) {
    const key = `${e.target}|${e.norm}`;
    if (seen.has(key)) continue;
    seen.add(key);
    out.push({...e, score});
    if (out.length >= limit) break;
  }
  return out;
}

/**
 * Where a place is on the loaded campus: { ll, labelLL, floorId, building }
 * or null when its building is not registered or its room is not drawn.
 */
export function resolvePlace(buildings, place) {
  if (place.ll) {
    const building = place.building ? findBuilding(buildings, place.building) : null;
    return {ll: place.ll, labelLL: place.ll, floorId: place.floor, building};
  }
  const building = findBuilding(buildings, place.building);
  if (!building) return null;
  const room = findRoom(building, place.room);
  return room ? {...room, building} : null;
}

/**
 * Schema problems in a places.json array (strings, empty when valid): ids
 * unique, at least one name, a room or a point, and no nickname naming two
 * different places.
 */
export function validatePlaces(places) {
  const problems = [];
  if (!Array.isArray(places)) return ['places.json must be an array'];
  const ids = new Set();
  const nameOwner = new Map();
  places.forEach((p, i) => {
    const who = p?.id ? `"${p.id}"` : `entry ${i}`;
    if (!p || typeof p !== 'object') { problems.push(`${who}: not an object`); return; }
    if (!p.id || typeof p.id !== 'string') problems.push(`${who}: missing id`);
    else if (ids.has(p.id)) problems.push(`${who}: duplicate id`);
    else ids.add(p.id);
    if (!Array.isArray(p.names) || !p.names.some((n) => nameTokens(n).length)) {
      problems.push(`${who}: needs at least one name`);
    }
    const isRoom = p.building && p.room;
    const isPoint = Array.isArray(p.ll) && p.ll.length === 2 && p.ll.every(Number.isFinite) && p.floor;
    if (!isRoom && !isPoint) problems.push(`${who}: needs building + room, or ll [lon, lat] + floor`);
    if (!isRoom && !isPoint) return;
    const target = placeTarget(p);
    for (const n of p.names ?? []) {
      const key = normName(n);
      if (!key) continue;
      const owner = nameOwner.get(key);
      if (owner && owner.target !== target) problems.push(`"${n}" names both ${owner.id} and ${p.id}`);
      else nameOwner.set(key, {id: p.id, target});
    }
  });
  return problems;
}
