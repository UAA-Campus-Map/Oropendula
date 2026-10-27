/**
 * Precompute the walkable OSM road network for the campus and write it to
 * osm-paths.json, so index.html does not re-decode the pmtiles archive on
 * every load (fetch + MVT decode is several seconds).
 *
 * Re-run this whenever the building set (and therefore the campus bbox)
 * changes, then deploy osm-paths.json. index.html falls back to decoding
 * pmtiles live when the cache is missing or does not cover the campus.
 *
 * Usage:  node gen-osm.mjs
 */
import { readFileSync, writeFileSync } from 'fs';
import { loadOsmShapes } from './osm.mjs';
import { pmtilesGetZxy } from './pmtiles-node.mjs';
import { makeToLL, metresPerDegree } from './geo.mjs';
import { portalsFromCache } from './campus.mjs';

const OSM_URL = 'https://uaa-campus-map.net/scak.pmtiles';
const PAD_M = 200;   // must match OSM_BBOX_PAD_M in index.html

const CONFIG = JSON.parse(readFileSync(new URL('./buildings-config.json', import.meta.url), 'utf8'));

// The app computes the bbox from every portal's lon/lat; mirror that exactly
// so the coverage check in index.html lines up.
function outdoorBBox(buildings, padM) {
  let minLon = Infinity, minLat = Infinity, maxLon = -Infinity, maxLat = -Infinity;
  for (const b of buildings) {
    for (const f of b.floors) {
      for (const p of [...f.elevators, ...f.stairs, ...f.exits]) {
        const [lon, lat] = p.ll;
        if (lon < minLon) minLon = lon;
        if (lon > maxLon) maxLon = lon;
        if (lat < minLat) minLat = lat;
        if (lat > maxLat) maxLat = lat;
      }
    }
  }
  const m = metresPerDegree((minLat + maxLat) / 2);
  return {
    minLon: minLon - padM / m.lon, maxLon: maxLon + padM / m.lon,
    minLat: minLat - padM / m.lat, maxLat: maxLat + padM / m.lat,
  };
}

const buildings = CONFIG.map(cfg => ({
  id: cfg.id,
  floors: cfg.floors.map(f => {
    const toLL = makeToLL(cfg, f);
    const p = f.portals ? portalsFromCache(f.portals, toLL) : {elevators: [], stairs: [], exits: []};
    // Manual doors (floor.exits) win over the algorithm's, matching index.html.
    const exits = f.exits?.length
      ? f.exits.map((e, i) => ({id: e.id ?? `exits-${i}`, ll: toLL(e.dxf[0], e.dxf[1])}))
      : p.exits;
    return {floorId: f.floorId, elevators: p.elevators, stairs: p.stairs, exits};
  }),
}));

const bbox = outdoorBBox(buildings, PAD_M);
console.log('bbox', bbox);
const osm = await loadOsmShapes({
  url: OSM_URL, bbox,
  fetchTile: (z, x, y) => pmtilesGetZxy(OSM_URL, z, x, y),
});
const out = {bbox, zoom: 15, stats: osm.stats, shapes: osm.shapes};
writeFileSync(new URL('./osm-paths.json', import.meta.url), JSON.stringify(out));
const kb = (JSON.stringify(out).length / 1024).toFixed(0);
console.log(`Wrote osm-paths.json: ${osm.shapes.length} shapes, ${osm.stats.tiles} tiles, ${kb}KB`);
