/**
 * Populate the portal cache in buildings-config.json.
 *
 * extractPortals (doorway clustering + coarse flood fill) costs seconds per
 * building at app load. The import tool writes this cache during normal use;
 * this script bootstraps or refreshes it in one shot - run it whenever a
 * floor JSON changes. Usage:  node gen-portals.mjs
 *
 * Only DXF positions are stored, so the cache is independent of a floor's
 * placement (origin/offset/rotation) and never goes stale when a plan is
 * re-aligned.
 */
import { readFileSync, writeFileSync } from 'fs';
import { extractPortals, cachePortals } from './campus.mjs';

const file = new URL('./buildings-config.json', import.meta.url);
const config = JSON.parse(readFileSync(file, 'utf8'));

let floors = 0, exits = 0;
for (const b of config) {
  for (const f of b.floors) {
    if (!f.url) continue;
    const data = JSON.parse(readFileSync(new URL('.' + f.url, import.meta.url), 'utf8'));
    const portals = cachePortals(extractPortals(data.shapes, (x, y) => [x, y]));
    f.portals = portals;
    floors++;
    exits += portals.exits.length;
    console.log(`  ${b.id}-${f.floorId}: ${portals.exits.length} exits, ` +
      `${portals.stairs.length} stairs, ${portals.elevators.length} elevators`);
  }
}

writeFileSync(file, JSON.stringify(config, null, 2) + '\n');
console.log(`Wrote ${floors} floors (${exits} exits) to buildings-config.json`);
