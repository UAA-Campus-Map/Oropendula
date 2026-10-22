/**
 * Generate floor-catalog.json from an Extraction-Tool "Floor Data" folder.
 *
 * The Extraction-Tool repository is private, so the browser cannot read its
 * directory listing. This script turns a local clone/download of
 * `Data Extraction/Floor Data` into the static catalog the progress page uses.
 *
 * Floor plans are `AS###-<floor>.json`; `-pN` page variants collapse onto the
 * same floor. Non-JSON previews (.svg) are ignored. One (smallest) JSON per
 * building is parsed for the human name and UAA ID code.
 *
 * Usage:
 *   node gen-floor-catalog.mjs "/path/to/Floor Data"
 */
import { readdirSync, readFileSync, statSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';

const dir = process.argv[2];
if (!dir) {
  console.error('usage: node gen-floor-catalog.mjs "<Floor Data directory>"');
  process.exit(1);
}

/** Numeric floors are two digits; names like B1/Interstitial stay as-is. */
const normaliseFloor = (s) => (/^\d+$/.test(s) ? s.padStart(2, '0') : s);

const names = readdirSync(dir);
const byBuilding = new Map();     // building -> Set<floor>
const representative = new Map(); // building -> filename (smallest JSON)
for (const name of names) {
  if (!/\.json$/i.test(name)) continue;
  // Strip page variants (-p1, -p2, ...) before matching building-floor.
  const base = name.replace(/\.json$/i, '').replace(/-p\d+$/i, '');
  const m = base.match(/^([A-Za-z]+\d*)-(.+)$/);
  if (!m) continue;
  const building = m[1].toUpperCase();
  const floor = normaliseFloor(m[2].toUpperCase());
  if (!byBuilding.has(building)) byBuilding.set(building, new Set());
  byBuilding.get(building).add(floor);
  const prev = representative.get(building);
  if (!prev) representative.set(building, name);
  else {
    try {
      if (statSync(join(dir, name)).size < statSync(join(dir, prev)).size) {
        representative.set(building, name);
      }
    } catch { /* keep the first */ }
  }
}

const buildings = {};
const buildingNames = {};
const buildingIds = {};
for (const building of [...byBuilding.keys()].sort((a, b) => a.localeCompare(b))) {
  buildings[building] = [...byBuilding.get(building)].sort();
  const file = representative.get(building);
  if (!file) continue;
  try {
    const data = JSON.parse(readFileSync(join(dir, file), 'utf8'));
    if (data.building_name) buildingNames[building] = String(data.building_name).trim();
    if (data.ID) buildingIds[building] = String(data.ID).trim();
  } catch { /* name is optional */ }
}

const totalFloors = Object.values(buildings).reduce((n, f) => n + f.length, 0);
const catalog = {
  source: 'UAA-Campus-Map/Extraction-Tool — Data Extraction/Floor Data',
  generated: new Date().toISOString().slice(0, 10),
  buildings,
  names: buildingNames,
  ids: buildingIds,
};

writeFileSync(new URL('./floor-catalog.json', import.meta.url),
  JSON.stringify(catalog, null, 2) + '\n');
console.log(`Wrote floor-catalog.json: ${Object.keys(buildings).length} buildings, ` +
  `${totalFloors} floors, ${Object.keys(buildingNames).length} names`);
