/**
 * Generate per-building floor navGraph bundles and (unless --check) add their
 * `graphUrl` to buildings-config.json.
 *
 * Usage:
 *   node gen-navgraphs.mjs                 # all buildings
 *   node gen-navgraphs.mjs --check         # build/measure, write nothing
 *   node gen-navgraphs.mjs AS127 AS154     # selected buildings
 *
 * The import tool's "Download navgraph bundle" button uses the same codec;
 * this CLI is for regenerating the initial/deployed set and for size checks.
 */
import { readFileSync, writeFileSync } from 'fs';
import { buildGraphState } from './navgraph.mjs';
import { encodeFloorBlock, encodeBundleFromBlocks } from './navgraph-codec.mjs';
import { gzipBytes } from './navgraph-codec-io.mjs';
import { navGraphInputs } from './navgraph-source.mjs';
import { makeToLL } from './geo.mjs';

const args = process.argv.slice(2);
const checkOnly = args.includes('--check');
const selected = new Set(args.filter((a) => !a.startsWith('-')));
const configPath = new URL('./buildings-config.json', import.meta.url);
const CONFIG = JSON.parse(readFileSync(configPath, 'utf8'));

function graphFileName(id) {
  const base = String(id || 'building').replace(/[^A-Za-z0-9._-]/g, '_') || 'building';
  return `${base}.nvg.gz`;
}

let totalRaw = 0, totalGz = 0, totalFloors = 0, totalNodes = 0, totalEdges = 0;
const failures = [];

for (const building of CONFIG) {
  if (selected.size && !selected.has(building.id)) continue;
  const floors = [];
  try {
    for (const floor of building.floors) {
      if (!floor.url) throw new Error(`${building.id} ${floor.floorId} has no URL`);
      const data = JSON.parse(readFileSync(new URL('.' + floor.url, import.meta.url), 'utf8'));
      const toLL = makeToLL(building, floor);
      const { lines, doors } = navGraphInputs(building, floor, data, toLL);
      process.stderr.write(`  ${building.id} ${floor.floorId}...\n`);
      const state = buildGraphState(lines, doors, { passages: false });
      const { block, meta } = encodeFloorBlock(state);
      floors.push({ floorId: floor.floorId, state, block, meta });
      totalNodes += meta.nodes;
      totalEdges += meta.edges;
    }
    const bytes = encodeBundleFromBlocks({ buildingId: building.id, floors });
    const gz = await gzipBytes(bytes);
    totalRaw += bytes.length;
    totalGz += gz.length;
    totalFloors += floors.length;

    // EncodeBundle convenience is intentionally not used here: we already have
    // floor blocks from the per-floor build and avoid rebuilding them.
    const filename = graphFileName(building.id);
    if (!checkOnly) {
      writeFileSync(new URL('./' + filename, import.meta.url), gz);
      building.graphUrl = '/' + filename;
    }
    console.log(`${building.id}: ${floors.length} floors, ` +
      `${(bytes.length / 1048576).toFixed(2)}MB raw, ` +
      `${(gz.length / 1048576).toFixed(2)}MB gzip` +
      (checkOnly ? ' (check only)' : ` -> ${filename}`));
  } catch (err) {
    failures.push({ id: building.id, error: String(err?.message || err) });
    console.error(`${building.id}: FAILED - ${err?.stack || err}`);
  }
}

if (!checkOnly && !failures.length) {
  writeFileSync(configPath, JSON.stringify(CONFIG, null, 2) + '\n');
  console.log(`updated ${configPath.pathname}`);
}

console.log(`TOTAL: ${totalFloors} floors, ${totalNodes} nodes, ${totalEdges} undirected edges, ` +
  `${(totalRaw / 1048576).toFixed(2)}MB raw, ${(totalGz / 1048576).toFixed(2)}MB gzip`);
if (failures.length) {
  console.error(`FAILED: ${failures.length} building(s)`);
  for (const f of failures) console.error(`  ${f.id}: ${f.error}`);
  process.exitCode = 1;
}
