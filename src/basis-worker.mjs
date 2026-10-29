/**
 * Worker for smoke.mjs - builds ONE floor's portal set (and validates the
 * cached navGraph bundle) off the main thread: parse the building JSON, derive
 * the lon/lat wall/door geometry, hash it against the bundle's `sourceHash`, and
 * extract the fresh portal set. When the hash matches, the decoded bundle floor
 * (sent in by the main thread) is passed straight back and the main thread
 * rehydrates it with `makeBundleGraph()` instead of rasterising walls. All
 * inputs and outputs are structured-cloneable. Loaded only by the test harness.
 */
import { readFileSync } from 'node:fs';
import { parentPort } from 'node:worker_threads';
import { extractPortals, portalsFromCache } from './campus.mjs';
import { navGraphInputs, graphInputHash } from './navgraph-source.mjs';
import { makeToLL } from './geo.mjs';

parentPort.on('message', ({ cfg, floor, bundleFloor, freshPortals: wantFresh }) => {
  try {
    const data = JSON.parse(readFileSync(new URL('.' + floor.url, import.meta.url), 'utf8'));
    const toLL = makeToLL(cfg, floor);
    const { lines, doors } = navGraphInputs(cfg, floor, data, toLL);
    const sourceHash = graphInputHash({ lines, doors });
    // Validate: only hand back a bundle whose baked inputs match this JSON.
    const bundleOk = !!bundleFloor && bundleFloor.sourceHash === sourceHash;
    const freshPortals = (wantFresh || !floor.portals)
      ? extractPortals(data.shapes, toLL)
      : portalsFromCache(floor.portals, toLL);
    parentPort.postMessage({
      ok: true,
      lines,
      doors,
      sourceHash,
      bundleFloor: bundleOk ? bundleFloor : null,
      freshPortals,
      textLines: data.text_lines ?? [],
    });
  } catch (error) {
    parentPort.postMessage({ ok: false, error: String(error?.stack || error) });
  }
});
