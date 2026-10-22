/**
 * Worker for smoke.mjs - builds ONE floor's basis off the main thread:
 * parse the building JSON, derive the lon/lat wall segments, build the
 * navGraph state and extract the fresh portal set. All inputs and outputs are
 * structured-cloneable; the main thread rehydrates the live graph with
 * navgraph.mjs graphFromState(). Loaded only by the test harness.
 */
import { readFileSync } from 'node:fs';
import { parentPort } from 'node:worker_threads';
import { buildGraphState } from './navgraph.mjs';
import { extractPortals, shapeSegments, isWallLayer } from './campus.mjs';
import { makeToLL } from './geo.mjs';

parentPort.on('message', ({ cfg, floor }) => {
  try {
    const data = JSON.parse(readFileSync(new URL('.' + floor.url, import.meta.url), 'utf8'));
    const toLL = makeToLL(cfg, floor);
    const walls = data.shapes.filter(s => isWallLayer(s.layer));
    const lines = walls.flatMap(s =>
      shapeSegments(s).map(([a, b]) => [toLL(a[0], a[1]), toLL(b[0], b[1])]));
    const state = buildGraphState(lines);
    const freshPortals = extractPortals(data.shapes, toLL);
    parentPort.postMessage({
      ok: true,
      lines,
      state,
      freshPortals,
      textLines: data.text_lines ?? [],
    });
  } catch (error) {
    parentPort.postMessage({ ok: false, error: String(error?.stack || error) });
  }
});
