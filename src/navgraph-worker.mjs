/**
 * navGraph build worker.
 *
 * `buildGraphState()` is the expensive, serializable half of navgraph.mjs
 * (~0.1-1.2 s per floor on desktop, 3-4x on mobile): it rasterises the wall
 * clearance bitmask and merges the free cells into nodes. Running it here keeps
 * the main thread responsive during boot; `graphFromState()` rehydrates the
 * cheap query closures back on the main thread.
 *
 * Protocol: {id, walls, doors} in, {id, state} out (the blocked raster is
 * TRANSFERRED, not copied), or {id, error} on failure.
 */
import { buildGraphState } from './navgraph.mjs';

self.onmessage = (e) => {
  const { id, walls, doors } = e.data;
  try {
    const state = buildGraphState(walls, doors);
    const transfer = [state.blocked.buffer];
    if (state.passages && state.passages.buffer) transfer.push(state.passages.buffer);
    self.postMessage({ id, state }, transfer);
  } catch (err) {
    self.postMessage({ id, error: String((err && err.message) || err) });
  }
};
