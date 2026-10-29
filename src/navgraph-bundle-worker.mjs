/**
 * Import-tool navGraph bundle worker.
 *
 * Builds one floor's graph and encodes it as a compact floor block off the
 * import tool's main thread. The block is transferred, not copied; the main
 * thread only assembles the tiny bundle header.
 *
 * Protocol:
 *   in:  {id, floorId, walls, doors, sourceHash, mask}
 *   out: {id, floorId, meta, block: ArrayBuffer}  or  {id, error}
 */
import { buildGraphState } from './navgraph.mjs';
import { encodeFloorBlock } from './navgraph-codec.mjs';

self.onmessage = (e) => {
  const { id, floorId, walls, doors, sourceHash, mask } = e.data || {};
  try {
    const state = buildGraphState(walls, doors, { passages: false, mask });
    const { block, meta } = encodeFloorBlock(state, { sourceHash });
    const buffer = block.byteOffset === 0 && block.byteLength === block.buffer.byteLength
      ? block.buffer
      : block.slice().buffer;
    self.postMessage({ id, floorId, meta, block: buffer }, [buffer]);
  } catch (err) {
    self.postMessage({ id, error: String((err && err.message) || err) });
  }
};
