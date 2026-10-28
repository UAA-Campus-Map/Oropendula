/**
 * Compact, per-building navGraph bundles.
 *
 * A bundle stores the serializable half of `buildGraphState()` without the
 * derived parts:
 *
 *   - node positions are stored exactly (Float64)
 *   - each undirected edge is stored once (CSR, j > i)
 *   - edge weights are omitted and recomputed from node positions
 *   - the blocked raster is run-length encoded
 *   - passages / heading fields are omitted
 *
 * File layout (after optional gzip):
 *
 *   "NGB1" | u32 headerLength | UTF-8 header JSON | payload
 *
 * The payload is an array of 8-byte-aligned floor blocks. Each floor block is:
 *
 *   Float64[nodes * 2]      node x/y
 *   Uint32[nodes + 1]       adjacency offsets for j > i
 *   Uint32[edges]           adjacency neighbours, sorted per row
 *   Uint8[runs]             blocked run values (0/1)
 *   pad to 4 bytes
 *   Uint32[runs]            blocked run lengths
 *
 * All arrays are little-endian. Current target browsers and devices are
 * little-endian; `assertLittleEndian()` makes the assumption explicit.
 */

const MAGIC = [0x4e, 0x47, 0x42, 0x31]; // "NGB1"
export const FORMAT = 'oropendula-navgraph';
export const VERSION = 1;

const ALIGN4 = (n) => (n + 3) & ~3;
const ALIGN8 = (n) => (n + 7) & ~7;

function assertLittleEndian() {
  if (new Uint8Array(new Uint16Array([1]).buffer)[0] !== 1) {
    throw new Error('navgraph-codec: only little-endian platforms are supported');
  }
}

function asUint8(bytes) {
  if (bytes instanceof Uint8Array) return bytes;
  if (bytes instanceof ArrayBuffer) return new Uint8Array(bytes);
  if (ArrayBuffer.isView(bytes)) {
    return new Uint8Array(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  }
  throw new TypeError('navgraph-codec: expected Uint8Array, ArrayBuffer, or a typed array');
}

function assertInteger(value, name, { min = 0, max = 0xffffffff } = {}) {
  if (!Number.isInteger(value) || value < min || value > max) {
    throw new Error(`navgraph-codec: invalid ${name}: ${value}`);
  }
  return value;
}

function assertFinite(value, name) {
  if (!Number.isFinite(value)) throw new Error(`navgraph-codec: invalid ${name}: ${value}`);
  return value;
}

function stateOrigin(state) {
  const origin = state.origin;
  if (!origin || !Number.isFinite(origin.lon) || !Number.isFinite(origin.lat)) {
    throw new Error('navgraph-codec: state.origin must be {lon, lat}');
  }
  return origin;
}

function stateScale(state) {
  const scale = state.scale;
  if (!scale || !Number.isFinite(scale.lon) || !Number.isFinite(scale.lat)) {
    throw new Error('navgraph-codec: state.scale must be {lon, lat}');
  }
  return scale;
}

/**
 * Encode one `buildGraphState()` result into a floor block.
 *
 * @param {object} state
 * @param {{sourceHash?: string}} [opts]
 * @returns {{block: Uint8Array, meta: object}}
 */
export function encodeFloorBlock(state, { sourceHash = null } = {}) {
  assertLittleEndian();
  if (!state || !Array.isArray(state.nodes) || !Array.isArray(state.adj)) {
    throw new Error('navgraph-codec: state must contain nodes[] and adj[]');
  }

  const { nodes, adj, blocked } = state;
  const n = nodes.length;
  if (adj.length !== n) throw new Error('navgraph-codec: adj length does not match nodes');
  assertInteger(state.W, 'W', { min: 1 });
  assertInteger(state.H, 'H', { min: 1 });
  assertFinite(state.OX, 'OX');
  assertFinite(state.OY, 'OY');
  if (!(blocked instanceof Uint8Array)) {
    throw new Error('navgraph-codec: state.blocked must be a Uint8Array');
  }
  if (blocked.length !== state.W * state.H) {
    throw new Error('navgraph-codec: blocked length does not match W * H');
  }

  // Collect the upper triangle of the undirected graph. The source graph has
  // every edge twice; the bundle stores it once.
  const rows = Array.from({ length: n }, () => []);
  for (let i = 0; i < n; i++) {
    for (const entry of adj[i]) {
      const j = entry?.[0];
      assertInteger(j, `adj[${i}] neighbour`, { max: n - 1 });
      if (j <= i) continue;
      rows[i].push(j);
    }
  }

  const offsets = new Uint32Array(n + 1);
  let edgeCount = 0;
  for (let i = 0; i < n; i++) {
    const row = rows[i];
    row.sort((a, b) => a - b);
    offsets[i] = edgeCount;
    let prev = -1;
    for (const j of row) {
      if (j === prev) throw new Error(`navgraph-codec: duplicate edge ${i}-${j}`);
      prev = j;
      edgeCount++;
    }
  }
  offsets[n] = edgeCount;
  assertInteger(edgeCount, 'edge count');

  const neighbours = new Uint32Array(edgeCount);
  let w = 0;
  for (let i = 0; i < n; i++) {
    for (const j of rows[i]) neighbours[w++] = j;
  }

  const nodeValues = new Float64Array(n * 2);
  for (let i = 0; i < n; i++) {
    const p = nodes[i];
    if (!Array.isArray(p) || !Number.isFinite(p[0]) || !Number.isFinite(p[1])) {
      throw new Error(`navgraph-codec: invalid node ${i}`);
    }
    nodeValues[2 * i] = p[0];
    nodeValues[2 * i + 1] = p[1];
  }

  // RLE the blocked raster. Most floors are long free/blocked runs, so this is
  // much smaller than raw bytes even before gzip.
  const runValues = [];
  const runLengths = [];
  if (blocked.length) {
    let value = blocked[0];
    let run = 1;
    for (let i = 1; i < blocked.length; i++) {
      const next = blocked[i] ? 1 : 0;
      if (next === value) {
        run++;
      } else {
        runValues.push(value);
        runLengths.push(run);
        value = next;
        run = 1;
      }
    }
    runValues.push(value);
    runLengths.push(run);
  }
  const runs = runValues.length;
  assertInteger(runs, 'blocked run count');
  const values = Uint8Array.from(runValues);
  const lengths = Uint32Array.from(runLengths);

  const nodesBytes = n * 8 * 2;
  const offsetsBytes = (n + 1) * 4;
  const neighboursBytes = edgeCount * 4;
  const valuesBytes = runs;
  const lengthsOffset = ALIGN4(nodesBytes + offsetsBytes + neighboursBytes + valuesBytes);
  const blockLength = ALIGN8(lengthsOffset + runs * 4);

  const block = new Uint8Array(blockLength);
  new Float64Array(block.buffer, 0, n * 2).set(nodeValues);
  new Uint32Array(block.buffer, nodesBytes, n + 1).set(offsets);
  new Uint32Array(block.buffer, nodesBytes + offsetsBytes, edgeCount).set(neighbours);
  block.set(values, nodesBytes + offsetsBytes + neighboursBytes);
  new Uint32Array(block.buffer, lengthsOffset, runs).set(lengths);

  const origin = stateOrigin(state);
  const scale = stateScale(state);
  if (sourceHash != null && typeof sourceHash !== 'string') {
    throw new Error('navgraph-codec: sourceHash must be a string');
  }
  const meta = {
    nodes: n,
    edges: edgeCount,
    runs,
    ...(sourceHash != null ? { sourceHash } : {}),
    origin: [origin.lon, origin.lat],
    scale: [scale.lon, scale.lat],
    OX: state.OX,
    OY: state.OY,
    W: state.W,
    H: state.H,
    length: blockLength,
  };
  return { block, meta };
}

/**
 * Assemble pre-encoded floor blocks into one uncompressed bundle.
 *
 * @param {{buildingId: string, floors: Array<{floorId: string, meta: object, block: Uint8Array}>}} bundle
 * @returns {Uint8Array}
 */
export function encodeBundleFromBlocks({ buildingId, floors }) {
  assertLittleEndian();
  if (typeof buildingId !== 'string' || !buildingId) {
    throw new Error('navgraph-codec: buildingId must be a non-empty string');
  }
  if (!Array.isArray(floors) || floors.length === 0) {
    throw new Error('navgraph-codec: bundle must contain at least one floor');
  }

  const entries = [];
  let payloadLength = 0;
  for (const floor of floors) {
    if (!floor || typeof floor.floorId !== 'string' || !floor.floorId) {
      throw new Error('navgraph-codec: every floor needs a floorId');
    }
    const block = asUint8(floor.block);
    if (!floor.meta || typeof floor.meta !== 'object') {
      throw new Error(`navgraph-codec: floor ${floor.floorId} needs metadata`);
    }
    const offset = ALIGN8(payloadLength);
    const length = block.byteLength;
    entries.push({
      floorId: floor.floorId,
      offset,
      length,
      meta: { ...floor.meta },
      block,
    });
    payloadLength = offset + length;
  }

  const header = {
    format: FORMAT,
    version: VERSION,
    building: buildingId,
    floors: entries.map((entry) => ({
      floorId: entry.floorId,
      ...entry.meta,
      offset: entry.offset,
      length: entry.length,
    })),
  };
  const headerBytes = new TextEncoder().encode(JSON.stringify(header));
  const payloadStart = ALIGN8(8 + headerBytes.length);
  const out = new Uint8Array(payloadStart + payloadLength);
  out.set(MAGIC, 0);
  new DataView(out.buffer).setUint32(4, headerBytes.length, true);
  out.set(headerBytes, 8);
  for (const entry of entries) {
    out.set(entry.block, payloadStart + entry.offset);
  }
  return out;
}

/**
 * Convenience wrapper: encode `buildGraphState()` results and assemble them.
 *
 * @param {{buildingId: string, floors: Array<{floorId: string, state: object}>}} bundle
 * @returns {Uint8Array}
 */
export function encodeBundle({ buildingId, floors }) {
  const encoded = floors.map(({ floorId, state }) => {
    const { block, meta } = encodeFloorBlock(state);
    return { floorId, meta, block };
  });
  return encodeBundleFromBlocks({ buildingId, floors: encoded });
}

function readHeader(bytes) {
  const u8 = asUint8(bytes);
  if (u8.byteLength < 8) throw new Error('navgraph-codec: payload is too short');
  for (let i = 0; i < MAGIC.length; i++) {
    if (u8[i] !== MAGIC[i]) throw new Error('navgraph-codec: bad magic (expected NGB1)');
  }
  const headerLength = new DataView(u8.buffer, u8.byteOffset, u8.byteLength).getUint32(4, true);
  if (headerLength > u8.byteLength - 8) {
    throw new Error('navgraph-codec: header length exceeds payload');
  }
  const headerText = new TextDecoder().decode(u8.subarray(8, 8 + headerLength));
  let header;
  try {
    header = JSON.parse(headerText);
  } catch (err) {
    throw new Error(`navgraph-codec: invalid header JSON: ${err.message || err}`);
  }
  if (header?.format !== FORMAT) throw new Error('navgraph-codec: unsupported bundle format');
  if (header?.version !== VERSION) {
    throw new Error(`navgraph-codec: unsupported bundle version ${header?.version}`);
  }
  if (typeof header.building !== 'string' || !header.building) {
    throw new Error('navgraph-codec: header is missing building id');
  }
  if (!Array.isArray(header.floors)) throw new Error('navgraph-codec: header.floors is missing');
  return { header, headerLength, payloadStart: ALIGN8(8 + headerLength), u8 };
}

/**
 * Decode one floor block into a graph state.
 *
 * @param {object} meta
 * @param {Uint8Array} block
 * @returns {object} state-shaped object with compact adjacency plus `nodes`,
 *          `blocked`, and minimal stats. Pass through `expandAdjacency()` before
 *          `graphFromState()`.
 */
export function decodeFloor(meta, block) {
  assertLittleEndian();
  const raw = asUint8(block);
  // Bundle floor blocks start on an 8-byte boundary. A caller may still hand
  // us a subarray from an arbitrary offset; copy in that rare case so the
  // typed-array views stay aligned.
  const u8 = raw.byteOffset % 8 === 0 ? raw : raw.slice();
  const n = assertInteger(meta?.nodes, 'node count');
  const edgeCount = assertInteger(meta?.edges, 'edge count');
  const runs = assertInteger(meta?.runs, 'blocked run count');
  const W = assertInteger(meta?.W, 'W', { min: 1 });
  const H = assertInteger(meta?.H, 'H', { min: 1 });
  if (!Array.isArray(meta?.origin) || meta.origin.length !== 2 ||
      !Number.isFinite(meta.origin[0]) || !Number.isFinite(meta.origin[1])) {
    throw new Error('navgraph-codec: invalid origin');
  }
  if (!Array.isArray(meta?.scale) || meta.scale.length !== 2 ||
      !Number.isFinite(meta.scale[0]) || !Number.isFinite(meta.scale[1])) {
    throw new Error('navgraph-codec: invalid scale');
  }
  if (!Number.isFinite(meta?.OX) || !Number.isFinite(meta?.OY)) {
    throw new Error('navgraph-codec: invalid raster origin');
  }
  if (meta?.sourceHash != null && typeof meta.sourceHash !== 'string') {
    throw new Error('navgraph-codec: invalid sourceHash');
  }
  const nodesBytes = n * 16;
  const offsetsBytes = (n + 1) * 4;
  const neighboursBytes = edgeCount * 4;
  const valuesOffset = nodesBytes + offsetsBytes + neighboursBytes;
  const lengthsOffset = ALIGN4(valuesOffset + runs);
  const needed = ALIGN8(lengthsOffset + runs * 4);
  if (u8.byteLength < needed) {
    throw new Error(`navgraph-codec: floor block is truncated (${u8.byteLength} < ${needed})`);
  }

  const nodesFlat = new Float64Array(u8.buffer, u8.byteOffset, n * 2);
  const offsets = new Uint32Array(u8.buffer, u8.byteOffset + nodesBytes, n + 1);
  const neighbours = new Uint32Array(u8.buffer, u8.byteOffset + nodesBytes + offsetsBytes, edgeCount);
  const values = u8.subarray(valuesOffset, valuesOffset + runs);
  const lengths = new Uint32Array(u8.buffer, u8.byteOffset + lengthsOffset, runs);

  if (offsets[0] !== 0 || offsets[n] !== edgeCount) {
    throw new Error('navgraph-codec: invalid adjacency offsets');
  }
  for (let i = 0; i < n; i++) {
    if (offsets[i + 1] < offsets[i]) throw new Error('navgraph-codec: adjacency offsets are not monotonic');
    let prev = -1;
    for (let k = offsets[i]; k < offsets[i + 1]; k++) {
      const j = neighbours[k];
      if (j <= i || j >= n || j <= prev) {
        throw new Error(`navgraph-codec: invalid neighbour ${j} in row ${i}`);
      }
      prev = j;
    }
  }

  const blocked = new Uint8Array(W * H);
  let cursor = 0;
  for (let r = 0; r < runs; r++) {
    const end = cursor + lengths[r];
    if (end > blocked.length) throw new Error('navgraph-codec: blocked runs exceed raster');
    if (values[r] !== 0 && values[r] !== 1) throw new Error('navgraph-codec: invalid blocked run value');
    blocked.fill(values[r], cursor, end);
    cursor = end;
  }
  if (cursor !== blocked.length) throw new Error('navgraph-codec: blocked runs do not cover raster');

  const nodes = new Array(n);
  for (let i = 0; i < n; i++) nodes[i] = [nodesFlat[2 * i], nodesFlat[2 * i + 1]];
  const origin = { lon: meta.origin[0], lat: meta.origin[1] };
  const scale = { lon: meta.scale[0], lat: meta.scale[1] };
  return {
    floorId: meta.floorId,
    sourceHash: meta.sourceHash ?? null,
    origin,
    scale,
    OX: meta.OX,
    OY: meta.OY,
    W,
    H,
    nodes,
    blocked,
    adjOffsets: offsets,
    adjNeighbors: neighbours,
    stats: {
      cells: W * H,
      nodes: n,
      edges: edgeCount,
      blockedPercent: +(100 * blocked.reduce((a, v) => a + v, 0) / blocked.length).toFixed(1),
    },
  };
}

/**
 * Decode an uncompressed bundle payload.
 *
 * @param {Uint8Array|ArrayBuffer} bytes
 * @returns {{buildingId: string, floors: object[], header: object}}
 */
export function decodeBundle(bytes) {
  const { header, payloadStart, u8 } = readHeader(bytes);
  const payload = u8.subarray(payloadStart);
  const floors = [];
  for (const meta of header.floors) {
    const offset = assertInteger(meta.offset, 'floor offset');
    const length = assertInteger(meta.length, 'floor length', { min: 1 });
    if (offset + length > payload.byteLength) {
      throw new Error(`navgraph-codec: floor ${meta.floorId} exceeds payload`);
    }
    floors.push(decodeFloor(meta, payload.subarray(offset, offset + length)));
  }
  return { buildingId: header.building, floors, header };
}

/**
 * Expand compact undirected adjacency into the array representation expected by
 * `graphFromState()`. The fill order matches `buildGraphState()` exactly.
 *
 * @param {object} floor result of `decodeFloor()`
 * @returns {Array<Array<[number, number]>>}
 */
export function expandAdjacency(floor) {
  const { nodes, adjOffsets, adjNeighbors } = floor;
  const n = nodes.length;
  const adj = Array.from({ length: n }, () => []);
  for (let i = 0; i < n; i++) {
    const xi = nodes[i][0], yi = nodes[i][1];
    for (let k = adjOffsets[i]; k < adjOffsets[i + 1]; k++) {
      const j = adjNeighbors[k];
      const dx = nodes[j][0] - xi;
      const dy = nodes[j][1] - yi;
      const distance = Math.sqrt(dx * dx + dy * dy);
      adj[i].push([j, distance]);
      adj[j].push([i, distance]);
    }
  }
  return adj;
}

/**
 * Turn one decoded floor into the state object accepted by `graphFromState()`.
 * The decoded floor keeps its compact adjacency for a future CSR-native path.
 */
export function floorGraphState(floor, adj = expandAdjacency(floor)) {
  return {
    origin: floor.origin,
    scale: floor.scale,
    OX: floor.OX,
    OY: floor.OY,
    W: floor.W,
    H: floor.H,
    nodes: floor.nodes,
    adj,
    blocked: floor.blocked,
    stats: floor.stats,
    passages: null,
  };
}
