# Cached navGraph bundles

This page documents the compact per-building navigation-graph format introduced
by `navgraph-codec.mjs`, plus the encode/decode/runtime methods that use it.

The bundle exists to remove the expensive on-device visibility-graph build from
boot. It stores the **serializable half** of `buildGraphState()` and deliberately
omits everything that can be recreated cheaply:

- node coordinates are stored exactly (`Float64`);
- each undirected edge is stored once (`j > i`);
- edge weights are **omitted** and recomputed from node positions;
- the clearance raster is run-length encoded;
- passages and heading fields are **omitted** and rebuilt lazily.

The current campus (24 buildings, 75 floors) is about **32.7 MB raw** and
**3.2 MB gzip**; the `.nvg.gz` files that cross the wire are comfortably below
the 30 MB target.

## Files and configuration

Each building gets one file:

```
<buildingId>.nvg.gz
```

The building entry in `buildings-config.json` points at it:

```json
{
  "id": "AS127",
  "groundFloor": "01",
  "graphUrl": "/AS127.nvg.gz",
  "floors": [ ... ]
}
```

`graphUrl` is building-level because one bundle contains all of that building's
floors. It is optional: if absent, the app uses the old on-device build path.

## Container format

The `.nvg.gz` file is a gzip stream. `decodeBundle()` expects the **decompressed**
bytes; the app calls `gunzipBytes()` first.

After decompression:

```
offset  size                  field
0       4                     magic "NGB1"
4       4                     headerLength (u32, little-endian)
8       headerLength          header JSON (UTF-8)
8+H     pad to 8              zero padding
P       ...                   floor payload blocks
```

Where:

```
P = align8(8 + headerLength)
```

All multi-byte integers and floating-point values are little-endian. The decoder
asserts a little-endian platform; all current target browsers/devices are.

### Header JSON

```json
{
  "format": "oropendula-navgraph",
  "version": 1,
  "building": "AS127",
  "floors": [
    {
      "floorId": "01",
      "offset": 0,
      "length": 12345678,
      "nodes": 8082,
      "edges": 479303,
      "runs": 4321,
      "origin": [-149.8339, 61.1894],
      "scale": [111412.84, 111132.92],
      "OX": 12.34,
      "OY": 56.78,
      "W": 1400,
      "H": 1200
    }
  ]
}
```

- `format`: must equal `oropendula-navgraph`.
- `version`: currently `1`; the decoder rejects unknown versions.
- `building`: the `buildingId` passed to the encoder.
- `offset`: byte offset of this floor block **relative to the payload start**.
- `length`: block byte length, including its internal padding.
- `nodes`: node count `n`.
- `edges`: undirected edge count `E`.
- `runs`: number of `blocked` RLE runs.
- `origin`, `scale`: local metre frame from `buildGraphState()`.
- `OX`, `OY`, `W`, `H`: clearance raster origin and dimensions.

Floor offsets are 8-byte aligned. The payload may contain padding between floor
blocks and at the end of a block.

### Floor block layout

For a floor with `n` nodes, `E` undirected edges, and `R` blocked runs:

```
block offset        type              count / meaning
0                   Float64           2n       node x/y pairs
16n                 Uint32            n+1      adjacency offsets
16n + 4(n+1)        Uint32            E        neighbour indices, j > i
16n + 4(n+1) + 4E   Uint8             R        blocked run values (0/1)
align4(...)         Uint32            R        blocked run lengths
align8(...)         end                        block length
```

Concretely, define:

```
nodesBytes      = 16n
offsetsBytes    = 4(n + 1)
neighboursBytes = 4E
valuesOffset    = nodesBytes + offsetsBytes + neighboursBytes
lengthsOffset   = align4(valuesOffset + R)
blockLength     = align8(lengthsOffset + 4R)
```

`valuesOffset` is where the RLE values start. `lengthsOffset` is padded to a
4-byte boundary so the `Uint32Array` view is legal.

The encoder initialises the whole block to zero, so alignment/padding bytes are
zero.

### Nodes

`nodes` is a flat array of `Float64` pairs:

```js
[nodes * 2] = [x0, y0, x1, y1, ...]
```

These are the node centres in the local metre frame, in the same order as
`state.nodes`. `Float64` is used so decoded weights and routes are exactly equal
to the source graph.

### Undirected CSR adjacency

`adjOffsets` is a standard CSR offset array over node rows:

```
row i neighbours = adjNeighbors[adjOffsets[i] .. adjOffsets[i + 1] - 1]
```

Only edges with `j > i` are stored. Each row is sorted ascending. Therefore:

- `adjOffsets[0] === 0`
- `adjOffsets[n] === E`
- offsets are monotonic
- every neighbour is in `(i, n)`
- no duplicate edges

The source graph stores each edge twice (`adj[i]` and `adj[j]`). The bundle
stores the upper triangle once; the decoder expands both directions.

### Blocked raster RLE

`blocked` is a byte per raster cell (`1 = wall/clearance`, `0 = free`). It is
encoded as alternating runs:

```
values  = Uint8[R]    // 0 or 1
lengths = Uint32[R]   // run lengths in cells
```

The lengths must sum to `W * H`. The decoder fills the raster from the runs.
RLE is valuable because floor rasters are mostly long free/blocked spans; gzip
then shrinks the result further.

## Reconstructing the exact graph

`expandAdjacency()` fills the live array representation with the same order the
original builder used:

```js
for (let i = 0; i < n; i++) {
  for (let k = off[i]; k < off[i + 1]; k++) {
    const j = to[k];
    const d = Math.sqrt(dx * dx + dy * dy);
    adj[i].push([j, d]);   // outgoing edge
    adj[j].push([i, d]);   // incoming edge
  }
}
```

Because rows are filled by ascending `i` and ascending `j`:

- node `i`'s incoming neighbours (from smaller nodes) are appended first, in
  ascending order;
- then its outgoing neighbours are appended in ascending order.

This matches `buildGraphState()` exactly, so Dijkstra tie-breaking and route
results are preserved. Weights are recomputed from `Float64` node coordinates,
so they are exactly the source weights.

## Codec API

### `buildGraphState` option

`navgraph.mjs` accepts an optional third argument:

```js
buildGraphState(walls, doors, { passages: false });
```

When `passages` is `false`, the expensive `buildPassages()` step is skipped and
the returned state has `passages: null`. Default is `true`, preserving the app's
worker/fallback behavior. Bundle generation always uses `false`.

### `navgraph-codec.mjs`

```js
import {
  FORMAT, VERSION,
  encodeFloorBlock,
  encodeBundleFromBlocks,
  encodeBundle,
  decodeBundle,
  decodeFloor,
  expandAdjacency,
  floorGraphState,
} from './navgraph-codec.mjs';
```

#### `encodeFloorBlock(state)`

```js
const { block, meta } = encodeFloorBlock(state);
```

- `state` is a `buildGraphState()` result.
- Returns an 8-byte-aligned `Uint8Array` block and its metadata.
- `meta` contains `nodes`, `edges`, `runs`, `origin`, `scale`, `OX`, `OY`, `W`,
  `H`, and `length`.
- Throws on malformed state, invalid adjacency, unsupported blocked length, or
  duplicate/self edges.

This is the method used inside the import-tool bundle worker.

#### `encodeBundleFromBlocks({ buildingId, floors })`

```js
const bytes = encodeBundleFromBlocks({
  buildingId: 'AS127',
  floors: [{ floorId: '01', meta, block }, ...],
});
```

- Assembles already-encoded floor blocks into one **uncompressed** container.
- Pads floor blocks to 8-byte boundaries.
- Writes the `NGB1` magic, header length, JSON header and payload.
- Does not gzip; wrap with `gzipBytes()` for a `.nvg.gz` file.

This is the method used by the import tool after its worker returns blocks.

#### `encodeBundle({ buildingId, floors })`

```js
const bytes = encodeBundle({
  buildingId: 'AS127',
  floors: [{ floorId: '01', state }, ...],
});
```

Convenience wrapper that calls `encodeFloorBlock()` for each state, then
`encodeBundleFromBlocks()`. Useful in Node/tests; the import tool avoids it so it
does not rebuild states that already came back from the worker.

#### `decodeBundle(bytes)`

```js
const { buildingId, floors, header } = decodeBundle(uncompressedBytes);
```

- Expects the **decompressed** container.
- Validates magic, version, header, floor ranges, offsets, neighbours and RLE.
- Returns `{ buildingId, floors, header }`.
- Each returned floor has the shape described under `decodeFloor()`.

The I/O layer should call `gunzipBytes()` first when reading a `.nvg.gz` file.

#### `decodeFloor(meta, block)`

```js
const floor = decodeFloor(meta, payload.subarray(meta.offset, meta.offset + meta.length));
```

Returns a state-shaped object with:

```js
{
  floorId, origin, scale, OX, OY, W, H,
  nodes,             // Array<[x, y]>
  blocked,           // Uint8Array(W * H)
  adjOffsets,        // Uint32Array(n + 1)
  adjNeighbors,      // Uint32Array(E)
  stats              // cells/nodes/edges/blockedPercent
}
```

It does **not** build `adj` arrays or passages. Use `expandAdjacency()` and
`graphFromState()` next.

#### `expandAdjacency(floor)`

```js
const adj = expandAdjacency(floor);
```

Builds `Array<Array<[neighbour, distance]>>` exactly as described above. This is
the compatibility path for `graphFromState()` and for tests.

#### `floorGraphState(floor, adj?)`

```js
const state = floorGraphState(floor /*, adj */);
const graph = graphFromState(state);
```

Creates the state object accepted by `graphFromState()`:

- includes `nodes`, `adj`, `blocked`, origin/scale/raster metadata and stats;
- sets `passages: null`.

Heading fields are therefore not built until explicitly requested.

### `navgraph-codec-io.mjs`

```js
import { gzipBytes, gunzipBytes, isGzip } from './navgraph-codec-io.mjs';
```

- `gzipBytes(bytes) -> Promise<Uint8Array>` — gzip via `CompressionStream`.
- `gunzipBytes(bytes) -> Promise<Uint8Array>` — validate gzip magic and decompress.
- `isGzip(bytes) -> boolean` — check the `1f 8b` magic.

The codec core is synchronous and environment-neutral; only these helpers use
the browser/Node stream APIs.

### `navgraph-bundle-runtime.mjs`

```js
import { makeBundleGraph } from './navgraph-bundle-runtime.mjs';
```

#### `makeBundleGraph(floor, { lines, doorPoints })`

```js
const graph = makeBundleGraph(decodedFloor, { lines, doorPoints });
```

- Expands adjacency and calls `graphFromState()`.
- Installs `graph.ensureHeadingField()`, which builds passages and a heading
  field from the supplied wall/door inputs **on first use**.
- Returns the normal graph object (`findPath`, `attach`, `dijkstraFromAttach`,
  `isWalkable`, etc.).
- Does not build passages at boot.

This is what `app.html` uses for a cached floor. The `lines` and `doorPoints` are
exactly the outputs of `navGraphInputs()`.

### `navgraph-source.mjs`

```js
import { navGraphInputs } from './navgraph-source.mjs';

const { lines, doors } = navGraphInputs(buildingConfig, floorConfig, data, toLL);
```

Shared wall/door extraction used by the app, import tool and generator. This
keeps bundle generation and fallback on-device builds from disagreeing about
which shapes are graph inputs.

## Generating bundles

### Import tool

The Deploy section of `import.html` has:

- **Download navgraph bundle (current building)**
- **Download all navgraph bundles**

The bundle worker protocol is:

```js
// main -> worker
{ id, floorId, walls, doors }

// worker -> main
{ id, floorId, meta, block }   // block is a transferred ArrayBuffer
{ id, error }                  // on failure
```

Generation:

1. For each floor, derive `lines`/`doors` with `navGraphInputs()`.
2. Send `{ floorId, walls, doors }` to `navgraph-bundle-worker.mjs`.
3. The worker runs `buildGraphState(..., { passages: false })` and
   `encodeFloorBlock()`, transferring the block back.
4. The main thread calls `encodeBundleFromBlocks()`, then `gzipBytes()`.
5. The file downloads as `<buildingId>.nvg.gz`.
6. The building's `graphUrl` is set and `buildings-config.json` is downloaded
   separately with the usual button.

The import tool deletes `graphUrl` when graph inputs change (floor replacement,
placement/rotation, id change, floor removal), forcing regeneration.

### CLI

```sh
node gen-navgraphs.mjs                 # all buildings, write .nvg.gz + config
node gen-navgraphs.mjs --check         # measure only, write nothing
node gen-navgraphs.mjs AS127 AS154     # selected buildings only
```

The CLI uses the same codec and source extraction as the import tool. Use
`--check` for size validation or after changing the format.

## App boot flow

`app.html`:

1. Starts floor-JSON and per-building bundle downloads in parallel.
2. For each bundle: fetch → `gunzipBytes()` → `decodeBundle()`.
3. Maps decoded floors by `floorId`.
4. For each floor, `loadFloor()` calls `makeBundleGraph()` with the floor's
   wall/door inputs.
5. If a bundle is missing, corrupt, has the wrong building id, or lacks a floor,
   that floor falls back to the existing worker/synchronous `buildGraph()` path.
6. `buildCampusRouter()` consumes the same graph objects as before.
7. Heading fields are built lazily when the locator first needs compass
   directions near a plan.

No app boot step builds passages for all floors. Bundle caching is not
implemented in the app; fetches use the browser/server's normal HTTP cache
behaviour, and the boots are expected to download bundles again.

## Validation and failure handling

`decodeBundle()`/`decodeFloor()` reject:

- wrong magic or unsupported version;
- header/floor ranges outside the payload;
- invalid `origin`, `scale`, `OX` or `OY`;
- non-monotonic offsets, wrong `off[n]`, out-of-range/self/duplicate neighbours;
- blocked runs that exceed or fail to cover `W * H`;
- unsupported platform endianness;
- truncated blocks.

Gzip corruption is detected by `gunzipBytes()`/the platform decompressor. The
app treats any decode failure as a signal to fall back to on-device graph
building, so a bad bundle cannot brick the app.

## Tests

```sh
node --test navgraph-codec.test.mjs
```

Covers:

- synthetic floor blocks;
- `buildGraphState({passages:false})` equivalence;
- lazy heading-field construction in `makeBundleGraph()`;
- multi-floor bundles and gzip round-trip;
- AS127-01 node/blocked/adjacency/weight/route parity;
- corrupt magic, truncated blocks, invalid neighbours and invalid RLE.

For a full artifact check:

```sh
node gen-navgraphs.mjs --check
```

Current output is 75 floors, 188,857 nodes, 6,748,323 undirected edges, about
**32.7 MB raw** and **3.2 MB gzip** across the 24 per-building files.

## Versioning

`FORMAT` is `oropendula-navgraph` and `VERSION` is `1`. Any incompatible change
to the container, header, floor layout, adjacency semantics or RLE must bump
`VERSION`; the decoder rejects versions it does not understand. Additive header
fields are safe for older decoders because they ignore unknown JSON keys, but
changing field meanings is not.

## Why this shape

- **No weights on the wire:** a weight is one `sqrt(dx² + dy²)` per edge, so it
  is cheap to rebuild and costly to store. Visibility edge generation is the
  expensive build step; weight recomputation is not.
- **Undirected once:** halves the edge payload; the directed runtime adjacency
  is reconstructed in the same order.
- **RLE blocked:** the raster is mostly long runs; gzip then does the rest.
- **No passages:** ~5M passage segments are only needed by the heading field,
  which is used on the active floor level, not at boot.
- **Per-building bundles:** a failure or missing cache affects one building, not
  the whole campus, and each file can be regenerated independently.
