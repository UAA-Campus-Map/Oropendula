# `navgraph.mjs` — indoor pathfinding

## What it does

Turns one floor's wall segments into a queryable navigation graph and answers
path queries on it. Exports:

- `buildGraph(walls)` → the live graph object (build + query closures).
- `buildGraphState(walls)` / `graphFromState(state)` → the same graph split into
  a **serializable half** (the expensive raster) and a **cheap rehydration half**,
  so the build can run in a Web Worker.

The graph object exposes `findPath(startLL, endLL)`, the O(1) predicates
`isWalkable(ll)`, `nearestWalkable(ll, maxRadius)`, `inBounds(ll, marginM)`, and
the low-level attachment API (`attach`, `dijkstraFromAttach`, `distToAttach`,
`pathToAttach`) used by the router and the particle filter.

## Why it's built this way

**Precompute once, query cheap.** Deciding where a body cannot go is expensive
(rasterise walls, mark clearance). Doing it per click would search a quarter of
a million cells each time. `buildGraph` does that work **once at load**, and each
later query is a Dijkstra over a few hundred nodes.

The pipeline:

1. **Rasterise a clearance bitmask.** Walls are painted into a bitmask inflated
   by `CLEARANCE = 0.30 m` (the body radius), so the free cells are the ones a
   person can actually stand in. `CELL = 0.10 m` is the grid pitch — small enough
   not to falsely seal a ~0.3 m doorway.
2. **Merge free cells into rectangles.** Each maximal free rectangle becomes one
   graph **node**. This is the step that makes the graph small (hundreds, not
   thousands, of nodes) while preserving connectivity.
3. **Link nodes that can see each other**, capped at `MAX_LINK = 14 m` to bound
   build time and memory.
4. **Attach + Dijkstra.** A query point attaches to the nodes it can see (via
   short "stubs"), then one Dijkstra yields distance and predecessor arrays from
   which paths are reconstructed.

**Why a local metre frame.** All geometry is computed in metres derived from the
data (`geo.mjs`). Mixing degrees with metre-sized lengths is the classic trap:
degrees of latitude and longitude differ in length, and a metre-scale grid
expressed in degrees vanishes.

**Why split state from closures.** `buildGraphState` is pure data (origin, scale,
rasters, adjacency) and `graphFromState` rehydrates the query closures. That
split lets `navgraph-worker.mjs` do the CPU-bound half off the render thread and
transfer the blocked raster without a copy.

### Cached bundles

`navgraph-codec.mjs` packs a building's floors into a compact per-building
bundle (`<buildingId>.nvg.gz`) with:

- `Float64` node positions,
- each undirected edge once (CSR over `j > i`),
- an RLE `blocked` raster,
- **no edge weights** (recomputed from node positions),
- **no passages/heading field** (rebuilt lazily).

The container is `NGB1` + a JSON directory + 8-byte-aligned floor blocks; the
whole file is gzipped. `buildings-config.json` points at it with a building-level
`graphUrl`. `decodeBundle()` returns compact adjacency; `expandAdjacency()`
reconstructs exactly the same adjacency order and weights as
`buildGraphState()` for `graphFromState()`.

`buildGraphState(walls, doors, { passages: false })` skips the passage build when
generating a bundle. The default remains `true`.

At boot `app.html` fetches each building's `graphUrl`, gunzips/decodes it, and
uses `makeBundleGraph()` to produce the same query graph. Missing or invalid
bundles fall back to the existing worker/synchronous build. Heading fields are
created lazily from the floor's wall/door inputs on first locator use.

See [`navgraph-bundles.md`](navgraph-bundles.md) for the complete byte layout,
codec API, runtime methods, generation workflow, and validation rules.

## Implementation notes / invariants

- The clearance raster inflates every wall by ~0.3 m each side, i.e. a ~0.6 m
  blocked band. A step shorter than 1 m whose midpoint **and** endpoint are both
  clear cannot have jumped a wall — used as a fast path in the particle filter.
- `isWalkable` is an O(1) raster lookup, not a geometric test; keep it that way.
- `inBounds(ll, marginM)` distinguishes "outside the plan" from "inside a wall":
  the raster is false everywhere beyond a floor's wall bbox, but that edge is not
  a wall. The particle filter relies on this to avoid piling up along the bbox.
- `attach` returns *visible* nodes plus stub distances; connection is by
  visibility, so a doorway (an opening in the raster) is how two rooms connect.

## See also

- [`campus.md`](campus.md) — composes per-floor graphs into a multi-building
  router.
- [`locate.md`](locate.md) — consumes `isWalkable`/`nearestWalkable`/`inBounds`.
- [`tooling.md`](tooling.md) — `navgraph-worker.mjs`, `navgraph-bundle-worker.mjs`,
  and `gen-navgraphs.mjs`.
