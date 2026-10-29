# Tooling — workers, generators, and the test suite

These files are **not** imported by the app's runtime path (except the workers,
which the app spawns). They exist to build data, move work off the main thread,
or verify behaviour.

## Web Workers

### `navgraph-worker.mjs`
Runs the expensive, serializable half of `navgraph.mjs` off the render thread.
Message protocol: `{ id, walls, doors }` in → `{ id, state }` out (the blocked
raster is **transferred**, not copied), or `{ id, error }`. The main thread rehydrates the
query closures with `graphFromState(state)`. This is what keeps boot responsive
while floors are rasterised (~0.1–1.2 s each on desktop).

### `basis-worker.mjs`
Test-only (`smoke.mjs`). Builds **one floor's basis** on a worker thread: parse
the building JSON, derive the wall/door geometry, compute `graphInputHash` and
**validate the cached navGraph bundle's `sourceHash`**, and produce the floor's
portal set (fresh `extractPortals` for the `portals` category, else the config
cache). It returns the decoded bundle floor when the hash matches (the main
thread rehydrates with `makeBundleGraph`) or the raw inputs to build on-device
when it does not. Everything in/out is structured-cloneable.

### `navgraph-bundle-worker.mjs`
Import-tool worker. Builds one floor with `buildGraphState(..., {passages:false})`
and returns an encoded `navgraph-codec` floor block. The block is transferred to
the main thread, which assembles the per-building bundle without ever cloning the
large adjacency arrays.

### `navgraph-bundle-runtime.mjs`
App-side helper. Converts one decoded bundle floor into the `graphFromState()`
object, recomputing adjacency weights, and installs `ensureHeadingField()` so
passages/compass directions are built lazily on the active floor level rather
than during boot.

## Data generators

### `gen-navgraphs.mjs`
Generates every per-building navGraph bundle and adds `graphUrl` to
`buildings-config.json`:

```sh
node gen-navgraphs.mjs              # all buildings; writes .nvg.gz + config
node gen-navgraphs.mjs --check      # measure only, writes nothing
node gen-navgraphs.mjs AS127 AS154  # selected buildings only
```

The import tool's Deploy section can also generate the selected building or all
buildings. Regenerate after changing floor JSON, placement, or any graph input;
the import tool removes `graphUrl`, floor hashes and the building fingerprint
when it detects such an edit. Each bundle floor stores a `sourceHash` of its
wall/door inputs, and boot falls back to an on-device build when that hash no
longer matches the floor JSON. A data-only refresh still needs regeneration for
every building whose wall/window geometry changed, and the JSON and `.nvg.gz`
should be deployed together.

### `zip.mjs`
Tiny dependency-free ZIP writer used by the import tool's deploy diff. JSON
entries are deflated with `CompressionStream` when available; already-compressed
`.nvg.gz` entries are stored unchanged. Falls back to ZIP STORE if deflate is
unavailable.

### `navgraph-codec.test.mjs`
Codec and round-trip tests:

```sh
node --test navgraph-codec.test.mjs
```

Covers synthetic graphs, a multi-floor bundle, gzip, AS127-01 route parity, and
corrupt/truncated data.

### `gen-osm.mjs`
Precomputes the walkable OSM road network for the campus into `osm-paths.json`,
so the app does not re-decode the PMTiles archive on every load. Re-run whenever
the building set (and therefore the campus bbox) changes, then ship
`osm-paths.json`. The app falls back to a live decode when the cache is missing
or does not cover the bbox.

### `gen-portals.mjs`
Populates the portal cache **inside `buildings-config.json`** using
`extractPortals` + `cachePortals`. Run it whenever a floor JSON changes. Only DXF
positions are stored, so the cache is independent of a floor's placement and
never goes stale when a plan is re-aligned. If a wall layer changed, also
regenerate that building's `.nvg.gz` with `gen-navgraphs.mjs` (or the import
tool) and deploy the JSON and bundle together.

## `smoke.mjs` — the test suite

`node smoke.mjs [categories]`; no arguments runs everything. Categories:

```
align, walkability, pdr, corridor, arbiter, portals, router, osm, multifloor,
motion, autocal, anchorcal, headingfield, doorways, roompin, places
```

The fast subset that needs **no real-floor navGraph build**:

```
node smoke.mjs motion pdr arbiter autocal anchorcal headingfield doorways places
```

(`places` reads the floor JSONs' room labels but builds no navGraph; see
`places.md`. `roompin` builds one real floor, AS121-02.)

Everything else loads the real floors (shared across groups via
`basis-worker`).

The shared floor load uses the **shipped navGraph bundles** (`graphUrl`, the
same `.nvg.gz` the app loads) instead of rasterising walls: each floor's
wall/door geometry is hashed with `graphInputHash` and the bundle is used only
when its `sourceHash` matches, so a stale cache is rebuilt and **fails the
gate** (`navGraph bundles decode` / `every navGraph bundle matches its floor
JSON`). Portals come from the config cache unless the `portals` category needs
the fresh extraction. This cuts the load from ~16 s to ~2.5 s (routing groups)
or ~7.6 s when fresh portals are extracted.

Why it is shaped this way: the suite **builds its campuses the way the app
does** — same registration math, same wall filter, same bundles, same portal
extraction — so a green test means the deployed app behaves the same. Timestamps
are **off by default**; `SMOKE_TIMING=1` prefixes every line with elapsed
seconds. `pdr` exercises the particle
filter on **synthetic corridors and turns** (no sensors); `autocal` synthesises
GPS walks; `router`/`multifloor` route the real campus, checking invariants like
"outdoor legs never cross a wall" and "all building pairs route".

When you add a feature, add a synthetic test that would fail without it (the
step-length mixture tests are a good template), and keep the fast subset fast.

## See also

- [`apps.md`](apps.md) — the HTML entry points.
- [`navgraph-bundles.md`](navgraph-bundles.md) — binary bundle format and codec/runtime API.
- [`navgraph.md`](navgraph.md), [`campus.md`](campus.md), [`osm.md`](osm.md).
