# Tooling — workers, generators, and the test suite

These files are **not** imported by the app's runtime path (except the workers,
which the app spawns). They exist to build data, move work off the main thread,
or verify behaviour.

## Web Workers

### `navgraph-worker.mjs`
Runs the expensive, serializable half of `navgraph.mjs` off the render thread.
Message protocol: `{ id, walls }` in → `{ id, state }` out (the blocked raster is
**transferred**, not copied), or `{ id, error }`. The main thread rehydrates the
query closures with `graphFromState(state)`. This is what keeps boot responsive
while floors are rasterised (~0.1–1.2 s each on desktop).

### `basis-worker.mjs`
Test-only (`smoke.mjs`). Builds **one floor's basis** on a worker thread: parse
the building JSON, derive lon/lat wall segments, build the navGraph state, and
extract a fresh portal set. Everything in/out is structured-cloneable; the main
thread rehydrates with `graphFromState`. Used to parallelise the suite's floor
loading.

## Data generators

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
never goes stale when a plan is re-aligned.

## `smoke.mjs` — the test suite

`node smoke.mjs [categories]`; no arguments runs everything. Categories:

```
align, walkability, pdr, arbiter, portals, router, osm, multifloor, motion, autocal
```

The fast subset that needs **no real floor data**:

```
node smoke.mjs motion pdr arbiter autocal
```

Everything else builds real floors (shared across groups via `basis-worker`).

Why it is shaped this way: the suite **builds its campuses the way the app
does** — same registration math, same wall filter, same portal extraction — so a
green test means the deployed app behaves the same. `pdr` exercises the particle
filter on **synthetic corridors and turns** (no sensors); `autocal` synthesises
GPS walks; `router`/`multifloor` route the real campus, checking invariants like
"outdoor legs never cross a wall" and "all building pairs route".

When you add a feature, add a synthetic test that would fail without it (the
step-length mixture tests are a good template), and keep the fast subset fast.

## See also

- [`apps.md`](apps.md) — the HTML entry points.
- [`navgraph.md`](navgraph.md), [`campus.md`](campus.md), [`osm.md`](osm.md).
