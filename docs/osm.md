# `osm.mjs` — OpenStreetMap roads as a translated layer

## What it does

Gives the router a real outdoor path network. The campus basemap is a Protomaps
extract of OpenStreetMap served as a single **PMTiles** archive
(`navigator.cheeseverse.net/scak.pmtiles`). Rather than add another map source,
this module reads that archive's `roads` layer for the tiles covering a lon/lat
bounding box, keeps the walkable ways, and translates them into the **same shape
format as the DXF files** — a `PATHS` layer of
`{ bbox, stroke, fill, width, closed, layer, items: [{ op:"line", points }] }`.

Because the points are already `[lon, lat]`, the DXF→lon/lat transform is the
identity for this layer. `campus.mjs` then treats OSM paths exactly like any
other shapes, so the source is fully swappable.

Key exports: `loadOsmShapes({ url, bbox, zoom, fetchTile })`, `decodeMvtRoads`,
`tilesToShapes`, `pmtilesFetcher(url)`, plus the slippy-map helpers
`lonLatToTile`, `tileRange`, `tilePixelToLL` and gzip helpers.

## Why it's built this way

**Reuse the existing pipeline.** The router already knows how to consume
"shapes". Translating OSM into that format (instead of teaching the router about
MVT) keeps the routing code source-agnostic and testable with synthetic shapes.

**Injected tile fetching.** The browser passes a closure over the `pmtiles`
global the page already loads; Node callers (the tests, `gen-osm.mjs`) pass their
own. The decode/slice logic is pure and unit-testable.

**Filter for walking.** Only `path`, `minor_road`, `major_road` kinds are kept,
and `access=private|no` ways are dropped — a "walkable" network, not every road.
Each emitted shape also keeps its OSM `kind` and `name`; `campus.mjs` uses them
to charge a heavy routing penalty for crossing named roads.

## Implementation notes

- MVT tiles use a 4096-square local grid (`TILE_EXTENT`); `tilePixelToLL`
  converts local pixels to lon/lat.
- `gen-osm.mjs` precomputes this at build time into `osm-paths.json` so the app
  does not pay the fetch+decode cost per load; the app falls back to live decode
  when the cache is missing or does not cover the campus bbox.
- `import.html`’s **Download osm-paths.json** builds the bbox from the union of
  the draft config, the **deployed** config (localStorage can lag the server),
  and each floor’s **wall extents** — not just portal points — so a stale draft
  cannot produce an undersized cache. Its status check also compares against the
  deployed config.
- Tiles overlap at their edges (buffer), so stitching can create duplicate/short
  ways; `campus.mjs` drops the ones that cut through buildings or are trivially
  short.

## See also

- [`campus.md`](campus.md) — consumes the shapes as the outdoor graph.
- [`pmtiles-node.md`](pmtiles-node.md) — the Node reader used by the tests.
- [`tooling.md`](tooling.md) — `gen-osm.mjs`.
