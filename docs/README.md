# Oropendula — Module Documentation

Instructive notes on each module in `src/`: **what it does**, **why we built it
that way**, and the **implementation details** (API, invariants, gotchas) you
need to change it safely.

Start with the monolith of hard-won context in [`src/agents.md`](../src/agents.md);
these pages expand it, one module at a time. Everything here reflects the code on
`main`.

## Canonical site

Production is served at **https://uaa-campus-map.net**. The static site root is
`/usr/share/nginx/html` on that host, and the basemap/style/PMTiles assets are
loaded from the same domain:

- `https://uaa-campus-map.net/style.json`
- `https://uaa-campus-map.net/scak.pmtiles`

The previous hostname is retired; use `uaa-campus-map.net` in code, docs,
deployment commands, and import-tool settings.

## Reading order

The runtime pipeline runs **sensors → motion → locate → arbiter → app**:

| # | Page | Module | In one line |
|---|------|--------|-------------|
| 1 | [geo.md](geo.md) | `geo.mjs` | DXF↔lon/lat registration and metres-per-degree |
| 2 | [navgraph.md](navgraph.md) | `navgraph.mjs` | per-floor clearance raster + visibility graph + Dijkstra |
| 2a | [navgraph-bundles.md](navgraph-bundles.md) | `navgraph-codec.mjs` + runtime | cached binary navGraph format, exports and boot path |
| 3 | [campus.md](campus.md) | `campus.mjs` | portals, doors, skybridges, the outdoor graph, and the tiered router |
| 4 | [osm.md](osm.md) | `osm.mjs` | OpenStreetMap roads → the same shape format, for outdoor legs |
| 5 | [pmtiles-node.md](pmtiles-node.md) | `pmtiles-node.mjs` | dependency-free PMTiles reader for the tests |
| 6 | [motion.md](motion.md) | `motion.mjs` | step detection, compass heading, step-length model, sensor wiring |
| 7 | [locate.md](locate.md) | `locate.mjs` | the map-matching **particle filter** (PDR) + corridor heading |
| 8 | [arbiter.md](arbiter.md) | `arbiter.mjs` | fuses GPS and PDR continuously, picks the more certain one |
| 9 | [autocal.md](autocal.md) | `autocal.mjs` | online step-length calibration from GPS |
| 10 | [align.md](align.md) | `align.mjs` | rigid floor alignment for the import tool |
| 11 | [tooling.md](tooling.md) | workers + scripts | `gen-osm`, `gen-navgraphs`, `gen-portals`, `navgraph-worker`, `navgraph-bundle-worker`, `zip`, `basis-worker`, `smoke.mjs` |
| 12 | [apps.md](apps.md) | HTML entry points | `app.html`, `index.html`, `import.html`, `places.html`, `locate-test.html` |
| 13 | [places.md](places.md) | `places.mjs` | place nicknames in search, student favorites, form voting |

## How to keep these true

- Each page mirrors the module's own top-of-file comment and its public exports.
  When you change an export or an invariant, update the page in the same commit.
- These docs are **explanatory**, not a second source of truth. If a doc and the
  code disagree, the code wins — then fix the doc.
- Internal math is **metric**; the user interface shows **Imperial** (see
  `apps.md`). Keep that boundary in mind when reading distances.
