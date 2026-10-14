# `pmtiles-node.mjs` — a dependency-free PMTiles reader

## What it does

A minimal **PMTiles v3** reader for Node, used by the test harness so the suite
can exercise the real OSM pipeline without an npm dependency (or a browser).
It parses the archive header, walks the tile directory, and fetches a single
`z/x/y` tile over HTTP `Range` requests. Export: `pmtilesGetZxy(url, z, x, y)`.

## Why it exists / why this design

The browser app uses the full `pmtiles` UMD bundle. The tests need the *same*
tiles by the *same* rules, but pulling a package into a zero-dependency repo
would couple the test runner to npm. Implementing just the slice we need (v3
header + directory walk + range fetch) is ~100 lines and keeps the project
self-contained.

The reader follows the v3 spec directly (varint directory entries, the root
directory, and leaf directories), so it stays compatible with the same archives
the browser library serves.

## Implementation notes

- Fetches use HTTP `Range` and `node:zlib` for gzip — no streaming, no cache; a
  test fetches a handful of tiles.
- The app never imports this file; it is **Node-only tooling**.
- The browser equivalent lives behind `osm.mjs`'s injected fetcher.

## See also

- [`osm.md`](osm.md) — the browser-side tile decode and shape translation.
- [`tooling.md`](tooling.md) — how the tests use it.
