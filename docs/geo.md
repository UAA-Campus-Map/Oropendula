# `geo.mjs` — DXF ↔ lon/lat registration

## What it does

Floor plans are authored as DXF drawings in **inches**, on an arbitrary local
coordinate system. The map, the router and the particle filter all work in
**lon/lat** and **metres**. This module is the single bridge between the two:

- `makeToLL(building, floor)` → a closure mapping `(dxfX, dxfY) → [lon, lat]`.
- `dxfCoordsToLonLat(origin, x, y, rot, scale)` and its inverse
  `lonLatToDxf(...)`.
- `metresPerDegree(lat)` — the local metres per degree of longitude/latitude.
- `crossBarrier(polylineM, a, b, toleranceM)` / `nearestApproach(polylineM, a, b)`
  — the tolerant door-crossing tests (all in metres): whether a route crosses a
  doorway barrier's line (with a small end tolerance) or passes through the
  opening beside it. Used by Room Pin's fallback path and interior-door blocking;
  see
  `campus.md` / `roompin.md`.
- `DXF_SCALE = 0.0254` (metres per inch).

## Why it exists / why this design

Registration is a classic source of silent, compounding error: get it wrong and
every wall, door and route is subtly offset. Three decisions keep it honest:

1. **One transform, imported everywhere.** `index.html`, `smoke.mjs`,
   `import.html` and `app.html` all import from here, so the app and the import
   tool can never disagree about where a wall is.
2. **Effective values win per floor.** A building has an `origin`/`offset`, and
   an individual floor may override them (AS106 is registered at floor level).
   `makeToLL` applies the floor's values when present, otherwise the building's.
3. **A local metre frame for geometry, not degrees.** `metresPerDegree` exists
   so that lengths (clearance, cell size, link length) can be computed in metres
   and converted to degrees only at the edges. Treating a "0.1 m cell" as a
   degree quantity makes the grid ~1e-6° wide and it silently collapses — the
   trap called out in `navgraph.mjs`.

The placement is `(origin, offset)`:

```
origin {lon, lat}   lon/lat of the DXF datum point
offset {x, y, rot}  the datum point in DXF inches, plus a rotation (radians)
```

## Implementation notes

- `metresPerDegree` uses the USGS Snyder series (a truncated ellipsoid expansion)
  and returns `{ lon, lat }`; it is accurate to well under a metre at campus
  scale and is cheap enough to call per step.
- `makeToLL`/`placementOf` resolve building-vs-floor overrides once and return a
  closure; callers should cache the closure rather than re-resolving per point.
- `DXF_SCALE` is exported so tests and the import tool apply the identical inch→
  metre factor.

## See also

- [`navgraph.md`](navgraph.md) — builds its metre frame from these segments.
- [`campus.md`](campus.md) — portal extraction consumes `toLL`.
- [`roompin.md`](roompin.md) — `crossBarrier` / `nearestApproach` in use.
