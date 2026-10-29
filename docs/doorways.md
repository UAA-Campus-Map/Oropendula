# `doorways.mjs` — physical doorways

## What it does

Turns the DOOR/DOORS layers of one floor into **physical doorways**, each with
one **wall-aligned barrier** and a stable geometry-derived id. A doorway is the
wall opening, not the drawn door leaf, so a route can be tested against it and a
blocked door can be suppressed without blocking the leaf's neighbours.

Export: `doorwaysFromShapes(shapes, toLL)` → `[{ id, ll, barrier, width, leaves,
dxfBarrier }]`, plus `barrierCrossesPolyline(doorway, polyline, toM)`.

- `ll` — the opening's midpoint in lon/lat.
- `barrier` — the **wall gap** as two lon/lat endpoints (`dxfBarrier` in DXF).
  This is what a crossing/block test uses; it is *not* the drawn leaf.
- `width` — the opening width in metres; `leaves` — how many drawn leaves made it.

## Why it's built this way

The extractor draws a door either closed in the wall or swung open (with a swing
arc). The opening itself is the wall gap, so a barrier must follow the **wall**,
not the open leaf. Processing:

1. Gather wall segments (`isWallLayer`) into a small uniform grid (`WALL_CELL_U`)
   so a leaf only looks at nearby walls.
2. For each DOOR leaf, derive a wall-aligned barrier: find the wall the leaf's
   hinge sits on, extend along the wall run, and use the leaf's **swing arc** to
   recover the closed (in-wall) direction. Reject implausible openings
   (`JAMB_MIN_M` 0.45 – `JAMB_MAX_M` 2.0 m).
3. **Cluster leaves by union-find**: adjacent, parallel leaves with a small gap
   (`CLUSTER_M` 1.4 m, `CLUSTER_GAP_M` 0.4 m) are one doorway (a double door);
   two separate doors along a corridor are not.
4. Combine each cluster into one barrier and give it a stable, position-hashed
   id (`stableId`), so ids survive unrelated edits.

## Crossing tests

Two primitives live here/`geo.mjs`:

- `barrierCrossesPolyline(doorway, polyline, toM)` — **exact** segment
  intersection with the barrier (used where a strict test is wanted).
- `geo.crossBarrier(polylineM, a, b, toleranceM)` + `geo.nearestApproach(...)` —
  the **tolerant** tests Room Pin and interior-door blocking use. The navGraph
  passage is carved at the DOOR leaf midpoint, a few decimetres off the wall-gap
  barrier, so a route often threads the opening *beside* the barrier; a strict
  in-segment intersection then misses a real door. Tolerant crossing counts the
  barrier line within a tolerance, **or** a pass within 0.815 m (ADA 32 in clear
  width) of the barrier segment.

## Who uses it

- **Room Pin** (`roompin.mjs`) — pick the room's door from the probes' first
  crossings.
- **Interior-door blocking** (`campus.mjs` `interiorPortalsForSegment`) — list the
  interior doors a route crosses, so one can be `avoidPortals`'d.
- `places.mjs` `doorwayFor` prefers a floor's physical `doorways` and falls back
  to raw DOOR-leaf midpoints when a floor has none.

## See also

- [`roompin.md`](roompin.md) — consumes doorways to place the Room Pin.
- [`campus.md`](campus.md) — interior-door blocking.
- [`geo.md`](geo.md) — `crossBarrier` / `nearestApproach`.
