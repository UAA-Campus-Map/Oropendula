# `watershed.mjs` — room / corridor segmentation

## What it does

Partitions one floor's free space into **room and corridor territories** from
the room labels, then exposes the boundaries between territories as **doors**.
Pure, DOM-free; used by Room Pin and the app's debug region overlay, and by the
`watershed.html` test page.

Export: `watershed({ wallsLL, graph, roomLabels, backgroundSeeds, cell, maxColors })`
→
`{ W, H, cell, x0, y0, x1, y1, labels, blocked, DT, boundary, regions, adjacency, colorCount, seedCount, seedCells, ms }`,
plus `REGION_COLORS` and `BACKGROUND_COLOR`.

## Why it's built this way

The DXF DOOR/DOORS layer is not reliable enough to identify every passage: leaves
are drawn open/closed inconsistently, barriers are offset from the navGraph
passages, and some openings have no leaf at all. Connectivity is the truth — **if
a path can cross from one region to another, it went through a door**. So the
module derives doors from a segmentation instead of from leaf geometry.

## Algorithm

1. **Rasterise the raw walls** at `cell` metres (default 0.2) over the navGraph's
   raster bbox. No navGraph clearance inflation, so real openings stay open.
2. **Distance transform (DT)**: distance from each free cell to the nearest wall.
3. **Marker watershed**: seed one region per room label plus background/corridor
   seeds (stairs, elevators, exits; if none, the widest open cell). Grow with a
   max-heap on DT (deterministic tie-break by cell index), so fronts meet at
   **narrow throats** rather than in the middle of open space.
4. **Adjacency / throats**: boundary cells between two regions are collected and
   split into **connected components** — two separate openings between the same
   two regions are two doors, not one. Each throat's width is `2 × max DT` on its
   component.
5. **4-colouring**: greedy colouring of the region-adjacency graph (4 colours by
   default; a 5th only if a floor forces it).

Everything is deterministic (no simulated annealing); the watershed is cheap
enough to compute lazily per floor (~0.1–0.4 s on the real campus).

## Door rule (consumers)

- A **door** is a throat at least a body wide (`0.6 m`).
- Doors into rooms smaller than **8 m²** (closets, comms, shafts) do not count.
- **More than one door** on a room's territory is ambiguous — Room Pin refuses
  (`Room X has multiple doors.`) rather than guessing.
- The door's barrier is the throat itself, so it cannot be offset from the
  passage the way a leaf-derived barrier can.

## Where it's used

- **Room Pin** (`roompin.mjs`) — primary door/placement path; route probes remain
  the fallback when a floor cannot be segmented.
- **Debug overlay** in `app.html` — Advanced Settings → Compass & graph → Debug →
  *Show regions (debug)* paints the 4-colour territories for the selected floor.
- **`watershed.html`** — test page that shows regions, throats, seeds, and the
  source JSON layers for one building/floor.

## See also

- [`roompin.md`](roompin.md) — the door/placement consumer.
- [`doorways.md`](doorways.md) — DXF leaf-derived barriers (fallback/blocking).
- [`apps.md`](apps.md) — the debug toggle and the test page.
