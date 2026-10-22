# `campus.mjs` — portals, the outdoor graph, and the tiered router

## What it does

`navgraph.mjs` gives one visibility graph per floor. This module builds
everything above that: how floors connect vertically, how buildings connect to
each other and to the outdoors, and how to plan a trip across all of it.

- `buildCampusRouter(buildings, outdoorShapes?)` → `{ route, buildingAt, stats }`.
  - `route(startLL, endLL, { startFloor, endFloor })` → a route object
    `{ segments, totalSeconds, totalMeters, summary }`, or `null`.
  - `buildingAt(ll, floorId?)` → `{ building, floor }` or `null`.
- Portal extraction and caching: `extractPortals`, `cachePortals`,
  `portalsFromCache`.
- The outdoor network: `buildOutdoorGraph`, `buildObstacleIndex`, `shapeSegments`.

## Why it's built this way

**Two graphs, two scales.** Local navigation (a corridor, a room) is a fine
Dijkstra per floor. Inter-building navigation is a small **macro graph** whose
nodes are *building exits*; its edges are:

- same-building edges: indoor ground-floor path lengths between exits,
- cross-building edges: outdoor path lengths,
- vertical edges: elevator/stair stops linking floors,
- skybridge edges: explicit floor-to-floor bridges (the two ends may be on
  different floors, e.g. HSB 02 ↔ Parrish Bridge 03 ↔ EIB 03).

Routing is then Dijkstra on the macro graph to choose the **chain of buildings**,
with a per-floor `navGraph` Dijkstra filling in the indoor legs. That yields the
optimal chain of any length — including "the fastest path goes through a *third*
building" — without a recursive special case.

**Portals come from DXF layers, not hand entry.** Elevators, stairs and doors are
tagged layers whose names vary in case between buildings, so matching is
case-insensitive. Door *segments* are clustered into **doorways**. A doorway is
an *exit* when a coarse flood fill of the surrounding free space — with every
drawn door sealed so the fill cannot slip indoors — reaches the outdoors through
it. Skybridges are explicit (`floor.skybridges`).

**The outdoor graph is real paths, not straight lines.** When OSM shapes are
supplied, outdoor legs walk an actual path network (`osm.mjs`). Edges that would
cut through a building (stale ways, tile-buffer stitching) are dropped against
the buildings' wall segments, and very short stitches re-join split ways.

**Crossing a road is heavily penalised.** OSM shapes carry their `kind`/`name`,
so the outdoor graph can tell a footpath from a road. An edge that properly
crosses a named `major_road`/`minor_road` (Providence Dr, Piper St, Seawolf Dr,
...) gets `ROAD_CROSS_PENALTY_M` added to its routing weight; walking **along** a
road (parallel/collinear) is free, and unnamed minor roads (parking aisles) are
ignored. Internal campus ways that OSM still tags as roads are exempt by name
(`ROAD_PENALTY_EXEMPT`, currently **Spirit Way** and **Seawolf Drive**). The
penalty only affects route selection — reported metres/seconds are recomputed
from the real geometry — so a skybridge/indoor route wins wherever one exists.

**Why the portal cache.** Extraction (clustering + flood fill) costs seconds per
building. `cachePortals` stores only DXF positions, so the cache is independent
of a floor's placement and never goes stale when a plan is re-aligned.

## Route segments (the tier vocabulary)

Each segment has a `tier`:

| tier | fields | meaning |
|------|--------|---------|
| `indoor` | `building, floor, polyline, meters, seconds` | a leg on one floor |
| `outdoor` | `polyline, meters, seconds` | a leg on the OSM network |
| `elevator` / `stairs` | `building, fromFloor, toFloor, ll` | a vertical connector (the `ll` is the landing) |
| `skybridge` | `building, floor, fromFloor, toFloor, polyline` | a bridge between floors/buildings |

Speeds and costs are constants at the top of the file (`INDOOR_SPEED`,
`OUTDOOR_SPEED`, `OUTDOOR_COST_MULT`, elevator/stairs wait times). Same-building,
same-floor trips short-circuit to a single indoor leg.

## Implementation notes / gotchas

- `buildingAt` uses a per-floor bbox reject, then `navGraph.attach`, retrying on
  a 1.2 m ring. With a `floorId` hint that floor wins (stacked floors share a
  footprint, so centroid-nearest cannot tell "03" from "01").
- Vertical connector stops are matched by **proximity** across floors, seeded from
  the ground floor where present, and from the first floor that has the kind
  otherwise (a stairwell that exists only above ground).
- Skybridge pairing: explicit `to` marks win and may join different floor
  numbers. Unmarked bridges auto-pair by proximity within `SKYBRIDGE_MAX_M`,
  preferring the same floor but allowing a cross-floor pair when no same-floor
  candidate exists. `bridgeUsed` gives each bridge node at most one connection,
  so one node per bridge end/building is sufficient.
- A `skybridge` route segment carries both `fromFloor` and `toFloor`; the app
  draws it on both 2D floors and shows the floor change in the route step.
- Outdoor edges never cross a wall — enforced against `buildObstacleIndex`.
- `stats` reports nodes/edges/exits and build time; the app shows it in the
  calibration panel's debug line.

## See also

- [`navgraph.md`](navgraph.md), [`osm.md`](osm.md), [`geo.md`](geo.md).
- [`apps.md`](apps.md) — how routes are drawn and turned into leg-by-leg rows.
