# Campus Router — Agent Notes

Tiered indoor/outdoor wayfinding over five UAA buildings (Anchorage, AK) from
DXF floor plans plus OpenStreetMap data. Plain static site: ES modules, no
build step, no npm dependencies in the app itself.

> **After any task that changes a site file, run `node smoke.mjs` and deploy the
> changed files** (see [Deployment](#deployment)). The live site must always
> match the working tree.

## Architecture

- `navgraph.mjs` — **frozen; only additive changes are permitted.** `buildGraph(
  wallLines)` returns `{findPath(a, b), nodes, stats, isWalkable, nearestWalkable}`;
  `findPath(p, p) !== null` doubles as a point-inside test (basis of
  `buildingAt`). Walls are *obstacles*; it routes free space around them on a
  ~0.1 m grid extending ~0.45 m past the wall bbox. `isWalkable(ll)` is an O(1)
  lookup into that same clearance raster (used by the PDR particle filter and
  the GPS-validity gate); `nearestWalkable(ll, radiusCells)` spirals out to the
  closest free cell. These two are the *only* additions; existing behaviour is
  untouched.
- `geo.mjs` — the registration math, shared by everything: `DXF_SCALE`,
  `metresPerDegree`, `dxfCoordsToLonLat`, `lonLatToDxf` (inverse),
  `makeToLL(building, floor)` (floor-level origin/offset wins, AS106 style).
  index.html, smoke.mjs and import.html all import from here.
- `campus.mjs` — the tiered router. `extractPortals(shapes, toLL)` pulls
  elevators/stairs/exits from tagged DXF layers; `buildCampusRouter(buildings,
  outdoorShapes?)` links connectors across floors, builds the macro graph over
  building exits, and answers `route(startLL, endLL, {startFloor, endFloor})`.
  Also exports `shapeSegments` (true geometry from `items` — see below).
- `osm.mjs` — translates OSM road/path data into the custom shape format on a
  `PATHS` layer (see below). Direct reader over the basemap PMTiles archive.
- `pmtiles-node.mjs` — dependency-free PMTiles v3 reader (header + leaf-dir
  walk + Hilbert tile IDs) for Node; used by smoke tests only. Byte-identical
  output to the official library.
- **`buildings-config.json`** (repo root) — the building registrations. index.html
  fetches it at boot (top-level await; **no fallback** — the app will not boot
  without it). Produced by import.html.
- **`import.html`** — the building/floor import tool: upload a building JSON
  (prebuilt by the external DXF extractor; DXF is *not* parsed in-browser),
  prefill from its metadata (`building_code`/`building_name`/`level`), drag the
  plan onto the basemap (draggable datum dot + rotation ring, arrow-key nudges,
  degrees rotation UI storing radians), edit per-floor overrides, and download
  a drop-in `buildings-config.json`. Persists work-in-progress to localStorage.
  MapLibre gotchas hit here: this build's `Evented.on()` returns
  `{unsubscribe}`, **not** the marker (never chain `.on(...).addTo(...)`), and
  `LngLat` properties are `lng`/`lat`. Marker handlers must read the placement
  at event time — the marker outlives refreshes.
- `motion.mjs` — PDR signal processing + sensor wiring. Pure, testable classes:
  `StepDetector` (scalar `|a|` high-pass + dynamic-envelope peak detection — see
  below), `HeadingTracker` (circular compass smoothing + gyro bridging + bias),
  `StepLengthModel` (cadence-scaled, clamped), `WalkCalibrator`.
  `createMotion(opts)` normalises DeviceMotion/DeviceOrientation (iOS needs a
  user-gesture permission and reports `webkitCompassHeading`). Headings are
  radians clockwise-from-north.
- `locate.mjs` — map-matching particle filter. `ParticleFilter` advances each
  particle one step per detected step and weights by `isWalkable`. `corridorAxis
  (isWalkable, ll)` ray-casts the free space to find a corridor bearing (null in
  open rooms).
- `arbiter.mjs` — `PositionArbiter` runs GPS and PDR **at the same time** and
  picks the smaller uncertainty (GPS `coords.accuracy` vs a PDR error that grows
  with distance walked). Hysteresis stops flapping; a validity gate rejects
  indoor fixes that land in a wall; a good fix re-seeds the filter (GPS as a
  free anchor). No mode switch — transitions happen automatically.
- `locate-test.html` — **test-only** live map for positioning: loads the floor
  plans + basemap, draws the GPS fix (blue) and the GPS/PDR estimate (red) with
  its uncertainty circle, heading arrow and breadcrumb trail, and lets you tap
  to place a manual anchor or replay a synthetic walk. Plus sensor/step/heading
  metrics and calibration. Never referenced by prod.
- `index.html` — the demo: loads buildings, asks osm.mjs for the road network,
  builds the router, draws portals and tiered routes, and drives the live marker
  from the arbiter (GPS + PDR). "I'm Here" taps a manual anchor at the map
  centre.
- `smoke.mjs` — the test gate (`node smoke.mjs`). Check count scales with
  exit count (~96 currently); **all must pass**, zero failures tolerated. It
  reads its registrations from `buildings-config.json` — the same file
  index.html fetches, so the two can never drift.

## Routing model

Tiers, in order: indoor → vertical (elevator/stairs) → indoor → outdoor →
…repeat for intermediate buildings… → mirror into the destination.
All costs are **seconds**: indoor 1.3 m/s, outdoor 1.4 m/s, elevator 15 s +
4 s/floor, stairs 6 s/floor.

**Outdoor legs are heavily penalised for route selection** (`OUTDOOR_COST_MULT`,
currently 15): Fairbanks reaches -40 C, so the router must take a skybridge (or
any indoor detour) over stepping outside whenever one exists. The multiplier
inflates only the Dijkstra cost of outdoor macro edges / attachments / the
direct outdoor walk — segment `seconds` are always recomputed from the real
speed, so reported ETAs stay honest. Outdoor is still used when nothing indoor
connects the two points (e.g. AS103 ↔ AS102, which share no bridge).

The macro graph node is an exterior door, a skybridge portal, **or a connector
stop** (an elevator/stairs landing on a floor). Modelling connector stops as
nodes lets the route change floors *anywhere* — at the start, the destination,
or an intermediate building — rather than only descending at the start and
ascending at the end. That earlier limitation produced bizarre routes: to reach
a bridge a floor above the start, it would go outside, around a building and
back in, because origin/destination costs only knew about the ground floor.
Edges:
- same building, same floor: indoor walk between any two nodes (one Dijkstra per
  node builds the whole floor);
- same connector, different floors: vertical ride (elevator/stairs);
- two bridges, same level, different buildings: skybridge (explicit `to` destination, else greedy min-distance matching);
- cross-building ground-floor doors: outdoor path-network length.

`route()` attaches the start/end point to the nodes on its own floor (doors,
bridges, connector stops), runs one Dijkstra over the macro graph with a virtual
source/sink, and rebuilds each leg's polyline from its edge descriptor.

`route()` never takes a same-building/same-floor shortcut wrongly: if both ends
are in the same building *and* floor it returns a single indoor leg, otherwise
it goes through the macro graph.

## Exterior doors (exits)

**Exterior doors are placed by hand in the import tool**, not trusted from the
algorithm. The app reads `floor.exits: [{id, dxf}]` from buildings-config.json
and uses those when present; otherwise it falls back to the algorithm's exits
in `floor.portals.exits`. The import tool:
- shows the effective doors as dots (red = manual, orange = algorithm);
- **Place door** / **Remove door** modes: a map tap adds a door at that DXF
  point, or removes the nearest manual door within 3 m;
- **Seed from algorithm** copies `portals.exits` into `exits` (edit a starting
  point), **Clear** empties the manual list.
Manual doors are why two buildings joined by a skybridge across **separate
DXFs** work: place the missing door on one floor and delete the surplus on the
other, then mark a **skybridge** portal on each side (see below).

### Skybridges are explicit portals (`floor.skybridges`)

A skybridge is its own manually-placed portal type, **not** an exit. The import
tool's **Place bridge / Remove bridge** modes write
`floor.skybridges: [{id, dxf, to?}]`; the **destination** select marks `to`
(another building's id) on newly placed bridges and **Set destination**
re-marks an existing dot (with "auto (nearest)" selected it clears the mark).
The map labels each dot with its destination.

Pairing (`buildCampusRouter`): **explicit destinations first** — candidate
pairs where either side's `to` names the other's building: mutual marks, then
one-sided, each shortest-first, one partner per portal. Explicit links ignore
both `SKYBRIDGE_MAX_M` and the floor number — the marks say exactly which
bridge goes where (AS121-01 ↔ AS119-02 is ~126 m and crosses a grade change;
the earlier same-floor rule and the 120 m cap both blocked it). Unmarked
bridges then fall back to greedy min-distance matching: same floor *number*
(so DXFs labelling the storey "2" vs "02" still link) and within the cap, and
a bridge with a destination never auto-pairs elsewhere (no accidental link to
a nearer neighbour). Strict mutual-nearest was the original bug source:
AS119's ECB-side bridge had AS117 closer, so AS117 took it and ECB linked to
nothing. Only bridges pair, so an ordinary rooftop door facing a neighbour
cannot connect a building; ground-level bridges are allowed too (AS117-01 ↔
AS119-01 share a first floor across two DXFs). Mark **both** sides and set
the destination on each.

The algorithm below is what `extractPortals` still computes (cached as the
seed/fallback for doors):

A doorway is an exit when a **coarse flood fill** of the space around the
footprint — with every drawn door opening sealed so the fill cannot slip
indoors — shows the outdoors reachable through that doorway. Details that
matter:

- Doors are clustered **leaves only**; swing arcs are attached to the doorway
  they root at. Clustering leaves+arcs merges the two rows of an arctic entry
  into one bogus doorway whose centre drifts off the outline.
- Each doorway owns its seal; its flood test may pass through its own opening
  (arcs included) but never through another doorway's.
- An exit's leaf must sit within `OUTLINE_MAX_M` of the outline.

### The seal may not cut through a wall run (AS102's phantom exit)

A doorway's seal punches through the halo its own wall's *ends* throw across
the opening — that is what makes the opening passable at all. It must **not**
cut through a wall or window that merely *runs past* the door, however close:
AS102's room-113 door stands ~0.15 m from a north–south window wall, and
sealing through that run let the door's exterior BFS walk straight out through
the glass and mark an inner door exterior. So the grid records `wallRun`
(cells blocked by a segment's interior, not by an end halo) with the run's
direction, and a seal may only override a run cell when the run is within 30°
of the doorway's own wall direction (the wall segment closest to the door's
drawn leaf/arc geometry) or the door's drawn geometry touches the cell within
`RUN_EPS` (0.1 m). Windows are solid (`WINDOWS` is in the wall set). Attempted
shortcuts that do NOT work and must not be retried: sealing leaves only
(unseals every open-swing doorway → interior doors flood green), and blocking
all run cells unconditionally (kills genuine exterior doors).

### Wall geometry: always read `items`, never the bbox

`shapeSegments(shape)` (campus.mjs) returns a shape's true segments from
`shape.items` (`line`/`polyline` points, plus the closing segment when
`shape.closed`; arc items carry no vertices and are skipped). A shape's bbox is
only its extent: **using the bbox diagonal as the geometry corrupts every
slanted wall and multi-vertex polyline** — a rotated wall run renders as a
diagonal slash across the corner it turns, a wall run can look "missing" when
it is just rotated, and the navgraph, the flood fill and the outdoor obstacle
set all inherit the corruption. index.html's `loadFloor` and smoke.mjs build
their wall sets through `shapeSegments`; anything reading DXF shapes must too.
This fixed AS106 (Lucy Cuddy Hall): 27 exits dropped to 12, the four other
buildings unchanged (AS105 gained one).

## Portal cache (performance)

`extractPortals` (doorway clustering + coarse flood fill) costs **~0.2–1.5 s
per floor** — ~4 s for the whole campus. The import tool computes it once and
stores the result on each floor entry of `buildings-config.json` as
`floor.portals`, so the app skips it:

```json
"portals": {
  "elevators": [{ "id": "elevators-0", "dxf": [x, y] }],
  "stairs":    [{ "id": "stairs-0",    "dxf": [x, y] }],
  "exits":     [{ "id": "exits-0",     "dxf": [x, y] }]
}
```

- **Only DXF positions are cached** (`cachePortals` / `portalsFromCache` in
  campus.mjs). `extractPortals` works entirely in DXF space and touches lon/lat
  only at the very end, so the cache survives re-aligning/rotating a plan.
  `index.html`'s `loadFloor` prefers `portalsFromCache(floor.portals, toLL)` and
  falls back to `extractPortals` when a floor has no cache.
- **Import tool**: `ensurePortals` caches a floor when its JSON loads; it uses
  an identity `toLL` (dxf is placement-independent) so it works before a floor
  has a registration. Re-importing a floor JSON drops and recomputes its cache;
  **Download** populates every configured floor before serializing.
- **`gen-portals.mjs`** (dev-only) bootstraps/refreshes the cache for the whole
  config in one shot. **Run it whenever a floor JSON changes**, then deploy
  `buildings-config.json`.
- smoke.mjs verifies every shipped floor's cache equals a fresh extraction.

## Performance

The campus grew to ~11 floors / 115 exits and load was ~40 s. Fixes so far:

1. **Portal cache** (above) — skips `extractPortals` (~4 s).
2. **Exit distance matrix, O(k) not O(k²)** — the macro graph used to call
   `navGraph.findPath()` for every pair of exits on a floor (888 pairs ×
   ~28 ms ≈ 25 s). navgraph.mjs now exposes an additive attachment API:
   `attach(ll)` (visible nodes + stub distances), `dijkstraFromAttach(att)`
   (one Dijkstra from an attachment set), `distToAttach(run, att)`, and
   `pathToAttach(run, att)`. `buildCampusRouter` attaches each exit once and
   runs **one Dijkstra per exit** to fill the whole floor matrix; `buildingAt`
   uses a per-floor bbox reject + `attach`; and `origin/destinationCosts`
   compute costs from one attach + one Dijkstra, with polylines materialised
   only for the exit the route actually uses (`originSegments` /
   `destinationSegments`). Result: `buildCampusRouter` 29 s → ~3 s, per-route
   934 ms → ~70 ms. The API is exact (same visibility test as `findPath`), so
   no behaviour change.
3. **OSM paths cache** — `gen-osm.mjs` decodes the pmtiles archive once and
   writes `osm-paths.json` (bbox + shapes); index.html loads it when its bbox
   covers the campus, else decodes live. Removes ~6 s. **Re-run
   `gen-osm.mjs` and deploy `osm-paths.json` whenever the building set changes**
   (the campus bbox grows and the cache stops covering).
4. **Selective floor drawing** — floor plans used to be added to the map for
   every floor at boot (visible), then hidden once the picker was ready, so all
   plans flashed during load and map time went to floors nobody viewed.
   `loadFloor` now only returns the plan data (walls FC, room-label FC);
   `ensureFloorLayers` creates a floor's sources/layers the first time that
   floor is actually drawn, and `applyFloorFilter` keeps only the selected
   floor visible. Loading is still **eager** — JSON, navGraph, portals and the
   room index are all built for every floor at boot; only the map layers are
   on demand. Layers are inserted before `portals` so dots/route stay on top.
5. **Server compression + HTTP/2** — nginx was serving every text asset raw
   over HTTP/1.1. `gzip on` + explicit `gzip_types` (json/js/mjs/…) in the
   http block cuts the first load from ~13 MB to ~1 MB (floor JSONs ~20x:
   AS119-01 1.13 MB → 53 KB; style.json 603 KB → 24 KB), and `http2 on`
   multiplexes the parallel floor fetches over one connection. Config lives in
   `/etc/nginx/nginx.conf` (the server block itself; `default.d/cache.conf`
   keeps `expires -1` → cheap 304 revalidation, so deploys are never stale).
6. **Parallel boot** — index.html used to `await` each floor fetch in a
   `for` loop (network serialized with CPU) and only then fetched OSM.
   It now prefetches **every** floor JSON concurrently at module load
   (`floorPrefetch`, storing `{data}|{error}` so a failure cannot surface as
   an unhandled rejection) and starts the OSM cache fetch using a
   config-derived bbox (`outdoorBBoxFromConfig`, mirroring gen-osm.mjs), so
   neither waits for navGraphs. `style.load` awaits the prefetches and builds
   floors sequentially (CPU-bound), yielding per floor so the status line
   paints. `<link rel=modulepreload>` for the three maplibre modules and
   `preload as=fetch` for buildings-config/osm-paths start those downloads
   during HTML parse. Measured from the live site: floor network 12.6 s serial
   raw → 1.3 s parallel gzipped.

Remaining first-load cost is **CPU on the main thread**: with 8 buildings /
17 floors, `buildGraph` measures ~6.7 s and `buildCampusRouter` ~2.5 s
(JSON.parse is only ~100 ms). Rebuilding the navGraph visibility graph is
inherent; the next levers are a Web Worker build (needs serialising the graph
data, since the returned object carries closures) or an IndexedDB cache keyed
by a wall hash. Lazy/background navGraph build would improve perceived
startup.

## The OSM layer (osm.mjs)

The basemap at `https://navigator.cheeseverse.net/style.json` is a Protomaps
extract of OpenStreetMap served as one PMTiles archive
(`https://navigator.cheeseverse.net/scak.pmtiles`, South Central Alaska,
max zoom 15, CORS `*`). **No Overpass** — the app already downloads this
archive for the basemap.

- `loadOsmShapes({url, bbox, zoom=15, fetchTile?})` fetches the z15 tiles
  covering `bbox`, decodes MVT with a **small built-in decoder** (no runtime
  dependency), keeps `kind ∈ {path, minor_road, major_road}` minus
  `access ∈ {private, no}`, dedupes tile-buffer copies, and emits shapes in the
  same format as the DXF files — `{bbox, layer:'PATHS', items:[{op:'line',
  points}]}` where points are `[lon,lat]` (identity transform).
- Browser fetch: `pmtilesFetcher(url)` uses `globalThis.pmtiles` (already
  loaded for the basemap); Node: pass `fetchTile` (see `pmtiles-node.mjs`).
- Campus needs 4 z15 tiles (~60 KB, ~0.5 s). Decoder output is verified
  byte-identical to `@mapbox/vector-tile`.
- Layer names beyond doors matter too: portal matching is case-insensitive
  (`DOORS`, `STAIRS`, `elevator`); walls are matched case-insensitively
  (`WALLS`/`Windows`/`WINDOWS`).

## Outdoor graph invariants (hard-won)

`buildOutdoorGraph(shapes, lat, obstacleSegs?, bridgeM?)` in campus.mjs:

1. **Roads only.** The graph is real OSM way edges plus *very short* stitches
   — dangling ends within `OUTDOOR_BRIDGE_M` (25 m ≈ the 2×64 px tile-buffer
   overlap at z15). Longer joins invent paths across unmapped ground; an old
   60 m radius produced routes that were mostly invented edges and detoured
   far off the mapped roads.
2. **Never through a building.** Every edge — real way, stitch, or straight
   fallback — is tested against an obstacle index built from the buildings'
   ground-floor wall segments (`floor.obstacleLines`, lon/lat). The index uses
   a 10 m grid with stamped dedup; the test is a proper intersection with an
   endpoint margin so edges meeting a wall at a doorway pass. Stale OSM ways
   that cross a footprint get dropped here.
3. **Attachment projects onto edges, not just vertices.** `snapClear(ll)` goes
   to the nearest node *or the nearest point along an edge* whose straight stub
   does not cross a wall, within `OUTDOOR_MAX_SNAP_M`; a projection splits that
   edge and returns the new node. Vertex-only snapping dragged an exit/click
   that faced the middle of a long way out to the way's nearest vertex — often
   tens of metres away — so a path that visibly ran under the finger was never
   used and the route detoured around it. (Snap both ends *before* running
   Dijkstra: an attach may add a node index the other run's array lacks.)
4. **Outdoor-to-outdoor trips get a direct network leg.** The macro graph only
   expresses an outdoor trip as exit → exit, so a walk straight along a path
   that passes no entrance used to detour to the nearest entrance and back —
   the "comically large detour" over a displayed path. `route()` now also
   builds the network path between the two clicks and takes whichever is
   quicker (a chain *can* still win by cutting through a building).
5. Straight-line outdoor legs only survive as a fallback when the path
   network is absent or the segment is clear of buildings. A leg that cannot
   be served without cutting a building is *omitted*, not drawn.

Smoke enforces each invariant, including "all building pairs route with zero
outdoor segments crossing a wall", Test 7c (mid-edge attachment lands on the
edge; a 20 m span of edge walks in 20 m) and the Test 8 live check that tapping
two points on one 305 m way walks 305 m.

## Building registration

Registrations live in **`buildings-config.json`** — edit it through
`import.html`, never by hand in index.html. Entry shape:
`{id, groundFloor, origin?, offset?, floors:[{floorId, url, origin?, offset?,
showRoomLabels, wallLayerId?, roomLayerId?}]}`.

- `origin {lon,lat}` is the lon/lat of the DXF datum; `offset {x,y,rot}` is
  the datum in DXF units (inches) plus rotation in **radians**. import.html
  shows/edits rotation in degrees and stores radians.
- Placement is applied per floor (`floor.origin/offset` win) — the tool always
  writes floor-level placement (AS106 style).
- `wallLayerId`/`roomLayerId` default to `${id}-${floorId}-walls|roomnumbers`
  (lowercased); the tool only stores them as **manual overrides** when non-
  default (AS106 needs this today).
- All plans are inches (`DXF_SCALE = 0.0254`); a "mm" scaling attempt was
  wrong and was reverted. Whatever you set **must** go through `makeToLL` so
  walls, labels, portals and the import tool all agree.

Current registrations: AS103 `{0,0,0}`, AS101 `{2880,390,0.005}`,
AS102 `{1392,1144,0}`, AS105 `{267,0,119.39}`, AS106 `{475,400,0}` (floor-
level). AS112 exists in the repo but is unregistered — add it with import.html.

Current exit counts after the geometry and seal fixes: AS103 9, AS101 8,
AS102 7, AS105 14, AS106 12 (50 total). Several still-drawn doorways are
correctly interior (open-swing inner doors, doors onto walled vestibules);
known leftovers the user deferred: AS105 has fewer exits than the drawing's
facade suggests, AS103 lost one.

## Positioning (PDR + GPS)

Target: a few metres indoors, on-device, offline, no extra hardware, browser
PWA (Chrome Android / iOS Safari / Firefox). No CV — the camera was excluded
and a learned "mouse sensor"/optical flow was judged the wrong tool for a
4-5 m target. No ML either: the floor plan does the heavy lifting.

- **Step detection lesson 1**: remove the baseline with a *scalar* high-pass on
  `|a|`, not per-axis gravity subtraction + magnitude. A train of unipolar step
  pulses has a non-zero mean, and per-axis gravity tracking leaks its lag back
  into the signal, leaving a baseline that swamps light gait. The shipped
  detector (`StepDetector`) uses a dynamic envelope threshold, so it is robust
  across cadence 1.2-2.4 Hz, acceleration amplitude 1.5-4 m/s² and noise.
- **Step detection lesson 2 (the "counts when I move a bit" bug)**: a peak is
  not a step. Shifting the phone / shifting weight produces isolated peaks too.
  `StepDetector` now requires a *walking rhythm* before counting: a run of
  `qualifySteps` peaks whose spacing stays within `rhythmLo..rhythmHi` of the
  previous interval. While unconfirmed, peaks are buffered and flushed the
  moment a walk is confirmed (so no real steps are lost); a gap past
  `maxIntervalMs` ends the walk. `push()` therefore returns an **array** of
  step timestamps (possibly several on confirmation), not a boolean.
- **Heading** is trusted from a calibrated compass (the user's call), with gyro
  bridging through magnetic disturbances and a captured bias. Wall-point
  calibration: stand at a known anchor, point at a known wall, set the bias so
  the heading matches the wall's true bearing (also calibrates location).
  Two hard-won gotchas: (1) **never mix heading frames.** By default
  (`headingMode: 'absolute'`) `createMotion` trusts only
  `deviceorientationabsolute` (plus iOS `webkitCompassHeading`) because the
  plain `deviceorientation` alpha is an arbitrary frame that would otherwise
  jump in and corrupt the heading; `headingMode: 'relative'` does the opposite,
  using only the plain event — engaged by the calibration walk, which measures
  the frame's offset from true north. (2) The magnetometer jitters several
  degrees per sample, so `HeadingTracker` low-passes it with a **time-based**
  blend (`1-exp(-dt/tau)`, tau 0.5 s, frame-rate independent) and the gyro only
  bridges when the compass is stale (`gyroBridgeAfterMs`).
- **GPS and PDR run simultaneously** — no indoor/outdoor mode flag. The arbiter
  picks the smaller uncertainty with hysteresis. Critical: indoor GPS accuracy
  is often optimistically small *and wrong*, so fixes landing in a wall (when
  near a known building) are discarded. GPS acts as a free anchor when good.
- **Anchors** are manual: press "I'm Here", then tap the map to place your
  location (not the map centre); the tap re-seeds the particle cloud and resets
  drift. The **displayed floor is authoritative**: `anchorAt` passes
  `floorOfView()` into `ensureLocator`, which uses
  `campusRouter.buildingAt(ll, hint)` to bind building+floor. Floor picks by
  wall centroid cannot tell stacked floors apart (01/02/03 share a footprint),
  so before this an upper-floor anchor bound to the ground floor and the PDR
  stayed on the wrong plan. GPS fixes and boot still use the centroid pick
  (they carry no floor). The status line reports the bound `ASxxx NN`.
- **Step-length calibration** is in the index.html "Calibrate" panel (it
  replaced the old nav sidebar): place a start and end point on the map (green/
  red markers, straight-line distance via turf), press Start, walk the line,
  press End → step length = distance / steps counted. Apply stores it to
  `StepLengthModel.base` and `localStorage['pdr-step-base']` (restored on load).
  The panel has a "Done · Hide" button.
- **Heading calibration rides on the same walk.** During the line walk the
  device orientation is known (the phone points along start→end), so the panel
  also accumulates the mean heading and computes the offset from the line's
  true bearing. Pressing Start switches `createMotion` to `headingMode:
  'relative'` (the plain `deviceorientation` alpha - a stable but arbitrary
  frame) and resets the tracker; End sets the bias, Apply persists
  `pdr-heading-bias` + `pdr-heading-mode=relative`. Absolute and relative
  frames are never mixed (that caused the old "haywire" heading). This lets the
  device avoid the unreliable indoor magnetometer after one calibration walk.
- **The filter must be seeded or PDR silently does nothing.** `ParticleFilter.
  predict()` is a no-op until `seed()`. So when a locator is created,
  index.html/locate-test snap the seed to the nearest walkable point; the
  arbiter also seeds on the first plausible GPS fix. Without this, steps are
  counted and heading is fine but the marker never moves (the "PDR not
  implemented" bug).
- Prod ships no telemetry; all tuning/overlays live in `locate-test.html`.

## Testing

`node smoke.mjs` — the only gate; run it after any change to campus.mjs,
geo.mjs, osm.mjs, buildings-config.json or a building registration. It covers:
portal sanity, exits outside the footprint / near real doors, exit reachability
for every building, six end-to-end routes (same floor, cross-building, outdoor
click, vertical, cross-floor, cross-campus), a hermetic synthetic outdoor grid
incl. the no-network fallback, and a live OSM test (network — degrades to
skipped offline). It also covers the positioning stack on synthetic signals:
`isWalkable`/`nearestWalkable` consistency, step detection vs a synthetic gait
across cadence/amplitude/noise, heading smoothing/gyro/bias, the particle
filter's kinematics and map constraint, and the GPS/PDR arbiter's validity
gate, drift growth, re-seed and hysteresis. Debug scripts live in
`/tmp/opencode/` (`dbg-*.mjs`) and are not part of the repo.

For registration changes, open `import.html` (serve the dir the same way) and
eyeball the plan over the basemap before trusting `buildings-config.json`.

## Deployment

**Deploy as part of finishing any task that changes a site file.** The live
site should always reflect the working tree. Workflow for every task:

1. Run the gate: `node smoke.mjs` must be all green.
2. Deploy **only the files you changed** (see below), then verify.

The site is served from `root@navigator.cheeseverse.net:/usr/share/nginx/html`.
`deploy.sh` (git-ignored) shows how - the host, the destination path and the
`scp` usage; read it before deploying:

    scp -r *.html root@navigator.cheeseverse.net:/usr/share/nginx/html
    # ...and *.png, *.json, *.mjs

**Do not run `deploy.sh` for a normal change** - its globs re-upload the whole
directory (every floor JSON, the ~30 MB `scak.pmtiles`, the MapLibre bundle)
every time. After a major change:

1. Run the gate: `node smoke.mjs` must be all green.
2. Upload **only the files you changed**, naming them explicitly, e.g.:

       scp index.html motion.mjs locate.mjs arbiter.mjs \
         root@navigator.cheeseverse.net:/usr/share/nginx/html/

List the changed paths with `git status --short` first so nothing is missed and
nothing extra is sent. The large static assets (floor JSONs, PMTiles, MapLibre
`*.mjs`) only need re-uploading when they themselves change.

## 2D / 3D view (index.html)

The app **starts in 2D** (flat, north-up) and a **2D/3D button** toggles the
extruded view. `is3D` drives everything: camera (3D eases to pitch 50 and
enables drag-rotate/two-finger pitch; 2D eases pitch/bearing to 0 and disables
rotation), the per-floor flat wall/label lines and connector dots (2D only), and
the extrusion layers. A floor's wall/label layers are created on first display
(`ensureFloorLayers`) and only the selected floor is ever visible (see
Performance). `renderRoute` draws the 3D ribbon (whole route) and, for
2D, a solid line filtered to the selected floor plus a faded (25% opacity)
ghost of the route's other-floor segments — so a route that changes floors
reads as continuing instead of ending at a stairwell. Whichever matches the
mode is shown.

### The 3D model (fill-extrusion)

This MapLibre build has no `line-z-offset`, so extrusions are the only way to
lift geometry (the MapLibre "display buildings in 3D" / "extrude polygons for
3D indoor mapping" examples are the pattern):

- Elevation: `FLOOR_HEIGHT_M = 5.5`, base = `(floorNum(floorId) - 1) * 5.5`; a
  floor's walls extrude `base -> base + 5.3`.
- Static structure source `campus-3d` (built once, `buildStructure3D`): each
  wall `obstacleLines` segment becomes a thin quad (±0.125 m, pure arithmetic),
  plus one floor plate per floor = the **convex hull** of its wall endpoints
  (NOT the axis-aligned bbox — that overhangs badly on a rotated building,
  AS117 is ~19 deg). It is drawn as **one `fill-extrusion` layer per floor
  level** (`campus-3d-level-k`), added in ascending order AFTER the route, each
  `fill-extrusion-opacity: STRUCTURE_OPACITY` (0.22, x-ray), data-driven
  base/height. Walls are
  darker than the floor plate (`kind` property drives the colour). Why per
  level: transparent fill-extrusions still **write depth** in this build, so a
  single combined layer depth-rejects everything behind it (looks solid) and
  upper floors cannot blend over lower ones.
- Top-level outline: the selected (topmost displayed) level's walls are split
  into two **stacked** extrusions. `campus-3d-top-outline` draws the bottom
  `base -> base + TOP_OUTLINE_M (0.3)` volume in solid black, and
  `applyStructureStyle` raises that level's `fill-extrusion-base` to
  `base + TOP_OUTLINE_M`, so its translucent wall sits directly above with no
  overlap. **Order is load-bearing**: this build's fill-extrusion painter
  draws EVERY extrusion in the translucent pass, in style order, with depth
  writes (`LEQUAL, ReadWrite`) even at opacity 1 (verify in the bundle:
  `if(i.currentPass==='translucent')`). A layer added after the walls renders
  last and is depth-rejected behind any nearer wall; added FIRST, the walls
  blend over it exactly like they blend over the route. So the outline layer
  is added **before** the `campus-3d-level-*` layers. The filter follows the
  selected level; the layer is hidden in 2D and in "Find Path" mode (whose
  selected walls are already solid black).
- Route source `campus-3d-route` (rebuilt in `renderRoute`, opaque): each route
  segment polyline becomes a raised ribbon (~0.35 m wide, `base+0.1 -> base+0.6`,
  coloured by tier); each vertical leg (which now carries `ll`, the connector
  landing) becomes a riser box between the two floors' bases.
- Floor picker shows **floors 1..n** (n = selected level) by toggling the
  per-level layers; the top displayed level's walls are drawn slightly darker
  (`#1f2328` vs `#2b3036`, no blue highlight) so the top of the stack reads,
  and the floor plates share one colour (`#6b7683`). The route is always drawn
  in full across floors (so it can float above the shown floors).
- **"Find Path" mode** (`findPathMode`): while picking the two taps, the floor
  plates are hidden and the selected level's walls are drawn solid black
  (opaque) so the room layout reads, and the selected floor's room-number
  `symbol` layer is shown (yellow, Noto Sans Medium — the basemap glyph server
  404s Open Sans/Arial). Both revert when the route is computed. Room labels
  can't be elevated in this build, so they sit on the ground plane.
- The old 2D wall `line` / room-label `symbol` layers and connector dots are
  hidden in 3D — symbols can't be elevated in this build, so they would sit at
  ground under the stack. The destination marker is a DOM marker in 2D and a
  raised pin extrusion on the destination's floor in 3D (DOM markers cannot be
  lifted).
- Camera: constructed with `dragRotate/pitchWithRotate`, `pitch: 50`,
  `maxPitch: 75`; the user pitches/rotates freely.

## Building / room search (index.html)

The search box takes "<building code> <room>", e.g. `BMH 114A`. The building
code is the floor JSON's `ID` field (BMH, ESH, SMH, RH, CUDY, AASC, SU); the
registration id (`AS103`) also works, and the longest known code prefix wins so
`BMH213` and `AS103213` split correctly. Rooms come from the **same
`roomLabels` array that feeds the displayed room-numbers `symbol` layer**
(built from the DXF `text_lines`, see `loadFloor`), so search and labels can
never disagree. Matching is case/space-insensitive, exact first then prefix;
an exact hit on the floor whose level equals the room's first digit wins
(`114A` -> floor `01`). `runSearch` routes from the live GPS/PDR estimate when
"My Location" is running, else the map centre, makes the hit the destination
marker/pin, jumps the floor picker to its floor and flies there. AS112 (WWA)
is not registered in `buildings-config.json`, so it is not searchable.

## Browser demo notes

- Connector dots are drawn from one `portals` source in 2D, filtered to the
  displayed floor: stairs orange, elevators purple. Same-kind dots within
  1.5 m collapse (one connector drawn as several clusters) — routing still
  uses all nodes. Doors are not drawn.
- Route tiers are coloured indoor red / outdoor blue / elevator purple /
  stairs orange / skybridge teal.
- If the OSM fetch fails the router still builds and works with straight-line
  outdoor legs — that path must keep working.
- Serve with `python3 -m http.server 8765` → http://localhost:8765.
