# Campus Router — Agent Notes

Tiered indoor/outdoor wayfinding over five UAA buildings (Anchorage, AK) from
DXF floor plans plus OpenStreetMap data. Plain static site: ES modules, no
build step, no npm dependencies in the app itself.

**Canonical production site:** `https://uaa-campus-map.net` (served from
`/usr/share/nginx/html`). The previous hostname is retired; use the new domain
in code, docs, deployment, and import-tool settings.

> **After any task that changes a site file, run `node smoke.mjs` and deploy the
> changed files** (see [Deployment](#deployment)). The live site must always
> match the working tree.

> **Per-module documentation lives in [`../docs/`](../docs/README.md)** — one page
> per module explaining what it does, why it is built that way, and its
> implementation invariants. This file stays the single narrative of hard-won
> context; the module pages expand it.

## Architecture

- `navgraph.mjs` — **frozen; only additive changes are permitted.** `buildGraph(
  walls, doors = [])` returns `{findPath(a, b), nodes, stats, isWalkable, nearestWalkable}`;
  `attach(ll)` returns the nodes a point can see and is the basis of
  `buildingAt` and route attachment. Walls are *obstacles*; it routes free space around them on a
  ~0.1 m grid extending ~0.45 m past the wall bbox. `isWalkable(ll)` is an O(1)
  lookup into that same clearance raster (used by the PDR particle filter and
  the GPS-validity gate); `nearestWalkable(ll, radiusCells)` spirals out to the
  closest free cell. Other additive surface: the attachment API (Performance,
  item 2) and `buildGraphState(walls, doors = [], { passages })` /
  `graphFromState(state)` — the graph as
  serializable raster/nodes/adjacency state plus a rehydrate step, so floors
  can be built in Worker threads; `buildGraph` is exactly
  `graphFromState(buildGraphState(walls, doors))` and unchanged in behaviour.
  Existing behaviour is untouched. `navgraph-codec.mjs` packs the serializable
  half into compact per-building `<id>.nvg.gz` bundles: Float64 nodes, undirected
  CSR adjacency, RLE blocked raster, no weights, no passages.
  `buildings-config.json` points at them with `graphUrl` and stores each floor's
  `graphHash`, the building `graphFingerprint` and `wallBBox`; each bundle floor
  also carries `sourceHash`, so boot rejects a bundle that does not match its
  floor JSON. `gen-navgraphs.mjs` and the import tool's Deploy section generate
  them. See
  `docs/navgraph-bundles.md` for the byte layout and codec/runtime API.
- `walkable.mjs` — the import-time **indoor walkability** mask.
  `floodIndoor({ wallsLL, extraWallsLL, doorways, seals, sealPoints,
  indoorSeed, outdoorSeeds, outdoorSeed })` floods the outside from the
  wall-bbox ring, seals
  interior doorways, exterior exits and skybridges, and returns
  `{mask, leak, ...}`. `buildGraphState(..., { mask })` blocks every cell
  outside the mask before nodes/edges are built. Bundle generation refuses on
  `error`/`leak`. See `docs/walkable.md`.
- `import-v2.html` — the six-step authoring wizard (Building → Floors →
  Placement → Connections → Walkable → Artifacts). It shares `import.html`'s
  config key and modules; all v1 controls live in per-step Advanced blocks. The
  artifact step preflights changed buildings and downloads one ZIP with the
  config, OSM paths, changed bundles, and **all floor JSONs for affected
  buildings** (no separate floor upload). See `docs/apps.md`.
- `geo.mjs` — the registration math, shared by everything: `DXF_SCALE`,
  `metresPerDegree`, `dxfCoordsToLonLat`, `lonLatToDxf` (inverse),
  `makeToLL(building, floor)` (floor-level origin/offset wins, AS106 style).
  Also `crossBarrier(polylineM, a, b, tol)` and `nearestApproach(polylineM, a,
  b)` — the tolerant door-crossing tests (see Room Pin). index.html, smoke.mjs
  and import.html all import from here.
- `campus.mjs` — the tiered router. `extractPortals(shapes, toLL)` pulls
  elevators/stairs/exits from tagged DXF layers; `buildCampusRouter(buildings,
  outdoorShapes?)` links connectors across floors, builds the macro graph over
  building exits, and answers `route(startLL, endLL, {startFloor, endFloor})`.
  Each connector carries an explicit first-stop `anchor` — a kind missing on
  the ground floor anchors on its first upper-floor stop (looking the ground
  stop up directly passed `undefined` into `distLL` and crashed).
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
  degrees rotation UI storing radians), and edit per-floor overrides. The
  **align to ground** button best-fits the selected floor's
  exterior perimeter onto the building's ground floor (see `align.mjs`);
  stairs are deliberately not used as anchors, elevators are. Persists
  work-in-progress to localStorage. Its **Deploy** section writes a drop-in
  `buildings-config.json` (with every floor's portal cache), `osm-paths.json`
  (in-browser PMTiles decode, same bbox/schema as gen-osm.mjs), and on demand
  per-building navGraph bundles (`<id>.nvg.gz`, generated by the bundle worker).
  Its deploy diff compares the draft config with the deployed one by building id
  and can download a single ZIP containing the full config, full OSM cache, and
  only the affected buildings' bundles.
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
  open rooms). `HeadingDriftCalibrator` is the older corridor-only drift trimmer
  (kept for reference; the app now uses `CompassSnapper`).
- `headingfield.mjs` — the floor's **allowed compass directions**. `buildPassages
  (nodes, adj, doorsM, segments)` turns every navgraph edge and every interior
  doorway into a bearing-bearing segment (built inside the navgraph worker);
  `makeHeadingField(passages, {toM})` is the spatial index + bearing histogram
  (`query(ll)` -> candidate bearings + a `confidence`); `CompassSnapper` snaps the
  live heading to the nearest candidate and holds it until a deliberate turn.
  See `docs/headingfield.md`.
- `doorways.mjs` — physical doorways. `doorwaysFromShapes(shapes, toLL)` clusters
  DOOR/DOORS-layer leaves (double doors into one opening) and turns each into a
  **wall-aligned barrier** (the wall gap, using the leaf's swing arc when drawn),
  with a stable geometry-derived id. `barrierCrossesPolyline` is the exact test;
  see Room Pin for the tolerant one. The same doorways back interior-door
  blocking and the Room Pin door lookup.
- `roompin.mjs` — **Room Pin** ("Locate by room"): turn an exact room number into
  a manual PDR anchor just outside its door. See the Room Pin section below.
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
  builds the router, draws the route's connector dots and tiered routes, and
  drives the live marker from the arbiter (GPS + PDR). "I'm Here" stops GPS
  and taps a manual anchor (for when the fix is wrong); "My Location" starts
  GPS again.
- `app.html` — the production Navigator frontend: the same modules and
  navigation logic as index.html behind a mobile-first UI (route preview card,
  calibration modal, cached navGraph bundle boot, place search/favorites,
  first-load tour). Planning a route **keeps the current floor and fits the whole
  route** (no zoom-to-destination), and reveals the destination pin even when it
  is on another floor (`destinationRevealed`); the first-load tour's Next button
  has a deliberate cooldown (2.5 s on the big modal pages, 1 s on the spotlight
  steps) and greys out while disabled. See `docs/apps.md`.
- `places.mjs` — place nicknames and the shared room lookup used by
  `app.html`, `places.html`, `roompin.mjs` and smoke: `findBuilding` /
  `splitRoomQuery` / `findRoom` / `findRoomExact` / `doorwayFor`, `normRoom`,
  name normalisation and ranking, `placeTarget` / `parseTarget`,
  `buildPlaceIndex` / `searchPlaces`, `resolvePlace` / `validatePlaces`.
  `places.json` is optional and edited in `places.html`. See
  `docs/places.md`.
- `places.html` — static places admin page: edits `places.json`, validates
  rooms against the registered floor labels, and downloads the file for an
  out-of-band upload. It has no server-side write endpoint and is served
  without auth.
- `smoke.mjs` — the test gate (`node smoke.mjs`). Check count scales with the
  floor/exit count (several hundred currently); **all must pass**, zero
  failures tolerated. It reads its registrations from
  `buildings-config.json` — the same file index.html fetches, so the two can
  never drift.

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
- two bridges, different buildings: skybridge (explicit `to` destination, else
  greedy min-distance matching). A bridge may join different floors, e.g. HSB
  floor 02 ↔ Parrish Bridge floor 03 ↔ EIB floor 03.
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
`floor.skybridges: [{id, dxf, to?, toFloor?}]`; the **destination building**
select marks `to` (another building's id) and the **destination floor** select
marks `toFloor` on newly placed bridges; **Set destination** re-marks an
existing dot (with "auto" selected it clears the marks). `toFloor` pins the
pair to one floor when a building has several skybridge portals (ECB has two
to the same building on different floors). The map labels each dot with its
destination.

Pairing (`buildCampusRouter`): **explicit destinations first** — candidate
pairs where either side's `to` names the other's building and any `toFloor`
matches the other floor: mutual marks, then one-sided, each shortest-first,
one partner per portal. Explicit links ignore `SKYBRIDGE_MAX_M` and the floor
number unless `toFloor` pins it — the marks say exactly which
bridge goes where (AS121-01 ↔ AS119-02 is ~126 m and crosses a grade change;
the earlier same-floor rule and the 120 m cap both blocked it). Unmarked
bridges then fall back to greedy min-distance matching within the cap, preferring
the same floor *number* (so DXFs labelling the storey "2" vs "02" still link)
but allowing a cross-floor pair when that is the only candidate. One partner per
portal; a bridge with a destination never auto-pairs elsewhere (no accidental
link to a nearer neighbour). Strict mutual-nearest was the original bug source:
AS119's ECB-side bridge had AS117 closer, so AS117 took it and ECB linked to
nothing. Only bridges pair, so an ordinary rooftop door facing a neighbour
cannot connect a building; ground-level bridges are allowed too (AS117-01 ↔
AS119-01 share a first floor across two DXFs). Mark **both** sides and set
the destination on each.

The route segment for a cross-floor bridge carries both `fromFloor` and
`toFloor`; the app draws it on both 2D floors and shows the floor change in the
route step. HSB (AS156) is the real example: its bridge lands on floor **02**,
while Parrish Bridge/EIB are on floor **03**.

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
- **`gen-portals.mjs`** (dev-only) is the CLI fallback for bootstrapping the
  cache; the import tool is the normal path (a floor JSON change is re-imported
  there and comes back cached). Then deploy `buildings-config.json`.
- smoke.mjs verifies every shipped floor's cache equals a fresh extraction when
  the `portals` category runs (the full gate); other categories load with or
  without the cache as above.

## Performance

The campus grew to 24 buildings / 75 floors and load was once ~40 s. Fixes so far:

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
3. **OSM paths cache** — `osm-paths.json` (bbox + shapes) is decoded from the
   pmtiles archive once; index.html loads it when its bbox covers the campus,
   else decodes live. Removes ~6 s. **Regenerate and deploy `osm-paths.json`
   whenever the building set / campus bbox changes** (the cache stops
   covering). Normal path: `import.html`'s **Download osm-paths.json** button
   (manual, in-browser decode via osm.mjs; same bbox and schema, verified
   bit-identical against the checked-in cache); `gen-osm.mjs` is the CLI
   fallback.
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

Before cached bundles, the remaining first-load cost was **CPU on the main
thread**: with the then-current 8 buildings / 17 floors, `buildGraph` measured
~6.7 s and `buildCampusRouter` ~2.5 s. The per-building `.nvg.gz` bundles now
avoid the visibility-graph build; the worker/synchronous path remains for a
missing or stale bundle (and the import tool).

## The OSM layer (osm.mjs)

The basemap at `https://uaa-campus-map.net/style.json` is a Protomaps
extract of OpenStreetMap served as one PMTiles archive
(`https://uaa-campus-map.net/scak.pmtiles`, South Central Alaska,
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
- Floor JSONs come from the external DXF extractor. It drops lines drawn with
  a dashed linetype (HIDDEN, DASHED, overhead, grids, centre/neighbour
  outlines) and records the count in `shapes_dashed_dropped`. Those are
  drafting data, not walls. The JSON does **not** store linetype, so a refresh
  is not auditable from the repo: eyeball any floor whose `wall`/`walls`/
  `window` layers change, and regenerate that building's `.nvg.gz` when they
  do. Door/stair/elevator layers must stay untouched; `node smoke.mjs portals`
  proves the cached portals still match a fresh extraction. The Oct 2026
  refresh changed walls only on AS105-02, AS121-03 and AS150-02; those three
  bundles were regenerated.

Current data: **24 registered buildings, 75 floors** (see
`buildings-config.json`; AS112 is registered). Effective exterior doors are the
manual `floor.exits` list, falling back to the algorithm's portal cache; smoke
checks every floor and every building's exits.

## Positioning (PDR + GPS)

Target: a few metres indoors, on-device, offline, no extra hardware, browser
PWA (Chrome Android / iOS Safari / Firefox). No CV — the camera was excluded
and a learned "mouse sensor"/optical flow was judged the wrong tool for a
4-5 m target. No ML either: the floor plan does the heavy lifting.

- **Step detection lesson 1**: measure movement ALONG GRAVITY, not the scalar
  magnitude `|a|`. The magnitude is orientation-invariant but NOT
  motion-invariant: rotating/waving the phone adds centripetal/tangential
  acceleration to `|a|` and fires the detector as though steps were taken.
  `StepDetector` low-passes the raw accelerometer VECTOR to estimate the gravity
  direction (walking ~1.8 Hz is far above that filter) and takes the component
  of acceleration along it, so a tilt sweeps gravity across the device axes but
  leaves the component ~constant while real gait oscillation survives. Do NOT go
  back to `mag - baseline`.
  **Lesson 1b (the front-end)**: that projection then goes through a
  **time-based band-pass (0.5-3.5 Hz)** - a high-pass (`hpTauS` ~0.32 s) that
  replaces the old baseline subtraction, and a low-pass (`lpTauS` ~0.045 s) that
  replaces the old one-pole smoothing. Time-based coefficients
  (`exp(-dt/tau)`) keep it frame-rate independent. This rejects both slow
  orientation drift and fast hand vibration; smoke checks 8 Hz jitter riding on
  a walk does not inflate the count.
  **Lesson 1c (hysteresis/prominence - the "occasional extra step" fix)**: a
  plain single-sample local maximum fires twice on a double-bump (footfall +
  hand). `StepDetector` now requires a peak to stand `minProminence` (0.3) above
  the valley since the last step and, after a step, will not fire again until
  the signal falls `releaseMps2` (0.3) below it (a Schmitt trigger). It also
  learned an **adaptive noise floor**: while the signal is quiet, a slow EMA of
  the band-passed magnitude raises the gate (gain `noiseGateGain`, capped at
  `noiseCeil`), so a noisy hold counts less without a fixed floor that would
  miss light gait. The old fixed absolute gate (`minPeakMps2`, 0.8 m/s²) is the
  fallback when the environment is quiet.
- **Step detection lesson 2 (the "counts when I move a bit" bug)**: a peak is
  not a step. Shifting the phone / shifting weight produces isolated peaks too.
  `StepDetector` now requires a *walking rhythm* before counting: a run of
  `qualifySteps` peaks whose spacing stays within `rhythmLo..rhythmHi` (now
  0.6..1.7, was 0.5..2.0) of the previous interval. While unconfirmed, peaks
  are buffered and flushed the moment a walk is confirmed (so no real steps are
  lost); a gap past `maxIntervalMs` ends the walk. `push()` therefore returns an
  **array** of step timestamps (possibly several on confirmation), not a
  boolean. The refractory is 300 ms.
  **Lesson 3 (the "too sensitive holding still" bug)**: rhythm alone is not
  enough - hand tremor, a foot tap on the phone or a shaky table can be
  rhythmic yet far too weak to be gait. `StepDetector` therefore also enforces
  an **absolute peak-intensity gate** (`minPeakMps2`, 0.8 m/s^2) on top of the
  dynamic envelope threshold. Without it those light peaks registered as steps
  and dead reckoning drifted while standing (which then looked like a jump on
  the first real step after a pause). Real gait peaks clear 1.5-4 m/s^2.
  Smoke covers light rhythmic jostling (0 steps) vs the same rhythm at step
  intensity (counts).
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
  blend (`1-exp(-dt/tau)`, tau 0.3 s, frame-rate independent) and the gyro only
  bridges when the compass is stale (`gyroBridgeAfterMs`). (3) The Heading panel
  has a **compass-trim slider (±45°)** for the "compass points a bit off"
  case: the trim is composed with the calibration bias
  (`HeadingTracker.bias = base + trim`), applied live while dragging and
  persisted as `pdr-heading-trim`. The calibration walk is authoritative - it
  absorbs the trim into the base and recentres the slider - so the two can
  never double-count. Note that the base itself now decays by ×0.95 per step
  whenever the corridor snapper is not locked (see the hallway bullet below);
  only the manual trim is fixed.
- **Hallway heading-drift correction** (`CompassSnapper` in `app.html` and
  `index.html`; `HeadingDriftCalibrator` remains the older reference class).
  The compass drifts indoors; a person walking a
  narrow hallway walks parallel to the walls, so the floor plan can correct it.
  The snapper locks as soon as the heading field is confident enough
  (`confidence >= 0.4`) and the heading is within 35° of a candidate direction -
  there is **no straight-step arming requirement**. While locked, a small
  proportional correction (`gain 0.5`, clamped to 3°/step, 3° deadband) is
  added each step to `baseHeadingBias` -> `applyHeadingTrim()`, i.e. the SAME
  bias the compass arrow, PDR and trim slider share (so one fix corrects all
  three; the filter's own `headingBias` is left alone to avoid double-counting).
  An **intentional turn** (a >25° step or >35° net over 3 steps) releases the
  lock and re-locks on the new direction when the field is confident.
  **While the snapper is not locked, `baseHeadingBias` decays by ×0.95 per
  step** toward the raw compass, so a bad correction (or a stale
  walk-calibration offset) can recover; the ±45° manual trim does not decay.
  This includes open rooms, free roam, a GPS-sourced estimate and a missing
  heading field. Decay is suppressed while snapping is disabled and during the
  guided calibration walk. A small on-map pill (`#corridor-badge`) shows while
  locked.
  The older reference class's axis comes from **`corridorAxis`** (locate.mjs), a
  runtime ray-cast from the current PDR estimate over `navGraph.isWalkable`: it
  measures the full free chord ALONG a candidate bearing vs the full chord
  ACROSS it and demands
  **aspect ratio ≥ 6:1** (`maxM = 17 m` caps each ray, so the measurable chord
  is up to 34 m; the widest detectable hallway is `2·maxM/6 ≈ 5.7 m`). The gate
  was 9:1 with `maxM = 15 m`, which fired on only ~8% of real walkable samples
  and could not even reach 9:1 on a hallway wider than 3.3 m; 6:1 with the 17 m
  ray roughly doubles the rate (measured **6.4% → 12.1% over 8907 samples on all
  75 floors**) while still rejecting squarish rooms. The sum-of-opposite-rays geometry
  makes the gate honest:
  open rooms/atria, wide lobbies, intersections (the cross-corridor blows up the
  across-run) and L-corners (both chords long) all return `null`, so the heading
  is only snapped where the axis is unambiguous. A coarse 24-direction sweep
  finds the rough axis, a ±8° sweep at 1° refines it to ~1°; rays are memoised.
- **GPS and PDR run simultaneously** — no indoor/outdoor mode flag. The arbiter
  picks the smaller uncertainty with hysteresis. Critical: indoor GPS accuracy
  is often optimistically small *and wrong*, so fixes landing in a wall (when
  near a known building) are discarded. GPS acts as a free anchor when good.
  **PDR walkability spans the whole floor level, and off-plan is not a wall.**
  A navgraph raster ends at its own wall bbox (padded by one clearance);
  `isWalkable` is false everywhere beyond it. That edge is **not a wall - it is
  just the end of the plan** - so treating it as one made the cloud pile up in a
  line along the bbox and the walker "can't get past it" (open corridor ends,
  atria, and the gaps between separate plans). Two changes in `ensureLocator`:
  - **Union of every plan at this floor level.** A connected complex is split
    across several plans (AS117 and AS119 are one building with separate
    navgraphs), so crossing into the neighbour hit that bbox as an invisible
    wall. `isWalkable`/`nearestWalkable` are the union over `sameLevel`; walls
    still block, a gap between separate buildings still blocks (nothing walkable
    in between), but a shared opening is continuous. The **arbiter** and
    `isOutsideBuildings` use this strict union (`insideBuilding` = any same-level
    walkable raster), and it feeds `corridorAxis` so drift is corrected in
    whichever plan the estimate is in. **`nearestWalkable` MUST short-circuit:**
    a naive union hands every plan the full radius, and a plan the point is far
    outside spirals every ring (~0.2 s each) - "I'm Here"/floor change took
    ~0.8 s. It returns `ll` immediately when the point is already walkable,
    searches the nearest plan first (`boundsRank` buckets by `inBounds` margin),
    and caps each later plan's search by the best distance found so far.
    Measured 777 ms -> 22 ms per `ensureLocator`, same nearest points.
  - **The filter gets a lenient `pdrWalkable`:** `navgraph.inBounds(ll)` tells
    "outside the raster" apart from "blocked by a wall", and `pdrWalkable(ll)`
    is `inBounds ? isWalkable : true`. Inside a plan (or any same-level
    neighbour's plan) the walls still block; **outside every plan the unmapped
    space is free and unbounded**, so outdoor PDR works: the cloud walks out of
    a door and keeps dead-reckoning, with no GPS fix needed. (`nearestWalkable`
    stays strict - real mapped space only - so GPS-validity and recovery still
    snap onto the plan.)
  - **A manual floor change re-binds the live locator**
    (`rebindLocatorToView`, wired to the +/- picker). The displayed floor is
    authoritative, exactly as for "I'm Here": stacked floors share a footprint,
    so the boot/GPS centroid pick can bind the wrong one, and stepping the
    picker used to move the plan under a stale cloud that kept dead-reckoning on
    its first floor (the "similar problems after changing floors" report).
    Rebinds only when the picked floor has a plan under the estimate.
  **Entering a building hands over to PDR immediately**: an indoor fix (one that
  lands on any same-level walkable raster - `insideBuilding`) or any
  implausible fix forces `source = 'pdr'` with no hysteresis delay, and such
  fixes are **never used to re-seed the particle cloud** (a metres-wrong indoor
  "good" fix would drag every particle with it - the poisoning that motivated
  this). **Outdoors PDR must not be tied to the floor plan at all**:
  whenever the latest believable fix is outside the bound building, the filter
  enters **free roam** (`ParticleFilter.setFreeRoam(on, isAllowed)`), which
  lifts the walkability weighting and snap-back recovery. It is **constrained
  to "outside every building that has a plan on this floor"**
  (`isOutsideBuildings`, built in index.html's `ensureLocator` from the
  same-level floors' navgraphs): the cloud flows around building footprints
  instead of cutting through them, while a skybridge passing over a building
  with no floor at this level stays free. The app also enables free roam when
  the estimate is within **3 m of an exterior door** or already outside every
  plan, so PDR can leave a building through its doorway band without GPS. PDR drives the estimate there; GPS
  overrides the displayed source only with a **very good fix,
  `gpsOverrideAccuracyM = 3` m**. The cloud is re-anchored from an outdoor fix
  when it is the **first outdoor fix of the session** (once per
  locator/arbiter, up to `gpsSeedAccuracyM = 20` m - the app may have snapped
  the cloud into a building) or whenever the fix is **more accurate than the
  PDR estimate** (`fix.accuracy < pdrAccuracy()`); otherwise it just
  dead-reckons. Re-anchoring is deliberately NOT tied to every indoor<->
  outdoor transition: a fix flickering at a doorway used to re-seed the cloud
  each time it read outdoors and teleported the estimate (the "jumps 20 ft"
  report). Indoor fixes never re-seed. Free-roam toggles per fix, so an exit frees as soon as
  a fix shows the doorway/outdoors.
- **Live re-route**: with a route on screen, every position update re-routes
  from the current estimate to the same destination (`rerouteFromEstimate`,
  called from `renderEstimate`), so the drawn line always leads forward instead
  of pointing back where you came from. It passes the locator's bound floor as
  the start-floor hint and `destinationFloor` as the end-floor hint, and is
  throttled (3 m moved or 5 s elapsed; one route costs ~70 ms) and skipped
  while "Find Path" taps are being picked. Before this, only GPS fixes
  re-routed (with no floor hints) and PDR steps left the line stale indoors.
- **Anchors** are manual: press "I'm Here", then tap the map to place your
  location (not the map centre); the tap re-seeds the particle cloud and resets
  drift. **"I'm Here" cancels GPS** (the user reaches for it when the fix is
  wrong): the handler calls `stopGps()` - clears the geolocation watch AND
  `arbiter.clearGps()` - and `ensureLiveSetup()` (marker/layers/PDR without
  geolocation), because a live-but-wrong fix would otherwise win the
  arbitration again seconds after the anchor (or be held through a source
  switch). "My Location" re-starts the GPS watch. `ensureLiveSetup` is shared
  by both flows; the old handler called `startLiveLocate()`, which turned GPS
  ON when it should be turning it off. The **displayed floor is authoritative**:
  `anchorAt` passes
  `floorOfView()` into `ensureLocator`, which uses
  `campusRouter.buildingAt(ll, hint)` to bind building+floor. Floor picks by
  wall centroid cannot tell stacked floors apart (01/02/03 share a footprint),
  so before this an upper-floor anchor bound to the ground floor and the PDR
  stayed on the wrong plan. GPS fixes and boot still use the centroid pick
  (they carry no floor). The status line reports the bound `ASxxx NN`.
- **Step-length calibration** is in the index.html "Calibrate walking model"
  panel (it replaced the old nav sidebar): place a start and end point on the
  map (green/
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
- **Continuous GPS step-length auto-calibration** (`autocal.mjs`) runs whenever
  GPS is good and the user walks - no dedicated calibration walk. Three signals
  feed it: (1) Doppler speed integrated between the first and last counted step
  of a segment, over the N-1 steps walked in exactly that time (the precise
  method; anchoring to step times removes the partial-step boundary error);
  (2) endpoint displacement over the steps counted between two good fixes (an
  independent cross-check, and the only channel on devices that report no
  speed); (3) step rate twice over - the segment cadence normalises each
  sample back to the model's reference cadence (`cadenceRef`/`cadenceGain`
  come from the StepLengthModel), and cadence itself is measured from the
  median step interval. **The calibrator's inputs are raw GPS fixes and raw
  step events only** - the arbiter/PDR estimate (position, speed or the step
  model it uses) is never fed in, so PDR can never calibrate itself; the
  `successPosition` wiring stamps each fix with its measurement time
  (`maximumAge` can return a cached fix) and ignores repeated cached fixes.
  A window must also be covered by *continuous* raw GPS: a gap >6 s between
  good fixes, or good fixes spanning <80% of the window, is rejected - so a
  GPS dropout (indoors, where the arbiter is on PDR) cannot be stitched into
  a position-only sample measured as a straight chord across the hole.
  A segment qualifies only at 5-60 s (≥10 steps, ≥6 m),
  straight (net
  course change ≤40° AND circular concentration R ≥ 0.75 of the fix courses -
  the endpoint check alone is blind to curves wrapping past 180° within the
  window), pause-free (≤1.5 s internal step gaps), fix accuracy ≤15 m
  throughout, and walking-range Doppler speeds. Accuracy is **dynamic, not a
  hard 10 m cutoff**: beyond 10 m ("fine") the speed and position
  uncertainties of a fix scale with its reported accuracy, so coarse-GPS
  samples are wider and the 90%/±6% confidence gate simply demands
  proportionally more walking before the claim opens (the same walk scores
  98.3% at 10 m and 92.7% at 15 m in the tests); at a leg boundary the
  interval's walking speed is taken
  from the walking endpoint only, so a decaying stationary speed cannot pull
  the distance (and the base) low. **Turns are allowed when the walker pauses
  at the turn**: a step gap >1.5 s closes the leg explicitly (bypassing the
  duration deferral), so a back-and-forth walk through a constrained indoor
  space calibrates leg by leg instead of being poisoned by its U-turn
  courses. Candidates otherwise close nearest to a
  30 s target; a sub-15 s window is only accepted once the chain has waited a
  full target interval, so a straight stretch after a corner becomes a long,
  precise sample instead of a barely-legal 5 s one. Samples combine
  inverse-variance weighted with a 1.5% irreducible systematic floor (a GPS
  speed scale bias does not average away); a "calibrated" claim additionally
  needs ≥20 steps, ≥10 m, ≥2 segments and early/late agreement. The sidebar
  panel and the amber/green dot inside the "Calibrate walking model" button
  show the live estimate, the confidence of being within ±6%, the 90% interval
  and the speed/position weight mix - and it updates while the user walks,
  even before the first sample: current GPS accuracy and Doppler speed, the
  steps/duration of the current stretch, and the newest attempt's outcome or
  reject reason ("turned without pausing", "stretch too short", "stopped
  mid-stretch", ...). `stepModel.base` is **only ever written
  when the estimate is within ±6% at 90% confidence**, and persisted as
  `pdr-step-auto` (+ `pdr-step-base`); a manual known-distance Apply sets
  `pdr-step-manual` (green dot) and a later certified GPS estimate may
  supersede the base.
- **The filter must be seeded or PDR silently does nothing.** `ParticleFilter.
  predict()` is a no-op until `seed()`. So when a locator is created,
  index.html/locate-test snap the seed to the nearest walkable point; the
  arbiter also seeds on the first plausible GPS fix. Without this, steps are
  counted and heading is fine but the marker never moves (the "PDR not
  implemented" bug).
- **Shape-preserving heading mode and allocation-free resampling.**
  - `persistentHeadingDeg`: draw ONE heading offset per particle instead of
    i.i.d. per-step jitter, so a trajectory's SHAPE constrains the belief.
  - The filter has a fixed particle count (arrays sized once); there is no
    adaptive/KLD count and no `seedUniform()` in the current class.
  - Perf: one `metresPerDegree` per step (was per particle), a 2-sample fast
    path in `_advance` (a step <= 1 m cannot jump the ~0.6 m clearance band),
    and no per-resample array allocation (fixed-1000 `predict` ~0.73 -> ~0.43 ms).
  - **Measured finding (important):** with PDR ALONE the belief does NOT
    collapse. A straight hallway is translation-invariant; even a square leaves
    many candidates. "Each step cuts the particles" is therefore NOT true for
    pure PDR - it only happens once an ABSOLUTE observation lands (a point / a
    known floor or room entry). The production filter uses a fixed 1000
    particles; keep it on a known floor rather than treating PDR as a global
    localiser.
- **The filter trusts the compass (≤90° off) and the arbiter's accuracy.**
  `ParticleFilter` now enforces, on top of the existing soft wall weighting
  (non-walkable = 0.02) and resampling:
  - **No tunnelling** (`predict` -> `_advance`): a step is swept and stops at
    the last walkable sample (`segmentStepM = 0.1 m`, matching navgraph's cell
    so a ~0.3 m doorway channel is never falsely blocked).
  - **Blocked -> destroyed and re-spawned.** A particle whose step a wall blocks
    is a wrong guess: it is immediately REMOVED and replaced by a fresh one
    (`_respawn`). Its POSITION is inherited from a random **surviving** particle
    (plus a small `respawnSpreadM = 0.3` jitter), NOT a fresh random point -
    which could sit in another room behind a wall the walker could never have
    reached. Its heading is the step heading. There is no sliding; the cloud
    cannot pile up in a line against a wall. Doorways work for particles whose
    step threads the opening.
  - **Heading jitter is an OFFSET of the measured heading.** Every step a
    particle draws `h = measuredHeading + headingBias + uniform(-jitter, +jitter)`
    with `headingJitterDeg = 35` - a random +/-35 deg around the measured
    heading, never a full-circle random heading.
  - **Along-track and cross-track jitter.** Along-track step jitter
    (`distanceNoise = 0.22`; heavy tails cover a mismatch between the model's
    step length and the walker's - some particles always take the right
    cumulative distance) and a **perpendicular** lateral jitter
    (`lateralNoiseM = 0.30`, applied across the travel direction, not on the
    east axis). Post-resample roughening (`roughenM = 0.20`) re-inflates the
    cloud in every direction after resampling collapses it.
  - **Heading prior**: a known heading seeds within ±90° (`seed` clamps
    `headingSpreadDeg`; per-step noise is clamped at ±90°).
  - **Forward-hemisphere prior** (`_applyPriors`): particles behind the mean
    relative to the measured heading are multiplied by `behindWeight = 0.05`
    (kept, weakly, so a genuine U-turn is still detectable), not killed.
  - **Accuracy radius**: nothing lives farther than the arbiter's
    `accuracyM` from the cloud's own mean (`arbiter.step` passes
    `pdrAccuracy()`); a U-turn step (`reversalDeg = 120`) or a stall-recovery
    step lifts this.
  - **Stall guard** (`_trackStall`): if the walker keeps stepping but the mean
    won't advance (`stallEpsM`, for `stallSteps`), the priors relax for
    `relaxSteps` so the noise re-diffuses the cloud toward a doorway — never
    letting it cross a wall.
  - **`estimate()` is walkable**: it averages the walkable particles (a cloud
    straddling a thin wall otherwise averages INTO it) and, if even that mean
    lands in a wall, reports the heaviest walkable particle — never a spiral
    snap that could pick the wrong side. Free roam (outdoors) skips the
    walkability parts (tunnelling test and snap), keeping the heading/accuracy
    priors.
- Prod ships no telemetry; all tuning/overlays live in `locate-test.html`.

## Testing

`node smoke.mjs` — the only gate; run it after any change to campus.mjs,
geo.mjs, osm.mjs, align.mjs, buildings-config.json, a building registration, a
floor JSON or a navGraph bundle.
**It takes category arguments so you don't run the whole suite every time**
(`node smoke.mjs --list` shows them): `align, walkability, pdr, corridor,
arbiter, portals, router, osm, multifloor, motion, autocal, anchorcal,
headingfield, doorways, roompin, places`. No args = everything (the full gate is
minutes, dominated by `router`/`multifloor` and the live OSM fetch). The groups
that need no real-floor load (`motion pdr arbiter autocal anchorcal headingfield
doorways roompin`) run in seconds/`<1 s`:
`node smoke.mjs motion pdr arbiter autocal anchorcal headingfield doorways roompin`.
`places` reads the floor JSON room labels but builds no navGraph; `doorways`
reads one real floor (AS121-02) and `roompin` builds it. The building-loading
groups share one load. It covers:
portal sanity, exits outside the footprint / near real doors, exit reachability
for every building, six end-to-end routes (same floor, cross-building, outdoor
click, vertical, cross-floor, cross-campus), a hermetic synthetic outdoor grid
incl. the no-network fallback, and a live OSM test (network — degrades to
skipped offline). It also covers the floor auto-align math (`align.mjs`:
convex hull/sampling, and rigid recovery of a known rotation+shift on a
synthetic asymmetric perimeter), the positioning stack on synthetic signals:
`isWalkable`/`nearestWalkable` consistency, step detection vs a synthetic gait
across cadence/amplitude/noise, heading smoothing/gyro/bias, the particle
filter's kinematics and map constraint, and the GPS/PDR arbiter's validity
gate, drift growth, re-seed and hysteresis. The GPS step-length auto-calibrator
is covered on synthetic GPS walks: certification only at ≥90% within ±6%,
poor fixes / pauses / sharp curves rejected, coarse fixes up to the 15 m floor
accepted (with accuracy-scaled confidence, >15 m rejected), GPS dropouts
never stitched across, noisy courses and
right-angle
corners tolerated, pause-at-turn-around back-and-forth walking in a
constrained space certifying without a downward base bias, the position-only
fallback certifying, cadence normalisation, and split-half inconsistency
withholding the claim. Debug
scripts live in `/tmp/opencode/` (`dbg-*.mjs`) and are not part of the repo.

The building-pair check now starts at a real exit for each building (the old
`text_lines[0]` fixture hit a shaft in AS118 and a doorless room in AS154) and
checks outdoor cuts against level-1 walls only, since a ground path may pass
under an upper-floor bridge (AS164-03).

The full suite is several hundred checks over 24 buildings / 75 floors; runtime
varies with machine load and the live OSM fetch. `router` loads the real campus
and takes minutes; use the focused categories while iterating.

**The shared floor load uses the shipped navGraph bundles** (`.nvg.gz` via
`graphUrl`, decoded with `makeBundleGraph`) instead of rasterising walls, and
verifies each floor's `sourceHash` against a freshly computed
`graphInputHash({lines, doors})`. A mismatch (or a bundle that fails to decode)
is reported and **fails the gate** (`navGraph bundles decode` /
`every navGraph bundle matches its floor JSON`), so the cache can never go
stale unnoticed; a mismatch falls back to the on-device build for that floor.
Portals come from the config cache unless the `portals` category needs the
fresh extraction (their equality is asserted there). Measured: ~2.5 s to load
for routing groups, ~7.6 s when fresh portals are extracted.

Per-floor work still runs in a small pool of **worker threads**
(`basis-worker.mjs`), which validates the bundle hash and returns the decoded
floor plus the portal set. `SMOKE_NO_WORKERS=1` runs it inline. Timestamps are
**off by default**; `SMOKE_TIMING=1` prefixes every line with elapsed seconds,
and the load prints or omits a per-building line accordingly.

**Model fidelity**: the suite builds its campuses the way `index.html` does -
same wall filter and `shapeSegments`, same `makeToLL`, portals from the shipped
cache with a fresh-extraction fallback (their equality is asserted), manual
doors win, skybridges are explicit, and the full-campus router is fed the
shipped `osm-paths.json` network the app walks (verified: a turf-shaped
FeatureCollection vs bare lines produce identical graphs, since `buildGraph`
reads the same coordinates either way). The ground-floor-only routers and
synthetic campuses are deliberate test constructs, not app paths. UI/3D code is
not Node-testable and is covered by eyeballing the deployed page.

**Never run headless-browser tests** (Firefox/Chromium screenshots, DOM dumps,
remote debugging) **without asking the user first.** Ask for a screenshot (or
permission) instead - the user can supply them. UI verification is otherwise
manual on the deployed page.

An older `node --cpu-prof` run (17 floors, before cached bundles) showed
`buildGraph` ~9.5 s of CPU (down to ~3.7 s wall in the worker pool), `dijkstraFromAttach` ~9 s
(router attachment/matrix + per-route walks), `attach` ~5.5 s (`buildingAt`
loops, exit reachability), GC ~4 s, live OSM fetch/decode ~3 s.

For registration changes, open `import.html` (serve the dir the same way) and
eyeball the plan over the basemap before trusting `buildings-config.json`.

## Deployment

**Deploy as part of finishing any task that changes a site file.** The live
site should always reflect the working tree. Workflow for every task:

1. Run the gate: `node smoke.mjs` must be all green.
2. Deploy **only the files you changed** (see below), then verify.

The site is served from `root@uaa-campus-map.net:/usr/share/nginx/html`.
`deploy.sh` (git-ignored) shows how - the host, the destination path and the
`scp` usage; read it before deploying:

    scp -r *.html root@uaa-campus-map.net:/usr/share/nginx/html
    # ...and *.png, *.json, *.mjs

**Do not run `deploy.sh` for a normal change** - its globs re-upload the whole
directory (every floor JSON, the ~30 MB `scak.pmtiles`, the MapLibre bundle)
every time. After a major change:

1. Run the gate: `node smoke.mjs` must be all green.
2. Upload **only the files you changed**, naming them explicitly, e.g.:

       scp index.html motion.mjs locate.mjs arbiter.mjs \
         root@uaa-campus-map.net:/usr/share/nginx/html/

List the changed paths with `git status --short` first so nothing is missed and
nothing extra is sent. The large static assets (floor JSONs, PMTiles, MapLibre
`*.mjs`) only need re-uploading when they themselves change.

A floor-data refresh should still be deployed as a pair — every changed floor
JSON plus the regenerated `.nvg.gz` for each building whose wall geometry
changed. The bundle now stores a `sourceHash` of its inputs and boot falls back
to an on-device build on a mismatch, so a mixed deploy is detected rather than
silently routing on the old graph (the first load is slower until the pair is
uploaded).

### Deploying building additions (no server-side commands)

Everything generated is produced by `import.html`; the server only receives
files:

1. Extract new floor JSONs (external DXF extractor) and import them in
   `import.html` (placement, manual doors, skybridges).
2. Press **Download changed buildings (.zip)** for the normal deploy. It
   compares against the deployed config and contains the full
   `buildings-config.json`, full `osm-paths.json` (freshly generated when stale),
   and only the added/changed buildings' `.nvg.gz` bundles. The older individual
   config/OSM/all-bundle buttons remain for manual recovery.
3. Extract the ZIP into the server root. Upload new floor JSONs separately when
   a building's floor data is new (the ZIP intentionally contains only config,
   OSM paths, and navGraph bundles).
4. `node smoke.mjs` stays a **local** pre-deploy gate (Node-only checks).

### Deploying place nicknames

Edit in `places.html`, **Download places.json**, run `node smoke.mjs places`
(every entry must resolve on a registered floor), and upload `places.json`
next to `app.html`. The first deploy of the feature also needs `places.html`
and `places.mjs`; upload `app.html` whenever its search changes. `places.html`
is static and has **no server-side write endpoint**, and it is served
**without basic auth** — publishing always requires the download + upload
step. Student votes go to an external form (`PLACE_VOTE_FORM_URL` in
`app.html`), so there is no server-side process or `/api/` endpoint.

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
  `base -> base + TOP_OUTLINE_M (0.2)` volume in near-black
  (`#16181c`) at opacity 0.7 - deliberately softened so the band hints at the
  plan instead of inking it. `applyStructureStyle` raises that level's
  `fill-extrusion-base` to `base + TOP_OUTLINE_M`, so its translucent wall
  sits directly above with no overlap. **Order is load-bearing**: this build's
  fill-extrusion painter draws EVERY extrusion in the translucent pass, in
  style order, with depth writes (`LEQUAL, ReadWrite`) even at opacity 1
  (verify in the bundle: `if(i.currentPass==='translucent')`). A layer added
  after the walls renders last and is depth-rejected behind any nearer wall;
  added FIRST, the walls blend over it exactly like they blend over the route.
  So the outline layer is added **before** the `campus-3d-level-*` layers. The
  filter follows the selected level; the layer is hidden in 2D and in
  "Find Path" mode (whose selected walls are already solid black).
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
  ground under the stack. Markers that must not sit under the stack become
  extrusions: the **destination** is a DOM marker in 2D and a raised pin
  extrusion on the destination's floor in 3D, and **"My Location"** is a DOM
  pin in 2D and a blocky **avatar** in 3D (`campus-3d-live` source: body
  column + lighter head + a facing wedge along the smoothed heading, plus a
  translucent accuracy disc; `applyLiveMode` toggles them with the DOM pin and
  the flat `location-heading`/`pdr-particles` layers). The avatar's base is
  always `floorBase(currentFloorId())`: real position sources (GPS/PDR) carry
  no height, so the **floor picker is authoritative** exactly as in 2D.
  `applyFloorFilter` calls `refreshLive3D()` so it follows floor changes
  without waiting for the next fix; the disc is omitted below 1.5 m accuracy
  and clamped at 40 m, and the layers are inserted before the structure so the
  translucent walls blend over the avatar instead of depth-rejecting it.
- Camera: constructed with `dragRotate/pitchWithRotate`, `pitch: 50`,
  `maxPitch: 75`; the user pitches/rotates freely.

### Follow mode (index.html)

The **Follow** button (`#follow-btn`) keeps the live position centred and, in
3D, rotates the map so the heading points up the screen - "walk forward and
the map walks with you". Semantics:

- **OFF -> ACTIVE** on tap: starts My Location when nothing feeds a position
  yet (the tap is the gesture the sensors need), suppresses the first-fix
  `flyTo` (`didAutoCenter = true`), runs the one-shot framing ease (below) and
  then starts the loop. Tap while ACTIVE turns it off.
- **A manual drag pauses** it (nav-app pattern): the button restyles (blue
  outline, text becomes "Resume") and rotation re-enables so the user can look
  around. Tapping then resumes and recentres. Detection is **DOM-level**
  (`touchstart`/`touchmove` + `mousedown`/`mousemove` on `getCanvasContainer`,
  pause after ~8 px of one-finger/mouse movement), **not** MapLibre's
  `dragstart`/`rotatestart`/`pitchstart`: this custom build fires
  rotate/pitch events for **programmatic** camera changes too (`jumpTo`/
  `easeTo` that alter bearing/pitch), which paused Follow the instant our own
  ease or the loop moved the map - the button stuck on "Resume" on mobile.
  Two-finger gestures and the wheel do not pause (rotation is disabled while
  following, so two fingers only zoom).
- **Mode owns the camera orientation, and the loop drives it**: 3D angles the
  view (pitch `FOLLOW_PITCH_3D` = 50) and turns the map to the heading; 2D is
  a plain north-up top-down plan (`pitch 0`, `bearing 0`), exactly like
  regular 2D, only panned to keep the walker on screen. The loop sets
  pitch/bearing itself (`jumpTo` carries both, smoothed with the shortest-path
  bearing across the +/-180 seam) because its own `jumpTo` cancels any
  `applyViewMode` easeTo while following - which is why toggling 2D/3D during
  follow used to change nothing. Rotation gestures stay disabled while ACTIVE
  (`applyRotationMode` owns `dragRotate`/`touchZoomRotate` and defers to
  Follow).
- The camera centre is, **in 3D only**, the estimate **offset
  `FOLLOW_LOOK_AHEAD_M` (22 m) ahead along the heading**
  (`turf.destination`), so the walker sits in the lower third facing forward.
  2D (and any mode before a heading exists) centres the estimate exactly.
- **Activation zooms in**: `enableFollow` runs a one-shot framing `easeTo`
  (to the walker, pitch/bearing per mode, `zoom = max(current, FOLLOW_ZOOM =
  19)`, `FOLLOW_FRAME_MS = 700`). The loop must not run during it - its
  `jumpTo` would cancel the ease, which is why the zoom-in never used to
  happen - so `followFraming` holds `renderEstimate` off until the timeout
  fires (and skips starting the loop if the gesture paused meanwhile).
- **Heading priority**: smoothed compass (`motion.mjs`) -> **GPS course**
  (`position.coords.heading`, while moving only) so desktop/no-compass
  devices still orient; both feed `lastHeadingDeg` (also the avatar's facing
  wedge), the last one holds when stopped.
- The loop is `requestAnimationFrame` with exponential smoothing (centre
  ~6/s, bearing ~5/s, pitch ~5/s) calling `map.jumpTo`, **not** repeated
  `easeTo` (fixes arrive continuously and restarting an animation per fix
  stutters). **Zoom is deliberately not part of the loop**, so pinch-zoom
  stays live. `renderEstimate` stores `followCenter` and restarts the loop
  while active; the loop stops itself when paused/disabled.

## Building / room search (app.html)

The search box takes "<building code> <room>", e.g. `BMH 114A`. The building
code is the floor JSON's `ID` field (BMH, ESH, SMH, RH, CUDY, AASC, SU); the
registration id (`AS103`) also works, and the longest known code prefix wins so
`BMH213` and `AS103213` split correctly. Rooms come from the **same
`roomLabels` array that feeds the displayed room-numbers `symbol` layer**
(built from the DXF `text_lines`, see `loadFloor`), so search and labels can
never disagree. Matching is case/space-insensitive, exact first then prefix;
an exact hit on the floor whose level equals the room's first digit wins
(`114A` -> floor `01`). The destination is the room's **DOORWAY, not its
number label**: `loadFloor` also extracts the `DOORS`-layer leaf midpoints
(`doorPoints`), and `doorwayFor` picks the nearest door whose straight segment
from the label stays walkable (no wall between -> same open space, so it is the
room's door, not an adjacent room's). Falls back to the label when no such door
exists. `runSearch` routes from the live GPS/PDR estimate when
"My Location" is running, else the map centre, makes the hit the destination
marker/pin, and draws the route. **Route entry does not change the floor or zoom
to the destination**: it fits the view to the whole route and reveals the
destination pin (see the app notes below). AS112 (WWA) is not registered in
`buildings-config.json`, so it is not searchable.

The room lookup (`splitRoomQuery`, `findRoom`, `findRoomExact`, `doorwayFor`)
lives in **`places.mjs`**, shared with `places.html`, `roompin.mjs` and smoke.
`app.html` also searches **place nicknames** (`places.json`, edited in
`places.html`) and the phone's **favorites** (localStorage), with a suggestion
dropdown. Order: a picked suggestion, a full name ("Subway"), then
`<code> <room>`, then a partial name. Favorites stay on the phone; the route
card links to the vote form (`PLACE_VOTE_FORM_URL`, empty = hidden), and
admins add accepted names to `places.json`. `places.json` is optional: missing means no
nicknames, never a failed boot. Details, server setup and invariants:
[`docs/places.md`](../docs/places.md).

## Room Pin ("Locate by room")

`roompin.mjs` turns an **exact room number** into a manual PDR anchor just
outside the room's door. `resolveRoomPin(building, hit, campusRouter)`:

1. **Exact, unique room** (`findRoomExact`); a duplicate label across floors is
   refused. No other same-floor room label -> `is the only room on its floor`.
2. **Watershed territories (primary).** `watershed.mjs` partitions the floor
   from the room labels + connector/exit seeds. A room's **doors are the throats
   (region boundaries)** on its territory; a throat narrower than a body
   (`0.6 m`) is not a door, and doors into rooms smaller than `8 m²`
   (closets/shafts) do not count. **More than one door -> `has multiple doors`**;
   one door is used. Two separate openings between the same two regions are counted
   separately (boundary cells are split into connected components).
3. **Placement**: 0.5 m beyond the throat into the neighbouring region,
   continuing the **label -> throat** line (the label is a stable
   inside-the-room reference). Fallbacks when that line runs into a wall: the
   nearest walkable point to the throat. The pin is always a walkable point.
4. **Fallback** when the watershed cannot segment (no walls/labels): route
   probes to **6 seeded-random other rooms on the same floor** (seeded from
   `building|floor|room`, stable across runs) and take each probe's **first door**
   via the tolerant crossing (`geo.crossBarrier` + `geo.nearestApproach`: the
   barrier line ±0.4 m past a jamb, or within 0.815 m / ADA 32 in of it, because
   the navGraph passage is offset from the wall-gap barrier). Majority first hop
   wins; distinct first doors >= 2 m apart are multiple doors.

Hard-won notes:
- The old approach (probe an 8 m-away door and prove no other doorway is crossed)
  rejected most rooms; doors now come from watershed connectivity, so an opening
  wide enough to walk through is a door regardless of the DXF leaf.
- **SU floor 02 is special**: its raster is one open ~1000 m2 blob (wall gaps not
  covered by door leaves), so room/door reasoning there is approximate. We tried a
  `buildGraphState` **gap-seal pass** (bridge wall ends < 1 m with no door leaf)
  and **reverted it** - sealing the leaks barely changed SU-02's region and did
  not fix the misplacement. Do not reintroduce it casually; if a floor leaks,
  prefer fixing the floor data.
- The placement side was once derived from the route tangent, which is
  meaningless for a parallel graze; the label is the reliable side reference.

## Browser demo notes

- Connector dots are **route-only**: the `portals` source is empty at boot and
  `renderRoute` fills it with one dot per stair/elevator leg of the current
  route (stairs orange, elevators purple), so unrelated stairwells never
  clutter the map. 2D shows a dot on either floor the leg joins (`fromFloor`/
  `toFloor` filter); 3D hides them (the riser carries the vertical leg).
  Doors are not drawn.
- Route colour encodes the floor: indoor and skybridge geometry ramps **red
  (level 1) → green (top level)** by hue interpolation (`routeColorFor`/
  `floorLevelColor`), so each floor reads distinctly. Outdoor legs run at
  ground level and take level 1's colour, matching a normal line on that
  level. Vertical connectors keep their tier colours (purple elevator / orange
  stairs) so the stair/elevator cue survives.
- **"Find Path"** (tap a start and destination) is a **dev tool** now: the
  button is hidden unless the page is opened with `?dev=1`, or shown from the
  console (`pathBtn.click()`); the mode's code is unchanged.
- If the OSM fetch fails the router still builds and works with straight-line
  outdoor legs — that path must keep working.
- The overlay columns (`#controls`, `.top-controls`) are **`pointer-events:
  none` with only their children set to `auto`**: the flex box's empty gaps
  (between buttons, and beside the shorter ones under the wide search row) used
  to swallow taps and pinch/rotate gestures, so map gestures did not work in
  those areas. Any new overlay must follow the same pattern.
- The PDR **particle cloud is the production position indicator**
  (`pdr-particles` circle layer, fed by `ParticleFilter.points()`): one dot per
  particle, larger/darker with weight, so map-matching is visible as the cloud
  squeezing into corridors. It **replaced the old accuracy circle**
  (`location-radius` fill/outline, now removed). Created lazily in
  `ensureParticleLayer()` (retried while the style loads) and refreshed from
  `renderEstimate`, for both GPS and PDR sources. 2D only - flat dots sit at
  ground level and read wrong under a pitched 3D view (the 3D avatar remains).
  **Performance**: map source updates are throttled (`PARTICLE_PAINT_MS = 100`)
  but the WHOLE cloud is drawn (no sub-sampling), so map rendering never keeps
  up with the sensors. The filter itself is cheap (~1 us/particle/step).
- The **heading indicator** is a `location-heading` line drawn from the live
  marker along the smoothed heading, at a **fixed `HEADING_LINE_M = 4 m`**
  length (so it reads the same at any GPS accuracy). It is redrawn on every
  orientation event (10-60 Hz), so the source update is throttled
  (`HEADING_PAINT_MS = 60`) and computed with plain trig (no turf). The compass
  text is only written when it changes. `updateHeadingLine()` is also called
  from `renderEstimate` so the line stays attached to the marker as it moves.
- The 3D live-avatar geometry (`live3DData`, a polygon build + `turf.circle`)
  is only rebuilt when `is3D`; in 2D nothing 3D is computed per step.
- **Frontend performance notes.** Benchmarked frequent tasks (Node): the
  sensor/estimator math is all sub-microsecond - `StepDetector.push` ~0.16 us,
  `HeadingTracker.pushCompass` ~0.12 us, `GpsStepCalibrator.addStep` ~0.7 us,
  `stats()` ~0.8 us, `metresPerDegree` ~0.17 us. The particle filter is
  **O(count) per step at ~1 us/particle** (`predict` ~0.92, `estimate` ~0.08;
  mobile ~3-4x slower). `count` default is **1000** (safe ~3-4 ms/step on a
  phone; up to ~4k affordable on mobile, ~16k desktop within a 16 ms frame),
  and raising it costs filter CPU too (the whole cloud is now drawn, so
  `setData` is O(count), throttled to 10 Hz). The **render loop is
  O(1) per frame** (follow rAF: constant camera math, skips `jumpTo` when
  converged). Other fixes: throttle map
  `setData` (particles 100 ms, heading-line 60 ms); defer the ~70 ms re-route
  Dijkstra off the frame (`requestIdleCallback`, throttled 3 m / 5 s); the
  follow loop uses plain trig instead of `turf.destination`; compass/status DOM
  text is only written when it changes.
- **navGraph builds run in a Worker** (`navgraph-worker.mjs`, one module worker
  created at module load). `buildGraphState` is ~0.1-1.2 s per floor on desktop
  (3-4x mobile) and boot once ran it synchronously for every floor (75 today;
  **~6.7 s** desktop / ~20 s+ mobile of frozen main thread in ~1 s chunks).
  Cached bundles now avoid the build for most floors; the worker remains the
  fallback for a missing/stale bundle and for the import tool.
  `loadFloor` now `await`s `buildGraphAsync(wallFeatureCollection)`: the worker
  returns the serializable state (the `blocked` raster is **transferred**, not
  copied) and the main thread only runs the cheap `graphFromState`. The state
  round-trips through structured clone with identical results (verified:
  `isWalkable`, `stats.nodes`, `findPath` all match `buildGraph`). **Any worker
  failure falls back to a synchronous `buildGraph`** (`onerror`, `postMessage`
  throw, or a 20 s timeout), so boot never depends on the worker. Only
  `index.html`, `locate.mjs` (+ this file) and the worker are uploaded; the
  worker imports the same `./navgraph.mjs`. Total boot time is unchanged
  (one worker, floors are still built one at a time) - the gain is a responsive
  main thread; a worker POOL + concurrent prefetch would also cut total time.
- **Particle overlay skips work when it cannot show.** `updateParticles` returns
  immediately in 3D (the layer is hidden there) and skips the O(count) rebuild +
  map upload unless `filter.version` changed (bumped in `seed`/`predict`), so a
  GPS-only estimate or a repeated render no longer re-uploads the cloud.
- **`app.html` — the modern "Navigator" frontend.** A single-page, mobile-first
  reskin of the demo: same inline-CSS/HTML + `<script type="module">` style and
  the SAME modules/logic as index.html, with a redesigned presentation layer
  (glass surfaces, CSS custom properties, light/dark via `prefers-color-scheme`,
  safe-area insets, top bar with search + floor stepper + status chip, right-edge
  icon FABs, bottom-left action pills, a draggable bottom sheet for calibration,
  a branded splash, and subtle haptics). The route result is a top-anchored
  **expandable route-preview card** (`showRoute` builds leg-by-leg rows via
  `routePreviewSteps`/`routeStepEl`, using human building names from
  `sourceName`) instead of the cryptic bottom pill; the calibration panel is the
  bottom sheet. The "Find Path" dev button was removed. It reuses every element
  id index.html's script expects (verified by an id cross-check).
- **Cached graph boot (`app.html`).** Each building's `graphUrl` bundle is
  fetched in parallel with the floor JSONs, gunzipped and decoded, then
  rehydrated with `makeBundleGraph()`. Missing/corrupt/floor-mismatched bundles
  fall back to the worker or synchronous build. The decoded graph is exact
  (same nodes, adjacency order, weights and routes); passages/heading fields are
  built lazily on the active floor level, not at boot.
- Serve with `python3 -m http.server 8765` → http://localhost:8765.
