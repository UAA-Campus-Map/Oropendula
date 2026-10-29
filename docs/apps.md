# HTML entry points

The project has no build step and no npm dependencies: each page is a single
HTML file with inline CSS, markup and a `<script type="module">` that imports the
`src/*.mjs` modules. Serve the `src/` folder statically (e.g.
`python3 -m http.server 8765`) and open a page.

Production entry point: **https://uaa-campus-map.net** (the previous hostname
is retired). Basemap/style and PMTiles assets are served from the same domain.

## `app.html` — the modern "Navigator" frontend

A single-page, mobile-first Navigator sharing the core routing, PDR and
search modules with `index.html`, with production-only additions (cached
navGraph bundles, place search/favorites, Room Pin, interior-door blocking and
the calibration modal). Presentation: glass surfaces,
CSS custom properties, light/dark via `prefers-color-scheme`, safe-area insets, a
top bar (search + floor stepper + status chip), right-edge icon buttons,
bottom-left action pills, a branded splash, and subtle haptics.

- **Route preview** is a top-anchored expandable card (`showRoute` →
  `routePreviewSteps`/`routeStepEl`) with tappable legs that fly the map to where
  each step happens, instead of the older bottom pill.
- **Planning a route never changes the floor** and no longer zooms to the
  destination: it **fits the view to the whole route** (`routeBounds`) and
  reveals the destination pin even when the destination is on another floor
  (`destinationRevealed`, cleared on the next manual floor change). Tapping a
  route step still flies to that step and switches floor.
- **Advanced Settings** is a **modal** (opened by the menu button, which is hidden
  on desktop) that holds calibration and the compass/graph controls. Calibration
  offers three ways to set the step length: *estimate from height*, *calibrate
  from GPS (outside)* — the `autocal.mjs` flow — and a *guided indoor walk*.
  The heading bias is applied live but decays by **×0.95 per step whenever the
  corridor snapper is not locked** (the ±45° trim does not decay). The splash
  mirrors the live status line while booting.
- **The guided indoor walk** asks the user to stand outside a doorway and type the
  room they are in; `findRoom` resolves it, `roomDoorViaRoute` confirms the room's
  egress door by routing, and `pickIndoorTarget` chooses a same-floor room a short
  walk away reachable **without passing through another room's door**. Start and
  end are marked with flags (a green pin and a checkered flag); the route is
  framed on the map. `recordIndoorWalk` computes `distance / steps` and applies
  it.

### Units
Internal math is metric. The **UI shows Imperial**: `distanceText` (feet, miles
past ¼ mile), `stepLenText` (inches), `speedText` (ft/s), and the height
estimator defaults to **ft·in** with a cm toggle.

Route ETAs use the user's calibrated step length and recent walking cadence for
walking legs. The scale is:

```text
reference speed = 0.72 m × 1.8 steps/s
current speed   = stepModel.lengthFor(cadence) × cadence
seconds        *= reference speed / current speed
```

Recent sensor cadence (last ~10 s) is used when available; otherwise the
model's reference cadence. The router still chooses the route with its nominal
speeds, so calibration changes the reported time, not the path. Elevator/stairs
time remains fixed.

### Floor ids

Floor ids are strings. `floors.mjs` is the shared source of truth: `B1`/`B2`
sort **below** `01`, display as **Basement 1**/**Basement 2**, and map to 3D
levels `0`/`-1` (ground `01` is level `1`). The floor stepper starts on the
ground floor even when a building has a basement.

### Room pin

The right-edge **Locate by room** button places a manual PDR anchor just
outside a room in the current building. It accepts either a bare room number
(`211`, defaulting to the current building) or a building code plus room
(`AS124 211` / `LIB 211`), using the same longest-code matching as the search
bar. It requires an exact, unique room match. The room's door comes from
**watershed connectivity** (`watershed.mjs`): the floor is partitioned into room
and corridor territories from the labels + connector/exit seeds, and a room's
**doors are the throats (region boundaries)** on its territory. A throat narrower
than a body (`0.6 m`) is not a door, and doors into rooms smaller than `8 m²`
(closets/shafts) do not count; **more than one door is refused**
(`has multiple doors`), and two separate openings between the same two regions
count separately. The user is placed **0.5 m beyond that throat** into the
neighbouring region, continuing the line from the room's label through the throat;
if that line hits a wall it falls back to the nearest walkable point. The pin is
always walkable.

When a floor cannot be segmented (no walls/labels), Room Pin falls back to
**routing** from the label to **6 seeded-random other rooms on the same floor**
and taking the **first doorway each route crosses** (majority first hop; distinct
first doors ≥ 2 m apart are multiple doors; the door must be within 15 m).

Door *crossing* is tolerant (`geo.crossBarrier` + `geo.nearestApproach`): a
doorway's barrier is the wall gap, but the navGraph passage is carved at the
door leaf midpoint, so a route often threads the opening a few decimetres to the
side of the barrier. A door counts as crossed when the route crosses the
barrier's line (±0.4 m past the jamb) **or** passes within 0.815 m (the ADA
minimum clear door width, 32 in) of the barrier segment through its opening. The
same rule backs interior-door detection for **blocked doors**.

### Blocked doors/staircases

The red crossed-circle button opens **Mark door blocked?**. Confirming blocks
the next exterior door, interior door or staircase on the current route for
this session and re-plans from the same origin/destination. Interior doors are
wall-aligned barriers, so blocking one removes that opening without blocking
other doors. Blocked portals are passed to
`campusRouter.route(..., {avoidPortals})`; the session-wide block set applies to
every later route until reload, and there is deliberately no unblock UI.

### First-load tour

The first visit is a guided tour, not a separate calibration popup followed by
spotlights. It runs in this order:

1. **Welcome** large modal: “UAA Campus Map”, a one-line explanation, and the
   first-visit note.
2. **How positioning works** large modal: GPS outdoors; step counting indoors;
   the next screen calibrates the step length.
3. **Calibration page**: opens the real calibration modal (height estimate,
   GPS auto-calibration, or guided indoor walk). Closing calibration resumes the
   tour at the correction page.
4. **Correcting your position** large modal: pins and the room search tool.
5. **Interface spotlight**, starting with the location pin (“I’m Here”) and
   Room Pin, then search, GPS, Follow, 2D/3D, blocked-door and Advanced Settings
   (the menu button). The floor-picker step is last and spells out that the
   picker must match the floor the user is actually on; that help text is also on
   the control itself.
6. **Summary** large modal with three numbered steps: set your position, input
   your destination, and correct your position/floor as necessary. The
   spotlight’s final **“Next →”** opens this page; **“Start navigating”**
   closes it.

The **Next button has a deliberate cooldown** so a rapid tap cannot skip several
screens: it is disabled (greyed out) for **2.5 s after each big modal page
appears** and **1 s after each spotlight step appears** (the cooldown starts when
the screen is shown, so it is preserved however the screen was reached — e.g. the
calibration flow handing back to the correction page).

Every tour screen — the large modals, the calibration modal header, the
spotlight, and the summary — shows **“Step n of m”** in the same sequence.
Completion or **Skip** stores `interface-tour` in `localStorage`, so the tour
does not reappear. Returning users who skipped calibration are still prompted
separately by `autoOpenCalibrationModal()`.

### Notable internals
- **Debug region overlay:** Advanced Settings → **Compass & graph** → *Debug* →
  **Show regions (debug)** paints the watershed room/corridor territories in 4
  colours over the selected floor (`watershed.mjs`). Visual aid only — no labels
  or boundaries, off by default, hidden in 3D.
- The particle cloud is the production position indicator (`pdr-particles`),
  drawn from `ParticleFilter.points()`.
- **Cached graphs:** boot fetches each building's `graphUrl` bundle in parallel,
  gunzips/decodes it, and rehydrates a `graphFromState` graph. Each floor block
  stores a `sourceHash` of the wall/door inputs; a missing, corrupt,
  building-mismatched, or hash-mismatched bundle falls back to the
  worker/synchronous on-device build for that floor. Passages/heading fields
  are built lazily when the locator first needs compass directions near a plan.
- `positionSeeded` suppresses the live marker and re-routing until an absolute
  source (GPS, "I'm Here", or a finished calibration) has seeded the cloud — the
  boot seed at the map centre is a guess and must not move the map.
- Map layers: `campus-3d*` (extruded structure/route/avatar), `pdr-particles`,
  `location-heading`, `portals`, plus the OSM basemap.

## `index.html` — the original demo

The older desktop-friendly UI, sharing the core routing/PDR modules but **not**
the app’s cached-bundle boot, place/favorites features, Room Pin, interior-door
blocking or calibration modal. It retains the line-walk "Calibrate walking
model" panel. `?dev=1` reveals the developer "Find Path" button and the on-map
particle overlay.

### Archived demos

`index2.html`, `index3.html`, `index_v1.html` and `index_v2.html` are older
experiments kept in the tree for reference. They are not part of the production
path, are not covered by the smoke suite, and some reference historical assets.
Do not deploy or document them as live entry points.

## `import.html` — the authoring tool

Loads an already-extracted floor JSON (the DXF extractor is external), places it
with `geo.mjs`, aligns stacked floors with `align.mjs`, tags portals, and writes
the `buildings-config.json` registration (including the cached portal set). Floor
JSONs themselves are uploaded separately. **Buildings are added through this
tool, never by hand-editing `index.html`.**

The Deploy section also generates per-building navGraph bundles
(`<buildingId>.nvg.gz`). Generating a bundle sets that building's `graphUrl` in
`buildings-config.json`; the bundle is gzipped and contains every floor in the
building. Use the current-building button for one bundle or "Download all
navgraph bundles" for a full regeneration.

**Walkable (flood):** before each floor's graph is built, the tool floods the
outside from the wall-bbox ring, seals interior doorways, exterior exits and
skybridges, and masks everything the flood reached. The panel can preview the
mask (green indoor, grey outdoor), override the indoor/outdoor seeds, and add
two-point seals for openings the DXF door layer misses. Same-level buildings
whose bbox overlaps contribute their walls. A leak (the outdoor flood reaches
the auto indoor room label) **refuses bundle generation**; seal it or move the
seed, then regenerate. The mask is baked into the `.nvg.gz`; see
[`walkable.md`](walkable.md).

**Deploy diff:** "Check changes vs deployed" recomputes each building's
graph-input fingerprint from the local floor JSONs and compares the draft
config with the server's `buildings-config.json` (canonical JSON, by building
id); it reports added/changed/removed buildings. This catches a wall change even
when no config field was edited. "Download changed buildings (.zip)" builds a
single ZIP with the full `buildings-config.json`, full `osm-paths.json` (or a
freshly generated one if the cache is stale), and `.nvg.gz` files only for added
or changed buildings. Unchanged bundles are not included. Floor JSONs are not
put in this ZIP; upload those separately when a building's floor data is new.

## `import-progress.html` — import dashboard

Compares the deployed `buildings-config.json` against `floor-catalog.json` and
shows how many buildings/floors are imported, per-building missing floors, and a
link into the import tool. The catalog is a snapshot of the private
Extraction-Tool `Data Extraction/Floor Data` folder; regenerate it with
`node gen-floor-catalog.mjs "<Floor Data dir>"`. The page can also read a local
folder directly (stored in `localStorage`) when the source changes.

## `places.html` — the places admin page

Edits `places.json`, the place nicknames the search box understands ("Subway",
"Robotics Club"). Add/edit/delete places, pick the room from the labels drawn
on each registered building's plans (a ✓/✗ column shows whether it resolves),
add the names students voted for in the suggestion form, then
**Download places.json** and upload it. Work in progress is kept in
localStorage like `import.html`. It is a static page with no server-side write
endpoint and is served without basic auth; publishing still requires the
download + upload step. See [`places.md`](places.md).

## `locate-test.html` — a positioning testbed

A test-only live map for PDR: loads floor plans + basemap, draws the GPS fix and
the GPS/PDR estimate with an uncertainty circle, heading arrow and breadcrumb
trail, and lets you tap to place an anchor. It imports the real modules, so it is
a convenient place to observe the filter without sensors.

## `watershed.html` — watershed test page

Test-only visualiser for `watershed.mjs`. Pick any registered building and floor
and it paints the room/corridor territories in 4 colours, with:
- **throats = region boundaries** as door markers (green = corridor-facing,
  orange = room↔room), sized by width;
- seeds (room labels vs corridor), and the source JSON layers colour-coded
  (walls black, windows cyan, doors red, stairs purple, elevators amber; other
  layers hashed) with a per-layer show/hide legend;
- a click-a-region popup with area and adjacent throats/widths.

It is a debugging aid for door/region behaviour, not a production entry point.
The same colouring is available inside `app.html` behind **Advanced Settings →
Compass & graph → Debug → Show regions (debug)**.

## See also

- [`tooling.md`](tooling.md) — the test suite that verifies the shared logic.
- [`navgraph-bundles.md`](navgraph-bundles.md) — the cached binary graph format used by `app.html`.
- [`roompin.md`](roompin.md), [`watershed.md`](watershed.md), [`doorways.md`](doorways.md)
  — Room Pin, the segmentation it uses, and the DXF doorway geometry.
- [`locate.md`](locate.md), [`autocal.md`](autocal.md) — the algorithms the
  calibration flows drive.
