# HTML entry points

The project has no build step and no npm dependencies: each page is a single
HTML file with inline CSS, markup and a `<script type="module">` that imports the
`src/*.mjs` modules. Serve the `src/` folder statically (e.g.
`python3 -m http.server 8765`) and open a page.

Production entry point: **https://uaa-campus-map.net** (the previous hostname
is retired). Basemap/style and PMTiles assets are served from the same domain.

## `app.html` — the modern "Navigator" frontend

A single-page, mobile-first reskin of the demo using the **same modules and
logic** as `index.html`, with a redesigned presentation layer: glass surfaces,
CSS custom properties, light/dark via `prefers-color-scheme`, safe-area insets, a
top bar (search + floor stepper + status chip), right-edge icon buttons,
bottom-left action pills, a branded splash, and subtle haptics.

- **Route preview** is a top-anchored expandable card (`showRoute` →
  `routePreviewSteps`/`routeStepEl`) with tappable legs that fly the map to where
  each step happens, instead of the older bottom pill.
- **Calibration** is a **modal** (phone-only; hidden on desktop) that replaces the
  old line-walk panel. It offers three ways to set the step length: *estimate from
  height*, *calibrate from GPS (outside)* — the `autocal.mjs` flow — and a
  *guided indoor walk*. The splash mirrors the live status line while booting.
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

The right-edge **Locate by room** button places a manual PDR anchor in the
hallway outside a room in the current building. It accepts either a bare room
number (`211`, defaulting to the current building) or a building code plus room
(`AS124 211` / `LIB 211`), using the same longest-code matching as the search
bar. It requires an exact, unique room match, clusters DOOR-layer leaves into
doorways, and only accepts a door that opens onto a hallway. If the room has
multiple hallway doors, multiple hallways, no hallway door, or an open exterior
gap, it refuses to place the pin. A successful placement stops GPS, binds PDR
to that building/floor, and seeds the particle cloud at the centre of the
hallway outside the door.

### Blocked doors/staircases

The red crossed-circle button opens **Mark door blocked?**. Confirming blocks
the next door or staircase on the current route for this session and re-plans
from the same origin/destination. Blocked portals are passed to
`campusRouter.route(..., {avoidPortals})`; no other route is affected.

### First-load interface tour

The first visit shows a short spotlight tour after the calibration modal closes
(or after boot when calibration is already complete). It points at the search
bar, floor control, 2D/3D, Follow, GPS, “I’m Here”, Room Pin, blocked-door, and
calibration controls with 3–10 word descriptions. The final button says
**“Let’s go”**. Completion or **Skip** stores `interface-tour` in
`localStorage`, so the tour does not reappear.

### Notable internals
- The particle cloud is the production position indicator (`pdr-particles`),
  drawn from `ParticleFilter.points()`.
- **Cached graphs:** boot fetches each building's `graphUrl` bundle in parallel,
  gunzips/decodes it, and rehydrates a `graphFromState` graph. A missing,
  corrupt, or floor-mismatched bundle falls back to the worker/synchronous
  on-device build for that floor. Passages/heading fields are built lazily when
  the locator first needs compass directions near a plan.
- `positionSeeded` suppresses the live marker and re-routing until an absolute
  source (GPS, "I'm Here", or a finished calibration) has seeded the cloud — the
  boot seed at the map centre is a guess and must not move the map.
- Map layers: `campus-3d*` (extruded structure/route/avatar), `pdr-particles`,
  `location-heading`, `portals`, plus the OSM basemap.

## `index.html` — the original demo

The older desktop-friendly UI, same pipeline. It retains the line-walk
"Calibrate walking model" panel. `?dev=1` reveals the developer "Find Path"
button and the on-map particle overlay.

## `import.html` — the authoring tool

Loads a DXF floor, places it with `geo.mjs`, aligns stacked floors with
`align.mjs`, tags portals, and writes the floor JSON + the `buildings-config.json`
registration (including the cached portal set). **Buildings are added through this
tool, never by hand-editing `index.html`.**

The Deploy section also generates per-building navGraph bundles
(`<buildingId>.nvg.gz`). Generating a bundle sets that building's `graphUrl` in
`buildings-config.json`; the bundle is gzipped and contains every floor in the
building. Use the current-building button for one bundle or "Download all
navgraph bundles" for a full regeneration.

**Deploy diff:** "Check changes vs deployed" compares the draft config with the
server's `buildings-config.json` (canonical JSON, by building id) and reports
added/changed/removed buildings. "Download changed buildings (.zip)" builds a
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
localStorage like `import.html`. Behind nginx basic auth. See
[`places.md`](places.md).

## `locate-test.html` — a positioning testbed

A test-only live map for PDR: loads floor plans + basemap, draws the GPS fix and
the GPS/PDR estimate with an uncertainty circle, heading arrow and breadcrumb
trail, and lets you tap to place an anchor. It imports the real modules, so it is
a convenient place to observe the filter without sensors.

## See also

- [`tooling.md`](tooling.md) — the test suite that verifies the shared logic.
- [`navgraph-bundles.md`](navgraph-bundles.md) — the cached binary graph format used by `app.html`.
- [`locate.md`](locate.md), [`autocal.md`](autocal.md) — the algorithms the
  calibration flows drive.
