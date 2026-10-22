# HTML entry points

The project has no build step and no npm dependencies: each page is a single
HTML file with inline CSS, markup and a `<script type="module">` that imports the
`src/*.mjs` modules. Serve the `src/` folder statically (e.g.
`python3 -m http.server 8765`) and open a page.

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

## `import-progress.html` — import dashboard

Compares the deployed `buildings-config.json` against `floor-catalog.json` and
shows how many buildings/floors are imported, per-building missing floors, and a
link into the import tool. The catalog is a snapshot of the private
Extraction-Tool `Data Extraction/Floor Data` folder; regenerate it with
`node gen-floor-catalog.mjs "<Floor Data dir>"`. The page can also read a local
folder directly (stored in `localStorage`) when the source changes.

## `locate-test.html` — a positioning testbed

A test-only live map for PDR: loads floor plans + basemap, draws the GPS fix and
the GPS/PDR estimate with an uncertainty circle, heading arrow and breadcrumb
trail, and lets you tap to place an anchor. It imports the real modules, so it is
a convenient place to observe the filter without sensors.

## See also

- [`tooling.md`](tooling.md) — the test suite that verifies the shared logic.
- [`locate.md`](locate.md), [`autocal.md`](autocal.md) — the algorithms the
  calibration flows drive.
