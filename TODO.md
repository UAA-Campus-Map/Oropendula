# Oropendula fix plan

Pause after each phase so the changes can be committed.

## Phase 1 — Routing and Room Pin

- [x] Shared doorway model: cluster DOOR leaves, stable IDs, barrier across each opening.
- [x] Use the doorway model in `places.mjs` and Room Pin; fix ECB 208 false negative.
- [x] Make interior doors blockable; barriers only for blocked doors.
- [x] Give every elevator/stair connector a unique ID.
- [x] Fix explicit skybridge destination conflicts.
- [x] Fix basement floor travel and buildings without a configured ground floor.
- [x] Harden `findPath`/`attach` against blocked endpoints and empty-wall floors.
- [x] Add `doorways` and `roompin` tests, including ECB 208.
- [x] Verify: codec tests, `places`, `doorways`, `roompin`, `walkability`, `portals`, `multifloor`, `osm`.

## Phase 2 — App and calibration

- [ ] Seed the live position after indoor calibration.
- [ ] Use one timestamp clock for steps, GPS fixes, and anchors.
- [ ] Validate stored auto-calibration before painting it.
- [ ] Fix ft/in ↔ cm conversion.
- [ ] Show outdoor routes on campus level 1 only (AS164 floor 03).
- [ ] Clear stale routes on failed live reroute; keep favorites in sync.
- [ ] Fix places regressions: basement “Floor 0”, failed-search favorite row, floor-level semantics.
- [ ] Remove dead tap-to-route/path-mode code and stale comments.
- [ ] Verify app behavior and targeted smoke groups.

## Phase 3 — Data freshness

- [ ] Add a graph-input hash to bundle metadata; fall back on mismatch.
- [ ] Compare the same hash in the deploy diff and include regenerated bundles.
- [ ] Include wall extents in the app/`gen-osm` bbox.
- [ ] Harden PMTiles/MVT decoding (compression, bounds checks, malformed tiles).
- [ ] Regenerate all bundles, config, and `osm-paths.json` as needed.
- [ ] Verify bundle/deploy/OSM tests.

## Phase 4 — Tests and docs

- [ ] Fix the router fixture (`text_lines[0]` shaft/doorless room).
- [ ] Update stale docs: Room Pin, blocked portals, calibration, headingfield, align, arbiter, tooling, locate, places, README, agents.
- [ ] Add `docs/anchorcal.md`; document `floors.mjs`.
- [ ] Archive legacy pages in docs without deleting them.
- [ ] Run the targeted suites and the router group.

## Phase 5 — Deploy

- [ ] Deploy changed files only.
- [ ] Verify SHA-256 for every deployed file.
- [ ] No commit or push.
