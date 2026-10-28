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

- [x] Seed the live position after indoor calibration.
- [x] Use one timestamp clock for steps, GPS fixes, and anchors.
- [x] Validate stored auto-calibration before painting it.
- [x] Fix ft/in ↔ cm conversion.
- [x] Show outdoor routes on campus level 1 only (AS164 floor 03).
- [x] Clear stale routes on failed live reroute; keep favorites in sync.
- [x] Fix places regressions: basement “Floor 0”, failed-search favorite row, floor-level semantics.
- [x] Remove dead tap-to-route/path-mode code and stale comments.
- [x] Verify app syntax and targeted smoke groups.

## Phase 3 — Data freshness

- [x] Add a graph-input hash to bundle metadata; fall back on mismatch.
- [x] Compare the same hash in the deploy diff and include regenerated bundles.
- [x] Include wall extents in the app/`gen-osm` bbox.
- [x] Harden PMTiles/MVT decoding (compression, bounds checks, malformed tiles).
- [x] Regenerate all bundles, config, and `osm-paths.json` as needed.
- [x] Verify bundle/deploy/OSM tests.

## Phase 4 — Tests and docs

- [x] Fix the router fixture (`text_lines[0]` shaft/doorless room).
- [x] Update stale docs: Room Pin, blocked portals, calibration, headingfield, align, arbiter, tooling, locate, places, README, agents.
- [x] Add `docs/anchorcal.md`; document `floors.mjs`.
- [x] Archive legacy pages in docs without deleting them.
- [x] Run the targeted suites and the router group.

## Phase 5 — Deploy

- [ ] Deploy changed files only.
- [ ] Verify SHA-256 for every deployed file.
- [ ] No commit or push.
