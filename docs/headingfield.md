# `headingfield.mjs` — allowed compass directions & snapping

## What it does

Indoor dead reckoning drifts because a phone's compass is unreliable near
steel and concrete. The floor plan knows better: in a hallway you face one way
along the axis or the other; at a junction the passages point along two axes; a
doorway points into a room. So the plan defines a **set of plausible headings**
at every point — the *heading field* — and the app snaps the live compass to the
nearest member of that set, holding it until the walker deliberately turns.

Exports: `buildPassages`, `makeHeadingField`, `CompassSnapper`, `foldPi`,
`distPi`, `shortestDelta`.

Bearings are radians, clockwise from north (north = +y metres, east = +x).
Directions are **undirected**: every bearing is folded into `[0, PI)`.

## Where the directions come from

`buildPassages(nodes, adj, doorsM, segments)` packs each cue as a segment
`[x1,y1,x2,y2,w, ...]`:

- **Graph edges.** The navgraph's visibility edges already run along hallways
  and out to each branch at a junction, so a bearing histogram of the edges near
  a point *is* the corridor/junction direction set. Weight 1.
- **Doorways.** Each interior door leaf's midpoint, with a short stub along the
  door normal (perpendicular to the nearest wall). Weight 3 — "through the door"
  is a strong option.

It is built **per floor, client-side**. The fallback on-device build creates it
inside the navgraph worker and returns it in the graph state; cached `.nvg.gz`
bundles omit passages, so `makeBundleGraph()` installs `ensureHeadingField()`
and builds the field lazily on the **main thread** the first time the locator
needs directions near that plan. `graphFromState` rehydrates a supplied field
into a query closure (`graph.headingField`). Nothing is precomputed on the
server; boot never pays the full campus passage cost.

## Querying: `makeHeadingField(passages, { toM })`

Builds a spatial grid over the passage segments (each rasterised into the cells
it crosses, so a query cell finds crossing segments, not just midpoints). Then:

`query(ll) -> { candidates: number[], weights: number[], confidence, axis } | null`

- Gathers passages within `radiusM` (default 4.5 m) of the point.
- Bins their bearings (72 bins), smooths circularly, and returns the distinct
  peaks as `candidates` (strongest first) with `axis` = the strongest.
- `confidence = (strongest peak) / (total weight)` — high in a corridor
  (bearings concentrate on the axis), lower at a junction (~0.5), low in an open
  room (bearings fan out). The snapper ignores a field below `minConfidence`.

## Snapping: `CompassSnapper`

State machine `SEARCHING -> LOCKED(dir) -> (turn) -> SEARCHING`.

- **SEARCHING:** lock the nearest candidate within `snapTol` (35°) once the field
  is confident enough (`minConfidence`, 0.4).
- **LOCKED:** pull the heading toward `dir` with a small, bounded correction
  (`gain` 0.5, deadband 3°, clamp 3°/step, distrust > 50°) — never a hard
  quantise, so the readout stays continuous.
- **Turn release:** an *obvious turn* — a single-step change > `turnStepDeg`
  (25°) or a net change > `turnWindowDeg` (35°) over `turnWindowSteps` (3) steps
  — releases the lock *before* it can fight the turn. At a junction the lock then
  re-snaps to the new branch.
- **Guards:** nothing outdoors / in free-roam, during the calibration walk, or
  when the field is absent (open room, wide lobby).

The correction is applied to the app's heading bias (session-only, like the old
corridor lock) — it never touches localStorage. While the snapper is **not
locked**, the app multiplies `baseHeadingBias` by **0.95 per step**, so a
correction (or a stale walk-calibration offset) eases back toward the raw
compass and can recover. The ±45° manual trim is separate and does not decay.
The Advanced panel's *Snap* row shows the locked direction and confidence, and
*Snap heading to corridors* disables it (which also freezes the decay).

## Why not use the gyro to correct the compass?

An earlier attempt used the gyro as a rotation reference to *predict* the compass
and fold the mismatch into an offset. It failed because the compass is laggy and
on a different scale from the gyro, so real turns looked like drift and were
suppressed. This design uses the plan for the *directions* and the heading
stream only to detect turns; the gyro is not needed for the correction.

## See also

- [`motion.md`](motion.md) — heading smoothing + gyro bridging.
- [`locate.md`](locate.md) — the particle filter that consumes heading.
- `navgraph.mjs` — builds and serialises the field per floor.
