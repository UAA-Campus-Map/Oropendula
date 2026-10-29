# `roompin.mjs` — Room Pin ("Locate by room")

## What it does

Turns an **exact room number** into a manual PDR anchor just outside the room's
door. The right-edge **Locate by room** button in `app.html` calls
`resolveRoomPin(building, hit, campusRouter)`; the result is fed to
`manualPdrAnchor`, which stops GPS, binds PDR to that building/floor and seeds
the particle cloud.

Export: `resolveRoomPin(building, hit, campusRouter)` →
`{ building, floor, room, doorLL, ll }` or `{ error }`. Helper exports used by
the tests: `floorDoorways`, `firstDoorCrossing`, `sampleTargets`,
`placementOutside`.

## Why it's built this way

An exact room's **door** is not the room's number label and not simply the
nearest doorway — for a large or suite room the nearest doorway may belong to a
neighbour. The door is found by **routing**:

1. **Exact, unique room.** `findRoomExact` (from `places.mjs`); a label drawn on
   more than one floor is refused. No other same-floor room label →
   `Room X is the only room on its floor.`
2. **Probe the floor.** Route from the label to **6 seeded-random other rooms on
   the same floor**. The seed is `building|floor|room`, so a room's probe set is
   deterministic across runs. Each probe contributes its **first door** crossed.
3. **Tolerant crossing** (`geo.crossBarrier` + `geo.nearestApproach`): a door
   counts as crossed when the route crosses its barrier line (±0.4 m past the
   jamb) **or passes within 0.815 m** (ADA 32 in clear width) of the barrier. This
   is essential because the navGraph passage is carved at the DOOR leaf midpoint,
   offset from the wall-gap barrier by ~0.3–0.5 m, so a strict in-segment test
   misses real doors (most of SU-02).
4. **Majority first hop wins** (ties break on distance). Probes that cross no door
   (same open space) are ignored. A door farther than 15 m from the label is not
   the room's. `Room X has multiple doors.` is reported only when two probes leave
   through first doors **≥ 2 m apart** (different openings); closer doors are one
   entrance (a double door / two leaves).
5. **Placement.** 0.5 m beyond the chosen door, continuing the **label → door**
   line (the label is a stable inside-the-room reference). When that line runs
   into a wall (a far, off-axis door — RH 210), it falls back to the **barrier
   normal away from the label**, then the **route direction**, then the **nearest
   walkable point** to the door. The pin is always a walkable point, backed off
   toward the door (0.5 / 0.4 / 0.3 / 0.2 m) if needed.

## Errors

- `Room X is the only room on its floor.` — no other same-floor label to probe.
- `Room X has no nearby doors.` — no probe crossed a door within the 15 m guard.
- `Room X has multiple doors.` — two probes' first doors are ≥ 2 m apart.
- `Room X: couldn't find a spot outside the door.` — a door was found but no
  walkable placement point (distinct from "no nearby doors").

## Hard-won notes

- The **old** approach (probe an 8 m-away door and prove no other doorway is
  crossed) rejected most rooms; every door is now assumed to open onto
  circulation space, so the geometric "hallway" test is gone.
- The placement side was once derived from the **route tangent**, which is
  meaningless for a parallel graze (`n · dir ≈ 0` gives a coin flip). The room
  **label** is the reliable side reference.
- **SU floor 02 is special**: its raster is one open ~1000 m² blob (wall gaps not
  covered by door leaves), so room/door reasoning there is approximate. A
  `buildGraphState` **gap-seal pass** (bridge wall ends < 1 m with no door leaf)
  was tried and **reverted** — sealing the leaks barely changed SU-02's region and
  did not fix the misplacement. Do not reintroduce it casually; prefer fixing the
  floor data.

## Tests

`node smoke.mjs roompin` builds the real AS121-02 floor and checks exact lookup,
a resolved door outside the doorway, the seeded probe set, the tolerant-crossing
helpers, the lone-room error, and interior-door blocking.

## See also

- [`doorways.md`](doorways.md) — the doorways this consumes.
- [`apps.md`](apps.md) — the Locate-by-room UI and the tour text.
- [`places.md`](places.md) — `findRoomExact`.
- [`geo.md`](geo.md) — `crossBarrier` / `nearestApproach`.
