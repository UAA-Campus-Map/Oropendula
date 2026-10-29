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
neighbour. The door is found from **watershed connectivity**:

1. **Exact, unique room.** `findRoomExact` (from `places.mjs`); a label drawn on
   more than one floor is refused.
2. **Watershed territories.** `watershed.mjs` partitions the floor from the room
   labels + connector/exit seeds. A room's **doors are the throats (region
   boundaries)** on its territory. A throat narrower than a body (`0.6 m`) is not
   a door, and doors into rooms smaller than `8 m²` (closets/shafts) do not count.
   Two separate openings between the same two regions are counted
   separately (boundary cells split into connected components). **More than one
   door → `Room X has multiple doors.`**; one door is used.
3. **Placement.** 0.5 m beyond the throat into the neighbouring region,
   continuing the **label → throat** line (the label is a stable inside-the-room
   reference). If that line hits a wall, fall back to the **nearest walkable point**
   to the throat. The pin is always walkable.
4. **Fallback (no watershed).** When the floor cannot be segmented (no
   walls/labels), route from the label to **6 seeded-random other rooms on the
   same floor** and take each probe's **first door** crossed (tolerant crossing:
   barrier line ±0.4 m past a jamb, or within 0.815 m / ADA 32 in of it, because
   the navGraph passage is offset from the wall-gap barrier). Majority first hop
   wins; distinct first doors ≥ 2 m apart are multiple doors.

## Errors

- `Room X is the only room on its floor.` — no other same-floor label to probe
  (fallback path only).
- `Room X has no nearby doors.` — no door found (fallback: no probe crossed one).
- `Room X has multiple doors.` — more than one watershed throat ≥ 0.6 m into a
  room ≥ 8 m² (fallback: two probes' first doors ≥ 2 m apart).
- `Room X: couldn't find a spot outside the door.` — a door was found but no
  walkable placement point.

## Hard-won notes

- The **old** approach (probe an 8 m-away door and prove no other doorway is
  crossed) rejected most rooms; doors now come from watershed connectivity, so an
  opening wide enough to walk through is a door regardless of the DXF leaf.
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
a multi-door room refused (ECB 208), a single-door room resolved outside its
doorway, the seeded probe fallback, the tolerant-crossing helpers, the lone-room
error, and interior-door blocking.

## See also

- [`doorways.md`](doorways.md) — the doorways this consumes.
- [`apps.md`](apps.md) — the Locate-by-room UI and the tour text.
- [`places.md`](places.md) — `findRoomExact`.
- [`geo.md`](geo.md) — `crossBarrier` / `nearestApproach`.
