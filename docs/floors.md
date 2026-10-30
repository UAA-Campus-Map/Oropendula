# `floors.mjs` — floor ids, ordering and labels

## What it does

One shared interpretation of floor-id strings for the app, router and import
tool. Exports `floorRank`, `isBasement`, `floorLabelOf` and `floorLevelOf`.

## Semantics

| id | rank | label | 3D level |
|----|------|-------|----------|
| `B2` | -2 | Basement 2 | -1 |
| `B1` | -1 | Basement 1 | 0 |
| `01` | 1 | Floor 1 | 1 |
| `02` | 2 | Floor 2 | 2 |

Basements sort below ground, never as upper floors. Non-numeric ids rank 0 and
render as themselves.

The import tool's advanced **floor offset** rewrites a building's effective
floor ids by N levels (e.g. `B1`→`01`, `01`→`02`, `02`→`03`). The original
level is kept as `sourceFloorId`, so the offset can be changed later. This is
how an elevated skybridge whose plan is labelled `01` can sit at level 2.

## Where it matters

- the app's floor picker starts on the ground floor, not the lowest basement;
- route colours and 3D stacking use `floorLevelOf`;
- `campus.mjs` uses `floorLevelOf(...) === 1` for the campus ground plane and
  for vertical-travel time (so B2→01 is two floors, not one);
- places/search labels basements as "Basement N".

## See also

- [`apps.md`](apps.md) — floor stepper and Room Pin.
- [`campus.md`](campus.md) — vertical travel and outdoor level.
