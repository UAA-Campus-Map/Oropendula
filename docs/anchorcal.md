# `anchorcal.mjs` — step-length refinement from manual anchors

## What it does

Turns two manual position marks ("I'm Here" taps) plus the steps and headings
walked between them into one step-length observation. Exports
`AnchorStepCalibrator`.

## Why it is separate from `autocal.mjs`

`autocal.mjs` learns from GPS speed/position while walking outdoors. This module
works **indoors**, where GPS is unusable, but only for a straight leg: if the
walker went from A to B in a straight line on one floor, the walked distance is
`|AB|` and the base step length is `|AB| / Σ f(cadence)`, where `f()` is the
`StepLengthModel` cadence scaling.

## What a leg must satisfy

- same building and floor;
- ≥10 steps, ≥6 m, no internal gap >1.5 s;
- heading concentration ≥0.97 and net turn ≤12°;
- mean heading within 25° of the A→B bearing;
- the straight A→B span stays walkable (not through a wall).

Samples are inverse-variance weighted with a 1.5 % systematic floor, and the
same +-6 % at 90 % gate as `autocal.mjs` must pass before the caller applies the
result. The feature is off by default (`pdr-anchor-cal` in the app's advanced
panel).

## See also

- [`autocal.md`](autocal.md) — the GPS counterpart.
- [`motion.md`](motion.md) — cadence scaling.
- [`apps.md`](apps.md) — the "Refine step length from my pins" toggle.
