# `arbiter.mjs` — fusing GPS and dead reckoning

## What it does

Runs **GPS and PDR at the same time** and reports whichever is more certain, so
the app transitions in and out of buildings on its own — no mode switch.
Export: `PositionArbiter`.

Public API: `gps(fix)`, `step({ stepLengthM, headingRad })`,
`setHeading(rad)`, `anchor([lon,lat], opts)`, `clearGps()`, `estimate(t)`,
`pdrAccuracy()`, `insideBuilding(fix)`, `isPlausible(fix)`.

`estimate(t)` returns
`{ lon, lat, source, accuracyM, heading, spreadM, gpsAccuracyM }`.

## Why it's built this way

**Both sources, always.** Switching modes is fragile (mode flaps, stale
transitions). Instead, each source reports an **uncertainty** — GPS its
`coords.accuracy`, PDR an error that grows with distance walked
(`pdrBaseAccM + pdrDriftPerM · metresSinceCorrection`) — and the smaller one
wins.

**Automatic anchoring.** A good fix re-seeds the particle cloud, i.e. GPS acts as
a free anchor and the two sources correct each other. Outdoors the cloud is put
into free roam (`setFreeRoam`) so it is not tied to a floor plan the walker has
left; indoors an indoor/implausible fix must **not** re-seed the cloud, so a
hand-over to PDR happens immediately.

**Hysteresis.** A winner only commits a source change after it stays ahead for
`hysteresisS`, so the marker does not flap at a doorway.

**Validity gate.** A fix near/at a building must land in the walkable area, not
in a wall (`isPlausible`); fixes worse than `gpsMaxAccuracyM` (or with no
accuracy) are ignored. `insideBuilding` uses the *bound floor's* own raster, so a
fix on it means "indoors here".

## What feeds it (and what must not)

The arbiter's inputs are the raw GPS fix and the raw steps + heading from
`motion.mjs`. The step-length auto-calibrator (`autocal.mjs`) deliberately does
**not** take the arbiter/PDR estimate as input — PDR must never calibrate itself.

## Implementation notes

- `metresSinceCorrection` grows by `stepLengthM` per step and resets on a fix or
  an anchor; it drives `pdrAccuracy()`.
- While committed to GPS, the last good fix is **held** for `gpsHoldS` through a
  switch window so the display neither jumps nor flaps before hysteresis commits
  to PDR.
- `anchor()` re-seeds with a known heading spread (±30°) and marks the source
  `pdr`; `clearGps()` forgets the fix so a wrong fix cannot win again after a
  manual anchor.

## See also

- [`locate.md`](locate.md) — the PDR filter it drives.
- [`autocal.md`](autocal.md) — GPS step-length calibration (kept separate).
- [`motion.md`](motion.md) — the heading for the forward-hemisphere prior.
