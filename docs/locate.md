# `locate.mjs` — the pedestrian dead-reckoning particle filter

## What it does

Estimates **where the walker is** from steps + heading + the floor plan, with no
beacon or camera. It also finds a corridor's axis (to correct heading drift
indoors). Exports:

- `ParticleFilter` — map-matching particle filter (PDR).
- `corridorAxis(isWalkable, ll, opts)` — ray-casts free space for a corridor
  bearing (null in an open room).
- `HeadingDriftCalibrator` — nudges the heading bias toward the corridor axis
  while the walker goes straight, and stops on an intentional turn.

## Theory: what a particle filter is

A particle filter represents a **probability distribution over a hidden state**
(here: the walker's position and heading) as a set of weighted samples —
"particles" — rather than as a formula. Each step runs the same loop:

1. **Predict.** Push every particle through the motion model with noise —
   "advance one step length along the heading, plus a little randomness."
2. **Weight.** Score every particle by how likely the newest observation is if
   that particle were the truth. Impossible particles get (near) zero weight.
3. **Normalise** so the weights sum to one.
4. **Resample.** When the weights concentrate — the *effective sample size*
   `1/Σw²` falls below half of `N` — draw a fresh generation in proportion to
   weight; low-weight particles die, high-weight ones reproduce.
5. **Estimate.** Report a summary (the weighted mean) as the position.

Particle filters are the right tool when the system is non-linear and/or
**multi-modal**, where a Kalman filter's single Gaussian is simply wrong — two
symmetric corridors, or both sides of a thin wall.

### Why this particular design

**The floor plan is the sensor.** There is no range/bearing measurement; what we
have is a precise map and one physical fact: a body cannot walk through a wall.
So the "measurement" is a binary `isWalkable` test (from `navgraph.mjs`) — the
building itself is the observation. This is **map matching**: grade each
hypothesis by whether it is physically possible. The filter and the router share
the same clearance raster, so they can never disagree about where a body fits.

**Motion model = step events.** On each detected step every particle advances
`stepLengthM` along its heading, with:

- a **bounded heading jitter** (`headingJitterDeg = 35`) that is an *offset of
  the measured heading*, never a full-circle draw;
- **along-track** distance noise (`distanceNoise = 0.22`) and **cross-track**
  lateral noise perpendicular to travel (`lateralNoiseM`), so the cloud spreads
  across a corridor rather than along it.

**Blocked ⇒ re-spawned, not merely down-weighted.** A particle whose step a wall
blocks is a wrong guess. It is destroyed and replaced by a **survivor's**
position (plus a small jitter) — never a fresh random point, which could sit in a
room the walker could not have reached. There is no sliding; the cloud cannot
pile up against a wall. Doorways work for particles whose step threads the gap.

**Cheap priors sharpen a weak signal.** A **forward-hemisphere** prior
down-weights particles behind the (measured) heading, and the arbiter's
**accuracy radius** bounds how far a particle may live from the cloud mean. A
U-turn step or a stall-recovery step lifts both.

**It must be able to recover.** If the cloud is misplaced and *every* step is
blocked, a naive filter freezes. The **stall guard** relaxes the priors for a few
steps so the noise re-diffuses the cloud toward a doorway — while collision stays
on, so a particle still never crosses a wall.

**Never report an impossible point.** `estimate()` averages only walkable
particles (a cloud straddling a thin wall would otherwise average *into* the
wall) and falls back to the heaviest walkable particle.

**It is a tracker, not a global localiser.** With no absolute measurement the
filter is translation-invariant: a straight hallway is equally consistent with
every parallel hallway. It needs an anchor (a GPS fix or a manual "I'm Here")
between corrections; the floor plan is what stops it drifting through walls.
(Measured: over a square+hallway, pure PDR keeps ~4–8k of the 1 m bins occupied —
the belief does *not* collapse on its own.)

## Multi-hypothesis step length (experimental)

**The problem.** People do not walk at one fixed pace; the cadence model is a
heuristic. But step length is only weakly observable from map matching.

**The idea.** Instead of estimating a step length directly, keep **K discrete
step-length hypotheses** — multipliers `λ` on `stepLengthM`, e.g.
`[0.80 … 1.20]`. Each particle belongs to one hypothesis and advances by
`stepLengthM · λ`. Hypotheses are scored by the geometry, and the belief is the
**population share** per hypothesis.

- **Why not a per-particle continuous scale?** That suffers *parameter
  impoverishment*: resampling collapses the scale distribution, and a persistent
  scale is biased by the filter's own heading-jitter forward-loss. Discrete
  sub-populations are stable and easy to reason about.
- **Where the signal comes from.** A straight corridor is translation-invariant,
  so every hypothesis survives equally — **no information, no adaptation**. At a
  turn, a doorway or a corridor end, the symmetry breaks: a wrong cumulative
  distance puts the particle in a wall, and it dies. Survivors are the right
  length. The mixture moves only at those constraints.
- **Migration (`hypMigrate`).** A wall-blocked particle **adopts a survivor's
  hypothesis** instead of dying. Its position is still inherited, so localisation
  is preserved — only the step-length offset changes. Wrong hypotheses drain into
  the ones that keep threading the geometry. A small random **mutation**
  (`hypMut`) keeps exploration alive.
- **Where it works / limits.** Under the app's aggressive heading jitter the
  absolute estimate carries a ~+4–5 % bias (heading-jitter forward loss), so it
  is a **relative** signal: the app keys its gated update on the *change*
  (`base *= est/baseline`). It orders slow < normal < fast reliably after a few
  turns, but should be treated as a slow refinement, not a precise measurement.

API: `setStepHypotheses(lambdas, { sigma, alpha, rho, migrate, mut })`,
`resetStepHypotheses()`, `stepScaleEstimate()`, `stepScaleConfidence()`,
`stepHypotheses()`. Default **off** (a single cloud).

## Corridor heading helpers

- `corridorAxis` ray-casts the free space around a point and returns the axis
  bearing when the space is strongly elongated (a corridor), else null (open
  room). This is how the app knows the walker is in a hallway.
- `HeadingDriftCalibrator` accumulates a straight streak and nudges the shared
  heading bias toward the corridor axis; a large heading swing (an intentional
  turn) disarms it. Indoors the magnetometer is unreliable; this borrows the
  building's own axis as truth.

## Implementation notes / invariants

- `setFreeRoam(on, isAllowed?)` lifts the floor-plan constraint outdoors
  (skybridges, gaps between buildings, unmapped walkways), optionally constrained
  by an `isAllowed(ll)` predicate so PDR cannot cut through a building.
- `points()` returns `{ lon, lat, w, g }` for the map overlay (`w` is relative
  weight, `g` the hypothesis index); empty until seeded.
- `version` is bumped whenever the cloud changes, so the renderer can skip
  rebuilding the overlay for GPS-only estimates.
- Fixed particle count (`DEFAULT_COUNT = 1000`); arrays are sized once and the
  resample is allocation-free (ping-pong buffers). `predict` is ~O(N) at ~1 µs
  per particle.

## See also

- [`navgraph.md`](navgraph.md) — supplies `isWalkable`.
- [`motion.md`](motion.md) — supplies steps and heading.
- [`arbiter.md`](arbiter.md) — decides GPS vs PDR and passes the accuracy radius.
- [`autocal.md`](autocal.md) — a stronger, GPS-based way to learn step length.
