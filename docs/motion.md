# `motion.mjs` — sensors → steps, heading and step length

## What it does

Turns raw phone sensors into the two things dead reckoning needs — **when a step
happened** and **which way the walker is facing** — plus a step-length model.
It is pure, testable code; the browser wiring is one small function.

Exports: `StepDetector`, `HeadingTracker`, `StepLengthModel`, `WalkCalibrator`,
`sensorSupport`, `headingFromOrientation`, `createMotion`, and the angle helpers
`wrapAngle`, `shortestDelta`, `clamp`.

## Theory & why

### Step detection (`StepDetector`)

A step is not "acceleration above a threshold". Naïve magnitude peaks count the
phone being shifted or a weight shift, and miss light gait. Instead the signal is
built in stages:

1. **Gravity projection.** A slow low-pass of the raw accelerometer vector gives
   the gravity direction *in the device frame*; the signal is the component of
   acceleration along it. Unlike raw `|a|`, this is inert to rotating or waving
   the phone — the axes and gravity sweep together, but the component along
   gravity stays stable.
2. **Band-pass 0.5–3.5 Hz.** A time-based high-pass removes slow drift and
   orientation, a low-pass removes fast jitter, leaving the gait band.
   Time-based coefficients (`exp(-dt/tau)`) keep it frame-rate independent.
3. **Schmitt trigger + prominence.** A step is a local maximum above a threshold
   that also stands `minProminence` above the valley since the last step; after a
   step the detector will not re-arm until the signal falls `releaseMps2` below
   it. This kills the double-bumps a plain local-max picks up.
4. **Adaptive floor.** While not moving, a slow EMA estimates ambient noise and
   the gate rises with it (capped), so a noisy device counts less without a fixed
   floor that would miss a light step.

A peak alone is still not a step: the detector buffers peaks until a **walking
rhythm** is established (`qualifySteps` peaks with stable spacing). Only then are
the buffered peaks released. A long gap ends the walk and resets the rhythm.

### Heading (`HeadingTracker`)

The compass sets the absolute heading (with a calibrated **bias**); the gyro
bridges gaps. Compass samples are smoothed **circularly** (sin/cos accumulators,
so the value never jumps across the ±π seam) with a time-based low-pass; gyro
integration is applied directly but only when the compass is stale
(`gyroBridgeAfterMs`) — otherwise it would just add jitter. This rides out a
transient magnetic disturbance on the gyro until the next trustworthy compass
sample. `gyroSign` maps device-frame rotation to our bearing convention.

**Gyro-compass fusion (windowed).** The gyro is the short-term rotation
reference, so a sustained mismatch between how far the *compass* moved and how
far the *gyro* says the device rotated is a magnetic disturbance — the compass
*offset* changed, not the device. `HeadingTracker` anchors a window at a compass
sample and compares the accumulated compass movement against the accumulated
gyro rotation. When the mismatch reaches `compassJumpDeg` (default **5°** —
indoors a compass drifts just from the metal in a building, so the gate is
deliberately low) it is folded into a learned `autoOffset` (gain
`gyroFuseGain`, clamped per step) and the window re-anchors. The window
(`compassWindowMs`, 6 s) is what catches *slow* drift: the accumulated drift
grows with the window while the two-sample magnetometer noise does not, so it
works down to roughly `compassJumpDeg / compassWindowMs` ≈ 1°/s. A real turn the
gyro corroborates, a small nudge, or a gyro gap longer than `gyroSilenceMs`
(which re-anchors without absorbing) all leave the offset untouched; a transient
glitch self-cancels. **Real turns are not broken:** the window spans the turn, so
a lagging compass (measured up to 300 ms) cancels out — validated by walking
routes taken from real floor plans. The one sensitivity is a *systematic gyro
scale error*: a turn genuinely under/over-reported by the gyro leaves a residual
≈ scale_error × turn_angle, which leaks into the offset when turns repeat in one
direction (≈2% error → a few degrees; ≈10% → ~18° over two 90° turns). Phone gyro
scale error after OS calibration is typically well under that.
Because the gain is <1 a disturbed step leaves a small residual, and the window
never integrates gyro bias for longer than `compassWindowMs` (0.1°/s × 6 s ≈
0.6°). The `autoOffset` is session-only; turning does not discard it (only the
hallway lock's arming state resets).

### Step length (`StepLengthModel`)

One unknown per person: the base step length at a reference cadence. The model
scales it roughly proportionally to cadence and clamps it — deliberately simple
and ML-free, aimed at the 4–5 m accuracy target. The base is what calibration
(walk a known line, GPS, or height estimate) tunes.

## `createMotion` and sensor permissions

`createMotion(opts)` wires `devicemotion`/`deviceorientation` listeners to the
classes and returns `{ start, stop, detector, heading, support, ... }`. `start()`
is async and handles the iOS user-gesture permission.

**The permission gotcha (fixed).** iOS exposes `DeviceMotionEvent.requestPermission`
and `DeviceOrientationEvent.requestPermission`. The original code awaited the
first and then called the second — but awaiting the first spends the user
activation, so the second rejects, and the old `catch` returned **without
registering any listeners** ("the user granted permission but the walking model
does nothing"). The fix: make **one** request (prefer the accelerometer — the
single "Motion & Orientation Access" prompt enables both), and guard against
concurrent `start()` calls so a second call joins the in-flight request instead
of re-prompting.

## Implementation notes / invariants

- Units: accelerations `m/s²` including gravity; headings **radians clockwise
  from north**; timestamps ms.
- `StepDetector.push({t, ax, ay, az})` returns an array of step timestamps
  detected by that sample (usually empty or one; the confirmation moment can
  return several buffered ones). `steps` is the running total.
- `StepLengthModel.lengthFor(cadenceHz)` returns `base` when cadence is unknown,
  and always clamps to `[min, max]`.
- `WalkCalibrator` is the one-shot "walk a known line" helper.

## See also

- [`locate.md`](locate.md) — consumes steps + heading.
- [`autocal.md`](autocal.md) — tunes the step-length base from GPS.
- [`apps.md`](apps.md) — `onStep`/`onHeading` wiring.
