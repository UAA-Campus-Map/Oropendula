# `motion.mjs` — sensors → steps, heading and step length

## What it does

Turns raw phone sensors into the two things dead reckoning needs — **when a step
happened** and **which way the walker is facing** — plus a step-length model.
It is pure, testable code; the browser wiring is one small function.

Exports: `StepDetector`, `HeadingTracker`, `StepLengthModel`, `WalkCalibrator`,
`sensorSupport`, `headingFromOrientation`, `createMotion`, `worldYawRate`,
`DECLINATION_E_DEG`, and the angle helpers `wrapAngle`, `shortestDelta`,
`clamp`.

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
carries it between samples **and through magnetic disturbances**. Since the
fusion work this is a complementary filter, not the old "bridge only while the
compass is stale" rule:

- **Continuous gyro integration.** Every gyro sample advances the fused heading
  (and a shadow prediction) by the bias-corrected yaw rate. `worldYawRate`
  rotates DeviceMotion's device-frame rate into the world frame using the
  attitude (`beta`/`gamma`) — integrating `rotationRate.alpha` alone is only
  correct when the phone is flat.
- **Consistency gate.** A compass sample is accepted only when it agrees with
  the gyro prediction within `compassTol` plus a turn-lag budget. The budget
  uses a **recent-peak turn rate** (`_rateLag`, decaying over `rateLagTauS`),
  not the instantaneous rate: after a fast turn the gyro reads ~0 while the
  low-passed compass is still catching up, and that catch-up must not look like
  magnetic interference. A mismatch that persists drops the compass out of
  trust (`state: 'coasting'`); the heading then runs on the bias-corrected gyro
  until the compass agrees for `reacquireSamples` / `reacquireS`. Gating on
  **rotation consistency**, never absolute offset, keeps a real turn from being
  mistaken for drift (the old gyro-correction idea's failure).
- **Gyro bias** is learned on a long baseline while the compass is trusted, the
  walker is moving and not turning, and the phone is not idle. A set-down phone
  or an elevator motor must not teach the gyro a fake rotation.
- **Degradation ladder.** `headingSigmaRad` widens with time-since-trust and is
  fed to the particle filter; after `maxUntrustedS` a doubtful absolute compass
  is re-accepted (a drifting gyro-only heading is worse). The app additionally
  re-biases from GPS course outdoors and the heading field indoors.
- **Magnetic disturbance.** `createMotion` can feed
  `HeadingTracker.setMagneticDisturbance()` from `magnetometer.mjs`
  (`MagneticFieldTracker`), the fast magnitude-based signal. The two gates
  compose.
- **No gyro / `fusion: false`**: the tracker is exactly the old compass-only
  smoother, so Firefox/desktop and the legacy tests are unaffected.

Compass samples are smoothed **circularly** (sin/cos accumulators, so the value
never jumps across the ±π seam) with a time-based low-pass. `rawHeading` is the
same smoothing before bias/trim. It exposes both so the readout can show the
**raw** and **corrected** headings side by side.

The gate only sees *changes*: a locally consistent distortion (a corridor whose
rebar bends the field uniformly) is invisible to it, and a one-time iron fit
cannot remove a field that varies with position. The floor plan and GPS course
remain the absolute witnesses. See [`magnetometer.md`](magnetometer.md).

### Frames and declination

All headings are **true-north** radians clockwise from north. Raw magnetometer
headings are magnetic and add `DECLINATION_E_DEG` (+15.4° at Anchorage,
WMM2025). iOS `webkitCompassHeading` depends on Location Services (true when
on); `deviceorientationabsolute` is OS-fused. Frames are **never mixed**:
`createMotion` uses one primary source — `raw-mag` (opportunistic Generic
Sensor magnetometer), else `os-fused`, else the calibrated `relative` frame.

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

The calibration UI now requests sensors when the user **chooses a calibration
method**, not when the modal is merely opened. That keeps the request inside the
method-choice tap, which is the user gesture iOS requires.

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
