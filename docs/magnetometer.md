# `magnetometer.mjs` — magnetic disturbance + phone iron calibration

## What it does

Two pure jobs that keep the compass from lying, extracted from `motion.mjs`
because they are independently testable and platform-independent:

1. **Field monitoring** (`MagneticFieldTracker`) — is the compass reading
   trustworthy right now? Walking past a steel doorframe, a rebar column or an
   elevator motor deforms the local field before it corrupts the heading.
2. **Phone self-calibration and heading** (`MagnetometerCalibration`,
   `tiltCompensatedHeading`) — remove the phone's own hard/soft iron and turn a
   corrected 3-axis field plus gravity into a bearing in any orientation.

Everything is in microtesla (µT). Earth's total field is ~25–65 µT
(~50 µT in Southcentral Alaska); a phone's own offset can be a large fraction
of that.

Exports: `MagneticFieldTracker`, `MagnetometerCalibration`,
`tiltCompensatedHeading`, `magnitude`.

## Theory & why

### Field monitoring

`MagneticFieldTracker.push({x,y,z}, up?)` keeps a median of the last ~2 s of
field magnitude and flags `disturbed` when:

- `|B| < minUT` (20 µT, `weak`) or `|B| > maxUT` (80 µT, `strong`) — you are on
  top of steel / the reading is broken; or
- `|B|` deviates from its own recent **median** by more than `deviationFrac`
  (15%, `offset`); or
- two consecutive samples jump more than `maxRateUT` (12 µT, `spike`).

The median (not the mean) keeps one bad reading from moving the reference, and
a long gradual distortion does not permanently desensitise the detector. When
`up` is supplied, `dipDeg` (inclination) is also reported.

This is the **fast** disturbance signal. The slower, more general signal is the
compass-vs-gyro consistency gate in `motion.mjs` (`HeadingTracker`). The two
compose: the field monitor trips instantly on a magnitude change; the rate gate
catches a purely directional offset that leaves the magnitude intact.

### The honest limitation

A *locally consistent* distortion — a corridor whose rebar bends the field 15°
along its whole length — produces no change over time. The rate gate sees
nothing and a one-time iron fit cannot remove a field that varies with position.
No phone fusion fixes that: the floor plan (heading field) and GPS course remain
the only absolute witnesses. The plan and docs say so explicitly.

### Iron calibration (`MagnetometerCalibration`)

The magnetometer's error is `raw = S · R · B_earth + h`, where `h` is the
hard-iron offset and `S` the soft-iron distortion. A figure-8/tumble sweeps
`R · B_earth` over a sphere. From the per-axis extremes we recover:

- **hard iron** = sphere centre = `(min+max)/2` per axis;
- **soft iron** = axis scale ratios = `meanRadius / halfRange` per axis
  (clamped), approximating the ellipsoid as axis-aligned.

`fit()` refuses `ok` unless the sample count and per-axis **coverage** (each
half-range at least `minCoverage` of the target radius) are sufficient and the
post-correction radius **residual** is small. A half-hearted wave must never
silently install a bogus calibration; `toJSON()` returns null when the fit is
not `ok`.

Soft iron is deliberately the diagonal approximation, not a full 9-parameter
ellipsoid: it removes the dominant distortion, cannot easily overfit, and has
no dependencies. `correct()` applies the fit (or identity).

The fit is per **device + case + hold** (hand vs pocket vs watch) and can go
stale mid-session, so treat it as a session-refreshable prior, never truth.

### Tilt-compensated heading (`tiltCompensatedHeading`)

Builds an east/north basis from the corrected field and the gravity ("up")
vector, projects the device's **top edge** onto the horizontal plane, and takes
its bearing:

```
east  = normalize(mag × up)
north = normalize(up × east)
top   = (0,1,0) - ((0,1,0)·up) up
heading = atan2(top·east, top·north)
```

The result is relative to **magnetic** north; add declination (`+15.4°` at
Anchorage, WMM2025 — exported as `DECLINATION_E_DEG` in `motion.mjs`). Returns
null when the attitude is unusable (field parallel to up, or the phone pointing
straight up/down), so callers keep the previous heading instead of publishing
nonsense.

## Implementation notes / invariants

- Device frame: x right, y top, z out of the screen. Accelerometer at rest reads
  up (specific force), so the gravity vector is the "up" vector.
- Rotating the phone about the vertical never changes yaw rate, so the gyro yaw
  rotation is screen-orientation independent (see `worldYawRate` in
  `motion.mjs`).
- `MagnetometerCalibration.add()` is a no-op unless `start()` was called; call
  `start()` from the user gesture that begins the figure-8.
- The `Magnetometer` / `UncalibratedMagnetometer` Web APIs are **flag-gated** in
  current Chrome (`chrome://flags/#enable-generic-sensor-extra-classes`). Treat the
  whole raw-magnetometer path as an enhancement; the default is the OS-fused
  `deviceorientationabsolute` / iOS compass heading.

## Smoke coverage

`node smoke.mjs magnetometer` exercises the field tracker (strong/weak
thresholds), a synthetic full-sphere iron fit (hard and soft iron recovered,
insufficient coverage refused), tilt compensation (flat, pitched, declination,
degenerate) and the `createMotion` raw-mag wiring against a fake
`Magnetometer`.

## See also

- [`motion.md`](motion.md) — the fusion core that consumes these signals.
- [`headingfield.md`](headingfield.md) — the magnetic-free direction reference.
- [`locate.md`](locate.md) — heading uncertainty in the particle filter.
