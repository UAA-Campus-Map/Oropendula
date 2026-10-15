# `autocal.mjs` — continuous GPS step-length calibration

## What it does

Learns the walker's **base step length** from ordinary walking whenever GPS is
good — no dedicated calibration walk. Exports `GpsStepCalibrator`,
`calibrationConfidence`, `normCdf`, `erf`, and `Z90`.

Inputs are **raw GPS fixes** (`addFix`) and **raw step events** (`addStep`); the
output is a certified estimate in `stats()` (`baseM`, `ci90RelPct`,
`confidencePct`, `calibrated`, plus a live view).

## Why it's built this way

The walking model has exactly one unknown per person: the step length at the
model's reference cadence. While GPS is good and the user is walking, that
unknown is observable — so watch for it opportunistically. Three signals are
combined:

1. **GPS Doppler speed over step rate.** Distance is the trapezoid integral of
   Doppler speed between the first and last counted step of a stretch, divided by
   the steps walked in exactly that time (`N−1`). Doppler speed is far quieter
   than position, and anchoring the integral to step times removes the
   partial-step boundary error that would bias short stretches by up to `1/N`.
2. **GPS position over steps.** Straight-line displacement between two good
   fixes over the steps counted between them. Noisier, but independent — it
   cross-checks the speed estimate and covers devices that report no speed.
3. **Cadence, twice over.** The measured step rate normalises each sample back to
   the model's reference cadence (the same scaling `StepLengthModel` applies), so
   every sample estimates the *same* unknown regardless of how fast the user
   walked.

**PDR must never calibrate itself.** The calibrator's inputs are raw GPS + raw
steps only — never the arbiter/PDR estimate or the model it is calibrating.

## What makes a usable stretch ("segment")

5–60 s, ≥10 steps, ≥6 m, good fixes throughout (accuracy ≤15 m), **straight**
(net course change ≤40° *and* circular concentration R ≥ 0.75 — the endpoint
check alone is blind to curves wrapping past 180°), and **pause-free** (≤1.5 s
internal step gaps). Turns are allowed **when the walker pauses**: a step gap
>1.5 s closes the leg explicitly, so a back-and-forth walk through a constrained
indoor space calibrates leg by leg instead of being poisoned by its U-turn.

**Accuracy is dynamic, not a hard cutoff.** Beyond 10 m ("fine"), speed and
position uncertainties scale with the reported accuracy, so coarse-GPS samples
are simply wider and the 90 % / ±6 % gate demands proportionally more walking
before it opens.

**Combining.** Samples combine inverse-variance weighted with a 1.5 %
irreducible systematic floor (a GPS speed scale bias does not average away). A
`calibrated` claim additionally needs ≥20 steps, ≥10 m, ≥2 segments, and
early/late agreement.

## Implementation notes

- `Z90 = 1.6448536269514722` is the 90 % normal quantile; `calibrationConfidence`
  maps a tolerance/sigma ratio to a probability via `normCdf`.
- A window must be covered by *continuous* raw GPS: a gap >6 s between good fixes,
  or good fixes spanning <80 % of the window, is rejected — a GPS dropout
  (indoors, where the arbiter is on PDR) cannot be stitched into a straight-chord
  sample across the hole.
- A repeated cached fix is ignored (the app stamps each fix with its measurement
  time; `maximumAge` can return a stale one).
- `stats()` is safe to call every fix/step; it returns a stable state object plus
  a `live` view for the UI.

## See also

- [`motion.md`](motion.md) — `StepLengthModel` provides the reference cadence.
- [`apps.md`](apps.md) — how `base` is applied and shown.
- [`locate.md`](locate.md) — the map-matching particle filter (consumes the step model).
