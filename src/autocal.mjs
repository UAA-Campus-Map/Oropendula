/**
 * Continuous GPS step-length auto-calibration.
 *
 * The walking model has one unknown per person: the base step length at the
 * model's reference cadence. While GPS is good and the user is actually
 * walking, that unknown is observable, and this module watches for it
 * whenever it happens - no dedicated calibration walk needed. Three signals
 * are used together:
 *
 *   1. GPS speed over step rate. Distance is the trapezoid integral of
 *      Doppler speed between the FIRST and LAST counted step of the stretch,
 *      divided by the steps walked in exactly that time (N-1). Doppler speed
 *      is far quieter than position, and anchoring the integral to step times
 *      removes the partial-step boundary error that would otherwise bias
 *      short stretches by up to 1/N.
 *   2. GPS position over steps. Straight-line displacement between two good
 *      fixes, divided by the steps counted between them. Noisier, but it is
 *      an independent channel: it cross-checks the speed estimate and covers
 *      devices that report no speed at all.
 *   3. Step rate, twice over. The cadence measured from the step intervals
 *      normalises each sample back to the model's reference cadence - the
 *      same scaling StepLengthModel applies - so every sample estimates the
 *      same unknown regardless of how fast the user happened to walk.
 *
 * A usable stretch (a "segment": 5-60 s, good fixes throughout, straight,
 * no pauses *inside*) yields one (base, sigma) sample. Turns are fine as long
 * as they happen during a pause: the pause closes the leg, so a back-and-forth
 * walk through a constrained indoor space calibrates leg by leg instead of
 * being poisoned by its U-turn courses. Fixes up to minFixAccuracyM (15 m)
 * are accepted; beyond fineAccuracyM (10 m) their speed and position
 * uncertainties scale with the reported accuracy, so a coarse-GPS walk yields
 * wider samples and simply has to walk longer before the 90% / +-6% gate
 * opens - the claim is governed by confidence, never by a hard cutoff.
 * Samples are combined inverse-variance
 * weighted with an irreducible systematic floor: a GPS speed scale bias or a
 * cadence-model mismatch will not average away, so the module never claims
 * more precision than that floor allows.
 *
 * The "calibrated" claim is withheld until the numbers support it: the
 * estimate must be within +-6% of the truth at 90% confidence, i.e.
 * 2*Phi(0.06/sigma_rel) - 1 >= 0.90, AND backed by enough data (steps,
 * metres, segments) AND consistent between the early and late halves of the
 * record. The caller must apply nothing until `stats().calibrated` is true.
 */

import { wrapAngle } from './motion.mjs';
import { metresPerDegree } from './geo.mjs';

const DEG = Math.PI / 180;

/** Z for a two-sided 90% interval. */
export const Z90 = 1.6448536269514722;

/** erf via Abramowitz & Stegun 7.1.26 (|error| < 1.5e-7). */
export function erf(x) {
  const sign = x < 0 ? -1 : 1;
  const ax = Math.abs(x);
  const t = 1 / (1 + 0.3275911 * ax);
  const poly = ((((1.061405429 * t - 1.453152027) * t + 1.421413741) * t
    - 0.284496736) * t + 0.254829592) * t;
  return sign * (1 - poly * Math.exp(-ax * ax));
}

/** Standard normal CDF. */
export function normCdf(x) {
  return 0.5 * (1 + erf(x / Math.SQRT2));
}

/**
 * P(|estimate - truth| <= toleranceRel * truth | estimate ~ N(truth,
 * sigmaRel^2)). This is the number the UI shows and the gate the claim hangs
 * on: 0.90 at sigmaRel = toleranceRel / Z90.
 */
export function calibrationConfidence(toleranceRel, sigmaRel) {
  if (!(sigmaRel > 0)) return 1;
  return Math.max(0, Math.min(1, 2 * normCdf(toleranceRel / sigmaRel) - 1));
}

/**
 * @param {object} [opts]
 * @param {number} [opts.cadenceRef=1.8]           model reference cadence (Hz)
 * @param {number} [opts.cadenceGain=0.35]         model cadence gain (StepLengthModel)
 * @param {number} [opts.minFixAccuracyM=15]       fixes worse than this are unusable
 * @param {number} [opts.fineAccuracyM=10]         at/below this, nominal sigmas; above it
 *                                                 (up to minFixAccuracyM) speed uncertainties
 *                                                 scale with accuracy, so coarse fixes just
 *                                                 need proportionally more walking
 * @param {number} [opts.segMinS=5]                shortest segment
 * @param {number} [opts.segMaxS=60]               longest segment
 * @param {number} [opts.segTargetS=30]            preferred segment length
 * @param {number} [opts.segMinSteps=10]           steps required in a segment
 * @param {number} [opts.segMinM=6]                metres required in a segment
 * @param {number} [opts.segMaxGapS=1.5]           longest internal step gap
 * @param {number} [opts.segEdgeGapS=2.0]          longest gap from a fix to a step
 * @param {number} [opts.maxNetTurnDeg=40]         net course change allowed
 * @param {number} [opts.straightnessR=0.75]       circular concentration of the course
 *                                                 (rejects curves that wrap past 180 deg)
 * @param {number} [opts.speedLoMps=0.45]          walking floor (m/s)
 * @param {number} [opts.speedHiMps=2.4]           walking ceiling (m/s)
 * @param {number} [opts.minSpeedCoverage=0.7]     fraction of fixes in speed range
 * @param {number} [opts.maxFixGapS=6]             longest gap between good fixes
 *                                                 inside a segment (raw GPS must be
 *                                                 continuously present)
 * @param {number} [opts.sigmaSpeedMps=0.2]        Doppler speed noise (1 sigma)
 * @param {number} [opts.posSigmaFactor=0.7]       common-mode discount on fix accuracy
 * @param {number} [opts.strideSigmaRel=0.005]     per-segment stride variability floor
 * @param {number} [opts.systematicSigmaRel=0.015] irreducible scale uncertainty
 * @param {number} [opts.toleranceRel=0.06]        the +-6% promise
 * @param {number} [opts.requiredConfidence=0.9]   the 90% promise
 * @param {number} [opts.minClaimSteps=20]         evidence gates for a claim
 * @param {number} [opts.minClaimM=10]
 * @param {number} [opts.minClaimSegments=2]
 * @param {number} [opts.minStepLenM=0.3]          plausibility bounds
 * @param {number} [opts.maxStepLenM=1.5]
 * @param {number} [opts.maxSegments=80]           rolling history cap
 * @param {number} [opts.historyS=120]             raw event history (s)
 */
export class GpsStepCalibrator {
  constructor(opts = {}) {
    Object.assign(this, {
      cadenceRef: 1.8, cadenceGain: 0.35,
      minFixAccuracyM: 15, fineAccuracyM: 10,
      segMinS: 5, segMaxS: 60, segTargetS: 30,
      segMinSteps: 10, segMinM: 6,
      segMaxGapS: 1.5, segEdgeGapS: 2.0,
      maxNetTurnDeg: 40, straightnessR: 0.75,
      speedLoMps: 0.45, speedHiMps: 2.4, minSpeedCoverage: 0.7, maxFixGapS: 6,
      sigmaSpeedMps: 0.2, posSigmaFactor: 0.7,
      strideSigmaRel: 0.005, systematicSigmaRel: 0.015,
      toleranceRel: 0.06, requiredConfidence: 0.9,
      minClaimSteps: 20, minClaimM: 10, minClaimSegments: 2,
      minStepLenM: 0.3, maxStepLenM: 1.5,
      maxSegments: 80, historyS: 120,
    }, opts);
    this.reset();
  }

  reset() {
    this.fixes = [];            // {t, lon, lat, accuracyM, speedMps, courseRad}
    this.steps = [];            // step timestamps (ms)
    this.samples = [];          // accepted {t, baseM, sigmaM, ...}
    this.goodFixes = 0;         // fixes good enough to calibrate from
    this.lastSegEndT = -Infinity;
    // Live-progress state (surfaced through stats().live).
    this._lastT = null;         // newest event time (step or fix)
    this._lastFix = null;       // newest fix, even a poor one
    this._lastStepT = null;
    this._lastReject = null;    // reason code of the last failed evaluation
    this.stretchSteps = 0;      // steps since the current leg/sample started
    this.stretchT0 = null;
    this.evaluated = 0;         // candidate evaluations (all reasons)
    this.lastAttempt = null;    // {ok, baseM, reason, t} of the newest closed attempt
  }

  /** One detected step. `t` must share the clock used for fixes. */
  addStep({ t } = {}) {
    if (!(t != null)) return;
    this._prune(t);
    const last = this.steps.length ? this.steps[this.steps.length - 1] : null;
    // A gap longer than segMaxGapS means the walker stopped; if walking
    // resumes, the leg that just ended is complete evidence, so close it now.
    if (last != null && t - last > this.segMaxGapS * 1000) {
      this._closeAtPause(last);
      this.stretchSteps = 0;
      this.stretchT0 = null;
    }
    if (this.stretchSteps === 0) this.stretchT0 = t;
    this.stretchSteps++;
    this._lastStepT = t;
    this._lastT = this._lastT == null ? t : Math.max(this._lastT, t);
    this.steps.push(t);
  }

  /**
   * One geolocation fix.
   * @param {{t:number, lon:number, lat:number, accuracyM:number,
   *          speedMps?:number|null, courseDeg?:number|null}} fix
   */
  addFix(fix) {
    const { t, lon, lat, accuracyM } = fix ?? {};
    if (!(t != null) || !isFinite(lon) || !isFinite(lat)) return;
    this._prune(t);
    const f = {
      t, lon, lat, accuracyM,
      speedMps: Number.isFinite(fix.speedMps) ? fix.speedMps : null,
      courseRad: Number.isFinite(fix.courseDeg) ? wrapAngle(fix.courseDeg * DEG) : null,
    };
    this._lastFix = f;
    this._lastT = this._lastT == null ? t : Math.max(this._lastT, t);
    if (f.accuracyM <= this.minFixAccuracyM) this.goodFixes++;
    this.fixes.push(f);
    this._trySegment(f);
  }

  /** The live-progress view of the calibrator (see stats().live). */
  _live() {
    const lastT = this._lastT;
    const walking = lastT != null && this._lastStepT != null
      && lastT - this._lastStepT <= this.segMaxGapS * 1000;
    const stretchS = this.stretchT0 != null && this._lastStepT != null
      ? Math.max(0, (this._lastStepT - this.stretchT0) / 1000) : 0;
    const la = this.lastAttempt;
    return {
      walking,
      fixAccuracyM: this._lastFix ? this._lastFix.accuracyM : null,
      fixSpeedMps: this._lastFix ? this._lastFix.speedMps : null,
      stretch: { steps: this.stretchSteps, seconds: stretchS },
      evaluated: this.evaluated,
      lastAttempt: la ? {
        ok: !!la.ok,
        baseM: la.baseM ?? null,
        reason: la.reason ?? null,
        ageS: lastT != null ? Math.max(0, (lastT - la.t) / 1000) : null,
      } : null,
    };
  }

  /**
   * Current calibration state. Shape is stable even before any data:
   * {status, baseM, sigmaM, ci90RelPct, confidencePct, calibrated,
   *  missing: [gate keys], deficit: {steps, distanceM, samples},
   *  live: {walking, fixAccuracyM, fixSpeedMps, stretch: {steps, seconds},
   *         evaluated, lastAttempt: {ok, baseM, reason, ageS}|null},
   *  samples: {total, speed, position, steps, distanceM, spanS, cadenceHz,
   *            speedSharePct}}
   */
  stats() {
    const empty = {
      status: this.goodFixes > 0 ? 'collecting' : 'idle',
      baseM: null, sigmaM: null, ci90RelPct: null, confidencePct: null,
      calibrated: false, missing: ['samples'],
      deficit: { steps: this.minClaimSteps, distanceM: this.minClaimM, samples: this.minClaimSegments },
      live: this._live(),
      samples: {
        total: 0, speed: 0, position: 0, steps: 0, distanceM: 0,
        spanS: 0, cadenceHz: null, speedSharePct: null,
      },
    };
    if (!this.samples.length) return empty;

    let wSum = 0, baseSum = 0, speedW = 0, posW = 0, wCad = 0, cadSum = 0;
    let steps = 0, dist = 0, speedCount = 0;
    for (const s of this.samples) {
      const w = 1 / (s.sigmaM * s.sigmaM);
      wSum += w;
      baseSum += w * s.baseM;
      speedW += w * s.speedShare;
      posW += w * (1 - s.speedShare);
      wCad += w;
      cadSum += w * s.cadenceHz;
      steps += s.steps;
      dist += s.distM;
      if (s.speedShare >= 0.5) speedCount++;
    }
    const baseM = baseSum / wSum;
    const sigmaRaw = Math.sqrt(1 / wSum);
    const sigmaM = Math.sqrt(sigmaRaw * sigmaRaw
      + (this.systematicSigmaRel * baseM) ** 2);
    const confidence = calibrationConfidence(this.toleranceRel, sigmaM / baseM);

    // Early vs late agreement: a person whose apparent step length changed
    // midway (terrain, injury, a stop for coffee) must not be averaged into a
    // confident claim - the halves would straddle the truth.
    let consistent = true;
    if (this.samples.length >= 3) {
      const sorted = [...this.samples].sort((a, b) => a.t - b.t);
      const h = sorted.length >> 1;
      const half = (arr) => {
        let a = 0, b = 0;
        for (const s of arr) { const w = 1 / (s.sigmaM * s.sigmaM); a += w; b += w * s.baseM; }
        return { m: b / a, s: Math.sqrt(1 / a) };
      };
      const first = half(sorted.slice(0, h));
      const second = half(sorted.slice(h));
      consistent = Math.abs(first.m - second.m)
        <= Z90 * Math.hypot(first.s, second.s);
    }

    const deficit = {
      steps: Math.max(0, this.minClaimSteps - steps),
      distanceM: Math.max(0, this.minClaimM - dist),
      samples: Math.max(0, this.minClaimSegments - this.samples.length),
    };
    const missing = [];
    for (const k of ['steps', 'distanceM', 'samples']) if (deficit[k] > 0) missing.push(k === 'distanceM' ? 'distance' : k);
    if (!consistent) missing.push('consistency');
    if (confidence < this.requiredConfidence) missing.push('confidence');
    const calibrated = missing.length === 0;

    return {
      status: calibrated ? 'calibrated' : 'calibrating',
      baseM, sigmaM,
      ci90RelPct: (Z90 * sigmaM / baseM) * 100,
      confidencePct: confidence * 100,
      calibrated, missing, deficit,
      live: this._live(),
      samples: {
        total: this.samples.length,
        speed: speedCount,
        position: this.samples.length - speedCount,
        steps, distanceM: dist,
        spanS: (this.samples[this.samples.length - 1].t - this.samples[0].t) / 1000,
        cadenceHz: cadSum / wCad,
        speedSharePct: (speedW / (speedW + posW)) * 100,
      },
    };
  }

  _prune(t) {
    const lo = t - this.historyS * 1000;
    while (this.fixes.length && this.fixes[0].t < lo) this.fixes.shift();
    while (this.steps.length && this.steps[0] < lo) this.steps.shift();
  }

  /**
   * Close the leg that ended in a pause, using the last fix before the last
   * step as the endpoint. Waiting for a longer window is pointless here - the
   * leg is over - so the deferral rule is bypassed: this is what lets short
   * indoor legs (walk, pause, turn around, walk back) calibrate.
   */
  _closeAtPause(tLastStep) {
    for (let i = this.fixes.length - 1; i >= 0; i--) {
      const f = this.fixes[i];
      if (f.t > tLastStep || f.accuracyM > this.minFixAccuracyM) continue;
      this._trySegment(f, true);
      return;
    }
  }

  /**
   * Try to close a segment at this fix, chained after the last accepted one.
   *
   * Candidates are tried in order of closeness to the target duration. A
   * candidate shorter than half the target is only considered once the chain
   * has waited a full target interval - that deferral is what lets a straight
   * stretch *after* a corner or pause become a longer (and far more precise)
   * sample instead of closing the first barely-legal 5 s window; the escape
   * hatch keeps corner recovery from stalling forever. `force` (a detected
   * pause) skips the deferral, because the leg is known to be finished.
   */
  _trySegment(end, force = false) {
    if (end.accuracyM > this.minFixAccuracyM) return;
    const lo = Math.max(end.t - this.segMaxS * 1000, this.lastSegEndT + 1000);
    const hi = end.t - this.segMinS * 1000;
    const targetMs = this.segTargetS * 1000;
    const candidates = this.fixes
      .filter((f) => f.t >= lo && f.t <= hi && f.accuracyM <= this.minFixAccuracyM)
      .sort((f, g) => Math.abs((end.t - f.t) - targetMs) - Math.abs((end.t - g.t) - targetMs));
    const waited = force || end.t - this.lastSegEndT >= targetMs;
    let firstReason = null, evaluated = 0;
    for (const start of candidates) {
      if (end.t - start.t < targetMs / 2 && !waited) continue;
      evaluated++;
      const sample = this._evalSegment(start, end);
      if (sample) {
        this.evaluated += evaluated;
        this.stretchSteps = 0;
        this.stretchT0 = null;
        this.lastAttempt = { ok: true, baseM: sample.baseM, t: end.t };
        this.samples.push(sample);
        if (this.samples.length > this.maxSegments) {
          this.samples.splice(0, this.samples.length - this.maxSegments);
        }
        this.lastSegEndT = end.t;
        return;
      }
      if (firstReason == null) firstReason = this._lastReject;
    }
    this.evaluated += evaluated;
    // Record why the preferred candidate failed, for the live panel.
    if (evaluated > 0) this.lastAttempt = { ok: false, reason: firstReason, t: end.t };
  }

  /** Record why a candidate failed (surfaced in stats().live) and reject it. */
  _reject(reason) {
    this._lastReject = reason;
    return null;
  }

  /** Evaluate one candidate segment; null if it is not usable evidence. */
  _evalSegment(a, b) {
    const dtS = (b.t - a.t) / 1000;
    if (dtS < this.segMinS || dtS > this.segMaxS) return this._reject('span');

    // Steps inside the window, with no pause anywhere (a stop makes the
    // speed integral and the step count describe different walks).
    const steps = this.steps.filter((t) => t > a.t && t <= b.t);
    if (steps.length < this.segMinSteps) return this._reject('steps');
    const maxGapMs = this.segMaxGapS * 1000, edgeGapMs = this.segEdgeGapS * 1000;
    if (steps[0] - a.t > edgeGapMs || b.t - steps[steps.length - 1] > edgeGapMs) {
      return this._reject('gap');
    }
    for (let i = 1; i < steps.length; i++) {
      if (steps[i] - steps[i - 1] > maxGapMs) return this._reject('gap');
    }

    // Cadence from the median step interval (robust to one jittery pair).
    const intervals = [];
    for (let i = 1; i < steps.length; i++) intervals.push(steps[i] - steps[i - 1]);
    intervals.sort((x, y) => x - y);
    const cadenceHz = 1000 / intervals[intervals.length >> 1];
    if (cadenceHz < 0.5 || cadenceHz > 3.5) return this._reject('cadence');

    const good = this.fixes.filter((f) =>
      f.t >= a.t && f.t <= b.t && f.accuracyM <= this.minFixAccuracyM);

    // Raw GPS must be CONTINUOUSLY present across the window. Without this,
    // two good fixes straddling a dropout (indoors, where the arbiter is on
    // PDR) would form a position-only chord while the steps walked a quite
    // different path - exactly the kind of stitched evidence that must never
    // enter the estimate. Only GPS witnesses the geometry here; the PDR
    // estimate is never an input.
    if (good.length < 2) return this._reject('coverage');
    if ((good[good.length - 1].t - good[0].t) / 1000 < 0.8 * dtS) return this._reject('coverage');
    for (let i = 1; i < good.length; i++) {
      if (good[i].t - good[i - 1].t > this.maxFixGapS * 1000) return this._reject('coverage');
    }

    // Straightness: a curved stretch makes displacement and steps describe
    // different distances. The endpoint check alone is blind to curves that
    // wrap past 180 deg within the window, so the circular concentration R of
    // the course samples is checked too (a uniform spread of directions has
    // R ~ 0). Missing course data (rare) is tolerated.
    const courses = good.filter((f) => f.courseRad != null).map((f) => f.courseRad);
    if (courses.length >= 2) {
      if (Math.abs(wrapAngle(courses[courses.length - 1] - courses[0])) > this.maxNetTurnDeg * DEG) {
        return this._reject('turn');
      }
      let sx = 0, sy = 0;
      for (const c of courses) { sx += Math.cos(c); sy += Math.sin(c); }
      if (Math.hypot(sx, sy) / courses.length < this.straightnessR) return this._reject('turn');
    }

    // Speed coverage: a segment spent standing at lights with a stale speed
    // reading must not masquerade as walking.
    const withSpeed = good.filter((f) => f.speedMps != null);
    if (withSpeed.length >= 2) {
      const inRange = withSpeed.filter((f) =>
        f.speedMps >= this.speedLoMps && f.speedMps <= this.speedHiMps).length;
      if (inRange / withSpeed.length < this.minSpeedCoverage) return this._reject('coverage');
    }

    // Method 1: Doppler speed integrated between the first and last step
    // (exact for constant speed, immune to partial steps at the window edges).
    const t1 = steps[0], tN = steps[steps.length - 1];
    const tSpanS = (tN - t1) / 1000;
    let speedDist = null, speedSigma = null;
    {
      let sum = 0, span = 0, varSum = 0, pairs = 0;
      for (let i = 1; i < good.length; i++) {
        const f0 = good[i - 1], f1 = good[i];
        if (f0.speedMps == null || f1.speedMps == null) continue;
        const s0 = Math.max(f0.t, t1), s1 = Math.min(f1.t, tN);
        if (s1 <= s0) continue;
        // Walking speed for the interval. At a leg boundary one endpoint can
        // still report a decaying (or ramping) speed from the pause; trusting
        // the walking endpoint there avoids biasing the distance low, which
        // averaging it with the stationary endpoint would do.
        const ok0 = f0.speedMps >= this.speedLoMps && f0.speedMps <= this.speedHiMps;
        const ok1 = f1.speedMps >= this.speedLoMps && f1.speedMps <= this.speedHiMps;
        if (!ok0 && !ok1) continue;
        const v = ok0 && ok1 ? 0.5 * (f0.speedMps + f1.speedMps)
          : (ok0 ? f0.speedMps : f1.speedMps);
        // Coarse fixes carry coarser speed (at worst it is position-derived):
        // inflate this interval's sigma with the worse endpoint accuracy, so
        // the confidence gate - not a hard accuracy cutoff - decides how much
        // extra walking the data needs.
        const vs = this.sigmaSpeedMps
          * Math.max(1, Math.max(f0.accuracyM, f1.accuracyM) / this.fineAccuracyM);
        const dt = (s1 - s0) / 1000;
        sum += v * dt;
        span += dt;
        varSum += (vs * dt) ** 2;
        pairs++;
      }
      if (pairs >= 3 && span >= 0.8 * tSpanS) {
        speedDist = sum;
        speedSigma = Math.sqrt(varSum);
      }
    }

    // Method 2: displacement between the endpoint fixes over the steps counted
    // in the window.
    const mpd = metresPerDegree((a.lat + b.lat) / 2);
    const dE = (b.lon - a.lon) * mpd.lon;
    const dN = (b.lat - a.lat) * mpd.lat;
    const posDist = Math.hypot(dE, dN);
    const posSigma = this.posSigmaFactor * Math.hypot(a.accuracyM, b.accuracyM);

    if (speedDist == null
        && (posDist / dtS < this.speedLoMps || posDist / dtS > this.speedHiMps)) {
      return this._reject('speed');
    }
    const dist = speedDist != null ? speedDist : posDist;
    if (dist < this.segMinM) return this._reject('distance');

    // Combine the methods as step lengths (they use different step counts by
    // one), then normalise to the model's reference cadence.
    const nSteps = steps.length;
    const sigSpeed = speedDist != null ? speedSigma / (nSteps - 1) : Infinity;
    const sigPos = posSigma / nSteps;
    const ws = speedDist != null ? 1 / (sigSpeed * sigSpeed) : 0;
    const wp = 1 / (sigPos * sigPos);
    const lSpeed = speedDist != null ? speedDist / (nSteps - 1) : null;
    const lPos = posDist / nSteps;
    const stepLen = (lPos * wp + (lSpeed != null ? lSpeed * ws : 0)) / (wp + ws);
    if (stepLen < this.minStepLenM || stepLen > this.maxStepLenM) return this._reject('plausibility');

    const factor = 1 + (this.cadenceGain * (cadenceHz - this.cadenceRef)) / this.cadenceRef;
    const baseM = stepLen / factor;
    const sigmaL = Math.sqrt(1 / (wp + ws));
    const sigmaM = Math.sqrt((sigmaL / factor) ** 2 + (this.strideSigmaRel * baseM) ** 2);

    return {
      t: b.t, baseM, sigmaM,
      distM: dist,
      steps: nSteps,
      dtS, cadenceHz,
      speedShare: (speedDist != null ? ws : 0) / (ws + wp),
    };
  }
}
