/**
 * Opportunistic step-length refinement from pairs of manual position anchors.
 *
 * The user occasionally marks a near-exact position ("I'm Here"). Two marks A
 * and B, plus the steps and headings taken in between, give one step-length
 * observation *provided the walk was straight*: the walked distance is then the
 * straight span |AB|, so with the model's cadence scaling f(),
 *
 *     base step length = |AB| / sum_i f(cadence_i)
 *
 * This is PHASE 1: only straight legs are used. A leg is rejected when it turns
 * or drifts (low heading concentration), changes floor/building, pauses
 * mid-leg, points away from B, or has a straight span that crosses a wall. Taps
 * are treated as +/-0.5 m, so the endpoint uncertainty dominates short legs.
 *
 * Samples are combined inverse-variance (plus an irreducible systematic floor)
 * and the estimate is only reported as `calibrated` - within +-6% at 90%
 * confidence, backed by enough legs/steps/distance and consistent between the
 * early and late halves - mirroring autocal.mjs. The caller must apply nothing
 * until `stats().calibrated` is true.
 *
 * Turning legs are deliberately out of scope: the reconstructed path is only
 * defined up to a single scale, so the endpoint alone cannot identify the step
 * length when the measured headings drift (that needs floor-plan map matching,
 * the harder phase-2 problem).
 */

import { metresPerDegree } from './geo.mjs';
import { shortestDelta } from './motion.mjs';
import { calibrationConfidence } from './autocal.mjs';

const DEG = Math.PI / 180;
// Z for a two-sided 90% interval. Kept local rather than imported from
// autocal.mjs, so this module only relies on that module's stable export
// (`calibrationConfidence`) - some deployed autocal.mjs revisions do not
// export Z90.
const Z90 = 1.6448536269514722;

/**
 * @param {object} [opts]
 * @param {number} [opts.cadenceRef=1.8]        model reference cadence (StepLengthModel)
 * @param {number} [opts.cadenceGain=0.35]      model cadence gain
 * @param {number} [opts.anchorSigmaM=0.5]      per-tap position accuracy (metres)
 * @param {number} [opts.segMinSteps=10]        steps required in a leg
 * @param {number} [opts.segMinM=6]             straight span required in a leg
 * @param {number} [opts.segMaxGapS=1.5]        longest internal step gap
 * @param {number} [opts.straightnessR=0.97]    circular concentration of headings
 * @param {number} [opts.maxNetTurnDeg=12]      net heading change over a leg
 * @param {number} [opts.maxHeadingErrDeg=25]   mean heading vs the A->B bearing
 * @param {number} [opts.strideSigmaRel=0.01]   per-leg stride variability
 * @param {number} [opts.systematicSigmaRel=0.015] irreducible scale uncertainty
 * @param {number} [opts.toleranceRel=0.06]     the +-6% promise
 * @param {number} [opts.requiredConfidence=0.9] the 90% promise
 * @param {number} [opts.minClaimSegments=2]    evidence gates for a claim
 * @param {number} [opts.minClaimSteps=20]
 * @param {number} [opts.minClaimM=10]
 * @param {number} [opts.minStepLenM=0.3]       plausibility bounds
 * @param {number} [opts.maxStepLenM=1.5]
 * @param {number} [opts.maxSegments=80]        rolling sample history cap
 */
export class AnchorStepCalibrator {
  constructor(opts = {}) {
    Object.assign(this, {
      cadenceRef: 1.8, cadenceGain: 0.35,
      anchorSigmaM: 0.5,
      segMinSteps: 10, segMinM: 6, segMaxGapS: 1.5,
      straightnessR: 0.97, maxNetTurnDeg: 12, maxHeadingErrDeg: 25,
      strideSigmaRel: 0.01, systematicSigmaRel: 0.015,
      toleranceRel: 0.06, requiredConfidence: 0.9,
      minClaimSegments: 2, minClaimSteps: 20, minClaimM: 10,
      minStepLenM: 0.3, maxStepLenM: 1.5,
      maxSegments: 80,
    }, opts);
    this.reset();
  }

  reset() {
    this.enabled = false;
    this.lastAnchor = null;   // {t, ll, buildingId, floorId}
    this.steps = [];          // {t, headingRad, cadenceHz} since lastAnchor
    this.samples = [];        // accepted {t, baseM, sigmaM, distM, steps}
    this.evaluated = 0;
    this.lastAttempt = null;  // {ok, baseM, reason, t}
  }

  /** Turn collection on/off. Turning off discards the in-progress leg. */
  setEnabled(on) {
    this.enabled = !!on;
    if (!this.enabled) { this.lastAnchor = null; this.steps = []; }
  }

  /**
   * One detected step between anchors. `headingRad` is the applied bearing
   * (radians clockwise from north) used to reconstruct the walk.
   */
  addStep({ t, headingRad, cadenceHz } = {}) {
    if (!this.enabled || !this.lastAnchor) return;
    if (!(t != null) || !Number.isFinite(headingRad)) return;
    this.steps.push({ t, headingRad, cadenceHz: Number.isFinite(cadenceHz) ? cadenceHz : 0 });
    if (this.steps.length > 5000) this.steps.splice(0, this.steps.length - 5000);
  }

  /**
   * A manual anchor. Closes the leg from the previous anchor (if any).
   * @param {{t:number, ll:number[], buildingId:string, floorId:string}} anchor
   * @param {(a:number[],b:number[])=>boolean} [segmentClear] straight span is walkable
   * @returns {object|null} the accepted sample, or null
   */
  addAnchor(anchor, segmentClear) {
    if (!this.enabled) return null;
    const prev = this.lastAnchor;
    this.lastAnchor = anchor;
    if (!prev) { this.steps = []; return null; }
    const sample = this._close(prev, anchor, segmentClear);
    this.steps = [];   // the leg is consumed either way
    return sample;
  }

  /** Evaluate the leg a -> b; null if it is not usable straight-walk evidence. */
  _close(a, b, segmentClear) {
    this.evaluated++;
    const reject = (reason) => { this.lastAttempt = { ok: false, reason, t: b.t }; return null; };

    if (a.buildingId !== b.buildingId || a.floorId !== b.floorId) return reject('floor');

    const steps = this.steps.filter((s) => s.t > a.t && s.t <= b.t);
    if (steps.length < this.segMinSteps) return reject('steps');

    const mpd = metresPerDegree((a.ll[1] + b.ll[1]) / 2);
    const dE = (b.ll[0] - a.ll[0]) * mpd.lon, dN = (b.ll[1] - a.ll[1]) * mpd.lat;
    const dist = Math.hypot(dE, dN);
    if (dist < this.segMinM) return reject('distance');

    // No pause inside: a stop breaks the single straight walk into two.
    for (let i = 1; i < steps.length; i++) {
      if (steps[i].t - steps[i - 1].t > this.segMaxGapS * 1000) return reject('pause');
    }

    // Heading concentration (bearings: east = sin, north = cos). A straight
    // walk keeps R near 1; a curve, a U-turn or a loop collapses it.
    let sumSin = 0, sumCos = 0;
    for (const s of steps) { sumSin += Math.sin(s.headingRad); sumCos += Math.cos(s.headingRad); }
    const R = Math.hypot(sumSin, sumCos) / steps.length;
    if (R < this.straightnessR) return reject('curve');
    const meanHeading = Math.atan2(sumSin, sumCos);

    const netTurn = Math.abs(shortestDelta(steps[0].headingRad, steps[steps.length - 1].headingRad));
    if (netTurn > this.maxNetTurnDeg * DEG) return reject('turn');

    // The walk must actually head from A toward B (not alongside then across).
    const bearingAB = Math.atan2(dE, dN);
    if (Math.abs(shortestDelta(meanHeading, bearingAB)) > this.maxHeadingErrDeg * DEG) {
      return reject('heading');
    }

    // The straight span must stay on the plan: two marks across a wall are not
    // a straight walk (the person went around), so the span is not the walked
    // distance.
    if (segmentClear && !segmentClear(a.ll, b.ll)) return reject('wall');

    // Solve base so the model's cumulative length equals the walked span.
    let sumF = 0;
    for (const s of steps) {
      const c = s.cadenceHz;
      sumF += c > 0 ? 1 + (this.cadenceGain * (c - this.cadenceRef)) / this.cadenceRef : 1;
    }
    const baseM = dist / sumF;
    if (baseM < this.minStepLenM || baseM > this.maxStepLenM) return reject('plausibility');

    // Two taps dominate the endpoint uncertainty (~sqrt(2)*0.5 m); the stride
    // term covers leg-to-leg variability.
    const sigmaDisp = Math.SQRT2 * this.anchorSigmaM;
    const sigmaM = Math.sqrt((sigmaDisp / sumF) ** 2 + (this.strideSigmaRel * baseM) ** 2);

    const sample = { t: b.t, baseM, sigmaM, distM: dist, steps: steps.length };
    this.samples.push(sample);
    if (this.samples.length > this.maxSegments) {
      this.samples.splice(0, this.samples.length - this.maxSegments);
    }
    this.lastAttempt = { ok: true, baseM, t: b.t };
    return sample;
  }

  /**
   * Current calibration state. Shape mirrors autocal.mjs's, so the two can be
   * displayed and gated the same way.
   */
  stats() {
    const empty = {
      status: this.lastAnchor ? 'collecting' : 'idle',
      baseM: null, sigmaM: null, ci90RelPct: null, confidencePct: null,
      calibrated: false, missing: ['segments'],
      sincePinSteps: this.steps.length,
      samples: { total: 0, steps: 0, distanceM: 0 },
      lastAttempt: this.lastAttempt,
    };
    if (!this.samples.length) return empty;

    let wSum = 0, baseSum = 0, steps = 0, dist = 0;
    for (const s of this.samples) {
      const w = 1 / (s.sigmaM * s.sigmaM);
      wSum += w;
      baseSum += w * s.baseM;
      steps += s.steps;
      dist += s.distM;
    }
    const baseM = baseSum / wSum;
    const sigmaRaw = Math.sqrt(1 / wSum);
    const sigmaM = Math.sqrt(sigmaRaw * sigmaRaw + (this.systematicSigmaRel * baseM) ** 2);
    const confidence = calibrationConfidence(this.toleranceRel, sigmaM / baseM);

    // Early vs late agreement: a step length that shifted midway (different
    // footwear, fatigue) must not be averaged into a confident claim.
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
      consistent = Math.abs(first.m - second.m) <= Z90 * Math.hypot(first.s, second.s);
    }

    const deficit = {
      segments: Math.max(0, this.minClaimSegments - this.samples.length),
      steps: Math.max(0, this.minClaimSteps - steps),
      distanceM: Math.max(0, this.minClaimM - dist),
    };
    const missing = [];
    for (const k of ['segments', 'steps', 'distanceM']) {
      if (deficit[k] > 0) missing.push(k === 'distanceM' ? 'distance' : k);
    }
    if (!consistent) missing.push('consistency');
    if (confidence < this.requiredConfidence) missing.push('confidence');
    const calibrated = missing.length === 0;

    return {
      status: calibrated ? 'calibrated' : 'calibrating',
      baseM, sigmaM,
      ci90RelPct: (Z90 * sigmaM / baseM) * 100,
      confidencePct: confidence * 100,
      calibrated, missing,
      sincePinSteps: this.steps.length,
      samples: { total: this.samples.length, steps, distanceM: dist },
      lastAttempt: this.lastAttempt,
    };
  }
}
