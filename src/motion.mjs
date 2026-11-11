/**
 * Motion sensing for pedestrian dead reckoning (PDR).
 *
 * This module turns device sensors into the two things the locator needs:
 *   - steps (with cadence), and
 *   - a heading (radians, clockwise from north).
 *
 * It is deliberately split into two halves:
 *   1. Pure signal processing (StepDetector, HeadingTracker, StepLengthModel,
 *      WalkCalibrator). No DOM, no sensors - unit-testable in Node.
 *   2. Thin browser wiring (createMotion) that normalises the two sensor APIs
 *      we can encounter and feeds the classes above.
 *
 * Sensor reality on the target platforms (Chrome Android, iOS Safari,
 * Firefox): the Generic Sensor API is Chrome-only, so DeviceMotion /
 * DeviceOrientation events are the portable path. iOS additionally requires a
 * user-gesture permission request and reports true north via
 * `webkitCompassHeading`; other platforms report `alpha`, which we convert to
 * the same clockwise-from-north convention and correct with a calibrated bias.
 *
 * Angle convention: all headings are radians clockwise from north (a bearing),
 * wrapped to (-PI, PI]. Bearing increases when the user turns right.
 */

export const TWO_PI = Math.PI * 2;
const DEG = Math.PI / 180;

import { MagneticFieldTracker, MagnetometerCalibration, tiltCompensatedHeading } from './magnetometer.mjs';


/** Wrap an angle to [-PI, PI]. */
export function wrapAngle(a) {
  a = a % TWO_PI;
  if (a > Math.PI) a -= TWO_PI;
  if (a < -Math.PI) a += TWO_PI;
  return a;
}

/** Shortest signed rotation from `a` to `b`, in [-PI, PI]. */
export function shortestDelta(a, b) {
  return wrapAngle(b - a);
}

export function clamp(v, lo, hi) {
  return v < lo ? lo : v > hi ? hi : v;
}

/**
 * Step detector.
 *
 * Pipeline: gravity-projected, band-passed, then peak-detected with hysteresis.
 *
 * 1. GRAVITY PROJECTION. A slow low-pass of the raw accelerometer vector gives
 *    the gravity direction in the device frame (walking ~1.8 Hz is far above
 *    it); the signal is the component of acceleration along that direction.
 *    Unlike the raw magnitude `|a|`, this is inert to rotating/waving the phone
 *    (the device axes and gravity sweep around, but the component along gravity
 *    stays ~constant).
 * 2. BAND-PASS (0.5-3.5 Hz). A time-based high-pass removes slow drift/orient-
 *    ation while a low-pass removes fast jitter, leaving the gait band. Time-
 *    based coefficients (`exp(-dt/tau)`) keep it frame-rate independent.
 * 3. SCHMITT + PROMINENCE. A step is a local maximum above a threshold that
 *    also stands `minProminence` above the valley since the last step; after a
 *    step the detector will not fire again until the signal falls `releaseMps2`
 *    below it. This kills the double-bumps/ripple that a plain local-max picks
 *    up, which is the "occasional extra step" while walking.
 * 4. ADAPTIVE FLOOR. While not moving, a slow EMA of the band-passed magnitude
 *    estimates the ambient noise; the gate rises with it (capped), so a noisy
 *    device/hold counts less without a fixed floor that would miss light gait.
 *
 * A peak alone is not a step: shifting the phone or shifting weight produces
 * isolated peaks too. So a peak only counts once a *walking rhythm* is
 * established - a run of `qualifySteps` peaks whose spacing stays within
 * `rhythmLo..rhythmHi` of the previous interval. Until then peaks are buffered
 * (so no real steps are lost once the walk is confirmed); a gap longer than
 * `maxIntervalMs` ends the walk and resets the rhythm. This is what stops
 * "moved a bit" from being counted as steps.
 *
 * Fed `{t (ms), ax, ay, az (m/s^2 incl. gravity)}`; returns an array of step
 * timestamps detected by this sample (usually empty, one, or - the moment a
 * walk is confirmed - the buffered ones). `steps` is the running total.
 */
export class StepDetector {
  constructor({
    minIntervalMs = 300,   // refractory: ignore peaks closer than this
    maxIntervalMs = 2000,
    gravityAlpha = 0.02,   // slow gravity-direction low-pass (tracks orientation)
    hpTauS = 0.32,         // band-pass high-pass time constant (~0.5 Hz)
    lpTauS = 0.045,        // band-pass low-pass time constant (~3.5 Hz)
    minThreshold = 0.35,   // dynamic-threshold noise floor, m/s^2
    minPeakMps2 = 0.8,     // ABSOLUTE minimum peak intensity to count a step
    peakFraction = 0.5,    // a peak must reach this fraction of the envelope
    minProminence = 0.3,   // peak must stand this far above the intervening valley
    releaseMps2 = 0.3,     // hysteresis: fall this far below a peak to arm the next
    envelopeDecay = 0.99,  // per-sample decay of the peak envelope
    noiseAlpha = 0.05,     // slow EMA of ambient noise (while not walking)
    noiseGateGain = 2,     // gate = this * noise (capped at noiseCeil)
    noiseCeil = 1.6,       // never let the adaptive gate exceed this
    qualifySteps = 3,      // rhythmic peaks needed before counting a walk
    rhythmLo = 0.6,        // interval must be >= this fraction of the previous
    rhythmHi = 1.7,        // ...and <= this multiple
  } = {}) {
    Object.assign(this, {
      minIntervalMs, maxIntervalMs, gravityAlpha, hpTauS, lpTauS,
      minThreshold, minPeakMps2, peakFraction, minProminence, releaseMps2,
      envelopeDecay, noiseAlpha, noiseGateGain, noiseCeil,
      qualifySteps, rhythmLo, rhythmHi,
    });
    this.reset();
  }

  reset() {
    this.prev = 0;           // band-passed signal, one and two samples ago
    this.prev2 = 0;
    this.hpPrevIn = 0;       // high-pass state
    this.hpPrevOut = 0;
    this.lp = 0;             // low-pass state
    this.envelope = 0;
    this.valley = Infinity;  // running min since the last peak
    this.armed = true;       // hysteresis: ready to accept a peak
    this.lastPeakVal = 0;
    this.noiseFloor = 0;
    this.lastT = null;
    this.gx = 0;             // low-passed gravity vector (device frame)
    this.gy = 0;
    this.gz = 0;
    this.lastPeakT = -Infinity;
    this.prevInterval = null;
    this.pending = [];       // peaks not yet confirmed as a walk
    this.walking = false;
    this.steps = 0;
    this.initialized = false;
  }

  push({ t, ax, ay, az }) {
    if (!this.initialized) {
      this.gx = ax; this.gy = ay; this.gz = az;
      // Seed the high-pass input with the resting projection (|a|) so it starts
      // at 0 instead of ringing down from a 9.8 m/s^2 step.
      this.hpPrevIn = Math.sqrt(ax * ax + ay * ay + az * az);
      this.lastT = t;
      this.initialized = true;
      this.lastPeakT = t;
      return [];
    }

    // --- 1. project onto the (slowly tracked) gravity direction ------------
    this.gx += this.gravityAlpha * (ax - this.gx);
    this.gy += this.gravityAlpha * (ay - this.gy);
    this.gz += this.gravityAlpha * (az - this.gz);
    const gnorm = Math.sqrt(this.gx * this.gx + this.gy * this.gy + this.gz * this.gz) || 1;
    const vertical = (ax * this.gx + ay * this.gy + az * this.gz) / gnorm;

    // --- 2. time-based band-pass (0.5-3.5 Hz) ------------------------------
    let dt = (t - this.lastT) / 1000;
    if (!(dt > 0)) dt = 1 / 50;              // bad/duplicate timestamp
    if (dt > 0.2) dt = 0.2;                  // app was backgrounded; don't overreact
    this.lastT = t;
    const aHp = Math.exp(-dt / this.hpTauS);
    const hp = aHp * (this.hpPrevOut + vertical - this.hpPrevIn);
    this.hpPrevIn = vertical;
    this.hpPrevOut = hp;
    this.lp += (1 - Math.exp(-dt / this.lpTauS)) * (hp - this.lp);
    const s = this.lp;

    // --- 3+4. thresholds and Schmitt/prominence peak detection -------------
    this.envelope *= this.envelopeDecay;
    // Ambient noise, learned only when the signal is quiet (never from gait, or
    // the gate would ratchet up during the buffering phase and deadlock).
    if (!this.walking && Math.abs(s) < this.minPeakMps2) {
      this.noiseFloor += this.noiseAlpha * (Math.abs(s) - this.noiseFloor);
    }
    const adaptive = Math.min(this.noiseCeil, this.noiseGateGain * this.noiseFloor);
    const threshold = Math.max(this.minThreshold, this.minPeakMps2, adaptive,
      this.peakFraction * this.envelope);

    let emits = [];
    if (this.armed) {
      this.valley = Math.min(this.valley, s);
      if (s < this.prev && this.prev >= this.prev2 && this.prev > threshold &&
          this.prev - this.valley >= this.minProminence) {
        this.armed = false;                  // require a release before the next peak
        this.lastPeakVal = this.prev;
        this.envelope = Math.max(this.envelope, this.prev);
        emits = this._onPeak(t);
      }
    } else if (s < this.lastPeakVal - this.releaseMps2) {
      this.armed = true;
      this.valley = s;
    }

    this.prev2 = this.prev;
    this.prev = s;
    this.steps += emits.length;
    return emits;
  }

  /** A local-maximum peak: only emit it if it is part of a real walk. */
  _onPeak(t) {
    const dt = t - this.lastPeakT;
    if (dt < this.minIntervalMs) return [];     // refractory: double trigger
    this.lastPeakT = t;

    if (dt > this.maxIntervalMs) {              // too slow to be gait: stop walking
      this.walking = false;
      this.prevInterval = null;
      this.pending = [t];
      return [];
    }

    // A sudden change of pace is not gait rhythm - restart the candidate chain.
    if (this.prevInterval != null &&
        (dt < this.rhythmLo * this.prevInterval || dt > this.rhythmHi * this.prevInterval)) {
      this.prevInterval = dt;
      this.pending = [t];
      return [];
    }

    this.prevInterval = dt;
    if (this.walking) return [t];

    // Idle: buffer until a rhythm is established, then flush so no real steps
    // are lost. Isolated movements never reach `qualifySteps`.
    this.pending.push(t);
    if (this.pending.length >= this.qualifySteps) {
      this.walking = true;
      const out = this.pending;
      this.pending = [];
      return out;
    }
    return [];
  }
}

/**
 * Magnetic declination for UAA (Anchorage, AK): magnetic north is ~15.4 deg
 * EAST of true north (NOAA WMM2025). Raw magnetometer headings are relative to
 * magnetic north and must have this added. The ~0.1 deg/yr secular drift is far
 * below the app's accuracy target and is ignored.
 */
export const DECLINATION_E_DEG = 15.4;

/**
 * World-frame yaw rate (rad/s, bearing convention) from a device-frame gyro
 * rate and the device attitude.
 *
 * `rotationRate` is DeviceMotion's deg/s triple (alpha = about device z,
 * beta = about x, gamma = about y). Integrating `alpha` alone is only correct
 * when the phone is flat: a phone held at 40 deg in the hand sweeps the yaw
 * component across the other axes. The world-up direction expressed in the
 * device frame is the third row of Rz(alpha)Rx(beta)Ry(gamma), which is
 * independent of alpha, so beta/gamma (from any DeviceOrientation event) are
 * enough to recover the rotation about the world vertical.
 *
 * Bearing increases clockwise from north, which is negative rotation about an
 * east-north-up +z axis, hence the sign at the end.
 *
 * Returns null when the attitude is unknown; callers then fall back to the raw
 * device z rate (today's approximation).
 */
export function worldYawRate(rotationRate, betaDeg, gammaDeg) {
  if (!rotationRate) return null;
  const { alpha, beta: gxDeg, gamma: gyDeg } = rotationRate;
  if (![alpha, gxDeg, gyDeg, betaDeg, gammaDeg].every(Number.isFinite)) return null;
  const b = betaDeg * DEG, g = gammaDeg * DEG;
  const wx = gxDeg * DEG, wy = gyDeg * DEG, wz = alpha * DEG;
  // up . omega_device = R[2] . omega
  const upRate = -Math.cos(b) * Math.sin(g) * wx
    + Math.sin(b) * wy
    + Math.cos(b) * Math.cos(g) * wz;
  return -upRate;
}

/**
 * Heading tracker: a compass/gyro complementary filter with magnetic-interference
 * gating.
 *
 * The compass is the absolute reference; the gyro carries the heading between
 * samples and, crucially, *through* magnetic disturbances. Two things make that
 * safe:
 *
 *   1. CONTINUOUS GYRO. The gyro is integrated every sample and its bias is
 *      learned slowly while the compass is trusted and the walker is moving
 *      steadily. (The old tracker integrated only when the compass was stale,
 *      so a fresh-but-wrong compass - exactly what steel produces - was never
 *      challenged.)
 *   2. CONSISTENCY GATE. A compass sample is only accepted when it agrees with
 *      the gyro-propagated heading within a tolerance that grows with the turn
 *      rate (to budget the compass's low-pass lag). A disagreement that
 *      persists for `anomalySamples` drops the compass out of trust; the
 *      heading then coasts on the bias-corrected gyro until the compass agrees
 *      again for `reacquireSamples` / `reacquireS`. Gating on rotation
 *      consistency - never on absolute offset - is what stops a real turn from
 *      being mistaken for drift (the failure of the old gyro-correction idea).
 *
 * If no gyro samples arrive (or `fusion: false`), the tracker behaves exactly
 * like the original compass-only smoother, so desktop/Firefox and the legacy
 * tests are unaffected.
 *
 * `gyroSign` maps a device-frame z rotation rate to the bearing convention:
 * screen-up, turning right is negative rotation about device +z, so -1.
 */
export class HeadingTracker {
  constructor({
    bias = 0, tau = 0.3, gyroSign = -1, gyroBridgeAfterMs = 250,
    // --- fusion ---
    fusion = true,
    compassTolDeg = 12,        // residual (deg) that counts as a disturbance
    compassTolRateS = 0.35,    // extra tolerance per rad/s of turn (compass lag)
    anomalySamples = 2,        // consecutive bad samples to lose trust
    reacquireSamples = 5,      // consecutive good samples to regain trust
    reacquireS = 0.4,          // ...and this much time
    gyroBiasWindowS = 5,       // residual baseline for bias learning
    gyroBiasGain = 0.2,        // fraction of each bias estimate applied
    gyroBiasMaxRadS = 0.05,    // clamp (~3 deg/s; phone gyros are far better)
    maxUntrustedS = 30,        // degradation: accept the compass again after this
    idleFreezeS = 2.0,         // no step for this long => hold heading, no bias
    turnRateDegS = 30,         // |yaw rate| above this = turning
    rateLagTauS = 0.8,         // how long a recent turn keeps widening the gate
    maxGyroDtS = 0.5,
    headingNoiseTrustedDeg = 8,
    headingNoiseCoastDeg = 40,
  } = {}) {
    this.bias = bias;
    this.tau = tau;
    this.gyroSign = gyroSign;
    this.gyroBridgeAfterMs = gyroBridgeAfterMs;
    this.fusionEnabled = fusion !== false;
    this.compassTol = compassTolDeg * DEG;
    this.compassTolRateS = compassTolRateS;
    this.anomalySamples = anomalySamples;
    this.reacquireSamples = reacquireSamples;
    this.reacquireS = reacquireS;
    this.gyroBiasWindowS = gyroBiasWindowS;
    this.gyroBiasGain = gyroBiasGain;
    this.gyroBiasMaxRadS = gyroBiasMaxRadS;
    this.maxUntrustedS = maxUntrustedS;
    this.idleFreezeS = idleFreezeS;
    this.turnRate = turnRateDegS * DEG;
    this.rateLagTauS = rateLagTauS;
    this.maxGyroDtS = maxGyroDtS;
    this.headingNoiseTrusted = headingNoiseTrustedDeg * DEG;
    this.headingNoiseCoast = headingNoiseCoastDeg * DEG;
    this.reset();
  }

  reset() {
    // Compass smoothing (sin/cos, so it never jumps across the +/-PI seam).
    this.sin = 0;
    this.cos = 1;
    this.heading = 0;
    this.rawSin = 0;
    this.rawCos = 1;
    this.rawHeading = 0;      // compass with no bias/trim applied
    this.hasHeading = false;
    this.absolute = false;
    this.lastCompassT = -Infinity;
    this.lastGyroT = null;

    // Fusion state.
    this.hasGyro = false;
    this.gyroBiasRadS = 0;
    this.gyroHeading = 0;      // pure gyro integration (bias removed)
    this._gyroPred = 0;        // gyro prediction anchored at the last accepted compass
    this._anchorT = null;
    this._lastT = null;
    this._bad = 0;
    this._good = 0;
    this._goodSince = null;
    this._biasResidual = 0;    // rad accumulated since the last bias update
    this._biasInterval = 0;    // s
    this._rate = 0;
    this._rateLag = 0;         // recent peak |rate|, decaying (gate tolerance)
    this.turning = false;
    this.lastStepT = null;
    this.state = 'searching';  // searching | trusted | coasting | reacquiring
    this.trusted = false;
    this.anomaly = null;       // 'magnetic' | 'residual' | 'forced' | null
    this.magDisturbed = false;
    this.untrustedSince = null;
  }

  /** Tell the tracker a real step happened (keeps the idle freeze honest). */
  noteStep(t) {
    if (t != null) {
      this.lastStepT = t;
      this._lastT = this._lastT == null ? t : Math.max(this._lastT, t);
    }
  }

  _idle(t) {
    return this.lastStepT != null && t - this.lastStepT > this.idleFreezeS * 1000;
  }

  /**
   * Apply a compass sample (radians, clockwise from north).
   *
   * Smoothed circularly (sin/cos) with a *time-based* low-pass, so the blend is
   * frame-rate independent. The raw (unbiased) compass is smoothed by the same
   * filter and exposed as `rawHeading`.
   */
  pushCompass(raw, t, absolute = false) {
    const rawWrapped = wrapAngle(raw);
    const s = Math.sin(rawWrapped + this.bias), c = Math.cos(rawWrapped + this.bias);
    const rs = Math.sin(rawWrapped), rc = Math.cos(rawWrapped);
    this._lastT = this._lastT == null ? t : Math.max(this._lastT, t);

    if (!this.hasHeading) {
      this.sin = s; this.cos = c;
      this.rawSin = rs; this.rawCos = rc;
      this.hasHeading = true;
      this.heading = Math.atan2(s, c);
      this.gyroHeading = this.heading;
      this._gyroPred = this.heading;
      this._anchorT = t;
      this.absolute = absolute;
      this.lastCompassT = t;
      if (this.magDisturbed) {
        this.state = 'coasting';
        this.trusted = false;
        this.untrustedSince = t;
      } else {
        this.state = 'trusted';
        this.trusted = true;
        this.untrustedSince = null;
      }
      return this.heading;
    }

    let dt = (t - this.lastCompassT) / 1000;
    if (!(dt > 0)) dt = 1 / 60;                  // bad/duplicate timestamp
    const alpha = 1 - Math.exp(-dt / this.tau);
    this.sin += alpha * (s - this.sin);
    this.cos += alpha * (c - this.cos);
    this.rawSin += alpha * (rs - this.rawSin);
    this.rawCos += alpha * (rc - this.rawCos);
    this.rawHeading = Math.atan2(this.rawSin, this.rawCos);
    this.lastCompassT = t;
    this.absolute = absolute;

    const smoothed = Math.atan2(this.sin, this.cos);

    // The lag budget must not stay inflated by a turn that already ended.
    if (this.lastGyroT != null && t - this.lastGyroT > 250) this._rate = 0;

    // No usable gyro (or fusion off): the smoothed compass IS the heading.
    if (!this.hasGyro || !this.fusionEnabled) {
      this.heading = smoothed;
      this.gyroHeading = smoothed;
      this._gyroPred = smoothed;
      this._anchorT = t;
      this.state = 'trusted';
      this.trusted = true;
      this.anomaly = null;
      return this.heading;
    }

    // --- consistency gate -------------------------------------------------
    const residual = shortestDelta(this._gyroPred, smoothed);
    // Budget the compass's low-pass lag. The lag outlives the turn itself: the
    // gyro reads ~0 the instant the walker stops turning while the smoothed
    // compass is still catching up. Using the instantaneous rate here made that
    // catch-up look exactly like a magnetic disturbance, so a legitimate fast
    // turn could drop the compass and freeze the heading. `_rateLag` holds the
    // recent peak (decaying over `rateLagTauS`) instead.
    const tol = this.compassTol + this.compassTolRateS * this._rateLag;
    const good = !this.magDisturbed && Math.abs(residual) <= tol;

    if (this.state === 'trusted') {
      if (good) {
        this._bad = 0;
        this._accept(smoothed, t, dt, residual);
      } else {
        this._bad++;
        this.anomaly = this.magDisturbed ? 'magnetic' : 'residual';
        if (this._bad >= this.anomalySamples) {
          this.state = 'coasting';
          this.trusted = false;
          this.untrustedSince = t;
          this._good = 0;
          this._goodSince = null;
        }
      }
      return this.heading;
    }

    // --- coasting / reacquiring ------------------------------------------
    if (good) {
      this._good++;
      if (this._goodSince == null) this._goodSince = t;
      if (this.state === 'coasting') {
        this.state = 'reacquiring';
        this.anomaly = null;
      }
      const longEnough = t - this._goodSince >= this.reacquireS * 1000;
      if (this._good >= this.reacquireSamples && longEnough) {
        this.state = 'trusted';
        this.trusted = true;
        this.anomaly = null;
        this.untrustedSince = null;
        this._good = 0;
        this._goodSince = null;
        this._bad = 0;
        // Blend halfway: the residual is bounded by the tolerance but a hard
        // snap would still flick the display (and the particle cloud).
        this.heading = wrapAngle(this.heading + 0.5 * shortestDelta(this.heading, smoothed));
        this.gyroHeading = this.heading;
        this._gyroPred = this.heading;
        this._anchorT = t;
      }
    } else {
      this._good = 0;
      this._goodSince = null;
      this.anomaly = 'residual';
      // Degradation ladder: a gyro-only heading drifts without bound, so after
      // `maxUntrustedS` a doubtful absolute compass is better than none. (The
      // app additionally re-biases from GPS course / the heading field.)
      if (this.untrustedSince != null &&
          t - this.untrustedSince >= this.maxUntrustedS * 1000) {
        this.state = 'trusted';
        this.trusted = true;
        this.anomaly = 'forced';
        this.untrustedSince = null;
        this.heading = smoothed;
        this.gyroHeading = smoothed;
        this._gyroPred = smoothed;
        this._anchorT = t;
      }
    }
    return this.heading;
  }

  /** Accept a consistent compass sample: blend, then learn the gyro bias. */
  _accept(smoothed, t, dt, residual) {
    const alpha = 1 - Math.exp(-dt / this.tau);
    this.heading = wrapAngle(this.heading + alpha * shortestDelta(this.heading, smoothed));
    this.gyroHeading = this.heading;
    this._gyroPred = this.heading;

    // Gyro-bias learning uses a long baseline (not the per-sample residual,
    // which is dominated by compass smoothing noise) and pauses while turning
    // or idle, where the "error" is motion rather than bias.
    if (!this._idle(t) && Math.abs(this._rate) < this.turnRate && dt > 0) {
      this._biasResidual += residual;
      this._biasInterval += dt;
      if (this._biasInterval >= this.gyroBiasWindowS) {
        // `residual` is (compass - gyro prediction), i.e. the negative of the
        // gyro's rate error, so subtract the estimate from the bias.
        const estimate = clamp(this._biasResidual / this._biasInterval,
          -this.gyroBiasMaxRadS, this.gyroBiasMaxRadS);
        this.gyroBiasRadS = clamp(
          this.gyroBiasRadS - this.gyroBiasGain * estimate,
          -this.gyroBiasMaxRadS, this.gyroBiasMaxRadS);
        this._biasResidual = 0;
        this._biasInterval = 0;
      }
    } else {
      this._biasResidual = 0;
      this._biasInterval = 0;
    }
    this._anchorT = t;
  }

  /**
   * Apply a gyro sample.
   *
   * `gz` is the device-frame z rotation rate (radians/s); `yawRate`, when
   * supplied, is the world-frame bearing rate (radians/s, cw-positive) computed
   * by `worldYawRate` and is preferred. With fusion on, the gyro is integrated
   * continuously; with fusion off, it only bridges a stale compass (the
   * original behaviour).
   */
  pushGyro(gz, t, yawRate = null) {
    const rate = (yawRate != null && Number.isFinite(yawRate))
      ? yawRate : this.gyroSign * gz;
    if (Number.isFinite(rate)) this.hasGyro = true;
    this._lastT = this._lastT == null ? t : Math.max(this._lastT, t);

    if (this.lastGyroT != null && this.hasHeading) {
      const dt = (t - this.lastGyroT) / 1000;
      if (dt > 0 && dt < this.maxGyroDtS) {
        const fresh = (t - this.lastCompassT) <= this.gyroBridgeAfterMs;
        if (this.fusionEnabled) {
          if (!this._idle(t)) {
            const corrected = rate - this.gyroBiasRadS;
            this.heading = wrapAngle(this.heading + corrected * dt);
            this.gyroHeading = wrapAngle(this.gyroHeading + corrected * dt);
            this._gyroPred = wrapAngle(this._gyroPred + corrected * dt);
          }
        } else if (!fresh) {
          // Legacy: bridge only while the compass is stale.
          this.heading = wrapAngle(this.heading + rate * dt);
        }
      }
    }

    if (Number.isFinite(rate)) {
      // Smooth the measured rate for the turn flag (fast, so `turning` is
      // timely), and keep a slowly-decaying peak for the gate tolerance (the
      // compass's low-pass lag persists after the turn stops).
      this._rate = this._rate * 0.6 + rate * 0.4;
      const dtLag = this.lastGyroT != null ? Math.max(0, (t - this.lastGyroT) / 1000) : 0;
      const decay = Math.exp(-dtLag / this.rateLagTauS);
      this._rateLag = Math.max(Math.abs(rate), this._rateLag * decay);
      this.turning = Math.abs(this._rate) > this.turnRate;
    }
    this.lastGyroT = t;
    return this.heading;
  }

  /** Compass disturbance from the field monitor; applied at the next sample. */
  setMagneticDisturbance(disturbed, t = null) {
    this.magDisturbed = !!disturbed;
    // Force the transition immediately: a raw-magnetometer pipeline may stop
    // feeding compass samples while the field is disturbed, so waiting for the
    // next one would leave `trusted` true (and the particle filter over-tight).
    if (!this.magDisturbed || !this.hasHeading || !this.hasGyro) return;
    if (this.state === 'trusted') {
      this.state = 'coasting';
      this.trusted = false;
      this.untrustedSince = t ?? this.lastGyroT ?? this._anchorT ?? this._lastT;
      this._good = 0;
      this._goodSince = null;
    }
  }

  /**
   * Signed error between the fused heading and an ABSOLUTE reference (GPS
   * course), or null when it is too large to trust. Callers add a fraction of
   * it to their heading bias. Only meaningful when the heading frame is
   * absolute (never during a relative-frame calibration walk).
   */
  courseError(headingRad, { tolDeg = 25 } = {}) {
    if (!this.hasHeading || !this.absolute || !this.trusted) return null;
    if (!Number.isFinite(headingRad)) return null;
    const err = shortestDelta(this.heading, headingRad);
    if (Math.abs(err) > tolDeg * DEG) return null;
    return err;
  }

  /** Set the compass bias (radians); applied to every later compass sample. */
  calibrate(bias) {
    this.bias = bias;
  }

  /** ms since the last compass sample (Infinity if none). */
  staleness(t) {
    return t - this.lastCompassT;
  }

  /**
   * Heading uncertainty (radians) for the particle filter: tight while the
   * compass is trusted, widening as the gyro coasts with no absolute check.
   */
  get headingSigmaRad() {
    // Only meaningful when the fusion core is actually running: with fusion
    // off (the kill switch) or no gyro, the caller must keep the legacy
    // particle-filter behaviour, which uses `null` = default jitter.
    if (!this.fusionEnabled || !this.hasGyro || !this.hasHeading) return null;
    if (this.state === 'trusted') return this.headingNoiseTrusted;
    const t = this._lastT ?? 0;
    const age = this.untrustedSince == null ? 0
      : Math.max(0, (t - this.untrustedSince) / 1000);
    const grow = Math.min(1, age / this.maxUntrustedS);
    return this.headingNoiseTrusted
      + (this.headingNoiseCoast - this.headingNoiseTrusted) * grow;
  }
}

/**
 * Step length as a function of cadence.
 *
 * A calibrated base length at a reference cadence is scaled roughly
 * proportionally to cadence and clamped. This is the deliberately simple,
 * ML-free model chosen for the 4-5 m accuracy target.
 */
export class StepLengthModel {
  constructor({ base = 0.72, cadenceRef = 1.8, gain = 0.35, min = 0.45, max = 0.95 } = {}) {
    Object.assign(this, { base, cadenceRef, gain, min, max });
  }

  lengthFor(cadenceHz) {
    if (!isFinite(cadenceHz) || cadenceHz <= 0) return this.base;
    const factor = 1 + (this.gain * (cadenceHz - this.cadenceRef)) / this.cadenceRef;
    return clamp(this.base * factor, this.min, this.max);
  }
}

/**
 * One-time walking calibration: count steps over a measured distance and
 * divide. `finish()` returns the base step length (metres), or null if no
 * steps were recorded.
 */
export class WalkCalibrator {
  constructor() {
    this.running = false;
    this.steps = 0;
  }

  start() {
    this.running = true;
    this.steps = 0;
  }

  addStep() {
    if (this.running) this.steps++;
  }

  finish(distanceMeters) {
    this.running = false;
    if (!this.steps || !(distanceMeters > 0)) return null;
    return distanceMeters / this.steps;
  }
}

/**
 * Which sensor APIs this environment exposes. `needsPermission` is true on
 * iOS 13+, where a user gesture must call requestPermission() first.
 */
export function sensorSupport(win = globalThis) {
  const has = (k) => typeof win[k] !== 'undefined';
  const dmPerm = typeof win.DeviceMotionEvent?.requestPermission === 'function';
  const doPerm = typeof win.DeviceOrientationEvent?.requestPermission === 'function';
  return {
    deviceMotion: has('DeviceMotionEvent'),
    deviceOrientation: has('DeviceOrientationEvent'),
    generic: has('Accelerometer') || has('Gyroscope'),
    magnetometer: has('Magnetometer') || has('UncalibratedMagnetometer'),
    needsPermission: dmPerm || doPerm,
  };
}

/**
 * Normalise a DeviceOrientationEvent to a heading (radians, cw from north).
 *
 * The `absolute` flag says whether the reading is a true compass bearing:
 *   - iOS `webkitCompassHeading` (on the plain event) is absolute truth;
 *   - `deviceorientationabsolute` is absolute;
 *   - the plain `deviceorientation` alpha is a stable but *arbitrary* frame
 *     (relative), which is only useful once an offset has been calibrated.
 * `createMotion` decides which of these to use (see `headingMode`).
 */
export function headingFromOrientation(event) {
  if (typeof event.webkitCompassHeading === 'number' && !isNaN(event.webkitCompassHeading)) {
    return { heading: wrapAngle(event.webkitCompassHeading * DEG), absolute: true };
  }
  if (typeof event.alpha === 'number' && event.alpha !== null) {
    const absolute = event.type === 'deviceorientationabsolute';
    return { heading: wrapAngle((360 - event.alpha) * DEG), absolute };
  }
  return null;
}

/**
 * Wire the sensors to the signal classes.
 *
 * @param {object}   [opts]
 * @param {Function} [opts.onStep]    ({t, cadenceHz}) on each detected step
 * @param {Function} [opts.onHeading] ({heading, raw, absolute, t, trusted,
 *        state, turning, headingSigmaRad, gyroBiasRadS, source}) heading change
 * @param {Function} [opts.onSample]  ({t, ax, ay, az}) raw acceleration (telemetry)
 * @param {Function} [opts.onStatus]  ({type, ...}) lifecycle/permission events
 * @param {Function} [opts.onMagnetometer] ({raw, t}) raw magnetometer sample
 * @param {Function} [opts.onMagneticField] field-tracker result (telemetry)
 * @param {object}   [opts.stepOptions]    passed to StepDetector
 * @param {object}   [opts.headingOptions] passed to HeadingTracker
 * @param {boolean}  [opts.fusion=true]   compass/gyro fusion (false = legacy)
 * @param {number}   [opts.declinationDeg] magnetic -> true north (default +15.4)
 * @param {object}   [opts.ironCalibration] a MagnetometerCalibration, restored
 *        from persistence by the caller
 * @param {object}   [opts.win]       window-like object (injectable for tests)
 * @returns {{start:Function, stop:Function, detector:StepDetector,
 *            heading:HeadingTracker, support:object, running:boolean,
 *            headingSource:string, fieldTracker:MagneticFieldTracker}}
 */
export function createMotion(opts = {}) {
  const {
    onStep, onHeading, onSample, onStatus,
    onMagnetometer, onMagneticField,
    stepOptions = {}, headingOptions = {},
    headingMode: initialHeadingMode = 'absolute',
    fusion = true,
    declinationDeg = DECLINATION_E_DEG,
    ironCalibration = null,
    win = globalThis,
  } = opts;

  const detector = new StepDetector(stepOptions);
  const heading = new HeadingTracker({ fusion, ...headingOptions });
  const support = sensorSupport(win);
  const declinationRad = declinationDeg * DEG;
  const fieldTracker = new MagneticFieldTracker();
  let magnetometerCal = ironCalibration;
  let running = false;
  let starting = null;   // in-flight start() promise (guards duplicate requests)
  let lastStepT = null;

  // Which frame the heading comes from. 'relative' uses the plain
  // deviceorientation alpha, which has an arbitrary but stable offset that the
  // calibration walk measures - good indoors where the magnetometer is
  // unreliable. Otherwise a raw Generic-Sensor magnetometer is preferred
  // ('raw-mag', opportunistic: flag-gated in Chrome), falling back to the
  // OS-fused absolute orientation event ('os-fused'). Frames are never mixed.
  let headingMode = initialHeadingMode;
  let headingSource = headingMode === 'relative' ? 'relative' : 'os-fused';
  let magnetometer = null;

  // Device attitude (beta/gamma only; alpha is not needed for the world yaw
  // rate) and a low-passed gravity vector for tilt compensation.
  let attitude = null;       // {beta, gamma, t}
  let gravity = null;        // {x, y, z} low-passed, device frame
  let gravityInit = false;

  // Sensor timestamps must share the clock the app uses for GPS fixes and
  // anchors (performance.now). Chrome's event.timeStamp does; some Safari /
  // WebView versions report epoch milliseconds instead, which cannot be
  // compared with performance.now().
  const perfNow = () => (win.performance ? win.performance.now() : Date.now());
  const now = (e) => {
    const perf = perfNow();
    const ts = e?.timeStamp;
    if (typeof ts !== 'number' || !Number.isFinite(ts)) return perf;
    // Epoch seconds/milliseconds are far ahead of the page's performance clock.
    if (ts > perf + 1e6) return perf;
    return ts;

  };

  /** The heading payload every consumer receives (extra fields are additive). */
  function headingEntry(absolute, t) {
    return {
      heading: heading.heading,
      raw: heading.rawHeading,
      absolute,
      t,
      trusted: heading.trusted,
      state: heading.state,
      turning: heading.turning,
      headingSigmaRad: heading.headingSigmaRad,
      gyroBiasRadS: heading.gyroBiasRadS,
      source: headingSource,
    };
  }

  function handleMotion(e) {
    const a = e.accelerationIncludingGravity || e.acceleration;
    if (!a || a.x == null) return;
    const t = now(e);
    onSample?.({ t, ax: a.x, ay: a.y, az: a.z });

    // Gravity: a slow low-pass of the raw accelerometer vector (the gait band
    // is far above it). Used only for raw-magnetometer tilt compensation.
    if (!gravityInit) {
      gravity = { x: a.x, y: a.y, z: a.z };
      gravityInit = true;
    } else {
      const k = 0.05;
      gravity = {
        x: gravity.x + k * (a.x - gravity.x),
        y: gravity.y + k * (a.y - gravity.y),
        z: gravity.z + k * (a.z - gravity.z),
      };
    }

    const emits = detector.push({ t, ax: a.x, ay: a.y, az: a.z });
    for (const stepT of emits) {
      const cadenceHz = lastStepT == null ? 0 : 1000 / (stepT - lastStepT);
      lastStepT = stepT;
      heading.noteStep(stepT);
      onStep?.({ t: stepT, cadenceHz });
    }

    const r = e.rotationRate;
    if (r) {
      // Rotate the device-frame rate into the world frame when attitude is
      // known, so a phone held at an angle still measures yaw correctly.
      const yawRate = attitude
        ? worldYawRate(r, attitude.beta, attitude.gamma) : null;
      // Only feed a real rate: a partial rotationRate (no alpha and no usable
      // attitude) must not enable fusion with a dead gyro.
      if (yawRate != null || r.alpha != null) {
        heading.pushGyro((r.alpha ?? 0) * DEG, t, yawRate);
        onHeading?.(headingEntry(heading.absolute, t));
      }
    }
  }

  function handleOrientation(e) {
    // Attitude for the gyro yaw rotation: any orientation event carries it,
    // including the plain one that is otherwise ignored for heading.
    if (Number.isFinite(e.beta) && Number.isFinite(e.gamma)) {
      attitude = { beta: e.beta, gamma: e.gamma, t: now(e) };
    }

    // Raw-mag mode computes its own heading; an OS-fused heading would be a
    // second, differently-biased frame. Attitude above is still useful.
    if (headingSource === 'raw-mag') return;

    const h = headingFromOrientation(e);
    if (!h) return;
    const isPlain = e.type === 'deviceorientation';
    const isIosCompass = typeof e.webkitCompassHeading === 'number';

    // Pick one source per mode; never mix, or the heading jumps between frames.
    if (headingMode === 'relative') {
      if (!isPlain) return;
    } else {
      if (isPlain && !isIosCompass) return;   // plain Android alpha is not a compass
    }

    const t = now(e);
    heading.pushCompass(h.heading, t, h.absolute);
    onHeading?.(headingEntry(h.absolute, t));
  }

  /** Feed one raw magnetometer reading through calibration + fusion. */
  function handleMagnetometer() {
    if (!magnetometer) return;
    const raw = { x: magnetometer.x, y: magnetometer.y, z: magnetometer.z };
    if (![raw.x, raw.y, raw.z].every(Number.isFinite)) return;
    const ts = Number.isFinite(magnetometer.timestamp)
      ? magnetometer.timestamp : perfNow();
    const t = now({ timeStamp: ts });

    onMagnetometer?.({ raw, t });
    const field = fieldTracker.push(raw, gravity);
    onMagneticField?.({ ...field, t, raw });
    heading.setMagneticDisturbance(field.disturbed, t);

    if (magnetometerCal) magnetometerCal.add(raw);
    if (field.disturbed) return;   // never feed a lying compass to the fusion

    const corrected = magnetometerCal
      ? magnetometerCal.correct(raw)
      : [raw.x, raw.y, raw.z];
    const h = gravityInit
      ? tiltCompensatedHeading(corrected, gravity, declinationRad) : null;
    if (!h) return;
    heading.pushCompass(h.heading, t, true);
    onHeading?.({
      ...headingEntry(true, t),
      magneticHeading: h.magneticHeading,
      dipDeg: h.dipDeg,
    });
  }

  function disableMagnetometer(reason) {
    if (magnetometer) {
      try {
        magnetometer.removeEventListener?.('reading', handleMagnetometer);
        magnetometer.removeEventListener?.('error', onMagnetometerError);
        magnetometer.stop?.();
      } catch (e) { /* ignore */ }
    }
    magnetometer = null;
    if (headingMode !== 'relative') {
      headingSource = 'os-fused';
      onStatus?.({ type: 'magnetometer-unavailable', reason });
    }
  }

  function onMagnetometerError(event) {
    disableMagnetometer(event?.error?.name || 'error');
  }

  /** Start the Generic-Sensor magnetometer, if the platform exposes it. */
  function startMagnetometer() {
    if (magnetometer || headingMode === 'relative') return false;
    const Mag = win.UncalibratedMagnetometer || win.Magnetometer;
    if (!Mag) return false;
    try {
      magnetometer = new Mag({ frequency: 20 });
      magnetometer.addEventListener('reading', handleMagnetometer);
      magnetometer.addEventListener('error', onMagnetometerError);
      magnetometer.start();
      headingSource = 'raw-mag';
      return true;
    } catch (error) {
      magnetometer = null;
      headingSource = 'os-fused';
      onStatus?.({ type: 'magnetometer-unavailable', error });
      return false;
    }
  }

  async function start() {
    if (running) return;
    if (starting) return starting;   // a second call joins the in-flight request
    starting = doStart();
    try { await starting; } finally { starting = null; }
  }

  async function doStart() {
    if (support.needsPermission) {
      // ONE request is enough: iOS shows a single "Motion & Orientation
      // Access" prompt, and granting it enables both `devicemotion` and
      // `deviceorientation`. Asking twice is what breaks things - awaiting the
      // first spends the user activation so the second call rejects (old bug),
      // and firing both in the same tick lets the second cancel the first, so
      // only one API ends up granted (steps never arrive). Request the one that
      // carries the accelerometer.
      const requestPermission =
        (typeof win.DeviceMotionEvent?.requestPermission === 'function')
          ? win.DeviceMotionEvent.requestPermission.bind(win.DeviceMotionEvent)
          : (typeof win.DeviceOrientationEvent?.requestPermission === 'function')
            ? win.DeviceOrientationEvent.requestPermission.bind(win.DeviceOrientationEvent)
            : null;
      if (requestPermission) {
        let state;
        try {
          state = await requestPermission();
        } catch (error) {
          onStatus?.({ type: 'permission-denied', error });
          return;
        }
        if (state === 'denied') {
          onStatus?.({ type: 'permission-denied' });
          return;
        }
      }
    }
    win.addEventListener?.('devicemotion', handleMotion, true);
    win.addEventListener?.('deviceorientationabsolute', handleOrientation, true);
    win.addEventListener?.('deviceorientation', handleOrientation, true);
    startMagnetometer();
    running = true;
    onStatus?.({ type: 'started', support });
  }

  function stop() {
    win.removeEventListener?.('devicemotion', handleMotion, true);
    win.removeEventListener?.('deviceorientationabsolute', handleOrientation, true);
    win.removeEventListener?.('deviceorientation', handleOrientation, true);
    disableMagnetometer('stopped');
    running = false;
    onStatus?.({ type: 'stopped' });
  }

  return {
    start, stop, detector, heading, support,
    fieldTracker,
    get magnetometer() { return magnetometer; },
    get ironCalibration() { return magnetometerCal; },
    setIronCalibration(cal) { magnetometerCal = cal; },
    get running() { return running; },
    get headingMode() { return headingMode; },
    get headingSource() { return headingSource; },
    get declinationRad() { return declinationRad; },
    setHeadingMode(mode) {
      headingMode = mode;
      if (mode === 'relative') {
        // Relative mode must never mix in the magnetometer frame.
        disableMagnetometer('relative-mode');
        headingSource = 'relative';
      } else if (!magnetometer) {
        startMagnetometer();
        if (!magnetometer) headingSource = 'os-fused';
      }
    },
  };
}
