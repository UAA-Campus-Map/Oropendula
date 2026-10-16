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
 * Heading tracker.
 *
 * Compass samples set the absolute heading (with a calibrated bias); gyro
 * samples bridge the gaps between them. The compass is smoothed circularly
 * (sin/cos accumulators, so it never jumps across the +/-PI seam), while gyro
 * integration is applied directly. This lets a transient magnetic disturbance
 * be ridden out on the gyro until the next trustworthy compass sample.
 *
 * Gyro-compass fusion: because the gyro is the short-term rotation reference, a
 * compass heading that moves while the gyro reports (almost) no rotation is a
 * magnetic disturbance - the compass OFFSET changed. `pushCompass` then folds
 * that movement into `autoOffset` so the heading does not jump.
 *
 * `gyroSign` maps device-frame z rotation rate to our bearing convention:
 * with the phone screen-up, turning right (bearing increasing) is negative
 * rotation about device +z, so the default is -1.
 */
export class HeadingTracker {
  constructor({ bias = 0, tau = 0.3, gyroSign = -1, gyroBridgeAfterMs = 250,
    gyroFuse = true, gyroFuseGain = 0.8, compassJumpDeg = 5,
    compassWindowMs = 6000, gyroSilenceMs = 500, gyroFuseMaxCorrDeg = 45 } = {}) {
    this.bias = bias;
    this.tau = tau;                       // smoothing time constant, seconds
    this.gyroSign = gyroSign;
    this.gyroBridgeAfterMs = gyroBridgeAfterMs;
    // Windowed gyro-compass fusion. The gyro is the short-term rotation
    // reference; over a bounded baseline we compare how far the COMPASS moved
    // against how far the GYRO says the device rotated. A sustained difference
    // is a compass-offset change (magnetic drift), not a real turn, so it is
    // folded into a learned offset instead of moving the heading. A window of a
    // few seconds catches slower drift than a per-sample gate: the accumulated
    // drift grows with the window while the two-sample magnetometer noise does
    // not.
    this.gyroFuse = gyroFuse;
    this.gyroFuseGain = gyroFuseGain;
    this.compassJump = compassJumpDeg * DEG;   // accumulated mismatch to absorb
    this.compassWindowMs = compassWindowMs;    // longest baseline (caps gyro bias)
    this.gyroSilenceMs = gyroSilenceMs;        // no gyro this long -> re-anchor
    this.gyroFuseMaxCorr = gyroFuseMaxCorrDeg * DEG;
    this.reset();
  }

  reset() {
    this.sin = 0;
    this.cos = 1;
    this.heading = 0;
    this.hasHeading = false;
    this.absolute = false;
    this.lastCompassT = -Infinity;
    this.lastGyroT = null;
    this.refCompass = null;    // raw compass at the fusion window's reference
    this.refGyro = 0;          // gyro rotation accumulated since that reference
    this.refT = -Infinity;     // reference time
    this.autoOffset = 0;       // compass offset learned from the windowed fusion
    this.offsetRevision = 0;   // bumps whenever the fused offset changes
    this.lastOffsetDelta = 0;  // the offset change applied (radians)
  }

  /** (Re-)anchor the fusion window at a fresh compass sample. */
  _anchorWindow(rawWrapped, t) {
    this.refCompass = rawWrapped;
    this.refGyro = 0;
    this.refT = t;
  }

  /** Forget the gyro-learned compass offset (e.g. after a fresh calibration). */
  resetAutoOffset() {
    this.autoOffset = 0;
  }

  /**
   * Apply a compass sample (radians, clockwise from north).
   *
   * Smoothed circularly (sin/cos, so it never jumps across the +/-PI seam) with
   * a *time-based* low-pass: the blend factor is `1 - exp(-dt/tau)`, so the
   * smoothing is the same whether compass events arrive at 10 Hz or 50 Hz. The
   * magnetometer jitters by several degrees sample to sample; without this the
   * readout and heading arrow flicker.
   */
  pushCompass(raw, t, absolute = false) {
    const rawWrapped = wrapAngle(raw);
    // Windowed gyro-compass fusion (see constructor).
    if (this.gyroFuse) {
      const gyroFresh = this.lastGyroT != null && (t - this.lastGyroT) <= this.gyroSilenceMs;
      if (this.refCompass == null || !gyroFresh) {
        this._anchorWindow(rawWrapped, t);              // first sample / gyro gap
      } else {
        const resid = wrapAngle(shortestDelta(this.refCompass, rawWrapped) - this.refGyro);
        const windowMs = t - this.refT;
        if (Math.abs(resid) >= this.compassJump && windowMs <= this.compassWindowMs) {
          const corr = clamp(this.gyroFuseGain * resid,
            -this.gyroFuseMaxCorr, this.gyroFuseMaxCorr);
          this.autoOffset = wrapAngle(this.autoOffset - corr);
          this.offsetRevision++;
          this.lastOffsetDelta = -corr;
          this._anchorWindow(rawWrapped, t);
        } else if (windowMs > this.compassWindowMs) {
          this._anchorWindow(rawWrapped, t);            // baseline expired
        }
        // else keep accumulating toward the threshold
      }
    }

    const corrected = wrapAngle(rawWrapped + this.bias + this.autoOffset);
    const s = Math.sin(corrected), c = Math.cos(corrected);
    if (!this.hasHeading) {
      this.sin = s; this.cos = c;
      this.hasHeading = true;
    } else {
      let dt = (t - this.lastCompassT) / 1000;
      if (!(dt > 0)) dt = 1 / 60;                  // bad/duplicate timestamp
      const alpha = 1 - Math.exp(-dt / this.tau);
      this.sin += alpha * (s - this.sin);
      this.cos += alpha * (c - this.cos);
    }
    this.heading = Math.atan2(this.sin, this.cos);
    this.absolute = absolute;
    this.lastCompassT = t;
    return this.heading;
  }

  /**
   * Apply a gyro sample (radians/s about device +z).
   *
   * Gyro integration only bridges a *gap* in the compass - if the compass is
   * still fresh it adds nothing but jitter, so it is ignored until the last
   * compass sample is older than `gyroBridgeAfterMs`.
   */
  pushGyro(gz, t) {
    if (this.lastGyroT != null) {
      const dt = (t - this.lastGyroT) / 1000;
      if (dt > 0 && dt < 0.5) {
        const d = this.gyroSign * gz * dt;
        // Accumulate the device's rotation since the fusion window's reference.
        this.refGyro = wrapAngle(this.refGyro + d);
        // Bridge a compass gap on the gyro (unchanged): only while the compass
        // is stale, so a fresh compass is not perturbed by gyro jitter.
        if (this.hasHeading && (t - this.lastCompassT) > this.gyroBridgeAfterMs) {
          this.heading = wrapAngle(this.heading + d);
          this.sin = Math.sin(this.heading);
          this.cos = Math.cos(this.heading);
        }
      }
    }
    this.lastGyroT = t;
    return this.heading;
  }

  /** Set the compass bias (radians); applied to every later compass sample. */
  calibrate(bias) {
    this.bias = bias;
  }

  /** ms since the last compass sample (Infinity if none). */
  staleness(t) {
    return t - this.lastCompassT;
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
 * @param {Function} [opts.onHeading] ({heading, absolute, t}) on heading change
 * @param {Function} [opts.onSample]  ({t, ax, ay, az}) raw acceleration (telemetry)
 * @param {Function} [opts.onStatus]  ({type, ...}) lifecycle/permission events
 * @param {object}   [opts.stepOptions]    passed to StepDetector
 * @param {object}   [opts.headingOptions] passed to HeadingTracker
 * @param {object}   [opts.win]       window-like object (injectable for tests)
 * @returns {{start:Function, stop:Function, detector:StepDetector,
 *            heading:HeadingTracker, support:object, running:boolean}}
 */
export function createMotion(opts = {}) {
  const {
    onStep, onHeading, onSample, onStatus,
    stepOptions = {}, headingOptions = {},
    headingMode: initialHeadingMode = 'absolute',
    win = globalThis,
  } = opts;

  const detector = new StepDetector(stepOptions);
  const heading = new HeadingTracker(headingOptions);
  const support = sensorSupport(win);
  let running = false;
  let starting = null;   // in-flight start() promise (guards duplicate requests)
  let lastStepT = null;

  // Which frame the heading comes from. 'absolute' trusts the magnetometer
  // (deviceorientationabsolute / iOS compass heading). 'relative' uses the
  // plain deviceorientation alpha, which has an arbitrary but stable offset
  // that the calibration walk measures - good indoors where the magnetometer
  // is unreliable. The two are never mixed.
  let headingMode = initialHeadingMode;

  const now = (e) => (e && typeof e.timeStamp === 'number') ? e.timeStamp
    : (win.performance ? win.performance.now() : Date.now());

  function handleMotion(e) {
    const a = e.accelerationIncludingGravity || e.acceleration;
    if (!a || a.x == null) return;
    const t = now(e);
    onSample?.({ t, ax: a.x, ay: a.y, az: a.z });

    const emits = detector.push({ t, ax: a.x, ay: a.y, az: a.z });
    for (const stepT of emits) {
      const cadenceHz = lastStepT == null ? 0 : 1000 / (stepT - lastStepT);
      lastStepT = stepT;
      onStep?.({ t: stepT, cadenceHz });
    }

    const r = e.rotationRate;
    if (r && r.alpha != null) {
      heading.pushGyro(r.alpha * DEG, t);
      onHeading?.({ heading: heading.heading, absolute: heading.absolute, t });
    }
  }

  function handleOrientation(e) {
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
    onHeading?.({ heading: heading.heading, absolute: h.absolute, t });
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
    running = true;
    onStatus?.({ type: 'started', support });
  }

  function stop() {
    win.removeEventListener?.('devicemotion', handleMotion, true);
    win.removeEventListener?.('deviceorientationabsolute', handleOrientation, true);
    win.removeEventListener?.('deviceorientation', handleOrientation, true);
    running = false;
    onStatus?.({ type: 'stopped' });
  }

  return {
    start, stop, detector, heading, support,
    get running() { return running; },
    get headingMode() { return headingMode; },
    setHeadingMode(mode) { headingMode = mode; },
  };
}
