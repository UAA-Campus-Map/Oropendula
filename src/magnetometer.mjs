/**
 * `magnetometer.mjs` — magnetometer hygiene for the compass/gyro fusion.
 *
 * Two independent jobs, both pure and unit-testable (no Sensor API here):
 *
 *  1. Detect *magnetic disturbance* — walking past a steel doorframe, a
 *     rebar column, an elevator motor or a filing cabinet changes the local
 *     field. `MagneticFieldTracker` watches the field magnitude and flags the
 *     disturbed stretches so the fusion core can stop trusting the compass
 *     before it lies.
 *
 *  2. Undo the *phone's own* magnetic distortion. `CompassCalibration` fits a
 *     hard-iron offset (a sphere centre — the magnet in a case, speaker, or
 *     the phone's own ferrous parts) and an optional diagonal soft-iron scale
 *     (an ellipsoid) from a figure-8 / tumble sample, then reports whether the
 *     fit is good enough to apply. `tiltCompensatedHeading` converts a
 *     corrected 3-axis field plus the gravity vector into a compass bearing
 *     that is valid in any phone orientation.
 *
 * Everything here works in microtesla (uT). Earth's field is ~25–65 uT
 * depending on latitude; around 50 uT in the Puget Sound / Southcentral
 * Alaska region. A phone's own offset can easily be 10–50 uT, which is why
 * raw magnetometer data is useless without at least a hard-iron fit.
 *
 * This module deliberately knows nothing about *true north*: the heading it
 * returns is relative to MAGNETIC north. The caller adds declination.
 */

const DEG = Math.PI / 180;

/** Length of a 3-vector. */
export function magnitude([x, y, z]) {
  return Math.sqrt(x * x + y * y + z * z);
}

function median(values) {
  if (!values.length) return 0;
  const a = values.slice().sort((p, q) => p - q);
  const m = a.length >> 1;
  return a.length % 2 ? a[m] : 0.5 * (a[m - 1] + a[m]);
}

function clamp(v, lo, hi) { return v < lo ? lo : v > hi ? hi : v; }

function wrapAngle(a) {
  a = a % (2 * Math.PI);
  if (a > Math.PI) a -= 2 * Math.PI;
  if (a < -Math.PI) a += 2 * Math.PI;
  return a;
}

/**
 * Rolling field-magnitude monitor.
 *
 * Flags `disturbed` when the total field leaves a plausible range, drifts far
 * from its own recent median, or jumps abruptly between samples. The median
 * (not the mean) keeps a single bad reading from moving the reference, and a
 * slow drift of the median means a long steel corridor cannot permanently
 * desensitise the detector — only an abrupt change trips it.
 */
export class MagneticFieldTracker {
  /**
   * @param {object} [opts]
   * @param {number} [opts.windowSamples=40]   median window (~2 s at 20 Hz)
   * @param {number} [opts.minUT=20]           below this the field is implausible
   * @param {number} [opts.maxUT=80]           above this you are next to steel
   * @param {number} [opts.deviationFrac=0.15] fraction of the median that counts
   *                                           as a disturbance
   * @param {number} [opts.maxRateUT=12]       per-sample jump that counts
   * @param {number} [opts.rateSamples=2]      consecutive jumps needed (noise guard)
   */
  constructor({
    windowSamples = 40, minUT = 20, maxUT = 80,
    deviationFrac = 0.15, maxRateUT = 12, rateSamples = 2,
  } = {}) {
    Object.assign(this, {
      windowSamples, minUT, maxUT, deviationFrac, maxRateUT, rateSamples,
    });
    this.reset();
  }

  reset() {
    this.history = [];
    this.lastMag = null;
    this.rateCount = 0;
    this.disturbed = false;
    this.reason = null;
    this.magnitudeUT = null;
    this.medianUT = null;
    /** @type {number|null} dip (inclination) in degrees, when gravity is known */
    this.dipDeg = null;
  }

  /**
   * Feed one raw sample.
   * @param {{x:number,y:number,z:number}} sample  device-frame field, uT
   * @param {{x:number,y:number,z:number}} [up]    world-up in device frame
   *        (from a low-passed accelerometer); enables dip and is otherwise
   *        ignored.
   * @returns {{magnitudeUT:number, medianUT:number, dipDeg:number|null,
   *            disturbed:boolean, reason:string|null}}
   */
  push(sample, up = null) {
    const v = [sample.x, sample.y, sample.z];
    const mag = magnitude(v);
    this.magnitudeUT = mag;
    if (up) {
      const u = magnitude([up.x, up.y, up.z]) || 1;
      const vertical = (v[0] * up.x + v[1] * up.y + v[2] * up.z) / u;
      const horizontal = Math.sqrt(Math.max(0, mag * mag - vertical * vertical));
      this.dipDeg = Math.atan2(-vertical, horizontal) / DEG;
    }

    this.history.push(mag);
    if (this.history.length > this.windowSamples) this.history.shift();
    const med = median(this.history);
    this.medianUT = med;

    let reason = null;
    if (!(mag > 0) || mag < this.minUT) reason = 'weak';
    else if (mag > this.maxUT) reason = 'strong';
    else if (med > 0 && Math.abs(mag - med) / med > this.deviationFrac) reason = 'offset';

    if (reason == null && this.lastMag != null && this.lastMag > 0) {
      const rate = Math.abs(mag - this.lastMag);
      if (rate > this.maxRateUT) this.rateCount++;
      else this.rateCount = 0;
      if (this.rateCount >= this.rateSamples) reason = 'spike';
    } else if (reason == null) {
      this.rateCount = 0;
    }

    this.lastMag = mag;
    this.disturbed = reason != null;
    this.reason = reason;
    return {
      magnitudeUT: mag, medianUT: med, dipDeg: this.dipDeg,
      disturbed: this.disturbed, reason,
    };
  }
}

/**
 * Hard-iron + (diagonal) soft-iron fit from a tumble sample.
 *
 * The magnetometer's error is `raw = S * R * B_earth + h` where `h` is the
 * hard-iron offset and `S` the soft-iron distortion. A figure-8 walk makes
 * `R * B_earth` sweep a sphere; a closed-form fit from the per-axis extremes
 * recovers `h` (sphere centre) and the axis scale ratios. The ellipsoid is
 * approximated as axis-aligned, which is enough to remove the dominant
 * distortion and keeps the fit small, dependency-free and hard to overfit.
 *
 * `fit()` refuses to report `ok` unless the sample covers enough of every axis
 * and the residual is small: a half-hearted wave must not silently install a
 * bogus calibration.
 */
export class MagnetometerCalibration {
  constructor({
    maxSamples = 4000,
    minSamples = 60,
    minCoverage = 0.55,   // each axis radius must reach this fraction of the mean
    maxResidualFrac = 0.12, // RMS radius error after correction
    minScale = 0.5,
    maxScale = 2.0,
  } = {}) {
    Object.assign(this, {
      maxSamples, minSamples, minCoverage, maxResidualFrac, minScale, maxScale,
    });
    this.reset();
  }

  reset() {
    this.samples = [];
    this.fitResult = null;
    this.collecting = false;
  }

  /** Begin a fresh calibration (discards any previous samples). */
  start() {
    this.reset();
    this.collecting = true;
  }

  /** Add one raw sample; safe to call when not collecting (returns false). */
  add(sample) {
    if (!this.collecting) return false;
    const { x, y, z } = sample ?? {};
    if (![x, y, z].every(Number.isFinite)) return false;
    this.samples.push([x, y, z]);
    if (this.samples.length > this.maxSamples) this.samples.shift();
    return true;
  }

  get sampleCount() { return this.samples.length; }

  /**
   * Compute the fit. Idempotent; stores and returns the result.
   * @returns {{ok:boolean, reason:string|null, hardIron:number[],
   *            softIron:number[], targetUT:number, coverage:number,
   *            residualFrac:number, samples:number}}
   */
  fit() {
    const n = this.samples.length;
    const fail = (reason, extra = {}) => {
      this.fitResult = {
        ok: false, reason, hardIron: [0, 0, 0], softIron: [1, 1, 1],
        targetUT: 0, coverage: 0, residualFrac: Infinity, samples: n, ...extra,
      };
      return this.fitResult;
    };
    if (n < this.minSamples) return fail('samples');

    const min = [Infinity, Infinity, Infinity];
    const max = [-Infinity, -Infinity, -Infinity];
    for (const s of this.samples) {
      for (let a = 0; a < 3; a++) {
        if (s[a] < min[a]) min[a] = s[a];
        if (s[a] > max[a]) max[a] = s[a];
      }
    }

    const center = [0, 1, 2].map((a) => (min[a] + max[a]) / 2);
    const half = [0, 1, 2].map((a) => (max[a] - min[a]) / 2);
    const target = (half[0] + half[1] + half[2]) / 3;
    if (!(target > 0)) return fail('degenerate');

    const soft = half.map((h) => clamp(target / (h || 1), this.minScale, this.maxScale));
    const coverage = Math.min(...half) / target;

    // Radius residual after correction.
    let sum2 = 0;
    for (const s of this.samples) {
      const r = magnitude([
        (s[0] - center[0]) * soft[0],
        (s[1] - center[1]) * soft[1],
        (s[2] - center[2]) * soft[2],
      ]);
      sum2 += (r - target) * (r - target);
    }
    const residualFrac = Math.sqrt(sum2 / n) / target;

    const result = {
      ok: false, reason: null, hardIron: center, softIron: soft,
      targetUT: target, coverage, residualFrac, samples: n,
    };
    if (coverage < this.minCoverage) result.reason = 'coverage';
    else if (residualFrac > this.maxResidualFrac) result.reason = 'residual';
    else result.ok = true;
    this.fitResult = result;
    return result;
  }

  /** Correct one raw sample with the current fit (or identity). */
  correct(sample, fit = this.fitResult) {
    const h = fit?.ok ? fit.hardIron : [0, 0, 0];
    const s = fit?.ok ? fit.softIron : [1, 1, 1];
    return [
      ((sample?.x ?? 0) - h[0]) * s[0],
      ((sample?.y ?? 0) - h[1]) * s[1],
      ((sample?.z ?? 0) - h[2]) * s[2],
    ];
  }

  /** Serialise the usable part for persistence (null if no good fit). */
  toJSON() {
    const f = this.fitResult;
    if (!f?.ok) return null;
    return {
      hardIron: f.hardIron, softIron: f.softIron,
      targetUT: f.targetUT, coverage: f.coverage,
      residualFrac: f.residualFrac,
    };
  }

  /** Restore a persisted fit. Returns true when it looked usable. */
  static fromJSON(json) {
    if (!json || !Array.isArray(json.hardIron) || !Array.isArray(json.softIron)) return null;
    const cal = new MagnetometerCalibration();
    cal.fitResult = {
      ok: true, reason: null,
      hardIron: json.hardIron.slice(0, 3), softIron: json.softIron.slice(0, 3),
      targetUT: json.targetUT ?? 0,
      coverage: json.coverage ?? 0,
      residualFrac: json.residualFrac ?? 0,
      samples: 0,
    };
    return cal;
  }
}

/**
 * Heading from a corrected 3-axis field and the gravity ("up") vector.
 *
 * The device's *top* edge (device +y) projected onto the horizontal plane is
 * the direction a person holding the phone is facing. We build an
 * east/north basis from the magnetic north direction and the up vector, then
 * take the bearing of the projected top edge.
 *
 * The result is relative to MAGNETIC north in a right-handed (east = +x,
 * north = +y, up = +z) frame, wrapped to (-PI, PI]. Add declination to get a
 * true bearing.
 *
 * Returns null when the attitude is unusable (field parallel to up, or the
 * phone pointing straight up/down), so the caller can keep the previous
 * heading instead of publishing nonsense.
 *
 * @param {number[]|{x:number,y:number,z:number}} mag  corrected field, uT
 * @param {{x:number,y:number,z:number}} up            world-up in device frame
 * @param {number} [declinationRad=0]                  magnetic -> true
 * @returns {{heading:number, magneticHeading:number, dipDeg:number,
 *            horizontalUT:number}|null}
 */
export function tiltCompensatedHeading(mag, up, declinationRad = 0) {
  const m = Array.isArray(mag) ? mag : [mag?.x, mag?.y, mag?.z];
  const u = [up?.x, up?.y, up?.z];
  if (!m.every(Number.isFinite) || !u.every(Number.isFinite)) return null;
  const un = magnitude(u);
  if (!(un > 1e-9)) return null;

  // East = field x up (right-handed). Tiny cross product => field vertical.
  const e = [
    m[1] * u[2] - m[2] * u[1],
    m[2] * u[0] - m[0] * u[2],
    m[0] * u[1] - m[1] * u[0],
  ];
  const en = magnitude(e);
  if (!(en > 1e-6)) return null;
  const east = [e[0] / en, e[1] / en, e[2] / en];
  // North = up x east.
  const north = [
    u[1] * east[2] - u[2] * east[1],
    u[2] * east[0] - u[0] * east[2],
    u[0] * east[1] - u[1] * east[0],
  ];
  const nn = magnitude(north);
  if (!(nn > 1e-9)) return null;
  const northN = [north[0] / nn, north[1] / nn, north[2] / nn];

  // Device top edge (+y), projected onto the horizontal plane.
  const dotUp = u[1];                              // (0,1,0) . u
  const tH = [-dotUp * u[0], 1 - dotUp * u[1], -dotUp * u[2]];
  const eastComp = tH[0] * east[0] + tH[1] * east[1] + tH[2] * east[2];
  const northComp = tH[0] * northN[0] + tH[1] * northN[1] + tH[2] * northN[2];
  if (Math.hypot(eastComp, northComp) < 1e-6) return null;

  const magneticHeading = Math.atan2(eastComp, northComp);
  const vertical = (m[0] * u[0] + m[1] * u[1] + m[2] * u[2]) / un;
  const mn = magnitude(m);
  const horizontalUT = Math.sqrt(Math.max(0, mn * mn - vertical * vertical));
  return {
    heading: wrapAngle(magneticHeading + declinationRad),
    magneticHeading,
    dipDeg: Math.atan2(-vertical, horizontalUT) / DEG,
    horizontalUT,
  };
}
