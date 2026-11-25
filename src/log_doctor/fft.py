"""Gyro noise spectrum and harmonic-notch suggestions.

ArduPilot copters filter motor noise with a harmonic notch (INS_HNTCH_*). Setting it needs the
frequency of the noise, which only shows up in high-rate gyro data:

  1. the batch sampler (ISBH / ISBD messages, enabled with INS_LOG_BAT_MASK = 1), or
  2. IMU messages logged at 400 Hz or faster (fast IMU logging).

This module finds the dominant noise peak in flight, checks it against the Nyquist limit, and
turns it into suggested INS_HNTCH_* values following the ArduPilot tuning guide.
"""

from dataclasses import dataclass, field

import numpy as np

MIN_RATE_HZ = 300.0       # below this (Nyquist 150 Hz) the motor-noise band can't be seen
MIN_PEAK_HZ = 40.0        # ignore flight dynamics; motor noise sits higher than this


@dataclass
class Spectrum:
    source: str                       # "batch sampler" or "IMU log"
    rate_hz: float
    freqs: np.ndarray
    psd: np.ndarray                   # summed over x/y/z, linear
    peak_hz: float = None
    peak_db: float = None             # peak above the median noise floor
    harmonic_hz: float = None         # second peak near 2 x fundamental, if present
    hover_throttle: float = None
    notes: list = field(default_factory=list)

    @property
    def nyquist(self):
        return self.rate_hz / 2.0


def _welch(x, fs, nseg=256):
    """Mean periodogram over 50 %-overlapping Hann-windowed segments (numpy only)."""
    x = np.asarray(x, float)
    x = x - np.mean(x)
    n = min(nseg, len(x))
    if n < 32:
        return None, None
    step = n // 2
    win = np.hanning(n)
    scale = 1.0 / (fs * (win ** 2).sum())
    acc, count = None, 0
    for i in range(0, len(x) - n + 1, step):
        seg = x[i:i + n] * win
        p = np.abs(np.fft.rfft(seg)) ** 2 * scale
        acc = p if acc is None else acc + p
        count += 1
    if not count:
        return None, None
    return np.fft.rfftfreq(n, 1.0 / fs), acc / count


def _from_batch(log, flights):
    """Gyro batches (ISBH type 1, instance 0) inside flight segments -> list of (rate, x, y, z)."""
    hdr, dat = log.get("ISBH"), log.get("ISBD")
    if hdr is None or dat is None or "N" not in hdr or "N" not in dat:
        return []
    batches = []
    for i in range(len(hdr["N"])):
        if int(hdr.get("type", np.zeros(len(hdr["N"])))[i]) != 1 or int(hdr.get("instance", np.zeros(len(hdr["N"])))[i]) != 0:
            continue
        t = hdr["t"][i]
        if flights and not any(a <= t <= b for a, b in flights):
            continue
        sel = dat["N"] == hdr["N"][i]
        if not sel.any():
            continue
        mul = float(hdr.get("mul", np.ones(len(hdr["N"])))[i]) or 1.0
        rate = float(hdr.get("smp_rate", np.zeros(len(hdr["N"])))[i])
        order = np.argsort(dat.get("seqno", np.arange(len(sel)))[sel])
        xs = np.concatenate([np.asarray(v, float) for v in dat["x"][sel][order]]) / mul
        ys = np.concatenate([np.asarray(v, float) for v in dat["y"][sel][order]]) / mul
        zs = np.concatenate([np.asarray(v, float) for v in dat["z"][sel][order]]) / mul
        if rate > 0 and len(xs) >= 64:
            batches.append((rate, xs, ys, zs))
    return batches


def analyse(log, flights):
    """Return a Spectrum, or (None, reason) when the log can't show motor noise."""
    batches = _from_batch(log, flights)
    if batches:
        rate = float(np.median([b[0] for b in batches]))
        psd_sum, freqs = None, None
        for _, x, y, z in batches:
            for axis in (x, y, z):
                f, p = _welch(axis, rate)
                if f is None:
                    continue
                if freqs is None:
                    freqs, psd_sum = f, p
                elif len(f) == len(freqs):
                    psd_sum = psd_sum + p
        if freqs is None:
            return None, "the batch-sampler data is too short for a spectrum"
        spec = Spectrum("batch sampler", rate, freqs, psd_sum)
    else:
        imu = log.get("IMU", 0)
        if imu is None or "GyrX" not in imu or len(imu["t"]) < 64:
            return None, "no high-rate gyro data: set INS_LOG_BAT_MASK = 1 (batch sampler) and fly again"
        t = imu["t"]
        rate = 1.0 / float(np.median(np.diff(t)))
        if rate < MIN_RATE_HZ:
            return None, (f"gyro is logged at {rate:.0f} Hz, too slow to see motor noise: set INS_LOG_BAT_MASK = 1 "
                          "(batch sampler) or enable fast IMU logging, and fly again")
        sel = np.zeros(len(t), bool)
        for a, b in flights:
            sel |= (t >= a + 5) & (t <= b - 5)        # steady flight, away from takeoff and landing
        if sel.sum() < 512:
            return None, "not enough steady flight for a spectrum"
        psd_sum, freqs = None, None
        for k in ("GyrX", "GyrY", "GyrZ"):
            # contiguous chunks only, so gaps between flights don't smear the spectrum
            idx = np.flatnonzero(sel)
            breaks = np.flatnonzero(np.diff(idx) > 1)
            for chunk in np.split(idx, breaks + 1):
                if len(chunk) < 256:
                    continue
                f, p = _welch(imu[k][chunk], rate)
                if f is None:
                    continue
                if freqs is None:
                    freqs, psd_sum = f, p
                else:
                    psd_sum = psd_sum + p
        if freqs is None:
            return None, "not enough steady flight for a spectrum"
        spec = Spectrum("IMU log", rate, freqs, psd_sum)

    # dominant peak above the flight-dynamics band
    band = spec.freqs >= MIN_PEAK_HZ
    if not band.any():
        return None, "sample rate too low for the motor-noise band"
    floor = float(np.median(spec.psd[band]))
    i = int(np.argmax(np.where(band, spec.psd, 0)))
    spec.peak_hz = float(spec.freqs[i])
    spec.peak_db = float(10 * np.log10(spec.psd[i] / max(floor, 1e-30)))
    # second harmonic: a clear peak within +-15 % of 2 x fundamental
    h = (spec.freqs > 1.7 * spec.peak_hz) & (spec.freqs < 2.3 * spec.peak_hz)
    if h.any():
        j = int(np.argmax(np.where(h, spec.psd, 0)))
        if 10 * np.log10(spec.psd[j] / max(floor, 1e-30)) > 6:
            spec.harmonic_hz = float(spec.freqs[j])
    if spec.peak_hz > 0.8 * spec.nyquist:
        spec.notes.append(f"the peak is close to the {spec.nyquist:.0f} Hz Nyquist limit, so the real noise may be "
                          "higher and folded down (aliased); log with the batch sampler to be sure")

    hover = log.params.get("MOT_THST_HOVER")
    if hover:
        spec.hover_throttle = float(hover)
    return spec, None


def notch_suggestion(spec, log):
    """Suggested INS_HNTCH_* parameters (dict) from a spectrum, per the ArduPilot tuning guide."""
    esc = log.get("ESC")
    has_rpm = esc is not None and "RPM" in esc and np.nanmax(esc["RPM"]) > 0
    # Throttle mode (1): FREQ is the noise frequency at the reference (hover) throttle, so centre it on the peak.
    # ESC RPM mode (3): FREQ is the lowest frequency the notch may track, so put it below the hover peak.
    freq = round(spec.peak_hz * (0.8 if has_rpm else 1.0))
    s = {
        "INS_HNTCH_ENABLE": 1,
        "INS_HNTCH_MODE": 3 if has_rpm else 1,      # 3 = ESC telemetry RPM, 1 = throttle based
        "INS_HNTCH_FREQ": max(freq, 40),
        "INS_HNTCH_BW": max(round(spec.peak_hz / 2), 20),
        "INS_HNTCH_ATT": 40,
        "INS_HNTCH_HMNCS": 3 if spec.harmonic_hz else 1,   # bitmask: 1 = fundamental, 2 = second harmonic
    }
    if not has_rpm:
        s["INS_HNTCH_REF"] = round(spec.hover_throttle, 3) if spec.hover_throttle else "MOT_THST_HOVER"
    else:
        s["INS_HNTCH_REF"] = 1
    return s
