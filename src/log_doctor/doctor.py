"""Run every health check on a log and collect the results."""

import math
from dataclasses import dataclass, field

import numpy as np

from . import fft as fft_mod
from . import limits as lim_mod
from .names import err_text, mode_name
from .reader import Log

PASS, WARN, FAIL, INFO = "PASS", "WARN", "FAIL", "INFO"
SEVERITY = {FAIL: 0, WARN: 1, INFO: 2, PASS: 3}

# What to try when a check doesn't pass.
HINTS = {
    "vibration": "Balance or replace props, check motor and frame screws, and soft-mount the flight controller.",
    "clip_flight": "The accelerometer hit its limit in flight, which can corrupt attitude estimates. Fix vibration first: props, motor screws, FC mounting.",
    "clip_impact": "Clipping only at touchdown or launch is usually the impact itself. Fine if the landings are hard by design; otherwise land more gently.",
    "log_end": "The log stops while the aircraft is flying: typical of a crash, a power loss or a brown-out. Check the last seconds of the log, the battery connector and the power module.",
    "cpu": "Reduce logging (LOG_BITMASK), disable unused features, or check for a scheduler overrun in the board's latest firmware.",
    "vcc": "Unstable board power: check the BEC / power module, and power servos and peripherals from a separate supply.",
    "idle_current": "High current while disarmed: check the current sensor calibration (BATT_AMP_PERVLT / BATT_AMP_OFFSET) or a peripheral drawing power.",
    "sag": "Large voltage sag: the pack may be worn, too small (C rating) for the load, or the wiring too thin.",
    "cell": "Cells went low under load: land earlier, raise the low-battery failsafe (BATT_LOW_VOLT), or use a bigger pack.",
    "ekf": "EKF innovations are high: check GPS reception, compass calibration and interference, and vibration.",
    "compass": "Compass field changes with current: move the compass away from power wires and ESCs, or run compass-motor calibration (COMPASS_MOT).",
    "tracking": "Attitude doesn't follow the demand closely: check for mechanical issues first, then retune (AutoTune).",
    "gps": "Poor GPS: improve antenna placement (clear sky view, away from VTX and wiring), and wait for more satellites before arming.",
    "gyro_noise": "High gyro noise at rest: the IMU may be faulty, or the board was moving or vibrating while disarmed.",
    "imu_agree": "The IMUs disagree: recalibrate the accelerometers; if it persists, one sensor may be faulty.",
    "forced": "Arming was forced past the pre-arm checks. Find out which check failed instead of forcing.",
    "arming_check": "Pre-arm safety checks are switched off. Set ARMING_CHECK back to 1 and fix whatever check was failing.",
    "err": "The flight controller logged errors or failsafes; see the event list for which subsystem.",
    "notch": "Motor noise isn't being filtered. Set up the harmonic notch: use the suggested values in this report, or if there are none, set INS_LOG_BAT_MASK = 1, fly a 1-minute hover and run Log Doctor on that log. A tuned notch lets the gyro filters open up and usually improves the tune noticeably.",
    "notch_freq": "The notch is set well away from the noise peak in this log. Re-check INS_HNTCH_FREQ / REF against the suggested values.",
    "corrupt": "Part of the log is damaged. Use a good-quality SD card, and check for power dips that interrupt logging.",
}


@dataclass
class Check:
    name: str
    measured: str
    limit: str
    status: str
    hint_key: str = ""
    value: float = None        # the raw number behind `measured`, for JSON / scripting

    @property
    def hint(self):
        return HINTS.get(self.hint_key, "") if self.status in (WARN, FAIL) else ""


@dataclass
class Report:
    path: str
    firmware: str
    vehicle: str
    board: str
    duration_s: float
    flights: list                       # [(start, end)] seconds
    summary: dict
    checks: list = field(default_factory=list)
    modes: list = field(default_factory=list)       # [(start, end, name)]
    events: list = field(default_factory=list)      # [(t, text)]
    not_checked: list = field(default_factory=list)  # [(area, reason)]
    spectrum: object = None                          # fft.Spectrum when the log had high-rate gyro data
    notch: dict = None                               # suggested INS_HNTCH_* values
    log: Log = None

    @property
    def counts(self):
        return {s: sum(c.status == s for c in self.checks) for s in (PASS, WARN, FAIL)}

    @property
    def complete(self):
        """Enough data to judge the aircraft: vibration and attitude must both be in the log."""
        names = {area for area, _ in self.not_checked}
        return not {"Vibration", "Attitude"} <= names

    @property
    def verdict(self):
        c = self.counts
        if c[FAIL]:
            return "FIX THE FAILED CHECKS BEFORE THE NEXT FLIGHT"
        if not self.complete:
            return "INCOMPLETE: this log has too little data to judge the aircraft"
        if c[WARN]:
            return "OK, with warnings to review"
        return "ALL CHECKS PASSED"

    @property
    def status(self):
        c = self.counts
        return FAIL if c[FAIL] else "INCOMPLETE" if not self.complete else WARN if c[WARN] else PASS

    def hints_by_severity(self):
        """(check, hint) for every warning and failure, failures first, each hint once."""
        seen, out = set(), []
        for c in sorted(self.checks, key=lambda c: SEVERITY[c.status]):
            if c.hint and c.hint not in seen:
                seen.add(c.hint)
                out.append((c, c.hint))
        return out


# --------------------------------------------------------------------------- helpers

def grade(val, pass_below, warn_below):
    return PASS if val < pass_below else WARN if val < warn_below else FAIL


def grade_high(val, pass_at_least, warn_at_least):
    return PASS if val >= pass_at_least else WARN if val >= warn_at_least else FAIL


def graded(value, limit):
    """Grade against a limit entry from limits.py."""
    if "pass_below" in limit:
        return grade(value, limit["pass_below"], limit["warn_below"])
    return grade_high(value, limit["pass_at_least"], limit["warn_at_least"])


def to_segments(t, mask, min_len=0.0, merge_gap=0.0):
    """[(start, end)] times where mask is true, merging short gaps and dropping short runs."""
    segs, start = [], None
    for ti, m in zip(t, mask, strict=False):
        if m and start is None:
            start = ti
        elif not m and start is not None:
            segs.append([start, ti])
            start = None
    if start is not None:
        segs.append([start, t[-1]])
    merged = []
    for s in segs:
        if merged and s[0] - merged[-1][1] <= merge_gap:
            merged[-1][1] = s[1]
        else:
            merged.append(s)
    return [(a, b) for a, b in merged if b - a >= min_len]


def in_segments(t, segs, pad=0.0):
    sel = np.zeros(len(t), dtype=bool)
    for a, b in segs:
        sel |= (t >= a + pad) & (t <= b - pad)
    return sel


def armed_segments(log):
    arm = log.get("ARM")
    if arm is not None and "ArmState" in arm:
        events = list(zip(arm["t"], arm["ArmState"], strict=False))
    else:
        ev = log.get("EV")
        events = [(t, 1 if i == 10 else 0) for t, i in zip(ev["t"], ev["Id"], strict=False) if i in (10, 11)] if ev is not None else []
    if not events:
        return [(0.0, log.t_end)]
    segs, start = [], None
    if events[0][1] == 0:                # log began while already armed
        start = 0.0
    for t, state in events:
        if state and start is None:
            start = t
        elif not state and start is not None:
            segs.append((start, t))
            start = None
    if start is not None:
        segs.append((start, log.t_end))
    return segs


def altitude_series(log):
    pos, baro, gps = log.get("POS"), log.get("BARO"), log.get("GPS")
    if pos is not None and "RelHomeAlt" in pos:
        return pos["t"], pos["RelHomeAlt"]
    if baro is not None and "Alt" in baro:
        return baro["t"], baro["Alt"]
    if gps is not None:
        return gps["t"], np.zeros(len(gps["t"]))
    return None, None


def flying_segments(log, armed):
    """Armed and actually airborne: above 1.5 m, moving faster than 4 m/s, or (copters) motors well above idle."""
    t, alt = altitude_series(log)
    if t is None:
        return list(armed)
    gps = log.get("GPS")
    airborne = alt > 1.5
    if gps is not None and "Spd" in gps:
        airborne |= np.interp(t, gps["t"], gps["Spd"]) > 4.0
    rc = log.get("RCOU")
    if log.vehicle == "copter" and rc is not None:
        chans = [rc[f"C{i}"] for i in range(1, 15) if f"C{i}" in rc and np.ptp(rc[f"C{i}"]) > 150]
        if chans:
            m = np.mean(chans, axis=0)
            airborne |= np.interp(t, rc["t"], m) > m.min() + 250
    airborne &= in_segments(t, armed)
    return to_segments(t, airborne, min_len=3.0, merge_gap=3.0)


def near_transition(t, flights, before=2.0, after=4.0):
    """True for times within a few seconds of a takeoff or touchdown."""
    sel = np.zeros(len(t), dtype=bool)
    for a, b in flights:
        sel |= (t >= a - before) & (t <= a + after)
        sel |= (t >= b - before) & (t <= b + after)
    return sel


def wrap180(d):
    return (d + 180.0) % 360.0 - 180.0


def local_xy(lat, lon, lat0, lon0):
    x = (lon - lon0) * 111320.0 * math.cos(math.radians(lat0))
    y = (lat - lat0) * 111320.0
    return x, y


# --------------------------------------------------------------------------- analysis

def analyse(path, limits=None):
    """Analyse one DataFlash log.

    `limits` is a dict from limits.merge() / limits.load(); None uses the defaults.
    Raises reader.LogError when the file can't be analysed.
    """
    L = limits or lim_mod.defaults()
    log = Log(path)
    armed = armed_segments(log)
    flights = flying_segments(log, armed)
    r = Report(path=path, firmware=log.firmware, vehicle=log.vehicle, board=log.board,
               duration_s=log.t_end, flights=flights, summary={}, log=log)
    has_flight = bool(flights)

    def add(name, measured, limit_key_or_text, status, hint="", value=None):
        text = lim_mod.describe(L[limit_key_or_text]) if limit_key_or_text in L else limit_key_or_text
        r.checks.append(Check(name, measured, text, status, hint, value))

    def skip(area, reason):
        r.not_checked.append((area, reason))

    # ---- flight summary
    s = r.summary
    s["Flight time"] = (f"{sum(b - a for a, b in flights) / 60:.1f} min in {len(flights)} flight(s)"
                        if has_flight else "no flight detected (bench or ground log)")
    pos, gps = log.get("POS"), log.get("GPS")
    if pos is not None and "RelHomeAlt" in pos and has_flight:
        s["Max altitude above home"] = f"{pos['RelHomeAlt'][in_segments(pos['t'], flights)].max():.0f} m"
    if gps is not None and "Status" in gps and has_flight:
        fix = gps["Status"] >= 3
        fl = fix & in_segments(gps["t"], flights)
        if fl.any():
            s["Max ground speed"] = f"{gps['Spd'][fl].max():.1f} m/s ({gps['Spd'][fl].max() * 3.6:.0f} km/h)"
            x, y = local_xy(gps["Lat"][fix], gps["Lng"][fix], gps["Lat"][fix][0], gps["Lng"][fix][0])
            s["Max distance from start"] = f"{np.hypot(x, y)[fl[fix]].max():.0f} m"
            step = np.hypot(np.diff(x), np.diff(y))
            s["Distance flown"] = f"{step[fl[fix][1:]].sum() / 1000:.2f} km"

    # ---- log integrity
    if log.corrupt_records:
        add("Damaged log records", str(log.corrupt_records), "0", WARN, "corrupt", log.corrupt_records)
    arm_end = armed and armed[-1][1] >= log.t_end - 0.5
    if has_flight and arm_end and flights[-1][1] >= log.t_end - 2.0:
        t_alt, alt = altitude_series(log)
        last_alt = float(alt[-1]) if alt is not None and len(alt) else float("nan")
        add("Log ends while flying", f"armed, {last_alt:.0f} m up at the last record", "lands and disarms", FAIL, "log_end", last_alt)

    # ---- vibration and clipping, per IMU
    where = "in flight" if has_flight else "whole log"
    vibe_insts = log.instances("VIBE")
    if not vibe_insts:
        skip("Vibration", "no VIBE messages (enable IMU logging in LOG_BITMASK)")
    for inst in vibe_insts:
        v = log.get("VIBE", inst)
        t = v["t"]
        steady = in_segments(t, flights) & ~near_transition(t, flights) if has_flight else np.ones(len(t), bool)
        if steady.any():
            vmax = np.max(np.vstack([v["VibeX"], v["VibeY"], v["VibeZ"]]), axis=0)[steady]
            p95 = float(np.percentile(vmax, 95))
            add(f"Vibration IMU{inst} (95th pct, {where})", f"{p95:.1f} m/s²", "vibration_mss",
                graded(p95, L["vibration_mss"]), "vibration", p95)
        clip_field = "Clip" if "Clip" in v else f"Clip{inst}" if f"Clip{inst}" in v else None
        if clip_field:
            c = v[clip_field]
            d = np.diff(c, prepend=c[0])
            d[d < 0] = 0                                   # counter reset
            impact = (d > 0) & near_transition(t, flights) if has_flight else np.zeros(len(t), bool)
            n_impact = int(d[impact].sum())
            n_flight = int(d.sum()) - n_impact
            add(f"Accel clipping IMU{inst} ({where})", str(n_flight), "0", PASS if n_flight == 0 else FAIL, "clip_flight", n_flight)
            if n_impact:
                add(f"Accel clipping IMU{inst} (takeoff / touchdown)", str(n_impact), "0", WARN, "clip_impact", n_impact)

    # ---- motor-noise spectrum and harmonic notch (multirotors)
    if r.vehicle == "copter":
        enabled = log.params.get("INS_HNTCH_ENABLE")
        if has_flight:
            spec, why = fft_mod.analyse(log, flights)
            if spec is None:
                skip("Vibration spectrum (FFT)", why)
            else:
                r.spectrum = spec
                text = f"{spec.peak_hz:.0f} Hz, {spec.peak_db:.0f} dB above floor"
                if spec.harmonic_hz:
                    text += f"; harmonic {spec.harmonic_hz:.0f} Hz"
                add(f"Gyro noise peak ({spec.source}, {spec.rate_hz:.0f} Hz)", text, "–", INFO, value=spec.peak_hz)
                r.notch = fft_mod.notch_suggestion(spec, log)
                for note in spec.notes:
                    r.events.append((0.0, "FFT: " + note))
        if enabled is not None:
            if int(enabled) == 0:
                add("Harmonic notch filter (INS_HNTCH_ENABLE)", "disabled", "enabled", WARN, "notch")
            else:
                f0 = log.params.get("INS_HNTCH_FREQ")
                measured = f"enabled, {f0:.0f} Hz" if f0 else "enabled"
                st = PASS
                if (f0 and r.spectrum is not None and r.spectrum.peak_db > 10
                        and not (0.5 * r.spectrum.peak_hz <= f0 <= 1.2 * r.spectrum.peak_hz)):
                    st = WARN
                add("Harmonic notch filter (INS_HNTCH_ENABLE)", measured, "enabled, near the noise peak", st, "notch_freq")

    # ---- CPU / scheduler
    pm = log.get("PM")
    if pm is not None and "Load" in pm:
        load = float(np.percentile(pm["Load"], 95)) / 10.0
        add("CPU load (95th pct)", f"{load:.0f} %", "cpu_load_pct", graded(load, L["cpu_load_pct"]), "cpu", load)
        nl = pm.get("NLoop", pm.get("NL"))
        if nl is not None and "NLon" in pm:
            pct = 100.0 * pm["NLon"].sum() / max(1.0, nl.sum())
            add("Long loops", f"{pct:.2f} %", "long_loops_pct", graded(pct, L["long_loops_pct"]), "cpu", pct)

    # ---- board 5 V rail
    pw = log.get("POWR")
    if pw is not None and "Vcc" in pw and np.any(pw["Vcc"] > 0.5):
        vcc = pw["Vcc"][pw["Vcc"] > 0.5]
        lv = L["board_vcc_v"]
        st = PASS if lv["min"] <= vcc.min() and vcc.max() <= lv["max"] else WARN if vcc.min() >= lv["warn_min"] else FAIL
        add("Board 5 V rail (min / max)", f"{vcc.min():.2f} / {vcc.max():.2f} V", "board_vcc_v", st, "vcc", float(vcc.min()))

    # ---- battery
    bat = log.get("BAT")
    if bat is None or "Volt" not in bat or np.nanmax(bat["Volt"]) <= 1.0:
        skip("Battery", "no battery monitor data (BATT_MONITOR not set)")
    else:
        tb, volt = bat["t"], bat["Volt"]
        curr = bat.get("Curr", np.zeros(len(tb)))
        has_curr = np.nanmax(curr) > 0
        disarmed = ~in_segments(tb, armed)
        if disarmed.any() and has_curr:
            idle = float(np.median(curr[disarmed]))
            add("Current while disarmed", f"{idle:.2f} A", "idle_current_a", graded(idle, L["idle_current_a"]), "idle_current", idle)
        cells = max(1, math.ceil(np.nanmax(volt) / 4.25))
        s["Battery"] = f"{cells}S, {np.nanmax(volt):.2f} V at start"
        if "CurrTot" in bat and np.nanmax(bat["CurrTot"]) > 0:
            s["Battery"] += f", {bat['CurrTot'][-1] - bat['CurrTot'][0]:.0f} mAh used"
        if has_flight:
            fl = in_segments(tb, flights)
            rest = ~fl
            v_rest = float(np.median(volt[rest])) if rest.any() else float(np.max(volt))
            v_min = float(volt[fl].min())
            sag = 100.0 * (v_rest - v_min) / max(v_rest, 1e-6)
            add("Battery sag under load", f"{sag:.1f} % ({v_rest:.2f} → {v_min:.2f} V)", "battery_sag_pct",
                graded(sag, L["battery_sag_pct"]), "sag", sag)
            cell_min = v_min / cells
            add(f"Lowest cell voltage in flight ({cells}S)", f"{cell_min:.2f} V", "cell_min_v",
                graded(cell_min, L["cell_min_v"]), "cell", cell_min)
            if has_curr:
                add("Max current in flight", f"{curr[fl].max():.1f} A", "–", INFO, value=float(curr[fl].max()))

    # ---- EKF health (primary core)
    k4 = log.get("XKF4", 0) or log.get("NKF4", 0)
    if k4 is None:
        skip("EKF", "no XKF4 / NKF4 messages")
    else:
        sel = in_segments(k4["t"], armed)
        names = [("SV", "velocity"), ("SP", "position"), ("SH", "height"), ("SM", "compass")]
        present = [(f, n) for f, n in names if f in k4]
        if sel.any() and present:
            ints = all(np.all(k4[f] == np.round(k4[f])) for f, _ in present)
            scale = 0.01 if ints and max(k4[f].max() for f, _ in present) > 5 else 1.0   # pre-4.0 logs store x100
            for f, label in present:
                val = float(np.percentile(k4[f][sel], 95)) * scale
                add(f"EKF {label} innovation ratio (95th pct)", f"{val:.2f}", "ekf_innovation_ratio",
                    graded(val, L["ekf_innovation_ratio"]), "ekf", val)

    # ---- compass interference vs current
    mag = log.get("MAG")
    if mag is not None and bat is not None and "Curr" in bat and np.nanmax(bat["Curr"]) > 0:
        field_s = np.sqrt(mag["MagX"] ** 2 + mag["MagY"] ** 2 + mag["MagZ"] ** 2)
        cur = np.interp(mag["t"], bat["t"], bat["Curr"], left=np.nan, right=np.nan)
        ok = ~np.isnan(cur)
        if ok.any() and np.ptp(cur[ok]) >= 3:
            lo = field_s[ok & (cur <= np.percentile(cur[ok], 10))]
            hi = field_s[ok & (cur >= np.percentile(cur[ok], 90))]
            pct = 100.0 * abs(hi.mean() - lo.mean()) / max(lo.mean(), 1e-6)
            add("Compass interference (field change with current)", f"{pct:.0f} %", "compass_interference_pct",
                graded(pct, L["compass_interference_pct"]), "compass", pct)
        else:
            skip("Compass interference", "current never rose by 3 A, so motor interference can't be measured")

    # ---- attitude tracking in flight
    att = log.get("ATT")
    if att is None:
        skip("Attitude", "no ATT messages")
    elif not has_flight:
        skip("Tuning", "no flight in this log")
    else:
        sel = in_segments(att["t"], flights) & ~near_transition(att["t"], flights)
        if sel.any():
            key = "tracking_plane_deg" if r.vehicle == "plane" else "tracking_copter_deg"
            for axis in ("Roll", "Pitch"):
                e = float(np.sqrt(np.mean((att["Des" + axis][sel] - att[axis][sel]) ** 2)))
                add(f"{axis} tracking error (RMS, in flight)", f"{e:.1f}°", key, graded(e, L[key]), "tracking", e)
            if r.vehicle == "copter":
                e = float(np.sqrt(np.mean(wrap180(att["DesYaw"][sel] - att["Yaw"][sel]) ** 2)))
                add("Yaw tracking error (RMS, in flight)", f"{e:.1f}°", "tracking_yaw_deg",
                    graded(e, L["tracking_yaw_deg"]), "tracking", e)

    # ---- GPS
    if gps is None or "Status" not in gps:
        skip("GPS", "no GPS messages")
    else:
        fix_pct = 100.0 * np.mean(gps["Status"] >= 3)
        add("GPS 3D fix (share of log)", f"{fix_pct:.0f} %", "gps_fix_pct", graded(fix_pct, L["gps_fix_pct"]), "gps", fix_pct)
        fix = gps["Status"] >= 3
        if fix.any():
            n = float(np.median(gps["NSats"][fix]))
            add("GPS satellites (median)", f"{n:.0f}", "gps_sats", graded(n, L["gps_sats"]), "gps", n)
            h = float(np.median(gps["HDop"][fix]))
            add("GPS HDOP (median)", f"{h:.2f}", "gps_hdop", graded(h, L["gps_hdop"]), "gps", h)

    # ---- IMU noise at rest, and IMU agreement
    imu0, imu1 = log.get("IMU", 0), log.get("IMU", 1)
    if imu0 is not None and "GyrX" in imu0:
        t = imu0["t"]
        rest = ~in_segments(t, armed)
        g = np.vstack([imu0["GyrX"], imu0["GyrY"], imu0["GyrZ"]]).T[rest]
        secs = np.floor(t[rest])
        best = None
        for sec in np.unique(secs):
            w = g[secs == sec]
            if len(w) > 5:
                sd = float(np.max(np.std(w, axis=0)))
                best = sd if best is None else min(best, sd)
        if best is not None:
            gn = math.degrees(best)
            add("Gyro noise at rest (IMU0, quietest second)", f"{gn:.3f} °/s", "gyro_noise_dps",
                graded(gn, L["gyro_noise_dps"]), "gyro_noise", gn)
    if imu0 is not None and imu1 is not None and "AccX" in imu0 and "AccX" in imu1:
        a0 = np.array([imu0[k].mean() for k in ("AccX", "AccY", "AccZ")])
        a1 = np.array([imu1[k].mean() for k in ("AccX", "AccY", "AccZ")])
        d = float(np.linalg.norm(a0 - a1))
        add("IMU0 vs IMU1 accel (mean difference)", f"{d:.2f} m/s²", "imu_agreement_mss",
            graded(d, L["imu_agreement_mss"]), "imu_agree", d)

    # ---- arming safety
    if "ARMING_CHECK" in log.params:
        off = int(log.params["ARMING_CHECK"]) == 0
        add("Pre-arm checks enabled (ARMING_CHECK)", "disabled (0)" if off else "enabled", "enabled",
            FAIL if off else PASS, "arming_check")
    arm = log.get("ARM")
    if arm is not None and "Forced" in arm:
        n = int(np.sum((arm["Forced"] == 1) & (arm["ArmState"] == 1)))
        add("Forced arms (pre-arm checks bypassed)", str(n), "0", PASS if n == 0 else FAIL, "forced", n)

    # ---- errors, failsafes and notable messages
    err = log.get("ERR")
    n_err = 0
    if err is not None:
        for t, sub, code in zip(err["t"], err["Subsys"], err["ECode"], strict=False):
            r.events.append((t, "ERR " + err_text(sub, code)))
            n_err += int(code) != 0 or int(sub) == 10
    add("Errors and failsafes (ERR)", str(n_err), "0", PASS if n_err == 0 else WARN, "err", n_err)
    keys = ("failsafe", "crash", "ekf", "vibe", "vibration", "clip", "battery", "thrust", "glitch", "lane switch",
            "switched to lane", "compass", "error", "fence breach", "breach", "radio", "lost", "potential",
            "parachute", "emergency", "fail")
    ignore = ("prearm", "arm:", "new fence", "new mission", "new rally", "ardu", "chibios", "param space")
    seen = {}
    for t, text in log.texts:
        low = text.lower()
        if any(k in low for k in keys) and not low.startswith(ignore):
            if text in seen:
                seen[text][1] += 1
            else:
                seen[text] = [t, 1]
    for text, (t, n) in seen.items():
        r.events.append((t, text if n == 1 else f"{text} (×{n})"))
    r.events.sort(key=lambda e: e[0])

    # ---- flight modes timeline
    md = log.get("MODE")
    if md is not None:
        nums = md.get("ModeNum", md.get("Mode"))
        for i, (t, num) in enumerate(zip(md["t"], nums, strict=False)):
            end = md["t"][i + 1] if i + 1 < len(md["t"]) else log.t_end
            r.modes.append((t, end, mode_name(r.vehicle, num)))

    return r
