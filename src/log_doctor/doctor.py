"""Run every health check on a log and collect the results."""

import math
from dataclasses import dataclass, field

import numpy as np

from .names import err_text, mode_name
from .reader import Log

PASS, WARN, FAIL, INFO, SKIP = "PASS", "WARN", "FAIL", "INFO", "SKIP"

# What to try when a check doesn't pass.
HINTS = {
    "vibration": "Balance or replace props, check motor and frame screws, and soft-mount the flight controller.",
    "clip_flight": "The accelerometer hit its limit in flight, which can corrupt attitude estimates. Fix vibration first: props, motor screws, FC mounting.",
    "clip_impact": "Clipping only at touchdown or launch is usually the impact itself. Fine if the landings are hard by design; otherwise land more gently.",
    "cpu": "Reduce logging (LOG_BITMASK), disable unused features, or check for a scheduler overrun in the board's latest firmware.",
    "vcc": "Unstable board power: check the BEC / power module, and power servos and peripherals from a separate supply.",
    "idle_current": "High current while disarmed: check the current sensor calibration (BATT_AMP_PERVLT / BATT_AMP_OFFSET) or a peripheral drawing power.",
    "sag": "Large voltage sag: the pack may be worn, too small (C rating) for the load, or the wiring too thin.",
    "cell": "Cells went low under load: land earlier, raise the low-battery failsafe (BATT_LOW_VOLT), or use a bigger pack.",
    "ekf": "EKF innovations are high: check GPS reception, compass calibration and interference, and vibration.",
    "compass": "Compass field changes with current: move the compass away from power wires and ESCs, or run compass-motor calibration (COMPASS_MOT).",
    "tracking": "Attitude doesn't follow the demand closely: retune (AutoTune), and check for mechanical issues first.",
    "gps": "Poor GPS: improve antenna placement (clear sky view, away from VTX and wiring), and wait for more satellites before arming.",
    "gyro_noise": "High gyro noise at rest: the IMU may be faulty or the board was moving / vibrating while disarmed.",
    "imu_agree": "The IMUs disagree: recalibrate the accelerometers; if it persists one sensor may be faulty.",
    "forced": "Arming was forced past the pre-arm checks. Find out which check failed instead of forcing.",
    "arming_check": "Pre-arm safety checks are switched off. Set ARMING_CHECK back to 1 and fix whatever check was failing.",
    "err": "The flight controller logged errors or failsafes; see the event list for which subsystem.",
}


@dataclass
class Check:
    name: str
    measured: str
    limit: str
    status: str
    hint_key: str = ""

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
    log: Log = None

    @property
    def counts(self):
        return {s: sum(c.status == s for c in self.checks) for s in (PASS, WARN, FAIL)}

    @property
    def verdict(self):
        c = self.counts
        if c[FAIL]:
            return "FIX THE FAILED CHECKS BEFORE THE NEXT FLIGHT"
        if c[WARN]:
            return "OK, with warnings to review"
        return "ALL CHECKS PASSED"


# --------------------------------------------------------------------------- helpers

def grade(val, pass_below, warn_below):
    return PASS if val < pass_below else WARN if val < warn_below else FAIL


def grade_high(val, pass_at_least, warn_at_least):
    return PASS if val >= pass_at_least else WARN if val >= warn_at_least else FAIL


def to_segments(t, mask, min_len=0.0, merge_gap=0.0):
    """[(start, end)] times where mask is true, merging short gaps and dropping short runs."""
    segs, start = [], None
    for ti, m in zip(t, mask):
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
        events = list(zip(arm["t"], arm["ArmState"]))
    else:
        ev = log.get("EV")
        events = [(t, 1 if i == 10 else 0) for t, i in zip(ev["t"], ev["Id"]) if i in (10, 11)] if ev is not None else []
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


def flying_segments(log, armed):
    """Armed and actually airborne: above 1.5 m, moving faster than 4 m/s, or (copters) motors well above idle."""
    pos, baro, gps = log.get("POS"), log.get("BARO"), log.get("GPS")
    if pos is not None and "RelHomeAlt" in pos:
        t, alt = pos["t"], pos["RelHomeAlt"]
    elif baro is not None and "Alt" in baro:
        t, alt = baro["t"], baro["Alt"]
    elif gps is not None:
        t, alt = gps["t"], np.zeros(len(gps["t"]))
    else:
        return list(armed)
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

def analyse(path):
    log = Log(path)
    armed = armed_segments(log)
    flights = flying_segments(log, armed)
    r = Report(path=path, firmware=log.firmware, vehicle=log.vehicle, board=log.board,
               duration_s=log.t_end, flights=flights, summary={}, log=log)
    has_flight = bool(flights)
    add = r.checks.append

    # ---- flight summary
    s = r.summary
    s["Flight time"] = f"{sum(b - a for a, b in flights) / 60:.1f} min in {len(flights)} flight(s)" if has_flight else "no flight detected (bench or ground log)"
    pos, gps = log.get("POS"), log.get("GPS")
    if pos is not None and "RelHomeAlt" in pos and has_flight:
        s["Max altitude above home"] = f"{pos['RelHomeAlt'][in_segments(pos['t'], flights)].max():.0f} m"
    if gps is not None and "Status" in gps:
        fix = gps["Status"] >= 3
        if fix.any() and has_flight:
            fl = fix & in_segments(gps["t"], flights)
            if fl.any():
                s["Max ground speed"] = f"{gps['Spd'][fl].max():.1f} m/s ({gps['Spd'][fl].max() * 3.6:.0f} km/h)"
                x, y = local_xy(gps["Lat"][fix], gps["Lng"][fix], gps["Lat"][fix][0], gps["Lng"][fix][0])
                s["Max distance from start"] = f"{np.hypot(x, y)[fl[fix]].max():.0f} m"
                step = np.hypot(np.diff(x), np.diff(y))
                s["Distance flown"] = f"{step[fl[fix][1:]].sum() / 1000:.2f} km"

    # ---- vibration and clipping, per IMU
    where = "in flight" if has_flight else "whole log"
    for inst in log.instances("VIBE") or [0]:
        v = log.get("VIBE", inst)
        if v is None:
            continue
        t = v["t"]
        steady = in_segments(t, flights) & ~near_transition(t, flights) if has_flight else np.ones(len(t), bool)
        if steady.any():
            vmax = np.max(np.vstack([v["VibeX"], v["VibeY"], v["VibeZ"]]), axis=0)[steady]
            p95 = float(np.percentile(vmax, 95))
            add(Check(f"Vibration IMU{inst} (95th pct, {where})", f"{p95:.1f} m/s²", "< 30 (warn < 60)", grade(p95, 30, 60), "vibration"))
        clip_field = "Clip" if "Clip" in v else f"Clip{inst}" if f"Clip{inst}" in v else None
        if clip_field:
            c = v[clip_field]
            inc = np.diff(c, prepend=c[0]) > 0
            impact = inc & near_transition(t, flights) if has_flight else np.zeros(len(t), bool)
            n_impact = int(np.sum(np.diff(c, prepend=c[0])[impact]))
            n_flight = int(c[-1] - c[0]) - n_impact
            add(Check(f"Accel clipping IMU{inst} ({where})", str(n_flight), "0", PASS if n_flight == 0 else FAIL, "clip_flight"))
            if n_impact:
                add(Check(f"Accel clipping IMU{inst} (takeoff / touchdown)", str(n_impact), "0", WARN, "clip_impact"))

    # ---- CPU / scheduler
    pm = log.get("PM")
    if pm is not None and "Load" in pm:
        load = float(np.percentile(pm["Load"], 95)) / 10.0
        add(Check("CPU load (95th pct)", f"{load:.0f} %", "< 70 % (warn < 85)", grade(load, 70, 85), "cpu"))
        nl = pm.get("NLoop", pm.get("NL"))
        if nl is not None and "NLon" in pm:
            pct = 100.0 * pm["NLon"].sum() / max(1.0, nl.sum())
            add(Check("Long loops", f"{pct:.2f} %", "< 5 % (warn < 15)", grade(pct, 5, 15), "cpu"))

    # ---- board 5 V rail
    pw = log.get("POWR")
    if pw is not None and "Vcc" in pw and np.any(pw["Vcc"] > 0.5):
        vcc = pw["Vcc"][pw["Vcc"] > 0.5]
        st = PASS if vcc.min() >= 4.8 and vcc.max() <= 5.4 else WARN if vcc.min() >= 4.6 else FAIL
        add(Check("Board 5 V rail (min / max)", f"{vcc.min():.2f} / {vcc.max():.2f} V", "4.8 – 5.4 V", st, "vcc"))

    # ---- battery
    bat = log.get("BAT")
    if bat is not None and "Volt" in bat and np.nanmax(bat["Volt"]) > 1.0:
        tb, volt, curr = bat["t"], bat["Volt"], bat.get("Curr", np.zeros(len(bat["t"])))
        disarmed = ~in_segments(tb, armed)
        if disarmed.any() and np.nanmax(curr) > 0:
            idle = float(np.median(curr[disarmed]))
            add(Check("Current while disarmed", f"{idle:.2f} A", "< 1 A (warn < 3)", grade(idle, 1, 3), "idle_current"))
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
            add(Check("Battery sag under load", f"{sag:.1f} % ({v_rest:.2f} → {v_min:.2f} V)", "< 15 % (warn < 25)", grade(sag, 15, 25), "sag"))
            cell_min = v_min / cells
            add(Check(f"Lowest cell voltage in flight ({cells}S)", f"{cell_min:.2f} V", "> 3.5 V (warn > 3.3)", grade_high(cell_min, 3.5, 3.3), "cell"))
            if np.nanmax(curr) > 0:
                add(Check("Max current in flight", f"{curr[fl].max():.1f} A", "–", INFO))

    # ---- EKF health (primary core)
    for typ in ("XKF4", "NKF4"):
        k4 = log.get(typ, 0)
        if k4 is None:
            continue
        sel = in_segments(k4["t"], armed)
        if not sel.any():
            break
        names = [("SV", "velocity"), ("SP", "position"), ("SH", "height"), ("SM", "compass")]
        present = [(f, n) for f, n in names if f in k4]
        scale = 0.01 if all(np.all(k4[f] == np.round(k4[f])) for f, _ in present) and max(k4[f].max() for f, _ in present) > 5 else 1.0
        for f, label in present:
            val = float(np.percentile(k4[f][sel], 95)) * scale
            add(Check(f"EKF {label} innovation ratio (95th pct)", f"{val:.2f}", "< 0.5 (warn < 1)", grade(val, 0.5, 1), "ekf"))
        break

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
            add(Check("Compass interference (field change with current)", f"{pct:.0f} %", "< 30 % (warn < 60)", grade(pct, 30, 60), "compass"))

    # ---- attitude tracking in flight
    att = log.get("ATT")
    if att is not None and has_flight:
        sel = in_segments(att["t"], flights) & ~near_transition(att["t"], flights)
        if sel.any():
            lim = (8, 12) if r.vehicle == "plane" else (3, 6)
            for axis in ("Roll", "Pitch"):
                e = float(np.sqrt(np.mean((att["Des" + axis][sel] - att[axis][sel]) ** 2)))
                add(Check(f"{axis} tracking error (RMS, in flight)", f"{e:.1f}°", f"< {lim[0]} (warn < {lim[1]})", grade(e, *lim), "tracking"))
            if r.vehicle == "copter":
                e = float(np.sqrt(np.mean(wrap180(att["DesYaw"][sel] - att["Yaw"][sel]) ** 2)))
                add(Check("Yaw tracking error (RMS, in flight)", f"{e:.1f}°", "< 10 (warn < 20)", grade(e, 10, 20), "tracking"))

    # ---- GPS
    if gps is not None and "Status" in gps:
        fix_pct = 100.0 * np.mean(gps["Status"] >= 3)
        add(Check("GPS 3D fix (share of log)", f"{fix_pct:.0f} %", "> 95 % (warn > 80)", grade_high(fix_pct, 95, 80), "gps"))
        fix = gps["Status"] >= 3
        if fix.any():
            n = float(np.median(gps["NSats"][fix]))
            add(Check("GPS satellites (median)", f"{n:.0f}", "≥ 10 (warn ≥ 6)", grade_high(n, 10, 6), "gps"))
            h = float(np.median(gps["HDop"][fix]))
            add(Check("GPS HDOP (median)", f"{h:.2f}", "< 1.5 (warn < 2.5)", grade(h, 1.5, 2.5), "gps"))

    # ---- IMU noise at rest, and IMU agreement
    imu0, imu1 = log.get("IMU", 0), log.get("IMU", 1)
    if imu0 is not None and "GyrX" in imu0:
        t = imu0["t"]
        rest = ~in_segments(t, armed)
        g = np.vstack([imu0["GyrX"], imu0["GyrY"], imu0["GyrZ"]]).T
        secs = np.floor(t[rest])
        best = None
        for sec in np.unique(secs):
            w = g[rest][secs == sec]
            if len(w) > 5:
                sd = float(np.max(np.std(w, axis=0)))
                best = sd if best is None else min(best, sd)
        if best is not None:
            gn = math.degrees(best)
            add(Check("Gyro noise at rest (IMU0, quietest second)", f"{gn:.3f} °/s", "< 0.3 (warn < 1)", grade(gn, 0.3, 1), "gyro_noise"))
    if imu0 is not None and imu1 is not None and "AccX" in imu0 and "AccX" in imu1:
        a0 = np.array([imu0[k].mean() for k in ("AccX", "AccY", "AccZ")])
        a1 = np.array([imu1[k].mean() for k in ("AccX", "AccY", "AccZ")])
        d = float(np.linalg.norm(a0 - a1))
        add(Check("IMU0 vs IMU1 accel (mean difference)", f"{d:.2f} m/s²", "< 0.5 (warn < 1)", grade(d, 0.5, 1), "imu_agree"))

    # ---- arming checks switched off
    if "ARMING_CHECK" in log.params:
        off = int(log.params["ARMING_CHECK"]) == 0
        add(Check("Pre-arm checks enabled (ARMING_CHECK)", "disabled (0)" if off else "enabled",
                  "enabled", FAIL if off else PASS, "arming_check"))

    # ---- forced arming
    arm = log.get("ARM")
    if arm is not None and "Forced" in arm:
        n = int(np.sum((arm["Forced"] == 1) & (arm["ArmState"] == 1)))
        add(Check("Forced arms (pre-arm checks bypassed)", str(n), "0", PASS if n == 0 else FAIL, "forced"))

    # ---- errors, failsafes and notable messages
    err = log.get("ERR")
    n_err = 0
    if err is not None:
        for t, sub, code in zip(err["t"], err["Subsys"], err["ECode"]):
            r.events.append((t, "ERR " + err_text(sub, code)))
            n_err += int(code) != 0 or int(sub) == 10
    add(Check("Errors and failsafes (ERR)", str(n_err), "0", PASS if n_err == 0 else WARN, "err"))
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
        for i, (t, num) in enumerate(zip(md["t"], nums)):
            end = md["t"][i + 1] if i + 1 < len(md["t"]) else log.t_end
            r.modes.append((t, end, mode_name(r.vehicle, num)))

    return r
