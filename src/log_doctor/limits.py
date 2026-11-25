"""Pass / warn limits for every check, and loading user overrides from a JSON file.

The defaults follow common ArduPilot guidance and suit most small and medium multirotors
and planes. Heavy-lift, racing or unusual airframes can override any of them:

    log-doctor --print-limits > my_limits.json      # edit the numbers you need
    log-doctor flight.BIN --limits my_limits.json   # partial files are fine
"""

import copy
import json

# "pass_below" / "warn_below": lower is better.  "pass_at_least" / "warn_at_least": higher is better.
DEFAULT_LIMITS = {
    "vibration_mss":            {"pass_below": 30, "warn_below": 60},
    "cpu_load_pct":             {"pass_below": 70, "warn_below": 85},
    "long_loops_pct":           {"pass_below": 5, "warn_below": 15},
    "board_vcc_v":              {"min": 4.8, "max": 5.4, "warn_min": 4.6},
    "idle_current_a":           {"pass_below": 1, "warn_below": 3},
    "battery_sag_pct":          {"pass_below": 15, "warn_below": 25},
    "cell_min_v":               {"pass_at_least": 3.5, "warn_at_least": 3.3},
    "ekf_innovation_ratio":     {"pass_below": 0.5, "warn_below": 1.0},
    "compass_interference_pct": {"pass_below": 30, "warn_below": 60},
    "tracking_copter_deg":      {"pass_below": 3, "warn_below": 6},
    "tracking_plane_deg":       {"pass_below": 8, "warn_below": 12},
    "tracking_yaw_deg":         {"pass_below": 10, "warn_below": 20},
    "gps_fix_pct":              {"pass_at_least": 95, "warn_at_least": 80},
    "gps_sats":                 {"pass_at_least": 10, "warn_at_least": 6},
    "gps_hdop":                 {"pass_below": 1.5, "warn_below": 2.5},
    "gyro_noise_dps":           {"pass_below": 0.3, "warn_below": 1.0},
    "imu_agreement_mss":        {"pass_below": 0.5, "warn_below": 1.0},
}


class LimitsError(ValueError):
    pass


def defaults():
    return copy.deepcopy(DEFAULT_LIMITS)


def merge(overrides):
    """Defaults with `overrides` applied. Unknown names are rejected so a typo can't be silently ignored."""
    lim = defaults()
    for name, values in (overrides or {}).items():
        if name.startswith("_"):
            continue  # room for comments in the JSON file
        if name not in lim:
            raise LimitsError(f"unknown limit '{name}'. Known limits: {', '.join(sorted(lim))}")
        if not isinstance(values, dict):
            raise LimitsError(f"limit '{name}' must be an object like {json.dumps(lim[name])}")
        for key, v in values.items():
            if key not in lim[name]:
                raise LimitsError(f"limit '{name}' has no field '{key}' (fields: {', '.join(lim[name])})")
            if not isinstance(v, (int, float)) or isinstance(v, bool):
                raise LimitsError(f"limit '{name}.{key}' must be a number, got {v!r}")
            lim[name][key] = float(v)
    for name, v in lim.items():
        if "pass_below" in v and v["pass_below"] > v["warn_below"]:
            raise LimitsError(f"limit '{name}': pass_below must not exceed warn_below")
        if "pass_at_least" in v and v["pass_at_least"] < v["warn_at_least"]:
            raise LimitsError(f"limit '{name}': pass_at_least must not be below warn_at_least")
    return lim


def load(path):
    try:
        with open(path, encoding="utf-8") as f:
            data = json.load(f)
    except OSError as e:
        raise LimitsError(f"cannot read limits file {path}: {e.strerror}") from e
    except json.JSONDecodeError as e:
        raise LimitsError(f"limits file {path} is not valid JSON: {e}") from e
    if not isinstance(data, dict):
        raise LimitsError(f"limits file {path} must contain a JSON object")
    return merge(data)


def describe(lim):
    """Human text for a limit, e.g. '< 30 (warn < 60)'."""
    if "pass_below" in lim:
        return f"< {_n(lim['pass_below'])} (warn < {_n(lim['warn_below'])})"
    if "pass_at_least" in lim:
        return f"≥ {_n(lim['pass_at_least'])} (warn ≥ {_n(lim['warn_at_least'])})"
    return f"{_n(lim['min'])} – {_n(lim['max'])}"


def _n(x):
    return f"{x:g}"
