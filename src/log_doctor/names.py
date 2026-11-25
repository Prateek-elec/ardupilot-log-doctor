"""Readable names for ArduPilot flight modes and ERR subsystems."""

COPTER_MODES = {
    0: "STABILIZE", 1: "ACRO", 2: "ALT_HOLD", 3: "AUTO", 4: "GUIDED", 5: "LOITER", 6: "RTL", 7: "CIRCLE",
    9: "LAND", 11: "DRIFT", 13: "SPORT", 14: "FLIP", 15: "AUTOTUNE", 16: "POSHOLD", 17: "BRAKE",
    18: "THROW", 19: "AVOID_ADSB", 20: "GUIDED_NOGPS", 21: "SMART_RTL", 22: "FLOWHOLD", 23: "FOLLOW",
    24: "ZIGZAG", 25: "SYSTEMID", 26: "AUTOROTATE", 27: "AUTO_RTL", 28: "TURTLE",
}

PLANE_MODES = {
    0: "MANUAL", 1: "CIRCLE", 2: "STABILIZE", 3: "TRAINING", 4: "ACRO", 5: "FBWA", 6: "FBWB", 7: "CRUISE",
    8: "AUTOTUNE", 10: "AUTO", 11: "RTL", 12: "LOITER", 13: "TAKEOFF", 14: "AVOID_ADSB", 15: "GUIDED",
    17: "QSTABILIZE", 18: "QHOVER", 19: "QLOITER", 20: "QLAND", 21: "QRTL", 22: "QAUTOTUNE", 23: "QACRO",
    24: "THERMAL", 25: "LOITER_ALT_QLAND", 26: "AUTOLAND",
}

ROVER_MODES = {
    0: "MANUAL", 1: "ACRO", 3: "STEERING", 4: "HOLD", 5: "LOITER", 6: "FOLLOW", 7: "SIMPLE", 8: "DOCK",
    9: "CIRCLE", 10: "AUTO", 11: "RTL", 12: "SMART_RTL", 15: "GUIDED",
}

# ERR message subsystems (ArduCopter numbering; most are shared with the other vehicles).
ERR_SUBSYS = {
    1: "Main", 2: "Radio", 3: "Compass", 4: "Optical flow", 5: "Radio failsafe", 6: "Battery failsafe",
    7: "GPS failsafe", 8: "GCS failsafe", 9: "Fence failsafe", 10: "Flight mode change", 11: "GPS",
    12: "Crash check", 13: "Flip", 14: "AutoTune", 15: "Parachute", 16: "EKF check", 17: "EKF failsafe",
    18: "Barometer", 19: "CPU load watchdog", 20: "ADS-B failsafe", 21: "Terrain data", 22: "Navigation",
    23: "Terrain failsafe", 24: "EKF primary change", 25: "Thrust loss check", 26: "Sensor failsafe",
    27: "Leak failsafe", 28: "Pilot input", 29: "Vibration failsafe", 30: "Internal error",
    31: "Dead-reckoning failsafe",
}


def mode_name(vehicle, num):
    table = {"copter": COPTER_MODES, "plane": PLANE_MODES, "rover": ROVER_MODES}.get(vehicle, {})
    return table.get(int(num), f"mode {int(num)}")


FAILSAFES = {5, 6, 7, 8, 9, 17, 20, 23, 26, 27, 29, 31}
SENSOR_CODES = {1: "failed to initialise", 2: "failed to read", 4: "unhealthy"}
SPECIFIC_CODES = {
    11: {2: "GPS glitch"},
    12: {1: "crash detected", 2: "loss of control"},
    16: {2: "bad variance"},
    25: {1: "thrust loss detected"},
}


def err_text(subsys, code):
    s, c = int(subsys), int(code)
    name = ERR_SUBSYS.get(s, f"subsystem {s}")
    if s == 10:
        return f"{name}: mode {c} refused"
    if s == 24:
        return f"{name}: switched to EKF lane {c}"
    if c == 0:
        return f"{name}: resolved"
    if s in FAILSAFES:
        return f"{name}: triggered"
    if s in SPECIFIC_CODES and c in SPECIFIC_CODES[s]:
        return f"{name}: {SPECIFIC_CODES[s][c]}"
    if s in (3, 4, 18, 11):
        return f"{name}: {SENSOR_CODES.get(c, f'error code {c}')}"
    return f"{name}: error code {c}"
