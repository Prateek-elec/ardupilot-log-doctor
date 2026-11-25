"""Write small synthetic ArduPilot DataFlash (.BIN) logs for tests.

Real logs carry people's locations and hardware IDs, so the test suite builds its own:
a copter or plane that arms, takes off, flies a circle, lands and disarms, with optional
faults injected so every check can be shown to catch what it should.
"""

import math
import struct

HEAD = b"\xa3\x95"
FMT_ID = 128

# format char -> (struct code, scale applied on write)
CODES = {"Q": ("Q", 1), "q": ("q", 1), "f": ("f", 1), "d": ("d", 1), "B": ("B", 1), "b": ("b", 1),
         "H": ("H", 1), "h": ("h", 1), "I": ("I", 1), "i": ("i", 1), "M": ("B", 1),
         "n": ("4s", 1), "N": ("16s", 1), "Z": ("64s", 1), "L": ("i", 1e7)}

# name -> (format, columns). Only the fields Log Doctor reads, in ArduPilot's names.
MESSAGES = {
    "PARM": ("QNf", "TimeUS,Name,Value"),
    "MSG": ("QZ", "TimeUS,Message"),
    "MODE": ("QMBB", "TimeUS,Mode,ModeNum,Rsn"),
    "ARM": ("QBIBB", "TimeUS,ArmState,ArmChecks,Forced,Method"),
    "EV": ("QB", "TimeUS,Id"),
    "ERR": ("QBB", "TimeUS,Subsys,ECode"),
    "VIBE": ("QBfffI", "TimeUS,IMU,VibeX,VibeY,VibeZ,Clip"),
    "BAT": ("QBffff", "TimeUS,Inst,Volt,Curr,CurrTot,EnrgTot"),
    "POWR": ("QffH", "TimeUS,Vcc,VServo,Flags"),
    "GPS": ("QBBIHBfLLfff", "TimeUS,I,Status,GMS,GWk,NSats,HDop,Lat,Lng,Alt,Spd,GCrs"),
    "POS": ("QLLff", "TimeUS,Lat,Lng,Alt,RelHomeAlt"),
    "BARO": ("QBff", "TimeUS,I,Alt,Press"),
    "ATT": ("QffffffB", "TimeUS,DesRoll,Roll,DesPitch,Pitch,DesYaw,Yaw,AEKF"),
    "XKF4": ("QBffff", "TimeUS,C,SV,SP,SH,SM"),
    "PM": ("QHHH", "TimeUS,NLon,NLoop,Load"),
    "MAG": ("Qhhh", "TimeUS,MagX,MagY,MagZ"),
    "IMU": ("QBffffff", "TimeUS,I,GyrX,GyrY,GyrZ,AccX,AccY,AccZ"),
    "RCOU": ("QHHHH", "TimeUS,C1,C2,C3,C4"),
}

# A field-free spot (ArduPilot's own SITL test site) so tests never carry a real location.
HOME = (-35.363262, 149.165237)


class Writer:
    def __init__(self):
        self.buf = bytearray()
        self.ids = {}
        self._fmt("FMT", FMT_ID, "BBnNZ", "Type,Length,Name,Format,Columns")
        for i, (name, (fmt, cols)) in enumerate(MESSAGES.items()):
            self.ids[name] = 130 + i
            self._fmt(name, 130 + i, fmt, cols)

    @staticmethod
    def _pack(fmt, values):
        out = b""
        for ch, v in zip(fmt, values, strict=False):
            code, scale = CODES[ch]
            if code.endswith("s"):
                v = v.encode() if isinstance(v, str) else v
            elif scale != 1:
                v = int(round(v * scale))
            elif code in "QqBbHhIi":
                v = int(v)
            out += struct.pack("<" + code, v)
        return out

    def _fmt(self, name, msg_id, fmt, cols):
        length = 3 + struct.calcsize("<" + "".join(CODES[c][0] for c in fmt))
        payload = self._pack("BBnNZ", (msg_id, length, name, fmt, cols))
        self.buf += HEAD + bytes([FMT_ID]) + payload

    def write(self, name, t, *values):
        fmt = MESSAGES[name][0]
        self.buf += HEAD + bytes([self.ids[name]]) + self._pack(fmt, (int(t * 1e6),) + values)

    def save(self, path):
        with open(path, "wb") as f:
            f.write(self.buf)
        return path


def flight_log(path, vehicle="copter", duration=150.0, vibe=3.0, vibe_from=None, clips_in_flight=0,
               clips_at_landing=0, cell_min=3.8, cells=4, arming_check=1, forced_arm=False,
               ekf=0.1, gps_sats=16, hdop=0.7, gps=True, battery=True, err=None, messages=(),
               truncate_in_flight=False, roll_error=1.0, rate_hz=10):
    """A whole flight: armed at 5 s, takeoff at 10 s, a 60 m circle at 20 m, land, disarm.

    Faults are injected through the arguments; defaults describe a healthy aircraft.
    """
    w = Writer()
    fw = "ArduCopter V4.6.2 (abcdef12)" if vehicle == "copter" else "ArduPlane V4.6.3 (abcdef12)"
    t_arm, t_off, t_land = 5.0, 10.0, duration - 15.0
    t_end = duration
    w.write("MSG", 0.5, fw)
    w.write("MSG", 0.6, "TestBoard 00000000 00000000 00000000")
    w.write("PARM", 0.7, "ARMING_CHECK", float(arming_check))
    w.write("PARM", 0.7, "LOG_BITMASK", 176126.0)
    for t, text in messages:
        w.write("MSG", t, text)
    w.write("MODE", 1.0, 5 if vehicle == "copter" else 5, 5, 0)
    w.write("ARM", t_arm, 1, 1, 1 if forced_arm else 0, 2)
    if err:
        for t, sub, code in err:
            w.write("ERR", t, sub, code)

    full = cells * 4.15
    clip_flight_times = [t_off + 20 + 7 * k for k in range(clips_in_flight)]
    clip_landing_times = [t_land + 0.5 + 0.1 * k for k in range(clips_at_landing)]
    clip = 0
    step = 1.0 / rate_hz
    n = int(t_end / step)
    for i in range(n):
        t = i * step
        if truncate_in_flight and t > (t_off + t_land) / 2:
            break
        flying = t_off <= t <= t_land
        armed = t_arm <= t <= t_land + 3
        phase = (t - t_off) / 40.0 * 2 * math.pi
        alt = 20.0 * min(1.0, max(0.0, (t - t_off) / 8.0)) if flying else 0.0
        if flying and t > t_land - 6:
            alt *= max(0.0, (t_land - t) / 6.0)
        r = 60.0 if flying else 0.0
        north, east = r * math.cos(phase) - r, r * math.sin(phase)
        lat = HOME[0] + north / 111320.0
        lng = HOME[1] + east / (111320.0 * math.cos(math.radians(HOME[0])))
        spd = 2 * math.pi * r / 40.0 if flying and alt > 2 else 0.0
        if vehicle == "plane" and flying:
            spd = max(spd, 12.0)

        des_roll = 15.0 * math.sin(phase * 3) if flying else 0.0
        roll = des_roll + roll_error * math.sin(t * 7.0)
        w.write("ATT", t, des_roll, roll, 0.0, 0.3 * math.sin(t), 90.0, 90.0 + 0.5 * math.sin(t), 1)

        v = (vibe if (vibe_from is None or t >= vibe_from) else 3.0) if flying else 0.3
        for tc in clip_flight_times + clip_landing_times:
            if t - step < tc <= t:
                clip += 1
        w.write("VIBE", t, 0, v * 0.8, v * 0.9, v, clip)

        w.write("POS", t, lat, lng, 584.0 + alt, alt)
        w.write("BARO", t, 0, alt, 101325.0 - alt * 12)
        w.write("XKF4", t, 0, ekf, ekf * 0.5, ekf * 0.7, ekf * 0.6)
        w.write("PM", t, 0, 400, 300)
        w.write("POWR", t, 5.1, 0.0, 0)
        load = 1.0 if flying else 0.0
        curr = 0.4 + 18.0 * load
        w.write("MAG", t, 200, 50, -400 + int(2 * curr))
        w.write("RCOU", t, *([1100 + int(600 * load)] * 4 if armed else [1000] * 4))
        if battery:
            sag = (full - cells * cell_min) * load
            w.write("BAT", t, 0, full - sag - 0.002 * t, curr, curr * t / 3.6, 0.0)
        if gps:
            w.write("GPS", t, 0, 3, 200000000 + int(t * 1000), 2390, gps_sats, hdop, lat, lng, 584.0 + alt, spd, 0.0)
        w.write("IMU", t, 0, 0.001 * math.sin(t * 13), 0.001 * math.cos(t * 11), 0.0005, 0.0, 0.0, -9.81)
        w.write("IMU", t, 1, 0.001 * math.sin(t * 17), 0.001 * math.cos(t * 5), 0.0005, 0.02, 0.0, -9.80)

    if not truncate_in_flight:
        w.write("ARM", t_land + 3, 0, 0, 0, 2)
    return w.save(path)
