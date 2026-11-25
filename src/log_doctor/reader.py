"""Read the messages Log Doctor needs from a DataFlash .BIN log into numpy arrays."""

import contextlib
import io
import os
from collections import defaultdict

import numpy as np
from pymavlink import DFReader

# Messages used by the checks and plots.
TYPES = ["VIBE", "PM", "POWR", "BAT", "XKF4", "NKF4", "MAG", "ATT", "GPS", "IMU", "RCOU",
         "MODE", "ERR", "ARM", "EV", "MSG", "POS", "BARO", "CTUN", "PARM", "ISBH", "ISBD", "ESC"]

# Field that holds the instance number (IMU 0/1/2, GPS 0/1, ...) for each message type.
INSTANCE_FIELD = {"VIBE": "IMU", "BAT": "Inst", "GPS": "I", "IMU": "I", "MAG": "I",
                  "XKF4": "C", "NKF4": "C", "BARO": "I", "ESC": "Instance"}

# The only fields the checks read, per message type. Reading just these, straight from the raw record,
# is several times faster than asking pymavlink for every field of every message.
FIELDS = {
    "VIBE": ("IMU", "VibeX", "VibeY", "VibeZ", "Clip", "Clip0", "Clip1", "Clip2"),
    "PM": ("Load", "NLon", "NLoop", "NL"),
    "POWR": ("Vcc",),
    "BAT": ("Inst", "Volt", "Curr", "CurrTot"),
    "XKF4": ("C", "SV", "SP", "SH", "SM"),
    "NKF4": ("C", "SV", "SP", "SH", "SM"),
    "MAG": ("I", "MagX", "MagY", "MagZ"),
    "ATT": ("DesRoll", "Roll", "DesPitch", "Pitch", "DesYaw", "Yaw"),
    "GPS": ("I", "Status", "NSats", "HDop", "Lat", "Lng", "Spd"),
    "IMU": ("I", "GyrX", "GyrY", "GyrZ", "AccX", "AccY", "AccZ"),
    "RCOU": tuple(f"C{i}" for i in range(1, 15)),
    "MODE": ("Mode", "ModeNum"),
    "ERR": ("Subsys", "ECode"),
    "ARM": ("ArmState", "Forced"),
    "EV": ("Id",),
    "MSG": ("Message",),
    "POS": ("Lat", "Lng", "RelHomeAlt"),
    "BARO": ("I", "Alt"),
    "CTUN": ("ThO",),
    "ISBH": ("N", "type", "instance", "mul", "smp_cnt", "smp_rate"),
    "ISBD": ("N", "seqno", "x", "y", "z"),
    "ESC": ("Instance", "RPM"),
}

DATAFLASH_HEADER = b"\xa3\x95"


def _field_plan(fmt, typ):
    """[(name, index, multiplier, is_array)] for the fields of `typ` this format actually has."""
    plan = []
    for name in ("TimeUS",) + FIELDS.get(typ, ()):
        i = fmt.colhash.get(name)
        if i is None:
            continue
        plan.append((name, i, fmt.msg_mults[i], fmt.format[i] == "a"))
    return plan


def _value(raw, mult, is_array):
    """pymavlink's own conversion: decode text, keep arrays, apply the unit multiplier."""
    if isinstance(raw, bytes):
        try:
            raw = raw.decode("utf-8")
        except UnicodeDecodeError:
            raw = raw.decode("ISO-8859-1")
        return raw.split("\0", 1)[0]
    if is_array or mult is None:
        return raw
    return raw / (1.0 / mult) if 0.0 < mult < 1.0 else raw * mult


class LogError(Exception):
    """The file can't be analysed: missing, empty, not a DataFlash log, or no usable data."""


def _check_file(path):
    if not os.path.isfile(path):
        raise LogError("no such file")
    size = os.path.getsize(path)
    if size == 0:
        raise LogError("the file is empty (0 bytes), often a log from a flight controller that lost power while writing")
    if path.lower().endswith(".tlog"):
        raise LogError("this is a telemetry log (.tlog). Log Doctor reads the DataFlash .BIN log from the "
                       "flight controller's SD card (or downloaded via Mission Planner's DataFlash Logs page)")
    with open(path, "rb") as f:
        head = f.read(min(size, 65536))
    if DATAFLASH_HEADER not in head:
        raise LogError("not an ArduPilot DataFlash .BIN log (no DataFlash records found)")


class Log:
    """One array per field, per message type and instance, with time in seconds from log start."""

    def __init__(self, path):
        self.path = path
        self.msgs = {}          # (type, instance) -> {field: np.ndarray, "t": np.ndarray}
        self.texts = []         # (t, text) from MSG
        self.params = {}        # parameter name -> last value in the log
        self.t0 = None
        self.t_end = 0.0
        self.corrupt_records = 0
        _check_file(path)
        self._load()
        if not self.msgs and not self.texts:
            raise LogError("no readable ArduPilot messages in the file; it may be corrupted or not a flight log")

    def _load(self):
        noise = io.StringIO()
        rows = defaultdict(list)
        texts_raw = []
        plans = {}               # format id -> field plan, built once per message type
        t_min = float("inf")
        # pymavlink prints a line per damaged record; keep the console clean and count them instead.
        with contextlib.redirect_stdout(noise):
            try:
                log = DFReader.DFReader_binary(self.path, zero_time_base=False)
            except Exception as e:  # noqa: BLE001 - pymavlink raises many types on bad files
                raise LogError(f"pymavlink could not open the log ({e.__class__.__name__}: {e})") from e
            while True:
                try:
                    m = log.recv_match(type=TYPES)
                except Exception:  # noqa: BLE001 - a damaged tail ends the log
                    break
                if m is None:
                    break
                typ = m.get_type()
                if typ == "BAD_DATA":
                    continue
                if typ == "PARM":
                    self.params[str(getattr(m, "Name", ""))] = getattr(m, "Value", None)
                    continue
                fmt = m.fmt
                plan = plans.get(fmt.type)
                if plan is None:
                    plan = plans[fmt.type] = _field_plan(fmt, typ)
                if not plan or plan[0][0] != "TimeUS":
                    continue
                el = m._elements
                try:
                    vals = {name: _value(el[i], mult, arr) for name, i, mult, arr in plan}
                except Exception:  # noqa: BLE001 - a field that won't decode
                    self.corrupt_records += 1
                    continue
                t_us = vals["TimeUS"]
                if self.t0 is None:
                    self.t0 = t_us
                if abs(t_us - self.t0) > 7 * 24 * 3600e6:   # a garbled timestamp
                    self.corrupt_records += 1
                    continue
                t_min = min(t_min, t_us)
                if typ == "MSG":
                    texts_raw.append((t_us, str(vals.get("Message", ""))))
                    continue
                inst = vals.get(INSTANCE_FIELD.get(typ, ""), 0) or 0
                try:
                    inst = int(inst)
                except (TypeError, ValueError):
                    inst = 0
                vals["t"] = t_us
                rows[(typ, inst)].append(vals)
        self.corrupt_records += noise.getvalue().count("bad msg") + noise.getvalue().count("Failed to parse")
        # Time zero is the earliest record: boot messages are often logged just after the first sensor data.
        self.t0 = t_min if t_min != float("inf") else 0
        self.texts = sorted(((tu - self.t0) / 1e6, text) for tu, text in texts_raw)
        for key, recs in rows.items():
            fields = {}
            recs.sort(key=lambda r: r["t"])
            for rec in recs:
                rec["t"] = (rec["t"] - self.t0) / 1e6
            for name in recs[0]:
                col = [r.get(name) for r in recs]
                try:
                    fields[name] = np.asarray(col, dtype=float)
                except (TypeError, ValueError):
                    fields[name] = np.asarray(col, dtype=object)
            self.msgs[key] = fields
            if len(fields["t"]):
                self.t_end = max(self.t_end, float(fields["t"][-1]))
        if self.texts:
            self.t_end = max(self.t_end, self.texts[-1][0])

    def get(self, typ, inst=0):
        """Fields of a message type / instance, or None when the log doesn't have it."""
        m = self.msgs.get((typ, inst))
        if m is None or len(m["t"]) == 0:
            return None
        return m

    def instances(self, typ):
        return sorted(i for (t, i) in self.msgs if t == typ)

    # ---- what flew
    @property
    def firmware(self):
        for _, text in self.texts:
            if text.startswith(("ArduPlane", "ArduCopter", "ArduRover", "ArduSub", "Blimp", "AntennaTracker")):
                return text.split(" (")[0]
        return "unknown firmware"

    @property
    def vehicle(self):
        fw = self.firmware.lower()
        for key, name in (("plane", "plane"), ("copter", "copter"), ("rover", "rover"), ("sub", "sub")):
            if key in fw:
                return name
        return "unknown"

    @property
    def board(self):
        """Board name from the boot banner '<Board> <id> <id> <id>' (the serial-number ids are never kept)."""
        for _, text in self.texts[:40]:
            parts = text.split()
            if (len(parts) == 4 and not parts[0].startswith(("Ardu", "ChibiOS"))
                    and all(len(p) == 8 and all(c in "0123456789ABCDEFabcdef" for c in p) for p in parts[1:])):
                return parts[0]
        return None
