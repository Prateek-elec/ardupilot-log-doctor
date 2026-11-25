"""Read the messages Log Doctor needs from a DataFlash .BIN log into numpy arrays."""

from collections import defaultdict

import numpy as np
from pymavlink import mavutil

# Messages used by the checks and plots.
TYPES = ["VIBE", "PM", "POWR", "BAT", "XKF4", "NKF4", "MAG", "ATT", "GPS", "IMU", "RCOU",
         "MODE", "ERR", "ARM", "EV", "MSG", "POS", "BARO", "CTUN", "PARM"]

# Field that holds the instance number (IMU 0/1/2, GPS 0/1, ...) for each message type.
INSTANCE_FIELD = {"VIBE": "IMU", "BAT": "Inst", "GPS": "I", "IMU": "I", "MAG": "I",
                  "XKF4": "C", "NKF4": "C", "BARO": "I"}


class Log:
    """Holds one array per field, per message type and instance, with time in seconds from log start."""

    def __init__(self, path):
        self.path = path
        self.msgs = {}          # (type, instance) -> {field: np.ndarray, "t": np.ndarray}
        self.texts = []         # (t, text) from MSG
        self.params = {}        # parameter name -> last value in the log
        self.t0 = None
        self.t_end = 0.0
        self._load()

    def _load(self):
        log = mavutil.mavlink_connection(self.path, robust_parsing=True)
        rows = defaultdict(list)
        while True:
            m = log.recv_match(type=TYPES)
            if m is None:
                break
            typ = m.get_type()
            if typ == "BAD_DATA":
                continue
            d = m.to_dict()
            if typ == "PARM":
                self.params[str(d.get("Name", ""))] = d.get("Value")
                continue
            t_us = d.get("TimeUS")
            if t_us is None:
                continue
            if self.t0 is None:
                self.t0 = t_us
            t = (t_us - self.t0) / 1e6
            self.t_end = max(self.t_end, t)
            if typ == "MSG":
                self.texts.append((t, str(d.get("Message", ""))))
                continue
            inst = int(d.get(INSTANCE_FIELD.get(typ, ""), 0) or 0)
            d["t"] = t
            rows[(typ, inst)].append(d)
        for key, recs in rows.items():
            fields = {}
            for name in recs[0]:
                if name == "mavpackettype":
                    continue
                col = [r.get(name) for r in recs]
                try:
                    fields[name] = np.asarray(col, dtype=float)
                except (TypeError, ValueError):
                    fields[name] = np.asarray(col, dtype=object)
            self.msgs[key] = fields

    def get(self, typ, inst=0):
        """Fields of a message type / instance, or None when the log doesn't have it."""
        m = self.msgs.get((typ, inst))
        if m is None or len(m["t"]) == 0:
            return None
        return m

    def has(self, typ, *fields, inst=0):
        m = self.get(typ, inst)
        return m is not None and all(f in m for f in fields)

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
        known = ("ChibiOS", "ArduPlane", "ArduCopter", "ArduRover", "ArduSub", "Param space", "RC Protocol",
                 "RCOut", "New ", "GPS", "u-blox", "Frame", "IMU", "EKF", "AHRS", "Barometer", "Mission",
                 "Throttle", "Arm", "Disarm", "Init", "Flight", "PreArm", "Field Elevation")
        # The board line follows the firmware lines at boot: "<BoardName> <hex id> <hex id> <hex id>".
        for _, text in self.texts[:40]:
            parts = text.split()
            if len(parts) == 4 and not text.startswith(known) and all(
                    len(p) == 8 and all(c in "0123456789ABCDEFabcdef" for c in p) for p in parts[1:]):
                return parts[0]
        return None
