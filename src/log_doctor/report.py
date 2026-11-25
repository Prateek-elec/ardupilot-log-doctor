"""Turn a Report into terminal text, Markdown or JSON."""

import json
import os
import sys

from . import __version__
from .doctor import FAIL, INFO, PASS, WARN

ICON = {PASS: "✅", WARN: "⚠️", FAIL: "❌", INFO: "ℹ️", "INCOMPLETE": "❔"}
ANSI = {PASS: "\033[32m", WARN: "\033[33m", FAIL: "\033[31m", INFO: "\033[36m", "INCOMPLETE": "\033[35m"}
RESET = "\033[0m"


def _mmss(t):
    return f"{int(t // 60):02d}:{int(t % 60):02d}"


def _colour_ok(stream):
    if os.environ.get("NO_COLOR"):
        return False
    if os.name == "nt":
        os.system("")  # enables ANSI colours in the Windows console
    return hasattr(stream, "isatty") and stream.isatty()


ASCII = {"→": "->", "²": "2", " °/s": " deg/s", "°": " deg", "•": "-", "–": "-", "×": "x", "≥": ">=", "…": "...", "·": "|"}


def _needs_ascii(stream):
    enc = getattr(stream, "encoding", None) or "ascii"
    try:
        "".join(ASCII).encode(enc)
        return False
    except (UnicodeEncodeError, LookupError):
        return True


def _printable(s, stream):
    """Swap the report's symbols for plain ASCII when the console can't show all of them (e.g. Windows cp1252)."""
    if not _needs_ascii(stream):
        return s
    for k, v in ASCII.items():
        s = s.replace(k, v)
    enc = getattr(stream, "encoding", None) or "ascii"
    try:
        return s.encode(enc, errors="replace").decode(enc, errors="replace")
    except LookupError:
        return s.encode("ascii", errors="replace").decode("ascii")


def text(r, stream=None):
    stream = stream or sys.stdout   # resolved at call time, so redirection and capture work
    col = _colour_ok(stream)
    paint = (lambda s, st: f"{ANSI[st]}{s}{RESET}") if col else (lambda s, st: s)
    out = []
    out.append("")
    out.append(f"ArduPilot Log Doctor · {os.path.basename(r.path)}")
    out.append(f"{r.firmware}" + (f" on {r.board}" if r.board else "") + f" · log length {_mmss(r.duration_s)}")
    for k, v in r.summary.items():
        out.append(f"  {k}: {v}")
    out.append("")
    rows = [(c.status, _printable(c.name, stream), _printable(c.measured, stream), _printable(c.limit, stream)) for c in r.checks]
    w = max((len(n) for _, n, _, _ in rows), default=10)
    wm = max((len(m) for _, _, m, _ in rows), default=10)
    for st, name, meas, lim in rows:
        out.append(f"  {paint(st.ljust(4), st)}  {name.ljust(w)}  {meas.ljust(wm)}  limit {lim}")
    n = r.counts
    out.append("")
    out.append(f"  PASS {n[PASS]}   WARN {n[WARN]}   FAIL {n[FAIL]}   →  {paint(r.verdict, r.status)}")
    if r.not_checked:
        out.append("")
        out.append("Not checked:")
        for area, reason in r.not_checked:
            out.append(f"  • {area}: {reason}")
    hints = r.hints_by_severity()
    if hints:
        out.append("")
        out.append("What to look at (most serious first):")
        for c, hint in hints:
            out.append(f"  • [{c.status}] {c.name}: {hint}")
    if r.notch:
        out.append("")
        out.append(f"Suggested harmonic notch (noise peak {r.spectrum.peak_hz:.0f} Hz; check, then load in Mission Planner):")
        for k, v in r.notch.items():
            out.append(f"  {k} = {v}")
    if r.modes:
        out.append("")
        out.append("Flight modes: " + " → ".join(f"{name} ({_mmss(a)})" for a, b, name in r.modes))
    if r.events:
        out.append("")
        out.append("Events:")
        for t, e in r.events[:25]:
            out.append(f"  {_mmss(t)}  {e}")
        if len(r.events) > 25:
            out.append(f"  … and {len(r.events) - 25} more")
    print(_printable("\n".join(out), stream), file=stream)


def markdown(r, figure_name=None):
    n = r.counts
    lines = [f"# Log Doctor report: `{os.path.basename(r.path)}`", ""]
    lines.append(f"**{r.firmware}**" + (f" on **{r.board}**" if r.board else "") + f" · log length {_mmss(r.duration_s)}")
    lines.append("")
    lines.append(f"## {ICON[r.status]} {r.verdict}")
    lines.append("")
    lines.append(f"PASS **{n[PASS]}** · WARN **{n[WARN]}** · FAIL **{n[FAIL]}**")
    lines.append("")
    if figure_name:
        lines += [f"![Flight summary]({figure_name})", ""]
    if r.summary:
        lines += ["## Flight", "", "| | |", "|---|---|"]
        lines += [f"| {k} | {v} |" for k, v in r.summary.items()]
        lines.append("")
    lines += ["## Checks", "", "| | Check | Measured | Limit |", "|---|---|---|---|"]
    lines += [f"| {ICON[c.status]} | {c.name} | {c.measured} | {c.limit} |" for c in r.checks]
    if r.not_checked:
        lines += ["", "## Not checked", ""]
        lines += [f"- **{area}:** {reason}" for area, reason in r.not_checked]
    hints = r.hints_by_severity()
    if hints:
        lines += ["", "## What to look at (most serious first)", ""]
        lines += [f"- {ICON[c.status]} **{c.name}:** {hint}" for c, hint in hints]
    if r.notch:
        s = r.spectrum
        lines += ["", "## Suggested harmonic notch", "",
                  f"Gyro noise peaks at **{s.peak_hz:.0f} Hz** ({s.peak_db:.0f} dB above the noise floor"
                  + (f", second harmonic at {s.harmonic_hz:.0f} Hz" if s.harmonic_hz else "")
                  + f"), from {s.source} data at {s.rate_hz:.0f} Hz. Starting values, following the ArduPilot tuning guide:", "",
                  "```text"] + [f"{k},{v}" for k, v in r.notch.items()] + ["```", "",
                  "After loading them, fly again with the batch sampler on and confirm the peak has dropped."]
    if r.modes:
        lines += ["", "## Flight modes", "", "| From | To | Mode |", "|---|---|---|"]
        lines += [f"| {_mmss(a)} | {_mmss(b)} | {name} |" for a, b, name in r.modes]
    if r.events:
        lines += ["", "## Events", "", "| Time | Event |", "|---|---|"]
        lines += [f"| {_mmss(t)} | {e} |" for t, e in r.events]
    lines += ["", "---", "", f"Generated by [ArduPilot Log Doctor](https://github.com/Prateek-elec/ardupilot-log-doctor) {__version__}. "
              "Limits follow common ArduPilot guidance; adjust them for your airframe."]
    return "\n".join(lines) + "\n"


def as_json(r, ascii_only=False):
    return json.dumps({
        "tool": "ardupilot-log-doctor", "version": __version__, "schema": 1,
        "file": os.path.basename(r.path), "firmware": r.firmware, "vehicle": r.vehicle, "board": r.board,
        "duration_s": round(r.duration_s, 1), "flights": [[round(a, 1), round(b, 1)] for a, b in r.flights],
        "summary": r.summary, "status": r.status, "verdict": r.verdict, "counts": r.counts,
        "checks": [{"name": c.name, "status": c.status, "measured": c.measured,
                    "value": None if c.value is None else round(float(c.value), 4),
                    "limit": c.limit, "hint": c.hint} for c in r.checks],
        "not_checked": [{"area": a, "reason": why} for a, why in r.not_checked],
        "spectrum": None if r.spectrum is None else {
            "source": r.spectrum.source, "rate_hz": round(r.spectrum.rate_hz, 1), "peak_hz": round(r.spectrum.peak_hz, 1),
            "peak_db": round(r.spectrum.peak_db, 1), "harmonic_hz": None if r.spectrum.harmonic_hz is None else round(r.spectrum.harmonic_hz, 1)},
        "notch_suggestion": r.notch,
        "modes": [{"from": round(a, 1), "to": round(b, 1), "mode": m} for a, b, m in r.modes],
        "events": [{"t": round(t, 1), "text": e} for t, e in r.events],
    }, indent=2, ensure_ascii=ascii_only)
