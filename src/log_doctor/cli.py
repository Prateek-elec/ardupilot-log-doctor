"""Command line: log-doctor FLIGHT.BIN [more logs] [--out DIR] [--json] [--no-plot] [--strict]"""

import argparse
import glob
import os
import sys

from . import __version__
from . import report as rep
from .doctor import FAIL, analyse


def main(argv=None):
    p = argparse.ArgumentParser(prog="log-doctor", description="Health check for ArduPilot DataFlash logs (.BIN).")
    p.add_argument("logs", nargs="+", help="one or more .BIN logs (wildcards work)")
    p.add_argument("--out", help="folder for the report (.md) and figure (.png); default: next to each log")
    p.add_argument("--no-plot", action="store_true", help="skip the summary figure")
    p.add_argument("--no-files", action="store_true", help="print the report only, write no files")
    p.add_argument("--json", action="store_true", help="print JSON instead of the text report")
    p.add_argument("--strict", action="store_true", help="exit with code 1 if any check fails (for scripts / CI)")
    p.add_argument("--version", action="version", version=f"log-doctor {__version__}")
    a = p.parse_args(argv)

    paths = []
    for pattern in a.logs:
        paths += sorted(glob.glob(pattern)) or [pattern]
    any_fail = False
    for path in paths:
        if not os.path.isfile(path):
            print(f"log-doctor: no such file: {path}", file=sys.stderr)
            any_fail = True
            continue
        size_mb = os.path.getsize(path) / 1e6
        if size_mb > 5 and not a.json:
            print(f"Reading {os.path.basename(path)} ({size_mb:.0f} MB, about {max(1, round(size_mb * 0.5))} s)...", file=sys.stderr)
        try:
            r = analyse(path)
        except Exception as e:  # noqa: BLE001 - report and move on to the next log
            print(f"log-doctor: could not read {path}: {e}", file=sys.stderr)
            any_fail = True
            continue
        any_fail |= r.counts[FAIL] > 0
        if a.json:
            js = rep.as_json(r)
            try:
                js.encode(sys.stdout.encoding or "ascii")
            except (UnicodeEncodeError, LookupError):
                js = rep.as_json(r, ascii_only=True)
            print(js)
        else:
            rep.text(r)
        if a.no_files:
            continue
        out = a.out or os.path.dirname(os.path.abspath(path))
        os.makedirs(out, exist_ok=True)
        stem = os.path.splitext(os.path.basename(path))[0]
        fig_name = None
        if not a.no_plot:
            from .plot import figure
            fig_name = f"{stem}_doctor.png"
            figure(r, os.path.join(out, fig_name))
        md = os.path.join(out, f"{stem}_doctor.md")
        with open(md, "w", encoding="utf-8") as f:
            f.write(rep.markdown(r, fig_name))
        if not a.json:
            print(f"\nReport: {md}" + (f"\nFigure: {os.path.join(out, fig_name)}" if fig_name else ""))
    return 1 if (a.strict and any_fail) else 0


if __name__ == "__main__":
    sys.exit(main())
