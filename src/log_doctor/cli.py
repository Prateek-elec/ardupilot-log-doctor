"""Command line: log-doctor FLIGHT.BIN [more logs] [options]

Exit codes: 0 analysed (or only warnings), 1 a check failed and --strict was given,
2 a log or the limits file could not be read.
"""

import argparse
import glob
import json
import os
import sys

from . import __version__
from . import limits as lim_mod
from . import report as rep
from .doctor import FAIL, analyse
from .reader import LogError

EXIT_OK, EXIT_FAILED_CHECKS, EXIT_UNREADABLE = 0, 1, 2


def _parser():
    p = argparse.ArgumentParser(
        prog="log-doctor",
        description="Health check for ArduPilot DataFlash logs (.BIN): pass / warn / fail with what to fix.",
        epilog="Exit codes: 0 ok, 1 failed checks (with --strict), 2 unreadable log or limits file.")
    p.add_argument("logs", nargs="*", help="one or more .BIN logs (wildcards work: logs/*.BIN)")
    p.add_argument("--out", help="folder for the .md report and .png figure (default: next to each log)")
    p.add_argument("--no-plot", action="store_true", help="skip the summary figure")
    p.add_argument("--no-files", action="store_true", help="print the report only, write no files")
    p.add_argument("--json", action="store_true", help="print JSON instead of the text report")
    p.add_argument("--limits", metavar="FILE", help="JSON file overriding some or all pass / warn limits")
    p.add_argument("--print-limits", action="store_true", help="print the default limits as JSON and exit")
    p.add_argument("--strict", action="store_true", help="exit with code 1 if any check fails (for scripts / CI)")
    p.add_argument("--version", action="version", version=f"log-doctor {__version__}")
    return p


def main(argv=None):
    p = _parser()
    a = p.parse_args(argv)

    if a.print_limits:
        print(json.dumps(lim_mod.defaults(), indent=2))
        return EXIT_OK
    if not a.logs:
        p.error("give at least one .BIN log")

    try:
        limits = lim_mod.load(a.limits) if a.limits else None
    except lim_mod.LimitsError as e:
        print(f"log-doctor: {e}", file=sys.stderr)
        return EXIT_UNREADABLE

    paths = []
    for pattern in a.logs:
        paths += sorted(glob.glob(pattern)) or [pattern]

    unreadable = failed = False
    results = []
    for path in paths:
        size_mb = os.path.getsize(path) / 1e6 if os.path.isfile(path) else 0
        if size_mb > 5 and not a.json:
            print(f"Reading {os.path.basename(path)} ({size_mb:.0f} MB)...", file=sys.stderr)
        try:
            r = analyse(path, limits)
        except LogError as e:
            print(f"log-doctor: {path}: {e}", file=sys.stderr)
            unreadable = True
            continue
        except Exception as e:  # noqa: BLE001 - never crash on one bad log in a batch
            print(f"log-doctor: {path}: unexpected error ({e.__class__.__name__}: {e}). "
                  "Please report it with the log at https://github.com/Prateek-elec/ardupilot-log-doctor/issues",
                  file=sys.stderr)
            unreadable = True
            continue
        failed |= r.counts[FAIL] > 0
        if a.json:
            results.append(r)
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

    if a.json:
        _print_json(results)
    if unreadable:
        return EXIT_UNREADABLE
    return EXIT_FAILED_CHECKS if (a.strict and failed) else EXIT_OK


def _print_json(results):
    """One log -> one object; several logs -> a JSON array (always valid JSON on stdout)."""
    def dump(ascii_only):
        parts = [rep.as_json(r, ascii_only) for r in results]
        return parts[0] if len(parts) == 1 else "[\n" + ",\n".join(parts) + "\n]"
    js = dump(False)
    try:
        js.encode(sys.stdout.encoding or "ascii")
    except (UnicodeEncodeError, LookupError):
        js = dump(True)
    print(js)


if __name__ == "__main__":
    sys.exit(main())
