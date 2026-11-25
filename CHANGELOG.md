# Changelog

## 0.3.0

- **Vibration spectrum (FFT) and harmonic-notch suggestions** for copters: finds the motor-noise peak (and its second harmonic) from batch-sampler data (`ISBH` / `ISBD`) or IMU logged at 300 Hz or faster, warns when the peak is near the Nyquist limit, and suggests `INS_HNTCH_*` values: throttle mode with the hover throttle as reference, or ESC-RPM mode when ESC telemetry is logged. Written to `<log>_notch.param` with a `<log>_spectrum.png` plot.
- New check: harmonic notch disabled, or set far from the measured noise peak.
- **About 4x faster** on large logs (75 MB: 53 s → 13 s) by reading only the fields the checks use.
- Release workflow for PyPI (trusted publishing).
- 54 tests; the synthetic log writer can now produce batch-sampler and fast-IMU data with injected motor noise.

## 0.2.0

- **Never a false all-clear:** empty, corrupted, non-DataFlash and `.tlog` files are rejected with a clear message instead of producing a report.
- **Crash signature:** a log that ends while the aircraft is armed and flying is reported as a FAIL.
- **Not-checked list and INCOMPLETE verdict:** data a check needs but the log lacks (no GPS, no battery monitor…) is listed instead of silently skipped.
- **Configurable limits:** `--print-limits` and `--limits my_limits.json` (partial files allowed, typos rejected).
- **Exit codes:** 0 ok, 1 failed checks with `--strict`, 2 unreadable log or limits file.
- **JSON schema 1:** raw `value` per check, `status`, `not_checked`, tool name and version; several logs give one JSON array.
- Fix hints are ordered by severity; damaged records are counted and reported.
- Fixes: time base taken from the earliest record; text output honours redirected stdout.
- Requires Python 3.10+ (3.9 is end-of-life). CI: Linux, Windows, macOS on 3.10 and 3.13.
- 44 tests on synthetic DataFlash logs with injected faults (no real flight data in the repo).

## 0.1.0

- First release: 20+ checks, Markdown report and summary figure, Copter and Plane support.
