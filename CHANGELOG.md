# Changelog

## 0.2.0

- **Never a false all-clear:** empty, corrupted, non-DataFlash and `.tlog` files are rejected with a clear message instead of producing a report.
- **Crash signature:** a log that ends while the aircraft is armed and flying is reported as a FAIL.
- **Not-checked list and INCOMPLETE verdict:** data a check needs but the log lacks (no GPS, no battery monitor…) is listed instead of silently skipped.
- **Configurable limits:** `--print-limits` and `--limits my_limits.json` (partial files allowed, typos rejected).
- **Exit codes:** 0 ok, 1 failed checks with `--strict`, 2 unreadable log or limits file.
- **JSON schema 1:** raw `value` per check, `status`, `not_checked`, tool name and version; several logs give one JSON array.
- Fix hints are ordered by severity; damaged records are counted and reported.
- Fixes: time base taken from the earliest record; text output honours redirected stdout.
- 44 tests on synthetic DataFlash logs with injected faults (no real flight data in the repo).

## 0.1.0

- First release: 20+ checks, Markdown report and summary figure, Copter and Plane support.
