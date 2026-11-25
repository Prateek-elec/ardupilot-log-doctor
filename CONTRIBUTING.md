# Contributing

The most useful contribution is a log that Log Doctor gets wrong: open an issue with the vehicle, firmware and (if you can share it) the log.

## Development

```bash
pip install -e .[test] ruff
pytest          # 44 tests on synthetic logs
ruff check .
```

Tests never use real flight logs, which carry locations and hardware IDs. `tests/synthlog.py` writes small DataFlash logs with faults injected on purpose; add a scenario there for any new check, and a test that shows the check catching it.
