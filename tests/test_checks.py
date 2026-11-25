"""Every check against synthetic logs with known, injected faults."""

import json
import os

import pytest

from log_doctor import limits as lim_mod
from log_doctor import report
from log_doctor.cli import EXIT_FAILED_CHECKS, EXIT_OK, EXIT_UNREADABLE, main
from log_doctor.doctor import FAIL, PASS, WARN, analyse
from log_doctor.reader import LogError
from synthlog import HOME, flight_log

DUR = 90.0


def run(tmp_path, name="f.bin", limits=None, **fault):
    return analyse(flight_log(str(tmp_path / name), duration=DUR, **fault), limits)


def status(r, startswith):
    hits = [c for c in r.checks if c.name.startswith(startswith)]
    assert hits, f"no check named {startswith!r}; have {[c.name for c in r.checks]}"
    return hits[0].status


@pytest.fixture(scope="module")
def healthy(tmp_path_factory):
    return analyse(flight_log(str(tmp_path_factory.mktemp("h") / "healthy.bin"), duration=DUR))


# --------------------------------------------------------------------------- healthy baseline

def test_healthy_copter_passes_everything(healthy):
    assert healthy.status == PASS, [(c.name, c.status) for c in healthy.checks if c.status != PASS]
    assert healthy.vehicle == "copter" and healthy.firmware.startswith("ArduCopter")
    assert len(healthy.flights) == 1
    assert healthy.complete and not healthy.not_checked


def test_flight_summary(healthy):
    s = healthy.summary
    assert s["Flight time"].startswith("1.") and "1 flight" in s["Flight time"]
    assert s["Max altitude above home"] == "20 m"
    assert "4S" in s["Battery"]


# --------------------------------------------------------------------------- each fault is caught

@pytest.mark.parametrize("vibe, expected", [(45.0, WARN), (75.0, FAIL)])
def test_vibration(tmp_path, vibe, expected):
    assert status(run(tmp_path, vibe=vibe), "Vibration IMU0") == expected


def test_clipping_in_flight_fails(tmp_path):
    r = run(tmp_path, clips_in_flight=3)
    assert status(r, "Accel clipping IMU0 (in flight)") == FAIL


def test_clipping_at_touchdown_is_only_a_warning(tmp_path):
    r = run(tmp_path, clips_at_landing=4)
    assert status(r, "Accel clipping IMU0 (in flight)") == PASS
    assert status(r, "Accel clipping IMU0 (takeoff / touchdown)") == WARN


@pytest.mark.parametrize("cell, expected", [(3.4, WARN), (3.1, FAIL)])
def test_low_cells(tmp_path, cell, expected):
    assert status(run(tmp_path, cell_min=cell), "Lowest cell voltage") == expected


def test_arming_checks_disabled(tmp_path):
    assert status(run(tmp_path, arming_check=0), "Pre-arm checks enabled") == FAIL


def test_forced_arming(tmp_path):
    assert status(run(tmp_path, forced_arm=True), "Forced arms") == FAIL


@pytest.mark.parametrize("ekf, expected", [(0.7, WARN), (1.6, FAIL)])
def test_ekf(tmp_path, ekf, expected):
    assert status(run(tmp_path, ekf=ekf), "EKF velocity") == expected


def test_poor_gps(tmp_path):
    r = run(tmp_path, gps_sats=5, hdop=2.0)
    assert status(r, "GPS satellites") == FAIL
    assert status(r, "GPS HDOP") == WARN


def test_tracking_error(tmp_path):
    assert status(run(tmp_path, roll_error=10.0), "Roll tracking") == FAIL


def test_log_ending_in_flight_is_a_crash_signature(tmp_path):
    r = run(tmp_path, truncate_in_flight=True)
    assert status(r, "Log ends while flying") == FAIL
    assert r.hints_by_severity()[0][0].status == FAIL   # failures are listed first


def test_cut_file_still_analyses(tmp_path):
    p = flight_log(str(tmp_path / "full.bin"), duration=DUR)
    with open(p, "rb") as f:
        data = f.read()
    cut = tmp_path / "cut.bin"
    cut.write_bytes(data[: int(len(data) * 0.6) + 7])   # ends mid-record, mid-flight
    r = analyse(str(cut))
    assert status(r, "Log ends while flying") == FAIL


def test_err_events_are_decoded(tmp_path):
    r = run(tmp_path, err=[(40.0, 6, 1), (45.0, 6, 0)])
    assert status(r, "Errors and failsafes") == WARN
    texts = [e for _, e in r.events]
    assert "ERR Battery failsafe: triggered" in texts and "ERR Battery failsafe: resolved" in texts


def test_notable_messages_reach_the_event_list(tmp_path):
    r = run(tmp_path, messages=[(30.0, "Potential Thrust Loss (3)"), (31.0, "Potential Thrust Loss (3)")])
    assert any("Potential Thrust Loss" in e and "×2" in e for _, e in r.events)


def test_plane_uses_plane_limits_and_skips_yaw(tmp_path):
    r = run(tmp_path, vehicle="plane", roll_error=7.0)   # ~5 deg RMS: too much for a copter, fine for a plane
    assert r.vehicle == "plane"
    assert status(r, "Roll tracking") == PASS
    assert not [c for c in r.checks if c.name.startswith("Yaw tracking")]


# --------------------------------------------------------------------------- missing data is reported, not hidden

def test_missing_sensors_are_listed_as_not_checked(tmp_path):
    r = run(tmp_path, gps=False, battery=False)
    areas = {a for a, _ in r.not_checked}
    assert {"GPS", "Battery"} <= areas
    assert r.complete                          # vibration and attitude are still there


# --------------------------------------------------------------------------- bad input never produces a report

def test_empty_file(tmp_path):
    p = tmp_path / "empty.bin"
    p.write_bytes(b"")
    with pytest.raises(LogError, match="empty"):
        analyse(str(p))


def test_random_bytes_are_rejected(tmp_path):
    p = tmp_path / "noise.bin"
    p.write_bytes(os.urandom(50_000).replace(b"\xa3\x95", b"\x00\x00"))
    with pytest.raises(LogError):
        analyse(str(p))


def test_random_bytes_with_headers_are_rejected(tmp_path):
    p = tmp_path / "noise2.bin"
    p.write_bytes(b"\xa3\x95\x80" + os.urandom(50_000))
    with pytest.raises(LogError):
        analyse(str(p))


def test_telemetry_log_gets_a_helpful_message(tmp_path):
    p = tmp_path / "flight.tlog"
    p.write_bytes(b"\xfe" * 100)
    with pytest.raises(LogError, match="telemetry log"):
        analyse(str(p))


def test_missing_file(tmp_path):
    with pytest.raises(LogError, match="no such file"):
        analyse(str(tmp_path / "nope.bin"))


# --------------------------------------------------------------------------- limits

def test_limits_override_changes_the_grade(tmp_path):
    lim = lim_mod.merge({"vibration_mss": {"pass_below": 2, "warn_below": 10}})
    assert status(run(tmp_path, limits=lim), "Vibration IMU0") == WARN


def test_partial_limits_file(tmp_path):
    f = tmp_path / "lim.json"
    f.write_text(json.dumps({"_comment": "heavy lift", "cell_min_v": {"pass_at_least": 3.6}}))
    lim = lim_mod.load(str(f))
    assert lim["cell_min_v"] == {"pass_at_least": 3.6, "warn_at_least": 3.3}
    assert lim["vibration_mss"] == lim_mod.DEFAULT_LIMITS["vibration_mss"]


@pytest.mark.parametrize("bad, msg", [
    ({"vibration": {"pass_below": 1}}, "unknown limit"),
    ({"vibration_mss": {"pass": 1}}, "no field"),
    ({"vibration_mss": {"pass_below": "30"}}, "must be a number"),
    ({"vibration_mss": {"pass_below": 90}}, "must not exceed"),
])
def test_bad_limits_are_rejected(bad, msg):
    with pytest.raises(lim_mod.LimitsError, match=msg):
        lim_mod.merge(bad)


# --------------------------------------------------------------------------- outputs

def test_json_schema(healthy):
    d = json.loads(report.as_json(healthy))
    assert d["tool"] == "ardupilot-log-doctor" and d["schema"] == 1
    assert d["status"] == "PASS" and d["counts"]["FAIL"] == 0
    vib = next(c for c in d["checks"] if c["name"].startswith("Vibration"))
    assert isinstance(vib["value"], float) and vib["status"] == "PASS"


def test_reports_never_contain_coordinates(healthy):
    outputs = report.markdown(healthy, "fig.png") + report.as_json(healthy)
    for coord in HOME:
        assert f"{coord:.3f}" not in outputs and f"{abs(coord):.3f}" not in outputs


def test_markdown_sections(tmp_path):
    md = report.markdown(run(tmp_path, gps=False, cell_min=3.1))
    assert "## Not checked" in md and "## What to look at (most serious first)" in md


# --------------------------------------------------------------------------- command line

def test_cli_exit_codes(tmp_path, capsys):
    good = flight_log(str(tmp_path / "good.bin"), duration=DUR)
    bad = flight_log(str(tmp_path / "bad.bin"), duration=DUR, arming_check=0)
    empty = tmp_path / "empty.bin"
    empty.write_bytes(b"")
    assert main([good, "--no-files"]) == EXIT_OK
    assert main([bad, "--no-files"]) == EXIT_OK                       # FAIL alone doesn't change the exit code
    assert main([bad, "--no-files", "--strict"]) == EXIT_FAILED_CHECKS
    assert main([str(empty), "--no-files"]) == EXIT_UNREADABLE
    assert main([good, str(empty), "--no-files"]) == EXIT_UNREADABLE   # the good one is still reported
    assert "ALL CHECKS PASSED" in capsys.readouterr().out


def test_cli_json_for_several_logs_is_one_valid_document(tmp_path, capsys):
    a = flight_log(str(tmp_path / "a.bin"), duration=DUR)
    b = flight_log(str(tmp_path / "b.bin"), duration=DUR, vibe=45.0)
    assert main([a, b, "--json", "--no-files"]) == EXIT_OK
    data = json.loads(capsys.readouterr().out)
    assert [d["status"] for d in data] == ["PASS", "WARN"]


def test_cli_writes_report_and_figure(tmp_path):
    p = flight_log(str(tmp_path / "w.bin"), duration=DUR)
    out = tmp_path / "out"
    assert main([p, "--out", str(out)]) == EXIT_OK
    assert (out / "w_doctor.md").is_file() and (out / "w_doctor.png").stat().st_size > 10_000


def test_cli_print_limits(capsys):
    assert main(["--print-limits"]) == EXIT_OK
    assert json.loads(capsys.readouterr().out) == lim_mod.DEFAULT_LIMITS


def test_cli_bad_limits_file(tmp_path, capsys):
    p = flight_log(str(tmp_path / "l.bin"), duration=DUR)
    f = tmp_path / "bad.json"
    f.write_text("{not json")
    assert main([p, "--limits", str(f), "--no-files"]) == EXIT_UNREADABLE
    assert "not valid JSON" in capsys.readouterr().err
