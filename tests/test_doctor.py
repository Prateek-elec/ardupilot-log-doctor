import os

import numpy as np
import pytest

from log_doctor.doctor import (FAIL, PASS, WARN, grade, grade_high, in_segments, near_transition,
                               to_segments, wrap180)
from log_doctor.names import err_text, mode_name


def test_grade_bands():
    assert grade(10, 30, 60) == PASS
    assert grade(45, 30, 60) == WARN
    assert grade(60, 30, 60) == FAIL
    assert grade_high(12, 10, 6) == PASS
    assert grade_high(7, 10, 6) == WARN
    assert grade_high(3, 10, 6) == FAIL


def test_segments_merge_short_gaps_and_drop_short_runs():
    t = np.arange(0, 20, 1.0)
    mask = np.zeros(20, bool)
    mask[2:8] = True        # 6 s run
    mask[9:12] = True       # 1 s gap -> merged with the first
    mask[16:17] = True      # 1 s blip -> dropped
    assert to_segments(t, mask, min_len=3, merge_gap=2) == [(2.0, 12.0)]


def test_in_segments_and_transitions():
    t = np.arange(0, 100, 1.0)
    flights = [(10.0, 50.0)]
    sel = in_segments(t, flights)
    assert sel[10] and sel[50] and not sel[9] and not sel[51]
    near = near_transition(t, flights, before=2, after=4)
    assert near[8] and near[14] and not near[15] and near[48] and near[54] and not near[30]


def test_wrap180():
    assert wrap180(350) == pytest.approx(-10)
    assert wrap180(-190) == pytest.approx(170)


def test_err_and_mode_names():
    assert err_text(3, 4) == "Compass: unhealthy"
    assert err_text(6, 1) == "Battery failsafe: triggered"
    assert err_text(6, 0) == "Battery failsafe: resolved"
    assert err_text(12, 1) == "Crash check: crash detected"
    assert mode_name("plane", 5) == "FBWA"
    assert mode_name("copter", 5) == "LOITER"
    assert mode_name("copter", 99) == "mode 99"


LOG = os.environ.get("LOGDOCTOR_TEST_LOG")


@pytest.mark.skipif(not LOG, reason="set LOGDOCTOR_TEST_LOG to a .BIN file to run the end-to-end test")
def test_end_to_end(tmp_path):
    from log_doctor import report
    from log_doctor.doctor import analyse
    from log_doctor.plot import figure

    r = analyse(LOG)
    assert r.checks, "no checks ran"
    assert r.firmware != "unknown firmware"
    md = report.markdown(r, "fig.png")
    assert "Checks" in md
    js = report.as_json(r)
    assert '"verdict"' in js
    # Privacy: the report never contains absolute GPS coordinates.
    gps = r.log.get("GPS")
    if gps is not None and (gps["Status"] >= 3).any():
        lat = gps["Lat"][gps["Status"] >= 3][0]
        assert f"{lat:.3f}" not in md and f"{lat:.3f}" not in js
    figure(r, str(tmp_path / "fig.png"))
    assert (tmp_path / "fig.png").stat().st_size > 10_000
