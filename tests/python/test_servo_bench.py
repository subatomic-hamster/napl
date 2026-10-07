"""STS3215 C001 servo model vs the Feetech datasheet points (6 V, 7.4 V) and the SO-101 holding sag (napl.servo_bench)."""

import functools
import math

import pytest
from napl import servo_bench as sb


@functools.cache
def rows():
    return sb.datasheet_rows()


def test_prints_tables():
    print()
    sb.main()


# row order per voltage: stall torque, stall current, no-load speed, no-load current, step (not a comparison)
COMPARED = [i for i in range(10) if i % 5 != 4]
XFAIL = {
    3: pytest.mark.xfail(
        strict=True,
        reason="6 V no-load current: the model motor i0 is a constant (the 7.4 V value) so it cannot follow the datasheet's "
        "lower 6 V current (sim 0.150 A vs 0.13 A, +15 %); a constant-friction model limit",
    )
}


@pytest.mark.parametrize(
    "i", [pytest.param(i, marks=XFAIL.get(i, ())) for i in COMPARED]
)
def test_datasheet_within_tolerance(i):
    r = rows()[i]
    assert r["pass"], (
        f"{r['quantity']} @ {r['V']} V: sim {r['sim']:.4g} vs datasheet {r['datasheet']:.4g} ({r['err_pct']:+.1f} %)"
    )


# The datasheet "0.192 s / 60 deg" is the no-load speed restated (60 deg / (52 rpm * 6 deg/s per rpm) = 0.1923 s), not a
# step-response measurement, so there is no step-time datasheet comparison; these test the model against that speed instead.
@pytest.fixture(scope="module", params=[6.0, 7.4])
def chk(request):
    return sb.step_checks(request.param)


def test_full_duty_step_cannot_beat_the_no_load_speed(chk):
    assert chk["t"] >= 0.9 * chk["t_ds"], chk


def test_step_excess_is_the_spin_up_time_constant(chk):
    assert chk["excess"] == pytest.approx(chk["tau"], rel=0.15), chk


def test_step_peak_speed_is_the_no_load_speed(chk):
    assert chk["peak_ratio"] == pytest.approx(1.0, abs=0.10), chk


def test_holding_sag_baseline_before_calibration():
    s = sb.holding_sag(pid_scale=8192)["shoulder_lift"]
    print(s)
    assert s["sag_deg"] == pytest.approx(8.99, abs=0.5)
    assert math.copysign(1, s["duty"]) < 0


def test_holding_sag_as_built_follows_the_servo_law():
    cfg = sb.servo_config()
    assert cfg["pid_scale"] == 4000 and cfg["p"] == 16
    s = sb.holding_sag()["shoulder_lift"]
    print(s)
    # static servo law: duty = p e / pid_scale  ->  e = duty pid_scale / p (D, I are 0 at rest)
    pred = abs(s["duty"]) * cfg["pid_scale"] / cfg["p"]
    assert abs(s["counts"]) == pytest.approx(pred, rel=0.15)
    assert abs(s["sag_deg"]) < 5.0
