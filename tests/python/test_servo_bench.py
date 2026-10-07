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


# The 6 V no-load current passes since the part's i0_visc (viscous loss share, solved from the 6 V / 7.4 V pair) is built into
# the servo: it was a strict xfail (constant i0: 0.150 A vs 0.13 A, +15 %). It is a fitted point, not an independent one.
@pytest.mark.parametrize("i", COMPARED)
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


def test_no_load_current_follows_the_two_point_loss_identification():
    r = {(x["quantity"], x["V"]): x for x in rows()}
    assert (
        abs(r[("no-load current A", 6.0)]["err_pct"]) < 1.0
    )  # fitted point (i0_visc), not independent
    assert abs(r[("no-load current A", 7.4)]["err_pct"]) < 1.0
    cfg = sb.servo_config()
    assert cfg["motor"]["i0_visc"] == pytest.approx(0.6916, abs=1e-4)
    assert cfg["motor"]["i0_w"] == pytest.approx(1881.68, abs=0.01)


def test_built_servo_powers_up_with_the_factory_protections_and_scales():
    cfg = sb.servo_config()
    got = {
        k: cfg[k]
        for k in (
            "min_voltage",
            "max_voltage",
            "max_temp",
            "overload_load",
            "overload_time",
            "unload_mask",
            "pid_scale",
            "d_scale",
        )
    }
    assert got == pytest.approx(
        {
            "min_voltage": 4.0,
            "max_voltage": 8.0,
            "max_temp": 70.0,
            "overload_load": 0.8,
            "overload_time": 2.0,
            "unload_mask": 36,
            "pid_scale": 4000,
            "d_scale": 8000,
        }
    )


def test_holding_sag_baseline_before_calibration():
    s = sb.holding_sag(pid_scale=8192)["shoulder_lift"]
    print(s)
    # 8.99 deg with the old constant-Coulomb loss (i0 = 0.15 A at all speeds); the viscous loss share i0_visc = 0.69 leaves
    # a standstill friction of 31 % of that, so more duty (0.27 vs 0.20) is needed to hold: 11.95 deg
    assert s["sag_deg"] == pytest.approx(11.95, abs=0.5)
    assert math.copysign(1, s["duty"]) < 0


def test_holding_sag_as_built_follows_the_servo_law():
    cfg = sb.servo_config()
    assert cfg["pid_scale"] == 4000 and cfg["p"] == 16
    s = sb.holding_sag()["shoulder_lift"]
    print(s)
    # static servo law: duty = p e / pid_scale  ->  e = duty pid_scale / p (D, I are 0 at rest)
    pred = abs(s["duty"]) * cfg["pid_scale"] / cfg["p"]
    assert abs(s["counts"]) == pytest.approx(pred, rel=0.15)
    assert (
        abs(s["sag_deg"]) < 8.0
    )  # 6.50 deg (4.4 deg before the speed-dependent loss split, see i0_visc)
