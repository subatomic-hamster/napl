"""Leg study runner (examples/dog/legdiag.py): the torque the driver can deliver follows the Qdd law (torque-speed line, back-driven cap), the window
statistics and the fall classification do what the module docstring says, and one short real run records one row per control tick."""

import sys
from pathlib import Path

import napl
import numpy as np
import pytest

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "examples" / "dog"))
import legdiag as G

needs_kernel = pytest.mark.skipif(napl._napl is None, reason="kernel not built")


def consts(n=1):
    """GIM8108-8 hip constants (steadywin-gim8108-8.toml): Ke 0.1789, N 8, eta 0.699, i_limit 22 A, R 0.67 ohm."""
    return {
        k: np.full(n, x)
        for k, x in dict(
            tau_ctl=22.0, ke=0.1789, n=8.0, eta=0.699, i_lim=22.0, r=0.67
        ).items()
    }


def test_avail_is_the_qdd_torque_speed_law():
    L, v = consts(), np.array([33.6])
    stall = (
        0.1789 * 8 * 0.699 * 22.0
    )  # 22 N m driving at rest (the published stall torque)
    assert G.avail(L, np.array([[0.0]]), v, np.array([[1.0]]))[0, 0] == pytest.approx(
        stall, rel=1e-3
    )
    corner = (33.6 - 0.67 * 22.0) / (
        0.1789 * 8
    )  # rad/s: the voltage starts to cut the torque
    assert G.avail(L, np.array([[corner * 0.99]]), v, np.array([[1.0]]))[
        0, 0
    ] == pytest.approx(stall, rel=0.05)
    w0 = 33.6 / (0.1789 * 8)  # no load
    assert G.avail(L, np.array([[w0]]), v, np.array([[1.0]]))[0, 0] == pytest.approx(
        0.0, abs=1e-6
    )
    cap = G.avail(L, np.array([[w0]]), v, np.array([[-1.0]]))[
        0, 0
    ]  # back-driven: k = 1, the full current at any speed
    assert cap == pytest.approx(0.1789 * 8 * 22.0, rel=1e-6) and cap > stall


def rec(n, tau, dq=0.0, tilt=0.0):
    t = np.arange(n) * 0.002
    z = np.zeros((n, 12), np.float32)
    return dict(t=t, q=z + 0.3, dq=z + dq, tau=z + tau, i=np.full(n, 40.0), v=np.full(n, 33.6), tilt=np.full(n, tilt), slip=np.zeros(n), relax=np.arange(n) * 0.0,
                wbc_fail=np.zeros(n), mpc_fail=np.zeros(n))  # fmt: skip


def test_window_counts_saturated_ticks_per_class_and_current():
    L, qlo, qhi = consts(12), np.full(12, -1.0), np.full(12, 1.0)
    w = G.window(
        rec(500, 22.0), L, qlo, qhi, 0.0, 1.0
    )  # every joint at the stall torque, mid-range
    assert (
        w["sat_any"] == 1.0
        and w["sat"] == [1.0, 1.0, 1.0]
        and w["rng_any"] == 0.0
        and w["i_over_bms"] == 0.0
    )
    w = G.window(
        rec(500, 5.0), L, qlo, qhi, 0.0, 1.0
    )  # light torque, q = 0.3 far from the stops
    assert w["sat_any"] == 0.0 and w["rng_any"] == 0.0
    w = G.window(
        rec(500, 5.0), L, qlo, np.full(12, 0.31), 0.0, 1.0
    )  # q = 0.3 is 0.01 below the stop
    assert w["rng_any"] == 1.0
    assert G.window(rec(500, 5.0), L, qlo, qhi, 5.0, 6.0) is None  # empty window


class Fake:
    def __init__(self, fell):
        self.fell, self.ctl = (
            fell,
            type(
                "C",
                (),
                {
                    "legs": [
                        type(
                            "L", (), {"q_lo": np.full(3, -1.0), "q_hi": np.full(3, 1.0)}
                        )()
                    ]
                    * 4
                },
            )(),
        )


def test_fall_is_classified_by_the_window_before_the_tilt_onset():
    L = consts(12)
    r = rec(1500, 22.0, tilt=0.0)  # 3 s; legs saturated all along
    r["tilt"][1000:] = 40.0  # the trunk goes at t = 2.0 s
    d = G.diagnose(Fake(True), 1500, r, L)
    assert (
        d["t_on"] == pytest.approx(1.998)
        and d["cls"] == "actuator"
        and d["w_on"]["sat_any"] == 1.0
    )
    r2 = rec(1500, 5.0, tilt=0.0)
    r2["tilt"][1000:] = 40.0
    assert (
        G.diagnose(Fake(True), 1500, r2, L)["cls"] == "control"
    )  # nothing saturated, no range stop
    r3 = rec(1500, 5.0, tilt=0.0)
    r3["q"][:] = 0.99  # every joint on its range stop
    r3["tilt"][1000:] = 40.0
    assert G.diagnose(Fake(True), 1500, r3, L)["cls"] == "kinematic"
    up = G.diagnose(
        Fake(False), 2000, rec(2000, 5.0), L
    )  # a run that stayed up (4 s): the baseline after t = 3 s only
    assert "w_on" not in up and up["base"]["n"] == 500


@needs_kernel
def test_fly_records_one_row_per_tick_and_limits_match_the_build(tmp_path):
    import dogcourse as C
    import dogwalk as W

    model = C.scene(tmp_path, "flat", 1)
    L = G.limits(model)
    assert L["tau_ctl"].shape == (12,) and L["n"][2] == pytest.approx(
        12.0
    )  # knee: 8 x linkage ratio 1.5
    assert (L["ke"] * L["n"] * L["eta"] * L["i_lim"])[1] == pytest.approx(
        22.0, rel=0.01
    )  # hip stall = the published 22 N m
    p = dict(W.DEFAULTS, seed=1, course="flat", v_cmd=0.0, duration=0.6, leg_ctl="wbc")
    c, k_end, r = G.fly(model, p)
    assert (
        k_end == 300
        and r["tau"].shape == (300, 12)
        and r["i"].shape == (300,)
        and not c.fell
    )
    assert (
        np.abs(r["tau"]).max() > 0.0 and r["v"].min() > 25.0
    )  # standing up draws torque; pack voltage is the 8S bus
