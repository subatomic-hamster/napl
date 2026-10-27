"""calib.toml schema (spec 4.3) and the bench fitter (spec 10): defaults, round trip, rejection, provenance, the example in
docs/cuas2/bench_validation.md, and a synthetic bench campaign with known parameters that fit_all must recover."""

import math
import re
from dataclasses import fields, replace
from pathlib import Path

import numpy as np
import pytest
from napl.cuas2 import ballistics, calib
from napl.cuas2.launcher import LauncherModel

DOC = Path(__file__).resolve().parents[2] / "docs" / "cuas2" / "bench_validation.md"


def test_defaults_load_with_no_file(tmp_path, monkeypatch):
    assert calib.loads("") == calib.Calib()
    monkeypatch.setattr(calib, "DEFAULT_PATH", tmp_path / "none.toml")
    assert calib.load() == calib.Calib()
    with pytest.raises(FileNotFoundError):
        calib.load(tmp_path / "none.toml")  # an explicit path must exist
    c = calib.Calib()
    assert (c.launcher.k_slip, c.launcher.v_cap_mps, c.ball.d_mm, c.wheel.J_kgm2) == (
        0.8,
        38.0,
        22.2,
        4.0e-6,
    )
    assert c.rpm_cap == 18143 and c.firmware_keys() == {23: 40000, 24: 380}
    assert all(line.endswith("[est]") for line in c.provenance())


def _changed(v):
    if isinstance(v, tuple):
        return (9.0, 9.0) if v else ((18.2, 0.7), (20.2, 0.8))
    if isinstance(v, int):
        return v + 1
    return v * 0.9 if v else 0.1  # 0.9 keeps k_slip, eta, v_cap inside their limits


@pytest.mark.parametrize("sec", ["ball", "launcher", "wheel", "feed"])
def test_every_key_round_trips(sec):
    c0 = calib.Calib()
    for f in fields(getattr(c0, sec)):
        obj = getattr(c0, sec)
        c1 = replace(
            c0, **{sec: replace(obj, **{f.name: _changed(getattr(obj, f.name))})}
        )
        c2 = calib.loads(calib.dumps(c1))
        assert c2 == c1, f.name
        assert getattr(c2, sec) != obj, f"{f.name} did not change"
    assert calib.loads(calib.dumps(c0)) == c0


def test_unknown_and_invalid_rejected():
    for bad in ("[launcher]\nk_slp = 0.8\n", "[ball]\nd = 22\n", "[wheels]\nkv = 1\n", "[launcher.wheel]\nnope = 1\n", "[launcher]\nk_slip = 1.2\n",
                '[launcher]\nk_slip = "fast"\n', "[launcher]\nspread_mrad = [1.0]\n", "[ball]\nd_mm = -1\n", '[launcher]\nk_slip_src = "guess"\n'):  # fmt: skip
        with pytest.raises(ValueError):
            calib.loads(bad)


def test_v_cap_above_38_rejected():
    assert calib.loads("[launcher]\nv_cap_mps = 38.0\n").launcher.v_cap_mps == 38.0
    with pytest.raises(ValueError, match="38"):
        calib.loads("[launcher]\nv_cap_mps = 38.1\n")
    assert calib.loads("[launcher]\nv_cap_mps = 36.5\n").firmware_keys()[24] == 365


def test_src_provenance_parsed():
    c = calib.loads(
        '[launcher]\nk_slip = 0.74\nk_slip_src = "bench"\n[ball]\ncd = 0.5\n[launcher.wheel]\nkv = 1750.0\nkv_src = "bench"\n'
    )
    assert c.src == {"launcher.k_slip": "bench", "launcher.wheel.kv": "bench"}
    lines = {ln.split(" = ")[0]: ln for ln in c.provenance()}
    assert lines["launcher.k_slip"].endswith("[bench]") and lines["ball.cd"].endswith(
        "[est]"
    )


def test_k_slip_gap_table_overrides():
    c = calib.loads(
        "[launcher]\ngap_mm = 19.2\nk_slip_gap = [[18.2, 0.70], [20.2, 0.80]]\n"
    )
    assert c.k_slip_at(17500) == pytest.approx(0.75)
    c = calib.loads(
        "[launcher]\nk_slip = 0.8\nk_slip_pm_per_krpm = -0.01\nrpm_max = 17500.0\n"
    )
    assert c.k_slip_at(15500) == pytest.approx(0.82)


def test_doc_calib_example_loads():
    """The calib.toml example in bench_validation.md loads and carries bench provenance on the fitted keys."""
    text = DOC.read_text()
    m = re.search(r"<!-- calib-example -->\s*```toml\n(.*?)```", text, re.DOTALL)
    assert m, "bench_validation.md has no <!-- calib-example --> toml block"
    c = calib.loads(m.group(1))
    assert c.src["launcher.k_slip"] == "bench" and c.launcher.v_cap_mps <= 38.0
    assert calib.loads(calib.dumps(c)) == c


# ---- synthetic bench campaign --------------------------------------------------------------------------------------------------------
TRUE = calib.loads(
    """
[ball]
cd = 0.55
[launcher]
k_slip = 0.74
sigma_v_rel = 0.025
sigma_v_floor_mps = 0.4
spread_mrad = [12.0, 8.0]
bias_mrad = [3.0, -2.0]
cl_k = 0.6
"""
)


def _hit(shot, r):
    """Where a shot crosses the sheet at x = r: (y_left, z - z0) in m. Barrel level at z0 = 10 m, the sampled direction error applied."""
    eh, ev = shot.dir_err
    u = np.array(
        [math.cos(ev) * math.cos(eh), math.cos(ev) * math.sin(eh), math.sin(ev)]
    )
    t, p, _ = ballistics.fly([0, 0, 10.0], shot.v0 * u, ballistics.omega_vec(u, shot.omega), (shot.d, shot.m, shot.cd), cl_k=TRUE.launcher.cl_k,
                             spin_tau=TRUE.launcher.spin_tau_s, dt=0.002, stop_x=r)  # fmt: skip
    i = np.searchsorted(p[:, 0], r)
    f = (r - p[i - 1, 0]) / (p[i, 0] - p[i - 1, 0])
    q = p[i - 1] + f * (p[i] - p[i - 1])
    return q[1], q[2] - 10.0


def synth_log(seed=5, chrono_sigma=0.15):
    rng = np.random.default_rng(seed)
    m = LauncherModel(TRUE, rng)
    rows = []

    def add(step, n, rpm, diff, r=None):
        rt, rb = m.split(rpm, diff)
        b = m.sample_shots(n, rng, (rt, rb))
        for k in range(n):
            shot = type(
                "S",
                (),
                dict(
                    v0=b.v0[k],
                    dir_err=tuple(b.dir_err[k]),
                    omega=b.omega[k],
                    d=b.d[k],
                    m=b.m[k],
                    cd=b.cd[k],
                ),
            )
            row = dict(
                step=step,
                gap_mm=19.2,
                diff_pm=diff,
                rpm_top=rt,
                rpm_bot=rb,
                v_mps=b.v0[k] + rng.normal(0, chrono_sigma),
            )
            if r:
                y, z = _hit(shot, r)
                row.update(range_m=r, dx_mm=y * 1000, dy_mm=z * 1000)
            rows.append(row)

    for rpm in (10000, 12500, 15000, 16500, 17500):  # B2
        add("B2", 60, rpm, 15)
    for r in (5.0, 10.0):  # B3 / B5
        add("B3", 40, 17000, 0, r)
    for d in (-15, 0, 15, 30, 45):  # B4
        add("B4", 60, 17000, d, 10.0)
    nan = float("nan")
    return {
        k: [r.get(k, "" if k == "step" else nan) for r in rows] for k in calib.COLUMNS
    }


@pytest.fixture(scope="module")
def fitted():
    log = synth_log()
    return calib.fit_all(log, calib.Calib()), log


def test_fitter_recovers_known_parameters(fitted):
    (c, rep), _ = fitted
    lo = c.launcher
    assert lo.k_slip == pytest.approx(0.74, abs=0.01)
    assert lo.sigma_v_rel == pytest.approx(0.025, abs=0.01)
    s_fit = math.hypot(
        lo.sigma_v_rel * 30, lo.sigma_v_floor_mps
    )  # total speed scatter at 30 m/s (the quantity the fire control uses)
    s_true = math.hypot(TRUE.launcher.sigma_v_rel * 30, TRUE.launcher.sigma_v_floor_mps)
    assert s_fit == pytest.approx(s_true, rel=0.15)
    assert c.ball.cd == pytest.approx(0.55, abs=0.05)
    assert lo.bias_mrad[0] == pytest.approx(3.0, abs=1.5) and lo.bias_mrad[
        1
    ] == pytest.approx(-2.0, abs=1.5)
    assert lo.spread_mrad[0] == pytest.approx(12.0, rel=0.2) and lo.spread_mrad[
        1
    ] == pytest.approx(8.0, rel=0.25)
    assert (
        lo.cl_k == pytest.approx(0.6, rel=0.3)
    )  # seeds 5/11/12/13 give 0.52/0.50/0.65/0.60: ~12 % statistical error for this campaign size
    assert rep["fit_cl"]["best_diff_pm"] in (-15, 0, 15, 30, 45)


def test_fitter_marks_bench_and_round_trips(fitted):
    (c, _), _ = fitted
    for k in (
        "launcher.k_slip",
        "launcher.sigma_v_rel",
        "launcher.spread_mrad",
        "launcher.bias_mrad",
        "ball.cd",
        "launcher.cl_k",
    ):
        assert c.src[k] == "bench", k
    assert c.src.get("launcher.k_spin", "est") == "est"
    assert calib.loads(calib.dumps(c)) == c


def test_cli_fit_writes_a_calib(tmp_path, fitted):
    _, log = fitted
    p, out = tmp_path / "shots.csv", tmp_path / "calib.toml"
    calib.write_log(p, log)
    none = tmp_path / "none.toml"
    assert (
        calib.main(["fit", str(p), "--gap", "18.2", "--out", str(none)]) == 0
    )  # no rows at that gap: every fit skipped, defaults written
    assert (
        calib.load(none).launcher.k_slip == 0.8
        and calib.load(none).launcher.gap_mm == 18.2
        and not calib.load(none).src
    )
    assert calib.main(["fit", str(p), "--gap", "19.2", "--out", str(out)]) == 0
    c = calib.load(out)
    assert (
        c.launcher.k_slip == pytest.approx(0.74, abs=0.01)
        and c.src["launcher.k_slip"] == "bench"
    )
    back = calib.read_log(p)
    assert len(back["rpm_top"]) == len(log["rpm_top"]) and back["step"][0] == "B2"


def test_fitter_negative_control_wrong_model_is_visible():
    """A log from a different truth gives a different k_slip: the fit is not returning its prior."""
    log = synth_log(seed=9)
    log["v_mps"] = [v * 1.1 for v in log["v_mps"]]  # chronograph reads 10 % high
    c, _ = calib.fit_all(log, calib.Calib())
    assert c.launcher.k_slip == pytest.approx(0.74 * 1.1, abs=0.015)
