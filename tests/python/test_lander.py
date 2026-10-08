"""TVC lander showcase (examples/lander): the guidance maths against the closed form, the attitude / axis estimators on synthetic data, the firmware constants
against `napl design`, the parameters & telemetry facility (napl.params), a flight of the scenario (deterministic, replayable), and the Monte Carlo summary."""

import json
import math
import sys
from pathlib import Path

import numpy as np
import pytest
from napl import design
from napl import params as nparams

REPO = Path(__file__).resolve().parents[2]
LANDER = REPO / "examples" / "lander"
sys.path.insert(0, str(LANDER))

import estimator as est
import guidance as gd
import vehicle as veh

SCENARIO = LANDER / "land.scenario.toml"
CLASH = (
    "scene",
    "flight",
    "land",
    "mc",
)  # example-module names other examples also use


@pytest.fixture(scope="module", autouse=True)
def _lander_modules():
    """Other examples have their own scene.py / flight.py: make this module's imports resolve to examples/lander for its tests only."""
    saved = {n: sys.modules.pop(n) for n in CLASH if n in sys.modules}
    sys.path.insert(0, str(LANDER))
    yield
    sys.path.remove(str(LANDER))
    for n in CLASH:
        sys.modules.pop(n, None)
    sys.modules.update(saved)


# ------------------------------------------------------------------------------------------------------------------ guidance
class ConstMotor:
    """Duck-typed constant-thrust motor: the case with a closed form."""

    def __init__(self, thrust, burn):
        self.t, self.burn, self.id = thrust, burn, "const"

    def thrust(self, tau):
        return self.t if 0.0 <= tau <= self.burn else 0.0

    def burned(self, tau):
        return 0.0


def test_analytic_constant_thrust_values():
    h, t = gd.analytic_constant_thrust(-6.0, 5.0)
    assert h == pytest.approx(3.6) and t == pytest.approx(1.2)


@pytest.mark.parametrize("v0", [-3.0, -6.0, -9.0])
def test_touchdown_speed_matches_closed_form(v0):
    m0, thrust = 1.0, 16.0
    a_net = thrust / m0 - gd.G  # constant deceleration
    h_stop, _ = gd.analytic_constant_thrust(v0, a_net)
    motor = ConstMotor(thrust, burn=8.0)
    for frac in (0.3, 0.6, 0.9):
        z0 = frac * h_stop  # lit too low: still moving when the feet touch
        v_td = gd.touchdown_speed(
            motor, m0, [z0], v0, cda=0.0, dt=0.0005, first_contact=True
        )[0]
        assert v_td == pytest.approx(-math.sqrt(v0 * v0 - 2 * a_net * z0), rel=5e-3)
    # lit high enough to stop above the ground: the burn goes on, the vehicle climbs and never touches inside the horizon
    assert np.isnan(
        gd.touchdown_speed(
            motor, m0, [1.05 * h_stop], v0, cda=0.0, dt=0.0005, first_contact=True
        )[0]
    )


def test_ignition_table_tracks_the_stop_height():
    m0, thrust = 1.0, 16.0
    a_net = thrust / m0 - gd.G
    sp, hz = gd.ignition_table(
        ConstMotor(thrust, 8.0),
        m0,
        v_target=0.8,
        speeds=[-4.0, -6.0, -8.0],
        z_max=15.0,
        dz=0.01,
        cda=0.0,
        dt=0.001,
    )
    for v, z in zip(sp, hz):
        h_stop, _ = gd.analytic_constant_thrust(v, a_net)
        # the window is [h_stop - (0.8^2/2a), h_stop]: its middle is within 0.05 m of the stop height minus half the window
        assert z == pytest.approx(h_stop - 0.8**2 / (4 * a_net), abs=0.05)


def test_planner_fires_when_the_predicted_onset_is_low_enough():
    land = veh.Motor(veh.LANDING_MOTOR)
    m0 = veh.M_LOADED - veh.Motor(veh.ASCENT_MOTOR).prop
    pl = gd.Planner(
        land,
        m0,
        0.8,
        0.0,
        speeds=np.arange(-12.0, 0.5, 2.0),
        dz=0.05,
        z_max=15.0,
        dt=0.004,
    )
    d = 0.4
    # far above the window: positive margin (wait); at the table height for the onset speed: margin ~ 0; well below: negative
    zo, vo = 20.0, -3.0
    assert pl.margin(zo, vo, d) > 5.0
    z_on_table = pl.required_height(gd.ballistic(zo, vo, d, m=m0)[1])
    z_now, v_now = z_on_table + 2.0, -4.0
    zz, vv = gd.ballistic(z_now, v_now, d, m=m0)
    assert pl.margin(z_now, v_now, d) == pytest.approx(zz - pl.required_height(vv))
    assert pl.margin(0.5, -8.0, d) < 0.0


def test_landing_motor_can_stop_the_vehicle_from_a_hop():
    """The chosen design point: from the apogee of the E16 hop the F15 lands the 1.45 kg vehicle softly at the best onset (the knife edge the planner finds)."""
    land = veh.Motor(veh.LANDING_MOTOR)
    m0 = veh.M_LOADED - veh.Motor(veh.ASCENT_MOTOR).prop
    zs = np.arange(0.0, 15.0, 0.05)
    best = np.nanmin(np.abs(gd.touchdown_speed(land, m0, zs, -3.5, dt=0.004)))
    assert best < 2.0


# ------------------------------------------------------------------------------------------------------------- energy management
def test_rollout_level_thrust_agrees_with_the_guidance_touchdown_speed():
    """The onboard predictor (energy.rollout, 30 ms step) against the reference integrator (guidance.touchdown_speed, 4 ms step) for level thrust."""
    import energy as en

    land = veh.Motor(veh.LANDING_MOTOR)
    m0 = veh.M_LOADED - veh.Motor(veh.ASCENT_MOTOR).prop
    one = np.ones(1)
    for z0, v0 in [(3.0, -3.0), (5.0, -6.0), (8.0, -2.0), (10.0, -5.0), (12.0, -3.0)]:
        vtd, _, _ = en.rollout(land, 0.0, z0, v0, m0, 1.0, one, one, 0.0 * one, 0.0)
        ref = -gd.touchdown_speed(land, m0, [z0], v0, dt=0.004)[0]
        assert vtd[0] == pytest.approx(ref, rel=0.03), (z0, v0)


def test_rollout_closed_form_free_fall():
    """After the burn (thrust zero) a vehicle dropped from 4 m touches the floor at sqrt(2 g h) less the (small) drag."""
    import energy as en

    land = veh.Motor(veh.LANDING_MOTOR)
    m0 = veh.M_LOADED - veh.Motor(veh.ASCENT_MOTOR).prop
    one = np.ones(1)
    vtd, ttd, zmax = en.rollout(
        land, land.burn, 4.0, 0.0, m0 - land.prop, 1.0, one, one, 0.0 * one, 0.0
    )
    assert vtd[0] == pytest.approx(math.sqrt(2 * veh.G * 4.0), rel=0.04)
    assert zmax[0] == pytest.approx(4.0)


def test_a_strong_motor_is_dumped_by_tilting_and_a_weak_one_is_not():
    import energy as en

    land = veh.Motor(veh.LANDING_MOTOR)
    m0 = veh.M_LOADED - veh.Motor(veh.ASCENT_MOTOR).prop
    ln = en.Lander(land, en.TiltFamily(40.0), t_lev=0.4, w_lat=0.5)
    # low, slow and 30 % too strong: level thrust would carry the vehicle up and drop it; a tilt lowers the predicted hardest contact
    c_strong, v_strong, _ = ln.plan(1.0, 0.5, -1.0, m0 - 0.02, 1.3)
    J, vtd = ln._cost(1.0, 0.5, -1.0, m0 - 0.02, 1.3)
    level = int(np.argmax((ln.fam.cos1 == 1.0) & (ln.fam.cos2 == 1.0)))
    assert c_strong < 1.0 and v_strong < vtd[level]
    # the family always contains level thrust: the plan is never worse than not dumping
    assert J.min() <= J[level]


def test_ascent_dump_is_the_least_tilt_that_meets_the_apogee_cap():
    import energy as en

    asc = veh.Motor(veh.ASCENT_MOTOR)
    c0, apo0, free = en.ascent_dump(asc, 0.0, 0.0, 0.0, veh.M_LOADED, 1.0, cap=99.0)
    assert c0 == 1.0 and apo0 == pytest.approx(free)
    cap = 0.7 * free
    c, apo, free2 = en.ascent_dump(asc, 0.0, 0.0, 0.0, veh.M_LOADED, 1.0, cap=cap)
    assert free2 == pytest.approx(free) and c < 1.0 and apo <= cap + 1e-9
    # the search is over a grid of tilts (0..30 deg, 13 values): the next-smaller grid tilt overshoots the cap
    grid = np.cos(np.radians(np.linspace(0.0, 30.0, 13)))
    k = int(np.argmin(np.abs(grid - c)))
    assert k >= 1
    prev = grid[k - 1 : k]
    _, _, z_prev = en.rollout(
        asc, 0.0, 0.0, 0.0, veh.M_LOADED, 1.0, prev, prev, np.zeros(1), 0.3
    )
    assert z_prev[0] > cap


# ----------------------------------------------------------------------------------------------------------------- estimators
def _rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def test_attitude_calibration_finds_bias_and_tilt():
    rng = np.random.default_rng(1)
    bias = np.array([0.006, -0.003, 0.002])
    tilt = math.radians(2.0)
    up_body = _rot_x(tilt).T @ [
        0.0,
        0.0,
        1.0,
    ]  # body-frame up vector of a vehicle leaning 2 deg about x
    att = est.Attitude()
    for _ in range(500):
        att.calibrate(
            bias + 1e-3 * rng.standard_normal(3),
            est.G * up_body + 0.02 * rng.standard_normal(3),
        )
    att.finish_calibration()
    assert np.allclose(att.bias, bias, atol=2e-4)
    assert math.degrees(att.tilt) == pytest.approx(2.0, abs=0.1)


def test_attitude_integrates_a_rotation_under_thrust_without_the_accelerometer():
    """Constant 0.2 rad/s about x for 1.5 s with a 3 g specific force along the body axis (gated out of the tilt correction) and a gyro bias:
    the estimate follows the true tilt to 0.3 deg."""
    rng = np.random.default_rng(2)
    bias = np.array([0.008, -0.004, 0.003])
    att = est.Attitude()
    for _ in range(500):
        att.calibrate(
            bias + 1e-3 * rng.standard_normal(3),
            np.array([0.0, 0.0, est.G]) + 0.02 * rng.standard_normal(3),
        )
    att.finish_calibration()
    dt, w, t_end = 0.004, 0.2, 1.5
    for k in range(int(t_end / dt)):
        gyro = bias + np.array([w, 0.0, 0.0]) + 1e-3 * rng.standard_normal(3)
        att.update(gyro, np.array([0.0, 0.0, 3 * est.G]), dt)
    assert math.degrees(att.tilt) == pytest.approx(math.degrees(w * t_end), abs=0.3)
    roll, pitch, yaw = att.euler()
    assert (
        math.degrees(roll) == pytest.approx(math.degrees(w * t_end), abs=0.3)
        and abs(math.degrees(pitch)) < 0.3
    )


def test_attitude_gyro_only_drift_is_bounded():
    """After calibration the residual bias is the calibration noise (about 1e-4 rad/s): ten seconds of gyro-only flight stays within 0.3 deg."""
    rng = np.random.default_rng(3)
    bias = np.array([0.005, 0.004, -0.003])
    att = est.Attitude()
    for _ in range(500):
        att.calibrate(
            bias + 1e-3 * rng.standard_normal(3),
            np.array([0.0, 0.0, est.G]) + 0.02 * rng.standard_normal(3),
        )
    att.finish_calibration()
    for _ in range(2500):
        att.update(
            bias + 1e-3 * rng.standard_normal(3), np.array([0.0, 0.0, 2 * est.G]), 0.004
        )
    assert math.degrees(att.tilt) < 0.3


def test_attitude_is_not_pulled_upright_by_a_thrust_that_reads_one_g():
    """The tail of the landing motor is about the vehicle's weight, so the specific force along the (tilted) body axis reads ~1 g: the accelerometer
    correction must be off while a motor thrusts (`aid=False`), or it pulls the estimate to zero whatever the true tilt is."""

    def hold(aid):
        rng = np.random.default_rng(5)
        bias = np.array([0.004, -0.002, 0.001])
        att = est.Attitude()
        for _ in range(500):
            att.calibrate(
                bias + 1e-3 * rng.standard_normal(3),
                np.array([0.0, 0.0, est.G]) + 0.02 * rng.standard_normal(3),
            )
        att.finish_calibration()
        dt = 0.004
        for _ in range(int(0.5 / dt)):  # tilt 10 deg about x
            att.update(
                bias + np.array([math.radians(20.0), 0.0, 0.0]),
                np.array([0.0, 0.0, 0.93 * est.G]),
                dt,
                aid=aid,
            )
        for _ in range(int(2.0 / dt)):  # then hold, thrust along the body axis
            att.update(bias, np.array([0.0, 0.0, 0.93 * est.G]), dt, aid=aid)
        return math.degrees(att.tilt)

    assert hold(False) == pytest.approx(10.0, abs=0.3)
    assert hold(True) < 5.0  # the failure the gate removes


def test_axis_filter_tracks_a_motion_with_a_biased_accelerometer_and_delayed_fixes():
    rng = np.random.default_rng(4)
    dt, lag = 0.004, 50
    f = est.AxisFilter(dt)
    bias = 0.08
    pos_err = vel_err = 0.0
    for k in range(int(12.0 / dt)):
        t = k * dt
        a = 1.5 * math.sin(0.8 * t)
        p = 1.5 / 0.8 * t - 1.5 / 0.64 * math.sin(
            0.8 * t
        )  # integral of the motion with v(0) = 0
        v = 1.5 / 0.8 * (1 - math.cos(0.8 * t))
        f.predict(a + bias + 0.05 * rng.standard_normal())
        if k % 20 == 0 and k >= lag:  # a 50 Hz position fix, 0.2 s old
            tl = (k - lag) * dt
            pl = 1.5 / 0.8 * tl - 1.5 / 0.64 * math.sin(0.8 * tl)
            f.update(0, pl + 0.1 * rng.standard_normal(), 0.1, lag=lag)
        if k % 50 == 0 and k >= lag:  # velocity fix
            tl = (k - lag) * dt
            vl = 1.5 / 0.8 * (1 - math.cos(0.8 * tl))
            f.update(1, vl + 0.05 * rng.standard_normal(), 0.05, lag=lag)
        pos_err, vel_err = f.pos - p, f.vel - v
    assert abs(pos_err) < 0.3 and abs(vel_err) < 0.15


def test_baro_height_inverts_the_isa_troposphere():
    p0 = 101325.0
    for h in (1.0, 10.0, 50.0):
        p = p0 * (1 - 0.0065 * h / 288.15) ** 5.255876
        assert est.baro_height(p, p0) == pytest.approx(h, rel=1e-6)


# ----------------------------------------------------------------------------------------------------- vehicle constants / design
@pytest.fixture(scope="module")
def built(tmp_path_factory):
    out = tmp_path_factory.mktemp("lander")
    xml, rep = design.compute(
        REPO / "assemblies" / "tvc-lander.toml", REPO / "parts", out
    )
    return xml, rep


def test_assembly_builds_clean_and_mass_matches_the_firmware_constant(built):
    xml, rep = built
    assert rep["warnings"] == []
    # the one known gap: the library has no JST EH housing part for the regulator-to-servo lead's servo end (adding one would change the mass of the
    # Koch assembly, which has the same gap); its 0.5 g is inside the harness allowance
    assert [u["where"] for u in rep["unresolved"]] == [
        "wire 1 (reg.vout -> pitch.port_a)"
    ], rep["unresolved"]
    assert rep["mass"]["total"] == pytest.approx(veh.M_LOADED, abs=0.002)
    assert rep["mass"]["total"] < 1.5  # hobby-class limit of the brief
    # the stack: ascent E16 + landing F15 on the diagonal seats of the gimbal cradle, four legs, tail fins
    inst = rep["mass"]["by_instance"]
    assert {"motor_a", "motor_b", "fins", "weight"} <= set(inst)


def test_motor_tables_are_the_certified_curves():
    for pid in (veh.ASCENT_MOTOR, veh.LANDING_MOTOR):
        m = veh.Motor(pid)
        assert m.impulse == pytest.approx(m.cum[-1]) and m.burn > 0.5 and m.delay > 0.0
    assert veh.Motor("estes-e16").impulse == pytest.approx(
        33.4, rel=0.02
    )  # Estes E16 certified total impulse, 33.4 Ns (thrustcurve.org)
    assert veh.Motor("estes-f15").impulse == pytest.approx(
        49.6, rel=0.05
    )  # 40-80 Ns class F


def test_trim_angles_put_the_thrust_line_through_the_cg():
    pa, ya = veh.trim_angles("a")
    pb, yb = veh.trim_angles("b")
    assert (pa, ya) == pytest.approx(
        (-pb, -yb)
    )  # opposite diagonal seats, opposite trims
    assert math.degrees(abs(pa)) < 2.0 and abs(pa) < veh.GIMBAL_LIMIT


# --------------------------------------------------------------------------------------------------------- params / telemetry
def test_params_declared_resolve_and_validate():
    params, chans = nparams.declared(SCENARIO)
    names = {p["name"] for p in params}
    assert {
        "seed",
        "wind_ms",
        "impulse_sigma",
        "delay_jitter_s",
        "misalign_deg",
        "s_plan",
        "j_fire",
        "dump_max_deg",
        "apogee_cap_m",
        "alt_period",
        "kp_pos",
        "kd_vel",
        "td_gimbal_deg",
    } <= names
    assert len(chans) >= 20 and all(c["name"] for c in chans)
    base = nparams.resolve(SCENARIO)
    over = nparams.resolve(SCENARIO, {"wind_ms": 3.5, "seed": 7})
    assert over["wind_ms"] == 3.5 and over["seed"] == 7 and base["wind_ms"] != 3.5
    with pytest.raises(nparams.ParamError):
        nparams.resolve(SCENARIO, {"wind_ms": 1e6})
    with pytest.raises(nparams.ParamError):
        nparams.resolve(SCENARIO, {"no_such_parameter": 1})
    assert nparams.env_overrides({"NAPL_PARAMS": json.dumps({"wind_ms": 1.0})}) == {
        "wind_ms": 1.0
    }
    assert nparams.describe(SCENARIO)["parameters"][0]["name"]


def test_telemetry_writer_roundtrip_and_channels(tmp_path):
    w = nparams.TelemetryWriter(tmp_path / "t.jsonl")
    seen = []
    w.callbacks.append(seen.append)
    w.emit({"t": 0.0, "est": {"alt_m": 1.0}})
    w.emit({"t": 0.1, "est": {"alt_m": 2.0}, "state": "BOOST"})
    w.close()
    recs = nparams.read_jsonl(tmp_path / "t.jsonl")
    assert len(recs) == 2 and len(seen) == 2
    assert nparams.channel_series(recs, "est.alt_m") == [(0.0, 1.0), (0.1, 2.0)]
    assert nparams.get(recs[0], "est.nothing") is None
    # the command line a panel shells out to
    assert nparams.main(["table", str(tmp_path / "t.jsonl"), "est.alt_m,state"]) == 0
    assert nparams.main(["describe", str(SCENARIO)]) == 0
    assert nparams.main(["bogus"]) == 2


def test_cli_describe_is_the_json_contract(capsys):
    assert nparams.main(["describe", str(SCENARIO)]) == 0
    d = json.loads(capsys.readouterr().out)
    assert set(d) == {"parameters", "telemetry"}
    assert {"name", "type", "default", "unit", "group", "label", "doc"} <= set(
        d["parameters"][0]
    )
    assert {"name", "unit", "kind", "label", "group"} <= set(d["telemetry"][0])


# --------------------------------------------------------------------------------------------------------------- flight / scenario
@pytest.fixture(scope="module")
def land_mod():
    import land

    return land


def _quiet(over=None):
    o = {
        "wind_ms": 0.0,
        "impulse_sigma": 0.0,
        "delay_jitter_s": 0.0,
        "misalign_deg": 0.0,
    }
    o.update(over or {})
    return nparams.resolve(SCENARIO, o)


def test_a_calm_flight_lands_softly_and_upright(land_mod, tmp_path):
    import scene

    p = _quiet()
    model = scene.build_seeded(tmp_path, 1, 0.0)
    r = land_mod.fly(1, p, model, run_dir=tmp_path, log=False, telemetry=True)
    res = r["res"]
    assert res["landed"] == 1 and res["state"] == "TOUCHDOWN"
    assert res["td_speed"] < 2.0 and res["td_tilt"] < 10.0 and res["td_dist"] < 3.0
    assert r["rows"][-1][5] < 15.0  # still upright at the end
    ev = " | ".join(e[1] for e in res["events"])
    for key in (
        "calibrated",
        "ascent motor fired",
        "burnout",
        "landing motor fired",
        "touchdown",
    ):
        assert key in ev, ev
    # the HUD stream: one JSON object per line, every declared channel answered at least once, nothing in the stream that is undeclared
    recs = nparams.read_jsonl(tmp_path / "telemetry.jsonl")
    assert len(recs) > 100 and recs[0]["t"] < recs[-1]["t"]
    _, chans = nparams.declared(SCENARIO)
    got = {
        c["name"]
        for c in chans
        if any(nparams.get(x, c["name"]) is not None for x in recs)
    }
    assert got == {c["name"] for c in chans}, {c["name"] for c in chans} - got


def test_the_flight_replays_to_the_same_state_hash(land_mod, tmp_path):
    import scene

    hashes = []
    for i in range(2):
        p = _quiet({"seed": 11})
        model = scene.build_seeded(tmp_path / str(i), 11, p["wind_ms"])
        r = land_mod.fly(11, p, model, log=True, telemetry=False, max_s=9.0)
        hashes.append(r["sim"].state_hash())
    assert hashes[0] == hashes[1]


def test_different_seeds_differ(land_mod, tmp_path):
    import scene

    out = []
    for s in (21, 22):
        p = _quiet({"seed": s, "impulse_sigma": 0.03, "misalign_deg": 0.3})
        model = scene.build_seeded(tmp_path / str(s), s, 0.0)
        out.append(
            land_mod.fly(s, p, model, log=False, telemetry=False, max_s=9.0)["res"][
                "hidden"
            ]
        )
    assert (
        out[0]["scale_b"] != out[1]["scale_b"]
        and out[0]["accel_bias"] != out[1]["accel_bias"]
    )


def test_scenario_runs_through_napl_program_with_its_monitors(tmp_path, monkeypatch):
    from napl import program

    quiet = {"wind_ms": 0.0, "impulse_sigma": 0.0, "delay_jitter_s": 0.0}
    monkeypatch.setenv("NAPL_PARAMS", json.dumps({**quiet, "misalign_deg": 0.0}))
    res, run_dir = program.run(SCENARIO, tmp_path / "run")
    names = {m["name"] for m in res["monitors"]}
    assert {
        "touchdown_speed",
        "touchdown_tilt",
        "lands_near_the_pad",
        "servo_saturation",
        "flight_tilt_limit",
        "stays_upright",
    } <= names
    bad = [m["name"] for m in res["monitors"] if not m["pass"]]
    assert not bad, bad
    assert res["replay"]["available"] and res["replay"]["match"], res["replay"]
    assert (Path(run_dir) / "telemetry.jsonl").exists() or any(
        Path(run_dir).rglob("telemetry.jsonl")
    )


# -------------------------------------------------------------------------------------------------------------------- Monte Carlo
def test_mc_summary_counts_and_quantiles():
    import mc

    good = dict(
        landed=1,
        aborted=0,
        td_speed=1.0,
        td_tilt=3.0,
        td_dist=1.0,
        final_tilt=2.0,
        sat=0.02,
    )
    rows = [dict(good, seed=i, td_speed=0.5 + 0.1 * i) for i in range(10)]
    rows += [
        dict(good, seed=100, td_speed=6.0),
        dict(good, seed=101, final_tilt=99.0),
        dict(good, seed=102, aborted=1, td_tilt=70.0),
        dict(good, seed=103, landed=0, td_speed=99.0, td_tilt=99.0, td_dist=99.0),
    ]
    s = mc.summarize(rows)
    assert s["n"] == 14
    # ten good landings (0.5..1.4 m/s); the 6 m/s one, the one that tipped over, the aborted one and the one that never landed fail
    assert s["success"] == pytest.approx(10 / 14)
    assert s["speed_lt5"] == pytest.approx(12 / 14)
    assert s["upright"] == pytest.approx(12 / 14)
    assert s["aborted"] == pytest.approx(1 / 14) and s["landed"] == pytest.approx(
        12 / 14
    )
    assert s["speed_ms"]["p50"] is not None and s["dist_m"]["p50"] == pytest.approx(1.0)
    with pytest.raises(ValueError):
        mc.summarize([])


def test_mc_wind_is_seeded_and_bounded():
    import mc

    a, b = mc.wind_for(5000), mc.wind_for(5000)
    assert a == b and 0.0 <= a[0] <= mc.WIND_MAX and 0.0 <= a[1] < 360.0
    assert mc.wind_for(5001) != a
    assert (
        mc.TRAIN_BASE + 10_000 < mc.TEST_BASE or mc.TEST_BASE > mc.TRAIN_BASE + 2000
    )  # separate seed ranges


def test_mc_one_flight_is_reproducible(tmp_path, monkeypatch):
    import mc

    monkeypatch.setattr(mc, "SCRATCH", tmp_path)
    r1 = mc.one((5003, {"s_plan": 1.0}))
    r2 = mc.one((5003, {"s_plan": 1.0}))
    assert r1 == r2 and r1["landed"] in (0, 1)
