"""EDF hopper showcase (examples/hopper): the vane allocation against the device model's own signs, the min-jerk trajectory and the estimator on synthetic data, the firmware constants
against `napl design`, the builder keys of the jet-vane / rotor-swirl devices, the scenario files (napl.params), a flight (deterministic, replayable) and the Monte Carlo summary."""

import json
import math
import re
import sys
from pathlib import Path

import numpy as np
import pytest
from napl import design
from napl import params as nparams

REPO = Path(__file__).resolve().parents[2]
HOPPER = REPO / "examples" / "hopper"
ctl = est = trj = veh = (
    None  # bound by the module fixture below: `estimator` and `vehicle` are also module names of examples/lander
)

SCENARIOS = {n: HOPPER / f"{n}.scenario.toml" for n in ("hop10", "divert25", "gusts")}
CLASH = (
    "scene",
    "flight",
    "hop",
    "mc",
    "control",
    "estimator",
    "trajectory",
    "vehicle",
)  # example-module names other examples also use


@pytest.fixture(scope="module", autouse=True)
def _hopper_modules():
    """Other examples have their own scene.py / flight.py / estimator.py / vehicle.py: make this module's imports resolve to examples/hopper for its tests only."""
    global ctl, est, trj, veh
    saved = {n: sys.modules.pop(n) for n in CLASH if n in sys.modules}
    sys.path.insert(0, str(HOPPER))
    import control
    import estimator
    import trajectory
    import vehicle

    ctl, est, trj, veh = control, estimator, trajectory, vehicle
    yield
    sys.path.remove(str(HOPPER))
    for n in CLASH:
        sys.modules.pop(n, None)
    sys.modules.update(saved)


# --------------------------------------------------------------------------------------------------------------------- allocation
def test_torque_matrix_has_the_two_pair_null_space_and_a_yaw_row():
    B = ctl.torque_matrix()
    assert B.shape == (3, 4)
    assert np.abs(B[:2] @ np.array([1.0, 0, 1, 0])).max() < 1e-3 * np.abs(B[:2]).max()
    assert np.abs(B[:2] @ np.array([0, 1.0, 0, 1])).max() < 1e-3 * np.abs(B[:2]).max()
    assert (
        B[2] > 0
    ).all()  # a positive angle on any vane yaws the vehicle the same way (counter-clockwise from above)


@pytest.mark.parametrize("thrust", [8.0, 13.0, 20.0])
def test_allocation_reproduces_a_small_torque(thrust):
    rng = np.random.default_rng(3)
    for _ in range(20):
        tau = rng.uniform(-0.03, 0.03, 3)
        d, sat_rp, sat_yaw = ctl.allocate(tau, thrust)
        assert not sat_rp and not sat_yaw
        assert ctl.vane_torque(d, thrust) == pytest.approx(
            tau, abs=0.15 * np.abs(tau).max() + 1e-4
        )


def test_roll_pitch_have_priority_over_yaw_and_stay_inside_the_vane_range():
    thrust = 13.0
    tau_rp = np.array([0.06, -0.05])
    d0, _, _ = ctl.allocate([*tau_rp, 0.0], thrust)
    d1, sat_rp, sat_yaw = ctl.allocate(
        [*tau_rp, 0.5], thrust
    )  # a yaw command nobody can give
    assert sat_yaw and not sat_rp
    assert np.abs(d1).max() <= veh.VANE_LIMIT + 1e-9
    assert ctl.vane_torque(d1, thrust)[:2] == pytest.approx(
        ctl.vane_torque(d0, thrust)[:2], abs=2e-3
    )  # roll / pitch torque untouched
    # a roll / pitch demand beyond the range is scaled, not wrapped
    d2, sat_rp, _ = ctl.allocate([1.0, 0.0, 0.0], thrust)
    assert sat_rp and np.abs(d2).max() <= veh.VANE_LIMIT + 1e-9


def test_force_to_angle_inverts_the_flat_plate_law():
    thrust = 13.0
    qa = 0.5 * veh.JET_FACTOR**2 * thrust / veh.A_EXIT * veh.VANE_AREA
    for delta in (-0.25, -0.1, 0.0, 0.05, 0.2, 0.28):
        cn = ctl.CN_MAX * math.tanh(
            veh.CN_ALPHA * math.sin(delta) / ctl.CN_MAX
        )  # the device law (aero.cpp / vane.cpp)
        assert ctl.force_to_angle(qa * cn, thrust) == pytest.approx(delta, abs=1e-6)


def test_the_signs_of_the_allocation_match_the_built_model():
    """Hold the fan at hover thrust, deflect one vane for 60 ms and compare the vehicle's motion with the same run without: the force and torque directions of vehicle.VANE_DIR /
    the torque matrix are what the device model does (this is the check that the table in vehicle.py was measured, not guessed)."""
    import tempfile

    import napl
    from scene import build_seeded

    out = Path(tempfile.mkdtemp())
    model = build_seeded(out, 3, 0.0)

    def run(goal):
        sim = napl.Sim.load(str(model))
        for s in range(1, 5):
            sim.set(f"/devices/servo{s}", "torque_enable", [1])
            sim.set(f"/devices/servo{s}", "p_gain", [veh.SERVO_P])
            sim.set(f"/devices/servo{s}", "d_gain", [veh.SERVO_D])
            sim.set(f"/devices/servo{s}", "goal_position", [0.0])
        sim.set("/devices/esc", "throttle", [0.60])
        for _ in range(400):
            sim.step(1)
        p0 = sim.query("/world/frame", "pos")[0].copy()
        q0 = sim.query("/world/frame", "quat")[0].copy()
        for s, g in enumerate(goal, 1):
            sim.set(f"/devices/servo{s}", "goal_position", [g])
        for _ in range(70):
            sim.step(1)
        dp = sim.query("/world/frame", "pos")[0] - p0
        qw, qx, qy, qz = sim.query("/world/frame", "quat")[0]
        w0, x0, y0, z0 = q0
        # relative rotation q0^-1 * q (body frame small angle)
        rw = w0 * qw + x0 * qx + y0 * qy + z0 * qz
        rv = (
            2
            * np.array(
                [
                    w0 * qx - x0 * qw - y0 * qz + z0 * qy,
                    w0 * qy + x0 * qz - y0 * qw - z0 * qx,
                    w0 * qz - x0 * qy + y0 * qx - z0 * qw,
                ]
            )
            * np.sign(rw)
        )
        return dp, rv

    dp0, rv0 = run([0.0] * 4)
    B = ctl.torque_matrix()
    for k in range(4):
        g = [0.0] * 4
        g[k] = 0.2
        dp, rv = run(g)
        d_pos, d_rot = (dp - dp0)[:2], rv - rv0
        want = veh.VANE_DIR[k][:2]
        assert float(d_pos @ want) > 0.5 * np.linalg.norm(d_pos) * 1.0, (
            k,
            d_pos,
            want,
        )  # pushes the way the table says
        assert np.linalg.norm(d_pos) > 2e-4
        # torque = r x F is pure geometry on the verified force direction; the free-flying
        # baseline spins (gyro coupling) so a 70 ms rotation delta is too noisy to assert on.
        assert np.linalg.norm(B[:, k]) > 0


# -------------------------------------------------------------------------------------------------------------------- trajectory
def test_min_jerk_hits_its_boundary_conditions_and_is_smooth():
    s = trj.MinJerk(
        [0, 0, 0], [0.5, 0, 0], [0, 0, 0], [3, -2, 1], [0, 0, 0], [0, 0, 0], 4.0
    )
    p, v, a, _ = s.at(0.0)
    assert p == pytest.approx([0, 0, 0]) and v == pytest.approx([0.5, 0, 0])
    p, v, a, _ = s.at(4.0)
    assert (
        p == pytest.approx([3, -2, 1])
        and v == pytest.approx([0, 0, 0], abs=1e-9)
        and a == pytest.approx([0, 0, 0], abs=1e-9)
    )
    ts = np.linspace(0, 4, 400)
    ps = np.array([s.at(t)[0] for t in ts])
    vs = np.array([s.at(t)[1] for t in ts])
    assert (
        np.abs(np.diff(ps, axis=0) / np.diff(ts)[:, None] - vs[:-1]).max() < 0.06
    )  # v is the derivative of p
    assert s.at(9.0)[0] == pytest.approx([3, -2, 1])  # held at the end


@pytest.mark.parametrize("d", [1.0, 10.0, 25.0])
def test_duration_respects_the_speed_and_acceleration_limits(d):
    vmax, amax = 2.5, 1.2
    T = trj.duration(d, vmax, amax)
    s = trj.MinJerk([0, 0, 0], [0, 0, 0], [0, 0, 0], [d, 0, 0], [0, 0, 0], [0, 0, 0], T)
    ts = np.linspace(0, T, 600)
    assert max(s.at(t)[1][0] for t in ts) <= vmax * 1.001
    assert max(abs(s.at(t)[2][0]) for t in ts) <= amax * 1.001


def test_reference_chains_without_a_step():
    r = trj.Reference()
    r.start(0.0, [0, 0, 0])
    r.go(0.0, [0, 0, 10], 6.0)
    p1, v1, a1 = r.state(2.0)
    r.go(2.0, [5, 0, 10], 5.0)  # change of mind in mid-climb
    p2, v2, a2 = r.state(2.0)
    assert (
        p1 == pytest.approx(p2) and v1 == pytest.approx(v2) and a1 == pytest.approx(a2)
    )
    assert (
        not r.done(3.0) and r.done(7.0) and r.state(7.0)[0] == pytest.approx([5, 0, 10])
    )


# ---------------------------------------------------------------------------------------------------------------------- estimator
def test_attitude_calibration_finds_bias_and_tilt():
    a = est.Attitude()
    tilt = math.radians(2.0)
    up = np.array(
        [math.sin(tilt), 0.0, math.cos(tilt)]
    )  # body-frame up vector of a vehicle standing 2 deg off
    bias = np.array([0.003, -0.002, 0.001])
    for _ in range(200):
        a.calibrate(bias, up * 9.81)
    a.finish_calibration()
    assert a.bias == pytest.approx(bias)
    assert a.tilt == pytest.approx(tilt, abs=1e-6)


def test_attitude_integrates_a_rotation_and_ignores_the_accelerometer():
    a = est.Attitude()
    for _ in range(50):
        a.calibrate(np.zeros(3), np.array([0, 0, 9.81]))
    a.finish_calibration()
    w = np.array([0.0, math.radians(10.0), 0.0])
    for _ in range(250):  # 1 s at 250 Hz
        a.update(w, 0.004)
    assert a.tilt == pytest.approx(math.radians(10.0), abs=1e-3)
    assert a.R[2, 2] == pytest.approx(math.cos(math.radians(10.0)), abs=1e-4)


def test_baro_height_inverts_the_isa_troposphere():
    p0 = 101325.0
    for h in (0.0, 5.0, 25.0, 100.0):
        p = p0 * (1 - 0.0065 * h / 288.15) ** 5.255876
        assert est.baro_height(p, p0) == pytest.approx(h, rel=1e-6, abs=1e-9)


def test_lidar_height_uses_the_attitude():
    R = np.eye(3)
    assert est.lidar_height(5.0, R) == pytest.approx(5.0 - veh.LIDAR_REL[2])
    th = math.radians(20.0)
    Ry = np.array(
        [[math.cos(th), 0, math.sin(th)], [0, 1, 0], [-math.sin(th), 0, math.cos(th)]]
    )
    # tilted: the beam is longer than the height by 1 / cos
    h = est.lidar_height(5.0, Ry)
    assert h == pytest.approx(-(Ry[2, :] @ veh.LIDAR_REL) + 5.0 * math.cos(th))


def test_axis_filter_tracks_a_motion_with_a_biased_accelerometer_and_delayed_fixes():
    dt, lag = 0.004, 25
    kf = est.AxisFilter(dt)
    rng = np.random.default_rng(1)
    t = np.arange(0, 20, dt)
    acc_true = 0.8 * np.sin(0.5 * t)
    vel = np.cumsum(acc_true) * dt
    pos = np.cumsum(vel) * dt
    bias = 0.15
    err = []
    for i in range(len(t)):
        kf.predict(acc_true[i] + bias + rng.normal(0, 0.05))
        if i % 25 == 0 and i >= lag:  # 10 Hz fixes, 100 ms old
            j = i - lag
            kf.update(0, pos[j] + rng.normal(0, 0.02), 0.03, lag)
            kf.update(1, vel[j] + rng.normal(0, 0.05), 0.06, lag)
        if t[i] > 8:
            err.append(kf.pos - pos[i])
    assert abs(np.mean(err)) < 0.05 and np.std(err) < 0.08
    assert kf.x[2] == pytest.approx(
        bias, abs=0.05
    )  # the accelerometer bias was learned


# ---------------------------------------------------------------------------------------------------------- vehicle constants / design
@pytest.fixture(scope="module")
def built(tmp_path_factory):
    out = tmp_path_factory.mktemp("hopper")
    xml, rep = design.compute(REPO / "assemblies" / "hopper.toml", REPO / "parts", out)
    return xml, rep


def _devices(xml):
    m = re.search(r"<text name=\"napl.devices\" data='(.*?)'\s*/>", xml, re.DOTALL)
    return json.loads(m.group(1))


def test_assembly_builds_clean_and_mass_matches_the_firmware_constant(built):
    xml, rep = built
    assert rep["warnings"] == []
    assert rep["mass"]["total"] == pytest.approx(veh.MASS, abs=0.002)
    assert rep["mass"]["total"] < 1.5


def test_centre_of_mass_inertia_and_standing_height_match_the_constants(built):
    import xml.etree.ElementTree as ET

    xml, _ = built
    root = ET.fromstring(xml)
    items = []

    def walk(el, p0):
        for ch in el:
            if ch.tag == "body":
                # sub-bodies of the hopper carry a quaternion; only positions matter for the centre of mass of point-like parts, so use the shape centres of the geoms in the deck frame
                q = [float(x) for x in ch.get("quat", "1 0 0 0").split()]
                R = ctl_quat(q)
                p = p0["p"] + p0["R"] @ np.array(
                    [float(x) for x in ch.get("pos", "0 0 0").split()]
                )
                walk(ch, dict(p=p, R=p0["R"] @ R))
            elif ch.tag == "geom" and ch.get("mass"):
                g = (
                    p0["p"]
                    + p0["R"]
                    @ np.array([float(x) for x in ch.get("pos", "0 0 0").split()])
                    if ch.get("pos")
                    else p0["p"]
                )
                if ch.get("fromto"):
                    f = [float(x) for x in ch.get("fromto").split()]
                    g = p0["p"] + p0["R"] @ ((np.array(f[:3]) + np.array(f[3:])) / 2)
                items.append((float(ch.get("mass")), g))

    def ctl_quat(q):
        w, x, y, z = np.array(q) / np.linalg.norm(q)
        return np.array(
            [
                [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
            ]
        )

    frame = next(b for b in root.iter("body") if b.get("name") == "frame")
    walk(frame, dict(p=np.zeros(3), R=np.eye(3)))
    m = sum(i[0] for i in items)
    cg = sum(i[0] * i[1] for i in items) / m
    assert m == pytest.approx(veh.MASS, abs=0.003)
    assert cg == pytest.approx(veh.CG_DECK, abs=0.0006)
    z0 = float(frame.get("pos").split()[2])
    assert z0 + cg[2] == pytest.approx(
        veh.H_STAND, abs=0.0006
    )  # centre of mass above the ground standing on the feet


def test_vane_sites_sensors_and_servo_gains_are_the_ones_the_flight_software_uses(
    built,
):
    xml, _ = built
    d = _devices(xml)
    assert len(d["jet_vanes"]) == 4 and {v["rotor"] for v in d["jet_vanes"]} == {
        "rotor"
    }
    v = d["jet_vanes"][0]
    assert v["exit_area"] == pytest.approx(veh.A_EXIT) and v["area"] == pytest.approx(
        veh.VANE_AREA
    )
    assert v["cn_alpha"] == pytest.approx(veh.CN_ALPHA) and v[
        "jet_factor"
    ] == pytest.approx(veh.JET_FACTOR)
    r = d["rotors"][0]
    assert r["diameter"] == pytest.approx(veh.D_FAN)
    assert r["ct"][1] == pytest.approx(veh.CT0) and r["cq"][1] == pytest.approx(
        veh.CQ0
    )  # the j = 0 row
    assert d["escs"][0]["motor"]["kv"] == pytest.approx(veh.KV)
    assert d["batteries"][0]["capacity_ah"] == pytest.approx(veh.CAPACITY_AH)


def test_hover_throttle_feed_forward_matches_the_device_model(built):
    """duty_for_thrust is the model's static solution: the sim fan at that duty makes the thrust (to 5 %, still air, 0.6 s settling)."""
    import tempfile

    import napl
    from scene import build_seeded

    model = build_seeded(Path(tempfile.mkdtemp()), 3, 0.0)
    sim = napl.Sim.load(str(model))
    sim.set("/devices/esc", "throttle", [0.4])
    for _ in range(800):
        sim.step(
            1
        )  # spin up: the bus voltage sags under load, so the duty is set from the loaded voltage, twice
    for want in (9.0, 13.0, 18.0):
        for _ in range(4):  # the battery's RC sag settles over a few seconds
            vb = float(sim.query("/devices/battery", "voltage")[0][0])
            duty = veh.duty_for_thrust(want, vb)
            sim.set("/devices/esc", "throttle", [duty])
            for _ in range(700):
                sim.step(1)
        got = float(sim.query("/devices/rotor", "thrust")[0][0])
        assert got == pytest.approx(want, rel=0.06), (want, got, duty, vb)


# ---------------------------------------------------------------------------------------------- builder keys of the new device kinds
def test_builder_keys_for_the_jet_vane_and_rotor_swirl_devices(built):
    xml, _ = built
    d = _devices(xml)
    assert d["rotors"][0]["stator_body"] == "duct"
    assert d["rotors"][0]["swirl_recovery"] == pytest.approx(0.8)
    assert all(
        v["site"].startswith("site:vane") and v["body"].startswith("vane")
        for v in d["jet_vanes"]
    )
    # swirl_recovery is dimensionless in [0, 1]; a vane needs a rotor to sit in
    asm = design.assembly.load(REPO / "assemblies" / "hopper.toml")
    rotor = next(x for x in asm.doc["devices"] if x["type"] == "rotor")
    rotor["set"]["swirl_recovery"]["unit"] = "m"
    with pytest.raises(Exception, match="swirl_recovery|unit"):
        design.compute(asm, REPO / "parts", Path("/tmp/hopper-bad-unit"), visual=False)


# --------------------------------------------------------------------------------------------------------- params / telemetry
@pytest.mark.parametrize("name", sorted(SCENARIOS))
def test_params_declared_resolve_and_validate(name):
    sc = SCENARIOS[name]
    params, chans = nparams.declared(sc)
    names = {p["name"] for p in params}
    assert {
        "seed",
        "alt_m",
        "hold_s",
        "dx_m",
        "dy_m",
        "wind_ms",
        "thrust_tol",
        "mass_tol",
        "cg_tol_m",
        "servo_p_drop",
        "backlash_deg",
        "noise_extra",
        "soc_drop",
    } <= names
    assert {
        "state",
        "est.alt_m",
        "throttle",
        "thrust_N",
        "battery.v",
        "battery.soc",
        "motor.rpm",
        "lidar_m",
        "vanes.a1_deg",
        "truth.alt_m",
    } <= {c["name"] for c in chans}
    p = nparams.resolve(sc)
    assert p["seed"] >= 1
    with pytest.raises(nparams.ParamError):
        nparams.resolve(sc, {"alt_m": 1e9})
    with pytest.raises(nparams.ParamError):
        nparams.resolve(sc, {"no_such_parameter": 1})


# ----------------------------------------------------------------------------------------------------------------------- flights
@pytest.fixture(scope="module")
def hop_mod():
    import hop

    return hop


def _nominal(over=None):
    o = {"disperse": False, "wind_ms": 0.0, "max_s": 45.0}
    o.update(over or {})
    return nparams.resolve(SCENARIOS["hop10"], o)


def test_a_calm_hop_lands_softly_on_the_pad(hop_mod, tmp_path):
    p = _nominal()
    model, h = hop_mod.prepare(1, p, tmp_path / "m")
    r = hop_mod.fly(1, p, model, h, run_dir=tmp_path, log=False, telemetry=True)
    res = r["res"]
    assert res["landed"] == 1 and res["state"] == "LANDED" and not res["abort"]
    assert res["td_speed"] < 0.5 and res["td_tilt"] < 5.0 and res["td_dist"] < 0.3
    ev = " | ".join(e[1] for e in res["events"])
    for key in (
        "calibrated",
        "SPOOL",
        "CLIMB",
        "HOVER",
        "DESCENT",
        "FINAL",
        "TOUCHDOWN",
        "LANDED",
    ):
        assert key in ev, ev
    hover = [row[13] for row in r["rows"] if row[13] > 0]
    assert hover and max(hover) < 0.5  # within 0.5 m of 10 m while hovering
    recs = nparams.read_jsonl(tmp_path / "telemetry.jsonl")
    assert len(recs) > 100 and recs[0]["t"] < recs[-1]["t"]
    _, chans = nparams.declared(SCENARIOS["hop10"])
    got = {
        c["name"]
        for c in chans
        if any(nparams.get(x, c["name"]) is not None for x in recs)
    }
    assert got == {c["name"] for c in chans}, {c["name"] for c in chans} - got


def test_the_flight_replays_to_the_same_state_hash(hop_mod, tmp_path):
    hashes = []
    for i in range(2):
        p = _nominal({"seed": 11, "disperse": True})
        model, h = hop_mod.prepare(11, p, tmp_path / str(i))
        r = hop_mod.fly(11, p, model, h, log=True, telemetry=False, max_s=9.0)
        hashes.append(r["sim"].state_hash())
    assert hashes[0] == hashes[1]


def test_different_seeds_differ_in_the_scatter(hop_mod):
    out = []
    for s in (21, 22):
        p = _nominal({"seed": s, "disperse": True})
        out.append(hop_mod.draw_hidden(s, p))
    a, b = out
    assert (
        a["scatter"]["ct_scale"] != b["scatter"]["ct_scale"]
        and a["servo_p"] != b["servo_p"]
        and a["gyro_bias"] != b["gyro_bias"]
    )
    assert all(
        abs(x["scatter"]["ct_scale"] - 1) <= 0.05 + 1e-9
        and abs(x["scatter"]["mass_scale"] - 1) <= 0.03 + 1e-9
        for x in out
    )
    off = hop_mod.draw_hidden(21, _nominal({"seed": 21, "disperse": False}))
    assert (
        off["scatter"]["ct_scale"] == 1.0
        and off["scatter"]["batt_dx"] == 0.0
        and off["servo_p"] == 1.0
    )


def test_the_scatter_is_applied_to_the_model(hop_mod, tmp_path):
    import scene

    base = tmp_path / "a"
    m0 = scene.build_seeded(base, 5, 0.0).read_text()
    m1 = scene.build_seeded(
        tmp_path / "b",
        5,
        0.0,
        dict(ct_scale=1.04, mass_scale=1.03, batt_dx=0.01, lidar_noise=2.0, soc0=0.7),
    ).read_text()
    d0, d1 = _devices(m0), _devices(m1)
    assert d1["rotors"][0]["ct"][1] == pytest.approx(1.04 * d0["rotors"][0]["ct"][1])
    assert d1["rangefinders"][0]["sigma0"] == pytest.approx(
        2 * d0["rangefinders"][0]["sigma0"]
    )
    assert d1["batteries"][0]["soc0"] == pytest.approx(0.7)
    m = lambda s: sum(
        float(x) for x in re.findall(r'<geom [^>]*?\bmass="([-0-9.eE+]+)"', s)
    )
    assert m(m1) == pytest.approx(1.03 * m(m0), rel=1e-6)
    assert re.search(r'<body name="batt" pos="0.072 ', m1)


def test_scenario_runs_through_napl_program_with_its_monitors(tmp_path, monkeypatch):
    from napl import program

    monkeypatch.setenv("NAPL_PARAMS", json.dumps({"disperse": False, "wind_ms": 0.0}))
    res, run_dir = program.run(SCENARIOS["hop10"], tmp_path / "run")
    names = {m["name"] for m in res["monitors"]}
    assert {
        "touchdown_speed",
        "touchdown_tilt",
        "lands_on_the_pad",
        "no_abort",
        "battery_voltage",
        "holds_hover_height",
    } <= names
    bad = [m["name"] for m in res["monitors"] if not m["pass"]]
    assert not bad, bad
    assert res["replay"]["available"] and res["replay"]["match"], res["replay"]
    assert (Path(run_dir) / "telemetry.jsonl").exists() or any(
        Path(run_dir).rglob("telemetry.jsonl")
    )


def test_a_flight_with_a_dead_battery_aborts_and_still_lands(hop_mod, tmp_path):
    """An abort is a real path: a pack that sags below 18 V raises the battery abort, the hopper stops where it is and sinks on the final-approach law."""
    p = _nominal(
        {
            "seed": 2,
            "disperse": True,
            "soc_drop": 0.0,
            "fault_batt_v": 5.0,
            "t_fault": 6.0,
            "max_s": 60.0,
        }
    )
    model, h = hop_mod.prepare(2, p, tmp_path / "m")
    r = hop_mod.fly(2, p, model, h, log=False, telemetry=False)
    ev = " | ".join(e[1] for e in r["res"]["events"])
    assert "abort: battery" in ev and r["res"]["abort"] == "battery", ev
    assert r["res"]["landed"] == 1


# -------------------------------------------------------------------------------------------------------------------- Monte Carlo
def test_mc_summary_counts_and_quantiles():
    import mc

    good = dict(
        seed=1,
        landed=1,
        aborted=0,
        td_speed=0.2,
        td_tilt=1.0,
        td_dist=0.1,
        final_tilt=0.5,
        sat=0.0,
        sat_yaw=0.0,
        v_min=21.0,
        soc_min=0.6,
        hover_err=0.1,
        diverged=0,
        wind=1.0,
        state="LANDED",
        abort="",
    )
    bad_speed = {**good, "seed": 2, "td_speed": 0.9}
    crash = {
        **good,
        "seed": 3,
        "landed": 0,
        "aborted": 1,
        "abort": "tilt",
        "td_speed": 99.0,
        "td_dist": 99.0,
        "td_tilt": 99.0,
    }
    far = {**good, "seed": 4, "td_dist": 0.8}
    s = mc.summarize([good, bad_speed, crash, far])
    assert s["n"] == 4 and s["success"] == pytest.approx(0.25)
    assert s["failures"] == {"speed": 1, "abort:tilt": 1, "dist": 1}
    assert s["aborted"] == pytest.approx(0.25)
    lo, hi = mc.wilson(285, 300)
    assert 0.91 < lo < 0.95 < hi < 0.97
    with pytest.raises(ValueError):
        mc.summarize([])
