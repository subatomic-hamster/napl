"""Offshore support vessel and landing-on-a-ship support (assemblies/osv.toml, examples/ship, napl.cuas.ship, napl.cuas.relnav): the design (DRC, mass, draft against the Damen FCS 1204
reference), the vessel in the sea (floats at its design draft, heave / roll response in Hs 1.5 m, heading and speed held underway), the helipad (a box on the deck stays while the
vessel moves, a downward rangefinder reads the deck distance), and the relative-navigation models (moving-base RTK states, deck datalink, deck-motion prediction)."""

import math
import os
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest

# the vessel's ~40 wires take minutes to route cold; wireroute caches them on disk (the same cache as test_asv)
os.environ.setdefault(
    "NAPL_ROUTE_CACHE", str(Path(tempfile.gettempdir()) / "napl-route-cache-asv")
)
REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "examples"))

import napl
from napl import design
from napl.cuas import relnav
from napl.cuas import ship as shipmod
from napl.design import Design
from ship.pilot import CTRL_TICKS, DT, ShipPilot, yaw_of

OSV = REPO / "assemblies" / "osv.toml"


def _q(sim, path, what):
    return np.asarray(sim.query(path, what)[0], float)


# ------------------------------------------------------------------------------------------------------------------------------------- design
def test_osv_design_is_drc_clean():
    findings = Design.open(OSV).check()
    bad = [f for f in findings if f.severity in ("error", "warning")]
    assert bad == [], [(f.code, f.message[:120]) for f in bad]


def test_mass_and_draft_against_the_reference(tmp_path):
    rep = design.build(OSV, REPO / "parts", tmp_path)["report"]
    m = rep["mass"]["total"]
    # Damen FCS 1204 (12.60 x 3.80 x 1.70 m, draft 0.80 m, 2.5 t deadweight, aluminium): no displacement is published; a 12.6 m crew boat is a 10-14 t vessel
    assert 9000 < m < 12000, m
    T = shipmod.equilibrium_draft(m)
    assert 0.6 < T <= 0.80, T  # the sheet's maximum draft is 0.80 m
    hull = design.Library(REPO / "parts").get("generic-osv-hull-1260")
    ext = np.array([s["pos"][0] + s["size"][0] for s in hull.geometry["shapes"]]).max()
    assert abs(2 * ext - 12.6) < 0.05  # length over all
    beam = 2 * max(s["size"][1] for s in hull.geometry["shapes"])
    assert 3.6 < beam < 3.9  # 3.80 m on the sheet


# ------------------------------------------------------------------------------------------------------------------------------------ sea
@pytest.fixture(scope="module")
def scene_factory(tmp_path_factory):
    cache = {}

    def get(name, **kw):
        if name not in cache:
            cache[name] = shipmod.ship_scene(
                tmp_path_factory.mktemp(name), visual=False, **kw
            )
        return cache[name]

    return get


def test_vessel_floats_at_the_design_draft(scene_factory):
    sc = scene_factory("calm", hs=0.0, seed=3)
    sim = napl.Sim.load(sc["model"])
    z = []
    for _ in range(20):
        sim.step(1000)
        z.append(_q(sim, "/world/osv_hull", "pos")[2])
    assert abs(np.mean(z[5:]) + sc["draft"]) < 0.02, (np.mean(z), sc["draft"])
    assert np.std(z[5:]) < 0.005  # settled, no drift
    q = _q(sim, "/world/osv_hull", "quat")
    r, p, _ = relnav.euler_zyx(q)
    assert (
        abs(math.degrees(r)) < 0.5 and abs(math.degrees(p)) < 0.5
    )  # level (trim and heel of the design)


def _record(sim, seconds, every=25, pilot=None, extra=None):
    out = []
    for k in range(int(seconds / (every * DT))):
        if pilot is not None:
            pilot.advance(sim)
        else:
            sim.step(every)
        out.append(
            (
                sim.tick * DT,
                _q(sim, "/world/osv_hull", "pos"),
                _q(sim, "/world/osv_hull", "quat"),
            )
        )
        if extra:
            extra(sim, out[-1])
    return out


def test_heave_and_roll_response_in_beam_seas_of_1p5_m(scene_factory):
    sc = scene_factory("beam", hs=1.5, wave_dir=90.0, seed=3)
    sim = napl.Sim.load(sc["model"])
    tr = _record(sim, 300.0)
    z = np.array([p[2] for _, p, _ in tr])[len(tr) // 5 :]
    ang = np.array([relnav.euler_zyx(q) for _, _, q in tr])[len(tr) // 5 :]
    sig_eta = 1.5 / 4.0  # sigma of the surface elevation for Hs = 4 sigma
    heave, roll, pitch = (
        np.std(z),
        np.degrees(np.std(ang[:, 0])),
        np.degrees(np.std(ang[:, 1])),
    )
    # a 12 m boat follows the waves of Tp 6.1 s (wavelength 58 m): heave about the wave height; roll = the wave slope (2 pi / lambda sigma_eta = 2.3 deg) times a roll RAO of 1-3
    # (the roll natural period of this hull is 3-4 s, inside the spectrum's band: measured 5.6 deg rms)
    assert 0.6 * sig_eta < heave < 1.3 * sig_eta, heave
    assert 1.5 < roll < 8.0, roll
    assert pitch < roll  # beam seas: pitch is the smaller
    assert (
        abs(np.mean(z) + sc["draft"]) < 0.05
    )  # the mean draft is the still-water draft


def test_pilot_holds_heading_and_speed_in_head_seas_of_1p5_m(scene_factory):
    sc = scene_factory("head", hs=1.5, wave_dir=180.0, seed=4)
    sim = napl.Sim.load(sc["model"])
    pl = ShipPilot(sim)
    pl.set_heading(0.0, 3.0)  # 5.8 kn, bow East, into waves that travel West
    tr = _record(sim, 160.0, every=CTRL_TICKS, pilot=pl)
    n = len(tr) // 2
    yaw = np.degrees([yaw_of(q) for _, _, q in tr[n:]])
    p = np.array([p for _, p, _ in tr[n:]])
    v = np.hypot(*(p[-1, :2] - p[0, :2])) / (tr[-1][0] - tr[n][0])
    assert np.sqrt(np.mean(yaw**2)) < 3.0, np.sqrt(np.mean(yaw**2))
    assert abs(v - 3.0) < 0.3, v
    assert abs(p[:, 1]).max() < 15.0  # drifted off the line by less than a hull length


def test_pilot_follows_a_route_and_reaches_the_end(scene_factory):
    sc = scene_factory("route", hs=0.8, wave_dir=135.0, seed=5)
    sim = napl.Sim.load(sc["model"])
    pl = ShipPilot(sim)
    pl.set_route([(0.0, 0.0), (150.0, 0.0), (150.0, 120.0)], speed=3.0)
    _record(sim, 190.0, every=CTRL_TICKS, pilot=pl)
    p = _q(sim, "/world/osv_hull", "pos")
    assert pl.route.done and math.hypot(p[0] - 150.0, p[1] - 120.0) < 40.0, (
        p,
        pl.route.k,
    )


def test_dp_lite_holds_a_position_in_a_cross_current(scene_factory):
    sc = scene_factory("dp", hs=0.5, wave_dir=90.0, seed=6, current=(0.0, 0.3))
    sim = napl.Sim.load(sc["model"])
    pl = ShipPilot(sim)
    pl.set_hold((0.0, 0.0))
    tr = _record(sim, 240.0, every=CTRL_TICKS, pilot=pl)
    d = np.array([np.hypot(*p[:2]) for _, p, _ in tr[len(tr) // 2 :]])
    assert d.max() < 12.0 and d.mean() < 6.0, (
        d.max(),
        d.mean(),
    )  # no lateral thruster: stays within a hull length or so


# ------------------------------------------------------------------------------------------------------------------------------------ helipad
def _with_box(path, centre, half=0.15, mass=5.0):
    tree = ET.parse(path)
    wb = tree.getroot().find("worldbody")
    b = ET.SubElement(
        wb, "body", name="cargo", pos=" ".join(f"{x:.6f}" for x in centre)
    )
    ET.SubElement(b, "freejoint", name="cargo_free")
    ET.SubElement(
        b,
        "geom",
        name="cargo",
        type="box",
        size=f"{half} {half} {half}",
        mass=str(mass),
        contype="1",
        conaffinity="1",
        rgba="0.8 0.5 0.1 1",
    )
    out = Path(path).with_name("scene_box.xml")
    tree.write(out)
    return str(out)


def test_box_on_the_helipad_stays_while_the_vessel_moves(scene_factory):
    sc = scene_factory("box", hs=1.5, wave_dir=180.0, seed=7)
    pad = np.array(sc["pad"])
    model = _with_box(sc["model"], pad + np.array([0.6, 0.4, 0.17]))
    sim = napl.Sim.load(model)
    pl = ShipPilot(sim)
    pl.set_heading(0.0, 3.0)
    rel = []
    for _ in range(int(90.0 / (CTRL_TICKS * DT))):
        pl.advance(sim)
        hp, hq = _q(sim, "/world/osv_hull", "pos"), _q(sim, "/world/osv_hull", "quat")
        rel.append(relnav.quat_to_mat(hq).T @ (_q(sim, "/world/cargo", "pos") - hp))
    rel = np.array(rel)
    assert (
        np.hypot(*(rel[-1, :2] - shipmod.PAD_CENTRE[:2])) < 1.3
    )  # still on the 3 x 3 m pad
    assert (
        abs(rel[-1, 2] - (shipmod.PAD_CENTRE[2] + 0.15)) < 0.03
    )  # sitting on the plate, not through it
    assert (
        np.ptp(rel[len(rel) // 2 :, :2], axis=0).max() < 0.25
    )  # and it has not been sliding about (friction 0.9)
    assert (
        np.hypot(*(_q(sim, "/world/osv_hull", "pos")[:2])) > 100.0
    )  # the vessel did move (3 m/s x 90 s)


def test_downward_rangefinder_above_the_pad_reads_the_deck_distance(scene_factory):
    read = {}
    for h in (1.0, 2.0, 3.0):
        sc = scene_factory(
            f"lidar{h:g}",
            hs=0.0,
            seed=3,
            lander="hopper",
            lander_start="air",
            air_height=h,
        )
        sim = napl.Sim.load(sc["model"])
        sim.step(50)
        r = [float(_q(sim, "/devices/hopper_lidar", "range")[0])]
        for _ in range(3):
            sim.step(25)
            r.append(float(_q(sim, "/devices/hopper_lidar", "range")[0]))
        z = _q(sim, "/world/hopper_frame", "pos")[2]
        read[h] = (np.mean(r), z - sc["pad"][2])
    for h, (rng, clearance) in read.items():
        assert abs(rng - clearance) < 0.25, (
            h,
            rng,
            clearance,
        )  # the sensor sits just under the airframe origin; the pad top is the first return
    assert abs((read[3.0][0] - read[1.0][0]) - 2.0) < 0.2


@pytest.mark.parametrize("lander", ["x500v2", "skydio", "hopper"])
def test_landers_rest_on_the_deck(scene_factory, lander):
    sc = scene_factory(
        f"deck_{lander}", hs=0.0, seed=3, lander=lander, lander_start="deck"
    )
    sim = napl.Sim.load(sc["model"])
    root = [
        k
        for k in sim.body_positions()
        if k.startswith(f"/world/{sc['lander']}_") and k.count("/") == 2
    ][0]
    z0 = _q(sim, root, "pos")[2]
    sim.step(4000)
    p = _q(sim, root, "pos")
    assert (
        abs(p[2] - z0) < 0.03
    )  # sat on the plate (did not fall through, was not thrown up)
    assert np.hypot(*(p[:2] - sc["pad"][:2])) < 0.2
    assert 1.0 < sc["pad"][2] < 1.1  # pad top 1.75 m above the keel, draft 0.70 m


# ------------------------------------------------------------------------------------------------------------------------------ relative navigation
def _rg_errors(rg, seconds, rel=np.array([5.0, -3.0, 4.0]), dt=0.04):
    by = {relnav.FIX: [], relnav.FLOAT: [], relnav.SINGLE: []}
    for k in range(int(seconds / dt)):
        t = k * dt
        f = rg.step(t, np.zeros(3), rel)
        if f is not None and f.t_meas > 1.0:
            by[f.state].append(f.rel - rel)
    return {k: np.array(v) for k, v in by.items()}


def test_rtk_states_fixed_float_single_error_statistics():
    # fixed: centimetres; float: decimetres; single: about a metre -- from cold starts with the corrections available
    rg = relnav.RelGnss(seed=1, fixed_at_start=False, p_loss_good=0.0, p_g2b=0.0)
    e = _rg_errors(rg, 400.0)
    fix, flt = e[relnav.FIX], e[relnav.FLOAT]
    assert len(fix) > 1000 and len(flt) > 20
    sd = lambda a: float(np.sqrt(np.mean(a[:, :2] ** 2)))
    assert sd(fix) < 0.04, sd(fix)
    assert 0.08 < sd(flt) < 0.5, sd(flt)
    # no corrections at all: single-point quality
    rg2 = relnav.RelGnss(seed=2, fixed_at_start=False, p_loss_good=1.0, p_loss_bad=1.0)
    e2 = _rg_errors(rg2, 200.0)[relnav.SINGLE]
    assert len(e2) > 500 and 0.5 < sd(e2) < 2.0, sd(e2)
    assert sd(fix) < sd(flt) < sd(e2)


def test_rtk_dropout_degrades_and_reconverges():
    rg = relnav.RelGnss(seed=3, p_loss_good=0.0, p_g2b=0.0)
    rel = np.array([2.0, 1.0, 3.0])
    states, t = [], 0.0
    for k in range(int(120.0 / 0.04)):
        t = k * 0.04
        if 20.0 <= t < 45.0:  # radio blackout
            rg.pl = (1.0, 1.0)
        else:
            rg.pl = (0.0, 0.0)
        f = rg.step(t, np.zeros(3), rel)
        states.append((t, f.state if f else None))
    at = lambda s: [st for tt, st in states if abs(tt - s) < 0.02][0]
    assert at(15.0) == relnav.FIX
    assert at(25.0) == relnav.FLOAT  # corrections older than float_age (2 s)
    assert at(44.0) == relnav.SINGLE  # older than single_age
    assert at(105.0) == relnav.FIX  # re-converged after the link came back
    assert at(50.0) != relnav.FIX  # not instantly


def test_rtk_latency_and_rate():
    rg = relnav.RelGnss(seed=1)
    seen = []
    for k in range(int(10.0 / 0.04)):
        t = k * 0.04
        f = rg.step(t, np.zeros(3), np.array([1.0, 0.0, 0.0]))
        if f is not None and (not seen or f.t_meas != seen[-1].t_meas):
            seen.append(f)
    assert abs(len(seen) - 80) <= 3  # 8 Hz
    assert all(abs((f.t_avail - f.t_meas) - 0.125) < 1e-9 for f in seen)


def test_deck_link_rate_latency_loss_and_attitude_error():
    link = relnav.DeckLink(seed=5)
    truth = None
    msgs, ages = [], []
    q = np.array([1.0, 0.0, 0.0, 0.0])
    for k in range(int(300.0 / 0.04)):
        t = k * 0.04
        truth = relnav.deck_truth(t, np.array([0.0, 0.0, 0.3 * math.sin(t)]), q, truth)
        m = link.step(t, truth)
        if m is not None and (not msgs or m.seq != msgs[-1].seq):
            msgs.append(m)
            ages.append(m.t_avail - m.t_meas)
    assert 6800 < len(msgs) <= 7500  # 25 Hz x 300 s = 7500 sent, 3 % + bursts lost
    assert 0.0 < link.lost / link.sent < 0.2
    assert abs(np.mean(ages) - 0.08) < 1e-6
    rolls = np.degrees([m.roll for m in msgs])
    assert 0.3 < np.std(rolls) < 0.8  # 0.5 deg rms at rest (VN-100 data sheet)


def test_deck_predictor_skill_on_simulated_ship_motion(scene_factory):
    sc = scene_factory("beam", hs=1.5, wave_dir=90.0, seed=3)
    sim = napl.Sim.load(sc["model"])
    link, pred = relnav.DeckLink(seed=9), relnav.DeckPredictor()
    truth, truth_h, errs, rms = None, [], [], []
    off = shipmod.PAD_CENTRE
    horizon = 2.0
    for k in range(int(260.0 / 0.04)):
        sim.step(40)
        t = sim.tick * DT
        hp, hq = _q(sim, "/world/osv_hull", "pos"), _q(sim, "/world/osv_hull", "quat")
        truth = relnav.deck_truth(t, hp, hq, truth)
        pred.add(link.step(t, truth), off)
        truth_h.append((t, shipmod.body_to_world(hp, hq, off)[2]))
        if t > 120.0 and k % 25 == 0 and pred.ready():
            tf, p = pred.predict(horizon)
            # compare with the pad height `horizon` seconds later
            errs.append((tf[-1], p["h"][-1]))
    th = np.array(truth_h)
    actual = np.interp([e[0] for e in errs], th[:, 0], th[:, 1], right=np.nan)
    guess = np.array([e[1] for e in errs])
    ok = ~np.isnan(actual)
    sig = np.std(th[th[:, 0] > 120.0, 1])
    rmse = float(np.sqrt(np.mean((guess[ok] - actual[ok]) ** 2)))
    assert ok.sum() > 20 and rmse < 0.75 * sig, (
        rmse,
        sig,
    )  # beats "no idea" (sigma) clearly at +2 s
    w = pred.next_window(
        horizon=10.0, duration=1.0, vz_max=0.4, tilt_max=math.radians(8.0)
    )
    assert w is None or w >= pred.t[-1] - 1e-6


# ------------------------------------------------------------------------------------------------------------------------------------ ASV pad
def test_asv_landing_pad_only_in_the_scenes_that_ask_for_it(tmp_path):
    from asv import encounters as enc
    from asv import scene as asv_scene

    spec = enc.make("head_on", seed=1, hs=0.0)
    plain = asv_scene.build_spec(tmp_path / "plain", spec)
    padded = asv_scene.build_spec(tmp_path / "padded", spec, pad=True)
    assert (
        "landing_pad" not in plain.read_text()
    )  # the scenarios (and their hashes) are untouched
    root = ET.parse(padded).getroot()
    pad = next(g for g in root.iter("geom") if g.get("name") == "landing_pad")
    assert (pad.get("contype"), pad.get("conaffinity"), pad.get("group")) == (
        "0",
        "1",
        "3",
    )
    assert abs(float(pad.get("friction").split()[0]) - 0.9) < 1e-9
    sim = napl.Sim.load(str(padded))
    sim.step(2000)
    z = _q(sim, "/world/asv_hull", "pos")[2]
    assert -0.1 < z < 0.1  # still floats
