"""examples/hopper-land: the EDF hopper taking off and landing in different places and environments. The density / thrust-margin math, the scenario files (napl.params), a landing flight
(terrain-aware, deterministic, replayable), the firmware's refusal to launch above its ceiling, the deck-state interface and its scripted-deck landing, the MC summary helpers and the PROGRAM page Run slot
listing the scenarios. Needs the built kernel (hfbox: tools/remote.sh run ...)."""

import json
import os
import sys
import tempfile
from pathlib import Path

import napl
import numpy as np
import pytest
from napl import params as nparams

# the vessel's ~40 wires take minutes to route cold; wireroute caches them on disk (the same cache as test_ship / test_asv)
os.environ.setdefault(
    "NAPL_ROUTE_CACHE", str(Path(tempfile.gettempdir()) / "napl-route-cache-asv")
)
REPO = Path(__file__).resolve().parents[2]
HOPPER, LAND = REPO / "examples" / "hopper", REPO / "examples" / "hopper-land"
needs_kernel = pytest.mark.skipif(napl._napl is None, reason="kernel not built")
CLASH = (
    "scene",
    "flight",
    "hop",
    "mc",
    "control",
    "estimator",
    "trajectory",
    "vehicle",
    "landveh",
    "landscene",
    "landing",
    "landest",
    "landflight",
    "landdeck",
    "landship",
    "landhop",
    "landmc",
    "ship",
)  # `ship`: examples/ship (landship imports its ShipPilot)
SCEN = {
    "mesa/down": "hopperland-mesa-down",
    "mesa/up": "hopperland-mesa-up",
    "ridge/summit": "hopperland-ridge-summit",
    "ledge/lee": "hopperland-ledge-lee",
    "rain/rain25": "hopperland-rain25",
    "rain/rain100": "hopperland-rain100",
    "deck/scripted": "hopperland-deck-scripted",
    "osv/underway": "hopperland-osv-underway",
}
toml = lambda k: LAND / f"{k}.scenario.toml"
lv = ld = lh = lm = lsh = None


@pytest.fixture(scope="module", autouse=True)
def _land_modules():
    """examples/hopper and examples/hopper-land share module names with other examples: resolve them to these two directories for this module's tests only."""
    global lv, ld, lh, lm, lsh
    saved = {n: sys.modules.pop(n) for n in CLASH if n in sys.modules}
    sys.path[:0] = [str(LAND), str(HOPPER)]
    import landdeck
    import landhop
    import landmc
    import landship
    import landveh

    lv, ld, lh, lm, lsh = landveh, landdeck, landhop, landmc, landship
    yield
    for p in (str(LAND), str(HOPPER)):
        sys.path.remove(p)
    for n in CLASH:
        sys.modules.pop(n, None)
    sys.modules.update(saved)


# ------------------------------------------------------------------------------------------------------------ density and the thrust ceiling
def test_isa_density_matches_the_kernel_atmosphere():
    assert lv.isa_rho(0.0) == pytest.approx(1.225, abs=2e-3)
    assert lv.isa_rho(3200.0) == pytest.approx(0.891, abs=2e-3)
    p = 101325.0 * (1 - 0.0065 * 3200 / 288.15) ** 5.2559
    assert lv.isa_rho_from_pressure(p) == pytest.approx(lv.isa_rho(3200.0), rel=2e-3)


def test_thrust_margin_falls_with_altitude_and_the_ceiling_is_where_it_hits_the_floor():
    m = [lv.thrust_margin(lv.isa_rho(h)) for h in (0, 1500, 3200, 4500, 5500)]
    assert m == sorted(m, reverse=True) and m[0] == pytest.approx(2.0, abs=0.05)
    ceil = lv.hover_ceiling_m()
    assert (
        7000 < ceil < 9000
    )  # the fit extrapolates past its 5500 m data; 9000 m is refused (below)
    assert lv.thrust_margin(lv.isa_rho(ceil)) == pytest.approx(lv.MARGIN_MIN, abs=1e-3)


def test_the_duty_feed_forward_rises_in_thin_air():
    d0 = lv.duty_for_thrust(13.0, 22.0, 1.225)
    d1 = lv.duty_for_thrust(13.0, 22.0, 0.8)
    assert 0 < d0 < d1 <= 1.0


# ------------------------------------------------------------------------------------------------------------------- the deck state
def test_deck_state_extrapolates_at_constant_velocity():
    s = ld.DeckState(2.0, np.array([1.0, 2.0, 0.5]), np.array([0.5, -1.0, 0.1]))
    p, v = s.predict(4.0)
    assert p == pytest.approx([2.0, 0.0, 0.7]) and v == pytest.approx([0.5, -1.0, 0.1])
    assert s.predict_z(3.0)[0] == pytest.approx(0.6)


# ------------------------------------------------------------------------------------------------------------------------ scenario files
@pytest.mark.parametrize("key", sorted(SCEN))
def test_params_declared_resolve_and_validate(key):
    sc = toml(key)
    params, chans = nparams.declared(sc)
    assert {"seed", "site", "wind_ms", "rain_mmh", "k_dob", "disperse"} <= {
        p["name"] for p in params
    }
    assert {"air.margin", "ground.agl_m"} <= {c["name"] for c in chans}
    assert nparams.resolve(sc)["seed"] >= 1
    with pytest.raises(nparams.ParamError):
        nparams.resolve(sc, {"wind_ms": 1e9})
    with pytest.raises(nparams.ParamError):
        nparams.resolve(sc, {"no_such_parameter": 1})


@pytest.mark.parametrize("key", sorted(SCEN))
def test_every_scenario_has_its_monitors_and_a_model_script(key):
    import tomllib

    d = tomllib.loads(toml(key).read_text())
    assert d["name"] == SCEN[key]
    names = {m["name"] for m in d["monitors"]}
    assert {"touchdown_speed", "touchdown_tilt", "no_abort", "thrust_margin"} <= names
    assert any(n.startswith("lands_on_the_") for n in names)
    assert (LAND / f"{key}.py").exists()


# ------------------------------------------------------------------------------------------------------------------------------ flights
def _fly(key, over, tmp, seed=1, max_s=None):
    p = lh.landing.resolve(toml(key), {"disperse": False, **over})
    model, h, info = lh.prepare(seed, p, tmp / "m")
    return lh.fly(
        seed, p, model, h, info, run_dir=tmp, log=True, telemetry=True, max_s=max_s
    )


@needs_kernel
def test_a_mesa_hop_lands_on_the_pad_30_m_below(tmp_path):
    r = _fly("mesa/down", {"wind_ms": 0.0}, tmp_path)
    res = r["res"]
    assert res["landed"] == 1 and not res["abort"] and res["state"] == "LANDED"
    assert res["td_speed"] < 0.5 and res["td_tilt"] < 5.0 and res["td_dist"] < 0.5
    z = [row[lh.COLS.index("z_m")] for row in r["rows"]]
    assert max(z) > 11.0  # cruised well above the plateau
    recs = nparams.read_jsonl(tmp_path / "telemetry.jsonl")
    assert recs and nparams.get(recs[-1], "air.rho_true") < 0.95  # 3200 m: thin air
    assert nparams.get(recs[-1], "air.rho") == pytest.approx(
        nparams.get(recs[-1], "air.rho_true"), rel=0.05
    )  # the barometer-derived density
    assert nparams.get(recs[-1], "air.margin") > 1.15


@needs_kernel
def test_the_same_seed_replays_to_the_same_state_hash(tmp_path):
    h = [
        _fly("mesa/down", {"disperse": True}, tmp_path / str(i), seed=7, max_s=12.0)[
            "sim"
        ].state_hash()
        for i in range(2)
    ]
    assert h[0] == h[1]


@needs_kernel
def test_the_firmware_refuses_to_launch_above_its_thrust_ceiling(tmp_path):
    r = _fly("mesa/down", {"origin_alt": 9000.0, "wind_ms": 0.0}, tmp_path, max_s=12.0)
    res = r["res"]
    assert not res["landed"]
    ev = " | ".join(e[1] for e in res["events"])
    assert "thrust margin" in ev and "not launching" in ev, ev


@needs_kernel
def test_a_scripted_deck_landing_is_judged_against_the_moving_deck(tmp_path):
    r = _fly("deck/scripted", {}, tmp_path)
    res = r["res"]
    assert res["landed"] == 1 and not res["abort"]
    assert res["td_speed"] < 0.5 and res["td_tilt"] < 5.0 and res["td_dist"] < 0.5


def test_the_prefix_proxy_maps_the_hoppers_paths_into_a_composed_scene():
    class Rec:
        tick = 7

        def __init__(self):
            self.calls = []

        def query(self, path, attr):
            self.calls.append(("q", path, attr))

        def set(self, path, attr, value):
            self.calls.append(("s", path, attr))

    rec = Rec()
    sim = lsh.PrefixSim(rec)
    for path in ("/world/frame", "/devices/imu", "/devices/air", "/devices/servo3"):
        sim.query(path, "x")
    sim.set("/devices/esc", "throttle", [0.1])
    assert [c[1] for c in rec.calls] == [
        "/world/hopper_frame",
        "/devices/hopper_imu",
        "/devices/air",  # the scene's, not the vehicle's
        "/devices/hopper_servo3",
        "/devices/hopper_esc",
    ]
    assert sim.tick == 7


@needs_kernel
def test_a_hop_off_and_back_onto_the_underway_vessels_helipad(tmp_path):
    """The deck state comes from RelGnss + DeckLink + DeckPredictor (napl.cuas.relnav), the flight lands on the heaving, rolling, moving 3 m pad."""
    r = _fly("osv/underway", {}, tmp_path)
    res = r["res"]
    assert res["landed"] == 1 and not res["abort"] and res["state"] == "LANDED"
    assert res["td_speed"] < 0.5 and res["td_tilt"] < 5.0 and res["td_dist"] < 0.5
    assert r["fl"].deck is not None and r["fl"].deck.ready  # the predictor ran
    z = [row[lh.COLS.index("z_m")] for row in r["rows"]]
    assert max(z) > 11.0
    x = [row[lh.COLS.index("x_m")] for row in r["rows"]]
    assert max(x) > 20.0  # the vessel was underway: the hopper flew along with it


# ------------------------------------------------------------------------------------------------------------------------------- MC helpers
def _row(**kw):
    r = dict(
        landed=1, aborted=0, td_speed=0.2, td_tilt=1.0, td_dist=0.1, final_tilt=1.0,
        sat=0.0, v_min=20.0, soc_min=0.5, clr_min=3.0, diverged=0, wind_peak=5.0,
        state="LANDED", abort="",
    )  # fmt: skip
    r.update(kw)
    return r


def test_mc_summary_counts_failures_by_cause_with_a_wilson_interval():
    rows = [_row() for _ in range(8)] + [
        _row(td_speed=0.9),
        _row(clr_min=0.4),
        _row(landed=0, aborted=1, abort="off-plan"),
    ]
    s = lm.summarize(rows)
    assert s["n"] == 11 and s["k"] == 8
    assert s["failures"] == {"speed": 1, "terrain clearance": 1, "abort:off-plan": 1}
    lo, hi = s["wilson"]
    assert lo < 8 / 11 < hi
    with pytest.raises(ValueError):
        lm.summarize([])


# ---------------------------------------------------------------------------------------------------------------- the PROGRAM page Run slot
@needs_kernel
def test_run_targets_lists_every_scenario_grouped_by_folder(tmp_path):
    from napl.design import Design
    from napl.design.viewer import Backend

    d = Design.open(REPO / "assemblies" / "hopper-land.toml")
    b = Backend(lambda: d, tmp_path / "out")  # default root: examples/hopper-land
    r = json.loads(b(json.dumps({"tool": "run.targets", "args": {}})))
    assert r["ok"], r
    assert set(r["result"]["scenarios"]) == {f"{k}.scenario.toml" for k in SCEN}
