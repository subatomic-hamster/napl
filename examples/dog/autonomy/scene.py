"""Scenario harness of the dog autonomy missions: builds the carrier scene from a `.scenario.toml`, runs the robot on it and
scores it against ground truth (trace.csv for the napl program monitors, summary.json)."""

import json
import time
import tomllib
from pathlib import Path

import napl
import numpy as np
from napl import params as nparams
from napl.marine import write_native
from napl.terrain import field

from . import carrier, config, lidar, metrics, mission

COLS = ["t", "x", "y", "x_est", "y_est", "err_xy", "err_z", "err_yaw", "phase", "coverage", "rms", "rms_edges", "rtk_ok",
        "gnss_lost", "bumps", "distance", "fallen", "ms_points", "ms_icp", "ms_fuse", "ms_trav", "ms_plan", "ms_metrics"]  # fmt: skip
METRIC_EVERY_S = 10.0  # scoring cadence (harness cost, not the robot's)
DENSITY = {
    "rubble": {"rocks": 2500, "boulders": 40},
    "scree": {"rocks": 6000},
}  # napl.terrain.field defaults per 100 x 100 m


def scenario(path):
    sc = tomllib.loads(Path(path).read_text())
    return config.leaves(sc["auto"], "auto"), nparams.values(path), sc


def cfg_for(auto):
    ov = {
        "map.size": auto["size"],
        "plan.t_budget": auto["t_budget"],
        "plan.pool": auto["pool"],
    }
    for k in (
        "lane_m",
        "return_margin",
        "near_wall_speed",
        "near_wall_lookahead",
    ):  # optional per-scenario overrides of [plan]
        if k in auto:
            ov[f"plan.{k}"] = auto[k]
    if (
        "block" in auto
    ):  # optional per-scenario planning block (ruins: doors of 0.8-1.6 m read closed at 0.5 m)
        ov["trav.block"] = auto["block"]
    for k, key in (("near_lethal", "trav"), ("edge_margin", "map")):
        if k in auto:  # optional per-scenario scoring / planning margins
            ov[f"{key}.{k}"] = auto[k]
    return config.override(config.load(), **ov)


def make_field(auto, seed):
    a = (auto["size"] / 100.0) ** 2
    kw = {k: max(1, round(v * a)) for k, v in DENSITY.get(auto["kind"], {}).items()}
    return field(
        auto["kind"], size=auto["size"], patch=auto["patch"], seed=int(seed), **kw
    )


def _start(auto):
    return (auto["start_x"], auto["start_y"], auto["start_yaw"])


def build(out, path):
    auto, prm, _ = scenario(path)
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    return carrier.build(
        make_field(auto, prm["seed"]),
        cfg_for(auto),
        int(auto["rays"]),
        _start(auto),
        out,
    )


def build_for(out, script_file):
    return build(out, Path(script_file).with_suffix("").with_suffix(".scenario.toml"))


def _loco(sim, model, auto, cfg, f, seed, start):
    """-> (locomotion, lidar extrinsic, GNSS lever arm, cfg). Task 11 adds body = "dog"."""
    car = carrier.Carrier(sim, f, cfg, start, seed=seed)
    ext = lidar.extrinsic(Path(model).read_text(), "lidar", "carrier")
    return (
        car,
        ext,
        np.array([cfg["gnss"]["lever_x"], 0.0, cfg["gnss"]["lever_z"]]),
        cfg,
    )


def run(model, scenario_path, out, t_max=None):
    auto, prm, _ = scenario(scenario_path)
    cfg, seed, start = cfg_for(auto), int(prm["seed"]), _start(auto)
    f = make_field(auto, seed)
    sim = napl.Sim.load(model)
    sim.start_log()
    car, ext, lever, cfg = _loco(sim, model, auto, cfg, f, seed, start)
    p0, q0 = car.truth()
    L = cfg["loc"]
    g = np.random.default_rng([seed, 0x5EED]).standard_normal(
        4
    )  # operator initialisation error (decision 3)
    x0 = np.r_[p0 + L["init_sigma"] * g[:3], lidar.yaw_of(q0) + L["init_yaw"] * g[3]]
    rob = mission.Robot(car, cfg, int(auto["rays"]), ext, x0, lever, seed)
    gt = metrics.truth_grid(f, cfg["map"]["res"])
    edges, tgt = metrics.edge_mask(gt, cfg), metrics.targets(gt, cfg, start[:2])
    pool = int(cfg["plan"]["pool"])
    fdt = 1.0 / cfg["lidar"]["rate_hz"]
    t_max = t_max or cfg["plan"]["t_budget"] + cfg["plan"]["return_margin"]
    every = int(round(METRIC_EVERY_S / fdt))
    rows, cov, r, ms_m = (
        [],
        float("nan"),
        {"rms": float("nan"), "rms_edges": float("nan")},
        0.0,
    )

    def score():
        t0 = time.perf_counter()
        c, rr = (
            metrics.coverage(rob.map, tgt, pool),
            metrics.height_rms(rob.map, gt, edges),
        )
        return c, rr, 1e3 * (time.perf_counter() - t0)

    for k in range(int(round(t_max / fdt))):
        car.advance(fdt)
        rob.frame(car.time_of_last_frame)
        done = rob.ex.phase in (rob.ex.DONE, rob.ex.ABORT)
        if k % every == 0 or done or k == int(round(t_max / fdt)) - 1:
            cov, r, dm = score()
            ms_m += dm
        p, q = car.truth()
        x = rob.loc.x
        rows.append([
            car.time_of_last_frame, p[0], p[1], x[0], x[1], float(np.hypot(x[0] - p[0], x[1] - p[1])), x[2] - p[2],
            abs(((x[3] - lidar.yaw_of(q)) + np.pi) % (2 * np.pi) - np.pi), rob.ex.phase, cov, r["rms"], r["rms_edges"],
            int(rob.loc.rtk_ok(car.time_of_last_frame)), int(car.gnss_lost), car.bumps, car.distance, int(getattr(car, "fallen", False)),
            rob.ms["points"], rob.ms["icp"], rob.ms["fuse"], rob.ms["trav"], rob.ms["plan"], ms_m,
        ])  # fmt: skip
        if done:
            break
    write_native(sim, car.dt, COLS, rows, out)
    last = dict(zip(COLS, rows[-1]))
    nfr = len(rows)
    summ = {
        **last,
        "frames": nfr,
        "ms_per_frame": {k: rob.ms[k] / nfr for k in rob.ms},
        "metrics_ms_per_frame": ms_m / nfr,
    }
    (Path(out) / "summary.json").write_text(json.dumps(summ, indent=1))
    return summ
