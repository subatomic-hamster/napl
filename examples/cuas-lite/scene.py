"""Scenes of the cuas-lite lockstep scenarios (the turret is assemblies/cuas-lite.toml; see python/napl/lite/sim_io.py). `napl program` calls
build(outdir) -> MJCF path; run.py / mc.py import make() to get the same scenario as a Spec.

World frame: the turret stands on a 1.0 m mast at the origin (pan axis x = y = 0, tilt axis z ~ 1.2 m), pan 0 = +x, targets are placed with place(az, range, z).
Targets: `Balloon` pendulums (hung balloons, hanging target discs: kernel physics, nothing moves them) and `Mover`s (drifting balloon, small quad, walking
person: kinematic closed-form paths, see sim_io.Mover). Detector classes: balloon, target, drone (quad), person.

    make("s2_balloon_drift", seed)        # nominal scenario of the table in docs/cuas-lite/spec.md 7.2
    make("mc_quad", seed, R=9.0, speed=5.0, wind=(0.4, -0.2, 0), v0_err=-1.0)    # Monte Carlo single target; keyword overrides below"""

import math
import os
from dataclasses import dataclass, field

import numpy as np
from napl.cuas.scene import Environment
from napl.lite import aim as aimlib
from napl.lite.fire import Sector
from napl.lite.sim_io import Balloon, Mover, build_scene

SECTOR = Sector(-60, 60, 0, 60)
G = 9.81


def place(az_deg, r, z):
    a = math.radians(az_deg)
    return (r * math.cos(a), r * math.sin(a), z)


def swing_for_peak(v_peak, length):
    """Release angle (deg) of a pendulum of `length` m whose bob passes the bottom at v_peak m/s."""
    return math.degrees(math.acos(1 - v_peak**2 / (2 * G * length)))


@dataclass
class Spec:
    name: str
    targets: list
    env: Environment
    launcher: (
        dict  # launcher config overrides of build_scene (the true ball dispersion)
    )
    wind_est: tuple  # what the app's anemometer reports
    detector_kw: dict = field(default_factory=dict)
    camera_kw: dict = field(default_factory=dict)
    sim_s: float = 45.0
    arm_at: float = 8.0
    shot_cap: int = 5
    burst: int = 1
    fault: str = None  # s7: deadman | detector | estop | key | link
    check_hold: bool = False  # s6: the 1.0 s person hold is asserted
    expect: dict = field(
        default_factory=dict
    )  # run.py checks (non-vacuity of the safety scenarios)


def spec(
    name,
    targets,
    seed,
    wind=(0.0, 0.0, 0.0),
    wind_noise=0.0,
    v0_err=0.0,
    v_std=aimlib.V0_SIGMA,
    spread=aimlib.SPREAD_SIGMA,
    cd=aimlib.BALL["cd"],
    miss_scale=1.0,
    px_sigma=1.5,
    det_lat=0.015,
    cam_lat=0.030,
    jitter=0.0,
    person_miss=0.0,
    **kw,
):
    """Spec with the dispersion knobs: true muzzle speed v0 = 30 + v0_err m/s (the app assumes 30), shot-to-shot v_std, direction spread, Cd; steady wind (the app gets it
    plus N(0, wind_noise) per axis); detector miss scale / pixel noise / latencies / per-frame latency jitter."""
    rng = np.random.default_rng(seed + 4242)
    west = tuple(
        float(x)
        for x in np.asarray(wind, float)
        + (rng.normal(0, wind_noise, 3) if wind_noise else 0.0)
    )
    return Spec(
        name,
        targets,
        Environment(wind=tuple(wind), seed=seed),
        dict(
            muzzle_velocity=aimlib.V0_NOMINAL + v0_err,
            v_std=v_std,
            spread_std=spread,
            cd=cd,
        ),
        west,
        dict(
            latency=det_lat,
            jitter=jitter,
            miss_scale=miss_scale,
            px_sigma=px_sigma,
            person_miss=person_miss,
        ),
        dict(latency=cam_lat, jitter=jitter),
        **kw,
    )


def hung(i, az, r, z, length=1.5, **kw):
    return Balloon(i, place(az, r, z + length), length, **kw)


def drifter(i, y_axis_az, r, z, wind, span=6.0, size=0.30, seed=0):
    """Balloon carried by the wind: moves along the wind direction at the wind speed over `span` m, reversing at the ends (gust shifts), with a slow bob."""
    wv = np.array(wind[:2], float)
    sp = float(np.linalg.norm(wv)) or 1.0
    u = wv / sp if np.linalg.norm(wv) else np.array([0.0, 1.0])
    c = np.array(place(y_axis_az, r, z))
    a, b = c - [*(u * span / 2), 0], c + [*(u * span / 2), 0]
    ph = (seed * 0.7) % (2 * math.pi)
    return Mover(
        i,
        (tuple(a), tuple(b)),
        sp,
        waves=(((0, 0, 0.20), 6.0, ph), ((0.1 * u[0], 0.1 * u[1], 0), 4.0, ph)),
        size_m=size,
    )


def crosser(i, r, speed, z, az_half=35.0, size=0.30, t_start=0.0):
    """Small quad crossing in front of the turret at range r between azimuth -az_half and +az_half (ping-pong), with a little altitude wobble."""
    y = r * math.tan(math.radians(az_half))
    return Mover(
        i,
        ((r, -y, z), (r, y, z)),
        speed,
        waves=(((0, 0, 0.25), 3.0, 0.0),),
        size_m=size,
        kind="quad",
        t_start=t_start,
    )


def walker(i, x, y0, y1, speed, t_start, z=0.85):
    return Mover(
        i, ((x, y0, z), (x, y1, z)), speed, size_m=0.5, kind="person", t_start=t_start
    )


def make(name, seed=1, **kw):
    """Scenario `name` -> Spec. Keywords go to spec() (dispersion) except R / speed / t_walk which shape the targets."""
    rng = np.random.default_rng(seed)
    R, speed = kw.pop("R", None), kw.pop("speed", None)
    j = lambda s: float(rng.uniform(-s, s))
    if name == "s1_balloon_static":
        t = [
            Balloon("b1", (5.0, -1.0, 3.0), 1.5),
            Balloon("b2", (8.0, 0.5, 3.2), 1.7),
            Balloon("b3", (10.0, 2.0, 3.3), 1.8),
        ]
        return spec(name, t, seed, sim_s=60.0, **kw)
    if name == "smoke":
        return spec(name, [Balloon("b1", (6.0, 0.5, 3.3), 1.5)], seed, sim_s=30.0, **kw)
    if name == "s2_balloon_drift":  # wind 1 m/s carries two balloons at 5-12 m
        wind = kw.pop("wind", (0.3, 1.0, 0.0))
        t = [
            drifter("b1", -8 + j(3), 6.5 + j(1), 3.2, wind, seed=seed),
            drifter("b2", 12 + j(3), 10.0 + j(1.5), 3.5, wind, seed=seed + 1),
        ]
        return spec(name, t, seed, wind=wind, wind_noise=0.3, sim_s=60.0, **kw)
    if (
        name == "s3_hanging_targets"
    ):  # discs on 1 m strings swinging with 1.5 m/s peak at 4, 8, 12 m
        L, a = 1.0, swing_for_peak(1.5, 1.0)
        t = [
            Balloon(
                f"h{k}",
                place(az, r, 3.6),
                L,
                0.30,
                swing_deg=a,
                swing_az=az + 90,
                kind="target",
            )
            for k, (az, r) in enumerate([(-10, 4.0), (6, 8.0), (-3, 12.0)])
        ]
        return spec(name, t, seed, sim_s=60.0, **kw)
    if name in ("s4_quad_3ms", "s4_quad_8ms"):
        v = speed or (3.0 if name.endswith("3ms") else 8.0)
        return spec(
            name, [crosser("q1", R or 9.0 + j(2), v, 3.0)], seed, sim_s=45.0, **kw
        )
    if (
        name == "s5_mix"
    ):  # two balloons (one hung, one drifting) + a 4 m/s quad; nearest-first engagement, shot cap 5
        wind = kw.pop("wind", (0.2, 0.6, 0.0))
        t = [
            hung("b1", -5, 6.0, 2.0),
            drifter("b2", 14, 9.0, 3.3, wind, seed=seed),
            crosser("q1", 12.0, 4.0, 3.2, 25.0, t_start=0.0),
        ]
        return spec(name, t, seed, wind=wind, wind_noise=0.3, sim_s=60.0, **kw)
    if (
        name == "s6_person_walk"
    ):  # a person walks across the whole field of view at t = t_walk, mid-engagement
        t_walk = kw.pop("t_walk", 13.0)
        t = [
            hung(f"b{k}", az, r, z)
            for k, (az, r, z) in enumerate(
                [
                    (-6, 5.0, 1.8),
                    (4, 7.0, 2.0),
                    (-2, 9.0, 2.2),
                    (9, 6.0, 2.4),
                    (-10, 10.0, 2.0),
                ],
                1,
            )
        ]
        t.append(walker("p1", 4.0, -7.0, 7.0, 1.4, t_walk))
        return spec(
            name,
            t,
            seed,
            shot_cap=10,
            check_hold=True,
            sim_s=60.0,
            expect=dict(shots_before=1, shots_after=1),
            **kw,
        )
    if (
        name == "s6b_person_beside_balloon"
    ):  # a person stands next to a balloon the whole time: no ball at all
        t = [
            hung("b1", 3, 8.0, 2.0),
            Mover("p1", ((8.0, 1.2, 0.85),), size_m=0.5, kind="person"),
        ]
        return spec(
            name, t, seed, sim_s=25.0, check_hold=True, expect=dict(no_shots=True), **kw
        )
    if name.startswith(
        "s7_fault_"
    ):  # a fault at the first ball of an engagement; the balls after it are counted
        t = [
            Balloon("b1", (5.0, -1.0, 3.0), 1.5),
            Balloon("b2", (8.0, 0.5, 3.2), 1.7),
            Balloon("b3", (10.0, 2.0, 3.3), 1.8),
        ]
        return spec(
            name,
            t,
            seed,
            sim_s=40.0,
            fault=name.removeprefix("s7_fault_"),
            shot_cap=10,
            **kw,
        )
    if (
        name == "mc_person"
    ):  # safety stress: two balloons, a person crosses the view at t_walk
        t = [hung("b1", -5 + j(3), 6 + j(1), 2.0), hung("b2", 6 + j(3), 8 + j(1.5), 2.2), walker("p1", 4.0, -7.0, 7.0, 1.4, kw.pop("t_walk", 11.0))]  # fmt: skip
        return spec(name, t, seed, shot_cap=10, sim_s=32.0, **kw)
    if name.startswith(
        "mc_"
    ):  # Monte Carlo single target at range R (m); kind balloon | drift | hanging | quad
        k, R = name[3:], R or 8.0
        if k == "balloon":
            t = [hung("b1", j(15), R, 2.0 + j(0.4))]
        elif k == "drift":
            wind = kw.pop("wind", (0.2, 1.0, 0.0))
            t = [drifter("b1", j(10), R, 3.0, wind, seed=seed)]
            kw["wind"] = wind
        elif k == "hanging":
            az = j(15)
            t = [
                Balloon(
                    "h1",
                    place(az, R, 3.4),
                    1.0,
                    0.30,
                    swing_deg=swing_for_peak(speed or 1.5, 1.0),
                    swing_az=az + 90,
                    kind="target",
                )
            ]
        elif k == "quad":
            t = [crosser("q1", R, speed or 3.0, 2.8 + j(0.4))]
        else:
            raise KeyError(name)
        return spec(name, t, seed, sim_s=kw.pop("sim_s", 30.0), **kw)
    raise KeyError(f"unknown scenario {name!r}")


SCENARIOS = ["s1_balloon_static", "s2_balloon_drift", "s3_hanging_targets", "s4_quad_3ms", "s4_quad_8ms", "s5_mix", "s6_person_walk",
             "s6b_person_beside_balloon"] + [f"s7_fault_{k}" for k in ("deadman", "detector", "estop", "key", "link")]  # fmt: skip


def scenario():
    return os.environ.get("CUAS_SCENARIO", "s1_balloon_static")


def seed():
    return int(os.environ.get("CUAS_SEED", 1))


def build(out):
    sp = make(scenario(), seed())
    return build_scene(out, sp.targets, env=sp.env, seed=seed(), launcher=sp.launcher)[
        "model"
    ]
