"""Ground sim budget (SP2 risk 1): sim-seconds per wall-second of a dummy dog (12 Qdd joints in hold mode, 4 foot contacts, a Mid-360-like
rosette lidar at 10 Hz with N rays/frame) on each terrain kind, with the terrain as one 2001^2 hfield or as 10 m patches.

    uv run python tools/bench_ground.py [--kinds rubble,slopes,scree,ruins] [--rays 5000,10000,20000] [--sim 1.0] [--dt 0.002]

Prints a markdown table (the one in docs/models.md "Ground sim budget"). Run on hfbox (tools/remote.sh), one job at a time."""

import argparse
import json
import tempfile
import time
from pathlib import Path

import napl
from napl.terrain import add_field, field

QDD = {
    "node": 1,
    "motor": {
        "kv": 60,
        "r20": 0.475,
        "i0": 0.05,
        "rotor_inertia": 8.66307e-5,
        "w_s": 5,
        "c_th": 150,
        "r_th": 1.5,
    },
    "gear_ratio": 9,
    "efficiency": 0.9,
    "encoder_cpr": 16384,
    "cmd_hz": 500,
    "latency_s": 0.0,
    "i_limit": 20,
    "kp_max": 500,
    "kd_max": 10,
    "tau_ff_max": 100,
    "timeout_s": 1e3,
    "timeout_mode": 0,
    "kd_timeout": 2,
    "t_max": 100,
    "derate_start": 60,
}
LEGS = [("fl", 0.2, 0.14), ("fr", 0.2, -0.14), ("hl", -0.2, 0.14), ("hr", -0.2, -0.14)]


def leg(n, x, y):
    return (
        f'<body name="{n}" pos="{x} {y} 0"><joint name="{n}_a" type="hinge" axis="1 0 0"/>'
        f'<geom type="capsule" fromto="0 0 0 0 0 -0.02" size="0.02" mass="0.3" contype="0" conaffinity="0"/>'
        f'<body name="{n}_t" pos="0 0 -0.02"><joint name="{n}_h" type="hinge" axis="0 1 0"/>'
        f'<geom type="capsule" fromto="0 0 0 0 0 -0.2" size="0.02" mass="0.4" contype="0" conaffinity="0"/>'
        f'<body name="{n}_c" pos="0 0 -0.2"><joint name="{n}_k" type="hinge" axis="0 1 0"/>'
        f'<geom type="capsule" fromto="0 0 0 0 0 -0.2" size="0.015" mass="0.2" contype="0" conaffinity="0"/>'
        f'<geom name="{n}_foot" type="sphere" pos="0 0 -0.2" size="0.03" mass="0.1" friction="1 0.005 0.0001"/>'
        "</body></body></body>"
    )


def dog_xml(z):
    return (
        f'<body name="dog" pos="0 0 {z}"><freejoint name="fj"/>'
        '<geom type="box" size="0.25 0.1 0.05" mass="8" contype="0" conaffinity="0" group="4"/>'
        '<site name="lidar" pos="0 0 0.06" euler="0 -90 0"/>'
        + "".join(leg(*lg) for lg in LEGS)
        + "</body>"
    )


def devices(rays):
    q = [
        {**QDD, "name": f"{n}_{j}", "joint": f"{n}_{j}"}
        for n, _, _ in LEGS
        for j in "ahk"
    ]
    d = {
        "psus": [{"name": "bus", "node": 1, "voltage": 24, "r_series": 0.01}],
        "qdds": q,
        "foot_contacts": [
            {
                "name": f"{n}_ft",
                "geom": f"{n}_foot",
                "mode": 0,
                "f_max": 500,
                "noise_std": 0.5,
                "hysteresis_on": 2,
                "hysteresis_off": 1,
                "debounce_s": 0.005,
                "rate_hz": 250,
            }
            for n, _, _ in LEGS
        ],
    }
    if rays:
        d["rangefinders"] = [
            {
                "name": "lidar",
                "site": "lidar",
                "rate_hz": 10,
                "projection": "rosette",
                "fov_h": 6.283185307179586,
                "fov_v_min": -0.12217304763960307,
                "fov_v_max": 0.9075712110370514,
                "rays_per_frame": rays,
                "min_range": 0.1,
                "max_range": 40,
                "ray_groups": 1,
                "seed": 1,
                "pattern_seed": 1,
            }
        ]
    return d


def run(f, rays, sim, dt, tmp):
    z = float(f.height(0.0, 0.0)) + 0.5
    xml = f'<mujoco><option timestep="{dt}"/><worldbody><light pos="0 0 10"/>{dog_xml(z)}</worldbody></mujoco>'
    text, _ = add_field(xml, f)
    text = text.replace(
        "</mujoco>",
        f"<custom><text name=\"napl.devices\" data='{json.dumps(devices(rays))}'/></custom></mujoco>",
    )
    p = Path(tmp) / "m.xml"
    p.write_text(text)
    s = napl.Sim.load(p)
    for n, _, _ in LEGS:
        for j in "ahk":
            s.set(f"/devices/{n}_{j}", "kp", 20.0)
            s.set(f"/devices/{n}_{j}", "kd", 1.0)
            s.set(f"/devices/{n}_{j}", "enable", 1.0)
    s.step(int(0.3 / dt))  # settle + first lidar frame (lazy setup)
    n = int(sim / dt)
    t0 = time.perf_counter()
    s.step(n)
    return sim / (time.perf_counter() - t0)


def main():
    a = argparse.ArgumentParser()
    a.add_argument("--kinds", default="rubble,slopes,scree,ruins")
    a.add_argument("--rays", default="5000,10000,20000")
    a.add_argument("--sim", type=float, default=1.0)
    a.add_argument("--dt", type=float, default=0.002)
    a = a.parse_args()
    rays = [0] + [int(r) for r in a.rays.split(",")]
    print(
        "| kind | layout | "
        + " | ".join("no lidar" if r == 0 else f"{r // 1000}k rays" for r in rays)
        + " |"
    )
    print("|---|---|" + "---|" * len(rays))
    with tempfile.TemporaryDirectory() as tmp:
        for kind in a.kinds.split(","):
            for layout, patch in (("10 m patches", 10.0), ("one hfield", 100.0)):
                f = field(kind, seed=3, patch=patch)
                row = [f"{run(f, r, a.sim, a.dt, tmp):.2f}" for r in rays]
                print(f"| {kind} | {layout} | " + " | ".join(row) + " |", flush=True)


if __name__ == "__main__":
    main()
