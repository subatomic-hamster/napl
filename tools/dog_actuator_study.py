#!/usr/bin/env python3
"""Analytic pre-study for the dog's actuator / knee ratio / leg length / bus voltage candidates (SP3a Task 4).

    python3 tools/dog_actuator_study.py > /tmp/study.md      (stdlib only; docs/dog/actuator-study.md is this output + prose)

Model (planar leg, thigh = shank = L, foot under the hip, half-knee angle PHI from vertical, hip height h = 2 L cos PHI):
  - actuator curve: torque tau(w) = tau_pk (1 - w / w0), w0 = kv_output * V * 2 pi / 60 (voltage-limited, linear: the DC-motor line
    that the Qdd device models); tau_pk is current-limited and does not change with V.
  - knee drive ratio r: joint torque = r * actuator torque, joint speed = actuator speed / r (belt/linkage between actuator and knee).
  - peak foot force (static bound): knee torque r*tau_pk = F L sin PHI  ->  F_peak = r tau_pk / (L sin PHI).
  - trot at duty 0.5, hip sweep half-angle THETA: stance and swing last equally; foot speed in swing v_sw = h (sin THETA / THETA) w_sw with w_sw =
    SWING_FRAC w0 (swing carries little torque); in stance v_st = h cos THETA w_st with w_st = w0 (1 - tau_hip_req / tau_pk).
    Trot top-speed estimate = min(v_sw, v_st): the stride-frequency bound and the torque bound (a Mini-Cheetah-style actuator-limited
    gait; Katz 2018, Kim/Katz 'Mini Cheetah' ICRA 2019 use the same torque/speed trade). Flying trot (duty < 0.5) can exceed this; SP3b measures it.
  - torque feasibility: stance peak force F_z = PEAK_FACTOR * m g / 2 (two legs, sinusoidal profile), hip torque = F_z h sin THETA,
    knee torque = F_z L sin PHI / r (actuator side); both must be <= tau_pk, else the point is infeasible.
  - bus current at that peak: I = sum over the 12 joints of I_pk * tau_req / tau_pk (current ~ torque), + idle + compute; compared to the battery.
"""

import math
import pathlib
import tomllib

ROOT = pathlib.Path(__file__).resolve().parents[1]
G = 9.81
PHI = math.radians(40.0)
THETA = math.radians(25.0)
SWING_FRAC = 0.7
PEAK_FACTOR = 1.57  # sinusoidal stance force profile: peak / mean
DYN = 1.5  # dynamic margin on the stance peak (terrain, landing)
LEGS = (0.20, 0.22, 0.25)
RATIOS = (1.0, 1.5, 2.0)
BUS = (24.0, 33.6)
ACTS = ["steadywin-gim8108-8", "steadywin-gim6010-8", "cubemars-ak70-9-v3-kv60"]


def part(rel):
    return tomllib.loads((ROOT / "parts" / f"{rel}.toml").read_text())["params"]


def val(rel, key):
    e = part(rel)[key]
    return e["value"]


# fixed (non-actuator) mass and cost lines: (label, kg, USD, fidelity of the price)
def fixed_lines(L):
    printed_kg = 0.0
    for n, c in (
        ("hip-housing-l", 2),
        ("hip-housing-r", 2),
        ("thigh-bracket-l", 2),
        ("thigh-bracket-r", 2),
        ("knee-drive-housing-l", 2),
        ("knee-drive-housing-r", 2),
        ("shank", 4),
        ("foot-core", 4),
        ("head-mount", 1),
        ("battery-tray", 1),
        ("side-plate-l", 1),
        ("side-plate-r", 1),
    ):
        printed_kg += c * part(f"structure/dog-{n}")["mass"]["value"]
    tube_kg = (
        4 * 2 * L * part("structure/cf-tube-16x14")["mass_per_m"]["value"]
    )  # thigh + shank carbon tubes, 8 per dog
    lines = [
        (
            "Livox Mid-360",
            val("electronics/livox-mid-360", "mass"),
            val("electronics/livox-mid-360", "price"),
            "estimated",
        ),
        (
            "Mid-360 cable",
            val("electronics/livox-mid-360-splitter-cable", "mass"),
            val("electronics/livox-mid-360-splitter-cable", "price"),
            "estimated",
        ),
        (
            "RealSense D435i",
            0.072,
            val("electronics/intel-realsense-d435i", "price"),
            "datasheet",
        ),
        ("Jetson Orin Nano devkit", 0.174, 399.0, "datasheet"),
        (
            "SparkFun RTK ZED-F9P",
            0.015,
            val("electronics/sparkfun-gps-rtk-sma-16481", "price"),
            "datasheet",
        ),
        ("RTK antenna ANN-MB", 0.050, 60.0, "estimated"),
        (
            "CANable USB-CAN",
            val("electronics/makerbase-mks-canable-v2-pro", "mass"),
            val("electronics/makerbase-mks-canable-v2-pro", "price"),
            "estimated",
        ),
        (
            "Battery 8S 4.5 Ah",
            val("propulsion/auline-21700-s70-8s1p-4500mah", "mass"),
            val("propulsion/auline-21700-s70-8s1p-4500mah", "price"),
            "datasheet",
        ),
        (
            "BMS",
            val("electronics/daly-k-series-smart-bms-8s-60a", "mass"),
            val("electronics/daly-k-series-smart-bms-8s-60a", "price"),
            "estimated",
        ),
        (
            "PDB",
            val("electronics/dog-power-distribution-board", "mass"),
            val("electronics/dog-power-distribution-board", "price"),
            "estimated",
        ),
        (
            "DC-DC x3",
            3 * 0.005,
            2 * val("electronics/pololu-d24v22f12", "price")
            + val("electronics/pololu-d24v22f5", "price"),
            "datasheet",
        ),
        ("E-stop", 0.080, 11.0, "estimated"),
        (
            "CAN/power cables x12",
            12 * 0.3 * 0.025,
            12 * val("connectors/cubemars-ak-v3-power-can-cable", "price"),
            "datasheet",
        ),
        (
            "Rubber balls x4",
            4 * 0.0245,
            4 * val("structure/mcmaster-natural-rubber-ball-1-3-8in", "price"),
            "estimated",
        ),
        (
            "Carbon tubes (8 x L)",
            tube_kg,
            8 * L * part("structure/cf-tube-16x14")["price_per_m"]["value"],
            "estimated",
        ),
        (
            "Printed parts (PLA)",
            printed_kg,
            printed_kg * 20.0,
            "estimated",
        ),  # 20 USD/kg filament, cad/cuas-lite/params.py COST_PER_KG
        ("Pulleys, belts, bearings, fasteners, wiring", 0.45, 80.0, "estimated"),
    ]
    return lines


def point(act, r, L, V):
    kv, tpk, ipk = (
        val(f"actuators/{act}", "kv_output"),
        val(f"actuators/{act}", "torque_peak"),
        val(f"actuators/{act}", "current_peak"),
    )
    m_act, price = val(f"actuators/{act}", "mass"), val(f"actuators/{act}", "price")
    w0 = kv * V * 2 * math.pi / 60.0
    h = 2 * L * math.cos(PHI)
    lines = fixed_lines(L)
    mass = 12 * m_act + sum(x[1] for x in lines)
    cost = 12 * price + sum(x[2] for x in lines)
    fz = DYN * PEAK_FACTOR * mass * G / 2
    t_hip = fz * h * math.sin(THETA)
    t_knee = fz * L * math.sin(PHI) / r
    t_ab = 0.3 * fz * 0.10
    feasible = max(t_hip, t_knee, t_ab) <= tpk
    f_peak = r * tpk / (L * math.sin(PHI))
    v_sw = h * math.sin(THETA) / THETA * SWING_FRAC * w0
    w_st = max(0.0, w0 * (1 - min(t_hip / tpk, 1.0)))
    v_st = h * math.cos(THETA) * w_st
    v_trot = min(v_sw, v_st)
    # bus current at the stance peak: 2 stance legs (hip, knee, abduction), 2 swing legs at 15 % torque, + idle and compute
    frac = lambda t: min(t / tpk, 1.0)
    i_stance = ipk * (frac(t_hip) + frac(t_knee) + frac(t_ab))
    i_swing = ipk * 0.15 * 3
    i_bus = 2 * i_stance + 2 * i_swing + 12 * 0.3 + 40.0 / V
    return dict(
        act=act,
        r=r,
        L=L,
        V=V,
        mass=mass,
        cost=cost,
        f_peak=f_peak,
        ratio=2 * f_peak / (mass * G),
        w0=w0,
        v_sw=v_sw,
        v_st=v_st,
        v_trot=v_trot,
        feasible=feasible,
        i_bus=i_bus,
        t_hip=t_hip,
        t_knee=t_knee,
        tpk=tpk,
        est_share=sum(x[2] for x in lines if x[3] == "estimated") / cost,
    )


def main():
    rows = [point(a, r, L, V) for a in ACTS for r in RATIOS for L in LEGS for V in BUS]
    ok = [x for x in rows if x["feasible"]]
    # Pareto: maximise v_trot, minimise cost, minimise mass
    par = [
        x
        for x in ok
        if not any(
            y is not x
            and y["v_trot"] >= x["v_trot"]
            and y["cost"] <= x["cost"]
            and y["mass"] <= x["mass"]
            and (y["v_trot"], -y["cost"], -y["mass"])
            != (x["v_trot"], -x["cost"], -x["mass"])
            for y in ok
        )
    ]
    par.sort(key=lambda x: x["cost"])
    print(
        f"{len(rows)} points, {len(ok)} torque-feasible, {len(par)} on the Pareto front (speed up, cost down, mass down)\n"
    )
    hdr = "| actuator | knee r | L (m) | bus (V) | mass (kg) | cost (USD) | foot force peak (N) | 2F/mg | w0 (rad/s) | v swing (m/s) | v stance (m/s) | trot v (m/s) | bus peak (A) | batt margin (70 A) |"
    sep = "|" + "---|" * 14
    print("## Pareto front\n")
    print(hdr, sep, sep="\n")
    for x in par:
        print(
            f"| {x['act']} | {x['r']} | {x['L']} | {x['V']} | {x['mass']:.1f} | {x['cost']:.0f} | {x['f_peak']:.0f} | {x['ratio']:.1f} | {x['w0']:.1f} | "
            f"{x['v_sw']:.2f} | {x['v_st']:.2f} | {x['v_trot']:.2f} | {x['i_bus']:.0f} | {70 / x['i_bus']:.1f}x |"
        )
    print("\n## Best feasible point per actuator (highest trot v)\n")
    print(hdr, sep, sep="\n")
    for a in ACTS:
        c = [x for x in ok if x["act"] == a]
        if not c:
            print(f"| {a} | infeasible at every point |" + " |" * 12)
            continue
        x = max(c, key=lambda y: (y["v_trot"], -y["cost"]))
        print(
            f"| {x['act']} | {x['r']} | {x['L']} | {x['V']} | {x['mass']:.1f} | {x['cost']:.0f} | {x['f_peak']:.0f} | {x['ratio']:.1f} | {x['w0']:.1f} | "
            f"{x['v_sw']:.2f} | {x['v_st']:.2f} | {x['v_trot']:.2f} | {x['i_bus']:.0f} | {70 / x['i_bus']:.1f}x |"
        )
    print("\n## Infeasible points (stance torque above the actuator peak)\n")
    for a in ACTS:
        bad = [x for x in rows if x["act"] == a and not x["feasible"]]
        print(
            f"- {a}: {len(bad)} of {len(rows) // len(ACTS)} infeasible"
            + (
                f"; e.g. r={bad[0]['r']}, L={bad[0]['L']}: hip {bad[0]['t_hip']:.1f} / knee {bad[0]['t_knee']:.1f} N*m vs peak {bad[0]['tpk']}"
                if bad
                else ""
            )
        )
    x = max(ok, key=lambda y: y["v_trot"])
    print(
        "\n## BOM lines at L = 0.22 m (fixed part of the dog, without the 12 actuators)\n"
    )
    print("| item | kg | USD | price fidelity |", "|---|---|---|---|", sep="\n")
    for lab, kg, usd, fid in fixed_lines(0.22):
        print(f"| {lab} | {kg:.3f} | {usd:.0f} | {fid} |")
    tot_kg = sum(l[1] for l in fixed_lines(0.22))
    tot_usd = sum(l[2] for l in fixed_lines(0.22))
    est = sum(l[2] for l in fixed_lines(0.22) if l[3] == "estimated")
    print(
        f"| **fixed total** | {tot_kg:.2f} | {tot_usd:.0f} | estimated share {est / tot_usd:.0%} |"
    )
    print("\n## Head geometry (Mid-360 blind cone)\n")
    fmin, fmax = math.radians(-7), math.radians(52)
    hs, xf = (
        0.45,
        0.30,
    )  # sensor height above ground, front foot distance ahead of the sensor (both estimates; docs/dog/actuator-study.md)
    print(
        "| tilt (deg, nose down) | lower FOV edge (deg) | upper FOV edge (deg) | ground seen from (m ahead of sensor) | gap ahead of the front foot (m) |",
        "|---|---|---|---|---|",
        sep="\n",
    )
    for tilt in (0, 15, 25, 35, 45):
        low = -7 - tilt
        up = 52 - tilt
        d = hs / math.tan(math.radians(-low)) if low < 0 else float("inf")
        print(f"| {tilt} | {low} | {up} | {d:.2f} | {max(0.0, d - xf):.2f} |")


if __name__ == "__main__":
    main()
