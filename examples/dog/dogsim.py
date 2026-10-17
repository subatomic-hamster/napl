"""Dog sim bench (SP3b): build the dog, per-leg joint signs from the build report, planar leg IK, MIT-mode joint PD on the 12 Qdd
devices, stand / squat ramps, and the [interfaces.native] artefacts. SP4 replaces the PD with its controller; this stays the bench.

Leg convention: q = 0 is the build pose (straight leg, foot under the hip). For the UNMIRRORED leg (fl) q_knee = KNEE_BACK * flexion
bends the knee backward; a mirrored leg multiplies by its report["mirror_signs"] entry; the rear legs are the front ones turned
180 deg, so the same q gives the turned pose (their knees point forward)."""

import csv
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from napl import design

REPO = Path(__file__).resolve().parents[2]
ASM, PARTS = REPO / "assemblies" / "dog" / "dog.toml", REPO / "parts"
LEGS, JOINTS = ("fl", "fr", "rl", "rr"), ("abad", "hip", "knee")
KNEE_BACK = -1.0  # pinned by tests/python/test_dog_stand.py::test_stance_is_symmetric (flip it if the knees point the wrong way)
KP, KD = (
    100.0,
    2.0,
)  # N*m/rad, N*m*s/rad, joint side: stand gains (60 / 1.5 sagged 13 mm of the 15 mm monitor)
SQUAT, STAND = (
    0.84,
    0.92,
)  # hip-to-ball distance as a fraction of l1 + l2; the knee stops at 1.32 rad of flexion, which is 0.80 of the leg length
CMD_HZ = 500.0  # joint command rate (spec s6)
TRACE_COLS = ["t", "h_err", "tilt", "fault", "v_bat"]


def build(out, asm=ASM):
    r = design.build(asm, PARTS, out)
    return Path(r["xml"]), r["report"]


def timestep(xml):
    return float(ET.parse(xml).getroot().find("option").get("timestep"))


def signs(report):
    ms = report["mirror_signs"]
    return {f"{leg}:{j}": ms.get(f"{leg}:{j}_j", 1) for leg in LEGS for j in JOINTS}


def body(sim, inst):
    for p, x in sim.body_positions().items():
        if p.rsplit("/", 1)[-1] == inst:
            return np.asarray(x, dtype=float)
    raise KeyError(inst)


def leg_lengths(sim, leg="fl"):
    hip, knee, ball = (body(sim, f"{leg}:{i}") for i in ("thigh", "shank", "foot_ball"))
    return float(np.linalg.norm(knee - hip)), float(np.linalg.norm(ball - knee))


def ik(d, l1, l2):
    c = (l1 * l1 + l2 * l2 - d * d) / (2 * l1 * l2)
    phi = math.pi - math.acos(max(-1.0, min(1.0, c)))  # knee flexion
    a = math.acos(
        max(-1.0, min(1.0, (l1 * l1 + d * d - l2 * l2) / (2 * l1 * d)))
    )  # thigh off the vertical
    return -KNEE_BACK * a, KNEE_BACK * phi


def enable(sim, sg):
    for dev in sg:
        sim.set(f"/devices/{dev}", "enable", 1.0)


def command(sim, sg, q, kp=KP, kd=KD):
    for dev, s in sg.items():
        path, j = f"/devices/{dev}", dev.split(":")[1]
        for attr, val in (
            ("p_des", s * q[j]),
            ("v_des", 0.0),
            ("kp", kp),
            ("kd", kd),
            ("tau_ff", 0.0),
        ):
            sim.set(path, attr, val)


def tilt_deg(sim):
    w, x, y, z = sim.query("/world/deck_top", "quat")[0]
    zz = 1 - 2 * (x * x + y * y)  # body z axis . world z
    return math.degrees(math.acos(max(-1.0, min(1.0, zz))))


def sample(sim, sg, z_expected, dt):
    fault = max(float(sim.query(f"/devices/{d}", "fault")[0][0]) for d in sg)
    v = min(
        float(sim.query(f"/devices/power:{p}", "voltage")[0][0])
        for p in ("pack_a", "pack_b")
    )
    z = float(sim.query("/world/deck_top", "pos")[0][2])
    return [sim.tick * dt, z - z_expected, tilt_deg(sim), fault, v]


def ramp(sim, sg, l1, l2, a, b, T, rows, z0=None, dt=0.001):
    """Hip-to-ball distance from a to b (fractions of l1 + l2) over T s at CMD_HZ; rows get one sample per command."""
    z0 = (
        float(sim.query("/world/deck_top", "pos")[0][2]) + (l1 + l2) * (1 - a)
        if z0 is None
        else z0
    )  # deck z of straight legs
    n, k = max(1, round(T * CMD_HZ)), round(1 / (CMD_HZ * dt))
    for i in range(n):
        f = a + (b - a) * (i + 1) / n
        qh, qk = ik(f * (l1 + l2), l1, l2)
        command(sim, sg, {"abad": 0.0, "hip": qh, "knee": qk})
        sim.step(k)
        rows.append(sample(sim, sg, z0 - (l1 + l2) * (1 - f), dt))
    return z0


def finish(sim, run_dir, rows, dt):
    r = Path(run_dir)
    with open(r / "trace.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(TRACE_COLS)
        w.writerows(rows)
    (r / "replay.naplog").write_bytes(sim.log())
    (r / "native.json").write_text(
        json.dumps(
            dict(
                tick=sim.tick,
                dt=dt,
                state_hash=f"{sim.state_hash():016x}",
                diverged=bool(sim.diverged),
            )
        )
    )
