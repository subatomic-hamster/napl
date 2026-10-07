"""N6-B+C: the simulated SO-101 (assemblies/so101.toml, 6 x STS3215 on the Feetech pty) does a pick-and-place driven by an
independent protocol client, and a payload sweep finds how much the arm holds at full reach.

    python tests/python/test_so101_pick.py        # runs both and prints the numbers

Scene (composed from the built MJCF in a temp dir, the assembly and the parts are not touched): a table whose top is the base's
bottom plane (the real arm is clamped to a table), a 20 mm solid PLA cube (density 1240 kg/m^3, friction 0.40, docs/research/
sts3215-calibration.md 5), and box contact pads on the two jaws (the jaw meshes' collision is switched off in this scene: MuJoCo
collides a mesh as its convex hull, which for these concave jaws fills the gap the cube has to fit in). Pad faces are 20 mm apart
and parallel at gripper angle G_REF, matching the jaw mesh faces (15.8 mm at 0 rad + 0.079 m/rad, measured on the STLs).
IK is damped least squares on the URDF forward kinematics (tools/so101_lib.py), checked against the built model's body poses.
Client: SYNC_WRITE goals + SYNC_READ present values of ids 1..6 at BUS_HZ, 1 Mbaud, port realtime, sim Host at rate 1.0.
"""

import math
import sys
import time
import xml.etree.ElementTree as ET
from math import pi
from pathlib import Path

import napl
import numpy as np
import pytest
from napl.feetech import FeetechPort

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "tools"))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_feetech import (
    JOINTS as ARM3_JOINTS,
)
from test_feetech import MODEL as ARM3_MODEL
from test_feetech import SERVOS as ARM3_SERVOS
from test_feetech import RawBus

# The reusable bits live in examples/so101 (the user program pick.py and the `napl program` scenario use them too).
sys.path.insert(0, str(REPO / "examples" / "so101"))
from ik import (
    ARM,
    BUS_HZ,
    G_OPEN,
    G_REF,
    HI,
    JNAMES,
    LO,
    fk_tcp,
    max_reach_pose,
    plan,
)
from scene import (
    _R,
    CUBE,
    CUBE_XY,
    TARGET_XY,
    _inv,
    built,
    mjcf_poses,
    scene,
)
from sts import (
    CPR,
    CURRENT_LSB,
    D_REG,
    GRIP_OVERLOAD,
    GRIP_PROT_CURRENT,
    GRIP_TORQUE,
    I_REG,
    P_REG,
    Bus,
    configure,
    decode,
    goal_bytes,
)

I_PROT = GRIP_PROT_CURRENT * CURRENT_LSB  # 1.625 A
I_STALL = 2.5  # A, datasheet stall current
SQUEEZE_DUTY = (
    0.15  # gripper duty while holding the cube: below its overload threshold (25 %)
)


def _servo_cfg():
    return {s["name"]: s for s in built()[2]["servos"]}


# ---- the pick-and-place run ------------------------------------------------------------------------------------------------
def run_pick(verbose=True):
    cfg = _servo_cfg()
    pid_scale = cfg["servo1"]["pid_scale"]
    path, zt = scene(cube_xy=CUBE_XY)
    c = np.array([*CUBE_XY, zt + CUBE / 2])
    t = np.array([*TARGET_XY, zt + CUBE / 2])
    e_sq = (
        SQUEEZE_DUTY * pid_scale / P_REG
    )  # counts of goal beyond the contact angle: duty = P e / pid_scale
    rows, q_home = plan(c, t, G_REF - e_sq * 2 * pi / CPR)
    sim = napl.Sim.load(path, host=True)
    _, jpath = mjcf_poses(ET.fromstring(built()[1]))
    servos = [f"/devices/servo{i}" for i in range(1, 7)]
    q = lambda i, a: float(sim.query(servos[i - 1], a)[0][0])
    for s in servos:
        sim.set(
            s, "torque_enable", 0
        )  # known state, then place the arm at the home pose
    for n, v in zip(JNAMES, [*q_home, G_OPEN]):
        sim.set(jpath[n], "qpos", float(v))
    sim.step(3)
    cube_z0 = float(sim.query("/world/cube", "pos")[0][2])
    m = dict(
        pid_scale=pid_scale,
        cube_z0=cube_z0,
        zt=zt,
        cube_zmax=cube_z0,
        grip_i_bus=0.0,
        grip_i_sim=0.0,
        alarm_attr=0,
        cycles=0,
        late=0,
        periods=[],
        i_bus=[0.0] * 6,
        i_sim=[0.0] * 6,
        maxerr=0.0,
        cube_hold_z=[],
        grip_by_phase={},
    )
    with FeetechPort(
        sim, {i: servos[i - 1] for i in range(1, 7)}, realtime=True
    ) as port:
        bus = Bus(port.device_path)
        configure(bus)
        off = np.zeros(
            5
        )  # client-side compensation of the P-only servo's gravity sag (see run_pick docstring in the report)
        assert sim.call({"cmd_type": "Run", "cmd": {"rate": 1.0}})["ok"]
        tick0, wall0, dt = sim.tick, time.monotonic(), 1.0 / BUS_HZ
        t_next, last = wall0, wall0
        for k, (qa, g, label) in enumerate(rows):
            now = time.monotonic()
            if t_next > now:
                time.sleep(t_next - now)
            else:
                m["late"] += 1
            t_next += dt
            qc = np.clip(qa + off, LO[:5], HI[:5])
            bus.sync_write(
                42,
                {i + 1: goal_bytes(v * CPR / (2 * pi)) for i, v in enumerate([*qc, g])},
            )
            r = bus.sync_read(56, 15, range(1, 7))
            st = {i: decode(d) for i, d in r.items()}
            qm = np.array([st[i]["q"] for i in range(1, 6)])
            off = np.clip(
                off + 3.0 * dt * (qa - qm), -0.4, 0.4
            )  # integrates the sag away (rate 3 /s)
            m["maxerr"] = max(m["maxerr"], float(np.abs(qa - qm).max()))
            for i in range(1, 7):
                m["i_bus"][i - 1] = max(m["i_bus"][i - 1], st[i]["current"])
                a = q(i, "alarm")
                m["alarm_attr"] |= int(a)
                m["i_sim"][i - 1] = max(m["i_sim"][i - 1], abs(q(i, "current")))
            cz = float(sim.query("/world/cube", "pos")[0][2])
            m["cube_zmax"] = max(m["cube_zmax"], cz)
            if label == "carry":
                m["cube_hold_z"].append(cz)
            m.setdefault("grip_duty", []).append(st[6]["load"])
            if bus.errs and "first_err" not in m:
                m["first_err"] = (k, label, bus.errs[0])
            gl = m["grip_by_phase"].setdefault(label, [0.0, 0.0, 0.0])
            gl[0] = max(gl[0], abs(st[6]["load"]))
            gl[1] = max(gl[1], abs(g - st[6]["q"]) * CPR / (2 * pi))
            gl[2] = max(gl[2], st[6]["current"])
            m["cycles"] += 1
            now = time.monotonic()
            m["periods"].append(now - last)
            last = now
        wall, ticks = time.monotonic() - wall0, sim.tick - tick0
        sim.call({"cmd_type": "Pause", "cmd": {}})
        bus.close()
        m.update(
            wall=wall,
            sim_s=ticks * 0.001,
            bus_errs=list(bus.errs),
            bad=bus.bad,
            port_errors=list(port.errors),
        )
    cp = np.array(sim.query("/world/cube", "pos")[0])
    m["cube_end"] = cp
    m["place_err"] = float(np.hypot(*(cp[:2] - np.array(TARGET_XY))))
    m["cube_vel"] = float(
        np.abs(np.array(sim.query("/world/cube/cube_free", "qvel")[0])).max()
    )
    m["grip_i_bus"], m["grip_i_sim"] = m["i_bus"][5], m["i_sim"][5]
    if verbose:
        print_pick(m, cfg)
    return m


def print_pick(m, cfg):
    p = np.array(m["periods"])
    print(
        f"\n[pick] pid_scale {m['pid_scale']:.0f} (read from the build), P/I/D {P_REG}/{I_REG}/{D_REG}, gripper torque limit "
        f"{GRIP_TORQUE}, overload {GRIP_OVERLOAD} %, protection current {GRIP_PROT_CURRENT} x 6.5 mA = {I_PROT:.3f} A"
    )
    print(
        f"[pick] bus {BUS_HZ} Hz at 1 Mbaud, port realtime, sim Host rate 1.0: {m['cycles']} cycles, mean period {p.mean() * 1e3:.2f} ms "
        f"(max {p.max() * 1e3:.1f} ms), {m['late']} late cycles, wall {m['wall']:.1f} s, sim {m['sim_s']:.1f} s"
    )
    print(
        f"[pick] lift height: cube centre max gain {m['cube_zmax'] - m['cube_z0']:.4f} m (start z {m['cube_z0']:.4f}, "
        f"carried at z {np.mean(m['cube_hold_z']):.4f} +- {np.ptp(m['cube_hold_z']) / 2:.4f})"
    )
    print(
        f"[pick] placement error: {m['place_err'] * 1000:.2f} mm (xy) to target {TARGET_XY}, cube ends at {np.round(m['cube_end'], 4)}, "
        f"rest height error {(m['cube_end'][2] - (m['zt'] + CUBE / 2)) * 1000:.2f} mm, speed {m['cube_vel']:.2e}"
    )
    print(
        f"[pick] gripper current max: bus {m['grip_i_bus']:.3f} A, sim {m['grip_i_sim']:.3f} A (protection {I_PROT:.3f} A, stall {I_STALL} A); "
        f"all servos max bus A {[round(x, 3) for x in m['i_bus']]}, sim A {[round(x, 3) for x in m['i_sim']]}"
    )
    if "first_err" in m:
        k = m["first_err"][0]
        print(
            f"[pick] first error byte {m['first_err']}; gripper duty every 10th cycle from 0: "
            f"{[round(x, 2) for x in m['grip_duty'][::10]]}"
        )
    for k, (d, e, i) in m["grip_by_phase"].items():
        print(
            f"[pick]   gripper {k:16s} max |duty| {d:.3f}  max goal error {e:6.1f} counts  max current {i:.3f} A"
        )
    print(
        f"[pick] alarms: status error bytes nonzero {len(m['bus_errs'])} of {m['cycles'] * 6} ({sorted(set(m['bus_errs'])) or 'none'}), device alarm attr OR {m['alarm_attr']}, bad frames {m['bad']}, "
        f"port.errors {m['port_errors']}; max arm tracking error {math.degrees(m['maxerr']):.2f} deg"
    )


def test_fk_matches_the_built_model():
    """The URDF forward kinematics used by the IK against the body poses of the built MJCF (through the kernel), at random poses."""
    path, _ = scene()
    sim = napl.Sim.load(path)
    root = ET.fromstring(built()[1])
    poses, jpath = mjcf_poses(root)
    T_in_fj = _inv(poses["fixed_jaw"]) @ fk_tcp(
        np.zeros(5)
    )  # TCP frame in the fixed jaw body frame (constant)
    rng = np.random.default_rng(1)
    worst = 0.0
    for _ in range(4):
        qa = rng.uniform(LO[:5] * 0.5, HI[:5] * 0.5)
        for n, v in zip(ARM, qa):
            sim.set(jpath[n], "qpos", float(v))
        pb = np.array(sim.query(_body_path(root, "fixed_jaw"), "pos")[0])
        Rb = _R(sim.query(_body_path(root, "fixed_jaw"), "quat")[0])
        Tb = np.eye(4)
        Tb[:3, :3], Tb[:3, 3] = Rb, pb
        worst = max(
            worst, float(np.abs((Tb @ T_in_fj)[:3, 3] - fk_tcp(qa)[:3, 3]).max())
        )
        mj, _ = mjcf_poses(
            root, dict(zip(ARM, qa))
        )  # the test's own MJCF walk agrees with the kernel too
        worst = max(worst, float(np.abs(mj["fixed_jaw"][:3, 3] - pb).max()))
    print(
        f"\n[fk] URDF FK vs built model, worst TCP / body position difference {worst:.2e} m"
    )
    assert worst < 1e-4


def _body_path(root, name):
    out = {}

    def walk(b, path):
        path = f"{path}/{b.get('name')}"
        out[b.get("name")] = path
        for c in b.findall("body"):
            walk(c, path)

    for b in root.find("worldbody").findall("body"):
        walk(b, "/world")
    return out[name]


def test_goal_from_a_negative_encoder_stays_in_turn_zero():
    """feetech.py regression: a joint at -65 counts must take a +100 goal as +100 counts (it was -3996: the turn was floored)."""
    sim = napl.Sim.load(ARM3_MODEL, host=True)
    with FeetechPort(sim, ARM3_SERVOS, realtime=False) as p:
        b = RawBus(p.device_path)
        sim.set(ARM3_JOINTS[1], "qpos", -65 * 2 * pi / CPR)
        sim.step(1)
        b.enable(1)
        goal = lambda: sim.query("/devices/s1", "goal_position")[0][0] * CPR / (2 * pi)
        b.write_pos(1, 100)
        assert goal() == pytest.approx(100)
        b.write(
            1, 11, 0, 2
        )  # max angle limit 0: no clamp, so a negative (bit 15) goal is allowed
        b.write_pos(1, 0x8000 | 100)
        assert goal() == pytest.approx(-100)
        b.close()


def test_pick_and_place_through_the_feetech_pty():
    m = run_pick()
    assert m["cube_zmax"] - m["cube_z0"] >= 0.05
    assert m["place_err"] < 0.01
    assert (
        abs(m["cube_end"][2] - (m["zt"] + CUBE / 2)) < 1e-3 and m["cube_vel"] < 0.02
    )  # resting on the table
    assert m["grip_i_bus"] <= I_PROT and m["grip_i_sim"] <= I_PROT
    assert max(m["i_bus"]) <= I_STALL and max(m["i_sim"]) <= I_STALL
    assert m["bus_errs"] == [] and m["alarm_attr"] == 0 and m["bad"] == 0
    assert m["port_errors"] == []
    assert m["wall"] < 60


# ---- payload sweep ----------------------------------------------------------------------------------------------------------
FACTORY_PROTECTION = dict(  # feetech.py _Servo.reset: temp 70 degC, voltage 4.0 .. 8.0 V, overload 80 % for 2 s, unload mask 36
    max_temp=70.0,
    max_voltage=8.0,
    min_voltage=4.0,
    overload_time=2.0,
    overload_load=0.8,
    unload_mask=36.0,
)
HOLD_S, SETTLE_S, ERR_LIMIT = 3.0, 3.0, math.radians(5.0)


def hold_payload(mass, q_pose):
    """Arm at q_pose (teleported there, torque on, goal = pose), payload `mass` kg welded at the TCP, simulated at full speed
    through direct SetAttr of the same registers' attrs (no pty: it is the model that is under test, not the wire). Runs
    SETTLE_S + HOLD_S; -> dict: alarm bits, per-joint max |goal - q| and max |duty| in the last HOLD_S."""
    path, _ = scene(payload_kg=mass)
    sim = napl.Sim.load(path)
    _, jpath = mjcf_poses(ET.fromstring(built()[1]))
    pose = [*q_pose, G_REF]
    for i, (n, v) in enumerate(zip(JNAMES, pose), 1):
        s = f"/devices/servo{i}"
        sim.set(s, "p_gain", P_REG)
        sim.set(s, "i_gain", I_REG)
        sim.set(s, "d_gain", D_REG)
        # the so101 build leaves protection off; these are the RESET defaults
        for attr, val in FACTORY_PROTECTION.items():
            sim.set(s, attr, val)
        if i == 6:
            sim.set(s, "max_torque", GRIP_TORQUE)
            sim.set(s, "overload_load", GRIP_OVERLOAD / 100)
        sim.set(s, "torque_enable", 0)
        sim.set(jpath[n], "qpos", float(v))
    sim.step(3)
    for i, v in enumerate(pose, 1):
        sim.set(f"/devices/servo{i}", "goal_position", float(v))
        sim.set(f"/devices/servo{i}", "torque_enable", 1)
    err, duty, alarm = np.zeros(6), np.zeros(6), np.zeros(6, int)
    first_trip = None
    for k in range(int((SETTLE_S + HOLD_S) * 10)):
        sim.step(100)
        late = k * 0.1 >= SETTLE_S
        for i in range(6):
            a = int(sim.query(f"/devices/servo{i + 1}", "alarm")[0][0])
            alarm[i] |= a
            if a & 32 and i < 5 and first_trip is None:
                first_trip = (i, (k + 1) * 0.1)
            if late:
                e = abs(float(sim.query(jpath[JNAMES[i]], "qpos")[0][0]) - pose[i])
                err[i] = max(err[i], e)
                duty[i] = max(
                    duty[i],
                    abs(float(sim.query(f"/devices/servo{i + 1}", "load")[0][0])),
                )
    return dict(
        mass=mass,
        err=err,
        duty=duty,
        alarm=alarm,
        trip=bool((alarm[:5] & 32).any()),
        trip_joint=first_trip,
        over=bool((err[:5] > ERR_LIMIT).any()),
        diverged=sim.diverged,
    )


COARSE_KG = [0.0, 0.025, 0.05, 0.1, 0.15, 0.2, 0.3, 0.4, 0.5, 0.75, 1.0, 1.5, 2.0]
trips = lambda r: (
    r["trip"] or r["diverged"]
)  # overload alarm on a loaded joint (or the sim blew up)
exceeds = lambda r: r["over"]  # a joint's tracking error above 5 deg in the last HOLD_S


def payload_sweep(verbose=True):
    """-> dict: per criterion (trip / err) the bracket (last passing mass, first failing mass) in kg: coarse list, then 6
    bisections (more mass, more torque: monotone). (None, 0.0) = the unloaded arm already fails; (None, None) = not reached."""
    q_pose, reach = max_reach_pose()
    res = {}

    def get(m):
        if m not in res:
            res[m] = hold_payload(m, q_pose)
            if verbose:
                print(f"[payload] {m * 1000:7.1f} g  {_row(res[m])}")
        return res[m]

    def bracket(pred):
        lo = None
        for m in COARSE_KG:
            if pred(get(m)):
                break
            lo = m
        else:
            return None, None
        if lo is None:
            return None, 0.0
        hi = m
        for _ in range(6):
            mid = round((lo + hi) / 2, 4)
            lo, hi = (mid, hi) if not pred(get(mid)) else (lo, mid)
        return lo, hi

    return dict(
        q_pose=q_pose,
        reach=reach,
        res=res,
        trip=bracket(trips),
        err=bracket(exceeds),
        pid_scale=_servo_cfg()["servo1"]["pid_scale"],
    )


def _row(r):
    j = int(np.argmax(r["err"][:5]))
    return (
        f"trip {'Y' if r['trip'] else '-'} {r['trip_joint'] or ''} err>5deg {'Y' if r['over'] else '-'}  "
        f"max err deg {np.round(np.degrees(r['err'][:5]), 2).tolist()} duty {np.round(r['duty'][:5], 3).tolist()} (worst joint {JNAMES[j]})"
    )


def print_sweep(s):
    print(
        f"\n[payload] pose {np.round(s['q_pose'], 3).tolist()} rad, TCP reach {s['reach'] * 1000:.0f} mm from the pan axis at z {fk_tcp(s['q_pose'])[2, 3] * 1000:.0f} mm, "
        f"pid_scale {s['pid_scale']:.0f}, P/I/D {P_REG}/{I_REG}/{D_REG}, hold {SETTLE_S:.0f} s settle + {HOLD_S:.0f} s"
    )
    for name, key in (
        ("overload trip (80 % for 2 s)", "trip"),
        ("tracking error > 5 deg", "err"),
    ):
        lo, hi = s[key]
        if hi is None:
            print(f"[payload] {name}: not reached up to {COARSE_KG[-1] * 1000:.0f} g")
            continue
        r = s["res"][hi]
        j = (
            r["trip_joint"][0]
            if key == "trip" and r["trip_joint"]
            else int(np.argmax(r["err"][:5]))
        )
        print(
            f"[payload] {name}: holds {'(nothing: the unloaded arm fails)' if lo is None else f'{lo * 1000:.1f} g'}, fails at {hi * 1000:.1f} g; "
            f"limiting joint {JNAMES[j]}, duty {r['duty'][j]:.3f}, tracking error {math.degrees(r['err'][j]):.2f} deg, alarm {int(r['alarm'][j])}"
        )
    his = [h for _, h in (s["trip"], s["err"]) if h is not None]
    print(
        f"[payload] LIMIT (first mass with a trip or > 5 deg error): {min(his) * 1000:.1f} g"
    )
    print(
        "[payload] published: 500 g only for the 12 V SO-ARM101 Pro kit (docs/research/sts3215-calibration.md 3); none for the "
        "7.4 V / 5 V standard kit"
    )


def test_payload_sweep_limit_is_consistent():
    s = payload_sweep()
    print_sweep(s)
    found = 0
    for key, pred in (("trip", trips), ("err", exceeds)):
        lo, hi = s[key]
        if hi is None:
            continue
        found += 1
        assert pred(s["res"][hi])  # at the reported mass the criterion fires ...
        if lo is not None:  # ... and just below it does not
            assert lo < hi and not pred(s["res"][lo])
        else:
            assert hi == 0.0  # the unloaded arm already fails
    assert found, "no criterion reached up to 2 kg"
    # a trip needs the 80 % duty for 2 s; the error criterion needs no alarm: they are independent
    assert all(not r["diverged"] for r in s["res"].values())


if __name__ == "__main__":
    test_fk_matches_the_built_model()
    test_goal_from_a_negative_encoder_stays_in_turn_zero()
    test_pick_and_place_through_the_feetech_pty()
    test_payload_sweep_limit_is_consistent()
