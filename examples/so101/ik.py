"""SO-101 kinematics and the pick trajectory: forward kinematics of the URDF (tools/so101_lib.py), damped-least-squares IK on the
5 arm joints, and `plan`: Cartesian waypoints -> per-tick joint / gripper goals. Pure numpy, no napl import."""

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))
import so101_lib

JNAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]
BUS_HZ = 50  # LeRobot's control loop default is 30 Hz; 50 Hz is a realistic teleop / policy rate
G_REF = 0.05  # rad: gripper angle at which the pads are parallel and 20 mm apart
G_OPEN = G_REF + 0.25  # about 40 mm between the pads
GRASP_TCP = np.array(
    [-0.0115, 0.0, -0.008]
)  # cube centre in the TCP frame: 1.5 mm from the fixed pad
HOVER = 0.075  # m above the grasp height (the wrist flex limit stops a top-down pose a little higher)

_, _UJ = so101_lib.parse_urdf()
ARM = JNAMES[:5]
LO = np.array([_UJ[j]["lower"] for j in JNAMES])
HI = np.array([_UJ[j]["upper"] for j in JNAMES])


def _rot(axis, q):
    a = np.array(axis, float) / np.linalg.norm(axis)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(q) * K + (1 - math.cos(q)) * K @ K


def fk_tcp(q):
    """4x4 pose of the URDF gripper_frame (the TCP) in the base_link = world frame; q = the 5 arm joint angles."""
    T = np.eye(4)
    for n, v in zip(ARM, q):
        R = np.eye(4)
        R[:3, :3] = _rot(_UJ[n]["axis"], v)
        T = T @ _UJ[n]["T"] @ R
    return T @ _UJ["gripper_frame_joint"]["T"]


def grasp_point(q):
    T = fk_tcp(q)
    return T[:3, :3] @ GRASP_TCP + T[:3, 3]


def _resid(q, p, xdir):
    T = fk_tcp(q)
    # cube centre at p, TCP z axis pointing down, TCP x axis (jaw opening direction) along xdir
    return np.concatenate(
        [
            T[:3, :3] @ GRASP_TCP + T[:3, 3] - p,
            0.1 * (T[:3, 2] - [0, 0, -1]),
            0.05 * (T[:3, 0] - xdir),
        ]
    )


def ik(p, xdir, q0, iters=100):
    """Damped least squares on the 5 arm joints (finite-difference Jacobian). -> (q, position error m)."""
    q = np.array(q0, float)
    for _ in range(iters):
        e = _resid(q, p, xdir)
        if np.linalg.norm(e[:3]) < 1e-6 and np.linalg.norm(e[3:]) < 1e-7:
            break
        J = np.empty((len(e), 5))
        for k in range(5):
            d = np.zeros(5)
            d[k] = 1e-6
            J[:, k] = (_resid(q + d, p, xdir) - e) / 1e-6
        q = np.clip(
            q - np.linalg.solve(J.T @ J + 1e-4 * np.eye(5), J.T @ e), LO[:5], HI[:5]
        )
    return q, float(np.linalg.norm(_resid(q, p, xdir)[:3]))


def max_reach_pose():
    """Arm angles (pan 0, roll 0) maximising the horizontal distance of the TCP from the pan axis: grid over lift / elbow /
    wrist flex, then coordinate ascent. At the URDF zero pose the upper arm points up, so the maximum is the arm stretched
    out horizontally (upper arm and forearm level). -> (arm angles, reach m)"""
    reach = lambda q: float(np.hypot(*fk_tcp(q)[:2, 3]))
    best, bq = -1.0, None
    g = np.linspace(-1.5, 1.5, 21)
    for a in g:
        for b in g:
            for c in g:
                q = np.array([0, a, b, c, 0.0])
                if reach(q) > best:
                    best, bq = reach(q), q
    step = 0.05
    while step > 1e-4:
        improved = False
        for k in (1, 2, 3):
            for s in (step, -step):
                q = bq.copy()
                q[k] = np.clip(q[k] + s, LO[k], HI[k])
                if reach(q) > best + 1e-12:
                    best, bq, improved = reach(q), q, True
        if not improved:
            step /= 2
    return bq, best


def _smooth(s):
    return s**3 * (10 - 15 * s + 6 * s * s)


def plan(c, t, g_squeeze, hz=BUS_HZ):
    """Per-tick (arm angles, gripper angle, phase) from home to the cube, to the target and back up. c, t: cube centre and target
    centre at rest (x, y, z). Returns the list and the home angles."""
    xd = np.array([0.0, 1.0, 0.0])
    up = np.array([0, 0, HOVER])
    home_p = (c + t) / 2 + up
    q, r = ik(home_p, xd, [0.1, -0.2, 0.3, 1.5, -1.3])
    assert r < 1e-4, f"home pose unreachable ({r})"
    q_home, rows, g = q.copy(), [], G_OPEN
    p = home_p

    def move(p1, T, label):
        nonlocal q, p
        for k in range(1, round(T * hz) + 1):
            q, r = ik(p + (p1 - p) * _smooth(k / (T * hz)), xd, q)
            assert r < 1e-4, f"{label}: IK error {r} m"
            rows.append((q.copy(), g, label))
        p = p1

    def grip(g1, T, label):
        nonlocal g
        g0 = g
        for k in range(1, round(T * hz) + 1):
            g = g0 + (g1 - g0) * _smooth(k / (T * hz))
            rows.append((q.copy(), g, label))

    def hold(T, label):
        rows.extend([(q.copy(), g, label)] * round(T * hz))

    hold(1.0, "settle at home")
    move(c + up, 1.5, "to pre-grasp")
    move(c, 2.0, "descend")
    hold(0.5, "settle")
    grip(g_squeeze, 1.0, "close")
    hold(0.7, "grip")
    move(c + up, 1.5, "lift")
    move(t + up, 2.0, "carry")
    move(t + [0, 0, 0.0005], 1.5, "lower")
    hold(0.4, "settle")
    grip(G_OPEN, 1.0, "open")
    hold(0.5, "released")
    move(t + up, 1.5, "retreat")
    hold(1.5, "cube settles")
    return rows, q_home
