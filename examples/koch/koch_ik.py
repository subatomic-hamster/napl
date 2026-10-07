"""Koch v1.1 follower kinematics and the pick trajectory: forward kinematics of meshes/koch/follower.urdf (via tools/so101_lib.py's
URDF parser), damped-least-squares IK on the 5 arm joints, and `plan`: Cartesian waypoints -> per-tick joint / gripper goals.
Same method as examples/so101/ik.py (named koch_ik so both scenes can be imported in one process). World = the URDF base_link frame
(assemblies/koch.toml root_frame = "urdf_link"); q = 0 is the URDF zero (upper arm vertical, forearm along -x), positive q = URDF
joint direction. Pure numpy, no napl import."""

import math
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "tools"))
import so101_lib

BUS_HZ = 50  # LeRobot's control loop default is 30 Hz; 50 Hz is a realistic teleop / policy rate
# Joint ranges of the assembly (the part files' `<joint>_range`, estimated; the URDF has +-3.14 placeholders).
LO = np.array([-2.2, -1.57, -1.45, -2.0, -3.14, -1.6])
HI = np.array([2.2, 0.6, 1.57, 2.0, 3.14, 0.032])
# Grip point in the link_5 frame (the fixed jaw's frame): fingers point along -x5, they open along y5 (the moving jaw swings to -y5).
# The static pad's inner face is at y5 = +0.0095, the cube (20 mm) centre 10 mm further to -y5, 62.5 mm out along the finger.
GRASP_L5 = np.array([-0.0625, -0.0005, 0.0])
G_REF = -0.30  # rad: gripper angle at which the moving pad face is parallel to the static one, 20 mm apart (scene.py)
G_OPEN = G_REF - 0.25  # about 33 mm between the pads
HOVER = 0.06  # m above the grasp height (the wrist flex limit stops a top-down pose a little higher)

_, _UJ = so101_lib.parse_urdf(REPO / "meshes" / "koch" / "follower.urdf")
ARM = [f"joint_{i}" for i in range(1, 6)]


def _rot(axis, q):
    a = np.array(axis, float) / np.linalg.norm(axis)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(q) * K + (1 - math.cos(q)) * K @ K


def fk_l5(q):
    """4x4 pose of link_5 (the fixed jaw / wrist-roll output) in the base_link = world frame; q = the 5 arm joint angles."""
    T = np.eye(4)
    for n, v in zip(ARM, q):
        R = np.eye(4)
        R[:3, :3] = _rot(_UJ[n]["axis"], v)
        T = T @ _UJ[n]["T"] @ R
    return T


def _resid(q, p, ydir):
    T = fk_l5(q)
    # grip point at p, fingers pointing down (-x5 = -z: x5 = +z), jaw opening direction y5 along ydir
    return np.concatenate(
        [
            T[:3, :3] @ GRASP_L5 + T[:3, 3] - p,
            0.1 * (T[:3, 0] - [0, 0, 1]),
            0.05 * (T[:3, 1] - ydir),
        ]
    )


def ik(p, ydir, q0, iters=100):
    """Damped least squares on the 5 arm joints (finite-difference Jacobian). -> (q, position error m)."""
    q = np.array(q0, float)
    for _ in range(iters):
        e = _resid(q, p, ydir)
        if np.linalg.norm(e[:3]) < 1e-6 and np.linalg.norm(e[3:]) < 1e-7:
            break
        J = np.empty((len(e), 5))
        for k in range(5):
            d = np.zeros(5)
            d[k] = 1e-6
            J[:, k] = (_resid(q + d, p, ydir) - e) / 1e-6
        q = np.clip(
            q - np.linalg.solve(J.T @ J + 1e-4 * np.eye(5), J.T @ e), LO[:5], HI[:5]
        )
    return q, float(np.linalg.norm(_resid(q, p, ydir)[:3]))


def _smooth(s):
    return s**3 * (10 - 15 * s + 6 * s * s)


def plan(c, t, g_squeeze, q_seed, ydir, hz=BUS_HZ):
    """Per-tick (arm angles, gripper angle, phase) from home to the cube, to the target and back up. c, t: cube centre and target
    centre at rest (x, y, z). Returns the list and the home angles."""
    up = np.array([0, 0, HOVER])
    home_p = (c + t) / 2 + up
    q, r = ik(home_p, ydir, q_seed)
    assert r < 1e-4, f"home pose unreachable ({r})"
    q_home, rows, g = q.copy(), [], G_OPEN
    p = home_p

    def move(p1, T, label):
        nonlocal q, p
        for k in range(1, round(T * hz) + 1):
            q, r = ik(p + (p1 - p) * _smooth(k / (T * hz)), ydir, q)
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
    hold(1.0, "settle")
    grip(g_squeeze, 1.0, "close")
    hold(0.7, "grip")
    move(c + up, 1.5, "lift")
    move(t + up, 2.0, "carry")
    move(t + [0, 0, 0.0005], 1.5, "lower")
    hold(0.6, "settle")
    grip(G_OPEN, 1.0, "open")
    hold(0.5, "released")
    move(t + up, 1.5, "retreat")
    hold(1.5, "cube settles")
    return rows, q_home
