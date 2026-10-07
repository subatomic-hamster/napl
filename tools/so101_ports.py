#!/usr/bin/env python3
"""Derive [[ports]] for the SO-101 follower arm parts from the official URDF and self-check them.

  python tools/so101_ports.py            print evidence, derived ports, TOML snippets, closure check
  python tools/so101_ports.py --check    additionally compose from the ports written in parts/arm/*.toml
                                         (exit 1 if any placement error > 1e-9 or a port is missing)
  python tools/so101_ports.py --evidence also print the (slow) servo-holder contact analysis

Conventions (see parts/README.md): port frame P = [x'=ref, y'=axis x ref, z'=axis | pos];
mate: T_b = T_a @ P_a @ Rz(2 pi k/n) @ Rx(pi) @ inv(P_b), n = gcd(sym_a, sym_b), k = 0.
Stdlib + numpy only. The tool never writes part files.
"""

import math
import sys
import tomllib

from so101_lib import ROOT, RX_PI, Rz, T_from, inv, np, parse_urdf, read_stl

TOL = 1e-9
SRC = "meshes/so101/so101_new_calib.urdf (SO-ARM100 a758567)"
SERVO = "feetech-sts3215-c001"
PRINTED = {
    "base_so101_v2": "so101-base",
    "base_motor_holder_so101_v1": "so101-base-motor-holder",
    "waveshare_mounting_plate_so101_v2": "so101-waveshare-mounting-plate",
    "motor_holder_so101_base_v1": "so101-motor-holder-shoulder",
    "rotation_pitch_so101_v1": "so101-rotation-pitch",
    "upper_arm_so101_v1": "so101-upper-arm",
    "under_arm_so101_v1": "so101-lower-arm",
    "motor_holder_so101_wrist_v1": "so101-motor-holder-wrist",
    "wrist_roll_pitch_so101_v2": "so101-wrist-roll-pitch",
    "wrist_roll_follower_so101_v1": "so101-gripper-fixed-jaw",
    "moving_jaw_so101_v1": "so101-moving-jaw",
}
JOINTS = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]
CHILD_PART = {  # part the servo horn drives (verified by --evidence: nearest to the joint origin)
    "shoulder_pan": "so101-rotation-pitch",
    "shoulder_lift": "so101-upper-arm",
    "elbow_flex": "so101-lower-arm",
    "wrist_flex": "so101-wrist-roll-pitch",
    "wrist_roll": "so101-gripper-fixed-jaw",
    "gripper": "so101-moving-jaw",
}
HOLDER = {  # printed part that holds the servo sitting in the joint's parent link (see --evidence)
    "shoulder_pan": "so101-base",
    "shoulder_lift": "so101-rotation-pitch",
    "elbow_flex": "so101-upper-arm",
    "wrist_flex": "so101-lower-arm",
    "wrist_roll": "so101-wrist-roll-pitch",
    "gripper": "so101-gripper-fixed-jaw",
}
FIXED = [  # (part a, port on a, part b, port on b, port type)
    (
        "so101-base",
        "motor_holder_seat",
        "so101-base-motor-holder",
        "base_seat",
        "bolt-pattern.so101-base-motor-holder",
    ),
    (
        "so101-base",
        "plate_seat",
        "so101-waveshare-mounting-plate",
        "base_seat",
        "bolt-pattern.so101-base-waveshare-plate",
    ),
    (
        "so101-rotation-pitch",
        "holder_seat",
        "so101-motor-holder-shoulder",
        "rotation_pitch_seat",
        "bolt-pattern.so101-rotation-pitch-motor-holder",
    ),
    (
        "so101-lower-arm",
        "holder_seat",
        "so101-motor-holder-wrist",
        "lower_arm_seat",
        "bolt-pattern.so101-lower-arm-motor-holder",
    ),
]
BOARD = "waveshare-bus-servo-adapter-a"
BOARD_HALF_H = 0.006  # adapter geometry box half height in parts/arm/waveshare-bus-servo-adapter-a.toml
PLATE_FACE_Z = 0.004  # plate main face (mesh z level), see plate_holes()
THK_PRINTED = (
    0.003,
    "No published or measurable value (exported mesh, no screw-boss data); 3 mm placeholder (2 mm printed wall plus boss margin). Estimated.",
)
THK_JOIN = (
    0.004,
    "No published or measurable value; 4 mm placeholder = two 2 mm printed walls clamped by M3 screws. Estimated.",
)


def say(*a):
    print(*a)  # ttsr-ignore: py/no-print (CLI tool)


def rnd(v):
    return [round(float(x), 12) + 0.0 for x in v]


def unit(v):
    v = np.asarray(v, float)
    return v / np.linalg.norm(v)


def P_from(pos, axis, ref):
    z = unit(axis)
    x = unit(ref)
    P = np.eye(4)
    P[:3, 0], P[:3, 1], P[:3, 2], P[:3, 3] = x, np.cross(z, x), z, pos
    return P


def mkport(P, **meta):
    """Round a port frame to 12 decimals; the stored P is rebuilt from the rounded vectors."""
    d = dict(pos=rnd(P[:3, 3]), axis=rnd(P[:3, 2]), ref=rnd(P[:3, 0]), **meta)
    d["P"] = P_from(d["pos"], d["axis"], d["ref"])
    return d


def ang(R):
    w = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) / 2
    return math.atan2(np.linalg.norm(w), (np.trace(R) - 1) / 2)


def frame_from_axes(origin, zax, xseed):
    z = unit(zax)
    x = np.asarray(xseed, float)
    x = unit(x - z * (x @ z))
    return P_from(origin, z, x)


class Model:
    def __init__(self):
        self.links, self.joints = parse_urdf()
        self.fk = {"base_link": np.eye(4)}
        todo = list(self.joints.values())  # URDF lists children before parents
        while todo:
            rest = [j for j in todo if j["parent"] not in self.fk]
            for j in todo:
                if j["parent"] in self.fk:
                    self.fk[j["child"]] = self.fk[j["parent"]] @ j["T"]  # q = 0
            assert len(rest) < len(todo), "disconnected joints"
            todo = rest
        self.servo_vis = {}  # joint -> (link, mesh, T_link_visual)
        for jn in JOINTS:
            link = self.joints[jn]["parent"]
            ((s, T),) = [(s, T) for s, T in self.links[link] if s.startswith("sts3215")]
            self.servo_vis[jn] = (link, s, T)
        self.part_vis = {}  # part id -> (link, mesh, T_link_visual)
        for link, vis in self.links.items():
            for s, T in vis:
                if s in PRINTED:
                    self.part_vis[PRINTED[s]] = (link, s, T)
        self.expected = {}  # instance -> (part id, T_world)
        for pid, (link, s, T) in self.part_vis.items():
            self.expected[pid] = (pid, self.fk[link] @ T)
        for jn, (link, s, T) in self.servo_vis.items():
            self.expected[f"servo:{jn}"] = (SERVO, self.fk[link] @ T)

    def mesh_pts(self, mesh, T):
        v = read_stl(ROOT / "meshes/so101" / f"{mesh}.stl").reshape(-1, 3)
        return v @ T[:3, :3].T + T[:3, 3]


def horn_vertices():
    """Vertices (rounded 1e-6) present in the horn mesh but not in the no-horn mesh."""
    v = {
        n: read_stl(ROOT / "meshes/so101" / f"{n}.stl").reshape(-1, 3)
        for n in ("sts3215_03a_v1", "sts3215_03a_no_horn_v1")
    }
    k = {n: {tuple(r) for r in np.round(a, 6)} for n, a in v.items()}
    return np.array(sorted(k["sts3215_03a_v1"] - k["sts3215_03a_no_horn_v1"]))


def derive(m, say_=say):
    ports = {}  # part id -> {port name: dict}
    ports.setdefault(SERVO, {})
    # ---- servo output port from the six joints
    Ps = {jn: inv(m.servo_vis[jn][2]) @ m.joints[jn]["T"] for jn in JOINTS}
    org = np.array([P[:3, 3] for P in Ps.values()])
    zs = np.array([P[:3, 2] for P in Ps.values()])
    # horn side from the mesh: vertices only in the horn mesh (the 20 mm horn disc) vs the no-horn mesh
    hv = horn_vertices()
    side = (
        -1.0 if hv[:, 2].mean() < 0 else 1.0
    )  # horn lies at -z (side = -1) of the servo mesh
    hz = hv[:, 2].min() if side < 0 else hv[:, 2].max()  # outer horn face
    hthk = float(hv[:, 2].max() - hv[:, 2].min())
    mean_o = org.mean(0)
    zmean = unit(
        zs.mean(0) * np.sign(zs.mean(0)[2] * side)
    )  # outward horn normal ~ +/- the joint z axis
    mean_o = mean_o + zmean * (
        (hz - mean_o[2]) / zmean[2]
    )  # slide along the axis to the outer horn face
    P_out = frame_from_axes(mean_o, zmean, [1, 0, 0])
    org = org - np.outer(
        org @ zmean - mean_o @ zmean, zmean
    )  # compare origins in the horn-face plane
    spread = dict(
        origin=max(np.linalg.norm(o - mean_o) for o in org),
        zaxis=max(math.acos(min(1.0, float(unit(z) @ zmean))) for z in zs),
        horn_side=side,
        horn_face_z=hz,
        horn_thickness=hthk,
        per_joint_rot_deg={
            jn: math.degrees(
                math.atan2(P_out[:3, 1] @ P[:3, 0], P_out[:3, 0] @ P[:3, 0])
            )
            for jn, P in Ps.items()
        },
        reverse={jn: bool((P[:3, 2] @ P_out[:3, 2]) < 0) for jn, P in Ps.items()},
        origin_mean=mean_o,
    )
    der = f"tools/so101_ports.py from {SRC}: P_out = frame at the mean over the 6 joints of inv(T_servo_i) @ J_i (servo mesh frame), moved along the axis to the outer face of the horn disc (z = {hz} in the servo mesh; horn disc = vertices only in sts3215_03a_v1.stl and not in sts3215_03a_no_horn_v1.stl); axis = outward horn normal = joint z axis here (the horn lies on the -z side of the servo mesh)"
    ports[SERVO]["output"] = mkport(
        P_out,
        type="servo-horn.25t",
        gender="male",
        symmetry=25,
        thickness=round(hthk, 4),
        fidelity="derived",
        derivation=der,
        thickness_rationale=f"Horn disc thickness read from the servo mesh (horn-only vertices span {hthk * 1e3:.1f} mm in z). No published value; estimated as the clamped stack.",
    )
    # ---- servo base port (convention, see rationale): outer case plane opposite the horn
    P_base = frame_from_axes([0, 0, -side * 0.0159], [0, 0, -side], [1, 0, 0])
    ports[SERVO]["base"] = mkport(
        P_base,
        type="bolt-pattern.feetech-sts3215-base",
        gender="neutral",
        symmetry=2,
        thickness=0.0015,
        fidelity="estimated",
        rationale="Frame convention, not a measured hole pattern: centre of the outer case plane opposite the horn (z = +0.0159 in the servo mesh; the horn disc is on the -z side), axis out of that face (+z), ref along the long (x) side. The matching seats on the printed parts are derived so that this port reproduces the URDF placement.",
        thickness_rationale="No published ear/lid thickness. The servo mesh shows a 1.5 mm slab between z = 0.0144 and 0.0159 spanning the case outline; estimated.",
    )
    # ---- child horn seats
    for jn in JOINTS:
        pid = CHILD_PART[jn]
        link, _, Tc = m.part_vis[pid]
        assert link == m.joints[jn]["child"], (jn, pid, link)
        Ts = m.servo_vis[jn][2]
        F = inv(Tc) @ inv(m.joints[jn]["T"]) @ Ts @ ports[SERVO]["output"]["P"] @ RX_PI
        ports.setdefault(pid, {})["horn_seat"] = mkport(
            F,
            type="servo-horn.25t",
            gender="female",
            symmetry=25,
            thickness=THK_PRINTED[0],
            fidelity="derived",
            derivation=f"tools/so101_ports.py from {SRC}: F = inv(T_part) @ inv(J_{jn}) @ T_servo_{jn} @ P_out @ Rx(pi); mating servo.output (k=0) reproduces the URDF; absorbs the {spread['per_joint_rot_deg'][jn]:.6f} deg rotation about z of joint {jn} relative to the canonical output frame",
            thickness_rationale=THK_PRINTED[1],
            joint=jn,
            joint_reverse=spread["reverse"][jn],
        )
    # ---- servo seats on holder parts: M = servo base frame in the link frame
    for jn in JOINTS:
        pid = HOLDER[jn]
        link, _, Tp = m.part_vis[pid]
        assert link == m.servo_vis[jn][0], (jn, pid)
        M = m.servo_vis[jn][2] @ ports[SERVO]["base"]["P"]
        ports.setdefault(pid, {})["servo_seat"] = mkport(
            inv(Tp) @ M @ RX_PI,
            type="bolt-pattern.feetech-sts3215-base",
            gender="neutral",
            symmetry=2,
            thickness=THK_PRINTED[0],
            fidelity="derived",
            derivation=f"tools/so101_ports.py from {SRC}: F = inv(T_part) @ T_servo_{jn} @ P_base @ Rx(pi); the servo holding part is chosen by mesh contact (tools/so101_ports.py --evidence)",
            thickness_rationale=THK_PRINTED[1],
            servo_joint=jn,
        )
    # ---- fixed mates between printed parts in the same link
    for a, pa, b, pb, typ in FIXED:
        la, _, Ta = m.part_vis[a]
        lb, _, Tb = m.part_vis[b]
        assert la == lb
        A = m.mesh_pts(m.part_vis[a][1], Ta)
        B = m.mesh_pts(m.part_vis[b][1], Tb)
        lo, hi = np.maximum(A.min(0), B.min(0)), np.minimum(A.max(0), B.max(0))
        ext = hi - lo
        # ext < 0 in one dimension = bboxes only touch/gap there: (lo+hi)/2 is then the gap mid-plane
        say(f"fixed mate {a} / {b}: bbox overlap extents (mm) {np.round(ext * 1e3, 3)}")
        i = int(np.argmin(np.abs(ext)))
        sign = 1.0 if (B.mean(0)[i] - A.mean(0)[i]) >= 0 else -1.0
        zax = np.zeros(3)
        zax[i] = sign
        xs = np.zeros(3)
        xs[(i + 1) % 3] = 1.0
        M = frame_from_axes((lo + hi) / 2, zax, xs)
        for pid, name, F in ((a, pa, inv(Ta) @ M), (b, pb, inv(Tb) @ M @ RX_PI)):
            ports.setdefault(pid, {})[name] = mkport(
                F,
                type=typ,
                gender="neutral",
                symmetry=1,
                thickness=THK_JOIN[0],
                fidelity="derived",
                derivation=f"tools/so101_ports.py from {SRC}: mate frame M at the centre of the bbox overlap of the two parts (mid-plane of the gap where the bboxes only touch), axis along the thinnest overlap dimension from {a} towards {b}; port = inv(T_part) @ M (a) or inv(T_part) @ M @ Rx(pi) (b)",
                thickness_rationale=THK_JOIN[1],
            )
    # ---- waveshare board: seat on the plate (hole rings found in the plate mesh), mount port on the board
    cen, dx, dy = plate_holes(m)
    ports["so101-waveshare-mounting-plate"]["board_seat"] = mkport(
        P_from([cen[0], cen[1], PLATE_FACE_Z], [0, 0, 1], [1, 0, 0]),
        type="bolt-pattern.37x28.m2.5",
        gender="neutral",
        symmetry=2,
        thickness=0.0016,
        fidelity="estimated",
        rationale=f"Hole pattern centre (x, y) derived from the four hole rings in the plate mesh (spacing {dx * 1e3:.3f} x {dy * 1e3:.3f} mm, centre ({cen[0] * 1e3:.4f}, {cen[1] * 1e3:.4f}) mm in the plate frame, matching the 37.00 x 28.00 mm board pattern; the mesh features there are 5 mm diameter circles, so M2.5 is the briefing's thread tag, not read from the mesh); the choice of the +z plate face (z = {PLATE_FACE_Z}) as the board face is an assumption, a central block rises to z = 0.0076 on that face and the URDF does not model the board.",
        thickness_rationale="Board PCB thickness not published; 1.6 mm is the common standard PCB thickness; estimated.",
    )
    ports[BOARD] = {
        "mount": mkport(
            P_from([0, 0, -BOARD_HALF_H], [0, 0, -1], [1, 0, 0]),
            type="bolt-pattern.37x28.m2.5",
            gender="neutral",
            symmetry=2,
            thickness=0.0016,
            fidelity="estimated",
            rationale="Board geometry is a box centred on the part frame; the mount pattern is taken on the underside (box bottom, assuming the tall connectors are on top), axis -z, ref along the 42 mm side.",
            thickness_rationale="PCB thickness not published; 1.6 mm standard; estimated.",
        )
    }
    return ports, spread


def plate_holes(m):
    link, mesh, T = m.part_vis["so101-waveshare-mounting-plate"]
    p = np.unique(
        np.round(read_stl(ROOT / "meshes/so101" / f"{mesh}.stl").reshape(-1, 3), 6),
        axis=0,
    )
    cs = []
    for cx in (-0.0185, 0.0185):
        for cy in (-0.014, 0.014):
            face = p[np.abs(p[:, 2] - PLATE_FACE_Z) < 1e-6][:, :2]
            # vertices on the r = 2.5 mm ring around the nominal hole position, then a least-squares circle fit
            ring = face[
                np.abs(np.hypot(face[:, 0] - cx, face[:, 1] - cy) - 0.0025) < 2e-4
            ]
            A = np.c_[2 * ring, np.ones(len(ring))]
            sol = np.linalg.lstsq(A, (ring**2).sum(1), rcond=None)[0]
            c = sol[:2]
            r = np.hypot(*(ring - c).T)
            say(
                f"plate hole near ({cx * 1e3:+.1f},{cy * 1e3:+.1f}) mm: centre ({c[0] * 1e3:.4f},{c[1] * 1e3:.4f}) mm, ring radius {r.min() * 1e3:.4f}..{r.max() * 1e3:.4f} mm, n={len(r)}"
            )
            cs.append(c)
    cs = np.array(cs)
    return cs.mean(0), cs[:, 0].max() - cs[:, 0].min(), cs[:, 1].max() - cs[:, 1].min()


MATES = []
for _i, _jn in enumerate(JOINTS):
    MATES.append((HOLDER[_jn], "servo_seat", f"servo:{_jn}", "base"))
    MATES.append((f"servo:{_jn}", "output", CHILD_PART[_jn], "horn_seat"))
for _a, _pa, _b, _pb, _t in FIXED:
    MATES.append((_a, _pa, _b, _pb))
MATES.append(("so101-waveshare-mounting-plate", "board_seat", "board", "mount"))


def compose(m, getP, getsym):
    """BFS over MATES from so101-base at its URDF placement; returns {instance: T_world}."""
    pid_of = {n: v[0] for n, v in m.expected.items()}
    pid_of["board"] = BOARD
    T = {"so101-base": m.expected["so101-base"][1]}
    todo = list(MATES)
    while todo:
        rest = []
        for a, pa, b, pb in todo:
            if a in T and b not in T:
                Pa, Pb = getP(pid_of[a], pa), getP(pid_of[b], pb)
                n = math.gcd(getsym(pid_of[a], pa), getsym(pid_of[b], pb))
                T[b] = T[a] @ Pa @ Rz(0.0 * 2 * math.pi / n) @ RX_PI @ inv(Pb)  # k = 0
            elif b in T and a not in T:
                raise SystemExit(
                    f"mate {a}.{pa} - {b}.{pb} reached from the b side; reorder MATES"
                )
            else:
                rest.append((a, pa, b, pb))
        if len(rest) == len(todo):
            raise SystemExit(f"unreachable mates: {rest}")
        todo = rest
    return T


def report(m, T, label):
    say(
        f"\n== closure {label}: max error vs URDF forward kinematics at q = 0 (visual origin included)"
    )
    say(f"{'instance':36s} {'position m':>12s} {'rotation rad':>13s}")
    worst = 0.0
    for n, (pid, Te) in m.expected.items():
        if n not in T:
            say(f"{n:36s} MISSING")
            worst = math.inf
            continue
        dp = float(np.abs(T[n][:3, 3] - Te[:3, 3]).max())
        dr = ang(T[n][:3, :3].T @ Te[:3, :3])
        worst = max(worst, dp, dr)
        say(f"{n:36s} {dp:12.3e} {dr:13.3e}")
    if "board" in T:
        say(
            f"{'board (no URDF reference)':36s} placed at {np.round(T['board'][:3, 3], 6)}"
        )
    say(
        f"-- worst over {len(m.expected)} instances: {worst:.3e}  ({'PASS' if worst <= TOL else 'FAIL'}, tol {TOL:g})"
    )
    return worst


def load_file_ports():
    d = {}
    for f in sorted((ROOT / "parts/arm").glob("*.toml")):
        doc = tomllib.loads(f.read_text())
        d[doc["id"]] = {
            p["name"]: p for p in doc.get("ports", []) if p.get("kind") == "mech"
        }
    return d


def toml_ports(pid, ports, order):
    out = []
    for name in order:
        p = ports[name]
        out.append("[[ports]]")
        out.append(f'name = "{name}"')
        out.append('kind = "mech"')
        out.append(f'type = "{p["type"]}"')
        out.append(f'gender = "{p["gender"]}"')
        out.append(f"pos = {p['pos']}")
        out.append(f"axis = {p['axis']}")
        out.append(f"ref = {p['ref']}")
        out.append(f"symmetry = {p['symmetry']}")
        out.append(f"thickness = {p['thickness']}")
        out.append(f'fidelity = "{p["fidelity"]}"')
        for k in ("derivation", "rationale", "thickness_rationale"):
            if k in p:
                out.append(f'{k} = "{p[k]}"')
        if "joint" in p:
            out.append(f'joint = "{p["joint"]}"')
            out.append(f"joint_reverse = {str(p['joint_reverse']).lower()}")
        if "servo_joint" in p:
            out.append(f'servo_joint = "{p["servo_joint"]}"')
        out.append("")
    return "\n".join(out)


def evidence(m):
    say("\n== evidence: horn side of the servo mesh")
    vs = {}
    for mesh in ("sts3215_03a_v1", "sts3215_03a_no_horn_v1"):
        v = read_stl(ROOT / "meshes/so101" / f"{mesh}.stl").reshape(-1, 3)
        vs[mesh] = v
        say(
            f"{mesh}: z max {v[:, 2].max():.4f} z min {v[:, 2].min():.4f}; z max within 3 mm of the output axis (x=0.0125,y=0): "
            f"{v[np.hypot(v[:, 0] - 0.0125, v[:, 1]) < 0.003][:, 2].max():.4f}"
        )
    key = lambda v: {tuple(r) for r in np.round(v, 6)}
    only = np.array(
        sorted(key(vs["sts3215_03a_v1"]) - key(vs["sts3215_03a_no_horn_v1"]))
    )
    say(
        f"vertices only in the horn mesh: n={len(only)} bbox min {only.min(0)} max {only.max(0)} "
        f"(horn extra geometry sits at +z of the servo mesh if z min > 0)"
    )
    say(
        "\n== evidence: horn-driven part = part in the child link with a vertex closest to the joint origin (link origin)"
    )
    for jn in JOINTS:
        link = m.joints[jn]["child"]
        row = []
        for s, T in m.links[link]:
            if s in PRINTED:
                d = np.linalg.norm(m.mesh_pts(s, T), axis=1).min()
                row.append(f"{PRINTED[s]} {d * 1e3:.2f} mm")
        say(
            f"{jn:14s} child link {link}: "
            + "; ".join(row)
            + f"   -> chosen {CHILD_PART[jn]}"
        )
    say(
        "\n== evidence: servo holder = part of the parent link whose surface is closest to the servo surface (random surface samples, seed 0)"
    )
    rng = np.random.default_rng(0)

    def surf(mesh, T, n):
        tri = read_stl(ROOT / "meshes/so101" / f"{mesh}.stl")
        a = np.linalg.norm(
            np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]), axis=1
        )
        i = rng.choice(len(tri), n, p=a / a.sum())
        u, v = rng.random((2, n))
        f = u + v > 1
        u[f], v[f] = 1 - u[f], 1 - v[f]
        t = tri[i]
        p = (
            t[:, 0]
            + u[:, None] * (t[:, 1] - t[:, 0])
            + v[:, None] * (t[:, 2] - t[:, 0])
        )
        return p @ T[:3, :3].T + T[:3, 3]

    for jn in JOINTS:
        link, smesh, sT = m.servo_vis[jn]
        sp = surf(smesh, sT, 2000)
        row = []
        for s, T in m.links[link]:
            if s in PRINTED:
                p = surf(s, T, 20000)
                d = np.concatenate(
                    [
                        np.sqrt(((sp[k : k + 100, None] - p[None]) ** 2).sum(-1)).min(1)
                        for k in range(0, len(sp), 100)
                    ]
                )
                row.append(
                    f"{PRINTED[s]} {np.mean(d < 5e-4):.3f}/{np.mean(d < 1e-3):.3f}"
                )
        say(
            f"{jn:14s} link {link}: fraction of servo surface within 0.5/1 mm of: "
            + "; ".join(row)
            + f"   -> chosen {HOLDER[jn]}"
        )


def main():
    args = sys.argv[1:]
    m = Model()
    say("== URDF joints (parsed from meshes/so101/so101_new_calib.urdf)")
    for jn, j in m.joints.items():
        T = j["T"]
        say(
            f"{jn:20s} {j['parent']:>16s} -> {j['child']:<26s} {j['type']:9s} xyz {np.round(T[:3, 3], 8)} lim {j['lower']} {j['upper']}"
        )
    if "--evidence" in args:
        evidence(m)
    ports, sp = derive(m)
    say(
        "\n== servo output frame agreement over the 6 joints (inv(T_servo_i) @ J_i, servo mesh frame)"
    )
    say(f"mean origin (m): {sp['origin_mean']}")
    say(f"origin spread (max |o_i - mean|): {sp['origin']:.3e} m")
    say(f"z-axis spread (max angle to mean axis): {sp['zaxis']:.3e} rad")
    say(
        f"horn disc side in the servo mesh: z {'-' if sp['horn_side'] < 0 else '+'} (outer face z = {sp['horn_face_z']}, thickness {sp['horn_thickness']:.4f} m); output axis = outward horn normal"
    )
    say(
        "rotation about z of each joint frame relative to the canonical output frame (deg, absorbed by the child horn seat): "
        + ", ".join(f"{k} {v:.6f}" for k, v in sp["per_joint_rot_deg"].items())
    )
    say(
        "reverse needed (joint +z = -output axis): "
        + ", ".join(f"{k} {v}" for k, v in sp["reverse"].items())
    )
    ke = 0.003695
    say(
        f"kv_equivalent = 60/(2*pi*ke) = {60 / (2 * math.pi * ke):.6g} rpm/V (ke = {ke} V*s/rad from the servo file)"
    )
    say("\n== joint table")
    for jn in JOINTS:
        j = m.joints[jn]
        say(
            f"{jn:14s} servo in {j['parent']:16s} held by {HOLDER[jn]:24s} horn seat on {CHILD_PART[jn]:24s} reverse={sp['reverse'][jn]} range [{j['lower']}, {j['upper']}] rad"
        )
        say(f"    {j['limit_text']}")
    # ---- closure from freshly derived ports
    worst = report(
        m,
        compose(
            m,
            lambda pid, n: ports[pid][n]["P"],
            lambda pid, n: ports[pid][n]["symmetry"],
        ),
        "derived ports",
    )
    ok = worst <= TOL
    if "--check" in args:
        fp = load_file_ports()
        need = {(a, pa) for a, pa, _, _ in MATES} | {(b, pb) for _, _, b, pb in MATES}
        pid_of = {n: v[0] for n, v in m.expected.items()}
        pid_of["board"] = BOARD
        miss = sorted(
            {(pid_of[n], pn) for n, pn in need if pn not in fp.get(pid_of[n], {})}
        )
        if miss:
            say(f"\nMISSING ports in parts/arm/*.toml: {miss}")
            ok = False
        else:

            def getP(pid, n):
                p = fp[pid][n]
                assert (
                    abs(np.dot(p["axis"], p["ref"])) < TOL
                    and abs(np.linalg.norm(p["axis"]) - 1) < TOL
                ), (pid, n)
                return P_from(p["pos"], p["axis"], p["ref"])

            wf = report(
                m,
                compose(m, getP, lambda pid, n: fp[pid][n]["symmetry"]),
                "ports read from parts/arm/*.toml",
            )
            ok = ok and wf <= TOL
            # numbers in files == tool output
            bad = [
                (pid, n)
                for pid, d in ports.items()
                for n, p in d.items()
                if n in fp.get(pid, {})
                and (
                    np.abs(np.array(fp[pid][n]["pos"]) - p["pos"]).max() > 1e-11
                    or np.abs(np.array(fp[pid][n]["axis"]) - p["axis"]).max() > 1e-11
                    or np.abs(np.array(fp[pid][n]["ref"]) - p["ref"]).max() > 1e-11
                )
            ]
            say(
                f"file ports differing from the tool output by > 1e-11: {bad if bad else 'none'}"
            )
            ok = ok and not bad
        # [geometry] origin_xyz/origin_rpy in the part files vs the URDF visual origins
        say(
            "\n== [geometry] placement in parts/arm/*.toml vs URDF visual origin (printed parts)"
        )
        worst_g = 0.0
        for pid, (link, mesh, T) in m.part_vis.items():
            g = tomllib.loads((ROOT / f"parts/arm/{pid}.toml").read_text())["geometry"]
            Tg = T_from(g["origin_xyz"], g["origin_rpy"])
            dp, dr = (
                float(np.abs(Tg[:3, 3] - T[:3, 3]).max()),
                ang(Tg[:3, :3].T @ T[:3, :3]),
            )
            worst_g = max(worst_g, dp, dr)
            say(
                f"{pid:36s} {dp:12.3e} m {dr:12.3e} rad  (urdf_link {g['urdf_link']} vs {link})"
            )
            ok = ok and g["urdf_link"] == link
        say(
            f"-- worst geometry placement error {worst_g:.3e} ({'PASS' if worst_g <= TOL else 'FAIL'}, tol {TOL:g})"
        )
        ok = ok and worst_g <= TOL
    else:
        say("\n== TOML snippets (paste with Edit)")
        for pid, d in ports.items():
            say(f"\n######## parts/arm/{pid}.toml\n" + toml_ports(pid, d, list(d)))
        say("\n== joint-limit params (child parts)")
        for jn in JOINTS:
            j = m.joints[jn]
            say(
                f'{CHILD_PART[jn]}: {jn}_range = {{ value = [{j["lower"]}, {j["upper"]}], unit = "rad", src = "S3", fidelity = "datasheet" }}   quote: {j["limit_text"]}'
            )
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
