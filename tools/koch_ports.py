#!/usr/bin/env python3
"""Derive [[ports]] for the Koch v1.1 follower arm (printed parts + Dynamixel servos) from the URDF and self-check them.

  python tools/koch_ports.py                     print derived ports (TOML), joint table, closure check
  python tools/koch_ports.py --servo             print [geometry] + mech [[ports]] blocks of the two servo parts
                                                 (robotis-xl430-w250-t, robotis-xl330-m288-t) for merging by hand
  python tools/koch_ports.py --write-servo-meshes   write meshes/koch/xl430_w250_t.stl and xl330_m288_t.stl (needs scipy):
                                                 convex hull of the servo bodies cut out of the *_motor.STL link meshes,
                                                 in the servo frame, metres (the cut-out bodies are not manifold)
  python tools/koch_ports.py --check             compose from the ports written in parts/arm/*.toml (servo ports from
                                                 the robotis-* files when present, else derived) and compare with the
                                                 URDF forward kinematics at q = 0 and at random q (exit 1 if error > 1e-9)

Source: meshes/koch/follower.urdf (s1lent4gnt/koch-v1-1 cc1a4eb, Koch v1.1 geometry), see meshes/koch/NOTICE.
Conventions as tools/so101_ports.py (port frame P = [x'=ref, y'=axis x ref, z'=axis | pos]; mate
T_b = T_a @ P_a @ Rz(2 pi k/n) @ Rx(pi) @ inv(P_b), k = 0). Helpers are reused from so101_ports / so101_lib.
Stdlib + numpy only. The tool never writes part files (only the servo meshes, on request).
"""

import math
import struct
import sys

from so101_lib import ROOT, RX_PI, T_from, inv, np, parse_urdf, read_stl
from so101_ports import P_from, ang, load_file_ports, mkport, say, toml_ports

TOL = 1e-9
SRC = "meshes/koch/follower.urdf (s1lent4gnt/koch-v1-1 cc1a4eb)"
MESH_DIR = ROOT / "meshes/koch"
URDF = MESH_DIR / "follower.urdf"
# (URDF joint, LeRobot joint name, servo type); LeRobot koch_follower.py: shoulder_pan/lift xl430-w250, rest xl330-m288
JOINTS = [
    ("joint_1", "shoulder_pan", "xl430"),
    ("joint_2", "shoulder_lift", "xl430"),
    ("joint_3", "elbow_flex", "xl330"),
    ("joint_4", "wrist_flex", "xl330"),
    ("joint_5", "wrist_roll", "xl330"),
    ("joint_6", "gripper", "xl330"),
]
SERVO = {"xl430": "robotis-xl430-w250-t", "xl330": "robotis-xl330-m288-t"}
SERVO_MESH = {"xl430": "xl430_w250_t", "xl330": "xl330_m288_t"}
PART = {  # URDF link -> printed part id (one printed part per link; mesh frame = link frame, visual origin = identity)
    "base_link": "koch-base",
    "link_1": "koch-shoulder-rotation",
    "link_2": "koch-shoulder-to-elbow",
    "link_3": "koch-elbow-to-wrist-extension",
    "link_4": "koch-elbow-to-wrist",
    "link_5": "koch-gripper-static",
    "link_6": "koch-gripper-moving",
}
THK_PRINTED = (
    0.003,
    "No published or measurable value (exported mesh, no screw-boss data); 3 mm placeholder (2 mm printed wall plus boss margin), the value used by the SO-101 parts. Estimated.",
)
THK_TABLE = (
    0.009,
    "Base plate thickness: the base mesh is 9 mm tall (bbox z 0 .. 0.009 m); the table clamp grips this plate. Derived from the mesh bounding box, not a published value.",
)


def components(tri, q=1e-7):
    """Per-triangle connected-component label (triangles sharing a vertex, vertices merged at q metres)."""
    v = np.round(tri.reshape(-1, 3) / q).astype(np.int64)
    _, idx = np.unique(v, axis=0, return_inverse=True)
    ids = idx.reshape(-1, 3)
    par = list(range(int(ids.max()) + 1))

    def find(a):
        while par[a] != a:
            par[a] = par[par[a]]
            a = par[a]
        return a

    for a, b, c in ids:
        ra, rb, rc = find(a), find(b), find(c)
        par[rb] = ra
        par[find(rc)] = ra
    root = np.array([find(a) for a in ids[:, 0]])
    return np.unique(root, return_inverse=True)[1].reshape(-1)


def vol(t):
    return float(np.einsum("ij,ij->i", t[:, 0], np.cross(t[:, 1], t[:, 2])).sum() / 6)


def cen(t):
    v = np.einsum("ij,ij->i", t[:, 0], np.cross(t[:, 1], t[:, 2])) / 6
    return ((t.sum(1) / 4) * v[:, None]).sum(0) / v.sum()


def hull_tris(tri):
    """Watertight, consistently oriented convex hull of a triangle soup (scipy/qhull, imported lazily: only the servo
    flags need it). The cut-out servo bodies are NOT manifold (touching shells: MuJoCo 'inconsistent orientation'),
    so the servo meshes shipped in meshes/koch are the hull of the body; the hull keeps the bounding box exactly."""
    from scipy.spatial import ConvexHull

    pts = tri.reshape(-1, 3)
    h = ConvexHull(pts)
    t = pts[h.simplices]
    flip = (
        np.einsum(
            "ij,ij->i",
            np.cross(t[:, 1] - t[:, 0], t[:, 2] - t[:, 0]),
            h.equations[:, :3],
        )
        < 0
    )
    t[flip] = t[flip][:, [0, 2, 1]]
    return t


def write_stl(path, tri):
    n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-30)
    dt = np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
    a = np.zeros(len(tri), dt)
    a["n"], a["v"] = n, tri
    with open(path, "wb") as f:
        f.write(b"Koch v1.1 servo body, tools/koch_ports.py".ljust(80, b" "))
        f.write(struct.pack("<I", len(tri)))
        f.write(a.tobytes())


def rot_axis(axis, a):
    """4x4 rotation by angle a about the unit vector axis through the origin (Rodrigues)."""
    x, y, z = axis
    K = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    T = np.eye(4)
    T[:3, :3] = np.eye(3) + math.sin(a) * K + (1 - math.cos(a)) * K @ K
    return T


def about(pos, axis, a):
    """Rotation by a about the line (pos, axis)."""
    T = np.eye(4)
    T[:3, 3] = pos
    U = np.eye(4)
    U[:3, 3] = -np.asarray(pos)
    return T @ rot_axis(axis, a) @ U


class Model:
    def __init__(self):
        self.links, self.joints = parse_urdf(URDF)
        assert all(len(v) >= 1 and v[0][0] == k for k, v in self.links.items()), (
            "link mesh is not the first visual"
        )
        for k, v in self.links.items():
            assert np.allclose(v[0][1], np.eye(4)), (
                f"{k}: visual origin is not identity"
            )
        self.fk = {"base_link": np.eye(4)}
        todo = list(self.joints.values())
        while todo:
            rest = [j for j in todo if j["parent"] not in self.fk]
            for j in todo:
                if j["parent"] in self.fk:
                    self.fk[j["child"]] = self.fk[j["parent"]] @ j["T"]
            assert len(rest) < len(todo), "disconnected joints"
            todo = rest
        self.servo = {}  # URDF joint -> dict(S = servo frame in the parent link frame, tri_link = servo body triangles, ...)
        for ujn, lj, typ in JOINTS:
            self.servo[ujn] = self.servo_in_link(ujn, typ)

    def fk_q(self, q):
        """URDF forward kinematics, {link: T_world} with joint angles q {urdf joint: rad}."""
        T = {"base_link": np.eye(4)}
        todo = list(self.joints.items())
        while todo:
            rest = []
            for n, j in todo:
                if j["parent"] in T:
                    ax = np.array(j["axis"], float)
                    T[j["child"]] = (
                        T[j["parent"]]
                        @ j["T"]
                        @ rot_axis(ax / np.linalg.norm(ax), q.get(n, 0.0))
                    )
                else:
                    rest.append((n, j))
            todo = rest
        return T

    def servo_in_link(self, ujn, typ):
        j = self.joints[ujn]
        link = j["parent"]
        tri = read_stl(MESH_DIR / f"{link}_motor.STL")
        lab = components(tri)
        vols = np.array([vol(tri[lab == k]) for k in range(lab.max() + 1)])
        main = int(np.argmax(vols))
        bb = lambda k: (
            tri[lab == k].reshape(-1, 3).min(0),
            tri[lab == k].reshape(-1, 3).max(0),
        )
        lo, hi = bb(main)
        pick = [main]
        if (
            typ == "xl330"
        ):  # the cover shell: a second large component inside the body's bounding box
            for k in range(lab.max() + 1):
                kl, kh = bb(k)
                if (
                    k != main
                    and vols[k] > 1e-6
                    and (kl >= lo - 1e-6).all()
                    and (kh <= hi + 1e-6).all()
                ):
                    pick.append(k)
        sel = tri[np.isin(lab, pick)]
        O = j["T"][:3, 3]
        zo = -(
            j["T"][:3, :3] @ np.array(j["axis"], float)
        )  # outward horn normal = -(joint axis): the horn drives the child
        zo /= np.linalg.norm(zo)
        P = sel.reshape(-1, 3)
        top = float(((P - O) @ zo).max())
        assert top <= 1e-6, (
            f"{ujn}: servo body sticks out beyond the joint plane by {top}"
        )
        d = (lo + hi) / 2 - O
        d -= zo * (d @ zo)
        i = int(np.argmax(np.abs(d)))
        xs = np.zeros(3)
        xs[i] = np.sign(d[i])
        rest = np.delete(np.abs(d), i).max()
        assert rest < 1e-3, f"{ujn}: body centre is {rest} off the long axis"
        S = P_from(O, zo, xs)
        return dict(
            S=S,
            link=link,
            tri_servo=((sel - O) @ S[:3, :3]),
            gap=-top,
            n_comp=len(pick),
            vol=vol(sel),
        )


def derive(m):
    """-> ports {part id: {port name: dict}}, servo meshes {type: triangles in servo frame}."""
    ports = {}
    meshes = {}
    for typ in SERVO:
        js = [u for u, _, t in JOINTS if t == typ]
        ref = m.servo[js[0]]
        for u in js[
            1:
        ]:  # all servos of one type must give the same body in the servo frame
            s = m.servo[u]
            dc = np.abs(cen(s["tri_servo"]) - cen(ref["tri_servo"])).max()
            db = max(
                np.abs(
                    s["tri_servo"].reshape(-1, 3).min(0)
                    - ref["tri_servo"].reshape(-1, 3).min(0)
                ).max(),
                np.abs(
                    s["tri_servo"].reshape(-1, 3).max(0)
                    - ref["tri_servo"].reshape(-1, 3).max(0)
                ).max(),
            )
            dv = abs(s["vol"] - ref["vol"]) / ref["vol"]
            say(
                f"servo {typ}: {u} vs {js[0]}: bbox diff {db:.2e} m, centroid diff {dc:.2e} m, volume rel diff {dv:.2e}, output-plane gap {s['gap'] * 1e3:.3f} mm"
            )
            # volume tolerance 5 %: the two XL430 shells differ in ~3 % internal detail (3897 vs 3877 triangles), outline identical
            # position tolerance 0.5 mm: the CAD joint-5 servo sits ~0.1 mm / 0.4 mm (centroid) off the others relative to its joint origin
            assert db < 5e-4 and dc < 5e-4 and dv < 0.05, (
                f"{typ}: {u} body differs from {js[0]} in the servo frame"
            )
        tri = ref["tri_servo"]
        meshes[typ] = tri
        lo, hi = tri.reshape(-1, 3).min(0), tri.reshape(-1, 3).max(0)
        sid = SERVO[typ]
        gap = ref["gap"]
        der = f"tools/koch_ports.py from {SRC}: servo frame S = [origin = joint origin (on the horn axis, at the outer face of the horn hub), z = outward horn normal = -(joint axis), x = in-plane direction from the horn axis to the body centre (the body's long axis)]; the body is the largest component of the link's *_motor.STL (plus the cover shell for XL330) expressed in S; this port is S itself"
        ports[sid] = {
            "output": mkport(
                P_from([0, 0, 0], [0, 0, 1], [1, 0, 0]),
                type=f"servo-horn.dynamixel-{typ}",
                gender="male",
                symmetry=1,
                thickness=0.003,
                fidelity="derived",
                derivation=der,
                thickness_rationale=f"Horn hub height beyond the body face read from the mesh: {gap * 1e3:.1f} mm between the body's output face and the joint (horn outer face) plane for XL330"
                + (
                    "; for XL430 the hub is part of the body mesh (height 37.9 mm vs the 34 mm datasheet height, 3.9 mm difference), estimated 3 mm clamped."
                    if typ == "xl430"
                    else ", measured on all four servos."
                ),
            ),
            "base": mkport(
                P_from(
                    [(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, lo[2]],
                    [0, 0, -1],
                    [1, 0, 0],
                ),
                type=f"bolt-pattern.dynamixel-{typ}-base",
                gender="neutral",
                symmetry=2,
                thickness=0.003,
                fidelity="estimated",
                rationale=f"Frame convention, not a measured hole pattern (same approach as the SO-101 servo base port): centre of the body face opposite the output (z = {lo[2]:.6f} in the servo frame, centre x {(lo[0] + hi[0]) / 2:.6f}, y {(lo[1] + hi[1]) / 2:.6f}), axis out of that face (-z), ref along the long (x) side. The Koch parts hold the servos in pockets by several faces; the printed seats are derived so that this port reproduces the URDF placement.",
                thickness_rationale="No published wall / ear thickness; 3 mm placeholder. Estimated.",
            ),
        }
    # ---- printed parts
    for ujn, lj, typ in JOINTS:
        j, s = m.joints[ujn], m.servo[ujn]
        sid = SERVO[typ]
        child, parent = j["child"], j["parent"]
        F = (
            inv(j["T"]) @ s["S"] @ RX_PI
        )  # child part frame = child link frame; servo frame S in the parent link frame
        ports.setdefault(PART[child], {})["horn_seat"] = mkport(
            F,
            type=f"servo-horn.dynamixel-{typ}",
            gender="female",
            symmetry=1,
            thickness=THK_PRINTED[0],
            fidelity="derived",
            derivation=f"tools/koch_ports.py from {SRC}: F = inv(J_{ujn}) @ S_{ujn} @ Rx(pi) with J the URDF joint frame ({ujn} = {lj}) and S the servo frame in the parent link; mating {sid}.output (k = 0) reproduces the URDF. The URDF joint axis is the INVERSE of the output axis, so the mate needs reverse = true to get the URDF sign",
            thickness_rationale=THK_PRINTED[1],
            joint=lj,
            joint_reverse=bool(
                float((j["T"][:3, :3] @ np.array(j["axis"], float)) @ s["S"][:3, 2]) < 0
            ),
        )
        ports.setdefault(PART[parent], {})["servo_seat"] = mkport(
            s["S"]
            @ P_from(
                ports[sid]["base"]["pos"],
                ports[sid]["base"]["axis"],
                ports[sid]["base"]["ref"],
            )
            @ RX_PI,
            type=f"bolt-pattern.dynamixel-{typ}-base",
            gender="neutral",
            symmetry=2,
            thickness=THK_PRINTED[0],
            fidelity="derived",
            derivation=f"tools/koch_ports.py from {SRC}: F = S_{ujn} @ P_base @ Rx(pi) (printed part frame = parent link frame); the holder is the printed part of the link whose *_motor mesh contains the servo",
            thickness_rationale=THK_PRINTED[1],
            servo_joint=lj,
        )
    # ---- base to table: the underside of the base plate (the arm is clamped to the table with a C-clamp)
    tri = read_stl(MESH_DIR / "base_link.STL").reshape(-1, 3)
    low = tri[np.abs(tri[:, 2] - tri[:, 2].min()) < 1e-6]
    c = (low.min(0) + low.max(0)) / 2
    ports[PART["base_link"]]["table_seat"] = mkport(
        P_from([c[0], c[1], tri[:, 2].min()], [0, 0, -1], [0, 1, 0]),
        type="flat-mount",
        gender="neutral",
        symmetry=1,
        thickness=THK_TABLE[0],
        fidelity="derived",
        derivation=f"tools/koch_ports.py from {SRC}: centre of the bounding box of the base mesh vertices on the lowest plane (z = {tri[:, 2].min():.6f} m, the underside of the plate), axis -z (out of the underside), ref along +y (the long side of the plate, 158.5 mm). Placement of the clamp along the plate is not specified upstream",
        thickness_rationale=THK_TABLE[1],
    )
    return ports, meshes


def pid_of(inst):
    return SERVO[inst.split(":")[2]] if inst.startswith("servo:") else inst


def mates(m):
    """[(a, port a, b, port b, urdf joint or None)] in tree order from koch-base."""
    out = []
    for ujn, lj, typ in JOINTS:
        j = m.joints[ujn]
        out.append((PART[j["parent"]], "servo_seat", f"servo:{lj}:{typ}", "base", None))
        out.append((f"servo:{lj}:{typ}", "output", PART[j["child"]], "horn_seat", ujn))
    return out


def compose(m, getP, getsym, q):
    """BFS over mates(m) from koch-base at the URDF base_link placement (identity); hinge angles q {urdf joint: rad}."""
    T = {"koch-base": np.eye(4)}
    todo = mates(m)
    while todo:
        rest = []
        for a, pa, b, pb, ujn in todo:
            if a in T and b not in T:
                Pa, Pb = getP(pid_of(a), pa), getP(pid_of(b), pb)
                n = math.gcd(getsym(pid_of(a), pa), getsym(pid_of(b), pb))
                Tb = (
                    T[a] @ Pa @ RX_PI @ inv(Pb)
                )  # k = 0 (n kept for the gcd bound only)
                assert n >= 1
                if (
                    ujn is not None
                ):  # hinge in b's body at b's port origin, axis = +b.axis (reverse = true), angle q
                    Tb = Tb @ about(Pb[:3, 3], Pb[:3, 2], q.get(ujn, 0.0))
                T[b] = Tb
            else:
                rest.append((a, pa, b, pb, ujn))
        if len(rest) == len(todo):
            raise SystemExit(f"unreachable mates: {rest}")
        todo = rest
    return T


def report(m, T, q, label):
    Tu = m.fk_q(q)
    say(
        f"\n== closure {label}: max error vs URDF forward kinematics, q = {{{', '.join(f'{k}: {v:.3f}' for k, v in q.items())}}} rad"
        if q
        else f"\n== closure {label}: max error vs URDF forward kinematics at q = 0"
    )
    say(f"{'instance':44s} {'position m':>12s} {'rotation rad':>13s}")
    worst_p = worst_r = 0.0
    exp = {PART[ln]: Tu[ln] for ln in PART}
    for ujn, lj, typ in JOINTS:
        exp[f"servo:{lj}:{typ}"] = Tu[m.joints[ujn]["parent"]] @ m.servo[ujn]["S"]
    for n, Te in exp.items():
        if n not in T:
            say(f"{n:44s} MISSING")
            return math.inf, math.inf
        dp = float(np.abs(T[n][:3, 3] - Te[:3, 3]).max())
        dr = ang(T[n][:3, :3].T @ Te[:3, :3])
        worst_p, worst_r = max(worst_p, dp), max(worst_r, dr)
        say(f"{n:44s} {dp:12.3e} {dr:13.3e}")
    say(
        f"-- worst over {len(exp)} instances: position {worst_p:.3e} m, rotation/axis {worst_r:.3e} rad ({'PASS' if max(worst_p, worst_r) <= TOL else 'FAIL'}, tol {TOL:g})"
    )
    return worst_p, worst_r


def servo_blocks(ports, meshes):
    for typ, sid in SERVO.items():
        tri = hull_tris(meshes[typ])
        lo, hi = tri.reshape(-1, 3).min(0), tri.reshape(-1, 3).max(0)
        say(
            f"\n######## parts/arm/{sid}.toml  (geometry + mechanical ports only; merge by hand)"
        )
        say(
            f'[geometry]\nshape = "mesh"\nmesh = "meshes/koch/{SERVO_MESH[typ]}.stl"\n'
            f"bbox_min = {[round(float(x), 6) for x in lo]}\nbbox_max = {[round(float(x), 6) for x in hi]}\n"
            f"# units metres; solid mesh volume {vol(tri):.4e} m^3. Servo frame (tools/koch_ports.py): origin on the horn axis at the outer face of the horn hub,\n"
            f"# z = outward horn normal, x = along the body's long axis towards the body centre, y = z x x. The body is cut out of the Koch v1.1\n"
            f"# *_motor.STL link meshes (Apache-2.0, meshes/koch/NOTICE) and replaced by its convex hull ({len(tri)} triangles; the cut-out body is not\n"
            f"# manifold, MuJoCo rejects it with inertia=exact); hub/horn pieces of the XL330 and screws/idler frames are not included.\n"
        )
        say(toml_ports(sid, ports[sid], ["output", "base"]))


def main():
    args = sys.argv[1:]
    m = Model()
    say(f"== URDF joints (parsed from {SRC})")
    for jn, j in m.joints.items():
        say(
            f"{jn:8s} {j['parent']:>10s} -> {j['child']:<8s} {j['type']:9s} xyz {np.round(j['T'][:3, 3], 8)} axis {j['axis']} lim {j['lower']} {j['upper']}"
        )
    say("\n== servo bodies in the link frame (from the *_motor.STL meshes)")
    for ujn, lj, typ in JOINTS:
        s = m.servo[ujn]
        t = s["tri_servo"].reshape(-1, 3)
        say(
            f"{ujn} {lj:14s} {typ} in {s['link']:7s}: S origin {np.round(s['S'][:3, 3], 6)} z {np.round(s['S'][:3, 2], 3)} x {np.round(s['S'][:3, 0], 3)}; body size mm {np.round((t.max(0) - t.min(0)) * 1e3, 2)}, {s['n_comp']} component(s), body-to-horn-plane gap {s['gap'] * 1e3:.3f} mm"
        )
    ports, meshes = derive(m)
    if "--write-servo-meshes" in args:
        for typ, raw in meshes.items():
            tri = hull_tris(raw)
            write_stl(MESH_DIR / f"{SERVO_MESH[typ]}.stl", tri)
            say(f"wrote meshes/koch/{SERVO_MESH[typ]}.stl ({len(tri)} triangles)")
    if "--servo" in args:
        servo_blocks(ports, meshes)
        return 0
    rng = np.random.default_rng(0)
    qs = [{}] + [
        {u: float(rng.uniform(-1.5, 1.5)) for u, _, _ in JOINTS} for _ in range(3)
    ]
    worst = 0.0
    for q in qs:
        wp, wr = report(
            m,
            compose(
                m,
                lambda p, n: P_from(
                    ports[p][n]["pos"], ports[p][n]["axis"], ports[p][n]["ref"]
                ),
                lambda p, n: ports[p][n]["symmetry"],
                q,
            ),
            q,
            "derived ports",
        )
        worst = max(worst, wp, wr)
    ok = worst <= TOL
    if "--check" in args:
        fp = load_file_ports()
        for sid in SERVO.values():
            if "output" not in fp.get(sid, {}) or "base" not in fp.get(sid, {}):
                say(
                    f"\nNOTE: {sid} has no output/base mech ports in parts/arm yet (S1's file, merged later): using the derived servo ports"
                )
                fp[sid] = {n: dict(p, P=None) for n, p in ports[sid].items()}
        need = (
            [(pid_of(a), pa) for a, pa, _, _, _ in mates(m)]
            + [(pid_of(b), pb) for _, _, b, pb, _ in mates(m)]
            + [(PART["base_link"], "table_seat")]
        )
        miss = sorted({x for x in need if x[1] not in fp.get(x[0], {})})
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

            wfm = 0.0
            for q in qs:
                wp, wr = report(
                    m,
                    compose(m, getP, lambda pid, n: fp[pid][n]["symmetry"], q),
                    q,
                    "ports read from parts/arm/*.toml",
                )
                wfm = max(wfm, wp, wr)
            ok = ok and wfm <= TOL
            bad = [
                (pid, n)
                for pid, d in ports.items()
                for n, p in d.items()
                if n in fp.get(pid, {})
                and fp[pid][n].get("P", 1) is not None
                and max(
                    np.abs(np.array(fp[pid][n][k]) - p[k]).max()
                    for k in ("pos", "axis", "ref")
                )
                > 1e-11
            ]
            say(
                f"file ports differing from the tool output by > 1e-11: {bad if bad else 'none'}"
            )
            ok = ok and not bad
        import tomllib

        say(
            "\n== [geometry] in parts/arm/koch-*.toml vs URDF (printed parts: mesh = meshes/koch/<link>.STL, origin identity)"
        )
        for ln, pid in PART.items():
            g = tomllib.loads((ROOT / f"parts/arm/{pid}.toml").read_text())["geometry"]
            Tg = T_from(g["origin_xyz"], g["origin_rpy"])
            good = (
                g["urdf_link"] == ln
                and g["mesh"] == f"meshes/koch/{ln}.STL"
                and np.abs(Tg - np.eye(4)).max() < TOL
            )
            say(
                f"{pid:32s} urdf_link {g['urdf_link']:8s} mesh {g['mesh']:28s} {'ok' if good else 'MISMATCH'}"
            )
            ok = ok and good
    else:
        say("\n== TOML snippets (paste with Edit)")
        for pid, d in ports.items():
            if pid in SERVO.values():
                continue
            say(f"\n######## parts/arm/{pid}.toml\n" + toml_ports(pid, d, list(d)))
        say("(servo geometry + ports: --servo)")
        say(
            "\n== joint table (URDF joint, name, axis in the parent link frame, URDF limits [rad], servo)"
        )
        for ujn, lj, typ in JOINTS:
            j = m.joints[ujn]
            say(
                f"{ujn} {lj:14s} axis {j['axis']} limits [{j['lower']}, {j['upper']}] {SERVO[typ]} horn seat on {PART[j['child']]}, servo held by {PART[j['parent']]}"
            )
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
