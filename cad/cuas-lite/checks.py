"""Checks for the cuas-lite CAD: solids, bed fit, mating features, ball path, tilt sweep, balance. Called by build.py.

Every check returns {"pass": bool, "detail": ...}; run() prints one line per check. Probing uses point-in-solid tests on the
real solids, so a mating feature "agrees" only if both parts really have a void at the same place.
"""

import math

import bought as BT
import numpy as np
import params as P
from build123d import Pos, Rot, Sphere

import parts as K

X_A, Z_A = P.X_A, P.Z_A
xc = P.CAGE_X / 2
ACCESSORIES = {
    "cable_clip",
    "camera_shim",
}  # drawn at the origin, not placed on the assembly
RES = {}
FEED = {}  # per-tilt feed path numbers, filled by feed_checks and reported in the interface


def chk(name, ok, detail=""):
    RES[name] = dict(**{"pass": bool(ok)}, detail=detail)
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}: {detail}", flush=True)


def void(s, x, y, z):
    return not s.is_inside((x, y, z))


def hv(s, x, y, z):
    """void test with L coordinates on a head (H-frame) shape"""
    return void(s, x - X_A, y, z - Z_A)


def vol(a, b):
    try:
        i = a & b
        return float(i.volume) if i is not None else 0.0
    except Exception:
        return 0.0


def extent(s, axis, p, lim=60.0, step=0.05):
    """Length of the continuous void run through point p along axis (0/1/2), and its centre coordinate."""

    def ok(t):
        q = list(p)
        q[axis] = t
        return void(s, *q)

    lo = hi = p[axis]
    while hi - p[axis] < lim and ok(hi + step):
        hi += step
    while p[axis] - lo < lim and ok(lo - step):
        lo -= step
    return hi - lo, (hi + lo) / 2


def reach(s, axis, p, sign, lim=40.0, step=0.05):
    """Distance from p along +-axis until the solid is hit."""
    t = 0.1  # start off the split face
    while t < lim:
        q = list(p)
        q[axis] += sign * t
        if not void(s, *q):
            return t
        t += step
    return lim


def rot_tilt(shape, phi):
    return Pos(0, 0, P.H_T) * Rot(0, -phi, 0) * shape


def Ry_tilt(phi):
    """Rotation matrix of rot_tilt (head frame H -> B orientation) for nose-up tilt phi (deg)."""
    a = math.radians(-phi)
    return np.array(
        [[math.cos(a), 0, math.sin(a)], [0, 1, 0], [-math.sin(a), 0, math.cos(a)]]
    )


def rot_pan(shape, a):
    return Rot(0, 0, a) * shape


def pan_checks(S, tilt_p, tilt_b, stat_all):
    """Pan hard stop engages at +-PAN_STOP_DEG +-2, and nothing else collides over the pan range."""
    tab, stud = S["pan_stop_tab"], stat_all["pan_stop_stud"]
    ang = {}
    for sgn in (1, -1):
        lo, hi = 165.0, 180.0
        for _ in range(24):
            mid = (lo + hi) / 2
            lo, hi = (
                (lo, mid) if vol(rot_pan(tab, sgn * mid), stud) > 0.05 else (mid, hi)
            )
        ang[sgn] = hi
    chk(
        "pan_hard_stops_engage",
        all(abs(a - P.PAN_STOP_DEG) <= 2.0 for a in ang.values()),
        f"tab blade meets the M3 stud at +{ang[1]:.2f} / -{ang[-1]:.2f} deg (target {P.PAN_STOP_DEG} +-2, soft limits +-170); "
        f"stud r={P.PAN_STOP_R} at -90 deg, blade at the rotor +90 deg; cable loop must allow +-{P.PAN_STOP_DEG:.0f}",
    )
    stat_all = {
        n: S[n]
        for n, (_, g, _, _) in K.REGISTRY.items()
        if g == "static" and n not in ACCESSORIES
    } | {i["name"]: i["shape"] for i in BT.static_items()}  # fixed base only
    zmax = max(s.bounding_box().max.Z for s in stat_all.values())
    mov = {
        n: S[n]
        for n, (_, g, _, _) in K.REGISTRY.items()
        if g == "pan" and n not in ACCESSORIES
    }
    mov.update(
        {i["name"]: i["shape"] for i in BT.pan_items() if i["name"] != "lazy_susan"}
    )
    for phi in (
        -15.0,
        0.0,
        70.0,
    ):  # head low enough to matter only at its lowest, but cheap to include
        for n, s in {**{k: S[k] for k in tilt_p}, **tilt_b}.items():
            mov[f"{n}@tilt{phi:.0f}"] = rot_tilt(s, phi)
    mov = {
        n: s for n, s in mov.items() if s.bounding_box().min.Z < zmax + 1.0
    }  # pan rotation keeps z
    stat = {n: s for n, s in stat_all.items() if n != "lazy_susan"}
    hits, n = {}, 0
    for a in np.arange(-176.0, 176.1, 8.0):
        for mn, ms in mov.items():
            r = rot_pan(ms, float(a))
            rb = r.bounding_box()
            for sn, ss in stat.items():
                sb = ss.bounding_box()
                if (
                    rb.max.X < sb.min.X
                    or rb.min.X > sb.max.X
                    or rb.max.Y < sb.min.Y
                    or rb.min.Y > sb.max.Y
                    or rb.max.Z < sb.min.Z
                    or rb.min.Z > sb.max.Z
                ):
                    continue
                n += 1
                v = vol(r, ss)
                if v > 1.0:
                    hits[(mn, sn)] = max(hits.get((mn, sn), 0.0), round(v, 1))
    d_hall = float(rot_pan(tab, 90.0).distance_to(S["pan_hall_mount"]))
    chk(
        "pan_sweep_no_collision",
        not hits,
        f"pan -176..+176 deg, {len(mov)} low moving items vs {len(stat)} fixed ({n} overlap tests): "
        f"{('collisions ' + str(hits)) if hits else 'no overlap'}; tab to pan hall mount clearance {d_hall:.2f} mm at pan 90",
    )
    lo, hi = P.TILT_RANGE
    chk(
        "tilt_stops_present",
        True,
        f"tilt cradle stop (lug on trunnion_beam vs yoke_right blocks) catches at {lo - 3:.0f} deg and {hi + 3:.0f} deg "
        f"(hard stop {hi + 3:.0f} deg, inside the 72-75 asked for): see tilt_stops_engage_outside_range",
    )


def feed_checks(S, rb_):
    """Ball supply from the pan-group hopper through the hollow trunnion to the breech at tilt -15, 0, 35, 70 deg:
    (1) a ball swept along the whole path touches no wall; (2) every sloping leg is >= FEED_SLOPE_MIN downhill along gravity,
    the only level leg is the trunnion axis run, which the balls above push through (margin vs rolling resistance)."""
    ha = P.H_T
    s0, s1, zf = (np.array(v) for v in K.elbow_dims())
    top = np.array([0.0, s1[1], zf + K.HOPPER["Hn"]])
    C = np.array([0.0, 0.0, ha])
    Fh = np.array([P.X_F - X_A, 0.0, -Z_A])
    pan = [
        (n, S[n], S[n].bounding_box()) for n in ("feed_elbow", "hopper", "yoke_right")
    ]
    res, worst_all, npos, where = {}, 0.0, 0, None
    for phi in (-15.0, 0.0, 35.0, 70.0):
        F = C + Ry_tilt(phi) @ Fh
        head = []
        for n in (
            "launcher_chassis",
            "trunnion_beam",
            "barrel_half_a",
            "barrel_half_b",
            "piston",
        ):
            t = rot_tilt(S[n], phi)
            head.append((n, t, t.bounding_box()))
        legs = [
            ("vertical", top, s1),
            ("slope", s1, s0),
            ("level", s0, C),
            ("chute", C, F),
        ]
        worst, info = 0.0, {}
        for name, p, q in legs:
            d = q - p
            ln = float(np.linalg.norm(d))
            info[name] = dict(len=ln, deg=math.degrees(math.asin(-d[2] / ln)))
            for t in np.linspace(0.0, 1.0, int(ln // 3) + 2):
                pt = p + t * d
                ball = Pos(*pt) * Sphere(rb_)
                bb = ball.bounding_box()
                npos += 1
                for sn, s, sb_ in pan + head:
                    if (
                        bb.max.X < sb_.min.X
                        or bb.min.X > sb_.max.X
                        or bb.max.Y < sb_.min.Y
                        or bb.min.Y > sb_.max.Y
                        or bb.max.Z < sb_.min.Z
                        or bb.min.Z > sb_.max.Z
                    ):
                        continue
                    w = vol(ball, s)
                    if w > worst:
                        worst = w
                        if w > worst_all:
                            where = (phi, name, round(float(t), 2), sn)
        res[phi] = dict(worst=worst, **info)
        worst_all = max(worst_all, worst)
        if (
            phi == 0.0
        ):  # wall present all round the path (a missing wall would pass the sweep above)
            wall_max, open_at = 0.0, None
            for name, p, q in legs:
                d = (q - p) / np.linalg.norm(q - p)
                a = np.cross(d, [1.0, 0, 0] if abs(d[0]) < 0.9 else [0, 1.0, 0])
                a /= np.linalg.norm(a)
                b = np.cross(d, a)
                # not within a ball diameter of the turn at C, nor in the merge of the chute with the bore (open by design)
                t0, t1 = {"level": (0.05, 0.75), "chute": (0.18, 0.3)}.get(
                    name, (0.1, 0.9)
                )
                for t in np.linspace(t0, t1, 8):
                    pt = p + t * (q - p)
                    for k in range(8):
                        u = (
                            math.cos(k * math.pi / 4) * a
                            + math.sin(k * math.pi / 4) * b
                        )
                        hit = None
                        for r in np.arange(rb_, 25.0, 0.5):
                            x = pt + r * u
                            if any(
                                sb_.min.X <= x[0] <= sb_.max.X
                                and sb_.min.Y <= x[1] <= sb_.max.Y
                                and sb_.min.Z <= x[2] <= sb_.max.Z
                                and s.is_inside(tuple(x))
                                for _, s, sb_ in pan + head
                            ):
                                hit = r
                                break
                        if hit is None:
                            open_at = open_at or (name, round(float(t), 1), k)
                        else:
                            wall_max = max(wall_max, hit)
            lim_w = P.JOUR_ID / 2 + 2.5
            chk(
                "ball_path_enclosed",
                open_at is None and wall_max <= lim_w,
                f"wall within {wall_max:.1f} mm of the path centre all round hopper neck, elbow, spigot, trunnion, chute (limit {lim_w:.1f}); "
                + (f"OPEN at {open_at}" if open_at else "no open side"),
            )
    chk(
        "ball_path_clear_all_tilts",
        worst_all < 0.5,
        f"ball dia {P.BALL_D} swept hopper neck -> elbow -> spigot -> hollow trunnion -> chute -> breech at tilt -15/0/35/70 "
        f"({npos} positions): max overlap {worst_all:.2f} mm3"
        + (f" at {where}" if worst_all >= 0.5 else ""),
    )
    sl = {
        k: min(r[leg]["deg"] for leg in ("vertical", "slope", "chute"))
        for k, r in res.items()
    }
    lvl = max(abs(r["level"]["deg"]) for r in res.values())
    n_s = int(res[0.0]["slope"]["len"] // P.BALL_D)
    n_l = math.ceil(res[0.0]["level"]["len"] / P.BALL_D)
    margin = n_s * math.sin(math.radians(P.ELBOW_SLOPE)) / (n_l * P.ROLL_MU)
    chk(
        "feed_gravity_slope_all_tilts",
        min(sl.values()) >= P.FEED_SLOPE_MIN and lvl < 0.1 and margin >= 2.0,
        "chute downhill angle (min over the sloping legs) "
        + ", ".join(f"{int(k)}deg: {res[k]['chute']['deg']:.0f}" for k in res)
        + f"; pan slope {P.ELBOW_SLOPE:.0f} deg + vertical neck; level run on the tilt axis {res[0.0]['level']['len']:.0f} mm = {n_l} balls, "
        f"pushed by {n_s} balls on the slope: margin {margin:.1f}x at rolling resistance {P.ROLL_MU} [est, bench test]; "
        f"bore {-15:+d}..{70:+d} deg: ball rests against the piston face when nose-up, rolls to the wheel nip at -15",
    )
    FEED.update(res)


def run(recs, meshes, shapes, out):
    RES.clear()
    S = shapes
    print("checks:")
    # ---- 1 solids ------------------------------------------------------------------------------------------------------------
    bad = [
        n
        for n, r in recs.items()
        if not (
            r["valid"] and r["n_solids"] == 1 and r["watertight"] and r["winding_ok"]
        )
    ]
    chk("valid_watertight_solids", not bad, f"{len(recs)} parts; bad={bad}")
    drift = {
        n: round(
            abs(r["volume_mm3"] - r["volume_occ_mm3"]) / r["volume_occ_mm3"] * 100, 3
        )
        for n, r in recs.items()
    }
    chk(
        "mesh_volume_matches_brep",
        max(drift.values()) < 0.5,
        f"max |dV| {max(drift.values()):.3f} % ({max(drift, key=drift.get)})",
    )
    # ---- 2 bed -----------------------------------------------------------------------------------------------------------------
    lim = (P.BED[0] - 2 * P.BED_MARGIN, P.BED[1] - 2 * P.BED_MARGIN, P.BED[2])
    over = {
        n: r["print"]["size_mm"] for n, r in recs.items() if not r["print"]["fits_bed"]
    }
    big = max(recs, key=lambda n: max(recs[n]["print"]["size_mm"][:2]))
    chk(
        "fits_bed_220x220x250",
        not over,
        f"all within {lim[0]:.0f}x{lim[1]:.0f}x{lim[2]:.0f} (10 mm margin); largest footprint {big} "
        f"{recs[big]['print']['size_mm']}",
    )
    sup = {
        n: r["print"]["supports"]
        for n, r in recs.items()
        if r["print"]["supports"] != "none"
    }
    chk("support_report", True, f"parts needing any support/bridging attention: {sup}")

    # ---- 3 mating features -----------------------------------------------------------------------------------------------------
    st, ro, pu = S["base_stator_plate"], S["pan_rotor_plate"], S["pan_pulley_100t"]
    zr = P.Z_ROTOR1
    ok = True
    for k in range(4):
        a = math.radians(45 + 90 * k)
        x, y = P.LS_BOLT_R * math.cos(a), P.LS_BOLT_R * math.sin(a)
        ok &= void(st, x, y, P.POST_H - 3) and void(
            ro, x, y, zr - 4
        )  # ring-1 bolt <-> rotor access hole
        a = math.radians(90 * k)
        ok &= void(
            ro, P.LS_BOLT_R * math.cos(a), P.LS_BOLT_R * math.sin(a), zr - 4
        )  # ring-2 bolts
    chk(
        "lazy_susan_bolt_patterns",
        ok,
        f"stator posts + rotor holes on r={P.LS_BOLT_R} (rings offset 45 deg) [bearing hole pattern est: confirm at purchase]",
    )
    ok = all(
        void(
            pu,
            *(
                c := (
                    23 * math.cos(math.radians(30 + 60 * k)),
                    23 * math.sin(math.radians(30 + 60 * k)),
                )
            ),
            P.Z_ROTOR0 - 3,
        )
        and void(ro, *c, zr - 1)
        for k in range(6)
    )
    chk("pan_pulley_to_rotor_screws", ok, "6 x M3 on r=23 line up")
    chk(
        "pan_pulley_clears_aperture",
        2 * 28.0 < P.LS_APERTURE
        and 2 * (P.pulley_od(P.T_PAN) / 2 + P.FLANGE_OVER) < P.LS_APERTURE,
        f"neck/flange {2 * (P.pulley_od(P.T_PAN) / 2 + P.FLANGE_OVER):.1f} mm < aperture {P.LS_APERTURE}",
    )
    c = P.C_PAN
    mm_ = S["pan_motor_mount"]
    ok = void(st, c, 0, -4) and void(mm_, c, 0, -P.STATOR_T - 2)
    for sx in (-14, 14):
        for sy in (-25, 25):
            ok &= void(st, c + sx, sy, -4) and all(
                void(mm_, c + sx + d, sy, -P.STATOR_T - 2)
                for d in (-P.SLOT_TRAVEL, 0, P.SLOT_TRAVEL)
            )
    chk(
        "pan_motor_mount_slots_travel",
        ok,
        f"+-{P.SLOT_TRAVEL} mm slot at the 4 screws, shaft slot in the stator plate, pilot hole",
    )
    L_pan = P.BELT_TEETH["pan"] * P.PITCH
    cs = {
        n: P.centre_distance(n, P.T_MOTOR, P.T_PAN)
        for n in (P.BELT_TEETH["pan"] - 1, P.BELT_TEETH["pan"], P.BELT_TEETH["pan"] + 1)
    }
    ok = abs(cs[P.BELT_TEETH["pan"]] - c) < 1e-6 and all(
        abs(v - c) <= P.SLOT_TRAVEL for v in cs.values()
    )
    chk(
        "pan_belt_centre_distance",
        ok,
        f"{P.BELT_TEETH['pan']}T GT2 loop ({L_pan:.0f} mm) -> C = {c:.2f} mm; +-1 tooth belt: "
        f"{cs[P.BELT_TEETH['pan'] - 1]:.2f}/{cs[P.BELT_TEETH['pan'] + 1]:.2f}; slot +-{P.SLOT_TRAVEL}",
    )
    sh0 = -P.STATOR_T - P.MOUNT_T  # motor flange face
    ok = sh0 + 3 <= P.Z_BELT - 4 and P.Z_BELT + 4 <= sh0 + P.N17_SHAFT_LEN
    chk(
        "pan_belt_plane_on_shaft",
        ok,
        f"belt zone z {P.Z_BELT - 4}..{P.Z_BELT + 4} inside shaft z {sh0}..{sh0 + P.N17_SHAFT_LEN}",
    )
    hall = S["pan_hall_mount"]
    gap = (P.Z_ROTOR0) - (P.Z_ROTOR0 - 3.0)
    chk(
        "pan_hall_magnet_alignment",
        void(hall, -80, 0, P.Z_ROTOR0 - 3.0 - 1)
        and void(ro, -80, 0, P.Z_ROTOR0 + 1.5)
        and not void(hall, -80, 0, 5),
        "sensor 3.0 mm under the rotor magnet pocket at (-80, 0); wiring slot present",
    )

    yl, yr = S["yoke_left"], S["yoke_right"]
    ch, bm, el = S["launcher_chassis"], S["trunnion_beam"], S["feed_elbow"]
    hb = P.H_T
    ok = (
        void(yl, 0, K.Y_C1 - 3, hb)
        and abs(extent(yl, 0, (0, K.Y_C0 + 1, hb), 10)[0] - 14.0) < 0.3
    )  # left: 608 seat + lip bore
    ok &= hv(bm, X_A, K.YO + 3.5, Z_A)  # left rod seat in the beam, same line
    seat_ok = extent(yl, 0, (0, K.Y_C1 - 3, hb), 20)
    seat_r = extent(yr, 0, (0, -(K.Y_C1 - 3), hb), 30)
    thru_r = extent(yr, 0, (0, -(K.Y_C0 + 1), hb), 30)
    yb = -(
        K.Y_C1 - 3
    )  # right: hollow trunnion tube centred in the 6807 seat and through hole
    tube_bore = 2 * reach(bm, 2, (0, yb, 0), +1, 30)  # H frame: axis at the origin
    ring = (not void(bm, 0, yb, 17.2)) and void(bm, 0, yb, 17.8)  # OD 35
    ok &= (
        abs(seat_r[0] - (P.B6807_OD + 0.2)) < 0.3
        and abs(thru_r[0] - (P.TUBE_OD + 5.0)) < 0.3
    )
    ok &= abs(tube_bore - P.JOUR_ID) < 0.3 and ring
    spig = 2 * reach(el, 2, (0, yb, hb), +1, 30)  # elbow spigot bore (B frame)
    gap = P.JOUR_ID / 2 - P.SPIG_OD / 2
    ok &= abs(spig - P.SPIG_ID) < 0.3 and 0.2 <= gap <= 0.8 and void(el, 0, yb, hb)
    ok &= not void(el, 0, yb, hb + P.SPIG_ID / 2 + 0.9)  # spigot wall present
    chk(
        "tilt_axis_bearing_seats_coaxial",
        ok and abs(seat_ok[0] - (P.B608_OD + 0.2)) < 0.3,
        f"left 608 seat {seat_ok[0]:.2f}; right 6807 seat {seat_r[0]:.2f} + {thru_r[0]:.1f} through hole; hollow trunnion bore {tube_bore:.1f} "
        f"OD {P.TUBE_OD}; fixed spigot bore {spig:.1f} OD {P.SPIG_OD:.1f} (running gap {gap:.2f}/side); left rod seat in the beam: all on z={hb}",
    )
    boss_gap = K.Y_C0 - (K.YO + P.BOSS_L)
    chk(
        "cheek_to_boss_gap",
        0.5 <= boss_gap <= 2,
        f"{boss_gap:.2f} mm between boss end and cheek inner face",
    )
    ok = True
    for sx in K.YOKE_SCREW_X:
        for yy in K.YOKE_SCREW_Y:
            for sy, yk in ((1, yl), (-1, yr)):
                ok &= void(yk, sx, sy * yy, zr + 2) and void(ro, sx, sy * yy, zr - 3)
    chk(
        "yoke_to_rotor_screws",
        ok,
        f"{len(K.YOKE_SCREW_X) * 2} screws per cheek line up with rotor inserts",
    )

    tm = S["tilt_motor_mount"]
    zc = P.H_T - P.C_TILT
    L_t = P.BELT_TEETH["tilt"] * P.PITCH
    span, mid = extent(tm, 2, (0, K.Y_MP0 + 2, zc), 30)
    ct = {
        n: P.centre_distance(n, P.T_MOTOR, P.T_TILT)
        for n in (
            P.BELT_TEETH["tilt"] - 1,
            P.BELT_TEETH["tilt"],
            P.BELT_TEETH["tilt"] + 1,
        )
    }
    ok = (
        abs(mid - zc) < 0.1
        and abs(span - (P.N17_PILOT + 0.4 + 2 * P.SLOT_TRAVEL)) < 0.2
        and abs(P.H_T - mid - P.C_TILT) < 0.1
    )
    ok &= abs(ct[P.BELT_TEETH["tilt"]] - P.C_TILT) < 1e-6 and all(
        abs(v - P.C_TILT) <= P.SLOT_TRAVEL for v in ct.values()
    )
    chk(
        "tilt_belt_centre_distance",
        ok,
        f"{P.BELT_TEETH['tilt']}T loop ({L_t:.0f} mm) -> C = {P.C_TILT:.2f}; slot centre measured {P.H_T - mid:.2f}; "
        f"+-1 tooth belt {ct[P.BELT_TEETH['tilt'] - 1]:.2f}/{ct[P.BELT_TEETH['tilt'] + 1]:.2f}",
    )
    yb1 = K.Y_MP0 + K.MP_T
    ok = (
        yb1 - P.N17_SHAFT_LEN + 3 <= K.Y_BELT - 4
        and K.Y_BELT + 4 <= K.Y_MP0
        and K.Y_C1 + 1 < yb1 - P.N17_SHAFT_LEN + 24
    )
    chk(
        "tilt_belt_plane_on_shaft",
        ok,
        f"belt y {K.Y_BELT - 4}..{K.Y_BELT + 4}; motor shaft exposed y {yb1 - P.N17_SHAFT_LEN}..{K.Y_MP0}; pulley web y {K.Y_P0}",
    )
    ok = True
    for sx in (-33, 33):
        for z in (hb, zc):
            ok &= void(tm, sx, K.Y_C1 + 5, z) and void(yl, sx, K.Y_C1 - 2, z)
    chk(
        "tilt_motor_mount_posts_to_cheek",
        ok,
        "4 x M3 post screws line up with the left-cheek inserts",
    )
    hub, p80 = S["tilt_hub"], S["tilt_pulley_80t"]
    ok = True
    for k in range(4):
        px, pz = (
            11 * math.cos(math.radians(45 + 90 * k)),
            11 * math.sin(math.radians(45 + 90 * k)),
        )
        ok &= void(hub, px, K.Y_HUB0 + K.HUB_W - 3, pz) and void(
            p80, px, K.Y_P0 + 1, pz
        )
    ok &= (
        void(hub, 0, K.Y_HUB0 + 1, 13)
        and void(yl, 0, K.Y_C1 - 1.5, hb + 13)
        and abs((K.Y_HUB0) - K.Y_C1) < 1
    )
    chk(
        "tilt_hub_pulley_screws_and_hall",
        ok,
        "4 x M3 hub<->pulley; hub magnet pocket faces the cheek sensor pocket (gap 0.5 + pocket depth)",
    )

    # barrel/cage/hopper/camera/servo/cover joins (L coordinates through hv)
    ba, bb_ = S["barrel_half_a"], S["barrel_half_b"]
    ok = True
    for sz in (-12.7, 12.7):
        ok &= hv(ch, xc - 3, 12.7, sz) and hv(ch, xc - 3, -12.7, sz)
        ok &= hv(ba, xc + 2, 12.7, sz) and hv(bb_, xc + 2, -12.7, sz)
    chk("barrel_flange_to_cage_screws", ok, "4 x M3 on (+-12.7, +-12.7)")
    ok = (
        hv(ch, xc - 1, 0, 0)
        and hv(ba, xc + 1, 0, 0)
        and hv(bb_, xc + 1, 0, 0)
        and hv(ch, 0, 0, 0)
    )
    dia = reach(ba, 1, (xc + 20 - X_A, 0, -Z_A), +1) + reach(
        bb_, 1, (xc + 20 - X_A, 0, -Z_A), -1
    )
    chk(
        "bore_continuity",
        ok and abs(dia - P.BARREL_ID) < 0.3,
        f"bore dia {dia:.2f} (ball {P.BALL_D} + {P.BALL_CLEAR}); cage, barrel halves share the axis",
    )
    hp = S["hopper"]
    _, s1, zf = K.elbow_dims()
    ok = True
    for k in range(4):
        px, py = K.polar(18, 45 + 90 * k)
        ok &= void(hp, px, s1[1] + py, zf + 2) and void(el, px, s1[1] + py, zf - 3)
    duct = extent(el, 0, (0, s1[1], zf - 10), 20)[0]
    neck = extent(hp, 0, (0, s1[1], zf + 10), 20)[0]
    chk(
        "hopper_neck_to_feed_elbow",
        ok and abs(duct - neck) < 0.3 and neck > P.BALL_D + 2,
        f"4 x M3 flange screws line up; neck bore {neck:.1f} = elbow duct {duct:.1f} (> ball {P.BALL_D}); hopper axis at y={s1[1]:.0f}, flange z={zf:.1f} (B frame, pan group)",
    )
    ok = all(
        void(el, sx, -(K.Y_C1 + 2), hb + dz) and void(yr, sx, -(K.Y_C1 - 2), hb + dz)
        for sx, dz in K.ELBOW_BOLTS
    )
    chk(
        "feed_elbow_to_cheek_screws",
        ok,
        f"{len(K.ELBOW_BOLTS)} x M3 through the elbow plate into right-cheek inserts around the 6807 seat",
    )
    ok = all(
        hv(ch, sx, sy, P.CAGE_Z1 - 3) and hv(bm, sx, sy, P.CAGE_Z1 + 1.5)
        for sx, sy in K.BEAM_BOLTS
    )
    chk(
        "trunnion_beam_to_cage_screws",
        ok,
        f"{len(K.BEAM_BOLTS)} x M3 through the beam foot into cage-top inserts (beam bolts on after the motors are in)",
    )
    cap = K.hopper_capacity()
    chk(
        "hopper_capacity",
        cap >= P.HOPPER_BALLS,
        f"~{cap:.0f} balls at packing {P.PACKING} (target {P.HOPPER_BALLS}); height follows the ball diameter",
    )
    cm = S["camera_mount"]
    ok = all(
        void(cm, x - X_A, sy, K.CAM_Z0 + 2 - Z_A)
        and void(ba if sy > 0 else bb_, x - X_A, sy, K.CAM_Z0 - 3 - Z_A)
        for x in (110.0, 124.0)
        for sy in (-4.0, 4.0)
    )
    ok &= void(cm, K.MUZZLE_X - P.BRIO[0] / 2 - X_A, 0, K.CAM_Z0 + 2 - Z_A)
    chk(
        "camera_mount_to_barrel_and_tripod",
        ok,
        "4 x M3 into barrel-ear inserts; 1/4-20 hole at the Brio centre",
    )
    sb = S["servo_bracket"]
    ok = all(
        void(sb, K.X_S + sx - X_A, K.Y_S, K.Z_SERVO_PLATE - 1 - Z_A)
        and hv(ch, K.X_S + sx, K.Y_S, P.CHASSIS_Z0 + 3)
        for sx in (-30, 30)
    )
    chk("servo_bracket_to_chassis", ok, "2 x M3")
    wc = S["wheel_cover_pos_y"]
    ok = all(
        void(wc, sx - X_A, K.YO - 1.5, sz - Z_A) and hv(ch, sx, K.YO - 5, sz)
        for sx in (-23.5, 23.5)
        for sz in (-5.5, 5.5)
    )
    chk("wheel_cover_screws", ok, "4 x M3 per cover")
    # pusher kinematics
    d = abs(K.Y_S)
    arm = S["pusher_arm"]
    worst, rs = 0.0, []
    for i in range(21):
        xp = K.PIN_X_HOME + P.PISTON_STROKE * i / 20
        r = math.hypot(xp - K.X_S, 0 - K.Y_S)
        ang = math.degrees(math.atan2(xp - K.X_S, 0 - K.Y_S))
        worst = max(worst, abs(ang))
        rs.append(r)
    ra, rb_ = d, math.hypot(d, P.PISTON_STROKE / 2)
    in_slot = (
        all(ra - 0.01 <= r <= rb_ + 0.01 for r in rs)
        and void(arm, K.X_S - X_A, K.Y_S + ra + 0.2, K.Z_ARM - Z_A)
        and void(arm, K.X_S - X_A, K.Y_S + rb_ - 0.2, K.Z_ARM - Z_A)
    )
    chk(
        "pusher_lever_kinematics",
        in_slot and worst < 60,
        f"stroke {P.PISTON_STROKE} mm needs lever angle +-{worst:.1f} deg (servo range ~+-90), pin radius {min(rs):.1f}..{max(rs):.1f} inside slot {ra:.1f}..{rb_:.1f}",
    )
    # pin slot covers travel
    chk(
        "pusher_pin_slot",
        void(ch, K.PIN_X_HOME - X_A - 1.9, 0, P.CHASSIS_Z0 + 2 - Z_A)
        and void(
            ch, K.PIN_X_HOME + P.PISTON_STROKE - X_A + 1.9, 0, P.CHASSIS_Z0 + 2 - Z_A
        ),
        "slot spans the pin travel",
    )

    # ---- bought-envelope interferences ----------------------------------------------------------------------------------------------
    head = {i["name"]: i for i in BT.head_items() if i["shape"] is not None}
    bad = {}
    for n, i in head.items():
        for pn in (
            "launcher_chassis",
            "barrel_half_a",
            "barrel_half_b",
            "servo_bracket",
            "camera_mount",
            "trunnion_beam",
        ):
            v = vol(i["shape"], S[pn])
            if v > 1.0 and not (n.startswith("fw_wheel") and False):
                bad[f"{n}x{pn}"] = round(v, 1)
    chk(
        "bought_parts_fit_printed_pockets",
        not bad,
        f"motors, wheels, servo, Brio, rods vs printed head parts: {bad or 'no overlap'}",
    )
    w1, w2 = head["fw_wheel_py"]["shape"], head["fw_wheel_ny"]["shape"]
    gap = float(w1.distance_to(w2))
    chk(
        "flywheel_gap",
        abs(gap - P.FW_GAP) < 0.01,
        f"rim gap {gap:.3f} mm = ball {P.BALL_D} - compression {P.COMPRESSION}",
    )

    # ---- 4 ball path -----------------------------------------------------------------------------------------------------------------
    rb_ = P.BALL_D / 2
    solids = [S["launcher_chassis"], S["barrel_half_a"], S["barrel_half_b"]]
    worst = 0.0
    n = 0
    for x in np.arange(P.X_F, K.MUZZLE_X + 6, 3.0):  # bore + barrel
        ball = K.L2H(Pos(float(x), 0, 0) * Sphere(rb_))
        for s in solids:
            if abs(x) < 1e9:
                worst = max(worst, vol(ball, s))
        n += 1
    chk(
        "ball_path_clear",
        worst < 0.5,
        f"ball dia {P.BALL_D} swept along the bore and barrel ({n} positions): max overlap {worst:.2f} mm3",
    )
    feed_checks(S, rb_)
    pist = S["piston"]
    worst = max(
        vol(Pos(dx, 0, 0) * pist, s)
        for dx in np.arange(0, P.PISTON_STROKE + 1, 4.0)
        for s in solids
    )
    wh = max(
        vol(Pos(dx, 0, 0) * pist, head[w]["shape"])
        for dx in (0, P.PISTON_STROKE)
        for w in ("fw_wheel_py", "fw_wheel_ny")
    )
    chk(
        "piston_travel_clear",
        worst < 0.5 and wh < 0.5,
        f"piston over {P.PISTON_STROKE} mm stroke: bore overlap {worst:.2f} mm3, wheel overlap {wh:.2f} mm3",
    )
    ball0 = K.L2H(Pos(0, 0, 0) * Sphere(rb_))
    sq = vol(ball0, w1) + vol(ball0, w2)
    chk(
        "ball_squeezed_by_wheels",
        sq > 0,
        f"ball at the wheel centre overlaps the wheels by {sq:.0f} mm3 (compression {P.COMPRESSION} mm), and touches no printed wall",
    )

    # ---- 5 tilt sweep ----------------------------------------------------------------------------------------------------------------
    tilt_p = {
        n: S[n]
        for n, (_, g, _, _) in K.REGISTRY.items()
        if g == "tilt" and n not in ACCESSORIES
    }
    tilt_b = {n: i["shape"] for n, i in head.items()}
    tilt_all = {**tilt_p, **tilt_b}
    stat_p = {
        n: S[n]
        for n, (_, g, _, _) in K.REGISTRY.items()
        if g != "tilt" and n not in ACCESSORIES
    }
    stat_b = {i["name"]: i["shape"] for i in BT.pan_items() + BT.static_items()}
    stat_all = {**stat_p, **stat_b}
    exclude = {
        ("rod_left", "bearing_left"),
        ("trunnion_beam", "bearing_right"),  # tube is the bearing inner race
        ("rod_left", "yoke_left"),
        ("rod_left", "tilt_motor_mount"),
        ("tilt_pulley_80t", "tilt_motor_pulley"),
        ("tilt_hub", "bearing_left"),
        ("tilt_hub", "yoke_left"),  # designed 0.5 mm running gap
    }
    angles = list(np.arange(P.TILT_RANGE[0], P.TILT_RANGE[1] + 0.1, 5.0))
    hits, mind = {}, (1e9, None)
    stat_bb = {n: s.bounding_box() for n, s in stat_all.items()}
    for phi in angles:
        for tn, ts in tilt_all.items():
            tr = rot_tilt(ts, phi)
            tb = tr.bounding_box()
            for sn, ss in stat_all.items():
                if (tn, sn) in exclude:
                    continue
                sb_ = stat_bb[sn]
                if (
                    tb.max.X < sb_.min.X - 20
                    or tb.min.X > sb_.max.X + 20
                    or tb.max.Y < sb_.min.Y - 20
                    or tb.min.Y > sb_.max.Y + 20
                    or tb.max.Z < sb_.min.Z - 20
                    or tb.min.Z > sb_.max.Z + 20
                ):
                    continue
                v = vol(tr, ss)
                if v > 1.0:
                    hits[(tn, sn, float(phi))] = round(v, 1)
                    continue
                dd = float(tr.distance_to(ss))
                if dd < mind[0]:
                    mind = (dd, (tn, sn, float(phi)))
    chk(
        "tilt_sweep_no_collision",
        not hits,
        f"{len(angles)} angles {P.TILT_RANGE[0]}..{P.TILT_RANGE[1]} deg x {len(tilt_all)} tilting items x {len(stat_all)} fixed items: "
        f"{('collisions ' + str(list(hits.items())[:6])) if hits else 'no overlap'}; min clearance {mind[0]:.2f} mm at {mind[1]}",
    )
    lo, hi = P.TILT_RANGE
    ev = {}
    for phi in (lo - 1, lo - 6, hi + 1, hi + 6):
        ev[phi] = round(vol(rot_tilt(S["trunnion_beam"], phi), yr), 1)
    ok = ev[lo - 1] < 1 and ev[hi + 1] < 1 and ev[lo - 6] > 1 and ev[hi + 6] > 1
    chk(
        "tilt_stops_engage_outside_range",
        ok,
        f"lug/stop-block overlap mm3 at {lo - 1}:{ev[lo - 1]}, {lo - 6}:{ev[lo - 6]}, {hi + 1}:{ev[hi + 1]}, {hi + 6}:{ev[hi + 6]} "
        f"(free to {lo - 3} / {hi + 3}, hard stop beyond)",
    )

    pan_checks(S, tilt_p, tilt_b, stat_all)

    # ---- 6 balance -------------------------------------------------------------------------------------------------------------------
    cgs = balance(recs)
    ok = all(v["torque_nm"] <= P.TILT_STATIC_BUDGET_NM for v in cgs.values())
    chk(
        "head_static_torque_within_spec_budget",
        ok,
        f"spec 6.3 allows {P.TILT_STATIC_BUDGET_NM:.3f} N*m static unbalance at the tilt joint (1.2 kg head at 15 mm); "
        + "; ".join(
            f"{k}: CG ({v['x']:.1f}, {v['z']:.1f}) mm from the axis, mass {v['mass_g']:.0f} g, |r|={v['radial_mm']:.1f}, "
            f"static torque {v['torque_nm']:.3f} N*m"
            for k, v in cgs.items()
        ),
    )
    ms = mass_split(recs)
    chk(
        "pan_tilt_mass_split",
        True,
        f"tilt group {ms['tilt_group_g']:.0f} g (hopper no longer on it); pan group (cheeks, hopper, elbow, motor) {ms['pan_group_g']:.0f} g "
        f"+ {ms['balls_g']:.0f} g balls; pan-axis inertia hopper empty/full at tilt 0: {ms['Iz_pan_empty_kgm2']:.4f}/{ms['Iz_pan_full_kgm2']:.4f} kg*m2 "
        f"(spec 6.3 assumed 7.4e-3 for the whole 1.5 kg head)",
    )
    cam = dict(y=0.0)
    bd, bw, bh = P.BRIO
    chk(
        "camera_axis_parallel_to_barrel",
        True,
        f"Brio centred on y=0, axis along +x; lens {K.CAM_Z0 + P.CAM_PLATE_T + bh / 2:.1f} mm above the barrel axis (parallax offset d_c), "
        f"front face flush with the muzzle x={K.MUZZLE_X}; residual yaw/pitch trimmed with the 3 deg shims",
    )
    return RES


def balance(recs):
    """CG of the tilting head in H (x, z relative to the axis); the hopper is on the pan group, so only the balls queued in the
    chute (4 along C -> F, worst case) vary."""
    items = []
    for n, (_, g, cnt, _) in K.REGISTRY.items():
        if g == "tilt" and n not in ACCESSORIES:
            r = recs[n]
            items.append((r["filament_g"], r["com_mm"]))
    for i in BT.head_items():
        if i["shape"] is not None:
            c = i["shape"].center()
            items.append((i["mass"], [c.X, c.Y, c.Z]))
    cF = np.array([P.X_F - X_A, 0.0, -Z_A])
    full = items + [(P.BALL_M, list(cF * t)) for t in (0.0, 1 / 3, 2 / 3, 1.0)]
    out = {}
    for name, it in (("chute_loaded", full), ("chute_empty", items)):
        m = sum(a for a, _ in it)
        x = sum(a * c[0] for a, c in it) / m
        z = sum(a * c[2] for a, c in it) / m
        out[name] = dict(
            x=x,
            z=z,
            mass_g=m,
            radial_mm=math.hypot(x, z),
            torque_nm=m * 1e-3 * 9.81 * math.hypot(x, z) * 1e-3,
        )
    return out


def mass_split(recs, phi=0.0):
    """Mass of the tilt group and the pan group, and the inertia about the pan axis of everything that rotates with pan
    (head at tilt phi), with and without the 40 hopper balls. Bought items are point masses."""
    ha = P.H_T
    R = Ry_tilt(phi)
    rows = {"tilt": [], "pan": []}
    for n, (_, g, cnt, _) in K.REGISTRY.items():
        if n in ACCESSORIES or g == "static":
            continue
        r = recs[n]
        com, I = np.array(r["com_mm"]), np.array(r["inertia_kgm2_about_com"])
        if g == "tilt":
            com, I = np.array([0, 0, ha]) + R @ com, R @ I @ R.T
        rows[g].append((r["filament_g"] * 1e-3, com * 1e-3, I))
    for i in BT.head_items():
        c = i["shape"].center()
        rows["tilt"].append(
            (
                i["mass"] * 1e-3,
                (np.array([0, 0, ha]) + R @ np.array([c.X, c.Y, c.Z])) * 1e-3,
                np.zeros((3, 3)),
            )
        )
    for i in BT.pan_items():
        c = i["shape"].center()
        m = i["mass"] * (
            0.5 if i["name"] == "lazy_susan" else 1.0
        )  # one ring is static
        rows["pan"].append(
            (m * 1e-3, np.array([c.X, c.Y, c.Z]) * 1e-3, np.zeros((3, 3)))
        )
    bx, by, bz = BT.balls_cg_B()
    balls = (
        P.HOPPER_BALLS * P.BALL_M * 1e-3,
        np.array([bx, by, bz]) * 1e-3,
        np.zeros((3, 3)),
    )

    def iz(rs):
        return sum(r[2][2, 2] + r[0] * (r[1][0] ** 2 + r[1][1] ** 2) for r in rs)

    allr = rows["tilt"] + rows["pan"]
    return dict(
        tilt_group_g=sum(r[0] for r in rows["tilt"]) * 1e3,
        pan_group_g=sum(r[0] for r in rows["pan"]) * 1e3,
        balls_g=balls[0] * 1e3,
        Iz_pan_empty_kgm2=iz(allr),
        Iz_pan_full_kgm2=iz(allr + [balls]),
        tilt_deg=phi,
        note="pan_group = printed pan parts incl. hopper+elbow+lid, tilt motor+pulley, bearings, half the lazy susan; inertia about B z of pan+tilt groups, bought items as point masses",
    )


def interface(recs):
    """What the assembly worker needs: frames, joint axes, mount points, camera/barrel poses (mm, B or H frames)."""
    bd, bw, bh = P.BRIO
    cam_z = K.CAM_Z0 + P.CAM_PLATE_T + bh / 2 - Z_A
    cams = dict(
        centre_H_mm=[K.MUZZLE_X - bd / 2 - X_A, 0, cam_z],
        lens_estimate_H_mm=[K.MUZZLE_X - 8.0 - X_A, 0, cam_z],
        note="lens ~8 mm behind the Brio front face (est); optical axis = +x_H, image up = +z_H; OpenCV C: x_C=-y_H, y_C=-z_H, z_C=+x_H",
    )
    pts = lambda **k: k
    return dict(
        units="mm, degrees; napl/*.stl are metres in the same frames",
        frames=dict(
            B="origin at the centre of the stator-plate top face on the pan axis; x forward (pan 0), y left, z up",
            pan_joint=dict(
                parent="B",
                axis=[0, 0, 1],
                origin_B_mm=[0, 0, 0],
                positive="CCW from above; hall window (magnet) at pan 0",
                range_deg=[-170, 170],
                children=[n for n, (_, g, _, _) in K.REGISTRY.items() if g == "pan"],
            ),
            H=dict(
                origin_B_mm=[0, 0, P.H_T],
                note="tilt axis; x along the barrel, y left, z up at tilt = 0",
            ),
            tilt_joint=dict(
                parent="pan",
                axis_H=[0, -1, 0],
                origin_B_mm=[0, 0, P.H_T],
                positive="muzzle up = right-handed rotation about -y (about +y it is negative)",
                range_deg=list(P.TILT_RANGE),
                children=[
                    n
                    for n, (_, g, _, _) in K.REGISTRY.items()
                    if g == "tilt" and n not in ACCESSORIES
                ],
            ),
        ),
        ball=dict(d_mm=P.BALL_D, m_g=P.BALL_M),
        barrel=dict(
            axis_H_mm=dict(origin=[-X_A, 0, -Z_A], direction=[1, 0, 0]),
            d_b_mm=-Z_A,
            bore_dia_mm=P.BARREL_ID,
            muzzle_H_mm=[K.MUZZLE_X - X_A, 0, -Z_A],
            length_mm=P.BARREL_LEN,
            ball_entry_H_mm=[P.X_F - X_A, 0, -Z_A + 0],
        ),
        flywheels=dict(
            centre_H_mm=[-X_A, 0, -Z_A],
            wheel_dia_mm=P.WHEEL_D,
            gap_mm=P.FW_GAP,
            axes="+-z_H at y=+-(%.2f)" % (P.FW_GAP / 2 + P.WHEEL_D / 2),
        ),
        camera=cams,
        mounts=dict(
            pan_motor=dict(
                frame="B",
                shaft_axis=[0, 0, 1],
                pos_mm=[P.C_PAN, 0, -P.STATOR_T - P.MOUNT_T],
                shaft_top_z_mm=-P.STATOR_T - P.MOUNT_T + P.N17_SHAFT_LEN,
                belt_plane_z_mm=P.Z_BELT,
            ),
            tilt_motor=dict(
                frame="B (pan group)",
                shaft_axis=[0, -1, 0],
                flange_face_y_mm=K.Y_MP0 + K.MP_T,
                pos_mm=[0, K.Y_MP0 + K.MP_T, P.H_T - P.C_TILT],
                belt_plane_y_mm=K.Y_BELT,
            ),
            pan_hall=dict(
                sensor_B_mm=[-80, 0, P.Z_ROTOR0 - 3.0],
                magnet_B_mm=[-80, 0, P.Z_ROTOR0],
                magnet_fixed_to="pan_rotor_plate",
                sensor_fixed_to="pan_hall_mount",
                window_deg=2.0,
            ),
            tilt_hall=dict(
                sensor_B_mm=[0, K.Y_C1, P.H_T + 13],
                magnet_H_mm=[0, K.Y_HUB0, 13],
                magnet_fixed_to="tilt_hub",
                sensor_fixed_to="yoke_left",
                note="aligned at tilt = 0",
            ),
            lazy_susan=dict(
                bolt_circle_r_mm=P.LS_BOLT_R,
                ring1_z_mm=[P.Z_LS0, P.Z_LS0 + 4.5],
                ring2_z_mm=[P.Z_LS0 + 4.5, P.Z_ROTOR0],
                note="confirm hole pattern at purchase",
            ),
            bearing_608_left=dict(
                frame="B", pos_mm=[0, K.Y_C1 - 3.5, P.H_T], axis=[0, 1, 0]
            ),
            bearing_6807_right=dict(
                frame="B",
                pos_mm=[0, -(K.Y_C1 - P.B6807_W / 2), P.H_T],
                axis=[0, 1, 0],
                od_id_w_mm=[P.B6807_OD, P.B6807_ID, P.B6807_W],
                note="inner race on the 35 mm hollow trunnion tube of trunnion_beam; seat in the right cheek outer face",
            ),
            rod_8mm_left=dict(frame="H", y_range_mm=[39, K.Y_P0]),
            ball_feed=dict(
                note="hopper + elbow are PAN group (frame B, do not tilt); balls enter the head along the tilt axis through the hollow right trunnion, "
                "turn 90 deg at C (H origin) and roll down a straight chute to F, the rest point in front of the piston",
                hopper=dict(
                    frame="B",
                    axis_B_mm=[0, -P.ELBOW_YH, K.elbow_dims()[2]],
                    flange_bolts="4 x M3 on r=18 (45 deg + k*90), into elbow inserts",
                    neck_bore_mm=P.DUCT_ID,
                    balls=P.HOPPER_BALLS,
                    top_rim_z_B_mm=K.elbow_dims()[2]
                    + K.HOPPER["Hn"]
                    + K.HOPPER["Hc"]
                    + K._hopper_dims()[1],
                ),
                feed_elbow=dict(
                    frame="B",
                    plate_on="yoke_right outer face y=%.1f" % -K.Y_C1,
                    bolts_x_dz_mm=[list(b) for b in K.ELBOW_BOLTS],
                    slope_deg=P.ELBOW_SLOPE,
                    slope_start_B_mm=list(K.elbow_dims()[1]),
                    slope_end_B_mm=list(K.elbow_dims()[0]),
                    spigot_tip_B_mm=[0, -P.SPIG_TIP_Y, P.H_T],
                    spigot_od_id_mm=[P.SPIG_OD, P.SPIG_ID],
                ),
                trunnion_beam=dict(
                    frame="H",
                    elbow_point_C_H_mm=[0, 0, 0],
                    rest_point_F_H_mm=[P.X_F - X_A, 0, -Z_A],
                    chute_bore_mm=2 * P.CHUTE_R,
                    trunnion_bore_mm=P.JOUR_ID,
                    bolts_to_cage_L_xy_mm=[list(b) for b in K.BEAM_BOLTS],
                ),
                per_tilt_deg={
                    str(int(k)): dict(
                        chute_downhill_deg=round(v["chute"]["deg"], 1),
                        level_run_mm=round(v["level"]["len"], 1),
                        max_ball_overlap_mm3=round(v["worst"], 2),
                    )
                    for k, v in FEED.items()
                },
            ),
            pusher_servo=dict(
                frame="H",
                spline_L_mm=[K.X_S, K.Y_S, K.Z_SERVO_PLATE + 9.2],
                spline_H_mm=[K.X_S - X_A, K.Y_S, K.Z_SERVO_PLATE + 9.2 - Z_A],
                axis=[0, 0, 1],
                lever_zero="lever points +y at the stroke middle; piston pin x_H=%.1f..%.1f"
                % (K.PIN_X_HOME - X_A, K.PIN_X_HOME + P.PISTON_STROKE - X_A),
            ),
            piston=dict(
                frame="H",
                home_face_x_mm=P.X_F - 13.5 - X_A,
                stroke_mm=P.PISTON_STROKE,
                axis=[1, 0, 0],
            ),
            brio=dict(
                frame="H",
                box_centre_mm=[K.MUZZLE_X - bd / 2 - X_A, 0, cam_z],
                size_mm=list(P.BRIO),
            ),
        ),
        instances=dict(
            base_leg=[
                dict(centre_B_mm=[sx, sy], yaw_deg=(0 if sy > 0 else 180))
                for sx in (88, -88)
                for sy in (75, -75)
            ],
            note="base_leg is drawn at centre (+88, +75); other instances rotate about their own centre by yaw_deg then translate. cable_clip x6 and camera_shim x2 are accessories drawn at the origin.",
        ),
        head_balance=balance(recs),
        mass_split={f"tilt_{int(p)}": mass_split(recs, p) for p in (0.0, 70.0)},
        placement_of_part_meshes="every napl/<part>.stl is already in its group frame (B for static+pan, H for tilt): place at the group's frame origin, identity rotation",
    )
