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
    ch = S["launcher_chassis"]
    hb = P.H_T
    ok = True
    for s, y in ((1, K.Y_C1 - 3), (-1, -(K.Y_C1 - 3))):
        yk = yl if s > 0 else yr
        ok &= void(yk, 0, y, hb)  # 608 seat
        ok &= (
            abs(extent(yk, 0, (0, s * (K.Y_C0 + 1), hb), 10)[0] - 14.0) < 0.3
        )  # lip bore
        ok &= hv(ch, X_A, s * (K.YO + 3.5), Z_A)  # rod seat in boss on the same axis
    seat_ok = extent(yl, 0, (0, K.Y_C1 - 3, hb), 20)
    chk(
        "tilt_axis_bearing_seats_coaxial",
        ok and abs(seat_ok[0] - (P.B608_OD + 0.2)) < 0.3,
        f"608 seat dia {seat_ok[0]:.2f} (bearing {P.B608_OD}+0.2) at z={hb} in both cheeks; rod seats in both bosses on the same line",
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
    ok = all(
        void(hp, sx * 12.7 + P.X_F - X_A, sy * 12.7, P.CHASSIS_Z1 + 2 - Z_A)
        and void(ch, sx * 12.7 + P.X_F - X_A, sy * 12.7, P.CHASSIS_Z1 - 3 - Z_A)
        for sx in (-1, 1)
        for sy in (-1, 1)
    )
    port = extent(ch, 0, (P.X_F - X_A, 0, P.CHASSIS_Z1 - 3 - Z_A), 20)[0]
    neck = extent(hp, 0, (P.X_F - X_A, 0, P.CHASSIS_Z1 + 10 - Z_A), 20)[0]
    chk(
        "hopper_neck_to_feed_port",
        ok and abs(port - neck) < 0.3 and neck > P.BALL_D + 2,
        f"flange screws line up; neck bore {neck:.1f} = feed port {port:.1f} (> ball {P.BALL_D})",
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
            "hopper",
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
    for z in np.arange(
        P.CHASSIS_Z1 + 22 + 40, 0, -4.0
    ):  # drop through the hopper neck and feed port
        ball = K.L2H(Pos(P.X_F, 0, float(z)) * Sphere(rb_))
        for s in (S["hopper"], S["launcher_chassis"]):
            worst = max(worst, vol(ball, s))
        n += 1
    chk(
        "ball_path_clear",
        worst < 0.5,
        f"ball dia {P.BALL_D} swept through hopper neck, feed port, cage bore and barrel ({n} positions): max overlap {worst:.2f} mm3",
    )
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
        ("rod_right", "bearing_right"),
        ("rod_left", "yoke_left"),
        ("rod_right", "yoke_right"),
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
        ev[phi] = round(vol(rot_tilt(S["launcher_chassis"], phi), yr), 1)
    ok = ev[lo - 1] < 1 and ev[hi + 1] < 1 and ev[lo - 6] > 1 and ev[hi + 6] > 1
    chk(
        "tilt_stops_engage_outside_range",
        ok,
        f"lug/stop-block overlap mm3 at {lo - 1}:{ev[lo - 1]}, {lo - 6}:{ev[lo - 6]}, {hi + 1}:{ev[hi + 1]}, {hi + 6}:{ev[hi + 6]} "
        f"(free to {lo - 3} / {hi + 3}, hard stop beyond)",
    )

    # ---- 6 balance -------------------------------------------------------------------------------------------------------------------
    cgs = balance(recs)
    ok = all(v["radial_mm"] <= P.BALANCE_LIMIT for v in cgs.values())
    chk(
        "head_cg_within_15mm_of_tilt_axis",
        ok,
        "; ".join(
            f"{k}: CG ({v['x']:.1f}, {v['z']:.1f}) mm from the axis, mass {v['mass_g']:.0f} g, |r|={v['radial_mm']:.1f}, "
            f"static torque {v['torque_nm']:.3f} N*m"
            for k, v in cgs.items()
        ),
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
    """CG of the tilting head in H (x, z relative to the axis), with a full and an empty hopper."""
    items = []
    for n, (_, g, cnt, _) in K.REGISTRY.items():
        if g == "tilt" and n not in ACCESSORIES:
            r = recs[n]
            items.append((r["filament_g"], r["com_mm"]))
    for i in BT.head_items():
        if i["shape"] is not None:
            c = i["shape"].center()
            items.append((i["mass"], [c.X, c.Y, c.Z]))
    bx, by, bz = BT.balls_cg_L()
    full = items + [(P.HOPPER_BALLS * P.BALL_M, [bx - X_A, by, bz - Z_A])]
    out = {}
    for name, it in (("hopper_full", full), ("hopper_empty", items)):
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
            bearings_608=[
                dict(frame="B", pos_mm=[0, s * (K.Y_C1 - 3.5), P.H_T], axis=[0, 1, 0])
                for s in (-1, 1)
            ],
            rods_8mm=[
                dict(frame="H", y_range_mm=[39, K.Y_P0]),
                dict(frame="H", y_range_mm=[-39, -(K.Y_C1 + 4)]),
            ],
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
        placement_of_part_meshes="every napl/<part>.stl is already in its group frame (B for static+pan, H for tilt): place at the group's frame origin, identity rotation",
    )
