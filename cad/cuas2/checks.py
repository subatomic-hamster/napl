"""Checks for the cuas2 CAD (spec 12 MECH acceptance 1). Every check measures the finished SOLIDS (point-in-solid probes, boolean volumes)
or evaluates the real design numbers; none just re-states a parameter. Each returns {"pass": bool, "detail": str, "value": {...}}.

C-GAP C-POCKET C-FEED C-NIP C-INTERLOCK C-BAL C-DOCK C-PATH C-FIT   (+ C-CYCLE, informational numbers for the R3 renegotiation)
"""

import math

import interfaces as I
import numpy as np
import params as P
from build123d import Pos, Rot, Sphere
from modules import feeder as FD
from modules import launcher_cartridge as LC
from modules import power_box as PB
from modules import sensor_head as SH
from modules import tilt_head as TH
from prims import about, hole, void, vol

RES = {}
G = 9.80665


def chk(name, ok, detail, value=None):
    RES[name] = {"pass": bool(ok), "detail": detail, "value": value or {}}
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}: {detail}", flush=True)


def pt(loc, p):
    return tuple((loc * Pos(*p)).position)


def ins(s, p):
    return not void(s, *p)


def edge(f, lo, hi, tol=0.01):
    """Bisection: f(lo) != f(hi); returns the boundary position."""
    a, b = f(lo), f(hi)
    assert a != b, "no boundary in range"
    while abs(hi - lo) > tol:
        m = (lo + hi) / 2
        if f(m) == a:
            lo = m
        else:
            hi = m
    return (lo + hi) / 2


def first_change(f, start, direction, tmax, step=0.1):
    """Walk from t=start in steps along `direction` until f changes; return refined t."""
    a = f(start)
    t = start
    while abs(t - start) < tmax:
        t2 = t + direction * step
        if f(t2) != a:
            return edge(f, t, t2, 0.005)
        t = t2
    return None


# ---------------------------------------------------------------------------------------------------------------------------------
def c_gap(S):
    tyre, plate = S["wheel_tyre"], S["gap_plate"]
    top = Pos(0, 0, P.WZ) * tyre
    bot = Pos(0, 0, -P.WZ) * tyre
    z_top = first_change(
        lambda z: ins(top, (0, 0, z)), 0.0, +1, 30, 0.1
    )  # lower surface of the top wheel at x = y = 0
    z_bot = first_change(lambda z: ins(bot, (0, 0, z)), 0.0, -1, 30, 0.1)
    gap = z_top - z_bot
    # slot travel from the gap plate solid: void run along z through the slot centre, and the hole diameter along x
    ymid = -P.CAGE_HALF_Y + P.PLATE_T / 2
    zc = -8.0
    f = lambda z: void(plate, 10.0, ymid, z)
    z_hi = first_change(lambda z: f(z), zc, +1, 12, 0.1)
    z_lo = first_change(lambda z: f(z), zc, -1, 12, 0.1)
    fx = lambda x: void(plate, x, ymid, zc)
    x_hi = first_change(fx, 10.0, +1, 6, 0.05)
    x_lo = first_change(fx, 10.0, -1, 6, 0.05)
    hole_d, slot_len = x_hi - x_lo, z_hi - z_lo
    travel = (slot_len - hole_d) / 2
    want = P.BALL_D - P.SQUEEZE
    ok = abs(gap - want) <= 0.05 and travel >= P.GAP_TRAVEL - 0.05
    chk(
        "C-GAP",
        ok,
        f"gap {gap:.3f} mm (want {want:.2f} +-0.05); slot travel +-{travel:.2f} mm -> reachable {gap - travel:.2f}..{gap + travel:.2f} (need {want - 1:.1f}..{want + 1:.1f}); wheel centres {2 * P.WZ:.1f} apart, D {P.WHEEL_D}, rpm_cap {P.RPM_CAP}",
        dict(
            gap_mm=gap,
            want_mm=want,
            travel_mm=travel,
            centre_sep_mm=2 * P.WZ,
            wheel_d_mm=P.WHEEL_D,
            rpm_cap=P.RPM_CAP,
        ),
    )


def pocket_len(plug):
    blank = hole("y", 2 * P.PLUG_R, P.XV, 0, -P.PLUG_L / 2, P.PLUG_L / 2)
    line = hole("x", 1.0, 0, 0, P.XV - P.PLUG_R - 5, P.XV + P.PLUG_R + 5)
    a = math.pi * 0.25
    return (vol(blank, line) - vol(plug, line)) / a


def c_pocket(S):
    plug = S["valve_plug"]
    L = pocket_len(plug)
    lim = 2 * P.BALL_D * 0.9
    ok = P.BALL_D <= L < lim
    chk(
        "C-POCKET",
        ok,
        f"pocket bore length {L:.2f} mm: one ball ({P.BALL_D}) fits, two centres (2 D = {2 * P.BALL_D:.1f}) cannot; limit {lim:.2f}",
        dict(len_mm=L, limit_mm=lim, ratio_to_D=L / P.BALL_D),
    )


def slope_deg(d, tilt):
    """Descent angle (deg below horizontal) of a head-frame direction d=(dx,dz) at tilt `tilt` (nose up)."""
    t = math.radians(tilt)
    dz_w = d[0] * math.sin(t) + d[1] * math.cos(t)
    return math.degrees(math.asin(max(-1.0, min(1.0, -dz_w))))


def c_feed(S):
    a, phi = math.radians(P.HOPPER_ALPHA), math.radians(P.GATE_SWING_DEG)
    segs = {
        "hopper_floor_facet": (
            -math.cos(a),
            -math.sin(a),
        ),  # descends toward the rear-bottom outlet
        "throat": (0.0, -1.0),
        "pocket_at_LOAD": (
            -math.cos(phi),
            -math.sin(phi),
        ),  # front end up: the ball runs to the rear end
    }
    table = {
        n: {str(t): round(slope_deg(d, t), 1) for t in P.TILT_CHECKS}
        for n, d in segs.items()
    }
    worst = min(min(v.values()) for v in table.values())
    # geometry continuity: the throat axis is clear from the hopper floor to the housing, and the LOAD mouth is open below it
    union_air = lambda p: void(S["valve_housing"], *p) and void(S["hopper"], *p)
    throat_ok = all(
        union_air((P.X_T, 0, z)) for z in np.arange(P.HOUSING_TOP, P.Z_HOP0 + 6, 2.0)
    )
    plug_load = about((P.XV, 0, 0), Rot(0, -P.GATE_SWING_DEG, 0)) * S["valve_plug"]
    mx, mz = P.XV + (P.PLUG_R - 4) * math.cos(phi), (P.PLUG_R - 4) * math.sin(phi)
    mouth_ok = void(plug_load, mx, 0, mz) and void(S["valve_housing"], mx, 0, mz)
    hop_vol = (
        abs(
            float(
                FD.prism_xz(FD.hopper_inner(), -P.HOPPER_W / 2, P.HOPPER_W / 2).volume
            )
        )
        / 1000.0
    )
    need = P.HOPPER_BALLS * math.pi / 6 * P.BALL_D**3 / P.PACKING / 1000.0
    ok = worst >= P.FEED_SLOPE_MIN and throat_ok and mouth_ok and hop_vol >= need * 0.99
    chk(
        "C-FEED",
        ok,
        f"min slope {worst:.1f} deg (>= {P.FEED_SLOPE_MIN}) over tilt {P.TILT_CHECKS}; throat air path {throat_ok}; LOAD mouth open {mouth_ok}; hopper {hop_vol:.0f} cm3 vs {need:.0f} needed for {P.HOPPER_BALLS} balls",
        dict(
            slopes_deg=table,
            worst_deg=worst,
            hopper_cm3=hop_vol,
            need_cm3=need,
            load_deg=P.GATE_SWING_DEG,
        ),
    )


def pose_overlaps(shape, S, step=2.5):
    """Overlap volume of `shape` with the plug at every pose BORE (0) .. LOAD, [(deg, mm3)] (pose-wise: no fragile unions)."""
    plug = S["valve_plug"]
    n = int(math.ceil(P.GATE_SWING_DEG / step))
    out = []
    for i in range(n + 1):
        deg = min(P.GATE_SWING_DEG, i * step)
        out.append((deg, vol(shape, about((P.XV, 0, 0), Rot(0, -deg, 0)) * plug)))
    return out


def c_nip_interlock(S):
    ret, ext = FD.pusher_rod(False), FD.pusher_rod(True)
    tip_r = ret.bounding_box().max.X
    tip_e = ext.bounding_box().max.X
    stroke = tip_e - tip_r
    po_ret, po_ext = pose_overlaps(ret, S), pose_overlaps(ext, S)
    clear_ret = max(v for _, v in po_ret)
    block_ext = max(v for _, v in po_ext)
    first_block = next((d for d, v in po_ext if v > 1.0), None)
    ok1 = stroke >= 9.5 + 5.0 and clear_ret < 1.0 and block_ext > 1.0
    chk(
        "C-NIP",
        ok1,
        f"stroke {stroke:.1f} mm (>= 14.5 = 9.5 + 5); retracted rod vs the valve at every pose {clear_ret:.1f} mm3 (must be 0: clears the shoulder); extended rod vs the valve: blocked from {first_block} deg, up to {block_ext:.0f} mm3 (> 0: valve cannot turn); ball centre ends at {tip_e + P.BALL_D / 2:.1f} (first contact -9.5)",
        dict(
            stroke_mm=stroke,
            tip_ret=tip_r,
            tip_ext=tip_e,
            retracted_overlap_mm3=clear_ret,
            extended_overlap_mm3=block_ext,
        ),
    )
    in_bore, at_load = po_ext[0][1], po_ext[-1][1]
    ok2 = in_bore < 1.0 and at_load > 1.0
    chk(
        "C-INTERLOCK",
        ok2,
        f"extended rod vs plug at BORE {in_bore:.1f} mm3 (free), at LOAD ({P.GATE_SWING_DEG:.0f} deg) {at_load:.0f} mm3 (plug wall closes the rod path)",
        dict(rod_vs_bore_mm3=in_bore, rod_vs_load_mm3=at_load),
    )


def head_mass_cg(Lshapes, recs, bought):
    """Head (tilt group) mass and CG in H, with real solids for printed parts and bought envelopes. Returns (m_g, cg(x,z), items)."""
    items = []
    for n, r in recs.items():
        if r["group"] != "tilt":
            continue
        for i in range(r["count"]):
            c = Pos(0, 0, 0)
            com = np.array(r["com_mm_L"])
            if r.get("at"):
                com = com + np.array(r["at"][i])
            items.append((n, r["filament_g"], com))
    for b in bought:
        if b["group"] != "tilt" or b.get("optional"):
            continue
        c = b["shape"].center()
        items.append((b["name"], b["mass"], np.array([c.X, c.Y, c.Z])))
    return items


def c_bal(S, recs, bought):
    items = head_mass_cg(S, recs, bought)
    hw = (
        ("hardware", P.HARDWARE_TILT_G, None),
        ("wiring", P.WIRING_TILT_G, np.array([P.X_A - 30.0, 0, P.Z_A])),
    )
    m = sum(i[1] for i in items)
    cg = sum(i[1] * i[2] for i in items) / m
    m_tot = m + P.BALLAST_G + sum(h[1] for h in hw)
    cgx = (
        m * cg[0]
        + P.BALLAST_G * BALLAST_POS[0]
        + hw[0][1] * P.X_A
        + hw[1][1] * hw[1][2][0]
    ) / m_tot
    cgz = (
        m * cg[2]
        + P.BALLAST_G * BALLAST_POS[1]
        + hw[0][1] * P.Z_A
        + hw[1][1] * hw[1][2][2]
    ) / m_tot
    cgy = (m * cg[1]) / m_tot
    x, z = cgx - P.X_A, cgz - P.Z_A  # offset from the tilt axis in H
    off = math.hypot(x, z)
    tq = {
        str(t): m_tot
        * 1e-3
        * G
        * (x * math.cos(math.radians(t)) - z * math.sin(math.radians(t)))
        * 1e-3
        for t in P.TILT_CHECKS
    }
    worst = max(abs(v) for v in tq.values())
    ok = (
        m_tot <= P.HEAD_MASS_MAX_G
        and off <= P.CG_OFFSET_MAX_MM
        and worst <= P.TORQUE_MAX_NM
    )
    top = sorted(items, key=lambda i: -i[1])[:8]
    chk(
        "C-BAL",
        ok,
        f"head {m_tot:.0f} g (<= {P.HEAD_MASS_MAX_G:.0f}); CG offset from tilt axis x {x:+.1f}, z {z:+.1f} -> {off:.1f} mm (<= {P.CG_OFFSET_MAX_MM}); |static torque| "
        + ", ".join(f"{t} deg {v:+.3f}" for t, v in tq.items())
        + f" N*m (<= {P.TORQUE_MAX_NM}); ballast {P.BALLAST_G:.0f} g; y offset {cgy:+.1f} mm",
        dict(
            mass_g=m_tot,
            cg_x_mm=x,
            cg_z_mm=z,
            cg_y_mm=cgy,
            offset_mm=off,
            torque_nm=tq,
            worst_nm=worst,
            ballast_g=P.BALLAST_G,
            heaviest=[(i[0], round(i[1], 1)) for i in top],
        ),
    )
    return m_tot, (x, z), tq


BALLAST_POS = (-65.0, -29.0)  # L x, z of the lead shot in the ballast cup


# ---------------------------------------------------------------------------------------------------------------------------------
def dock_world(name):
    D = I.DOCKS[name]
    return D


def scan_halfwidth(shape, loc, z, axis, want_inside):
    """Distance from the dock-frame point (0,0,z) along dock `axis` (0=x,1=y) to the first sample where inside-ness != want_inside."""

    def f(t):
        p = [0.0, 0.0, z]
        p[axis] = t
        return ins(shape, pt(loc, p)) == want_inside

    # start inside the wanted state; walk out
    if not f(0.0):
        return None
    t = first_change(f, 0.0, +1, 30, 0.1)
    return t


def holes_ok(shape, loc, D, z, side):
    """Every bolt (u,v) is a void of the right diameter at depth z (dock frame) and has wall material 0.8 mm beyond the hole edge."""
    d = D.male_hole_d if side == "male" else D.female_hole_d
    bad = []
    for u, v in D.bolts:
        if not void(shape, *pt(loc, (u, v, z))):
            bad.append(("no hole", u, v))
            continue
        if void(shape, *pt(loc, (u, v + d / 2 + 0.8, z))):
            bad.append(("no wall", u, v))
    return bad


def c_dock(S):
    out, ok = {}, True
    # D1: two rails (cartridge, male) into two arm slots (cradle, female), both sides
    D = I.DOCKS["D1"]
    hn, hh = D.loc["base_to_face"], D.loc["head_h"]
    zt = hn + hh
    for sy in (1, -1):
        lm, lf = LC.d1_loc(sy), TH.d1_loc(sy)
        assert pt(lm, (0, 0, 0)) == pt(lf, (0, 0, 0))
        zmid = hn + hh * 0.5
        # head width: rail material half-width at mid head height vs void half-width in the arm at the same point
        u = D.loc["length"] / 2
        wm = first_change(
            lambda t: ins(S["cartridge_cage"], pt(lm, (u, t, zmid))), 0.0, +1, 15, 0.1
        )
        wf = first_change(
            lambda t: ins(S["cradle"], pt(lf, (u, t, zmid))), 0.0, +1, 15, 0.1
        )
        cl = wf - wm
        # neck at the arm face plane
        nm = first_change(
            lambda t: ins(S["cartridge_cage"], pt(lm, (u, t, hn - 2.0))),
            0.0,
            +1,
            15,
            0.1,
        )
        nf = first_change(
            lambda t: ins(S["cradle"], pt(lf, (u, t, hn + 0.5))), 0.0, +1, 15, 0.1
        )
        bm = holes_ok(S["cartridge_cage"], lm, D, zt - 3.0, "male")
        bf = holes_ok(S["cradle"], lf, D, zt + 2.0, "female")
        good = abs(cl - D.clear()) <= 0.1 and not bm and not bf
        ok &= good
        out[f"D1{'+' if sy > 0 else '-'}"] = dict(
            head_clear=round(cl, 3),
            pilot_bad=bm,
            clearance_bad=bf,
            head_w=round(2 * wm, 2),
        )
    # D2 bridge (male dowel) / sensor bar (female)
    out["D2"], g2 = dock_flat(
        "D2",
        TH.d2_loc(),
        SH.d2_loc(),
        S["cradle"],
        S["sensor_bar"],
        dowel_z=2.0,
        male_hole_z=-4.0,
        female_hole_z=2.0,
    )
    ok &= g2
    # F1 ring + 4 x M3
    out["F1"], g3 = dock_flat(
        "F1",
        LC.f1_loc(),
        FD.f1_loc(),
        S["cartridge_cage"],
        S["valve_housing"],
        ring=True,
        male_hole_z=-3.0,
        female_hole_z=3.0,
    )
    ok &= g3
    # Y1 on both rotor sides
    for sy in (1, -1):
        part = S["yoke_left"] if sy > 0 else S["yoke_right"]
        lo, lh = (
            I.place((0.0, sy * 60.0, P.Z_ROTOR0 + P.ROTOR_T), (1, 0, 0), (0, 0, 1)),
            TH.y1_loc(sy),
        )
        res, g = dock_flat(
            "Y1",
            lo,
            lh,
            S["rotor_plate"],
            part,
            dowel_z=1.5,
            male_hole_z=-4.0,
            female_hole_z=3.0,
        )
        out[f"Y1{'+' if sy > 0 else '-'}"] = res
        ok &= g
    # H1: panels on two different faces, compared in dock-local coordinates (cutout half widths + bolts)
    res, g = h1_panels(S)
    out["H1"] = res
    ok &= g
    chk("C-DOCK", ok, "; ".join(f"{k}: {v}" for k, v in out.items())[:900], out)


def dock_flat(
    name,
    loc_m,
    loc_f,
    male,
    female,
    dowel_z=None,
    ring=False,
    male_hole_z=-4.0,
    female_hole_z=4.0,
):
    D = I.DOCKS[name]
    same = all(
        abs(a - b) < 1e-6 for a, b in zip(pt(loc_m, (7, 3, 5)), pt(loc_f, (7, 3, 5)))
    )
    bm = holes_ok(male, loc_m, D, male_hole_z, "male")
    bf = holes_ok(female, loc_f, D, female_hole_z, "female")
    cl = None
    if ring:
        zr = D.loc["lip_h"] / 2
        r_in = (P.BORE_ID + D.loc["lip_od"]) / 4  # mid wall
        rm = first_change(
            lambda t: ins(male, pt(loc_m, (t, 0, zr))),
            D.loc["lip_od"] / 2 - 1.0,
            +1,
            6,
            0.05,
        )
        rf = first_change(
            lambda t: ins(female, pt(loc_f, (t, 0, zr))),
            D.loc["lip_od"] / 2 - 1.0,
            +1,
            6,
            0.05,
        )
        cl = rf - rm
    elif dowel_z is not None:
        u, v = D.loc["dowel_uv"]
        rm = first_change(
            lambda t: ins(male, pt(loc_m, (u + t, v, dowel_z))), 0.0, +1, 8, 0.05
        )
        rf = first_change(
            lambda t: ins(female, pt(loc_f, (u + t, v, dowel_z))), 0.0, +1, 8, 0.05
        )
        cl = rf - rm
    good = same and not bm and not bf and (cl is None or abs(cl - D.clear()) <= 0.1)
    return dict(
        datum_same=same,
        clear=None if cl is None else round(cl, 3),
        male_bad=bm,
        female_bad=bf,
    ), good


def h1_panels(S):
    D = I.DOCKS["H1"]
    res, ok = {}, True
    for tag, shape, loc in (
        ("box", S["power_box"], PB.h1_loc()),
        ("yoke", S["yoke_left"], TH.h1_loc()),
    ):
        bad = holes_ok(shape, loc, D, -3.0, "male")
        # cutout half widths at the panel centre
        wx = first_change(
            lambda t: ins(shape, pt(loc, (t, 0.0, -3.0))), 0.0, +1, 12, 0.05
        )
        wy = first_change(
            lambda t: ins(shape, pt(loc, (0.0, t, -3.0))), 0.0, +1, 12, 0.05
        )
        c = (wx - D.loc["conn_w"] / 2, wy - D.loc["conn_h"] / 2)
        good = not bad and abs(c[0] - D.clear()) <= 0.1 and abs(c[1] - D.clear()) <= 0.1
        ok &= good
        res[tag] = dict(clear=[round(c[0], 3), round(c[1], 3)], bolts_bad=bad)
    return res, ok


# ---------------------------------------------------------------------------------------------------------------------------------
def c_path(S, recs):
    """(a) a ball-sized sphere (compressed 0.6 mm: foam) can travel the whole feed path: throat, mouth, pocket at BORE, channel, barrel;
    (b) the head sweeps tilt -15..+70 clear of the yokes, rotor plate and base (boolean volumes)."""
    r = P.BALL_D / 2 - 0.6
    housing, cage, hop = S["valve_housing"], S["cartridge_cage"], S["hopper"]
    plug = S["valve_plug"]
    allsolid = housing + cage + hop + plug

    def clear(x, y, z, s=allsolid):
        return vol(Pos(x, y, z) * Sphere(r), s) < 0.5

    pts = [
        (P.X_T, 0, z) for z in np.arange(P.Z_HOP0 + 10, P.HOUSING_TOP + 14, -6.0)
    ]  # down the throat to the mouth region
    pts += [
        (x, 0, 0) for x in np.arange(P.XV, P.BARREL_X1 - r, 6.0)
    ]  # pocket at BORE, channel through the nip, barrel
    bad = [p for p in pts if not clear(*p)]
    path_ok = not bad
    # swept head vs the pan group / base
    head = None
    for n, r_ in recs.items():
        if r_["group"] != "tilt" or n in ("wheel_hub", "wheel_tyre"):
            continue
        s = S[n]
        head = s if head is None else head + s
    for n in ("wheel_hub", "wheel_tyre"):
        for z in (P.WZ, -P.WZ):
            head = head + Pos(0, 0, z) * S[n]
    worst, where = 0.0, None
    stat = [S["yoke_left"], S["yoke_right"], S["rotor_plate"], S["base_plate"]]
    stat = [(Pos(0, 0, 0) * s) for s in stat]
    angles = [P.TILT_RANGE[0], 0.0, 35.0, P.TILT_RANGE[1]]
    for t in angles:
        hb = Pos(0, 0, P.H_T) * Rot(0, -t, 0) * Pos(-P.X_A, 0, -P.Z_A) * head
        for nm, s in zip(
            ("yoke_left", "yoke_right", "rotor_plate", "base_plate"), stat
        ):
            v = vol(hb, s)
            if v > worst:
                worst, where = v, (nm, t)
    ok = path_ok and worst < 1.0
    chk(
        "C-PATH",
        ok,
        f"ball path spheres clear: {path_ok} ({len(pts)} probes{', blocked at ' + str(bad[:3]) if bad else ''}); swept head vs yokes/rotor/base at {angles} deg: worst overlap {worst:.1f} mm3 {where if worst else ''}",
        dict(blocked=bad[:6], sweep_overlap_mm3=worst, where=where),
    )


def c_fit(recs):
    over = {
        n: r["print"]["size_mm"] for n, r in recs.items() if not r["print"]["fits_bed"]
    }
    ok = not over
    big = max(((max(r["print"]["size_mm"][:2]), n) for n, r in recs.items()))
    chk(
        "C-FIT",
        ok,
        f"every printed part <= {P.PRINT_MAX:.0f} x {P.PRINT_MAX:.0f} x {P.PRINT_MAX:.0f} in some orientation; largest footprint {big[0]:.1f} mm ({big[1]})"
        + (f"; OVER: {over}" if over else ""),
        dict(over=over, largest=big),
    )


def cycle_ms():
    """Command-to-ready model of spec 5.4 with the geometry of THIS design (servo speeds [est], bench item B-TIME)."""
    out = P.PUSH_STROKE / P.PUSHER_MM_PER_MS
    back = (
        P.PUSH_STROKE / P.PUSHER_MM_PER_MS
    )  # rod must retract fully behind the plug before the gate may turn
    gate = P.GATE_SWING_DEG * P.SERVO_S_PER_DEG * 1000.0
    return dict(
        extend=out,
        retract_clear=back,
        gate_to_load=gate,
        fill=P.FILL_MS,
        gate_to_bore=gate,
        total=out + back + 2 * gate + P.FILL_MS,
    )


def c_cycle():
    c = cycle_ms()
    chk(
        "C-CYCLE",
        True,
        "INFO (not a gate): "
        + ", ".join(f"{k} {v:.0f}" for k, v in c.items())
        + f" ms vs spec 5.4 365 ms / R3 400 ms; pusher stroke {P.PUSH_STROKE:.1f} mm vs spec 32",
        c,
    )


def run(S, recs, bought):
    RES.clear()
    c_gap(S)
    c_pocket(S)
    c_feed(S)
    c_nip_interlock(S)
    c_bal(S, recs, bought)
    c_dock(S)
    c_path(S, recs)
    c_fit(recs)
    c_cycle()
    return RES
