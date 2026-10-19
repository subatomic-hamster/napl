#!/usr/bin/env python3
"""Hand-calc structural check of the dog leg (docs/dog/leg-survey.md): the printed PLA leg against the "-al" leg (hollow 6061 plate-shell thigh, 25 x 23
carbon tube shank). Stdlib only. Run: python3 tools/dog_leg_struct.py [--md]

Model: planar stance (the thigh and shank symmetric about the vertical, half-knee angle THETA, foot under the hip, a front leg: knee back),
statics of the lower leg in the THIGH frame (x forward, y outboard, z up, hip axis at the origin). Loads at the foot centre:
  a) 3 x body weight, vertical   b) 1 x bw, lateral (along y)   c) 1 x bw, fore-aft (along x)
The knee push rod's force P (along the rod, lever from the real four-bar of cad/dog/params.py) balances the foot force's moment about the knee
axis; it acts on the shank head and its reaction goes to the crank (the housing), so the thigh sees R = F + P u at the knee pin. A section at
distance s from the hip carries the lower body's force and moment about the section point: axial, shear, two bending moments and torsion are
the components along / across the link axis. Stresses are linear-elastic: sigma = N/A + Mb_sag/Z_sag + Mb_lat/Z_lat, tau = T/W_t (+ V/A), von Mises
sqrt(sigma^2 + 3 tau^2) against the allowable of the material. Not modelled: stress concentrations at holes and bolts, fatigue, joint slack,
the actuator output bearing's tilting-moment rating (not published), the abduction angle (the lateral case is taken at 0 deg abduction).
"""

import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "cad", "dog"))
import params as P

G = 9.81
MASS = 11.653  # kg, default dog.toml build report (assemblies/dog/README.md): the load cases below; each leg scales them by its own dog mass (LEGS mass_kg)
W = MASS * G  # N, body weight
THETA = math.radians(
    35.0
)  # half-knee angle: thigh and shank 35 deg off the vertical, knee angle q = -70 deg (inside the linkage range)
L = P.L_THIGH * 1e-3
LS = P.L_SHANK * 1e-3
CASES = {  # name -> foot force in the world frame (x fwd, y left, z up), N
    "a vertical 3 bw": (0.0, 0.0, 3 * W),
    "b lateral 1 bw": (0.0, W, 0.0),
    "c fore-aft 1 bw": (W, 0.0, 0.0),
}
ACT_HIP, ACT_KNEE = (
    22.0,
    33.0,
)  # N m: GIM8108-8 peak (datasheet) and x 1.5 at the knee (the linkage ratio)

# --- materials (Pa) -------------------------------------------------------------------------------------------------------------------
MAT = {
    # PLA 30 % infill: E = 2.0 GPa (Prusament PLA, solid 2.3 GPa; thin walls + 30 % infill taken 13 % lower); strength 25 MPa = mid of the 20.8-32.4 MPa
    # of 30 % infill dog-bones in the literature search (solid Prusament PLA yield 51 MPa), i.e. conservative for layer-direction loads.
    "pla": dict(E=2.0e9, G=0.77e9, sig=25e6, tau=25e6 / math.sqrt(3), bear=50e6),
    # 6061-T6 (MatWeb / ASM): E 68.9 GPa, yield 276 MPa, shear 207 MPa, bearing yield 386 MPa
    "al": dict(E=68.9e9, G=26e9, sig=276e6, tau=207e6, bear=386e6),
    # roll-wrapped CF tube 25 x 23 (Easy Composites): tensile 650 MPa, E 64 GPa; compressive / flexural strength not published -> 0.6 x 650 [est]
    "cf": dict(E=64e9, G=5e9, sig=390e6, tau=40e6, bear=100e6),
}


# --- section properties (mm -> m inside) ------------------------------------------------------------------------------------------------
def rects(rs):
    """Composite of rectangles (x0, x1, y0, y1) in mm in the (x, y) section plane -> A, I about x (bends in y: LATERAL), I about y (SAGITTAL),
    Z_lat, Z_sag (mm units) and the open-section torsion constant sum b t^3 / 3."""
    A = sum((x1 - x0) * (y1 - y0) for x0, x1, y0, y1 in rs)
    xc = sum((x1 - x0) * (y1 - y0) * (x0 + x1) / 2 for x0, x1, y0, y1 in rs) / A
    yc = sum((x1 - x0) * (y1 - y0) * (y0 + y1) / 2 for x0, x1, y0, y1 in rs) / A
    Ix = sum(
        (x1 - x0) * (y1 - y0) ** 3 / 12
        + (x1 - x0) * (y1 - y0) * ((y0 + y1) / 2 - yc) ** 2
        for x0, x1, y0, y1 in rs
    )
    Iy = sum(
        (y1 - y0) * (x1 - x0) ** 3 / 12
        + (x1 - x0) * (y1 - y0) * ((x0 + x1) / 2 - xc) ** 2
        for x0, x1, y0, y1 in rs
    )
    cy = max(max(y1 - yc, yc - y0) for _, _, y0, y1 in rs)
    cx = max(max(x1 - xc, xc - x0) for x0, x1, _, _ in rs)
    J = sum(
        max(x1 - x0, y1 - y0) * min(x1 - x0, y1 - y0) ** 3 / 3 for x0, x1, y0, y1 in rs
    )
    tmax = max(min(x1 - x0, y1 - y0) for x0, x1, y0, y1 in rs)
    return dict(A=A, Ilat=Ix, Isag=Iy, Zlat=Ix / cy, Zsag=Iy / cx, J=J, Wt=J / tmax)


def hollow_rect(w, t, wi, ti):
    return dict(
        A=w * t - wi * ti,
        Ilat=(w * t**3 - wi * ti**3) / 12,
        Isag=(t * w**3 - ti * wi**3) / 12,
        Zlat=(w * t**3 - wi * ti**3) / 12 / (t / 2),
        Zsag=(t * w**3 - ti * wi**3) / 12 / (w / 2),
        J=0.0,
        Wt=1e9,  # torsion of the shank is zero by construction (foot on the axis); the entry only avoids a divide by zero
    )


def tube(od, idm):
    I = math.pi / 64 * (od**4 - idm**4)
    return dict(
        A=math.pi / 4 * (od**2 - idm**2),
        Ilat=I,
        Isag=I,
        Zlat=I / (od / 2),
        Zsag=I / (od / 2),
        J=2 * I,
        Wt=2 * I / (od / 2),
    )


THIGH_PLA = rects(
    [(-12, 12, 0, 5), (-3, 3, 5, 12)]
)  # 24 x 5 plate + 6 x 7 outboard rib (cad/dog/parts.py thigh_bracket)
SHANK_PLA = hollow_rect(
    22, 10, 12, 6
)  # 22 x 10 beam with the enclosed 12 x 6 lightening void (shank), section at the slot


def plate_box(w, tp=P.AL_PLATE, Y=P.AL_Y, g_ratio=0.11):
    """Hollow thigh shell at one station: two 6061 plates (w x tp) at y [0, tp] and [Y - tp, Y]; the printed PA12-CF bands are NOT counted in
    A / I (conservative) and only close the torsion cell (shear modulus ratio PA12-CF / 6061 = 0.11 [est: 2.9 / 26 GPa]); Bredt: W_t = 2 A_m t_p."""
    s = rects([(-w / 2, w / 2, 0, tp), (-w / 2, w / 2, Y - tp, Y)])
    Am = w * (Y - tp)
    path = 2 * w / tp + 2 * (Y - tp) / (P.AL_BAND * g_ratio)
    s["J"] = 4 * Am**2 / path
    s["Wt"] = 2 * Am * tp
    return s


# the shell at s = 60 mm (hull width 60 mm, plates + bands) and the hip station just below the lowest actuator bolt row (s = 21.6 mm): the
# inboard plate + 3/16 in doubler only (the outboard plate starts at AL_TOP = 38), net of the two M3 holes on the bolt circle
HIP_NET_W = 60.0 - 2 * P.M3_CLR
THIGH_AL_SHELL = plate_box(60.0)
THIGH_AL_HIP = rects([(-HIP_NET_W / 2, HIP_NET_W / 2, 0, P.AL_PLATE + P.AL_HIP_PAD)])
SHANK_CF = tube(P.SH_OD, P.SH_ID)

LEGS = {
    "printed PLA leg": dict(
        thigh=THIGH_PLA,
        shank=SHANK_PLA,
        mt="pla",
        mass_kg=11.653,  # dog.toml
        ms="pla",
        s_thigh=30.0,
        s_shank=30.0,
        span=(2.5, 13.0),
        pin_d=5.3,
        pin_len=(5.0, 10.0),
        pin_yield=640e6,
    ),
    "'-al' leg": dict(
        thigh=THIGH_AL_SHELL,
        thigh_secs=[
            ("hip, s = 21.6", 21.6, THIGH_AL_HIP),
            ("shell, s = 60", 60.0, THIGH_AL_SHELL),
            ("shell, s = 150", 150.0, THIGH_AL_SHELL),
        ],
        shank=SHANK_CF,
        mt="al",
        mass_kg=12.46,  # dog-al.toml build report
        ms="cf",
        s_thigh=60.0,
        s_shank=8.0,
        span=(-(P.AL_PAD_T - P.AL_PLATE) / 2, P.AL_Y + (P.AL_PAD_T - P.AL_PLATE) / 2),
        pin_d=P.KNEE_PIN_D,
        pin_len=(P.AL_PLATE + P.AL_PAD_T, P.SH_BOSS_W),
        pin_arm=P.AL_Y / 2 - P.SH_BOSS_W / 2 + (P.AL_PLATE + P.AL_PAD_T) / 2,
        pin_yield=640e6,
        plate_local=(P.AL_PLATE + P.AL_PAD_T, 40.0, 66.0),
    ),
}


# --- kinematics: four-bar solve (same as cad/dog/params.py linkage) ---------------------------------------------------------------------
def rod_geometry(q):
    """Thigh frame (x, z), hip at the origin, thigh down -z: lug pin, crank pin, rod direction u (lug -> crank) at knee angle q (rad)."""
    ell = P.linkage(L * 1e3)["ell"]
    b = math.radians(P.LUG_DEG) - q
    px, pz = P.KNEE_R2 * math.cos(b), -L * 1e3 + P.KNEE_R2 * math.sin(b)
    d = math.hypot(px, pz)
    c = (P.KNEE_R1**2 + d**2 - ell**2) / (2 * P.KNEE_R1 * d)
    phi = math.atan2(pz, px)
    a = phi + math.acos(
        max(-1.0, min(1.0, c))
    )  # the branch that matches the build pose (crank angle 51.65 deg at q = 0)
    cx, cz = P.KNEE_R1 * math.cos(a), P.KNEE_R1 * math.sin(a)
    ux, uz = cx - px, cz - pz
    n = math.hypot(ux, uz)
    return (px * 1e-3, pz * 1e-3), (cx * 1e-3, cz * 1e-3), (ux / n, uz / n)


def cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def statics(F_world, ys_lug):
    """-> dict of the posture and loads. F_world = foot force on the leg (world xyz); ys_lug = y offset of the rod pin from the knee axis (m)."""
    q = -2 * THETA
    phi = -THETA  # thigh frame rotation about y (CCW in the x-z plane), the thigh axis points at -90 deg + phi
    cp, sp = math.cos(phi), math.sin(phi)
    # world -> thigh frame: x' = x cos(phi) + z sin(phi), z' = -x sin(phi) + z cos(phi)
    F = (
        F_world[0] * cp + F_world[2] * sp,
        F_world[1],
        -F_world[0] * sp + F_world[2] * cp,
    )
    knee = (0.0, 0.0, -L)
    sa = -math.pi / 2 - q  # shank axis angle in the thigh frame
    s_ax = (math.cos(sa), 0.0, math.sin(sa))
    foot = (knee[0] + LS * s_ax[0], 0.0, knee[2] + LS * s_ax[2])
    (lx, lz), (cx, cz), (ux, uz) = rod_geometry(q)
    lug = (lx, ys_lug, lz)
    u = (ux, 0.0, uz)
    # rod force P u on the shank at the lug balances the y-moment of the foot force about the knee axis
    my_foot = cross(tuple(foot[i] - knee[i] for i in range(3)), F)[1]
    lever = cross(tuple(lug[i] - knee[i] for i in range(3)), u)[1]
    P_rod = -my_foot / lever
    return dict(
        F=F, knee=knee, foot=foot, lug=lug, u=u, P=P_rod, s_ax=s_ax, lever=lever
    )


def section(st, s_along, link):
    """Loads at distance s_along (m) from the hip along the thigh (link = 'thigh') or from the knee along the shank ('shank').
    -> N (compression +), Vx, Vy shear, Mb_sag (about y), Mb_lat (about the in-plane transverse axis), T (about the axis)."""
    F, P_rod, u = st["F"], st["P"], st["u"]
    if link == "thigh":
        a = (0.0, 0.0, -1.0)
        pt = (0.0, 0.0, -s_along)
        Fr = tuple(
            F[i] + P_rod * u[i] for i in range(3)
        )  # foot force + rod force on the lower body
        M = tuple(
            c1 + c2
            for c1, c2 in zip(
                cross(tuple(st["foot"][i] - pt[i] for i in range(3)), F),
                cross(
                    tuple(st["lug"][i] - pt[i] for i in range(3)),
                    tuple(P_rod * x for x in u),
                ),
            )
        )
    else:  # the shank tube / beam sits below the head: foot force only
        a = st["s_ax"]
        pt = tuple(st["knee"][i] + s_along * a[i] for i in range(3))
        Fr = F
        M = cross(tuple(st["foot"][i] - pt[i] for i in range(3)), F)
    n = (-a[2], 0.0, a[0])  # in-plane transverse unit vector
    return dict(
        N=abs(dot(Fr, a)),
        V=abs(dot(Fr, n)),
        Vy=abs(Fr[1]),
        Msag=abs(M[1]),
        Mlat=abs(dot(M, n)),
        T=abs(dot(M, a)),
        Fr=Fr,
    )


def stress(sec, prop, mat):
    s_ax = sec["N"] / (prop["A"] * 1e-6)
    s_b = sec["Msag"] / (prop["Zsag"] * 1e-9) + sec["Mlat"] / (prop["Zlat"] * 1e-9)
    tau = (
        sec["T"] / (prop["Wt"] * 1e-9)
        + math.hypot(sec["V"], sec["Vy"]) / (prop["A"] * 1e-6) * 1.5
    )
    vm = math.sqrt((s_ax + s_b) ** 2 + 3 * tau**2)
    return vm, s_ax + s_b, tau, MAT[mat]["sig"] / vm


def lateral_deflection(prop_t, mt, prop_s, ms, Fy):
    """Foot deflection along y from bending + torsion of the thigh and the bending of the shank, virtual work with the unit foot load. Joint slack ignored."""
    st = statics((0, 1.0, 0), 0.0)
    foot = st["foot"]
    c = -foot[
        2
    ]  # projection of the foot on the thigh axis (thigh axis points -z in the thigh frame)
    p = abs(foot[0])  # perpendicular offset of the foot from the thigh axis
    E, Gm = MAT[mt]["E"], MAT[mt]["G"]
    d_bend = (c * c * L - c * L * L + L**3 / 3) / (E * prop_t["Ilat"] * 1e-12)
    d_tors = p * p * L / (Gm * prop_t["J"] * 1e-12)
    d_sh = LS**3 / (3 * MAT[ms]["E"] * prop_s["Ilat"] * 1e-12)
    return Fy * (d_bend + d_tors + d_sh), Fy * d_bend, Fy * d_tors, Fy * d_sh


def euler(prop, mat, length):
    return (
        math.pi**2 * MAT[mat]["E"] * min(prop["Ilat"], prop["Isag"]) * 1e-12 / length**2
    )


def run():
    rows = []
    for name, leg in LEGS.items():
        ys = (
            P.ROD_Y - 13.0 if "PLA" in name else P.ROD_Y - P.KNEE_Y_AL
        ) * 1e-3  # rod pin y offset from the knee axis
        k = (
            leg["mass_kg"] / MASS
        )  # this leg's dog is heavier (dog-al.toml 12.46 kg): its loads are scaled
        for case, Fw0 in CASES.items():
            Fw = tuple(k * c for c in Fw0)
            st = statics(Fw, ys)
            secs = leg.get("thigh_secs") or [("beam", leg["s_thigh"], leg["thigh"])]
            cand = []
            for label, s_mm, prop in secs:
                th_i = section(st, s_mm * 1e-3, "thigh")
                cand.append((stress(th_i, prop, leg["mt"]), th_i, label))
            vt, th, label = min(cand, key=lambda c: c[0][3])  # the worst station
            sh = section(st, leg["s_shank"] * 1e-3, "shank")
            vs = stress(sh, leg["shank"], leg["ms"])
            rows.append(
                dict(
                    leg=name,
                    case=case,
                    P=st["P"],
                    hip=section(st, 0.0, "thigh")["Msag"],
                    knee=abs(st["P"] * st["lever"]),
                    thigh=th,
                    shank=sh,
                    vt=vt,
                    vs=vs,
                    station=label,
                    stations={c[2]: c[0][3] for c in cand},
                )
            )
        # torsion case d: the thigh torsion of case b is reported through rows[...]["thigh"]["T"]
        # knee pin: the lateral foot force's moment about the knee is a couple on the pin
        st = statics(tuple(k * c for c in CASES["b lateral 1 bw"]), ys)
        M = k * abs(CASES["b lateral 1 bw"][1]) * LS
        arm = leg["span"][1] - leg["span"][0]
        Fb = M / (arm * 1e-3)
        thigh_len, shank_len = leg["pin_len"]
        bear = max(
            Fb / (leg["pin_d"] * 1e-3 * thigh_len * 1e-3),
            Fb / (leg["pin_d"] * 1e-3 * shank_len * 1e-3),
        )
        arm_pin = leg.get("pin_arm", min(thigh_len, shank_len) / 2)
        Mpin = (
            Fb * arm_pin * 1e-3
        )  # pin bending: couple force acting over pin_arm (default half the thinner engagement)
        sig_pin = Mpin / (math.pi * (leg["pin_d"] * 1e-3) ** 3 / 32)
        row = dict(
            leg=name,
            case="knee pin (lateral couple)",
            Fb=Fb,
            bear=bear,
            bear_allow=MAT["pla" if "PLA" in name else "al"]["bear"],
            sig_pin=sig_pin,
            pin_yield=leg["pin_yield"],
        )
        if "plate_local" in leg:
            # plate + pad stack (t) as a strip of width b between the bands (span a), point load Fb at mid-span: sigma = 6 (Fb a / 4) / (b t^2).
            # Needs the pad bonded / riveted to the plate (laminate); unbonded the plate alone (AL_PLATE) is t_alone.
            t, b, a = (v * 1e-3 for v in leg["plate_local"])
            row["plate_sig"] = 6 * (Fb * a / 4) / (b * t**2)
            row["plate_sig_unbonded"] = (
                6 * (Fb * a / 4) / (b * (P.AL_PLATE * 1e-3) ** 2)
            )
            row["plate_allow"] = MAT["al"]["sig"]
        rows.append(row)
    return rows


def fmt_table(rows):
    out = []
    for r in rows:
        if "Fb" in r:
            out.append(
                f"{r['leg']:16s} {r['case']:26s} couple {r['Fb']:7.0f} N  bearing {r['bear'] / 1e6:6.1f} MPa (allow {r['bear_allow'] / 1e6:.0f})"
                f"  pin bending {r['sig_pin'] / 1e6:6.0f} MPa (pin yield {r['pin_yield'] / 1e6:.0f})"
                + (
                    f"  plate local bending {r['plate_sig'] / 1e6:5.0f} MPa bonded pad / {r['plate_sig_unbonded'] / 1e6:5.0f} MPa plate alone (yield {r['plate_allow'] / 1e6:.0f})"
                    if "plate_sig" in r
                    else ""
                )
            )
            continue
        th, sh, vt, vs = r["thigh"], r["shank"], r["vt"], r["vs"]
        out.append(
            f"{r['leg']:16s} {r['case']:26s} P {r['P']:7.0f} N  hip M {r['hip']:5.1f}  | thigh N {th['N']:6.0f} Msag {th['Msag']:5.1f} Mlat {th['Mlat']:5.1f} T {th['T']:5.1f}"
            f" -> vM {vt[0] / 1e6:6.1f} MPa SF {vt[3]:5.2f} | shank Msag {sh['Msag']:5.1f} Mlat {sh['Mlat']:5.1f} -> vM {vs[0] / 1e6:6.1f} MPa SF {vs[3]:5.2f}"
        )
    return "\n".join(out)


if __name__ == "__main__":
    print(
        f"W = {W:.1f} N, theta = {math.degrees(THETA):.0f} deg, L = {L * 1e3:.0f} mm; actuator limits hip {ACT_HIP} / knee {ACT_KNEE} N m"
    )
    rows = run()
    print(fmt_table(rows))
    for name, leg in LEGS.items():
        d = lateral_deflection(
            leg["thigh"], leg["mt"], leg["shank"], leg["ms"], W * leg["mass_kg"] / MASS
        )
        pc = euler(leg["thigh"], leg["mt"], L)
        print(
            f"{name}: lateral foot deflection at 1 bw {d[0] * 1e3:.1f} mm (thigh bending {d[1] * 1e3:.1f}, thigh torsion {d[2] * 1e3:.1f}, shank {d[3] * 1e3:.1f}); "
            f"thigh Euler (pinned) {pc:.0f} N"
        )
