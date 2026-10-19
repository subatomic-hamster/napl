#!/usr/bin/env python3
"""Hand-calc structural check of the dog leg (docs/dog/leg-survey.md): the printed PLA leg against the "-al" leg (6061 box-tube thigh, carbon
tube shank). Stdlib only. Run: python3 tools/dog_leg_struct.py [--md]

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
MASS = 11.653  # kg, build report (assemblies/dog/README.md)
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
    # roll-wrapped CF tube 20 x 18 (Easy Composites): tensile 650 MPa, E 64 GPa; compressive / flexural strength not published -> 0.6 x 650 [est]
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


def box_tube(w, t, wall):
    """Closed rectangular tube w (x) x t (y), wall: A, I, Z and Bredt torsion constant J = 4 A_m^2 s / perimeter, W_t = 2 A_m s."""
    wi, ti = w - 2 * wall, t - 2 * wall
    Am = (w - wall) * (t - wall)
    per = 2 * ((w - wall) + (t - wall))
    return dict(
        A=w * t - wi * ti,
        Ilat=(w * t**3 - wi * ti**3) / 12,
        Isag=(t * w**3 - ti * wi**3) / 12,
        Zlat=(w * t**3 - wi * ti**3) / 12 / (t / 2),
        Zsag=(t * w**3 - ti * wi**3) / 12 / (w / 2),
        J=4 * Am**2 * wall / per,
        Wt=2 * Am * wall,
    )


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
THIGH_AL = box_tube(P.AL_W, P.AL_T, P.AL_WALL)
SHANK_CF = tube(P.SH_OD, P.SH_ID)

LEGS = {
    "printed PLA leg": dict(
        thigh=THIGH_PLA,
        shank=SHANK_PLA,
        mt="pla",
        ms="pla",
        s_thigh=30.0,
        s_shank=30.0,
        span=(2.5, 13.0),
        pin_d=5.3,
        pin_len=(5.0, 10.0),
        pin_yield=640e6,
    ),
    "'-al' leg": dict(
        thigh=THIGH_AL,
        shank=SHANK_CF,
        mt="al",
        ms="cf",
        s_thigh=P.AL_TOP,
        s_shank=8.0,
        span=(P.AL_T / 2, P.KNEE_Y_AL),
        pin_d=P.KNEE_PIN_D,
        pin_len=(P.AL_T, P.SH_OD),
        pin_yield=640e6,
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
        for case, Fw in CASES.items():
            st = statics(Fw, ys)
            th = section(st, leg["s_thigh"] * 1e-3, "thigh")
            sh = section(st, leg["s_shank"] * 1e-3, "shank")
            vt = stress(th, leg["thigh"], leg["mt"])
            vs = stress(sh, leg["shank"], leg["ms"])
            rows.append(
                dict(
                    leg=name,
                    case=case,
                    P=st["P"],
                    hip=th["Msag"],
                    knee=abs(st["P"] * st["lever"]),
                    thigh=th,
                    shank=sh,
                    vt=vt,
                    vs=vs,
                )
            )
        # torsion case d: the thigh torsion of case b is reported through rows[...]["thigh"]["T"]
        # knee pin: the lateral foot force's moment about the knee is a couple on the pin
        st = statics(CASES["b lateral 1 bw"], ys)
        M = abs(CASES["b lateral 1 bw"][1]) * LS
        arm = leg["span"][1] - leg["span"][0]
        Fb = M / (arm * 1e-3)
        thigh_len, shank_len = leg["pin_len"]
        bear = max(
            Fb / (leg["pin_d"] * 1e-3 * thigh_len * 1e-3),
            Fb / (leg["pin_d"] * 1e-3 * shank_len * 1e-3),
        )
        Mpin = (
            Fb * (min(thigh_len, shank_len) / 2) * 1e-3
        )  # pin bending: couple force acting over half the thinner engagement
        sig_pin = Mpin / (math.pi * (leg["pin_d"] * 1e-3) ** 3 / 32)
        rows.append(
            dict(
                leg=name,
                case="knee pin (lateral couple)",
                Fb=Fb,
                bear=bear,
                bear_allow=MAT["pla" if "PLA" in name else "al"]["bear"],
                sig_pin=sig_pin,
                pin_yield=leg["pin_yield"],
            )
        )
    return rows


def fmt_table(rows):
    out = []
    for r in rows:
        if "Fb" in r:
            out.append(
                f"{r['leg']:16s} {r['case']:26s} couple {r['Fb']:7.0f} N  bearing {r['bear'] / 1e6:6.1f} MPa (allow {r['bear_allow'] / 1e6:.0f})"
                f"  pin bending {r['sig_pin'] / 1e6:6.0f} MPa (pin yield {r['pin_yield'] / 1e6:.0f})"
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
        d = lateral_deflection(leg["thigh"], leg["mt"], leg["shank"], leg["ms"], W)
        pc = euler(leg["thigh"], leg["mt"], L)
        print(
            f"{name}: lateral foot deflection at 1 bw {d[0] * 1e3:.1f} mm (thigh bending {d[1] * 1e3:.1f}, thigh torsion {d[2] * 1e3:.1f}, shank {d[3] * 1e3:.1f}); "
            f"thigh Euler (pinned) {pc:.0f} N"
        )
