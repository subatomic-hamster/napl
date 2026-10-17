"""Dog printed-part parameters (mm, g, degrees unless named). The leg study picks L_THIGH / L_SHANK; everything regenerates from here.

Leg frame (LF, = the abad mount frame): origin on the abduction axis at the outer face of the abad mount flange; x outward along
the abduction axis (forward for a front leg), y outboard, z up. The hip pitch and knee motor axes are parallel to y.
Values marked [est] are design estimates (no vendor source); bought-part sizes come from the part files in parts/.
"""

import math
import os

L_THIGH = float(
    os.environ.get("DOG_L_THIGH", 220.0)
)  # hip pitch axis to knee axis [study range 200-250]
L_SHANK = float(os.environ.get("DOG_L_SHANK", 220.0))  # knee axis to foot centre
TILT_DEG = float(
    os.environ.get("DOG_LIDAR_TILT", 25.0)
)  # Mid-360 nose-down tilt: lower FOV edge -7 deg -> -32 deg (docs/dog/actuator-study.md)

# fab (same print model as cad/cuas-lite: shell + infill, assemblies/README.md "Mass properties")
BED = (220.0, 220.0, 250.0)
NOZZLE, WALLS, INFILL = 0.4, 3, 0.30  # [est] 3 walls / 30 % for load-bearing leg parts
DENSITY = {"pla": 1.24e-3, "petg": 1.27e-3}  # g/mm^3
FLOW_MM3_S, TIME_OVERHEAD = 8.0, 1.35
M3_CLR, M3_INSERT_D, M5_CLR = 3.4, 4.2, 5.3

# bought parts the prints hold
AB_D = 92.4  # GIM8108-8 body 92 mm + 0.4 [steadywin-gim8108-8]
AB_LEN = 55.0  # its body length with the driver
BALL_D = 34.925 + 0.3  # rubber ball seat [mcmaster-natural-rubber-ball-1-3-8in]
LIDAR = (65.0, 65.0, 60.0)  # Mid-360 envelope [livox-mid-360]
BOLT_R = 25.0  # output-flange bolt circle radius, 6 x M3 [est: the vendor drawing was not retrieved]

# --- knee linkage (crank on the knee motor, push rod, lug on the shank; docs/dog/actuator-study.md candidate 1) ---
KNEE_R1 = 20.0  # crank radius [est]: rod force = 22 N m / 0.020 m = 1.1 kN (the rod and rod ends are sized for it)
KNEE_RATIO = float(os.environ.get("DOG_KNEE_RATIO", 1.5))  # study candidate 1
KNEE_R2 = KNEE_R1 * KNEE_RATIO  # shank lug radius
LUG_DEG = 115.0  # lug pin angle from +x (LF), rest pose: arm up and back [chosen by a scan, see linkage()]
ALPHA0_DEG = 51.65  # crank pin angle at q = 0: ONE crank for every leg length (the rod length absorbs L)

# --- leg frame layout (y stack in the thigh frame, origin = hip motor output face) ---
HIP_Y = 60.0  # |y| of the abduction axes on a bulkhead [est]
PITCH_X, PITCH_Y = (
    60.0,
    95.0,
)  # hip pitch axis: forward along the abad axis; y of the hip motor OUTPUT face
THIGH_T = 5.0  # thigh plate thickness, outboard of the hip motor output face
SHANK_W = 10.0  # shank beam thickness
LUG_T = 3.0  # lug block thickness beyond the beam, each face
KNEE_Y = (
    THIGH_T + LUG_T + SHANK_W / 2
)  # knee axis y: the inner lug touches the thigh plate
ROD_Y = (
    KNEE_Y + SHANK_W / 2 + LUG_T + 4.0
)  # rod-end centre y: the 8 mm rod end sits outside the outer lug
CRANK_T = 7.0  # crank plate thickness
SLAB = ROD_Y + 4.0 + 1.0 + CRANK_T  # hip output face -> knee motor output face
KNEE_FACE = PITCH_Y + SLAB  # y (LF) of the knee motor output face

# --- body ---
BODY_L = 420.0  # inner length between the bulkheads
BULK_X = BODY_L / 2 + 3.0  # bulkhead centre x (body frame)
BULK_W, BULK_H, BULK_T = 224.0, 120.0, 6.0  # bulkhead [est]
HIP_BOLT_Y, HIP_BOLT_Z = 40.0, 45.0  # abad mount <-> bulkhead: 4 x M3
HIP_FLANGE = (
    52.0  # half-size of the abad mount flange (covers the 80 x 90 bolt pattern) [est]
)


def linkage(L):
    """Planar four-bar (crank on the hip axis, rod, lug on the shank), thigh frame (x, z), hip axis at the origin.
    Crank angle a (from +x) and lug angle b = LUG_DEG - q; q = knee angle, negative = flexion (front leg).
    Returns P (lug pin), C0 (crank pin at q = 0), ell (rod pin length), tilt_deg, mid (rod centre), knee_range = (lo, hi) rad where
    both transmission angles stay in [30, 150] deg, alpha0_deg (crank rest angle)."""
    b0 = math.radians(LUG_DEG)
    a_r = math.radians(ALPHA0_DEG)
    ell = math.hypot(
        KNEE_R2 * math.cos(b0) - KNEE_R1 * math.cos(a_r),
        -L + KNEE_R2 * math.sin(b0) - KNEE_R1 * math.sin(a_r),
    )

    def solve(q, prev):
        b = b0 - q
        px, pz = KNEE_R2 * math.cos(b), -L + KNEE_R2 * math.sin(b)
        d = math.hypot(px, pz)
        c = (KNEE_R1**2 + d**2 - ell**2) / (2 * KNEE_R1 * d)
        if abs(c) > 1:
            return None
        phi = math.atan2(pz, px)
        cands = [phi + math.acos(c), phi - math.acos(c)]
        a = min(
            cands, key=lambda s: abs((s - prev + math.pi) % (2 * math.pi) - math.pi)
        )
        a = prev + ((a - prev + math.pi) % (2 * math.pi) - math.pi)
        cx, cz = KNEE_R1 * math.cos(a), KNEE_R1 * math.sin(a)
        rx, rz = px - cx, pz - cz
        t_c = math.degrees(
            math.acos(max(-1, min(1, (rx * math.cos(a) + rz * math.sin(a)) / ell)))
        )
        t_f = math.degrees(
            math.acos(max(-1, min(1, -(rx * math.cos(b) + rz * math.sin(b)) / ell)))
        )
        return a, t_c, t_f

    def ok(r):
        return r is not None and all(30 <= t <= 150 for t in (r[1], r[2]))

    a0 = solve(0.0, math.radians(50))[0]
    px, pz = KNEE_R2 * math.cos(b0), -L + KNEE_R2 * math.sin(b0)
    cx, cz = KNEE_R1 * math.cos(a0), KNEE_R1 * math.sin(a0)
    lims = []
    for sgn in (-1, 1):
        prev, last = a0, 0.0
        for i in range(1, 400):
            r = solve(sgn * i * 0.005, prev)
            if not ok(r):
                break
            last, prev = sgn * i * 0.005, r[0]
        lims.append(last)
    ux, uz = cx - px, cz - pz
    return dict(
        P=(px, pz),
        C0=(cx, cz),
        ell=math.hypot(ux, uz),
        tilt_deg=math.degrees(math.atan2(ux, uz)),
        mid=((px + cx) / 2, (pz + cz) / 2),
        knee_range=(lims[0], lims[1]),
        alpha0_deg=math.degrees(a0),
    )
