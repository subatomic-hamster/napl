"""cuas-lite CAD parameters: the ONE place every dimension lives (mm, g, degrees unless named otherwise).

Values marked [est] are estimates/typical-catalogue numbers to confirm at purchase (spec section 6.4/6.5); [src] come from the spec.
Frames (spec 5.1): B = base frame (origin on the pan axis at the top face of the stator plate, x forward, z up). H = head frame
(origin on the tilt axis, x along the barrel, z up). Parts are modelled in their assembly frame at pan = 0, tilt = 0.
"""

import math
import os

# ---- ball (single source of truth: the user will measure real balls; every derived dimension below regenerates) -------------
BALL_D = float(
    os.environ.get("CUAS_BALL_D", 22.2)
)  # mm (env override only to try other balls, e.g. CUAS_BALL_D=17.8)  [src spec 0: Rival round 7/8 in]; brief had 17.8
BALL_M = float(os.environ.get("CUAS_BALL_M", 1.86))  # g
BALL_CLEAR = 1.8  # barrel/bore diametral clearance [spec 6.5]
COMPRESSION = 3.0  # mm squeeze between flywheels [est]
BARREL_ID = BALL_D + BALL_CLEAR
FW_GAP = BALL_D - COMPRESSION  # flywheel rim-to-rim gap
HOPPER_BALLS = 40
PACKING = 0.6  # random loose packing of spheres [est]

# ---- fab ---------------------------------------------------------------------------------------------------------------------
BED = (220.0, 220.0, 250.0)
BED_MARGIN = 10.0  # spec 6.4: max footprint 200 x 200
TOL_HOLE = 0.2  # added to printed hole diameters
TOL_SLIP = 0.3  # sliding fits (piston in bore, bearing seat)
NOZZLE, LAYER, WALLS, INFILL = 0.4, 0.2, 2, 0.20  # spec 6.4
DENSITY = {"PLA": 1.24e-3, "PETG": 1.27e-3}  # g/mm^3  [spec 6.4, PETG est]
FLOW_MM3_S = 8.0  # volumetric print speed [est]; time = V_filament / flow * OVERHEAD
TIME_OVERHEAD = 1.35
COST_PER_KG = 20.0  # USD [spec 6.6]

# ---- fasteners ---------------------------------------------------------------------------------------------------------------
M3_CLR = 3.4
M3_INSERT_D, M3_INSERT_DEPTH = 4.2, 6.0  # heat-set insert hole (insert 5.7 long) [est]
M3_NUT_AF, M3_NUT_H = 5.5 + 0.3, 2.4 + 0.2
M3_HEAD_D, M3_CSK_D = 5.7 + 0.4, 6.4
M4_CLR, M4_INSERT_D, M4_INSERT_DEPTH = 4.5, 5.6, 8.0
M2_CLR = 2.4
UNC_QTR_CLR = 6.6  # 1/4-20 tripod screw clearance

# ---- GT2 belts / pulleys (spec 6.1) --------------------------------------------------------------------------------------------
PITCH = 2.0
T_MOTOR, T_PAN, T_TILT = 20, 100, 80
BELT_W = 6.0
PAN_RATIO, TILT_RATIO = T_PAN / T_MOTOR, T_TILT / T_MOTOR  # 5, 4
PLD = 0.762  # GT2 pitch-line offset: OD = pitch dia - 2*0.254*... (OD = PD - 0.762)
BELT_TEETH = {
    "pan": 126,
    "tilt": 102,
}  # closed loops; stock length to be verified at purchase (spec 6.1)
TOOTH_R, TOOTH_DEPTH = (
    0.75,
    0.75,
)  # printed groove: circle cut radius / depth [est, 0.4 nozzle]
FLANGE_T, FLANGE_OVER = (
    1.5,
    1.6,
)  # belt flange thickness / radial overhang past teeth OD


def pitch_d(teeth):
    return teeth * PITCH / math.pi


def pulley_od(teeth):
    return pitch_d(teeth) - PLD


def centre_distance(belt_teeth, t1, t2):
    """Exact open-belt centre distance for a closed loop of `belt_teeth` teeth (bisection on the belt length)."""
    L = belt_teeth * PITCH
    r1, r2 = pitch_d(t1) / 2, pitch_d(t2) / 2
    R, r = max(r1, r2), min(r1, r2)

    def length(c):
        a = math.asin((R - r) / c)
        return (
            2 * math.sqrt(c * c - (R - r) ** 2)
            + R * (math.pi + 2 * a)
            + r * (math.pi - 2 * a)
        )

    lo, hi = R - r + 1e-6, L
    for _ in range(100):
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if length(mid) < L else (lo, mid)
    return lo


C_PAN = centre_distance(BELT_TEETH["pan"], T_MOTOR, T_PAN)
C_TILT = centre_distance(BELT_TEETH["tilt"], T_MOTOR, T_TILT)
SLOT_TRAVEL = 4.0  # belt tension adjustment +-4 mm [spec 6.1]

# ---- NEMA 17 (parts/actuators/soyo-sy42sth38-1684a-nema17.toml) ------------------------------------------------------------------
N17_FACE, N17_BOLT, N17_PILOT = 42.3, 31.0, 22.0
N17_LEN, N17_SHAFT_D, N17_SHAFT_LEN, N17_BOSS_H = (
    40.0,
    5.0,
    24.0,
    2.0,
)  # body length 40 [17HS4401S part toml] / shaft len/boss [est typical]
N17_MASS = 285.0

# ---- bearings ----------------------------------------------------------------------------------------------------------------------
B608_OD, B608_ID, B608_W = 22.0, 8.0, 7.0
B608_MASS = 12.0
LS_OD = 152.4  # 6 in lazy susan
LS_BOLT_R = 63.5  # bolt circle radius, 4 holes per ring, rings offset 45 deg [est: confirm at purchase]
LS_H = 9.0  # bearing height [est]
LS_APERTURE = 117.5  # clear opening [RF Parts unit, bom.md]
LS_MASS = 218.0  # [RF Parts unit, bom.md]

# ---- 180-size flywheel motors + wheels ----------------------------------------------------------------------------------------
M180_FLAT = 15.3  # can across the flats [part toml]
M180_D, M180_LEN, M180_SHAFT_D, M180_SHAFT_LEN, M180_MASS = (
    20.0,
    32.0,
    2.0,
    8.0,
    18.0,
)  # can length 32 [part toml / listing]; rest [est typical 180-size]
WHEEL_D, WHEEL_W, WHEEL_MASS = 35.0, 7.0, 6.0  # bought silicone wheels [est]

# ---- other bought items ------------------------------------------------------------------------------------------------------------
BRIO = (26.5, 102.0, 27.0)  # x depth, y width, z height [src spec 10]
BRIO_MASS = 65.0
SERVO = dict(
    body=(12.2, 22.7, 22.7), tab_hole_dy=27.5, spline_from_end=6.0, mass=9.0
)  # SG90 class [est, not fetched]
JETSON_BOARD = (103.0, 90.5, 34.8)  # devkit envelope incl. heatsink [part toml]
JETSON_HOLES = (86.0, 58.0)  # [est] confirm against NVIDIA drawing
PICO = (
    51.0,
    21.0,
    11.4,
    47.0,
)  # board, hole spacing (short, long) -- RP2040 Pico mechanical drawing [est]
STANDOFF_H = 6.0

# ---- derived layout, base frame B (z=0 stator plate top) --------------------------------------------------------------------------
STATOR_T, STATOR_W = 8.0, 200.0
POST_H = 18.0
ROTOR_T = 8.0
ROTOR_W = 180.0
Z_LS0 = POST_H  # bearing bottom
Z_ROTOR0 = Z_LS0 + LS_H  # rotor plate underside = 27
Z_ROTOR1 = Z_ROTOR0 + ROTOR_T
Z_BELT = 7.0  # belt centre height above the stator top (both axes' pan belt)
MOUNT_T = 4.0  # motor mount plate
CABLE_HOLE_D = 28.0

H_T = 202.0  # tilt axis height in B [spec est 120]; raised so the breech clears the rotor plate at +70 deg (tilt sweep check)

# ---- ball feed: hopper on the PAN group, balls enter the head through the hollow right trunnion (axis = tilt axis) ----------------
# pan-side elbow (fixed) ends in a spigot that sits inside the rotating hollow trunnion (journal, in a 6807 bearing); the ball
# stops at the elbow point C on the tilt axis and rolls down a straight chute (head side) to the breech port, whatever the tilt.
SPIG_ID = BALL_D + 1.8  # fixed spigot bore
SPIG_WALL = 1.8
SPIG_OD = SPIG_ID + 2 * SPIG_WALL
JOUR_ID = (
    SPIG_OD + 0.8
)  # bore of the rotating hollow trunnion = ball chute bore (running clearance 0.4/side)
CHUTE_R = (
    JOUR_ID / 2 - 0.2
)  # chute bore radius (a hair under the trunnion bore: no equal-radius cylinder intersection)
DUCT_ID = BARREL_ID + 2.4  # hopper neck / elbow duct bore
TUBE_OD = 35.0  # hollow trunnion OD = 6807 bearing bore
B6807_OD, B6807_ID, B6807_W, B6807_MASS = (
    47.0,
    35.0,
    7.0,
    45.0,
)  # 6807ZZ [est dims/mass]
assert JOUR_ID + 5.0 <= TUBE_OD, "ball too large for the 35 mm trunnion tube"
CHUTE_WALL = 1.6
BEAM_FOOT_T = 4.0  # +1 so the 32 mm flywheel motor tops (z 36.5) clear the ball bore under the hollow trunnion
FEED_SLOPE_MIN = (
    15.0  # deg downhill along gravity required over the sloping feed path at every tilt
)
ELBOW_SLOPE = 38.0  # deg, sloping duct on the pan side
ELBOW_YS, ELBOW_YH = (
    76.0,
    116.0,
)  # |y| where the slope meets the axis spigot / hopper axis |y|
SPIG_TIP_Y = 15.0  # spigot tip |y| (ball leaves it into the head chute elbow at y = 0)
ROLL_MU = (
    0.1  # rolling/sliding resistance of a foam ball in the level run [est, bench test]
)

# ---- head, launcher frame L (barrel axis z=0, flywheel centre x=0); head frame H = L - (X_A, 0, Z_A) ---------------------------------
CAGE_X, CAGE_W, CAGE_Z0, CAGE_Z1 = 70.0, 100.0, -17.0, 31.0
X_F = -50.0  # feed (ball drop) x
PISTON_LEN, PISTON_STROKE = 48.0, 44.0
BARREL_LEN = 110.0
BARREL_R = BARREL_ID / 2 + 3.0  # outer radius
CHASSIS_X0, CHASSIS_Z0, CHASSIS_Z1, CHASSIS_W = -118.0, -17.0, 22.0, 70.0
X_A = 15.0  # tilt axis x in L (set from the head CG, see checks.balance)
Z_A = (
    CAGE_Z1 + BEAM_FOOT_T + TUBE_OD / 2
)  # tilt axis height in L: the trunnion beam sits on the cage top (motor pockets stay open)
WC = CAGE_W  # cradle width between trunnion bosses
BOSS_L, BOSS_D = 7.0, 16.0
CHEEK_Y0, CHEEK_T = WC / 2 + BOSS_L + 1.0, 10.0
CAM_PLATE_T = 4.0
BALANCE_LIMIT = 15.0  # mm, spec 6.3 (for the 1.2 kg head the spec allows 1.2 * 9.81 * 0.015 = 0.177 N*m static)
TILT_STATIC_BUDGET_NM = (
    1.2 * 9.81 * 0.015
)  # spec 6.3 static unbalance torque allowance at the tilt joint
TILT_RANGE = (-15.0, 70.0)

# ---- pan hard stop (spec 2.3): printed tab under the rotor edge (+90 deg) hits an M3 stud in the stator (-90 deg) at ~+-178 deg ----
PAN_STOP_R = 89.75  # radius of the stud axis and the tab blade centre
PAN_STOP_STUD_D, PAN_STOP_STUD_H = (
    3.0,
    14.0,
)  # M3 threaded stud in a heat-set insert, height above the stator top
PAN_STOP_TAB_W = 3.6  # tangential width of the tab blade
PAN_STOP_DEG = 178.0  # target engagement angle, +-2
