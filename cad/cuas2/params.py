"""cuas2 CAD parameters: the ONE place every dimension lives (mm, g, degrees unless named otherwise).

Spec: docs/cuas2/spec.md (section numbers in comments). Tags: [src] sourced, [est] estimate, [der] derived here.
Frames: B = base frame (origin on the pan axis at the top face of the stator plate, x forward, z up).
L = launcher frame (origin at the wheel-nip centre on the barrel axis, x along the barrel, y left, z up).
H = head frame (origin on the tilt axis) = L - (X_A, 0, Z_A). Tilt-group parts are authored in L and shifted to H by the registry.

Environment overrides exist only for the negative controls of acceptance 2 (CUAS2_BALL_D, CUAS2_WHEEL_D, CUAS2_GATE_SWING_DEG,
CUAS2_POCKET_D_BALLS): a changed value regenerates every dependent dimension and the checks that depend on it change value.
"""

import math
import os

# ---- ball (copied from cad/cuas-lite/params.py; tests/cuas2/test_cad_consts.py asserts equality) ----------------------------------
BALL_D = float(os.environ.get("CUAS2_BALL_D", 22.2))  # mm [src spec 0]
BALL_M = 1.86  # g
BALL = BALL_D  # the diameter, the single shared constant
BALL_CLEAR = 1.3  # barrel ID = ball + 1.3 [spec 4.1]
BORE_ID = BALL_D + BALL_CLEAR  # barrel ID 23.5
PACKING = 0.6
HOPPER_BALLS = 24

# ---- fab ----------------------------------------------------------------------------------------------------------------------
BED = (220.0, 220.0, 250.0)
BED_MARGIN = 10.0  # max printed footprint 200 x 200 (spec 3, C-FIT)
PRINT_MAX = 200.0
TOL_HOLE = 0.2
DOVE_CLEAR = 0.3  # per side, dovetail and dowel fits [est FDM, spec 3]
NOZZLE, LAYER, WALLS, INFILL = 0.4, 0.2, 2, 0.20
DENSITY = {"PLA": 1.24e-3, "PETG": 1.27e-3, "TPU": 1.21e-3}  # g/mm^3
FLOW_MM3_S = 8.0
TIME_OVERHEAD = 1.35
COST_PER_KG = 20.0  # USD, same as v1
M3_CLR, M3_INSERT_D = 3.4, 4.2
M4_CLR, M4_INSERT_D = 4.5, 5.6

# ---- flywheels (spec 4.1) ---------------------------------------------------------------------------------------------------------
WHEEL_D = float(os.environ.get("CUAS2_WHEEL_D", 40.0))
WHEEL_W = 14.0
SQUEEZE = 3.0  # mm [spec 4.1]
FW_GAP = BALL_D - SQUEEZE  # wheel-surface gap, 19.2 nominal
WZ = (
    FW_GAP + WHEEL_D
) / 2  # wheel-centre height above/below the barrel axis, 29.6 (separation 59.2)
GAP_TRAVEL = 1.0  # required +-1.0 mm gap adjustment (squeeze 2.0..4.0)
SLOT_TRAVEL_BUILT = 1.5  # the printed slot gives +-1.5 mm
V_CAP_MPS = 38.0
RPM_CAP = math.floor(
    V_CAP_MPS * 60 / (math.pi * WHEEL_D / 1000)
)  # spec I13, 18,144 at D 40

# 2207 brushless motor envelope [est, typical 2207: OD 27.9, ~20 mm base-to-bell, 5 mm shaft]
MOT_R, MOT_H, MOT_SHAFT_D, MOT_SHAFT_PROT, MOT_MASS = 13.95, 20.0, 5.0, 15.0, 33.0
MOT_HOLES = 16.0  # 4 x M3 on a 16 x 16 pattern [est]
MOT_CENTRE_HOLE = 9.0
WHEEL_HUB_R, WHEEL_HUB_MASS_NOTE = 14.0, 0
ESC_SIZE = (45.0, 15.0, 5.0)  # single 30 A BLHeli_S-class ESC board [est]
ESC_MASS = 9.0

# ---- cage / cartridge (M3) layout in L --------------------------------------------------------------------------------------------
X_F1 = -30.0  # breech flange plane (cartridge rear face), faces -x
CORE_X1 = 14.0  # cartridge core front; barrel tube continues to the muzzle
CORE_HALF = 17.0  # core half width y and half height z
BARREL_OD_R = BORE_ID / 2 + 3.0
BARREL_X1 = 70.0  # muzzle 70 mm after the nip [spec 4.1]
BEAM_X = 55.0  # muzzle beam 55 mm after the nip [spec 4.1]
PLATE_T = 3.0
PLATE_HALF_X = 16.0
LIP_X, LIP_D_UNDER = (
    -22.0,
    0.8,
)  # ball stop lip in the channel (stops gravity roll-forward at tilt < 0; foam squeezes over it)

# ---- gate / pocket / pusher (M4) ----------------------------------------------------------------------------------------------------
GATE_SWING_DEG = float(
    os.environ.get("CUAS2_GATE_SWING_DEG", 60.0)
)  # [spec 5.2: 45..90]
POCKET_LEN = (
    float(os.environ["CUAS2_POCKET_D_BALLS"]) * BALL_D
    if "CUAS2_POCKET_D_BALLS" in os.environ
    else 28.0
)  # bore length = plug diameter (a diametral through-bore); spec text 26/30, 28 chosen: 26 leaves a 1.1 mm wall
PLUG_R = POCKET_LEN / 2
PLUG_L = 30.0  # plug length along y
POCKET_BORE_D = BALL_D + 1.6  # 23.8 [spec 5.2]
PLUG_CLR = 0.3
HOUSING_WALL = 3.5
F1_LIP_H = 2.0  # ring lip height of dock F1
XV = (
    X_F1 - F1_LIP_H - 1.5 - PLUG_R - PLUG_CLR
)  # plug axis x: the plug cavity stays 1.5 mm clear of the F1 ring lip
ROD_D = 20.0  # rod and soft pad diameter [spec 5.2]
APERTURE_D = ROD_D + 1.0
ROD_TIP_RET = XV - PLUG_R - 1.0  # retracted tip clears the plug cylinder
ROD_TIP_EXT = (
    -9.5 + 5.0 - BALL_D / 2
)  # ball centre ends 5 mm past first contact (-9.5): -4.5; the wheels take it from there  [der spec 4.1 / 5.2]
PUSH_STROKE = ROD_TIP_EXT - ROD_TIP_RET
THROAT_D = 26.0  # one-ball-wide throat 1.17 D [spec 5.1]
THROAT_WALL = 2.4
THROAT_L = 40.0
MOUTH_X = XV + PLUG_R * math.cos(
    math.radians(GATE_SWING_DEG)
)  # LOAD pose: front end up, mouth under the throat
X_T = XV  # throat axis over the plug axis: the throat tube stays rearward of the wheels (x < -31); balls drop onto the mouth
HOUSING_TOP = PLUG_R + HOUSING_WALL
HOPPER_ALPHA = 50.0  # floor facet angle to the head x-y plane [spec 5.1]
FEED_SLOPE_MIN = 15.0
HOPPER_W = 60.0  # y inside
HOPPER_T = 2.0
Z_HOP0 = HOUSING_TOP + THROAT_L  # hopper floor start (throat top)
X_HOP_REAR = (
    X_T - THROAT_D / 2
)  # rear wall inner face (outlet at the rear-bottom corner)
X_HOP_FRONT = X_HOP_REAR + 70.0  # 70 mm long [spec 5.1]
SERVO_BODY = (
    12.2,
    22.7,
    22.7,
)  # SG90 class x depth, y width, z height envelope [est, same as v1 params.SERVO]
SERVO_MASS = 9.0
# servo time model [est, spec 5.4, B-TIME bench item]: 0.1 s per 60 deg gate; pusher ~0.4 mm/ms (32 mm in 80 ms)
SERVO_S_PER_DEG = 0.1 / 60.0
PUSHER_MM_PER_MS = 32.0 / 80.0
FILL_MS = 30.0

# ---- head frame (tilt axis) ------------------------------------------------------------------------------------------------------
# axis position in L: tuned so the CG sits on the axis (checks.balance reads the real CG; these two numbers + BALLAST_G are the design knobs)
X_A, Z_A = (
    -8.0,
    27.0,
)  # tilt axis in L: placed on the CG that checks.C-BAL measures (CG at x -7.8, z +27.6 before ballast)
BALLAST_G = 0.0
TILT_RANGE = (-15.0, 70.0)
TILT_CHECKS = (-15.0, 0.0, 35.0, 70.0)
HEAD_MASS_MAX_G = 600.0
CG_OFFSET_MAX_MM = 8.0
TORQUE_MAX_NM = 0.06

# ---- trunnion / yoke / base (M1, M2) -----------------------------------------------------------------------------------------------
B608_OD, B608_ID, B608_W, B608_MASS = 22.0, 8.0, 7.0, 12.0
ROD_8 = 8.0
RAIL_Y0 = CORE_HALF  # rail base on the core side face
RAIL_NECK = 8.0
RAIL_HEAD_W = 12.0  # "60 deg dovetail 12 mm wide" [spec 3]
RAIL_HEAD_H = (
    (RAIL_HEAD_W - RAIL_NECK) / 2 * math.tan(math.radians(60.0))
)  # 3.46 flare height for 60 deg flanks
CAGE_HALF_Y = 31.0  # outer face of the motor plates
ESC_FACE_Y = CAGE_HALF_Y + ESC_SIZE[2]
ARM_IN_Y = ESC_FACE_Y + 2.0  # cradle arm inner faces (+-38)
RAIL_Y1 = ARM_IN_Y - DOVE_CLEAR  # rail tip
ARM_T = 8.0
BOSS_L = 6.0
CHEEK_IN_Y = ARM_IN_Y + ARM_T + BOSS_L  # inner face of the yoke cheeks
CHEEK_T = 10.0
STATOR_T, STATOR_W = 8.0, 200.0
LS_OD, LS_APERTURE, LS_H, LS_MASS = 152.4, 117.5, 9.0, 218.0  # v1 lazy susan
POST_H = 18.0
ROTOR_T, ROTOR_W = 8.0, 180.0
Z_ROTOR0 = POST_H + LS_H  # 27
N17_FACE, N17_LEN, N17_MASS = 42.3, 40.0, 285.0
T_MOTOR, T_PAN, T_TILT = 20, 100, 80
PITCH = 2.0
H_T = 200.0  # tilt axis height above the stator top face in B [est; C-PATH sweeps the head over -15..+70 against yokes, rotor, base]

# ---- bought-item masses (g) [est unless noted] --------------------------------------------------------------------------------------
BRIO = (26.5, 102.0, 27.0)  # depth x width x height, v1 [src spec 10]
BRIO_MASS = 65.0
HALL_MASS, MAGNET_MASS, LED_MASS = 0.5, 0.3, 0.3
HARDWARE_TILT_G = 25.0  # screws, inserts, ties [est, same as v1], placed at the CG
WIRING_TILT_G = 20.0  # head harness share [spec 3 mass table]
BALLS_G = HOPPER_BALLS * BALL_M

# ---- sensor head (M5) -----------------------------------------------------------------------------------------------------------------
BRIO_HOLE_PITCH = 40.0  # D2: 2 x M4 on a 40 mm pitch + 6 mm dowel [spec 3]
DOWEL_D = 6.0
SENSOR_X0 = 22.0

# ---- power box (M6) ---------------------------------------------------------------------------------------------------------------------
BOX = (100.0, 80.0, 40.0)  # DIN-style 100 x 80 x 40 [spec 3]
BOX_WALL = 2.4

# GT2 belts (v1 numbers: 126-tooth pan loop, 102-tooth tilt loop; closed-form centre distances from cad/cuas-lite/params.py)
C_PAN, C_TILT = 60.563, 48.161
LS_BOLT_R = 63.5  # bolt circle radius of the lazy susan [est, measure on the delivered bearing (v1 open item)]
PAN_PULLEY_OD = 100 * PITCH / math.pi - 0.762
TILT_PULLEY_OD = 80 * PITCH / math.pi - 0.762
LEG_H = 120.0  # base legs + deck: the lower deck carries the Jetson (top at -83 mm) clear of the 60 mm power box (bottom at -70 mm)
DECK_Z = -(8.0 + LEG_H)  # deck underside z in B
FOOT_T = 8.0
# timing model (documented in manifest 'cycle'): see checks.cycle_ms
