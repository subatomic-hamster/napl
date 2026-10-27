"""M3 LAUNCHER CARTRIDGE (spec 3, 4): cage + barrel in one print, slotted gap plate, wheel hubs + TPU tyres, brushless 2207 pair, 2 ESCs,
muzzle beam. Frame L (nip centre at the origin, x along the barrel). Rails of dock D1 on both sides, breech flange F1 at the rear.
"""

import interfaces as I
import params as P
from build123d import Pos
from prims import bxr, hole, slot_z, tube

D1, F1 = I.DOCKS["D1"], I.DOCKS["F1"]
Z_TIP = (
    D1.loc["base_to_face"] + D1.loc["head_h"]
)  # dovetail tip in the dock frame (z along the outward direction)


def d1_loc(sy):
    return I.place((P.X_F1, sy * P.CORE_HALF, 0.0), (1, 0, 0), (0, sy, 0))


def f1_loc():
    return I.place((P.X_F1, 0.0, 0.0), (0, 1, 0), (-1, 0, 0))


def rails():
    out = None
    for sy in (1, -1):
        loc = d1_loc(sy)
        r = loc * I.male_feature(D1)
        # clamp-screw pilots from the tip down the rail
        h = loc * Pos(0, 0, Z_TIP) * I.bolt_holes(D1, "male")
        r = r - h
        out = r if out is None else out + r
    return out


def cartridge_cage():
    core = bxr(P.X_F1, P.CORE_X1, -P.CORE_HALF, P.CORE_HALF, -P.CORE_HALF, P.CORE_HALF)
    barrel = hole("x", 2 * P.BARREL_OD_R, 0, 0, P.CORE_X1 - 1, P.BARREL_X1)
    s = core + barrel
    # rear flange (F1) with ring lip
    s = s + bxr(P.X_F1, P.X_F1 + 6.0, -32, 32, -26, 26)
    s = s + f1_loc() * I.male_feature(F1)
    # wheel windows, motor plate (top, fixed) and its struts
    for sz in (1, -1):
        s = s - hole("y", P.WHEEL_D + 3.0, 0, sz * P.WZ, -9.5, 9.5)
    s = s + bxr(
        -P.PLATE_HALF_X,
        P.PLATE_HALF_X,
        P.CAGE_HALF_Y - P.PLATE_T,
        P.CAGE_HALF_Y,
        12,
        48,
    )
    s = s + bxr(-P.PLATE_HALF_X, P.CORE_X1, P.CORE_HALF - 1, P.CAGE_HALF_Y, 11, 15)
    # struts for the gap plate (bottom, -y): tab bolts go into these
    s = s + bxr(
        -P.PLATE_HALF_X,
        P.CORE_X1,
        -(P.CAGE_HALF_Y - P.PLATE_T),
        -P.CORE_HALF + 1,
        -12,
        -4,
    )
    # top motor holes: 16 x 16 M3 + centre
    for sx in (-1, 1):
        for sz in (-1, 1):
            s = s - hole(
                "y",
                P.M3_CLR,
                sx * P.MOT_HOLES / 2,
                P.WZ + sz * P.MOT_HOLES / 2,
                P.CAGE_HALF_Y - P.PLATE_T - 1,
                P.CAGE_HALF_Y + 1,
            )
    s = s - hole(
        "y",
        P.MOT_CENTRE_HOLE,
        0,
        P.WZ,
        P.CAGE_HALF_Y - P.PLATE_T - 1,
        P.CAGE_HALF_Y + 1,
    )
    # gap-plate bolt inserts (into the struts, along y)
    for sx in (-1, 1):
        s = s - hole(
            "y",
            P.M3_INSERT_D,
            sx * 10.0,
            -8.0,
            -(P.CAGE_HALF_Y - P.PLATE_T) - 1,
            -(P.CAGE_HALF_Y - P.PLATE_T) + 6,
        )
    # bore + stop lip
    s = s - hole("x", P.BORE_ID, 0, 0, P.X_F1 - 1, P.BARREL_X1 + 1)
    lip = tube(
        "x",
        P.BORE_ID / 2 + 1.5,
        (P.BALL_D - P.LIP_D_UNDER) / 2,
        0,
        0,
        P.LIP_X,
        P.LIP_X + 2.0,
    )
    s = s + lip
    # muzzle beam bosses + holes (LED +y, phototransistor -y), cross hole through the barrel wall
    for sy in (1, -1):
        s = s + bxr(
            P.BEAM_X - 4,
            P.BEAM_X + 4,
            min(sy * (P.BARREL_OD_R - 1), sy * 19.0),
            max(sy * (P.BARREL_OD_R - 1), sy * 19.0),
            -4,
            4,
        )
        s = s - hole(
            "y", 5.2, P.BEAM_X, 0, min(sy * 8.0, sy * 20.0), max(sy * 8.0, sy * 20.0)
        )
    s = s - hole("y", 3.2, P.BEAM_X, 0, -20, 20)
    # clearance for the stop-lip insert side (none); D1 rails
    s = s + rails()
    # F1 bolt pilots into the flange
    s = s - f1_loc() * I.bolt_holes(F1, "male")
    s = s - hole("x", P.BORE_ID, 0, 0, P.X_F1 - 4, P.X_F1 + 8)
    return s


def gap_plate():
    """Bottom motor plate (-y side), vertical slots give +-SLOT_TRAVEL_BUILT mm of gap adjustment (spec 4.1)."""
    y1, y0 = -P.CAGE_HALF_Y, -P.CAGE_HALF_Y + P.PLATE_T
    s = bxr(-P.PLATE_HALF_X, P.PLATE_HALF_X, y1, y0, -48, -4)
    z0 = -P.WZ
    for sx in (-1, 1):
        for sz in (-1, 1):
            s = s - hole(
                "y",
                P.M3_CLR,
                sx * P.MOT_HOLES / 2,
                z0 + sz * P.MOT_HOLES / 2,
                y1 - 1,
                y0 + 1,
            )
        s = s - slot_z(
            "y", P.M3_CLR, sx * 10.0, -8.0, y1 - 1, y0 + 1, P.SLOT_TRAVEL_BUILT
        )
    s = s - hole("y", P.MOT_CENTRE_HOLE, 0, z0, y1 - 1, y0 + 1)
    # push-pull adjuster tab with the M3 screw hole (0.5 mm per turn, scale marks on the tab are not modelled)
    s = s + bxr(-4, 4, y1, y0, -53, -48)
    s = s - hole("z", P.M3_CLR, 0, (y0 + y1) / 2, -54, -47)
    return s


def wheel_hub():
    return (
        hole("y", 2 * P.WHEEL_HUB_R, 0, 0, -P.WHEEL_W / 2, P.WHEEL_W / 2)
        - hole("y", P.MOT_SHAFT_D + 0.2, 0, 0, -9, 9)
        - hole("z", 3.2, 0, 0, 3, 16)
    )


def wheel_tyre():
    return tube(
        "y", P.WHEEL_D / 2, P.WHEEL_HUB_R - 0.2, 0, 0, -P.WHEEL_W / 2, P.WHEEL_W / 2
    )


PARTS = {
    "cartridge_cage": dict(
        fn=cartridge_cage, group="tilt", count=1, material="PETG", module="M3"
    ),
    "gap_plate": dict(
        fn=gap_plate, group="tilt", count=1, material="PETG", module="M3"
    ),
    "wheel_hub": dict(
        fn=wheel_hub,
        group="tilt",
        count=2,
        material="PETG",
        module="M3",
        at=[(0, 0, P.WZ), (0, 0, -P.WZ)],
    ),
    "wheel_tyre": dict(
        fn=wheel_tyre,
        group="tilt",
        count=2,
        material="TPU",
        module="M3",
        at=[(0, 0, P.WZ), (0, 0, -P.WZ)],
    ),
}


def _motor(sz):
    """2207 envelope: top motor on +y, bottom on -y; axis along y, base on the plate, shaft toward y = 0."""
    y_base = sz * P.CAGE_HALF_Y - sz * P.PLATE_T
    y_bell = y_base - sz * P.MOT_H
    y_tip = y_bell - sz * P.MOT_SHAFT_PROT
    body = hole(
        "y", 2 * P.MOT_R, 0, sz * P.WZ, min(y_base, y_bell), max(y_base, y_bell)
    )
    return body + hole(
        "y", P.MOT_SHAFT_D, 0, sz * P.WZ, min(y_bell, y_tip), max(y_bell, y_tip)
    )


def bought():
    out = []
    for sz, nm in ((1, "top"), (-1, "bot")):
        out.append(
            dict(
                name=f"motor_{nm}",
                lib="brushless-2207-1800kv",
                mass=P.MOT_MASS,
                shape=_motor(sz),
                module="M3",
                group="tilt",
            )
        )
        y0 = sz * (P.CAGE_HALF_Y)
        out.append(
            dict(
                name=f"esc_{nm}",
                lib="esc-30a-bluejay-single",
                mass=P.ESC_MASS,
                module="M3",
                group="tilt",
                shape=bxr(
                    -P.ESC_SIZE[0] / 2,
                    P.ESC_SIZE[0] / 2,
                    min(y0, y0 + sz * P.ESC_SIZE[2]),
                    max(y0, y0 + sz * P.ESC_SIZE[2]),
                    sz * P.WZ - P.ESC_SIZE[1] / 2,
                    sz * P.WZ + P.ESC_SIZE[1] / 2,
                ),
            )
        )
    for sy, lib, nm in (
        (1, "ir-led-940nm-5mm", "muzzle_led"),
        (-1, "ir-phototransistor-940nm-5mm", "muzzle_pt"),
    ):
        out.append(
            dict(
                name=nm,
                lib=lib,
                mass=P.LED_MASS,
                module="M3",
                group="tilt",
                shape=hole(
                    "y",
                    5.0,
                    P.BEAM_X,
                    0,
                    min(sy * 9.0, sy * 17.0),
                    max(sy * 9.0, sy * 17.0),
                ),
            )
        )
    return out
