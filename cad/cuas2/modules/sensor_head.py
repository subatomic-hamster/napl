"""M5 SENSOR HEAD (spec 3): bar on the cradle bridge through dock D2 (2 x M4 on 40 mm + 6 mm dowel), Brio retained by two end walls
(1/4-20 tripod hole), TF-Luna pocket on the barrel axis side (boresight shims: 0-1.5 mm, +-3 deg, not modelled). Frame L.
The thermal pocket and the second-camera beam of the spec are optional modules and are NOT built here.
"""

import interfaces as I
import params as P
from prims import bxr, hole

D2 = I.DOCKS["D2"]
BAR_X0, BAR_X1 = 22.0, 92.0
BRIO_X0 = 28.0
SENSOR_ORIGIN = (30.0, 0.0, 22.0)
LUNA = (
    35.0,
    21.0,
    13.0,
)  # TF-Luna module 35 x 21 x 13 mm [est, vendor drawing not fetched]


def d2_loc():
    return I.place(SENSOR_ORIGIN, (1, 0, 0), (0, 0, 1))


def sensor_bar():
    z0, z1 = SENSOR_ORIGIN[2], SENSOR_ORIGIN[2] + 4.0
    s = bxr(BAR_X0, BAR_X1, -55.0, 55.0, z0, z1)
    # Brio end walls
    bd, bw, bh = P.BRIO
    for sy in (-1, 1):
        s = s + bxr(
            BRIO_X0 - 1.5,
            BRIO_X0 + bd + 1.5,
            min(sy * (bw / 2 + 0.3), sy * (bw / 2 + 2.3)),
            max(sy * (bw / 2 + 0.3), sy * (bw / 2 + 2.3)),
            z1,
            z1 + 14.0,
        )
    s = s - hole(
        "z",
        P.UNC_QTR_CLR if hasattr(P, "UNC_QTR_CLR") else 6.6,
        BRIO_X0 + bd / 2,
        0,
        z0 - 1,
        z1 + 1,
    )
    # TF-Luna pocket block, on the barrel axis side, ahead of the Brio
    lx0 = BRIO_X0 + bd + 4.0
    s = s + bxr(
        lx0,
        lx0 + LUNA[0] + 4,
        -(LUNA[1] / 2 + 2),
        LUNA[1] / 2 + 2,
        z1,
        z1 + LUNA[2] + 2,
    )
    s = s - bxr(
        lx0 + 2,
        lx0 + 2 + LUNA[0],
        -LUNA[1] / 2,
        LUNA[1] / 2,
        z1 + 2.0,
        z1 + LUNA[2] + 3,
    )
    # D2 female side (dowel socket + M4 clearance)
    f = d2_loc()
    s = s - f * I.female_void(D2)
    s = s - f * I.bolt_holes(D2, "female")
    return s


PARTS = {
    "sensor_bar": dict(
        fn=sensor_bar, group="tilt", count=1, material="PETG", module="M5"
    ),
}


def bought():
    bd, bw, bh = P.BRIO
    z1 = SENSOR_ORIGIN[2] + 4.0
    lx0 = BRIO_X0 + bd + 4.0 + 2.0
    return [
        dict(
            name="brio",
            lib="logitech-brio-4k",
            mass=P.BRIO_MASS,
            module="M5",
            group="tilt",
            shape=bxr(BRIO_X0, BRIO_X0 + bd, -bw / 2, bw / 2, z1, z1 + bh),
        ),
        dict(
            name="tf_luna",
            lib="benewake-tf-luna",
            mass=5.0,
            module="M5",
            group="tilt",
            optional=True,
            shape=bxr(
                lx0,
                lx0 + LUNA[0],
                -LUNA[1] / 2,
                LUNA[1] / 2,
                z1 + 2.0,
                z1 + 2.0 + LUNA[2],
            ),
        ),
    ]
