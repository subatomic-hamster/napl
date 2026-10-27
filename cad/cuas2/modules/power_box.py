"""M6 POWER / SAFETY BOX (spec 3): 100 x 80 x 60 (spec 40: the safety ICs and the 40 A relay need the height) under the base plate (4 x M3 lugs into the plate underside, bolt heads inside), open
at the bottom, lid screwed on. Holds the Pico, 2 TMC2209, relay, safety boards, buck, regulator and the 20 A fuse. Key switch and
E-stop mount on the -y wall. H1 head-harness panel on the +x face. Frame B.
"""

import interfaces as I
import params as P
from prims import bxr, hole

H1 = I.DOCKS["H1"]
BX0, BX1 = -97.0, 3.0
BY = 40.0
BZ0, BZ1 = (
    -68.0,
    -8.0,
)  # 60 mm deep: Pico, 2 TMC2209, relay, buck, 5 V regulator and the safety ICs sit on the lid
W = P.BOX_WALL
KEY_Y, ESTOP_Y = (-BY, -BY)


def h1_loc():
    return I.place((BX1 + 6.0, 0.0, -28.0), (0, 1, 0), (1, 0, 0))


def box_body():
    s = bxr(BX0, BX1, -BY, BY, BZ0, BZ1) - bxr(
        BX0 + W, BX1 - W, -BY + W, BY - W, BZ0 - 1, BZ1 - W
    )
    # corner lugs for the base plate (clearance holes through the top wall)
    for bx in (-45.0, 45.0):
        for by in (-35.0, 35.0):
            s = s - hole("z", P.M3_CLR, -47.0 + bx, by, BZ1 - W - 1, BZ1 + 1)
    # lid screws (inserts into 4 corner posts)
    for x in (BX0 + 5.0, BX1 - 5.0):
        for y in (-BY + 5.0, BY - 5.0):
            s = s + hole("z", 8.0, x, y, BZ0, BZ0 + 12.0)
            s = s - hole("z", P.M3_INSERT_D, x, y, BZ0 - 1, BZ0 + 8.0)
    # key (12 mm) and E-stop (22 mm) holes in the -y wall, harness panel + side service hole on the +x face
    s = s - hole("y", 12.2, -75.0, -28.0, -BY - 1, -BY + W + 1)
    s = s - hole("y", 22.4, -35.0, -28.0, -BY - 1, -BY + W + 1)
    s = s + h1_loc() * bxr(-20, 20, -8, 8, -6, 0)
    s = s - h1_loc() * I.female_void(H1)
    s = s - h1_loc() * I.bolt_holes(H1, "male")
    s = s - hole(
        "x", 10.0, 0, -28.0, BX1 - W - 1, BX1 + 1
    )  # cable pass to the H1 panel
    return s


def box_lid():
    s = bxr(BX0, BX1, -BY, BY, BZ0 - 2.0, BZ0)
    for x in (BX0 + 5.0, BX1 - 5.0):
        for y in (-BY + 5.0, BY - 5.0):
            s = s - hole("z", P.M3_CLR, x, y, BZ0 - 3, BZ0 + 1)
    return s


PARTS = {
    "power_box": dict(
        fn=box_body, group="static", count=1, material="PETG", module="M6"
    ),
    "power_box_lid": dict(
        fn=box_lid, group="static", count=1, material="PETG", module="M6"
    ),
}


def bought():
    return []
