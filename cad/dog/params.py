"""Dog printed-part parameters (mm, g, degrees unless named). The leg study picks L_THIGH / L_SHANK; everything regenerates from here.

Leg frame: hip origin; abduction axis along +x (forward), hip/knee pitch axes along y, the leg hangs along -z.
Values marked [est] are design estimates (no vendor source); bought-part sizes come from the part files in parts/.
"""

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
M3_CLR, M3_INSERT_D = 3.4, 4.2

# bought parts the prints hold
AB_D = 92.4  # GIM8108-8 body 92 mm + 0.4 [steadywin-gim8108-8]
KN_D = 80.4  # GIM6010-8 body 80 mm + 0.4 [steadywin-gim6010-8]
BALL_D = 34.925 + 0.3  # rubber ball seat [mcmaster-natural-rubber-ball-1-3-8in]
LIDAR = (65.0, 65.0, 60.0)  # Mid-360 envelope [livox-mid-360]
