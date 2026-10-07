# Koch v1.1 follower arm: sources, license, meshes, kinematics (stage A3 / S2)

Accessed 2026-10-07. Follower arm only (the leader uses XL330-M077 and a handle/trigger; out of scope).
Files: `parts/arm/koch-*.toml` (7 printed parts), `meshes/koch/` (URDF + 13 upstream STL + 2 derived servo STL, 4.1 MB), `tools/koch_ports.py`.

## Source repos, commits, licenses

| Repo | Commit | License | Content relevant here |
|---|---|---|---|
| https://github.com/jess-moss/koch-v1-1 ("Koch v1.1", Hugging Face) | `1a553e1fa7d5404b51bbd83b99b0f7eba54888e0` (2024-09-17, "Merge pull request #12 from torotoki/main"; last commit on `main`) | Apache-2.0 (`LICENSE` at the root) | BOM / print settings (README), printable STL `hardware/follower/STL/*.STL` (mm, 7 parts), STEP + SolidWorks of every follower part and of the servos (`Robotis_DYNAMIXEL_XL430_W250_T.STEP`, `..._XL330_M288_T.STEP`, FPX330-H101, HN11-I101), wiring diagram PDF. **No URDF / MJCF.** |
| https://github.com/AlexanderKoch-Koch/low_cost_robot (original "Low-Cost Robot Arm") | `0602b0c1e6a67d8f6ab96f81d1378639018d98af` (2024-09-21) | MIT ("Copyright (c) 2024 Alexander Koch") | v1.0 hardware (STL/STEP/F3Z), `simulation/low_cost_robot/` URDF + MJCF + 8 STL. **v1.0 geometry**: its `base_link.stl` is 63729 mm^3 vs 24888 mm^3 for the v1.1 base (different part, the servo is fused into it). Not used. |
| https://github.com/s1lent4gnt/koch-v1-1 (fork of jess-moss, adds `simulation/`) | `cc1a4eb9b27978b6bc3bb34f12fa5a7d229f1fde` (2025-02-25, "separate arm parts to links and motor links") | Apache-2.0 (same `LICENSE`, `cmp` identical to jess-moss's) | `simulation/follower.urdf` + `simulation/meshes/{base_link,link_1..6}.STL` + `*_motor.STL` (servos per link). **v1.1 geometry** (volumes match, below). **Chosen source.** |
| https://github.com/perezjln/gym-lowcostrobot (`gym_lowcostrobot/assets/low_cost_robot_6dof/follower.xml` + `follower_meshes/`) | `a1a2286bcaa45ee2844a695f339fd48d9055da83` (2025-01-13) | Apache-2.0 ("Copyright 2024 Julien Perez"; README "licensed under the Apache 2.0 License") | MJCF with the same bodies, joint origins and masses as the s1lent4gnt URDF; `link_N.STL` byte-identical to s1lent4gnt's (cmp) -> **v1.1 geometry** although the README only credits Alexander Koch's repo. Elbow axis (0 -1 0), limits placeholder +-3.14. |
| https://github.com/google-deepmind/mujoco_menagerie `low_cost_robot_arm/` | `f054586a8e90465d49ee5be15335c4a0c7f57caf` (repo HEAD 2026-10-05; model initial release 2024-11-25) | Apache-2.0 (own `LICENSE`, README: "This model is released under an Apache-2.0 License") | MJCF + 22 STL (visual, servo "_motor" and convex collision meshes), position actuators, **approximate joint limits**. README: "derived from the publicly available URDF description (https://github.com/s1lent4gnt/koch-v1-1/blob/main/simulation/follower.urdf)", named "Kochv1.1" -> v1.1 geometry; joint origins identical to the s1lent4gnt URDF. Used here only for the joint limits (not copied). |
| https://github.com/haosulab/ManiSkill `mani_skill/assets/robots/koch/` (`follower_arm_v1.1.urdf`) | HEAD at access time (not pinned) | Apache-2.0 (README: assets from jess-moss/koch-v1-1 "provided under a Apache 2.0 License") | v1.1, re-modelled: different link frames (`.glb`, centred meshes), different q = 0 pose, "Fixed joint limits and axes to reflect real world behavior" (shoulder lift 0..2.2, elbow 0..3.05, wrist flex -1.5..2.14, gripper 0..1.6 in its own convention). Not used (frames not comparable with the Koch STEP/URDF; limits are in another zero pose). |

### License (quoted)

`LICENSE` of jess-moss/koch-v1-1 and of the s1lent4gnt fork (identical file, copied to `meshes/koch/LICENSE`) begins
"Copyright 2024 The Hugging Face team. All rights reserved." followed by "Apache License Version 2.0, January 2004" and the standard
Appendix boilerplate (it ends with "Copyright 2020 - present, Facebook, Inc ... Licensed under the Apache License, Version 2.0").
Apache-2.0 allows use, modification and redistribution with the license text and a statement of changes (`meshes/koch/NOTICE`). Koch v1.1 is
derived from Alexander Koch's MIT-licensed work; the MIT notice is reproduced in the NOTICE. Compatible with the project's
Apache-2.0 / MIT / BSD / CC-BY rule. Unverified: ROBOTIS' own terms for the servo CAD that sits inside the `*_motor.STL` meshes (the Koch repo
ships ROBOTIS' STEP files under its Apache-2.0 LICENSE and states nothing else); the files we ship from them are two convex hulls of 192 / 136 triangles.

## Simulation model chosen

**s1lent4gnt/koch-v1-1 `simulation/follower.urdf` + meshes at `cc1a4eb`**, copied to `meshes/koch/`. Why: it is a URDF (the format `tools/so101_lib.py`
parses), the meshes are in the link frames at metres with identity visual origins, each link has its servo(s) in a separate `link_N_motor.STL`, and
it is the very file Menagerie derived from. It is v1.1 geometry: the signed volume of each URDF mesh equals the volume of the corresponding printable STL
of jess-moss/koch-v1-1 at `1a553e1` (mm^3): base 24888.1 / 24888.1, shoulder rotation 6893.4 / 6894.3, shoulder-to-elbow 34356.3 / 34360.8,
elbow-to-wrist extension 47962 / 47982.5, elbow-to-wrist 4543.4 / 4543.0, gripper static 13418.8 / 13420.7, gripper moving 12843.1 / 12840.6.
The URDF is not an official Hugging Face file (a fork by a LeRobot contributor; the same meshes are in gym-lowcostrobot, whose commit it follows), and its joint limits
(+-3.14, effort 0.5, velocity 100) and masses are placeholders: masses are CAD volumes at about 1000 kg/m^3 (link_1 0.05014 kg = 50.6 cm^3 x 990, link_2 0.050177 kg), i.e. not real.
They are NOT used; printed-part masses come from mesh volume x PLA x infill (below) and servos from the datasheet. The kinematics (joint origins, axes) are the geometry of the v1.1 CAD.

Differences to the other v1.1 models, all sign / limit conventions at the same zero pose: gym-lowcostrobot and Menagerie flip the elbow axis to (0 -1 0)
(URDF: (0 1 0)); Menagerie limits: base_rotation -2.2..2.2, pitch -1.57..0.6, elbow -1.57..1.45 (its axis), wrist_pitch -2.0..2.0, wrist_roll -3.14..3.14, gripper -1.60..0.032
(README: "Added approximate joint limits."). The part files use those (elbow converted to the URDF axis: [-1.45, 1.57]) with fidelity `estimated`.

### Joints (URDF names; LeRobot names from `src/lerobot/robots/koch_follower/koch_follower.py` at lerobot `200ee53`: `"shoulder_pan": Motor(1, "xl430-w250", ...)`, `"shoulder_lift": Motor(2, "xl430-w250", ...)`, `"elbow_flex": Motor(3, "xl330-m288", ...)`, `"wrist_flex": Motor(4, "xl330-m288", ...)`, `"wrist_roll": Motor(5, "xl330-m288", ...)`, `"gripper": Motor(6, "xl330-m288", ...)`)

| URDF joint | LeRobot | parent -> child link | origin xyz in the parent link (m) | axis | URDF limit (rad) | part range used (rad, estimated) | servo | servo held by -> horn drives |
|---|---|---|---|---|---|---|---|---|
| joint_1 | shoulder_pan (ID 1) | base_link -> link_1 | 0.012 0 0.0409 | 0 0 -1 | -3.14 .. 3.14 | -2.2 .. 2.2 | XL430-W250-T | koch-base -> koch-shoulder-rotation |
| joint_2 | shoulder_lift (ID 2) | link_1 -> link_2 | 0 -0.0209 0.0154 | 0 1 0 | -3.14 .. 3.14 | -1.57 .. 0.6 | XL430-W250-T | koch-shoulder-rotation -> koch-shoulder-to-elbow |
| joint_3 | elbow_flex (ID 3) | link_2 -> link_3 | -0.0148 0.0065 0.1083 | 0 1 0 | -3.14 .. 3.14 | -1.45 .. 1.57 | XL330-M288-T | koch-shoulder-to-elbow -> koch-elbow-to-wrist-extension |
| joint_4 | wrist_flex (ID 4) | link_3 -> link_4 | -0.10048 5e-05 0.0026999 | 0 1 0 | -3.14 .. 3.14 | -2.0 .. 2.0 | XL330-M288-T | koch-elbow-to-wrist-extension -> koch-elbow-to-wrist |
| joint_5 | wrist_roll (ID 5) | link_4 -> link_5 | -0.045 0.013097 0 | 1 0 0 | -3.14 .. 3.14 | -3.14 .. 3.14 | XL330-M288-T | koch-elbow-to-wrist -> koch-gripper-static |
| joint_6 | gripper (ID 6) | link_5 -> link_6 | -0.01315 -0.0075 0.0145 | 0 0 -1 | -2.45 .. 0.032 | -1.60 .. 0.032 | XL330-M288-T | koch-gripper-static -> koch-gripper-moving |

(The original README of Alexander Koch says the same: "The XL430 motors are almost twice as strong and are used for the first two joints.")
q = 0 is the URDF zero (upper arm vertical, forearm along -x; read from the mesh bounding boxes). Every joint's URDF axis is the inverse of the servo output axis
(the horn outer face normal), so each mate needs `reverse = true` (stored as `joint_reverse = true` on the horn seats).

## BOM (follower), from the Koch v1.1 README "Follower Arm" table (US column; total $278, EUR 368, GBP 285, RMB 2251, JPY 15446)

| Part | Qty | Note |
|---|---|---|
| Dynamixel XL430-W250-T | 2 | $50 each; joints 1-2 |
| Dynamixel XL330-M288-T | 4 | $24 each; joints 3-6 |
| XL330 Frame and Idler Wheel 4pcs set (FPX330-H101) | 1 | footnote: "You will only use three idler wheels that come in this four piece set." |
| XL430 Idler Wheel set (HN11-I101) | 1 | $7 |
| Waveshare Serial Bus Servo Driver Board | 1 | $10; USB-serial to the Dynamixel TTL bus |
| Voltage Reducer | 1 | $14; steps the 12 V down for the 5 V XL330 chain (inferred from the original README step "adjust the screw on the voltage reducer until the input voltage is 5V" and the XL330 range 3.7-6 V; v1.1: "Removed the need ... for manually adjusting the voltage convertor, by replacing the DC convertor") |
| 12V Power Supply | 1 | $12; README: "power the leader arm using the 5V power supply, and the follower arm using the 12V power supply" |
| Jumper Wires 3x40 pcs set | 1 | $7 |
| Table Clamp | 1 | $6; C-clamp, not modelled; `koch-base` has a `table_seat` port on the plate underside |
| 1.5mm Star/Cruciform Screwdriver 2pcs set, USB C cable 2pcs set | 1 each | tools / host link, not modelled |
| 3D printed parts | 7 | Follower_Base, _Shoulder_Rotation, _Shoulder_To_Elbow, _Elbow_To_Wrist_Extension, _Elbow_To_Wrist, _Gripper_Static_Part, _Gripper_Moving_Part |

**Screws: the follower BOM lists none** (grep of the README: only "Four M2x5 Machine Screws" and "Four M2 Nuts" for the *leader* platform). The servo kits' own screws are assumed, not verified. No
screw part files were written, so the mates carry no fasteners. Print settings (README Printing the Parts, step 1): "Precision: 0.2mm minimum height layer", "Material: PLA+, ABS, PETG or other reasonably strong plastics.",
"Nozzle Diameter: 0.4mm maximum nozzle diameter", "Infill Density: ~30%" (footnote 8: "I used 37% to ensure strong parts."). The original v1.0 README: only the gripper moving part needs supports, bus 1 Mbps, IDs 1 (shoulder) .. 5 or 6 (gripper).

## Masses

Published: **none** (README of both repos, the LeRobot Koch doc and the ManiSkill docs state no arm or part mass). Only servo masses, from ROBOTIS e-manual: XL430-W250-T "Weight: 57.2 [g]", dimensions
28.5 x 46.5 x 34 mm, 258.5:1, 6.5-12 V, stall 1.4 N*m at 11.1 V; XL330-M288-T "Weight: 18 [g]", 20.0 x 34.0 x 26.0 mm, 288.4:1, 3.7-6 V (recommended 5 V), stall 0.52 N*m at 5 V
(https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/ and /xl330-m288/); Alexander Koch's README: "The XL330 motors are weaker but weigh only 18g each."

Printed parts (no `mass` param: the builder derives it, `V` solid mesh volume, PLA 1240 kg/m^3, 30 % infill, 0.4 mm nozzle, 2 walls (estimated), `mass = rho (V_shell + infill (V - V_shell))`, V_shell = min(V, A x 2 x 0.4 mm)), checked with `napl.design.mass.printed_mass`:

| Part | V (cm^3) | A (cm^2) | mass, 2 walls (g) | 1 wall / 3 walls (g) |
|---|---|---|---|---|
| koch-base | 24.888 | 187.69 | 22.29 | 15.77 / 28.81 |
| koch-shoulder-rotation | 6.894 | 69.65 | 7.40 | 4.98 / 8.55 |
| koch-shoulder-to-elbow | 34.361 | 136.98 | 22.29 | 17.54 / 27.05 |
| koch-elbow-to-wrist-extension | 47.982 | 106.55 | 25.25 | 21.55 / 28.95 |
| koch-elbow-to-wrist | 4.543 | 41.20 | 4.55 | 3.12 / 5.63 |
| koch-gripper-static | 13.421 | 77.65 | 10.38 | 7.69 / 13.08 |
| koch-gripper-moving | 12.841 | 52.00 | 8.39 | 6.58 / 10.19 |
| total printed | 144.93 | | 100.56 | |

Servos 2 x 57.2 + 4 x 18 = 186.4 g, so the modelled follower is about 287 g before idler frames, screws, cables and the driver board (estimated, not published; infill 30 % is the README's "~30%", the author used 37 %, the filament may be PLA+/ABS/PETG, not PLA).

## Meshes and ports

`meshes/koch/NOTICE` lists every file. All STL <= 1 MB (largest 693 KB), 4.1 MB in total, units metres, no decimation needed. The two servo meshes (`xl430_w250_t.stl`, `xl330_m288_t.stl`)
are convex hulls of the servo body cut out of the merged `*_motor.STL` meshes, because the cut-out bodies are not manifold (MuJoCo `inertia="exact"` fails on them and on the whole `*_motor.STL`, which are therefore not
referenced by any part). `link_1.STL` has 20 unmatched edges but loads with exact inertia; the other six printed meshes are closed (checked on hfbox with the `mujoco` wheel).

`tools/koch_ports.py` derives, from `meshes/koch/follower.urdf` only: the servo frame (origin = joint origin, z = -(joint axis), x = long body axis), the printed-part horn seats (`F = inv(J) S Rx(pi)`), the holders' servo seats (`F = S P_base Rx(pi)`), the base `table_seat`, the
servo `output` / `base` ports and geometry (`--servo`), and `--check` (rebuilds the chain from the ports in `parts/arm/*.toml`, at q = 0 and at three random q sets, and compares with the URDF forward kinematics, errors ~1e-17 m / 0 rad; tolerance 1e-9).
Port types (new families, names only; no hole pattern is claimed): `servo-horn.dynamixel-xl430` / `-xl330` (male on the servo, female on the child, symmetry 1), `bolt-pattern.dynamixel-xl430-base` / `-xl330-base` (frame convention on the body
face opposite the output, estimated, same approach as the SO-101 servo base), `flat-mount` (table). Servo geometry + mechanical ports for `robotis-xl430-w250-t` / `robotis-xl330-m288-t` are printed by `python tools/koch_ports.py --servo` for the later merge (S1 owns those files).

## Open items

- The Koch servo bays hold the servos in pockets by several faces; the `servo_seat` ports are frame conventions (estimated), so a builder `INTERFERENCE` check will likely flag servo vs printed part overlaps (the servo sits inside the printed pocket). Not tested: no Koch assembly was built in this stage.
- The idler frames (FPX330-H101 x3, HN11-I101) and the Waveshare board / voltage reducer / 12 V supply are not modelled (no mesh geometry beyond the merged `*_motor.STL`).
- Joint limits are third-party approximations (Menagerie) with a flipped elbow axis converted; hardware limits are the design's mechanical interference, not measured.
- ROBOTIS CAD terms (see License).
