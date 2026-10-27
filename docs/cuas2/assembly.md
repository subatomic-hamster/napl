# cuas2 assembly (CAD + `assemblies/cuas2.toml`)

Spec: `docs/cuas2/spec.md` sections 3-5, 12 (MECH). CAD: `cad/cuas2/` (build123d, six modules). Library parts: `parts/cuas2/` (19 printed parts, mesh `meshes/cuas2/<name>.stl`) plus bought parts from `parts/`.
Wiring: `wiring.md`. Cost: `bom.md`.

## Build and checks

```
cad-venv/bin/python cad/cuas2/build.py --out build/cad/cuas2 [--meshes meshes/cuas2]   # checks C-GAP .. C-FIT, prints PASS
PYTHONPATH=python python3 -c "import napl.design as d; print(d.check('assemblies/cuas2.toml'))"   # DRC, [] = clean
bash cad/cuas2/negctl.sh    # negative controls through CUAS2_GATE_SWING_DEG / CUAS2_WHEEL_D / CUAS2_BALL_D / CUAS2_POCKET_D_BALLS
python3 -m pytest -p no:cacheprovider tests/cuas2/test_cad_consts.py   # ball diameter and mass equal cad/cuas-lite
```

| Check       | Value at nominal                                                                                                                                                |
| ----------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| C-GAP       | wheel-surface gap 19.197 mm (want 19.20 +-0.05); slot travel +-1.5 mm reaches 17.70..20.70 mm                                                                   |
| C-POCKET    | pocket bore length 28.00 mm (plug diameter); limit 39.96 mm; one ball fits, two centres cannot                                                                  |
| C-FEED      | min feed slope 20.0 deg at tilt -15/0/35/70; hopper 229 cm3 for 24 balls                                                                                        |
| C-NIP       | pusher stroke 47.2 mm (>= 14.5); retracted rod clears the valve; extended rod blocks it from 15 deg of plug turn (1152 mm3)                                     |
| C-INTERLOCK | rod swept volume vs plug: 0 at BORE, 1152 mm3 at LOAD (60 deg)                                                                                                  |
| C-BAL       | head 523 g (<= 600); CG offset from the tilt axis 3.6 mm (<= 8); static torque 0.003 / 0.007 / 0.016 / 0.018 N*m at -15 / 0 / 35 / 70 deg (<= 0.06); no ballast |
| C-DOCK      | D1, D2, F1, H1, Y1: same bolt pattern and datum both sides, clearance 0.297 mm                                                                                  |
| C-PATH      | ball path clear, swept head vs yokes/rotor/base overlap 0 mm3                                                                                                   |
| C-FIT       | every part fits 200 x 200 x 200 mm (largest: lower deck 200)                                                                                                    |
| C-CYCLE     | info only: geometry cycle 466 ms (spec 365, R3 400)                                                                                                             |

Negative controls (all confirmed): `GATE_SWING_DEG=90` changes the interlock volume (1505 mm3) and the cycle (566 ms); `WHEEL_D=50` changes the wheel spacing (69.2 mm) and rpm cap (14514); `BALL_D=23.0` changes the gap (19.997), hopper (255 cm3) and
stroke; `POCKET_D_BALLS=2.2` makes C-POCKET FAIL (bore 48.84 mm) and the script exits 1.

## Modules and frames

| Module                | Parts                                                                               | Frame                                            |
| --------------------- | ----------------------------------------------------------------------------------- | ------------------------------------------------ |
| M1 base / pan         | base plate, 4 legs, lower deck (Jetson, fuse, XT60), rotor plate                    | B: origin on the pan axis at the stator top face |
| M2 yokes              | left yoke (tilt motor, hall), right yoke                                            | B, tilt axis H at z = 200 mm                     |
| M3 launcher cartridge | cartridge cage, gap plate, wheel hubs, TPU tyres, two 2205 motors, two ESCs         | head frame H (identity)                          |
| M4 feeder             | valve housing, plug, pusher rod, hopper, hopper lid, two SG92R, beams, halls        | H                                                |
| M5 sensor head        | sensor bar, Brio                                                                    | H                                                |
| M6 power box          | box and lid under the base, relay, Pico, TMC2209 x2, regulators, safety electronics | B                                                |

Docks (data in `interfaces.py`, consumed by both mating modules): D1 dovetail (cartridge to cradle), D2 camera, F1 flange (cartridge to valve housing), H1 (power box to base and head), Y1 (yoke to rotor).
The cradle (head carrier, y = +-38 arms) hangs from the yokes on the tilt axis at the hinge port; the printed trunnion stubs ride in 608ZZ bearings.

## Assembly file

- Root `base` (static); pan group (rotor, yokes, lazy susan, tilt motor and pulley, bearings) mated by `pan_joint` (hinge, +-170 deg); tilt group (cradle and everything on it) by `tilt_joint` (hinge on `yoke_l.tilt_axis`, `cradle.hinge`, -15..+70 deg).
- Printed parts keep their CAD frame; identity mates run from `cradle.frame_up` to each `<part>.origin`, bought parts from `cradle.seat_*` ports. Closure mates declare the bolted pairs (cage to gap plate and valve housing, hopper to lid and valve housing, right yoke to cradle).
- Launcher device `gun`: `cuas2-foam-launcher` on `cradle.seat_launcher` (muzzle 70 mm after the nip), device map `foam_launcher` with `cl_k` and `spin_tau`; the marker's `muzzle` port has the barrel axis +x and ref up (site x = up, site z = barrel, spin axis = site x cross site z), as MODEL expects.
- Instances not modelled as joints: the plug hinge and the pusher slider (shown at BORE and retracted), foam rounds, servo and camera devices, belts, the buzzer and DIAG pin.
- Two legs (leg1, leg4) stand 0.2 mm above the deck top: the DRC mesh contact reports a 4 mm overlap for flush faces, 0.2 mm is inside the print tolerance of a bolted joint.

## Changes made while building the CAD (deviations from spec, see the report)

- Plug and pocket length 28 mm (spec text said 26 and 30); pusher stroke 47.2 mm (spec 32) because the rod must clear the plug cylinder; the geometric cycle is 466 ms (spec 365 / R3 400): relax R3 to about 0.5 s or fit faster servos.
- GATE_SWING 60 deg LOAD pose is front-up; a ball-stop lip ring at x = -22 holds the staged ball; the breech beam is 9 mm behind the plug axis (the gate servo sits on the plug axis) and the gate-home magnet pocket is in the plug's -y end face (hall on the -y side).
- Tilt axis H = 200 mm (v1: 202); power box 60 mm deep (spec 40) and legs 120 mm so the Jetson clears it.
- No GT2 teeth, hard-stop lugs mostly unmodelled, optional TF-Luna pocket without part, thermal pocket and second camera not built, lazy-susan bolt radius unmeasured (as v1).
