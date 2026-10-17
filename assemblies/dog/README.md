# The dog (SP3b): recon quadruped assembly

`dog.toml` is the whole robot: 12 SteadyWin GIM8108-8 quasi-direct-drive joints in four 3-DOF legs, a printed + carbon chassis, a Livox Mid-360 + RealSense D435i head,
a Jetson Orin Nano with four CAN buses, two 8S1P Li-ion packs behind a 100 A contactor with an e-stop coil loop. It is a recon robot: no weapon, launcher or payload-release part,
device or wiring. Spec: `docs/superpowers/specs/2026-10-16-quadruped-design.md`; actuator study: `docs/dog/actuator-study.md`.

World frame: x forward, y left, z up. The root is the top carbon deck; the build drops the dog so the lowest foot-ball point is on z = 0
(the legs are straight in the build pose: hip axis at z = 0.4825 m, deck top face at 0.5455 m).

## Sub-assembly tree

```
dog.toml            deck_top (root), bulk_f, bulk_r, side_l, side_r             chassis
  fl, fr, rl, rr    leg.toml  (fr, rl mirrored "xz"; rl, rr on the rear bulkhead turned 180 deg about z)
  head              head.toml     head_mount, lidar (Mid-360, rosette), cam (D435i)
  power             power.toml    deck_bottom, 2 x (tray, pack, BMS), contactor, pdb, 2 x DC-DC, estop
  compute           compute.toml  tray, jetson, 4 x CANable, imu (MinIMU-9), rtk (ZED-F9P board), ant
leg.toml            abad_mount > abad_motor > hip_housing > {hip_motor, knee_motor, knee_crank} > thigh > {knee_rod, 2 rod ends} > shank > foot_core > foot_ball; hub
```

Joints per leg: `abad_j` (abduction, about the body x axis), `hip_j` (pitch), `knee_j` (crank + push rod, ratio 1.5). Devices per leg: three `qdd` (`<leg>:abad|hip|knee`) and one virtual
`foot_contact` (`<leg>:foot`, no foot sensor hardware). Other devices: `power:pack_a|pack_b` (battery), `head:lidar`, `head:depth`, `compute:imu`, `compute:gnss`.

## Layout rules and wiring

- The front bulkhead's `hip_l` / `hip_r` ports carry `fl` / `fr` (`fr` is the leg mirrored about the XZ plane); the rear bulkhead is the same part turned 180 deg about z by the deck's rear port,
  so its `hip_l` lies at world -y: `rr` (unmirrored) on `bulk_r.hip_l`, `rl` (mirrored) on `bulk_r.hip_r`. Front knees point backward, rear knees forward.
- CAN: one bus per leg (CANable on the compute tray, a 120 ohm terminated hub per leg, 3 drives each: about 40 % load at 1 Mbit/s, 500 Hz).
- Power: packs, BMS, 100 A contactor (coil on the 12 V aux DC-DC through the 10 A NC e-stop), PDB, one XT60 feed per leg hub. The Jetson and lidar DC-DCs hang on the battery side
  of the contactor: the e-stop drops joint power only.
- The leg hub has both drive outputs on its drive-side edge and is not its own mirror: the right-hand legs take the twin `dog-leg-hub-r`. The top deck has a 21 x 30 mm slot for the lidar's 12 V lead.
- Pin-level wires with a length and rationale each; wire and connector prices are NOT in the BOM (below).

## Decisions of the plan (numbers)

1. **Knee drive is a crank + push rod, not a belt.** 22 N m on a 20 T GT2 pulley (pitch radius 6.37 mm) is 3.45 kN of belt tension, an order above a 6 mm GT2 belt. Crank radius 20 mm, shank lug 30 mm: ratio 1.5, so
   the Qdd `knee` has `gear_ratio` = `knee_motor.drive_ratio` = 8 x 1.5 = 12.0 (export `knee_drive_ratio`, override per leg). Rod force at stall: 22 N m / 0.020 m = 1.10 kN. Euler buckling of the carbon
   rod (pin length 190 / 210 / 240 mm for the 200 / 220 / 250 mm legs): 5.06 / 4.15 / 3.18 kN, margin 2.9 x at the worst; the PHS5 rod end carries 9.12 kN static radial (8.3 x). Pinned by `test_knee_rod_does_not_buckle_at_peak_knee_torque`.
2. **Two 8S1P packs, two 60 A BMS in parallel:** 140 A pack / 120 A BMS continuous against the study's 73-87 A peak.
3. **E-stop on the contactor coil:** Altran ALEV100-B (100 A carry, 12 V coil, 0.461 A) in the motor feed, DC-DCs before it. Margins against 87 A: packs 140 A (1.6 x), BMS 120 A (1.4 x), contactor 100 A (1.15 x),
   PDB input XT90 90 A (1.03 x, the thinnest). Inrush precharge is an open item (not modelled, not a part). Pinned by `test_estop_switches_only_the_coil_and_ratings_cover_the_peak`.
4. **Leg chain reworked:** abad mount (bulkhead) > abduction hinge > hip housing (pitch + knee motors) > thigh > shank. Hips bolt to the bulkheads, abduction axis along x.
5. **Length variants are real parts** (`-200`, `-250`, with right-hand twins), the sweep writes variant files. Overridable exports: `knee_drive_ratio`, `lidar_rays` (2000 rays / frame here; SP5 raises it).
6. **FootContact is virtual** (stand-in for the SP4 torque-based estimate); no foot sensor in the BOM.
7. **RTK on the Jetson UART, IMU on its I2C, the four USB-A sockets take the four CANables,** the D435i takes the USB-C port.
8. **Front knees backward, rear forward** (the rear legs are the front legs turned 180 deg).

## Mass (hfbox build, `napl design build`)

| group                                                                           | kg                                                              |
| ------------------------------------------------------------------------------- | --------------------------------------------------------------- |
| legs (4 x 1.670)                                                                | 6.864                                                           |
| power (2 packs 1.14, BMS 0.35, contactor 0.19, PDB, DC-DCs, trays, deck, wires) | 2.602                                                           |
| chassis (top deck 0.397, bulkheads, side plates, mounting)                      | 0.929                                                           |
| compute (Jetson devkit, 4 CANables, IMU, RTK board, antenna, tray)              | 0.680                                                           |
| head (Mid-360 0.265, D435i, mount, splitter cable)                              | 0.578                                                           |
| **total**                                                                       | **11.653** (target 12 kg, gap -2.9 %, within tolerance 0.53 kg) |

Point-mass centre of mass (every instance at its part frame, an approximation): 9.4 mm forward and 0.4 mm left of the hip centroid, z = 0.472 m. Trunk inertia about the CoM from the same point masses,
`sum m_i (|r_i|^2 I - r_i r_i^T)`: Ixx 0.204, Iyy 0.538, Izz 0.614 kg m^2 (off-diagonals below 0.004). An estimate for SP4 to identify, not a measurement.

## BOM

Prices come from each part's `price` param (dated 2026-10-16 in the part files); wires and connectors are not counted, nor are fasteners. Printed parts are filament only (20 USD/kg).

| sub-assembly                               | USD      |
| ------------------------------------------ | -------- |
| fl, fr, rl, rr (698.70 each)               | 2,794.80 |
| head                                       | 1,150.24 |
| compute                                    | 931.80   |
| power                                      | 571.56   |
| chassis (top deck, bulkheads, side plates) | 105.36   |

BOM total (lower bound, wires excluded): 5,554 USD

That is above the 3-4.5 k USD target of the spec: the 12 actuators alone are 2,568 USD. 21.8 % of the total is price-estimated (Mid-360 749 USD vendor price, D435i / Jetson at listed
prices, BMS, PDB, printed parts). Separate estimated line, not in the total: cables, connectors and fasteners, 80 USD (the study's allowance). The carbon sheet prices are lower bounds (sheet
granularity and cutting are not priced); the Livox splitter cable (80 USD) is a wire part and is also outside the total.

## Open items

- Inrush precharge for the 100 A contactor and the PDB's 940 uF bus capacitance (not modelled).
- The GDS68 driver's MIT-mode protocol is assumed to be the CubeMars/GDS68 sibling (the GIM8108-8 pages do not state it); Qdd parameters are in `steadywin-gim8108-8.toml` with provenance.
- The knee linkage ratio varies 1.46-1.61 over the knee range (the Qdd uses the constant 1.5), and the hip / knee cross-coupling of a hip-mounted knee motor is not modelled.
- The D24V22F12 DC-DCs are rated 36 V in: 2.4 V above the 33.6 V full pack, no TVS, regenerative spikes are absorbed by the PDB capacitors only.
- No foot sensor hardware; the BMS is not simulated (its FET resistance and 60 A cut-off); the Ethernet pairs of the lidar cable carry no ground (a `PIN_GND_MISSING` warning).
- Printed parts: the 420 mm side plates do not fit the 220 mm bed (print as two halves with a lap splice).
