# cuas-lite build guide

Plain text only. Figures are referred to by name; none are committed. The CAD renders them on a build host: `cad/cuas-lite/build.py` (without `--no-preview`) writes the "iso tilt 0", "iso tilt +70", "iso tilt -15",
"side", "front" and "top" pictures to `build/cad/cuas-lite/preview/` (gitignored). Numbers marked **[est]** are recommendations, not CAD or datasheet values; everything else is
from `cad/cuas-lite/` (`params.py`, `parts.py`, the build manifest of CAD d165505) or from `docs/cuas-lite/spec.md`. Coordinates: B = stator top face centre on the pan axis (x forward, y left,
z up); H = tilt axis (B z = 202 mm), x along the barrel. Bill of materials: `docs/cuas-lite/bom.md`. Wiring tables: `docs/cuas-lite/wiring.md`. Bench tests: `docs/cuas-lite/bench_validation.md`.

Read section 8 (safety) before you plug anything in. The weapon rail stays dead (key out, E-stop pressed, fuse out) until bench_validation section 1.4 has passed.

## 0. Before you print: measure the lazy susan

The stator posts and the rotor plate carry the lazy-susan bolt holes at `LS_BOLT_R` = 63.5 mm (4 holes per ring, rings offset 45 deg). **No vendor publishes the real hole pattern** (bom.md, "Lazy susan").
Buy the bearing first, measure the bolt-hole circle radius, and if it is not 63.5 mm change `LS_BOLT_R` in `cad/cuas-lite/params.py` and rebuild (`cad-venv/bin/python cad/cuas-lite/build.py`
on the build host; the checks re-run) before printing `base_stator_plate` and `pan_rotor_plate`. Also confirm the clear opening (the CAD uses 117.5 mm) and the height (9 mm).

## 1. Print list

Printer: 220 x 220 x 250 mm bed or larger, 0.4 mm nozzle, 0.2 mm layers, 2 perimeters, 20 % infill (the CAD masses and the tilt torque budget assume exactly this; see section 6 before changing
infill on any tilt-group part). The STL files in `build/cad/cuas-lite/stl/` are already in the print orientation (the CAD picks the best of six axis orientations, fewest overhangs); do not rotate them.
Supports: from the CAD's own overhang check ("none", "minor" = small local supports, "yes" = support needed). Temperatures [est]: PLA 205-215 C nozzle / 60 C bed; PETG 235-245 C / 75-80 C, part fan low.
Total: 30 part types in the manifest (29 printed parts plus the cable-clip accessory), 1099.7 g of filament, about 41 h of print time (CAD estimate).

Material rule from the CAD: PETG for loaded, hot or flexing parts (plates, yokes, chassis, pulleys, beam); PLA for covers, mounts, the hopper and camera parts.

| part | qty | group | material | supports | printed size mm (x y z) | filament g | time h |
|---|---|---|---|---|---|---|---|
| base_stator_plate | 1 | static | PETG | none | 200 x 200 x 26 | 153.3 | 5.7 |
| base_leg | 4 | static | PETG | none | 80 x 20 x 20 | 14.1 each | 0.5 each |
| electronics_tray | 1 | static | PLA | minor | 196 x 129 x 25 | 81.8 | 3.1 |
| pan_motor_mount | 1 | static | PETG | none | 56 x 56 x 4 | 8.1 | 0.3 |
| pan_hall_mount | 1 | static | PLA | none | 12 x 24 x 24 | 2.1 | 0.1 |
| front_panel | 1 | static | PLA | none | 70 x 120 x 16 | 22.8 | 0.9 |
| cable_clip | 6 | static | PLA | none | 13.8 x 20 x 10 | 1.2 each | 0.05 each |
| pan_rotor_plate | 1 | pan | PETG | none | 180 x 180 x 8 | 121.8 | 4.5 |
| pan_pulley_100t | 1 | pan | PETG | none | 66.1 x 66.1 x 25.5 | 24.5 | 0.9 |
| cable_collar | 1 | pan | PLA | none | 38 x 38 x 9 | 3.3 | 0.1 |
| pan_stop_tab | 1 | pan | PETG | none | 24 x 12 x 24 | 1.2 | 0.04 |
| yoke_left | 1 | pan | PETG | minor | 84 x 24 x 199 | 75.4 | 2.8 |
| yoke_right | 1 | pan | PETG | minor | 84 x 24 x 199 | 71.6 | 2.6 |
| tilt_motor_mount | 1 | pan | PETG | none | 92.2 x 76 x 29 | 23.6 | 0.9 |
| feed_elbow | 1 | pan | PETG | yes | 85.3 x 72 x 127 | 41.0 | 1.5 |
| hopper | 1 | pan | PLA | none | 88.8 x 88.8 x 138.5 | 61.3 | 2.3 |
| hopper_lid | 1 | pan | PLA | none | 100.8 x 88.8 x 7 | 17.8 | 0.7 |
| tilt_hub | 1 | tilt | PETG | none | 34 x 34 x 8 | 4.3 | 0.2 |
| tilt_pulley_80t | 1 | tilt | PETG | none | 53.4 x 53.4 x 15 | 12.4 | 0.5 |
| launcher_chassis | 1 | tilt | PETG | yes | 49.4 x 100 x 153 | 166.2 | 6.1 |
| trunnion_beam | 1 | tilt | PETG | yes | 39 x 77 x 124.5 | 51.0 | 1.9 |
| barrel_half_a, barrel_half_b | 1 each | tilt | PETG | minor | 50 x 25 x 110 | 21.1, 21.2 | 0.8 each |
| piston | 1 | tilt | PETG | none | 22.2 x 22.2 x 48 | 8.1 | 0.3 |
| servo_bracket | 1 | tilt | PETG | none | 72 x 52 x 15 | 10.3 | 0.4 |
| pusher_arm | 1 | tilt | PETG | none | 15 x 40.9 x 3 | 1.3 | 0.05 |
| wheel_cover_pos_y, wheel_cover_neg_y | 1 each | tilt | PETG | none | 16.6 x 53.6 x 2.8 | 2.4 each | 0.09 each |
| camera_mount | 1 | tilt | PLA | none | 44 x 110 x 15 | 17.1 | 0.7 |
| camera_shim | 2 | tilt | PLA | none | 26.5 x 92 x 2.2 | 2.3 each | 0.09 each |

Print notes:
- Print `base_stator_plate`, `pan_rotor_plate` and `launcher_chassis` first and fit-check them (sections 2 and 3) before committing to the rest; they carry most of the inserts and bearing seats.
- Test the ball path early: print `barrel_half_a`/`b` and drop a Rival round (22.2 mm) through the 24 mm bore. It must pass without a catch; if it catches, ream by hand, do not scale the model.
- The hopper neck and elbow bores are 26.4 mm for a 22.2 mm ball: calibrate the printer's extrusion so that a bore comes out within 0.3 mm [est].
- Deburr the first layer ("elephant foot") of every part that has a bearing seat or a pulley bore with a knife.
- Do not change infill on tilt-group parts (tilt_hub, tilt_pulley_80t, launcher_chassis, trunnion_beam, barrels, piston, servo_bracket, pusher_arm, wheel covers, camera_mount, camera_shim). Static margin is 12 % (section 6).
  Pan-group and static parts may take 3 perimeters [est] for stiffness; only the pan axis inertia changes (about 1 % per 10 g).

## 2. Heat-set inserts: locations and counts

Hole: 4.2 mm, 6 mm deep, for M3 x 5.7 mm brass inserts (M4 posts: 5.6 mm, 8 mm deep). **Total from the CAD hole count: 82 M3 inserts and 4 M4 inserts** (the BOM buys 100 M3 + 4 M4). Counts by part
(method: `cad/cuas-lite/parts.py` run with `hole()` instrumented):

| part | M3 | M4 | where |
|---|---|---|---|
| base_stator_plate | 3 | 4 | M4: top of the four lazy-susan posts, on the bolt circle `LS_BOLT_R` at 45/135/225/315 deg. M3: two on the underside at x = +96, y = +-40 (front panel ears), one in the top face at radius 89.75 mm, angle -90 deg (the pan hard-stop stud) |
| base_leg (x4) | 2 each = 8 | | one in the top (stator screw), one in the side face at z = -64 (tray ear) |
| electronics_tray | 4 | | tops of the four standoffs (Jetson dev kit hole pattern) |
| pan_rotor_plate | 18 | | 16 in the top face for the yoke feet (4 x-positions x = -30, -12, 12, 30, times 2 y-positions, times 2 yokes); 2 in the underside at x = +-7, y = 84 (pan stop tab) |
| pan_pulley_100t | 6 | | on a 23 mm radius circle, 30 deg + k x 60 deg (pulley screws, flat heads from the top of the rotor plate) |
| yoke_left | 4 | | at x = +-33, two heights (tilt axis height and one belt-centre distance below it): tilt motor mount |
| yoke_right | 4 | | around the 6807 seat at x = +-22, z = +-22 from the axis: feed elbow plate |
| feed_elbow | 4 | | hopper flange, on an 18 mm radius circle (hopper bolts) |
| tilt_hub | 4 | | on an 11 mm radius circle: the 80T pulley screws |
| launcher_chassis | 22 | | 4 cage top (trunnion beam), 4 front (barrel halves), 2 bottom (servo bracket), 8 at the wheel-cover recesses (4 per side), 4 radial motor clamps (2 per side) |
| barrel_half_a, barrel_half_b | 2 each = 4 | | ears (camera mount) |
| piston | 1 | | bottom (pusher pin) |

Method [est]: soldering iron with an insert tip at 220-245 C (PLA 200-220, PETG 230-250); iron and insert vertical, press until the insert top is flush with or 0.2 mm below the surface, let the iron
off at once, do not push after it cools (it floats). Insert first, then screw a spare M3 screw in and out once to clear melted plastic. Do the pan_rotor_plate and chassis first (most inserts); stop and check
the thread depth of the first insert in each part with an M3 x 8 screw before doing the rest. Spares: 18 M3 in the pack of 100.

## 3. Prep and fit checks (before assembly)

1. **Hall magnets and sensors (bench).** A3144 is active-low and responds to a south pole facing its marked face [datasheet behaviour; check by test]: put the magnet near the sensor with power on (3.3-5 V, 10k pull-up to the
   output) and mark the magnet face that turns the output low. Both magnets are 6 x 2 mm discs; keep both with the same marked face toward the sensor.
2. **Bearings.** The 608ZZ (8 x 22 x 7) goes in the left yoke seat (22.2 mm seat from the cheek's outer face): it must press in with thumb force. The 6807ZZ (35 x 47 x 7) goes in the right yoke
   seat (47.2 mm seat from the outer face), same rule. If the fit is loose, wrap one layer of PTFE tape or a drop of retaining compound; if too tight, scrape the seat, do not heat the bearing. Spin each by
   finger: smooth, no grit. The 6807ZZ inner race must slide onto the 35 mm trunnion tube of the beam (tube OD = 35.0 mm): test-fit it now and sand the tube if it binds.
3. **Rod.** The 8 mm stainless rod is the left tilt stub: it runs y = 39 to 76.5 mm in H (through the 608, the tilt_hub bore 8.2 mm and into the beam boss). Test that it slides through the 608 and the hub
   bore with a light push and no rocking; ream the printed bore with an 8 mm drill by hand if tight.
4. **Pulleys.** GT2 20T pulleys (5 mm bore) go on the NEMA shafts; the printed 100T and 80T pulleys have a printed tooth groove [est: the print tooth form is rough]: lay a belt around each, it must sit in the
   grooves without riding on the flange and rotate with no bind. File teeth lightly if the belt is tight. Flanges: 1.5 mm, overhang 1.6 mm.
5. **Belts.** Pan belt: 252 mm closed GT2 (126 teeth), centre distance 60.56 mm (slot travel +-4 mm; +-1 tooth gives 59.46 to 61.66 mm). Tilt belt: 204 mm (102 teeth), centre distance 48.16 mm (47.07 to 49.25 mm).
   Check the stock belt lengths when they arrive; a different tooth count is covered by the slot only up to +-1 tooth.
6. **Motors.** 17HS4401S NEMA 17: body 40 mm, 31 mm hole pattern, 22 mm pilot. Test-fit each in its mount. Flywheel motors (180 size, 20 x 15.3 mm flat can, 32 mm long) go in the chassis pockets; they must slide in with a
   slip fit and be clamped by the two radial M3 screws per side.
7. **Ball gauge.** Print a 22.2 mm / 24 mm test ring or use a drill bit: the hopper neck bore (26.4 mm), elbow, spigot (24.0 mm ID), trunnion bore (28.4 mm), chute (28 mm) and barrel (24 mm) must all pass
   a ball. Keep one clean ball for the bench feed check (section 7).

## 4. Sub-assemblies

All screws are M3 from the kit (8, 10, 12, 16 mm). Use a drop of thread lock [est] only on metal-to-metal; printed parts self-lock.

### 4.1 Stator (static group)
1. Insert the M3/M4 inserts (section 2) into `base_stator_plate`, the four `base_leg`, `electronics_tray`.
2. Screw the four legs to the stator underside (countersunk screws from the top into the leg inserts; the legs sit at the corners). Screw the tray to the legs through the tray ears into the side inserts.
3. Fit `pan_motor_mount` under the stator: four M3 through the slots (each slot gives +-4 mm of belt adjustment), shaft through the stator slot. Keep the screws just snug; they are tightened at the belt step.
4. Fit `pan_hall_mount` at x = -80 on the stator top: two M3 from the top, nuts in the underside hex pockets. Glue the A3144 into the pocket, its marked face up, 3.0 mm below where the rotor magnet will sit; thread
   its three leads through the wiring slot down to the tray.
5. Fit `front_panel` to the stator front edge (two M3 into the underside inserts at x = +96, y = +-40): key switch (hole at y = +28 mm, z = -43) and the E-stop (22 mm hole at y = -44 mm, z = -43). Tighten the
   panel nuts; the E-stop contact block is 35 mm deep and clears the pan motor by design.
6. Mount electronics on the tray: Jetson dev kit on the four standoffs (M3 into the insert tops), Pico, two TMC2209 boards, LM2596, the relay, the two MOSFETs and diodes, the fuse holder, the XT60 inlet at the rear wall.
   Pin map: `docs/cuas-lite/wiring.md`. Do **not** connect the fuse or the supply now.
7. Pan stud: screw the M3 x 14 stud (M3 x 16 from the kit, cut or ground to 14 mm [est]) into the stator insert at radius 89.75 mm, angle -90 deg, to a protrusion that clears the tab blade by 1 mm at the stop (check in 4.6).

### 4.2 Rotor (pan group base)
1. Insert the M3 inserts into `pan_rotor_plate` (16 yoke inserts, 2 tab inserts) and `pan_pulley_100t` (6).
2. Fit the lazy susan: the lower ring on the four M4-insert posts of the stator (M4 screws), the upper ring to the rotor plate underside (the CAD plate has through holes on the same circle). Spin the rotor by
   hand: it must turn evenly with no tight spot; shim with washers if the posts are not coplanar.
3. Press `cable_collar` into the 28 mm rotor hole.
4. Fit `pan_pulley_100t` on the rotor plate top, concentric with the bearing, six countersunk M3 into its inserts. Check run-out by eye: the belt groove should stay centred over the motor pulley through a full turn.
5. Glue the pan hall magnet (6 x 2 mm disc, marked face down toward the sensor) in the pocket at B (-80, 0, 27 mm), flush with the underside. The sensor window is about +-2 deg.
6. Fit `pan_stop_tab` on the rotor underside at +90 deg (two countersunk M3 into the inserts at x = +-7, y = 84 mm). The tab blade at radius 89.75 mm meets the stud at +-177.9 deg.
7. Fit the pan group parts to the rotor plate with the yoke feet screws: `yoke_left`, `yoke_right` (four M3 per foot, 16 in total), `tilt_motor_mount` to `yoke_left` (four M3 into its inserts), and the tilt NEMA 17 on the mount (flange on the mount,
   shaft toward -y, flange face at y = 97 mm, 20T pulley on the shaft with its belt plane at y = 86 mm).
8. Press the 608 in the left seat and the 6807ZZ in the right seat (section 3).
9. Hopper group (bolts only; the hopper is **not** on the head): `feed_elbow` on the outer face of `yoke_right` (four M3 into the right cheek inserts, x = +-22, z = +-22 around the 6807 seat), `hopper` on the elbow flange (four M3 on r = 18 mm),
   `hopper_lid` last (leave it off until the ball-feed check).
10. Fit the tilt hall sensor in the pocket of `yoke_left` at B (0, 68, 215), marked face toward +y. Lead out through the cheek.

### 4.3 Head (tilt group)
1. Insert the chassis inserts (22), the barrel inserts (4), the piston insert (1) and the hub inserts (4) (section 2).
2. Flywheel motors: each 180 motor in its pocket from the top, shaft down into the wheel cavity, clamped by the two radial M3 screws; press a 35 x 7 mm rubber wheel on each shaft. The rim gap must measure 19.2 mm
   (ball 22.2 minus 3.0 mm squeeze): measure with calipers and adjust the wheel position on the 2 mm shaft. Close each wheel window with its `wheel_cover` (M3 x 8 into the cover inserts).
3. `trunnion_beam` on the chassis cage top: four M3 into the cage inserts. This is a hollow beam; its 35 mm tube carries the right 6807ZZ and the ball path. Do this after the motors are in (the CAD notes the
   beam bolts go on after the motors).
4. Servo: SG92R in the `servo_bracket`; bracket on the chassis underside (two M3 into the bottom inserts); `pusher_arm` on the servo horn, the `piston` on the arm pin. Move the servo by hand to check the piston stroke
   (44 mm) reaches the ball rest point and clears the wheels; do not power the servo yet.
5. Barrel: `barrel_half_a` + `barrel_half_b` clamp on the chassis front (four M3 into the chassis front inserts); `camera_mount` on the barrel ears (M3 into the ear inserts), Brio clamped with its 1/4-20 screw;
   `camera_shim` pairs set the boresight (+-3 deg) later (bench_validation section 4).
6. `tilt_hub` on the 80T pulley (four M3 into the hub inserts); magnet in the hub pocket at H (0, 68.5, 13 mm) with its marked face toward the cheek sensor.

### 4.4 Joining the head to the pan group
1. Slide the head between the cheeks with the beam's tube entering the right 6807ZZ (inner race on the 35 mm tube) and the left side lining up with the 608.
2. Push the 8 mm rod through the 608 and the hub (rod span y = 39 to 76.5 mm in H, per the CAD; check against the "iso tilt 0" figure); lock the hub on the rod with its M3 set screw (the hub has a side-loaded nut for it).
3. The tilt axis should turn by hand with no side play. If there is axial play, shim between the hub and the 608 with 0.2 mm washers.
4. Fit the tilt belt (204 mm) round the 20T motor pulley and the 80T pulley; tension per 4.5. Put the head at tilt 0 (barrel level): the hub magnet must face the cheek sensor with a gap of about 0.5 mm plus the pocket depth.

### 4.5 Belts and tension
Slots are +-4 mm on the pan motor mount and +-4 mm on the tilt motor mount (belt adjustment).
1. Loosen the motor screws; place the belt on the two pulleys. 2. Pull the motor away along the slot with a spring balance (about 5 N preload per side [est]; a 500 g mass on a string over a bar does the same), tighten the four
screws while holding the pull. 3. Rotate the output pulley by hand through a full turn: no tight spot, the belt must not climb the flange. 4. The target is the same deflection on both belts under a 1 N weight at mid-span;
bench_validation 2.1 sets the real figure. 5. Keep the motor shaft radial load below 28 N (the shaft limit): 5 N preload plus the dynamic load (15.7 N for the pan) stays under it.

### 4.6 Pan hard stop check
Turn the rotor slowly by hand: the tab must meet the stud at +-177.9 deg (+- 2 deg) from the hall zero. If the tab misses the stud, raise the stud (a washer under its head) rather than sanding the tab.
Check that nothing else hits at the stop: cables (4.7), the hopper elbow and the tilt motor against the stator and legs.

### 4.7 Pan cable loop for +-178 deg
The head leads must pass the turntable. The leads come off the tray, go up through the 40 mm centre hole of the stator and the 20 mm hole of the cable collar (28 mm hole in the plate). The rule: **the loop
is sized for +-178 deg, not the +-170 deg software range, because the homing search can drive into the hard stop (spec 2.3).**
1. Gather every lead that goes up to the pan group into one bundle: the head leads (flywheel motors x2, servo power and signal, Brio USB) and the tilt NEMA 17 and tilt hall leads. About 8-10 mm bundle diameter [est].
2. Clip the bundle at the stator side (a `cable_clip` on the tray or stator underside, near the centre hole) and at the rotor side (a `cable_clip` on the rotor plate top beside the collar). Between the clips leave a free
   length of at least 100 mm [est: 10 to 12 bundle diameters for 180 deg of twist each way] so the bundle twists instead of pulling; the centre hole keeps the twist on the pan axis.
3. Test: turn the rotor by hand to each hard stop (tab on stud, +-177.9 deg), watch the bundle: no tension on a solder joint or a connector, no pinch at the collar, bend radius above 5 bundle diameters [est].
4. Keep the high current weapon-rail leads (22 AWG) separate from the USB and signal leads in the bundle by 5 mm at the clips, to limit noise [est].

### 4.8 Head wiring across the LEFT trunnion
The head leads cross the tilt axis on the LEFT side; the right trunnion is the hollow ball path and must stay clear.
1. From the rotor plate, run the bundle up the **inner** face of the left cheek (the side facing the head, y 45 to 51 mm), not outboard where the tilt belt, pulley and motor are.
2. At the axis, leave a service loop that follows the trunnion: about 150 mm of extra length, curled around the axis, bend radius at least 5 bundle diameters [est], so that the head tilting -18 to +73 deg (hard stops) twists the loop
   without stretching it. Secure the loop to the head at the beam boss (left side) with a `cable_clip`.
3. Fan out inside the head: flywheel motor leads (0.85 m runs in the wiring tables) to the cage pockets, servo lead to the bracket, Brio USB (1.0 m) to the camera mount. Strain-relieve each at its
   connector with a clip or zip tie on the chassis.
4. Check by hand: tilt the head through its full range with the bundle fitted: no snag on the belt, the hub, the barrel clamp or the wheel covers, and no tension at the connectors.

## 5. Final assembly checklist (order)
1. Stator (4.1), rotor (4.2), head (4.3), join (4.4), belts (4.5).
2. Magnets/sensors (3.1, 4.1.4, 4.2.5, 4.3.6, 4.2.10): output low only with the magnet near, with the head at tilt 0 and the rotor at pan 0.
3. Cable loop (4.7) and left trunnion wiring (4.8). Wiring from `wiring.md`, tray first.
4. Hopper lid on after the ball-feed check (7). Balls out until section 7.
5. Homing direction: the firmware homes from any start; the final approach is **always positive**, so the zero is the rising edge of the hall window met moving positive (spec 2.3). Fit the magnets so the window is met in that
   order: pan zero at the rotor position where the pan axis points forward (x), tilt zero with the barrel level. Home offsets are set later (bench_validation 1.3).

## 6. Tilt balance: static torque budget and ballast fallback

CAD result (d165505, `head_static_torque_within_spec_budget`): the tilt group (454 g empty, 462 g with the ball chute loaded) has its centre of gravity **34.5 mm below the tilt axis** and 1.9 to 2.4 mm ahead of it. Static
torque about the axis is **0.154 N*m (empty) / 0.156 N*m (loaded)** against a budget of **0.177 N*m** (the original 1.2 kg x 15 mm); the margin is about 12 %. A 0.156 N*m torque at the motor (gear 4, 90 % efficiency)
is 0.043 N*m, about 12 % of the stepper holding torque (0.36 N*m, spec 6.3). A free head hangs 3 deg nose-down from level (atan(1.9/34.4)).

Check on the finished head (before the belt is on):
1. Put the head at tilt 0 with the belt off, hang a spring scale on the muzzle and pull up to level. **Expect 1.2 N at the muzzle** (0.156 N*m / 0.130 m). Pass: at most 1.36 N (= 0.177 / 0.130).
2. Or let it hang free: it should settle within 3 +- 2 deg nose-down. A head that hangs far from this has added mass off-axis (wiring slack, Brio position): re-seat.
3. Every addition on the head (wire bundle, wheels, a bigger camera) shifts the CG; re-run the CAD check or this scale test. Dynamic torque at the motor, 1500 deg/s^2: at most 0.056 N*m (spec 6.3).

Fallback if the scale reads more than 1.36 N: **ballast above the axis** (spec 6.3 names about 300 g; for a full cancel at 55 mm above the axis it is 462 g x 34.5 mm / 55 mm = 290 g). Less is enough: each 100 g at
55 mm above the axis removes 0.054 N*m [m x g x r, position 55 mm is my example]. Use lead or steel shot in a printed box bolted to the chassis top rear, away from the ball path and the belt; every 100 g adds to the 608 and 6807ZZ radial load and to the tilt inertia, so
re-check the bench validation tilt tests after fitting it. Other fallbacks: a rubber-band counterbalance on the left side. Power loss: with the motor off the head drops to the cradle stop at -15 deg, as designed.

## 7. Hopper, feed elbow and the ball-feed bench check

Fit through the 6807ZZ: the balls drop from the hopper neck (26.4 mm bore) into the elbow, slide down a 38 deg slope to the right cheek, enter the spigot (tip at B (0, -15, 202) mm; OD 27.6, ID 24.0), pass through the hollow trunnion tube
(35 mm OD, 28.4 mm bore, inside the 6807ZZ inner race) into the beam, turn 90 deg at the axis (point C) and roll down the chute to the rest point F in front of the piston (H (-65, 0, -52.5)). The **76 mm level run along the tilt
axis** (4 balls long) is the one leg without slope: the balls above push the balls in it (CAD: pushed by 2 balls on the slope, margin 3.1 x at a rolling resistance of 0.1 [est, bench test]).

CAD results: ball path clear at tilt -15, 0, 35, 70 deg (360 positions swept, 0 mm3 overlap); chute downhill angles -15 deg: 24 deg, 0 deg: 39 deg, 35 deg: 74 deg, 70 deg: 71 deg.

Bench check (weapon rail dead, flywheels and servo unpowered, steppers off, key out; eye protection for the person who runs it):
1. Put the head at tilt 0 by hand. Drop 5 balls into the hopper neck one at a time; they must run through the elbow, spigot, trunnion and chute to the rest point and stop against the piston face. Time them: expect under 3 s.
2. Fill the hopper with 40 balls (the CAD capacity). They must keep feeding to the rest point without a tap. Rotate the pan by hand through +-90 deg while the head is level: no ball escapes, no jam.
3. Repeat at tilt -15, 35, 70 deg (hold the head by hand or with a prop). At -15 deg the balls must still roll to the wheel nip; at 70 deg they rest against the piston face when nose-up.
4. Pass: the 5th ball reaches the rest point at tilt 0 within 3 s; 40 balls feed without a manual tap at every tilt; no ball leaves the path. Fail: a stall in the level run (the 76 mm leg) at any tilt: sand or ream the trunnion tube
   bore and chute with a round file, wipe talc or a dry PTFE spray on the ball path, check the tube/6807ZZ alignment (a ball catching at the elbow joint or at the tube-to-beam seam is the usual cause).
5. If the level run still stalls, the margin of 3.1 x was optimistic [est]: raise the hopper (the neck sits at B z = 233 mm; the rim is at 392 mm), or add a mild vibration (a small 3 V pager motor on the elbow) before touching the geometry. Report the measured rolling resistance.
6. Fit `hopper_lid` only when the check passed; keep the lid to jam clearing.

Do not load more than the hopper capacity; do not leave balls in the path with the flywheels spinning (the flywheels pinch the ball at the rest point).

## 8. Before first power: safety checklist

Work through this list in order. Tick every line on paper and keep it with the log.
1. Eye protection on for everyone in the room. The barrier (plywood or foam board behind the targets) is not required for the dry check, but is in place before any ball is loaded (bench_validation section 0).
2. Weapon rail dead: key out, E-stop **pressed**, fuse F1 **out of its holder**. The flywheel motor leads disconnected from the MOSFET drains, servo disconnected. No balls anywhere in the hopper or the path.
3. Visual: no stray strands, no bare lugs touching, all four motor connectors seated, polarity of the XT60 correct, wires clear of the belts, pulleys, hall magnets and the hard stop.
4. Meter, power off, fuse out: resistance between 12 V bus and GND not a short (above 1 kohm); the same for the 5 V rail. Continuity: the key and E-stop in series open when either is open (key off or E-stop pressed).
5. Verify the XT60 inlet polarity with the DMM at the brick (or the pack) before you connect it: + on the XT60 +, GND on -.
6. Fit F1 = 10 A ATC blade (the supplied 15 to 40 A blades are too big). Do not use a bench supply set above 16.8 V (TMC2209 limit 29 V, the Jetson 5-20 V module).
7. Power on with the key out and the E-stop pressed. Expected: Jetson boots, the relay does **not** click, the rail at the LM2596 output reads 0 V.
8. Do not hot-plug the stepper motors (a live unplug can kill a TMC2209). Disconnect the supply before changing motor leads.
9. Release the E-stop, turn the key on: the relay must click and the LM2596 output must read 5.0 V (set the LM2596 trimmer to 5.0 V with no load first). Press the E-stop: rail falls to 0 V within 20 ms. Do this
   with the flywheel motors still disconnected.
10. Check EN pull-up: with the Pico unplugged the TMC2209 EN pin must read about 3.3 V (high = driver off, motor free). A floating EN is a fail (the BTT V1.3 has no internal pull-up; the external 10k is in the harness).
11. Only then follow `docs/cuas-lite/bench_validation.md`, in order, starting at section 0 (safety rules) and section 1.1 to 1.4. Do not connect the flywheel motors until section 1.4 has passed.
12. Never aim at a person or a pet, even with the flywheels off. First live test of anything is dry: flywheels at 0, pusher cycling with no balls. Balls go in only after the dry test passed.

## 9. After the build

Calibration and tests: `docs/cuas-lite/bench_validation.md` (belt tension and backlash 2.1, pointing 2.2, launcher section 3, camera/boresight 4). The values you measure feed back into the sim as listed in its section 6.
Open items: the lazy-susan bolt circle (`LS_BOLT_R`), the belt stock length, the real insert and fastener pack contents, ball rolling resistance, hall magnet pole direction.
