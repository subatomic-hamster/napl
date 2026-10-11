# ASV: autonomous surface vessel with COLREG-compliant avoidance

A BlueBoat-class vessel (1.2 m twin-hull catamaran, two Blue Robotics T200 thrusters on bidirectional ESCs, Pixhawk 6C + Raspberry Pi 5, u-blox M8N GNSS + compass,
RPLIDAR S2 lidar, SiK telemetry radio) built from catalogue parts through `napl.design`, put in a harbour with waves, a shoreline, a pier, buoys and scripted traffic.
It navigates from GNSS + IMU + compass only, sees other vessels through its lidar and (simulated) AIS only, and avoids them under COLREG Rules 13-17.

```
tools/remote.sh run '.venv/bin/napl program examples/asv/asv-head-on.scenario.toml --runs-dir build/runs'     # one scenario (the other five: see below)
tools/remote.sh bg mc '.venv/bin/python examples/asv/mc.py --seeds 40 --seed-base 20000 --workers 5 --out build/asv-mc.json'   # Monte Carlo, 40 geometries per scenario
tools/remote.sh bg ncmc '.venv/bin/python examples/asv/mc.py --noncomplying --kinds crossing_stand_on --seeds 40 --seed-base 20000 --workers 5'   # Rule 17(a)(ii)
tools/remote.sh run '.venv/bin/python -m pytest tests/python/test_asv.py -q'                                  # the tests
```

Everything runs on the build host (hfbox); a full scenario takes 1.5 - 3 minutes of wall time for 2 minutes of simulated time (up to 4 vessels' worth of physics, 125 Hz IMU, a
10 Hz 1600-ray lidar scan and a 2 Hz planner), the three-traffic one longer. The first build of the ASV assembly routes its cosmetic wire tubes (about 4 minutes of A*
on one core); the routed paths are cached on disk (`$TMPDIR/napl-wire-cache`, `NAPL_WIRE_CACHE=off` disables it), so later builds and every Monte Carlo worker take under a second.
`mc.py --fast` runs the same planner / tracker / judge in the synthetic plant of `fastsim.py` (seconds per run, TUNING ONLY: nothing in the results below comes from it).

## What is here

| file | role |
|---|---|
| `assemblies/asv.toml`, `assemblies/asv-traffic.toml` | the ASV and the traffic vessel (same hull, thrusters, ESCs, batteries, GNSS; no autopilot or lidar) as `napl.design` assemblies, DRC clean |
| `scene.py`, `encounters.py` | the harbour (shoreline, pier, breakwater, dock, four channel buoys, floats), the JONSWAP sea, the scripted encounters as plain data (`Spec`) |
| `estimator.py` | Mahony MARG attitude filter + 6-state Kalman filter (position, velocity, accelerometer bias) with GNSS latency compensation |
| `control.py` | LOS path following along a waypoint route, heading PD with a rate-limited reference, speed PI with drag feed-forward, differential-thrust mixer |
| `perception.py`, `ais.py` | lidar scan -> clusters -> global-nearest-neighbour constant-velocity tracker, fused with AIS reports (Class A rate, 1 s latency, 2 % loss) |
| `colreg.py`, `planner.py` | Rule 13-17 situation classification; sampled-manoeuvre planner with COLREG costs on the tracks and the static lidar points |
| `traffic.py` | the scripted traffic vessels (keep course, give way, turn to starboard), same autopilot as the ASV, steering on the true state |
| `judge.py`, `run.py` | judgement from TRUE tracks (collision, closest approach, passing side, manoeuvre per Rule); the program that owns the Sim and writes the artefacts |
| `asv-*.scenario.toml` | `napl program` scenarios: `patrol`, `head-on`, `crossing-give-way`, `crossing-stand-on`, `overtaking`, `multi-traffic` (three vessels, mixed roles) |
| `mc.py`, `fastsim.py`, `render_frames.py` | Monte Carlo over seeded geometries and sea states (Wilson intervals); the fast synthetic-plant harness the planner was tuned in; PNG frames of a finished run (GPU) |

The planner and every estimator read only sensor devices (`/devices/asv_imu`, `asv_mag`, `asv_gnss`, `asv_lidar`) and the AIS channel; the true state is read only to
write the trace, the telemetry `pose`/`truth` fields and the judgement, and to steer the *traffic* vessels (they are the scenario's script, not the system under test).

## Parts list (the real vessel this models)

| part | qty | price (USD) | source |
|---|---|---|---|
| Blue Robotics T200 thruster (3.71 kgf forward / 2.92 kgf reverse at 12 V) | 2 | 230.00 each (+27.60 tariff surcharge for US customers) | https://bluerobotics.com/store/thrusters/t100-t200-thrusters/t200-thruster-r2-rp/ |
| Blue Robotics Basic ESC (bidirectional, 30 A, BLHeli_S) | 2 | 40.00 each (+4.80 tariff) | https://bluerobotics.com/store/thrusters/speed-controllers/besc30-r3/ |
| Holybro Pixhawk 6C autopilot (PX4), plastic case, no power module | 1 | 199.00 | https://holybro.com/products/pixhawk-6c |
| Holybro M8N GPS (u-blox NEO-M8N + IST8310 compass) | 1 | 51.59 (listed as sold out and discontinued, replaced by the M9N / M10) | https://holybro.com/products/m8n-gps |
| Holybro SiK telemetry radio V3, 100 mW 915 / 433 MHz | 1 | 58.99 | https://holybro.com/products/sik-telemetry-radio-v3 |
| Slamtec RPLIDAR S2 (30 m, 32 kSa/s, 10 Hz, IP65) | 1 | about 355 (retailer listings, not a Slamtec price; `fidelity = "estimated"` in the part file) | https://www.slamtec.com/en/s2/spec |
| Raspberry Pi 5 | 1 | not re-checked here: the Skydio README lists the 8 GB at about 200 (https://www.adafruit.com/product/5813); raspberrypi.com lists the 16 GB at 305 | https://www.raspberrypi.com/products/raspberry-pi-5/ |
| Gens ace 5300 mAh 3S 60C LiPo, EC5 | 2 | about 49 each from the listing's URL slug (the page returned HTTP 403 when checked: unverified) | https://rcdrone.top/products/src-gea533s60e5gt-gens-ace-5300mah-3s-60c-11-1v-hardcase-g-tech-lipo-battery-49-with-ec5-plug |
| Pololu D24V50F5 5 V 5 A step-down regulator | 1 | 32.95 | https://www.pololu.com/product/2851 |
| twin-hull catamaran hull, 1.2 m, with two skegs and sensor masts | 1 | not priced (a BlueBoat hull is a Blue Robotics product; the model is "sized after" it, not a product) | https://bluerobotics.com/store/boat/blueboat/blueboat/ |
| **Total of the priced parts** | | **about 1,535** (2 x 230 + 2 x 40 + 199 + 51.59 + 58.99 + 355 + 200 for the Pi + 2 x 49 + 32.95; without tariff surcharges) | retailer listings found during the build (Oct 2026), not quotes; the full BlueBoat kit is a different, larger price |

The hull is the least certain part of the model (strip-theory coefficients, `fidelity = "estimated"`, see `parts/marine/generic-catamaran-hull-1200-skeg.toml`); the
thrust, ESC, battery, GNSS, IMU and lidar numbers come from datasheets and are sourced in the part files (`tools/check_parts.py`: 691 parts, 0 problems).
The simulated mass is checked to be BlueBoat-class (8-20 kg) in the tests; the exact figure is in the `napl design` build report (BlueBoat: 14.5 kg without batteries or payload, 16.9 kg with two).

## Wiring (assemblies/asv.toml, DRC clean: 0 errors)

```
bat1 3S --EC5--> Basic ESC 1 --3 phase--> T200 port          bat2 3S --EC5--> Basic ESC 2 --3 phase--> T200 starboard
bat1 --tap--> Pololu D24V50F5 (5 V) --+--> Raspberry Pi 5 (USB-C power)
                                      +--> Pixhawk 6C POWER1
                                      +--> RPLIDAR S2 power
Pixhawk 6C  PWM OUT 1 / 2 (1100-1900 us, 1500 = stop) --> ESC 1 / ESC 2 signal
Pixhawk 6C  GPS1 --> M8N GPS + IST8310 compass (mast, 0.30 m)          Pixhawk 6C TELEM1 --> SiK radio --> ground station
Raspberry Pi 5  USB --> RPLIDAR S2 UART/USB adapter (mast, 0.45 m)     Raspberry Pi 5  UART --> Pixhawk 6C TELEM2 (MAVLink companion link)
```

A bidirectional Basic ESC reads 1500 us as zero thrust, 1100 us as full reverse and 1900 us as full forward; in the model the ESC device's `throttle` is signed in
[-1, 1] and `bidirectional = true` lets the bridge reverse (`docs/models.md`). In this showcase the autonomy runs on the host computer standing in for the Pi 5 + Pixhawk pair
(it writes the throttle of the two ESC devices directly); no PX4 firmware flies the boat.

## The directional instability, and the fix

The vessel device has diagonal damping plus the inviscid Munk moment `N = (A11 - A22) u v`. A bare twin hull in that model is directionally unstable above 0.4 m/s
(sway-yaw stability needs `D_v D_r - (Y_r - (m + A11) U) N_v > 0`: 30 against 192 U^2): a drift to one side turns the bow away from the velocity vector and nothing aft resists it.
Real hulls have hull lift and skegs that do. The model has no hull lift, so the lift is put where practice puts it: a **skeg under the stern of each hull**, a new `Vessel.fins`
table (quasi-steady lift `cl_alpha |u| w` plus cross-flow drag `cd |w| w`, with immersion; see `docs/models.md`). Two 0.3 x 0.1 m skegs 0.45 m aft give a yaw stiffness of
13.7 U N m s/m against the Munk 9.3 U: stable at every speed (net +4.4 U). The fin force law is tested in `tests/cpp/test_vessel.cpp`
(`vessel.skeg_force_and_yaw_moment_oppose_sideslip`); the stability margin itself is the analytic argument above, not a free-running experiment, and the skeg's lift slope
(Helmbold, effective aspect ratio 0.67) and size are estimates. Reverse thrust is the forward tables mirrored times 0.787 (2.92 / 3.71 kgf at 12 V), tested in
`vessel.bidirectional_esc_reverse_thrust_and_rev_ratio`.

## Autonomy

- **State estimation** (`estimator.py`): TRIAD initial attitude from the accelerometer + magnetometer (waits for the first non-zero samples), then a Mahony MARG filter
  (accelerometer corrects roll / pitch, magnetometer yaw, gyro bias integrated). A 6-state Kalman filter (position, velocity, accelerometer bias in the horizontal
  plane) integrates the world-frame acceleration at 125 Hz and is corrected by every GNSS fix (position moved forward by the receiver latency along the fix's velocity). No ground truth.
- **Guidance and control** (`control.py`): LOS along the legs of a waypoint route with a look-ahead of `max(4 m, 1.7 |xte|)` (approach angle under 30 degrees); a heading PD
  (`kp 1.5`, `kd 0.9`, at most 0.7 differential throttle) behind a 0.7 rad/s rate-limited reference; a speed PI with a drag feed-forward; a mixer that keeps the differential
  and shaves the common throttle. Gains were found in this simulator (Hs 0.8 m headings std 2.3 degrees; an earlier `kp 0.55` gave 10 degrees and loops).
- **Perception** (`perception.py`, `ais.py`): the 1600-ray scan (RPLIDAR S2, every 2nd of its 3200 samples per revolution) is segmented into clusters; clusters longer than 6 m are
  shoreline and stay a static point set (the planner's obstacles); the rest feed a GNN constant-velocity Kalman tracker (gate 4 m, confirm after 4 hits, drop after 4 s).
  AIS reports (Class A: 10 s, or 3.3 s while turning; 1 s latency; 2 % loss; fed by the *traffic vessel's own* noisy GNSS) are fused into the same tracks; an AIS contact
  the lidar has not seen yet sits in AIS's frame (our GNSS error plus the other's), so it is matched with a wide gate and snapped to the lidar position at the first return.
  A contact is "moving" above 0.35 m/s. The planner never sees a ground-truth position of a vessel or an obstacle. Returns of small fixed objects within 8 m are remembered for 1 s (`run.py`, `STATIC_MEM_*`; see the honesty notes).
- **COLREG** (`colreg.py`, `planner.py`): contacts with risk of collision (closest point of approach below 20 m within 60 s, at most 250 m) are classified once their track is
  4 s old and the situation keeps that class until finally past and clear (Rule 13(d)). Roles: head-on, crossing with the contact on the starboard side, overtaking: **give way**
  (early, substantial action to starboard, no turn to port; head-on passes port-to-port; a crossing avoids passing ahead of the other vessel); crossing with the contact on the port
  side and being overtaken: **stand on** (keep course and speed, costs 40 per unit of deviation) until the contact's closest point stays under 7 m with 13 s to go (Rule 17(a)(ii)),
  and then act without turning to port for a vessel on the port side (17(c)); within 6.5 s the stand-on vessel must act (17(b), `emergency` in the telemetry).
  The planner rolls out, over 75 s with a kinematic boat model, 4 speed fractions x two families of manoeuvres: **15 course offsets** (7-80 degrees each way, relative to the leg direction) held
  for 8 / 16 / 28 s, and **9 parallel lines** (4 m to port ... 24 m to starboard of the leg) followed for 20 / 40 / 75 s (a pass of a slow vessel takes longer than any heading hold, and a heading held long enough
  to clear it ends tens of metres off the route; a parallel line costs exactly its offset), each followed by the return to the leg. It scores progress along the leg, the widest excursion
  from the leg, clearance to static points (6 m wanted), the closest point of approach to each track (12 m wanted, 9 m when overtaking; a closest point more than 35 s away counts less,
  down to a quarter at 65 s: the constant-velocity prediction of a vessel that will still act), the Rule costs and a commitment margin (kept only while a fixed object is near or a
  contact is still to be passed). It re-plans at 2 Hz. Details that came out of the Monte Carlo and matter for the Rules: the situation is judged on the mission course while no manoeuvre is
  in force (a boat still settling on its heading would otherwise hide a collision course); a give-way duty is never shed by a re-read of the class (a head-on vessel that has turned to starboard is on our port bow, i.e.
  "crossing", but the situation stays head-on until we are past); the stand-on vessel acts under 17(a)(ii) only once the trigger (the other vessel's closest point under 7 m with 13 s to go) has held for 1 s, and then
  for 6 s.
  The thresholds (the 15 degree head-on cone, 12 m, 7 m / 13 s, ...) are tuning choices; the Rules give none. They are listed in the file headers and `judge.py`.

COLREG text the code follows (International Regulations for Preventing Collisions at Sea 1972; as reproduced at https://ialacolreg.com/en/colreg/rule-13 ... rule-17, which cite the U.S. Coast Guard
International Navigation Rules): Rule 13(a) "any vessel overtaking any other shall keep out of the way of the vessel being overtaken", 13(b) overtaking = approaching from more than 22.5 degrees abaft the
other vessel's beam, 13(d) a later change of bearing does not make her a crossing vessel; the duty lasts until she is "finally past and clear". Rule 14(a) "each shall alter her course to starboard so that
each shall pass on the port side of the other". Rule 15 "the vessel which has the other on her own starboard side shall keep out of the way and shall, if the circumstances of the case admit, avoid
crossing ahead of the other vessel". Rule 16 "take early and substantial action to keep well clear". Rule 17(a)(i) "the other shall keep her course and speed"; (a)(ii) she "may take action to avoid collision by her
manoeuvre alone" when it becomes apparent that the give-way vessel is not taking appropriate action; (b) "such action as will best aid to avoid collision"; (c) "if the circumstances of the case admit,
not alter course to port for a vessel on her own port side".

## Scenarios

`napl program examples/asv/asv-<name>.scenario.toml --runs-dir <dir>`; every scenario has the same parameters (`seed`, `hs`, `jitter`, `ais`, `traffic_complies`, `visual`)
and the same telemetry channels. The ASV starts at the origin heading East; the mission leg is 160 m at 1.5 m/s. The traffic starts on a collision course
(its constant-velocity closest point with the ASV is below 3.5 m if nobody acts).

| scenario | situation | judged by the monitors |
|---|---|---|
| `asv-patrol` | 100 x 18 m rectangle, two floats on the legs, waves from the bow quarter | mission complete, no collision, at least 2 m from every fixed object |
| `asv-head-on` | Rule 14: traffic on the reciprocal course turns to starboard at 45 m | closest approach >= 5 m, port-to-port, no turn to port above 10 degrees, mission complete |
| `asv-crossing-give-way` | Rule 15/16: traffic from the starboard side, keeps course and speed | >= 5 m, passes astern of it, no turn to port, mission complete |
| `asv-crossing-stand-on` | Rule 15/17: traffic from the port side, turns 45 degrees to starboard at 50 m | >= 5 m, heading within 15 degrees of the route and at least 70 % of the speed it made before (5 s smoothed) while it gives way |
| `asv-overtaking` | Rule 13: slower traffic (0.6-0.9 m/s) on the route, keeps course | >= 5 m, ASV ahead of it at the end, mission complete |
| `asv-multi-traffic` | three vessels one after the other, action windows overlapping: crossing from port (gives way: the ASV stands on), crossing from starboard (keeps course: the ASV gives way and passes astern), reciprocal course (turns to starboard: the ASV turns to starboard, port-to-port) | each vessel by its own Rule (`judge.verdict_all`), >= 5 m to every one, mission complete |

Parameters: `seed`, `hs`, `jitter`, `ais`, `traffic_complies`, `visual`, and `t_stop` (end the run early, used by the tests; 0 = the whole scenario).

### How the Rules are judged (`judge.py`, on the TRUE tracks, never the ASV's estimates)

A run succeeds when the mission completes (true position within 8 m of the end waypoint), nothing is hit (centre distance to a vessel under 1.2 m, or to a fixed object under 0.75 m,
counts as a collision) and the Rule judgement holds. The Rules give no numbers, so these are mine, and each is argued from the Rule text:
- **closest approach >= 5 m** (four hull lengths of a 1.2 m craft): Rule 8(c) "a large alteration ... passing at a safe distance"; 8(d) "the effectiveness of the action shall be carefully checked until the other vessel is finally past and clear".
- **head-on (Rule 14(a))**: "each shall alter her course to starboard so that each shall pass on the port side of the other": at the closest point the other vessel is on our port side, and (Rule 16/8(b): "large enough to be readily apparent") we never head more than 10 degrees to port of the route course while it is within 60 m before the closest point.
- **crossing, ASV gives way (Rule 15/16)**: "avoid crossing ahead of the other vessel": at the closest point we are in the stern half of the other vessel's track; the same 10 degree rule (a turn to port for a vessel on the starboard bow is what the Rules discourage).
- **crossing, ASV stands on (Rule 17(a)(i))**: "keep her course and speed": inside 60 m before the closest point we stay within 15 degrees of the route and above 70 % of the speed made before (5 s smoothed, because a 1.2 m hull in 0.8 m waves surges instantaneously to near zero and back). With a vessel that does NOT give way, 17(a)(ii) / (b) allow and then require action, so the test is only "no collision" (see the non-complying results).
- **overtaking (Rule 13(a))**: "shall keep out of the way": the ASV is ahead of the other vessel at the end.
- **several vessels (Rule 2(b), 8)**: the stand-on vessel is judged only on the samples where the ASV is not itself giving way to another vessel inside 120 m (Rule 16 gives way "early"): a vessel that must give way to one vessel cannot also keep its course for another.
A criterion that was wrong would be argued here rather than relaxed: none has been changed to make a number better since the first Monte Carlo, except that the stand-on speed test now uses the 5 s smoothed speed against what the boat made before the encounter (the instantaneous 70 % of the nominal speed failed runs that took no action but surged).

All scenarios also monitor the estimate (position error under 8 m, heading error under 15 degrees). The distances are judged on the TRUE tracks (columns of `trace.csv`), the replay log
(`replay.naplog`) reproduces the final state hash (`napl-replay`), and a repeated run ends on the same hash.

## Results (Monte Carlo, `mc.py`: 40 seeded runs per scenario, seeds 50000-50039, jitter 1, Hs uniform 0.2-0.8 m, AIS on, wave direction within +-60 degrees of the bow)

Each run draws the crossing angle, speeds, lateral offsets, meeting point and sea direction from its seed. Success = mission complete (true position within 8 m of the end waypoint), no
collision, and the Rule judgement of `judge.py` (closest approach >= 5 m and the manoeuvre of the Rule). 95 % Wilson intervals in brackets. Command, per scenario or all at once:
`python examples/asv/mc.py --seeds 40 --seed-base 50000 --workers 8 --out runs/asv-mc.json` (the kernel, not `--fast`).

| scenario | success | Rule-compliant | collisions | closest approach min / p10 / median (m) | truth cross-track error rms, median / p90 (m) |
|---|---|---|---|---|---|
| patrol | 100 % (40/40; 91-100) | 100 % | 0 | 2.7 / 3.2 / 3.9 (to a float on the leg, centre to centre, 6 m wanted) | 3.1 / 3.8 |
| head-on | 100 % (40/40; 91-100) | 100 % | 0 | 17.2 / 19.3 / 22.5 | 8.1 / 11.4 |
| crossing, give way | 98 % (39/40; 87-100) | 98 % | 0 | 10.4 / 12.0 / 13.4 | 8.5 / 10.1 |
| crossing, stand on | 100 % (40/40; 91-100) | 100 % | 0 | 8.5 / 10.2 / 13.5 | 1.1 / 3.3 |
| overtaking | 100 % (40/40; 91-100) | 100 % | 0 | 8.7 / 8.9 / 9.6 | 7.7 / 9.0 |
| three vessels, mixed roles (`multi_traffic`) | 100 % (40/40; 91-100) | 100 % | 0 | 5.8 / 7.4 / 10.8 (smallest over the three) | 10.8 / 16.1 |
| stand-on, give-way vessel does NOT comply (`--noncomplying`, 40 seeds) | 92 % (37/40; 80-97) | 92 % | 0 | 4.3 / 5.1 / 6.1 | 4.7 / 7.3 |

The last row is the Rule 17(a)(ii) / (b) test: the traffic keeps its course, and the criterion asked for is "no collision" (100 %, 40/40, Wilson 91-100 %); the judge's own 5 m closest-approach test
fails 3 of the 40 (4.3, 4.4 and 5.0 m). Widest excursion from the route (p90 of the per-run maximum): patrol 8.5 m, head-on 18 m, give way 17 m, stand-on 5 m, overtaking 14 m, three vessels 26 m, non-complying stand-on 18 m.

How these numbers came about (read this before trusting them):
- **They are not blind.** Three seed blocks were flown before this one; each used up its block. 20000-20039 gave 40/40 in head-on, give-way, stand-on and overtaking, 40/40 in the three-vessel scenario, 39/40 in patrol with
  ONE collision (seed 20011: the hull centre passed 0.69 m from a float, under the 0.75 m collision line; the lidar plane misses a 1 m drum on many revolutions of a pitching hull, and the planner kept only the last scan). I added a short memory of
  lidar returns of fixed objects, which was a change to the system under test after seeing its test result. The first version (3 s, every return) regressed 30000-30039 to stand-on 17/40 and patrol 25/40 (it smeared quay walls and held the planner on stale hazards, and a
  float that was always in view kept a manoeuvre committed); the second (1 s, small objects only) was 35/40 on stand-on on 40000-40039, where the memory-off planner gave 38/40 (and patrol 38/40 with a float collision, 39/40 and none with the memory); the third (only
  within 8 m, which is where the float collisions happen) is the final one and was flown here, on 50000-50039, once. Nothing was retuned after seeing these numbers. The 8 m range itself was chosen on the failures of the 40000 block, so only 50000+ is clean for it.
- With 40 runs, 40/40 only says the true rate is above 91 % (95 % Wilson); a rate of 95 % is not excluded for any row, and the run-to-run spread between seed blocks is of that size (stand-on: 40/40, 17/40 regressed, 35/40, 38/40, 40/40 across blocks and planner versions).
- Patrol is the scenario with a hard 100 % target and a fixed-object collision criterion. Without the memory it had a float collision in each of the two blocks it was flown on (20000 and 40000, one run in 40 each); with the final planner it had none in 40 (50000). One patrol run on 40000 (seed 40010, Hs 0.22 m) ends 8.5 m short of the last waypoint with and without the memory, so the 100 % on 50000 should not be read as a rate of 100 %.

Reading it honestly:
- No run collided in 280. The single failure is a Rule failure with a large clearance (crossing give way, seed 50034: 12.3 m, a turn to port above 10 degrees or passing ahead, not traced further).
- Cross-track error is large where the planner deliberately leaves the route (head-on 11 m p90, three vessels 16 m): it is an avoidance offset, not tracking error. The GNSS bias alone is about 3.7 m.
- **The planner was tuned in two stages.** First on seeds 5000-5199 in the fast simulation (`fastsim.py`: the real planner, tracker and judge in a kinematic plant; tuning only, never a result) and in the kernel on the textbook geometry; an
  earlier planner gave 40-57 % on head-on, give-way and stand-on and 37 % on overtaking before the faults it showed were fixed (class re-read in the first 12 s, no pricing of the predicted closest point by a stand-on vessel before Rule 17(a)(ii),
  no return to port of the leg while a give-way contact is not yet passed, a cost for slowing down in overtaking, a parallel-line family of manoeuvres, a commit only with something to avoid, a give-way latch, a persistence of the stand-on trigger). Second on the
  kernel Monte Carlo blocks listed above. Every block of seeds is used up once it has shaped a decision; 50000-50039 shaped none.
- Textbook geometries (seed 1, jitter 0, Hs 0.4 m): the scenarios pass their monitors and replay to the same state hash (`tests/python/test_asv.py`: the head-on one in full, the others to 45 s so the suite stays fast; `napl program` runs them whole).
- Remaining weaknesses: overtaking passes at 8.7-9.6 m (12 m wanted) with 11 m p90 excursions; the non-complying stand-on test passes within 5 m in 3 of 40 (never within 4 m); patrol passes floats at 2.7-3.9 m (6 m wanted, 0.75 m is a collision) and one run of 40 on another block
  (seed 40010) did not finish in time; the lidar, tracker and planner are tuned against one harbour, one speed and one hull.

## Telemetry contract (`telemetry.jsonl`, one JSON object per line, every 200 ms of sim time)

Georeferenced: `lat_lon` = `[lat, lon]` degrees (WGS84, flat-earth map around the scene origin 47.3769 N, 8.5417 E, the GNSS device's frame; East/North metres `x_m`, `y_m` from that origin).
Headings and courses are degrees clockwise from North; speeds m/s; angles in degrees.

```
t                                    sim time, s
pose   {lat_lon, x_m, y_m, heading_deg, roll_deg, pitch_deg, sog_ms}          TRUTH of the ASV (for the HUD / replay; the autonomy never reads it)
est    {lat_lon, x_m, y_m, heading_deg, sog_ms, cog_deg, yaw_rate_dps, surge_ms, sway_ms}   the navigation estimate
sp     {heading_deg, speed_ms, thr_port, thr_stbd, course_offset_deg, speed_frac}  setpoints: commanded heading / speed, the two ESC throttles (-1..1), the manoeuvre in force
mission {leg, xte_m, dist_to_wp_m, done}                                         cross-track error (estimate, + left of the leg), distance to the next waypoint
tracks [ {id, mmsi|null, src (lidar|ais|fused|coast), moving, lat_lon, x_m, y_m, sog_ms, cog_deg|null,
          range_m, bearing_deg (+ starboard), cpa_m, tcpa_s|null, rule, role (give_way|stand_on|none), risk, ext_m} ]    one per confirmed contact, per-contact CPA / TCPA
traj   [[lat, lon], ...]             the chosen trajectory, every 2 s over the 60 s horizon (empty before the mission starts)
rule   "none|head_on|overtaking|overtaken|crossing_give_way|crossing_stand_on"   the situation of the most critical contact
emergency  bool                      Rule 17(b): the stand-on vessel is acting
scan   {n_static_pts}                static lidar points seen (shoreline, piers, floats)
truth  {traffic: [{name, lat_lon, sog_ms, cog_deg|null}]}                       the traffic vessels' true state, for the HUD
```

`result.json` (the run dir) carries the scalar results (`cpa_m`, `side`, `xte_rms_m`, `colreg_ok`, ...); `trace.csv` the judged columns. `napl.params` declares
the parameters and channels (`napl program ... --describe`).

## For the video / HUD / C2 map worker

```
tools/remote.sh run --gpu '.venv/bin/python -c "import os,json; os.environ[\"NAPL_PARAMS\"]=json.dumps({\"visual\":True,\"hs\":0.4}); \
   from napl import program; program.run(\"examples/asv/asv-head-on.scenario.toml\", \"build/runs_asv/head_on\")"'
tools/remote.sh run --gpu '.venv/bin/python examples/asv/render_frames.py build/runs_asv/head_on --out build/frames --fps 30 --from 40 --to 75 --cam wide'
```

Seeds for the video (all from the 50000 block: passed, calm sea Hs 0.2-0.4 m, wide clearances; flown with `NAPL_PARAMS` `seed`, `hs`, `jitter 1`, `ais true` and `hs` = `mc.hs_for(seed)`, i.e. the Monte Carlo's own draw):
seed 50025 (Hs 0.31 m) gives head-on (closest approach 26.5 m), crossing give way (18.6 m), crossing stand on (16.2 m) and patrol (2.7 m to a float, a close pass with a small craft; seed 50006 gives 3.4 m); overtaking: seed 50015 (Hs 0.25 m, 11.6 m);
three vessels: seed 50037 (Hs 0.37 m, 15.2 m). The encounter is at about 40-80 s of sim time (the traffic meets the ASV 70-90 m along the 160 m leg at 1.5 m/s, plus the 8 s settling): render `--from 40 --to 80`
(`--from 30 --to 125` for the three vessels, the whole run 8-177 s for patrol). These windows are estimates from the geometry, not read from the run's `t_cpa`; check `telemetry.jsonl` before cutting.

Cameras: `chase`, `side`, `top`, `bow`, `wide`. The headless renderer draws the sea as a mesh driven by the physics `Sea` (`sea_surface_mesh`, the same wave model the hull floats on), shaded blue-green with
a Fresnel sky reflection, a sun specular and distance haze (`water_shade` in `geom.slang`). The solver marks the plane named `water` in a model with a Sea as `GeomVisual.water` and refreshes its mesh from the Sea each frame (`mujoco_solver.cpp`). I did not run or touch the interactive viewer (`src/viewer`, owned by CADUI), so whether it animates the water is untested.
The C2 map layers: `pose.lat_lon` (own ship), `tracks[].lat_lon` (contacts, coloured by `rule`), `traj` (the chosen path), `mission` (route legs are in `encounters.py`).

## Honesty notes

- **GNSS bias dominates the position error.** The M8N model (2.5 m CEP, slowly varying, 600 s correlation) gives an estimation error of about 3.7 m RMS; it is mostly a *bias*, so the
  filter cannot remove it and the truth cross-track error of a perfect follower is about that. The acceptance radius (3 m) and the "mission complete" test (true position within 8 m of the end) are set with it in mind.
- **The lidar is idealised.** The 1600-ray scan is instantaneous (no motion distortion over the 0.1 s revolution), the plane tilts with roll / pitch, there is no sea clutter or spray and no
  rain; the datasheet's 30 m range is for white targets and 10 m for black ones, the model uses one reflectivity. The lidar sees an other vessel only because of a **cabin proxy**: a 0.6 x 0.28 x 0.9 m box on
  each traffic hull stands in for the superstructure a work boat carries at lidar height (the catamaran deck itself is below the plane).
- **AIS on a 1.2 m craft is an idealisation:** Class A transponders are for ships; the rates, the 1 s latency and the 2 % loss are the standard's, the vessel size is not. No slot collisions, no reception range limit.
- **Traffic is scripted and steers on the truth**, with the same autopilot as the ASV; it is the scenario's script, not a second system under test. Its Rule behaviour (turn to starboard, give way) is a script too.
- **Collision is geometric** (the vessels are `conaffinity 0`): centre distance below 1.2 m to a vessel, or the hull centre within 0.75 m of a fixed object's surface. No contact forces, no damage.
- **The wave self-propulsion artefact is fixed (a real modelling error, not a tuning matter).** The vessel device damped surge / sway / heave in the BODY frame: in a pitching
  hull the heave damping force `-d_heave w_body` has a horizontal component, whose mean over a wave period is `1/2 d_heave a^2 omega k` (a net push, not zero), so an unpowered hull
  in waves sailed on its own. Measured on this ASV hull, thrusters off, no current, seed 1, 80 s window after 20 s: mean drift speed **0.55 m/s at Hs 0.3 m, 0.88 m/s at 0.5 m,
  0.87 m/s at 0.8 m (against the waves, toward -x)**; with the new `Vessel.heading_frame = true` (damping and added mass in the yaw-only frame: x along the body x axis projected on the
  horizontal plane, z vertical; `docs/models.md`) it is **0.0001, 0.0020, 0.0016 m/s**. The powered boat's speed in waves was therefore optimistic in the first stage's runs (an inference from the passive measurement: the powered case was not re-measured with the old model); that is gone. Tests: `vessel.passive_hull_in_waves_drifts_only_with_body_frame_heave_damping` (C++, regular waves, body frame vs heading frame) and
  `test_passive_asv_hull_in_waves_drifts_only_with_body_frame_heave_damping` (this hull, both frames). The flag is off by default, so no other model changes: `boat-float` still
  ends on state hash `60e9d2b502df354f` (it does not use the changed path); the two ASV assemblies switch it on, so every ASV scenario hash is new (and so is every other quantity of
  these runs; the ASV hashes were never pinned, only replay-matched).
- **Hull and skegs are strip-theory estimates**, not tank or towing data; the reverse-thrust curve is the forward one mirrored (times 0.787).
- **Thresholds are choices** (12 m wanted CPA, 6 m from fixed objects, 8 s / 7 m for the stand-on vessel to act, the 15 degree head-on cone); the Rules give none. The judge's pass criteria
  (5 m, 10 degrees, 15 degrees and 70 % of the earlier speed) are mine and stated in `judge.py`; a different set changes the success numbers. The speed test was changed after the first
  Monte Carlo: it used 70 % of the nominal 1.5 m/s on the instantaneous speed, which failed stand-on runs that took no action but surged to 0.1-1.0 m/s in Hs 0.6-0.8 m
  waves; it now uses the 5 s smoothed speed against the speed made before the encounter. The thresholds were not moved after the 50000 block was flown.
- **Overtaking** was the weakest behaviour (37 % on the first Monte Carlo; wide detours, runs out of time, an early version waited behind the slower vessel at 0.1 m/s). It is now 40/40 on 50000-50039 and 39/40 on 40000-40039 with the final or near-final planner,
  by a parallel-line family of manoeuvres; it passes at 8.7-9.6 m, not the 12 m the planner asks for.
- **Stand-on against a vessel that does NOT give way:** the ASV keeps its course until the closest point stays under 7 m with 13 s to go (Rule 17(a)(ii)), then acts. 0 collisions in 40 on 50000-50039 (final planner), and also 0 in 40 on 20000-20039 (planner before the memory; 34/40 passed the 5 m test) and 0 in 40 on 40000-40039 (1 s memory over any range; 38/40 passed the 5 m test): 0 in 120 over three blocks.
  Closest approach in the worst runs is 3.6-4.9 m, i.e. the 5 m criterion is missed by about a metre in 3 to 6 runs of 40.
- **The static-object memory is a patch for a sensor limit** (a one-row lidar at 0.5 m height misses a 1 m drum on many revolutions of a pitching hull). Fixed objects are remembered for 1 s within 8 m, only if small (a wall or a vessel is not), in the world frame from the
  estimated pose (so it inherits the GNSS scatter). It exists because the planner had no other way to see a float that the lidar loses; a 3D or multi-row lidar would make it unnecessary.
- **Nothing here is a real-ship collision-avoidance system:** the Rules are for ships in sight of one another, at ship scales and speeds; this is a 1.5 m/s craft in a 300 m harbour.
