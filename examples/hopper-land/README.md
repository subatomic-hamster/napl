# hopper-land

Copy of `examples/hopper` (1.29 kg EDF VTVL, jet vanes, TF-Luna lidar, RTK GNSS; design `assemblies/hopper-land.toml`)
that takes off and lands in different places and environments. `examples/hopper`, its tests and hashes are untouched.

Flight software: `landflight.py` (state machine), `landest.py` (estimator), `landveh.py` (density / thrust margin),
`landscene.py` (sites and terrain), `landing.py` (parameters, scatter, gust, mission planner), `landdeck.py` (deck-state interface),
`landship.py` (OSV vessel stack), `landhop.py` (harness: owns the Sim, writes trace/replay), `landmc.py` (Monte Carlo),
`render_land.py` (screenshots of a run dir, not committed).

## What changed vs examples/hopper
- Estimator no longer assumes flat ground: vertical channel fuses the lidar only over surveyed pads (or the deck); elsewhere lidar AGL is
  reported (`height_above_ground()`) but not used as altitude. GNSS ignores the device's pre-first-sample ORIGIN output (t < 0.3 s); WGS84 local frame.
- Feed-forward thrust is density-aware: `T_full = 25.3 (rho/1.225)^0.63` N (fit to the kernel EDF data, 0..5500 m; EXTRAPOLATED above),
  density from the barometer-style ISA estimate. Firmware go/no-go margin 1.15 on T/W. Standard-day density only (kernel limit).
- Gust handling: wind-push disturbance observer (`k_dob`), gust fronts / uniform gusts through the wind-field device.
- Landing at a different pad height; planner picks cruise height from the site survey (terrain between pads + `clear_m`).
- Deck interface `landdeck.DeckState` (pos, vel, up, sigmas, ready, window, omega_b, up_b) consumed by `Flight.step(..., deck=)`.

## Altitude (honest)
| altitude m | 0 | 1500 | 2500 | 3200 | 3800 | 4500 | 5500 |
|---|---|---|---|---|---|---|---|
| T/W full throttle | 2.0 | 1.83 | 1.72 | 1.64 | 1.58 | 1.50 | 1.40 |

`landveh.hover_ceiling_m()` = 8253 m, but that extrapolates the fit past 5500 m. At `origin_alt=9000` the firmware refuses
("thrust margin ... not launching"). MC: site 6500 m 85 %, 7500 m 20 % (aborts: off-plan / tilt in thin air).

## Scenarios (Run combo groups by folder)
| name | shows |
|---|---|
| hopperland-mesa-down / -up | pad-to-pad hop between mesa pads of different height, down and reverse |
| hopperland-ridge-summit | summit hop on a ridge, ridge wind + gust front |
| hopperland-ledge-lee | ledge landing in lee turbulence |
| hopperland-rain25 / rain100 | 25 / 100 mm/h rain hop, lidar dropouts and false near returns |
| hopperland-deck-scripted | deck-relative landing on a scripted moving platform (heave/roll/pitch/drift, 10 Hz link) |
| hopperland-osv-underway | landing on the OSV helipad (napl.cuas.ship.ship_scene) while it gets underway in a JONSWAP sea; RelGnss + DeckLink + DeckPredictor |

Golden state hashes (`napl program`, replay match True, all monitors pass): mesa-down `2edc21d6239668ef`, mesa-up `32eab344880c606c`,
ridge-summit `7112e72f2aaa9413`, ledge-lee `f5a07ae0c134ec4d`, rain25 `70261b039c7a881f`, rain100 `5ab944fd12ed0b63`,
deck-scripted `9d6f726a26b0f81b`, osv-underway `e8ba413a6b88c468`.

Each is seeded, judged by monitors on touchdown speed / tilt / distance, and replays to a state hash (`napl program`).
`NAPL_ROUTE_CACHE=<dir>` avoids ~3 min of cold vessel wire routing.

## Monte Carlo (landmc.py; 60 report seeds from 21000, held out from training/sweeps; sweeps use 20 seeds)
Success = speed < 0.5 m/s (relative on a deck), tilt < 5 deg, dist < 0.5 m, no abort, clearance >= 1 m. Wilson 95 % intervals.
| family | TEST n | success | notes |
|---|---|---|---|
| mesa_down | 60 | 100 % [94, 100] | wind x2: 65 %; site 6500 m 85 %, 7500 m 20 % |
| mesa_up | 60 | 93.3 % [84, 97] | wind x2: 65 % |
| rain25 | 60 | 100 % [94, 100] | 150 mm/h and wind x2 also 100 % |
| rain100 | 60 | 100 % [94, 100] | wind x2 100 % |
| deck (scripted) | 60 | 51.7 % [39, 64] | all 28 misses are touchdown speed (p50 0.50 m/s relative); heave 1.0 m 100 %, roll 4 deg 42 %, drift 2 m/s 33 %. An earlier run with the first-contact speed metric gave 90 %; the 0.3 s peak metric is the stricter one (A/B with the deck feed-forward unweighted gave 40 %, so the metric, not the weighting, moved it) |
| ridge | 60 | 75.0 % [63, 84] | k_dob trained on 24 seeds; wind x1.5 45 %, x2 0 % |
| ledge | 60 | 78.3 % [66, 87] | wind x1.5 30 %, x2 5 %; k_dob 0: 50 % |
| osv (underway vessel) | 30 | 20.0 % [9.5, 37] | see below |

Failures in ridge/ledge are aborts (off-plan / tilt) at peak winds of 9-18 m/s; the 2:1 T/W vehicle runs out of vane authority.
`terrain clearance` fails (clr < 1 m) occur at wind ~6 m/s+ over crests.

### OSV envelope (does NOT meet a usable floor)
n=30 test: 20 %; sweeps (n=12): Hs 0.25 m 33 %, 0.5 m 8 %, 1.0 m 33 %, 1.5 m 17 %; vessel stopped 50 %; vessel 3.5 m/s 17 %;
beam seas 17 %; no quiet-window wait 17 %; no deck-rise hold 58 %; deck link 0.3 s late 0 %; wind x2 8 %.
Dominant failure: touchdown speed relative to the deck 0.5-1 m/s (the pad heaves 0.3-1.3 m with up to 1.5 m/s rate; a 2:1 T/W
hopper with ~0.1 s spool cannot track it), then abort:tilt / FAULT. The scenario's default flight is seed 3 (lands, passes the monitors); seeds 1, 2, 5 and 6 of the first six miss a limit (tilt 5.9, speed 0.8, tilt 7.0, abort), seed 4 passes. Seed 3 was picked for a passing golden, not as typical: the rate is the claim.
Not tuned to pass. No claim for Hs above ~0.25 m. Aborting over a ship lands where the hopper is (in the sea): not handled.

## Estimates / simplifications (marked)
- Rain catch area 0.045 m2 and wet area 0.12 m2 are estimates. Scripted-deck motion and link noise are invented, not a measured platform.
- Standard-day density only; `T_full` fit extrapolated above 5500 m.
- Planner uses the terrain survey (map) for cruise height; no DEM-free terrain-clearance guard (adjacent work).
- Rover antenna = hopper frame origin. No vehicle mass/thrust scatter on the ship site.
- Touchdown speed metric = peak approach speed in the 0.3 s after first contact (rejects post-contact chatter).
- Moving-base alignment: on a ship the gyro bias is calibrated with the deck rate subtracted (datalink), otherwise attitude drifts.

## API for other stages
`landdeck.DeckState`, `landflight.Flight.step(..., deck=)`, `landscene.build_model / Site / SITES`, `landship.PrefixSim / ShipDeck / ShipLink`,
`landing.resolve`, `landhop.fly`, `landmc.summarize`. Run slot: each scenario has a sibling `<name>.py` (`landscene.build_for`).

## Commands
`napl program examples/hopper-land/<folder>/<name>.scenario.toml --runs-dir <dir>`;
`python examples/hopper-land/landmc.py --family <mesa_down|mesa_up|ridge|ledge|rain25|rain100|deck|osv> --test 60 --out x.json`;
`python examples/hopper-land/render_land.py <run dir> --out <dir> [--cam follow|close|wide|top|side]` (GPU lane).
