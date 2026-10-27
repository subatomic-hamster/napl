# cuas2 bench validation: from a chronograph and a target sheet to `calib.toml`

This is the step-by-step procedure of spec section 10 ("Bench validation plan"). You build the turret, shoot foam balls at a sheet and a chronograph, write one CSV row per shot, and one
command fits the launcher parameters the simulator and the fire control share (`k_slip`, `sigma_v`, `cd`, `spread`, `bias`, `cl_k`). Nothing here is committed as media; the CSVs and
`calib.toml` stay on your machine. Every key in the simulator has a built-in `[est]` default, so you can run the sim before any of this and replace the guesses one step at a time.

The fitter was tested on synthetic shots with known parameters (`tests/cuas2/test_calib.py`): the numbers in section 5 are the errors it made there, not promises for your hardware.

## 0. Safety (read first, every session)

- Foam balls at up to 36 m/s still hurt an eye. Eye protection for everyone in the room, nobody downrange or beside the sheet while the rail is on, a backstop behind the sheet (a hanging
  blanket or a cardboard box wall), barrel never pointed at a person even unloaded.
- **Keep every shot below 36 m/s while tuning.** The firmware cap is 38 m/s (I13) and cannot be exceeded, but the margin for chronograph error and for a first-time spin-up is yours:
  start each speed step at 60 % of `rpm_max` and walk up (`speed_permille` 600, 700, ... 1000); stop if any reading is above 36.
- Turret ARMed only from the operator's hand: key on, E-stop released, geofence sector set to the sheet and its backstop and nothing else, dead-man held. Hit the E-stop between
  setup changes. The fire-sector, inhibit and person rules apply on the bench exactly as in the field; nothing in this document needs them relaxed.
- Do the ESC bring-up (B1) with the wheels off or the barrel cap on, hands clear, supply on a current-limited bench source the first time.

## 1. What you need

| item                                                     | note                                                                                                                                              |
| -------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------- |
| chronograph                                              | any optical one that reads 10-40 m/s; sits **on the barrel axis, as close to the muzzle as its housing allows** (see 2.2 for the drag correction) |
| target sheet                                             | 1 m x 1 m of paper or cardboard on a frame, vertical, a metre grid or a tape measure, a dot marking the **barrel-axis point**                     |
| tape measure, spirit level, laser line or plumb bob      | range from the muzzle, level barrel, the axis point on the sheet                                                                                  |
| digital caliper and 0.01 g scale                         | step B0                                                                                                                                           |
| logic analyser or a phone slow-motion video (>= 240 fps) | step B-TIME                                                                                                                                       |
| a spreadsheet                                            | one row per shot, saved as CSV                                                                                                                    |
| the repo venv                                            | `python -m napl.cuas2.calib ...` below (from the repo root: `.venv/bin/python -m napl.cuas2.calib ...`)                                           |

## 2. The shot log

### 2.1 Columns

One CSV, a header row, one row per shot, **blank = not measured**. The fitter keys on which columns are filled, so the three campaigns (speed, hit grid, spin sweep) can live in one file.

| column               | unit     | meaning                                                                                                                              |
| -------------------- | -------- | ------------------------------------------------------------------------------------------------------------------------------------ |
| `step`               | text     | `B2`, `B3`, `B4`, `B5`: for you; the fitter ignores it                                                                               |
| `gap_mm`             | mm       | wheel-surface gap used (feeler stock, spec 4.1), needed to fit a gap at a time                                                       |
| `diff_pm`            | permille | commanded differential, **positive = bottom wheel faster (backspin)**                                                                |
| `rpm_top`, `rpm_bot` | rpm      | measured wheel speeds at the feed (STATUS `rpm_top`, `rpm_bot`, the values when the pusher starts)                                   |
| `v_mps`              | m/s      | **muzzle** speed from the chronograph (2.2)                                                                                          |
| `range_m`            | m        | muzzle to sheet, measured along the barrel axis                                                                                      |
| `dx_mm`, `dy_mm`     | mm       | impact point relative to the barrel-axis point on the sheet: `dx` positive to the **left** (looking downrange), `dy` positive **up** |

```csv
step,gap_mm,diff_pm,rpm_top,rpm_bot,v_mps,range_m,dx_mm,dy_mm
B2,19.2,15,16530,16780,26.9,,,
B3,19.2,0,17010,17020,28.3,5.0,12,-285
B3,19.2,0,16990,17030,27.8,10.0,-31,-1240
B4,19.2,30,16530,17540,28.0,10.0,25,-1105
```

### 2.2 Chronograph reading to muzzle speed

A foam ball loses about 6 % of its speed per metre at 30 m/s (`k = rho cd A / 2m` = 0.06 per metre with the defaults). A chronograph whose beam is 0.3 m from the muzzle therefore reads
1.8 % low. Enter `v_mps = v_read * exp(0.06 * distance_m)`. Put the chronograph as close as it goes and always at the same distance; the error then folds into the fitted `k_slip` as a constant,
which is exactly what you want when the fire control uses the same number.

### 2.3 The barrel-axis point on the sheet

Put the cartridge level (spirit level on the barrel; tilt 0 is the head's mechanical zero), measure the height of the muzzle centre from the floor, and mark that height on the sheet
straight in front of the muzzle (laser line or plumb bob). Everything in `dx_mm`, `dy_mm` is measured from that dot. Foam drops 1.0 to 1.3 m at 10 m, so for B3 and B4 hang the sheet with that dot
**near the top**.

## 3. The procedure

Each step says what to shoot, how many, which columns to fill, and what it updates. Work in order: the later steps use the earlier results. Keys not named in the fitter change by hand in
`calib.toml`, with `<key>_src = "bench"` next to the value.

| step        | shoot / measure                                                                                                                                                                                                                 | columns                                                | updates                                                                                                                                                                                                                         |
| ----------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **B0**      | 50 Rival rounds: caliper the diameter on two axes, weigh each                                                                                                                                                                   | none (type the results in)                             | `[ball] d_mm, m_g, d_sigma_mm, m_sigma_g` (mean and standard deviation), then regenerate the CAD `BALL`                                                                                                                         |
| **B-TIME**  | 30 shot cycles with PUSH_HOME, GATE_HOME, FEED_REQ and the beams on a logic analyser (or the pusher on 240 fps video)                                                                                                           | none                                                   | `[launcher.feed] t_push_ms`, `t_commit_ms`, `t_exit_ms`, `t_exit_sigma_ms`; the firmware `gate_load_us`, `gate_bore_us`, `fill_ms`; the minimum shot interval. If one cycle is slower than 0.40 s, R3 is renegotiated (spec Q2) |
| **B1**      | prop-free spin-up of each motor and ESC: DShot arm, eRPM against a scope on one phase wire (pole check), ramp, current from the bench supply, spin-down time with the ESC off                                                   | none                                                   | `[launcher.wheel] pole_pairs`, `J_kgm2` (from the spin-down time), `kv`, `tau_motor_s`, `p_max_w`; firmware `kp`, `ki`, `throttle_max_pm`                                                                                       |
| **B2**      | speed sweep: the three gaps 18.2, 19.2, 20.2 mm x five speeds (`speed_permille` 600, 700, 800, 900, 1000) x 20 shots, `diff_pm` 15 (default). Chronograph every shot. Then at the gap you choose, 40 more shots at three speeds | `gap_mm, diff_pm, rpm_top, rpm_bot, v_mps`             | `k_slip` (per gap, and its slope with rpm), `sigma_v_rel`, `sigma_v_floor_mps`; `k_beam` from the firmware beam speed against the chronograph                                                                                   |
| **B3 + B5** | hit grid at fixed set point 17,000 rpm, `diff_pm` 0: 40 shots at 5 m and 40 at 10 m (the same shots give the drop and the dispersion). Chronograph every shot                                                                   | `gap_mm, diff_pm, rpm_*, v_mps, range_m, dx_mm, dy_mm` | `spread_mrad`, `bias_mrad` (B3) and `ball.cd` (B5). Cd and the vertical bias are told apart by the two ranges, so shoot both                                                                                                    |
| **B4**      | backspin sweep at 10 m: `diff_pm` in -15, 0, 15, 30, 45, **40 shots each** (>= 40 per level; the lift is a fraction of the scatter), chronograph every shot                                                                     | as B3                                                  | `cl_k`; `best_diff_pm` is the level with the smallest vertical scatter, to copy into `diff_pm` if you like it                                                                                                                   |
| **B6**      | dispersion against the rpm window: repeat 20 shots at `rpm_window_pm` 20, 10, 5 and `settle_ms` 50, 100, 200                                                                                                                    | none (compare by eye)                                  | the firmware window defaults                                                                                                                                                                                                    |
| **B7**      | jam and hopper: 24 balls through the head at tilt -15, 0, 35, 70 on a jig, 10 runs each; count hangs and double feeds                                                                                                           | none                                                   | hopper geometry, or the ERM fallback (spec 5.1)                                                                                                                                                                                 |
| **B8**      | fault table rows 3-15, 17, 18, 22-25 (spec 1.3) on the bench, hopper empty of balls first                                                                                                                                       | none                                                   | a pass/fail table; every row must pass before a ball goes into the hopper unattended                                                                                                                                            |

`k_spin` is not identified by heights: the lift depends on `cl_k * k_spin`. The fitter keeps `k_spin` at the value in the file and solves for `cl_k`. If you film the ball spin (slow motion,
a dot on the ball) and get a different `k_spin`, put it in the file first and re-run: `cl_k` then follows. `spin_tau_s` is not fitted either; it matters little over a 0.5 s flight.

## 4. Fitting

From the repo root, with the same `calib.toml` the sim and the app read (`~/.config/cuas2/calib.toml`):

```sh
# one gap at a time (B2 is run at three gaps): rows with that gap_mm, written to its own file
.venv/bin/python -m napl.cuas2.calib fit shots.csv --gap 19.2 --calib ~/.config/cuas2/calib.toml --out /tmp/calib_19p2.toml
.venv/bin/python -m napl.cuas2.calib fit shots.csv --gap 18.2 --calib ~/.config/cuas2/calib.toml --out /tmp/calib_18p2.toml
.venv/bin/python -m napl.cuas2.calib fit shots.csv --gap 20.2 --calib ~/.config/cuas2/calib.toml --out /tmp/calib_20p2.toml

# the operating gap becomes the calibration
.venv/bin/python -m napl.cuas2.calib fit shots.csv --gap 19.2 --calib ~/.config/cuas2/calib.toml --out ~/.config/cuas2/calib.toml
.venv/bin/python -m napl.cuas2.calib show        # provenance of every key: [est] or [bench]
```

It prints one line per fit and writes the updated file (`--out`; without it nothing is written):

```
fit_speed: {'k_slip': 0.7430, 'sigma_v_rel': 0.0265, 'sigma_v_floor_mps': 0.351, 'k_slip_pm_per_krpm': 0.00012}
fit_aim: {'cd': 0.552, 'bias_mrad': (3.18, -1.91), 'spread_mrad': (12.2, 6.5)}
fit_cl: {'cl_k': 0.516, 'best_diff_pm': 15}
wrote /home/you/.config/cuas2/calib.toml
```

What each fit does (so you can tell when to distrust it):

- **`fit_speed` (B2).** `k = v / (v_mean (1 - dip/2))` per shot, where `v_mean` is the mean wheel surface speed and `dip` is the model's own energy dip of the wheels in the nip (the fit divides
  out what the simulator multiplies back in). With three or more distinct speeds it also fits a slope of `k_slip` against rpm (`k_slip_pm_per_krpm`). The speed scatter is split into
  `sigma_v_rel` (proportional to speed) and `sigma_v_floor_mps` (constant) from the variance at each speed, so it needs at least two speeds with 3+ shots each; with one speed it
  keeps the floor and puts the rest into `sigma_v_rel`. The chronograph's own error is inside the floor.
- **`fit_aim` (B3, B5).** Needs rows at `diff_pm` 0 with `range_m`, `dx_mm`, `dy_mm`. Impact height is `drop(range; cd, v) + range * bias_v`, each shot with its own measured `v_mps`; with two
  or more ranges the drag coefficient and the vertical bias separate. Horizontal bias and spread come straight from `dx / range`. The vertical spread has the simulator's own ball
  scatter (diameter, mass, cd) taken out in quadrature, because the simulator adds that itself.
- **`fit_cl` (B4).** Lift is linear in `cl_k` at these spin rates, so the best `cl_k` is a least-squares fit over every shot, each at its own speed, with a free height offset per range.

## 5. How well it fits

Synthetic campaign (`tests/cuas2/test_calib.py::synth_log`: 300 B2 shots, 80 B3 shots, 300 B4 shots, chronograph noise 0.15 m/s), true values against the fit; seeds 5, 11, 12, 13:

| key           | true    | fitted                                                | note                                                                                                     |
| ------------- | ------- | ----------------------------------------------------- | -------------------------------------------------------------------------------------------------------- |
| `k_slip`      | 0.74    | 0.743 / 0.745 / 0.743 / 0.744                         | within 0.5 %, a small residual from the dip being evaluated at the prior `k_slip`                        |
| `sigma_v_rel` | 0.025   | 0.027 / 0.028 / 0.026 / 0.022                         | the split against the floor is the weak part; trust the total at 30 m/s (`hypot(sigma_rel * 30, floor)`) |
| `ball.cd`     | 0.55    | 0.552 / 0.543 / 0.582 / 0.557                         | +-0.03                                                                                                   |
| `bias_mrad`   | (3, -2) | (3.2, -1.9) / (2.4, -1.4) / (3.8, 0.3) / (3.7, -3.1)  | vertical bias is correlated with cd; +-1.5 mrad                                                          |
| `spread_mrad` | (12, 8) | (12.2, 6.5) / (11.3, 8.4) / (11.5, 8.6) / (12.6, 8.8) | +-15 %                                                                                                   |
| `cl_k`        | 0.60    | 0.52 / 0.50 / 0.65 / 0.60                             | +-12 % at 60 shots per level; **plan 40+ per level, 20 is too few**                                      |

More shots narrow every row as `1/sqrt(n)`. Whatever the statistics, the checks that matter are the physical ones: after fitting, shoot 20 more rounds at 10 m with the fitted
`calib.toml` and a fire-control aim (the sim's `solve_lead2` for a static sheet target); the mean should land within the sheet dot's size, and the next fit on the new data should barely
move.

## 6. The `calib.toml` example

A file as the fitter leaves it after a full campaign (the numbers are illustrative; `_src = "bench"` on what a step measured, `"est"` on what is still a guess). Copy it to
`~/.config/cuas2/calib.toml` as a starting point, or let the fitter write yours. A test loads this exact block (`tests/cuas2/test_calib.py::test_doc_calib_example_loads`).

<!-- calib-example -->

```toml
[ball]
d_mm = 22.3
d_mm_src = "bench"
m_g = 1.84
m_g_src = "bench"
cd = 0.55
cd_src = "bench"
cd_sigma = 0.05
d_sigma_mm = 0.15
d_sigma_mm_src = "bench"
m_sigma_g = 0.05
m_sigma_g_src = "bench"

[launcher]
wheel_d_mm = 40.0
gap_mm = 19.2
gap_mm_src = "bench"
k_slip = 0.743
k_slip_src = "bench"
k_slip_pm_per_krpm = 0.00012
k_slip_pm_per_krpm_src = "bench"
k_slip_gap = [[18.2, 0.700], [19.2, 0.743], [20.2, 0.781]]
sigma_v_rel = 0.027
sigma_v_rel_src = "bench"
sigma_v_floor_mps = 0.35
sigma_v_floor_mps_src = "bench"
spread_mrad = [12.2, 8.0]
spread_mrad_src = "bench"
bias_mrad = [3.2, -1.9]
bias_mrad_src = "bench"
rpm_max = 17500.0
rpm_idle = 4500.0
diff_pm = 15.0
k_spin = 0.8
cl_k = 0.52
cl_k_src = "bench"
spin_tau_s = 6.0
v_cap_mps = 38.0

[launcher.wheel]
J_kgm2 = 4.0e-6
kv = 1800.0
pole_pairs = 7
pole_pairs_src = "bench"
tau_motor_s = 0.04
p_max_w = 100.0
eta_transfer = 0.6
rpm_sigma_pm = 3.0

[launcher.feed]
t_push_ms = 80.0
t_commit_ms = 45.0
t_exit_ms = 8.0
t_exit_sigma_ms = 2.0
```

Notes on the file: `k_slip_gap`, when present, overrides `k_slip` by interpolating at `gap_mm` (set it from the three per-gap fits, and `k_slip` stays as the fallback). `v_cap_mps` above 38 is
rejected by the loader, and the MCU enforces the cap on its own from `wheel_d_mm` and `v_cap_mps` (`calib.firmware_keys()` gives the two CFG values to write: 23 and 24).

## 7. When the fit disagrees with the spec

| symptom                                             | likely cause                                                                                                                             |
| --------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------- |
| `k_slip` below 0.7                                  | gap too wide for the ball, or a worn tyre; fit a smaller gap, or the 45 mm wheel variant (spec 11), and the rpm cap follows `wheel_d_mm` |
| `k_slip` falls with rpm (negative slope)            | wheel slip rises with speed; keep the slope, it is used by the sim and the fire control                                                  |
| `sigma_v_floor_mps` large against `sigma_v_rel * v` | the chronograph or the beam timing dominates; check the chronograph with a second device                                                 |
| `cd` outside 0.3-0.8                                | the distance or the range reference is wrong, or the barrel is not level (a 1 degree error is 17 cm at 10 m); re-level and repeat B3     |
| `bias_mrad` vertical larger than 10                 | barrel not level, or the sheet's axis point is misplaced; the boresight tool (v1) writes the camera-to-barrel offset, not this           |
| `cl_k` fits negative (reported as 0)                | the lift is below the noise at your shot count; shoot more at `diff_pm` 45, or accept the default                                        |
| `fit_*: skipped` lines                              | the log lacks the columns that step needs (the message says which)                                                                       |
