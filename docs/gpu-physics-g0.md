# GPU physics G0: measurements and gate verdict (2026-10-10)

Branch p6-mjx. Plan: `docs/gpu-physics-plan.md` sections 6 and 7. Everything ran on hfbox (16 cores, RX 9070 XT gfx1201), other napl workers running
at the same time: **load average 20 to 30 during every CPU number and the G0a runs**. Treat splits as relative and absolute rates as lower bounds
for the CPU. The GPU may also have been shared with GPUREN. No physics or C++ was changed (the branch carries INFRA's cherry-picked thread-cap commit b9a00f9 only).

**Status: INCOMPLETE.** hfbox went down before three items finished: the accuracy rerun with a non-contact trajectory, the deterministic-flag
runs, and the optional cProfile pass. They are marked below. The gate verdict does not depend on them (E2 fails by 10x).

## Verdict

**NO-GO. Stop rule S3 applies** (E1 install passes, E2 throughput fails). S2 does not apply, S1 does not apply (skydio native p = 31 %, asv 95 %).

| Gate term | Result |
|---|---|
| E1 JAX-ROCm sees RX 9070 XT, MJX scan runs, no NaN | pass, first attempt |
| E2 MJX at N >= 4096 >= 5x 16-thread CPU MuJoCo | **fail**: x500v2 0.10x (f32), 0.15x (f64); asv 0.82x (f32). Even the plan's 2x bar fails |
| E3 f32 within 1 % of travelled distance, 5 s | borderline, see accuracy (0.33 % and 1.14 %, both including a floor impact, both flawed tests) |
| E4 bitwise repeats | **fail** with default flags (3 different hashes in one process) |
| G0a p >= 25 % for at least one showcase | pass for skydio native (31 %) and asv (95 %); hopper 10 %, lander 15 % |

## G0a: where the wall time goes (CPU, `napl program` with the Sim API timers)

Hook: `tools/g0a/sitecustomize.py` (PYTHONPATH=tools/g0a G0A_OUT=<json>). It wraps every public `napl.Sim` method/property, outermost call only:
`step` (Solver step: MuJoCo plus C++ devices plus bus logging), `render`, everything else = marshal (query/set/call/tick/diverged), `load`.
loop = first to last Sim call. python = loop - step - marshal (controller, estimator, planner, perception, numpy). pre-loop = process start until the first Sim call
(imports, scene/design build). Command per scenario: `bash g0a_run.sh <name> <scenario.toml>` = `.venv/bin/napl program <scenario> --runs-dir <d>` with the hook env.

| Showcase (one run) | process | pre-loop | loop | step | marshal | python | p = step/loop | p = step/process |
|---|---|---|---|---|---|---|---|---|
| hopper divert25 (run 1 / run 2) | 38.5 / 39.2 s | 26.4 / 26.9 s (per-seed `design` build) | 12.0 / 12.3 s | 1.25 / 1.27 s | 6.5 / 6.75 s | 4.3 / 4.3 s | **10 %** | 3.2 % |
| lander | 2.29 s | 0.96 s | 1.34 s | 0.205 s | 0.63 s | 0.50 s | **15 %** | 9 % |
| skydio native goal | 84.7 s | 0.2 s | 84.4 s | 26.0 s | 6.5 s | 51.9 s | **31 %** | 31 % |
| asv head_on (PARTIAL: killed at the scenario's 900 s wall limit) | 899.8 s | 722.8 s (scene build, load-inflated, unexplained) | 177.1 s | 168.4 s | 2.4 s | 6.2 s | **95 %** | 19 % (build dominated) |

Facts and caveats:
- Hopper and lander: the MC drivers (`mc.py`) call `prepare` (design build) per seed as well, so a hopper MC run is about 39 s of which MuJoCo plus devices is 1.3 s. The biggest bucket inside the loop is **marshalling** (54 % of hopper's loop, 280k bus calls), larger than the Python flight code. A CPU-side fix (fewer or batched get/set) beats any GPU port for these two.
- Skydio `step` includes the C++ rangefinder rays (`mj_multiRay`) inside the devices; Python (planner, voxel map, depth processing) is 61 %.
- ASV is physics bound (5-vessel hydro, 64 wave components): 2.9 ms per tick under load. Its timings are partial and the 722 s pre-loop was not investigated.
- Not run: cProfile pass (optional). `G0A_CPROF=<pstats>` is supported by the hook.
- Single runs each (hopper twice), box loaded. No variance estimate.

## G0b step 1: install recipe that worked (first attempt)

Python 3.12 venv at /workspace/wt/napl-mjx-venv, about 5 GB; pip cache default.

```
python3.12 -m venv /workspace/wt/napl-mjx-venv && source /workspace/wt/napl-mjx-venv/bin/activate
pip install --index-url https://stable.repo.amd.com/rocm/whl-next/ "rocm[libraries,device-gfx1201]==10.1.0"
pip install --index-url https://stable.repo.amd.com/rocm/whl-next/ "jax_rocm10_plugin==0.11.1+rocm10.1.0" "jax_rocm10_pjrt==0.11.1+rocm10.1.0"
pip install jax==0.11.1 jaxlib==0.11.1
pip install mujoco==3.7.0 mujoco-mjx==3.7.0 numpy      # 3.7.0 = napl cmake/deps.cmake pin; numpy 2.5.3
export XLA_FLAGS="--xla_gpu_enable_command_buffer=" XLA_PYTHON_CLIENT_MEM_FRACTION=0.75
```

Source: AMD JAX install page (ROCm 10.1.0 row, gfx1201). `jax.devices()` -> `[RocmDevice(id=0)]`, `device_kind` "AMD Radeon RX 9070 XT". 4096x4096 f32 matmul and a jit
reduction pass; `memory_stats()` works (bytes_limit 12.8 GB at fraction 0.75). The container's own ROCm 7.2 torch is not needed. The warning "Failed to import warp" from mujoco-mjx is harmless.

## G0b step 2/3: throughput (physics only, bare MJCF, 1000-step `lax.scan`)

Models: `napl design build assemblies/{x500v2,asv}.toml` then `tools/mjx_bench.py prep` strips the napl XML to bare dynamics: massless visual geoms
(contype=conaffinity=0, mass=0), meshes, sites, wire tendons (1530 sites), `custom` devices, plus one `<motor site=...>` thrust actuator per prop. Total mass checked unchanged
(1.54661 kg, 16.16162 kg). Remaining collision geoms are boxes, cylinders and the floor plane only, so **no convex-hull / maxhullvert edits were needed**.
x500v2: 45 bodies, 152 geoms, nv 10, dt 1 ms. asv: 18 bodies, 27 geoms, nv 8, dt 2 ms. Default solver (Newton) and iteration limits kept.

Note: the throughput runs used the first control pattern (hover share x (1 + 0.1 x pattern)), which tumbles and falls; no floor contact happens inside the 1000 steps (z0 = 20 m).
`_ctrl` was changed afterwards to 1.05 x hover + 0.002 x pattern (accuracy rerun pending); step cost does not depend on thrust.

MJX (RX 9070 XT, JAX 0.11.1, flag above), best of 3 reps (1 rep for entries marked *):

| model | dtype | N | compile | env-steps/s | peak VRAM |
|---|---|---|---|---|---|
| x500v2 | f32 | 1 | 49 s | 305 | 0.11 GB |
| x500v2 | f32 | 256 | 39 s | 29,800 | 0.15 GB |
| x500v2 | f32 | 4096 | 200 s | 31,700 | 1.07 GB |
| x500v2 | f32 | 16384* | 154 s | 33,800 | 4.20 GB |
| x500v2 | f64 | 1* | 15 s | 371 | 0.11 GB |
| x500v2 | f64 | 256* | 29 s | 39,800 | 0.20 GB |
| x500v2 | f64 | 4096* | 30 s | 48,500 | 2.20 GB |
| asv | f32 | 1* | 27 s | 315 | 0.09 GB |
| asv | f32 | 256* | 46 s | 64,800 | 0.10 GB |
| asv | f32 | 4096* | 42 s | 542,000 | 0.26 GB |
| asv | f32 | 16384* | 69 s | 557,000 | 0.78 GB |

(x500v2 f64 N=16384 and asv f64 not run. A first, less loaded x500v2 f32 run gave 469 and 35,100 env-steps/s at N=1 and 256.)
x500v2 saturates at about 32k to 48k env-steps/s from N=256 on (linear in N, so the GPU is already full: about 87 box/cylinder-vs-plane collision geoms per env). f64 is not slower than f32 here, so the card is not ALU bound.

CPU, same bare MJCF, 256 rollouts x 1000 steps, best of 2:

| model | `mujoco.rollout` 1 thread | `mujoco.rollout` 16 threads | napl C++ Sweep 1 thread | napl C++ Sweep 16 threads |
|---|---|---|---|---|
| x500v2 | 53,400 | **321,000** | 24,600 | 152,000 |
| asv | 117,000 | **663,000** | 45,400 | 368,000 |

The napl Sweep is slower than raw rollout (bus, per-run clone, one monitor); the gate uses the faster `mujoco.rollout` figure. Thread scaling is only 6x at 16 threads because the box was loaded; an idle box makes the CPU side better, not worse.

Ratio MJX / CPU-16T (physics only): x500v2 f32 N=4096 **0.10x**, N=16384 0.105x, f64 N=4096 0.15x; asv f32 N=4096 **0.82x**, N=16384 0.84x.

## G0b step 3 continued: numerical sanity (5 s, constant controls, x500v2)

Method: `tools/mjx_bench.py acc`. Same start, constant per-motor thrust, MuJoCo f64 reference vs MJX f32 and f64, position sampled every 10 steps.
**Flawed run, as measured:** the control pattern tumbled the vehicle, it fell 20 m and hit the floor at about 2.3 s (displacement 20.2 m), so the numbers include a contact impact. The corrected non-contact test (`--asym 0.002`) was not run.

| asym | travelled | MJX f32 vs MuJoCo f64 (max / % travelled) | MJX f64 vs MuJoCo f64 | MJX f32 vs MJX f64 | first error > 1 cm |
|---|---|---|---|---|---|
| 0.1 | 21.5 m | 0.071 m / **0.33 %** | 0.063 m / 0.29 % | 0.0099 m / 0.046 % | 2.33 s (floor impact) |
| 0.01 | 25.4 m | 0.290 m / **1.14 %** | 0.204 m / 0.80 % | 0.117 m / 0.46 % | 2.91 s |

Reading: after the floor impact, MJX differs from MuJoCo even in f64 (0.29 to 0.80 %), so most of the error is MJX's contact/solver differing from MuJoCo, not float32. The f32-only error (f32 vs MJX f64) is 0.05 to 0.46 %. E3 is therefore unresolved, not cleanly passed: no non-contact result exists.

## E4 determinism (N=256, x500v2, f32)

- Default flags: 3 repeats in one process gave 3 different hashes (517d65bc…, d19953fa…, 9e5bcf44…); 3 separate processes also differed. **Not bitwise reproducible.**
- `--xla_gpu_exclude_nondeterministic_ops=true`: the child process did not finish within 1500 s (hang or extreme slowdown, N=256 normally compiles in about 40 s). Not measured.
- `--xla_gpu_deterministic_ops=true`: not run (box went down). Retry script: `/workspace/wt/napl-mjx-bench/gpu3.sh` (DET loop at N=16, 900 s timeouts).

## Commands (all from the worktree synced to /workspace/wt/napl-video/src; Python = the MJX venv unless noted)

```
napl design build assemblies/x500v2.toml --out $B/models/x500v2          # also asv; $B=/workspace/wt/napl-mjx-bench (napl .venv)
python tools/mjx_bench.py prep  --xml $B/models/x500v2/x500v2.xml --out $B/bare
python tools/mjx_bench.py mjx   --model $B/bare/x500v2.xml --n 1 256 4096 16384 --dtypes f32 f64 --reps 3
python tools/mjx_bench.py cpu   --model $B/bare/x500v2.xml --n 256 --threads 1 16 --reps 2
.venv/bin/python tools/mjx_bench.py sweep --model $B/bare/x500v2.xml --n 256 --threads 1 16        # napl venv
python tools/mjx_bench.py acc   --model $B/bare/x500v2.xml --out $B/acc --asym 0.002
python tools/mjx_bench.py det   --model $B/bare/x500v2.xml --n 256
PYTHONPATH=tools/g0a G0A_OUT=out.json .venv/bin/napl program examples/hopper/divert25.scenario.toml --runs-dir d   # G0a
```

The x64 scan needed a dtype cast of the contact index arrays (int64 out vs int32 carry); the bench does that in the scan body.

## What this means

- S3 per the plan: the code path works, this card is too slow for MJX. A faithful port of napl's device set would then run on a platform where MJX/MJWarp is faster (G5 NVIDIA), not on this box.
- Cheaper wins found by G0a: marshalling dominates hopper and lander loops (54 % and 47 % of the loop); skydio's loop is 61 % Python; none are physics bound except ASV.
- MJX scales badly with the number of collision geoms per env (x500v2 152 geoms at 32k env-steps/s vs asv 27 geoms at 550k). Collision-geom simplification (one hull per body) would be a prerequisite for any further MJX try.
