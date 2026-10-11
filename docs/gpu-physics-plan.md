# napl GPU physics migration plan (research and plan, no code changed)

Date: 2026-10-10. Author: GPUPHYS worker. Scope: read-only repo study plus web research. Nothing was run on hfbox.
All web citations accessed 2026-10-10. Labels used throughout: **[C]** confirmed from a fetched primary source or from repo code
I read; **[I]** inference; **[U]** could not confirm, treat as unknown. Web pages are treated as data.

## 0. Bottom line up front

1. The first thing to settle is not "which GPU engine" but "where does Monte Carlo wall time go". Every napl MC I read is dominated
   by per-step Python (planners, estimators, perception) or by PX4 wall-clock lockstep, not by MuJoCo. A GPU port of physics alone
   is Amdahl-capped. This is the G0a profiling gate and it may end the project cheaply.
2. On the current AMD box the only GPU path with an upstream-supported story is **MJX-JAX on JAX-ROCm** (gfx1201 is named in AMD's JAX
   install page). **MuJoCo Warp / Newton / Isaac Lab are NVIDIA-only today**; Warp has AMD work in flight but nothing shipped.
3. The cheapest real speedup is not GPU: the kernel's own sweep caps at 6 threads (`src/core/bus.cpp:320`), a house rule the user lifted
   for hfbox on 2026-10-10. Raising it to 16 gives up to ~2.7x on native sweeps with no port. Do that first (outside this plan's scope; flagged).
4. A GPU backend only pays at **>= 10^3 to 10^6 parallel envs** (RL, design optimisation, rare-event MC). For today's 24 to 300-run
   MCs it does not. The recommendation (section 9) is: do G0a + G0b (about 2 weeks), then decide with numbers.
5. Determinism: a GPU backend can never carry the "bit-for-bit replay hash" guarantee. It must be a separate, labelled, statistical
   tier. The CPU kernel stays the reference and the only thing that issues replay hashes.

## 1. What napl is, as it matters for porting (all [C], read from the repo)

- Seam: `Solver` (`src/core/solver.h`) is a **single-world, string-addressed, scalar** interface: `step(n)`, `get/set(prim, attr, frame)`,
  `save/restore` bytes, `clone()`, `state_hash()`, plus render and `raycast` hooks. It is not shaped for N-env batches. A batch backend
  therefore cannot sit behind it without contortions; it needs a sibling interface (section 5).
- `MujocoSolver` (`src/mujoco/mujoco_solver.cpp`) calls `devs::step` (`devices.cpp:1026`) instead of `mj_step`: `prepare_motors` ->
  `mj_step1` -> zero applied forces -> servo firmware -> electrical solve -> motor drive -> `step_forces` (env, rotors, drag) ->
  launchers -> solid motors -> airframes -> jet vanes -> vessels -> `mj_step2` -> sensor and range sampling. So **the custom devices
  are the physics**; stock MuJoCo is only the rigid-body and contact core. Device physics lives outside `mjModel`, so MuJoCo's own
  `rollout` module does not run it (see 2.8).
- Timestep 0.001 s for most assemblies, 0.002 s for asv/boat (`assemblies/*.toml`). Hence 1000 or 500 steps per sim second.
- Determinism machinery: own xoshiro256** RNG and Box-Muller (`devices.h:Rng`), state is doubles only and hashed byte-for-byte,
  `dof_damping` canonicalised before `mj_step1` so restores are bitwise equal. Everything is float64.
- Existing batch path: `World::clone()` (`world.h:51`) plus `run_sweep` in `src/core/bus.cpp:241`, workers pull indices from an atomic
  counter, **capped at 6 threads** (`bus.cpp:320`, "house rule: <= 6") and `max_sweep_runs = 1000` (`world.h:21`). Scenario MCs in
  `examples/*/mc.py` and `examples/skydio/montecarlo.py` instead use one OS process per run (`multiprocessing` / subprocess), workers <= 6.
- MC scale today (from the scripts): skydio `--seeds 24` per mission; asv `--seeds 30`; lander `--train 60 --test 300`; hopper
  `--train 40 --test 300`. Single native runs: skydio goal ~25 s wall, follow ~40 s (`examples/skydio/README.md:10-11`); asv ~2 min wall
  for ~2 min sim, five vessels, 125 Hz IMU (`examples/asv/README.md:13`); PX4 runs ~2.5 min each and are **wall-clock lockstep with a
  real PX4 process** (`examples/skydio/README.md:139, 165`; px4_montecarlo is not GPU-able, period).
- Python control code: `examples/**` is 13,354 lines of Python. The per-showcase logic that runs in the loop: skydio 4,150 lines
  (fly 841, planner 427, mapping 297, estimator 236, detector 185, search 149), asv 2,564 (planner 328, run 422, perception 257,
  estimator 230, control 131), hopper 1,787, lander 1,670, cuas 652. Imports are overwhelmingly numpy, math, dataclasses (no scipy/torch
  in the control loops by grep), which is good news for vectorisation, bad news for the amount of rewriting (section 4).

## 2. Options landscape (status as of 2026-10-10)

Summary table, then per-option notes. "Determinism" means bitwise reproducibility of one run on the same binary and hardware.

| Option | AMD gfx1201 | NVIDIA | License | MuJoCo MJCF | Differentiable | Determinism (GPU) |
|---|---|---|---|---|---|---|
| MJX-JAX | listed, caveats [C] | yes | Apache-2.0 (MuJoCo) | subset, see 2.1 | yes | not guaranteed; XLA flags help [C] |
| MuJoCo Warp (MJWarp) | **no** [C-absence] | yes, Turing+ | Apache-2.0 [C] | most features | no [C] | **no** on GPU [C] |
| MJX with `impl='warp'` | no (NVIDIA-only) [C] | yes | Apache-2.0 | as MJWarp | no [C] | as MJWarp |
| Newton | no [C-absence] | yes (via Warp) | Apache-2.0 [C] | via MJWarp solver; MJCF import not confirmed [U] | claims yes | not stated [U] |
| Genesis | claimed via "Quadrants" compiler [C-docs], unverified in practice | yes | Apache-2.0 [C] | parses MJCF [C] | claims yes | not stated [U] |
| Madrona | no (CUDA only) [C] | Volta+ | MIT [C] | no (own XPBD) | no | n/a |
| Brax | via JAX | via JAX | Apache-2.0 [C] | only through its MJX pipeline | yes | n/a |
| Isaac Lab / PhysX | no | RTX with RT cores required | BSD-3 (Lab), Apache-2.0 (Sim, with proprietary bundled parts) [C] | via importer | partially | not guaranteed |
| CPU batch, napl threads | n/a | n/a | n/a | full | n/a | **bitwise, per run** |

### 2.1 MuJoCo MJX (JAX) on AMD ROCm, RDNA4 gfx1201

- **JAX on ROCm lists gfx1201.** AMD's install page names RX 9070 XT, 9070, 9070 GRE and Radeon AI PRO R9700 as gfx1201, with ROCm
  7.14.0/7.14.1 and 10.0.0/10.1.0, JAX 0.9.1, 0.10.0, 0.10.2, 0.11.0, 0.11.1, Python 3.11-3.14, Docker or pip, and pip packages such as
  `rocm[libraries,device-gfx1201]==10.1.0` [C] https://rocm.docs.amd.com/projects/ai-ecosystem/en/latest/frameworks/jax/install.html
  Which JAX versions pair with which ROCm and install method is table-dependent; I did not extract the exact pairing [U]. Linux only.
- **Counter-signal:** the Radeon compatibility page for ROCm 7.2.1 lists the RX 9070 XT but covers only PyTorch, ONNX Runtime, TensorFlow and
  Triton, no JAX row [C] https://rocm.docs.amd.com/projects/radeon-ryzen/en/latest/docs/compatibility/compatibilityrad/native_linux/native_linux_compatibility.html
  So "listed on the JAX install page" is not the same as "validated in the Radeon matrix". [I] Expect rough edges.
- **Evidence of real use on Radeon is thin.** AMD's MuJoCo Playground + JAX tutorial was run on an RX 7900 XTX (RDNA3), ROCm 7.2.0, Ubuntu 24.04,
  and needed `LLVM_PATH`, `HIP_DEVICE_LIB_PATH`, `XLA_FLAGS="--xla_gpu_enable_command_buffer="` and warns Triton GEMM "can destabilize ROCm runs"; no
  throughput numbers given [C] https://rocm.blogs.amd.com/artificial-intelligence/rocm-jax-mujoco/README.html . I found **no** public MJX benchmark on
  RX 9070 or any gfx1201 [C-absence from two searches]. Absence of a benchmark is not evidence it fails; G0 measures it.
- AMD's June 2026 blog says JAX-ROCm CI gates on Instinct silicon, not Radeon [C via search summary, source https://rocm.blogs.amd.com/software-tools-optimization/openxla-jax-rocm/README.html, not re-fetched] [I]: Radeon is best-effort.
- MuJoCo docs: MJX-JAX "runs on Nvidia and AMD GPUs, Apple Silicon, and Google Cloud TPUs" [C]. Gaps: joint types FREE/BALL/SLIDE/HINGE only (fine for napl,
  the devices code requires hinge joints for motors/rotors, `devices.cpp:bind`), solvers CG and NEWTON only, **dense Jacobian only**, no SDF, some primitive-vs-mesh
  pairs unsupported (cylinder vs sphere/box/mesh/hfield), mesh collision by SAT is slow and memory heavy for large meshes, no flex [C]
  https://mujoco.readthedocs.io/en/latest/mjx.html . napl assemblies carry one `<mesh>` per part, so **collision-mesh hygiene (convex hulls, `maxhullvert <= 64`)
  would be required** [I, from the MJX performance tip and the design builder emitting part meshes, `assemblies/README.md:228`].
- Performance statement from the docs: "for single scenes MJX-JAX can be 10x slower than MuJoCo; it works best with thousands of parallel scenes" [C].
- Precision: JAX defaults to float32; float64 needs `jax_enable_x64` set before first import; consumer-GPU FP64 throughput is a small fraction of FP32 [C via
  search summaries; ratio figures are generic, not measured for gfx1201 -> U]. napl's whole device set is float64. [I] A float32 MJX run of a 1 kHz, stiff-ish quadrotor
  with motor inertia and ESC networks is a real numerical-risk item; G0b must test it.
- Determinism: XLA documents nondeterministic GPU ops (scatter, GEMM, convs) and `--xla_gpu_exclude_nondeterministic_ops` to force deterministic variants,
  with autotuning as a further variation source [C] https://openxla.org/xla/determinism . Even then results can differ across GPU architectures [C via search summary].
  It is plausible that a fixed (binary, GPU, driver, flags) tuple reproduces itself; **I do not promise it** [I].
- Verdict: the only option runnable on hfbox now. JAX-on-ROCm-on-RDNA4 is the single largest feasibility unknown (section 7, G0 gate).

### 2.2 MuJoCo Warp (MJWarp) and does NVIDIA Warp support AMD

- **Warp (NVIDIA/warp), Apache-2.0.** README: CUDA wheels need "an NVIDIA R580-series or newer driver and a Turing (sm_75) or newer GPU"; CPU on Windows/Linux/mac;
  no Metal acceleration; **no mention of AMD/ROCm/HIP** [C] https://github.com/NVIDIA/warp .
- **AMD work is happening upstream but is not shipped.** The issue tracker search for ROCm/AMD/HIP shows: PR #1932 "Add WIP HIP/ROCm shims" (open, WIP, from an AMD
  contributor, scaffolding only, "later PRs expected to add full HIP build and runtime support"); an earlier PR #1770 closed unmerged; merged AMD-authored fixes #1702,
  #1865, #1958, #1974, #1997, #1998 (tile reductions "on AMDGPU", HIP mempool, graph-capture); open issue #1957 (HIP stream-capture error paths) [C]
  https://github.com/NVIDIA/warp/pull/1932 and https://github.com/NVIDIA/warp/issues?q=ROCm+OR+AMD+OR+HIP (listing page, dates Aug-Oct 2026).
  [I] AMD engineers are upstreaming a port; there is no released Warp that runs on AMD and no NVIDIA commitment I could find. Do not plan on it before 2027.
- **MuJoCo Warp:** Apache-2.0, jointly developed by NVIDIA and Google DeepMind, "designed for NVIDIA hardware", "requires an NVIDIA GPU for fast simulation but supports CPU for
  development" [C] https://github.com/google-deepmind/mujoco_warp . Officially released with MuJoCo 3.5.0 (2026-02-12) per the changelog [C via search summary]; latest MuJoCo is
  **3.15.0, 2026-10-05** [C] https://mujoco.readthedocs.io/en/latest/changelog.html .
- Docs: throughput-oriented ("a single step is expected to be slower than MuJoCo"), scales better than MJX "on nearly every workload", float32 (MuJoCo defaults to float64),
  **GPU results are not deterministic (atomics)**, deterministic on CPU; **not differentiable**; no named-access/`bind` API; unsupported: `o_margin/o_solref/o_solimp/o_friction`
  overrides, `mjDSBL_MIDPHASE`, `mjENBL_FWDINV`, PGS and noslip solvers, IMPLICITFAST, PLUGIN actuators/sensors; flex experimental; non-zero margins unsupported for some CCD pairs;
  scenes beyond ~60 connected DoF degrade; model edits need a host call per new instance and only certain fields are device-editable [C]
  https://mujoco.readthedocs.io/en/latest/mjwarp/index.html . napl's `prepare_motors` rewrites mass/inertia every step (section 3), which maps onto "device-editable model fields" only if
  those fields are among the batched ones [U].
- Throughput claims: README says the batch renderer reaches "millions of frames per second on NVIDIA GPUs"; a secondary blog quotes 70-313x speedups on an RTX 4090 with unknown baseline
  [C that the blog says it; U whether true] https://pub.towardsai.net/isaac-sim-vs-mujoco-the-4-000-question-that-will-define-robotics-in-2025-4c41a2984c2c . Use `mjwarp-testspeed` on our models instead.
- MJX also has an `impl='warp'` mode (NVIDIA only, no autodiff) [C] https://mujoco.readthedocs.io/en/latest/mjx.html .
- Verdict: **the right engine for the AWS NVIDIA path, not for hfbox.** MJWarp supports all joint types, sparse Jacobians and mesh collision, which removes MJX-JAX's main compatibility gaps.

### 2.3 Newton

- Linux Foundation project initiated by Disney Research, Google DeepMind and NVIDIA; Apache-2.0 code, CC-BY-4.0 docs; MuJoCo Warp is the primary solver backend; fresh installs pick Warp 1.18;
  needs NVIDIA Turing+ and R580+; CPU-only on macOS; no AMD mention; the Releases page was empty when fetched, so I cannot state a stable version [C] https://github.com/newton-physics/newton .
- [I] Newton adds scene authoring, USD and other solvers (cloth, soft bodies) napl does not need. For napl it is a wrapper layer over MJWarp; extra dependency churn (API still moving) for no capability we use.
  Prefer MJWarp directly unless Newton's multi-solver coupling becomes a requirement. Isaac Lab 3.0 beta docs describe an MJWarp solver via Newton, which is a hint about the ecosystem direction [C via search listing].

### 2.4 Genesis (Genesis World)

- Apache-2.0 [C, LICENSE file https://raw.githubusercontent.com/Genesis-Embodied-AI/Genesis/main/LICENSE]. Docs say its "Quadrants" compiler lowers kernels to CUDA, AMD ROCm, Apple Metal, Vulkan, x86 and ARM64,
  parses MJCF among other formats, claims "10-80x" over earlier GPU simulators (methodology not reviewed), and calls itself early: "a small team will not get everything right on a first release"
  [C] https://genesis-world.readthedocs.io/en/latest/_sources/user_guide/overview/what_is_genesis.md.txt . The GitHub README fetch failed (503), so version and rigid-body feature list are [U].
- [I] Interesting only because it nominally targets AMD. But its rigid-body semantics are not MuJoCo's (different contact solver, parameter meanings), so our MJCF, our calibrated contact parameters and our tests
  would not transfer; we would be re-validating the whole sim, not porting devices. Not recommended except as a second-opinion spike if MJX-ROCm fails.

### 2.5 Madrona
MIT, "research code base", GPU backend requires Volta+ NVIDIA and CUDA 12.4/12.8+ (12.5, 12.6 do not work), Linux, own XPBD rigid-body physics, CPU backend for debugging only [C] https://github.com/shacklettbp/madrona .
Not MuJoCo-compatible, NVIDIA only. Not applicable. (Used by other projects for batched rendering; irrelevant here.)

### 2.6 Brax
Apache-2.0; README: "Only `brax/training` is actively being maintained as of 0.13.0", `brax/envs` deprecated for MuJoCo Playground; for MuJoCo simulation use MJX or MuJoCo Warp directly [C]
https://github.com/google/brax . Not an independent option: its useful pipeline is MJX. Skip.

### 2.7 Isaac Lab / PhysX
Isaac Lab is BSD-3-Clause (some packages Apache-2.0); Isaac Sim is documented as Apache-2.0 but Lab's README says it requires Isaac Sim "which includes components under proprietary licensing terms";
Isaac Sim requires RT-core RTX GPUs (A100/H100 unsupported), minimum listed RTX 4080-class with 16 GB [C] https://isaac-sim.github.io/IsaacLab/main/source/refs/license.html ,
https://docs.isaacsim.omniverse.nvidia.com/latest/installation/requirements.html . [I] Heavy, NVIDIA-only, own physics semantics, redistribution licence needs legal review for a proprietary product. Not recommended.
(PhysX-5 licensing itself: not confirmed in this research [U].)

### 2.8 CPU baseline: `mujoco.rollout` and napl's own sweep

- `mujoco.rollout` runs `nbatch` rollouts on an internal C++ thread pool, GIL released, but takes a **precomputed control array** `nbatch x nstep x ncontrol`; the docs do not describe per-step Python
  feedback inside a rollout [C] https://mujoco.readthedocs.io/en/latest/python.html . It also steps stock `mj_step`, so **it would skip napl's device layer entirely**. Useless for napl closed-loop MC as-is.
- The real CPU baseline is napl's own: `Sweep` (threads, atomic work queue, `clone()`) in C++ and process pools in Python. Known limits: threads capped at 6, `max_sweep_runs 1000`, Python controllers
  hold the GIL if run in-process (hence one process per run, memory heavy: ~31 GB box, ~9 GB free on hfbox per `hfbox-access` notes).
- Cheap CPU wins before any GPU work: (a) raise the 6-thread/6-worker caps on hfbox where the user allowed 16 jobs, (b) measure per-run RSS and cut it, (c) profile Python vs physics (G0a),
  (d) keep the controllers in a fast loop (numba/Cython on the hot spots) if the profile says Python dominates. None of these changes determinism.

## 3. Fit: which devices port to GPU, effort per family

Method: I read `devices.h` in full, `devices.cpp:1026-1075` (the tick), `aero.cpp:100-199` (env, rotors, drag), and the header comments of `electrical.cpp`, `rocket.cpp`, `vessel.cpp`, `sensors.cpp`,
`launcher.cpp`, `rangefinder.cpp`. Line counts: devices.cpp 1,336 (mostly parse, bind, attribute tables), electrical 250, aero 199, vessel 197, launcher 162, sensors 157, rocket 156, servo 115,
rangefinder 63, vane 49. The estimates below are person-weeks for one engineer **including tests against the CPU device**, assuming the engine host (MJX-JAX or MJWarp) already steps a napl MJCF. They are [I] and uncalibrated;
the first family done (rotors) recalibrates the rest.

Porting principle [I]: express every device as `state' , forces = f(cfg, state, qpos/qvel/kinematics, inputs)` over struct-of-arrays with leading env axis `[N, ...]`. The CPU code is already
data-oriented (config structs + POD state, free functions, no virtual calls), so the shape matches. Device state is already "doubles only, no padding", i.e. it maps 1:1 to a flat array.

| Family | Code | Pure array fn? | GPU difficulty and why | Est. effort |
|---|---|---|---|---|
| Rotor / prop / EDF thrust maps | `aero.cpp` rotor loop, `RotorCfg` | Yes. `interp` of Ct(J), Cq(J), Reynolds factors, rho, ground effect, hub drag | Easy. Two non-pure bits: (a) **ground effect uses `mj_ray` downward**, so needs a ray against the scene (MJX has ray functions for some geom types [U for meshes]; or approximate with analytic terrain height); (b) marine props read sea clock/eta. EDF stator torque handoff is a second `xfrc` add. | 1 to 1.5 wk |
| Body drag, ISA, wind | `aero.cpp` | Yes | Easy. Dryden turbulence is a 3-state filter driven by our xoshiro RNG: replace with `jax.random` / Warp RNG per env. **Statistics match, bit streams do not** (loses CPU-vs-GPU per-sample comparability, keeps distributional comparability). | 0.5 to 1 wk |
| Battery (OCV table, R0 + RC), PSU, wire, load | `electrical.cpp` | Yes | Easy for state update; noise via RNG as above. | 0.5 wk |
| Electrical network + ESC modes | `electrical.cpp` (dense nodal analysis <= 128 nodes, Gaussian elimination, mode re-solve up to 32 iterations, current-limit fixed point) | Mostly. Linear solve is a batched small dense solve; the mode and current-limit loop is data-dependent control flow | **Medium.** Needs a fixed-iteration (`lax.fori_loop`/masked) formulation; per-env different iteration counts waste lanes; zero-multiplier skipping tricks irrelevant. Batched `solve` of n~10-30 nodes is fine on GPU. Bitwise-equal to CPU impossible, tolerance-equal is testable using the existing `network_star`/`network_chain`/`servo_chain_kcl_and_regen` tests as oracles. | 2 to 3 wk |
| BLDC motor + ESC drive (`drive_motor`, loss torque tanh model) | `devices.cpp:1001` | Yes | Easy. Writes `qfrc_applied` and `dof_damping`/`armature` on the driven dof (`bind` overrides armature, damping "owned"): need the engine to allow per-env armature or fold into forces. | 0.5 to 1 wk |
| Servo firmware (Feetech STS, Dynamixel, RoboClaw modes) | `servo.cpp` + `ServoState` (~45 fields) | Per-env scalar logic, heavy on branches (`ticks % period`, alarm latches, unload masks, ramping, 3 modes) | Medium. Every `if` becomes `where`/select; integer counters (`ticks`, `counts`) need exact int semantics. Jam spring/damper is a force term. Only the arm/gimbal showcases use servos; skip for quadrotor/hopper MC. | 2 to 3 wk |
| IMU, baro, mag, GNSS | `sensors.cpp` | Mostly. Bias random walks, noise, quantisation, **ring-buffer latency** (kGnssRing=64), rate gating by tick modulus, Gauss-Markov GNSS error | Medium-easy. The IMU specific force uses `mj_rnePostConstraint` with the *pre-step* `qvel` (devices store `qvel_k` and `qvel_k1`): in MJX use finite differences of qvel plus gravity, **numerically different from rne**; must be validated, [U] whether MJX exposes an equivalent. | 2 wk |
| Solid motor, airframe aero, jet vanes | `rocket.cpp`, `vane.cpp` | Forces yes; **`prepare_motors` mutates `body_mass/inertia/ipos/subtreemass` every step before `mj_step1`**, replicating `mj_setConst` summation order | Hard-ish. Needs per-env mutable model fields. In JAX the model pytree can be vmapped over env so per-env `body_mass` is feasible [I]; derived `body_subtreemass` and composite inertia must be recomputed consistently; the deliberate "keep stale invweight0" behaviour (documented in `rocket.cpp`) must be copied. Hopper/lander depend on these. | 2 to 3 wk |
| Vessel hydro (buoyancy cells, waves, added mass, fins) and sea | `vessel.cpp`, `SeaCfg` | Cell loops over submerged volume with up to 64 wave components: vmappable. **Added-mass correction needs `mj_jacBodyCom`, `mj_solveM`, `mj_applyFT`** (mass-matrix solve per env) | Medium-hard. MJX exposes jacobian and mass-matrix solve utilities [U exact names/shape support]; ASV is 5 vessels so this is the core cost. | 3 to 4 wk |
| Launcher (pellet pool, swept ray test, hit ring) | `launcher.cpp` | **No.** Event-driven: spawn on `fire`, pool of 16, swept `mj_ray` per pellet per tick, ring-buffer hit log, RK4 | Hard. Dynamic-size pools become masks; needs batched ray casts against scene geoms. Only the cuas showcase. | 3 to 4 wk, defer |
| Rangefinder / camera depth | `rangefinder.cpp` (`mj_multiRay` 160x90 = 14,400 rays at 25 Hz in skydio) | **No** (scene queries) | Hard. MJX-JAX ray is per geom; MJWarp has a batch renderer on NVIDIA only. Skydio's perception loop consumes this depth image. | 4+ wk, skip |
| Fault inputs (F3), spawn/delete, structural edits, save/restore bytes, replay log | `Solver::spawn/remove`, `save/restore`, `model_bytes` | n/a | The GPU batch tier should **not** support structural edits; faults map to input arrays (damage, fault_open, ext_torque, jam). | 0.5 wk |

Totals [I]: a minimum useful set for a quadrotor-class MC (rotors, drag/wind, battery+ESC network, motors, IMU/baro/GNSS) is about **7 to 10 person-weeks**; adding rocket/hopper/lander
(solid motor, vanes, airframe) is about +3 wk; ASV hydro +4 wk; servo +3 wk. Skydio's depth perception and PX4 variants are outside the GPU tier.

Hard cases stated plainly:
- **Event-driven logic** (launcher pool, servo alarms, solid-motor phases, GNSS ring) ports as masked state machines. Doable, but each is a new correctness surface.
- **Contact-dependent effects.** Rotor ground effect (ray), launcher hits, rangefinder. Contacts themselves are the engine's job and carry the engine's differences (soft contacts, solver, float32).
- **Anything that mutates the model per step** (solid-motor mass). Engine-specific.

## 4. Python controllers: what happens to the per-step loops

Today: `Sim.step(n)` is called from Python, which reads sensors via `get()`, runs a planner/estimator, and writes setpoints via `set()` (example: `examples/skydio/fly.py:258-266`, `IMU_TICKS` per control tick).
To run N envs on a GPU, the controller must either (a) be vectorised over the env axis, run on the same device (JAX `vmap`/`jit` or Warp kernels) inside the rollout `lax.scan`, or (b) run on the host CPU
once per control tick with a device-to-host copy of N x sensor rows (feasible at 100 to 1,000 envs, a PCIe round trip per control tick, then you're back to CPU-bound Python on N envs).

Reality check [I]: option (b) gives no win when the controller dominates, and (a) is a rewrite. Per showcase (lines taken from section 1):

| Showcase | In-loop Python | Vectorise cost | Verdict |
|---|---|---|---|
| lander (1,670 lines) / hopper (1,787) | guidance (160), estimator (159 to 293), control (216), flight loop (284 to 559); numpy-only math, small fixed-size state | Moderate: EKF and guidance in fixed-size array form port to `jax.numpy` almost line-by-line; branchy mode logic becomes `where`. Best candidate. | **G3 target**. About 4 to 6 wk to vectorise + validate. |
| asv (2,564) | planner (328, sampled COLREG decisions), estimator (230), perception (257), AIS + traffic simulation, judge | Moderate-high: multi-vessel, discrete encounter logic, per-run randomised geometry. Needs hydro device (+4 wk). | Second target. 6 to 8 wk plus hydro. |
| skydio (4,150) | depth image -> point cloud -> rolling voxel map -> sampled-primitive planner (427 + 297 lines) + detector; PX4 path uses MAVLink to a real process | High: voxel map and primitive scoring are vectorisable but the depth rays make the sim side hard. PX4 variant is impossible on GPU. | **Do not target.** Keep on CPU. |
| cuas (652), faults (179), koch/so101 arms | launchers, servos, IK | Event-driven and servo heavy | Do not target. |
| `examples/*/mc.py` drivers | process pools, JSON/Markdown summaries | Rewrite to a single batched call; summary/Wilson-interval code in numpy can stay | Small (~1 wk each) |

Rough rewrite share: of 13,354 lines in `examples/**`, **about 3,500 to 4,000 lines** (hopper + lander control stacks and their mc drivers, plus ~1,000 shared estimator/guidance helpers) would need
a vectorised twin for the G3 + G4 scope; ASV adds ~1,500; skydio ~0 (stays CPU). The CPU originals must remain as the reference, so this is **a second implementation, not a migration** [I]. A shared pure-numpy
"core step" function operated on arrays shaped `[N, ...]` (N=1 on CPU) is the lowest-drift strategy: one source for both tiers, checked at N=1 against the existing per-run scripts.

## 5. Recommended architecture

```
                         +-------------------------------+
  Scenario / MC driver   |  napl.batch (Python)          |  seeds -> param arrays [N, ...]
  (Python)  ------------>|  BatchRunner API              |  ---> results table + stats
                         +------+------------------+-----+
                                |                  |
                    reference tier (CPU)      batch tier (GPU, statistical)
                    Solver (unchanged seam)   BatchSolver (NEW sibling interface)
                    MujocoSolver, C++ devices  MJX-JAX (hfbox) / MJWarp (AWS NVIDIA)
                    bit-exact, replay hashes   device ports as array functions, float32 (maybe 64)
                                |                  |
                                +--- cross-validation harness (held-out seeds) ---+
```

Design decisions [I]:

1. **Keep `Solver` untouched.** It is string-addressed scalar get/set with `save/restore` bytes and `state_hash()`; none of that is meaningful for 10^4 envs in device memory.
   Add `BatchSolver` as a sibling (not a subclass): `reset(params[N,P], seeds[N])`, `step(n)` or `rollout(horizon, controller)` fused, `observe(names) -> array[N,...]`, `inputs(names, array[N,...])`,
   `done_mask`, `diverged_mask`. Lives in Python (`napl.batch`) with the engine adapters; no C++ change in `napl_core` (the seam rule "napl_core must never include a solver's headers" is respected trivially).
2. **One model source.** Both tiers consume the same `napl design build` MJCF + `napl.devices` JSON (`schema/devices.fbs`). The batch tier parses the same JSON into struct-of-arrays configs. A parity test asserts every config field
   the CPU parser reads is either consumed or explicitly marked unsupported by the batch tier (fail closed: unsupported device in a model => the batch runner refuses, never silently drops it).
3. **Engine adapter layer, two adapters, one device layer.** Write the device functions as pure functions of arrays against a thin array namespace; first adapter MJX-JAX (runs on hfbox ROCm and on AWS NVIDIA via JAX CUDA),
   later adapter MJWarp via `mjx impl='warp'` or `mujoco_warp` directly (NVIDIA only). Avoid Warp-only and JAX-only device code where a shared formulation exists [I]. Warp kernels are not callable on AMD today (section 2.2).
4. **CPU tier gets a float64 JAX twin for free.** JAX on CPU with x64 gives a way to run the *same device code* in float64 on CPU and compare to the C++ devices at tolerance ~1e-9 relative on identical inputs (inject recorded `qpos/qvel/kinematics`
   per tick). That isolates device-port bugs from engine (contact/solver/float32) differences. This is the most important test-design trick in the plan [I].
5. **Statistical cross-validation on held-out seeds.** The GPU tier is never validated on seeds used to tune it. Protocol: pick training seeds for any parameter fix-ups (e.g. contact tweaks); define frozen held-out seeds (like the existing `SEED_BASE = 5000` convention in `examples/asv/mc.py`);
   compare distributions of per-run outcomes between CPU (reference) and GPU: success rate with Wilson intervals (already used in asv), two-sample KS or Anderson-Darling on continuous metrics (touchdown error, cpa, xte, min clearance), and a paired comparison of the *same seed* on both tiers
   (they should be close for the first seconds, and may diverge later: chaos). Pass thresholds set before running (e.g. success-rate difference within the sum of the two 95 % half-widths; KS p > 0.01 on each metric; no metric mean shifted by more than X % of its CPU std, X chosen in G4).
6. **Hashes and reporting for GPU runs.**
   - GPU runs **never** emit a replay hash, never go into `.naplog`, and never claim "bit-exact". Output header: `tier=batch`, engine name+version, device name, driver/ROCm/CUDA version, dtype, XLA/Warp flags, model hash, device-config hash, seed array hash, napl git SHA.
   - Provide a *config digest* (hash of model + devices + seeds + software versions) so a result is identifiable and comparable, and a *result digest* over the output table rounded to a stated precision. The result digest is advisory (same tuple may or may not repeat); say so in the report.
   - Any run the user wants to keep as evidence is **re-run on the CPU kernel from the same seed** (selected failures, the worst-k, a random sample) to produce a real replay hash. "Triage on GPU, certify on CPU" is the contract. The CPU re-run may differ in outcome from the GPU run (chaos, float32): report both, never silently overwrite.
   - Determinism *best effort* settings are recorded and tested for self-consistency (same machine, same flags, 3 repeats) so that "GPU run repeated itself" is a measured claim per release, not a promise.
7. **Fail closed on divergence.** `diverged_mask` is reported per env; NaN/inf envs are excluded from stats but counted and listed; the success rate denominator includes them as failures.
8. **Licence hygiene.** MJX/MuJoCo, JAX, Warp, MJWarp and Newton are Apache-2.0 [C]. Check ROCm user-space licences (mixed, includes MIT/Apache/NCSA and some LGPL-ish components) and the NVIDIA CUDA runtime redistribution terms before shipping a binary [U, not researched]. Treat this as a dev/CI tool, not shipped inside the proprietary product, until legal checks pass. No GPL dependency is introduced by any plan item [I; verify transitive deps in G0].

## 6. Cost/benefit: expected speedups on napl's actual MC workloads

Honest framing: I have **no measurement** of napl's per-run time split and no MJX throughput on gfx1201. Numbers below are explicit hypotheses with ranges, each tagged for G0 to confirm or kill.

Workload units: runs R x sim seconds T x steps/s (1,000 for dt 1 ms; 500 for ASV). Examples from the scripts:

| MC | R | T (s) | steps | Wall today (est.) | Where wall goes (hypothesis) |
|---|---|---|---|---|---|
| lander test set | 300 (+60 train) | tens of s | 3e5 to 1e6 per 100 runs | `--workers 6`, 3600 s outer timeout in the docstring | Python flight loop + estimator probably dominates [U] |
| hopper test set | 300 (+40) | tens of s | similar | same | same |
| skydio goal/follow | 24 per mission | <= 45 s | ~4e4 each | 24 x 25 to 40 s / 5 workers ~ 3 to 4 min [I] | depth rays + voxel map + planner [I] |
| asv | 30 | ~120 s | 2.4e5 (5 vessels, 2 ms) | ~2 min per run, 6 workers -> ~10 min [I] | Python planner/perception/AIS + 5-vessel hydro [U] |
| skydio PX4 | 12 to 24 | ~60 s | n/a | ~2.5 min each, lockstep | wall-clock PX4: **GPU gives 0x** |

Amdahl sketch [I]. If physics (MuJoCo + C++ devices) is a fraction p of per-run time and a GPU backend with vectorised controllers speeds the *whole* loop by factor S_g at high N, but you only do N runs once per batch:
- Today's MC sizes (R <= 300): a 300-env batch on a GPU that is 10x slower than CPU per step but massively parallel finishes the physics in ~1 batch time. Even a 5x end-to-end win saves a few minutes per MC invocation; **the port costs 10 to 20 person-weeks.** Break-even is on the order of hundreds of MC invocations [I]. This does not justify itself for the existing suites.
- A back-of-envelope for the physics alone: a quadrotor-scale model on one CPU core at roughly 20 to 100 µs/step for MuJoCo plus devices [U, unmeasured] gives 1e4 to 5e4 steps/s per core, 16 cores ~1.6e5 to 8e5 steps/s (not counting Python). A 300-run x 60 s x 1 kHz MC is 1.8e7 steps -> 20 to 110 s of pure physics on all cores. **Physics is minutes, not hours.** If runs currently take ~10 min wall at 6 workers, the other 80 to 95 % is Python/process/IO [I].
- Where the GPU can win: 10^4 to 10^6 envs (RL/policy training, tail-risk estimation at 1e-4 failure rates needs ~1e5 runs for 10 failures, design-space sweeps `napl explore sweep|wind|mc` marked "[not run]" in `docs/guide.md:555`). MJX-JAX on a 16 GB consumer card might do on the order of 1e5 to 1e6 env-steps/s for small models [U, extrapolation from the docs' "thousands of scenes" guidance, no gfx1201 number exists]; float32; JIT compile times of minutes per new model/scan are a fixed cost. At those rates 1e5 runs x 30 s x 1 kHz = 3e9 steps = 1.5 to 8 hours on the GPU versus about 1 to 5 hours of *physics-only* on 16 cores. **So even for a million-step scale, a mid-range AMD card may only roughly match 16 CPU cores on pure physics** [I, high uncertainty, factor ~5 either way]. The real GPU advantage (and the 100x claims) comes from NVIDIA-class hardware plus MJWarp [I].
- Expected shape of results by phase: G0 physics-only speedup vs 16-core CPU at N=4096 for a quadrotor MJCF with no devices: **0.3x to 3x** on hfbox/MJX-ROCm; on a g5/g6e-class NVIDIA with MJWarp: **5x to 50x** [I; the 50x end is the blog-level claim, unverified]. With devices ported and controllers vectorised, add Python removal on top (the large factor is eliminating the interpreter, which a CPU-side vectorised numpy/numba loop can also capture; G0a quantifies how much).
- Costs: engineering 10 to 20 pw for a useful tier (section 3 + 4 + G4), ongoing dual-implementation maintenance (every device change lands twice, mitigated by the parity harness), risk of GPU/driver churn on a consumer card with best-effort JAX support.

## 7. Phased plan with failable exit checks

Style: each milestone has a deliverable, an exit check that can fail, rough effort, and a stop rule. Effort in person-weeks (pw). Numbers marked "set at G0" are thresholds to fix before the data is seen.

### G-1 (optional, outside GPU scope, 0.5 pw): lift CPU caps
- Change: raise `bus.cpp:320` thread cap and MC worker caps behind a config for hosts that allow it; measure per-run RSS (memory limit ~9 GB free on hfbox).
- Exit: skydio/asv/lander MC wall time at 16 workers vs 6 on identical seeds; **identical per-seed summaries** (determinism preserved). Fails if any per-seed result changes.

### G0a (1 pw): where does the time go
- Deliverable: profiling report for lander, hopper, asv, skydio-native runs: wall split into (i) `Solver.step` physics+devices, (ii) Python controller/estimator/planner, (iii) get/set marshalling, (iv) process start/IO/render. Use `py-spy`/`cProfile` for Python, existing `src/mujoco` timers or perf for C++. Run on hfbox (not now: this plan ran nothing).
- Exit check: table with p = physics fraction per showcase. **Stop rule S1:** if p < 25 % for every candidate showcase, a physics-only GPU port yields < 1.33x end-to-end; stop the GPU line, spend the effort on numba/vectorising controllers on CPU and on the caps above. (Threshold set now: 25 %.)

### G0b (2 pw): feasibility spike on hfbox, JAX-ROCm + MJX
Steps (all in a private venv/dir; GPU jobs under `flock /workspace/airtools/.gpu.lock`; /workspace was 87 % full (26 GB free) on 2026-10-10, wheels are GB-scale: check `df` first; no apt):
1. Check ROCm version on hfbox (not known to me [U]); install JAX per AMD's page with the gfx1201 device package; `jax.devices()` shows the GPU; run a matmul and `jit` smoke test; repeat with `XLA_FLAGS=--xla_gpu_enable_command_buffer=` if hangs (per AMD's tutorial workaround).
2. `pip install mujoco mujoco-mjx` (pin to MuJoCo 3.15.0 if the wheel exists [U]); load an existing napl MJCF (`napl design build assemblies/x500v2.toml` -> `build/assemblies/x500v2.xml`; collision meshes may need `maxhullvert<=64` and decimation); `mjx.put_model`, `jax.vmap(mjx.step)`, `lax.scan` of 1000 steps at N in {1, 64, 1024, 4096, 16384}.
3. Throughput table: env-steps/s vs N vs the CPU (`mujoco.rollout` and a pure C++ loop on the same MJCF without devices, 16 threads), float32 and float64 (`jax_enable_x64`), compile time, peak VRAM. Run the same model with `--xla_gpu_exclude_nondeterministic_ops` and measure the slowdown and whether 3 repeats are bitwise equal.
4. Numerical sanity: same initial state, same constant input, compare 5 s of trajectory MJX-f32, MJX-f64 vs C++ MuJoCo f64 (position/velocity RMS, divergence time).
- Exit checks (fail any -> branch):
  - **E1 install:** JAX-ROCm sees gfx1201 and passes a 100-step MJX scan without NaN or driver hang. Fail -> **Stop rule S2** below.
  - **E2 throughput:** at N >= 4096, env-steps/s >= **2x** the 16-core CPU physics-only rate for the same model (threshold set now). Fail -> MJX-ROCm on this card is not a win; the AWS-NVIDIA path may still be (see S3).
  - **E3 fidelity:** MJX-f32 vs MuJoCo-f64 hover trajectory differs by < 1 % of travelled distance over 5 s with the quadrotor MJCF. Fail -> float64 only; re-measure E2 in f64 (consumer FP64 will likely fail it) -> stop on this hardware.
  - **E4 determinism:** with the deterministic flags, 3 identical repeats are bitwise equal on one machine. Fail -> record, still allowed (the batch tier is statistical anyway) but the report must not claim repeatability.
- Decision gate after G0a + G0b (this is the "stop or switch" gate):
  - **S1** (G0a) p < 25 %: stop GPU physics; CPU-side vectorisation instead.
  - **S2** (E1 fails, or ROCm hangs/crashes on RDNA4 in two attempts of at most 2 days each with documented workarounds): **drop AMD GPU for physics**; keep the work product (profiling, parity-harness design) and switch to an NVIDIA-only plan: skip G1 to G4 on hfbox, do them on a rented AWS NVIDIA instance with MJWarp from the start, or defer until the AWS move. Do not invest in Genesis-on-ROCm unless the user wants a long-shot.
  - **S3** (E2 fails but E1 passes): the code path works, hardware is too slow. Continue only if the user wants the AWS target: develop device ports against MJX-JAX CPU-f64 (the oracle) and benchmark on NVIDIA in G5 first.
  - **GO** if E1, E2, E3 pass and p >= 25 % (or controllers will be vectorised anyway).

### G1 (2 pw): batch interface (design + skeleton)
- Deliverable: `napl.batch` Python package with `BatchSolver` protocol, `JaxMjxBatch` adapter running the **bare MJCF** (no devices) in a `lax.scan`, param/seed arrays, `diverged_mask`, result table writer with the section-5 header. Unit tests only for plumbing.
- Exit: (a) a batch of N=1 reproduces C++ MuJoCo f64 on a rigid-body-only napl model (free fall + hinge pendulum, no devices) within tolerance set by the E3 result; (b) refusing a model that has any `napl.devices` entry the batch tier does not implement (test must fail if the refusal is removed); (c) header fields present and a CPU re-run of seed k is requestable by seed index.

### G2 (6 to 10 pw depending on scope): device ports, in order of value
Order: (1) rotors/drag/env/ISA (1.5 pw); (2) battery+PSU+wire+ESC network+motor drive (3 pw); (3) IMU/baro/GNSS (2 pw); (4) solid motor/vane/airframe for hopper+lander (3 pw, optional by showcase); (5) hydro for ASV (4 pw, optional); servo/launcher/rangefinder explicitly out of scope.
- Exit per device, same shape each time: run the **C++ devices and the JAX-f64-CPU twin on the same recorded inputs** (recorded tick inputs: qpos, qvel, kinematics, commanded throttle) for 10^3 ticks; per-output relative error < 1e-9 for pure functions, < 1e-6 for the iterative electrical solve; then on-GPU f32 error < 1e-3 relative. Failing any bound blocks the next device.
- Coverage ledger: a generated table `docs/` entry of device types x {CPU, batch f64-CPU, batch f32-GPU} so unsupported combinations are visible (not silent).

### G3 (4 to 6 pw): one vectorised showcase, lander or hopper (not skydio)
- Why lander/hopper: numpy-only controllers, small state, solid-motor/jet-vane devices already enumerated; ASV needs hydro and 5-vessel logic (second candidate); skydio needs depth rays and cannot do PX4 variants.
- Deliverable: guidance/estimator/controller as array code shared between CPU (N=1) and batch (N>1); the flight loop as a `lax.scan` over control ticks.
- Exit: (a) **N=1 parity**: the vectorised controller reproduces the current scripted controller's outputs on 20 recorded CPU runs to tolerance (command arrays, per tick); (b) the batch tier flies the same seed set and the success flag matches the CPU on >= **X %** of seeds (X fixed at G0; 85 % suggested as a first pass for a chaotic landing task, as an agreement rate not an accuracy target); (c) throughput at N=1024 on target hardware recorded with the Amdahl split.

### G4 (3 pw): Monte Carlo parity on held-out seeds
- Protocol: freeze seed ranges (training for tuning vs held-out, e.g. 5000+ like `asv/mc.py`) before running; CPU reference run of >= 300 held-out seeds (lander/hopper already do 300 test seeds); GPU run of the same seeds plus 10x extra seeds.
- Exit checks (thresholds committed to the repo before the run): success-rate difference within combined 95 % Wilson intervals; KS test on each reported continuous metric not rejected at p = 0.01 (or effect-size bound); no failure *class* seen on CPU is absent on GPU or vice versa, including contact-class failures; divergence rate on GPU <= CPU + 1 %. Failing a check is a valid outcome: it produces the "known GPU model differences" list, and G5 does not start until the owner accepts or fixes each.
- Also ship the "certify on CPU" flow: top-k worst GPU seeds re-run on CPU with replay hashes; report disagreement.

### G5 (3 to 5 pw): AWS NVIDIA path
- Adapter swap to MJWarp (`mjx impl='warp'` or `mujoco_warp`), keep device code; measure vs MJX-JAX on CUDA on the same instance. Target instance class: any CUDA Turing+ GPU with a supported driver (R580+ per the Warp README); candidate instance families (e.g. g5/g6/g6e) and spot prices are **not researched here** [U].
- Exit: (a) all G2 per-device tests and the G4 statistical checks pass on the NVIDIA stack with the same thresholds; (b) throughput table vs 16-core CPU physics-only and vs the end-to-end MC; (c) cost per 10^4 runs compared to CPU instances (AWS pricing to be filled in; USD/run on both); fail -> keep the CPU-only cloud path.
- Note: MJWarp is float32 and its determinism statement is explicit (GPU non-deterministic) [C], so the header and "certify on CPU" rule stay the same.

Timeline [I]: G-1 + G0a + G0b about 3.5 pw (can finish in 2 calendar weeks with parallel work); G1 2 pw; G2 6 to 10; G3 4 to 6; G4 3; G5 3 to 5. Full path 18 to 30 pw, with decision gates after the first 3.5.

## 8. Risks and mitigations

| Risk | Likelihood | Impact | Mitigation |
|---|---|---|---|
| JAX-ROCm unstable on RDNA4 consumer card (CI is Instinct-only per AMD blog) | medium-high [I] | blocks hfbox path | G0b E1 with 2-day timebox; AMD page lists gfx1201 so install should work; docker image is the fallback if hfbox allows [U]; S2 switch |
| Physics is a small share of MC wall time | high [I] | GPU port pointless | G0a first, 25 % rule |
| float32 inadequate for 1 kHz device-coupled dynamics | medium | forces f64, consumer FP64 slow | E3; JAX-CPU-f64 oracle; scale/normalise states; per-device tolerance tests |
| Dual implementation drift (C++ and JAX devices) | high over time | silent mismatches | generated coverage ledger; N=1 parity tests in CI against recorded CPU tick inputs; schema-driven config parity check; every device PR must touch both or mark "batch: unsupported" |
| Mesh collision in MJX-JAX: SAT cost/memory, unsupported pairs | medium | slow or error on part-mesh MJCFs | convex-hull collision geoms with `maxhullvert<=64` (docs tip); use collision groups; for MJWarp mesh collision is fully supported [C] |
| Batch tier diverges from CPU in contact-rich events (landing, ground effect) | medium-high | G4 fail | tolerate; classify; keep CPU certification flow; report contact-class stats separately |
| User trusts GPU numbers as certified | medium | integrity | header `tier=batch`, no replay hash, mandatory CPU re-run for any quoted failure, docs wording |
| JIT compile time minutes per model/scan, VRAM 16 GB limit, shared box GPU lock (jobs "minutes long") | medium | usability | persistent compile cache; chunk N; schedule under `flock`; AWS box has no such limits |
| Licence / redistribution of ROCm or CUDA runtime in a proprietary product | low-medium | ship blocker | keep GPU tier out of the shipped artifact until legal review [U]; no GPL deps; Apache-2.0 for all core libs [C] |
| Upstream churn (Warp 1.x, MJWarp, Newton API) | high | maintenance | pin versions (MuJoCo 3.15.0, Warp as required by MJWarp); upgrade deliberately; MJX-JAX path less churn than Newton |
| Warp/AMD arrives mid-project | low-medium | positive | adapter design allows swapping later; do not plan on it |

## 9. Recommendation

1. **Do not start a GPU physics port yet.** Run G-1 (raise CPU caps; cheap, deterministic, maybe 2 to 3x) and G0a (profile) first. If physics is less than a quarter of MC wall time, a GPU port cannot pay and the money is better spent vectorising or JIT-compiling the Python controllers on CPU.
2. If G0a says physics is a meaningful share, or you want 10^4+ run sweeps / RL / tail-risk work, run **G0b on hfbox with MJX-JAX on ROCm** (2 pw, timeboxed). It is the only GPU stack that is both licence-clean and nominally supported on gfx1201. Set the stop/switch rules S1 to S3 now so the outcome is not argued afterwards.
3. Architecture if it proceeds: CPU kernel stays the deterministic reference and the only hash issuer; a Python `BatchSolver` tier beside (not behind) the `Solver` seam; device ports as pure array functions with a float64-CPU JAX twin as test oracle; statistical parity on held-out seeds; GPU results labelled non-reproducible and always certified by CPU re-runs.
4. First vectorised showcase: **lander or hopper** (numpy-only controllers, rocket devices), then ASV. **Skydio stays CPU** (depth rasters, PX4 wall-clock lockstep). Cuas, arms and faults stay CPU.
5. Target hardware: treat hfbox as a feasibility and correctness bench and AWS NVIDIA with MJWarp as the performance target; MJWarp and Newton do not run on the RX 9070 XT today and I found no committed Warp/AMD timeline.

## 10. Open questions for the user

1. What do you need the GPU for: faster existing 24 to 300-run MCs, or new scale (10^4+ runs, RL, design optimisation, rare-event estimates)? The answer decides whether this is worth doing at all.
2. Is a statistical, non-bit-reproducible batch tier acceptable as long as quoted results are re-certified on the CPU kernel? Any customer-facing claim that must be hash-backed from the GPU tier kills the plan.
3. Which showcase matters most for the first GPU MC: lander/hopper, ASV, or skydio? (Skydio's depth rays and PX4 variants make it the worst fit.)
4. When is the AWS move, and may we prefer NVIDIA there from day 1 (MJWarp) and treat hfbox only as a bench?
5. May G0b install a ~GB-scale ROCm JAX stack on hfbox given /workspace is ~87 % full, no apt, and the GPU lock rule (jobs "minutes long")? What ROCm version does hfbox run?
6. Is a float32 physics tier acceptable, or must the GPU tier match float64 (which probably rules out the RX 9070 XT at useful speed)?
7. Does the proprietary product ship a GPU component, or is the GPU tier an internal validation/analysis tool only (affects the ROCm/CUDA redistribution review)?
8. Should the `Sweep` thread cap (6) and `max_sweep_runs` (1000) be made configurable now, independent of this plan?
   (Orchestrator note 2026-10-10: G-1 assigned to the INFRA worker, p6-infra.)

## Sources (all accessed 2026-10-10)

- AMD, Install JAX for ROCm: https://rocm.docs.amd.com/projects/ai-ecosystem/en/latest/frameworks/jax/install.html
- AMD, ROCm on Radeon compatibility (native Linux): https://rocm.docs.amd.com/projects/radeon-ryzen/en/latest/docs/compatibility/compatibilityrad/native_linux/native_linux_compatibility.html
- AMD, MuJoCo + JAX tutorial on ROCm (RX 7900 XTX): https://rocm.blogs.amd.com/artificial-intelligence/rocm-jax-mujoco/README.html
- AMD, OpenXLA/JAX ROCm CI state (search summary only): https://rocm.blogs.amd.com/software-tools-optimization/openxla-jax-rocm/README.html
- MuJoCo MJX docs: https://mujoco.readthedocs.io/en/latest/mjx.html
- MuJoCo Warp docs: https://mujoco.readthedocs.io/en/latest/mjwarp/index.html ; repo: https://github.com/google-deepmind/mujoco_warp
- MuJoCo changelog (3.15.0, 2026-10-05): https://mujoco.readthedocs.io/en/latest/changelog.html
- MuJoCo Python bindings, rollout: https://mujoco.readthedocs.io/en/latest/python.html
- NVIDIA Warp: https://github.com/NVIDIA/warp ; AMD PR https://github.com/NVIDIA/warp/pull/1932 ; issue search https://github.com/NVIDIA/warp/issues?q=ROCm+OR+AMD+OR+HIP
- Newton: https://github.com/newton-physics/newton
- Genesis: https://genesis-world.readthedocs.io/en/latest/_sources/user_guide/overview/what_is_genesis.md.txt ; LICENSE https://raw.githubusercontent.com/Genesis-Embodied-AI/Genesis/main/LICENSE
- Madrona: https://github.com/shacklettbp/madrona
- Brax: https://github.com/google/brax
- Isaac Lab licence: https://isaac-sim.github.io/IsaacLab/main/source/refs/license.html ; Isaac Sim requirements: https://docs.isaacsim.omniverse.nvidia.com/latest/installation/requirements.html
- OpenXLA GPU determinism: https://openxla.org/xla/determinism
- Secondary (unverified speedup claims): https://pub.towardsai.net/isaac-sim-vs-mujoco-the-4-000-question-that-will-define-robotics-in-2025-4c41a2984c2c

## Appendix: things I could not confirm (do not treat as facts)

- Exact ROCm and JAX version pairing for gfx1201 beyond the install page's lists; ROCm version on hfbox.
- Any MJX or MJWarp benchmark on gfx1201; any public statement of Warp AMD support timeline.
- Whether MJX exposes `mj_rnePostConstraint`-equivalent specific force, and ray/Jacobian/solve-M helper coverage for every geom type napl uses.
- Whether MJWarp lets per-world `body_mass/inertia` change at run time the way `prepare_motors` needs.
- Genesis current version and rigid-body feature list (README fetch returned 503); PhysX 5 licence; AWS instance prices and families.
- napl per-run time split (physics vs Python) and single-core steps/s: no measurement was made.
- The "70-313x on RTX 4090" figures: secondary blog, baseline unknown.
