# Unified Robotics & Defense Simulator — Landscape Research

Oct 6, 2026 · @SD

No open or commercial tool today simulates air, sea, ground, legged, arm and effector systems together at engineering fidelity, across all three desktop OSes, with agent control built in. The pieces exist as open source; the integration, the effectors layer and the agent-native design do not.

## TL;DR

- **Foundation stack:** C++20 kernel; MuJoCo (articulated contact) and Jolt (fast, deterministic, cross-vendor GPU) as solvers; JSBSim for fixed-wing; Chrono::Vehicle for wheeled and tracked ground vehicles; Fossen-model hydrodynamics as in Stonefish and HoloOcean.
- **Graphics:** Vulkan with Slang shaders, running on macOS through KosmicKrisp or MoltenVK. Not OpenGL: macOS OpenGL stops at 4.1 with no compute shaders, which blocks GPU sensors.
- **Interop:** ROS 2 simulation\_interfaces, MAVLink SITL/HITL, OpenUSD + URDF/MJCF import, FMI 3.0 FMUs, DIS/HLA for defense exercises.
- **Agent-native:** one command bus behind every client, a queryable scene graph, snapshot-and-fork time control, declarative pass/fail tests, exposed over MCP (2026-07-28 spec, with Tasks and Apps).
- **Differentiator:** effectors and counter-UAS (lasers, RF jamming, interceptors, turrets) have no open physics-grade simulator.
- **Main risks:** scope and accuracy claims. Start with multirotors and swarms, report fidelity limits per model, and calibrate against real test logs.

## 1. General-purpose robotics simulators

No existing simulator covers air, sea, ground, legged, arms and effectors in one cross-platform C++ package. Each is strong in one band and weak in others.

| Simulator | License | Core | OS | Physics | Strongest at | Gap for our goals |
| --- | --- | --- | --- | --- | --- | --- |
| [Gazebo Jetty](https://discourse.openrobotics.org/t/gazebo-jetty-released/50349) (LTS to Sep 2030) | Apache 2.0 | C++ | Ubuntu primary; macOS, Windows, ARM best-effort | DART, Bullet (plugin system) | ROS 2 integration, sensor plugins, PX4/ArduPilot SITL | Slow with many agents; camera fidelity weak; non-Linux is second-class |
| [Isaac Sim 5.1–6.1](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/common/licenses-isaac-sim.html) + Isaac Lab 3 | Apache 2.0 code; Omniverse Kit, PhysX/RTX assets proprietary | C++ / Python on Omniverse Kit | Linux, Windows (NVIDIA RTX GPU required) | PhysX 5, Newton | Photoreal RTX sensors, synthetic data, massively parallel RL | No macOS, no AMD/Apple GPUs, heavy, vendor-locked runtime |
| [MuJoCo 3.7](https://mujoco.readthedocs.io/en/3.7.0/changelog.html) | Apache 2.0 | C (C++ API), Python | Linux, macOS, Windows | Own solver; MJX (JAX), MuJoCo Warp (GPU) | Contact accuracy, legged/manipulation RL, DC motor and actuator models | Weak sensors and rendering; no aero, hydro or terrain domains |
| [Genesis World 1.0](https://app.dealroom.co/news/note/genesis-ai-open-sources-world-1-0-simulation-platform-for-robotics) (May 2026) | Open source (Apache 2.0 historically) | Python on GPU kernels (Quadrants) | Linux, macOS (Metal), Windows | Rigid, IPC deformable, SPH, MPM | Multi-physics in one API, very high throughput | Python-first; manipulation/humanoid focus; little aero/marine |
| Webots R2025a | Apache 2.0 | C/C++ | Linux, macOS, Windows | ODE fork | Easy setup, education, wide robot library | ODE accuracy limits; dated renderer; small multi-agent scale |
| [CoppeliaSim 4.10](https://en.wikipedia.org/wiki/CoppeliaSim) | GPL or commercial | C++ + Lua/Python | Linux, macOS, Windows | MuJoCo, Bullet, ODE, Vortex, Newton GD | Swappable physics engines, scripting, remote API over ZeroMQ | GPL/commercial split; aging UI |
| O3DE + ROS 2 Gem | Apache 2.0 / MIT | C++ | Linux, Windows (macOS partial) | PhysX 5 | Full game engine (Vulkan/DX12/Metal), ROS 2 sensors | Small community; robotics is a side use case |
| Unity / Unreal (+ [Project AirSim](https://microsoft.github.io/AirSim/), CARLA, HoloOcean) | Proprietary engine; plugins MIT | C# / C++ | All three desktops | PhysX / Chaos | Visuals, large environments | Engine license; game physics, not engineering physics |

Takeaways for us:

- **MuJoCo is the accuracy benchmark** for rigid-body contact and actuators; Isaac Sim is the benchmark for sensor realism and GPU scale.
- **CoppeliaSim's swappable-engine design** is the closest model to an "all-in-one": one scene format, several solvers behind it.
- **Gazebo Jetty's "ROS Standard Sim Interface"** is worth adopting so existing ROS stacks can drive our simulator unchanged.
- **macOS is the weak spot everywhere** except MuJoCo, Webots, CoppeliaSim and Genesis; Isaac Sim does not run there at all.

## 2. Physics engines to build on

Do not write a rigid-body contact solver from scratch: embed MuJoCo (accuracy) and Jolt (cross-platform speed) behind one interface, and spend our own effort on the domain models no engine ships (aero, hydro, terrain, propulsion, effectors).

| Engine | License | Language | GPU path | Accuracy / notes | Role in our stack |
| --- | --- | --- | --- | --- | --- |
| [MuJoCo 3.7](https://mujoco.readthedocs.io/en/3.7.0/changelog.html) | Apache 2.0 | C, C++ API | MJX (JAX), MuJoCo Warp (CUDA) | Best-in-class soft-contact, tendons, DC motor models, mjSpec procedural models, OpenUSD import | Default solver for arms, legged robots, grippers, turrets |
| [Newton 1.0](https://dataconomy.com/2026/03/17/nvidia-launches-newton-1-0-physics-engine-for-industrial-robot-training/) (Mar 2026) | Apache 2.0, Linux Foundation | Python on NVIDIA Warp | CUDA | Multi-solver: MuJoCo Warp, Kamino, VBD for cables/cloth; SDF and hydroelastic contact | Watch / borrow designs; CUDA-only, so not a portable core |
| [PhysX 5](https://www.phoronix.com/news/NVIDIA-OSS-PhysX-Flow-GPU) | BSD-3 (GPU kernels open since Apr 2025) | C++ | 500+ CUDA kernels | Game-grade rigid, articulations, FEM soft bodies, PBD fluids | Option for large rigid scenes on NVIDIA hardware |
| [Jolt Physics 5.6](https://gamedev.net/news/4751-jolt-physics-v560-released/) (Jul 2026) | MIT | C++17 | Vulkan / DX12 / Metal compute shaders | Fast, multithreaded, deterministic, double-precision build option | Broad-phase, vehicles, debris, large worlds on any GPU vendor |
| Drake (TRI) | BSD-3 | C++ | CPU | Hydroelastic contact, multibody + systems framework, optimization/MPC | Reference for validation and controls design |
| [Project Chrono 9](https://api.chrono.projectchrono.org/manual_vehicle.html) | BSD-3 | C++ | Partial (Chrono::GPU, FSI) | Wheeled and tracked vehicle templates, tire and deformable-terrain models (SCM, granular, SPH) | Ground vehicles, tanks, rovers on soft soil |
| DART | BSD-2 | C++ | CPU | Featherstone articulated bodies, used by Gazebo | Alternative articulated solver |
| Bullet 3 / ODE | zlib / BSD-LGPL | C++ / C | Limited | Mature but older contact models | Legacy compatibility only |

Design implications:

- **Fixed-step, deterministic core.** Jolt and MuJoCo both support deterministic stepping; we need it for replay, regression tests and agent reproducibility.
- **Double precision for world coordinates.** Aircraft and drone-swarm scenarios span tens of kilometres; float32 jitter at that scale breaks sensors and contact.
- **Avoid CUDA lock-in.** Newton, PhysX GPU and MuJoCo Warp are NVIDIA-only. A Vulkan-compute (or Jolt-style multi-backend) path is required to run on macOS and AMD.

## 3. Domain-specific simulators

The real fidelity in each vehicle class lives in small, specialised open-source projects, not in the big platforms. Our simulator should wrap or re-implement these models as plugins rather than compete with them.

| Domain | Projects worth studying | What to borrow |
| --- | --- | --- |
| Fixed-wing, gliders, planes | [JSBSim](https://awesome.ecosyste.ms/projects/github.com%2Fjsbsim-team%2Fjsbsim) (C++, LGPL 2.1; NASA-checked; used by ArduPilot, PX4, FlightGear, Unreal plugin), FlightGear (GPL) | Table-driven aero coefficients in XML, atmosphere and propulsion models; embed JSBSim directly as our fixed-wing FDM |
| Multirotor drones | PX4 and ArduPilot SITL, [Aerial Gym](https://arxiv.org/html/2503.01471v1) (Isaac Gym, GPU ray-cast sensors via Warp, motor time-constant models), Pegasus (Isaac Sim + PX4), Flightmare, RotorPy, gym-pybullet-drones | Run the real autopilot firmware in the loop (SITL/HITL over MAVLink); per-motor thrust/torque lag; IMU bias random walk |
| Swarms and multi-agent | [SCRIMMAGE](https://ardupilot.org/dev/docs/sitl-with-scrimmage.html) (GTRI; hundreds of low-fidelity or tens of high-fidelity aircraft on a laptop; ArduPilot link unmaintained as of Apr 2026), VMAS (vectorised multi-agent), Crazyswarm/CrazySim | Fidelity-switchable motion models per agent so 1,000-agent runs stay real-time |
| Counter-UAS and engagement | [MIXR / OpenEaagles](https://openeaagles.org) (C++, LGPL; radar, IR, weapons, DIS networking, JSBSim); AFSIM (US government, not public) | Sensor-to-shooter chain: detect, track, classify, assign, engage; DIS/HLA interop with defense sims |
| Underwater (AUV/ROV) | [Stonefish](https://arxiv.org/pdf/2502.11887) (C++, own OpenGL renderer, per-triangle hydrodynamics, sonar, DVL, USBL, acoustic modems); [HoloOcean 2.0](https://arxiv.org/html/2510.06160v1) (UE 5.3, Fossen dynamics, sonar, ROS 2); DAVE and VRX (Gazebo) | Fossen 6-DOF marine equations; geometry-based buoyancy and drag; ray-cast sonar |
| Ground: wheeled, tracked, rovers | [Project Chrono::Vehicle](https://api.chrono.projectchrono.org/manual_vehicle.html) (wheeled + tracked templates, tyre and deformable-terrain models), CARLA (UE driving) | Template vehicles (suspension, driveline, track shoes, sprockets); SCM soft-soil terrain for rovers and tanks |
| Legged robots | MuJoCo + Menagerie, Isaac Lab, Genesis | Actuator and contact models validated by sim-to-real legged RL |
| Robotic arms, turrets | MuJoCo, Drake, ManiSkill3 / SAPIEN | Joint friction, backlash, gear ratios, motor current limits; turret = arm with a ballistics/effector payload |
| Directed energy (lasers) | No mature open-source engagement simulator found. Building blocks (approximate, from general knowledge): wave-optics libraries such as LightPipes, PROPER, HCIPy; AFIT scaling-law models for thermal blooming and turbulence are published but not open code | Scaling-law beam propagation (fast) plus target heating model; full wave-optics only for offline validation |
| RF, radar, comms | [Sionna RT](https://arxiv.org/abs/2303.11103v2) (NVIDIA, differentiable radio ray tracing), openEMS (FDTD) | Ray-traced radar returns and link budgets; jamming and GPS-denial scenarios |

The hardest gap is the **effectors and counter-drone layer**: lasers, RF jamming, interceptors and kinetic turrets have no open, physics-faithful simulator. That is where our project can be clearly differentiated.

## 4. Rendering and cross-platform graphics

Use Vulkan, not OpenGL. OpenGL on macOS is frozen at 4.1 with no compute shaders, which rules out GPU lidar, sensor ray tracing and GPU physics there. Vulkan now reaches macOS two ways: MoltenVK, and LunarG's [KosmicKrisp](https://www.phoronix.com/news/KosmicKrisp-Vulkan-1.4) Mesa driver, which passes the Vulkan conformance suite and exposes Vulkan 1.4 as of July 2026.

| Option | License | Backends | Fit for us |
| --- | --- | --- | --- |
| Raw Vulkan 1.3/1.4 + MoltenVK or KosmicKrisp | Open (Khronos / Apache / MIT) | Vulkan everywhere; Metal via translation on Apple | Best control and performance; most engineering work. Recommended core |
| SDL3 GPU API | zlib | Vulkan, Metal, D3D12 | Thin, modern abstraction; good if we want native Metal without translation |
| WebGPU via Dawn (C++) or wgpu-native | BSD-3 / MIT-Apache | Vulkan, Metal, D3D12, browser | Same code can ship a browser viewer; ray tracing not in the spec yet |
| bgfx, Diligent Engine, The Forge | BSD-2 / Apache 2.0 | All major APIs | Proven abstraction layers; less control over compute and RT |
| Filament (Google) | Apache 2.0 | Vulkan, Metal, OpenGL | Ready PBR renderer for the visual layer, not for sensors |
| Slang shading language | Apache 2.0 (Khronos) | Compiles to SPIR-V, MSL, HLSL, CUDA | Write sensor and physics kernels once, target every GPU vendor |

Sensor simulation is where rendering earns its keep, and each sensor needs its own physical model:

- **Cameras:** physically based shading plus lens distortion, rolling shutter, motion blur, exposure and noise. Fisheye and event cameras as variants.
- **Lidar, depth, ToF:** GPU ray casting against a BVH (Vulkan ray query where supported, compute-shader BVH elsewhere). Aerial Gym does this on Warp; we would do it in Vulkan compute.
- **Radar, RF, sonar:** ray-traced returns with material reflectivity, multipath and Doppler; Sionna RT and Stonefish show the approach.
- **Thermal / IR:** temperature per material plus emissivity, needed for counter-UAS and laser scenarios.
- **Real-world scenes:** Gaussian-splat reconstructions (NVIDIA's [3DGUT](https://developer.nvidia.com/blog/how-to-instantly-render-real-world-scenes-in-interactive-simulation), open on GitHub) let users test against a scan of their actual test site.
- **Headless mode:** offscreen Vulkan rendering with no window, so CI jobs and agents can run thousands of scenarios on servers.

## 5. Interoperability standards to adopt

Speaking existing standards on day one is what lets teams bring their real robot code, CAD models and autopilots into our simulator unchanged. The list below is the minimum set.

| Layer | Standard | Why it matters |
| --- | --- | --- |
| Sim control API | [ROS 2 simulation\_interfaces](https://index.ros.org/p/simulation_interfaces/) v2.1.0 (Apr 2026, Apache 2.0): spawn/delete entities, get/set state, step, reset, load world, feature discovery | Already implemented by Gazebo, Isaac Sim and O3DE; implementing it makes us a drop-in replacement |
| Robot middleware | ROS 2 over DDS or Zenoh (Gazebo Jetty ships Zenoh transport) | Lets users run their real ROS stacks against the sim |
| Autopilots | MAVLink (PX4, ArduPilot SITL/HITL) | Real flight firmware in the loop for drones, planes, rovers, boats, subs |
| Scene and assets | [OpenUSD Core Spec 1.0](https://digitalproduction.com/2026/01/05/aousd-releases-openusd-core-specification-1-0/) (Jan 2026); physics is explicitly out of that spec, so use the UsdPhysics schemas on top | Industry convergence point (Isaac Sim, Newton, MuJoCo 3.7 all read USD) |
| Robot descriptions | URDF, SDF, MJCF, glTF | Import what users already have; MJCF for actuator detail |
| Component models | [FMI 3.0](https://fmi-standard.org/assets/FMI_3.0_Press_Release.pdf) FMUs (170+ supporting tools, incl. Modelica, Simulink) | Plug in a battery, motor, ESC or hydraulic model built in another tool |
| Defense interoperability | DIS (IEEE 1278) and HLA (IEEE 1516), as in MIXR | Join larger exercises and connect to existing defense simulators |
| RL / agents | Gymnasium and PettingZoo APIs | Standard reset/step loop for single- and multi-agent training |

The consistent pattern: big simulators win adoption by importing standard formats, not by inventing new ones. Our native format can be USD-based with our own schemas for aero, hydro and effector parameters.

## 6. Agent-native interfaces

Every agent interface to a simulator today is a bolt-on: a community MCP server wrapping an existing Python or ROS API. None was designed in from the start, so this is a real opening for us.

| Existing interface | How it works | Lesson |
| --- | --- | --- |
| [ros-mcp-server](https://github.com/robotmcp/ros-mcp-server) (Apache 2.0, \~1.5k stars) | Talks to any ROS 1/2 system through rosbridge: topics, services, actions, parameters, message-type discovery | Generic middleware access works without touching robot code, but the agent sees raw messages, not meaning |
| [isaac-sim-mcp](https://enterprisedna.co/directories/mcp/omni-mcp-isaac-sim-mcp/) (MIT) | stdio MCP server driving an Isaac Sim extension: scene building, robot control, synthetic data | Natural-language scene construction is the most-used feature |
| [mujoco-mcp](https://github.com/sandraschi/mujoco-mcp) | 20 tools: load model, step, get state, apply control, export frame, record trajectory, population sweeps, train PPO/SAC; one subprocess per job for crash isolation | Parameter sweeps and pass/fail analysis are what agents actually need |
| Gymnasium / PettingZoo | reset/step/observation/reward loop | Standard for RL agents; too low-level for LLM agents on its own |

The [MCP 2026-07-28 spec](https://aaif.io/blog/mcp-2026-07-28-whats-changing-and-how-to-migrate) suits a simulator well: stateless Streamable HTTP for remote sim farms, a Tasks extension for long runs (start, poll, update mid-flight, cancel), and MCP Apps for an interactive viewer rendered inside the agent's chat.

Design principles for an agent-native simulator:

1. **One command bus for everything.** Every GUI action is a typed, serialisable command. The GUI, Python bindings, CLI, MCP server and replay log are all clients of the same bus, so an agent can do anything a human can.
2. **The world is queryable as data.** Agents read a typed scene graph by path (USD-style prim paths) with units and frames, not screenshots. Images stay available for vision checks.
3. **Time is a resource.** Pause, step N ticks, run faster than real time, snapshot and fork. Forking lets an agent try 50 controller gains from the same state.
4. **Tests are first-class.** A declarative requirement ("hold altitude within 0.2 m in 15 kt gusts for 60 s") compiles to a monitor that returns pass/fail plus evidence: plots, logs, the frame where it failed. This is the core of "verify before buying hardware".
5. **Batch and headless by default.** Monte Carlo and parameter sweeps across cores or machines, returned as summarised metrics, not gigabytes of logs.
6. **Three API tiers:** MCP for LLM agents; gRPC or shared memory for high-rate control loops; Python bindings (nanobind) and Gymnasium for RL.
7. **Sandboxed and permissioned.** Agent-authored scripts run with resource limits; destructive actions (overwrite model, delete run) need confirmation.

## 7. Commercial and defense platforms for ideas

The paid and government tools split into mission-level wargaming (wide, low physics fidelity) and engineering simulation (deep, narrow). Nothing combines both with agent access.

| Platform | What it is | Idea to take |
| --- | --- | --- |
| [AFSIM](https://csiac.dtic.mil/articles/afsim-the-air-force-research-laboratorys-approach-to-making-ms-ubiquitous-in-the-weapon-system-concept-development-process/) (AFRL) | C++ multi-domain framework: land, sea, air, space and cyber platforms built from sensors, weapons, comms and processor components; 275+ approved organisations, not public | Component-based platform definition and selectable fidelity, from a point mass to full 6-DOF, per entity |
| [Applied Intuition Drone Stack](https://thedefensepost.com/2026/07/22/applied-intuition-autonomy-software/) (Jul 2026) | Platform-agnostic ISR autonomy with SIL, HIL and a flight-test facility | One autonomy stack across many airframes; sim tightly coupled to HIL and flight test |
| [Duality AI Falcon](https://workos.com/blog/duality-ai-building-reality-grade-digital-twins-for-ai-and-robotics) | Unreal-based digital twins and synthetic data; US Army contract for drone-detection AI; FalconCloud runs in the browser | Synthetic data for counter-UAS perception; cloud sims for users without a GPU |
| MATLAB Simulink / Simscape / UAV Toolbox (general knowledge) | Block-diagram multi-physics, code generation, FMU export | Electrical, hydraulic and thermal subsystem models; auto-generated firmware |
| Ansys, Bohemia VBS4 (general knowledge) | High-fidelity CFD/FEA and EM; large-scale military training worlds | Use CFD offline to fit fast aero tables; whole-earth terrain streaming for large scenarios |

Where we can stand apart: open core, engineering-grade physics across all domains, effectors and counter-UAS included, runs on a laptop on any OS, and an agent can drive the whole thing.

## 8. Gaps and proposed architecture

Build a thin, agent-native C++ kernel and wrap proven open-source domain models as plugins; spend original engineering on effectors, counter-UAS and the test/verification layer, where nothing open exists.

&#91;embedded content: proposed architecture · 4 layers\]

The command bus is the agent-native piece: the GUI, Python, MCP and ROS bridges are all clients of it, so anything a person can do, an agent can do.

Gaps no current product fills:

- **One tool across domains.** Air, sea, ground, legged, arms and effectors in one scene, at engineering fidelity.
- **Effectors and counter-UAS.** Lasers, RF jamming, interceptors and turrets have no open physics-grade simulator.
- **True macOS + Linux + Windows parity** with GPU sensors; Isaac Sim and Newton are NVIDIA-only.
- **Requirements-driven verification.** Users want "will this work?", answered with pass/fail and evidence, not a sandbox.
- **Agent-native control** designed in, not wrapped on afterwards.

Critical caveats:

- **"Incredible accuracy for everything" is not achievable, and claiming it would mislead users.** Accuracy comes from validated models plus the user's real parameters (motor curves, mass properties, aero data). The tool should report fidelity level and known error bounds per model, and support fitting parameters to real test logs.
- **Scope is the main risk.** Isaac Sim and Gazebo took large teams many years. A credible v1 is one domain done well, e.g. multirotors and swarms with PX4/ArduPilot SITL, lidar/camera sensors, and the MCP/test layer. Then add ground (Chrono), arms and legs (MuJoCo), marine (Fossen) and effectors.
- **Licenses shape the product.** MuJoCo, Jolt, Chrono and Drake are permissive. JSBSim (LGPL) and MIXR (LGPL) need dynamic linking; Stonefish and FlightGear are GPL and should be used as references only if we want a closed or commercial edition.
- **Export control.** Directed-energy and counter-UAS models can fall under ITAR/EAR. Keep the open core generic and get advice before publishing weapon-effect models.

## Sources

Pages opened for this research (Oct 2026). Rows marked "general knowledge" in the tables were not checked against a source.

- [Gazebo Jetty release announcement](https://discourse.openrobotics.org/t/gazebo-jetty-released/50349)
- [Isaac Sim licensing (5.1)](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/common/licenses-isaac-sim.html)
- [MuJoCo 3.7.0 changelog](https://mujoco.readthedocs.io/en/3.7.0/changelog.html)
- [Newton 1.0 launch (Dataconomy)](https://dataconomy.com/2026/03/17/nvidia-launches-newton-1-0-physics-engine-for-industrial-robot-training/)
- [Genesis World 1.0 (Dealroom)](https://app.dealroom.co/news/note/genesis-ai-open-sources-world-1-0-simulation-platform-for-robotics)
- [AirSim / Project AirSim status](https://microsoft.github.io/AirSim/)
- [CoppeliaSim (Wikipedia)](https://en.wikipedia.org/wiki/CoppeliaSim)
- [Black Coffee Robotics: robot simulation software, 2026 perspective](https://www.blackcoffeerobotics.com/blog/which-robot-simulation-software-to-use)
- [PhysX GPU code open-sourced (Phoronix)](https://www.phoronix.com/news/NVIDIA-OSS-PhysX-Flow-GPU)
- [Jolt Physics v5.6.0 (GameDev.net)](https://gamedev.net/news/4751-jolt-physics-v560-released/)
- [Chrono::Vehicle manual](https://api.chrono.projectchrono.org/manual_vehicle.html)
- [JSBSim project summary](https://awesome.ecosyste.ms/projects/github.com%2Fjsbsim-team%2Fjsbsim)
- [SCRIMMAGE with ArduPilot SITL](https://ardupilot.org/dev/docs/sitl-with-scrimmage.html)
- [MIXR / OpenEaagles](https://openeaagles.org)
- [Stonefish paper (arXiv 2502.11887)](https://arxiv.org/pdf/2502.11887)
- [A Preview of HoloOcean 2.0 (arXiv 2510.06160)](https://arxiv.org/html/2510.06160v1)
- [Aerial Gym Simulator (arXiv 2503.01471)](https://arxiv.org/html/2503.01471v1)
- [KosmicKrisp exposes Vulkan 1.4 (Phoronix)](https://www.phoronix.com/news/KosmicKrisp-Vulkan-1.4)
- [NVIDIA: rendering real-world scenes in simulation (3DGUT)](https://developer.nvidia.com/blog/how-to-instantly-render-real-world-scenes-in-interactive-simulation)
- [ROS 2 simulation\_interfaces](https://index.ros.org/p/simulation_interfaces/)
- [OpenUSD Core Specification 1.0 (Digital Production)](https://digitalproduction.com/2026/01/05/aousd-releases-openusd-core-specification-1-0/)
- [FMI 3.0 press release](https://fmi-standard.org/assets/FMI_3.0_Press_Release.pdf)
- [ros-mcp-server](https://github.com/robotmcp/ros-mcp-server)
- [isaac-sim-mcp listing](https://enterprisedna.co/directories/mcp/omni-mcp-isaac-sim-mcp/)
- [mujoco-mcp](https://github.com/sandraschi/mujoco-mcp)
- [MCP 2026-07-28 spec changes (AAIF)](https://aaif.io/blog/mcp-2026-07-28-whats-changing-and-how-to-migrate)
- [AFSIM overview (CSIAC)](https://csiac.dtic.mil/articles/afsim-the-air-force-research-laboratorys-approach-to-making-ms-ubiquitous-in-the-weapon-system-concept-development-process/)
- [Applied Intuition drone autonomy (The Defense Post)](https://thedefensepost.com/2026/07/22/applied-intuition-autonomy-software/)
- [Duality AI Falcon (WorkOS)](https://workos.com/blog/duality-ai-building-reality-grade-digital-twins-for-ai-and-robotics)
