# Assemblies

`assemblies/<name>.toml` describes a robot as instances of library parts (`parts/`), mates between their ports, fasteners,
wires and device bindings. `napl design build assemblies/<name>.toml` turns it into `build/assemblies/<name>.xml` (MJCF with
a `napl.devices` block) and `<name>.report.json` (mass budget, BOM, provenance, warnings). Python: `napl.design`
(stdlib + numpy only; no C++ kernel needed to build, only to simulate the result).

```
napl design build assemblies/x500v2.toml [--parts DIR] [--out DIR]   # default parts: <repo>/parts, out: build/assemblies
napl design info  assemblies/x500v2.toml [--parts DIR]               # mass budget table, writes nothing
napl design check assemblies/x500v2.toml [--parts DIR] [--json]      # design-rule check, see "Design rules"; exit 1 on an error
```
Exit code 2 on any assembly error, with a message naming the key, mate or part.

## Schema

```toml
[assembly]
name = "x500v2"            # output file stem and <mujoco model=...>
description = "..."
base = "free"              # free: root gets a freejoint, placed so the lowest collision point is at z = 0
                           # fixed: root welded to the world at the origin
timestep = 0.001           # s (default 0.001); sim settings are the only bare numbers besides the ones listed below
seed = 3                   # -> napl.devices "seed"
mesh_inertia = "exact"     # MJCF <mesh inertia=...>: exact (default) | convex | legacy | shell
root_frame = "urdf_link"   # optional, needs base = "fixed": world = the root part's URDF link frame (root mesh placed at its
                           # [geometry] origin_xyz / origin_rpy, URDF fixed-axis rpy), instead of the root's mesh frame

[[instances]]
id = "frame"               # [A-Za-z0-9_-]+, unique; becomes the MJCF body name
part = "holybro-x500v2-frame-kit"
root = true                # exactly one; never offboard
group = "frame"            # optional, for mass budgets / targets
offboard = false           # true: no body, no geometry, not in mass totals; still usable in wires / devices (bench PSU)
params = { mass = { value = 1.0, unit = "kg", fidelity = "estimated", rationale = "..." } }   # overrides: provenance tables only
pos = [0.10, 0.0, 0.05]    # optional, FREE placement: position (m) in the ROOT's frame, for an instance held by no mate (see "Build UI")
rpy = [0.0, 0.0, 1.5708]   # optional: orientation (rad), URDF fixed-axis roll-pitch-yaw; the root cannot have pos / rpy
color = [0.9, 0.2, 0.1, 1.0]   # optional: cosmetic rgb(a) override, 0..1 (3 or 4 numbers); applied to every geom of the instance at build

[[mates]]
name = "arm1--frame"       # default "<a>--<b>" (strings as written); also the MJCF joint name
a = "frame.arm_port1"      # "inst.port", or just "inst" (see below). a = parent, b = child
b = "arm1.root"
rotation = 0               # int, 0 <= k < gcd(symmetry_a, symmetry_b)
fasteners = "auto"         # or { part = "iso4762-m3x12", count = 4, nut = "iso4032-m3-nut" }
nut = "iso4032-m3-nut"     # optional; also accepted inside the fasteners table
joint = { type = "hinge", range = {...}, reverse = false, spin = false }   # hinge | slide | free; range = a provenance
                           # table, a list of two references, or one "<inst>.<param>" reference whose value is [lo, hi]
closure = false            # true: loop closure, checked against the tree, contributes fasteners only

[[wires]]
from = "pack.out"          # "inst.port" (power or signal ports, same kind at both ends)
to = "pdb.in"
part = "awg-10-silicone-wire"   # a wire-category part, or a connector part for a directly mated pair
length = "pack.discharge_wire_length"    # reference string or provenance table (m)
conductors = 2             # int, default 1 (2 = round trip)
included_in = "pack"       # optional: mass already inside that instance's datasheet mass (electrical only)
connectors = { from = "amass-xt60", to = "amass-xt60" }   # optional: pin the implied mating connector per end; an
                           # unpinned end with several fitting library connectors warns CONNECTOR_AMBIGUOUS

[device_maps.motor_esc]    # dotted device-JSON field -> "<param>" (of the binding's instance)
"motor.kv" = "motor:kv"    #                           or "<field>:<param>" (of the instance named by the binding's <field>)
"motor.r20" = "motor:r_winding"
rotor_inertia = "@table.col"   # "@<table>.<column>" (also "<field>:@..."): a part table column as a list

[[devices]]
type = "esc"               # battery psu wire load esc rotor servo imu baro mag gnss environment body_drag
name = "esc1"              # unique over all devices
instance = "esc1"          # the part whose params feed `map`
map = "motor_esc"
motor = "m1"               # role field: an instance, usable as "<field>:<param>" in maps; not emitted
port = "power_in"          # node devices (battery psu load esc servo): the power port giving the electrical node
joint = "m1--prop1"        # a mate name -> the generated joint name (also backlash_joint)
body = "m1"                # an instance -> its body name (also ref_body)
site = "fc"                # an instance -> an auto-generated site at its origin ("site:<inst>"),
site_port = "imu"          #   or at that port's pose ("site:<inst>:<port>")
a = "bat"                  # type = "wire" (internal resistive edge): two ports of `instance`, or "ground"
b = "out"
ocv = "lipo-cell-ocv-chen2006"   # battery: OCV table part (table ocv_vs_soc, columns soc, ocv_V)
cells = "cells_series"     # battery: series cell count param of `instance` (default cells_series); pack OCV = cell OCV x cells
set = { i_limit = { value = 40.0, unit = "A", fidelity = "estimated", rationale = "..." } }   # values in no part
reynolds = true            # rotor: add the pooled UIUC Reynolds factor tables (re_x/re_ft/re_fq) and re-reference ct/cq to x = 4.5 m^2/s
                           #   (needs the prop's ct_static, cq_static, [tables.ct_cq_static_derived])
dir = 1                    # flags (complementary, ground_effect, enabled) and `dir` pass through as given

[[targets]]
name = "takeoff mass"
groups = ["frame", "power"]            # or all = true
value = { value = 1.76, unit = "kg", src = "S1", quote = "...", fidelity = "datasheet" }

[[targets]]                            # an arm's reach: points the end effector must reach (DRC `ARM_REACH`); no value / groups / all
name = "pick zone"
reach = [{ value = [0.20, -0.05, 0.01], unit = "m", fidelity = "estimated", rationale = "cube" }]   # one table per point, [x, y, z] in the world frame
workspace = { value = [0.1, -0.1, 0.0, 0.3, 0.1, 0.2], unit = "m", fidelity = "estimated", rationale = "..." }   # or a box [x0, y0, z0, x1, y1, z1] (3 x 3 x 3 grid)
```
A servo's bus ID and baud are instance params set with `set` (provenance tables; no part file carries them): `servo_id` (default: unset =
factory ID) and `baud` (default: the part's `baud_default`).

### Numbers rule (enforced by the loader)

Every numeric leaf is either a reference string `"<inst>.<param>"` (resolved from that instance's part params, with
overrides) or a provenance table `{value, unit, fidelity, + src & quote (datasheet/literature/measured) | derivation
(derived) | rationale (estimated)}`; `value` may be a list of numbers (joint `range`, device arrays). A bare number is a load
error naming the key (`bare number at 'wires.0.length'`). Bare numbers are allowed only in `[assembly]` and for the keys
`rotation`, `count`, `conductors`, `dir`. Every `estimated` value actually used lands in the report's `estimated` list.
Units are checked where they matter: lengths m, targets kg, hinge ranges rad (deg converts), slide ranges m, and every device
field in the builder's expected-unit table (`kv` rpm/V; `r20 r0 r1 resistance r_series` ohm; `i0 i_limit` A;
`rotor_inertia` kg*m^2; `c_th` J/K; `r_th` K/W; `c1` F; `capacity_ah` Ah (mAh and C convert, only because the unit says so);
`voltage` V; `diameter length_m` m; `gear_ratio efficiency` 1; `encoder_cpr` 1 or counts/rev). A wrong unit is an error; a
field without a table entry is accepted with an `UNIT_UNCHECKED` warning.

## Snapping

Part frame: right-handed, metres. Port frame P (in the part frame): z' = `axis`, x' = `ref`, y' = z' x x', origin = `pos`;
`ref` must be a unit vector perpendicular to `axis` (1e-6). Compatible ports: identical `type` or a paired family of the
same size (`shaft.<d>` with `bore.<d>`, `tube.<od>` with `tube-clamp.<od>`; one function, `napl.design.library.types_ok`, used
everywhere compatibility is decided), genders male+female or either neutral. `n_mate = gcd(symmetry_a, symmetry_b)`; the mate `rotation` k is in `[0, n_mate)`.

```
T_world_b = T_world_a  P_a  Rz(2 pi k / n_mate)  Rx(pi)  inv(P_b)
```
Mated ports coincide, their axes are opposite, and b's `ref` is a's `ref` turned k * 2 pi / n_mate about a's axis
(right-hand rule). If a mate names no port (`"inst"`), the unique compatible mech port pair of the two parts is used; zero or
several pairs are an error that lists the candidates (name a port to disambiguate).

Kinematic tree: from the root, every non-offboard instance must be reached. A mate between two already-connected instances
is a cycle (error naming the loop) unless `closure = true`: then its transform is checked against the tree (1e-6 m, 1e-6
rad) and it only adds fasteners; a joint on a closure mate is an error. A mate may be written child-first (b is the parent)
unless it has a joint. Each instance is one MJCF body (body frame = part frame) nested in its parent's body; the body pose
relative to the parent is the mated pose.

Joints live in b's body at b's port origin; the axis is a's axis expressed in b (= -b.axis), positive by the right-hand rule
about a's axis, `reverse = true` negates it; q = 0 is the mated pose. `spin = true`: unlimited hinge (rotor), no range.
`free`: b gets a freejoint at the mated pose. Joints are named by the mate name (root freejoint: `<root>:free`).

## Fastener selection (`fasteners = "auto"` or explicit)

* Thread d: the port type's last dotted segment `m<d>` (`bolt-pattern.16x16.m3`, `bolt-circle.25.4.m3`, `thread.m2.5`;
  decimals joined) else the port's `thread` key; both ports must agree.
* Holes: `holes` (count override, uses the first n positions) else bolt-pattern 4, `bolt-circle.<D>.<n>.<thr>` n, `thread.*` 1.
  Positions in the port frame: `hole_pos` if given; `bolt-pattern.<A>x<B>` -> (+-A/2, +-B/2) mm; `bolt-circle.<D>.<n>` ->
  radius D/2 at angles 2 pi j / n from `ref`; else the origin.
* `stack` = sum of `thickness` of the non-tapped ports.
* Tapped port (`tapped = true`; both tapped is an error): `engagement = 1.5 d`, capped at that port's `thread_depth`; no nut.
  **The 1.5 d factor is a design rule of thumb for tapped holes, not a sourced fact.**
* Through-bolt: nut = the mate's `nut`, else the library nut of that thread with the smallest height (ties by id; height =
  `height`, else `height_max`, else the nut port's `length`); `engagement = nut height + 2 x thread_pitch`
  (**design rule: the screw protrudes two pitches past the nut**). Pitch = `thread_pitch` of the screw, else of the nut. No nut
  or no pitch: refused.
* `L_min = stack + engagement`. auto: the shortest library screw (category `fastener`, male `thread.m<d>` port, `length`
  param) with `length >= L_min - 1e-9` (ties by id); none: error giving thread, L_min and the longest available. Explicit
  `{part, count}`: the thread must match (error); `length < L_min` gives warning `FASTENER_SHORT`.
* Mass model: each screw (and nut) is a steel cylinder geom (contype 0, radius d/2, half-height length/2 resp. height/2),
  mass = the part's `mass` param, centred on the mating plane at each hole position, axis along the mate axis, in b's body.
  The report lists mate, thread, holes, stack, engagement, L_min, part, length, nut, count, mass.

## Mass properties

Instance mass: `params.mass` (any fidelity); else a printed part (mesh geometry with a material part that has `density`, and
params `wall_count`, `infill_fraction`, `nozzle_diameter`): V and A from the binary STL (signed tetrahedron volume, triangle
areas), `t = wall_count x nozzle_diameter`, `V_shell = min(V, A t)`, `f = (V_shell + infill (V - V_shell)) / V`,
`mass = rho V f` (fidelity `derived`, estimated inputs are listed); else the sum of the shapes' `mass_param`s; else error
`no mass for part X`. Multi-shape parts: shapes with `mass_param` get it, the rest share the remainder by shape volume. A part
without geometry carries its mass on a placeholder sphere (radius 1e-3 m, numerical placeholder, no collision). Every geom has an
explicit `mass=`; MuJoCo derives the inertia (mesh assets use `inertia="exact"`: needs watertight, well-oriented STLs).

Uncertainty per item (absolute kg): `params.mass_tol` if present; printed parts: max |mass(wall_count +-1) - mass|
(wall_count >= 1); mass fidelity `estimated` without `mass_tol`: **0.5 x mass (documented default)**; else 0. Fasteners and
wires use the same rules with their own mass fidelity. A target reports `model` (in-scope total), `gap = model - target`,
`gap_pct`, `tolerance` (sum of in-scope item tolerances), `within = |gap| <= tolerance` and `estimated_share` (in-scope
estimated mass / model mass). Item group: instances their own; fasteners the child (b) instance's; wires and connectors the
`from` / owning instance's.

## Wires, connectors, electrical nodes

* Wire part with `awg` -> Wire device `{awg, length_m, conductors}`; else `resistance_per_m x length x conductors`; a
  connector part (non-wire category) needs `resistance` or `contact_resistance`. Port types must be compatible unless the wire
  part's category is `wire`. Signal-port wires carry mass and BOM only (no device).
* Wire mass = (`mass_per_m`, else the sum of the part's `kg/m` params) x length x k; a part without a rate is listed
  under `unresolved`. **Conductor convention:** a wire part WITHOUT a `conductors` param is a single conductor (AWG silicone
  wire): k = the entry's `conductors`. A wire part WITH a `conductors` param is a whole multi-conductor cable (e.g. the 3-pin
  servo cable): its `mass_per_m` is per cable and k = 1. The entry's `conductors` always sets the electrical side only (current
  -carrying conductors: 2 = supply + return, so R = 2 x per-conductor resistance), never the mass of such a cable. It is a contype-0 capsule between the two port positions (instance origins when a port has no `pos`) in
  the `from` instance's body (an offboard `from` uses the `to` body).
* For a wire-category part, each end whose port has a gender implies one mating connector: a library part with a port of that
  type and the opposite gender, restricted to parts of category exactly `connector` (smallest id first), else `unresolved`; a
  library connector without a `mass` is `unresolved` too. If the wire part itself has `mass_per_connector`, each end uses that
  (reported with the wire part as the connector part) and no library lookup is made. Its mass counts (sphere geom
  `wire:<i>:conn:<k>`, k = 0 from / 1 to) unless `included_in` is set or its instance is offboard.
* Nodes: union-find over (instance, power port). Passthrough power ports of one instance are one node, except ports named
  by an internal `type = "wire"` binding. Node 0 = ground (the negative side is implicit); referenced ports get nodes 1.. in
  instance / port declaration order; more than 32 is an error. Each device carries `parts` (sorted unique part ids that
  contributed).

### Wire syntax with pins

```toml
[[wires]]
name = "pi_uart_adapter"
from = "pi.gpio_uart"                  # port-level ends, as before
to = "adapter.bus"
from_pins = ["J8.8", "J8.10", "J8.6"]  # optional, one entry per conductor: CONNECTOR.PIN of the `from` part ...
to_pins = ["UART.1", "UART.2", "UART.3"]  # ... and of the `to` part, same order
part = "awg-26-silicone-wire"
conductors = 3
length = { value = 0.12, unit = "m", fidelity = "estimated", rationale = "..." }

[[wires]]                              # shorthand: one conductor between two pins; folded into port + pin lists on load
name = "tx"
from = "pi.J8.8"
to = "adapter.UART.2"
part = "awg-26-silicone-wire"
length = { value = 0.1, unit = "m", fidelity = "estimated", rationale = "..." }
```

* A wire without pin lists keeps working: it resolves to the pins its two ports list (`pins` in the part's `[[ports]]`) when the counts
  match, otherwise it is drawn at port level (one bundle from the port position).
* Pins change no mass, device or hash: the `Wire` device and the mass rule above are unchanged by `from_pins` / `to_pins`.
* One conductor per pin pair; the 2D canvas draws every pin, and `pinwire` (`{op: "pinwire", a: "pi.J8.8", b: "adapter.UART.2"}`)
  adds one conductor between two pins (a port may carry several of them; `autowire` joins whole ports and refuses a second wire).
* 3D: each wire is a bundle routed around the parts (never through one): it leaves its connector along the connector `axis`, keeps at
  least `CLEAR` (2 mm) from every part outside the lead-out, and bends no tighter than 6 x its outer diameter. The routing is pure
  geometry (`python/napl/design/wireroute.py`), deterministic, and changes neither mass nor dynamics. A wire it cannot route (the
  connector axis runs inside an arm tube, for example) is drawn as the straight chord.

## MJCF conventions

`<compiler angle="radian"/>`, `<option timestep>`, one `<mesh>` asset per part mesh (file path relative to the output file).
Ground: plane geom `floor` (group 0, contype 1, conaffinity 1) at z = 0. Every vehicle geom is in **geom group 4** with
`conaffinity="0"`; part geoms have `contype="1"` (they collide with the floor only), fastener, wire and connector geoms
`contype="0"`. Rotor devices get `ray_groups = 47` (all groups except 4) so ground-effect rays skip the vehicle. Geom names: `<inst>` or
`<inst>:<k>`, `<mate>:screw:<k>`, `<mate>:nut:<k>`, `wire:<i>`. Floats are written with `repr`, quaternions have w >= 0, so
building twice gives byte-identical files (no timestamps, no absolute paths).

## Report (`<name>.report.json`)

`assembly`, `instances` (id, part, group, body, mass, mass_fidelity, mass_tol, mass_note), `fasteners`, `wires`, `connectors`,
`geoms` ({geom name: part id} for every geom except `floor`), `devices` (name, type, parts), `estimated` (where, key, value,
unit, rationale), `unresolved`, `warnings` ({code, message}), `closures`, `mass` (total, by_group, by_instance: sums to total),
`targets`.

## Python API

`napl.design`: `Library(parts_dir)` (`.get`, `.by_category(prefix)`, `.by_port_type`, `.compatible(type, gender)`, `.errors`; a
part file that fails to parse is only an error when an assembly uses it), `load` / `loads` / `save` / `dumps` (a minimal TOML
writer; `load(save(load(x))) == load(x)`), `port_frame`, `mate_transform`, `find_compatible_ports`, `solve_tree` (+ `MateSpec`),
`select_fasteners`, `compute` (in memory), `build`, `format_budget`, `check`, `Finding`; all failures raise `DesignError`.

## Design rules (DRC / ERC)

```
napl design check assemblies/x500v2.toml [--parts DIR] [--json]    # exit 1 if any finding has severity error
```
`napl.design.check(asm, lib=None) -> [Finding]` (`asm` = path or `Assembly`; mutate `asm.doc` in memory to try an edit).
`Finding` = `code` (stable UPPER_SNAKE), `severity` (`error` | `warning` | `info`), `message`, `where` (instance / port / wire /
mate ids), `fix` (a concrete suggestion: a library part id, a fastener id, a mate). `napl design build` runs the same check and
puts the list in the report as `findings`; findings never fail a build. A rule that crashes becomes `DRC_INTERNAL_ERROR`; a
model that cannot be built skips the geometry / mass rules (`CHECKS_SKIPPED`) or is itself a finding (`BUILD_FAILED`).

Everything is computed from part data (ports, params, `thrust_test` and stall tables) and from the built MJCF. A threshold is
either a fact in a part file or one of the named constants of `python/napl/design/drc.py`, listed here as a **design margin**
(a choice, not a standard) or with its source.

**Electrical model.** Nodes: the passthrough power ports of an instance and the two ports of an internal `wire` device binding
are one node; explicit `[[wires]]` join nodes into nets. A net's source is its battery / psu device, else its `role = "source"`
ports. Loads: an `esc` device with a `motor` draws the motor's worst-case current; a `servo` draws its stall current at the bus
voltage. Current flows from the source along a spanning tree (a net with several sources is split by its first). **Worst case
(peak)** = every ESC at full throttle with the largest `current_A` of the motor's `thrust_test` table, scaled by (V_pack_max /
V_table)^2 when the pack is fuller than the table voltage (rpm ~ V, P ~ rpm^3, I = P / V; first-order); every servo stalled.
**Sustained** = hover current (ESCs) or holding current `idle_current + tau_gravity / kt_output_datasheet` (servos). A rating is
exceeded as an **error** when sustained > continuous rating, or peak > the port's `i_burst` (when it declares one); peak above
the continuous rating alone is a **warning** (`i_max` is a continuous rating, full throttle on all motors is transient).
Ports that declare `i_peak` (servo sockets) state the device's own draw, not a through rating, and are not used for chain currents.

| code | severity | what it checks | threshold and its source |
|---|---|---|---|
| `BUILD_FAILED` | error | the assembly does not build and no other rule explains it (unknown part, missing mass, ...) | `napl design build` error text |
| `CHECKS_SKIPPED` | info | geometry / mass / performance rules were skipped because the model cannot be built (an error above says why) | - |
| `DRC_INTERNAL_ERROR` | warning | a rule group crashed (bug); the other rules ran | - |
| `MATE_INVALID` | error | a mate that cannot be formed: unknown / offboard instance, unknown port, non-mech port, port type or gender mismatch (`snap.incompatible`), ambiguous port pair | snapping rules above; fix lists library parts that mate the port |
| `FLOATING_INSTANCE` | error | an on-board instance not connected to the root by any mate (invalid mates still count as edges, they have their own finding) | graph reachability; fix lists free compatible port pairs |
| `PORT_UNMATED_REQUIRED` | error | a mech port whose part file says `required = true` and that no mate uses | declared per port in the part file (no part sets it yet) |
| `FASTENER_THREAD` | error | thread of the two ports differs (`snap.thread_of`), or the explicit fastener is not a screw of that thread | thread tags of the ports; fix = what `fasteners = "auto"` selects |
| `FASTENER_SHORT` | warning | explicit fastener shorter than L_min = stack + engagement | the builder's own rule (1.5 d tapped, nut height + 2 pitches through); fix = shortest library screw that fits |
| `FASTENER_GRIP` | warning | the selected / explicit screw is longer than stack + `thread_depth` of the tapped port (blind hole of known depth): it bottoms out before the head clamps. Too short is `FASTENER_SHORT` (same L_min). Not checked: through-bolt tip protrusion into other parts (tip position not modelled), washers (no washer parts in the library), tapped ports without `thread_depth` | engagement rule of thumb >= 1 x d in steel, 2 x d in aluminium (Nord-Lock, "Thread engagement in a tapped hole", https://www.nord-lock.com/insights/bolting-tips/2015/the-experts-thread-engagement-in-a-tapped-hole/); the builder's L_min uses 1.5 x d. Fix = the L_min .. stack + depth window |
| `FASTENER_UNAVAILABLE` | error | no screw / nut / pitch for the mate, count above the number of holes, unknown fastener part | the builder's `select_fasteners` error text |
| `INTERFERENCE` | error | collision geometry of two instances that are not mated to each other overlaps at rest by more than 1 mm | **1 mm = design margin** (STL / print tolerance). Box-box: separating-axis test. Anything with a mesh, cylinder, sphere or capsule: up to 60 surface points of each shape (mesh vertices, primitive corners / rims / centre) tested inside the other (mesh: ray parity, needs a watertight STL; depth = nearest of the 6 axis rays), so a thin sliver between sample points is missed |
| `PROP_OVERLAP` | error | two rotor discs (prop `diameter`, centre on the spin joint) overlap in the disc plane: centre distance < R1 + R2 and axial gap < blade sweep thickness (prop `geom_thickness`) | geometry from the MJCF |
| `PROP_FRAME_CLASH` | error | a disc (radius D/2, thickness `geom_thickness`, sampled at 3 layers x 4 radii x 16 angles) lies inside the collision box of another instance (not its own prop / motor) | box approximation of other shapes |
| `PROP_CLEARANCE` | warning / error / info | (a) tip-to-tip gap of two discs in the same plane (centre distance - R1 - R2) under 5 % of the larger diameter; (b) another instance's collision geometry within 5 % of the diameter outside a disc tip (ring at 1.05 R, 3 layers x 16 angles, a thin member between samples is missed); (c) free-base vehicles: lowest disc point above the lowest collision point (z = 0 at rest) under 5 % of D (warning, error when below it); (d) info: Cheeseman-Bennett ground-effect thrust gain 1 / (1 - (R / 4 z)^2) - 1 at rest when above 5 % (z < 1.15 R), no number below z / R = 0.6 where the model is invalid | **5 % of D = design margin** (no standard; builders quote 3 to 35 mm). Ground effect: Cheeseman & Bennett 1955 |
| `CG_OFFSET` | warning / error | horizontal offset of the centre of gravity (all geom masses, mesh centroids) from the thrust centre; the least-norm change of the equal-throttle thrusts that zeroes the moment about the CG; the message gives the worst-motor extra differential thrust as % of that motor's hover thrust; warning when it uses > 20 % of the per-motor thrust headroom or the differential exceeds 15 % of hover thrust, error when hover needs a thrust above max or below 0 | **20 % headroom and 15 % differential = design margins** (datum: a 50 mm CG shift on an X quad needs a 16-17 % motor speed difference, E-MASS, https://www.ncbi.nlm.nih.gov/pmc/articles/PMC12736611/; thrust ~ speed^2) |
| `DEVICE_UNSET` | error | a `[[devices]]` binding lacks a field the simulator's device validation needs (`auto_devices.REQUIRED`: e.g. `motor.c_th` / `motor.r_th` on an ESC, `ocv` on a battery) in its own keys, its `set` and its device map, so a run would fail. Typical after a motor swap whose new part carries no thermal numbers. The fix names the part param that would feed it, or the `set` / `auto_devices(fill=...)` entry (a provenance table) that supplies it. |
| `THERMAL` | info | per motor at hover: winding current I_m = pack current / throttle (ideal ESC), copper loss I_m^2 R_winding as % of V I_pack (worst motor reported; notes when above 20 %). Winding temperature T = 25 degC + r_th P_cu(T), R(T) = R20 (1 + 0.00393 (T - 20)), only when the esc device binding sets `motor.r_th`; otherwise no temperature. No part states a winding limit, so never a warning | **20 % = design margin**; 25 degC and 0.00393 /K as in the `r_th` derivations of the assemblies; r_th only from the device binding (the x500 value is a surface-calibrated lower bound) |
| `SERVO_TORQUE_MARGIN` | warning | stall torque at the bus voltage / worst static holding torque (same pose grid as `SERVO_TORQUE`) between 1 and 2; below 1 is the `SERVO_TORQUE` error. The static torque is the arm's own weight, a payload adds to it (`PAYLOAD_LIMIT`; no assembly declares a payload) | **2 = design margin**, top of the 1.5-2 safety factor for vertical gravity loads (Kpower servo selection notes, https://www.kpower.com/insight_servo/7642.html) |
| `WIRE_INVALID` | error | a wire whose end instance / port / part is unknown, or whose ends are not both power or both signal | - |
| `WIRE_CONNECTOR_MISMATCH` | error | connector part wire: ends not compatible (type + gender) or the part has no port of the end's type; cable part: a non-`wire-lead` cable end that does not mate the port | `types_ok` / `genders_ok`; fix lists wire / connector parts that mate the port |
| `WIRE_RATING_UNKNOWN` | info | a wire part with no `i_max` on its ports (awg 10, 14, 16, 18 today): that wire cannot be checked | add `i_max` (AWG ampacity table) to the part |
| `PIN_ALREADY_USED` | error | one connector pin on two conductors (ground pins and pins of a `joinable` connector may be shared) | pin lists of the wires |
| `PIN_KIND_MISMATCH` | error / warning | a conductor joins unlike pins (power to ground, signal to power); a no-connect pin is a warning | `kind` of the pins in the part files |
| `PIN_VOLTAGE_MISMATCH` | error / warning | a driving pin's `v` above the sink's `v_max` (error) or below `v_min` (warning); with no range the two nominal levels differ by more than max(0.5 V, 10 %) (signals: warning, a level shifter is needed) | `v`, `v_min`, `v_max` of the pins; **0.5 V / 10 % = design margin** |
| `PIN_TWO_OUTPUTS` | error | two signal outputs (`dir = "out"`) drive one net | `dir` of the pins |
| `PIN_GND_MISSING` | warning | a multi-conductor DC power / signal wire carries no ground return | topology of the wire's pin pairs |
| `PIN_ALT_CONFLICT` | error | one peripheral function claimed by two pins of an instance (two UART TX of one UART, two SDA of one I2C), or a wire's TX / RX on different peripherals | `alt` functions of the pins (`aN:FUNC`), family keyed (UART / I2C) |
| `PIN_FUNCTION_MISSING` | warning | a multiplexed pin wired to a device pin whose role (TX, RX, SDA, SCL) it cannot provide through its alternate functions | `alt` / `name` of the pins |
| `VOLTAGE_OVER_MAX` | error | net maximum voltage (source port `v_max`, battery full voltage) above `v_max` of a sink / passthrough port on it | `v_max` in the part files; fix lists same-category parts with that port rated high enough |
| `VOLTAGE_BELOW_MIN` | warning | net nominal voltage (`v_nominal` / `v_out_nominal`) below a port's `v_min` | `v_min` in the part files |
| `CURRENT_OVER_RATING` | error | sustained current above a wire's / port's continuous rating, or peak above its `i_burst` | the limiting element is the lowest `i_max` over the wire's two ports and its wire part's ports |
| `CURRENT_PEAK_OVER_RATING` | warning | worst-case (peak) current above the continuous rating | see electrical model |
| `BATTERY_C_RATING` | warning / error | total load of the battery net vs `c_rating_cont x capacity_ah` (burst: `c_rating_burst x capacity_ah`) | datasheet C ratings; same warning / error rule as currents |
| `PSU_OVERLOAD` | warning / error | total load of a supply net vs `i_out_rated` | datasheet; same rule |
| `FLOATING_PORT` | warning / error | an on-board power port (not a source) whose node has no wire; error when its instance has no powered port at all | topology |
| `NET_NO_SOURCE` | error | a wired net with no battery / psu device and no source-role port | topology |
| `THRUST_TO_WEIGHT` | info / warning / error | sum of max thrust of the motor tables, each scaled by (V_pack_nominal / V_row)^2 per table row (thrust ~ rpm^2 ~ V^2), over all-up weight (`report.mass.total` x g) | warning < 2.0: community rule of thumb (ArduPilot forum "Minimum Thrust to Weight Ratio", https://discuss.ardupilot.org/t/minimum-thrust-to-weight-ratio/50980); error < 1.3: **design margin** (hover then needs 77 % of max thrust) |
| `HOVER_THROTTLE` | info | throttle and current per motor at equal thrust fraction f = W / sum(Tmax), interpolated in the thrust table (linear from the origin below the first row; current x (V_pack / V_row)^2) | - |
| `FLIGHT_TIME` | info | capacity x 0.8 / hover current, propulsion only | **0.8 usable = design margin** ("land with 20 % left") |
| `SERVO_TORQUE` | warning / error / info | gravity torque about each driven hinge, maximised over a pose grid (n samples per joint between the joint limits, n^joints <= 20000), subtree masses and centres from the MJCF; error when stall torque at the bus voltage < torque, warning when above the rated (continuous) torque, info (tightest servo) otherwise | stall / rated torque from the servo's `stall_torque_<V>v` / `rated_torque_<V>v` params: linear between table voltages, proportional to V below the lowest, clamped above the highest |
| `PAYLOAD_LIMIT` | info / warning | extra mass at the end-effector point (farthest collision-geometry point of the deepest driven body, from the first joint) such that every servo stays within stall torque at every sampled pose; warning when it is 0 | same as `SERVO_TORQUE` |
| `SERVO_ID_CONFLICT` | error | servos whose power ports are in one net (one half-duplex bus: the 3-pin cable carries supply and signal) declare the same `servo_id`, or an ID outside 0..252 | Feetech STS IDs 0-253, Dynamixel 0-252, 254 = broadcast: the stricter range |
| `SERVO_ID_UNSET` | info | several servos on one bus, some without `servo_id`: new servos all answer to the factory ID (1 on the STS3215) | datasheet / e-Manual |
| `SERVO_BAUD_MISMATCH` | error | servos on one bus with different `baud` (instance override, else the part's `baud_default`) | protocol |
| `SERVO_BUS_CURRENT` | warning / error | sum of the bus servos' stall currents at the bus voltage (`stall_current_<V>v`, as `PSU_OVERLOAD`) vs the bus driver board's (`pcb.servo_driver` on the net) `max_continuous_current`; error when the holding current (idle + torque / kt at the `SERVO_TORQUE` pose) is above it. The supply is `PSU_OVERLOAD`'s | part param (the Waveshare adapter's is an estimate) |
| `SERVO_CHAIN_LENGTH` | warning / error | error: more servos on a bus than the board's `max_servos`. Warning: with all servos stalled, the drop along the wire runs from the source to the farthest servo, sum(current through the wire x `resistance_per_m` x length x conductors), leaves less than the servo's `v_operating_min` (or its min-voltage register default) of the nominal bus voltage; supply output resistance and board resistance are not counted | part params |
| `ARM_REACH` | warning / info | each `reach` point / `workspace` grid point is tried with damped-least-squares IK on the built model (24 seeded starts x 80 steps, joint limits clamped, tip = the farthest collision-geometry point of the deepest driven body, its own joint left out): warning when the closest approach is > `REACH_TOL` 5 mm, with the approach and the fix (too far: a longer link / base closer; too close: base farther); info when all are reached | **5 mm = design margin** (a gripper is not a point) |

Limits of the checks, not hidden: a motor without a `thrust_test` table has no thrust / current model (`HOVER_THROTTLE` info);
avionics current is not in the hover current; the pose grid is a lower bound of the true worst case; interference of meshes is
sampled, not exact; an `estimated` placeholder envelope (e.g. the Waveshare board box) interferes like a real shape.

## Editing API (`napl.design.Design`, MCP `design_*` tools)

```python
d = Design.open("assemblies/x500v2.toml")          # + Library; d.save(path) writes deterministic TOML (comments are not kept)
r = d.swap("mot1", "other-motor")                  # also add / remove / mate / unmate / wire / unwire / set; d.edit([ops]) = atomic batch
r.changes, r.new, r.resolved, r.changed            # what changed + DRC delta (matched on code + where); d.undo() / d.redo()
d.suggest("con1.motor"); d.suggest("mot1")         # ranked parts for an open port / ranked swap candidates (what swap keeps / loses)
d.suggest("tube1.esc", category="esc")             # only that dotted category, filtered before `limit` (a flat seat fits every board)
d = Design.new("quad")                             # an EMPTY design (also MCP design_open {new}); d.add(frame, root=True) first, then snap
                                                   # everything to named open ports (`add ... to="bottom.arm4"`): no poses, no TOML
d = Design.new("arm", base="fixed", root_frame="urdf_link")   # an arm: add(servo, to="base.servo_seat", port="base", k=0), then
                                                   # add(link, to="servo1.output", port="horn_seat", mate="shoulder_pan", joint={"type": "hinge", "range": "link.shoulder_pan_range"})
                                                   # (the whole SO-101 from nothing: tests/python/test_design_scratch_arm.py)
d.target("pick zone", reach=[{...}])               # reach / workspace target (see Schema) -> DRC ARM_REACH
d.suggest("shoulder_lift")                         # a JOINT (a mate with `joint`): servos ranked by stall torque at the bus voltage / the worst static
                                                   # torque of that joint (SERVO_TORQUE's pose grid); rows: margin, stall_Nm, tau_worst_Nm, mates_fit;
                                                   # servos not operable at the bus voltage are left out; the arm keeps its mass (a heavier servo adds)
d.snap("holybro-m8n-gps", to="mast")               # add + mate to the best open compatible port pair; ambiguity -> snap.alternatives
d.check(); d.build(out); d.simulate("hover" | "hold")   # simulate needs the kernel
```
Every edit is validated (numbers rule included) and re-checked; a failure changes nothing. `swap` re-mates by port name, then by
same port type, then by any compatible port; it keeps mate names (device bindings use them), resets fasteners the new thread
cannot take to `auto`, drops instance overrides that shadowed a param of the old part, and lists in `dangling` every reference
the new part cannot satisfy (`'<inst>.<param>'`, device-map params / table columns, device joints of a lost mate); it never
invents a value: supply it with `set` (unit, fidelity, src+quote | derivation | rationale). Suggest / snap scores are named
design weights at the top of `design/edit.py`, not physical data.

**Placement search (`snap`, `add ... to`, `mate` with `k` omitted).** Candidates = open compatible port pairs (best 10 by port
score) x rotations k (all of `0..n-1`; at most 12, evenly spaced, for a 360-fold shaft). Each is built as a trial design
(`compute` + the DRC's geometry code, `drc.placement_metrics`) and scored: port score (as before) + `W_CLEAR` 15 x clearance
(bounding-box gap to the nearest non-mated instance, capped at 2 cm) + `W_UPRIGHT` 20 x (z . up + 1) / 2 of the part's own +z (not for
jointed mates) + `W_INBOARD` 10 x (1 - d / reach), d = horizontal distance of the part's CoM from the rest of the vehicle's (free base
only), minus `W_INTERFERE` 1000 when it overlaps a non-mated instance by more than 1 mm (same rule as `INTERFERENCE`). The geometry
terms sum to at most 45 < the 50 points between an exact-type and a paired-family port pair, so geometry picks between rotations and
near-equal pairs and never overrules the port fit. `changes[].snap` = {score, why, k, evaluated, interferes, clearance_mm,
ambiguous (another pair within 2 points), alternatives: ranked [{to | a/b, port, k, score, why, interferes}]}. When there is a choice
(>= 2 placements) and every one interferes the edit is refused, listing why (`force = true` places anyway); a single determined
placement is made and the DRC reports it. If the trial cannot be built (another instance is still floating) the port score alone ranks,
and `why` says so.

**Auto-wiring (`autowire(a, b=None, kind=None, part=None, length=None)`, op `autowire`; `autowire_all()`, op `autowire_all`).** An open
port = a power / signal port no `[[wires]]` entry names. Target: same kind and connector family (`types_ok`; a wire carries its own
connectors, genders need not complement), power roles must complement (no source-source / sink-sink; passthrough-passthrough only when one side is fed), sink
`v_max` not below the source's; power tier 0 = exactly one of the two nets is fed (the DRC's notion: a battery / psu / source-role port
reaches it) = the unfed side needs the other (two pass-through ports, a PM02 and a PDB, pair only here: the fed one supplies); signal ports of the same part never pair (an ESC does not signal an ESC). One target:
taken. Several: identical outlets of one part (a PDB's `out1..4`) -> the first; anything else (e.g. `fc.pwm1..4` for an ESC: which motor
is which channel is the user's call) -> `autowire` raises, `autowire_all` reports it under `ambiguous`; nothing is guessed. Direction:
the fed / supplying end is `from`. **Wire part** = the lightest library wire (`mass_per_m`, else `mass_per_m_copper`) whose `i_max` >=
need x `WIRE_MARGIN` 1.2, need = max(peak, sustained) current on that wire in the DRC electrical model (a trial design with the wire is
checked; `check(..., diag=...)`), floor 2 A power / 1 A signal when nothing models a load (the part files' ampacities state no bundling
condition, hence the 20 % headroom; the floors give a 26 AWG signal lead). **Length** = straight line between the two port positions
(world poses of the built model) x `ROUTING_FACTOR` 1.3 (leads follow the frame and need slack; no routing is solved), at least 3 cm,
rounded up to the wire part's `stock_lengths` / `length_default` when it lists them, else to 5 mm; written as a `derived` provenance
table with the arithmetic. If a port position is unknown (offboard end, design does not build) pass `length`. **Connectors** are pinned
for every gendered end (the builder would otherwise warn `CONNECTOR_AMBIGUOUS`): candidates are the library parts of category exactly
`connector` with the opposite-gender port of that type (the builder's rule); preference: (1) the one pinned on that port type + gender on
another instance of the same part in this design, (2) the one pinned most often on that type + gender, (3) the smallest id (the builder's
default). `conductors` = 2 for power (supply + return), 3 for `motor-3phase`, 1 for signal. The record has `wire` (the `[[wires]]` entry
written) and `reasons` (part, length, connector_from / connector_to, current_A, target, warning when a port's own `i_max` is below the
need); the DRC delta is the edit's, as always. `autowire_all` initiators: every open signal port, any port with `required = true`, and
power ports the DRC would call floating (not source-role, no wire on their node); it then sweeps again, while it wires something,
over unfed non-source power ports with no wire of their own, taking tier-0 targets only (the PM02 the battery just reached now feeds the
PDB). It returns `wired`, `ambiguous`, `unmatched`. `ambiguous` has ONE entry per decision, `{ports, candidates, why}`: reports that
share a port or a candidate are the same choice seen from both sides (4 ESC signal ports x 4 PWM channels is one entry, not eight). A
lead that a datasheet mass already contains (the pack's XT60 lead, a motor's phase leads) must not weigh twice: a port carrying the
machine-readable `lead = {length?, awg?}` flag (parts/README.md) makes `autowire` / `autowire_all` set the wire's `included_in` to its
instance and use the lead's own length and gauge, with no argument; `autowire(..., included_in = "inst")` or
`autowire_all(included_in = {"bat.power_out": "bat", ...})` still sets it for ports without the flag.
Limits: no typed (connector-terminated) cable parts are chosen, only `wire-lead` wires; wiring into a net with no source yet sizes for the
floor (the reason says so; run `auto_devices` first, the DRC current model then sizes every wire).

**Devices from parts (`auto_devices(scope=None, overwrite=False, fill=None, ocv=None)`, op `auto_devices`).** Writes the `[[devices]]`
+ `[device_maps]` a part swap would otherwise need hand-edited, from parts and topology; every bound field is a map entry to a part
param (so provenance is the part's own), every computed value is a `set` table with `derivation` / `rationale`; generated bindings
carry `auto = true`. Rules: battery = pack params (cells, capacity, R0, Thevenin RC) + the OCV table (`ocv`, default
`lipo-cell-ocv-chen2006`); ESC device = ESC part + the motor on its `motor_out` wire + the prop on that motor's shaft mate (rotor
table from the prop's `ct_cq_j`, else a one-point static table; `dir` from the world joint axis so that dir x axis points +z); servo
= servo part on its arm joint (no-load point nearest the bus voltage); PSU / PM02 / PDB internal drop from `r_series` / `r_path`;
avionics = constant-current loads on the battery side of the BEC; imu / baro / mag / gnss from the FC / GPS parts (host board
fixed by part). The wire R is gauge x length (the DRC / `wire` devices). A value no part carries (motor `c_th` / `r_th`, sensor
`rate_hz`, GNSS `lat0` / `lon0` / `alt0`, a servo `pid_scale`, the servo-bus adapter resistance ...) stays unset and is listed in
the record's `unset` (`[{device, fields}]`); the binding is still written so the DRC works, the sim says what is missing. Supply
them with `fill = {"esc*": {"set": {"motor.c_th": {value, unit, fidelity, rationale}}}}` (fnmatch on device names; filled values
persist across regeneration as `carried`). Existing explicit (hand-written) bindings win; `overwrite = true` (or a list of device
names) replaces them. Idempotent: the record has `created / refreshed / unchanged / kept / removed / unset / notes`. Stripped of
their device sections, x500v2 / aos5 / so101 / koch reproduce the reference bindings exactly except for the documented rules in
`tests/python/test_design_devices_auto.py` (`DIFFS`: bench-fit `cq_ref`, rounded literals, LeRobot gains, estimates only the
reference makes).

**Swap and devices.** `swap` adds `stale_devices` to its change record: bindings naming the swapped instance (`generated` says
whether `auto_devices` wrote it). A hand-written binding is named, not touched; an `auto_devices_offer` is attached. `swap(...,
auto=True, overwrite=False, fill=None)` runs `auto_devices` in the same edit (record under `auto_devices`; `overwrite = true`
also replaces the stale hand-written bindings of that instance), so undo reverts both. After swapping a motor the values filled for the
old one (`c_th`, `r_th`) are carried over and reported: they describe the old motor.

**From an empty design (`tests/python/test_design_scratch.py`).** `Design.new(name, parts=None, base="free")` has no instances and
nothing to check; `add(part, root=True)` places the frame (one root, never `to`, never offboard) and every other `add` before it is
refused ("no root yet"). `fasteners=None` on an `add` / `mate` writes no `fasteners` (a plate clamped by bolts that another mate
already counts, e.g. the X500 top plate on the M3x38 clamp bolts); the default `"auto"` adds them wherever a port has a thread.
Test result: a complete X500 (frame by snapping to named ports, 4 motors / props / ESCs, PDB, PM02, Pixhawk 6C, M8N GPS, SiK radio,
pack, sensors, wires) built this way has 0 DRC errors, hovers, and agrees with `x500v2.toml` to 0.1 % in mass and hover current.
A 4-wheel skid-steer car (`assemblies/car4.toml`: 4 x Pololu 37D 4751, 2 x RoboClaw 2x7A, 3S pack, custom 4-bracket plate
`car4-chassis-plate-custom`) was built the same way by an agent, one request at a time, and is driven by `examples/car4/` over two RoboClaw ptys
(`[interfaces.roboclaw] controllers = [...]`: more controllers, one pty each, `NAPL_SERIAL_2`...). Op log, video and what is measured:
`docs/showcase/car/README.md` (the rover-style `servo` wheel devices and `wheel_drive` map are written with the `device_map` / `device` ops: `auto_devices`
only derives the battery there; `autowire_all` pairs a RoboClaw motor output with the other controller's supply input, see that README).

`suggest("inst.port")` rows and the `add ... to` change record carry `needs`: up to four hints from the part's other ports (a male /
female mech port: parts that take it; a power / signal port: open compatible ports in this design to wire to, else library parts that
have one). A hint from port kinds, not a planner.

**Editing in the viewer (`napl design view x.toml`, `Sim.show(design=Backend)`, `napl mcp --view`).** The viewer's Design window is a
front end to this same `Design`, not a second editor: each control sends one JSON request (`design_edit` / `design_suggest` /
`design_undo` / `design_simulate` / `design_save`, the MCP tools' arguments) to `napl.design.viewer.Backend`, which calls the matching
`Design` method, rebuilds, and answers with the edit's record (`changes`, DRC delta) plus the new state (report, DRC findings, ports,
undo depth); the window then loads the new MJCF into the World (LoadModel, source `viewer`) keeping the camera and selection. Under
`napl mcp --view` the backend edits the server's own design under its lock: an agent's `design_edit` shows up in the window within a
second, and the window's edits are the agent's next state. Controls: *Library* tab = parts for the selected port (category filter;
`suggest`), "snap here" (placements ranked, `changes[].snap.alternatives` listed with score / why, "< prev / next >" or a row click previews
one: undo + the add pinned to that port pair and rotation, one undo step), "suggest swaps" + "swap" (device bindings refreshed,
`auto=True`); *Wiring* tab = autowire the selected port / all, ambiguous decisions listed per port with a candidate picker (each pick is
`autowire a, b`); *Simulate* tab = `hover` / `hold` (the numbers `Design.simulate` returns); toolbar = undo / redo (depth shown),
remove selected, refresh, save (path box; writes the TOML, comments of a hand-written file are not kept); *DRC* window = every finding after
each edit (colour = severity, `*` = new from the last edit, tooltip / line = fix); clicking one selects the instance it involves. Scripted
for tests as `dremove | dsnap | dcycle | dswap | dautowire | dautowire_all | dundo | dredo | dsave | dsim | dfinding | dsuggest | dsync`
(`napl-view --script`, `Sim.show(script=...)`). Tests: `tests/python/test_viewer_design.py` (window session == the same ops on the Python
API, DRC deltas and document revisions compared step by step), `tests/cpp/test_viewer.cpp` `viewer_design.*`; screenshot
`docs/showcase/design_window.png` from `tools/design_shot.py`.

`simulate("hover")`: built-in controller (not agent code) on ground-truth state, driving only the ESC throttles through the command
bus: altitude PID on the height above take-off (ramped to `altitude` in `climb_s`) + attitude PD with a small integral,
rotor mixer from the model's own rotor positions / axes / spin directions, thrust-vs-throttle and torque/thrust calibrated on a
fork of the sim, inertia approximated from instance masses (controller scaling only). Reports pass (height within `tol` for
`hold_s`), altitude error, hover throttle, battery / motor currents, SoC slope and endurance (time to the DRC's 20 % reserve at
that slope). `simulate("hold")`: servos torque-on at `pose` (default zero), reports sag (|joint - goal|) and currents.

MCP tools: `design_open` (control; `path`, or `new` = a name for an empty design), `design_edit` (control, list of ops, atomic), `design_suggest` (read), `design_check` (read),
`design_build` (control, writes `<root>/build/design`), `design_simulate` (control), `design_undo` (control; `redo = true`),
`design_save` (destructive: client elicitation or `--allow-destructive`). Paths are confined to the server root; arguments are
validated against the advertised JSON schemas.
CLI: `napl design suggest <asm> <inst.port|inst> [--limit N] [--json]`.

## Build UI (`napl design view`): modes, free placement and the scriptable operations

**Modes** (top bar tabs): BUILD (3D view, sidebar with the compact library, inspector with INSPECT / PARTS / DRC / PLACE / HISTORY tabs;
HISTORY lists the undo steps, a click travels to one), LIBRARY (fuzzy search, category tree, parameter range filters, thumbnail grid from
`build/thumbs/<id>.png`, detail panel with provenance, ADD / SWAP). Missing thumbnails are rendered on demand: `napl design view`, the MCP
window and the screenshot / record tools start `tools/thumbs.py --missing` in the background (the parts of the open design first; needs a
Vulkan device, `NAPL_THUMBS=0` disables it); the grid fills in as they land and the cache in `build/thumbs` is kept between runs (tiles of
dark parts are brightened by a gamma lift, the scene lighting is untouched). WIRING (the 2D wiring canvas, `wire2d.cpp`: a layered
signal-flow layout, sources on the left, loads on the right, orthogonal wires that never cross a card, a legend), SIMULATE (run / pause / step / rate, inspector), PROGRAM (read-only assembly TOML, `design_text`).
The status bar shows the selection, DRC counts and the sim time. A card dragged from LIBRARY or the BUILD sidebar shows a ghost that
snaps to the open compatible ports; dropping it is `add` + snap = ONE undo step. BUILD: click / shift-click (or Ctrl-click) multi-select, an axis-
coloured move / rotate gizmo (grid 5 mm, 15 degrees, numeric fields in the inspector), Figma-style guides (edge / centre / corner
alignment lines in the axis colours) and port snapping while dragging (Ctrl inverts snapping), per-instance colour, duplicate,
mirror, align / distribute. Keys: G / R move / rotate, Del or X remove, Esc cancel a drag / clear the selection, Ctrl+Z / Ctrl+Y (Ctrl+Shift+Z) undo / redo,
F frame the selection, Ctrl+A select all, Ctrl+D duplicate, 1-4 or the numpad = camera presets front / side / top / iso (`ui.camera`).

**Free placement (`pos` / `rpy` on an instance).** An instance held by no mate may carry `pos` (m) and `rpy` (rad, URDF fixed-axis)
in the ROOT instance's frame; the solver treats it as a child of the root (`snap.PoseSpec` in `solve_tree`), so it moves with the root,
counts as connected for the DRC (`FLOATING` ignores it; the mech BFS has a root-placed edge), and still takes mates for its own children.
The root has no `pos` / `rpy` (it defines the frame); `validate` rejects wrong lengths. Gizmo edits and mates: moving a MATED
instance asks "detach from mate?"; confirming sends `place ... unmate = true` which drops the mate that held it to the root side
(its own children stay mated to it) and writes `pos` / `rpy`; without `unmate` the backend refuses. Dropping a part on a port is a mate
(no `pos`); dropping it on empty space / the ground is a free pose. `place` is a normal `design_edit` op, so undo / redo, the DRC delta
and MCP all see it: `{"op":"place","id":"m1","pos":[..],"rpy":[..]}` or `delta` / `rot` (3x3, about `about`), `unmate`;
`{"op":"color","id":"m1","rgba":[r,g,b,a]}` (empty clears); `duplicate {id, delta?, new_id?}`; `mirror {ids, axis, at?}` (free-placed
copies; the orientation is M R M, the part itself is not flipped). `add` also takes `color`, and `pos` / `rpy` when it has no `to`.

**Every UI action is a named backend op** (`napl.design.uiops`, a mixin of `viewer.Backend`; same JSON as the MCP tools:
`{"tool": NAME, "args": {...}}`). An op does its real work on the Design immediately (an agent without a window gets the same
document) and queues an *event* the window drains with `ui_poll` (every 100 ms) and shows: tab switch, search typed one character per
`type_ms`, ghost flight, guide flashes, selection. World-frame arguments are metres, +z up.

| op | args | what the window shows |
|---|---|---|
| `ui.mode` | `name` build / library / wiring / simulate / program | the tab switches |
| `ui.camera` | `preset` top / front / side / iso, or `yaw`, `pitch`, `dist` | a camera flight |
| `ui.frame` | `instances?` | the camera frames them |
| `ui.gizmo` / `ui.guides` / `ui.snap` | `mode` move / rotate / none; `on`; `grid_m?`, `angle_deg?`, `edges?`, `ports?` | toolbar state |
| `ui.view` | `tooltips?` auto / on / off, `emphasise_wires?` | hover cards: auto = only while a person moves the mouse (recordings stay clean), on = always, off = never; emphasise = cables at least 4 px wide in BUILD / LIBRARY / WIRING (the data keeps their diameter) |
| `ui.hover` | `instance`, `seconds?` | that part's compact tooltip (name, id, mass, three parameters) for ~6 s, whatever `tooltips` says; switches to BUILD |
| `ui.state` | | the `Backend.ui` dict (mode, selection, search, snap settings, tooltips, emphasise_wires) |
| `select` | `instances`, `add?` | highlight + gizmo |
| `library.search` | `text`, `filters` {param: [lo, hi]}, `category`, `limit`, `offset`, `type_ms?` | the box fills; the grid filters |
| `library.categories` / `.ranges` / `.info` / `.targets` | `{}` / `category?` / `part` / `part` | filters, detail panel, ghost poses on every open compatible port |
| `library.preview` | `part`, `to` ('inst.port') | the ghost at that port |
| `library.drop` | `part`, `to` or `point`, `port?`, `id?`, `rpy?` | = `add` + snap, one undo step; selects the new instance |
| `library.drag` | `part`, `to` {instance, port} or {point}, `animate?` | the ghost flies, then drops (with `animate` and a window attached; otherwise same as drop) |
| `gizmo.move` | `instance`, `delta` or `pos`, `unmate?`, `snap?` | edge / grid snap guides flash |
| `gizmo.rotate` | `instance`, `axis`, `angle_deg` or `rpy` / `rot`, `unmate?` | |
| `set_color` | `instance`, `rgba` or null | the colour picker opens, the part recolours |
| `duplicate` / `mirror` / `align` / `distribute` | `instance`, `delta?` / `instances`, `axis`, `at?` / `instances`, `axis`, `to` min / center / max / `instances`, `axis` | |

The viewer's script verbs `mode`, `uisel`, `search`, `uicam`, `uiframe` (`napl-view --script`) do the window-only part of the same
actions. `tools/record_build.py build` uses these ops so a recording shows the LIBRARY tab, the typed search and the ghost before each
`add` (`--no-ui` skips them; the design is identical). Tests: `tests/python/test_ui_ops.py` (backend ops), `tests/cpp/test_viewer.cpp`
`viewer_design.every_mode_draws_*` (the modes under a fake backend).

## Driving an arm design (`napl program`, `[design]` with a feetech / dynamixel interface)
```toml
[design]
path = "my-arm.toml"          # a saved design; optional parts = "dir"
[model]                       # optional: script = "scene.py" composes a scene around the design: build(outdir, design=<path>, parts=<dir>)
[interfaces.feetech]          # without `servos` / `baud` they come from the design: {servo_id: "/devices/<servo>"}, the bus baud
```
Without [model] the design is built as it is. `servos` needs a `servo_id` param on every servo device (else a `ScenarioError` naming them).
`examples/so101/pick.scenario.toml` with this `[design]` (the SO-101 built from nothing, tests/python/test_design_scratch_arm.py) passes its
monitors and its replay hash matches; `examples/so101/scene.py` uses the design's instance / joint names (`base`, `fixed_jaw`, `gripper`, ...).

## Flying a design with PX4 (`napl program`, `[design]`)
A design saved with `Design.save` (or any assembly) flies against real PX4 SITL with no hand-written bridge config:

```toml
[design]
path = "my-quad.toml"          # optional: parts = "dir"
[interfaces.px4]               # bare: no `bridge`, no [model]
[program]
cmd = ["python", "{dir}/hover.py"]   # see examples/x500/hover.design.scenario.toml
```
`napl.design.px4cfg` builds the world from the design and writes `<run dir>/bridge.toml`: PX4 motor N (`controls[N-1]`, PWM_MAIN_FUNCN =
100 + N in `px4/airframes/4902_napl_x500v2`) is the ESC wired to `fc.pwmN` (`fc.pwmN -> escK.signal`), sensors / battery are the design's
first imu / baro / mag / gnss / battery devices, home is the gnss `lat0 / lon0 / alt0`. Before PX4 starts the design is checked against
PX4's quad-X airframe (4001_quad_x geometry; motor numbering and spin directions from the PX4 airframe reference,
https://docs.px4.io/main/en/airframes/airframe_reference.html#quadrotor-x: 1 front-right CCW, 2 rear-left CCW, 3 front-left CW, 4 rear-right
CW; `CA_ROTORn_KM` > 0 = CCW). Errors, each with its fix (the run is refused as a `ScenarioError`; `check_px4(asm, xml)` returns them as findings):

| code | when |
|---|---|
| `PX4_MOTOR_ORDER` | the rotor behind `fc.pwmN` is not at motor N's corner (front/rear, left/right of the rotor centroid, +x forward); the fix lists the full correct `fc.pwmN -> escK` order |
| `PX4_PROP_DIRECTION` | the rotor's spin (joint axis sign; a CW prop = `reverse` joint + rotor `dir = -1`) differs from motor N's CCW / CW |
| `PX4_PROP_THRUST` | a rotor pushes down (`dir` x joint axis z <= 0) |
| `PX4_PWM_UNWIRED` / `PWM_DUPLICATE` / `PWM_RANGE` | an ESC without a PWM wire, wired twice, or on an output outside 1..4 |
| `PX4_MOTOR_COUNT` / `NO_ROTOR` / `GEOMETRY` | not four ESCs, an ESC without a rotor on its joint, rotors not on the corners of a square X |

Only the quad-X airframe (4902) is supported; the PX4 parameters (rate gains, `BAT1_N_CELLS 4`) stay those of the airframe file.
Measured (tests/python/test_px4_design.py, PX4 v1.17.0, 5 m / 20 s hold / land): the scratch-built quad and the stock X500 hover at throttle
0.425 / 11.5 A; with the four motors swapped for the KDE 2315XF-965 (`swap(auto=True)`) 0.409 / 11.3 A, PX4's hover-thrust estimate 0.427 -> 0.410
(the `hover_thrust_estimate` topic, polled with px4-listener, tracks the hover throttle; the airframe's `MPC_THR_HOVER` default 0.5 is only its initial value).
