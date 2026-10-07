# Assemblies

`assemblies/<name>.toml` describes a robot as instances of library parts (`parts/`), mates between their ports, fasteners,
wires and device bindings. `napl design build assemblies/<name>.toml` turns it into `build/assemblies/<name>.xml` (MJCF with
a `napl.devices` block) and `<name>.report.json` (mass budget, BOM, provenance, warnings). Python: `napl.design`
(stdlib + numpy only; no C++ kernel needed to build, only to simulate the result).

```
napl design build assemblies/x500v2.toml [--parts DIR] [--out DIR]   # default parts: <repo>/parts, out: build/assemblies
napl design info  assemblies/x500v2.toml [--parts DIR]               # mass budget table, writes nothing
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
dir = 1                    # flags (complementary, ground_effect, enabled) and `dir` pass through as given

[[targets]]
name = "takeoff mass"
groups = ["frame", "power"]            # or all = true
value = { value = 1.76, unit = "kg", src = "S1", quote = "...", fidelity = "datasheet" }
```

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
`select_fasteners`, `compute` (in memory), `build`, `format_budget`; all failures raise `DesignError`.
