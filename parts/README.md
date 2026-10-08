# Parts database

One TOML file per real, purchasable part: `parts/<domain>/<id>.toml` (`domain` = `drone`, `arm`,
`common`). Parts are facts about real hardware; every number must be traceable.

```toml
id = "holybro-2216-kv920"          # kebab-case, unique, matches file name
name = "Holybro 2216 KV920 brushless motor"
category = "motor.bldc"            # dotted: motor.bldc, motor.servo, prop, battery.lipo, esc,
                                   # pcb.fc, pcb.pm, sensor.imu, wire, connector, fastener,
                                   # frame, material, psu, ...
vendor = "Holybro"
part_number = "..."                # vendor P/N if any
url = "https://..."                # product page

[params]
# value + SI unit + source id + fidelity. Units are SI unless the quantity is conventionally
# non-SI and named that way (kv in rpm/V).
mass       = { value = 0.0, unit = "kg",    src = "S1", fidelity = "datasheet" }
kv         = { value = 920, unit = "rpm/V", src = "S1", fidelity = "datasheet" }
# Inline tables must stay on ONE line (TOML 1.0 / tomllib). For long entries use a sub-table:
[params.r_winding]
value = 0.0
unit = "ohm"
fidelity = "derived"
derivation = "R = (V - Kv^-1*w)/I at no-load point ..."

[params.backlash]
value = 0.0
unit = "rad"
fidelity = "estimated"
rationale = "no published value; typical for ... per S3"

[tables.thrust_test]               # optional tabular data, columns carry units
columns = ["throttle_pct", "voltage_V", "current_A", "thrust_N", "rpm"]
rows = [[50, 16.0, 3.1, 4.9, 6500]]
src = "S2"
fidelity = "datasheet"

[geometry]                          # optional; what the builder needs to place/draw it
shape = "cylinder"                  # box | cylinder | mesh
size = [0.0, 0.0]                   # SI, shape-specific (cylinder: radius, half-height)
mesh = "meshes/so101/base.stl"      # repo-relative, license noted in sources
material = "aluminium-6061"         # optional material part id (density etc.; its [appearance] rgba colours the geom)
# Material parts only: [appearance] rgba = [r, g, b, a] is COSMETIC (renderer colour; no physics, no provenance). The builder
# emits it as the MJCF geom rgba; parts without a material get a colour from the builder's category table (build.py).

[[sources]]
id = "S1"
url = "https://..."
title = "..."
accessed = "2026-10-06"
quote = "verbatim text (or table cell text) the value came from"
license = "..."                     # only for sources whose files we copy into the repo
```

## Ports (interfaces for snapping, wiring and design-rule checks)

Every part that connects to anything declares its interfaces. The builder snaps parts together
by mating compatible ports; the design-rule checker (DRC) uses port ratings.

```toml
[[ports]]
name = "base"                          # unique within the part
kind = "mech"                          # mech | power | signal
type = "bolt-pattern.16x16.m3"         # interface family (see table below)
gender = "female"                      # male | female | neutral (mech holes = neutral)
pos = [0.0, 0.0, 0.0]                  # m, part frame
axis = [0.0, 0.0, -1.0]                # unit outward normal: mating parts face opposite axes
ref = [1.0, 0.0, 0.0]                  # unit in-plane reference (orientation about axis)
symmetry = 4                           # n-fold rotations about axis that still fit
thickness = 0.002                      # m of material clamped at this port (fastener stack)

[[ports]]
name = "power_in"
kind = "power"
type = "xt60"                          # connector family
gender = "male"
v_max = { value = 25.2, unit = "V", src = "S1", fidelity = "datasheet" }
i_max = { value = 60.0, unit = "A", src = "S1", fidelity = "datasheet" }   # continuous
role = "sink"                          # source | sink | passthrough

[[ports]]
name = "bus"
kind = "signal"
type = "feetech-ttl"                   # pwm | dshot | uart | i2c | can | feetech-ttl | usb
gender = "female"
level = { value = 5.0, unit = "V", src = "S1", fidelity = "datasheet" }
```

**Lead flag (`lead`, power / signal ports).** A part whose datasheet mass already includes a cable permanently attached to a port
(the pack's XT60 lead, a motor's phase leads, a GPS cable) marks that port machine-readably:

```toml
[[ports]]
name = "power_out"
kind = "power"
# ...
lead = { length = "discharge_wire_length", awg = "discharge_wire_awg" }   # both optional; `lead = {}` = flag only
```

`length` / `awg` are each either the name of a numeric param of the same part (preferred: it keeps the provenance) or an inline
provenance table `{ value, unit, fidelity, ... }`; a param name the part lacks is an error at autowire time. Effect: `autowire` /
`autowire_all` set `included_in` to that instance on any wire ending on the port (no `included_in` argument needed), use the lead's
length instead of the routed one (warning when the routed run is longer), and, when `awg` is given and the library has a wire part
with that `awg`, use that wire. Only mark a port whose cable mass really is inside the part's `mass`.

Every `mech` port must have `ref` (unit vector perpendicular to `axis`) and `symmetry` (integer >= 1); `tools/check_parts.py`
enforces it, because snapping needs both. Round shafts (`shaft.<d>`, `prop_adapter`) are axisymmetric: `symmetry = 360`, any
`ref`. Prop hubs (`bore.<d>`): `ref` along a blade, `symmetry = 1` (the prop spins freely, the builder uses a hinge).

Mate compatibility: same `type` (or the paired families `shaft.<d>` / `bore.<d>` and `tube.<od>` / `tube-clamp.<od>` of
the same size), and genders male+female or either neutral. Two mated mech
ports coincide in position, have opposite axes, and their `ref`s align up to the `symmetry`
rotations. Interface families in use (extend as needed, keep names stable):

| type | meaning |
|---|---|
| `bolt-pattern.<a>x<b>.<thread>` | rectangular hole pattern, centre-to-centre mm (e.g. 16x16.m3, 30.5x30.5.m3) |
| `bolt-circle.<d>.<n>.<thread>` | n holes on a d-mm circle |
| `shaft.<d>` / `bore.<d>` | round shaft (male) and matching bore (female), mm; these two types mate each other |
| `tube.<od>` / `tube-clamp.<od>` | tube and the clamp that holds it, mm; these two types mate each other (the X500 files use `tube.<od>` male/female on both sides) |
| `flat-mount` | board or body lying on a face, no hole pattern (tape / strap / foam seat), neutral |
| `bolt-pattern.x500v2-arm-clamp.m3` | X500 V2 arm-connector clamp footprint: 4 M3 through-bolts (hole positions not recorded), thread from the `thread` key; used between plates and arm connectors |
| `mount.x500v2-landing-gear` | X500 V2 landing-gear pole-top seat under the bottom plate (placeholder: no hole pattern, hardware not modelled) |
| `servo-horn.<spline>` | servo output spline (e.g. 25t) |
| `thread.<m>` | single threaded hole or stud (fastener seat) |
| `xt30`, `xt60`, `xt90`, `jst-gh.<n>`, `jst-ph.<n>`, `servo-3pin`, `solder-pad` | connectors |

Fidelity tags:
- `datasheet`: stated by the manufacturer/vendor for this exact part.
- `literature`: peer-reviewed or well-known reference measurement of this part or exact model.
- `measured`: independent test data of this exact part (e.g. public thrust-stand logs).
- `derived`: computed from other sourced values; `derivation` required.
- `estimated`: nothing published; `rationale` required (cite the closest evidence).

Rules: no value without `src` unless `derived`/`estimated`; quotes are verbatim so a verifier can
re-fetch the page and find them; never invent a URL. Unit conversions from the source (g to kg,
kg*cm to N*m) are fine under the original fidelity tag and should be noted in the quote context.

## Visual layer (`[visual]`, cosmetic only)

`[geometry]` stays the physics shape (collision, mass, inertia). `[visual]` says what the part *looks like* in the viewer,
renders, thumbnails and videos; it never feeds physics and its values carry no fidelity tag (they are excluded from the
fidelity tally: `tools/check_parts.py` counts only params / tables / ports). The builder (`napl.design.build`) emits every
visual element as a zero-density, non-colliding mesh geom (`mass="0" contype="0" conaffinity="0"`, same geom group as the plain
shape) with a MuJoCo `<material>`, and makes the plain `[geometry]` shape fully transparent (alpha 0; it still collides and
carries the mass). Meshes are generated at build time into `<out_dir>/visuals/` (content-addressed STL, never committed).
`build(..., visual=False)` / `compute(..., visual=False)` skips the layer; assemblies simulate bit-identically either way.

```toml
[visual]
src = "V1"                          # [[sources]] id of the appearance reference (product page / photo), or `rationale = "..."`
note = "black anodised bell, blue base ring (product photo)"   # free text: what was copied from the photo

[[visual.elements]]                 # one element = one generator or one mesh file; several elements stack
gen = "bldc_outrunner"              # family generator (python/napl/design/visuals/gens_*.py), or: mesh = "meshes/x.stl"
params = { vents = false }          # generator arguments; "param:NAME" takes the part's own params.NAME.value. Dimensions
                                    # (diameter, length_body, length / width / height ...) are read from [params] / [geometry]
pos = [0.0, 0.0, 0.0]               # optional placement in the part frame (default per generator, see below)
zaxis = [0.0, 0.0, 1.0]             # optional: shortest-arc rotation taking +z to this axis (or rpy = [r, p, y], radians)
colors = { bell = "alu-anodised-black", base = [0.05, 0.5, 0.85] }   # slot -> material name or [r, g, b] (0..1)
```

A generator returns one mesh per **slot** (motor: `base bell stator copper magnet shaft adapter`); each slot has a default
finish and `colors` overrides it. A material is `(rgb, specular, shininess, metallic, roughness)` = the MuJoCo `<material>`
attributes (all 0..1; `metallic` / `roughness` exist in MuJoCo 3.7); the named finishes are in
`python/napl/design/visuals/materials.py`. A part without `[visual]` still gets its category's default family
(`visuals.DEFAULTS`, e.g. `motor.bldc` -> `bldc_outrunner`). A part without `[geometry]` is drawn too: the family reads its
dimensions from the part's own params (length / width / height, diameter, `package_*`, `envelope_*` ...) and draws at the
instance frame (the part frame the ports define; boards and packs with their underside at z = 0) on the zero-mass point body the
builder gives such an instance, so physics is unchanged. Where a part publishes no dimension, give an explicit `[visual]` with
`params = { size = [...] }` (sensor_ic) or a family's own params and a cited source. Not drawn: raw materials, OCV curves, wires
(the wire layer) and frame-kit data records.

Generator frames (an element `pos` is only needed when a part deviates): motor mounting face at z = 0, shaft +z; propeller hub
underside at z = 0; gearmotor / stepper / actuator output face at z = 0, body towards -z; boards, packs, pucks, camera / sensor
modules sit with their underside on the underside of the `[geometry]` bounds, centred in x, y; plates, tubes and the `shapes`
fallback are drawn in the part frame from `[geometry]`. Families: `bldc_outrunner propeller lipo_pack cell_18650 cf_tube
board_generic board_esc board_fc board_power board_sbc gnss_puck camera_module sensor_module box_device connector servo_case
gearmotor wheel_tyre stepper_nema qdd_actuator bearing standoff screw heat_insert diabolo_pellet pulley hub_disc shaft_collar
plate_outline shapes` (plus the structure / servo / launcher families listed in `visuals/__init__.py`). Rocket and marine
families (`gens_rocket.py`, `gens_marine.py`, each used by an explicit `[visual]`): `solid_motor body_tube ogive_nose fin_set
landing_leg tvc_gimbal` (`params = { kind = "plate" | "ring" | "cradle" }`) `catamaran_hull ducted_thruster esc_sealed`.

Vendor CAD as a visual: `mesh = "meshes/..."` (binary or ASCII STL, one slot `body`, finish from `colors.body`). Only files whose
licence allows redistribution in a proprietary product belong in the repo (record it in the source's `license`); today that is
only the SO-101 and Koch STLs (Apache-2.0, `meshes/*/LICENSE`). Every other shape is generated.
