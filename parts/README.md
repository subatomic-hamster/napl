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
material = "aluminium-6061"         # optional material part id (density etc.)

[[sources]]
id = "S1"
url = "https://..."
title = "..."
accessed = "2026-10-06"
quote = "verbatim text (or table cell text) the value came from"
license = "..."                     # only for sources whose files we copy into the repo
```

Fidelity tags:
- `datasheet`: stated by the manufacturer/vendor for this exact part.
- `literature`: peer-reviewed or well-known reference measurement of this part or exact model.
- `measured`: independent test data of this exact part (e.g. public thrust-stand logs).
- `derived`: computed from other sourced values; `derivation` required.
- `estimated`: nothing published; `rationale` required (cite the closest evidence).

Rules: no value without `src` unless `derived`/`estimated`; quotes are verbatim so a verifier can
re-fetch the page and find them; never invent a URL. Unit conversions from the source (g to kg,
kg*cm to N*m) are fine under the original fidelity tag and should be noted in the quote context.
