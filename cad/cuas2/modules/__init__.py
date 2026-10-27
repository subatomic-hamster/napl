"""Registry of the six cuas2 modules. Each module exposes PARTS {name: dict(fn, group, count, material, module)} and bought() [items].
Tilt-group shapes are authored in the launcher frame L; the build shifts them to the head frame H (origin on the tilt axis)."""

from . import base_pan, feeder, launcher_cartridge, power_box, sensor_head, tilt_head

MODULES = {
    "M1": base_pan,
    "M2": tilt_head,
    "M3": launcher_cartridge,
    "M4": feeder,
    "M5": sensor_head,
    "M6": power_box,
}


def registry():
    reg = {}
    for key, mod in MODULES.items():
        for name, rec in mod.PARTS.items():
            assert name not in reg, name
            reg[name] = dict(rec, module=key)
    return reg


def bought_items():
    out = []
    for mod in MODULES.values():
        out += mod.bought()
    return out
