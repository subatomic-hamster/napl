"""Numbers-rule TOML -> plain values. Every numeric leaf is { value, unit, fidelity, rationale }."""

import copy
import tomllib
from pathlib import Path

FIDELITY = {"datasheet", "measured", "literature", "estimated", "design"}
SHIPPED = Path(__file__).with_name("autonomy.toml")


def _leaf(key, v):
    if isinstance(v, bool) or isinstance(v, (int, float)):
        raise ValueError(
            f"{key}: bare number (numbers rule: value, unit, fidelity, rationale)"
        )
    for f in ("value", "unit", "fidelity", "rationale"):
        if f not in v:
            raise ValueError(f"{key}: missing {f}")
    if v["fidelity"] not in FIDELITY:
        raise ValueError(f"{key}: fidelity {v['fidelity']!r} not in {sorted(FIDELITY)}")
    if not str(v["rationale"]).strip():
        raise ValueError(f"{key}: missing rationale")
    return v["value"]


def leaves(sec, prefix):
    """One TOML table -> plain values; strings and booleans pass through, every number must carry its provenance."""
    return {
        k: v if isinstance(v, (str, bool)) else _leaf(f"{prefix}.{k}", v)
        for k, v in sec.items()
    }


def load(path=None):
    raw = tomllib.loads(Path(path or SHIPPED).read_text())
    return {s: leaves(sec, s) for s, sec in raw.items()}


def override(cfg, **kv):
    out = copy.deepcopy(cfg)
    for dotted, v in kv.items():
        s, k = dotted.split(".")
        if k not in out[s]:
            raise KeyError(dotted)
        out[s][k] = v
    return out
