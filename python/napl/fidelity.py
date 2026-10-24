"""Fidelity report: where every number of every simulated design comes from and how well each model is validated.

    napl report fidelity [--out docs/fidelity.md] [--json build/fidelity.json] [--date YYYY-MM-DD] [--no-servo]

Sections: library provenance (counted exactly like tools/check_parts.py), one section per assemblies/*.toml (each built
with napl.design.build: mass by group x fidelity, every `estimated` value the build used, unresolved items, DRC), one
section per model family (equations -> docs/models.md, parameter provenance from the library, validation numbers parsed
from docs/validation/*.md and the servo bench, known gaps from those docs), and the determinism / replay test list.
Deterministic: sorted everywhere, no timestamps unless --date is given. The servo bench needs the C++ kernel (skipped,
and said so, when `napl._napl` is not importable).
"""

import collections
import json
import math
import re
import tomllib
from pathlib import Path

from ._resources import data_dir

ROOT = data_dir()
FIDS = ("datasheet", "literature", "measured", "derived", "estimated")
SOURCED = ("datasheet", "literature", "measured")

# (model, docs/models.md row prefixes, library category prefixes, params that feed the model)
MODELS = [
    (
        "battery",
        ["Battery", "PSU"],
        ["battery."],
        [
            "capacity_ah",
            "r_internal_pack",
            "thevenin_r1",
            "thevenin_tau",
            "v_full",
            "i_max_continuous",
        ],
    ),
    (
        "motor + ESC",
        ["ESC", "BLDC / DC motor + thermal", "Damping split"],
        ["motor.bldc", "esc"],
        [
            "kv",
            "r_winding",
            "i0",
            "kt",
            "j_rotor_bell",
            "r_on_total",
            "switching_loss_fraction",
            "quiescent_current",
        ],
    ),
    (
        "rotor (aero)",
        ["Rotor aero"],
        ["prop"],
        ["diameter", "ct_static", "cq_static", "j_prop", "ct_borrowed_from", "mass"],
    ),
    (
        "servo",
        ["Servo firmware"],
        ["motor.servo"],
        [
            "stall_torque_6v",
            "no_load_speed_6v",
            "no_load_current_6v",
            "r_effective",
            "ke",
            "gear_efficiency",
            "i0_visc",
            "rotor_inertia_motor",
            "backlash",
            "reg_d_coefficient_default",
            "reg_max_input_voltage_default",
        ],
    ),
    (
        "sensors",
        ["IMU", "Barometer", "Magnetometer", "GNSS"],
        ["sensor."],
        [
            "gyro_noise_density",
            "accel_noise_density",
            "accel_noise_density_xy",
            "gyro_zro_initial",
            "gyro_fs_max",
        ],
    ),
    ("environment", ["ISA", "Wind", "Body drag"], [], []),
]

RESEARCH = [  # docs/research cited per model (file, gap heading)
    ("battery", ["battery-data.md"]),
    ("motor + ESC", ["propulsion-library.md", "x500v2-kit.md"]),
    ("rotor (aero)", ["propulsion-library.md", "x500v2-airframe.md"]),
    ("servo", ["actuator-library.md", "so101.md", "sts3215-calibration.md"]),
    ("sensors", ["x500v2-airframe.md", "electronics-structure-library.md"]),
    ("environment", ["px4-sitl.md"]),
]

# (suite.name | pytest file::name, what it proves)
DETERMINISM = [
    (
        "determinism.two_worlds_equal",
        "tests/cpp/test_determinism.cpp",
        "two worlds from one seed hash equal",
    ),
    (
        "determinism.restore_continues_bitwise",
        "tests/cpp/test_determinism.cpp",
        "restore then run = uninterrupted run, bitwise",
    ),
    (
        "determinism.clone_continues_bitwise",
        "tests/cpp/test_determinism.cpp",
        "clone continues bitwise",
    ),
    (
        "replay.random_session_replays_to_equal_state",
        "tests/cpp/test_replay.cpp",
        "a recorded random session replays to the same state hash",
    ),
    (
        "replay.rerecord_is_byte_identical",
        "tests/cpp/test_replay.cpp",
        "re-recording a replay gives a byte-identical log",
    ),
    (
        "devices.save_restore_bitwise",
        "tests/cpp/test_devices.cpp",
        "battery / ESC / motor state restores bitwise",
    ),
    (
        "devices.save_restore_bitwise_aero",
        "tests/cpp/test_devices.cpp",
        "rotor / environment (turbulence RNG) state restores bitwise",
    ),
    (
        "devices.servo_save_restore_bitwise",
        "tests/cpp/test_devices.cpp",
        "servo firmware / encoder state restores bitwise",
    ),
    (
        "devices.all_save_restore_clone_bitwise",
        "tests/cpp/test_devices.cpp",
        "every device type in one model: mid-run restore and clone bitwise",
    ),
    (
        "devices.all_bus_log_and_replay",
        "tests/cpp/test_devices.cpp",
        "bus log of all device types replays to the same hash",
    ),
    (
        "devices.servo_bus_log_and_replay",
        "tests/cpp/test_devices.cpp",
        "servo inputs replay to the same hash",
    ),
    (
        "devices.clone_equal_hash",
        "tests/cpp/test_devices.cpp",
        "clone state hash equals the original",
    ),
    (
        "tests/python/test_sim.py::test_hash_matches_cpp_replay",
        "tests/python/test_sim.py",
        "Python Sim hash equals the C++ replay hash",
    ),
    (
        "tests/python/test_sim.py::test_restore_gives_original_hash",
        "tests/python/test_sim.py",
        "restore returns the original hash",
    ),
    (
        "tests/python/test_devices.py::test_device_state_is_part_of_save_and_fork",
        "tests/python/test_devices.py",
        "device state is in save / fork",
    ),
    (
        "tests/python/test_px4_flight.py::test_px4_flight",
        "tests/python/test_px4_flight.py",
        "PX4 flight bridge log replays to that run's own hash (the flight itself is not lockstep-deterministic)",
    ),
]


def _trunc(s, n):
    s = " ".join(str(s).split())
    return s if len(s) <= n else s[: n - 3].rstrip() + "..."


def _cell(s):
    return str(s).replace("|", "\\|").replace("\n", " ")


def _pct(x, d=1):
    return f"{100 * x:.{d}f}"


def _table(head, rows):
    out = ["| " + " | ".join(head) + " |", "|" + "---|" * len(head)]
    out += ["| " + " | ".join(_cell(c) for c in r) + " |" for r in rows]
    return "\n".join(out)


# ---------------------------------------------------------------- docs parsing
def _section(text, heading):
    """Body of the `## heading` (prefix match) section."""
    m = re.search(
        rf"^## {re.escape(heading)}[^\n]*\n(.*?)(?=^## |\Z)",
        text,
        re.DOTALL | re.MULTILINE,
    )
    return m.group(1) if m else ""


def _md_table(body):
    lines = [ln for ln in body.splitlines() if ln.startswith("|")]
    if len(lines) < 3:
        return []
    split = lambda ln: [
        c.strip() for c in re.split(r"(?<!\\)\|", ln.strip().strip("|"))
    ]
    head = split(lines[0])
    return [dict(zip(head, split(ln))) for ln in lines[2:]]


def _read(rel):
    p = ROOT / rel
    return p.read_text(encoding="utf-8") if p.exists() else ""


def _bullets(body):
    out, cur = [], None
    for ln in body.splitlines():
        if ln.startswith("- "):
            cur = [ln[2:]]
            out.append(cur)
        elif cur is not None and ln.startswith("  "):
            cur.append(ln.strip())
        elif not ln.strip():
            cur = None
    return [" ".join(b) for b in out]


def _models_rows():
    rows = {}
    for ln in _read("docs/models.md").splitlines():
        if (
            ln.startswith("| ")
            and not ln.startswith("| Model")
            and not ln.startswith("|---")
        ):
            c = [x.strip() for x in ln.strip().strip("|").split(" | ")]
            if len(c) == 5:
                rows[c[0]] = {
                    "state": c[2],
                    "simplifications": c[3],
                    "tests": re.findall(r"`([^`]+)`", c[4]),
                }
    return rows


# ---------------------------------------------------------------- library
def _tally(path):
    """Fidelity entries of one part exactly as tools/check_parts.py counts them (params, tables, port entries)."""
    d = tomllib.loads(path.read_text())
    ents = [e for e in d.get("params", {}).values() if isinstance(e, dict)]
    ents += list(d.get("tables", {}).values())
    for p in d.get("ports", []):
        ents += [
            e for k, e in p.items() if isinstance(e, dict) and k != "lead"
        ]  # lead: a flag, not a value
    return d, collections.Counter(e.get("fidelity") for e in ents), ents


def library(parts_dir):
    parts_dir = Path(parts_dir)
    tot, by_dir, by_cat = collections.Counter(), {}, {}
    n_parts = 0
    for f in sorted(parts_dir.glob("*/*.toml")):
        d, t, _ = _tally(f)
        n_parts += 1
        tot += t
        for key, tab in ((f.parent.name, by_dir), (d["category"], by_cat)):
            r = tab.setdefault(key, {"parts": 0, "values": collections.Counter()})
            r["parts"] += 1
            r["values"] += t

    def row(r):
        n = sum(r["values"].values())
        return {
            "parts": r["parts"],
            "values": n,
            "by_fidelity": {k: r["values"].get(k, 0) for k in FIDS},
            "sourced_pct": 100 * sum(r["values"].get(k, 0) for k in SOURCED) / n
            if n
            else 0.0,
            "derived_pct": 100 * r["values"].get("derived", 0) / n if n else 0.0,
            "estimated_pct": 100 * r["values"].get("estimated", 0) / n if n else 0.0,
        }

    n = sum(tot.values())
    return {
        "parts": n_parts,
        "values": n,
        "by_fidelity": {k: tot.get(k, 0) for k in FIDS},
        "sourced_pct": 100 * sum(tot.get(k, 0) for k in SOURCED) / n,
        "derived_pct": 100 * tot.get("derived", 0) / n,
        "estimated_pct": 100 * tot.get("estimated", 0) / n,
        "by_dir": {k: row(v) for k, v in sorted(by_dir.items())},
        "by_category": {k: row(v) for k, v in sorted(by_cat.items())},
    }


def param_fidelity(parts_dir, cats, names):
    """{param: {fidelity: n parts}} over library parts whose category starts with one of `cats`."""
    out = {k: collections.Counter() for k in names}
    for f in sorted(Path(parts_dir).glob("*/*.toml")):
        d = tomllib.loads(f.read_text())
        if any(
            d["category"] == c
            or d["category"].startswith(c.rstrip(".") + ".")
            or d["category"].startswith(c)
            for c in cats
        ):
            for k in names:
                e = d.get("params", {}).get(k)
                if isinstance(e, dict):
                    out[k][e.get("fidelity")] += 1
    return {k: dict(sorted(v.items())) for k, v in out.items() if v}


# ---------------------------------------------------------------- assemblies
def mass_items(rep):
    """(group, mass, fidelity) of every item in the mass total: instances, fastener sets, counted wires and connectors."""
    it = [
        (i["group"] or "(none)", i["mass"], i["mass_fidelity"])
        for i in rep["instances"]
        if not i.get("offboard")
    ]
    it += [
        (f["group"] or "(none)", f["mass"], f["mass_fidelity"])
        for f in rep["fasteners"]
    ]
    it += [
        (w["group"] or "(none)", w["mass"], w["mass_fidelity"])
        for w in rep["wires"]
        if w.get("counted")
    ]
    it += [
        (c["group"] or "(none)", c["mass"], c["mass_fidelity"])
        for c in rep["connectors"]
        if c.get("counted")
    ]
    return it


def assembly_section(path, parts_dir, out_dir):
    from .design import build

    rep = build(path, parts_dir, out_dir)["report"]
    groups = collections.defaultdict(collections.Counter)
    for g, m, fid in mass_items(rep):
        groups[g][fid if fid in FIDS else "derived"] += m
    gtab = {}
    for g, c in sorted(groups.items()):
        tot = math.fsum(c.values())
        gtab[g] = {
            "mass": tot,
            **{k: (c.get(k, 0.0) / tot if tot else 0.0) for k in FIDS},
        }
    allc = collections.Counter()
    for c in groups.values():
        allc.update(c)
    tot = math.fsum(allc.values())
    gtab["(all)"] = {
        "mass": tot,
        **{k: (allc.get(k, 0.0) / tot if tot else 0.0) for k in FIDS},
    }
    est = sorted(
        rep["estimated"], key=lambda e: (e["where"], e["key"], str(e["value"]))
    )
    sev = collections.Counter(f["severity"] for f in rep["findings"])
    codes = collections.Counter((f["code"], f["severity"]) for f in rep["findings"])
    return {
        "name": rep["assembly"],
        "file": str(Path(path).name),
        "mass_total": rep["mass"]["total"],
        "mass_items_total": tot,
        "groups": gtab,
        "targets": rep["targets"],
        "estimated": [
            {k: e.get(k) for k in ("where", "key", "value", "unit", "rationale")}
            for e in est
        ],
        "unresolved": rep["unresolved"],
        "warnings": rep["warnings"],
        "drc": {
            "errors": sev["error"],
            "warnings": sev["warning"],
            "info": sev["info"],
            "codes": [
                {"code": c, "severity": s, "count": n}
                for (c, s), n in sorted(codes.items())
            ],
            "findings": [
                {k: f.get(k) for k in ("code", "severity", "where", "message")}
                for f in rep["findings"]
                if f["severity"] != "info"
            ],
        },
    }


# ---------------------------------------------------------------- models / validation
def stand_validation():
    t = _read("docs/validation/thrust-stands.md")
    rows = _md_table(_section(t, "Per-table results"))
    num = lambda s: float(s.split()[0]) if s and s[0].isdigit() else None
    out = [
        {
            "motor": r["motor"],
            "table": r["table"],
            "prop": r["prop"],
            "held_out": int(r["held-out"]),
            "thrust_mape": num(r["thrust MAPE %"]),
            "current_mape": num(r["current MAPE %"]),
            "rpm_mape": num(r["rpm MAPE %"]),
            "power_mape": r["power MAPE %"],
            "max_err": num(r["max err %"]),
        }
        for r in rows
    ]
    mean = lambda k: (
        sum(r[k] for r in out if r[k] is not None)
        / max(1, sum(r[k] is not None for r in out))
    )
    return {
        "rows": out,
        "mean_thrust_mape": mean("thrust_mape"),
        "mean_current_mape": mean("current_mape"),
        "mean_rpm_mape": mean("rpm_mape"),
        "limits": _bullets(_section(t, "Fidelity limits")),
        "thermal_rows": _md_table(_section(t, "Thermal")),
    }


def hover_validation():
    t = _read("docs/validation/x500-hover.md")
    rows = _md_table(_section(t, "Comparison"))
    dev = _bullets(_section(t, "Deviations"))
    return {
        "rows": rows,
        "not_validated": [b for b in dev if b.startswith("Not validated")],
        "deviations": [b for b in dev if not b.startswith("Not validated")],
    }


def servo_validation():
    try:
        from . import servo_bench as sb

        rows = sb.datasheet_rows()
        checks = [sb.step_checks(v) for v in (6.0, 7.4)]
    except (
        Exception
    ) as e:  # no kernel (napl._napl) on this machine: say so in the report
        return {"available": False, "reason": _trunc(f"{type(e).__name__}: {e}", 160)}
    return {"available": True, "tol_pct": sb.TOL, "rows": rows, "step_checks": checks}


def models(parts_dir):
    mrows = _models_rows()
    out = []
    for name, prefixes, cats, params in MODELS:
        sel = [
            (k, v)
            for k, v in sorted(mrows.items())
            if any(k.startswith(p) for p in prefixes)
        ]
        out.append(
            {
                "name": name,
                "doc": "docs/models.md",
                "rows": [k for k, _ in sel],
                "tests": sorted({t for _, v in sel for t in v["tests"]}),
                "gaps": {k: _trunc(v["simplifications"], 420) for k, v in sel},
                "param_fidelity": param_fidelity(parts_dir, cats, params)
                if cats
                else {},
                "research": [f"docs/research/{f}" for f in dict(RESEARCH)[name]],
            }
        )
    return out


# ---------------------------------------------------------------- generate
def generate(
    parts_dir=None, asm_dir=None, out_dir="build/assemblies", servo=True, date=None
):
    parts_dir = Path(parts_dir or ROOT / "parts")
    asm_dir = Path(asm_dir or ROOT / "assemblies")
    return {
        "date": date,
        "library": library(parts_dir),
        "assemblies": [
            assembly_section(p, parts_dir, out_dir)
            for p in sorted(asm_dir.glob("*.toml"))
        ],
        "models": models(parts_dir),
        "validation": {
            "stand": stand_validation(),
            "hover": hover_validation(),
            "servo": servo_validation()
            if servo
            else {"available": False, "reason": "skipped (--no-servo)"},
        },
        "determinism": [{"test": t, "file": f, "proves": w} for t, f, w in DETERMINISM],
    }


# ---------------------------------------------------------------- markdown
def _lib_md(L):
    o = [
        "## 1. Parts library",
        "",
        "Fidelity tags: `datasheet` / `literature` / `measured` = sourced (a quote in the part file); `derived` = computed from "
        "sourced values (derivation in the part file); `estimated` = no source (rationale in the part file). "
        "Counted as `tools/check_parts.py` counts (params, tables, port ratings).",
        "",
        f"**{L['parts']} parts, {L['values']} tagged values: {_pct(L['sourced_pct'] / 100)} % sourced, "
        f"{_pct(L['derived_pct'] / 100)} % derived, {_pct(L['estimated_pct'] / 100)} % estimated.**",
        "",
        _table(
            ["fidelity", "values", "%"],
            [
                (k, L["by_fidelity"][k], _pct(L["by_fidelity"][k] / L["values"]))
                for k in FIDS
            ],
        ),
        "",
        "### By directory",
        "",
    ]
    head = ["", "parts", "values", *FIDS, "sourced %", "derived %", "estimated %"]
    row = lambda k, r: (
        k,
        r["parts"],
        r["values"],
        *[r["by_fidelity"][f] for f in FIDS],
        f"{r['sourced_pct']:.1f}",
        f"{r['derived_pct']:.1f}",
        f"{r['estimated_pct']:.1f}",
    )
    o += [
        _table(["directory"] + head[1:], [row(k, r) for k, r in L["by_dir"].items()]),
        "",
        "### By category (top 15 by number of estimated values)",
        "",
    ]
    top = sorted(
        L["by_category"].items(),
        key=lambda kv: (-kv[1]["by_fidelity"]["estimated"], kv[0]),
    )[:15]
    o += [
        _table(["category"] + head[1:], [row(k, r) for k, r in top]),
        "",
        "All categories are in `build/fidelity.json` (`library.by_category`).",
        "",
    ]
    return o


def _asm_md(a, n):
    o = [
        f"### 2.{n} `{a['file']}` ({a['name']})",
        "",
        f"Built with `napl.design.build`; mass total {a['mass_total']:.4f} kg (items sum {a['mass_items_total']:.4f} kg). "
        "Mass-weighted fidelity of every mass item (instances, fastener sets, wires, connectors):",
        "",
        _table(
            [
                "group",
                "mass kg",
                "datasheet %",
                "literature %",
                "measured %",
                "derived %",
                "estimated %",
            ],
            [
                (g, f"{r['mass']:.4f}", *[_pct(r[k]) for k in FIDS])
                for g, r in a["groups"].items()
            ],
        ),
        "",
    ]
    if a["targets"]:
        o += [
            "Mass targets (published figures):",
            "",
            _table(
                [
                    "target",
                    "scope",
                    "model kg",
                    "published kg",
                    "gap %",
                    "tolerance kg",
                    "within",
                    "estimated share %",
                ],
                [
                    (
                        t["name"],
                        ",".join(t["scope"]) if t["scope"] != "all" else "all",
                        f"{t['model']:.4f}",
                        f"{t['target']:.4f}",
                        f"{t['gap_pct']:+.1f}" if t["gap_pct"] is not None else "-",
                        f"{t['tolerance']:.4f}",
                        "yes" if t["within"] else "NO",
                        _pct(t["estimated_share"]),
                    )
                    for t in a["targets"]
                ],
            ),
            "",
        ]
    o += [
        f"**Estimated values used by this build: {len(a['estimated'])}** (every one; rationale truncated, "
        "full text in the part file or `<name>.report.json`):",
        "",
    ]
    o += [
        _table(
            ["instance", "key", "value", "rationale"],
            [
                (
                    e["where"],
                    e["key"],
                    f"{e['value']:.6g} {e['unit']}"
                    if isinstance(e["value"], (int, float))
                    else f"{e['value']} {e['unit'] or ''}",
                    _trunc(e["rationale"] or "", 150),
                )
                for e in a["estimated"]
            ],
        )
        if a["estimated"]
        else "(none)",
        "",
    ]
    o += ["Unresolved items: " + ("none." if not a["unresolved"] else ""), ""]
    o += [
        f"- {_trunc(u if isinstance(u, str) else json.dumps(u, sort_keys=True), 200)}"
        for u in a["unresolved"]
    ]
    d = a["drc"]
    o += [
        "",
        f"Design-rule check: {d['errors']} errors, {d['warnings']} warnings, {d['info']} info.",
        "",
    ]
    if d["codes"]:
        o += [
            _table(
                ["code", "severity", "count"],
                [(c["code"], c["severity"], c["count"]) for c in d["codes"]],
            ),
            "",
        ]
    if d["findings"]:
        o += [
            _table(
                ["code", "severity", "where", "message"],
                [
                    (
                        f["code"],
                        f["severity"],
                        ", ".join(f["where"] or []),
                        _trunc(f["message"], 170),
                    )
                    for f in d["findings"]
                ],
            ),
            "",
        ]
    return o


def _model_md(m, V, n):
    o = [
        f"### 3.{n} {m['name']}",
        "",
        f"Equations: [{m['doc']}]({m['doc'].replace('docs/', '')}) rows "
        + ", ".join(f"`{r}`" for r in m["rows"])
        + ". "
        "Unit tests: " + (", ".join(f"`{t}`" for t in m["tests"]) or "none") + ".",
        "",
    ]
    if m["param_fidelity"]:
        o += [
            "Parameter provenance (library parts feeding this model; part counts per fidelity tag):",
            "",
            _table(
                ["param", *FIDS],
                [
                    (k, *[v.get(f, "") for f in FIDS])
                    for k, v in m["param_fidelity"].items()
                ],
            ),
            "",
        ]
    elif m["name"] == "environment":
        o += [
            "Parameters: ISA constants (standard), MIL-F-8785C Dryden low-altitude spectra (literature, in the code, not in "
            "the part library); no part file feeds it. Wind / turbulence level is a scenario input.",
            "",
        ]
    o += _validation_md(m["name"], V)
    o += ["Known gaps (docs/models.md, simplifications column, truncated):", ""]
    o += [f"- **{k}**: {v}" for k, v in m["gaps"].items()]
    o += [
        "",
        "Research notes: "
        + ", ".join(
            f"[{Path(r).name}](research/{Path(r).name})" for r in m["research"]
        ),
        "",
    ]
    return o


def _validation_md(name, V):
    S, H, SV = V["stand"], V["hover"], V["servo"]
    o = ["Validation evidence:", ""]
    hov = lambda *keys: [
        r for r in H["rows"] if any(k in r.get("quantity", "") for k in keys)
    ]
    if name == "battery":
        o += [
            "- No discharge-curve comparison against measured packs. The OCV curve is literature (Chen 2006); r0 / r1 / c1 are "
            "estimated (see table above), so voltage sag is not validated.",
            "- X500 hover/endurance vs the vendor claim (docs/validation/x500-hover.md, `Comparison`):",
            "",
            _table(
                [
                    "quantity",
                    "PX4 flies the assembly",
                    "bench hover",
                    "2216 vendor table",
                    "Holybro claim",
                ],
                [
                    (
                        r["quantity"],
                        r.get("PX4 flies the assembly", ""),
                        r.get(
                            "bench hover (2216 stand fit, assembly AUW / battery / loads)",
                            "",
                        ),
                        r.get("2216 vendor table", ""),
                        r.get("Holybro claim", ""),
                    )
                    for r in hov("pack", "endurance")
                ],
            ),
            "",
        ]
    elif name in ("motor + ESC", "rotor (aero)"):
        col = "current_mape" if name.startswith("motor") else "thrust_mape"
        o += [
            f"- Held-out rows only (3 training rows per table; the fit is C_T0, C_Q0, duty map). Mean over {len(S['rows'])} tables: "
            f"thrust {S['mean_thrust_mape']:.1f} %, current {S['mean_current_mape']:.1f} %, rpm {S['mean_rpm_mape']:.1f} % (MAPE).",
            "",
            _table(
                [
                    "motor",
                    "table",
                    "held-out rows",
                    "thrust MAPE %",
                    "current MAPE %",
                    "rpm MAPE %",
                    "power MAPE %",
                    "max err %",
                ],
                [
                    (
                        r["motor"],
                        r["table"],
                        r["held_out"],
                        *[
                            ("-" if r[k] is None else f"{r[k]:.1f}")
                            for k in ("thrust_mape", "current_mape", "rpm_mape")
                        ],
                        r["power_mape"],
                        "-" if r["max_err"] is None else f"{r['max_err']:.1f}",
                    )
                    for r in S["rows"]
                ],
            ),
            "",
            "- X500 V2 hover against Holybro / the vendor table (the stand fit is NOT trained on these):",
            "",
            _table(
                [
                    "quantity",
                    "PX4 flies the assembly",
                    "bench hover",
                    "2216 vendor table",
                ],
                [
                    (
                        r["quantity"],
                        r.get("PX4 flies the assembly", ""),
                        r.get(
                            "bench hover (2216 stand fit, assembly AUW / battery / loads)",
                            "",
                        ),
                        r.get("2216 vendor table", ""),
                    )
                    for r in hov("throttle", "current", "rpm")
                ],
            ),
            "",
            "Known gaps from docs/validation/thrust-stands.md (`Fidelity limits`):",
            "",
        ]
        o += [f"- {_trunc(b, 380)}" for b in S["limits"]]
        o += [
            "",
            "Not validated in hover: "
            + " ".join(
                _trunc(b.removeprefix("Not validated here: "), 300)
                for b in H["not_validated"]
            ),
            "",
        ]
    elif name == "servo":
        if SV["available"]:
            o += [
                f"- STS3215 C001 vs datasheet (tolerance +-{SV['tol_pct']:g} %). Every comparison is circular: the part's derived "
                "parameters were fitted to those very points, so agreement shows the model reproduces its inputs, not that it is "
                "independently validated:",
                "",
                _table(
                    ["quantity", "V", "sim", "datasheet", "err %", "pass", "kind"],
                    [
                        (
                            r["quantity"],
                            f"{r['V']:g}",
                            f"{r['sim']:.4g}",
                            f"{r['datasheet']:.4g}",
                            f"{r['err_pct']:+.1f}",
                            "n/a"
                            if r["pass"] is None
                            else ("yes" if r["pass"] else "NO"),
                            r["kind"],
                        )
                        for r in SV["rows"]
                    ],
                ),
                "",
                "- 60 deg full-duty step checks (model-internal consistency):",
                "",
                _table(
                    [
                        "V",
                        "t s",
                        "t floor s",
                        "excess s",
                        "tau = J/b s",
                        "peak / no-load",
                    ],
                    [
                        (
                            f"{c['V']:g}",
                            f"{c['t']:.4f}",
                            f"{c['t_ds']:.4f}",
                            f"{c['excess']:.4f}",
                            f"{c['tau']:.4f}",
                            f"{c['peak_ratio']:.3f}",
                        )
                        for c in SV["step_checks"]
                    ],
                ),
                "",
            ]
        else:
            o += [
                f"- Servo bench not run for this report ({SV['reason']}). Run `python -m napl.servo_bench` on a machine with the kernel.",
                "",
            ]
        o += [
            "- No measured step / stall data from a real SO-101; the PID register scaling (`pid_scale`) is a calibration knob "
            "(docs/research/sts3215-calibration.md).",
            "",
        ]
    elif name == "sensors":
        o += [
            "- Validated against closed forms only (static noise statistics, quantisation, pendulum / centripetal truth, ISA pressure, "
            "WGS84 flat-earth offset): the model implements the datasheet numbers, no hardware recording was compared.",
            "- Noise parameters are datasheet values; vibration, temperature drift and multipath are not modelled. GNSS position error is white unless a device sets corr_time_s (the X500 assembly does: Gauss-Markov part, estimated split).",
            "",
        ]
    else:
        o += [
            "- ISA density / pressure and the Dryden filter are checked against their closed-form definitions "
            "(`environment_isa_density`, `environment_dryden_turbulence`); no flight-test comparison.",
            "",
        ]
    return o


def _det_md(D):
    o = [
        "## 4. Determinism and replay evidence",
        "",
        "What is saved, hashed and replayed is described in docs/models.md `Determinism`. Tests that prove it "
        "(ctest `suite.name`, or pytest `file::test`):",
        "",
    ]
    return o + [
        _table(
            ["test", "file", "proves"], [(d["test"], d["file"], d["proves"]) for d in D]
        ),
        "",
    ]


def to_markdown(data):
    L, V = data["library"], data["validation"]
    o = [
        "# Fidelity report: how real is this sim",
        "",
        "Generated by `napl report fidelity` (python/napl/fidelity.py). Do not edit by hand."
        + (f" Date: {data['date']}." if data["date"] else ""),
        "",
        "Every number in a design traces to a part file (`parts/`), where it carries a fidelity tag. This page counts those tags, "
        "shows which estimated values each assembly actually uses, and states per model how it was validated and where it is known "
        "to be wrong. Numbers in sections 3 come from `docs/validation/*.md` (thrust-stand bench, X500 hover) and the servo bench; "
        "they are not recomputed here.",
        "",
    ]
    o += _lib_md(L)
    o += ["## 2. Assemblies", ""]
    for i, a in enumerate(data["assemblies"], 1):
        o += _asm_md(a, i)
    o += ["## 3. Models", ""]
    for i, m in enumerate(data["models"], 1):
        o += _model_md(m, V, i)
    o += _det_md(data["determinism"])
    return "\n".join(o).rstrip() + "\n"


def main_report(args) -> int:
    data = generate(
        args.parts, None, args.build_dir, servo=not args.no_servo, date=args.date
    )
    out, js = Path(args.out), Path(args.json)
    out.parent.mkdir(parents=True, exist_ok=True)
    js.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(to_markdown(data), encoding="utf-8", newline="\n")
    js.write_text(
        json.dumps(data, indent=2, sort_keys=True, default=str) + "\n",
        encoding="utf-8",
        newline="\n",
    )
    L = data["library"]
    print(
        f"fidelity: {out} + {js}: {L['parts']} parts, {L['values']} values ({L['estimated_pct']:.1f} % estimated), "
        f"{len(data['assemblies'])} assemblies"
    )
    return 0
