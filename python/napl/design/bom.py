"""Bill of materials: a pure function of the build report + library (+ the sub-assembly origins of the flat document).

Lines are the placed instances and the fasteners of the mates (screws and nuts); wires, connectors and their prices are not
counted (a wire part's price would be per metre; it is not modelled yet). A part's unit price is its `price` param, in USD
(tools/check_parts.py enforces the unit); a part without one is listed in `missing` and contributes nothing to the cost, so
`total.cost` is a lower bound whenever `missing` is not empty.

    {"lines": [{part, count, unit_price, fidelity}], "by_sub": {sub: {mass, cost, missing}}, "total": {mass, cost,
     estimated_share, parent_mass, parent_cost}, "missing": [part ids]}

`by_sub` is keyed by the outermost sub-assembly id ("fl" also holds "fl:knee:motor"). `total.mass` is the build report's total
(wires included); `parent_mass` / `parent_cost` are the lines that belong to no sub-assembly, so by_sub + parent = the counted
instances and fasteners.
"""


def _unit_price(lib, pid):
    """-> (USD, fidelity) or None."""
    if pid not in lib.parts:
        return None
    e = lib.get(pid).params.get("price")
    v = e.get("value") if isinstance(e, dict) else None
    if isinstance(v, bool) or not isinstance(v, (int, float)) or e.get("unit") != "USD":
        return None
    return float(v), e.get("fidelity", "")


def bom(report: dict, lib, origins: dict | None = None) -> dict:
    origins = origins or {}
    rows = []  # (part id, count, mass, sub or "")
    for r in report["instances"]:
        if r.get("offboard"):
            continue
        rows.append((r["part"], 1, r["mass"], origins.get(r["id"], {}).get("sub", "")))
    for f in report.get("fasteners", []):
        sub = origins.get(f["mate"], {}).get("sub", "")
        rows.append(
            (f["part"], f["count"], f["mass"], sub)
        )  # the screws carry the mate's fastener mass (screw + nut)
        if f.get("nut"):
            rows.append((f["nut"], f["count"], 0.0, sub))
    lines, by_sub, missing = {}, {}, []
    est_cost = cost = 0.0
    parent = {"mass": 0.0, "cost": 0.0}
    for pid, n, mass, sub in rows:
        up = _unit_price(lib, pid)
        top = sub.split(":")[0]
        tgt = (
            by_sub.setdefault(top, {"mass": 0.0, "cost": 0.0, "missing": []})
            if top
            else parent
        )
        tgt["mass"] += mass
        line = lines.setdefault(
            pid,
            {
                "part": pid,
                "count": 0,
                "unit_price": up[0] if up else None,
                "fidelity": up[1] if up else None,
            },
        )
        line["count"] += n
        if up is None:
            if pid not in missing:
                missing.append(pid)
            if top and pid not in tgt["missing"]:
                tgt["missing"].append(pid)
            continue
        c = n * up[0]
        tgt["cost"] += c
        cost += c
        if up[1] == "estimated":
            est_cost += c
    return {
        "lines": sorted(lines.values(), key=lambda x: x["part"]),
        "by_sub": dict(sorted(by_sub.items())),
        "total": {
            "mass": report["mass"]["total"],
            "cost": cost,
            "estimated_share": est_cost / cost if cost else 0.0,
            "parent_mass": parent["mass"],
            "parent_cost": parent["cost"],
        },
        "missing": sorted(missing),
    }
