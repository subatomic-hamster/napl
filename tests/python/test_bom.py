"""BOM roll-up (python/napl/design/bom.py): lines, per-sub-assembly subtotals, missing prices."""

import pytest
from napl import design
from napl.design import assembly as A
from test_subasm import BOT, LEG, make_parts, port, prm, write, write_part


def usd(v, fid="datasheet"):
    if fid == "estimated":
        return {"value": v, "unit": "USD", "fidelity": fid, "rationale": "test"}
    return prm(v, "USD", fid)


def report(tmp_path):
    parts = make_parts(tmp_path)  # plate / brk / mot: no prices yet
    ports = lambda name: None

    write_part(
        parts,
        "plate",
        {"mass": prm(0.1), "price": usd(10.0)},
        [
            port("mount_l", "male", (0.1, 0.1, 0)),
            port("mount_r", "male", (0.1, -0.1, 0)),
        ],
    )
    write_part(
        parts,
        "brk",
        {"mass": prm(0.1), "price": usd(2.0, "estimated")},
        [port("base", "female"), port("out", "male")],
    )
    write(tmp_path / "leg.toml", LEG)
    p = write(tmp_path / "bot.toml", BOT.replace("{l_extra}", ""))
    return design.compute(A.load(p), parts, tmp_path / "out")[1]


def test_bom_totals_missing_and_subtotals(tmp_path):
    b = report(tmp_path)["bom"]
    lines = {x["part"]: x for x in b["lines"]}
    assert (
        lines["brk"]["count"] == 2
        and lines["brk"]["unit_price"] == 2.0
        and lines["brk"]["fidelity"] == "estimated"
    )
    assert lines["plate"]["count"] == 1
    assert b["missing"] == ["mot"] and lines["mot"]["unit_price"] is None
    # cost = sum of the priced lines, the unpriced motors contribute nothing
    assert b["total"]["cost"] == pytest.approx(
        sum(
            x["count"] * x["unit_price"]
            for x in b["lines"]
            if x["unit_price"] is not None
        )
    )
    assert b["total"]["cost"] == pytest.approx(14.0) and b["total"][
        "estimated_share"
    ] == pytest.approx(4 / 14)
    # per-sub subtotals + the parent's own parts = everything counted
    assert set(b["by_sub"]) == {"l", "r"}
    assert b["by_sub"]["l"] == {
        "mass": pytest.approx(0.2),
        "cost": 2.0,
        "missing": ["mot"],
    }
    assert sum(s["cost"] for s in b["by_sub"].values()) + b["total"][
        "parent_cost"
    ] == pytest.approx(b["total"]["cost"])
    assert sum(s["mass"] for s in b["by_sub"].values()) + b["total"][
        "parent_mass"
    ] == pytest.approx(0.5)
    assert b["total"]["mass"] == pytest.approx(0.5)


def test_format_budget_prints_cost(tmp_path):
    text = design.format_budget(report(tmp_path))
    assert "cost: 14.00 USD" in text and "no price: mot" in text
