"""napl.explore: part sweep (3 batteries, deterministic), seeded wind scenarios, failure detection. Needs the kernel."""

from pathlib import Path

import napl
import pytest
from napl import explore

ASM = Path(__file__).resolve().parents[2] / "assemblies" / "x500v2.toml"
needs_kernel = pytest.mark.skipif(napl._napl is None, reason="kernel not built")
BATS = [
    "tattu-4s-5200mah-35c",  # the baseline itself is not a candidate; these three are
    "ovonic-4000mah-4s-130c-xt60",
    "gnb-3000mah-4s-100c-xt60",
    "tattu-1800mah-4s-75c-xt60",
]
FAST = {"hold_s": 4.0, "climb_s": 3.0}


def test_fit_filter_drops_cells_and_keeps_packs():
    """no kernel needed: the planner keeps XT60 packs (re-seated on the rails), drops bare cells."""
    from napl.design import Design

    d = Design.open(ASM)
    rows = {r["part"]: r for r in d.suggest("bat", limit=500)}
    ok = explore._plan(d, ["bat"], rows["ovonic-4000mah-4s-130c-xt60"])
    assert ok[3] is None and ok[1]  # adapter for the rail mates
    assert {f["param"] for f in ok[2]} >= {"thevenin_r1", "discharge_wire_length"}
    assert explore._plan(d, ["bat"], rows["sony-us18650vtc6"])[
        3
    ]  # dropped, with a reason


@needs_kernel
def test_sweep_three_batteries_deterministic():
    kw = dict(asm=ASM, ids=["bat"], only=BATS[1:], hover=FAST)
    a = explore.sweep(**kw, workers=1)
    b = explore.sweep(**kw, workers=2)
    assert [r["part"] for r in a["rows"]] == sorted(BATS[1:])
    assert all(r["flight_min"] for r in a["rows"])
    assert a["rows"] == b["rows"]  # same numbers, serial or forked


@needs_kernel
def test_wind_reproducible_with_seed():
    d = explore.Design.open(ASM)
    explore._CTX.update(
        d=d, base=__import__("copy").deepcopy(d.doc), cda=explore.CDA_DEFAULT
    )
    hov = {"hold_s": 6.0, "climb_s": 3.0, "tol": 0.5}
    sc = {
        "name": "t",
        "wind": 3.0,
        "w20": explore.TURBULENCE_W20["moderate"],
        "seed": 7,
    }
    a = explore._scenario((sc, hov))
    b = explore._scenario((sc, hov))
    c = explore._scenario(({**sc, "seed": 8}, hov))
    assert a == b
    assert a["xy_error_max_m"] != c["xy_error_max_m"]


@needs_kernel
def test_failure_detected_at_absurd_wind():
    d = explore.Design.open(ASM)
    explore._CTX.update(
        d=d, base=__import__("copy").deepcopy(d.doc), cda=explore.CDA_DEFAULT
    )
    hov = {"hold_s": 6.0, "climb_s": 3.0, "tol": 0.5}
    calm = explore._scenario(({"name": "calm"}, hov))
    gale = explore._scenario(({"name": "gale", "wind": 60.0}, hov))
    assert not calm["failed"]
    assert gale["failed"]
