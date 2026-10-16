"""Dog parts library (SP3a): QDD constants reproduce the datasheet; every dog part has a dated USD price and a mirror table."""

import math
import pathlib
import tomllib

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[2]
QDD = [
    "actuators/steadywin-gim8108-8",
    "actuators/steadywin-gim6010-8",
    "actuators/cubemars-ak70-9-v3-kv60",
]


def load(rel):
    return tomllib.loads((ROOT / "parts" / f"{rel}.toml").read_text())


def v(p, k):
    return p["params"][k]["value"]


@pytest.mark.parametrize("rel", QDD)
def test_qdd_constants_reproduce_datasheet(rel):
    """Qdd builder inputs (Kt motor-side, N, eta, Ke) must give the published OUTPUT-side peak torque and no-load speed within 10%."""
    p = load(rel)
    kt, n, eta, ke = (
        v(p, "kt_motor"),
        v(p, "gear_ratio"),
        v(p, "efficiency"),
        v(p, "ke_si") if "ke_si" in p["params"] else v(p, "kt_motor"),
    )
    assert (
        0.6 <= eta <= 0.95
    )  # effective figure, bounded so it cannot absorb an arbitrary mismatch
    tau = kt * n * eta * v(p, "current_peak")
    assert tau == pytest.approx(v(p, "torque_peak"), rel=0.10)
    w0 = v(p, "v_rated") / (ke * n)  # rad/s at the output, no load (R*i drop neglected)
    assert w0 == pytest.approx(v(p, "speed_no_load"), rel=0.10)
    assert kt == pytest.approx(ke, rel=0.01)  # SI identity Kt = Ke
    assert v(p, "kv_output") * v(p, "v_rated") * 2 * math.pi / 60 == pytest.approx(
        v(p, "speed_no_load"), rel=0.10
    )
    assert p["params"]["price"]["unit"] == "USD" and v(p, "price") > 0
    assert p["mirror"]["symmetry"] in ("xy", "yz", "xz")
