"""The whole dog (assemblies/dog/dog.toml): DRC clean, mass ~12 kg with the CoM between the hips, BOM with every price, mirrored and
rotated legs in place, and every joint stalling at its datasheet torque from the real pack (Review Focus 2)."""

import pathlib
import re

import napl
import numpy as np
import pytest
from napl import design
from napl.design import Library
from napl.design import assembly as A
from napl.design.drc import check

ROOT = pathlib.Path(__file__).resolve().parents[2]
PARTS, DOG = ROOT / "parts", ROOT / "assemblies" / "dog"
LIB = Library(PARTS)
LEGS = ("fl", "fr", "rl", "rr")
needs_kernel = pytest.mark.skipif(napl._napl is None, reason="kernel not built")


@pytest.fixture(scope="module")
def built(tmp_path_factory):
    return design.build(DOG / "dog.toml", PARTS, tmp_path_factory.mktemp("dog"))


def test_dog_drc_has_no_errors():
    fs = check(A.load(DOG / "dog.toml"), LIB)
    assert [(f.code, f.message) for f in fs if f.severity == "error"] == []


def test_mass_bom_and_readme(built):
    rep = built["report"]
    assert 10.0 <= rep["mass"]["total"] <= 13.0  # spec s1: ~12 kg
    bom = rep["bom"]
    assert (
        bom["missing"] == []
    )  # every part priced (bom.py: wires / connectors are not counted, README says so)
    assert set(bom["by_sub"]) == {"fl", "fr", "rl", "rr", "head", "power", "compute"}
    readme = (DOG / "README.md").read_text()
    m = re.search(r"BOM total \(lower bound, wires excluded\): ([0-9,]+) USD", readme)
    assert (
        m and abs(int(m.group(1).replace(",", "")) - round(bom["total"]["cost"])) <= 1
    )


def test_mirrors_and_knee_ratio_override():
    asm = A.load(DOG / "dog.toml")
    subs = {s["id"]: s for s in asm.raw["subassemblies"]}
    assert subs["fr"].get("mirror") == "xz" and subs["rl"].get("mirror") == "xz"
    assert "mirror" not in subs["fl"] and "mirror" not in subs["rr"]
    p = DOG / "dog.toml"
    text = p.read_text().replace(
        'id = "rl"\n',
        'id = "rl"\nparams = { knee_drive_ratio = { value = 16.0, unit = "1", fidelity = "derived", derivation = "test" } }\n',
        1,
    )
    flat = A.loads(text, path=p).doc
    by = {i["id"]: i for i in flat["instances"]}
    assert by["rl:knee_motor"]["params"]["drive_ratio"]["value"] == 16.0
    assert by["fl:knee_motor"]["params"]["drive_ratio"]["value"] == 12.0


@needs_kernel
def test_feet_touch_the_floor_and_com_is_between_the_hips(built):
    sim = napl.Sim.load(built["xml"])
    pos = {p.rsplit("/", 1)[-1]: np.asarray(x) for p, x in sim.body_positions().items()}
    r_ball = LIB.get("mcmaster-natural-rubber-ball-1-3-8in").geometry["shapes"][0][
        "size"
    ][0]
    for leg in LEGS:
        assert (
            0.0 <= pos[f"{leg}:foot_ball"][2] - r_ball <= 0.002
        )  # build pose: straight legs standing on the floor
    rep = built["report"]
    m = {r["id"]: r["mass"] for r in rep["instances"] if r["id"] in pos}
    com = sum(m[i] * pos[i] for i in m) / sum(
        m.values()
    )  # point masses at the part frames: an approximation, README says so
    hips = np.mean([pos[f"{leg}:abad_mount"] for leg in LEGS], axis=0)
    assert np.linalg.norm((com - hips)[:2]) <= 0.02


@needs_kernel
def test_every_joint_stalls_at_its_datasheet_torque(built):  # Review Focus 2
    sim = napl.Sim.load(built["xml"])
    act = LIB.get("steadywin-gim8108-8")
    r = LIB.get("dog-shank").value("lug_radius") / LIB.get("dog-knee-crank-l").value(
        "crank_radius"
    )
    for leg in LEGS:
        for j, ratio in (("abad", 1.0), ("hip", 1.0), ("knee", r)):
            s = sim.fork()
            d = f"/devices/{leg}:{j}"
            s.set(d, "enable", 1.0)
            s.set(
                d, "tau_ff", 2.0 * act.value("torque_peak")
            )  # saturating: the current limit decides
            s.step(3)  # 1 tick of CAN latency + 1 command period
            tau = float(s.query(d, "tau_est")[0][0])
            assert abs(tau) == pytest.approx(
                act.value("torque_peak") * ratio, rel=0.05
            ), (d, tau)
            assert (
                float(s.query(d, "voltage")[0][0]) > 8 * 3.6
            )  # fed from the packs through contactor, PDB, hub, cable
