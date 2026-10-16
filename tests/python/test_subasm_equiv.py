"""Split-equivalence: assemblies/rover.toml vs the same rover as a parent + two wheel-module sub-assemblies builds to the same physics."""

import re
from pathlib import Path

import napl
from napl import design
from napl.design import assembly as A

ROOT = Path(__file__).resolve().parents[2]
DATA = ROOT / "tests" / "data" / "subasm"
RENAME = {  # flattened id -> the monolithic rover's id
    "wl:bracket": "bracket_l",
    "wl:motor": "motor_l",
    "wl:hub": "hub_l",
    "wl:tire": "tire_l",
    "wr:bracket": "bracket_r",
    "wr:motor": "motor_r",
    "wr:hub": "hub_r",
    "wr:tire": "tire_r",
    "wl:wheel": "wheel_l",
    "wr:wheel": "wheel_r",
}


def canonical(xml):
    for k, v in RENAME.items():
        xml = xml.replace(k, v)
    return re.sub(r"\s+", " ", xml)


def hash_after(xml, tmp, ticks):
    f = tmp / "m.xml"
    f.write_text(xml)
    sim = napl.Sim.load(f)
    sim.step(ticks)
    return sim.state_hash()


def test_split_rover_builds_identical(tmp_path):
    a, ra = design.compute(
        ROOT / "assemblies/rover.toml", ROOT / "parts", tmp_path / "a"
    )
    asm = A.load(DATA / "rover_split/rover.toml")
    b, rb = design.compute(asm, ROOT / "parts", tmp_path / "b")
    assert asm.origins["wl:motor"] == {
        "sub": "wl",
        "file": "wheel_l.toml",
        "id": "motor",
    }
    assert ra["mass"]["total"] == rb["mass"]["total"]
    (tmp_path / "a").mkdir(exist_ok=True)
    (tmp_path / "b").mkdir(exist_ok=True)
    assert hash_after(a, tmp_path / "a", 2000) == hash_after(
        canonical(b), tmp_path / "b", 2000
    )
