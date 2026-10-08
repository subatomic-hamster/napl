"""Scene of the rocket hop: assemblies/rocket-hopper.toml standing on its legs on the floor. `napl program` calls build(outdir) -> MJCF path."""

from pathlib import Path

from napl import design

REPO = Path(__file__).resolve().parents[2]


def build(out):
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    xml, _ = design.compute(
        REPO / "assemblies" / "rocket-hopper.toml", REPO / "parts", out
    )
    (out / "scene.xml").write_text(xml)
    return out / "scene.xml"
