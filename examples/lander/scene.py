"""Scene of the TVC lander: assemblies/tvc-lander.toml standing on its legs on the floor. `napl program` calls build(outdir) -> MJCF path;
land.py and mc.py call build_seeded() for a per-run noise seed and Dryden intensity (the device seeds and `w20` are build-time constants)."""

from pathlib import Path

from napl import design
from napl.design.assembly import load

REPO = Path(__file__).resolve().parents[2]
ASSEMBLY = REPO / "assemblies" / "tvc-lander.toml"
_LIB = None


def build_seeded(out, seed=1, w20=None):
    """Compute the MJCF of the lander with every noisy device seeded `seed` and the Dryden intensity `w20` (m/s; None = the assembly's value)."""
    global _LIB
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    asm = load(ASSEMBLY)
    asm.doc["assembly"]["seed"] = int(seed)
    if w20 is not None:
        env = next(d for d in asm.doc["devices"] if d["type"] == "environment")
        env["set"]["w20"]["value"] = float(w20)
    if _LIB is None:
        _LIB = design.Library(REPO / "parts")
    xml, _ = design.compute(asm, REPO / "parts", out, lib=_LIB, visual=False)
    (out / "scene.xml").write_text(xml)
    return out / "scene.xml"


def build(out):
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    xml, _ = design.compute(ASSEMBLY, REPO / "parts", out)
    (out / "scene.xml").write_text(xml)
    return out / "scene.xml"
