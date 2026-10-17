"""Scene of the dog stand / squat sanity: assemblies/dog/dog.toml on the design builder's floor. `napl program` calls build(outdir) -> MJCF path."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import dogsim


def build(outdir):
    out = Path(outdir)
    out.mkdir(parents=True, exist_ok=True)
    return str(dogsim.build(out)[0])
