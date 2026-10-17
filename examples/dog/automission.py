"""Runner of the dog autonomy missions (examples/dog/missions/*/*.scenario.toml).

    napl program examples/dog/missions/coverage_smoke/coverage_smoke.scenario.toml --runs-dir <dir>
    python examples/dog/automission.py <scenario> <out dir>            # standalone; NAPL_PARAMS='{"seed": 7}' picks the seed
AUTO_T_MAX (s) caps the run (tests)."""

import json
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from autonomy import scene


def main():
    path = Path(sys.argv[1]).resolve()
    if "NAPL_MODEL" in os.environ:  # under `napl program`
        model, out = Path(os.environ["NAPL_MODEL"]), Path(os.environ["NAPL_RUN_DIR"])
    else:
        out = Path(sys.argv[2])
        out.mkdir(parents=True, exist_ok=True)
        model = scene.build(out / "model", path)
    t = os.environ.get("AUTO_T_MAX")
    s = scene.run(model, path, out, float(t) if t else None)
    sys.stdout.write(json.dumps(s) + "\n")


if __name__ == "__main__":
    main()
