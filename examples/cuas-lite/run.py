"""Lockstep harness of the cuas-lite scenarios: World(ctl_ticks=5) + SimRP2040 (LoopbackLink) + SimCamera + TruthDetector + the real App, one process,
deterministic, CPU only. Run through `napl program examples/cuas-lite/s1_balloon_static.scenario.toml` (sets NAPL_MODEL / NAPL_RUN_DIR).

Env: CUAS_SIM_S (default 60), CUAS_SEED, CUAS_SCENARIO (see scene.py), CUAS_ARM_AT (s, default 8: link up and homed by then).
World.finish() writes trace.csv (monitor columns of napl.cuas.world: rounds, hits, fires_unauthorized ...), replay.naplog, native.json.
This harness adds lite.json: the lite-specific safety checks (every ball fired while armed, inside the sector, within the cap) and the pops;
exit status 1 if one of them failed."""

import json
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

import scene as sc
from napl.lite import protocol as p
from napl.lite.sim_io import Lockstep


def main():
    run_dir = Path(os.environ["NAPL_RUN_DIR"])
    targets = sc.SCENARIOS[sc.scenario()]
    h = Lockstep(
        dict(model=os.environ["NAPL_MODEL"]),
        run_dir,
        targets,
        sector=sc.SECTOR,
        seed=sc.seed(),
    )
    h.hold_deadman = True
    h.script.append(
        (float(os.environ.get("CUAS_ARM_AT", 8.0)), lambda s: s.app.post("arm"))
    )
    bad, seen, cap = dict(outside=0, unarmed=0, over_cap=0), 0, h.app.fc.cfg.shot_cap

    def monitor(s):
        nonlocal seen
        for _ in s.twin.shots[seen:]:
            pan, tilt = s.plant.joint_deg()
            bad["outside"] += not s.app.fc.sector.contains(pan, tilt)
            bad["unarmed"] += not s.app.fc.armed
        seen = len(s.twin.shots)
        bad["over_cap"] = max(bad["over_cap"], s.twin.shots_arm - cap)
        return False

    h.run(float(os.environ.get("CUAS_SIM_S", 60.0)), until=monitor)
    nat = h.finish()
    out = dict(
        shots=len(h.twin.shots),
        rounds=h.world.rounds,
        popped=sorted(h.popped),
        bad=bad,
        state_hash=nat["state_hash"],
        app_failed=h.app.failed,
        mcu=p.STATE_NAMES[h.twin.state],
    )
    (run_dir / "lite.json").write_text(json.dumps(out, indent=1))
    sys.stdout.write(json.dumps(out) + "\n")
    sys.exit(1 if any(bad.values()) or h.app.failed else 0)


if __name__ == "__main__":
    main()
