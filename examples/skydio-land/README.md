# Skydio landing: take off, fly, and land somewhere else

`examples/skydio-land` is the Skydio-style vision-only quadrotor of `examples/skydio` (Holybro X500 V2, Pixhawk 6C, RealSense D435i, Raspberry Pi 5; ESKF on IMU + VIO + baro, voxel map,
sampled-primitive planner, `cuas.flyer.Flyer`) taught to LAND: in other places than it took off, avoiding obstacles, in different environments. Nothing of `examples/skydio` or
`assemblies/skydio.toml` is edited: the code imports it (`Skydio`, `mapping`, `planner`, the forest) and `git diff` against the parent branch is empty for those paths (test
`test_the_originals_are_untouched`).

```
tools/remote.sh run '.venv/bin/napl program examples/skydio-land/summit/summit.scenario.toml --runs-dir build/runs'    # one scenario: PASS + replay hash match
NAPL_PARAMS='{"seed": 1004, "wind_ms": 8}' .venv/bin/napl program examples/skydio-land/summit/summit.scenario.toml --runs-dir build/runs   # any seed / any parameter of the scenario
tools/remote.sh run '.venv/bin/python examples/skydio-land/land_mc.py examples/skydio-land/ledge/ledge.scenario.toml --seeds 2001:2013 --jobs 3 --out build/mc/ledge --set wind_ms=8'
.venv/bin/python examples/skydio-land/land_fly.py <scenario.toml> <out dir>        # one flight without the judge (trace.csv, summary.json, telemetry.jsonl)
.venv/bin/python examples/skydio-land/build_design.py                                # regenerates assemblies/skydio-land.toml from assemblies/skydio.toml
```

CPU only. Wall time is 1 to 1.7 s per simulated second on terrain, about 1 s in the forest and 4 to 5 s on the decks (the vessel's hull cells, the sea and 1 ms steps), all measured on
the shared box while it was loaded. In the viewer's PROGRAM page the Run slot lists the scenarios of this folder (`run.targets` walks `*.scenario.toml` under the program root,
recursively; the folders group them) and runs the one you pick; a test checks that (`tests/python/test_skydio_land_flights.py`).

@@RESULTS@@
