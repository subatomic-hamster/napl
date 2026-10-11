"""Latency measurement of the lockstep chain, for the lead-time defaults (aim.T_ACTUATOR, aim.T_FIRE) and as the template of the bench measurement
(docs/cuas-lite/bench_validation.md 3.4 / 2.2). Runs a few Monte Carlo drift / hanging / crosser episodes and reports

  fire_to_feed   FIRE decision (App) -> ball fed (SimRP2040 extend complete = `fire` attr set; the kernel spawns the ball in the same control period)
  head_lag       how long the barrel (STATUS pan / tilt) lags the AIM the app commands while it tracks: best time shift of the AIM series that fits the
                 measured angle (least squares over the tracked samples)

    python examples/cuas-lite/latency.py --runs 4 --kinds drift quad3
"""

import argparse
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

import mc
import numpy as np
import run as runner
import scene as sc
from napl.lite import fire as firemod
from napl.lite.sim_io import build_scene

REC = dict(fire=[], aim=[])


def patch():
    f_sent, f_step = firemod.FireControl.fire_sent, firemod.FireControl.step

    def fire_sent(self, now, n=None):
        REC["fire"].append(now)
        return f_sent(self, now, n)

    def step(self, now, v):
        d = f_step(self, now, v)
        if d.aim is not None and d.target is not None:  # tracking a target
            REC["aim"].append((now, d.aim[0], d.aim[1], v.pan, v.tilt))
        return d

    firemod.FireControl.fire_sent, firemod.FireControl.step = fire_sent, step


def head_lag(aim):
    a = np.array(aim)
    if len(a) < 40:
        return None
    t = a[:, 0]
    best = {}
    for k, name in ((1, "pan"), (2, "tilt")):
        err = []
        for lag in np.arange(0.0, 0.31, 0.01):
            cmd = np.interp(t - lag, t, a[:, k])
            err.append(np.mean((cmd[t > t[0] + 0.5] - a[t > t[0] + 0.5, k + 2]) ** 2))
        best[name] = float(np.argmin(err) * 0.01)
    return best


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--runs", type=int, default=4)
    ap.add_argument("--kinds", nargs="*", default=["drift", "quad3"])
    a = ap.parse_args()
    patch()
    base = str(sc.cached_base())
    ff, hl = [], []
    for kind in a.kinds:
        for i in range(a.runs):
            name, kw, seed = mc.draw(kind, "5-10", i)
            REC["fire"].clear()
            REC["aim"].clear()
            with tempfile.TemporaryDirectory() as work:
                work = Path(work)
                sp = sc.make(name, seed, **kw)
                man = build_scene(
                    work / "scene",
                    sp.targets,
                    env=sp.env,
                    seed=seed,
                    launcher=sp.launcher,
                    base=base,
                )
                res = runner.run_episode(man["model"], work, sp, seed, write=False)
            fed = [t for t in res["shot_t"]]
            # first burst: every FIRE is followed by a feed of its first ball
            for t0 in REC["fire"]:
                nxt = [t for t in fed if t >= t0]
                if nxt:
                    ff.append(nxt[0] - t0)
            lag = head_lag(REC["aim"])
            if lag:
                hl.append(lag["pan"])
                sys.stdout.write(
                    f"{kind} {i}: FIREs {len(REC['fire'])} head lag pan {lag['pan']:.2f} tilt {lag['tilt']:.2f}\n"
                )  # ttsr-ignore: py/no-print (CLI)
    ff = np.array(ff)
    sys.stdout.write(
        f"fire_to_feed s: n {len(ff)} min {ff.min():.3f} median {np.median(ff):.3f} max {ff.max():.3f}\n"
    )  # ttsr-ignore: py/no-print (CLI)
    if hl:
        sys.stdout.write(
            f"head_lag pan s: median {np.median(hl):.3f} (n {len(hl)})\n"
        )  # ttsr-ignore: py/no-print (CLI)


if __name__ == "__main__":
    main()
