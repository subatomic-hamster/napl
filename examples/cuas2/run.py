"""Lockstep harness of the cuas2 scenarios: the cuas-lite harness (examples/cuas-lite/run.py: arm script, dead-man, Referee = ground-truth safety verdict of every ball,
s7 faults, hit statistics) with napl.cuas2.sim_lock.Lockstep2 (App2 + Sim2RP2040 + the cuas2 weapon) in place of the lite one.

    python examples/cuas2/run.py s6b_person_beside_balloon --seed 3 [--tracker imm3-gnn] [--no-ph] [--ph-tau 0.3] [--sim-s 30]

run_episode() is what mc.py calls. The result is the lite result dict (shots, hits_on_targets, popped, violations, checks, ...) plus `ph_fired` (the Ph of every FIRE
the app sent) and `cfg` (the FireCfg2 overrides). Exit status 1 on a violation, a failed check or an app failure."""

import argparse
import json
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

import scene as sc
from napl.cuas2.app import AppCfg2
from napl.cuas2.miss import ClosestApproach
from napl.cuas2.sim_lock import Lockstep2

lrun = sc.load("cuas_lite_run", "cuas-lite/run.py")


def ball_unarmed(arm_iv, fire_sess, t, fire_id):
    """arm_iv: [[t_arm, t_disarm or inf]] per session; fire_sess: fire_id -> session index the FIRE was sent in. A ball fed at `t` by FIRE `fire_id` is
    unarmed iff `t` is outside every session, or the session it falls in is not the one that FIRE was sent in (or the FIRE was never sent)."""
    j = next((i for i, (a, b) in enumerate(arm_iv) if a <= t <= b), None)
    return j is None or fire_sess.get(fire_id) != j


class Referee2(lrun.Referee):
    """The lite Referee judges a ball fed in control period k against the app's armed flag AFTER period k's step. The app disarms on the
    cap-th shot in that same step (spec: the cap ends the arm), so the cap-th ball read as 'unarmed' (3 of 165 episodes with the Ph gate off).
    Here a ball is 'unarmed' iff its feed time (exact, from the twin) is outside every arm session [Lockstep2.arm_iv, exact sim time of ARM / disarm],
    or the session it falls in is not the session in which ITS OWN FIRE was sent (twin.shots carries (t, fire_id, idx); Lockstep2.fire_sess maps
    fire_id -> session). So a ball fed after a disarm, or a session-0 FIRE's ball fed after a quick re-arm (even when a session-1 FIRE was already
    sent), is flagged."""

    def _unarmed(self, t, fire_id):
        return ball_unarmed(self.h.arm_iv, self.h.fire_sess, t, fire_id)

    def __call__(self, h=None):
        fc = self.h.app.fc
        n0, new = self.n_seen, self.h.twin.shots[self.n_seen :]
        real, fc.armed = (
            fc.armed,
            True,
        )  # the base class's flag check is replaced by _unarmed
        try:
            r = super().__call__(h)
        finally:
            fc.armed = real
        self.violations["unarmed"] += sum(self._unarmed(t, f) for t, f, _i in new)
        assert self.n_seen == n0 + len(new)
        self._refine_closest()
        return r

    def _refine_closest(self):
        """Replace the lite 5 ms-sampled closest approach (it reads low on a climbing ball that hits: napl.cuas2.miss) by the segment / extrapolated one."""
        h, ca = self.h, self.__dict__.setdefault("_ca", ClosestApproach())
        cur = h.world.prev_pellets
        if not cur and not ca.rounds_in_flight():
            return
        now = h.world.t
        tg = (
            [t for t in h._truth(now)["targets"] if t["cls"] != "person"] if cur else []
        )
        for rnd, pos in cur.items():
            for t in tg:
                ca.add(rnd, t["id"], now, pos - h.base - t["pos"])
        for rnd in ca.rounds_in_flight() - set(cur):
            ca.end(rnd)
        for rnd, b in ca.best.items():
            tr = h.targets[b["id"]]
            tvel = tr.pv(b["t"])[1] if hasattr(tr, "pv") else None
            self.closest[rnd] = dict(d=round(b["d"], 3), vec=[round(float(x), 3) for x in b["vec"]], id=b["id"], t=round(b["t"], 3),
                                     tvel=None if tvel is None else [round(float(x), 3) for x in tvel])  # fmt: skip


def run_episode(
    model, run_dir, sp, seed, early_stop=True, write=True, fire_cfg=None, app_calib=None
):
    """One lockstep episode of Spec `sp` (scene.make) on the built scene `model`. fire_cfg: FireCfg2 overrides (tracker, ph_gate, ph_tau, t_actuator ...)."""
    over = {**sp.fire_cfg, **(fire_cfg or {})}
    cfg = AppCfg2()
    cfg.fire.vmax, cfg.fire.amax = (100.0, 70.0), (600.0, 500.0)
    for k, v in over.items():
        setattr(cfg.fire, k, v)
    ph, seen, last = [], [None], [None]

    def hook(s):
        last[0] = s
        d = s.app.dec
        if d is not seen[0]:
            seen[0] = d
            if d.fire:
                ph.append(
                    None if getattr(d, "ph", None) is None else round(float(d.ph), 3)
                )

    orig, orig_ref = lrun.Lockstep, lrun.Referee
    lrun.Referee = Referee2
    lrun.Lockstep = lambda *a, **k: Lockstep2(
        *a, app_cfg=cfg, true_calib=sp.true_calib, app_calib=app_calib, **k
    )
    try:
        out = lrun.run_episode(
            model, run_dir, sp, seed, early_stop=early_stop, write=write, hook=hook
        )
    finally:
        lrun.Lockstep, lrun.Referee = orig, orig_ref
    out["ph_fired"], out["cfg"] = ph, over
    s = last[
        0
    ]  # [Ph of the decision, 1 if that ball hit a target]: ball i is kernel round i + 1
    hit_rounds = (
        {r for _, name, _, r in s.hit_log if s.targets[name].klass != "person"}
        if s
        else set()
    )
    out["ph_shot_hit"] = (
        [[x, int(i + 1 in hit_rounds)] for i, x in enumerate(s.app.fc.ph_shots)]
        if s
        else []
    )
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("scenario", choices=sc.SCENARIOS + ["smoke"])
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--sim-s", type=float, default=None)
    ap.add_argument("--tracker", default=None)
    ap.add_argument(
        "--no-ph",
        action="store_true",
        help="ablation: lite's pointing tolerance instead of the Ph gate",
    )
    ap.add_argument("--ph-tau", type=float, default=None)
    ap.add_argument("--brief", action="store_true", help="print the verdict keys only")
    ap.add_argument("--out", default=None, help="run directory (default: a temp dir)")
    a = ap.parse_args(argv)
    from napl.lite.sim_io import build_scene

    sp = sc.make(a.scenario, a.seed)
    if a.sim_s:
        sp.sim_s = a.sim_s
    fc = {
        k: v
        for k, v in dict(
            tracker=a.tracker, ph_gate=False if a.no_ph else None, ph_tau=a.ph_tau
        ).items()
        if v is not None
    }
    work = Path(a.out or tempfile.mkdtemp(prefix="cuas2run"))
    man = build_scene(
        work / "scene",
        sp.targets,
        env=sp.env,
        seed=a.seed,
        launcher=sp.launcher,
        base=sc.cached_base(),
    )
    res = run_episode(man["model"], work, sp, a.seed, fire_cfg=fc)
    if a.brief:
        keys = (
            "ok",
            "shots",
            "hits_on_targets",
            "n_violations",
            "checks",
            "app_failed",
        )
        res = {
            "scenario": res["scenario"],
            **{k: res[k] for k in keys},
            "popped": f"{len(res['popped'])}/{res['n_targets']}",
        }
    sys.stdout.write(json.dumps(res, default=float) + "\n")
    sys.exit(0 if res["ok"] else 1)


if __name__ == "__main__":
    main()
