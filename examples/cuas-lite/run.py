"""Lockstep harness of the cuas-lite scenarios: World(ctl_ticks=5) + SimRP2040 (LoopbackLink) + SimCamera + TruthDetector + the real App, one process,
deterministic, CPU only. Run through `napl program examples/cuas-lite/<scenario>.scenario.toml` (sets NAPL_MODEL / NAPL_RUN_DIR); mc.py calls run_episode().

Env: CUAS_SCENARIO (scene.make name), CUAS_SEED, CUAS_SIM_S (override the scenario's length), CUAS_ARM_AT (s, default from the scenario: link up and homed by then).
World.finish() writes trace.csv (napl.cuas.world columns); this harness appends the Referee columns (shots, violations, viol_person, viol_cut, person_visible, popped) and
writes lite.json: the ground-truth safety verdict of every ball (python/napl/lite/sim_eval.py), the scenario's own checks (`expect` / the s7 fault) and the hit
statistics. Exit status 1 when a violation, a failed check or an app failure."""

import csv
import json
import math
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

import scene as sc
from napl.lite import protocol as p
from napl.lite.sim_eval import CUT_LATENCY_S as LATENCY
from napl.lite.sim_eval import Referee
from napl.lite.sim_io import Lockstep

DETECTOR_OUTAGE_S, LINK_DROP_S, AFTER_CUT_S = 2.0, 0.5, 6.0


class Fault:
    """s7: the interlock / link / dead-man / detector fault applied at the first moment a pusher cycle is in progress (the FIRE is accepted, the ball not yet fed)."""

    def __init__(self, kind, ref):
        self.kind, self.ref, self.t_cut = kind, ref, None

    def __call__(self, h):
        if self.t_cut is not None or h.twin.state != p.FIRING:
            return
        now, k = h.world.t, self.kind
        self.t_cut = now
        end = math.inf
        if k == "deadman":
            h.hold_deadman = False
        elif k == "detector":
            h.detector.outages.append((now, now + DETECTOR_OUTAGE_S))
            end = now + DETECTOR_OUTAGE_S
        elif k == "estop":
            h.plant.estop_open = True
        elif k == "key":
            h.plant.key_on = False
        elif k == "link":
            h.link.drop = True
            h.script.append(
                (now + LINK_DROP_S, lambda s: setattr(s.link, "drop", False))
            )
        else:
            raise KeyError(k)
        self.ref.windows.append((now, end, LATENCY[k], k))


def run_episode(model, run_dir, sp, seed, early_stop=True, write=True, hook=None):
    """One lockstep episode of Spec `sp` on the built scene `model`. -> result dict (also lite.json + the extended trace.csv in run_dir when write)."""
    run_dir = Path(run_dir)
    h = Lockstep(
        dict(model=str(model)),
        run_dir,
        sp.targets,
        sector=sc.SECTOR,
        seed=seed,
        detector_kw=sp.detector_kw,
        camera_kw=sp.camera_kw,
    )
    fc = h.app.fc.cfg
    fc.shot_cap, fc.burst, h.app.cfg.wind = sp.shot_cap, sp.burst, sp.wind_est
    for k, v in sp.fire_cfg.items():
        setattr(fc, k, v)
    h.hold_deadman = True
    arm_at = float(os.environ.get("CUAS_ARM_AT", sp.arm_at))
    h.script.append((arm_at, lambda s: s.app.post("arm")))
    ref = Referee(h, check_hold=sp.check_hold)
    fault = Fault(sp.fault, ref) if sp.fault else None
    movers = [t for t in sp.targets if t.klass != "person"]
    rows = [{}] * len(
        h.world.rows
    )  # the settle steps of Lockstep.__init__ already wrote rows
    seen = dict(person_blocked=0, confirmed=0)
    why = {}  # control periods (5 ms) spent armed with each fire-control reason (why no ball was fed)
    first_hit = []

    def cb(s):
        ref(s)
        if hook:
            hook(
                s
            )  # observation only (tools/record_cuas_lite.py logs the HUD state); must not touch the harness
        if fault:
            fault(s)
        if s.world.k % 4 == 0:
            rows.append(ref.row())
        seen["person_blocked"] += "person" in s.app.dec.reasons
        if s.app.fc.armed:
            for r in s.app.dec.reasons:
                why[r] = why.get(r, 0) + 1
        seen["confirmed"] += any(t.confirmed for t in s.app.tracks)
        if s.hit_log and not first_hit:
            first_hit.append(s.hit_log[0][0])
        now = s.world.t
        if fault and fault.t_cut is not None and now > fault.t_cut + AFTER_CUT_S:
            return True
        if early_stop and not fault and now > arm_at + 1.0:
            if len(s.popped) >= len(movers) or (
                not s.app.fc.armed and s.world.in_flight == 0 and s.twin.shots
            ):
                return (
                    ref.first_vis is not None and now > ref.last_vis + 3.0
                    if sp.check_hold
                    else True
                )
        return False

    h.run(float(os.environ.get("CUAS_SIM_S", sp.sim_s)), until=cb)
    nat = h.finish()
    w, tw = h.world, h.twin
    shots = list(ref.shot_t)
    hits_on_targets = sum(
        n for i, n in h.hits.items() if h.targets[i].klass != "person"
    )
    out = dict(
        scenario=sp.name, seed=seed, sim_s=w.t, shots=len(shots), rounds=w.rounds, shot_t=[round(t, 3) for t in shots], arm_at=arm_at,
        ttfs=round(shots[0] - arm_at, 3) if shots else None, t_first_hit=round(first_hit[0] - arm_at, 3) if first_hit else None,
        hits=h.hits, hits_on_targets=hits_on_targets, hit_rate=hits_on_targets / len(shots) if shots else None,
        popped=sorted(h.popped), popped_t={k: round(v, 3) for k, v in h.popped.items()}, n_targets=len(movers),
        violations=ref.violations, outside_deg=round(ref.outside_deg, 2), n_violations=ref.n_violations, person_raw=ref.person_raw, person_blocked_ticks=seen["person_blocked"],
        confirmed_ticks=seen["confirmed"], reasons=dict(sorted(why.items(), key=lambda kv: -kv[1])), miss={str(k): v for k, v in sorted(ref.closest.items())}, state_hash=nat["state_hash"], app_failed=h.app.failed, mcu=p.STATE_NAMES[tw.state],
    )  # fmt: skip
    checks = {}
    if sp.expect.get("shots_before"):
        checks["engaged_before_person"] = (
            ref.first_vis is not None
            and sum(t < ref.first_vis for t in shots) >= sp.expect["shots_before"]
        )
        checks["resumed_after_person"] = (
            sum(t >= ref.last_vis + 1.0 - 0.03 for t in shots)
            >= sp.expect["shots_after"]
        )
    if sp.expect.get("no_shots"):
        checks["no_shots"] = not shots
        checks["target_was_engageable_but_for_the_person"] = (
            seen["confirmed"] > 0 and seen["person_blocked"] > 0
        )
    if fault:
        k = fault.kind
        checks["fault_applied"] = fault.t_cut is not None
        out["t_cut"] = fault.t_cut
        if k == "estop":  # stays pressed: SAFE for good
            checks["state_safe"] = (
                tw.state == p.SAFE and tw.fw == 0.0 and not h.app.fc.armed
            )
        elif (
            k == "key"
        ):  # SAFE at once (key_off), back to IDLE with valid heartbeats, never ARMED by itself
            checks["key_off_safe_no_rearm"] = (
                any(r == p.R_KEY_OFF for _, _, r in tw.transitions)
                and tw.state in (p.SAFE, p.IDLE)
                and tw.fw == 0.0
                and not h.app.fc.armed
            )
        elif k == "deadman":
            checks["flywheel_off_not_firing"] = tw.fw == 0.0 and tw.state != p.FIRING
        elif k == "link":
            checks["watchdog_tripped_and_no_rearm"] = (
                any(r == p.R_WATCHDOG for _, _, r in tw.transitions)
                and tw.state in (p.IDLE, p.SAFE)
                and not h.app.fc.armed
            )
        elif k == "detector":
            checks["outage_does_not_disarm"] = (
                h.app.fc.armed or bool(h.popped) or tw.shots_arm >= fc.shot_cap
            )  # the stale detector only inhibits; the engagement may resume (or has finished: kill / cap)
    out["checks"] = checks
    out["ok"] = ref.ok and all(checks.values()) and not h.app.failed
    if write:
        (run_dir / "lite.json").write_text(json.dumps(out, indent=1))
        _extend_trace(run_dir / "trace.csv", rows)
    return out


def _extend_trace(path, rows):
    """Append the Referee columns to the World's trace.csv (one row per World row)."""
    with open(path, newline="") as f:
        tr = list(csv.reader(f))
    cols = [
        "shots",
        "violations",
        "viol_person",
        "viol_cut",
        "person_visible",
        "popped",
    ]
    rows = (rows + [{}] * len(tr))[: len(tr) - 1]
    tr[0] += cols
    for r, x in zip(tr[1:], rows):
        r += [x.get(c, 0) for c in cols]
    with open(path, "w", newline="") as f:
        csv.writer(f).writerows(tr)


def main():
    run_dir = Path(os.environ["NAPL_RUN_DIR"])
    sp = sc.make(sc.scenario(), sc.seed())
    out = run_episode(os.environ["NAPL_MODEL"], run_dir, sp, sc.seed())
    sys.stdout.write(json.dumps(out) + "\n")
    sys.exit(0 if out["ok"] else 1)


if __name__ == "__main__":
    main()
