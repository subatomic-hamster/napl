"""Aim-bias diagnostic (P13 BIAS): per shot at the static targets of the cuas2 Monte Carlo, split the closest-approach z into the terms of the aim chain.

    python examples/cuas2/aimdiag.py --kinds balloon hanging --bands 3-5 5-10 --runs 10 --out build/aimdiag/before   # resumable (episodes.jsonl)
    python examples/cuas2/aimdiag.py --report build/aimdiag/before [build/aimdiag/after]

Per shot, all vectors are ball minus target at the plane through the TRUE target centre (at the ball's closest-approach time) perpendicular to the line of sight:
    z_miss = T_est + T_muz + T_dir + T_spd + T_phys  (exact, sequential differences of five path evaluations P0..P4)
    P0 = python ballistics from the muzzle the solver assumed (geom.pose at the solved attitude), solved direction u, model v0 / spin / ball / app wind
    P1 = same, from the kernel's actual muzzle position                    T_muz  = P1 - P0   (camera-to-barrel / pivot geometry, attitude lag)
    P2 = same, along the direction the kernel launched (muzzle_dir + dir_offset)  T_dir  = P2 - P1   (pointing error incl. the launcher scatter draw)
    P3 = same, with the plant's actual v0 / spin / true wind               T_spd  = P3 - P2   (launch speed, spin, wind)
    P4 = the kernel ball's own path (samples every 5 ms, linear)           T_phys = P4 - P3   (python vs kernel flight integration)
    T_est = P0 - T_true: the aim point of the solved shot against the true target centre (tracker estimate + target motion + solver residual).
Also logged: `meas` = the 5 ms-sampled closest approach of lite's Referee (what mc.py `bias()` reports) against the refined one (segment minimum + 20 ms
extrapolation past the last sample of a ball that hit), the tracker's position error at the decision, kernel vs assumed camera pose and muzzle."""

import argparse
import json
import math
import shutil
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

import mc as cmc
import numpy as np
import run as runner
import scene as sc
from napl.cuas2 import ballistics as bal
from napl.cuas2 import sim_lock
from napl.cuas2.calib import Calib
from napl.cuas2.launcher import LauncherModel

CUR = {}  # current episode: dict(h=Lockstep2, shots=[...], ticks=[...])
UPV = np.array([0.0, 0.0, 1.0])


def _unit(a):
    a = np.asarray(a, float)
    return a / np.linalg.norm(a)


class DiagRef(runner.Referee2):
    """Referee2 that also logs every ball in flight and the truth target positions each control period."""

    def __call__(self, h=None):
        r = super().__call__(h)
        w = self.h.world
        if self.h.targets:
            q = w.sim.query
            CUR["hist"].append(
                (
                    w.t,
                    {
                        i: q(w.body_path[w.man["body_names"].index(i)], "pos")[0]
                        - self.h.base
                        for i in CUR["tids"]
                    },
                )
            )
        if w.prev_pellets:
            tg = {
                t["id"]: np.array(t["pos"])
                for t in self.h._truth(w.t)["targets"]
                if t["cls"] != "person"
            }
            CUR["ticks"].append(
                (w.t, {k: v - self.h.base for k, v in w.prev_pellets.items()}, tg)
            )
        return r


_gun_fire = sim_lock.GunRig.fire


def _fire(self, shot):
    """Record the launch state, then fire."""
    h, w = CUR["h"], self.w
    m, d = w.muzzle()
    pan, tilt = w.pan_tilt()
    tr = h._truth(w.t)
    fd = CUR["fire_dec"][-1] if CUR["fire_dec"] else None
    CUR["shots"].append(dict(
        t=w.t, muzzle=(m - h.base).tolist(), mdir=d.tolist(), shot=dict(v0=shot.v0, omega=shot.omega, dir_err=list(shot.dir_err), d=shot.d, m=shot.m, cd=shot.cd, dip=shot.dip),
        pt=(math.degrees(pan), math.degrees(tilt)), cam_pos=tr["cam_pos"].tolist(), cam_fwd=tr["cam_fwd"].tolist(),
        wind=np.array(w._q(w.air, "wind")).tolist(), rpm_after=list(h.weapon.model.rpm), dec=fd, targets={t["id"]: t["pos"].tolist() for t in tr["targets"]}))  # fmt: skip
    _gun_fire(self, shot)


sim_lock.GunRig.fire = _fire
_init = sim_lock.Lockstep2.__init__


def _init2(self, *a, **k):
    _init(self, *a, **k)
    CUR["h"] = self
    fc = self.app.fc
    fs = fc.fire_sent

    def fire_sent(now, n=None):
        d, tg = self.app.dec, self.app.dec.target
        sol = d.solution
        CUR["fire_dec"].append(dict(
            t=self.world.t, sol=None if sol is None else dict(pan=sol["pan"], tilt=sol["tilt"], u=sol["u"].tolist(), tf=sol["t_flight"], icpt=np.asarray(sol["intercept"]).tolist()),
            trk=None if tg is None else dict(t=tg.t, mean=tg.mean.tolist(), id=tg.id), ph=d.ph, wind=list(self.app.cfg.wind)))  # fmt: skip
        fs(now, n)

    fc.fire_sent = fire_sent


sim_lock.Lockstep2.__init__ = _init2
runner.Referee2 = DiagRef


def F(M, u, v, om, ball, wind, T, n):
    """Python path from M along u (speed v, spin om about the lateral axis, ball (d, m, cd)); position where it crosses the plane through T with normal n."""
    t, p, _ = bal.fly(
        M,
        v * np.asarray(u),
        bal.omega_vec(u, om),
        ball,
        wind=wind,
        dt=0.0005,
        t_max=2.5,
    )
    s = (p - T) @ n
    i = int(np.argmax(s >= 0))
    if s[i] < 0:
        return None
    f = -s[i - 1] / (s[i] - s[i - 1])
    return p[i - 1] + f * (p[i] - p[i - 1])


def refine(ticks, rnd, tid):
    """Closest approach of kernel round `rnd` to target `tid`: -> (t*, ball pos, target pos, vec) from the 5 ms samples (piecewise linear) plus 20 ms extrapolation."""
    S = [(t, pp[rnd], tg[tid]) for t, pp, tg in ticks if rnd in pp and tid in tg]
    if len(S) < 2:
        return None
    S.append(
        (
            S[-1][0] + 0.02,
            S[-1][1] + 4 * (S[-1][1] - S[-2][1]),
            S[-1][2] + 4 * (S[-1][2] - S[-2][2]),
        )
    )
    best = None
    for (t0, b0, g0), (t1, b1, g1) in zip(S, S[1:]):
        r0, r1 = b0 - g0, b1 - g1
        dr = r1 - r0
        s = float(np.clip(-(r0 @ dr) / max(dr @ dr, 1e-12), 0, 1))
        r = r0 + s * dr
        if best is None or np.linalg.norm(r) < best[0]:
            best = (
                float(np.linalg.norm(r)),
                t0 + s * (t1 - t0),
                b0 + s * (b1 - b0),
                g0 + s * (g1 - g0),
                r,
            )
    return best[1:]


def decompose(ep, sp, true_calib):
    """-> list of per-shot dicts (z / y components of every term, m)."""
    h = ep["h"]
    geom, model = h.geom, LauncherModel(Calib())
    mod_ball = Calib().ball
    ball_m = (mod_ball.d_mm / 1000, mod_ball.m_g / 1000, mod_ball.cd)
    tb = true_calib.ball
    ball_t = (tb.d_mm / 1000, tb.m_g / 1000, tb.cd)
    rt, rb = model._rpms(model.c.launcher.rpm_max, None)
    v_mod, om_mod = model.mean_v0((rt, rb), with_dip=True), model.spin_rad_s(rt, rb)
    out = []
    ids = [t for t in h.targets if h.targets[t].klass != "person"]
    for k, s in enumerate(ep["shots"]):
        dec = s["dec"]
        if dec is None or dec["sol"] is None or not ids:
            continue
        rnd = k + 1
        tid = ids[0]
        ref = refine(ep["ticks"], rnd, tid)
        if ref is None:
            continue
        tstar, bpos, tpos, vec = ref
        M_k, dir_k = np.array(s["muzzle"]), np.array(s["mdir"])
        n = _unit(tpos - M_k)
        sol = dec["sol"]
        M_app = geom.pose(sol["pan"], sol["tilt"]).muzzle
        # kernel launch direction: muzzle_dir + dir_offset_x * site x (up) + dir_offset_y * site y (right), re-normalised
        ax = _unit(UPV - (UPV @ dir_k) * dir_k)
        ay = np.cross(dir_k, ax)
        e_h, e_v = s["shot"]["dir_err"]  # (horizontal left+, vertical up+)
        u_l = _unit(dir_k + e_v * ax - e_h * ay)
        w_app, w_true = np.array(dec["wind"], float), np.array(s["wind"], float)
        sh = s["shot"]
        P0 = F(M_app, sol["u"], v_mod, om_mod, ball_m, w_app, tpos, n)
        P1 = F(M_k, sol["u"], v_mod, om_mod, ball_m, w_app, tpos, n)
        P1b = F(
            M_k, dir_k, v_mod, om_mod, ball_m, w_app, tpos, n
        )  # the barrel as pointed at release, before the launcher's scatter draw
        P2 = F(M_k, u_l, v_mod, om_mod, ball_m, w_app, tpos, n)
        P3 = F(M_k, u_l, sh["v0"], sh["omega"], ball_t, w_true, tpos, n)
        if any(p is None for p in (P0, P1, P1b, P2, P3)):
            continue
        # P4: the kernel ball where it crosses the same plane (its own samples, linear + the extrapolation of refine())
        pts = [(t, pp[rnd]) for t, pp, _tg in ep["ticks"] if rnd in pp]
        pts.append((pts[-1][0] + 0.02, pts[-1][1] + 4 * (pts[-1][1] - pts[-2][1])))
        P4 = None
        for (_, a), (_, b) in zip(pts, pts[1:]):
            sa, sb = (a - tpos) @ n, (b - tpos) @ n
            if sa < 0 <= sb:
                P4 = a + (-sa / (sb - sa)) * (b - a)
                break
        if P4 is None:
            continue
        cam = geom.pose(*s["pt"])  # the assumed pose at the TRUE attitude
        pz = geom.pose(*(float(x) for x in s["pt"]))
        trk = dec["trk"]
        # T_est split (exact): icpt - truth(t*) = E_solver + E_pos + E_vel + E_curv + E_time, all evaluated at the scan time ts of the track
        ht = np.array([t for t, _ in ep["hist"]])
        hp = np.array([p[tid] for _, p in ep["hist"]])

        def truth(t):
            return np.array([np.interp(t, ht, hp[:, i]) for i in range(3)])

        def tvel(t, d=0.01):
            return (truth(t + d) - truth(t - d)) / (2 * d)

        e = {}
        if trk:
            ts, cfg = trk["t"], h.app.fc.cfg
            dlt = (
                (dec["t"] - ts) + cfg.t_actuator + cfg.t_fire + sol["tf"]
            )  # the horizon the app extrapolated over
            mp, mv, icpt = (
                np.array(trk["mean"][:3]),
                np.array(trk["mean"][3:6]),
                np.array(sol["icpt"]),
            )
            e = dict(
                E_pos=(mp - truth(ts)).tolist(), E_vel=((mv - tvel(ts)) * dlt).tolist(),
                E_curv=(truth(ts) + tvel(ts) * dlt - truth(ts + dlt)).tolist(), E_time=(truth(ts + dlt) - tpos).tolist(), E_solver=(icpt - (mp + mv * dlt)).tolist(),
                E_p0=(P0 - icpt).tolist(), dlt_app=dlt, dlt_act=tstar - ts, t_rel=s["t"] - dec["t"], tvel=tvel(ts).tolist(), trk_vel=mv.tolist())  # fmt: skip
        e_trk = e.get("E_pos")
        rpm = np.array(s["rpm_after"]) / (
            1 - sh["dip"]
        )  # the wheel speeds before the ball's dip (approx.)
        out.append(dict(
            **e, rpm_pre=rpm.tolist(),
            k=k + 1, id=tid, R=float(np.linalg.norm(tpos - M_k)), el_deg=math.degrees(math.asin(n[2])), closest=float(np.linalg.norm(vec)), vec=vec.tolist(),
            T_est=(P0 - tpos).tolist(), T_muz=(P1 - P0).tolist(), T_dir=(P2 - P1).tolist(), T_point=(P1b - P1).tolist(), T_scat=(P2 - P1b).tolist(),T_spd=(P3 - P2).tolist(), T_phys=(P4 - P3).tolist(), miss_plane=(P4 - tpos).tolist(),
            e_trk=e_trk, trk_age=None if trk is None else s["t"] - trk["t"],
            muz_err=(M_k - pz.muzzle).tolist(), cam_err=(np.array(s["cam_pos"]) - cam.cam_pos).tolist(),
            cam_dir_err_deg=math.degrees(math.acos(min(1.0, float(np.array(s["cam_fwd"]) @ cam.cam_fwd)))),
            dir_k_vs_pose_deg=math.degrees(math.acos(min(1.0, float(dir_k @ pz.muzzle_dir)))), dir_k_el_minus_pose_deg=math.degrees(math.asin(dir_k[2]) - math.asin(pz.muzzle_dir[2])),
            point_vs_sol_deg=math.degrees(math.acos(min(1.0, float(dir_k @ np.array(sol["u"]))))), ph=dec["ph"], hit=bool(np.linalg.norm(vec) < 0.0 + 0.2),
            v0=sh["v0"], v_mod=v_mod, dv_err=sh["dir_err"], tilt_deg=s["pt"][1], icpt_minus_trk=(np.array(sol["icpt"]) - np.array(trk["mean"][:3])).tolist() if trk else None))  # fmt: skip
    return out


def episode(job):
    cell, name, kw, seed, root, base, fire_cfg = job
    work = Path(tempfile.mkdtemp(prefix="ep", dir=root))
    try:
        from napl.lite.sim_io import build_scene

        sp = sc.make(name, seed, **kw)
        man = build_scene(
            work / "scene",
            sp.targets,
            env=sp.env,
            seed=seed,
            launcher=sp.launcher,
            base=base,
        )
        CUR.clear()
        CUR.update(
            shots=[],
            ticks=[],
            fire_dec=[],
            hist=[],
            h=None,
            tids=[t.id for t in sp.targets if t.klass != "person"],
        )
        res = runner.run_episode(
            man["model"], work, sp, seed, write=False, fire_cfg=fire_cfg
        )
        res.update(cell=cell, kw=dict(kw))
        res["aim"] = decompose(CUR, sp, sp.true_calib)
        res["miss_ref"] = res.get("miss")
        return res
    except Exception as e:  # noqa: BLE001
        import traceback

        return dict(
            cell=cell,
            seed=seed,
            error=f"{type(e).__name__}: {e}",
            tb=traceback.format_exc(),
            ok=False,
            n_violations=0,
        )
    finally:
        shutil.rmtree(work, ignore_errors=True)


def ci(x, g=None):
    """mean +- 1.96 se; with groups g (episode ids) the se is cluster-robust (shots of one episode are not independent)."""
    x = np.asarray(x, float)
    if g is None or len(x) < 2:
        return f"{x.mean():+.3f} +-{1.96 * x.std(ddof=1) / math.sqrt(len(x)) if len(x) > 1 else math.nan:.3f}"
    g = np.asarray(g)
    r = x - x.mean()
    se = math.sqrt(sum(r[g == k].sum() ** 2 for k in set(g.tolist()))) / len(x)
    return f"{x.mean():+.3f} +-{1.96 * se:.3f}"


def report(dirs):
    lines = []
    for d in dirs:
        rs = [
            json.loads(ln)
            for ln in (Path(d) / "episodes.jsonl").read_text().splitlines()
        ]
        ok = [r for r in rs if "error" not in r]
        lines.append(f"## {d}: {len(rs)} episodes, {len(rs) - len(ok)} crashed")
        for kind in sorted({r["cell"].split("/")[0] for r in ok}):
            eps = [r for r in ok if r["cell"].startswith(kind + "/")]
            S = [
                (r["seed"], s)
                for r in eps
                for s in r.get("aim", [])
                if s["closest"] < 0.6
            ]
            shots = [s for _, s in S]

            def C(f, sel=lambda s: True):
                return ci([f(s) for _, s in S if sel(s)], [g for g, s in S if sel(s)])

            balls = sum(r["shots"] for r in eps)
            hits = sum(r["hits_on_targets"] for r in eps)
            kills = sum(len(r["popped"]) for r in eps)
            lines.append(
                f"### {kind}: {len(eps)} episodes, balls {balls}, hits {hits} (hit/ball {hits / max(balls, 1):.2f}), kills/episode {kills / len(eps):.2f}, decomposed shots (closest < 0.6 m) {len(shots)}"
            )
            if len(shots) < 2:
                continue
            ms = [
                m["vec"][2]
                for r in eps
                for m in (r.get("miss") or {}).values()
                if m["d"] < 0.6
            ]
            lines.append(f"- referee (5 ms sampled, episode-level se not available) z {ci(ms)} (n={len(ms)}); refined closest-approach z {C(lambda s: s['vec'][2])}; at the plane z {C(lambda s: s['miss_plane'][2])}, y {C(lambda s: s['miss_plane'][1])} (se clustered by episode)")  # fmt: skip
            lines.append(
                "| term | mean z (m) +- 95% | mean y (m) +- 95% |\n|---|---|---|"
            )
            for key in (
                "T_est",
                "T_muz",
                "T_dir",
                "T_point",
                "T_scat",
                "T_spd",
                "T_phys",
                "miss_plane",
            ):
                if key not in shots[0]:
                    continue
                lines.append(
                    f"| {key} | {C(lambda s: s[key][2])} | {C(lambda s: s[key][1])} |"
                )
            if "E_pos" in shots[0]:
                lines.append("| T_est split: | | |")
                for key in ("E_pos", "E_vel", "E_curv", "E_time", "E_solver", "E_p0"):
                    lines.append(
                        f"| - {key} | {C(lambda s: s[key][2], lambda s: key in s)} | {C(lambda s: s[key][1], lambda s: key in s)} |"
                    )
                lines.append(f"- horizon the app extrapolated over {C(lambda s: s['dlt_app'], lambda s: 'dlt_app' in s)} s, actual scan-to-closest {C(lambda s: s['dlt_act'], lambda s: 'dlt_app' in s)} s, decision-to-release {C(lambda s: s['t_rel'], lambda s: 'dlt_app' in s)} s")  # fmt: skip
            lines.append(f"- kernel muzzle - assumed muzzle (z): {C(lambda s: s['muz_err'][2])}; camera pos err (z): {C(lambda s: s['cam_err'][2])}; pointing: launch dir vs solved {C(lambda s: s['point_vs_sol_deg'])} deg; v0 plant - model {C(lambda s: s['v0'] - s['v_mod'])} m/s; wheel rpm before the dip (top, bot) {np.mean([s['rpm_pre'] for s in shots if 'rpm_pre' in s], axis=0).round(0).tolist()}")  # fmt: skip
            lines.append("- by range (plane z): " + "; ".join(f"{lo}-{hi} m: {C(lambda s: s['miss_plane'][2], lambda s: lo <= s['R'] < hi)} (n={sum(lo <= s['R'] < hi for s in shots)})" for lo, hi in ((0, 5), (5, 8), (8, 11), (11, 99)) if sum(lo <= s['R'] < hi for s in shots) > 1))  # fmt: skip
    text = "\n".join(lines)
    sys.stdout.write(text + "\n")  # ttsr-ignore: py/no-print (CLI)
    return text


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--runs", type=int, default=10)
    ap.add_argument("--kinds", nargs="*", default=["balloon", "hanging"])
    ap.add_argument("--bands", nargs="*", default=["3-5", "5-10"])
    ap.add_argument("--seed0", type=int, default=0)
    ap.add_argument("--out", default="build/aimdiag")
    ap.add_argument("--fire-cfg", default="{}")
    ap.add_argument("--report", nargs="*", default=None)
    a = ap.parse_args(argv)
    if a.report:
        return report(a.report)
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    base = str(sc.cached_base())
    log = out / "episodes.jsonl"
    done = (
        {(r["cell"], r["seed"]) for r in map(json.loads, log.read_text().splitlines())}
        if log.exists()
        else set()
    )
    fire_cfg = json.loads(a.fire_cfg)
    with log.open("a") as fh:
        for kind in a.kinds:
            for band in a.bands:
                for i in range(a.runs):
                    name, kw, seed = cmc.lmc.draw(kind, band, i, a.seed0)
                    cell = f"{kind}/{band}"
                    if (cell, seed) in done:
                        continue
                    r = episode((cell, name, kw, seed, str(out), base, fire_cfg))
                    fh.write(json.dumps(r, default=float) + "\n")
                    fh.flush()
                    sys.stdout.write(
                        f"{cell} seed {seed}: "
                        + (
                            r.get("error")
                            or f"shots {r['shots']} hits {r['hits_on_targets']} aim {len(r['aim'])}"
                        )
                        + "\n"
                    )  # ttsr-ignore: py/no-print (CLI)


if __name__ == "__main__":
    main()
