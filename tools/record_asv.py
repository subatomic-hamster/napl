"""Recording harness of the ASV (COLREG collision avoidance) end-to-end video. Media is never committed: runs, frames and the webm stay on the host
(hfbox); exact commands and what is real / stand-in: docs/asv/video.md.

Everything is OFFSCREEN (Sim.render, GPU) with a Pillow HUD composited on the frame: no xvfb viewer. The pipeline reuses what exists:
  run     CASE ROOT      the real `napl program` scenario (examples/asv/asv-<kind>.scenario.toml, NAPL_PARAMS seed / Hs / jitter 1 / AIS, visual = true): its monitors,
                         telemetry.jsonl, replay.naplog and the napl-replay hash check are the run's own (ROOT/runs/<case>/<run dir>)
  build  ROOT            the vessel (assemblies/asv.toml compiled by napl.design, mass from its report) revealed part by part, orbit camera (ROOT/build/f*.rgz)
  sim    CASE ROOT       the run's replay.naplog applied to a fresh World (tools/screenshots.py Replay: same hash, checked) and drawn with chase / overhead / orbit / abeam
                         cameras that blend into each other, plus the HUD (rule, role, CPA, tracks, lidar, plan, trails) -> ROOT/sim/<case>/f*.rgz + meta.json
  encode ROOT OUT.webm   title, build, scenario segments, result cards -> AV1 webm through tools/videnc.py (GStreamer VA, LGPL; nothing GPL)
  png    DIR OUT N...    raw .rgz frames N... of DIR as PNGs (for looking at them)
  mc-seeds               the cases and their seeds, as JSON

Frames are raw RGBA (.rgz, tools/videnc.py); the encode / png / sim / build stages need Pillow, which the project venv lacks: run them with
PYTHONPATH pointing at a directory holding only PIL (docs/asv/video.md).
"""

import argparse
import bisect
import json
import math
import os
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "python"))
sys.path.insert(0, str(REPO / "examples"))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import videnc

W, H, FPS = 1280, 720, 20
TAU = 2 * math.pi

# case -> (scenario kind, seed, traffic complies, title, rule quote shown for the first seconds). Seeds: examples/asv/README.md "For the video" (all in the 50000 block).
CASES = {
    "head_on": (
        "head_on",
        50025,
        True,
        "HEAD-ON",
        "Rule 14: each shall alter her course to starboard, so that each shall pass on the port side of the other",
    ),
    "crossing_give_way": (
        "crossing_give_way",
        50025,
        True,
        "CROSSING, ASV GIVES WAY",
        "Rule 15: the vessel which has the other on her own starboard side shall keep out of the way and avoid crossing ahead",
    ),
    "crossing_stand_on": (
        "crossing_stand_on",
        50025,
        True,
        "CROSSING, ASV STANDS ON",
        "Rule 17(a)(i): where one of two vessels is to keep out of the way, the other shall keep her course and speed",
    ),
    "stand_on_noncomplying": (
        "crossing_stand_on",
        50025,
        False,
        "STAND-ON, THE OTHER VESSEL DOES NOT GIVE WAY",
        "Rule 17(a)(ii): the stand-on vessel may take action when it becomes apparent that the give-way vessel is not acting",
    ),
    "overtaking": (
        "overtaking",
        50015,
        True,
        "OVERTAKING",
        "Rule 13(a): any vessel overtaking any other shall keep out of the way of the vessel being overtaken",
    ),
    "multi_traffic": (
        "multi_traffic",
        50037,
        True,
        "THREE VESSELS, MIXED ROLES",
        "Rules 14, 15, 17 one after the other: stand on for the vessel from port, give way to the one from starboard, port-to-port with the reciprocal one",
    ),
    "patrol": (
        "patrol",
        50025,
        True,
        "PATROL, FIXED OBJECTS",
        "A 100 x 18 m rectangle past two floats; waves from the bow quarter; lidar-only avoidance of fixed objects",
    ),
}
ORDER = [
    "head_on",
    "crossing_give_way",
    "crossing_stand_on",
    "stand_on_noncomplying",
    "overtaking",
    "multi_traffic",
    "patrol",
]

SHORT = {
    "head_on": "head-on",
    "crossing_give_way": "crossing, ASV gives way",
    "crossing_stand_on": "crossing, ASV stands on",
    "stand_on_noncomplying": "stand-on, other does NOT comply",
    "overtaking": "overtaking",
    "multi_traffic": "three vessels, mixed roles",
    "patrol": "patrol",
}
AMBER, GREEN, RED, CYAN, WHITE, GREY = (
    (255, 190, 60),
    (110, 230, 140),
    (255, 90, 80),
    (110, 215, 255),
    (255, 255, 255),
    (170, 180, 195),
)
ROLE_COL = {"give_way": AMBER, "stand_on": GREEN, "none": GREY}
RULE_TXT = {
    "head_on": ("RULE 14  HEAD-ON", "give way: turn to starboard, pass port to port"),
    "crossing_give_way": (
        "RULE 15  CROSSING",
        "give way: keep clear, do not cross ahead",
    ),
    "crossing_stand_on": ("RULE 17  CROSSING", "stand on: keep course and speed"),
    "overtaking": (
        "RULE 13  OVERTAKING",
        "give way: keep out of the way of the vessel ahead",
    ),
    "overtaken": ("RULE 13  BEING OVERTAKEN", "stand on: keep course and speed"),
    "none": ("NO RISK OF COLLISION", "following the route"),
}

# README "Results" table (examples/asv/README.md, 40 seeded runs per scenario, seeds 50000-50039). Copied here, not recomputed; the file says where they come from.
MC_ROWS = [
    ("patrol", "100 % (40/40; 91-100)", "0", "2.7 / 3.2 / 3.9", "to a float; 6 m wanted"),
    ("head-on", "100 % (40/40; 91-100)", "0", "17.2 / 19.3 / 22.5", ""),
    ("crossing, give way", "98 % (39/40; 87-100)", "0", "10.4 / 12.0 / 13.4", "1 Rule failure, 12.3 m"),
    ("crossing, stand on", "100 % (40/40; 91-100)", "0", "8.5 / 10.2 / 13.5", ""),
    ("overtaking", "100 % (40/40; 91-100)", "0", "8.7 / 8.9 / 9.6", "12 m wanted"),
    ("three vessels", "100 % (40/40; 91-100)", "0", "5.8 / 7.4 / 10.8", "smallest of three"),
    ("stand-on, other does NOT comply", "92 % (37/40; 80-97)", "0", "4.3 / 5.1 / 6.1", "3 of 40 inside 5 m"),
]  # fmt: skip


def fonts():
    from PIL import ImageFont

    reg, bold, mono = (
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf",
    )
    f = ImageFont.truetype
    return dict(
        xs=f(reg, 12),
        s=f(reg, 15),
        m=f(reg, 18),
        b=f(bold, 20),
        bb=f(bold, 28),
        t=f(bold, 44),
        h=f(bold, 15),
        mono=f(mono, 14),
        mono_b=f(mono, 16),
    )


def smooth(u):
    u = min(max(u, 0.0), 1.0)
    return u * u * (3 - 2 * u)


def rundir(root, case):
    ds = sorted(d for d in (Path(root) / "runs" / case).glob("asv-*") if d.is_dir())
    if not ds:
        raise SystemExit(f"no run of {case} under {root}/runs/{case}")
    return ds[-1]


# ---- run: the real scenario ---------------------------------------------------------------------------------------------------------------
def cmd_run(a):
    from asv.mc import hs_for
    from napl import program

    kind, seed, complies, *_ = CASES[a.case]
    seed = a.seed or seed
    hs = hs_for(seed)
    os.environ["NAPL_PARAMS"] = json.dumps(
        dict(
            seed=seed,
            hs=hs,
            jitter=1.0,
            ais=True,
            traffic_complies=complies,
            visual=True,
        )
    )
    sc = REPO / "examples" / "asv" / f"asv-{kind.replace('_', '-')}.scenario.toml"
    res, rd = program.run(sc, Path(a.root) / "runs" / a.case)
    mons = {
        m["name"]: dict(final=m.get("final"), ok=m["pass"]) for m in res["monitors"]
    }
    summary = {
        "case": a.case,
        "kind": kind,
        "seed": seed,
        "hs": round(hs, 3),
        "traffic_complies": complies,
        "run_dir": str(rd),
        "pass": res["pass"],
        "fail_reasons": res["fail_reasons"],
        "sim": res["sim"],
        "replay": res["replay"],
        "wall_s": round(res["program"]["wall_s"], 1),
        "monitors": mons,
    }
    (Path(rd) / "video_case.json").write_text(
        json.dumps(summary, indent=1, default=float)
    )
    print(json.dumps(summary, default=float))
    sys.exit(0 if res["pass"] and res["replay"].get("match") else 1)


# ---- camera --------------------------------------------------------------------------------------------------------------------------------
def projector(pos, look, fovy, w=W, h=H):
    """-> f(point) = (px, py, depth) or None behind the camera. World +z up; fovy vertical, like Sim.render."""
    pos, look = np.asarray(pos, float), np.asarray(look, float)
    f = look - pos
    f /= np.linalg.norm(f)
    r = np.cross(f, [0.0, 0.0, 1.0])
    if np.linalg.norm(r) < 1e-6:
        r = np.cross(f, [0.0, 1.0, 0.0])
    r /= np.linalg.norm(r)
    u = np.cross(r, f)
    th = math.tan(math.radians(fovy) / 2)

    def proj(p):
        d = np.asarray(p, float) - pos
        z = float(d @ f)
        if z < 0.3:
            return None
        return (
            w / 2 * (1 + float(d @ r) / (z * th * w / h)),
            h / 2 * (1 - float(d @ u) / (z * th)),
            z,
        )

    return proj


def cam_mode(mode, S, tv):
    """-> (pos, look, fovy) of one camera mode. S: dict(p ASV (x, y, 0), h unit heading, side unit to port, psi math yaw, c centre of the encounter, sep, q focus point | None)."""
    p, h, c, sep = S["p"], S["h"], S["c"], S["sep"]
    side = np.array([-h[1], h[0], 0.0])
    if mode == "top":
        ht = float(np.clip(1.25 * sep + 14.0, 24.0, 130.0))
        # north-up map view, tilted 6 degrees: an exactly vertical view makes the renderer ignore the camera's roll (checked), and heading-up would spin the picture in every turn
        return c + np.array([0.0, -0.1 * ht, ht]), c, 55.0
    if mode == "orbit":
        r, el = (
            float(np.clip(sep + 10.0, 16.0, 70.0)),
            math.radians(35.0),
        )  # far enough that both vessels stay in view at any azimuth
        az = S["psi"] + math.pi + 0.6 + 0.25 * tv
        return (
            c
            + r
            * np.array(
                [math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el)]
            ),
            c,
            55.0,
        )
    if mode == "pan":  # abeam and low, drifting along the heading
        d = float(np.clip(0.8 * sep + 12.0, 14.0, 55.0))
        return (
            c - side * d + h * (9.0 * math.sin(0.16 * tv)) + np.array([0.0, 0.0, 6.0]),
            c,
            60.0,
        )
    q = S[
        "q"
    ]  # chase: behind and above the ASV, looking along the route towards the encounter
    look = p + (
        np.clip(0.45 * np.linalg.norm(q - p), 6.0, 30.0)
        * (q - p)
        / max(np.linalg.norm(q - p), 1e-6)
        if q is not None
        else 6.0 * h
    )
    return p - 7.0 * h + 1.8 * side + np.array([0.0, 0.0, 3.2]), look, 55.0


class Rig:
    """Camera schedule [(sim time, mode)] with 2.5 s blends and a short low-pass on the result."""

    def __init__(self, sched, blend=2.5, tau=0.35):
        self.sched, self.blend, self.tau, self.cur = sched, blend, tau, None

    def at(self, t, S, tv, dt):
        ts = [s[0] for s in self.sched]
        k = max(bisect.bisect_right(ts, t) - 1, 0)
        a = cam_mode(self.sched[k][1], S, tv)
        if (
            k + 1 < len(ts) and t > ts[k + 1] - self.blend / 2
        ):  # blend towards the next mode before the switch
            u = smooth((t - (ts[k + 1] - self.blend / 2)) / self.blend)
            b = cam_mode(self.sched[k + 1][1], S, tv)
            a = tuple(x * (1 - u) + y * u for x, y in zip(a, b))
        if k > 0 and t < ts[k] + self.blend / 2:  # ... and out of the previous one
            u = smooth((t - (ts[k] - self.blend / 2)) / self.blend)
            b = cam_mode(self.sched[k - 1][1], S, tv)
            a = tuple(x * u + y * (1 - u) for x, y in zip(a, b))
        pos, look, fov = (np.asarray(x, float) for x in a)
        if self.cur is None:
            self.cur = (pos, look)
        else:
            g = 1 - math.exp(-dt / self.tau)
            self.cur = (
                self.cur[0] + (pos - self.cur[0]) * g,
                self.cur[1] + (look - self.cur[1]) * g,
            )
        return self.cur[0], self.cur[1], float(fov)


# ---- the run's data -------------------------------------------------------------------------------------------------------------------------
def enu(ll):
    from asv.geo import enu_from_latlon

    return enu_from_latlon(ll[0], ll[1])


class Data:
    """telemetry.jsonl + trace of one run: truth tracks (pose, traffic) and the plan of the video window / speed / cameras."""

    def __init__(self, rd):
        self.rd = Path(rd)
        self.tel = [
            json.loads(x)
            for x in (self.rd / "telemetry.jsonl").read_text().splitlines()
            if x.strip()
        ]
        self.t = np.array([r["t"] for r in self.tel])
        self.pos = np.array([[r["pose"]["x_m"], r["pose"]["y_m"]] for r in self.tel])
        self.names = (
            [x["name"] for x in self.tel[0]["truth"]["traffic"]]
            if self.tel[0].get("truth", {}).get("traffic")
            else []
        )
        self.tpos = {
            n: np.array([enu(r["truth"]["traffic"][k]["lat_lon"]) for r in self.tel])
            for k, n in enumerate(self.names)
        }
        self.dist = {n: np.hypot(*(self.tpos[n] - self.pos).T) for n in self.names}

    def rec(self, t):
        return self.tel[max(bisect.bisect_right(self.t, t) - 1, 0)]

    def plan(self, kind):
        """-> dict(w0, w1, cpa {name: t}, man, speed(t), sched [(t, mode)])."""
        end = float(self.t[-1])
        cpa = {n: float(self.t[int(np.argmin(self.dist[n]))]) for n in self.names}
        risk = [r["t"] for r in self.tel if any(x.get("risk") for x in r["tracks"])]
        t_risk = risk[0] if risk else None
        offs = [
            r["t"]
            for r in self.tel
            if t_risk is not None
            and r["t"] >= t_risk
            and abs(r["sp"]["course_offset_deg"]) > 3.0
        ]  # the first manoeuvre after a risk of collision was seen (small offsets before it are route keeping around fixed objects)
        if kind == "patrol" or not self.names:
            w0, w1, zones = 6.0, end, []
            man = None
        else:
            man = offs[0] if offs else min(cpa.values()) - 14.0
            w0 = max(0.0, (t_risk if t_risk is not None else man) - 8.0)
            w1 = min(end, max(cpa.values()) + 12.0)
            zones = [(man - 2.0, man + 8.0)] + [
                (c - 8.0, c + 6.0) for c in cpa.values()
            ]

        def speed(t):
            if kind == "patrol" or not zones:
                return 3.0
            d = min(
                0.0 if lo <= t <= hi else min(abs(t - lo), abs(t - hi))
                for lo, hi in zones
            )
            return 1.0 + 2.5 * smooth(d / 5.0)

        cs = sorted(cpa.values())
        if kind == "patrol":
            sched = [
                (w0, "chase"),
                (w0 + 0.2 * (w1 - w0), "top"),
                (w0 + 0.45 * (w1 - w0), "orbit"),
                (w0 + 0.7 * (w1 - w0), "pan"),
                (w0 + 0.85 * (w1 - w0), "chase"),
            ]
        else:
            sched = [(w0, "chase"), (man + 6.0, "top")]
            for i, c in enumerate(cs):
                sched += [
                    (c - 6.0, "orbit"),
                    (c + 7.0, "top" if i + 1 < len(cs) else "pan"),
                ]
        sched = sorted(sched)
        out = [
            s
            for i, s in enumerate(sched)
            if i == 0 or i + 1 == len(sched) or sched[i + 1][0] - s[0] >= 5.0
        ]  # a mode that would last under 5 s is skipped (the next one starts instead)
        return dict(w0=w0, w1=w1, cpa=cpa, man=man, speed=speed, sched=out)


# ---- the HUD ---------------------------------------------------------------------------------------------------------------------------------
def panel(d, box, fill=(8, 12, 18, 205), line=(255, 255, 255, 70)):
    d.rounded_rectangle(box, radius=6, fill=fill, outline=line, width=1)


def tag(d, F, xy, text, col, font="s", anchor="mb"):
    x0, y0, x1, y1 = d.textbbox(xy, text, font=F[font], anchor=anchor)
    d.rounded_rectangle((x0 - 5, y0 - 3, x1 + 5, y1 + 3), radius=4, fill=(0, 0, 0, 190))
    d.text(xy, text, font=F[font], fill=col, anchor=anchor)


def vessel_info(rec, tpos_now):
    """Per traffic vessel: the ASV's track of it (nearest track within 15 m of the vessel's true position) -> dict(role, rule, cpa, tcpa, range)."""
    out = {}
    for n, p in tpos_now.items():
        best = None
        for tr in rec["tracks"]:
            dd = math.hypot(tr["x_m"] - p[0], tr["y_m"] - p[1])
            if dd < 15.0 and (best is None or dd < best[0]):
                best = (dd, tr)
        out[n] = best[1] if best else None
    return out


def draw_hud(img, F, S):
    """img: (H, W, 4) uint8 render. S: everything the HUD needs (see cmd_sim). -> RGBA PIL image."""
    from PIL import Image, ImageDraw

    base = Image.fromarray(img).convert("RGBA")
    ov = Image.new("RGBA", base.size, (0, 0, 0, 0))
    d = ImageDraw.Draw(ov)
    proj, rec, meta = S["proj"], S["rec"], S["meta"]

    # trails and the plan on the water (world z 0.05), the lidar returns at the lidar height
    def poly(pts, col, w):
        q = [proj((x, y, 0.05)) for x, y in pts]
        q = [(p[0], p[1]) for p in q if p]
        if len(q) > 1:
            d.line(q, fill=col, width=w)

    poly(S["trail_asv"], (255, 255, 255, 150), 2)
    for n, tr in S["trail_tr"].items():
        poly(tr, (150, 190, 255, 140), 2)
    poly(S["traj"], CYAN + (230,), 3)
    for pt in S["lidar_pts"]:
        p = proj(pt)
        if p:
            d.ellipse((p[0] - 2, p[1] - 2, p[0] + 2, p[1] + 2), fill=(255, 80, 70, 230))
    # fixed objects the lidar must avoid: floats and channel buoys within 45 m
    for nm, (fx, fy) in S["fixed"]:
        dd = math.hypot(fx - S["p"][0], fy - S["p"][1])
        pr = proj((fx, fy, 0.5)) if dd < 45.0 else None
        if pr:
            d.ellipse(
                (pr[0] - 9, pr[1] - 9, pr[0] + 9, pr[1] + 9),
                outline=(255, 150, 60, 255),
                width=2,
            )
            tag(
                d,
                F,
                (pr[0], pr[1] - 12),
                f"{nm.upper()}  {dd:.0f} m",
                (255, 170, 90),
                "xs",
            )
    # vessel markers and tags
    infos = S["info"]
    for n, p3 in [("ASV", S["p"])] + list(S["tpos_now"].items()):
        z0 = p3 if len(p3) == 3 else (p3[0], p3[1], 0.0)
        pr = proj((z0[0], z0[1], 0.9))
        if not pr:
            continue
        col, extra = WHITE, ""
        if n != "ASV":
            tr = infos.get(n)
            col = ROLE_COL.get(tr["role"], GREY) if tr else GREY
            rng = math.hypot(z0[0] - S["p"][0], z0[1] - S["p"][1])
            extra = f"  {rng:.0f} m"
        d.ellipse(
            (pr[0] - 9, pr[1] - 9, pr[0] + 9, pr[1] + 9), outline=col + (255,), width=3
        )
        tag(
            d,
            F,
            (pr[0], pr[1] - 12),
            ("ASV" if n == "ASV" else n.upper()) + extra,
            col,
            "s",
        )
    # banner: scenario, rule, state
    risky = [tr for tr in rec["tracks"] if tr.get("risk")]
    ahead = [
        tr for tr in risky if tr["tcpa_s"] is not None and tr["tcpa_s"] >= 0
    ]  # closest point still to come
    crit = (
        min(ahead, key=lambda tr: tr["tcpa_s"])
        if ahead
        else min(risky, key=lambda tr: tr["range_m"], default=None)
    )
    rule = crit["rule"] if crit else "none"
    role = crit["role"] if crit else "none"
    head, sub = RULE_TXT.get(rule, RULE_TXT["none"])
    if meta["kind"] == "patrol":
        head, sub = (
            "NO VESSEL TRAFFIC",
            "follows its route and keeps clear of the floats and the shore",
        )
    off = rec["sp"]["course_offset_deg"]
    col = ROLE_COL.get(role, GREY)
    panel(d, (16, 16, 560, 128))
    d.text((30, 24), meta["title"], font=F["h"], fill=(255, 255, 255, 255))
    d.text((30, 46), head, font=F["bb"], fill=col + (255,))
    d.text((30, 82), sub, font=F["s"], fill=(235, 240, 245, 255))
    if rec["emergency"]:
        state, scol = "RULE 17(b): the stand-on vessel acts", RED
    elif role == "stand_on" and abs(off) > 3.0:
        state, scol = "RULE 17(a)(ii): stand-on vessel acts alone", RED
    elif abs(off) > 3.0:
        state, scol = (
            f"manoeuvring: {abs(off):.0f} deg to {'port' if off > 0 else 'starboard'} of the leg",  # course_offset_deg is a math-yaw offset: + = counter-clockwise = port
            AMBER,
        )
    elif (
        role == "give_way" and abs(rec["mission"]["xte_m"]) > 3.0
    ):  # the planner's parallel-line manoeuvres hold the heading but leave the route
        xte = rec["mission"]["xte_m"]  # + = left of the leg
        state, scol = (
            f"giving way: {abs(xte):.0f} m to {'port' if xte > 0 else 'starboard'} of the route",
            AMBER,
        )
    else:
        state, scol = "keeping course and speed", GREEN
    d.text((30, 102), state, font=F["m"], fill=scol + (255,))
    # right: ASV numbers + contact table
    px0 = W - 16 - 400
    panel(d, (px0, 16, W - 16, 44 + 22 * (1 + max(len(rec["tracks"]), 1)) + 8))
    d.text(
        (px0 + 12, 22),
        f"ASV  SOG {rec['pose']['sog_ms']:.1f} m/s   HDG {rec['pose']['heading_deg']:03.0f}   GNSS err {S['esterr']:.1f} m",
        font=F["mono"],
        fill=WHITE + (255,),
    )
    d.text(
        (px0 + 12, 44),
        "CONTACT   RNG   BRG   CPA  TCPA  ROLE",
        font=F["mono"],
        fill=GREY + (255,),
    )
    rows = sorted(rec["tracks"], key=lambda t: t["range_m"])[:5]
    if not rows:
        d.text((px0 + 12, 66), "no contact", font=F["mono"], fill=GREY + (255,))
    for i, tr in enumerate(rows):
        s = f"{tr['id']:<5}{tr['src'][:5]:<6}{tr['range_m']:4.0f}  {tr['bearing_deg']:+4.0f}  {tr['cpa_m']:4.0f}  {('%4.0f' % tr['tcpa_s']) if tr['tcpa_s'] is not None else '  - '}  {tr['role'].replace('_', ' ')}"
        d.text(
            (px0 + 12, 66 + 22 * i),
            s,
            font=F["mono"],
            fill=ROLE_COL.get(tr["role"], GREY) + (255,),
        )
    # the ASV's own picture: lidar returns + tracks + plan, heading up
    ppi(d, F, S)
    # clock, camera, honesty line
    tag(
        d,
        F,
        (W - 16, H - 16),
        f"t = {rec['t']:6.1f} s   x{S['speed']:.1f}   cam {S['mode']}   seed {meta['seed']}  Hs {meta['hs']:.2f} m",
        WHITE,
        "mono",
        "rb",
    )
    tag(
        d,
        F,
        (310, H - 16),
        "contacts from the lidar + simulated AIS; traffic is scripted",
        GREY,
        "xs",
        "lb",
    )
    if S["tv"] < 5.5:
        a = int(255 * min(1.0, (5.5 - S["tv"]) / 1.0))
        lines, cur = [], ""
        for w in meta["quote"].split():  # wrapped to the 920 px box
            if d.textlength(cur + " " + w, font=F["s"]) > 920 and cur:
                lines.append(cur)
                cur = w
            else:
                cur = (cur + " " + w).strip()
        lines.append(cur)
        y0 = H - 46 - 22 * len(lines) - 10
        d.rounded_rectangle(
            (310, y0, 1264, H - 46), radius=5, fill=(0, 0, 0, int(a * 0.8))
        )
        for i, ln in enumerate(lines):
            d.text(
                (322, y0 + 9 + 22 * i),
                ln,
                font=F["s"],
                fill=(255, 255, 255, a),
                anchor="la",
            )
    return Image.alpha_composite(base, ov)


PPI_R, PPI_RANGE = 135, 35.0


def ppi(d, F, S):
    rec = S["rec"]
    cx, cy = 16 + PPI_R + 4, H - 40 - PPI_R - 4
    k = PPI_R / PPI_RANGE
    panel(d, (16, cy - PPI_R - 22, 16 + 2 * PPI_R + 8, cy + PPI_R + 6))
    d.text(
        (24, cy - PPI_R - 18),
        "ASV'S VIEW (lidar + tracks), heading up",
        font=F["xs"],
        fill=GREY + (255,),
    )
    for rr in (10, 20, 30):
        d.ellipse(
            (cx - rr * k, cy - rr * k, cx + rr * k, cy + rr * k),
            outline=(255, 255, 255, 45),
        )
    for dx, dy in S["ppi_hits"]:  # (right, forward) metres
        if math.hypot(dx, dy) < PPI_RANGE:
            d.ellipse(
                (
                    cx + dx * k - 1.5,
                    cy - dy * k - 1.5,
                    cx + dx * k + 1.5,
                    cy - dy * k + 1.5,
                ),
                fill=(255, 255, 255, 200),
            )
    pts = [
        (cx + dx * k, cy - dy * k)
        for dx, dy in S["ppi_traj"]
        if math.hypot(dx, dy) < PPI_RANGE
    ]
    if len(pts) > 1:
        d.line(pts, fill=CYAN + (230,), width=2)
    for tr in rec["tracks"]:
        b, r = math.radians(tr["bearing_deg"]), tr["range_m"]
        if r > PPI_RANGE:
            continue
        x, y = cx + r * k * math.sin(b), cy - r * k * math.cos(b)
        col = ROLE_COL.get(tr["role"], GREY) + (255,)
        d.polygon([(x, y - 6), (x + 6, y), (x, y + 6), (x - 6, y)], outline=col)
        v = S["ppi_vel"].get(tr["id"])
        if v:
            d.line([(x, y), (x + v[0] * k * 20, y - v[1] * k * 20)], fill=col, width=1)
        d.text((x + 8, y - 14), f"#{tr['id']}", font=F["xs"], fill=col)
    d.polygon([(cx, cy - 8), (cx + 5, cy + 6), (cx - 5, cy + 6)], fill=WHITE + (255,))


# ---- sim: replay + render + HUD ---------------------------------------------------------------------------------------------------------------
def cmd_sim(a):
    import napl
    from asv.perception import scan_points
    from napl import _napl  # noqa: F401  (import check: the kernel must be there)
    from screenshots import Replay

    kind, seed, _, title, quote = CASES[a.case]
    rd = rundir(a.root, a.case)
    out = Path(a.root) / "sim" / a.case
    out.mkdir(parents=True, exist_ok=True)
    data = Data(rd)
    pl = data.plan(kind)
    vc = (
        json.loads((rd / "video_case.json").read_text())
        if (rd / "video_case.json").exists()
        else {}
    )
    meta = dict(
        case=a.case,
        kind=kind,
        seed=vc.get("seed", seed),
        hs=vc.get("hs", 0.0),
        title=title,
        quote=quote,
    )
    from asv import encounters as enc

    spec = enc.make(
        kind, meta["seed"], meta["hs"], 1.0
    )  # as run.py: the floats' positions
    fixed = [(o.name, tuple(o.pos)) for o in spec.obstacles] + [
        (n, tuple(xy)) for n, xy, _ in enc.BUOYS
    ]
    napl.render_style("outdoor,shadows")
    r = Replay(rd)
    r.dt = (
        0.002  # the kernel step (assemblies/asv.toml), as examples/asv/render_frames.py
    )
    F = fonts()
    rig = Rig(pl["sched"])
    t = pl["w0"] if a.t0 is None else a.t0
    t1 = pl["w1"] if a.t1 is None else a.t1
    k, tv, frames_t = a.start, 0.0, []
    tr_names = data.names
    while t < t1 and (a.max_frames is None or k - a.start < a.max_frames):
        sp = pl["speed"](t)
        r.to(t)
        rec = data.rec(t)
        hull = np.asarray(r.sim.query("/world/asv_hull", "pos")[0], float)
        q = np.asarray(r.sim.query("/world/asv_hull", "quat")[0], float)
        psi = math.atan2(
            2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] ** 2 + q[3] ** 2)
        )
        h = np.array([math.cos(psi), math.sin(psi), 0.0])
        p = np.array([hull[0], hull[1], 0.0])
        tnow = {
            n: np.asarray(r.sim.query(f"/world/{n}_hull", "pos")[0], float)
            for n in tr_names
        }
        near = (
            min(tnow, key=lambda n: np.hypot(*(tnow[n][:2] - p[:2]))) if tnow else None
        )
        if near and np.hypot(*(tnow[near][:2] - p[:2])) < 150.0:
            qf = np.array([tnow[near][0], tnow[near][1], 0.0])
            Sc = dict(
                p=p,
                h=h,
                psi=psi,
                c=(p + qf) / 2,
                sep=float(np.linalg.norm(qf - p)),
                q=qf,
            )
        else:
            Sc = dict(p=p, h=h, psi=psi, c=p + 8.0 * h, sep=12.0, q=None)
        pos, look, fov = rig.at(t, Sc, tv, 1.0 if a.step else 1.0 / FPS)
        mode = (
            next(m for tt, m in reversed(pl["sched"]) if tt <= t)
            if t >= pl["sched"][0][0]
            else pl["sched"][0][1]
        )
        rgba, _ = r.sim.render(pos=pos, lookat=look, fovy=fov, width=W, height=H)
        proj = projector(pos, look, fov)
        hits, ppi_hits = [], []
        # the lidar's own latest scan (the replayed device state), drawn from the TRUE pose it was taken at
        wpts, bpts, *_ = scan_points(
            r.sim.query("/devices/asv_lidar", "range")[0], psi, p[:2]
        )
        hits = [(x, y, 0.85) for x, y in wpts]
        ppi_hits = [(-y, x) for x, y in bpts]  # (right, forward) m
        es = rec["est"]
        e_xy = np.array([es["x_m"], es["y_m"]])
        t_xy = np.array([rec["pose"]["x_m"], rec["pose"]["y_m"]])
        psi_e = math.radians(90.0 - es["heading_deg"])
        traj, ppi_traj = [], []
        for ll in rec["traj"]:
            w = np.array(enu(ll)) - e_xy
            traj.append(tuple(t_xy + w))
            ppi_traj.append(
                (
                    w[0] * math.sin(psi_e) - w[1] * math.cos(psi_e),
                    w[0] * math.cos(psi_e) + w[1] * math.sin(psi_e),
                )
            )
        vel = {}
        for tr in rec["tracks"]:
            if tr["cog_deg"] is not None:
                c = math.radians(tr["cog_deg"])
                vx, vy = tr["sog_ms"] * math.sin(c), tr["sog_ms"] * math.cos(c)
                vel[tr["id"]] = (
                    vx * math.sin(psi_e) - vy * math.cos(psi_e),
                    vx * math.cos(psi_e) + vy * math.sin(psi_e),
                )
        i0 = bisect.bisect_left(data.t, t - 30.0)
        i1 = bisect.bisect_right(data.t, t)
        S = dict(
            proj=proj, rec=rec, meta=meta, p=p, tpos_now={n: tnow[n] for n in tr_names}, info=vessel_info(rec, {n: tnow[n][:2] for n in tr_names}),
            trail_asv=[tuple(x) for x in data.pos[i0:i1:2]], trail_tr={n: [tuple(x) for x in data.tpos[n][i0:i1:2]] for n in tr_names},
            traj=traj, lidar_pts=hits, ppi_hits=ppi_hits, ppi_traj=ppi_traj, ppi_vel=vel, speed=sp, mode=mode, tv=tv,
            esterr=float(np.hypot(*(e_xy - t_xy))), fixed=fixed,
        )  # fmt: skip
        img = draw_hud(np.ascontiguousarray(rgba), F, S)
        videnc.save_frame(out, f"f{k:05d}", np.asarray(img.convert("RGB")))
        frames_t.append(round(t, 3))
        k += 1
        tv += 0.3 if a.step else 1.0 / FPS
        t += a.step or sp / FPS
    done = t >= t1
    h_, ok = r.finish() if done and a.t1 is None and a.t0 is None else (None, None)
    m = dict(
        meta,
        w0=pl["w0"],
        w1=pl["w1"],
        cpa_t=pl["cpa"],
        man_t=pl["man"],
        sched=pl["sched"],
        frames=k - a.start,
        start=a.start,
        frames_t=frames_t,
        replay_hash=h_,
        replay_matches_run=ok,
        run=str(rd),
    )
    (out / f"meta_{a.start:05d}.json").write_text(
        json.dumps(m, indent=1, default=float)
    )
    print(
        json.dumps(
            {
                x: m[x]
                for x in (
                    "case",
                    "seed",
                    "hs",
                    "w0",
                    "w1",
                    "frames",
                    "replay_hash",
                    "replay_matches_run",
                )
            },
            default=float,
        )
    )
    if ok is False:
        raise SystemExit("replay hash differs from the run's")


# ---- build: the vessel revealed part by part ---------------------------------------------------------------------------------------------------
STAGES = [
    (
        "HULL",
        [
            (
                "hull",
                "twin-hull catamaran, 1.2 m, a skeg under each stern (directional stability)",
                "estimated strip-theory coefficients",
            )
        ],
    ),
    (
        "PROPULSION",
        [
            (
                "thr1",
                "Blue Robotics T200 thruster, port: 3.71 kgf fwd / 2.92 kgf rev at 12 V",
                "USD 230",
            ),
            ("prop1", "T200 propeller, port", ""),
            ("thr2", "Blue Robotics T200 thruster, starboard", "USD 230"),
            ("prop2", "T200 propeller, starboard", ""),
        ],
    ),
    (
        "POWER",
        [
            ("bat1", "Gens ace 5300 mAh 3S 60C LiPo, port hull", "about USD 49"),
            ("bat2", "Gens ace 5300 mAh 3S 60C LiPo, starboard hull", "about USD 49"),
            ("esc1", "Blue Robotics Basic ESC, bidirectional 30 A, port", "USD 40"),
            ("esc2", "Blue Robotics Basic ESC, starboard", "USD 40"),
            ("reg", "Pololu D24V50F5 5 V regulator for the avionics", "USD 32.95"),
        ],
    ),
    (
        "AVIONICS",
        [
            ("fc", "Holybro Pixhawk 6C autopilot (PX4)", "USD 199"),
            ("pi", "Raspberry Pi 5 companion computer", "about USD 200"),
            ("sik", "Holybro SiK telemetry radio V3", "USD 58.99"),
        ],
    ),
    (
        "SENSORS",
        [
            ("mast1", "lidar mast, 0.45 m", ""),
            ("lidar", "Slamtec RPLIDAR S2: 30 m, 10 Hz, 360 degrees", "about USD 355"),
            ("mast2", "GNSS mast, 0.30 m", ""),
            ("gps", "Holybro M8N GPS + IST8310 compass", "USD 51.59"),
        ],
    ),
    (
        "WIRING",
        [
            (
                "wires",
                "power, signal and data leads routed around the parts",
                "DRC clean",
            )
        ],
    ),
]


def pruned_xml(src, dst, shown, wires):
    """Writes dst = the scene with the geoms of the bodies not in `shown` (and the wire geoms unless `wires`) made fully transparent. dst sits next to src: mesh paths are relative."""
    tree = ET.parse(src)
    root = tree.getroot()
    for body in root.iter("body"):
        nm = body.get("name", "")
        iid = "hull" if nm == "asv_hull" else nm[4:] if nm.startswith("asv/") else None
        if iid is None:
            continue
        for g in list(body.findall("geom")):
            gn = g.get("name", "")
            if (gn.startswith("asv/wire") and not wires) or (
                not gn.startswith("asv/wire") and iid not in shown
            ):
                g.set(
                    "rgba", "0 0 0 0"
                )  # removing the geoms crashes the solver on a body left without any; alpha 0 draws nothing (checked)
    tree.write(dst)


def cmd_build(a):
    import napl
    from asv import encounters as enc
    from asv import scene
    from napl import Sim, design
    from napl.design import Design
    from napl.design import assembly as A
    from PIL import Image, ImageDraw

    out = Path(a.root) / "build"
    out.mkdir(parents=True, exist_ok=True)
    model = Path(a.root) / "build_model"
    path = scene.build_spec(
        model, enc.make("patrol", seed=1, hs=0.0), visual=True
    )  # the harbour scene with the ASV alone, flat water
    asm = A.load(REPO / "assemblies" / "asv.toml")
    _, rep = design.compute(
        asm, REPO / "parts", Path(a.root) / "build_report", visual=False, route=False
    )
    chk = Design.open(REPO / "assemblies" / "asv.toml", REPO / "parts").check()
    n_err = sum(f.severity == "error" for f in chk)
    n_warn = sum(f.severity == "warning" for f in chk)
    mass = rep["mass"]["by_instance"]
    wire_m = sum(v for k, v in mass.items() if k.startswith("wire:"))
    napl.render_style("outdoor,shadows")
    F = fonts()
    n_parts = sum(len(s[1]) for s in STAGES if s[0] != "WIRING")
    shown, k, n, tv = [], 0, 0, 0.0
    mtot = 0.0
    steps = [(si, st, p) for si, (st, parts) in enumerate(STAGES) for p in parts]
    for idx, (si, stage, (iid, desc, price)) in enumerate(steps):
        wires = iid == "wires"
        if not wires:
            shown.append(iid)
            n += 1
            mtot += mass.get(iid, 0.0)
        else:
            mtot += wire_m
        xml = model / "stage.xml"
        pruned_xml(
            path, xml, set(shown), wires or any(s[2][0] == "wires" for s in steps[:idx])
        )
        sim = Sim.load(xml)
        body = np.asarray(sim.query("/world/asv_hull", "pos")[0], float)
        pbody = np.asarray(
            sim.query(
                "/world/asv_hull"
                if iid in ("hull", "wires")
                else next(p for p in sim.body_positions() if p.endswith("/" + iid)),
                "pos",
            )[0],
            float,
        )
        frames = int((3.0 if idx == 0 else 1.5 if not wires else 4.0) * FPS)
        if idx == len(steps) - 1:
            frames = int(7.0 * FPS)
        for j in range(frames):
            u = tv / 38.0
            az = math.radians(20.0 + 330.0 * u)
            el = math.radians(32.0 - 12.0 * math.sin(math.pi * u))
            dist = 2.3 - 0.55 * smooth(
                (tv - 24.0) / 8.0
            )  # closing in on the finished boat and its wiring
            tgt = body + np.array([0.0, 0.0, 0.18])
            pos = tgt + dist * np.array(
                [math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el)]
            )
            rgba, _ = sim.render(pos=pos, lookat=tgt, fovy=45.0, width=W, height=H)
            proj = projector(pos, tgt, 45.0)
            base = Image.fromarray(np.ascontiguousarray(rgba)).convert("RGBA")
            ov = Image.new("RGBA", base.size, (0, 0, 0, 0))
            d = ImageDraw.Draw(ov)
            panel(d, (16, 16, 640, 74))
            d.text(
                (30, 22),
                "BUILD  an ASV from catalogue parts (napl.design, assemblies/asv.toml)",
                font=F["h"],
                fill=WHITE + (255,),
            )
            d.text((30, 44), f"{stage}", font=F["bb"], fill=CYAN + (255,))
            panel(d, (W - 316, 16, W - 16, 100))
            d.text(
                (W - 304, 22),
                f"parts placed   {n:2d} / {n_parts}",
                font=F["mono_b"],
                fill=WHITE + (255,),
            )
            d.text(
                (W - 304, 46),
                f"mass   {mtot:6.2f} kg"
                + ("" if idx == len(steps) - 1 else "  (so far)"),
                font=F["mono_b"],
                fill=WHITE + (255,),
            )
            d.text(
                (W - 304, 70),
                f"DRC  {n_err} errors, {n_warn} warnings"
                if idx == len(steps) - 1
                else "DRC  checked once complete",
                font=F["mono"],
                fill=(GREEN if n_err == 0 else RED)
                + ((255,) if idx == len(steps) - 1 else (150,)),
            )
            if not wires:
                pr = proj(pbody + np.array([0.0, 0.0, 0.05]))
                age = j / FPS
                if pr and age < 1.4:
                    a_ = int(255 * (1 - smooth((age - 0.9) / 0.5)))
                    d.ellipse(
                        (pr[0] - 16, pr[1] - 16, pr[0] + 16, pr[1] + 16),
                        outline=AMBER + (a_,),
                        width=3,
                    )
            cap = f"{iid}: {desc}" + (f"   ({price})" if price else "")
            tag(d, F, (W / 2, H - 22), cap, WHITE, "m", "mb")
            img = Image.alpha_composite(base, ov).convert("RGB")
            videnc.save_frame(out, f"f{k:05d}", np.asarray(img))
            k += 1
            tv += 1.0 / FPS
    (out / "meta.json").write_text(
        json.dumps(
            dict(
                frames=k,
                parts=n,
                mass_kg=round(mtot, 3),
                mass_report_total_kg=rep["mass"]["total"],
                steps=[s[2][0] for s in steps],
            ),
            indent=1,
        )
    )
    print(
        f"{k} build frames, {n} parts, mass {mtot:.2f} kg (report total {rep['mass']['total']}), DRC {n_err} errors {n_warn} warnings"
    )


# ---- encode ----------------------------------------------------------------------------------------------------------------------------------------
def text_card(F, title, lines, back=None, size=(W, H)):
    from PIL import Image, ImageDraw

    if back is not None:
        img = Image.fromarray(back).convert("RGB")
        img = Image.blend(img, Image.new("RGB", img.size, (6, 10, 16)), 0.78)
    else:
        img = Image.new("RGB", size, (8, 14, 22))
    d = ImageDraw.Draw(img)
    d.text((70, 60), title, font=F["t"], fill=WHITE)
    y = 140
    for ln in lines:
        if isinstance(ln, tuple):
            ln, font, col = ln
        else:
            font, col = "m", (225, 232, 240)
        d.text((70, y), ln, font=F[font], fill=col)
        y += 30 if font == "m" else 24 if font in ("s", "mono") else 34
    return img


def cmd_encode(a):
    from PIL import Image, ImageDraw

    root = Path(a.root)
    F = fonts()
    enc = videnc.Encoder(a.out, W, H, FPS, gpl=False)
    n = [0]

    def emit(img, secs=None):
        arr = np.asarray(img.convert("RGB") if hasattr(img, "convert") else img)
        for _ in range(1 if secs is None else int(secs * FPS)):
            enc.write(arr)
            n[0] += 1

    def frames(d):
        for s in videnc.frame_stems(d):
            enc.write(videnc.load_array(d, s)[..., :3].copy())
            n[0] += 1

    def fade(img_a, img_b, secs=0.6):
        for i in range(int(secs * FPS)):
            u = (i + 1) / (secs * FPS)
            emit(Image.blend(img_a, img_b, u))

    bstems = videnc.frame_stems(root / "build")
    hero = videnc.load_array(root / "build", bstems[-1])[..., :3]
    runs = {
        c: json.loads((rundir(root, c) / "video_case.json").read_text())
        for c in ORDER
        if (root / "runs" / c).exists()
    }

    title = text_card(
        F,
        "ASV: COLREG collision avoidance",
        [
            "A BlueBoat-class 1.2 m catamaran, designed from catalogue parts with napl.design,",
            "navigating on GNSS + IMU + compass only, seeing other vessels through its lidar and (simulated) AIS,",
            "and avoiding them under Rules 13-17: head-on, crossing, overtaking, stand-on, three vessels at once.",
            "",
            (
                "Every scenario here is a real napl run (physics, waves, sensors, planner); the video replays its log.",
                "s",
                (160, 175, 190),
            ),
            (
                "Stand-ins and limits are listed on the last card: read them.",
                "s",
                (160, 175, 190),
            ),
        ],
        back=hero,
    )
    emit(title, 5.5)
    fade(title, Image.fromarray(videnc.load_array(root / "build", bstems[0])[..., :3]))
    frames(root / "build")
    intro = text_card(
        F,
        "The encounters",
        [
            "Same vessel, same planner; seeds from the 50000 block that passed, on a calm sea (Hs 0.25-0.37 m).",
            "White line: where the ASV has been.  Cyan: the path it has chosen (60 s ahead).  Red dots: lidar returns.",
            "Amber = the ASV gives way.  Green = the ASV stands on.  Replays run x1 near the manoeuvre, faster elsewhere.",
            "",
            (
                "Rule thresholds (12 m wanted, 5 m judged, 15 degree head-on cone) are my choices; the Rules give no numbers.",
                "s",
                (160, 175, 190),
            ),
        ],
    )
    emit(intro, 4.5)
    for c in ORDER:
        if (root / "sim" / c).exists():
            frames(root / "sim" / c)
    # result cards
    img = Image.new("RGB", (W, H), (8, 14, 22))
    d = ImageDraw.Draw(img)
    d.text(
        (70, 40),
        "Results: 40 seeded runs per scenario (examples/asv/README.md)",
        font=F["bb"],
        fill=WHITE,
    )
    d.text(
        (70, 78),
        "seeds 50000-50039, jitter 1, Hs 0.2-0.8 m, AIS on; success = mission done + no collision + the Rule judgement",
        font=F["s"],
        fill=GREY,
    )
    ys = 116
    cols = (70, 420, 650, 740, 1030)
    for x, t in zip(
        cols,
        (
            "scenario",
            "success (95 % Wilson)",
            "collisions",
            "closest m: min / p10 / med",
            "note",
        ),
    ):
        d.text((x, ys), t, font=F["h"], fill=CYAN)
    ys += 28
    for r in MC_ROWS:
        for x, t in zip(cols, r):
            d.text((x, ys), t, font=F["s"], fill=WHITE)
        ys += 26
    for i, ln in enumerate(
        [
            "Not blind: three earlier seed blocks were flown and shaped the planner; 50000-50039 shaped nothing, and was flown once.",
            "40/40 only says the true rate is above 91 %; 95 % is not excluded for any row. Seed blocks vary by that much (stand-on: 40/40, 17/40, 35/40, 38/40, 40/40).",
            "Weak spots: overtaking passes at 8.7-9.6 m (12 m wanted); non-complying stand-on is inside 5 m in 3 of 40 (never under 4 m);",
            "patrol passes floats at 2.7-3.9 m (0.75 m is a collision) and one run of another block (seed 40010) did not finish; one run of 280 failed a Rule (crossing, 12.3 m).",
            "Tuned against one harbour, one speed, one hull. Not a real-ship collision-avoidance system: a 1.5 m/s craft in a 300 m harbour.",
        ]
    ):
        d.text((70, ys + 14 + 24 * i), ln, font=F["s"], fill=(235, 220, 190))
    emit(img, 24.0)
    img = Image.new("RGB", (W, H), (8, 14, 22))
    d = ImageDraw.Draw(img)
    d.text((70, 40), "The runs in this video", font=F["bb"], fill=WHITE)
    d.text(
        (70, 78),
        "each one a real `napl program` run, judged on the TRUE tracks; the replay of its log ends on the same state hash",
        font=F["s"],
        fill=GREY,
    )
    ys = 116
    cols = (70, 470, 560, 650, 790, 930, 1100)
    for x, t in zip(
        cols, ("scenario", "seed", "Hs m", "closest m", "monitors", "replay hash", "")
    ):
        d.text((x, ys), t, font=F["h"], fill=CYAN)
    ys += 30
    for c in ORDER:
        if c not in runs:
            continue
        v = runs[c]
        mons = v["monitors"]
        cpa = (mons.get("min_cpa_5m") or mons.get("clear_of_fixed_objects") or {}).get(
            "final"
        )
        npass = sum(m["ok"] for m in mons.values())
        row = (
            SHORT[c],
            str(v["seed"]),
            f"{v['hs']:.2f}",
            "-"
            if cpa is None
            else f"{cpa:.1f}" + (" (float)" if "min_cpa_5m" not in mons else ""),
            f"{npass}/{len(mons)} pass",
            v["sim"]["state_hash"]
            + ("  match" if v["replay"].get("match") else "  DIFFERS"),
        )
        for x, t in zip(cols, row):
            d.text((x, ys), t, font=F["mono_b"], fill=WHITE if v["pass"] else RED)
        ys += 30
    d.text(
        (70, ys + 20),
        "closest = the smallest centre-to-centre distance to any vessel (to a float on the leg, for the patrol), from the trace's true positions.",
        font=F["s"],
        fill=GREY,
    )
    d.text(
        (70, ys + 44),
        "One seed per scenario: these are not the success rates (see the previous card), and the seeds were picked for wide clearances.",
        font=F["s"],
        fill=(235, 220, 190),
    )
    emit(img, 14.0)
    notes = text_card(
        F,
        "What is real, what is a stand-in",
        [
            (
                "REAL: the physics (hull, thrusters, waves), the sensor models, the estimator, tracker, COLREG classifier, planner and judge; all are the scenario's own runs,",
                "s",
                WHITE,
            ),
            (
                "the replay of each run ends on the run's state hash (checked per scenario), and the monitors and the numbers on screen come from its own telemetry.",
                "s",
                WHITE,
            ),
            ("", "s", WHITE),
            (
                "STAND-IN: the autonomy runs on the host computer standing in for the Pi 5 + Pixhawk pair: no PX4 firmware flies the boat.",
                "s",
                AMBER,
            ),
            (
                "STAND-IN: the lidar is idealised (instant scan, no spray or clutter, one reflectivity) and sees other vessels only through a cabin proxy box on each hull;",
                "s",
                AMBER,
            ),
            (
                "the red dots and the heading-up display are the lidar's own latest scan in the replayed state, drawn from the true pose it was taken at.",
                "s",
                AMBER,
            ),
            (
                "STAND-IN: AIS on a 1.2 m craft is an idealisation (Class A rates / latency / 2 % loss; no range limit).",
                "s",
                AMBER,
            ),
            (
                "STAND-IN: the other vessels are scripted (same autopilot, steering on the truth): their Rule behaviour is a script, not a second system under test.",
                "s",
                AMBER,
            ),
            (
                "STAND-IN: the build segment reveals the finished assembly part by part (the design is one TOML, not typed live); the hull and skeg coefficients are estimates.",
                "s",
                AMBER,
            ),
            (
                "The sea is the physics JONSWAP sea; collisions are geometric (centre distances), there are no contact forces.",
                "s",
                WHITE,
            ),
            ("", "s", WHITE),
            (
                "napl: kernel + agent layer. Source and per-scenario numbers: examples/asv/README.md.",
                "s",
                (160, 175, 190),
            ),
        ],
    )
    emit(notes, 14.0)
    enc.close()
    print(
        f"wrote {a.out}: {n[0]} frames = {n[0] / FPS:.1f} s at {W}x{H}@{FPS}, {Path(a.out).stat().st_size / 1e6:.1f} MB, encoder {enc.backend}"
    )


def cmd_png(a):
    from PIL import Image

    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    for n in a.n:
        Image.fromarray(videnc.load_array(a.dir, f"f{int(n):05d}")[..., :3]).save(
            out / f"{Path(a.dir).name}_{int(n):05d}.png"
        )
    print(f"{len(a.n)} PNGs in {out}")


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    sub = ap.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("run")
    s.add_argument("case", choices=list(CASES))
    s.add_argument("root")
    s.add_argument("--seed", type=int, default=0, help="override the case's seed")
    s = sub.add_parser("sim")
    s.add_argument("case", choices=list(CASES))
    s.add_argument("root")
    s.add_argument("--t0", type=float, default=None)
    s.add_argument("--t1", type=float, default=None)
    s.add_argument(
        "--start",
        type=int,
        default=0,
        help="number of the first frame (chunks continue the numbering)",
    )
    s.add_argument("--max-frames", type=int, default=None)
    s.add_argument(
        "--step",
        type=float,
        default=0.0,
        help="preview: one frame every STEP sim seconds instead of the real time warp",
    )
    s = sub.add_parser("build")
    s.add_argument("root")
    s = sub.add_parser("encode")
    s.add_argument("root")
    s.add_argument("out")
    s = sub.add_parser("png")
    s.add_argument("dir")
    s.add_argument("out")
    s.add_argument("n", nargs="+")
    sub.add_parser("mc-seeds")
    a = ap.parse_args()
    if a.cmd == "mc-seeds":
        print(
            json.dumps(
                {
                    c: dict(kind=v[0], seed=v[1], complies=v[2])
                    for c, v in CASES.items()
                },
                indent=1,
            )
        )
        return
    globals()["cmd_" + a.cmd.replace("-", "_")](a)


if __name__ == "__main__":
    main()
