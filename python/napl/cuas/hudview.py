"""The counter-UAS HUD (after the Waterloo Defence sim): stage tabs + status line, counters, per-track range labels in the 3D view, tracer
lines, the TURRET CAMERA picture-in-picture with track boxes / crosshair / zoomed crop, an OPERATOR AUTHORIZED flash and hit / destroyed
marks -- as the generic items of `napl.hud` (the window knows nothing about turrets), built from one `hud.jsonl` record plus a little
live state (muzzle, pellets) and the run's events.

    doc = hudview.document(View(t=12.3, rec=rec, run_dir=run, muzzle=(pos, dir), pellets=[...], events=events, vp=(1280, 720)), seq=n)
    hud.set(doc)

`tools/record_sim.py` feeds it from a replay of a run (play / record); a program that owns a World can call the same function from its
`hud_callbacks`. Camera presets: `camera_for(preset, scene, prev, dt)` (overview / chase / target / top)."""

import json
import math
from dataclasses import dataclass, field
from pathlib import Path

from .. import hud as H

STAGES = ("SENSE", "TRACK", "PICK", "AIM", "FIRE")
AIM_TOL = 0.003  # rad, turret.py: the pointing error under which it may fire
PIP_FRAC, MARGIN = 0.21, 16  # the camera picture-in-picture is 21 % of the window width
SELECTED = (255, 255, 255)


@dataclass
class View:
    t: float  # sim time of this frame
    rec: dict = None  # latest hud.jsonl record at or before t (None before the first)
    run_dir: Path = None  # camera_frame paths are relative to it
    muzzle: tuple = None  # (pos, dir) of the launcher now
    pellets: list = field(
        default_factory=list
    )  # world positions of the pellets in flight now
    events: list = field(
        default_factory=list
    )  # events.json (any length: only the recent ones are shown)
    vp: tuple = (1280, 720)  # the window (viewport) size in pixels
    frame_px: tuple = (1280, 720)  # size of the turret camera frames


def _recent(events, t, kinds, span):
    return [e for e in events if e.get("type") in kinds and 0 <= t - e["t"] <= span]


def _alpha(age, span, fade=0.35):
    """1 until the last `fade` seconds of the span, then down to 0."""
    return max(0.0, min(1.0, (span - age) / fade))


def _a(col, k):
    return [*col[:3], int((col[3] if len(col) > 3 else 255) * k)]


def status_line(rec, auth_pending=False):
    stage, sel = rec["stage"], rec.get("selected")
    tr = next((x for x in rec["tracks"] if x["id"] == sel), None)
    n = sum(x["confirmed"] for x in rec["tracks"])
    if stage == "SENSE":
        return "Scanning the sky"
    if stage == "TRACK":
        return f"Tracking {len(rec['tracks'])} contact{'s' if len(rec['tracks']) != 1 else ''}, {n} confirmed"
    if tr is None:
        return ""
    if stage == "PICK":
        return f"Target {sel} selected at {tr['range_m']:.0f} m"
    err = (rec.get("lead") or {}).get("aim_err")
    if stage == "AIM":
        return (
            f"Centring {sel} in the camera"
            if err is None or err > 3 * AIM_TOL
            else f"Locked on {sel}, awaiting operator"
        )
    return f"Engaging {sel} at {tr['range_m']:.0f} m"


def _pip(v, ui):
    """The TURRET CAMERA panel: rectangle (x, y, w, h) in window pixels and its items."""
    W, Hh = v.vp
    pw = round(PIP_FRAC * W)
    fw, fh = v.frame_px
    ph = round(pw * fh / fw)
    m = MARGIN * ui
    x0, y0 = W - m - pw, Hh - m - ph
    rec = v.rec
    path = (
        v.run_dir / rec["camera_frame"]
        if rec and rec.get("camera_frame") and v.run_dir
        else None
    )
    inner = []
    if path is not None:
        inner.append(H.image(path, (0, 0), (fw, fh), id="pip"))
        for tk in rec["tracks"]:
            x0b, y0b, x1b, y1b = tk["bbox_px"]
            sel = tk["id"] == rec.get("selected")
            col = SELECTED if sel else H.CYAN
            inner.append(
                H.rect(
                    (x0b - 3, y0b - 3),
                    (x1b - x0b + 7, y1b - y0b + 7),
                    line=_a(col, 0.45 if tk.get("coasting") else 1),
                    lw=1.2,
                )
            )
            inner.append(
                H.text(tk["id"], (x0b - 3, y0b - 5), a="bl", size=10 * ui, col=col)
            )
        cx, cy, s = fw / 2, fh / 2, 16
        inner += [H.line([(cx - s, cy), (cx - 4, cy)], H.WHITE, 1.0), H.line([(cx + 4, cy), (cx + s, cy)], H.WHITE, 1.0),
                  H.line([(cx, cy - s), (cx, cy - 4)], H.WHITE, 1.0), H.line([(cx, cy + 4), (cx, cy + s)], H.WHITE, 1.0)]  # fmt: skip
    items = [
        H.group(
            inner,
            (x0, y0),
            (pw, ph),
            space=(fw, fh),
            fill=(8, 10, 12, 255),
            line=(255, 255, 255, 70),
            lw=1.0,
        )
    ]
    items.append(
        H.text(
            "TURRET CAMERA",
            (x0, y0 - 5 * ui),
            a="bl",
            size=11 * ui,
            col=H.WHITE,
            bg=H.BLACK,
            pad=4 * ui,
        )
    )
    sel = (
        next((t for t in rec["tracks"] if t["id"] == rec.get("selected")), None)
        if rec
        else None
    )
    if (
        sel is not None and path is not None
    ):  # the zoomed crop of the selected target, bottom-left of the panel, cyan border
        x0b, y0b, x1b, y1b = sel["bbox_px"]
        cw = max(112.0, 5.0 * max(x1b - x0b, (y1b - y0b) * fw / fh))
        ch = cw * fh / fw
        ccx = min(max((x0b + x1b) / 2, cw / 2), fw - cw / 2)
        ccy = min(max((y0b + y1b) / 2, ch / 2), fh - ch / 2)
        zw = round(0.34 * pw)
        zh = round(zw * fh / fw)
        zx, zy = x0 + 6 * ui, y0 + ph - zh - 6 * ui
        uv = [
            (ccx - cw / 2) / fw,
            (ccy - ch / 2) / fh,
            (ccx + cw / 2) / fw,
            (ccy + ch / 2) / fh,
        ]
        items += [H.image(path, (zx, zy), (zw, zh), uv=uv, id="crop"), H.rect((zx, zy), (zw, zh), line=H.CYAN, lw=1.5),
                  H.text(sel["id"], (zx + 3 * ui, zy + 2 * ui), size=10 * ui, col=H.CYAN)]  # fmt: skip
    return items


def items(v: View):
    ui = v.vp[1] / 720.0  # sizes are authored for a 720 px high window
    W, Hh = v.vp
    m = MARGIN * ui
    rec, t = v.rec, v.t
    out = []
    if rec is None:
        out.append(
            H.tabs(
                STAGES,
                -1,
                (m, m),
                size=13 * ui,
                status="Waiting for the first camera frame",
                pad=10 * ui,
                gap=22 * ui,
            )
        )
        return out
    stage = rec["stage"]
    out.append(
        H.tabs(
            STAGES,
            STAGES.index(stage),
            (m, m),
            size=13 * ui,
            status=status_line(rec),
            pad=10 * ui,
            gap=22 * ui,
        )
    )
    out.append(
        H.kv(
            [
                ("ROUNDS", rec["rounds"]),
                ("HITS", rec["hits"]),
                ("DESTROYED", rec["destroyed"]),
            ],
            (W - m, m),
            a="tr",
            size=13 * ui,
            pad=10 * ui,
            gap=26 * ui,
        )
    )

    # flashes: a granted authorization (top centre), a denied one, a hit / destroyed drone (under the counters)
    for e in _recent(v.events, t, ("authorization",), 1.6):
        k = _alpha(t - e["t"], 1.6)
        out.append(
            H.text(
                f"OPERATOR AUTHORIZED  {e['target']}",
                (W / 2, m),
                a="t",
                size=14 * ui,
                col=_a(H.GREEN, k),
                bg=_a(H.BLACK, k),
                pad=9 * ui,
            )
        )
    for e in _recent(v.events, t, ("authorization_request",), 1.4):
        if not e.get("granted"):
            k = _alpha(t - e["t"], 1.4)
            out.append(
                H.text(
                    f"AUTHORIZATION DENIED  {e['target']}",
                    (W / 2, m + 44 * ui),
                    a="t",
                    size=12 * ui,
                    col=_a(H.AMBER, k),
                    bg=_a(H.BLACK, k),
                    pad=7 * ui,
                )
            )
    ind = [
        e
        for e in _recent(v.events, t, ("hit", "destroyed"), 3.0)
        if e["type"] == "destroyed" or e.get("kind") == "drone"
    ]
    y = m + 96 * ui
    for e in sorted(ind, key=lambda e: e["t"])[-2:]:
        span = 3.0 if e["type"] == "destroyed" else 1.8
        if t - e["t"] > span:
            continue
        k = _alpha(t - e["t"], span)
        name = e["drone"].upper()
        if e["type"] == "destroyed":
            out.append(
                H.text(
                    f"DESTROYED  {name}",
                    (W - m, y),
                    a="tr",
                    size=13 * ui,
                    col=_a(H.RED, k),
                    bg=_a(H.BLACK, k),
                    pad=7 * ui,
                )
            )
        else:
            out.append(
                H.text(
                    f"HIT  {name}",
                    (W - m, y),
                    a="tr",
                    size=13 * ui,
                    col=_a(H.AMBER, k),
                    bg=_a(H.BLACK, k),
                    pad=7 * ui,
                )
            )
        y += 36 * ui
    for e in _recent(v.events, t, ("hit",), 0.9):
        if e.get("kind") == "drone":
            age = t - e["t"]
            out.append(
                H.wring(
                    e["pos"],
                    r=(6 + 26 * age) * ui,
                    col=_a(H.AMBER, _alpha(age, 0.9)),
                    lw=1.5,
                )
            )

    # the 3D view: a small black "A 24 m" label above each confirmed track, tracers from the muzzle
    for tk in rec["tracks"]:
        if not tk["confirmed"]:
            continue
        sel = tk["id"] == rec.get("selected")
        out.append(H.wtext(tk["world_xyz_est"], f"{tk['id']}  {tk['range_m']:.0f} m", off=(0, -14 * ui), size=12 * ui,
                           col=H.WHITE if sel else H.DIM, bg=_a(H.BLACK, 0.85 if sel else 0.7), pad=4 * ui))  # fmt: skip
    if v.muzzle is not None:
        mz = list(v.muzzle[0])
        sel = next(
            (tk for tk in rec["tracks"] if tk["id"] == rec.get("selected")), None
        )
        if sel is not None and stage in ("AIM", "FIRE"):
            out.append(
                H.wline([mz, sel["world_xyz_est"]], col=(255, 255, 255, 60), lw=1.0)
            )
        for p in v.pellets:
            out.append(H.wline([mz, list(p)], col=(255, 255, 255, 235), lw=1.2))
            out.append(H.wdot(p, r=2.2 * ui, col=H.WHITE))
    out += _pip(v, ui)
    return out


def document(v: View, seq=0, tick=None, camera=None):
    return H.doc(items(v), seq=seq, tick=tick, camera=camera)


# ---- camera presets -------------------------------------------------------------------------------------------------------------
PRESETS = ("overview", "chase", "target", "top")


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def camera_for(
    preset,
    turret,
    drones,
    muzzle_dir,
    target=None,
    prev=None,
    dt=1 / 30,
    t=0.0,
    tau=0.7,
    fovy=45.0,
):
    """The orbit camera (dict for `doc(camera=...)`) of a preset, smoothed towards its goal from `prev` (a dict or None: no smoothing).
    turret = world position of the turret; drones = positions of the drones still flying; muzzle_dir = unit vector; target = the
    selected track's estimated position or None."""
    pts = [turret] + [d for d in drones]
    cx = [sum(p[i] for p in pts) / len(pts) for i in range(3)]
    lo = [min(p[i] for p in pts) for i in range(3)]
    hi = [max(p[i] for p in pts) for i in range(3)]
    mid = [(lo[i] + hi[i]) / 2 for i in range(3)]
    rad = max(math.dist(mid, p) for p in pts) + 5.0
    fit = rad / math.tan(math.radians(fovy / 2)) * 0.95
    if preset == "chase":
        yaw0 = math.atan2(muzzle_dir[1], muzzle_dir[0])
        goal = dict(
            target=[
                turret[0] + math.cos(yaw0) * 12,
                turret[1] + math.sin(yaw0) * 12,
                turret[2] + 1.0,
            ],
            yaw=yaw0 + math.pi - 0.28,
            pitch=0.3,
            dist=26.0,
        )
    elif preset == "target" and target is not None:
        goal = dict(
            target=list(target),
            yaw=math.atan2(turret[1] - target[1], turret[0] - target[0]) + 0.35,
            pitch=0.22,
            dist=34.0,
        )
    elif preset == "top":
        goal = dict(target=mid, yaw=math.pi, pitch=1.45, dist=fit * 1.15)
    else:  # overview (also the fallback of "target" before there is one): behind and beside the turret, slowly drifting
        goal = dict(
            target=[mid[0], mid[1], mid[2] * 0.7],
            yaw=math.pi - 0.45 + 0.12 * math.sin(0.08 * t),
            pitch=0.24,
            dist=max(24.0, fit),
        )
    if prev is None:
        return goal
    k = 1 - math.exp(-dt / tau)
    return dict(
        target=[
            prev["target"][i] + (goal["target"][i] - prev["target"][i]) * k
            for i in range(3)
        ],
        yaw=prev["yaw"] + _wrap(goal["yaw"] - prev["yaw"]) * k,
        pitch=prev["pitch"] + (goal["pitch"] - prev["pitch"]) * k,
        dist=prev["dist"] + (goal["dist"] - prev["dist"]) * k,
    )


def read_hud(run_dir):
    """The run's hud.jsonl as a list of records (time ordered)."""
    return [
        json.loads(x)
        for x in (Path(run_dir) / "hud.jsonl").read_text().splitlines()
        if x.strip()
    ]
