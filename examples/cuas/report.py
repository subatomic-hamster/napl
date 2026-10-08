"""Post-run report for a counter-UAS run directory (pulled from hfbox): annotated camera frames, a top-down trajectory plot, a text log.

    python examples/cuas/report.py <run dir> <out dir>        # needs Pillow + matplotlib on the machine running it (the laptop, not hfbox)

Reads hud.jsonl, truth.csv, events.json, summary.json and frames/ of the run. Writes <out>/annotated/*.png (track boxes + IDs + HUD text on
the frames around every shot and every hit, plus every 60th frame), <out>/contact.png (a sheet of them), <out>/trajectories.png and
<out>/report.txt. Rendered media is never committed (it stays in remote-out/ or on the remote)."""

import csv
import json
import sys
from collections import Counter
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image, ImageDraw

COL = [
    "#ff4040",
    "#40ff40",
    "#4080ff",
    "#ffff40",
    "#ff40ff",
    "#40ffff",
    "#ff9030",
    "#b070ff",
    "#90ff90",
    "#ff90a0",
]


def color(i):
    return COL[sum(ord(c) for c in i) % len(COL)]


def annotate(run, rec, out):
    im = Image.open(run / rec["camera_frame"]).convert("RGB")
    d = ImageDraw.Draw(im)
    for t in rec["tracks"]:
        x0, y0, x1, y1 = t["bbox_px"]
        sel = t["id"] == rec["selected"]
        c = color(t["id"])
        pad = 6
        d.rectangle(
            [x0 - pad, y0 - pad, x1 + pad, y1 + pad], outline=c, width=3 if sel else 1
        )
        d.text(
            (x0 - pad, y0 - pad - 12),
            f"{t['id']} {t['range_m']:.0f}m{'' if t['confirmed'] else ' ?'}",
            fill=c,
        )
    w, h = im.size
    d.line([w / 2 - 8, h / 2, w / 2 + 8, h / 2], fill="white")
    d.line([w / 2, h / 2 - 8, w / 2, h / 2 + 8], fill="white")
    d.text((8, 6), f"t={rec['t']:.2f}s  stage={rec['stage']}  sel={rec['selected']}  rounds={rec['rounds']} hits={rec['hits']} destroyed={rec['destroyed']}"
           + ("  FIRE" if rec["fired"] else ""), fill="white")  # fmt: skip
    p = out / "annotated" / Path(rec["camera_frame"]).name
    im.save(p)
    return p


def main(run, out):
    run, out = Path(run), Path(out)
    (out / "annotated").mkdir(parents=True, exist_ok=True)
    hud = [json.loads(line) for line in open(run / "hud.jsonl")]
    ev = json.load(open(run / "events.json"))
    sm = json.load(open(run / "summary.json"))
    truth = list(csv.DictReader(open(run / "truth.csv")))
    names = sorted({k.rsplit("_", 1)[0] for k in truth[0] if k != "t"})
    P = {
        n: np.array(
            [
                [
                    float(r[f"{n}_x"]),
                    float(r[f"{n}_y"]),
                    float(r[f"{n}_z"]),
                    float(r[f"{n}_s"]),
                ]
                for r in truth
            ]
        )
        for n in names
    }
    # annotated frames: around shots / hits, plus every 60th
    times = [
        e["t"] for e in ev if e["type"] in ("fire", "hit") and e.get("kind") != "ground"
    ]
    pick = {
        r["frame"]
        for r in hud
        if r["frame"] % 60 == 0 or any(abs(r["t"] - t) < 0.07 for t in times)
    }
    shots = [
        annotate(run, r, out)
        for r in hud
        if r["frame"] in pick
        and r["camera_frame"]
        and (run / r["camera_frame"]).exists()
    ]
    sheet = [
        Image.open(p).resize((427, 240))
        for p in shots[:: max(1, len(shots) // 12)][:12]
    ]
    if sheet:
        cs = Image.new("RGB", (427 * 3, 240 * ((len(sheet) + 2) // 3)))
        for i, im in enumerate(sheet):
            cs.paste(im, ((i % 3) * 427, (i // 3) * 240))
        cs.save(out / "contact.png")
    # trajectories
    fig, ax = plt.subplots(
        1, 2, figsize=(15, 7), gridspec_kw=dict(width_ratios=[1.3, 1])
    )
    a = ax[0]
    a.plot(0, 0, "k^", ms=12, label="turret")
    for r in (15, 52):
        a.add_patch(plt.Circle((0, 0), r, fill=False, ls=":", color="gray"))
    for n in names:
        s = P[n]
        a.plot(s[:, 0], s[:, 1], "-", lw=1.2, label=n)
        a.plot(s[0, 0], s[0, 1], "o", color=a.lines[-1].get_color())
        bad = s[:, 3] > 0
        if bad.any():
            i = np.argmax(bad)
            a.plot(s[i, 0], s[i, 1], "x", color="red", ms=10, mew=2)
        if (s[:, 3] == 2).any():
            i = np.argmax(s[:, 3] == 2)
            a.plot(s[i, 0], s[i, 1], "s", color="black", ms=8)
    seen = set()
    for r in hud:
        for tr in r["tracers"]:
            k = (tr["round"], round(tr["t"], 3))
            if k not in seen and tr["p1"][2] > 0:
                seen.add(k)
                a.plot(
                    [tr["p0"][0], tr["p1"][0]],
                    [tr["p0"][1], tr["p1"][1]],
                    "-",
                    color="orange",
                    lw=0.7,
                    alpha=0.8,
                )
    for e in ev:
        if e["type"] == "hit" and e["kind"] == "drone":
            a.plot(e["pos"][0], e["pos"][1], "*", color="red", ms=14)
    a.plot([], [], "-", color="orange", label="pellet tracers")
    a.plot([], [], "x", color="red", label="first damage")
    a.plot([], [], "s", color="black", label="destroyed")
    a.plot([], [], "*", color="red", label="hit")
    a.set_xlabel("x (m)")
    a.set_ylabel("y (m)")
    a.set_aspect("equal")
    a.set_xlim(-5, 65)
    a.set_ylim(-60, 60)
    a.legend(loc="upper left", fontsize=8)
    a.set_title(
        f"top-down: rounds {sm['rounds']}, hits on drones {sm['hits_on_drones']}, destroyed {sm['drones_destroyed']}"
    )
    b = ax[1]
    for n in names:
        s = P[n]
        b.plot([float(r["t"]) for r in truth], s[:, 2], label=n)
    for e in ev:
        if e["type"] == "fire":
            b.axvline(e["t"], color="orange", lw=0.5, alpha=0.5)
        if e["type"] == "destroyed":
            b.axvline(e["t"], color="red", lw=1.5)
    b.set_xlabel("t (s)")
    b.set_ylabel("altitude z (m)")
    b.set_title("drone altitude; orange = fire, red = destroyed")
    b.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(out / "trajectories.png", dpi=110)
    # log
    stages = Counter(r["stage"] for r in hud)
    lines = [f"run {run.name}", f"sim {sm['sim_s']:.1f} s, wall {sm['wall_s']:.0f} s, frames {len(hud)}", f"stage frames: {dict(stages)}",
             f"rounds {sm['rounds']}  hits on drones {sm['hits_on_drones']}  hit rate {sm['hit_rate']}  ground impacts {sm['ground_impacts']}",
             f"drones damaged {sm['drones_damaged']}  destroyed {sm['drones_destroyed']}  engaged drones {sm['engaged_drones']}", ""]  # fmt: skip
    for n, d in sm["drones"].items():
        lines.append(
            f"{n}: {d['status']} hits {d['hits']} damage {[x['effect'] for x in d['damage']]} t_hit {d['t_hit']} t_destroyed {d['t_destroyed']}"
        )
    lines += ["", "closest approach of each round to a drone (m):"]
    lines += [
        f"  round {k}: {v['dist']:.2f} to {v['drone']} at t={v['t']:.2f}"
        for k, v in sm["closest_approach"].items()
    ]
    lines += ["", "events:"] + [
        json.dumps(
            {
                k: (round(v, 3) if isinstance(v, float) else v)
                for k, v in e.items()
                if k != "pos"
            }
        )
        for e in ev
    ]
    (out / "report.txt").write_text("\n".join(lines) + "\n")
    print("\n".join(lines[:12]))  # ttsr-ignore: py/no-print (CLI tool)


if __name__ == "__main__":
    main(*sys.argv[1:3])
