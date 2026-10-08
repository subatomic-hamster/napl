"""Follow-run debugging: python timeline.py <run dir> [step_s] -> one line per step_s of telemetry.jsonl (true distance / bearing of the subject in the
camera frame, seen flag, planner mode, speed, height) and the unseen stretches. Evaluation only (reads the subject's true position)."""

import json
import math
import sys
from pathlib import Path

run = Path(sys.argv[1])
recs = [json.loads(line) for line in open(run / "telemetry.jsonl")]
if (
    sys.argv[2:3] == ["depth"]
):  # timeline.py <run> depth <t>: the depth frame of that step as a digit map (metres, . = no return), subject marked S
    import numpy as np

    r = min(
        (x for x in recs if x["phase"] == "fly"),
        key=lambda x: abs(x["t"] - float(sys.argv[3])),
    )
    img = np.load(run / "depth.npz")["depth"][r["depth_index"]].astype(float)
    s = r["subject"]
    d = np.array(s["truth"][:2]) - np.array(r["pos"][:2])
    col = (
        int(
            (
                math.tan(math.atan2(d[1], d[0]) - r["yaw"])
                / math.tan(math.radians(43.5))
                * -1
                + 1
            )
            / 2
            * 160
        )
        if abs(math.atan2(d[1], d[0]) - r["yaw"]) < 1.5
        else -1
    )
    print(
        f"t={r['t']} yaw={r['yaw']:.2f} subject range {math.hypot(*d):.1f} m, column {col} (left of the image = left of the drone), seen={s['seen']}"
    )
    for i in range(0, 90, 3):
        print(
            "".join(
                "." if not np.isfinite(img[i, j]) else str(min(int(img[i, j]), 9))
                for j in range(0, 160, 2)
            )
        )
    print(" " * (col // 2) + "^ subject")
    sys.exit()
step = float(sys.argv[2]) if len(sys.argv) > 2 else 0.5
if "subject" not in recs[0]:  # goal run: the 3 s around the worst clearance
    tm, last = min(recs, key=lambda x: x["min_clearance"])["t"], -9.0
    print("   t   speed  vdes_x vdes_y  vdes_z  clear  est_clr tilt mode  safe   cost")
    for r in recs:
        if abs(r["t"] - tm) <= 1.6 and r["t"] - last >= step:
            last = r["t"]
            vd = r["setpoint"]["vdes"]
            print(
                f"{r['t']:6.2f} {math.hypot(*r['vel'][:2]):5.2f} {vd[0]:6.2f} {vd[1]:6.2f} {vd[2]:6.2f} {r['min_clearance']:6.2f} {r['map']['clearance_est']:6.2f} {r['tilt_deg']:5.1f} {r['mode']:6s} {r['n_safe']:4d} {r['chosen_cost']} open {r['map'].get('openness')}"
            )
    op = [
        r["map"]["openness"]
        for r in recs
        if r["phase"] == "fly" and "openness" in r["map"]
    ]
    if op:
        print(
            "openness over the flight: p5/p25/p50/p75",
            [round(sorted(op)[int(q * len(op))], 2) for q in (0.05, 0.25, 0.5, 0.75)],
        )
    sys.exit()
last, lost0 = -9.0, None
print("   t  dist bear_deg seen mode    speed   z   tilt  est_sig  stuck")
for r in recs:
    s = r.get("subject")
    if not s or r["phase"] != "fly":
        continue
    d = [s["truth"][0] - r["pos"][0], s["truth"][1] - r["pos"][1]]
    rng, b = (
        math.hypot(*d),
        math.degrees(
            (math.atan2(d[1], d[0]) - r["yaw"] + math.pi) % (2 * math.pi) - math.pi
        ),
    )
    if not s["seen"] and lost0 is None:
        lost0 = r["t"]
    if s["seen"] and lost0 is not None:
        print(f"     -- lost {lost0:.1f}..{r['t']:.1f} s ({r['t'] - lost0:.1f} s)")
        lost0 = None
    if r["t"] - last >= step:
        last = r["t"]
        sp = math.hypot(*r["vel"][:2])
        e = s["est"] or s["truth"][:2]
        ee = math.hypot(
            e[0] - s["truth"][0], e[1] - s["truth"][1]
        )  # belief / track error (m)
        vd = r["setpoint"]["vdes"]
        print(
            f"{r['t']:6.1f} {rng:5.2f} {b:7.1f}  {'Y' if s['seen'] else '.'}{'v' if s.get('visible') else ' '}  {r['mode']:7s} {sp:5.2f} {r['pos'][2]:5.2f} {r['tilt_deg']:5.1f} {s['sigma'] or 0:6.2f}"
            f"   est_err {ee:5.2f} vdes {vd[0]:5.2f} {vd[1]:5.2f} {vd[2]:5.2f} safe {r['n_safe']}/{r['n_candidates']}"
        )
