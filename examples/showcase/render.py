"""Render the N7 showcase from a `napl program` run dir: replay the bridge's replay log (every logged command is a SetAttr: the arm's
servo writes over the Feetech pty and PX4's ESC throttles) into a fresh Sim of the run's model, check the final state hash against the
run's, and write a short APNG of the hand-off (the arm placing the payload) and the take-off.

    python examples/showcase/render.py <run dir> <out.apng> [--width 480 --height 270 --frames 18]

Needs a Vulkan device (on hfbox: tools/remote.sh run --gpu ...). The frames are deterministic per GPU / driver only."""

import argparse
import csv
import json
import struct
import sys
from pathlib import Path

import numpy as np
from napl import Sim, _napl, showcase

SETATTR = 2  # napl.Command union value (schema/napl.fbs: Query = 1, SetAttr = 2)


def _flat(
    buf, tab, slot, fmt, size
):  # one field of a FlatBuffers table: the value, or None if absent
    vt = tab - struct.unpack_from("<i", buf, tab)[0]
    off = (
        struct.unpack_from("<H", buf, vt + 4 + 2 * slot)[0]
        if 4 + 2 * slot < struct.unpack_from("<H", buf, vt)[0]
        else 0
    )
    return struct.unpack_from(fmt, buf, tab + off)[0] if off else None, tab + off


def _ref(buf, pos):  # follow a u32 forward offset stored at pos
    return pos + struct.unpack_from("<I", buf, pos)[0]


def records(log: bytes):
    """Replay-log bytes -> [(applied_tick, source, path, attr, values)]; format: src/core/replay.h. Only SetAttr is decoded (the
    bridge logs nothing else); anything else raises."""
    assert log[:8] == b"NAPLLOG\0", "not a napl replay log"
    p = 8 + 4 + 8
    (n,) = struct.unpack_from("<I", log, p)
    p += 4 + n
    for _ in range(2):  # model bytes, initial state
        (n,) = struct.unpack_from("<Q", log, p)
        p += 8 + n
    end, out = len(log) - 24, []
    while p < end:
        tick, _err, n = struct.unpack_from("<QBI", log, p)
        p += 13
        env = bytes(log[p : p + n])
        p += n
        tab = _ref(env, 0)
        kind, _ = _flat(env, tab, 2, "<B", 1)
        assert kind == SETATTR, f"log record is not a SetAttr (command type {kind})"
        _, sp = _flat(env, tab, 1, "<I", 4)
        s = _ref(env, sp)
        source = env[s + 4 : s + 4 + struct.unpack_from("<I", env, s)[0]].decode()
        _, cp = _flat(env, tab, 3, "<I", 4)
        sa = _ref(env, cp)

        def field(slot, sa=sa, env=env):
            v, fp = _flat(env, sa, slot, "<I", 4)
            return None if v is None else _ref(env, fp)

        def text(o):
            return env[o + 4 : o + 4 + struct.unpack_from("<I", env, o)[0]].decode()

        vo = field(3)
        vals = (
            list(
                struct.unpack_from(
                    f"<{struct.unpack_from('<I', env, vo)[0]}d", env, vo + 4
                )
            )
            if vo
            else []
        )
        out.append((tick, source, text(field(0)), text(field(1)), vals))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run_dir")
    ap.add_argument("out")
    ap.add_argument("--png-dir", help="also write every frame as a PNG here")
    ap.add_argument("--width", type=int, default=480)
    ap.add_argument("--height", type=int, default=270)
    ap.add_argument(
        "--frames",
        type=int,
        default=18,
        help="half for the hand-off, half for the take-off",
    )
    a = ap.parse_args()
    run = Path(a.run_dir)
    res = json.loads((run / "result.json").read_text())
    model = next((run / "model").glob("showcase.xml"))
    recs = records((run / "replay.naplog").read_bytes())
    dt = 0.001
    stages = {s["name"]: s for s in res["programs"]}
    with open(run / "trace.csv") as f:
        tr = list(csv.DictReader(f))
    t = np.array([float(r["t"]) for r in tr])
    z = np.array([float(r["z"]) for r in tr])
    lift = t[np.argmax(z - z[0] > 0.02)]  # first liftoff
    top = t[np.argmax(z - z[0] > 3.0)]  # 3 m reached
    arm_end, k = stages["arm"]["end_t"], a.frames // 2
    shots = [
        (tt, "handoff") for tt in np.linspace(arm_end - 12.0, arm_end - 2.0, k)
    ] + [(tt, "takeoff") for tt in np.linspace(lift - 1.0, top, a.frames - k)]
    sim, tick, i, imgs = Sim.load(model), 0, 0, []
    payload = lambda: sim.query("/world/payload", "pos")[0]
    adv = lambda tick_: (
        sim.step(tick_ - sim.tick) if tick_ > sim.tick else None
    )  # step the Sim up to a tick
    for tt, phase in shots:
        target = round(tt / dt)
        while (
            i < len(recs) and recs[i][0] <= target
        ):  # commands applied at ticks <= target, in order
            adv(recs[i][0])
            sim.set(recs[i][2], recs[i][3], recs[i][4])
            i += 1
        adv(target)
        base = sim.query("/world/bottom", "pos")[0]
        if phase == "handoff":  # close on the plate: from +x, a little above
            cam = dict(
                camera="orbit",
                az=20.0,
                el=22.0,
                dist=0.8,
                target=[0.0, -0.13, base[2] + 0.04],
            )
        else:  # follow the climb
            h = base[2] - 0.217
            cam = dict(
                camera="orbit",
                az=30.0,
                el=18.0,
                dist=0.8 * (h + 1.3),
                target=[0.0, -0.1, base[2] - 0.1],
            )
        imgs.append(showcase.shot(sim, a.width, a.height, 1, **cam))
        print(
            f"frame {len(imgs):2d} {phase:8s} t={tt:6.1f}s tick={sim.tick} drone z={base[2]:.3f} payload={np.round(payload(), 3)}",
            file=sys.stderr,
        )
    # the whole log must reproduce the run's final state hash (the replay is faithful, so the frames are the run's)
    for tick_, src, path, attr, vals in recs[i:]:
        adv(tick_)
        sim.set(path, attr, vals)
    adv(res["sim"]["tick"])
    h = f"{sim.state_hash():016x}"
    print(
        f"replay state hash {h} vs run {res['sim']['state_hash']}: {'MATCH' if h == res['sim']['state_hash'] else 'DIFFERENT'}",
        file=sys.stderr,
    )
    Path(a.out).write_bytes(showcase.apng(imgs, fps=4))
    if a.png_dir:  # stills of every frame
        Path(a.png_dir).mkdir(parents=True, exist_ok=True)
        for n, im in enumerate(imgs):
            (Path(a.png_dir) / f"frame{n:02d}.png").write_bytes(_napl.encode_png(im))
    print(a.out)
    return 0 if h == res["sim"]["state_hash"] else 1


if __name__ == "__main__":
    sys.exit(main())
