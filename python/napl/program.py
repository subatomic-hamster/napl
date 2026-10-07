"""`napl program <scenario.toml>`: run an ordinary program against the simulated robot over the real protocol, with evidence.

USER CODE: the program (and a scenario's `model.script`) is user-authored code run on the user's machine. This is a CLI feature
only; there is deliberately no MCP tool for it (v1 rule: agents author declarative data - designs, TestSpecs - never code).
Limits are resource limits (setrlimit) and a process-group kill, NOT a security sandbox: a program that calls setsid() leaves
the group and is not killed, and it runs with the invoking user's permissions.

Scenario (TOML; relative paths are relative to the scenario file):

    name = "so101-pick"
    [model]                      # exactly one of (optional for px4: the bridge config names the model)
    assembly = "../../assemblies/so101.toml"   # + optional parts = "dir": built with napl.design.build
    mjcf = "robot.xml"                         # MJCF / URDF as is
    script = "scene.py"          # python file defining build(outdir: Path) -> Path of the MJCF (for composed scenes)
    [interfaces.feetech]         # a Feetech STS pty on a Host sim running at `limits.rate`; env NAPL_SERIAL=<pty path>
    servos = { 1 = "/devices/servo1", 2 = "/devices/servo2" }
    baud = 1000000
    [interfaces.dynamixel]       # instead of feetech: a Dynamixel Protocol 2.0 pty, same Host sim / env; models = { id = "xl430-w250-t" |
    servos = { 1 = "/devices/servo1" }         #   "xl330-m288-t" | "xl330-m077-t" } per bus id (default xl430-w250-t), baud as above
    [interfaces.px4]             # PX4 SITL + napl-px4-bridge (own World); env NAPL_MAVLINK=udp://:14540. Exactly one interface per scenario.
    bridge = "../../px4/bridge_x500.toml"
    # px4 + feetech TOGETHER (the one allowed combination): one World, the bridge's, stepped in lockstep with PX4; the Feetech pty is
    # served from that World over the bridge's --ctl socket (napl.ctl). The programs run in order, each with NAPL_SERIAL and
    # NAPL_MAVLINK set: [[programs]] name = "arm"  cmd = [...]   then   name = "fly"  cmd = [...]   (optional wall_s / cpu_s each).
    # Monitors there: `column` (trace) or path + attr [+ frame = a body prim: the value in that body's frame] sampled at 10 Hz;
    # `phase = "<program name>"` makes from / to relative to that program's start (to defaults to its end), sim seconds.
    [[setup]]                    # optional SetAttr commands before the run (logged, replayed)
    path = "/devices/servo1"; attr = "torque_enable"; values = [0]
    [program]
    cmd = ["python", "{dir}/pick.py"]          # "python" = this interpreter, {dir} = scenario dir; cwd = a temp run dir
    [limits]                     # all optional
    wall_s = 120  sim_s = 60  rate = 1.0  cpu_s = 300  address_space_mb = 4096  open_files = 256
    [[monitors]]                 # TestSpec-style: op "always" (every sample in [lo, hi]) | "eventually" | "final" (last sample)
    name = "cube_at_target"      # sim kinds: signal = path/paths + attr + (index | indices [+ ref] -> norm of the difference)
    path = "/world/cube"  attr = "pos"  indices = [0, 1]  ref = [0.2, 0.04]  op = "final"  hi = 0.01
    # px4: column = a trace.csv column or the derived `alt` (z - z at start) / `dist_home` (horizontal m from the start)
    # optional from / to = seconds of sim time since the start (sim kinds: ticks x `dt`, default 0.001).

Env of the program: NAPL_SERIAL / NAPL_MAVLINK, NAPL_SCENARIO_DIR, plus the caller's environment. Run dir `<runs-dir>/<name>-<time>/`:
result.json, program.log (stdout+stderr), replay.naplog (replay log), trace.csv (sampled monitor signals / bridge trace), and for
px4 bridge.log + px4.log. result.json["pass"] = program exit 0 AND no timeout AND every monitor passed. The final state hash of
the run is replayed with napl-replay when that exe exists (result.json["replay"]).
Exit code (see cli): 0 pass, 1 fail, 2 error.
"""

import csv
import importlib.util
import json
import math
import os
import re
import signal
import subprocess
import sys
import threading
import time
import tomllib
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
REPLAY = Path(os.environ.get("NAPL_REPLAY_EXE", REPO / "build" / "napl-replay"))
LIMITS = dict(wall_s=120.0, cpu_s=300, address_space_mb=4096, open_files=256, rate=1.0)
# Runs in the child before exec: apply the rlimits, then become the program (no preexec_fn: this process has threads).
LAUNCH = (
    "import os,sys,json,resource;"
    "[resource.setrlimit(getattr(resource,k),tuple(v)) for k,v in json.loads(sys.argv[1]).items()];"
    "os.execvp(sys.argv[2],sys.argv[2:])"
)


class ScenarioError(Exception):
    pass


def load_scenario(path) -> dict:
    path = Path(path).resolve()
    try:
        sc = tomllib.loads(path.read_text())
    except (OSError, ValueError) as e:
        raise ScenarioError(f"cannot read scenario '{path}': {e}") from None
    sc["dir"] = path.parent
    sc.setdefault("name", path.stem.removesuffix(".scenario"))
    sc["limits"] = {**LIMITS, **sc.get("limits", {})}
    ifs = sc.get("interfaces", {})
    if not (set(ifs) <= {"feetech", "dynamixel", "px4"} and ifs):
        raise ScenarioError("[interfaces] needs one of feetech / dynamixel / px4")
    if set(ifs) == {
        "feetech",
        "px4",
    }:  # one World in the bridge, the pty served over its ctl socket (_combo)
        if not sc.get("programs") or any(
            not p.get("cmd") or not p.get("name") for p in sc["programs"]
        ):
            raise ScenarioError(
                "px4 + feetech needs [[programs]], each with name and cmd"
            )
        return sc
    if (
        len(ifs) > 1
    ):  # px4 has its own World in the bridge; the two serial buses would share one NAPL_SERIAL
        raise ScenarioError(
            f"[interfaces] takes exactly one interface, got {sorted(ifs)}"
        )
    if not sc.get("program", {}).get("cmd"):
        raise ScenarioError("[program] cmd is required")
    return sc


def _val(m, v):
    if "index" in m:
        return float(v[m["index"]])
    x = v if "indices" not in m else v[m["indices"]]
    return (
        float(np.linalg.norm(x - np.asarray(m["ref"], float)))
        if "ref" in m
        else float(x[0] if len(x) == 1 else np.linalg.norm(x))
    )


def judge(m, series) -> dict:
    """series [(t, value)] -> result dict. Same semantics as napl.TestSpec monitors, plus op "final"."""
    lo, hi = float(m.get("lo", -math.inf)), float(m.get("hi", math.inf))
    win = [(t, v) for t, v in series if m.get("from", 0) <= t <= m.get("to", math.inf)]
    r = dict(
        name=m["name"],
        op=m.get("op", "always"),
        lo=lo,
        hi=hi,
        samples=len(win),
    )
    if not win:
        return {**r, "pass": False, "error": "no samples in the window"}
    win = [(float(t), float(v)) for t, v in win]
    vals = [v for _, v in win]
    ok = [lo <= v <= hi for v in vals]
    hit = {"always": lambda: next((i for i, k in enumerate(ok) if not k), None),
           "eventually": lambda: next((i for i, k in enumerate(ok) if k), None),
           "final": lambda: len(ok) - 1}[r["op"]]()  # fmt: skip
    p = {"always": hit is None, "eventually": hit is not None, "final": ok[-1]}[r["op"]]
    r.update(min=min(vals), max=max(vals), final=vals[-1], **{"pass": p})
    if hit is not None:
        r.update(hit_t=win[hit][0], hit_value=win[hit][1])
    return r


def _expand(monitors, kind):
    out = []
    for m in monitors:
        if "name" not in m:
            raise ScenarioError(f"monitor without a name: {m}")
        if kind == "px4" or (kind == "combo" and "column" in m):
            if "column" not in m:
                raise ScenarioError(f"px4 monitor '{m['name']}' needs `column`")
            out.append(m)
            continue
        paths = m.get("paths") or [m.get("path")]
        if None in paths or "attr" not in m:
            raise ScenarioError(f"monitor '{m['name']}' needs path|paths and attr")
        out += [
            {
                **m,
                "path": p,
                "name": m["name"] if len(paths) == 1 else f"{m['name']}:{p}",
            }
            for p in paths
        ]
    return out


def replay_hash(log, root=None):
    """napl-replay on a replay log -> (tick, hash hex) or None if the exe is missing."""
    if not REPLAY.exists():
        return None
    argv = [str(REPLAY), str(log)] + (["--root", str(root)] if root else [])
    r = subprocess.run(argv, capture_output=True, text=True, timeout=600)
    m = re.search(r"tick=(\d+) state_hash=([0-9a-f]+)", r.stdout)
    if r.returncode or not m:
        raise RuntimeError(f"napl-replay failed: {r.stderr or r.stdout}")
    return int(m[1]), m[2]


def _group_alive(pgid):
    try:
        os.killpg(pgid, 0)
        return True
    except ProcessLookupError:
        return False


def kill_group(proc, grace=2.0):
    """SIGTERM then SIGKILL proc's whole process group; returns once no member is left (or after a few seconds)."""
    pgid = proc.pid
    for sig, wait in ((signal.SIGTERM, grace), (signal.SIGKILL, 5.0)):
        try:
            os.killpg(pgid, sig)
        except ProcessLookupError:
            return
        end = time.monotonic() + wait
        while time.monotonic() < end:
            proc.poll()  # reap the leader: a zombie keeps its group "alive"
            if not _group_alive(pgid):
                return
            time.sleep(0.02)


def run_child(argv, env, cwd, log, limits, stop_if=None):
    """Start the program in its own session / process group with rlimits; wait up to wall_s (or until stop_if() is a reason
    string). -> dict(returncode, signal, wall_s, timeout: None|"wall"|<stop_if reason>). The group is always killed at the end."""
    lim = {"RLIMIT_CPU": [int(limits["cpu_s"]), int(limits["cpu_s"]) + 2],
           "RLIMIT_AS": [int(limits["address_space_mb"]) << 20] * 2,
           "RLIMIT_NOFILE": [int(limits["open_files"])] * 2}  # fmt: skip
    t0 = time.monotonic()
    with open(log, "wb") as lf:
        p = subprocess.Popen([sys.executable, "-c", LAUNCH, json.dumps(lim), *argv], cwd=cwd, env=env,
                             stdin=subprocess.DEVNULL, stdout=lf, stderr=subprocess.STDOUT, start_new_session=True)  # fmt: skip
        timeout = None
        while p.poll() is None:
            if time.monotonic() - t0 > limits["wall_s"]:
                timeout = "wall"
            elif stop_if and (timeout := stop_if()):
                pass
            if timeout:
                break
            time.sleep(0.05)
        kill_group(p)  # also reaps background children a finished program left behind
        rc = p.wait()
    return dict(returncode=rc, signal=signal.Signals(-rc).name if rc < 0 else None,
                wall_s=time.monotonic() - t0, timeout=timeout, pgid=p.pid)  # fmt: skip


def _build_model(sc, out: Path):
    m = sc.get("model", {})
    d = sc["dir"]
    n = sum(k in m for k in ("assembly", "mjcf", "script"))
    if n != 1:
        raise ScenarioError("[model] needs exactly one of assembly / mjcf / script")
    out.mkdir(parents=True, exist_ok=True)
    if "mjcf" in m:
        return d / m["mjcf"]
    if "assembly" in m:
        from .design import DesignError, build

        try:
            return Path(
                build(d / m["assembly"], d / m.get("parts", REPO / "parts"), out)["xml"]
            )
        except DesignError as e:
            raise ScenarioError(str(e)) from None
    spec = importlib.util.spec_from_file_location(
        "napl_scenario_script", d / m["script"]
    )
    mod = importlib.util.module_from_spec(spec)
    sys.path.insert(0, str((d / m["script"]).parent))  # the script's own siblings
    spec.loader.exec_module(mod)
    return Path(mod.build(out))


def run(scenario, runs_dir="runs") -> tuple:
    """-> (result dict, run dir). Raises ScenarioError for a bad scenario / missing prerequisites."""
    sc = load_scenario(scenario)
    ifs = sc["interfaces"]
    lim, kind = sc["limits"], ("combo" if len(ifs) > 1 else next(iter(ifs)))
    run_dir = Path(runs_dir) / f"{sc['name']}-{time.strftime('%Y%m%d-%H%M%S')}"
    run_dir.mkdir(parents=True)
    cwd = run_dir / "cwd"
    cwd.mkdir()
    sub = lambda s: s.replace("{dir}", str(sc["dir"]))
    argvs = lambda cmd: [
        sys.executable if a == "python" and i == 0 else sub(a)
        for i, a in enumerate(cmd)
    ]
    argv = argvs(sc["program"]["cmd"]) if kind != "combo" else None
    env = {**os.environ, "NAPL_SCENARIO_DIR": str(sc["dir"])}
    mons = _expand(sc.get("monitors", []), kind)
    res = dict(
        name=sc["name"],
        kind=kind,
        scenario=str(Path(scenario).resolve()),
        argv=argv,
        limits=lim,
    )
    (run_dir / "program.log").touch()
    log = run_dir / "program.log"
    if kind == "combo":
        res["argv"] = argv = [argvs(p["cmd"]) for p in sc["programs"]]
    {"px4": _px4, "combo": _combo}.get(kind, _sim)(
        sc, mons, run_dir, cwd, argv, env, log, res
    )
    p = res["program"]
    why = []
    if p["returncode"]:
        why.append(
            f"program exit {p['returncode']}"
            + (f" ({p['signal']})" if p["signal"] else "")
        )
    if p["timeout"]:
        why.append(f"timeout: {p['timeout']}")
    why += [f"monitor {m['name']}" for m in res["monitors"] if not m["pass"]]
    if not res["replay"].get("match", True):
        why.append("replay hash mismatch")
    res.update({"pass": not why, "fail_reasons": why})
    (run_dir / "result.json").write_text(json.dumps(res, indent=2))
    return res, run_dir


def _sim(sc, mons, run_dir, cwd, argv, env, log, res):
    from . import Sim
    from .dynamixel import DynamixelPort
    from .feetech import FeetechPort

    fc, lim = sc["interfaces"][res["kind"]], sc["limits"]
    dxl = res["kind"] == "dynamixel"
    Port = DynamixelPort if dxl else FeetechPort
    extra = (
        dict(models={int(i): m for i, m in fc.get("models", {}).items()}) if dxl else {}
    )
    model = _build_model(sc, run_dir / "model")
    sim = Sim.load(model, host=True)
    try:
        for s in sc.get("setup", []):
            for p in s.get("paths") or [s["path"]]:
                sim.set(p, s["attr"], s["values"])
        port = Port(
            sim,
            {int(i): p for i, p in fc["servos"].items()},
            baud=fc.get("baud", 1_000_000),
            realtime=True,
            **extra,
        )
    except (ValueError, KeyError) as e:
        raise ScenarioError(str(e)) from None
    dt, series, rows, stop = (
        sc.get("dt", 0.001),
        {m["name"]: [] for m in mons},
        [],
        threading.Event(),
    )
    tick0 = sim.tick
    wall0 = time.monotonic()

    def sample():
        t, row = (
            (sim.tick - tick0) * dt,
            [time.monotonic() - wall0, sim.tick - tick0, (sim.tick - tick0) * dt],
        )
        for m in mons:
            v = _val(m, sim.query(m["path"], m["attr"], m.get("frame", ""))[0])
            series[m["name"]].append((t, v))
            row.append(v)
        rows.append(row)

    def sampler():
        while not stop.wait(0.02):
            sample()

    env["NAPL_SERIAL"] = port.device_path
    th = threading.Thread(target=sampler, daemon=True)
    try:
        r = sim.call({"cmd_type": "Run", "cmd": {"rate": float(lim["rate"])}})
        if not r["ok"]:
            raise ScenarioError(r["message"])
        th.start()
        sim_s = lim.get("sim_s")
        res["program"] = run_child(argv, env, cwd, log, lim,
                                   (lambda: "sim_time" if (sim.tick - tick0) * dt > sim_s else None) if sim_s else None)  # fmt: skip
    finally:
        stop.set()
        th.join()
        port.close()
        sim.call({"cmd_type": "Pause", "cmd": {}})
    sample()
    h = f"{sim.state_hash():016x}"
    (run_dir / "replay.naplog").write_bytes(sim.log())
    res["sim"] = dict(tick=sim.tick - tick0, sim_s=(sim.tick - tick0) * dt, state_hash=h, diverged=sim.diverged,
                      port_errors=list(port.errors))  # fmt: skip
    res["monitors"] = [judge(m, series[m["name"]]) for m in mons]
    with open(run_dir / "trace.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["wall_s", "tick", "t", *[m["name"] for m in mons]])
        w.writerows(rows)
    rp = replay_hash(run_dir / "replay.naplog", Path(model).parent)
    res["replay"] = (
        {"available": False}
        if rp is None
        else dict(
            available=True, tick=rp[0], state_hash=rp[1], match=rp == (sim.tick, h)
        )
    )
    if res["sim"]["diverged"]:
        res["monitors"].append({"name": "solver_diverged", "pass": False})


def _px4(sc, mons, run_dir, cwd, argv, env, log, res):
    from .px4 import Stack

    cfg = sc["dir"] / sc["interfaces"]["px4"]["bridge"]
    model = _build_model(sc, run_dir / "model") if sc.get("model") else None
    st = Stack(cfg, run_dir, model=model)
    env["NAPL_MAVLINK"] = st.mavlink
    try:
        st.start()
    except RuntimeError as e:
        raise ScenarioError(f"PX4 stack: {e}") from None
    try:
        res["program"] = run_child(argv, env, cwd, log, sc["limits"])
    finally:
        final = st.stop()
    res["sim"] = dict(final or {}, bridge_log="bridge.log", px4_log="px4.log")
    with open(st.trace) as f:
        rows = list(csv.DictReader(f))
    tr = {k: np.array([float(r[k]) for r in rows]) for k in rows[0]}
    tr["alt"] = tr["z"] - tr["z"][0]
    tr["dist_home"] = np.hypot(tr["x"] - tr["x"][0], tr["y"] - tr["y"][0])
    res["monitors"] = [
        judge(m, list(zip(tr["t"] - tr["t"][0], tr[m["column"]])))
        for m in mons
        if m["column"] in tr
    ]
    bad = [m["name"] for m in mons if m["column"] not in tr]
    if bad:
        raise ScenarioError(f"monitors {bad}: no such trace column (have {sorted(tr)})")
    rp = replay_hash(st.log)
    res["replay"] = {"available": False} if rp is None else dict(
        available=True, tick=rp[0], state_hash=rp[1], match=bool(final) and rp == (final["tick"], final["state_hash"]))  # fmt: skip


def _combo(sc, mons, run_dir, cwd, argvs, env, log, res):
    """px4 + feetech. ONE World: the bridge's, stepped in lockstep with PX4 (wall = sim time). The arm's Feetech pty is served by
    FeetechPort in this process over the bridge's --ctl socket (RemoteSim), so its SetAttr writes run in that same World and land in
    the bridge's replay log next to the ESC throttles. The programs run in order (the arm one first, PX4 idling disarmed in the same
    lockstep, then the flight), each with NAPL_SERIAL + NAPL_MAVLINK; a failing program stops the rest."""
    from .ctl import RemoteSim
    from .feetech import FeetechPort
    from .px4 import Stack

    lim, dt = sc["limits"], sc.get("dt", 0.001)
    model = _build_model(sc, run_dir / "model")
    st = Stack(
        sc["dir"] / sc["interfaces"]["px4"]["bridge"], run_dir, model=model, ctl=True
    )
    env["NAPL_MAVLINK"] = st.mavlink
    try:
        st.start()
    except RuntimeError as e:
        raise ScenarioError(f"PX4 stack: {e}") from None
    fc = sc["interfaces"]["feetech"]
    sig = [
        m for m in mons if "column" not in m
    ]  # world signals, sampled over ctl at 10 Hz
    series, rows, stop, stages = {m["name"]: [] for m in sig}, [], threading.Event(), []
    rs = port = th = None

    def sample():
        row = []
        for m in sig:
            tick, v = rs.get(m["path"], m["attr"], m.get("frame", ""))
            series[m["name"]].append((tick * dt, _val(m, v)))
            row.append(series[m["name"]][-1][1])
        rows.append([tick * dt, *row] if sig else [])

    def sampler():
        while not stop.wait(0.1):
            try:
                sample()
            except (OSError, ValueError):  # the bridge went away
                return

    try:
        rs = RemoteSim(st.ctl)
        for s in sc.get(
            "setup", []
        ):  # SetAttr before the programs (logged, replayed): e.g. torque on, the arm holds its pose
            for p in s.get("paths") or [s["path"]]:
                rs.set(p, s["attr"], s["values"])
        port = FeetechPort(
            rs,
            {int(i): p for i, p in fc["servos"].items()},
            baud=fc.get("baud", 1_000_000),
            realtime=True,
        )
        env["NAPL_SERIAL"] = port.device_path
        if sig:
            th = threading.Thread(target=sampler, daemon=True)
            th.start()
        for p, a in zip(sc["programs"], argvs):
            t0 = rs.tick * dt
            r = run_child(
                a,
                env,
                cwd,
                run_dir / f"program.{p['name']}.log",
                {**lim, **{k: p[k] for k in ("wall_s", "cpu_s") if k in p}},
            )
            stages.append(dict(name=p["name"], start_t=t0, end_t=rs.tick * dt, **r))
            if r["returncode"] or r["timeout"]:
                break
        if sig:
            sample()
    except (ValueError, KeyError) as e:
        raise ScenarioError(str(e)) from None
    finally:
        stop.set()
        if th:
            th.join()
        if port:
            port.close()
        if rs:
            rs.close()
        final = st.stop()
    log.write_text(
        "".join(
            f"== {s['name']} ==\n" + (run_dir / f"program.{s['name']}.log").read_text()
            for s in stages
        )
    )
    res["programs"] = stages
    res["program"] = next(
        (s for s in stages if s["returncode"] or s["timeout"]), stages[-1]
    )
    res["sim"] = dict(
        final or {},
        bridge_log="bridge.log",
        px4_log="px4.log",
        port_errors=list(port.errors),
    )
    if sig:
        with open(run_dir / "signals.csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t", *[m["name"] for m in sig]])
            w.writerows(rows)
    with open(st.trace) as f:
        trows = list(csv.DictReader(f))
    tr = {k: np.array([float(r[k]) for r in trows]) for k in trows[0]}
    tr["alt"] = tr["z"] - tr["z"][0]
    tr["dist_home"] = np.hypot(tr["x"] - tr["x"][0], tr["y"] - tr["y"][0])
    bad = [m["name"] for m in mons if "column" in m and m["column"] not in tr]
    if bad:
        raise ScenarioError(f"monitors {bad}: no such trace column (have {sorted(tr)})")
    out = []
    for m in mons:
        if (
            "phase" in m
        ):  # from / to relative to that program's start, `to` defaulting to its end
            ph = next((s for s in stages if s["name"] == m["phase"]), None)
            if ph is None:
                out.append(
                    {
                        "name": m["name"],
                        "pass": False,
                        "error": f"program '{m['phase']}' did not run",
                    }
                )
                continue
            m = {
                **m,
                "from": ph["start_t"] + m.get("from", 0),
                "to": ph["start_t"] + m["to"] if "to" in m else ph["end_t"],
            }
        out.append(
            judge(
                m,
                list(zip(tr["t"], tr[m["column"]]))
                if "column" in m
                else series[m["name"]],
            )
        )
    res["monitors"] = out
    rp = replay_hash(st.log, Path(model).parent)
    res["replay"] = {"available": False} if rp is None else dict(
        available=True, tick=rp[0], state_hash=rp[1], match=bool(final) and rp == (final["tick"], final["state_hash"]))  # fmt: skip
