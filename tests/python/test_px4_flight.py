"""Real PX4 v1.17.0 SITL flies the simulated X500 V2 through napl-px4-bridge (N5), driven by MAVSDK (BSD-3).

Two models, same flight: "asm" = the parts-built design (assemblies/x500v2.toml, built by this test with `napl.design.build` into
build/assemblies, flown through px4/bridge_x500v2_asm.toml) and "fixture" = the hand-written tests/data/px4/x500_fixture.xml
(px4/bridge_x500.toml), kept as the independent reference model (cheap: ~70 s). The asm run writes build/px4/x500v2_flight.json,
the input of docs/validation/x500-hover.md.

Skipped unless the PX4 build, the bridge exe and mavsdk are present. Everything binds 127.0.0.1.
Env: NAPL_PX4_DIR (PX4 checkout, default hfbox), NAPL_PX4_INSTANCE (default 0), NAPL_BRIDGE_EXE, NAPL_REPLAY_EXE.
Run: .venv/bin/python -m pytest tests/python/test_px4_flight.py -m px4 -v -s
Note: mavsdk>=4 is an in-process binding (libcmavsdk); there is no mavsdk_server / gRPC port to allocate.
"""

import asyncio
import json
import os
import queue
import re
import shutil
import signal
import subprocess
import tempfile
import threading
import time
import tomllib
from pathlib import Path

import numpy as np
import pytest
from test_px4_fixture import G, fixture_mass

pytestmark = pytest.mark.px4

REPO = Path(__file__).resolve().parents[2]
PX4_DIR = Path(os.environ.get("NAPL_PX4_DIR", "/workspace/wt/napl/px4/src"))
B = PX4_DIR / "build" / "px4_sitl_default"
PX4_BIN = B / "bin" / "px4"
BRIDGE = Path(os.environ.get("NAPL_BRIDGE_EXE", REPO / "build" / "napl-px4-bridge"))
REPLAY = Path(os.environ.get("NAPL_REPLAY_EXE", REPO / "build" / "napl-replay"))
N = int(os.environ.get("NAPL_PX4_INSTANCE", "0"))
CFGS = {
    "asm": REPO / "px4" / "bridge_x500v2_asm.toml",
    "fixture": REPO / "px4" / "bridge_x500.toml",
}
ASM = REPO / "assemblies" / "x500v2.toml"
ASM_OUT = REPO / "build" / "assemblies"
FLIGHT_JSON = REPO / "build" / "px4" / "x500v2_flight.json"
PARAMS = REPO / "px4" / "params" / "napl_sitl.params"
MOTOR_TOML = REPO / "parts" / "drone" / "holybro-2216-kv920.toml"
PORTS_TCP = [4560 + N]
PORTS_UDP = [14540 + N, 14550 + N, 14580 + N, 18570 + N]
FORBIDDEN = set(range(5177, 5195)) | {8000, 8010, 8093}

try:
    import mavsdk
except ImportError:
    mavsdk = None

SKIP = (
    "mavsdk not installed (pip install -e '.[px4]')"
    if mavsdk is None
    else f"PX4 binary missing: {PX4_BIN}"
    if not PX4_BIN.exists()
    else f"bridge exe missing: {BRIDGE}"
    if not BRIDGE.exists()
    else f"napl-replay missing: {REPLAY}"
    if not REPLAY.exists()
    else None
)


def busy_ports(proto, ports):
    """Local ports in use, from /proc/net (no `ss` on the host). TCP TIME_WAIT is not a conflict (SO_REUSEADDR)."""
    busy = set()
    for name in (proto, proto + "6"):
        for line in Path(f"/proc/net/{name}").read_text().splitlines()[1:]:
            f = line.split()
            port = int(f[1].rsplit(":", 1)[1], 16)
            if port in ports and not (proto == "tcp" and f[3] == "06"):
                busy.add(port)
    return busy


class Proc:
    """Subprocess in its own process group, stdout lines pumped into a queue (and a log file)."""

    def __init__(self, argv, log, cwd=None, env=None, stdin=subprocess.DEVNULL):
        self.q, self.lines = queue.Queue(), []
        self.log = open(log, "w")
        self.p = subprocess.Popen(
            argv,
            cwd=cwd,
            env=env,
            stdin=stdin,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            start_new_session=True,
        )
        threading.Thread(target=self._pump, daemon=True).start()

    def _pump(self):
        for line in self.p.stdout:
            self.log.write(line)
            self.log.flush()
            self.lines.append(line.rstrip())
            self.q.put(line.rstrip())
        self.q.put(None)

    def wait_line(self, prefix, timeout):
        end = time.monotonic() + timeout
        while (left := end - time.monotonic()) > 0:
            try:
                line = self.q.get(timeout=min(left, 1.0))
            except queue.Empty:
                continue
            if line is None:
                raise AssertionError(
                    f"{argv_name(self.p)} exited (rc={self.p.poll()}) before '{prefix}'"
                )
            if line.startswith(prefix):
                return line
        raise AssertionError(
            f"timeout {timeout}s waiting for '{prefix}' from {argv_name(self.p)}"
        )

    def kill(self):
        for sig in (signal.SIGTERM, signal.SIGKILL):
            try:
                os.killpg(self.p.pid, sig)
            except ProcessLookupError:
                break
            try:
                self.p.wait(5)
                break
            except subprocess.TimeoutExpired:
                pass
        # the group is gone once the leader is; sweep remaining members (px4 forks nothing outside its group)
        self.log.close()


def argv_name(p):
    return Path(p.args[0]).name


def strays(tmp):
    """Live processes of this run: command line names one of our binaries / the tmp dir, or cwd is the tmp dir."""
    needles = [str(PX4_BIN.parent) + "/", str(BRIDGE), str(REPLAY), str(tmp)]
    out = []
    for d in Path("/proc").iterdir():
        if not d.name.isdigit() or int(d.name) == os.getpid():
            continue
        try:
            cmd = (
                (d / "cmdline")
                .read_bytes()
                .replace(b"\0", b" ")
                .decode(errors="replace")
            )
            cwd = os.readlink(d / "cwd")
        except OSError:
            continue
        if any(n in cmd for n in needles) or cwd.startswith(str(tmp)):
            out.append(f"{d.name}: {cmd[:120]}")
    return out


def read_trace(path):
    import csv

    with open(path) as f:
        rows = list(csv.DictReader(f))
    return {k: np.array([float(r[k]) for r in rows]) for k in rows[0]}


async def fly(bridge, tmp, ev):
    from mavsdk.asyncio import ComponentType, Configuration, Mavsdk
    from mavsdk.asyncio.plugins.action import ActionAsync
    from mavsdk.asyncio.plugins.telemetry import FlightMode, LandedState, TelemetryAsync

    st = {"mode": None, "landed": None, "armed": None, "health": None, "texts": []}
    batt = []  # (wall time, V)

    async def sub(gen, fn):
        async for x in gen:
            fn(x)

    async def until(cond, timeout, what):
        end = time.monotonic() + timeout
        while not cond():
            if time.monotonic() > end:
                raise AssertionError(
                    f"timeout {timeout}s waiting for {what}; state={ {k: v for k, v in st.items() if k != 'texts'} }; "
                    f"PX4 status: {st['texts'][-8:]}"
                )
            await asyncio.sleep(0.2)

    mv = Mavsdk(Configuration.create_with_component_type(ComponentType.GROUND_STATION))
    tasks = []
    try:
        await mv.add_any_connection(f"udp://:{14540 + N}")
        system = await mv.first_autopilot(timeout_s=60.0)
        assert system is not None, "MAVSDK found no autopilot on the offboard link"
        tel, act = TelemetryAsync(system), ActionAsync(system)
        await tel.set_rate_battery(5.0)
        for gen, fn in (
            (
                tel.subscribe_battery(),
                lambda b: batt.append((time.time(), b.voltage_v)),
            ),
            (tel.subscribe_flight_mode(), lambda m: st.update(mode=m)),
            (tel.subscribe_landed_state(), lambda s: st.update(landed=s)),
            (tel.subscribe_armed(), lambda a: st.update(armed=a)),
            (tel.subscribe_health(), lambda h: st.update(health=h)),
            (
                tel.subscribe_status_text(),
                lambda t: st["texts"].append(f"{t.type.name}: {t.text}"),
            ),
        ):
            tasks.append(asyncio.create_task(sub(gen, fn)))

        await until(
            lambda: (
                st["health"]
                and st["health"].is_global_position_ok
                and st["health"].is_home_position_ok
            ),
            150,
            "global position + home position OK",
        )
        print(
            f"[flight] healthy at wall {time.time() - ev['t0']:.1f}s; PX4 status: {st['texts'][-3:]}",
            flush=True,
        )
        t_idle0 = time.time()
        await asyncio.sleep(6)
        idle = [v for t, v in batt if t >= t_idle0]
        assert len(idle) >= 10, (
            f"only {len(idle)} PX4 battery samples at idle (BATTERY_STATUS not accepted by PX4?)"
        )

        # z0: base z at rest on the ground (the model starts a few mm above it and settles), /world/base pos[2] as traced
        zs = read_trace(ev["trace"])["z"][-500:]  # last 2 s of ticks
        assert np.ptp(zs) < 1e-3, f"base not at rest before takeoff (ptp {np.ptp(zs)})"
        ev["z0"] = float(np.mean(zs))
        await act.set_takeoff_altitude(5.0)
        await until(lambda: st["health"].is_armable, 60, "armable")
        await act.arm()
        await act.takeoff()
        await until(lambda: st["mode"] == FlightMode.TAKEOFF, 30, "TAKEOFF mode")
        await until(lambda: st["mode"] == FlightMode.HOLD, 90, "HOLD after takeoff")
        # PX4 switches TAKEOFF -> HOLD once within NAV_MC_ALT_RAD (0.8 m) of the target, still climbing at ~0.5 m/s; let that
        # approach finish (5 s, lockstep at speed 1.0 so wall == sim) before the 30 s altitude window opens.
        await asyncio.sleep(5)
        z0 = ev["z0"]
        bridge.p.stdin.write(
            f"monitor alt_hold {ev['base']} pos 2 {z0 + 4.5} {z0 + 5.5} 30\n"
        )
        bridge.p.stdin.flush()
        ev["t_mon"] = time.time()
        line = await asyncio.get_running_loop().run_in_executor(
            None, bridge.wait_line, "monitor_result", 60
        )
        ev["mon"] = json.loads(line.split(" ", 1)[1])
        ev["t_mon_end"] = time.time()

        await act.land()
        await until(
            lambda: st["landed"] == LandedState.ON_GROUND and st["armed"] is False,
            90,
            "landed + disarmed",
        )
        await asyncio.sleep(1)
        ev.update(
            idle=idle,
            batt=batt,
            landed=st["landed"],
            armed=st["armed"],
            in_air=await tel.in_air(),
            texts=st["texts"],
        )
    finally:
        for t in tasks:
            t.cancel()
        await asyncio.gather(*tasks, return_exceptions=True)
        mv.destroy()


def model_info(model):
    """-> (bridge config, base prim, mass kg). asm: builds the assembly first (deterministic, no kernel needed)."""
    if model == "fixture":
        return CFGS[model], "/world/base", fixture_mass()
    from napl.design import build

    r = build(ASM, REPO / "parts", ASM_OUT)
    assert (ASM_OUT / "x500v2.xml").exists()
    return CFGS[model], "/world/bottom", r["report"]["mass"]["total"]


@pytest.mark.skipif(SKIP is not None, reason=SKIP or "")
@pytest.mark.parametrize("model", ["asm", "fixture"])
def test_px4_flight(model):
    assert not (set(PORTS_TCP + PORTS_UDP) & FORBIDDEN), (
        "instance maps onto a forbidden port"
    )
    assert not busy_ports("tcp", PORTS_TCP), (
        f"tcp ports busy: {busy_ports('tcp', PORTS_TCP)}"
    )
    assert not busy_ports("udp", PORTS_UDP), (
        f"udp ports busy: {busy_ports('udp', PORTS_UDP)}"
    )

    tmp = Path(tempfile.mkdtemp(prefix="napl_px4_"))
    run = tmp / "px4run"
    run.mkdir()
    trace, rlog = tmp / "trace.csv", tmp / "replay.log"
    cfg, base, mass = model_info(model)
    ev = {"t0": time.time(), "base": base, "mass": mass, "model": model}
    bridge = px4 = None
    keep = os.environ.get("NAPL_PX4_KEEP")
    try:
        bridge = Proc(
            [
                str(BRIDGE),
                "--config",
                str(cfg),
                "--port",
                str(4560 + N),
                "--px4-udp",
                str(18570 + N),
                "--udp-bind",
                str(14550 + N),
                "--speed",
                "1.0",
                "--trace",
                str(trace),
                "--log",
                str(rlog),
                "--connect-timeout",
                "120",
            ],
            tmp / "bridge.out",
            stdin=subprocess.PIPE,
        )
        bridge.wait_line("listening", 30)

        env = {k: v for k, v in os.environ.items() if not k.startswith("PX4_")}
        env["PX4_SYS_AUTOSTART"] = "4902"  # PX4_SIM_MODEL stays unset
        for line in PARAMS.read_text().splitlines():
            if line.strip() and not line.lstrip().startswith("#"):
                name, val = line.split(None, 1)
                env[f"PX4_PARAM_{name}"] = val.strip()
        assert env["PX4_PARAM_SIM_BAT_ENABLE"] == "0"
        px4 = Proc(
            [
                str(PX4_BIN),
                "-d",
                "-i",
                str(N),
                "-s",
                "etc/init.d-posix/rcS",
                str(B / "etc"),
            ],
            tmp / "px4.out",
            cwd=run,
            env=env,
        )
        bridge.wait_line("connected", 90)

        ev["trace"] = trace
        asyncio.run(fly(bridge, tmp, ev))

        bridge.p.stdin.write("quit\n")
        bridge.p.stdin.flush()
        bridge.p.stdin.close()
        bridge.wait_line("stats", 30)
        final = bridge.wait_line("final", 30)
        assert bridge.p.wait(30) == 0
        ev["wall"] = time.time() - ev["t0"]
        check(tmp, trace, rlog, final, ev)
    except BaseException:
        for p in (px4, bridge):
            if p:
                print(
                    f"----- tail of {p.log.name} -----\n" + "\n".join(p.lines[-40:]),
                    flush=True,
                )
        raise
    finally:
        for p in (bridge, px4):
            if p:
                try:
                    if p is bridge and p.p.poll() is None and not p.p.stdin.closed:
                        p.p.stdin.write("quit\n")
                        p.p.stdin.flush()
                        p.p.wait(10)
                except (OSError, ValueError, subprocess.TimeoutExpired):
                    pass  # best-effort graceful quit; kill() below is the guarantee
                p.kill()
        left = strays(tmp)
        if keep:
            print(f"[keep] run dir {tmp}")
        else:
            shutil.rmtree(tmp, ignore_errors=True)
        assert not left, f"stray processes after cleanup: {left}"


def check(tmp, trace, rlog, final, ev):
    tr = read_trace(trace)
    dt = tr["t"][-1] / tr["tick"][-1]
    mon = ev["mon"]

    # ---- hold window in trace rows: from the first row at/after the monitor command, `samples` ticks long ----
    i0 = int(np.searchsorted(tr["wall_s"], ev["t_mon"]))
    i1 = int(np.searchsorted(tr["tick"], tr["tick"][i0] + mon["samples"]))
    hold = slice(i0, i1)

    # a) monitor
    assert mon["pass"] and not mon["hit"], mon
    assert mon["samples"] == round(30 / dt), (mon["samples"], dt)

    # b) end state
    from mavsdk.asyncio.plugins.telemetry import LandedState

    assert (
        ev["landed"] == LandedState.ON_GROUND
        and not ev["in_air"]
        and ev["armed"] is False
    ), (ev["landed"], ev["in_air"], ev["armed"])
    z0 = ev["z0"]
    assert abs(tr["z"][-1] - z0) < 0.05, (tr["z"][-1], z0)
    assert abs(tr["vz"][-1]) < 0.05, tr["vz"][-1]

    # c) PX4 battery voltage drops under load
    batt = np.array(ev["batt"])
    np.savetxt(
        tmp / "px4_batt.csv", batt, delimiter=","
    )  # wall_s, PX4-reported V (kept with NAPL_PX4_KEEP=1)
    idle_v = float(np.mean(ev["idle"]))
    in_hold = batt[(batt[:, 0] >= ev["t_mon"]) & (batt[:, 0] <= ev["t_mon_end"]), 1]
    assert len(in_hold) >= 50, len(in_hold)
    hover_v = float(np.mean(in_hold))
    assert hover_v < idle_v - 0.1, (hover_v, idle_v)

    # d) PX4 voltage vs our battery device voltage at the nearest trace row by wall time
    t, v = batt[1:, 0], batt[1:, 1]
    j = np.clip(np.searchsorted(tr["wall_s"], t), 1, len(tr["wall_s"]) - 1)
    j = np.where(
        np.abs(tr["wall_s"][j - 1] - t) < np.abs(tr["wall_s"][j] - t), j - 1, j
    )
    err = np.abs(v - tr["batt_v"][j])
    # A PX4 sample is stamped at MAVSDK arrival, up to one 5 Hz period + link/stream latency after the bridge sampled it, and
    # the device voltage has true steps (ESC bus current drops 10 A -> 0 within ms when PX4 cuts throttle at touchdown: 0.2 V),
    # so the nearest-row error is unbounded for the sample straddling a step. Hard assertion: the PX4 value matches (<0.1 V)
    # some device voltage held in the preceding LAG seconds; the strict nearest-row error is printed and limited to 2 % of samples.
    LAG = 0.25
    lo = np.searchsorted(tr["wall_s"], t - LAG)
    hi = np.searchsorted(tr["wall_s"], t) + 1
    err_win = np.array(
        [np.abs(tr["batt_v"][a:b] - x).min() for a, b, x in zip(lo, hi, v)]
    )
    n_strict = int((err > 0.1).sum())
    assert err_win.max() < 0.1, (
        err_win.max(),
        float(t[err_win.argmax()] - ev["t_mon"]),
    )
    assert n_strict <= 0.02 * len(err), (n_strict, len(err))

    # e) ESC bus current vs the 2216 thrust table at per-motor hover thrust
    with open(MOTOR_TOML, "rb") as f:
        tab = tomllib.load(f)["tables"]["thrust_test"]
    col = {c: [r[k] for r in tab["rows"]] for k, c in enumerate(tab["columns"])}
    mass = ev["mass"]
    thrust = mass * G / 4
    i_tab = float(np.interp(thrust, col["thrust_N"], col["current_A"]))
    bus = [float(np.mean(tr[f"bus_current{k}"][hold])) for k in range(4)]
    for k, ib in enumerate(bus):
        assert ib > 0.5 and abs(ib - i_tab) <= 0.2 * i_tab, (k, ib, i_tab)

    # f) replay reproduces the bridge's final tick + hash
    m = re.fullmatch(r"final tick=(\d+) state_hash=([0-9a-f]+)", final)
    assert m, final
    r = subprocess.run(
        [str(REPLAY), str(rlog)], capture_output=True, text=True, timeout=300
    )
    assert r.returncode == 0, r.stderr
    m2 = re.search(r"tick=(\d+) state_hash=([0-9a-f]+)", r.stdout)
    assert m2 and m2.groups() == m.groups(), (r.stdout, final)

    # ---- results table ----
    thr = float(np.mean([np.mean(tr[f"throttle{k}"][hold]) for k in range(4)]))
    cur = float(np.mean(tr["batt_i"][hold]))
    tt = tr["t"][hold]
    slope = float(np.polyfit(tt, tr["batt_soc"][hold], 1)[0])
    soc0 = float(tr["batt_soc"][i0])
    full, to20 = soc0 / abs(slope), (soc0 - 0.2) / abs(slope)
    print(
        f"\n================ PX4 SITL flight results ({ev['model']}) ================"
    )
    rows = [
        ("hover throttle (mean of 4)", f"{thr:.4f}"),
        ("hover battery current", f"{cur:.2f} A"),
        ("per-ESC bus current [A]", " ".join(f"{b:.2f}" for b in bus)),
        (
            f"table current @ {thrust:.3f} N/motor (m={mass:.4f} kg, table 16.0 V)",
            f"{i_tab:.2f} A  (+-20%: {0.8 * i_tab:.2f}..{1.2 * i_tab:.2f})",
        ),
        ("our battery V in hold (mean)", f"{float(np.mean(tr['batt_v'][hold])):.2f} V"),
        (
            "PX4 voltage idle / hover",
            f"{idle_v:.3f} V / {hover_v:.3f} V  (drop {idle_v - hover_v:.3f} V)",
        ),
        (
            "max |PX4 V - device V| nearest row / within 250 ms lag",
            f"{err.max():.3f} V / {err_win.max():.3f} V over {len(err)} samples ({n_strict} strictly > 0.1 V)",
        ),
        ("SoC slope in hold", f"{slope:.3e} /s  (soc at hold start {soc0:.4f})"),
        (
            "predicted flight time to 0 % / to 20 % reserve",
            f"{full / 60:.1f} min / {to20 / 60:.1f} min  (Holybro claim ~18 min)",
        ),
        (
            "altitude min / max / rms (monitor, m)",
            f"{mon['min']:.3f} / {mon['max']:.3f} / {mon['rms']:.3f}  (z0={z0:.3f}, window {z0 + 4.5:.3f}..{z0 + 5.5:.3f})",
        ),
        ("monitor samples", f"{mon['samples']}"),
        ("sim duration / wall duration", f"{tr['t'][-1]:.1f} s / {ev['wall']:.1f} s"),
        ("final z - z0 / vz", f"{tr['z'][-1] - z0:+.4f} m / {tr['vz'][-1]:+.4f} m/s"),
        ("replay", f"{final}  == napl-replay"),
    ]
    for k, vtxt in rows:
        print(f"  {k:<58} {vtxt}")
    if ev["model"] == "asm":
        FLIGHT_JSON.parent.mkdir(parents=True, exist_ok=True)
        FLIGHT_JSON.write_text(
            json.dumps(
                {
                    "mass_kg": mass,
                    "thrust_per_motor_N": thrust,
                    "hover_throttle_mean": thr,
                    "hover_throttle_per_motor": [
                        float(np.mean(tr[f"throttle{k}"][hold])) for k in range(4)
                    ],
                    "bus_current_per_motor_A": bus,
                    "table_current_A": i_tab,
                    "battery_current_A": cur,
                    "battery_v_hold_mean": float(np.mean(tr["batt_v"][hold])),
                    "px4_v_idle": idle_v,
                    "px4_v_hover": hover_v,
                    "soc_at_hold_start": soc0,
                    "soc_slope_per_s": slope,
                    "flight_min_to_0": full / 60,
                    "flight_min_to_20pct_reserve": to20 / 60,
                    "alt_min_max_rms": [mon["min"], mon["max"], mon["rms"]],
                    "hold_samples": mon["samples"],
                    "replay": final,
                },
                indent=1,
            )
            + "\n"
        )
    print("=========================================================", flush=True)
