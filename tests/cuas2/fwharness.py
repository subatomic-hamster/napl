"""Test harness for the cuas2 firmware: loads the host build of the C core (ctypes), wraps the launcher + feeder unit with the same interface as the python twin
(napl.cuas2.fsm.FeederFsm), and plays the board around it (Plant: wheels, hardware limiter, pusher, gate, hopper, beams, fault injection) and, for the
whole-MCU tests, the axes, halls, E-stop and the host protocol (CoreRig).

The library is build/cuas2-fw-host/libcuas2_fwcore.so (tools/cuas2_fw_host_test.sh), or NAPL_CUAS2_FW=PATH.
"""

import copy
import ctypes as C
import math
import os
import types
import zlib
from pathlib import Path

from napl.cuas2 import protocol as p
from napl.cuas2.fsm import FeederFsm, FeederIn, FeederOut

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_LIB = ROOT / "build" / "cuas2-fw-host" / "libcuas2_fwcore.so"
INT32_MIN = -(2**31)
NEV = 12


def lib_path():
    v = os.environ.get("NAPL_CUAS2_FW", "")
    return Path(v) if v else DEFAULT_LIB


def available():
    return lib_path().exists()


def require_lib():
    """For test fixtures: a missing host library is a failure when CUAS2_FW_REQUIRED=1 (tools/cuas2_fw_host_test.sh sets it), else a skip."""
    if not available():
        import pytest

        msg = f"host library missing: {lib_path()} (run tools/cuas2_fw_host_test.sh)"
        (pytest.fail if os.environ.get("CUAS2_FW_REQUIRED") else pytest.skip)(msg)


_libs = {}


def lib(path=None):
    path = Path(path) if path else lib_path()
    L = _libs.get(path)
    if L is None:
        L = C.CDLL(str(path))
        u8p, i32p = C.POINTER(C.c_uint8), C.POINTER(C.c_int32)
        L.fwh_new.restype = C.c_void_p
        L.fwh_new.argtypes = [i32p, C.c_int, C.c_char_p]
        L.fwh_clone.restype = C.c_void_p
        L.fwh_clone.argtypes = [C.c_void_p]
        L.fwh_free.argtypes = [C.c_void_p]
        L.fwh_tick.restype = C.c_int
        L.fwh_tick.argtypes = [C.c_void_p, C.c_uint32, C.c_char_p, C.c_int, i32p]
        L.fwh_tx.restype = u8p
        L.fwh_tx.argtypes = [C.c_void_p]
        L.fwh_out.argtypes = [C.c_void_p, i32p]
        L.fwh_snap.argtypes = [C.c_void_p, i32p, C.POINTER(C.c_double)]
        L.fwh_cfg.restype = C.c_int
        L.fwh_cfg.argtypes = [C.c_void_p, C.c_int, C.c_int]
        L.fwh_poke_cfg.argtypes = [C.c_void_p, C.c_int, C.c_int]
        L.fwh_trace_count.argtypes = [C.c_void_p]
        L.fwh_trace_get.argtypes = [C.c_void_p, C.c_int, i32p]
        L.fwh_trace_clear.argtypes = [C.c_void_p]
        L.fwh_crc16.argtypes = [C.c_char_p, C.c_int]
        L.fwh_cobs_encode.argtypes = [C.c_char_p, C.c_int, u8p]
        L.fwh_cobs_decode.argtypes = [C.c_char_p, C.c_int, u8p, C.c_int]
        L.fwh_frame_encode.argtypes = [C.c_int, C.c_int, C.c_char_p, C.c_int, u8p]
        L.fwh_decode_stream.argtypes = [
            C.c_char_p,
            C.c_int,
            u8p,
            C.c_int,
            C.POINTER(C.c_uint32),
        ]
        L.fwh_cfg_defaults.argtypes = [i32p]
        L.fwh_rpm_cap.argtypes = [i32p]
        L.fwh_dshot_frame.argtypes = [C.c_int, C.c_int, C.c_int]
        L.fwh_gcr_encode.restype = C.c_uint32
        L.fwh_gcr_encode.argtypes = [C.c_uint32]
        L.fwh_gcr_decode.argtypes = [C.c_uint32, i32p]
        L.fwh_payload_rpm.argtypes = [C.c_int, C.c_int]
        L.fdrh_new.restype = C.c_void_p
        L.fdrh_free.argtypes = [C.c_void_p]
        L.fdrh_step.argtypes = [C.c_void_p, i32p, i32p, i32p]
        _libs[path] = L
    return L


def cfg_array(overrides=None):
    """CFG_COUNT int32 with the C defaults and the overrides applied (the way the feeder wants it)."""
    a = (C.c_int32 * p.CFG_COUNT)()
    lib().fwh_cfg_defaults(a)
    for k, v in (overrides or {}).items():
        a[k] = v
    return a


# ---- the launcher + feeder unit, C and python behind one interface ------------------------------------------------------------------------
class CFeeder:
    """napl.cuas2.fsm.FeederFsm's interface over the C core (fdrh_step). `path` selects another build of the library (negative controls)."""

    def __init__(self, path=None):
        self.L = lib(path)
        self.h = self.L.fdrh_new()
        self._in = (C.c_int32 * 18)()
        self._out = (C.c_int32 * (22 + 2 * NEV))()
        self._cfg_key, self._cfg = None, None

    def __del__(self):
        h, self.h = getattr(self, "h", None), None
        if h:
            self.L.fdrh_free(h)

    def step(self, inp: FeederIn) -> FeederOut:
        key = tuple(sorted((inp.cfg or {}).items()))
        if key != self._cfg_key:
            self._cfg_key, self._cfg = key, cfg_array(inp.cfg)
        a = self._in
        dpm = 0x7FFF if inp.diff_pm is None else inp.diff_pm
        cmd = {None: 0, "FIRE": 1, "CLEAR": 2}[inp.cmd]
        vals = (inp.t_ms, -1 if inp.rpm_top is None else inp.rpm_top, -1 if inp.rpm_bot is None else inp.rpm_bot, inp.beam_breech, inp.beam_muzzle,
                inp.gate_home, inp.push_home, inp.estop_open, inp.key_on, inp.hb_ok, inp.deadman, inp.inhibit, inp.in_sector, cmd, inp.speed_req_pm,
                dpm, inp.fire_id, inp.muzzle_dt_us)  # fmt: skip
        for i, v in enumerate(vals):
            a[i] = int(v)
        if dpm == 0x7FFF:  # FDR_DIFF_CFG: the C side takes the value of cfg 29
            a[15] = self._cfg[p.CFG_DIFF_PM]
        self.L.fdrh_step(self.h, a, self._cfg, self._out)
        o = list(self._out)
        names = ("rpm_top", "rpm_bot", "rpm_set", "throttle_mean_pm", "beams", "gate_state", "ready_flags", "feed_ms", "feed_busy", "wheels_ready",
                 "last_v_dmps", "last_dip_pm", "shots_confirmed", "fault_bits")  # fmt: skip
        sf = dict(zip(names, o[7:21]))
        sf["rpm_cap"] = self.L.fwh_rpm_cap(self._cfg)
        sf = {k: sf[k] for k in ("rpm_top", "rpm_bot", "rpm_set", "throttle_mean_pm", "beams", "gate_state", "ready_flags", "feed_ms", "feed_busy",
                                 "wheels_ready", "last_v_dmps", "last_dip_pm", "shots_confirmed", "fault_bits", "rpm_cap")}  # fmt: skip
        nev = o[21]
        ev = [(o[22 + 2 * i], o[23 + 2 * i]) for i in range(nev)]
        return FeederOut(
            o[0],
            o[1],
            o[2],
            o[3],
            bool(o[4]),
            bool(o[5]),
            sf,
            ev,
            None if o[6] < 0 else o[6],
        )


# ---- the board around the feeder ----------------------------------------------------------------------------------------------------------
class Plant:
    """Ideal-ish physical model at 1 ms: two flywheels (first order, tau 40 ms, 20,100 rpm at full scale), the hardware limiter (servo power 250 ms after an
    accepted FEED_REQ edge, further edges ignored for 330 ms), the pusher rod (80 ms stroke), the plug-valve gate (100 ms swing, blocked while the rod is
    out), the pocket / hopper, the breech and muzzle beams, and fault injection.

    faults (kwargs): breech_stuck, muzzle_stuck (None | 0 | 1, from t >= stuck_t); pusher ('ok' | 'dead' | 'stuck_ext' | 'stuck_home'); gate ('ok' |
    'stuck_load' | 'stuck_bore'); esc_dead [bool, bool]; esc_runaway [bool, bool]; telemetry_dead_t (rpm replies stop from this tick); feed_req_stuck
    ('low' | 'high' from stuck_t); limiter_shorted; exit_blocked (the ball never leaves the barrel); breech_delay_ms (extra block time after the push: a 2-ball
    pocket); spurious_muzzle (list of ticks with a ghost pulse); hopper (balls); ball_rpm_min."""

    def __init__(self, cfg=None, **faults):
        self.cfg = dict(p.CFG_DEFAULTS2)
        self.cfg.update(cfg or {})
        self.f = dict(faults)
        self.t = 0
        self.rpm = [0.0, 0.0]
        self.rod = 0.0
        self.gate = 0.0
        self.pocket = 0
        self.hopper = faults.get("hopper", 24)
        self.load_ms = 0
        self.barrel = (
            None  # ms left until the ball in the barrel leaves (None = no ball)
        )
        self.breech_hold = 0
        self.powered_until = -1
        self.last_edge = -(10**9)
        self.edge_prev = False
        self.exits = []  # (t, v_mps)
        self.pulse = 0
        self.pulse_dt = 0
        self.feed_edges = []  # ticks at which the limiter accepted an edge
        self.req_edges = []  # ticks at which FEED_REQ rose (as the MCU drove it)

    def inputs(self, t):
        """Sampled inputs for tick t (before the MCU sees them)."""
        f = self.f
        st = f.get("stuck_t", 0)
        breech = bool(
            self.gate >= 0.95
            and ((self.pocket and self.rod < 0.5) or self.breech_hold > 0)
        )
        if f.get("breech_stuck") is not None and t >= st:
            breech = bool(f["breech_stuck"])
        muz = bool(self.pulse > 0 or t in f.get("spurious_muzzle", ()))
        if f.get("muzzle_stuck") is not None and t >= st:
            muz = bool(f["muzzle_stuck"])
        rpm = [
            None if t >= f.get("telemetry_dead_t", 10**9) else int(r) for r in self.rpm
        ]
        dt = (
            self.pulse_dt
            if self.pulse > 0
            else (800 if t in f.get("spurious_muzzle", ()) else 0)
        )
        return dict(rpm_top=rpm[0], rpm_bot=rpm[1], beam_breech=breech, beam_muzzle=muz, gate_home=self.gate <= 0.02, push_home=self.rod <= 0.02,
                    muzzle_dt_us=dt if muz else 0)  # fmt: skip

    def step(self, out: FeederOut, t, rail=True):
        """Advance one millisecond with the MCU's outputs of tick t."""
        f, c = self.f, self.cfg
        self.t = t
        # wheels
        for i, ds in enumerate((out.dshot_top, out.dshot_bot)):
            tgt = (ds - 48) / 1952.0 * 20100.0 if ds > 0 else 0.0
            if f.get("esc_dead", (0, 0))[i]:
                tgt = 0.0
            if f.get("esc_runaway", (0, 0))[i] and t >= f.get("runaway_t", 0):
                tgt = 22700.0
            if not rail:
                tgt = 0.0
            self.rpm[i] += (tgt - self.rpm[i]) / 40.0
        # hardware limiter on the pusher servo supply
        req = bool(out.feed_req)
        mode = f.get("feed_req_stuck")
        st = f.get("stuck_t", 0)
        if mode == "low" and t >= st:
            req = False
        if mode == "high" and t >= st:
            req = True
        rising = req and not self.edge_prev
        self.edge_prev = req
        if rising:
            self.req_edges.append(t)
            if t - self.last_edge >= 330:
                self.last_edge = t
                self.powered_until = t + 250
                self.feed_edges.append(t)
        powered = rail and (t < self.powered_until or f.get("limiter_shorted"))
        mode = f.get("pusher", "ok") if t >= st else "ok"
        if powered and mode != "dead" and mode != "stuck_home":
            go_out = (
                out.push_us == c[p.CFG_PUSHER_EXT_US]
                or mode == "stuck_ext"
                and self.rod >= 0.5
            )
            self.rod = (
                min(1.0, self.rod + 1 / 80.0)
                if go_out
                else max(0.0, self.rod - 1 / 80.0)
            )
        # gate (valve cannot turn while the rod is out)
        gm = f.get("gate", "ok") if t >= st else "ok"
        if rail and self.rod <= 0.3 and gm != "stuck_load":
            to_bore = out.gate_us == c[p.CFG_GATE_BORE_US]
            if gm == "stuck_bore":
                to_bore = True
            self.gate = (
                min(1.0, self.gate + 0.01) if to_bore else max(0.0, self.gate - 0.01)
            )
        if gm == "stuck_load":
            self.gate = 0.0
        # pocket / hopper
        if self.gate <= 0.05 and not self.pocket and self.hopper > 0:
            self.load_ms += 1
            if self.load_ms >= 10:
                self.pocket, self.hopper, self.load_ms = 1, self.hopper - 1, 0
        else:
            self.load_ms = 0
        # the push: the ball leaves the bore at rod 0.5, reaches the nip, and leaves the barrel 15 ms later if the wheels turn
        if self.pocket and self.gate >= 0.95 and self.rod >= 0.5:
            self.pocket = 0
            self.barrel = 15
            self.breech_hold = f.get("breech_delay_ms", 0)
        elif self.breech_hold > 0:
            self.breech_hold -= 1
        self.pulse = max(0, self.pulse - 1)
        if self.barrel is not None:
            mean = (self.rpm[0] + self.rpm[1]) / 2
            if self.barrel > 0:
                self.barrel -= 1
            if (
                self.barrel == 0
                and mean >= f.get("ball_rpm_min", 3000)
                and not f.get("exit_blocked")
            ):
                v = 0.8 * math.pi * 0.040 * mean / 60.0
                self.exits.append((t, v))
                self.pulse, self.pulse_dt = 1, max(1, int(22200 / v))
                self.rpm = [r * 0.947 for r in self.rpm]
                self.barrel = None


# ---- scripted runs -----------------------------------------------------------------------------------------------------------------------------
IN_FIELDS = ("rpm_top", "rpm_bot", "beam_breech", "beam_muzzle", "gate_home", "push_home", "estop_open", "key_on", "hb_ok", "deadman", "inhibit",
             "in_sector", "speed_req_pm", "diff_pm", "fire_id", "muzzle_dt_us")  # fmt: skip
DISCRETE = (
    "feed_req",
    "kick",
    "gate_us",
    "push_us",
    "gate_state",
    "ready_flags",
    "beams",
    "fault_bits",
    "shots_confirmed",
    "feed_busy",
    "wheels_ready",
)
CONTINUOUS = (
    "dshot_top",
    "dshot_bot",
    "rpm_top",
    "rpm_bot",
    "rpm_set",
    "throttle_mean_pm",
    "last_v_dmps",
    "last_dip_pm",
    "feed_ms",
)
CTRL0 = dict(
    estop_open=False,
    key_on=True,
    hb_ok=True,
    deadman=True,
    inhibit=False,
    in_sector=True,
    speed_req_pm=0,
    diff_pm=None,
)


def run_script(stepper, plant: Plant, script, n, cfg=None):
    """Closed loop: `script` maps tick -> dict of control overrides (estop_open, key_on, hb_ok, deadman, inhibit, in_sector, speed_req_pm, diff_pm) and the keys
    'FIRE' (fire_id) / 'CLEAR' (True) for commands. -> (inputs, outs): per-tick FeederIn field dicts (+ 'cmd') and FeederOut."""
    ctl = dict(CTRL0)
    ins, outs = [], []
    for t in range(1, n + 1):
        ev = script.get(t, {})
        ctl.update({k: v for k, v in ev.items() if k in CTRL0})
        cmd, fid = None, 0
        if "FIRE" in ev:
            cmd, fid = "FIRE", ev["FIRE"]
        if ev.get("CLEAR"):
            cmd = "CLEAR"
        d = plant.inputs(t)
        d.update(ctl)
        d.update(fire_id=fid, cmd=cmd)
        o = stepper.step(FeederIn(t_ms=t, cfg=cfg, **d))
        plant.step(o, t, rail=not ctl["estop_open"] and ctl["key_on"])
        ins.append(d)
        outs.append(o)
    return ins, outs


def trace_of(outs, ins=None, block=100):
    """Canonical comparable trace of a run: change lists of the discrete outputs, events, acks and a CRC32 per `block` ticks of the continuous ones."""
    chg = {k: [] for k in DISCRETE}
    last = {}
    events, acks, crcs = [], [], []
    buf = bytearray()
    for i, o in enumerate(outs):
        t = i + 1
        row = dict(o.status_fields)
        row.update(
            feed_req=int(o.feed_req),
            kick=int(o.kick),
            gate_us=o.gate_us,
            push_us=o.push_us,
            dshot_top=o.dshot_top,
            dshot_bot=o.dshot_bot,
        )
        for k in DISCRETE:
            if last.get(k) != row[k]:
                chg[k].append([t, row[k]])
                last[k] = row[k]
        for code, arg in o.events:
            events.append([t, code, arg])
        if o.ack is not None:
            acks.append([t, o.ack])
        for k in CONTINUOUS:
            buf += int(row[k]).to_bytes(4, "little", signed=False)
        if t % block == 0:
            crcs.append(zlib.crc32(bytes(buf)))
            buf = bytearray()
    if buf:
        crcs.append(zlib.crc32(bytes(buf)))
    return dict(changes=chg, events=events, acks=acks, crc=crcs)


def inputs_to_changes(ins):
    """Per-tick input dicts -> change lists per field (+ the commands) for the vector file."""
    ch = {k: [] for k in IN_FIELDS}
    cmds = []
    last = {}
    for i, d in enumerate(ins):
        t = i + 1
        for k in IN_FIELDS:
            v = d[k]
            v = int(v) if isinstance(v, bool) else v
            if last.get(k, "unset") != v:
                ch[k].append([t, v])
                last[k] = v
        if d["cmd"]:
            cmds.append([t, d["cmd"], d["fire_id"]])
    return ch, cmds


def changes_to_inputs(ch, cmds, n):
    cur = {k: 0 for k in IN_FIELDS}
    idx = {k: 0 for k in IN_FIELDS}
    cm = {t: (c, f) for t, c, f in cmds}
    for t in range(1, n + 1):
        for k in IN_FIELDS:
            lst = ch[k]
            while idx[k] < len(lst) and lst[idx[k]][0] == t:
                cur[k] = lst[idx[k]][1]
                idx[k] += 1
        d = dict(cur)
        for k in (
            "beam_breech",
            "beam_muzzle",
            "gate_home",
            "push_home",
            "estop_open",
            "key_on",
            "hb_ok",
            "deadman",
            "inhibit",
            "in_sector",
        ):
            d[k] = bool(d[k])
        c, fid = cm.get(t, (None, 0))
        d["cmd"], d["fire_id"] = c, fid
        yield d


def replay(stepper, vec, cfg_over=None):
    """Drive `stepper` with the stored inputs of a vector; -> its trace (same structure as trace_of)."""
    cfg = dict(vec.get("cfg", {}))
    cfg = {int(k): v for k, v in cfg.items()}
    outs = []
    for t, d in enumerate(
        changes_to_inputs(vec["inputs"], vec["cmds"], vec["ticks"]), start=1
    ):
        outs.append(stepper.step(FeederIn(t_ms=t, cfg=cfg, **d)))
    return trace_of(outs)


def expect_view(outs):
    """The quantities a vector's hand-written `expect` block speaks about, read off a run's outputs."""
    tr = trace_of(outs)
    f = outs[-1].status_fields
    ev = tr["events"]
    kick = tr["changes"]["kick"]
    t_fault = next((t for t, c, _ in ev if c == p.EV_FAULT), None)
    t_low = next(
        (t for t, v in kick if v == 0 and t_fault is not None and t >= t_fault), None
    )
    return dict(
        acks=[a[1] for a in tr["acks"]],
        codes=[e[1] for e in ev],
        rises=sum(1 for _, v in tr["changes"]["feed_req"] if v == 1),
        faults=f["fault_bits"],
        confirmed=f["shots_confirmed"],
        gate=f["gate_state"],
        kick_low_within=None if t_low is None else t_low - t_fault,
    )  # fmt: skip


def check_expect(name, expect, view):
    """-> list of mismatch strings (empty = the view satisfies the expect block; 'gate' and 'kick_low_within' are checked only when given)."""
    bad = []
    for k, want in expect.items():
        if view[k] != want:
            bad.append(f"{name}: {k}: want {want!r}, got {view[k]!r}")
    return bad


# ---- the whole MCU: C core + board (axes, halls, wheels, limiter, relay supervisor) + host protocol --------------------------------------------
_TICK_IN = (C.c_int32 * 12)()
_TICK_OUT = (C.c_int32 * 20)()
_SNAP_I = (C.c_int32 * 24)()
_SNAP_D = (C.c_double * 6)()
SNAP_KEYS = ("state", "hb_run", "last_hb_ms", "shot_cap", "shots_arm", "shots_total", "fault_bits", "version_bad", "hello_done", "wd_tripped", "homed0",
             "homed1", "session_valid", "steppers_enabled", "estop_open", "key_on", "gstate", "feeds", "muz_all", "ready_flags", "sector_set", "conf_arm",
             "last_fire_id", "wheels_ready")  # fmt: skip
OUT_KEYS = ("steppers_en", "step_n0", "step_n1", "dir_invert0", "dir_invert1", "fw_duty", "pusher_us", "gate_us", "dshot_top", "dshot_bot", "feed_req",
            "kick_ok", "kick_level", "shots_fed", "request_bootsel", "led_red", "led_green", "buzzer")  # fmt: skip
UID = bytes.fromhex("e66050a8d3156f21")


class CoreRig:
    """One whole simulated cuas2 MCU at 1 ms: the C core (fwh_*), the board around it (steppers -> joint angles -> hall windows, the Plant of the feeder,
    the hardware supervisor: relay opens when KICK has not toggled for 150 ms, key off or E-stop open) and the host (framing via napl.cuas2.protocol,
    heartbeats every 50 ms, FLYWHEEL every 100 ms). `fork()` deep-copies the whole rig (the C state with fwh_clone) for the fuzz."""

    def __init__(self, cfg=None, lib_file=None, flash_mode=1, **plant_faults):
        self.L = lib(lib_file)
        spr0, spr1 = p.CFG_DEFAULTS2[p.CFG_PAN_SPR], p.CFG_DEFAULTS2[p.CFG_TILT_SPR]
        ov = (C.c_int32 * p.CFG_COUNT)(*([INT32_MIN] * p.CFG_COUNT))
        ov[p.CFG_PAN_HOME_OFF], ov[p.CFG_TILT_HOME_OFF] = (
            round(-spr0 / 360),
            round(-spr1 / 360),
        )
        for k, v in (cfg or {}).items():
            ov[k] = v
        self.h = self.L.fwh_new(ov, flash_mode, UID)
        self.diag = 0  # TMC2209 DIAG level
        self.fault_t = None
        self.plant = Plant(cfg=cfg, **plant_faults)
        self.t = 0
        self.joint = [0.0, 0.0]
        self.dec = p.Decoder2()
        self.seq = 0
        self.hb_seq = 0
        self.rx = bytearray(b"\0")  # resync byte: the MCU's decoder starts unsynced
        self.hb, self.deadman, self.inhibit = True, True, False
        self.fw_pm = (
            None  # None: no FLYWHEEL frames; else the speed permille sent every 100 ms
        )
        self.fw_diff = None
        self.estop_open, self.key_on = False, True
        self.st = None  # latest Status2
        self.events, self.acks = [], []  # (t, code, arg), (t, p.Ack)
        self.feed_rises = []  # ticks at which the MCU raised FEED_REQ
        self.kick_log = []  # (t, kick_ok) changes
        self.last_toggle = 0
        self.rail = True
        self.o = dict.fromkeys(OUT_KEYS, 0)
        self.snap = dict.fromkeys(SNAP_KEYS, 0)
        self.accepted_ids = set()  # distinct fire_ids acked OK (a retransmission's repeated OK is not a second shot)
        self.fire_by_seq = {}
        self._prev_req = 0
        self._prev_kick = None

    def __del__(self):
        h, self.h = getattr(self, "h", None), None
        if h:
            self.L.fwh_free(h)

    def fork(self):
        h, L, self.h, self.L = self.h, self.L, None, None
        try:
            c = copy.deepcopy(self)
        finally:
            self.h, self.L = h, L
        c.L, c.h = L, L.fwh_clone(h)
        return c

    # ---- host side ----
    def send(self, m):
        self.seq = (self.seq + 1) & 0xFF
        self.rx += p.encode(self.seq, m)
        if isinstance(m, p.Fire):
            self.fire_by_seq[self.seq] = m.fire_id
        return self.seq

    @property
    def accepted_fires(self):
        return len(self.accepted_ids)

    def send_raw(self, b):
        self.rx += b

    def fire(self, fire_id):
        return self.send(p.Fire(fire_id, 1))

    def last_ack(self, cmd_id):
        for _, a in reversed(self.acks):
            if a.cmd_id == cmd_id:
                return a
        return None

    # ---- time ----
    def tick(self):
        self.t += 1
        t, pl, L = self.t, self.plant, self.L
        if self.hb and t % 50 == 0:
            self.hb_seq = (self.hb_seq + 1) & 0xFFFF
            self.send(
                p.Heartbeat(
                    self.hb_seq,
                    (p.HB_DEADMAN_OK if self.deadman else 0)
                    | (p.HB_FIRE_INHIBIT if self.inhibit else 0),
                )
            )
        if self.fw_pm is not None and t % 100 == 0:
            self.send(p.Flywheel(self.fw_pm, self.fw_diff))
        d = pl.inputs(t)
        a = _TICK_IN
        hall = [int(abs(self.joint[i]) < 1.0) for i in (0, 1)]
        for i, v in enumerate((int(self.estop_open), int(self.key_on), hall[0], hall[1], self.diag, d["beam_breech"], d["beam_muzzle"], d["gate_home"],
                               d["push_home"], -1 if d["rpm_top"] is None else d["rpm_top"], -1 if d["rpm_bot"] is None else d["rpm_bot"],
                               d["muzzle_dt_us"])):  # fmt: skip
            a[i] = int(v)
        rx, self.rx = bytes(self.rx), bytearray()
        n = L.fwh_tick(self.h, t, rx, len(rx), a)
        if n:
            for f in self.dec.feed(bytes(L.fwh_tx(self.h)[:n])):
                m = f.msg
                if isinstance(m, p.Status2):
                    self.st = m
                    self.status_len = len(f.payload)
                elif isinstance(m, p.Event):
                    self.events.append((t, m.code, m.arg))
                elif isinstance(m, p.Ack):
                    self.acks.append((t, m))
                    if m.cmd_id == p.FIRE and m.result == p.OK:
                        self.accepted_ids.add(self.fire_by_seq.get(m.cmd_seq))
        L.fwh_out(self.h, _TICK_OUT)
        o = dict(zip(OUT_KEYS, _TICK_OUT))
        self.o = o
        L.fwh_snap(self.h, _SNAP_I, _SNAP_D)
        self.snap = dict(zip(SNAP_KEYS, _SNAP_I))
        if self.fault_t is None and self.snap["fault_bits"]:
            self.fault_t = t  # first tick with a latched fault
        for i in (0, 1):
            if o[f"step_n{i}"]:
                spr = L.fwh_cfg(self.h, p.CFG_PAN_SPR + i, 0)
                self.joint[i] += (
                    (-1 if o[f"dir_invert{i}"] else 1) * o[f"step_n{i}"] * 360.0 / spr
                )
        # supervisor: KICK must toggle (kick_ok high and level moving) or the relay opens after 150 ms; key off / E-stop open open it at once
        if o["kick_ok"] and o["kick_level"] != self._prev_kick:
            self.last_toggle = t
        self._prev_kick = o["kick_level"]
        if o["kick_ok"] != (self.kick_log[-1][1] if self.kick_log else None):
            self.kick_log.append((t, o["kick_ok"]))
        self.rail = bool(
            self.key_on and not self.estop_open and t - self.last_toggle < 150
        )
        if o["feed_req"] and not self._prev_req:
            self.feed_rises.append(t)
        self._prev_req = o["feed_req"]
        pl.step(types.SimpleNamespace(dshot_top=o["dshot_top"], dshot_bot=o["dshot_bot"], feed_req=o["feed_req"], push_us=o["pusher_us"],
                                      gate_us=o["gate_us"]), t, rail=self.rail)  # fmt: skip

    def run(self, ms):
        for _ in range(ms):
            self.tick()

    def up(self, home=True, sector=True):
        """HELLO, heartbeats -> IDLE, sector keys, home both axes, tilt above the fire minimum."""
        self.send(p.Hello(p.MAJOR, p.MINOR, 7))
        self.run(1300)
        assert self.st.state == p.IDLE, p.STATE_NAMES[self.st.state]
        if sector:
            for k, v in zip(p.SECTOR_KEYS, (0, -170000, 170000, 70000)):
                self.send(p.CfgSet(k, v))
            self.run(50)
        if home:
            self.send(p.Home(3))
            self.run(3000)
            assert self.st.flag1(p.F1_PAN_HOMED) and self.st.flag1(p.F1_TILT_HOMED)
            self.send(p.Aim(0, 5000))
            self.run(1000)
        return self

    def arm(self, cap=5, timeout=60):
        self.send(p.Arm(1, cap, timeout))
        self.run(30)

    def engage(self, pm=1000, spin_ms=1500):
        self.arm()
        self.fw_pm = pm
        self.run(spin_ms)
        return self


__all__ = [
    "CFeeder",
    "CoreRig",
    "FeederFsm",
    "Plant",
    "available",
    "cfg_array",
    "lib",
    "replay",
    "run_script",
    "trace_of",
]
