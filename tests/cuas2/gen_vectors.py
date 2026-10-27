"""Prints tests/cuas2/protocol_vectors2.json or tests/cuas2/fsm_vectors.json:

    PYTHONPATH=python python tests/cuas2/gen_vectors.py protocol > tests/cuas2/protocol_vectors2.json
    PYTHONPATH=python python tests/cuas2/gen_vectors.py fsm      > tests/cuas2/fsm_vectors.json

protocol: wire frames of the new/changed messages from napl.cuas2.protocol, plus the REFERENCE values that do not come from the module (CRC check value
0x29B1, the DShot article's frames); behaviour scripts (frames in, ACKs out) for the whole C core are in test_protocol2.py.

fsm: the spec 6.6 vectors (a)-(o) plus extras. Each vector is a scripted closed-loop run of the python twin against a board model (fwharness.Plant); what is
stored is the INPUT stream the feeder saw (change lists), the output trace the twin produced, and an `expect` block written by hand from the spec (ack
results, event codes, fault bits, counts) that test_fsm.py checks independently of both implementations. The C core must reproduce the same trace from the
same inputs (test_fw_vectors2.py).
"""

import json
import sys
from pathlib import Path

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
    from cuas2 import fwharness as H
else:
    from . import fwharness as H

from napl.cuas2 import protocol as p
from napl.lite import protocol as lite

FIRE = "FIRE"
SPEED = {1: dict(speed_req_pm=1000)}  # the app asks for 17,500 rpm from the first tick


def sc(*extra):
    s = {t: dict(v) for t, v in SPEED.items()}
    for t, v in extra:
        s.setdefault(t, {}).update(v)
    return s


# name -> dict(desc, n, cfg, plant, script, expect). expect keys: acks (list of results), codes (event codes in order), rises (FEED_REQ rising edges),
# faults (final fault_bits), confirmed (final shots_confirmed), gate (final gate_state), kick_low_within (ticks from the first FAULT event to kick low).
V = {}


def vec(name, desc, n, script, plant=None, cfg=None, **expect):
    V[name] = dict(
        desc=desc, n=n, script=script, plant=plant or {}, cfg=cfg or {}, expect=expect
    )


# ---- the 15 of spec 6.6 -----------------------------------------------------------------------------------------------------------------------
vec("a_nominal_two_shots", "(a) nominal shots: two accepted FIREs 500 ms apart, one stroke and one exit each", 1900,
    sc((800, {FIRE: 1}), (1300, {FIRE: 2})),
    acks=[0, 0], codes=[14, 2, 15, 10, 14, 2, 15, 10, 14], rises=2, faults=0, confirmed=2, gate=4)  # fmt: skip
vec("b_not_spun", "(b) FIRE while the wheels are still ramping -> NOT_SPUN, nothing fed", 500, sc((300, {FIRE: 1})),
    acks=[10], codes=[], rises=0, faults=0, confirmed=0, gate=4)  # fmt: skip
vec("c_no_ball", "(c) the staged ball is gone (breech reads clear) -> NO_BALL", 1000, sc((800, {FIRE: 1})), plant=dict(breech_stuck=0, stuck_t=700),
    acks=[12], codes=[14], rises=0, faults=0, confirmed=0, gate=4)  # fmt: skip
vec("d_not_clear", "(d) the muzzle beam is blocked from power-up -> NOT_CLEAR", 1000, sc((800, {FIRE: 1})), plant=dict(muzzle_stuck=1),
    acks=[14], codes=[14], rises=0, faults=0, confirmed=0, gate=4)  # fmt: skip
vec("e_sector_abort", "(e) sector lost 20 ms into the stroke (aborted, EVENT 11+9); a second one lost 60 ms in (past the commit: the ball leaves)", 1700,
    sc((800, {FIRE: 1}), (820, {"in_sector": False}), (830, {"in_sector": True}), (1300, {FIRE: 2}), (1360, {"in_sector": False}), (1400, {"in_sector": True})),
    acks=[0, 0], codes=[14, 11, 9, 2, 15, 10, 14], rises=2, faults=0, confirmed=1)  # fmt: skip
vec("f_deadman_drop", "(f) dead-man drops 15 ms into the stroke -> abort before commit, the ball stays staged", 1300,
    sc((800, {FIRE: 1}), (815, {"deadman": False}), (900, {"deadman": True})),
    acks=[0], codes=[14, 11], rises=1, faults=0, confirmed=0, gate=4)  # fmt: skip
vec("g_no_exit_jam", "(g) the ball never leaves the barrel -> FAULT no_exit at 250 ms, KICK stops, CLEAR_FAULT re-homes", 1900,
    sc((800, {FIRE: 1}), (1200, {"CLEAR": True})), plant=dict(exit_blocked=True),
    acks=[0], codes=[14, 2, 4, 13, 14], rises=1, faults=0, confirmed=0, kick_low_within=0)  # fmt: skip
vec("h_unexpected_exit", "(h) a muzzle pulse with no feed -> FAULT unexpected_exit + shot_overcount, KICK stops, FIRE answers JAMMED", 1000,
    sc((800, {FIRE: 1})), plant=dict(spurious_muzzle=[600]),
    acks=[13], codes=[14, 4, 4, 13], rises=0, faults=0x4800, confirmed=1, kick_low_within=0)  # fmt: skip
vec("i_interval_clamp", "(i) cfg min interval 100 ms is clamped to 400: FIRE at +360 ms is TOO_SOON, at +410 ms accepted", 1700,
    sc((800, {FIRE: 1}), (1160, {FIRE: 2}), (1210, {FIRE: 3})), cfg={p.CFG_MIN_SHOT_INTERVAL_MS: 100, p.CFG_PUSHER_DWELL_MS: 50},
    acks=[0, 15, 0], codes=[14, 2, 15, 10, 14, 2, 15, 10, 14], rises=2, faults=0, confirmed=2, gate=4)  # fmt: skip
vec("j_overspeed", "(j) one ESC runs away to 22,700 rpm -> FAULT overspeed 20 ms above 1.05 x cap, DShot 0, KICK stops", 1400,
    sc((1000, {FIRE: 1})), plant=dict(esc_runaway=[True, False], runaway_t=800),
    acks=[13], codes=[14, 15, 4], rises=0, faults=0x80, confirmed=0, kick_low_within=0)  # fmt: skip
vec("k_telemetry_loss", "(k) the eRPM replies stop -> FAULT telemetry_lost after 50 ms, wheels 0, KICK stops", 1300,
    sc((1000, {FIRE: 1})), plant=dict(telemetry_dead_t=800),
    acks=[13], codes=[14, 15, 4], rises=0, faults=0x40, confirmed=0, kick_low_within=0)  # fmt: skip
vec("l_estop_pre_commit", "(l) E-stop 20 ms into the stroke -> FEED_ABORT rail, nothing leaves; released later the gate re-homes and the ball is staged again", 2400,
    sc((800, {FIRE: 1}), (820, {"estop_open": True}), (1000, {"estop_open": False})),
    acks=[0], codes=[14, 11, 14], rises=1, faults=0, confirmed=0, gate=4)  # fmt: skip
vec("l2_estop_post_commit", "(l2) E-stop 50 ms into the stroke (past the commit): the ball in the nip still leaves", 1300,
    sc((800, {FIRE: 1}), (850, {"estop_open": True})),
    acks=[0], codes=[14, 2, 10], rises=1, faults=0, confirmed=1)  # fmt: skip
vec("m_watchdog_trip", "(m) heartbeat lost 15 ms into the stroke (the 300 ms watchdog -> SAFE): abort; FIRE afterwards is INHIBIT", 1300,
    sc((800, {FIRE: 1}), (815, {"hb_ok": False, "speed_req_pm": 0}), (1000, {FIRE: 2})),
    acks=[0, 3], codes=[14, 11], rises=1, faults=0, confirmed=0)  # fmt: skip
vec("n_feed_req_stuck", "(n) FEED_REQ stuck high in the limiter (second edge ignored): the rod never leaves home -> FAULT pusher_stuck at 40 ms, no commit", 1700,
    sc((800, {FIRE: 1}), (1300, {FIRE: 2})), plant=dict(feed_req_stuck="high", stuck_t=1100),
    acks=[0, 0], codes=[14, 2, 15, 10, 14, 4, 13], rises=2, faults=0x1000, confirmed=1, kick_low_within=0)  # fmt: skip
vec("o_empty_hopper", "(o) empty hopper: 3 failed loads -> EVENT EMPTY once, FIRE -> NO_BALL; refilled, the next retry stages a ball and FIRE works", 4200,
    sc((1500, {FIRE: 1}), (2000, {"refill": True}), (3600, {FIRE: 2})), plant=dict(hopper=0),
    acks=[12, 0], codes=[12, 14, 2, 15, 10, 14], rises=1, faults=0, confirmed=1)  # fmt: skip

# ---- extras (the rest of fault table 1.3 that the feeder sees, and one case per gate for the negative controls) ---------------------------------------
vec("p_pusher_stuck_extended", "(row 6) pusher servo stuck extended after the stroke -> FAULT pusher_stuck at 300 ms", 1500, sc((800, {FIRE: 1})),
    plant=dict(pusher="stuck_ext"),
    acks=[0], codes=[14, 2, 15, 10, 14, 4, 13], rises=1, faults=0x1000, confirmed=1)  # fmt: skip
vec("q_breech_double", "(row 11) the breech stays blocked 20 ms too long after the push (2-ball pocket) -> FAULT breech_double", 1300, sc((800, {FIRE: 1})),
    plant=dict(breech_delay_ms=20),
    acks=[0], codes=[14, 2, 15, 10, 4, 13], rises=1, faults=0x200, confirmed=1)  # fmt: skip
vec("r_gate_stuck_bore", "(row 9) gate stuck at the bore -> LOAD hall never seen -> FAULT gate_stuck", 1700, sc((800, {FIRE: 1})),
    plant=dict(gate="stuck_bore", stuck_t=700),
    acks=[0], codes=[14, 2, 15, 10, 14, 4, 13], rises=1, faults=0x2000, confirmed=1)  # fmt: skip
vec("s_wheel_not_ready", "(row 16) one ESC fails to spin -> FIRE NOT_SPUN, FAULT wheel_not_ready after 1.5 s", 2300, sc((1000, {FIRE: 1})),
    plant=dict(esc_dead=[True, False]),
    acks=[10], codes=[4], rises=0, faults=0x100, confirmed=0, kick_low_within=0)  # fmt: skip
vec("t_inhibit_abort", "(row 24) the app sets FIRE_INHIBIT 25 ms into the stroke -> abort before commit", 1300,
    sc((800, {FIRE: 1}), (825, {"inhibit": True}), (900, {"inhibit": False})),
    acks=[0], codes=[14, 11], rises=1, faults=0, confirmed=0, gate=4)  # fmt: skip
vec("u_wheels_abort", "the speed request drops 10 ms into the stroke -> wheels out of window -> abort", 1300,
    sc((800, {FIRE: 1}), (810, {"speed_req_pm": 0})),
    acks=[0], codes=[14, 11], rises=1, faults=0, confirmed=0, gate=4)  # fmt: skip
vec("v_gates_interlock_range", "FIRE with the E-stop open -> INTERLOCK; out of the sector -> RANGE; with FIRE_INHIBIT -> INHIBIT", 1300,
    sc((800, {FIRE: 1, "estop_open": True}), (820, {"estop_open": False}), (1100, {FIRE: 2, "in_sector": False}), (1120, {"in_sector": True}),
       (1200, {FIRE: 3, "inhibit": True}), (1220, {"inhibit": False})),
    acks=[2, 5, 3], codes=[14], rises=0, faults=0, confirmed=0)  # fmt: skip
vec("w_breech_early_clear", "the staged ball vanishes 10 ms into the stroke (breech clears early) -> abort (beam)", 1300, sc((800, {FIRE: 1})),
    plant=dict(breech_stuck=0, stuck_t=810),
    acks=[0], codes=[14, 11], rises=1, faults=0, confirmed=0)  # fmt: skip
vec("x_second_exit", "a ghost muzzle pulse 100 ms after a good exit -> FAULT unexpected_exit (+ shot_overcount), KICK stops", 1400, sc((800, {FIRE: 1})),
    plant=dict(spurious_muzzle=[953]),
    acks=[0], codes=[14, 2, 15, 10, 4, 4, 13], rises=1, faults=0x4800, confirmed=2, kick_low_within=0)  # fmt: skip
vec("y_diff_override", "FLYWHEEL differential +30 permille: the bottom wheel runs faster; one shot at the window", 1700, sc((800, {FIRE: 1}), (1, {"diff_pm": 30})),
    acks=[0], codes=[14, 2, 15, 10, 14], rises=1, faults=0, confirmed=1, gate=4)  # fmt: skip
vec("z_pusher_dead", "(row 7) pusher dead: the rod never leaves home -> FAULT pusher_stuck at 40 ms", 1300, sc((800, {FIRE: 1})), plant=dict(pusher="dead"),
    acks=[0], codes=[14, 4, 13], rises=1, faults=0x1000, confirmed=0, kick_low_within=0)  # fmt: skip


def refill_hook(script, plant):
    pass


def run_vec(name, stepper=None):
    v = V[name]
    plant = H.Plant(cfg=v["cfg"], **v["plant"])
    script = {t: dict(d) for t, d in v["script"].items()}
    refills = [t for t, d in script.items() if d.pop("refill", False)]
    st = stepper or H.FeederFsm()
    if refills:  # the operator tops up the hopper at a given tick: wrap the plant step
        orig = plant.step

        def step(out, t, rail=True):
            if t in refills:
                plant.hopper = 24
            return orig(out, t, rail)

        plant.step = step
    ins, outs = H.run_script(st, plant, script, v["n"], cfg=v["cfg"] or None)
    return ins, outs, plant


def fsm_vectors():
    out = dict(
        spec="docs/cuas2/spec.md sections 5, 6.6 (a)-(o) and 1.3; inputs are what the feeder saw, trace what the python twin produced",
        note="trace = change lists of the discrete outputs, events [t, code, arg], acks [t, result], and one CRC32 per 100 ticks of the continuous outputs",
        vectors=[],
    )
    for name, v in V.items():
        ins, outs, _ = run_vec(name)
        ch, cmds = H.inputs_to_changes(ins)
        out["vectors"].append(dict(name=name, desc=v["desc"], ticks=v["n"], cfg={str(k): x for k, x in v["cfg"].items()}, inputs=ch, cmds=cmds,
                                   trace=H.trace_of(outs), expect=v["expect"]))  # fmt: skip
    return out


# ---- protocol vectors ----------------------------------------------------------------------------------------------------------------------------
def entry(direction, name, seq, m):
    wire = p.encode(seq, m)
    return dict(
        name=name,
        dir=direction,
        id=m.ID,
        seq=seq,
        fields=m.asdict(),
        payload_hex=m.pack().hex(),
        frame_hex=wire.hex(),
    )


def protocol_vectors():
    st = p.Status2(123456, p.ENGAGING, 0xAF, 0x35, -4444, 3555, 868, 17500, 1234, 3, 5, 0x0840, 57, 0x0102, 17490, 17510, 17500, 0x13, p.GS_STAGED,
                   p.RDY_READY, 293, 52, 1001, 91)  # fmt: skip
    host = [
        ("flywheel_no_tail", 0, p.Flywheel(800)),
        ("flywheel_tail", 1, p.Flywheel(800, diff_pm=-25)),
        ("flywheel_off", 2, p.Flywheel(0)),
        ("fire_n1", 3, p.Fire(0x0100, 1)),
        ("fire_n0", 4, p.Fire(0x0101, 0)),
        ("fire_n2", 5, p.Fire(0x0102, 2)),
        ("cfg_set_wheel_d", 6, p.CfgSet(p.CFG_WHEEL_D_UM, 25000)),
        ("cfg_set_v_cap_381", 7, p.CfgSet(p.CFG_V_CAP_DMPS, 381)),
        ("cfg_set_wheel_d_zero", 8, p.CfgSet(p.CFG_WHEEL_D_UM, 0)),
        ("hello_1_1", 9, p.Hello(1, 1, 0x01020304)),
    ]
    mcu = [
        (
            "hello_ack_cuas2",
            0,
            p.HelloAck(1, 1, 0, 2, 0, 0, bytes.fromhex("e66050a8d3156f21"), 0xDEADBEEF),
        ),
        ("status2", 1, st),
        ("status2_default_tail", 2, p.Status2()),
        ("event_muzzle", 3, p.Event(p.EV_MUZZLE, 293 | 52 << 16)),
        ("event_feed_abort", 4, p.Event(p.EV_FEED_ABORT, 0x0100 << 8 | p.FA_SECTOR)),
        ("event_empty", 5, p.Event(p.EV_EMPTY, 0)),
        ("event_jam", 6, p.Event(p.EV_JAM, p.GS_EXIT_WAIT)),
        ("event_wheels_ready", 7, p.Event(p.EV_WHEELS_READY, 17500)),
        ("event_wheels_lost", 8, p.Event(p.EV_WHEELS_LOST, 16573)),
        ("event_hw_counter", 9, p.Event(p.EV_HW_COUNTER, 10)),
        ("ack_too_soon", 10, p.Ack(p.FIRE, 5, p.TOO_SOON, 0)),
        ("ack_not_clear", 11, p.Ack(p.FIRE, 6, p.NOT_CLEAR, 0)),
    ]
    legacy = lite.Status(
        123456,
        p.ENGAGING,
        0xAF,
        0x35,
        -4444,
        3555,
        800,
        0xFFFF,
        1234,
        3,
        5,
        0x0011,
        57,
        0x0102,
    )
    return dict(
        spec="docs/cuas2/spec.md section 6.6 items 1-7 (protocol 1.1)",
        crc16_check=dict(input_ascii="123456789", crc="0x29B1"),
        frames=[entry("host_to_mcu", *v) for v in host]
        + [entry("mcu_to_host", *v) for v in mcu],
        status_v1_payload_hex=legacy.pack().hex(),
        status_v1_len=len(legacy.pack()),
        status2_len=p.STATUS_LEN,
        # reference vectors NOT derived from the module: the bidirectional-DShot article's examples and the Betaflight GCR table (docs: brushlesswhoop.com,
        # betaflight dshot_bitbang_decode.c)
        dshot=dict(
            frame_throttle_1046_regular="0x%04x" % (1046 << 5 | 0x6),
            frame_throttle_1046_inverted_crc="0x%04x" % (1046 << 5 | 0x9),
            gcr_nibble_to_symbol=[
                "19",
                "1B",
                "12",
                "13",
                "1D",
                "15",
                "16",
                "17",
                "1A",
                "09",
                "0A",
                "0B",
                "1E",
                "0D",
                "0E",
                "0F",
            ],
            worst_case_gcr="10101010101010101010",
            worst_case_line="011001100110011001100",
        ),
        cfg_keys={str(k): v for k, v in sorted(p.CFG_DEFAULTS2.items())},
        cfg_ranges={str(k): list(v) for k, v in sorted(p.CFG_RANGES.items())},
    )


if __name__ == "__main__":
    which = sys.argv[1] if len(sys.argv) > 1 else "fsm"
    if which == "protocol":
        print(json.dumps(protocol_vectors(), indent=1))
    else:  # compact: the per-tick rpm inputs make this file big
        print(json.dumps(fsm_vectors(), separators=(",", ":")))
