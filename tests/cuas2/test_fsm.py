"""The python twin (napl.cuas2.fsm) against the hand-written expectations of every vector, the cycle time, the v_cap / rpm_cap rules, the hardware shot counter,
the drift check against lite's safety FSM, the single-fault safety matrix (fault table 1.3) and the 20k command fuzz, the last three on the whole C core.
Needs the host library (tools/cuas2_fw_host_test.sh builds it, or NAPL_CUAS2_FW=PATH)."""

import os
import random
import subprocess
from pathlib import Path

import pytest
from napl.cuas2 import protocol as p

from cuas2 import fwharness as H
from cuas2 import gen_vectors as G

ROOT = Path(__file__).resolve().parents[2]


@pytest.fixture(scope="module", autouse=True)
def need_lib():
    H.require_lib()


# ---- the twin against the expectations ----
@pytest.mark.parametrize("name", list(G.V))
def test_python_twin_meets_the_expectations(name):
    _, outs, _ = G.run_vec(name)
    assert not H.check_expect(name, G.V[name]["expect"], H.expect_view(outs))


def test_every_spec_vector_a_to_o_exists():
    names = {n.split("_")[0] for n in G.V}
    assert set("abcdefghijklmno") <= names and "l2" in names


def test_cycle_time_fire_to_restaged_under_400ms():
    _, outs, plant = G.run_vec("a_nominal_two_shots")
    ch = H.trace_of(outs)["changes"]["gate_state"]
    t_push = next(t for t, v in ch if v == p.GS_PUSH)
    t_back = next(t for t, v in ch if v == p.GS_STAGED and t > t_push)
    assert t_back - t_push <= 400, (t_push, t_back)
    assert len(plant.exits) == 2  # and the plant agrees two balls left


# ---- v_cap / rpm_cap ----
def test_rpm_cap_python_equals_c():
    L = H.lib()
    for d, v in (
        (40000, 380),
        (25000, 380),
        (25000, 300),
        (100000, 380),
        (10000, 1),
        (0, 380),
        (40000, 999),
    ):
        cfg = H.cfg_array({p.CFG_WHEEL_D_UM: d, p.CFG_V_CAP_DMPS: v})
        assert L.fwh_rpm_cap(cfg) == p.rpm_cap(d, v), (d, v)


def test_wheel_d_25000_rpm_cap_and_absolute_38ms_cap():
    cap = p.rpm_cap(25000, 380)
    assert cap == 29029  # spec "~29,000"
    assert (
        cap * 3.141592653589793 * 0.025 / 60 <= 38.0
    )  # surface speed at the cap never exceeds 38 m/s
    for d in range(10000, 100001, 5000):
        assert p.rpm_cap(d, 380) * 3.141592653589793 * d / 1e6 / 60 <= 38.0 + 1e-9
    assert p.rpm_cap(40000, 381) == p.rpm_cap(
        40000, 380
    )  # a stored 381 would fall back to the cap


@pytest.mark.parametrize("stepper", ["python", "c"])
def test_overspeed_above_1p05_cap_for_20ms_faults_within_25ms(stepper):
    st = H.FeederFsm() if stepper == "python" else H.CFeeder()
    ins, outs, _ = G.run_vec("j_overspeed", st)
    cap = outs[-1].status_fields["rpm_cap"]
    t_over = next(
        i + 1
        for i, d in enumerate(ins)
        if max(d["rpm_top"] or 0, d["rpm_bot"] or 0) * 100 > cap * 105
    )
    t_fault = next(
        i + 1 for i, o in enumerate(outs) if any(c == p.EV_FAULT for c, _ in o.events)
    )
    assert 20 <= t_fault - t_over + 1 <= 25, (t_over, t_fault)
    assert (
        outs[t_fault - 1].dshot_top == 0
        and outs[t_fault - 1].dshot_bot == 0
        and not outs[t_fault - 1].kick
    )


# ---- drift against lite's safety FSM ----
def drift(env=None):
    return subprocess.run(
        [str(ROOT / "tools" / "cuas2_fsm_drift.sh")],
        capture_output=True,
        text=True,
        env={**os.environ, **(env or {})},
    )


def test_shared_safety_functions_have_not_drifted():
    r = drift()
    assert r.returncode == 0, r.stdout
    assert "0 drifted" in r.stdout


@pytest.mark.parametrize(
    "func,old,new",
    [
        ("watchdog", "CFG_WD_MS", "CFG_WD_MS + 1"),
        ("cmd_arm", "mode != 1 ||", "mode != 1 &&"),
        ("cmd_heartbeat", "hb_run", "hb_run + 0"),
    ],
)
def test_drift_check_catches_a_changed_shared_function(tmp_path, func, old, new):
    src = (ROOT / "firmware" / "cuas2-rp2040" / "src" / "fsm.c").read_text()
    i = src.index(f" {func}(")
    j = src.index(old, i)
    mut = tmp_path / "fsm.c"
    mut.write_text(src[:j] + new + src[j + len(old) :])
    r = drift({"CUAS2_FSM": str(mut)})
    assert r.returncode == 1 and f"DIFFERS {func}" in r.stdout, r.stdout


# ---- hardware shot counter (fault row 30) ----
def shoot(r, n, fid0=1):
    for k in range(n):
        r.send(p.Fire(fid0 + k, 1))
        r.run(600)


def test_hw_counter_trip_opens_the_estop_chain_and_is_reported():
    """CD4017 on the muzzle beam opens the relay after N balls: modelled as the E-stop chain opening at the N-th exit. The MCU sees the E-stop edge with the key
    on and N balls counted: SAFE + EVENT 16, wheels off."""
    r = H.CoreRig(cfg={p.CFG_HW_COUNTER_N: 3}).up().engage()
    assert r.L.fwh_cfg(r.h, p.CFG_HW_COUNTER_N, 0) == 3
    shoot(r, 3)
    assert r.st.shots_confirmed == 3 and len(r.plant.exits) == 3
    assert [
        e for e in r.events if e[1] == p.EV_HW_COUNTER
    ] == []  # nothing yet: the relay has not opened
    r.estop_open = True  # the counter's relay contact
    r.run(50)
    ev = [e for e in r.events if e[1] == p.EV_HW_COUNTER]
    assert len(ev) == 1 and ev[0][2] == 3
    assert r.st.state == p.SAFE and r.o["dshot_top"] == 0 and not r.rail
    r.send(p.Fire(9, 1))
    r.run(20)
    assert r.last_ack(p.FIRE).result != p.OK


def test_hw_counter_negative_controls_plain_estop_is_not_labelled():
    r = H.CoreRig(cfg={p.CFG_HW_COUNTER_N: 3}).up().engage()
    shoot(r, 2)
    r.estop_open = True  # an operator E-stop with 2 of 3 balls: no EVENT 16
    r.run(50)
    assert not [e for e in r.events if e[1] == p.EV_HW_COUNTER] and r.st.state == p.SAFE
    r = H.CoreRig(cfg={p.CFG_HW_COUNTER_N: 0}).up().engage()  # N = 0: counter disabled
    shoot(r, 3)
    r.estop_open = True
    r.run(50)
    assert not [e for e in r.events if e[1] == p.EV_HW_COUNTER]


# ---- safety matrix: each single fault of spec 1.3 that software can see ----
def inject(r, **f):
    r.plant.f.update(f, stuck_t=r.t + 1)


def fire_then(r, gaps=(700,)):
    r.fire(1)
    for k, g in enumerate(gaps):
        r.run(g)
        if k + 1 < len(gaps):
            r.fire(2 + k)


def sec(r, fn):
    fn(r)
    return r


# row -> (builder, injector/driver, expectations)
#   fault: fault-bit mask that must be latched (None: no fault); ack: result of the last FIRE (None: not checked); state: final state;
#   kick_off: KICK must be stopped at the end; rises: FEED_REQ rising edges in total; exits: balls that left the barrel (at most the accepted FIRE).
ROWS = {}


def row(n, name, build, drive, **expect):
    ROWS[f"{n:02d}_{name}"] = (build, drive, expect)


def eng(**kw):
    return lambda: H.CoreRig(**kw).up().engage()


def drive_fire(**f):
    def d(r):
        inject(r, **f)
        r.run(30)  # sensors show the fault before the operator fires
        fire_then(r, (800,))

    return d


def drive_fire_twice(**f):
    def d(r):
        inject(r, **f)
        r.run(30)
        fire_then(r, (500, 800))

    return d


M = p.FDR_FAULT_MASK
row(
    1,
    "link_lost",
    eng(),
    lambda r: (setattr(r, "hb", False), r.run(500)),
    fault=None,
    state=p.SAFE,
    rises=0,
    exits=0,
    wheels_off=True,
)
row(
    4,
    "feed_req_stuck_high",
    eng(),
    drive_fire_twice(feed_req_stuck="high"),
    fault=p.FAULT_PUSHER_STUCK,
    kick_off=True,
    rises=2,
    exits=1,
)
row(
    5,
    "feed_req_stuck_low",
    eng(),
    drive_fire(feed_req_stuck="low"),
    fault=p.FAULT_PUSHER_STUCK,
    kick_off=True,
    rises=1,
    exits=0,
)
row(
    6,
    "pusher_stuck_extended",
    eng(),
    drive_fire(pusher="stuck_ext"),
    fault=p.FAULT_PUSHER_STUCK,
    kick_off=True,
    rises=1,
    exits=1,
)
row(
    7,
    "pusher_dead",
    eng(),
    drive_fire(pusher="dead"),
    fault=p.FAULT_PUSHER_STUCK,
    kick_off=True,
    rises=1,
    exits=0,
)
row(
    9,
    "gate_stuck_bore",
    eng(),
    drive_fire_twice(gate="stuck_bore"),
    fault=p.FAULT_GATE_STUCK,
    kick_off=True,
    rises=1,
    exits=1,
)
row(
    10,
    "gate_stuck_load",
    eng(),
    drive_fire(gate="stuck_load"),
    ack=p.NO_BALL,
    rises=0,
    exits=0,
)
row(
    11,
    "two_balls_in_pocket",
    eng(),
    drive_fire(breech_delay_ms=20),
    fault=p.FAULT_BREECH_DOUBLE,
    kick_off=True,
    rises=1,
    exits=1,
)
row(
    12,
    "breech_stuck_clear",
    eng(),
    drive_fire(breech_stuck=0),
    ack=12,
    rises=0,
    exits=0,
)
row(
    13,
    "breech_stuck_blocked",
    eng(),
    drive_fire(breech_stuck=1),
    fault=p.FAULT_NO_EXIT,
    kick_off=True,
    rises=1,
    exits=1,
)
row(
    14,
    "muzzle_stuck_clear",
    eng(),
    drive_fire(muzzle_stuck=0),
    fault=p.FAULT_NO_EXIT,
    kick_off=True,
    rises=1,
    exits=1,
)
row(
    15,
    "muzzle_stuck_blocked",
    lambda: (
        H.CoreRig(muzzle_stuck=1).up().engage()
    ),  # blocked from power-up (a mid-run blocking is an unexpected exit, vector h)
    drive_fire(muzzle_stuck=1),
    ack=p.NOT_CLEAR,
    rises=0,
    exits=0,
)
row(
    16,
    "esc_not_spinning",
    eng(),
    lambda r: (inject(r, esc_dead=[True, False]), r.run(300), fire_then(r, (1500,))),
    ack=p.NOT_SPUN,
    fault=p.FAULT_WHEEL_NOT_READY,
    kick_off=True,
    rises=0,
    exits=0,
)
row(
    17,
    "esc_runaway",
    eng(),
    lambda r: (inject(r, esc_runaway=[True, False], runaway_t=r.t + 10), r.run(200)),
    fault=p.FAULT_OVERSPEED,
    kick_off=True,
    rises=0,
    exits=0,
    wheels_off=True,
)
row(
    18,
    "telemetry_lost",
    eng(),
    lambda r: (inject(r, telemetry_dead_t=r.t + 10), r.run(300)),
    fault=p.FAULT_TELEMETRY_LOST,
    kick_off=True,
    rises=0,
    exits=0,
    wheels_off=True,
)
row(
    20,
    "limiter_shorted_interval_still_enforced",
    eng(),
    lambda r: (
        inject(r, limiter_shorted=True),
        r.fire(1),
        r.run(390),  # the cycle is done (364 ms) but the 450 ms interval is not
        r.fire(2),
        r.run(600),
    ),
    ack=p.TOO_SOON,
    rises=1,
    exits=1,
)
row(
    23,
    "estop_pressed",
    eng(),
    lambda r: (setattr(r, "estop_open", True), r.run(100), r.fire(1), r.run(100)),
    state=p.SAFE,
    ack=p.BAD_STATE,  # SAFE: not ENGAGING (the feeder's own INTERLOCK gate is vector v)
    rises=0,
    exits=0,
    wheels_off=True,
)
row(
    23,
    "key_off",
    eng(),
    lambda r: (setattr(r, "key_on", False), r.run(100), r.fire(1), r.run(100)),
    state=p.SAFE,
    ack=p.BAD_STATE,
    rises=0,
    exits=0,
    wheels_off=True,
)
row(
    24,
    "fire_inhibit",
    eng(),
    lambda r: (setattr(r, "inhibit", True), r.run(60), r.fire(1), r.run(300)),
    ack=p.INHIBIT,
    rises=0,
    exits=0,
)
row(
    25,
    "outside_fire_sector",
    eng(),
    lambda r: (r.send(p.Aim(0, -10000)), r.run(1500), r.fire(1), r.run(100)),
    ack=p.RANGE,
    rises=0,
    exits=0,
)
row(
    26,
    "stepper_diag",
    eng(),
    lambda r: (setattr(r, "diag", 1), r.run(80), setattr(r, "diag", 0), r.run(100)),
    fault=p.FAULT_DRIVER_DIAG,
    wheels_off=True,
    rises=0,
    exits=0,
)
row(
    28,
    "hopper_empty",
    lambda: H.CoreRig(hopper=0).up().engage(spin_ms=3000),
    lambda r: (r.fire(1), r.run(300)),
    ack=12,
    rises=0,
    exits=0,
    event=p.EV_EMPTY,
)


@pytest.mark.parametrize("rowid", sorted(ROWS))
def test_safety_matrix(rowid):
    build, drive, ex = ROWS[rowid]
    r = build()
    base_rises = len(r.feed_rises)
    base_t = r.t
    drive(r)
    rises = [t for t in r.feed_rises[base_rises:]]
    if "ack" in ex:
        assert r.last_ack(p.FIRE).result == ex["ack"], r.last_ack(p.FIRE)
    if "fault" in ex:
        if ex["fault"] is None:
            assert r.snap["fault_bits"] == 0
        else:
            assert r.snap["fault_bits"] & ex["fault"], hex(r.snap["fault_bits"])
    if "state" in ex:
        assert r.snap["state"] == ex["state"], p.STATE_NAMES[r.snap["state"]]
    if ex.get("kick_off"):
        assert r.o["kick_ok"] == 0, "KICK still running after the fault"
        t_stop = next(t for t, v in r.kick_log if v == 0 and t >= r.fault_t)
        assert t_stop - r.fault_t <= 5
    if ex.get("wheels_off"):
        assert r.o["dshot_top"] == 0 and r.o["dshot_bot"] == 0
    if "event" in ex:
        assert any(c == ex["event"] for _, c, _ in r.events)
    # no FEED_REQ after the fault except the allowed first (the rise that started the faulted cycle is before it)
    if r.fault_t is not None and r.fault_t > base_t:
        assert not [t for t in rises if t > r.fault_t], (r.fault_t, rises)
    assert len(rises) == ex["rises"], rises
    assert len(r.plant.exits) == ex["exits"], r.plant.exits
    assert (
        len(r.plant.exits) <= r.accepted_fires
    )  # balls out never exceed accepted FIREs


def test_matrix_covers_the_software_observable_rows():
    rows = {int(k.split("_")[0]) for k in ROWS}
    assert rows == {
        1,
        4,
        5,
        6,
        7,
        9,
        10,
        11,
        12,
        13,
        14,
        15,
        16,
        17,
        18,
        20,
        23,
        24,
        25,
        26,
        28,
    }
    # rows 2, 3, 8, 19, 21, 22, 29 are hardware / physical (hang, firmware bug, servo runaway, DShot noise, supervisor, welded relay, tolerances): not software
    # observable here; row 3 is covered by the fuzz and the mutation controls, row 27 below, row 30 above.


def test_row_27_corrupt_flash_config_defaults_and_arm_refused():
    r = H.CoreRig(flash_mode=2)
    r.send(p.Hello(p.MAJOR, p.MINOR, 7))
    r.run(1400)
    assert r.snap["fault_bits"] & p.FAULT_CFG_INVALID
    assert (
        r.L.fwh_cfg(r.h, p.CFG_V_CAP_DMPS, 0) == 380
        and r.L.fwh_cfg(r.h, p.CFG_WHEEL_D_UM, 0) == 40000
    )
    r.send(p.Arm(1, 5, 60))
    r.run(20)
    assert r.last_ack(p.ARM).result != p.OK and r.feed_rises == []


# ---- the fuzz: 20k random command sequences on the whole C core ----
FUZZ_CASES, FUZZ_CHUNKS, FUZZ_TICKS = 20000, 8, 700
_base = {}


def fuzz_base():
    if "r" not in _base:
        _base["r"] = H.CoreRig().up().engage()
    return _base["r"]


def fuzz_case(seed, plant_faults=False):
    rng = random.Random(seed)
    r = fuzz_base().fork()
    if plant_faults and rng.random() < 0.8:
        kind = rng.choice(
            ["pusher", "gate", "breech", "muzzle", "exit", "req", "limiter", "delay"]
        )
        st = r.t + rng.randrange(0, 400)
        f = dict(stuck_t=st)
        if kind == "pusher":
            f["pusher"] = rng.choice(["dead", "stuck_ext", "stuck_home"])
        elif kind == "gate":
            f["gate"] = rng.choice(["stuck_load", "stuck_bore"])
        elif kind == "breech":
            f["breech_stuck"] = rng.choice([0, 1])
        elif kind == "muzzle":
            f["muzzle_stuck"] = rng.choice([0, 1])
        elif kind == "exit":
            f["exit_blocked"] = True
        elif kind == "req":
            f["feed_req_stuck"] = rng.choice(["low", "high"])
        elif kind == "limiter":
            f["limiter_shorted"] = True
        else:
            f["breech_delay_ms"] = rng.randrange(1, 60)
        r.plant.f.update(f)
    fid = 100
    t0 = r.t
    sched = {}
    for _ in range(rng.randrange(6, 28)):
        sched.setdefault(rng.randrange(1, FUZZ_TICKS), []).append(rng.choice(
            ["fire"] * 8 + ["fire_dup", "fire_bad_n", "estop", "key", "hb", "deadman", "inhibit", "fw", "clear", "arm", "aim", "cfg", "junk", "home"]))  # fmt: skip
    for k in range(FUZZ_TICKS):
        for a in sched.get(k, ()):
            if a == "fire":
                fid += 1
                r.send(p.Fire(fid, 1))
            elif a == "fire_dup":
                r.send(p.Fire(fid, 1))
            elif a == "fire_bad_n":
                fid += 1
                r.send(p.Fire(fid, rng.choice([0, 2, 3, 255])))
            elif a == "estop":
                r.estop_open = not r.estop_open
            elif a == "key":
                r.key_on = not r.key_on
            elif a == "hb":
                r.hb = not r.hb
            elif a == "deadman":
                r.deadman = not r.deadman
            elif a == "inhibit":
                r.inhibit = not r.inhibit
            elif a == "fw":
                r.fw_pm = rng.choice([None, 0, 500, 1000, 1000])
            elif a == "clear":
                r.send(p.ClearFault())
            elif a == "arm":
                r.send(
                    p.Arm(
                        rng.choice([0, 1, 1]),
                        rng.choice([1, 3, 10, 200]),
                        rng.choice([5, 60]),
                    )
                )
            elif a == "aim":
                r.send(
                    p.Aim(rng.randrange(-60000, 60000), rng.randrange(-20000, 80000))
                )
            elif a == "cfg":
                key = rng.choice(
                    [
                        p.CFG_MIN_SHOT_INTERVAL_MS,
                        p.CFG_MIN_SHOT_INTERVAL_MS,
                        p.CFG_PUSHER_DWELL_MS,
                        p.CFG_EXIT_TIMEOUT_MS,
                        p.CFG_FILL_MS,
                        p.CFG_RPM_WINDOW_PM,
                        p.CFG_SETTLE_MS,
                    ]
                )
                r.send(p.CfgSet(key, rng.choice([0, 1, 50, 100, 300, 450, 1000, -5])))
            elif a == "junk":
                r.send_raw(
                    bytes(rng.randrange(256) for _ in range(rng.randrange(1, 12)))
                    + b"\0"
                )
            else:
                r.send(p.Home(3))
        r.tick()
    return r, t0


def check_invariants(seed, r, plant_faults):
    gaps = [b - a for a, b in zip(r.feed_rises, r.feed_rises[1:])]
    assert all(g >= 400 for g in gaps), (seed, "two feeds < 400 ms apart", r.feed_rises)
    assert len(r.feed_rises) == r.accepted_fires, (seed, r.feed_rises, r.accepted_fires)
    assert len(r.plant.exits) <= r.accepted_fires, (
        seed,
        "more balls out than accepted FIREs",
    )
    if not plant_faults:  # with a ghost-pulse fault shots_confirmed counts the ghost by design (it trips shot_overcount)
        assert r.st.shots_confirmed <= r.accepted_fires, (
            seed,
            r.st.shots_confirmed,
            r.accepted_fires,
        )
    assert r.snap["feeds"] >= r.snap["muz_all"] or r.snap["fault_bits"], (
        seed,
        "more exits than feeds without a fault",
    )


@pytest.mark.parametrize("chunk", range(FUZZ_CHUNKS))
def test_fuzz_20k_random_command_sequences(chunk):
    n = FUZZ_CASES // FUZZ_CHUNKS
    for seed in range(chunk * n, (chunk + 1) * n):
        r, _ = fuzz_case(seed)
        check_invariants(seed, r, False)


@pytest.mark.parametrize("chunk", range(4))
def test_fuzz_with_plant_faults_2k(chunk):
    for seed in range(10**6 + chunk * 500, 10**6 + (chunk + 1) * 500):
        r, _ = fuzz_case(seed, plant_faults=True)
        check_invariants(seed, r, True)


def test_fuzz_actually_feeds_balls():
    """The fuzz is not vacuous: a good share of the sequences get a ball out."""
    out = sum(len(fuzz_case(s)[0].plant.exits) > 0 for s in range(200))
    assert out >= 40, out
