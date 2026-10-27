"""Sim2RP2040 (python/napl/cuas2/sim_mcu.py) against the C whole core: the same host script, the same weapon plant, the same E-stop / key schedule go into
both, and every ACK, EVENT and STATUS word that leaves them must match tick for tick. Scripted scenarios (every FIRE gate, the abort rules, the faults of the
feeder, the hardware-counter branches, config ranges) plus a fuzz sample."""

import copy
from collections import deque

import pytest
from napl.cuas2 import protocol as p
from napl.cuas2.sim_mcu import Sim2RP2040
from napl.lite.sim_mcu import KinematicPlant

from cuas2 import fwharness as H
from cuas2 import test_fsm as TF

# STATUS words compared: everything the weapon and the safety FSM own. Axis positions and the homed / hall bits depend on the stepper plant (the C rig integrates
# steps ideally, the lite KinematicPlant lags 2 ms) and are compared only as "homed" in up().
F1_MASK = p.F1_ESTOP | p.F1_KEY | p.F1_WD_TRIPPED | p.F1_HB_OK
F2_MASK = p.F2_PUSHER_EXT | p.F2_INHIBIT | p.F2_DEADMAN


def status_word(s):
    return (s.state, s.flags1 & F1_MASK, s.flags2 & F2_MASK, s.fw_duty, s.fw_rpm, s.shots_total, s.shots_arm, s.shot_cap, s.fault_bits, s.last_fire_id,
            s.rpm_top, s.rpm_bot, s.rpm_set, s.beams, s.gate_state, s.ready_flags, s.last_v_dmps, s.last_dip_pm, s.shots_confirmed, s.feed_ms)  # fmt: skip


class ListLink:
    def __init__(self):
        self.rx, self.tx = bytearray(), bytearray()

    def mcu_read(self, t):
        d = bytes(self.rx)
        self.rx.clear()
        return d

    def mcu_write(self, data, t):
        self.tx += data


class SimRig:
    """H.CoreRig's host side and board, with Sim2RP2040 instead of the C core (the same methods: up / arm / engage / send / run / fire ...)."""

    send, run, fire, last_ack, up, arm, engage = (
        H.CoreRig.send,
        H.CoreRig.run,
        H.CoreRig.fire,
        H.CoreRig.last_ack,
        H.CoreRig.up,
        H.CoreRig.arm,
        H.CoreRig.engage,
    )
    accepted_fires = H.CoreRig.accepted_fires
    send_raw = H.CoreRig.send_raw

    def __init__(self, cfg=None, **plant_faults):
        self.kin = KinematicPlant()
        self.kin.estop_open, self.kin.key_on = False, True
        self.link = ListLink()
        self.plant = H.Plant(cfg=cfg, **plant_faults)
        self.sim = Sim2RP2040(
            self.kin, self.link, home_off_deg=(-1.0, -1.0), weapon=self.plant
        )
        for k, v in (cfg or {}).items():
            self.sim.cfg[k] = v
        self.sim.step(0.0)
        self.t, self.seq, self.hb_seq = 0, 0, 0
        self.dec = p.Decoder2()
        self.rx = bytearray(b"\0")
        self.hb, self.deadman, self.inhibit = True, True, False
        self.fw_pm = self.fw_diff = None
        self.estop_open, self.key_on = False, True
        self.st = None
        self.events, self.acks, self.status_log = [], [], []
        self.accepted_ids, self.fire_by_seq = set(), {}
        self.feed_rises, self._prev_req = [], 0
        self.log_status = True

    def fork(self):
        return copy.deepcopy(self)

    @property
    def rail(self):
        return self.sim.rail_hw

    def tick(self):
        self.t += 1
        t = self.t
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
        self.kin.estop_open, self.kin.key_on = self.estop_open, self.key_on
        self.link.rx += bytes(self.rx)
        self.rx = bytearray()
        self.sim.step(t * 0.001)
        data, self.link.tx = bytes(self.link.tx), bytearray()
        for f in self.dec.feed(data):
            m = f.msg
            if isinstance(m, p.Status2):
                self.st = m
                self.status_log.append((t, m))
            elif isinstance(m, p.Event):
                self.events.append((t, m.code, m.arg))
            elif isinstance(m, p.Ack):
                self.acks.append((t, m))
                if m.cmd_id == p.FIRE and m.result == p.OK:
                    self.accepted_ids.add(self.fire_by_seq.get(m.cmd_seq))
        req = self.sim.fo.feed_req
        if req and not self._prev_req:
            self.feed_rises.append(t)
        self._prev_req = req


def pair(**kw):
    c = H.CoreRig(**kw)
    c.log_status = True
    return c, SimRig(**kw)


def both(rigs, fn):
    for r in rigs:
        fn(r)


def compare(c, s, since=0):
    """Everything logged after the `since` ticks must match."""
    ea = [e for e in c.events if e[0] > since]
    eb = [e for e in s.events if e[0] > since]
    assert ea == eb, ("events", ea[:12], eb[:12])
    aa = [(t, a.cmd_id, a.result, a.info) for t, a in c.acks if t > since]
    ab = [(t, a.cmd_id, a.result, a.info) for t, a in s.acks if t > since]
    assert aa == ab, ("acks", aa[:12], ab[:12])
    sa = {t: status_word(m) for t, m in c.status_log if t > since}
    sb = {t: status_word(m) for t, m in s.status_log if t > since}
    common = sorted(set(sa) & set(sb))
    assert len(common) >= 0.8 * min(len(sa), len(sb)), (len(sa), len(sb), len(common))
    for t in common:
        assert sa[t] == sb[t], ("status", t, sa[t], sb[t])
    assert c.feed_rises == s.feed_rises
    assert len(c.plant.exits) == len(s.plant.exits)
    assert c.rail == s.rail


def up_pair(**kw):
    c, s = pair(**kw)
    both((c, s), lambda r: r.up())
    assert c.st.state == s.st.state == p.IDLE
    assert (
        c.st.flag1(p.F1_PAN_HOMED)
        and s.st.flag1(p.F1_PAN_HOMED)
        and c.st.flag1(p.F1_TILT_HOMED)
        and s.st.flag1(p.F1_TILT_HOMED)
    )
    return c, s


def eng_pair(**kw):
    c, s = up_pair(**kw)
    both((c, s), lambda r: r.engage())
    return c, s


# ---- scripted scenarios ----
def nominal(r):
    r.fire(1)
    r.run(700)
    r.fire(2)
    r.run(700)


def gates(r):
    r.fire(1)  # TOO_SOON-free first shot
    r.run(10)
    r.fire(2)  # BUSY
    r.run(700)
    r.send(p.Fire(3, 2))  # BAD_ARG
    r.send(p.Fire(1, 1))  # a retransmission of fire 1: the old answer
    r.run(20)
    r.inhibit = True
    r.run(60)
    r.fire(4)  # INHIBIT
    r.run(20)
    r.inhibit = False
    r.send(p.Aim(0, -10000))  # tilt below the fire sector
    r.run(900)
    r.fire(5)  # RANGE
    r.run(20)
    r.send(p.Aim(0, 5000))
    r.run(900)
    r.estop_open = True
    r.run(30)
    r.fire(6)  # not ENGAGING any more
    r.run(30)
    r.estop_open = False
    r.run(1200)


def cap(r):
    r.send(p.Arm(0, 0, 0))
    r.run(30)
    r.send(p.Arm(1, 2, 60))
    r.run(30)
    r.fw_pm = 1000
    r.run(1500)
    for i in range(3):
        r.fire(10 + i)
        r.run(700)


def deadman_stroke(r):
    for ms in (30, 80, 120):
        r.fire(20 + ms)
        r.run(ms)
        r.send(p.Heartbeat(999, 0))
        r.run(800)
        r.run(1200)


def estop_stroke(r):
    r.fire(1)
    r.run(90)
    r.estop_open = True
    r.run(300)
    r.estop_open = False
    r.run(1500)


def key_cycle(r):
    shoot = TF.shoot
    shoot(r, 2)
    r.key_on = False
    r.run(100)
    r.key_on = True
    r.run(1500)


def hb_loss(r):
    r.fire(1)
    r.run(80)
    r.hb = False
    r.run(600)
    r.hb = True
    r.run(1500)


def flywheel_diff(r):
    r.fw_diff = 30
    r.run(500)
    r.fire(1)
    r.run(900)
    r.fw_pm = 400
    r.run(1000)
    r.fw_pm = 0
    r.run(300)


def clear_flow(r):
    r.fire(1)
    r.run(900)
    r.send(p.ClearFault())
    r.run(50)
    r.send(p.Arm(1, 5, 60))
    r.run(50)


def cfg_ranges(r):
    for key, v in (
        (p.CFG_RPM_WINDOW_PM, 21),
        (p.CFG_RPM_WINDOW_PM, 20),
        (p.CFG_SETTLE_MS, 19),
        (p.CFG_PUSHER_DWELL_MS, 131),
        (p.CFG_PUSHER_DWELL_MS, 100),
        (p.CFG_MIN_SHOT_INTERVAL_MS, 100),
        (p.CFG_HW_COUNTER_N, 4),
        (p.CFG_V_CAP_DMPS, 381),
        (p.CFG_WD_MS, 900),
        (p.CFG_EXIT_TIMEOUT_MS, 99),
        (p.CFG_K_BEAM, 3001),
        (99, 1),
        (0, 1),
    ):
        r.send(p.CfgSet(key, v))
        r.send(p.CfgGet(key))
        r.run(5)
    r.send(p.Arm(0, 0, 0))
    r.run(10)


def hw_trip(n_balls):
    def f(r):
        TF.shoot(r, n_balls)
        TF.trip(r)
        r.run(300)

    return f


def arm_gates(r):
    """ARM refused with the dead-man low (INHIBIT), then accepted with a cap above the hard cap (clamped), then DISARM."""
    r.send(p.Arm(0, 0, 0))
    r.run(30)
    r.deadman = False
    r.run(55)
    r.send(p.Arm(1, 3, 60))  # INHIBIT
    r.run(20)
    r.deadman = True
    r.run(55)
    r.send(p.Arm(1, 250, 60))  # OK, the cap is the hard cap
    r.run(150)
    r.send(p.Arm(0, 0, 0))
    r.run(30)


def sector_slew(r):
    """A FIRE while the barrel slews into the last 1.9 deg (pan, 240 deg/s) / 1.2 deg (tilt, 150 deg/s) of the fire sector: refused (spec 5.4 shrink)."""
    r.send(p.Arm(0, 0, 0))
    r.run(30)
    r.send(p.CfgSet(p.SECTOR_KEYS[2], 100000))
    r.send(p.CfgSet(p.SECTOR_KEYS[3], 40000))
    r.run(10)
    r.engage()
    r.send(p.Aim(150000, 5000))
    r.run(470)
    r.fire(1)  # pan 98.5 deg at 240 deg/s: RANGE
    r.run(100)
    r.send(p.Aim(0, 5000))
    r.run(2500)
    r.send(p.Aim(0, 60000))
    r.run(276)
    r.fire(2)  # tilt 38.9 deg at 150 deg/s: RANGE
    r.run(100)
    r.send(p.Aim(98500, 5000))
    r.run(2500)
    r.fire(3)  # parked 1.5 deg inside the end: accepted
    r.run(700)


SCENARIOS = {
    "nominal": (nominal, {}),
    "arm_gates": (arm_gates, {}),
    "sector_slew": (sector_slew, {}),
    "gates": (gates, {}),
    "cap": (cap, {}),
    "deadman_stroke": (deadman_stroke, {}),
    "estop_stroke": (estop_stroke, {}),
    "key_cycle": (key_cycle, {}),
    "hb_loss": (hb_loss, {}),
    "flywheel_diff": (flywheel_diff, {}),
    "clear_flow": (clear_flow, dict(exit_blocked=True)),
    "pusher_dead": (nominal, dict(pusher="dead")),
    "pusher_stuck_ext": (nominal, dict(pusher="stuck_ext", stuck_t=7000)),
    "gate_stuck_bore": (nominal, dict(gate="stuck_bore", stuck_t=7000)),
    "breech_stuck_clear": (nominal, dict(breech_stuck=0, stuck_t=7000)),
    "muzzle_stuck_clear": (nominal, dict(muzzle_stuck=0, stuck_t=7000)),
    "breech_delay": (nominal, dict(breech_delay_ms=30)),
    "ghost_muzzle": (nominal, dict(spurious_muzzle=[7300])),
    "telemetry_dead": (nominal, dict(telemetry_dead_t=7200)),
    "runaway": (nominal, dict(esc_runaway=[True, False], runaway_t=7100)),
    "hopper_empty": (nominal, dict(hopper=0)),
    "limiter_shorted": (nominal, dict(limiter_shorted=True)),
    "cfg_ranges": (cfg_ranges, {}),
}


@pytest.mark.parametrize("name", list(SCENARIOS))
def test_sim2_equals_the_c_core_on_scenario(name):
    fn, faults = SCENARIOS[name]
    c, s = eng_pair(**faults)
    t0 = c.t
    both((c, s), fn)
    compare(c, s, since=t0)


@pytest.mark.parametrize("balls,labelled", [(3, True), (2, False)])
def test_sim2_equals_the_c_core_on_the_hw_counter_branches(balls, labelled):
    c, s = eng_pair(cfg={p.CFG_HW_COUNTER_N: 3})
    t0 = c.t
    both((c, s), hw_trip(balls))
    compare(c, s, since=t0)
    assert bool([e for e in s.events if e[1] == p.EV_HW_COUNTER]) == labelled
    assert bool(s.st.fault_bits & p.FAULT_HW_COUNTER) == labelled


def test_sim2_is_not_vacuous_it_shoots_and_refuses():
    c, s = eng_pair()
    both((c, s), gates)
    results = [a.result for _, a in s.acks if a.cmd_id == p.FIRE]
    assert (
        p.OK in results
        and p.BUSY in results
        and p.BAD_ARG in results
        and p.INHIBIT in results
        and p.RANGE in results
    )
    assert len(s.plant.exits) >= 1


# ---- fuzz sample: the same random command sequences into both ----
_bases = {}


def fuzz_pair_base():
    if "p" not in _bases:
        _bases["p"] = eng_pair()
    return _bases["p"]


@pytest.mark.parametrize("seed", list(range(60)) + list(range(10**6, 10**6 + 20)))
def test_sim2_equals_the_c_core_on_fuzz_sequence(seed):
    cb, sb = fuzz_pair_base()
    faults = seed >= 10**6
    c, _ = TF.fuzz_case(seed, plant_faults=faults, base=cb)
    s, _ = TF.fuzz_case(seed, plant_faults=faults, base=sb)
    compare(c, s, since=cb.t)


def test_real_fsm_python_twin_is_what_sim2_runs():
    s = SimRig()
    assert type(s.sim.fdr).__module__ == "napl.cuas2.fsm"


@pytest.mark.parametrize(
    "breakage",
    [
        "n_check",
        "debounce",
        "ring_dedupe",
        "hw_label",
        "arm_dead_man",
        "arm_hard_cap",
        "heartbeat_watchdog",
        "shot_cap",
        "sector_shrink",
    ],
)
def test_compare_fails_when_the_twin_is_broken(monkeypatch, breakage):
    """Negative control of the cross-check: a twin with one rule changed no longer matches the C core on the scenario that exercises it."""
    import napl.cuas2.sim_mcu as S
    from napl.lite.sim_mcu import SimRP2040

    scenario = gates
    if breakage == "arm_dead_man":  # ARM no longer looks at the dead-man bit
        orig_arm = Sim2RP2040._arm

        def arm(self, f, m):
            self.hb_flags |= p.HB_DEADMAN_OK
            return orig_arm(self, f, m)

        monkeypatch.setattr(Sim2RP2040, "_arm", arm)
        scenario = arm_gates
    elif (
        breakage == "arm_hard_cap"
    ):  # ARM takes the requested cap, not min(cap, hard cap)
        orig_arm = Sim2RP2040._arm

        def arm(self, f, m):
            orig_arm(self, f, m)
            if self.state == p.ARMED:
                self.shot_cap = m.shot_cap

        monkeypatch.setattr(Sim2RP2040, "_arm", arm)
        scenario = arm_gates
    elif (
        breakage == "heartbeat_watchdog"
    ):  # the watchdog waits 100 ms longer than cfg 18
        orig_wd = Sim2RP2040._watchdog

        def wd(self):
            self.cfg[p.CFG_WD_MS] += 100
            try:
                orig_wd(self)
            finally:
                self.cfg[p.CFG_WD_MS] -= 100

        monkeypatch.setattr(Sim2RP2040, "_watchdog", wd)
        scenario = hb_loss
    elif breakage == "shot_cap":  # the twin allows one ball more than the ARM's cap
        orig_arm = Sim2RP2040._arm

        def arm(self, f, m):
            orig_arm(self, f, m)
            if self.state == p.ARMED:
                self.shot_cap += 1

        monkeypatch.setattr(Sim2RP2040, "_arm", arm)
        scenario = cap
    elif (
        breakage == "sector_shrink"
    ):  # lite's raw-position sector, no |v| * t_exit shrink
        monkeypatch.setattr(Sim2RP2040, "_in_sector", SimRP2040._in_sector)
        scenario = sector_slew
    if breakage == "n_check":  # a FIRE with n = 2 is treated as n = 1
        orig = Sim2RP2040._precheck
        monkeypatch.setattr(
            Sim2RP2040, "_precheck", lambda self, m: orig(self, p.Fire(m.fire_id, 1))
        )
    elif breakage == "debounce":
        monkeypatch.setattr(S, "DEBOUNCE_MS", 1)
    elif breakage == "hw_label":
        monkeypatch.setattr(S, "HWC_SKEW_MS", -1)
    if breakage == "hw_label":
        c, s = eng_pair(cfg={p.CFG_HW_COUNTER_N: 3})
        t0 = c.t
        both((c, s), hw_trip(3))
    else:
        c, s = eng_pair()
        if breakage == "ring_dedupe":  # the fire_id ring forgets everything
            s.sim.fire_ids = deque(maxlen=0)
        t0 = c.t
        both((c, s), scenario)
    with pytest.raises(AssertionError):
        compare(c, s, since=t0)
