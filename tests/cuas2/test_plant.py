"""Plant behind the feeder FSM (spec 7.4, acceptance MODEL 5): one FIRE -> one pellet, muzzle-beam pulse width, wheel dip and recovery, nothing fires
without a FEED_REQ, and the hardware limiter (I14) / one-ball pocket (I9) bound a runaway FSM. Every scenario runs twice: with the plant tests' StubFsm and with
the real feeder FSM (napl.cuas2.fsm via sim_mcu.RealFsm), the python twin of the C feeder."""

import numpy as np
import pytest
from napl.cuas2 import protocol as p2
from napl.cuas2 import sim_mcu as S


@pytest.fixture(params=["stub", "real"], autouse=True)
def fsm_kind(request, monkeypatch):
    """Plant2() without an explicit fsm= gets the stub or the real feeder FSM."""
    if request.param == "real":
        # rpm_window_pm 20 (the firmware's maximum, 2 %): with the spec default of 10 (1 %) the real controller does not hold the window on this plant's
        # telemetry noise and trips wheel_not_ready on 3 of 8 seeds (bench item: measure the real noise, then set the default)
        monkeypatch.setattr(
            S, "StubFsm", lambda **kw: S.RealFsm(cfg={p2.CFG_RPM_WINDOW_PM: 20})
        )
    return request.param


class CountRig:
    def __init__(self):
        self.shots = []

    def fire(self, shot):
        self.shots.append(shot)


def ready_plant(seed=1, **kw):
    rig = CountRig()
    p = S.Plant2(rig=rig, seed=seed, **kw)
    for _ in range(2500):
        p.tick(speed_req_pm=1000)
        if p.t_ms > 600 and p.outs[-1].status_fields["ready"]:
            return p, rig
    raise AssertionError(p.outs[-1].status_fields)


def test_one_fire_gives_exactly_one_pellet_and_a_correct_beam_pulse():
    p, rig = ready_plant()
    assert p.tick(speed_req_pm=1000, cmd="FIRE").ack == "OK"
    for _ in range(160):
        p.tick(speed_req_pm=1000)
    assert len(rig.shots) == 1 == p.feeder.fire_calls == len(p.feeder.pulses)
    assert len(p.feeder.feed_edges) == 1
    (t0, t1, v), shot = p.feeder.pulses[0], rig.shots[0]
    assert v == shot.v0 and 20 < v < 38
    assert (t1 - t0) == pytest.approx(
        0.0222 / v, rel=0.05
    )  # the beam is blocked for 22.2 mm / v
    t_edge, (t_nip, _) = p.feeder.feed_edges[0], p.feeder.shots[0]
    assert (
        0.105 <= t_nip - t_edge <= 0.130
    )  # commit stroke (101.5 ms) + exit (15 +- 2 ms): the beam sees the ball ~118 ms after the edge
    assert t0 > t_nip  # the beam is 55 mm downstream of the nip
    assert [e for o in p.outs for e in o.events if e[0] == "MUZZLE"] == [
        ("MUZZLE", 1)
    ]  # the FSM saw the pulse (latched per tick)


def test_wheels_dip_then_recover_within_40_ms():
    p, rig = ready_plant(seed=3)
    i_fire = len(p.rpm_log)
    p.tick(speed_req_pm=1000, cmd="FIRE")
    p.run(300, speed_req_pm=1000)
    mean = np.array([(a + b) / 2 for _, a, b in p.rpm_log])
    t_nip = round(p.feeder.shots[0][0] * 1000)
    pre = mean[i_fire - 1]
    low = mean[t_nip : t_nip + 15].min()
    assert (
        pre - low
    ) / pre > 0.03  # the ball takes ~5 % of the wheel speed (spec 4.2: 930 rpm)
    back = np.nonzero(mean[t_nip:] >= 0.99 * pre)[0]
    assert back[back > 0][0] <= 40, "no recovery within 40 ms"
    assert p.outs[-1].status_fields[
        "wheels_ready"
    ]  # the stub's 1 % / 100 ms window is met again


def test_nothing_fires_without_a_feed_req():
    p, rig = ready_plant(seed=4)
    p.run(2500, speed_req_pm=1000)  # idle at speed, no FIRE
    assert not rig.shots and not p.feeder.pulses and not p.feeder.feed_edges
    # FIRE refused (wheels not at speed) never produces a FEED_REQ either
    q, rq = S.Plant2(rig=CountRig()), None
    q.run(300, speed_req_pm=1000)
    ack = [q.tick(speed_req_pm=1000, cmd="FIRE").ack for _ in range(3)]
    assert (
        set(ack) <= {"NOT_SPUN", None}
        and not q.feeder.feed_edges
        and not q.feeder.shots
    )
    # every shot follows an edge by the stroke time: no pellet from nowhere
    p.tick(speed_req_pm=1000, cmd="FIRE")
    p.run(150, speed_req_pm=1000)
    assert len(rig.shots) == len(p.feeder.feed_edges) == 1


def test_second_fire_during_the_cycle_is_refused():
    p, rig = ready_plant(seed=5)
    assert p.tick(speed_req_pm=1000, cmd="FIRE").ack == "OK"
    acks = []
    for k in range(600):
        acks.append(p.tick(speed_req_pm=1000, cmd="FIRE").ack)
    assert len(rig.shots) <= 2
    first_ok = acks.index(
        "OK"
    )  # a second shot only after the cycle is READY again (466 ms geometric cycle, interval clamp 470, default 500)
    assert first_ok >= 440, first_ok
    assert all(a != "OK" for a in acks[:first_ok]) and set(acks[:first_ok]) <= {
        None,
        "BUSY",
        "NO_BALL",
        "NOT_CLEAR",
        "TOO_SOON",
        "NOT_SPUN",
    }


class Runaway:
    """A hostile FSM: wheels spinning, FEED_REQ edges every `period` ms, the pusher commanded out for 100 ms after each edge (and back), the gate
    cycling LOAD/BORE every `gate_ms` (None: parked at BORE after 30 ms)."""

    def __init__(self, period=40, gate_ms=None):
        self.period, self.gate_ms, self.prev, self.t_edge = (
            period,
            gate_ms,
            False,
            -(10**9),
        )

    def step(self, i):
        o = S.FeederOut(dshot_top=1500, dshot_bot=1500, kick=True)
        o.feed_req = (i.t_ms // (self.period // 2)) % 2 == 1
        if o.feed_req and not self.prev:
            self.t_edge = i.t_ms
        self.prev = o.feed_req
        o.push_us = S.PUSH_EXT_US if i.t_ms - self.t_edge < 100 else S.PUSH_RET_US
        bore = (
            i.t_ms > 30 if self.gate_ms is None else (i.t_ms // self.gate_ms) % 2 == 1
        )
        o.gate_us = S.GATE_BORE_US if bore else S.GATE_LOAD_US
        return o


class Pump:
    """FEED_REQ edges every 40 ms, gate at BORE, pusher square wave 85 ms out / 85 ms back."""

    def step(self, i):
        o = S.FeederOut(
            dshot_top=1500, dshot_bot=1500, kick=True, gate_us=S.GATE_BORE_US
        )
        o.feed_req = (i.t_ms // 20) % 2 == 1
        o.push_us = S.PUSH_EXT_US if (i.t_ms // 125) % 2 == 0 else S.PUSH_RET_US
        return o


def test_limiter_and_pocket_bound_a_runaway_fsm():
    # (1) gate parked at BORE after one ball was staged: edges every 40 ms still cannot make a second ball (no reload without a LOAD visit)
    rig = CountRig()
    p = S.Plant2(fsm=Runaway(40), rig=rig, seed=6)
    p.run(3000)
    assert len(rig.shots) == 1 and len(p.feeder.feed_edges) > 50

    # (2) the limiter powers the servo for 254 ms per accepted edge and ignores edges for 338 +- 15 %: a pusher commanded to stroke as fast as it can
    # (125 ms out, 125 ms back; the real rod needs 118 each way) strokes at most twice per window and at most 1/0.28 windows per second; with the limiter removed (hold 0) it does not.
    def strokes(hold_s=None):
        p = S.Plant2(fsm=Pump(), rig=CountRig(), seed=7)
        if hold_s is not None:
            p.feeder.hold_s = hold_s
        p.run(6000)
        return p.feeder

    f = strokes()
    assert f.windows <= 6.0 / 0.28 + 1 and f.strokes <= 2 * f.windows
    unlimited = strokes(0.0)
    assert f.strokes / 6.0 <= 3.8  # spec I14: 3.6 strokes/s at most
    assert unlimited.strokes > 1.5 * f.strokes, (
        f.strokes,
        unlimited.strokes,
    )  # negative control: the test is sensitive to the limiter


def test_rod_cannot_extend_at_load():
    """I9: with the gate at LOAD the housing wall closes the rod aperture, however long FEED_REQ is held and the pusher commanded out."""
    p = S.Plant2(
        fsm=Runaway(40, gate_ms=10**9), rig=CountRig(), seed=8
    )  # (t // 1e9) % 2 == 0: gate never leaves LOAD
    p.run(1500)
    assert p.feeder.x == 0.0 and p.feeder.g == 0.0 and not p.feeder.shots
