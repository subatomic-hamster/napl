"""FW twin of the cuas2 launcher + feeder (docs/cuas2/spec.md sections 4.4, 5.3-5.5, 1.2 I9-I13, I15, 1.3): the same statements as
firmware/cuas2-rp2040/src/{launcher,feeder}.c, integer arithmetic only, so the C core and this module give byte-identical outputs for the same inputs
(tests/cuas2/fsm_vectors.json drives both). One FeederFsm.step() per millisecond:

    fsm = FeederFsm()
    out = fsm.step(FeederIn(t_ms=1, rpm_top=17500, rpm_bot=17500, beam_breech=True, ..., cmd="FIRE", speed_req_pm=1000, cfg={}))
    out.dshot_top, out.gate_us, out.feed_req, out.kick, out.events, out.ack, out.status_fields

The safety FSM around it (heartbeat, ARM, fire_id idempotence, shot cap, sector, STATUS framing) lives in the C core only (firmware/cuas2-rp2040/src/fsm.c);
its inputs reach this unit as `hb_ok, deadman, inhibit, in_sector, estop_open, key_on` and `speed_req_pm` (0 unless armed and the rail is on).
`FeederIn`/`FeederOut` follow the contract of spec section 12 (MODEL); `fire_id` and `muzzle_dt_us` are optional trailing extras (default 0).
"""

from dataclasses import dataclass, field

from . import protocol as p

# gate states (STATUS.gate_state)
(
    GS_UNKNOWN,
    GS_HOMING,
    GS_LOADING,
    GS_TO_BORE,
    GS_STAGED,
    GS_PUSH,
    GS_EXIT_WAIT,
    GS_RETRACT,
    GS_TO_LOAD,
    GS_EMPTY,
    GS_JAM,
) = range(11)
# timing constants of the feed cycle [est]; bench item B-TIME (spec 10). Same names and values as include/fw_core.h.
COMMIT_MS = 45
PUSH_LEAVE_MS = 40
BREECH_EARLY_MS = 20
BREECH_EXP_MS = 40
BREECH_CLEAR_MAX_MS = 150
RETRACT_CLEAR_MS = 55
RETRACT_TIMEOUT_MS = 300
GATE_TRAVEL_MS = 100
GATE_HOME_TIMEOUT_MS = 400
HOMING_TIMEOUT_MS = 1000
FEED_PULSE_MS = 10
WHEEL_AGE_MAX_MS = 20
NOT_READY_MS = 1500
OVERSPEED_MS = 20
MIN_INTERVAL_FLOOR = 400
MUZ_MIN_GAP_MS = 300
EMPTY_RETRIES = 3
EMPTY_RETRY_MS = 1000
MAX_DIFF_PM = 60
DIP_WINDOW_MS = 120
MAX_EV = 12
U32 = 0xFFFFFFFF


def _td(a, b):
    """Integer division truncating toward zero (C semantics)."""
    q = abs(a) // abs(b)
    return q if (a >= 0) == (b > 0) else -q


def _clamp(x, lo, hi):
    return lo if x < lo else (min(x, hi))


def _s32(x):
    x &= U32
    return x - (1 << 32) if x & 0x80000000 else x


@dataclass
class FeederIn:
    """One millisecond of sampled inputs. rpm_*: mechanical rpm from the eRPM reply, None = no valid reply this tick. beam_*: 1 = blocked (muzzle: level OR
    an edge latched since the last tick). cmd: None | "FIRE" | "CLEAR". diff_pm: None = cfg 29. cfg: {key: value}, missing keys take the defaults."""

    t_ms: int
    rpm_top: int | None = None
    rpm_bot: int | None = None
    beam_breech: bool = False
    beam_muzzle: bool = False
    gate_home: bool = False
    push_home: bool = True
    estop_open: bool = False
    key_on: bool = True
    hb_ok: bool = True
    deadman: bool = True
    inhibit: bool = False
    in_sector: bool = True
    cmd: str | None = None
    speed_req_pm: int = 0
    diff_pm: int | None = None
    cfg: dict | None = None
    fire_id: int = 0
    muzzle_dt_us: int = 0


@dataclass
class FeederOut:
    dshot_top: int = 0
    dshot_bot: int = 0
    gate_us: int = 0
    push_us: int = 0
    feed_req: bool = False
    kick: bool = True
    status_fields: dict = field(default_factory=dict)
    events: list = field(default_factory=list)  # [(code, arg)]
    ack: int | None = None


class FeederFsm:
    def __init__(self):
        # launcher
        self.ramp_mean = 0
        self.integ = [0, 0]
        self.thr = [0, 0]
        self.last_rpm = [0, 0]
        self.rpm_cap = 0
        self.age = [0, 0]
        self.win_ms = self.over_ms = self.nr_ms = self.on_ms = 0
        self.prev_target = 0
        self.ready_prev = False
        self.inwin_now = False
        # feeder
        self.gstate = GS_UNKNOWN
        self.gate_load = True
        self.t_state = self.t_feed = self.t_last_feed = self.t_last_muz = 0
        self.has_feed = self.has_muz = self.prev_muz = self.inited = False
        self.committed = self.aborted = self.exp_exit = self.exit_seen = (
            self.left_seen
        ) = False
        self.breech_clr_seen = self.empty_reported = self.trk = self.feed_req = False
        self.retries = 0
        self.fire_id = self.feeds = self.muz_all = 0
        self.rpm_feed_mean = self.rpm_min_mean = 0
        self.last_v_dmps = 0xFFFF
        self.last_dip_pm = 0
        self.feed_ms = 0
        self.jam_reported = False
        self.fault_bits = 0
        self._ev = []

    # ---- helpers ----------------------------------------------------------------------------------------------------------------------
    def _enter(self, gs, now):
        self.gstate = gs
        self.t_state = now
        self.gate_load = gs in (GS_UNKNOWN, GS_HOMING, GS_LOADING, GS_TO_LOAD)

    def _ev_add(self, code, arg):
        if len(self._ev) < MAX_EV:
            self._ev.append((code, _s32(arg)))

    def _fault(self, bit):
        if self.fault_bits & bit:
            return
        self.fault_bits |= bit
        self._ev_add(p.EV_FAULT, bit.bit_length() - 1)

    @staticmethod
    def _min_interval(c):
        return max(c[p.CFG_MIN_SHOT_INTERVAL_MS], MIN_INTERVAL_FLOOR)

    def _dip_pm(self):
        if self.rpm_feed_mean <= 0:
            return 0
        v = (self.rpm_feed_mean - self.rpm_min_mean) * 1000 // self.rpm_feed_mean
        return _clamp(v, 0, 65535)

    @staticmethod
    def _busy(g):
        return g in (
            GS_PUSH,
            GS_EXIT_WAIT,
            GS_RETRACT,
            GS_TO_LOAD,
            GS_LOADING,
            GS_TO_BORE,
        )

    # ---- launcher (launcher.c) ----------------------------------------------------------------------------------------------------------
    def _launcher(self, inp, c, rail):
        cap = p.rpm_cap(c[p.CFG_WHEEL_D_UM], c[p.CFG_V_CAP_DMPS])
        tl, window, tmax = (
            c[p.CFG_TELEMETRY_LOSS_MS],
            c[p.CFG_RPM_WINDOW_PM],
            c[p.CFG_THROTTLE_MAX_PM],
        )
        self.rpm_cap = cap
        rpm_max = max(min(c[p.CFG_RPM_MAX], cap), 0)
        req = min(inp.speed_req_pm, 1000)
        dpm = c[p.CFG_DIFF_PM] if inp.diff_pm is None else inp.diff_pm
        diff = _clamp(dpm, -MAX_DIFF_PM, MAX_DIFF_PM)
        target = (
            (req * rpm_max // 1000)
            if (rail and not (self.fault_bits & p.FDR_FAULT_MASK))
            else 0
        )
        dd = _td(target * diff, 2000)
        st = [target - dd, target + dd]

        step = max(c[p.CFG_RAMP_RPM_S] // 1000, 1)
        if target == 0:
            self.ramp_mean = 0
        elif self.ramp_mean < target:
            self.ramp_mean = min(self.ramp_mean + step, target)
        else:
            self.ramp_mean = max(self.ramp_mean - step, target)
        rd = _td(self.ramp_mean * diff, 2000)
        rs = [self.ramp_mean - rd, self.ramp_mean + rd]

        for i, r in enumerate((inp.rpm_top, inp.rpm_bot)):
            if r is not None and r >= 0:
                self.last_rpm[i] = r
                self.age[i] = 0
            elif self.age[i] < 65535:
                self.age[i] += 1
        self.on_ms = self.on_ms + 1 if target > 0 else 0

        kp, ki, capd = c[p.CFG_KP], c[p.CFG_KI], max(cap, 1)
        ilim = tmax * 1000 // max(ki, 1)
        for i in range(2):
            if target == 0:
                self.integ[i] = 0
                self.thr[i] = 0
                continue
            err = rs[i] - self.last_rpm[i]
            self.integ[i] = _clamp(self.integ[i] + err, -ilim, ilim)
            u = (
                rs[i] * tmax // capd
                + _td(kp * err, 1000)
                + _td(ki * self.integ[i], 1000)
            )
            self.thr[i] = _clamp(u, 0, tmax)

        all_ok = target > 0 and self.ramp_mean == target
        for i in range(2):
            all_ok = (
                all_ok
                and self.age[i] <= WHEEL_AGE_MAX_MS
                and st[i] > 0
                and abs(self.last_rpm[i] - st[i]) * 1000 <= window * st[i]
            )
        all_ok = (
            all_ok
            and abs((self.last_rpm[1] - self.last_rpm[0]) - (st[1] - st[0])) * 1000
            <= window * target
        )
        self.inwin_now = all_ok
        if all_ok:
            self.win_ms = min(self.win_ms + 1, 65535)
        else:
            self.win_ms = 0
        ready = all_ok and self.win_ms >= c[p.CFG_SETTLE_MS]
        mean_rpm = (self.last_rpm[0] + self.last_rpm[1]) // 2
        if ready and not self.ready_prev:
            self._ev_add(p.EV_WHEELS_READY, mean_rpm)
        if not ready and self.ready_prev and target > 0:
            self._ev_add(p.EV_WHEELS_LOST, mean_rpm)
        self.ready_prev = ready

        if target > 0 and self.on_ms > tl and (self.age[0] > tl or self.age[1] > tl):
            self._fault(p.FAULT_TELEMETRY_LOST)
        over = any(
            self.age[i] <= WHEEL_AGE_MAX_MS and self.last_rpm[i] * 100 > cap * 105
            for i in range(2)
        )
        self.over_ms = self.over_ms + 1 if over else 0
        if self.over_ms >= OVERSPEED_MS:
            self._fault(p.FAULT_OVERSPEED)
        if target > 0:
            if target != self.prev_target or ready:
                self.nr_ms = 0
            else:
                self.nr_ms += 1
            if self.nr_ms >= NOT_READY_MS:
                self._fault(p.FAULT_WHEEL_NOT_READY)
        else:
            self.nr_ms = 0
        self.prev_target = target

        return (
            target,
            ready,
            [(48 + self.thr[i] * 1952 // 1000) if target > 0 else 0 for i in range(2)],
            cap,
        )

    # ---- one tick (feeder.c: fdr_step) ----------------------------------------------------------------------------------------------------
    def clear_faults(self, now=0):
        self.fault_bits = 0
        self.over_ms = self.nr_ms = 0
        self.retries = 0
        self.empty_reported = self.jam_reported = False
        self.exp_exit = self.aborted = self.trk = self.feed_req = self.committed = False
        self._enter(GS_UNKNOWN, now)

    def _fire_result(self, inp, c, rail):
        now = inp.t_ms
        if not rail:
            return p.INTERLOCK
        if not inp.hb_ok or not inp.deadman or inp.inhibit:
            return p.INHIBIT
        if not inp.in_sector:
            return p.RANGE
        if self.fault_bits & p.FDR_FAULT_MASK:
            return p.JAMMED
        if self.gstate == GS_EMPTY:
            return p.NO_BALL
        if self.gstate != GS_STAGED:
            return p.BUSY
        if inp.beam_muzzle:
            return p.NOT_CLEAR
        if self.has_feed and ((now - self.t_last_feed) & U32) < self._min_interval(c):
            return p.TOO_SOON
        if not self.ready_prev:
            return p.NOT_SPUN
        if not inp.beam_breech:
            return p.NO_BALL
        if not inp.push_home:
            return p.JAMMED
        return p.OK

    def step(self, inp: FeederIn) -> FeederOut:
        c = dict(p.CFG_DEFAULTS2)
        if inp.cfg:
            c.update(inp.cfg)
        now = inp.t_ms & U32
        rail = bool(inp.key_on and not inp.estop_open)
        self._ev = []
        ack = None
        if inp.cmd == "CLEAR":
            self.clear_faults(now)
        target, ready, dshot, cap = self._launcher(inp, c, rail)
        tf = (now - self.t_feed) & U32

        if self.has_feed and tf < DIP_WINDOW_MS:
            m = (self.last_rpm[0] + self.last_rpm[1]) // 2
            self.rpm_min_mean = min(self.rpm_min_mean, m)
            self.last_dip_pm = self._dip_pm()

        mz = bool(inp.beam_muzzle)
        if not self.inited:  # the first sample is the initial condition, not an edge (a beam blocked at power-up reads NOT_CLEAR)
            self.inited = True
            self.prev_muz = mz
        if mz and not self.prev_muz:
            ok = self.exp_exit and not self.exit_seen
            if self.has_muz and ((now - self.t_last_muz) & U32) < MUZ_MIN_GAP_MS:
                ok = False
            self.muz_all = (self.muz_all + 1) & 0xFFFF
            if ok:
                v = 0xFFFF
                if inp.muzzle_dt_us > 0:
                    v = min(222 * c[p.CFG_K_BEAM] // inp.muzzle_dt_us, 0xFFFE)
                self.exit_seen = True
                self.exp_exit = False
                self.last_v_dmps = v
                self.last_dip_pm = self._dip_pm()
                self.feed_ms = min((tf + 2) // 4, 255)
                self._ev_add(p.EV_MUZZLE, v | self.last_dip_pm << 16)
            else:
                self._fault(p.FAULT_UNEXPECTED_EXIT)
            self.has_muz = True
            self.t_last_muz = now
        self.prev_muz = mz
        if self.muz_all > self.feeds:
            self._fault(p.FAULT_SHOT_OVERCOUNT)
        if self.exp_exit and not self.exit_seen and tf >= c[p.CFG_EXIT_TIMEOUT_MS]:
            self._fault(p.FAULT_NO_EXIT)

        if self.trk and not (
            self.fault_bits & p.FDR_FAULT_MASK
        ):  # (frozen once a fault is latched: no cascade)
            if not inp.push_home:
                self.left_seen = True
            if not self.aborted:
                if not self.breech_clr_seen and not inp.beam_breech:
                    self.breech_clr_seen = True
                    if tf * 10 > BREECH_EXP_MS * 13:
                        self._fault(p.FAULT_BREECH_DOUBLE)
                if not self.breech_clr_seen and tf >= BREECH_CLEAR_MAX_MS:
                    self._fault(p.FAULT_NO_EXIT)
                if not self.left_seen and tf >= PUSH_LEAVE_MS:
                    self._fault(p.FAULT_PUSHER_STUCK)
            if tf >= RETRACT_TIMEOUT_MS:
                if not inp.push_home:
                    self._fault(p.FAULT_PUSHER_STUCK)
                self.trk = False
        if self.feed_req and tf >= FEED_PULSE_MS:
            self.feed_req = False

        te = (now - self.t_state) & U32
        dwell = c[p.CFG_PUSHER_DWELL_MS]
        jam = bool(self.fault_bits & p.FDR_FAULT_MASK)
        g = self.gstate
        if jam:
            self.feed_req = False
        elif not rail:
            if g == GS_PUSH and not self.committed:
                self._ev_add(p.EV_FEED_ABORT, self.fire_id << 8 | p.FA_RAIL)
            if g != GS_UNKNOWN:
                self._enter(GS_UNKNOWN, now)
            self.feed_req = False
            self.trk = False
        elif g == GS_UNKNOWN:
            self._enter(GS_HOMING, now)
        elif g == GS_HOMING:
            if inp.gate_home and inp.push_home:
                self._enter(GS_LOADING, now)
            elif te >= HOMING_TIMEOUT_MS:
                self._fault(
                    p.FAULT_PUSHER_STUCK if inp.gate_home else p.FAULT_GATE_STUCK
                )
        elif g == GS_LOADING:
            if te >= c[p.CFG_FILL_MS]:
                self._enter(GS_TO_BORE, now)
        elif g == GS_TO_BORE:
            if te >= GATE_TRAVEL_MS:
                if inp.beam_breech:
                    self.retries = 0
                    self.empty_reported = False
                    self._enter(GS_STAGED, now)
                else:
                    self.retries += 1
                    if self.retries >= EMPTY_RETRIES:
                        if not self.empty_reported:
                            self.empty_reported = True
                            self._ev_add(p.EV_EMPTY, 0)
                        self._enter(GS_EMPTY, now)
                    else:
                        self._enter(GS_TO_LOAD, now)
        elif g == GS_TO_LOAD:
            if inp.gate_home:
                self._enter(GS_LOADING, now)
            elif te >= GATE_HOME_TIMEOUT_MS:
                self._fault(p.FAULT_GATE_STUCK)
        elif g == GS_EMPTY:
            if te >= EMPTY_RETRY_MS:
                self.retries = EMPTY_RETRIES - 1
                self._enter(GS_TO_LOAD, now)
        elif g == GS_PUSH:
            reason = 0
            # commit = the EARLIER of COMMIT_MS and the breech beam clearing after BREECH_EARLY_MS (the ball has left the breech: no abort may follow)
            left = self.breech_clr_seen and tf >= BREECH_EARLY_MS
            if tf < COMMIT_MS and not left:
                if not inp.in_sector:
                    reason = p.FA_SECTOR
                elif not inp.hb_ok or not inp.deadman:
                    reason = p.FA_DEADMAN
                elif inp.inhibit:
                    reason = p.FA_INHIBIT
                elif not self.inwin_now:
                    reason = p.FA_WHEELS
                elif tf < BREECH_EARLY_MS and not inp.beam_breech:
                    reason = p.FA_BEAM
            if reason:
                self._ev_add(p.EV_FEED_ABORT, self.fire_id << 8 | reason)
                if reason == p.FA_SECTOR:
                    self._ev_add(p.EV_SECTOR_ABORT, self.fire_id << 8)
                self.aborted = True
                self.feed_req = False
                self._enter(GS_RETRACT, now)
            elif tf >= COMMIT_MS or left:
                self.committed = True
                self.feeds = (self.feeds + 1) & 0xFFFF
                self.exp_exit = True
                self.exit_seen = False
                self._ev_add(p.EV_SHOT, self.fire_id << 8)
                self._enter(GS_EXIT_WAIT, now)
        elif g == GS_EXIT_WAIT:
            if self.exit_seen:
                self._enter(GS_RETRACT, now)
        elif g == GS_RETRACT:
            if self.aborted:
                if inp.push_home and te >= 10:
                    self._enter(
                        GS_STAGED, now
                    )  # (aborted stays set until the next feed)
            elif tf >= dwell + RETRACT_CLEAR_MS and inp.push_home:
                self._enter(GS_TO_LOAD, now)

        if inp.cmd == "FIRE":
            res = self._fire_result(inp, c, rail)
            ack = res
            if res == p.OK:
                if (
                    self.has_feed
                    and ((now - self.t_last_feed) & U32) < MIN_INTERVAL_FLOOR
                ):
                    self._fault(p.FAULT_SHOT_OVERCOUNT)
                self.has_feed = True
                self.t_feed = self.t_last_feed = now
                self.fire_id = inp.fire_id & 0xFFFF
                self.committed = self.aborted = False
                self.exit_seen = self.left_seen = self.breech_clr_seen = False
                self.trk = True
                self.feed_req = True
                self.rpm_feed_mean = (self.last_rpm[0] + self.last_rpm[1]) // 2
                self.rpm_min_mean = self.rpm_feed_mean
                self.last_dip_pm = 0
                self._enter(GS_PUSH, now)

        if (self.fault_bits & p.FDR_JAM_MASK) and not self.jam_reported:
            self.jam_reported = True
            self._ev_add(p.EV_JAM, self.gstate)

        jam_now = bool(self.fault_bits & p.FDR_FAULT_MASK)
        tf2 = (now - self.t_feed) & U32
        ext = (self.trk and not self.aborted and not jam_now and rail and tf2 < dwell
               and self.gstate in (GS_PUSH, GS_EXIT_WAIT, GS_RETRACT))  # fmt: skip
        ready_flag = (rail and not jam_now and self.gstate == GS_STAGED and not inp.beam_muzzle
                      and (not self.has_feed or ((now - self.t_last_feed) & U32) >= self._min_interval(c))
                      and self.ready_prev and inp.beam_breech and inp.push_home)  # fmt: skip
        thr_mean = (self.thr[0] + self.thr[1]) // 2
        if jam_now:
            dshot = [0, 0]
            thr_mean = 0
            self.ramp_mean = 0
            self.integ = [0, 0]
        tlim = c[p.CFG_TELEMETRY_LOSS_MS]
        busy = self._busy(self.gstate)
        out = FeederOut(
            dshot_top=dshot[0], dshot_bot=dshot[1],
            gate_us=c[p.CFG_GATE_LOAD_US] if self.gate_load else c[p.CFG_GATE_BORE_US],
            push_us=c[p.CFG_PUSHER_EXT_US] if ext else c[p.CFG_PUSHER_RET_US],
            feed_req=bool(self.feed_req and not jam_now), kick=not jam_now, events=self._ev, ack=ack,
        )  # fmt: skip
        out.status_fields = {
            "rpm_top": 0xFFFF
            if self.age[0] > tlim
            else _clamp(self.last_rpm[0], 0, 0xFFFE),
            "rpm_bot": 0xFFFF
            if self.age[1] > tlim
            else _clamp(self.last_rpm[1], 0, 0xFFFE),
            "rpm_set": target,
            "throttle_mean_pm": thr_mean,
            "beams": (
                (p.BM_BREECH if inp.beam_breech else 0)
                | (p.BM_MUZZLE if inp.beam_muzzle else 0)
                | (p.BM_GATE_HOME if inp.gate_home else 0)
                | (p.BM_PUSH_HOME if inp.push_home else 0)
                | (p.BM_WHEELS_READY if ready else 0)
                | (p.BM_FEED_BUSY if busy else 0)
            ),  # fmt: skip
            "gate_state": GS_JAM if (self.fault_bits & p.FDR_JAM_MASK) else self.gstate,
            "ready_flags": (
                (p.RDY_READY if ready_flag else 0)
                | ((p.RDY_EMPTY | p.RDY_HOPPER_LOW) if self.gstate == GS_EMPTY else 0)
                | (p.RDY_JAM if (self.fault_bits & p.FDR_JAM_MASK) else 0)
            ),  # fmt: skip
            "feed_ms": self.feed_ms,
            "feed_busy": int(busy),
            "wheels_ready": int(ready),
            "last_v_dmps": self.last_v_dmps,
            "last_dip_pm": self.last_dip_pm,
            "shots_confirmed": self.muz_all,
            "fault_bits": self.fault_bits,
            "rpm_cap": cap,
        }
        return out


__all__ = ["FeederFsm", "FeederIn", "FeederOut"]
