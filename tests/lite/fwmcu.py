"""FwMcu: the RP2040 firmware core (firmware/cuas-lite-rp2040/src, built for x86 as libcuas_lite_fwcore.so) behind the SimRP2040 interface, so the
test rigs (rig.py, apprig.py) run unchanged against the real C logic. Select it with `pytest --fw-core=PATH` or NAPL_FW_CORE=PATH.

The C core is the MCU; this class is the board around it: it plays the stepper driver + joint (microsteps -> plant joint goals), the hall sensor
(window of +-1 deg around q_hall, as the twin), the TMC2209 DIAG pin (raised when the joint is more than 4 full steps from the commanded angle, the twin's
lost-steps rule), the E-stop and key GPIOs (plant.estop_open / plant.key_on), the flywheel PWM and the pusher (plant.set_flywheel / fire_pulse).
"""

import ctypes as C
import os
from pathlib import Path

from napl.lite import protocol as p

INT32_MIN = -(2**31)
DEFAULT_LIB = (
    Path(__file__).resolve().parents[2] / "build" / "fw-host" / "libcuas_lite_fwcore.so"
)
TR_TRANSITION, TR_EVENT, TR_SHOT, TR_REFUSAL = 1, 2, 3, 4


def lib_path():
    v = os.environ.get("NAPL_FW_CORE", "")
    return Path(v) if v and v != "auto" else DEFAULT_LIB


def available():
    return lib_path().exists()


_lib = None


def lib():
    global _lib
    if _lib is None:
        L = C.CDLL(str(lib_path()))
        u8p = C.POINTER(C.c_uint8)
        L.fwh_new.restype = C.c_void_p
        L.fwh_new.argtypes = [C.POINTER(C.c_int32), C.c_int, C.c_char_p]
        L.fwh_free.argtypes = [C.c_void_p]
        L.fwh_tick.restype = C.c_int
        L.fwh_tick.argtypes = [C.c_void_p, C.c_uint32, C.c_char_p, C.c_int] + [
            C.c_int
        ] * 5
        L.fwh_tx.restype = C.POINTER(C.c_uint8)
        L.fwh_tx.argtypes = [C.c_void_p]
        L.fwh_out.argtypes = [C.c_void_p, C.POINTER(C.c_int32)]
        L.fwh_snap.argtypes = [C.c_void_p, C.POINTER(C.c_int32), C.POINTER(C.c_double)]
        L.fwh_clear_homed.argtypes = [C.c_void_p, C.c_int]
        L.fwh_cfg.restype = C.c_int
        L.fwh_cfg.argtypes = [C.c_void_p, C.c_int, C.c_int]
        L.fwh_flash_valid.argtypes = [C.c_void_p]
        L.fwh_flash_image.argtypes = [C.c_void_p, u8p]
        L.fwh_set_boot_note.argtypes = [C.c_void_p, C.c_char_p]
        L.fwh_trace_count.argtypes = [C.c_void_p]
        L.fwh_trace_get.argtypes = [C.c_void_p, C.c_int, C.POINTER(C.c_int32)]
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
        _lib = L
    return _lib


class _Cfg:
    def __init__(self, mcu, flash):
        self.mcu, self.flash = mcu, flash

    def __getitem__(self, key):
        return lib().fwh_cfg(self.mcu.h, key, int(self.flash))

    def items(self):
        return [(k, self[k]) for k in sorted(p.CFG_DEFAULTS)]


class _Axis:
    def __init__(self, mcu, i):
        self.mcu, self.i = mcu, i

    @property
    def v(self):
        return self.mcu._snap()[1][4 + self.i]

    @property
    def pos(self):
        return self.mcu._snap()[1][2 + self.i]

    @property
    def homed(self):
        return bool(self.mcu._snap()[0][10 + self.i])

    @homed.setter
    def homed(self, val):
        assert not val, "the firmware owns the homed flag"
        lib().fwh_clear_homed(self.mcu.h, self.i)
        self.mcu._snap_cache = None


class FwMcu:
    def __init__(
        self,
        plant,
        link,
        uid=bytes.fromhex("e66050a8d3156f21"),
        q_hall=(0.0, 0.0),
        step_loss=True,
        cfg_invalid=False,
        home_off_deg=(-1.0, -1.0),
        status_from_joint=False,
        flash_mode=None,  # None: image with the calibrated home offsets (or a corrupt one if cfg_invalid); 0 blank flash
    ):
        assert not status_from_joint, (
            "FwMcu reports its own step counts (an NaplPlant run needs the python twin)"
        )
        cfg = (C.c_int32 * 23)(*([INT32_MIN] * 23))  # CFG_COUNT
        cfg[p.CFG_PAN_HOME_OFF] = round(
            home_off_deg[0] * p.CFG_DEFAULTS[p.CFG_PAN_SPR] / 360
        )  # same calibration as the twin's constructor argument
        cfg[p.CFG_TILT_HOME_OFF] = round(
            home_off_deg[1] * p.CFG_DEFAULTS[p.CFG_TILT_SPR] / 360
        )
        if flash_mode is None:
            flash_mode = 2 if cfg_invalid else 1
        self._boot_args = (cfg, flash_mode, uid)
        self.h = lib().fwh_new(cfg, flash_mode, uid)
        self.step_log = None  # a list switches on recording of (ms, pan_steps, tilt_steps) per tick, the stream the step generator would emit
        self.plant, self.link, self.uid = plant, link, uid
        self.q_hall, self.step_loss = q_hall, step_loss
        plant.set_torque(False)
        self.t = self.t0 = None
        self.cmd_joint = list(
            plant.joint_deg()
        )  # absolute joint angle the commanded microsteps correspond to (steps = 0 at power-up)
        self.diag = 0
        self.goal_sent = None
        self.steppers_en = 0
        self.shots_fed = 0
        self.fw_duty = None
        self.transitions, self.events, self.shots, self.refusals = [], [], [], []
        self.ax = [_Axis(self, 0), _Axis(self, 1)]
        self.cfg, self.flash = _Cfg(self, False), _Cfg(self, True)
        self.bootsel = False
        self._snap_cache = None

    def reboot(self):
        """Power cycle: a fresh C core (SAFE, not homed, no session, sector UNSET, config from the boot-time flash image); plant and logs stay."""
        lib().fwh_free(self.h)
        self.h = lib().fwh_new(*self._boot_args)
        self.t = self.t0 = None
        self.cmd_joint = list(self.plant.joint_deg())
        self.diag = 0
        self.goal_sent = None
        self.steppers_en = 0
        self.fw_duty = None
        self.plant.set_torque(False)
        self._snap_cache = None

    def __del__(self):
        h, self.h = getattr(self, "h", None), None
        if h:
            lib().fwh_free(h)

    # ---- views the tests use (names as SimRP2040) ----
    def _snap(self):
        if self._snap_cache is None:
            i, d = (C.c_int32 * 16)(), (C.c_double * 6)()
            lib().fwh_snap(self.h, i, d)
            self._snap_cache = (list(i), list(d))
        return self._snap_cache

    state = property(lambda s: s._snap()[0][0])
    shot_cap = property(lambda s: s._snap()[0][3])
    shots_arm = property(lambda s: s._snap()[0][4])
    fault_bits = property(lambda s: s._snap()[0][6])
    version_bad = property(
        lambda s: bool(s._snap()[0][7]),
        lambda s, v: (
            None
        ),  # test_gates sets it to False (a no-op for a MCU that has not seen a bad HELLO)
    )
    hello_done = property(lambda s: bool(s._snap()[0][8]))
    steppers_enabled = property(lambda s: bool(s._snap()[0][13]))
    fw_req = property(lambda s: s._snap()[1][0])
    fw = property(lambda s: s._snap()[1][1])

    @property
    def last_hb_t(self):
        ms = self._snap()[0][2]
        return None if ms < 0 else self.t0 + ms / 1000.0

    # ---- time ----
    def step(self, now):
        if self.t is None:
            self.t = self.t0 = now
            return
        while self.t + 0.001 <= now + 1e-9:
            self.t += 0.001
            self._tick()
        self._write_plant()

    def _tick(self):
        L, pl = lib(), self.plant
        data = self.link.mcu_read(self.t)
        if hasattr(pl, "advance"):
            pl.advance(0.001)
        q = pl.joint_deg()
        hall = [int(abs(q[i] - self.q_hall[i]) < 1.0) for i in (0, 1)]
        ms = int(round((self.t - self.t0) * 1000))
        n = L.fwh_tick(
            self.h,
            ms,
            data,
            len(data),
            int(bool(pl.estop_open)),
            int(bool(pl.key_on)),
            hall[0],
            hall[1],
            self.diag,
        )
        self._snap_cache = None
        if n:
            self.link.mcu_write(bytes(L.fwh_tx(self.h)[:n]), self.t)
        o = (C.c_int32 * 12)()
        L.fwh_out(self.h, o)
        if o[0] != self.steppers_en:
            self.steppers_en = o[0]
            pl.set_torque(bool(o[0]))
        if self.step_log is not None and (o[1] or o[2]):
            self.step_log.append((ms, o[1], o[2]))
        for i in (0, 1):
            # logical microsteps; the DIR level is (n > 0) ^ invert, so a set invert bit reverses the physical motion
            if o[1 + i]:
                spr = L.fwh_cfg(self.h, p.CFG_PAN_SPR + i, 0)
                self.cmd_joint[i] += (-1 if o[3 + i] else 1) * o[1 + i] * 360.0 / spr
        if o[5] != self.fw_duty:
            self.fw_duty = o[5]
            pl.set_flywheel(float(o[5]))
        while self.shots_fed < o[7]:
            self.shots_fed += 1
            pl.fire_pulse()
        if o[8]:
            self.bootsel = True
        self._drain_trace()

    def _drain_trace(self):
        L = lib()
        n = L.fwh_trace_count(self.h)
        if not n:
            return
        r = (C.c_int32 * 5)()
        for k in range(n):
            L.fwh_trace_get(self.h, k, r)
            kind, ms, a, b, c = r
            t = self.t0 + ms / 1000.0
            if kind == TR_TRANSITION:
                self.transitions.append((t, a, b))
            elif kind == TR_EVENT:
                self.events.append((t, a, b))
            elif kind == TR_SHOT:
                self.shots.append((t, a, b))
            elif kind == TR_REFUSAL:
                self.refusals.append((t, a, b))
        L.fwh_trace_clear(self.h)

    def _write_plant(self):
        """Once per step() call (as the twin): joint goals, and the TMC2209 DIAG rule: a joint more than 4 full motor steps from the command stalled."""
        if self.t is None:
            return
        q = self.plant.joint_deg()
        en = self.steppers_en
        homed = [self.ax[0].homed, self.ax[1].homed]
        self.diag = 0
        if (
            self.step_loss and en and self.goal_sent is not None
        ):  # the joint is compared with the goal of the PREVIOUS call (it has had 5 ms to follow)
            for i in (0, 1):
                spr = lib().fwh_cfg(self.h, p.CFG_PAN_SPR + i, 0)
                if homed[i] and abs(q[i] - self.goal_sent[i]) > 4 * 16 * 360.0 / spr:
                    self.diag = 1
        if en:
            self.plant.set_joint_goal(self.cmd_joint[0], self.cmd_joint[1])
            self.goal_sent = list(self.cmd_joint)
        else:
            self.goal_sent = None
