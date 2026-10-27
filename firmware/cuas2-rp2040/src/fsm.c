/* cuas2 safety FSM: command dispatch, the six-state machine, the 300 ms heartbeat watchdog, ARM / FIRE gates, shot accounting, E-stop / key inputs, faults,
 * STATUS. FORKED from firmware/cuas-lite-rp2040/src/fsm.c @ 8f173e6 (spec 2.3): the functions listed in tests/cuas2/fsm_shared_functions.txt are byte-identical
 * to lite's (tools/cuas2_fsm_drift.sh fails if one drifts); the weapon section (flywheel request, FIRE gates, pusher, shot accounting, STATUS tail) is replaced
 * by the launcher + feeder module (launcher.c, feeder.c), called once per tick from feeder_stage().
 * Order of one tick: rx, inputs, hardware-counter note, launcher + feeder, watchdog, timers, axes, status, outputs.
 * SPDX-License-Identifier: BSD-3-Clause */
#include <math.h>
#include <string.h>
#include "fw_priv.h"

#define FW_REFRESH_MS 500
#define DEADMAN_LOW_MS 1000
#define STATUS_PERIOD_MS 10
#define DIAG_FAULT_MS 50
#define DEBOUNCE_MS 10            /* E-stop released / key on must be stable this long; open / off acts at once */

static uint16_t get16(const uint8_t *p) { return (uint16_t)(p[0] | p[1] << 8); }
static uint32_t get32(const uint8_t *p) { return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24; }
static void put16(uint8_t *p, uint32_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static void put32(uint8_t *p, uint32_t v) { put16(p, v); put16(p + 2, v >> 16); }

static void trace(fw_t *f, int kind, int32_t a, int32_t b, int32_t c) {
    if (f->hal.trace) f->hal.trace(f->hal.ctx, kind, f->now, a, b, c);
}

static void send(fw_t *f, uint8_t id, const uint8_t *payload, size_t n) {
    uint8_t out[72];
    if (!f->tx_started) {  /* resync byte: a receiver ignores everything up to the first 0x00 */
        f->tx_started = 1;
        out[0] = 0;
        f->hal.tx(f->hal.ctx, out, 1);
    }
    size_t m = fw_frame_encode(id, f->tx_seq++, payload, n, out);
    f->hal.tx(f->hal.ctx, out, m);
}

void fw_event(fw_t *f, uint8_t code, int32_t arg) {
    uint8_t p[5] = {code};
    trace(f, TR_EVENT, code, arg, 0);
    put32(p + 1, (uint32_t)arg);
    if (f->hello_done) send(f, ID_EVENT, p, 5);
}

static void ack_raw(fw_t *f, uint8_t cmd_id, uint8_t cmd_seq, uint8_t result, uint8_t info) {
    uint8_t p[4] = {cmd_id, cmd_seq, result, info};
    send(f, ID_ACK, p, 4);
}

static void ack(fw_t *f, const fw_frame_t *fr, uint8_t result, uint8_t info) {
    if (result) trace(f, TR_REFUSAL, fr->id, result, 0);
    ack_raw(f, fr->id, fr->seq, result, info);
}

/* ---- helpers --------------------------------------------------------------------------------------------------------------------------- */
static int armed_state(const fw_t *f) { return f->state == ST_ARMED || f->state == ST_ENGAGING || f->state == ST_FIRING; }
static int rail_on(const fw_t *f) { return f->key_on && !f->estop_open; }
static int hb_ok(const fw_t *f) { return f->hello_done && f->has_hb && MS_SINCE(f, f->last_hb_ms) <= (uint32_t)f->cfg[CFG_WD_MS]; }
static int deadman_ok(const fw_t *f) { return hb_ok(f) && (f->hb_flags & HB_DEADMAN_OK); }
static int inhibit(const fw_t *f) { return (f->hb_flags & HB_FIRE_INHIBIT) != 0; }

static void steppers(fw_t *f, int on) {
    if (on == f->steppers_enabled) return;
    f->steppers_enabled = (uint8_t)on;
    if (!on)
        for (int i = 0; i < 2; i++) f->ax[i].v = 0.0;
}

static void cut(fw_t *f) {  /* flywheel PWM to 0 and the pusher back, in the same tick */
    f->fw_req = f->fw = 0.0;
    f->balls_left = 0;
    f->phase = 0;
    f->pusher_ext = 0;
}

static void to_state(fw_t *f, uint8_t state, uint8_t reason, int quiet) {
    const uint8_t old = f->state;
    f->state = state;
    if (!armed_state(f)) {
        cut(f);
        f->shot_cap = 0;
    }
    if ((state == ST_SAFE || state == ST_FAULT) && (reason == R_ESTOP || reason == R_FAULT)) {  /* motion without drive is not tracked */
        steppers(f, 0);
        for (int i = 0; i < 2; i++) {
            f->ax[i].homed = 0;
            f->ax[i].homing = 0;
            f->ax[i].v = 0.0;
        }
    }
    if (state == ST_SAFE && (reason == R_WATCHDOG || reason == R_KEY_OFF || reason == R_NEW_SESSION))
        for (int i = 0; i < 2; i++) fw_axis_hold(&f->ax[i]);
    if (state == ST_IDLE) steppers(f, 1);
    if (state == ST_SAFE) f->hb_run = 0;  /* back to IDLE only after a fresh second of heartbeats */
    if (state != old) {
        trace(f, TR_TRANSITION, state, reason, 0);
        if (!quiet) fw_event(f, EV_STATE_CHANGE, state | (reason << 8));
    }
}

/* ---- commands -------------------------------------------------------------------------------------------------------------------------- */
static void cmd_hello(fw_t *f, const fw_frame_t *fr) {
    const uint8_t *p = fr->payload;
    const int res = p[0] == FW_PROTO_MAJOR ? 0 : 1;
    uint8_t img[FW_CFG_IMAGE_SIZE], a[18];
    if (res) {
        f->version_bad = 1;
    } else {
        f->version_bad = 0;
        const uint32_t session = get32(p + 2);
        if (!f->session_valid || f->session != session) {
            f->session_valid = 1;
            f->session = session;
            f->shots_arm = 0;
            f->hb_run = 0;
            f->has_hb = 0;
            if (f->state != ST_SAFE) to_state(f, ST_SAFE, R_NEW_SESSION, 0);
        }
        f->hello_done = 1;
    }
    fw_cfg_pack(f->flash_cfg, img);
    a[0] = FW_PROTO_MAJOR; a[1] = FW_PROTO_MINOR; a[2] = (uint8_t)res;
    a[3] = FW_VER_MAJOR; a[4] = FW_VER_MINOR; a[5] = FW_VER_PATCH;
    memcpy(a + 6, f->uid, 8);
    memcpy(a + 14, img + FW_CFG_IMAGE_SIZE - 4, 4);  /* cfg_crc = the CRC-32 of the stored config image */
    send(f, ID_HELLO_ACK, a, 18);
    if (!res && f->boot_note_len && !f->boot_note_sent) {
        f->boot_note_sent = 1;
        send(f, ID_DEBUG, f->boot_note, f->boot_note_len);
    }
}

static void cmd_heartbeat(fw_t *f, const fw_frame_t *fr) {
    const uint32_t wd = (uint32_t)f->cfg[CFG_WD_MS];
    if (f->has_hb && MS_SINCE(f, f->last_hb_ms) <= wd) {
        if (f->hb_run == 0) f->hb_run_t0 = f->now;
        if (f->hb_run < 0xFFFF) f->hb_run++;
    } else {
        f->hb_run = 1;
        f->hb_run_t0 = f->now;
    }
    f->last_hb_ms = f->now;
    f->has_hb = 1;
    f->hb_flags = fr->payload[2];
}

static void cmd_aim(fw_t *f, const fw_frame_t *fr) {
    const uint8_t *p = fr->payload;
    if (f->state == ST_SAFE || f->state == ST_FAULT) return ack(f, fr, RES_BAD_STATE, 0);
    const uint16_t vmax[2] = {get16(p + 12), get16(p + 14)}, amax[2] = {get16(p + 16), get16(p + 18)};
    if (!vmax[0] || !vmax[1] || !amax[0] || !amax[1]) return ack(f, fr, RES_BAD_ARG, 0);
    const int32_t tgt[2] = {(int32_t)get32(p), (int32_t)get32(p + 4)};
    const int16_t vff[2] = {(int16_t)get16(p + 8), (int16_t)get16(p + 10)};
    for (int i = 0; i < 2; i++) {
        fw_axis_t *a = &f->ax[i];
        int32_t lo = f->cfg[i ? CFG_TILT_MIN : CFG_PAN_MIN], hi = f->cfg[i ? CFG_TILT_MAX : CFG_PAN_MAX], t = tgt[i];
        t = t < lo ? lo : (t > hi ? hi : t);
        a->target = t / 1000.0;
        a->vff = vff[i] / 10.0;
        a->vmax = vmax[i] / 10.0;
        a->amax = amax[i];
        a->t_aim_ms = f->now;
        a->has_aim = 1;
    }
}

static void cmd_home(fw_t *f, const fw_frame_t *fr) {
    const uint8_t mask = fr->payload[0];
    if (f->estop_open) return ack(f, fr, RES_INTERLOCK, 0);
    if (f->state != ST_IDLE) return ack(f, fr, RES_BAD_STATE, 0);
    if (!(mask & 3)) return ack(f, fr, RES_BAD_ARG, 0);
    for (int i = 0; i < 2; i++)
        if (mask >> i & 1) fw_homing_start(f, i);
    ack_raw(f, fr->id, fr->seq, RES_OK, mask & 3);
}

/* FLYWHEEL: speed_permille (fraction of rpm_max, 0 = off) + optional tail diff_pm:i16 (overrides cfg 29, clamped +-60). The request is refreshed at 10 Hz and
 * zeroed after 500 ms without refresh (timers()). */
static void cmd_flywheel(fw_t *f, const fw_frame_t *fr) {
    const uint16_t speed = get16(fr->payload);
    if (speed > 1000) return ack(f, fr, RES_BAD_ARG, 0);
    if (fr->len >= 4) {
        int32_t d = (int16_t)get16(fr->payload + 2);
        f->diff_ovr = (int16_t)(d < -FDR_MAX_DIFF_PM ? -FDR_MAX_DIFF_PM : (d > FDR_MAX_DIFF_PM ? FDR_MAX_DIFF_PM : d));
        f->diff_ovr_valid = 1;
    } else {
        f->diff_ovr_valid = 0;
    }
    if (speed == 0) {
        f->fw_req = 0.0;
        if (f->state == ST_ENGAGING) to_state(f, ST_ARMED, R_NONE, 0);
        return;
    }
    if (!rail_on(f)) return ack(f, fr, RES_INTERLOCK, 0);
    if (!armed_state(f)) return ack(f, fr, RES_BAD_STATE, 0);
    if (speed > f->fw_req) f->fw_req_t = f->now;
    f->fw_req = speed;
    f->fw_refresh_t = f->now;
    if (f->state == ST_ARMED) to_state(f, ST_ENGAGING, R_NONE, 0);
}

/* The planner position (no encoder) inside the fire sector (cfg 20, 21 pan; 17, 22 tilt, mdeg). Checked at FIRE and on every tick of the pusher stroke. */
static int in_sector(const fw_t *f) {
    const double pan = f->ax[0].pos * 1000.0, tilt = f->ax[1].pos * 1000.0;
    return pan >= f->cfg[CFG_FIRE_PAN_MIN] && pan <= f->cfg[CFG_FIRE_PAN_MAX] && tilt >= f->cfg[CFG_FIRE_TILT_MIN] && tilt <= f->cfg[CFG_FIRE_TILT_MAX];
}

/* FIRE gates owned by the safety FSM, in v1's order; the launcher/feeder gates (JAMMED, BUSY, NOT_CLEAR, TOO_SOON, NOT_SPUN, NO_BALL) follow in fdr_step(). */
static uint8_t fire_precheck(const fw_t *f, uint8_t n) {
    if (f->state == ST_FIRING || f->fire_cmd_pending) return RES_BUSY;
    if (f->state != ST_ENGAGING) return RES_BAD_STATE;
    if (n != 1) return RES_BAD_ARG;  /* one ball per FIRE (I9b) */
    if (f->fault_bits) return RES_JAMMED;  /* any latched fault (also one set by fw_rx earlier in this very tick, e.g. USB overflow) */
    if (!rail_on(f)) return RES_INTERLOCK;
    if (!hb_ok(f) || !deadman_ok(f) || inhibit(f)) return RES_INHIBIT;
    if (!f->ax[0].homed || !f->ax[1].homed) return RES_NOT_HOMED;
    if (f->sector_set != SECTOR_ALL) return RES_NO_SECTOR;
    if (!in_sector(f)) return RES_RANGE;
    if (f->shots_arm + 1 > f->shot_cap) return RES_CAP;
    return RES_OK;
}

static void cmd_fire(fw_t *f, const fw_frame_t *fr) {
    const uint16_t id = get16(fr->payload);
    const uint8_t n = fr->payload[2];
    for (int i = 0; i < 8; i++)
        if (f->fire_ids[i].used && f->fire_ids[i].id == id) {  /* retransmission: the old answer, nothing shot */
            ack_raw(f, fr->id, fr->seq, f->fire_ids[i].res, 0);
            return;
        }
    const uint8_t res = fire_precheck(f, n);
    f->fire_ids[f->fire_ids_head].id = id;
    f->fire_ids[f->fire_ids_head].res = res ? res : RES_BUSY;  /* provisional: a retransmission in the same tick must not hear OK before the feeder has decided (feeder_stage() fixes it up) */
    f->fire_ids[f->fire_ids_head].used = 1;
    f->fire_ids_head = (f->fire_ids_head + 1) & 7;
    if (res) return ack(f, fr, res, 0);
    f->fire_cmd_pending = 1;  /* the feeder decides in this tick's feeder_stage(), which sends the ACK */
    f->fire_pending_id = id;
    f->fire_pending_seq = fr->seq;
}

static void cmd_arm(fw_t *f, const fw_frame_t *fr) {
    const uint8_t mode = fr->payload[0], cap = fr->payload[1];
    const uint16_t timeout_s = get16(fr->payload + 2);
    if (mode == 0) {
        if (armed_state(f)) to_state(f, ST_IDLE, R_DISARM, 0);
        return ack(f, fr, RES_OK, 0);
    }
    if (mode != 1 || cap == 0 || timeout_s == 0) return ack(f, fr, RES_BAD_ARG, 0);
    if (f->state != ST_IDLE) return ack(f, fr, RES_BAD_STATE, 0);
    if (f->estop_open || !f->key_on) return ack(f, fr, RES_INTERLOCK, 0);
    if (f->fault_bits) return ack(f, fr, RES_BAD_STATE, 0);
    if (!hb_ok(f) || !deadman_ok(f)) return ack(f, fr, RES_INHIBIT, 0);
    if (!f->ax[0].homed || !f->ax[1].homed) return ack(f, fr, RES_NOT_HOMED, 0);
    if (f->sector_set != SECTOR_ALL) return ack(f, fr, RES_NO_SECTOR, 0);  /* the host must write the fire sector every boot */
    const int32_t hcap = f->cfg[CFG_HARD_SHOT_CAP], hto = f->cfg[CFG_HARD_ARM_TIMEOUT];
    f->shot_cap = (uint8_t)(cap < hcap ? cap : (hcap < 0 ? 0 : hcap));
    f->arm_deadline = f->now + 1000u * (uint32_t)(timeout_s < hto ? timeout_s : (hto < 0 ? 0 : hto));
    f->shots_arm = 0;
    f->has_deadman_low = 0;
    to_state(f, ST_ARMED, R_OPERATOR_ARM, 0);  /* (hb_run is only reset on entering SAFE) */
    ack(f, fr, RES_OK, f->shot_cap);
}

static int cfg_valid(int key, int32_t v) {
    switch (key) {
    case CFG_PAN_SPR: case CFG_TILT_SPR: return v > 0;
    case CFG_DIR_INVERT: return v >= 0 && v <= 3;
    case CFG_HARD_SHOT_CAP: return v >= 0 && v <= 255;
    case CFG_HARD_ARM_TIMEOUT: return v >= 0 && v <= 65535;
    case CFG_PUSHER_EXT_US: case CFG_PUSHER_RET_US: case CFG_GATE_LOAD_US: case CFG_GATE_BORE_US: return v >= 500 && v <= 2500;  /* servo pulse sanity */
    case CFG_PUSHER_DWELL_MS: return v >= 50 && v <= 130;  /* dwell + the 118 ms retract must fit the limiter's 254 ms power window (wiring.md) */
    case CFG_MIN_SHOT_INTERVAL_MS: case CFG_SPINUP_MS: return v >= 0 && v <= 60000;
    case CFG_HOMING_SPEED: return v > 0 && v <= 3000;  /* <= 300 deg/s */
    case CFG_WHEEL_D_UM: return v >= 10000 && v <= 100000;
    case CFG_V_CAP_DMPS: return v >= 1 && v <= 380;  /* the 38 m/s cap can only be lowered */
    case CFG_RPM_MAX: case CFG_RPM_IDLE: return v >= 0 && v <= 60000;
    case CFG_RPM_WINDOW_PM: return v >= 1 && v <= 20;   /* <= 2 %: a wider window would let a ball go with the wheels far from the setpoint (I12) */
    case CFG_SETTLE_MS: return v >= 20 && v <= 2000;    /* >= 20 ms: telemetry age is limited to 20 ms, a shorter settle verifies nothing */
    case CFG_DIFF_PM: return v >= -100 && v <= 100;
    case CFG_POLE_PAIRS: return v >= 1 && v <= 32;
    case CFG_RAMP_RPM_S: return v >= 1000 && v <= 400000;
    case CFG_THROTTLE_MAX_PM: return v >= 100 && v <= 1000;
    case CFG_KP: case CFG_KI: return v >= 0 && v <= 100000;
    case CFG_FILL_MS: return v >= 0 && v <= 1000;
    case CFG_EXIT_TIMEOUT_MS: return v >= 100 && v <= 1000;
    case CFG_TELEMETRY_LOSS_MS: return v >= 10 && v <= 1000;
    case CFG_K_BEAM: return v >= 100 && v <= 3000;
    case CFG_HW_COUNTER_N: return v >= 0 && v <= 255;
    default: return 1;
    }
}

static void cmd_cfg_set(fw_t *f, const fw_frame_t *fr) {
    const uint8_t key = fr->payload[0];
    int32_t v = (int32_t)get32(fr->payload + 1);
    if (f->state != ST_SAFE && f->state != ST_IDLE) return ack(f, fr, RES_BAD_STATE, 0);
    if (key < 1 || key >= CFG_COUNT) return ack(f, fr, RES_BAD_ARG, 0);
    if ((key == CFG_HARD_SHOT_CAP || key == CFG_HARD_ARM_TIMEOUT || key == CFG_WHEEL_D_UM || key == CFG_V_CAP_DMPS || key == CFG_HW_COUNTER_N) && f->state != ST_SAFE)
        return ack(f, fr, RES_BAD_STATE, 0);  /* maintenance act */
    if (key == CFG_WD_MS) v = v < 1 ? 1 : (v > 300 ? 300 : v);
    if (!cfg_valid(key, v)) return ack(f, fr, RES_BAD_ARG, 0);
    if (key == CFG_MIN_SHOT_INTERVAL_MS && v < FDR_MIN_INTERVAL_FLOOR) v = FDR_MIN_INTERVAL_FLOOR;  /* firmware clamp (I9b, I14) */
    f->cfg[key] = v;
    if (key == CFG_FIRE_TILT_MIN) f->sector_set |= SECTOR_17;
    else if (key == CFG_FIRE_PAN_MIN) f->sector_set |= SECTOR_20;
    else if (key == CFG_FIRE_PAN_MAX) f->sector_set |= SECTOR_21;
    else if (key == CFG_FIRE_TILT_MAX) f->sector_set |= SECTOR_22;
    if (key == CFG_PAN_SPR || key == CFG_TILT_SPR) {  /* the position scale changed: the old home no longer holds (firmware addition) */
        const int i = key == CFG_TILT_SPR;
        f->ax[i].homed = 0;
        f->ax[i].emitted = (int32_t)lround(f->ax[i].pos * fw_spd(f, i));
    }
    ack(f, fr, RES_OK, 0);
}

static void cmd_cfg_get(fw_t *f, const fw_frame_t *fr) {
    const uint8_t key = fr->payload[0];
    uint8_t p[5] = {key};
    if (key < 1 || key >= CFG_COUNT) return ack(f, fr, RES_BAD_ARG, 0);
    put32(p + 1, (uint32_t)f->cfg[key]);
    send(f, ID_CFG_VAL, p, 5);
}

static void cmd_cfg_save(fw_t *f, const fw_frame_t *fr) {
    uint8_t img[FW_CFG_IMAGE_SIZE];
    if (f->state != ST_SAFE && f->state != ST_IDLE) return ack(f, fr, RES_BAD_STATE, 0);
    fw_cfg_pack(f->cfg, img);
    if (f->hal.flash_write && f->hal.flash_write(f->hal.ctx, img, sizeof img) != 0) return ack(f, fr, RES_BUSY, 0);
    memcpy(f->flash_cfg, f->cfg, sizeof f->cfg);
    ack(f, fr, RES_OK, 0);
}

static void cmd_clear_fault(fw_t *f, const fw_frame_t *fr) {
    if (f->state != ST_SAFE && f->state != ST_FAULT) return ack(f, fr, RES_BAD_STATE, 0);
    if (f->estop_open) return ack(f, fr, RES_INTERLOCK, 0);
    f->fault_bits = 0;
    f->faults_reported = 0;
    f->diag_ms = 0;
    f->clear_pending = 1;  /* also clears the jam / no_exit latches in the feeder (it re-homes the gate) */
    if (f->state == ST_FAULT) to_state(f, ST_SAFE, R_DISARM, 0);
    ack(f, fr, RES_OK, 0);
}

static void cmd_bootsel(fw_t *f, const fw_frame_t *fr) {
    if (f->state == ST_SAFE && get32(fr->payload) == 0xB007B007u) f->out.request_bootsel = 1;  /* platform: reset into the ROM USB bootloader */
    else ack(f, fr, f->state != ST_SAFE ? RES_BAD_STATE : RES_BAD_ARG, 0);
}

static const uint8_t PAYLOAD_MIN[0x10] = {0, 6, 3, 20, 1, 2, 3, 4, 5, 1, 0, 0, 0, 0, 0, 4};

static void handle(fw_t *f, const fw_frame_t *fr) {
    const int known = fr->id <= 0x0F && (fr->id == ID_BOOTSEL || (fr->id >= 1 && fr->id <= 0x0B));
    if (!known) return ack(f, fr, RES_UNKNOWN_CMD, 0);
    if (fr->len < PAYLOAD_MIN[fr->id]) return ack(f, fr, RES_BAD_ARG, 0);
    if (fr->id == ID_HELLO) return cmd_hello(f, fr);
    if (!f->hello_done || f->version_bad) return ack(f, fr, RES_BAD_STATE, 0);
    switch (fr->id) {
    case ID_HEARTBEAT: cmd_heartbeat(f, fr); break;
    case ID_AIM: cmd_aim(f, fr); break;
    case ID_HOME: cmd_home(f, fr); break;
    case ID_FLYWHEEL: cmd_flywheel(f, fr); break;
    case ID_FIRE: cmd_fire(f, fr); break;
    case ID_ARM: cmd_arm(f, fr); break;
    case ID_CFG_SET: cmd_cfg_set(f, fr); break;
    case ID_CFG_GET: cmd_cfg_get(f, fr); break;
    case ID_CFG_SAVE: cmd_cfg_save(f, fr); break;
    case ID_CLEAR_FAULT: cmd_clear_fault(f, fr); break;
    case ID_BOOTSEL: cmd_bootsel(f, fr); break;
    }
}

/* ---- per-tick stages ------------------------------------------------------------------------------------------------------------------- */
static void rx_drain(fw_t *f) {
    fw_frame_t fr;
    int any = 0;
    while (f->rxq_tail != f->rxq_head) {
        const uint8_t b = f->rxq[f->rxq_tail];
        f->rxq_tail = (uint16_t)((f->rxq_tail + 1) % sizeof f->rxq);
        any = 1;
        if (fw_decoder_byte(&f->dec, b, &fr)) handle(f, &fr);
    }
    const uint32_t bad = f->dec.bad_frames - f->bad_reported;
    if (any && bad && (!f->has_bad_event || MS_SINCE(f, f->bad_event_ms) >= 1000)) {
        f->bad_reported = f->dec.bad_frames;
        f->bad_event_ms = f->now;
        f->has_bad_event = 1;
        fw_event(f, EV_BAD_FRAMES, (int32_t)bad);
    }
}

static void inputs(fw_t *f) {
    const fw_in_t *in = &f->in;
    if (!f->inputs_init) {  /* the first sample is the initial condition, not an edge */
        f->inputs_init = 1;
        f->estop_open = in->estop_open ? 1 : 0;
        f->key_on = in->key_on ? 1 : 0;
        f->estop_closed_ms = f->estop_open ? 0 : DEBOUNCE_MS;
        f->key_on_ms = f->key_on ? DEBOUNCE_MS : 0;
    }
    if (in->estop_open) f->estop_closed_ms = 0;
    else if (f->estop_closed_ms < DEBOUNCE_MS) f->estop_closed_ms++;
    if (!in->key_on) f->key_on_ms = 0;
    else if (f->key_on_ms < DEBOUNCE_MS) f->key_on_ms++;
    const uint8_t e = f->estop_closed_ms < DEBOUNCE_MS, k = f->key_on_ms >= DEBOUNCE_MS;
    if (e != f->estop_open) {
        f->estop_open = e;
        fw_event(f, EV_ESTOP_EDGE, e);
        if (e) {
            f->fw_req = f->fw = 0.0;
            if (f->state != ST_SAFE) {
                to_state(f, ST_SAFE, R_ESTOP, 0);
            } else {
                steppers(f, 0);
                for (int i = 0; i < 2; i++) f->ax[i].homed = f->ax[i].homing = 0;
            }
        }
    }
    if (k != f->key_on) {
        f->key_on = k;
        fw_event(f, EV_KEY_EDGE, k);
        if (!k && f->state != ST_SAFE) to_state(f, ST_SAFE, R_KEY_OFF, 0);
    }
    if (!rail_on(f)) {  /* hardware: the rail is open, nothing downstream of the key + E-stop has power */
        f->fw = f->fw_req = 0.0;
        f->pusher_ext = 0;
        f->balls_left = 0;
        f->phase = 0;
    }
}

static void watchdog(fw_t *f) {
    const uint32_t wd = (uint32_t)f->cfg[CFG_WD_MS];
    if (f->hello_done && f->has_hb && MS_SINCE(f, f->last_hb_ms) > wd) {
        f->hb_run = 0;
        if (f->state == ST_IDLE || armed_state(f)) {
            f->wd_tripped = 1;
            fw_event(f, EV_WATCHDOG, (int32_t)MS_SINCE(f, f->last_hb_ms));
            to_state(f, ST_SAFE, R_WATCHDOG, 0);
        }
    }
    if (f->state == ST_ARMED || f->state == ST_ENGAGING || f->state == ST_FIRING) {
        if (f->shots_arm > f->shot_cap) f->fault_bits |= FAULT_SHOT_COUNTER;  /* internal assertion */
    }
    if (f->in.diag && f->steppers_enabled) {
        if (f->diag_ms < 255) f->diag_ms++;
        if (f->diag_ms > DIAG_FAULT_MS) f->fault_bits |= FAULT_DRIVER_DIAG;
    } else {
        f->diag_ms = 0;
    }
    const uint16_t fresh = f->fault_bits & ~f->faults_reported;
    if (fresh) {
        for (int b = 0; b < 6; b++)
            if (fresh >> b & 1) fw_event(f, EV_FAULT, b);
        f->faults_reported |= fresh;
    }
    if (f->fault_bits && f->state != ST_FAULT) to_state(f, ST_FAULT, R_FAULT, 0);
    if (f->state == ST_SAFE && f->hello_done && !f->version_bad && !f->fault_bits && !f->estop_open && f->hb_run >= 20 &&
        MS_SINCE(f, f->hb_run_t0) >= 1000) {
        f->wd_tripped = 0;
        to_state(f, ST_IDLE, R_LINK_UP, 0);
    }
}

static void timers(fw_t *f) {
    if (armed_state(f) && (int32_t)(f->now - f->arm_deadline) >= 0) to_state(f, ST_IDLE, R_ARM_TIMEOUT, 0);
    if ((f->state == ST_ENGAGING || f->state == ST_FIRING) && f->fw_req > 0 && MS_SINCE(f, f->fw_refresh_t) > FW_REFRESH_MS) {
        f->fw_req = 0.0;
        if (f->state == ST_ENGAGING) to_state(f, ST_ARMED, R_NONE, 0);
    }
    if (armed_state(f)) {
        if (deadman_ok(f)) {
            f->has_deadman_low = 0;
        } else {
            if (!f->has_deadman_low) {
                f->has_deadman_low = 1;
                f->deadman_low_since = f->now;
            }
            if (MS_SINCE(f, f->deadman_low_since) > DEADMAN_LOW_MS && f->fw_req > 0) {
                f->fw_req = 0.0;
                if (f->state == ST_ENGAGING) to_state(f, ST_ARMED, R_NONE, 0);
            }
        }
    }
}

/* ---- cuas2: hardware shot counter, launcher + feeder ----------------------------------------------------------------------------------- */
/* Hardware shot counter (two 74HCT4017, DIP-set N, reset only by the key's power cycle): the Nth muzzle-beam pulse trips a clamp on the gate of the supervisor's
 * weapon MOSFET. Relay coil, key and E-stop stay closed, so GP10 sees NOTHING (docs/cuas2/wiring.md 3.2/3.4): firmware sees weapon power lost with the key on and
 * the E-stop closed (both ESC telemetry streams drop together). The trip is not sensed, only inferred: when the launcher raises TELEMETRY_LOST with both streams
 * lost within FDR_HWC_SKEW_MS of each other, key on, E-stop closed, and the per-key-session beam pulse count has reached cfg 41, label it EVENT 16 + fault bit b15
 * (feeder_stage). Anything else is an ordinary telemetry_lost fault. Both stop the weapon (the launcher drops its target on any feeder fault). This function only
 * mirrors the count: the key resets it as it resets the hardware (a count per key-on session, not per ARM). */
static void hw_counter_note(fw_t *f) {
    if (!f->key_on) f->muz_key = 0;
    else if (f->in.beam_muzzle && !f->muz_prev_hw && f->muz_key < 0xFFFF) f->muz_key++;  /* every beam pulse, as the 4017 counts them (also unexpected ones) */
    f->muz_prev_hw = f->in.beam_muzzle;
}

static void finish_burst(fw_t *f) {  /* the cycle is back to STAGED / EMPTY: end FIRING */
    if (f->shots_arm >= f->shot_cap) to_state(f, ST_IDLE, R_SHOT_CAP, 0);
    else to_state(f, f->fw_req > 0 ? ST_ENGAGING : ST_ARMED, R_NONE, 0);  /* ENGAGING needs a flywheel request (spec 3.1) */
}

static void feeder_stage(fw_t *f) {
    fdr_in_t in;
    memset(&in, 0, sizeof in);
    in.t_ms = f->now;
    in.rpm_top = f->in.rpm[0];
    in.rpm_bot = f->in.rpm[1];
    in.beam_breech = f->in.beam_breech;
    in.beam_muzzle = f->in.beam_muzzle;
    in.gate_home = f->in.gate_home;
    in.push_home = f->in.push_home;
    in.estop_open = f->estop_open;
    in.key_on = f->key_on;
    in.hb_ok = (uint8_t)hb_ok(f);
    in.deadman = (uint8_t)deadman_ok(f);
    in.inhibit = (uint8_t)inhibit(f);
    in.in_sector = (uint8_t)in_sector(f);
    if (f->fire_cmd_pending && f->fault_bits) {  /* a fault latched after the precheck, before this stage (e.g. in rx_drain / inputs of this tick): refuse here as well */
        f->fire_cmd_pending = 0;
        for (int i = 0; i < 8; i++)
            if (f->fire_ids[i].used && f->fire_ids[i].id == f->fire_pending_id) f->fire_ids[i].res = RES_JAMMED;
        ack_raw(f, ID_FIRE, f->fire_pending_seq, RES_JAMMED, 0);
    }
    in.cmd = f->fire_cmd_pending ? FDR_CMD_FIRE : (f->clear_pending ? FDR_CMD_CLEAR : FDR_CMD_NONE);
    in.speed_req_pm = (uint16_t)((rail_on(f) && armed_state(f)) ? lround(f->fw_req) : 0);
    in.diff_pm = (int16_t)(f->diff_ovr_valid ? f->diff_ovr : f->cfg[CFG_DIFF_PM]);
    in.fire_id = f->fire_pending_id;
    in.muzzle_dt_us = f->in.muzzle_dt_us;
    in.cfg = f->cfg;
    f->clear_pending = 0;
    fdr_step(&f->fdr, &in, &f->fo);
    const fdr_out_t *o = &f->fo;

    if ((o->fault_bits & FAULT_TELEMETRY_LOST) && !(f->fault_bits & FAULT_TELEMETRY_LOST) && f->key_on && !f->estop_open && f->cfg[CFG_HW_COUNTER_N] > 0 &&
        f->muz_key >= f->cfg[CFG_HW_COUNTER_N] && (f->fdr.age[0] > f->fdr.age[1] ? f->fdr.age[0] - f->fdr.age[1] : f->fdr.age[1] - f->fdr.age[0]) <= FDR_HWC_SKEW_MS) {
        f->fault_bits |= FAULT_HW_COUNTER;  /* latched until CLEAR_FAULT */
        fw_event(f, EV_HW_COUNTER, f->muz_key);
    }
    f->fault_bits |= (uint16_t)(o->fault_bits & FDR_FAULT_MASK);
    f->faults_reported |= (uint16_t)(o->fault_bits & FDR_FAULT_MASK);  /* the feeder reports its own faults (EVENT FAULT / JAM in o->ev) */
    if (f->shots_arm == 0) f->conf_arm = 0;

    if (f->fire_cmd_pending) {  /* answer the FIRE the feeder just decided */
        const uint8_t res = o->ack_valid ? o->ack : RES_BAD_STATE;
        f->fire_cmd_pending = 0;
        for (int i = 0; i < 8; i++)
            if (f->fire_ids[i].used && f->fire_ids[i].id == f->fire_pending_id) f->fire_ids[i].res = res;
        if (res) trace(f, TR_REFUSAL, ID_FIRE, res, 0);
        ack_raw(f, ID_FIRE, f->fire_pending_seq, res, 0);
        if (res == RES_OK) {
            f->last_fire_id = f->fire_pending_id;
            f->cur_fire_id = f->fire_pending_id;
            to_state(f, ST_FIRING, R_NONE, 0);
        }
    }

    int aborted = 0, sector = 0;
    for (int i = 0; i < o->nev; i++) {
        const int32_t arg = o->ev[i].arg;
        switch (o->ev[i].code) {
        case EV_SHOT:  /* the ball is committed to the nip */
            f->ball_idx = 0;
            f->shots_arm++;
            f->shots_total++;
            f->last_ball_ms = f->now;
            f->has_ball = 1;
            f->out.shots_fed++;
            trace(f, TR_SHOT, f->cur_fire_id, f->ball_idx, 0);
            break;
        case EV_MUZZLE: f->conf_arm++; break;
        case EV_FEED_ABORT: aborted = 1; break;
        case EV_SECTOR_ABORT: sector = 1; break;
        default: break;
        }
        fw_event(f, o->ev[i].code, arg);
    }
    if (aborted && f->state == ST_FIRING) {
        if (sector) f->fw_req = 0.0;  /* spec 1.3 row 25: ARMED, no flywheel request */
        finish_burst(f);
        if (sector && f->state == ST_ENGAGING) to_state(f, ST_ARMED, R_NONE, 0);
    } else if (f->state == ST_FIRING && !o->feed_busy) {
        finish_burst(f);
    }
    f->fw = o->throttle_mean_pm;
    f->pusher_ext = (uint8_t)(o->feed_busy && o->push_us == (uint16_t)f->cfg[CFG_PUSHER_EXT_US] && o->push_us != (uint16_t)f->cfg[CFG_PUSHER_RET_US]);
}

static int at_limit(const fw_t *f, int i) {
    const fw_axis_t *a = &f->ax[i];
    const double lo = f->cfg[i ? CFG_TILT_MIN : CFG_PAN_MIN] / 1000.0, hi = f->cfg[i ? CFG_TILT_MAX : CFG_PAN_MAX] / 1000.0;
    return (a->homed && fabs(a->pos - lo) < 0.01) || fabs(a->pos - hi) < 0.01;  /* (as the twin: homed gates only the lower limit) */
}

static void status(fw_t *f) {
    uint8_t p[45];
    const fdr_out_t *o = &f->fo;
    const fw_axis_t *a0 = &f->ax[0], *a1 = &f->ax[1];
    const uint8_t f1 = (uint8_t)(f->estop_open * F1_ESTOP | f->key_on * F1_KEY | a0->homed * F1_PAN_HOMED | a1->homed * F1_TILT_HOMED |
                                 a0->hall * F1_PAN_HALL | a1->hall * F1_TILT_HALL | f->wd_tripped * F1_WD_TRIPPED | hb_ok(f) * F1_HB_OK);
    const uint8_t f2 = (uint8_t)((fabs(a0->v) > 0.05) * F2_PAN_MOVING | (fabs(a1->v) > 0.05) * F2_TILT_MOVING | f->steppers_enabled * F2_STEPPERS_EN |
                                 f->pusher_ext * F2_PUSHER_EXT | inhibit(f) * F2_INHIBIT | ((f->hb_flags & HB_DEADMAN_OK) != 0) * F2_DEADMAN |
                                 at_limit(f, 0) * F2_PAN_LIMIT | at_limit(f, 1) * F2_TILT_LIMIT);
    uint32_t left = 0;
    if (armed_state(f) && (int32_t)(f->arm_deadline - f->now) > 0) left = (f->arm_deadline - f->now + 999) / 1000;
    put32(p, f->now);
    p[4] = f->state;
    p[5] = f1;
    p[6] = f2;
    put32(p + 7, (uint32_t)lround(a0->pos * fw_spd(f, 0)));
    put32(p + 11, (uint32_t)lround(a1->pos * fw_spd(f, 1)));
    put16(p + 15, (uint32_t)lround(f->fw));
    put16(p + 17, o->rpm_top == 0xFFFF || o->rpm_bot == 0xFFFF ? 0xFFFFu : ((uint32_t)o->rpm_top + o->rpm_bot) / 2);
    put16(p + 19, f->shots_total);
    p[21] = f->shots_arm;
    p[22] = armed_state(f) ? f->shot_cap : 0;
    put16(p + 23, f->fault_bits);
    put16(p + 25, left > 0xFFFF ? 0xFFFF : left);
    put16(p + 27, f->last_fire_id);
    put16(p + 29, o->rpm_top);
    put16(p + 31, o->rpm_bot);
    put16(p + 33, o->rpm_set);
    p[35] = o->beams;
    p[36] = o->gate_state;
    p[37] = (uint8_t)((o->ready_flags & ~RDY_READY) | ((o->ready_flags & RDY_READY) && f->state == ST_ENGAGING && f->shots_arm < f->shot_cap ? RDY_READY : 0));
    put16(p + 38, o->last_v_dmps);
    put16(p + 40, o->last_dip_pm);
    put16(p + 42, o->shots_confirmed);
    p[44] = o->feed_ms;
    send(f, ID_STATUS, p, 45);
}

static void outputs(fw_t *f) {
    fw_out_t *o = &f->out;
    const fdr_out_t *fo = &f->fo;
    o->steppers_en = f->steppers_enabled;
    o->dir_invert[0] = f->cfg[CFG_DIR_INVERT] & 1;
    o->dir_invert[1] = (f->cfg[CFG_DIR_INVERT] >> 1) & 1;
    o->fw_duty = (uint16_t)lround(f->fw);
    o->pusher_us = fo->push_us ? fo->push_us : (uint16_t)f->cfg[CFG_PUSHER_RET_US];
    o->gate_us = fo->gate_us ? fo->gate_us : (uint16_t)f->cfg[CFG_GATE_LOAD_US];
    o->dshot[0] = fo->dshot[0];
    o->dshot[1] = fo->dshot[1];
    o->feed_req = fo->feed_req;
    /* I15: KICK runs only while the main loop is alive (this function ran), no assertion failed and no fault is latched */
    o->kick_ok = (uint8_t)(fo->kick && !f->fault_bits && f->state <= ST_FAULT && (!armed_state(f) || f->shots_arm <= f->shot_cap) && f->fdr.muz_all <= f->fdr.feeds);
    if (o->kick_ok && f->now % 5 == 0) f->kick_level ^= 1;
    o->kick_level = f->kick_level;
    const uint32_t ph = f->now % 1000;
    o->led_red = f->state == ST_SAFE || f->state == ST_ENGAGING || f->state == ST_FIRING || (f->state == ST_FAULT && (ph / 250) % 2 == 0);
    o->led_green = f->state == ST_IDLE || f->state == ST_ENGAGING || f->state == ST_FIRING || (f->state == ST_ARMED && ph < 500);
    o->buzzer = (f->state == ST_ENGAGING || f->state == ST_FIRING) && (ph % 500) < 100;
}

/* ---- public ---------------------------------------------------------------------------------------------------------------------------- */
void fw_init(fw_t *f, const fw_hal_t *hal, const uint8_t uid[8], const uint8_t *flash_img, size_t flash_n) {
    memset(f, 0, sizeof *f);
    f->hal = *hal;
    memcpy(f->uid, uid, 8);
    fw_cfg_defaults(f->cfg);
    const int r = fw_cfg_unpack(flash_img, flash_n, f->cfg);
    if (r < 0) {
        fw_cfg_defaults(f->cfg);
        f->fault_bits = FAULT_CFG_INVALID;
        f->faults_reported = FAULT_CFG_INVALID;
    }
    memcpy(f->flash_cfg, f->cfg, sizeof f->cfg);
    f->state = ST_SAFE;
    f->out.pusher_us = (uint16_t)f->cfg[CFG_PUSHER_RET_US];
    f->out.gate_us = (uint16_t)f->cfg[CFG_GATE_LOAD_US];
    f->next_status = 0;
    fdr_init(&f->fdr);
    if (f->fault_bits) to_state(f, ST_FAULT, R_FAULT, 1);
    for (int i = 0; i < 2; i++) f->ax[i].emitted = 0;
}

void fw_set_boot_note(fw_t *f, const char *s) {
    size_t n = strlen(s);
    if (n > sizeof f->boot_note) n = sizeof f->boot_note;
    memcpy(f->boot_note, s, n);
    f->boot_note_len = (uint8_t)n;
}

void fw_rx(fw_t *f, const uint8_t *d, size_t n) {
    for (size_t i = 0; i < n; i++) {
        const uint16_t next = (uint16_t)((f->rxq_head + 1) % sizeof f->rxq);
        if (next == f->rxq_tail) {
            f->fault_bits |= FAULT_USB_OVERFLOW;
            return;
        }
        f->rxq[f->rxq_head] = d[i];
        f->rxq_head = next;
    }
}

void fw_tick(fw_t *f, uint32_t now_ms, const fw_in_t *in) {
    f->now = now_ms;
    f->in = *in;
    rx_drain(f);
    inputs(f);
    hw_counter_note(f);
    feeder_stage(f);
    watchdog(f);
    timers(f);
    fw_axes_tick(f);
    if (f->hello_done && (int32_t)(f->now - f->next_status) >= 0) {
        f->next_status += STATUS_PERIOD_MS;
        if ((int32_t)(f->now - f->next_status) >= 0) f->next_status = f->now + STATUS_PERIOD_MS;
        status(f);
    }
    outputs(f);
}
