/* cuas2 RP2040 firmware core: hardware-independent. A SUPERSET of firmware/cuas-lite-rp2040/include/fw_core.h (same field and enum names, so the lite
 * files compiled by path here -- cobs.c crc16.c planner.c homing.c -- build against this header). Adds: the launcher/feeder module (fdr_*), protocol 1.1,
 * the new fault bits, events, results and config keys 23-41 (docs/cuas2/spec.md sections 1.2, 1.3, 5, 6). No malloc, no globals.
 * SPDX-License-Identifier: BSD-3-Clause */
#ifndef FW_CORE_H
#define FW_CORE_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FW_PROTO_MAJOR 1
#define FW_PROTO_MINOR 1                 /* 1.1: payload tails and new codes only */
#define FW_VER_MAJOR 2                   /* HELLO_ACK.fw_major = 2: cuas2 firmware */
#define FW_VER_MINOR 0
#define FW_VER_PATCH 0
#define FW_MAX_PAYLOAD 60
#define FW_MAX_BODY 64

/* host -> MCU ids */
enum { ID_HELLO = 0x01, ID_HEARTBEAT = 0x02, ID_AIM = 0x03, ID_HOME = 0x04, ID_FLYWHEEL = 0x05, ID_FIRE = 0x06, ID_ARM = 0x07, ID_CFG_SET = 0x08,
       ID_CFG_GET = 0x09, ID_CFG_SAVE = 0x0A, ID_CLEAR_FAULT = 0x0B, ID_BOOTSEL = 0x0F };
/* MCU -> host ids */
enum { ID_HELLO_ACK = 0x81, ID_STATUS = 0x82, ID_ACK = 0x83, ID_CFG_VAL = 0x84, ID_EVENT = 0x85, ID_DEBUG = 0x86 };
enum { ST_SAFE, ST_IDLE, ST_ARMED, ST_ENGAGING, ST_FIRING, ST_FAULT };
enum { F1_ESTOP = 1, F1_KEY = 2, F1_PAN_HOMED = 4, F1_TILT_HOMED = 8, F1_PAN_HALL = 16, F1_TILT_HALL = 32, F1_WD_TRIPPED = 64, F1_HB_OK = 128 };
enum { F2_PAN_MOVING = 1, F2_TILT_MOVING = 2, F2_STEPPERS_EN = 4, F2_PUSHER_EXT = 8, F2_INHIBIT = 16, F2_DEADMAN = 32, F2_PAN_LIMIT = 64, F2_TILT_LIMIT = 128 };
enum { HB_DEADMAN_OK = 1, HB_FIRE_INHIBIT = 2 };
/* fault_bits: b0-b5 are v1's, b6-b14 new (spec 6.4), b15 = hw shot counter label (cuas2) */
enum { FAULT_HOMING_TIMEOUT = 1, FAULT_DRIVER_DIAG = 2, FAULT_CFG_INVALID = 4, FAULT_SHOT_COUNTER = 8, FAULT_HALL_STUCK = 16, FAULT_USB_OVERFLOW = 32,
       FAULT_TELEMETRY_LOST = 1 << 6, FAULT_OVERSPEED = 1 << 7, FAULT_WHEEL_NOT_READY = 1 << 8, FAULT_BREECH_DOUBLE = 1 << 9, FAULT_NO_EXIT = 1 << 10,
       FAULT_UNEXPECTED_EXIT = 1 << 11, FAULT_PUSHER_STUCK = 1 << 12, FAULT_GATE_STUCK = 1 << 13, FAULT_SHOT_OVERCOUNT = 1 << 14,
       FAULT_HW_COUNTER = 1 << 15 /* the hardware shot counter tripped (inferred from weapon power loss + beam pulse count, spec 1.3 row 30); not a feeder fault: does not enter FDR_FAULT_MASK */ };
#define FDR_FAULT_MASK 0x7FC0u            /* b6-b14: the launcher/feeder faults; each stops KICK (I15) */
#define FDR_JAM_MASK 0x7E00u              /* b9-b14: the feeder faults shown as gate_state JAM / ready_flags jam latch */
enum { EV_STATE_CHANGE = 1, EV_SHOT, EV_HOMED, EV_FAULT, EV_WATCHDOG, EV_ESTOP_EDGE, EV_KEY_EDGE, EV_BAD_FRAMES, EV_SECTOR_ABORT /* 9: arg = fire_id << 8 | ball index */,
       EV_MUZZLE /* 10: arg = v_dmps | dip_pm << 16 */, EV_FEED_ABORT /* 11: arg = fire_id << 8 | reason (FA_*) */, EV_EMPTY /* 12 */, EV_JAM /* 13: arg = gate_state */,
       EV_WHEELS_READY /* 14: arg = mean rpm */, EV_WHEELS_LOST /* 15 */,
       EV_HW_COUNTER /* 16 (cuas2 addition): both ESC telemetry streams dropped with key on, E-stop closed and the per-key-session pulse count at N: the hardware shot counter tripped (inferred); arg = pulses counted */ };
enum { FA_SECTOR = 1, FA_DEADMAN, FA_INHIBIT, FA_WHEELS, FA_BEAM, FA_RAIL };
enum { R_NONE, R_BOOT, R_WATCHDOG, R_ESTOP, R_KEY_OFF, R_DISARM, R_ARM_TIMEOUT, R_SHOT_CAP, R_FAULT, R_OPERATOR_ARM, R_LINK_UP, R_NEW_SESSION };
enum { RES_OK, RES_BAD_STATE, RES_INTERLOCK, RES_INHIBIT, RES_CAP, RES_RANGE, RES_BUSY, RES_UNKNOWN_CMD, RES_BAD_ARG, RES_NOT_HOMED, RES_NOT_SPUN, RES_NO_SECTOR,
       RES_NO_BALL /* 12: no staged ball */, RES_JAMMED /* 13: feeder lock-out */, RES_NOT_CLEAR /* 14: muzzle beam blocked */, RES_TOO_SOON /* 15: interval */ };
/* bits of fw_t.sector_set: one per sector key written (CFG_SET ok) since boot; SECTOR_ALL = the sector is set */
enum { SECTOR_17 = 1, SECTOR_20 = 2, SECTOR_21 = 4, SECTOR_22 = 8, SECTOR_ALL = 15 };
/* config keys. 1-22 are v1's (12-14 = pusher extend us / retract us / extend dwell ms; 15 clamps >= 400; 16 spinup_ms is stored but unused: key 28 is the window settle
 * time). 23-40 per spec 6.4 table; 41 hw_counter_n is a cuas2 addition (the DIP setting of the hardware shot counter, 0 = none fitted). */
enum { CFG_PAN_SPR = 1, CFG_TILT_SPR, CFG_PAN_MIN, CFG_PAN_MAX, CFG_TILT_MIN, CFG_TILT_MAX, CFG_PAN_HOME_OFF, CFG_TILT_HOME_OFF, CFG_DIR_INVERT,
       CFG_HARD_SHOT_CAP, CFG_HARD_ARM_TIMEOUT, CFG_PUSHER_EXT_US, CFG_PUSHER_RET_US, CFG_PUSHER_DWELL_MS, CFG_MIN_SHOT_INTERVAL_MS, CFG_SPINUP_MS,
       CFG_FIRE_TILT_MIN, CFG_WD_MS, CFG_HOMING_SPEED,
       CFG_FIRE_PAN_MIN, CFG_FIRE_PAN_MAX, CFG_FIRE_TILT_MAX,
       CFG_WHEEL_D_UM, CFG_V_CAP_DMPS, CFG_RPM_MAX, CFG_RPM_IDLE, CFG_RPM_WINDOW_PM, CFG_SETTLE_MS, CFG_DIFF_PM, CFG_POLE_PAIRS, CFG_RAMP_RPM_S, CFG_THROTTLE_MAX_PM,
       CFG_KP, CFG_KI, CFG_GATE_LOAD_US, CFG_GATE_BORE_US, CFG_FILL_MS, CFG_EXIT_TIMEOUT_MS, CFG_TELEMETRY_LOSS_MS, CFG_K_BEAM, CFG_HW_COUNTER_N,
       CFG_COUNT /* = 42: array size, index 0 unused */ };

/* trace kinds for the optional fw_hal_t.trace hook (tests; NULL on the Pico) */
enum { TR_TRANSITION = 1, TR_EVENT, TR_SHOT, TR_REFUSAL };

/* ---- crc16 / cobs / frame (lite cobs.c, crc16.c; cuas2 proto.c) ---------------------------------------------------------------------------- */
uint16_t fw_crc16(const uint8_t *d, size_t n);
uint32_t fw_crc32(const uint8_t *d, size_t n);                                      /* IEEE 802.3, for the config image */
size_t fw_cobs_encode(const uint8_t *in, size_t n, uint8_t *out);                   /* out >= n + n/254 + 1; no trailing 0x00 */
int fw_cobs_decode(const uint8_t *in, size_t n, uint8_t *out, size_t cap);          /* -> decoded length or -1 */
/* One frame on the wire: COBS(id | seq | payload | crc16 LE) 0x00. out >= 70 bytes. -> length. */
size_t fw_frame_encode(uint8_t id, uint8_t seq, const uint8_t *payload, size_t n, uint8_t *out);

typedef struct { uint8_t id, seq, len; uint8_t payload[FW_MAX_PAYLOAD]; } fw_frame_t;
typedef struct {
    uint8_t buf[82];
    uint8_t n, synced;
    uint32_t bad_frames, good_frames;
} fw_decoder_t;
/* Feed one byte; returns 1 when a good frame is complete in *f. Bytes before the first 0x00 are ignored (resync); bad frames are counted. */
int fw_decoder_byte(fw_decoder_t *d, uint8_t b, fw_frame_t *f);

/* ---- config (config.c) ------------------------------------------------------------------------------------------------------------------ */
#define FW_CFG_IMAGE_SIZE 176                                                       /* magic u32, version u32 (= 3), 41 x i32, crc32 */
void fw_cfg_defaults(int32_t cfg[CFG_COUNT]);
void fw_cfg_pack(const int32_t cfg[CFG_COUNT], uint8_t img[FW_CFG_IMAGE_SIZE]);
/* 0 = loaded, 1 = blank (all 0xFF / n == 0), -1 = invalid (bad magic / version / CRC: a v1 or v2 image is refused) */
int fw_cfg_unpack(const uint8_t *img, size_t n, int32_t cfg[CFG_COUNT]);

/* ---- launcher + feeder (launcher.c, feeder.c): the weapon section as one pure step function, mirrored in python/napl/cuas2/fsm.py ------------------ */
enum { GS_UNKNOWN, GS_HOMING, GS_LOADING, GS_TO_BORE, GS_STAGED, GS_PUSH, GS_EXIT_WAIT, GS_RETRACT, GS_TO_LOAD, GS_EMPTY, GS_JAM };  /* = STATUS.gate_state */
enum { FDR_CMD_NONE, FDR_CMD_FIRE, FDR_CMD_CLEAR };
enum { RDY_READY = 1, RDY_EMPTY = 2, RDY_JAM = 4, RDY_HOPPER_LOW = 8 };
enum { BM_BREECH = 1, BM_MUZZLE = 2, BM_GATE_HOME = 4, BM_PUSH_HOME = 8, BM_WHEELS_READY = 16, BM_FEED_BUSY = 32 };
#define FDR_DIFF_CFG 0x7FFF               /* fdr_in_t.diff_pm: use cfg 29 */
#define FDR_MAX_EV 12
/* timing constants of the feed cycle [est]; bench items B-TIME (spec 10). Milliseconds from the FEED start unless stated.
 * Geometry (MECH, docs/cuas2/assembly.md C-NIP / C-CYCLE): pusher stroke 47.2 mm at 0.4 mm/ms = 118 ms out, 118 ms back; the rod reaches the staged ball at about
 * half stroke (~65 ms), the breech beam (9 mm behind the plug axis, so it clears after only ~2.5 mm of ball travel) clears at ~71 ms, the ball reaches the nip at
 * ~101 ms and leaves the barrel ~15 ms later; the rod is home again at ~235 ms (limiter power window 254 ms) and the geometric cycle is 466 ms. */
#define FDR_COMMIT_MS 105                 /* the ball is in the nip and leaves regardless (spec 5.4); breech clear commits earlier */
#define FDR_PUSH_LEAVE_MS 40              /* push_home must have dropped by now, else pusher_stuck */
#define FDR_BREECH_EARLY_MS 40            /* the staged ball must not vanish from the breech before this (FEED_ABORT beam) */
#define FDR_BREECH_EXP_MS 75              /* expected breech clear time; > 1.3x -> breech_double */
#define FDR_BREECH_CLEAR_MAX_MS 150       /* breech must be clear by now, else no_exit (spec I11: 150 ms = the 118 ms out-stroke + 32 ms; a late clear is already breech_double at 98 ms, this
                                           * catches a ball that never leaves the breech) */
#define FDR_SECTOR_T_EXIT_MS 8            /* fire sector shrunk by |axis speed| * this (spec 5.4: the ball is in the barrel for t_exit after the commit) */
#define FDR_RETRACT_CLEAR_MS 118          /* added to cfg 14 (dwell = extended time, default 120): the gate may only turn after this (full retract stroke); the rod must also read home.
                                           * dwell + 118 must stay inside the limiter's 254 ms power window, hence cfg 14 <= 130 */
#define FDR_RETRACT_TIMEOUT_MS 330        /* push_home must be back by now, else pusher_stuck (just short of the limiter's 338 ms lockout) */
#define FDR_GATE_TRAVEL_MS 100            /* 60 deg of SG92R */
#define FDR_GATE_HOME_TIMEOUT_MS 400
#define FDR_HOMING_TIMEOUT_MS 1000
#define FDR_FEED_PULSE_MS 10              /* FEED_REQ high time (the limiter is edge triggered) */
#define FDR_WHEEL_AGE_MAX_MS 20           /* telemetry older than this does not count as in window (I12) */
#define FDR_HWC_SKEW_MS 5                 /* both telemetry streams lost within this of each other = weapon power lost (hw shot counter label) */
#define FDR_NOT_READY_MS 1500
#define FDR_OVERSPEED_MS 20
#define FDR_MIN_INTERVAL_FLOOR 470        /* firmware clamp of cfg 15: >= the 466 ms geometric cycle (C-CYCLE) and > the limiter's 338 ms lockout */
#define FDR_MUZ_MIN_GAP_MS 300
#define FDR_EMPTY_RETRIES 3
#define FDR_EMPTY_RETRY_MS 1000
#define FDR_MAX_DIFF_PM 60
#define FDR_DIP_WINDOW_MS 150

typedef struct {
    uint32_t t_ms;
    int32_t rpm_top, rpm_bot;             /* mechanical rpm from the eRPM reply; -1 = no valid reply this tick */
    uint8_t beam_breech, beam_muzzle, gate_home, push_home;
    uint8_t estop_open, key_on, hb_ok, deadman, inhibit, in_sector;
    uint8_t cmd;                          /* FDR_CMD_* */
    uint16_t speed_req_pm;
    int16_t diff_pm;                      /* flywheel differential, permille of the mean (clamped +-60) */
    uint16_t fire_id;
    uint32_t muzzle_dt_us;                /* width of the last muzzle pulse from the 1 us capture, 0 = none */
    const int32_t *cfg;                   /* CFG_COUNT entries */
} fdr_in_t;

typedef struct { uint8_t code; int32_t arg; } fdr_ev_t;
typedef struct {
    uint16_t dshot[2];                    /* 0 = no signal / disarmed; 48..2047 = throttle */
    uint16_t gate_us, push_us;
    uint8_t feed_req, kick, ack_valid, ack;
    uint16_t rpm_top, rpm_bot, rpm_set;   /* STATUS tail; 0xFFFF = telemetry lost */
    uint16_t throttle_mean_pm;
    uint8_t beams, gate_state, ready_flags, feed_ms, feed_busy, wheels_ready;
    uint16_t last_v_dmps, last_dip_pm, shots_confirmed, fault_bits;
    uint8_t nev;
    fdr_ev_t ev[FDR_MAX_EV];
} fdr_out_t;

typedef struct {
    /* launcher */
    int32_t ramp_mean, integ[2], thr[2], last_rpm[2], rpm_cap;
    uint32_t age[2], win_ms, over_ms, nr_ms, on_ms;
    int32_t prev_target;
    uint8_t ready_prev, inwin_now;
    /* feeder */
    uint8_t gstate, gate_load;
    uint32_t t_state, t_feed, t_last_feed, t_last_muz;
    uint8_t has_feed, has_muz, prev_muz, committed, aborted, exp_exit, exit_seen, left_seen, breech_clr_seen, retries, empty_reported, trk, feed_req, inited;
    uint16_t fire_id, feeds, muz_all;
    int32_t rpm_feed_mean, rpm_min_mean;
    uint16_t last_v_dmps, last_dip_pm;
    uint8_t feed_ms;
    uint8_t jam_reported;
    uint16_t fault_bits;
} fdr_t;

void fdr_init(fdr_t *d);
void fdr_step(fdr_t *d, const fdr_in_t *in, fdr_out_t *out);
int32_t fdr_rpm_cap(const int32_t *cfg);           /* floor(v_cap * 60 / (pi * wheel_d)), from cfg 23, 24 with the clamps of I13 */
/* the launcher part (launcher.c), called by fdr_step */
void lau_step(fdr_t *d, const fdr_in_t *in, fdr_out_t *out, int32_t *mean_target);
int32_t fdr_tdiv(int32_t a, int32_t b);            /* integer division truncating toward zero (the python twin does the same) */
void fdr_ev(fdr_out_t *o, uint8_t code, int32_t arg);
void fdr_fault(fdr_t *d, fdr_out_t *o, uint16_t bit);

/* ---- DShot (dshot.c): frame and bidirectional-DShot telemetry codec, pure functions ------------------------------------------------------------- */
uint16_t dshot_frame(uint16_t value11, int telemetry_bit, int inverted_crc);  /* 11-bit throttle | telemetry | 4-bit crc */
uint32_t dshot_gcr_encode(uint16_t payload16);                                    /* ESC side, for tests: 16 bits -> 21-bit line frame */
int dshot_gcr_decode(uint32_t v21, uint16_t *payload12);                          /* 0 = ok; -1 bad GCR symbol or crc */
int32_t dshot_payload_rpm(uint16_t payload12, int pole_pairs);                    /* eeemmmmmmmmm period in us -> mechanical rpm; 0xFFF = 0 */

/* ---- platform interface ----------------------------------------------------------------------------------------------------------------- */
typedef struct {
    uint8_t estop_open;  /* 1 = E-stop circuit open (pressed) */
    uint8_t key_on;
    uint8_t hall[2];     /* 1 = magnet at the sensor (pan, tilt) */
    uint8_t diag;        /* TMC2209 DIAG, either driver, 1 = stall / overtemp / short */
    uint8_t beam_breech, beam_muzzle, gate_home, push_home;
    int32_t rpm[2];      /* top, bottom; -1 = no valid eRPM reply this millisecond */
    uint32_t muzzle_dt_us;
} fw_in_t;

typedef struct {
    uint8_t steppers_en;
    int8_t step_n[2];       /* microsteps to emit during the next millisecond, logical sign (pos delta), |n| <= 25 */
    uint8_t dir_invert[2];  /* HAL: DIR level = (step_n > 0) ^ dir_invert */
    uint16_t fw_duty;       /* mean applied throttle, permille (0 whenever not ENGAGING / FIRING) */
    uint16_t pusher_us;     /* pusher servo pulse width */
    uint16_t gate_us;       /* gate servo pulse width */
    uint16_t dshot[2];      /* top, bottom */
    uint8_t feed_req;       /* FEED_REQ (GP19): rising edge starts one stroke in the hardware limiter */
    uint8_t kick_ok;        /* the firmware allows KICK to run (I15) */
    uint8_t kick_level;     /* KICK (GP28) square wave, toggles every 5 ms while kick_ok */
    uint8_t led_red, led_green, buzzer;
    uint8_t request_bootsel;
    uint32_t shots_fed;     /* running count of balls fed (committed) */
} fw_out_t;

typedef struct {
    void (*tx)(void *ctx, const uint8_t *d, size_t n);                              /* bytes to the host (one whole frame per call) */
    int (*flash_write)(void *ctx, const uint8_t *img, size_t n);                    /* persist the config image; 0 = ok */
    void (*trace)(void *ctx, int kind, uint32_t t_ms, int32_t a, int32_t b, int32_t c);  /* optional */
    void *ctx;
} fw_hal_t;

typedef struct {
    double pos, v, target, vff, vmax, amax;  /* deg, deg/s, deg/s^2, home frame */
    uint32_t t_aim_ms;
    int32_t emitted;                         /* microsteps emitted in the home frame */
    int homing;                              /* 0 none, 1 clear, 2 approach, 3 backoff */
    uint32_t homing_t0;
    uint8_t homed, has_aim, hall, prev_hall, marked;
    double mark;                             /* position where the hall released while backing out of the window */
    uint32_t stuck_ms;
} fw_axis_t;

typedef struct {
    fw_hal_t hal;
    fw_decoder_t dec;
    fw_out_t out;
    fw_in_t in;
    int32_t cfg[CFG_COUNT];
    int32_t flash_cfg[CFG_COUNT];
    uint8_t uid[8];
    uint8_t boot_note[48];
    uint8_t boot_note_len, boot_note_sent;

    uint32_t now;                            /* ms since boot */
    uint8_t state;
    fw_axis_t ax[2];
    uint8_t rxq[512];
    uint16_t rxq_head, rxq_tail;
    uint8_t tx_seq, tx_started;

    uint8_t session_valid, hello_done, version_bad;
    uint32_t session;
    uint8_t has_hb;
    uint32_t last_hb_ms, hb_run_t0;
    uint8_t hb_flags;
    uint16_t hb_run;
    uint8_t wd_tripped;
    uint16_t fault_bits;
    uint8_t steppers_enabled, estop_open, key_on;
    uint8_t estop_raw_prev, key_raw_prev;
    uint8_t estop_closed_ms, key_on_ms;

    double fw_req, fw;                       /* fw_req: requested speed, permille of rpm_max; fw: mean applied throttle, permille */
    uint32_t fw_req_t, fw_refresh_t;
    uint16_t shots_total;
    uint8_t shots_arm, shot_cap;
    uint32_t arm_deadline;
    uint8_t has_deadman_low;
    uint32_t deadman_low_since;
    uint16_t last_fire_id;
    struct { uint16_t id; uint8_t res, used; } fire_ids[8];
    uint8_t fire_ids_head;
    uint8_t balls_left, ball_idx, phase;     /* legacy v1 burst fields, kept so the shared functions stay textually identical; unused */
    uint16_t cur_fire_id;
    uint32_t phase_end;
    uint8_t has_ball;
    uint32_t last_ball_ms;
    uint8_t pusher_ext;

    uint32_t next_status;
    uint32_t bad_reported, bad_event_ms;
    uint8_t has_bad_event;
    uint8_t diag_ms;                         /* ms the DIAG input has been high */
    uint8_t inputs_init;
    uint8_t sector_set;                      /* SECTOR_* bits; zeroed at boot only (never by HELLO / session, never from flash) */
    uint16_t faults_reported;

    /* cuas2 */
    fdr_t fdr;
    fdr_out_t fo;                            /* the feeder's outputs of the latest tick */
    int16_t diff_ovr;                        /* FLYWHEEL tail, valid while diff_ovr_valid */
    uint8_t diff_ovr_valid;
    uint8_t fire_cmd_pending, fire_pending_seq, clear_pending;
    uint16_t fire_pending_id;
    uint8_t kick_level;
    uint16_t conf_arm;                       /* muzzle-confirmed balls since ARM (diagnostics) */
    uint16_t muz_key;                        /* muzzle beam pulses since the key went on: mirrors the hardware shot counter (reset by the key) */
    uint8_t muz_prev_hw;
} fw_t;

/* flash_img: the stored config image (FW_CFG_IMAGE_SIZE bytes) or NULL for a blank flash; a bad image loads defaults + FAULT_CFG_INVALID. */
void fw_init(fw_t *f, const fw_hal_t *hal, const uint8_t uid[8], const uint8_t *flash_img, size_t flash_n);
void fw_set_boot_note(fw_t *f, const char *s);
void fw_rx(fw_t *f, const uint8_t *d, size_t n);   /* queue received bytes (overflow raises FAULT_USB_OVERFLOW) */
void fw_tick(fw_t *f, uint32_t now_ms, const fw_in_t *in);  /* exactly once per millisecond, now_ms = ms since boot */

/* planner / homing (lite planner.c, homing.c): called from fw_tick */
double fw_spd(const fw_t *f, int axis);            /* microsteps per degree */
void fw_axes_tick(fw_t *f);
void fw_axis_hold(fw_axis_t *a);
void fw_homing_start(fw_t *f, int axis);
void fw_homing_tick(fw_t *f, int axis);            /* hall edge detection + 3-phase routine; the planner skips an axis while homing */

#define FW_STEP_RATE_MAX 25000.0                   /* steps/s */

#ifdef __cplusplus
}
#endif
#endif
