/* cuas-lite RP2040 firmware core: hardware-independent. Protocol (docs/cuas-lite/spec.md section 2), state machine and gates (section 3),
 * planner, homing, shot accounting, config store. The platform (src/pico/ or host/) calls fw_rx() with received bytes and fw_tick() once per
 * millisecond with the sampled inputs; the core answers through the fw_hal_t callbacks and the fw_out_t outputs. No malloc, no globals.
 * SPDX-License-Identifier: BSD-3-Clause */
#ifndef FW_CORE_H
#define FW_CORE_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FW_PROTO_MAJOR 1
#define FW_PROTO_MINOR 0
#define FW_VER_MAJOR 0
#define FW_VER_MINOR 1
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
enum { FAULT_HOMING_TIMEOUT = 1, FAULT_DRIVER_DIAG = 2, FAULT_CFG_INVALID = 4, FAULT_SHOT_COUNTER = 8, FAULT_HALL_STUCK = 16, FAULT_USB_OVERFLOW = 32 };
enum { EV_STATE_CHANGE = 1, EV_SHOT, EV_HOMED, EV_FAULT, EV_WATCHDOG, EV_ESTOP_EDGE, EV_KEY_EDGE, EV_BAD_FRAMES, EV_SECTOR_ABORT /* 9: arg = fire_id << 8 | ball index; the pusher stroke was aborted, the barrel left the fire sector */ };
enum { R_NONE, R_BOOT, R_WATCHDOG, R_ESTOP, R_KEY_OFF, R_DISARM, R_ARM_TIMEOUT, R_SHOT_CAP, R_FAULT, R_OPERATOR_ARM, R_LINK_UP, R_NEW_SESSION };
enum { RES_OK, RES_BAD_STATE, RES_INTERLOCK, RES_INHIBIT, RES_CAP, RES_RANGE, RES_BUSY, RES_UNKNOWN_CMD, RES_BAD_ARG, RES_NOT_HOMED, RES_NOT_SPUN };
/* config keys, 1..19 (spec section 2.2) */
enum { CFG_PAN_SPR = 1, CFG_TILT_SPR, CFG_PAN_MIN, CFG_PAN_MAX, CFG_TILT_MIN, CFG_TILT_MAX, CFG_PAN_HOME_OFF, CFG_TILT_HOME_OFF, CFG_DIR_INVERT,
       CFG_HARD_SHOT_CAP, CFG_HARD_ARM_TIMEOUT, CFG_PUSHER_EXT_US, CFG_PUSHER_RET_US, CFG_PUSHER_DWELL_MS, CFG_MIN_SHOT_INTERVAL_MS, CFG_SPINUP_MS,
       CFG_FIRE_TILT_MIN, CFG_WD_MS, CFG_HOMING_SPEED,
       CFG_FIRE_PAN_MIN, CFG_FIRE_PAN_MAX, CFG_FIRE_TILT_MAX, /* 20-22: with 17 (fire_tilt_min) the fire sector at the feed window, mdeg */
       CFG_COUNT /* = 23: array size, index 0 unused */ };

/* trace kinds for the optional fw_hal_t.trace hook (tests; NULL on the Pico) */
enum { TR_TRANSITION = 1, TR_EVENT, TR_SHOT, TR_REFUSAL };

/* ---- crc16 / cobs / frame (cobs.c, crc16.c, proto.c) ---------------------------------------------------------------------------------- */
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
#define FW_CFG_IMAGE_SIZE 100                                                       /* magic u32, version u32, 22 x i32, crc32 */
void fw_cfg_defaults(int32_t cfg[CFG_COUNT]);
void fw_cfg_pack(const int32_t cfg[CFG_COUNT], uint8_t img[FW_CFG_IMAGE_SIZE]);
/* 0 = loaded, 1 = blank (all 0xFF / n == 0), -1 = invalid (bad magic / CRC) */
int fw_cfg_unpack(const uint8_t *img, size_t n, int32_t cfg[CFG_COUNT]);

/* ---- platform interface ----------------------------------------------------------------------------------------------------------------- */
typedef struct {
    uint8_t estop_open;  /* 1 = E-stop circuit open (pressed) */
    uint8_t key_on;
    uint8_t hall[2];     /* 1 = magnet at the sensor (pan, tilt) */
    uint8_t diag;        /* TMC2209 DIAG, either driver, 1 = stall / overtemp / short */
} fw_in_t;

typedef struct {
    uint8_t steppers_en;
    int8_t step_n[2];       /* microsteps to emit during the next millisecond, logical sign (pos delta), |n| <= 25 */
    uint8_t dir_invert[2];  /* HAL: DIR level = (step_n > 0) ^ dir_invert */
    uint16_t fw_duty;       /* applied flywheel duty, permille (0 whenever not ENGAGING / FIRING) */
    uint16_t pusher_us;     /* servo pulse width */
    uint8_t led_red, led_green, buzzer;
    uint8_t request_bootsel;
    uint32_t shots_fed;     /* running count of balls fed (a pulse the plant model counts) */
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

    double fw_req, fw;                       /* permille */
    uint32_t fw_req_t, fw_refresh_t;
    uint16_t shots_total;
    uint8_t shots_arm, shot_cap;
    uint32_t arm_deadline;
    uint8_t has_deadman_low;
    uint32_t deadman_low_since;
    uint16_t last_fire_id;
    struct { uint16_t id; uint8_t res, used; } fire_ids[8];
    uint8_t fire_ids_head;
    uint8_t balls_left, ball_idx, phase;     /* phase: 0 none, 1 wait, 2 ext, 3 ret */
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
    uint16_t faults_reported;
} fw_t;

/* flash_img: the stored config image (FW_CFG_IMAGE_SIZE bytes) or NULL for a blank flash; a bad image loads defaults + FAULT_CFG_INVALID. */
void fw_init(fw_t *f, const fw_hal_t *hal, const uint8_t uid[8], const uint8_t *flash_img, size_t flash_n);
void fw_set_boot_note(fw_t *f, const char *s);
void fw_rx(fw_t *f, const uint8_t *d, size_t n);   /* queue received bytes (overflow raises FAULT_USB_OVERFLOW) */
void fw_tick(fw_t *f, uint32_t now_ms, const fw_in_t *in);  /* exactly once per millisecond, now_ms = ms since boot */

/* planner / homing (planner.c, homing.c): called from fw_tick */
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
