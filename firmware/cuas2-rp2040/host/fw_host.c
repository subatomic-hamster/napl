/* Host (x86) build of the cuas2 firmware core as a shared library for the Python tests (ctypes): the same sources the Pico runs. Two handles:
 *  fwh_*  one whole simulated MCU (fw_t): bytes in/out, sampled inputs in, outputs out; the Python side plays the board (steppers, halls, wheels, gate, pusher).
 *  fdrh_* the launcher + feeder step alone (fdr_t), the unit the python twin (python/napl/cuas2/fsm.py) mirrors.
 * Flat int32 arrays are the interface so the ctypes side needs no struct layout. SPDX-License-Identifier: BSD-3-Clause */
#include <stdlib.h>
#include <string.h>
#include "fw_core.h"

#define TXCAP 16384
#define TRCAP 8192

typedef struct { int32_t kind, t_ms, a, b, c; } trace_rec;

typedef struct {
    fw_t fw;
    uint8_t tx[TXCAP];
    size_t ntx;
    int32_t ntrace;
    trace_rec trace[TRCAP];
    uint8_t flash[FW_CFG_IMAGE_SIZE];
    int flash_valid;
} fwh_t;

static void h_tx(void *ctx, const uint8_t *d, size_t n) {
    fwh_t *h = ctx;
    if (h->ntx + n <= TXCAP) { memcpy(h->tx + h->ntx, d, n); h->ntx += n; }
}
static int h_flash(void *ctx, const uint8_t *img, size_t n) {
    fwh_t *h = ctx;
    memcpy(h->flash, img, n);
    h->flash_valid = 1;
    return 0;
}
static void h_trace(void *ctx, int kind, uint32_t t_ms, int32_t a, int32_t b, int32_t c) {
    fwh_t *h = ctx;
    if (h->ntrace < TRCAP) h->trace[h->ntrace++] = (trace_rec){kind, (int32_t)t_ms, a, b, c};
}

/* cfg_override: CFG_COUNT int32 (index 0 unused; INT32_MIN = keep the default) or NULL. flash_mode: 0 blank flash, 1 image from the overrides, 2 corrupt image,
 * 3 a v2 (lite) image: magic ok, version 2 -> refused. */
fwh_t *fwh_new(const int32_t *cfg_override, int flash_mode, const uint8_t uid[8]) {
    fwh_t *h = calloc(1, sizeof *h);
    int32_t cfg[CFG_COUNT];
    uint8_t img[FW_CFG_IMAGE_SIZE];
    fw_hal_t hal = {h_tx, h_flash, h_trace, h};
    fw_cfg_defaults(cfg);
    if (cfg_override)
        for (int k = 1; k < CFG_COUNT; k++)
            if (cfg_override[k] != INT32_MIN) cfg[k] = cfg_override[k];
    fw_cfg_pack(cfg, img);
    if (flash_mode == 2) img[10] ^= 0x55;
    if (flash_mode == 3) img[4] = 2;
    if (flash_mode == 0) memset(img, 0xFF, sizeof img);
    fw_init(&h->fw, &hal, uid, img, sizeof img);
    return h;
}
void fwh_free(fwh_t *h) { free(h); }
/* deep copy for the fuzz (the HAL context pointer is re-aimed at the copy) */
fwh_t *fwh_clone(const fwh_t *h) {
    fwh_t *c = malloc(sizeof *c);
    memcpy(c, h, sizeof *c);
    c->fw.hal.ctx = c;
    return c;
}

/* in[13]: estop_open, key_on, hall0, hall1, diag, breech, muzzle, gate_home, push_home, rpm_top, rpm_bot, muzzle_dt_us, (unused).
 * One 1 ms tick. rx bytes are queued first. Returns the number of tx bytes produced during this tick (readable with fwh_tx until the next call). */
int fwh_tick(fwh_t *h, uint32_t now_ms, const uint8_t *rx, int nrx, const int32_t *in) {
    fw_in_t u;
    memset(&u, 0, sizeof u);
    u.estop_open = (uint8_t)in[0];
    u.key_on = (uint8_t)in[1];
    u.hall[0] = (uint8_t)in[2];
    u.hall[1] = (uint8_t)in[3];
    u.diag = (uint8_t)in[4];
    u.beam_breech = (uint8_t)in[5];
    u.beam_muzzle = (uint8_t)in[6];
    u.gate_home = (uint8_t)in[7];
    u.push_home = (uint8_t)in[8];
    u.rpm[0] = in[9];
    u.rpm[1] = in[10];
    u.muzzle_dt_us = (uint32_t)in[11];
    h->ntx = 0;
    if (nrx > 0) fw_rx(&h->fw, rx, (size_t)nrx);
    fw_tick(&h->fw, now_ms, &u);
    return (int)h->ntx;
}
const uint8_t *fwh_tx(fwh_t *h) { return h->tx; }

/* o[20]: steppers_en, step_n0, step_n1, dir_invert0, dir_invert1, fw_duty, pusher_us, gate_us, dshot_top, dshot_bot, feed_req, kick_ok, kick_level, shots_fed,
 * request_bootsel, led_red, led_green, buzzer, 0, 0 */
void fwh_out(fwh_t *h, int32_t *o) {
    const fw_out_t *u = &h->fw.out;
    int32_t v[20] = {u->steppers_en, u->step_n[0], u->step_n[1], u->dir_invert[0], u->dir_invert[1], u->fw_duty, u->pusher_us, u->gate_us, u->dshot[0], u->dshot[1],
                     u->feed_req, u->kick_ok, u->kick_level, (int32_t)u->shots_fed, u->request_bootsel, u->led_red, u->led_green, u->buzzer, 0, 0};
    memcpy(o, v, sizeof v);
}

/* i[24]: state, hb_run, last_hb_ms (-1 = none), shot_cap, shots_arm, shots_total, fault_bits, version_bad, hello_done, wd_tripped, homed0, homed1, session_valid,
 * steppers_enabled, estop_open, key_on, gstate (feeder), feeds, muz_all, ready_flags, sector_set, conf_arm, last_fire_id, ready (wheels). d[6]: fw_req, fw, pos0, pos1, v0, v1 */
void fwh_snap(fwh_t *h, int32_t *i, double *d) {
    const fw_t *f = &h->fw;
    int32_t v[24] = {f->state, f->hb_run, f->has_hb ? (int32_t)f->last_hb_ms : -1, f->shot_cap, f->shots_arm, f->shots_total, f->fault_bits,
                     f->version_bad, f->hello_done, f->wd_tripped, f->ax[0].homed, f->ax[1].homed, f->session_valid, f->steppers_enabled,
                     f->estop_open, f->key_on, f->fdr.gstate, f->fdr.feeds, f->fdr.muz_all, f->fo.ready_flags, f->sector_set, f->conf_arm, f->last_fire_id,
                     f->fo.wheels_ready};
    memcpy(i, v, sizeof v);
    double w[6] = {f->fw_req, f->fw, f->ax[0].pos, f->ax[1].pos, f->ax[0].v, f->ax[1].v};
    memcpy(d, w, sizeof w);
}
void fwh_clear_homed(fwh_t *h, int axis) { h->fw.ax[axis].homed = 0; }
int fwh_cfg(fwh_t *h, int key, int flash) { return flash ? h->fw.flash_cfg[key] : h->fw.cfg[key]; }
void fwh_poke_cfg(fwh_t *h, int key, int value) { h->fw.cfg[key] = value; }  /* tests only: bypasses CFG_SET's range checks and state rules */
/* tests only: force one precondition of a FIRE to be false while the state machine still says ENGAGING, to exercise each fire_precheck() gate on its own.
 * what: 0 sector_set, 1 shots_arm, 2 fault_bits, 3 estop_open, 4 state */
void fwh_poke_field(fwh_t *h, int what, int value) {
    fw_t *f = &h->fw;
    switch (what) {
    case 0: f->sector_set = (uint8_t)value; break;
    case 1: f->shots_arm = (uint8_t)value; break;
    case 2: f->fault_bits = (uint16_t)value; break;
    case 3: f->estop_open = (uint8_t)value; break;
    case 4: f->state = (uint8_t)value; break;
    default: break;
    }
}
int fwh_flash_valid(fwh_t *h) { return h->flash_valid; }
void fwh_flash_image(fwh_t *h, uint8_t *out) { memcpy(out, h->flash, FW_CFG_IMAGE_SIZE); }
void fwh_set_boot_note(fwh_t *h, const char *s) { fw_set_boot_note(&h->fw, s); }

int fwh_trace_count(fwh_t *h) { return h->ntrace; }
void fwh_trace_get(fwh_t *h, int idx, int32_t *o) { memcpy(o, &h->trace[idx], sizeof(trace_rec)); }
void fwh_trace_clear(fwh_t *h) { h->ntrace = 0; }

/* thin exports for the vector tests */
int fwh_crc16(const uint8_t *d, int n) { return fw_crc16(d, (size_t)n); }
int fwh_cobs_encode(const uint8_t *in, int n, uint8_t *out) { return (int)fw_cobs_encode(in, (size_t)n, out); }
int fwh_cobs_decode(const uint8_t *in, int n, uint8_t *out, int cap) { return fw_cobs_decode(in, (size_t)n, out, (size_t)cap); }
int fwh_frame_encode(int id, int seq, const uint8_t *payload, int n, uint8_t *out) { return (int)fw_frame_encode((uint8_t)id, (uint8_t)seq, payload, (size_t)n, out); }
void fwh_cfg_defaults(int32_t *cfg) { fw_cfg_defaults(cfg); }
int fwh_cfg_count(void) { return CFG_COUNT; }
int fwh_rpm_cap(const int32_t *cfg) { return fdr_rpm_cap(cfg); }
int fwh_dshot_frame(int v, int tel, int inv) { return dshot_frame((uint16_t)v, tel, inv); }
uint32_t fwh_gcr_encode(uint32_t payload16) { return dshot_gcr_encode((uint16_t)payload16); }
int fwh_gcr_decode(uint32_t v21, int32_t *payload12) {
    uint16_t p = 0;
    const int r = dshot_gcr_decode(v21, &p);
    *payload12 = p;
    return r;
}
int fwh_payload_rpm(int payload12, int pole_pairs) { return dshot_payload_rpm((uint16_t)payload12, pole_pairs); }

/* streaming decoder: out = [id, seq, len, payload...] per good frame, concatenated; returns bytes written; *bad = bad_frames so far */
int fwh_decode_stream(const uint8_t *in, int n, uint8_t *out, int cap, uint32_t *bad) {
    fw_decoder_t d;
    fw_frame_t fr;
    int o = 0;
    memset(&d, 0, sizeof d);
    for (int i = 0; i < n; i++)
        if (fw_decoder_byte(&d, in[i], &fr) && o + 3 + fr.len <= cap) {
            out[o++] = fr.id;
            out[o++] = fr.seq;
            out[o++] = fr.len;
            memcpy(out + o, fr.payload, fr.len);
            o += fr.len;
        }
    *bad = d.bad_frames;
    return o;
}

/* ---- the launcher + feeder alone ---------------------------------------------------------------------------------------------------------- */
typedef struct { fdr_t d; fdr_out_t o; } fdrh_t;

fdrh_t *fdrh_new(void) {
    fdrh_t *h = calloc(1, sizeof *h);
    fdr_init(&h->d);
    return h;
}
void fdrh_free(fdrh_t *h) { free(h); }

/* in[18]: t_ms, rpm_top, rpm_bot, breech, muzzle, gate_home, push_home, estop_open, key_on, hb_ok, deadman, inhibit, in_sector, cmd, speed_req_pm, diff_pm, fire_id,
 * muzzle_dt_us. out[22 + 2 * FDR_MAX_EV]: dshot0, dshot1, gate_us, push_us, feed_req, kick, ack (-1 none), rpm_top, rpm_bot, rpm_set, throttle_mean, beams,
 * gate_state, ready_flags, feed_ms, feed_busy, wheels_ready, last_v, last_dip, shots_confirmed, fault_bits, nev, then (code, arg) pairs. */
void fdrh_step(fdrh_t *h, const int32_t *in, const int32_t *cfg, int32_t *out) {
    fdr_in_t u;
    memset(&u, 0, sizeof u);
    u.t_ms = (uint32_t)in[0];
    u.rpm_top = in[1];
    u.rpm_bot = in[2];
    u.beam_breech = (uint8_t)in[3];
    u.beam_muzzle = (uint8_t)in[4];
    u.gate_home = (uint8_t)in[5];
    u.push_home = (uint8_t)in[6];
    u.estop_open = (uint8_t)in[7];
    u.key_on = (uint8_t)in[8];
    u.hb_ok = (uint8_t)in[9];
    u.deadman = (uint8_t)in[10];
    u.inhibit = (uint8_t)in[11];
    u.in_sector = (uint8_t)in[12];
    u.cmd = (uint8_t)in[13];
    u.speed_req_pm = (uint16_t)in[14];
    u.diff_pm = (int16_t)in[15];
    u.fire_id = (uint16_t)in[16];
    u.muzzle_dt_us = (uint32_t)in[17];
    u.cfg = cfg;
    fdr_step(&h->d, &u, &h->o);
    const fdr_out_t *o = &h->o;
    const int32_t v[22] = {o->dshot[0], o->dshot[1], o->gate_us, o->push_us, o->feed_req, o->kick, o->ack_valid ? o->ack : -1, o->rpm_top, o->rpm_bot, o->rpm_set,
                           o->throttle_mean_pm, o->beams, o->gate_state, o->ready_flags, o->feed_ms, o->feed_busy, o->wheels_ready, o->last_v_dmps, o->last_dip_pm,
                           o->shots_confirmed, o->fault_bits, o->nev};
    memcpy(out, v, sizeof v);
    for (int i = 0; i < FDR_MAX_EV; i++) {
        out[22 + 2 * i] = i < o->nev ? o->ev[i].code : 0;
        out[23 + 2 * i] = i < o->nev ? o->ev[i].arg : 0;
    }
}
