/* Host build of the firmware core as a shared library for the Python tests (ctypes): the same core sources the Pico runs, compiled for x86.
 * One fwh_t = one simulated MCU; the Python side plays the plant (steppers, hall, E-stop, key) and the wire. SPDX-License-Identifier: BSD-3-Clause */
#include <stdlib.h>
#include <string.h>
#include "fw_core.h"

#define TXCAP 8192
#define TRCAP 4096

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

/* cfg_override: CFG_COUNT int32 (index 0 unused; INT32_MIN = keep the default) or NULL. flash_mode: 0 blank flash, 1 image from the overrides, 2 corrupt image. */
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
    if (flash_mode == 0) memset(img, 0xFF, sizeof img);
    fw_init(&h->fw, &hal, uid, img, sizeof img);
    return h;
}
void fwh_free(fwh_t *h) { free(h); }

/* One 1 ms tick. rx bytes are queued first. Returns the number of tx bytes produced during this tick (readable with fwh_tx until the next call). */
int fwh_tick(fwh_t *h, uint32_t now_ms, const uint8_t *rx, int nrx, int estop_open, int key_on, int hall0, int hall1, int diag) {
    fw_in_t in = {(uint8_t)estop_open, (uint8_t)key_on, {(uint8_t)hall0, (uint8_t)hall1}, (uint8_t)diag};
    h->ntx = 0;
    if (nrx > 0) fw_rx(&h->fw, rx, (size_t)nrx);
    fw_tick(&h->fw, now_ms, &in);
    return (int)h->ntx;
}
const uint8_t *fwh_tx(fwh_t *h) { return h->tx; }

/* outputs: steppers_en, step_n0, step_n1, dir_invert0, dir_invert1, fw_duty, pusher_us, shots_fed, request_bootsel, led_red, led_green, buzzer */
void fwh_out(fwh_t *h, int32_t *o) {
    const fw_out_t *u = &h->fw.out;
    int32_t v[12] = {u->steppers_en, u->step_n[0], u->step_n[1], u->dir_invert[0], u->dir_invert[1], u->fw_duty, u->pusher_us, (int32_t)u->shots_fed,
                     u->request_bootsel, u->led_red, u->led_green, u->buzzer};
    memcpy(o, v, sizeof v);
}

/* internal state for the Python twin-compatible view: state, hb_run, last_hb_ms (-1 = none), shot_cap, shots_arm, shots_total, fault_bits, version_bad,
 * hello_done, wd_tripped, homed0, homed1, session_valid, steppers_enabled, estop_open, key_on, cfg_crc... ; doubles: fw_req, fw, pos0, pos1, v0, v1 */
void fwh_snap(fwh_t *h, int32_t *i, double *d) {
    const fw_t *f = &h->fw;
    int32_t v[16] = {f->state, f->hb_run, f->has_hb ? (int32_t)f->last_hb_ms : -1, f->shot_cap, f->shots_arm, f->shots_total, f->fault_bits,
                     f->version_bad, f->hello_done, f->wd_tripped, f->ax[0].homed, f->ax[1].homed, f->session_valid, f->steppers_enabled,
                     f->estop_open, f->key_on};
    memcpy(i, v, sizeof v);
    double w[6] = {f->fw_req, f->fw, f->ax[0].pos, f->ax[1].pos, f->ax[0].v, f->ax[1].v};
    memcpy(d, w, sizeof w);
}
void fwh_clear_homed(fwh_t *h, int axis) { h->fw.ax[axis].homed = 0; }
int fwh_cfg(fwh_t *h, int key, int flash) { return flash ? h->fw.flash_cfg[key] : h->fw.cfg[key]; }
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
