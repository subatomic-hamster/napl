/* Config store: defaults (spec 6.4) and the flash image (last sector): magic "CLC1", version 3, 41 x i32 LE (keys 1-41), CRC-32 of the first 172 bytes.
 * The cuas2 firmware refuses a v1 / v2 image (cfg_invalid, defaults) and lite refuses this one (it wants version 2). SPDX-License-Identifier: BSD-3-Clause */
#include "fw_core.h"

#define MAGIC 0x31434C43u /* "CLC1" little-endian */
#define IMG_VERSION 3u

static void put32(uint8_t *p, uint32_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24); }
static uint32_t get32(const uint8_t *p) { return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24; }

void fw_cfg_defaults(int32_t c[CFG_COUNT]) {
    static const int32_t d[CFG_COUNT] = {
        0, 16000, 12800, -170000, 170000, -15000, 70000, 0, 0, 0,   /* 0-9: spr, pan/tilt min/max, home offsets, dir invert */
        10, 120, 1900, 1100, 80, 450, 100, 0, 300, 200,             /* 10 hard cap, 11 hard timeout, 12 ext us, 13 ret us, 14 dwell ms, 15 min interval, 16 (unused), 17 fire_tilt_min, 18 wd, 19 homing */
        -170000, 170000, 70000,                                     /* 20-22: fire sector */
        40000, 380, 17500, 4500, 10, 100, 15, 7, 40000, 900,        /* 23 wheel_d_um .. 32 throttle_max_pm */
        600, 40, 1150, 1700, 30, 250, 50, 1000,                     /* 33 kp, 34 ki, 35 gate_load_us, 36 gate_bore_us, 37 fill_ms, 38 exit_timeout_ms, 39 telemetry_loss_ms, 40 k_beam */
        10,                                                         /* 41 hw_counter_n */
    };
    for (int i = 0; i < CFG_COUNT; i++) c[i] = d[i];
}

void fw_cfg_pack(const int32_t c[CFG_COUNT], uint8_t img[FW_CFG_IMAGE_SIZE]) {
    put32(img, MAGIC);
    put32(img + 4, IMG_VERSION);
    for (int k = 1; k < CFG_COUNT; k++) put32(img + 8 + 4 * (k - 1), (uint32_t)c[k]);
    put32(img + FW_CFG_IMAGE_SIZE - 4, fw_crc32(img, FW_CFG_IMAGE_SIZE - 4));
}

int fw_cfg_unpack(const uint8_t *img, size_t n, int32_t c[CFG_COUNT]) {
    int blank = 1;
    if (!img || n == 0) return 1;
    for (size_t i = 0; i < n && blank; i++) blank = img[i] == 0xFF;
    if (blank) return 1;
    if (n < FW_CFG_IMAGE_SIZE || get32(img) != MAGIC || get32(img + 4) != IMG_VERSION) return -1;
    if (get32(img + FW_CFG_IMAGE_SIZE - 4) != fw_crc32(img, FW_CFG_IMAGE_SIZE - 4)) return -1;
    for (int k = 1; k < CFG_COUNT; k++) c[k] = (int32_t)get32(img + 8 + 4 * (k - 1));
    return 0;
}
