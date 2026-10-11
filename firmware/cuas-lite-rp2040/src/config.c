/* Config store: defaults (spec section 2.2) and the flash image (last sector): magic "CLC1", version, 22 x i32 LE, CRC-32 of the first 96 bytes.
 * SPDX-License-Identifier: BSD-3-Clause */
#include "fw_core.h"

#define MAGIC 0x31434C43u /* "CLC1" little-endian */
#define IMG_VERSION 2u  /* 2: fire sector keys 20-22 (an image of version 1 is refused: cfg_invalid, defaults) */

static void put32(uint8_t *p, uint32_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24); }
static uint32_t get32(const uint8_t *p) { return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24; }

void fw_cfg_defaults(int32_t c[CFG_COUNT]) {
    static const int32_t d[CFG_COUNT] = {
        0, 16000, 12800, -170000, 170000, -15000, 70000, 0, 0, 0,
        10, 120, 1900, 1100, 120, 300, 700, 0, 300, 200,
        -170000, 170000, 70000,
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
