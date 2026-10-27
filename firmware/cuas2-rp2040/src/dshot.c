/* DShot frame and bidirectional-DShot telemetry codec: pure functions, written from the public protocol description (brushlesswhoop.com "DShot and
 * Bidirectional DShot", Betaflight dshot_bitbang_decode.c as fetched 2026-10-27). ESC firmware is NOT part of this repo (spec 4.5); this file only talks the
 * electrical protocol. SPDX-License-Identifier: BSD-3-Clause
 *
 * TX (host -> ESC), 16 bits MSB first: 11-bit value (0 = disarmed, 48..2047 throttle) | telemetry request | 4-bit crc. In bidirectional mode the line is inverted
 * and the crc is the complement of the nibble xor.
 * RX (ESC -> host), 21 bits at 5/4 of the DShot rate: payload16 = 12-bit value (eeemmmmmmmmm: period = m << e microseconds per electrical revolution) | 4-bit crc,
 * crc = ~(v ^ v>>4 ^ v>>8) & 0xF over the 12 bits (so the xor of all four nibbles is 0xF); the 16 bits are mapped nibble by nibble to 5-bit GCR symbols (20 bits),
 * then to line levels: a GCR 1 is a level change, a 0 keeps the level, first level 0 (so 21 bits). */
#include "fw_core.h"

uint16_t dshot_frame(uint16_t value11, int telemetry_bit, int inverted_crc) {
    uint16_t v = (uint16_t)((value11 & 0x7FF) << 1 | (telemetry_bit ? 1 : 0));
    uint16_t crc = (uint16_t)((v ^ (v >> 4) ^ (v >> 8)) & 0x0F);
    if (inverted_crc) crc = (uint16_t)(~crc & 0x0F);
    return (uint16_t)(v << 4 | crc);
}

static const uint8_t GCR[16] = {0x19, 0x1B, 0x12, 0x13, 0x1D, 0x15, 0x16, 0x17, 0x1A, 0x09, 0x0A, 0x0B, 0x1E, 0x0D, 0x0E, 0x0F};

uint32_t dshot_gcr_encode(uint16_t payload16) {
    uint32_t gcr = 0;
    for (int i = 3; i >= 0; i--) gcr = gcr << 5 | GCR[(payload16 >> (4 * i)) & 0xF];
    uint32_t line = 0, level = 0;  /* l0 = 0; each GCR 1 flips the level; the first transmitted bit is the MSB (bit 20) */
    for (int i = 19; i >= 0; i--) {
        level ^= gcr >> i & 1;
        line |= level << i;
    }
    return line;  /* bit 20 is l0 = 0; bit i is l(20 - i) */
}

int dshot_gcr_decode(uint32_t v21, uint16_t *payload12) {
    const uint32_t gcr = (v21 ^ (v21 >> 1)) & 0xFFFFFu;
    uint32_t out = 0;
    for (int i = 3; i >= 0; i--) {
        const uint32_t sym = gcr >> (5 * i) & 0x1F;
        int n = -1;
        for (int k = 0; k < 16; k++)
            if (GCR[k] == sym) n = k;
        if (n < 0) return -1;
        out = out << 4 | (uint32_t)n;
    }
    uint32_t x = out ^ (out >> 8);
    x ^= x >> 4;
    if ((x & 0xF) != 0xF) return -1;
    *payload12 = (uint16_t)(out >> 4);
    return 0;
}

int32_t dshot_payload_rpm(uint16_t payload12, int pole_pairs) {
    if (payload12 == 0xFFF || pole_pairs <= 0) return 0;
    const uint32_t period_us = (uint32_t)(payload12 & 0x1FF) << (payload12 >> 9);
    if (period_us == 0) return 0;
    return (int32_t)(60000000u / period_us / (uint32_t)pole_pairs);  /* eRPM = 60e6 / period, rpm = eRPM / pole pairs */
}
