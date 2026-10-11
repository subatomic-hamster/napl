/* COBS, byte-identical to python/napl/lite/protocol.py (a data run that ends exactly on a full 254-byte block gets no trailing empty block).
 * SPDX-License-Identifier: BSD-3-Clause */
#include "fw_core.h"

size_t fw_cobs_encode(const uint8_t *in, size_t n, uint8_t *out) {
    size_t o = 1, code_at = 0;
    uint8_t code = 1;
    int full = 0;
    for (size_t i = 0; i < n; i++) {
        full = 0;
        if (in[i] == 0) {
            out[code_at] = code;
            code_at = o++;
            code = 1;
        } else {
            out[o++] = in[i];
            if (++code == 0xFF) {
                out[code_at] = code;
                code_at = o++;
                code = 1;
                full = 1;
            }
        }
    }
    if (!full) out[code_at] = code;
    else o--;  /* the placeholder for a next block that never comes */
    return o;
}

int fw_cobs_decode(const uint8_t *in, size_t n, uint8_t *out, size_t cap) {
    size_t i = 0, o = 0;
    for (size_t k = 0; k < n; k++)
        if (in[k] == 0) return -1;
    while (i < n) {
        size_t code = in[i];
        if (i + code > n) return -1;
        if (o + code > cap) return -1;
        for (size_t k = 1; k < code; k++) out[o++] = in[i + k];
        i += code;
        if (code != 0xFF && i < n) {
            if (o >= cap) return -1;
            out[o++] = 0;
        }
    }
    return (int)o;
}
