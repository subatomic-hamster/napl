/* Frame layer: COBS(id | seq | payload | crc16 LE) 0x00 and the streaming decoder (spec section 2).
 * SPDX-License-Identifier: BSD-3-Clause */
#include "fw_core.h"

size_t fw_frame_encode(uint8_t id, uint8_t seq, const uint8_t *payload, size_t n, uint8_t *out) {
    uint8_t body[FW_MAX_BODY];
    if (n > FW_MAX_PAYLOAD) n = FW_MAX_PAYLOAD;
    body[0] = id;
    body[1] = seq;
    for (size_t i = 0; i < n; i++) body[2 + i] = payload[i];
    uint16_t crc = fw_crc16(body, 2 + n);
    body[2 + n] = (uint8_t)crc;
    body[3 + n] = (uint8_t)(crc >> 8);
    size_t m = fw_cobs_encode(body, 4 + n, out);
    out[m++] = 0;
    return m;
}

static int decode_chunk(const uint8_t *chunk, size_t n, fw_frame_t *f) {
    uint8_t body[96];
    int m = fw_cobs_decode(chunk, n, body, sizeof body);
    if (m < 4 || m > FW_MAX_BODY) return -1;  /* id + seq + crc16 is 4 bytes; the spec's "length < 3" floor is subsumed */
    uint16_t want = (uint16_t)(body[m - 2] | (body[m - 1] << 8));
    if (fw_crc16(body, (size_t)m - 2) != want) return -1;
    f->id = body[0];
    f->seq = body[1];
    f->len = (uint8_t)(m - 4);
    for (int i = 0; i < f->len; i++) f->payload[i] = body[2 + i];
    return 0;
}

int fw_decoder_byte(fw_decoder_t *d, uint8_t b, fw_frame_t *f) {
    if (b != 0) {
        if (d->n <= 80) d->buf[d->n++] = b;  /* longer than any frame: garbage, it fails at the next delimiter */
        return 0;
    }
    size_t n = d->n;
    d->n = 0;
    if (!d->synced) {  /* first delimiter: what came before it is a partial frame */
        d->synced = 1;
        return 0;
    }
    if (n == 0) return 0;
    if (decode_chunk(d->buf, n, f) < 0) {
        d->bad_frames++;
        return 0;
    }
    d->good_frames++;
    return 1;
}
