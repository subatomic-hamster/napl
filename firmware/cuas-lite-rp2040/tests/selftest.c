/* Stand-alone C check of the firmware core (no Python): CRC check value, COBS, frame, the 300 ms watchdog, an ARM refusal and the hard cap clamp.
 * The full vector set and the gate matrix run from Python (tests/lite/test_fw_core.py, test_fw_vectors.py) against the same sources.
 * SPDX-License-Identifier: BSD-3-Clause */
#include <stdio.h>
#include <string.h>
#include "fw_core.h"

static int fails;
#define CHECK(c) do { if (!(c)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); fails++; } } while (0)

static struct { uint8_t acks[64][4]; int nack; int last_state, state_changes; uint32_t wd_t; } rec;
static fw_decoder_t txdec;

static void on_tx(void *ctx, const uint8_t *d, size_t n) {
    (void)ctx;
    fw_frame_t fr;
    for (size_t i = 0; i < n; i++)
        if (fw_decoder_byte(&txdec, d[i], &fr)) {
            if (fr.id == ID_ACK && rec.nack < 64) memcpy(rec.acks[rec.nack++], fr.payload, 4);
            if (fr.id == ID_STATUS) rec.last_state = fr.payload[4];
        }
}
static void on_trace(void *ctx, int kind, uint32_t t, int32_t a, int32_t b, int32_t c) {
    (void)ctx; (void)c;
    if (kind == TR_TRANSITION) { rec.state_changes++; if (a == ST_SAFE && b == R_WATCHDOG) rec.wd_t = t; }
}

static fw_t f;
static uint32_t now;
static uint8_t seq;
static fw_in_t in = {0, 1, {0, 0}, 0};

static void send_msg(uint8_t id, const uint8_t *p, size_t n) {
    uint8_t w[80];
    size_t m = fw_frame_encode(id, seq++, p, n, w);
    fw_rx(&f, w, m);
}
static void run(uint32_t ms, int heartbeats) {
    for (uint32_t i = 0; i < ms; i++) {
        now++;
        if (heartbeats && now % 50 == 0) { uint8_t hb[3] = {1, 0, HB_DEADMAN_OK}; send_msg(ID_HEARTBEAT, hb, 3); }
        fw_tick(&f, now, &in);
    }
}

int main(void) {
    CHECK(fw_crc16((const uint8_t *)"123456789", 9) == 0x29B1);
    CHECK(fw_crc32((const uint8_t *)"123456789", 9) == 0xCBF43926u);
    uint8_t enc[8];
    const uint8_t d1[] = {0x11, 0x22, 0x00, 0x33};
    CHECK(fw_cobs_encode(d1, 4, enc) == 5 && !memcmp(enc, "\x03\x11\x22\x02\x33", 5));
    uint8_t dec[8];
    CHECK(fw_cobs_decode(enc, 5, dec, 8) == 4 && !memcmp(dec, d1, 4));
    CHECK(fw_cobs_decode((const uint8_t *)"\x05\x11\x22", 3, dec, 8) < 0);

    const uint8_t hello_wire[] = {0x02, 0x01, 0x02, 0x01, 0x07, 0x04, 0x03, 0x02, 0x01, 0xaf, 0xd6, 0x00};  /* protocol_vectors.json "hello" */
    uint8_t w[80], hp[6] = {1, 0, 4, 3, 2, 1};
    CHECK(fw_frame_encode(ID_HELLO, 0, hp, 6, w) == sizeof hello_wire && !memcmp(w, hello_wire, sizeof hello_wire));

    fw_hal_t hal = {on_tx, NULL, on_trace, NULL};
    const uint8_t uid[8] = {1, 2, 3, 4, 5, 6, 7, 8};
    fw_init(&f, &hal, uid, NULL, 0);
    fw_rx(&f, (const uint8_t *)"\0", 1);  /* a cold decoder drops everything up to the first 0x00 */
    send_msg(ID_HELLO, hp, 6);
    run(900, 1);
    CHECK(f.state == ST_SAFE);              /* needs a full second of heartbeats */
    run(300, 1);
    CHECK(f.state == ST_IDLE);
    /* ARM refused while not homed, nothing armed */
    uint8_t arm[4] = {1, 5, 60, 0};
    rec.nack = 0;
    send_msg(ID_ARM, arm, 4);
    run(5, 1);
    CHECK(rec.nack >= 1 && rec.acks[rec.nack - 1][0] == ID_ARM && rec.acks[rec.nack - 1][2] == RES_NOT_HOMED && f.state == ST_IDLE);
    /* watchdog: heartbeats stop; the trip comes after > 300 ms and not before */
    run(1, 0);
    const uint32_t t_last = f.last_hb_ms;
    run(300 - (now - t_last), 0);  /* exactly 300 ms since the last heartbeat: still IDLE */
    CHECK(f.state == ST_IDLE);
    run(5, 0);
    CHECK(f.state == ST_SAFE && f.wd_tripped && rec.wd_t - t_last <= 301 && rec.wd_t - t_last >= 300);
    printf(fails ? "selftest: %d FAILED\n" : "selftest: ok\n", fails);
    return fails != 0;
}
