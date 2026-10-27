/* Stand-alone C check of the cuas2 core (no Python): CRC / COBS / frame (the shared lite code), DShot codec, rpm cap, config image, HELLO_ACK fw_major = 2, the
 * FIRE / CFG argument gates, and one nominal shot through the launcher + feeder against a small ideal plant. The full vector set, the single-fault matrix, the
 * fuzz and the negative controls run from Python (tests/cuas2/) against the same sources. SPDX-License-Identifier: BSD-3-Clause */
#include <stdio.h>
#include <string.h>
#include "fw_core.h"

static int fails;
#define CHECK(c) do { if (!(c)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); fails++; } } while (0)

/* ---- feeder against an ideal plant ----------------------------------------------------------------------------------------------------- */
typedef struct { double rpm[2], rod, gate; int pocket, pushed, edge_prev; uint32_t powered_until; } plant_t;

static void plant_step(plant_t *p, const fdr_out_t *o, const int32_t *cfg, fdr_in_t *in, uint32_t t) {
    for (int i = 0; i < 2; i++) {
        const double tgt = o->dshot[i] > 0 ? (o->dshot[i] - 48) / 1952.0 * 20100.0 : 0.0;
        p->rpm[i] += (tgt - p->rpm[i]) / 40.0;
    }
    if (o->feed_req && !p->edge_prev) p->powered_until = t + 250;
    p->edge_prev = o->feed_req;
    if (t < p->powered_until) p->rod += (o->push_us == (uint16_t)cfg[CFG_PUSHER_EXT_US] ? 1.0 : -1.0) / 80.0;
    p->rod = p->rod < 0 ? 0 : (p->rod > 1 ? 1 : p->rod);
    if (p->rod < 0.3) p->gate += (o->gate_us == (uint16_t)cfg[CFG_GATE_BORE_US] ? 1.0 : -1.0) / 100.0;
    p->gate = p->gate < 0 ? 0 : (p->gate > 1 ? 1 : p->gate);
    if (p->gate < 0.02) p->pocket = p->pocket ? p->pocket : 1;
    const int in_bore = p->pocket && p->gate > 0.95 && !p->pushed;
    if (in_bore && p->rod >= 0.5) { p->pushed = 1; p->pocket = 0; }
    in->rpm_top = (int32_t)p->rpm[0];
    in->rpm_bot = (int32_t)p->rpm[1];
    in->beam_breech = p->pocket && p->gate > 0.95 && p->rod < 0.5;
    in->gate_home = p->gate < 0.02;
    in->push_home = p->rod < 0.02;
    in->beam_muzzle = 0;
    if (p->pushed && p->rod >= 0.69) { in->beam_muzzle = 1; in->muzzle_dt_us = 740; p->pushed = 0; }
}

static void test_feeder(void) {
    int32_t cfg[CFG_COUNT];
    fw_cfg_defaults(cfg);
    fdr_t d;
    fdr_out_t o;
    fdr_in_t in;
    plant_t p;
    memset(&p, 0, sizeof p);
    memset(&o, 0, sizeof o);
    p.gate = 1.0;
    memset(&in, 0, sizeof in);
    in.cfg = cfg;
    in.key_on = in.hb_ok = in.deadman = in.in_sector = 1;
    in.push_home = 1;
    in.speed_req_pm = 1000;  /* 17,500 rpm */
    in.diff_pm = 0;
    in.rpm_top = in.rpm_bot = 0;
    fdr_init(&d);
    int feed_edges = 0, shots = 0, muz = 0, acks = 0, ack_res = -1, prev_req = 0, kick_drop = 0;
    uint32_t t_fire = 0;
    for (uint32_t t = 1; t < 3000; t++) {
        in.t_ms = t;
        in.cmd = FDR_CMD_NONE;
        if (!t_fire && o.gate_state == GS_STAGED && o.ready_flags & RDY_READY) {
            in.cmd = FDR_CMD_FIRE;
            in.fire_id = 7;
            t_fire = t;
        }
        fdr_step(&d, &in, &o);
        if (o.ack_valid) { acks++; ack_res = o.ack; }
        for (int i = 0; i < o.nev; i++) {
            shots += o.ev[i].code == EV_SHOT;
            muz += o.ev[i].code == EV_MUZZLE;
        }
        feed_edges += o.feed_req && !prev_req;
        prev_req = o.feed_req;
        kick_drop += !o.kick;
        plant_step(&p, &o, cfg, &in, t);
    }
    CHECK(t_fire > 0 && t_fire < 1500);               /* homed, loaded, wheels in window, READY */
    CHECK(acks == 1 && ack_res == RES_OK);
    CHECK(feed_edges == 1 && shots == 1 && muz == 1);
    CHECK(o.gate_state == GS_STAGED && o.shots_confirmed == 1 && o.fault_bits == 0 && kick_drop == 0);
}

/* ---- protocol-level gates ---------------------------------------------------------------------------------------------------------------- */
static struct { uint8_t acks[64][4]; int nack; uint8_t hello_ack[18]; int got_hello; } rec;
static fw_decoder_t txdec;

static void on_tx(void *ctx, const uint8_t *d, size_t n) {
    (void)ctx;
    fw_frame_t fr;
    for (size_t i = 0; i < n; i++)
        if (fw_decoder_byte(&txdec, d[i], &fr)) {
            if (fr.id == ID_ACK && rec.nack < 64) memcpy(rec.acks[rec.nack++], fr.payload, 4);
            if (fr.id == ID_HELLO_ACK) { memcpy(rec.hello_ack, fr.payload, 18); rec.got_hello = 1; }
        }
}

static fw_t f;
static uint32_t now;
static uint8_t seq;
static fw_in_t in0;

static void send_msg(uint8_t id, const uint8_t *p, size_t n) {
    uint8_t w[80];
    size_t m = fw_frame_encode(id, seq++, p, n, w);
    fw_rx(&f, w, m);
}
static void run(uint32_t ms, int heartbeats) {
    for (uint32_t i = 0; i < ms; i++) {
        now++;
        if (heartbeats && now % 50 == 0) { uint8_t hb[3] = {1, 0, HB_DEADMAN_OK}; send_msg(ID_HEARTBEAT, hb, 3); }
        fw_tick(&f, now, &in0);
    }
}
static int last_ack_result(uint8_t id) {
    for (int i = rec.nack - 1; i >= 0; i--)
        if (rec.acks[i][0] == id) return rec.acks[i][2];
    return -1;
}

int main(void) {
    CHECK(fw_crc16((const uint8_t *)"123456789", 9) == 0x29B1);
    CHECK(fw_crc32((const uint8_t *)"123456789", 9) == 0xCBF43926u);
    uint8_t enc[8];
    const uint8_t d1[] = {0x11, 0x22, 0x00, 0x33};
    CHECK(fw_cobs_encode(d1, 4, enc) == 5 && !memcmp(enc, "\x03\x11\x22\x02\x33", 5));

    /* DShot: the article's example (throttle 1046, no telemetry bit): regular crc 0110, inverted 1001 */
    CHECK(dshot_frame(1046, 0, 0) == (uint16_t)(1046 << 5 | 0x6));
    CHECK(dshot_frame(1046, 0, 1) == (uint16_t)(1046 << 5 | 0x9));
    uint16_t pl = 0;
    const uint16_t payload = (uint16_t)((0x123 << 4) | ((~(0x1 ^ 0x2 ^ 0x3)) & 0xF));  /* 12-bit value 0x123 + inverted nibble xor */
    CHECK(dshot_gcr_decode(dshot_gcr_encode(payload), &pl) == 0 && pl == 0x123);
    CHECK(dshot_gcr_decode(dshot_gcr_encode(payload) ^ 0x10, &pl) != 0);
    CHECK(dshot_payload_rpm(0x0E8, 7) == (int32_t)(60000000u / 232u / 7u));  /* e = 0, m = 232 us */
    CHECK(dshot_payload_rpm(0xFFF, 7) == 0);

    int32_t cfg[CFG_COUNT];
    fw_cfg_defaults(cfg);
    CHECK(fdr_rpm_cap(cfg) == 18143);                 /* floor(38 * 60 / (pi * 0.040)) */
    cfg[CFG_WHEEL_D_UM] = 25000;
    CHECK(fdr_rpm_cap(cfg) == 29029);                 /* floor(38 * 60 / (pi * 0.025)) = 29029.86 */
    uint8_t img[FW_CFG_IMAGE_SIZE];
    int32_t back[CFG_COUNT];
    fw_cfg_defaults(cfg);
    fw_cfg_pack(cfg, img);
    CHECK(fw_cfg_unpack(img, sizeof img, back) == 0 && !memcmp(cfg + 1, back + 1, sizeof cfg - sizeof cfg[0]));
    img[4] = 2;                                       /* a lite (v2) image is refused */
    CHECK(fw_cfg_unpack(img, sizeof img, back) == -1);

    test_feeder();

    fw_hal_t hal = {on_tx, NULL, NULL, NULL};
    const uint8_t uid[8] = {1, 2, 3, 4, 5, 6, 7, 8};
    uint8_t hp[6] = {1, 0, 4, 3, 2, 1};
    in0.key_on = 1;
    in0.gate_home = in0.push_home = 1;  /* gate parked at LOAD and rod home: the feeder homes without a fault */
    fw_init(&f, &hal, uid, NULL, 0);
    fw_rx(&f, (const uint8_t *)"\0", 1);
    send_msg(ID_HELLO, hp, 6);
    run(900, 1);
    CHECK(rec.got_hello && rec.hello_ack[1] == 1 && rec.hello_ack[3] == 2);  /* proto 1.1, fw_major 2 */
    CHECK(f.state == ST_SAFE);
    run(300, 1);
    CHECK(f.state == ST_IDLE);
    uint8_t cs[5] = {CFG_V_CAP_DMPS, 0, 0, 0, 0};
    cs[1] = 381 & 0xFF; cs[2] = 381 >> 8;
    rec.nack = 0;
    send_msg(ID_CFG_SET, cs, 5);
    run(3, 1);
    CHECK(last_ack_result(ID_CFG_SET) == RES_BAD_STATE);  /* maintenance key outside SAFE */
    uint8_t fire2[3] = {1, 0, 2};
    send_msg(ID_FIRE, fire2, 3);
    run(3, 1);
    CHECK(last_ack_result(ID_FIRE) == RES_BAD_STATE);     /* not ENGAGING; the n != 1 gate is checked from Python in an ENGAGING session */
    printf(fails ? "selftest: %d FAILED\n" : "selftest: ok\n", fails);
    return fails != 0;
}
