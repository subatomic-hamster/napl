/* DShot300 bidirectional on PIO0 (SM0 top, SM1 bottom): one frame per millisecond per ESC, the eRPM reply of the previous frame decoded by dshot.c.
 * UNTESTED on hardware (README lists the bench tests). SPDX-License-Identifier: BSD-3-Clause */
#include "hardware/pio.h"
#include "hal_cuas2.h"
#include "pins.h"
#include "dshot.pio.h"

static PIO pio = pio0;
static uint offset;
static const uint dpin[2] = {PIN_DSHOT_TOP, PIN_DSHOT_BOT};

void dshot_pio_init(void) {
    offset = pio_add_program(pio, &dshot_bidir_program);
    for (uint i = 0; i < 2; i++) {
        pio_sm_claim(pio, i);
        dshot_bidir_program_init(pio, i, offset, dpin[i], DSHOT_BIT_RATE);
        pio_sm_set_enabled(pio, i, true);
    }
}

void dshot_pio_service(const uint16_t value[2], int pole_pairs, int32_t rpm_out[2]) {
    for (uint i = 0; i < 2; i++) {
        rpm_out[i] = -1;
        if (!pio_sm_is_rx_fifo_empty(pio, i)) {  /* the reply to the previous frame */
            const uint32_t v21 = pio_sm_get(pio, i) & 0x1FFFFFu;
            uint16_t payload;
            if (dshot_gcr_decode(v21, &payload) == 0) rpm_out[i] = dshot_payload_rpm(payload, pole_pairs);
        }
        /* restart: the state machine may still wait for a reply edge that never came (no ESC, noise) */
        pio_sm_set_enabled(pio, i, false);
        pio_sm_clear_fifos(pio, i);
        pio_sm_restart(pio, i);
        pio_sm_exec(pio, i, pio_encode_jmp(offset));
        pio_sm_set_enabled(pio, i, true);
        pio_sm_put(pio, i, (uint32_t)dshot_frame(value[i], 1, 1) << 16);
    }
}
