/* cuas2 RP2040 firmware: USB CDC (pico stdio_usb over TinyUSB) <-> fw_core, 1 kHz tick, hardware watchdog 8 s. UNTESTED on hardware.
 * SPDX-License-Identifier: BSD-3-Clause */
#include <stdio.h>
#include "hardware/watchdog.h"
#include "pico/bootrom.h"
#include "pico/stdlib.h"
#include "pico/unique_id.h"
#include "fw_core.h"
#include "hal_cuas2.h"
#include "tmc2209.h"

static fw_t fw;

static void usb_tx(void *ctx, const uint8_t *d, size_t n) {
    (void)ctx;
    stdio_put_string((const char *)d, (int)n, false, false);  /* raw bytes; dropped by stdio_usb when no host has the port open */
}

int main(void) {
    hal_init();  /* outputs off first: the stepper drivers disabled, DShot 0, FEED_REQ and KICK low */
    stdio_init_all();
    const int wd_reset = watchdog_caused_reboot();

    pico_unique_board_id_t id;
    pico_get_unique_board_id(&id);
    const fw_hal_t hal = {usb_tx, hal_flash_write, NULL, NULL};
    fw_init(&fw, &hal, id.id, hal_flash_cfg(), FW_CFG_IMAGE_SIZE);

    const int pan = tmc2209_init(0, TMC_IRUN_CS, TMC_IHOLD_CS), tilt = tmc2209_init(1, TMC_IRUN_CS, TMC_IHOLD_CS);
    char note[48];
    snprintf(note, sizeof note, "tmc pan=%d tilt=%d%s", pan, tilt, wd_reset ? " wd-reset" : "");  /* 0 = configured; <0 = no reply, strap + VREF apply */
    fw_set_boot_note(&fw, note);

    watchdog_enable(8000, true);  /* a hung firmware resets into SAFE (RP2040 maximum is ~8.3 s); KICK stops with it and the supervisor opens the relay */
    const uint64_t t0 = time_us_64();
    uint32_t last_ms = 0xFFFFFFFFu;
    fw_in_t in;
    for (;;) {
        watchdog_update();
        for (int i = 0; i < 64; i++) {  /* drain what the host sent */
            const int c = getchar_timeout_us(0);
            if (c == PICO_ERROR_TIMEOUT) break;
            const uint8_t b = (uint8_t)c;
            fw_rx(&fw, &b, 1);
        }
        const uint32_t ms = (uint32_t)((time_us_64() - t0) / 1000);
        if (ms != last_ms) {  /* a stall skips ticks (the core's timers use ms, so the 300 ms watchdog stays honest) */
            last_ms = ms;
            hal_read_inputs(&in, fw.cfg[CFG_POLE_PAIRS]);
            fw_tick(&fw, ms, &in);
            hal_apply(&fw.out);
            if (fw.out.request_bootsel) {
                hal_safe_outputs();
                sleep_ms(20);
                reset_usb_boot(0, 0);
            }
        }
    }
}
