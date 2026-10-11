/* TMC2209 single-wire UART configuration (current, 16 microsteps with interpolation, stealthChop below ~60 rpm / spreadCycle above).
 * One driver per UART (see pins.h). UNTESTED on hardware: the register values follow the TMC2209 datasheet; verify on the bench (README).
 * SPDX-License-Identifier: BSD-3-Clause */
#include <stdio.h>
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "pico/time.h"
#include "pins.h"
#include "tmc2209.h"

#define REG_GCONF 0x00
#define REG_IFCNT 0x02
#define REG_IHOLD_IRUN 0x10
#define REG_TPOWERDOWN 0x11
#define REG_TPWMTHRS 0x13
#define REG_CHOPCONF 0x6C

/* GCONF: pdn_disable (bit 6, PDN_UART pin is the UART), mstep_reg_select (bit 7, microsteps from MRES), multistep_filt (bit 8). I_scale_analog = 0:
 * the current comes from IHOLD_IRUN, not the VREF pot. en_spreadcycle = 0: stealthChop below TPWMTHRS, spreadCycle above. */
#define GCONF_VAL 0x000001C0u
/* CHOPCONF: reset value 0x10000053 (intpol = 1, toff = 3, hstrt = 5) with MRES = 4 (bits 27..24): 16 microsteps, interpolated to 256. */
#define CHOPCONF_VAL (0x10000053u | (4u << 24))

static uint8_t crc8(const uint8_t *d, int n) {
    uint8_t crc = 0;
    for (int i = 0; i < n; i++) {
        uint8_t b = d[i];
        for (int j = 0; j < 8; j++) {
            crc = ((crc >> 7) ^ (b & 1)) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
            b >>= 1;
        }
    }
    return crc;
}

static void drain(uart_inst_t *u) {
    while (uart_is_readable(u)) (void)uart_getc(u);
}

/* wire: TX through 1 kohm to the PDN_UART node and RX on the node, so every byte sent is also received (echo) */
static int rd(uart_inst_t *u, uint8_t addr, uint8_t reg, uint32_t *val) {
    uint8_t q[4] = {0x05, addr, reg, 0};
    q[3] = crc8(q, 3);
    drain(u);
    uart_write_blocking(u, q, 4);
    uint8_t r[12];
    for (int i = 0; i < 12; i++) {
        if (!uart_is_readable_within_us(u, 5000)) return -1;
        r[i] = uart_getc(u);
    }
    const uint8_t *p = r + 4;  /* skip the echo of our request */
    if (p[0] != 0x05 || p[1] != 0xFF || p[2] != reg || p[7] != crc8(p, 7)) return -2;
    *val = (uint32_t)p[3] << 24 | (uint32_t)p[4] << 16 | (uint32_t)p[5] << 8 | p[6];
    return 0;
}

static void wr(uart_inst_t *u, uint8_t addr, uint8_t reg, uint32_t val) {
    uint8_t w[8] = {0x05, addr, (uint8_t)(reg | 0x80), (uint8_t)(val >> 24), (uint8_t)(val >> 16), (uint8_t)(val >> 8), (uint8_t)val, 0};
    w[7] = crc8(w, 7);
    drain(u);
    uart_write_blocking(u, w, 8);
    uart_tx_wait_blocking(u);
    sleep_us(500);
    drain(u);  /* the echo */
}

/* 0 = configured and read back, <0 = no / bad reply (the driver keeps its strapped standalone settings and the VREF current) */
int tmc2209_init(int axis, int irun, int ihold) {
    uart_inst_t *u = axis == 0 ? uart1 : uart0;
    const int tx = axis == 0 ? PIN_PAN_UART_TX : PIN_TILT_UART_TX, rx = axis == 0 ? PIN_PAN_UART_RX : PIN_TILT_UART_RX;
    const uint8_t addr = 3;  /* MS1 = MS2 = VIO (16 microsteps strap) = address 3 on each driver's own wire */
    uart_init(u, 115200);
    gpio_set_function(tx, GPIO_FUNC_UART);
    gpio_set_function(rx, GPIO_FUNC_UART);
    uint32_t v;
    int r = rd(u, addr, REG_IFCNT, &v);
    if (r) return r;
    wr(u, addr, REG_GCONF, GCONF_VAL);
    wr(u, addr, REG_CHOPCONF, CHOPCONF_VAL);
    wr(u, addr, REG_IHOLD_IRUN, (uint32_t)ihold | (uint32_t)irun << 8 | 6u << 16);
    wr(u, addr, REG_TPOWERDOWN, 20);
    wr(u, addr, REG_TPWMTHRS, 234);  /* TSTEP = 12 MHz / (step rate x 16): 234 = 3200 steps/s = 60 rpm of a 200-step motor */
    if (rd(u, addr, REG_CHOPCONF, &v) || ((v >> 24) & 0xF) != 4) return -3;
    if (rd(u, addr, REG_IHOLD_IRUN, &v) || (int)(v & 0x1F) != ihold || (int)((v >> 8) & 0x1F) != irun) return -4;
    return 0;
}
