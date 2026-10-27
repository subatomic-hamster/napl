/* cuas2 Pico HAL (fork of the v1 hal_pico.c: that one claims GP12/13/15/16 for the flywheel PWM, tach and LEDs, which the cuas2 map uses for DShot, the
 * muzzle beam and the status LED). Step generator (timer ISR, 50 kHz, unchanged), pusher + gate servo PWM, DShot via PIO, beam / hall / sense inputs, the muzzle
 * beam edge capture, FEED_REQ and KICK outputs, the flash config sector. UNTESTED on hardware (README lists the bench tests).
 * SPDX-License-Identifier: BSD-3-Clause */
#include "hardware/clocks.h"
#include "hardware/flash.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"
#include "hardware/sync.h"
#include "hardware/timer.h"
#include "pico/flash.h"
#include "pico/stdlib.h"
#include "hal_cuas2.h"
#include "pins.h"

#define STEP_ALARM 2                    /* hardware alarm 2; the SDK's default pool uses alarm 3 */
#define STEP_ISR_US 20                  /* 50 kHz: one STEP pulse per 2 ticks at most = 25 ksteps/s */
#define TICKS_PER_WINDOW 50             /* one planner millisecond */
#define FLASH_CFG_OFFSET (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE)

static const uint8_t step_pin[2] = {PIN_PAN_STEP, PIN_TILT_STEP}, dir_pin[2] = {PIN_PAN_DIR, PIN_TILT_DIR};
static volatile int32_t target[2];      /* cumulative microsteps the planner asked for (written by the main loop) */
static volatile uint8_t dir_inv[2];
static volatile int32_t done[2];        /* written by the ISR only */
static int16_t win_n[2];
static uint8_t acc[2], pulse[2], win_tick;

static void __not_in_flash_func(step_isr)(void) {
    hw_clear_bits(&timer_hw->intr, 1u << STEP_ALARM);
    uint32_t next = timer_hw->alarm[STEP_ALARM] + STEP_ISR_US;
    const uint32_t now = timer_hw->timerawl;
    if ((int32_t)(next - now) < 3) next = now + STEP_ISR_US;  /* late (interrupts were off): re-base, an alarm in the past would not fire for 71 min */
    timer_hw->alarm[STEP_ALARM] = next;
    for (int i = 0; i < 2; i++)
        if (pulse[i]) {
            gpio_put(step_pin[i], 0);
            pulse[i] = 0;
        }
    if (win_tick == 0) {  /* new window: take what the planner has queued since (at most 25 steps, the rest waits for the next window) */
        for (int i = 0; i < 2; i++) {
            int32_t d = target[i] - done[i];
            d = d > 25 ? 25 : (d < -25 ? -25 : d);
            win_n[i] = (int16_t)d;
            if (d) {
                gpio_put(dir_pin[i], (d > 0) ^ dir_inv[i]);
                __asm volatile("nop\nnop\nnop\nnop\nnop\nnop");  /* DIR setup time before the first STEP edge */
            }
        }
    }
    for (int i = 0; i < 2; i++) {
        const int n = win_n[i] < 0 ? -win_n[i] : win_n[i];
        acc[i] = (uint8_t)(acc[i] + n);
        if (acc[i] >= TICKS_PER_WINDOW) {  /* Bresenham: n steps spread over the 50 ticks of the window */
            acc[i] = (uint8_t)(acc[i] - TICKS_PER_WINDOW);
            gpio_put(step_pin[i], 1);
            pulse[i] = 1;
            done[i] += win_n[i] > 0 ? 1 : -1;
        }
    }
    win_tick = (uint8_t)((win_tick + 1) % TICKS_PER_WINDOW);
}

/* ---- muzzle beam: a 22 mm ball at 30 m/s blocks it for ~0.7 ms, shorter than the 1 ms tick, so the edges are captured in a GPIO IRQ ------------------------- */
static volatile uint32_t mz_rise_us, mz_dt_us;
static volatile uint8_t mz_pending, mz_closed;  /* pending: a rising edge not yet delivered to the core; closed: its falling edge has been seen */

static void __not_in_flash_func(muzzle_irq)(uint gpio, uint32_t events) {
    if (gpio != PIN_BM_MUZZLE) return;
    const uint32_t t = timer_hw->timerawl;
    if (events & GPIO_IRQ_EDGE_RISE) {
        mz_rise_us = t;
        mz_closed = 0;
        mz_pending = 1;
    }
    if ((events & GPIO_IRQ_EDGE_FALL) && mz_pending && !mz_closed) {
        mz_dt_us = t - mz_rise_us;
        mz_closed = 1;
    }
}

static uint16_t servo_wrap;

static void pwm_setup(uint pin, float hz, uint16_t wrap) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    const uint slice = pwm_gpio_to_slice_num(pin);
    pwm_config c = pwm_get_default_config();
    pwm_config_set_clkdiv(&c, (float)clock_get_hz(clk_sys) / (hz * (float)(wrap + 1)));
    pwm_config_set_wrap(&c, wrap);
    pwm_init(slice, &c, true);
}

static uint16_t dshot_now[2];

void hal_init(void) {
    gpio_init(PIN_PAN_EN); gpio_put(PIN_PAN_EN, 1); gpio_set_dir(PIN_PAN_EN, GPIO_OUT);  /* shared by both drivers; disabled until the core enables */
    for (int i = 0; i < 2; i++) {
        gpio_init(step_pin[i]); gpio_put(step_pin[i], 0); gpio_set_dir(step_pin[i], GPIO_OUT);
        gpio_init(dir_pin[i]); gpio_put(dir_pin[i], 0); gpio_set_dir(dir_pin[i], GPIO_OUT);
    }
    const uint8_t pd[] = {PIN_ESTOP_SENSE, PIN_KEY_SENSE, PIN_DIAG, PIN_BM_BREECH, PIN_BM_MUZZLE},  /* floating = pressed / off / no fault / no ball: the safe reading */
        pu[] = {PIN_PAN_HALL, PIN_TILT_HALL, PIN_GATE_HOME, PIN_PUSH_HOME};
    for (unsigned i = 0; i < sizeof pd; i++) { gpio_init(pd[i]); gpio_set_dir(pd[i], GPIO_IN); gpio_pull_down(pd[i]); }
    for (unsigned i = 0; i < sizeof pu; i++) { gpio_init(pu[i]); gpio_set_dir(pu[i], GPIO_IN); gpio_pull_up(pu[i]); }
    const uint8_t outs[] = {PIN_BUZZER, PIN_FEED_REQ, PIN_KICK, PIN_STATUS_LED};  /* GP15 stays a low output: the WS2812 program is not written (README) */
    for (unsigned i = 0; i < sizeof outs; i++) { gpio_init(outs[i]); gpio_put(outs[i], 0); gpio_set_dir(outs[i], GPIO_OUT); }

    pwm_setup(PIN_PUSHER_SERVO, SERVO_PWM_HZ, 19999);   /* 1 us per count */
    pwm_set_gpio_level(PIN_PUSHER_SERVO, 1100);
    pwm_setup(PIN_GATE_SERVO, SERVO_PWM_HZ, 19999);
    pwm_set_gpio_level(PIN_GATE_SERVO, 1150);
    servo_wrap = 19999;

    dshot_pio_init();

    gpio_set_irq_enabled_with_callback(PIN_BM_MUZZLE, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, muzzle_irq);

    hardware_alarm_claim(STEP_ALARM);
    irq_set_exclusive_handler(TIMER_IRQ_2, step_isr);
    hw_set_bits(&timer_hw->inte, 1u << STEP_ALARM);
    timer_hw->alarm[STEP_ALARM] = timer_hw->timerawl + 100;
    irq_set_priority(TIMER_IRQ_2, 0);                                /* above USB */
    irq_set_enabled(TIMER_IRQ_2, true);
}

void hal_read_inputs(fw_in_t *in, int pole_pairs) {
    const uint32_t g = gpio_get_all();
    in->estop_open = !((g >> PIN_ESTOP_SENSE) & 1);
    in->key_on = (g >> PIN_KEY_SENSE) & 1;
    in->hall[0] = !((g >> PIN_PAN_HALL) & 1);  /* active low */
    in->hall[1] = !((g >> PIN_TILT_HALL) & 1);
    in->diag = (g >> PIN_DIAG) & 1;
    in->beam_breech = (g >> PIN_BM_BREECH) & 1;
    in->gate_home = !((g >> PIN_GATE_HOME) & 1);
    in->push_home = !((g >> PIN_PUSH_HOME) & 1);
    /* muzzle: blocked now, or blocked since the last tick (an edge pair shorter than 1 ms); the dt is delivered with the tick that sees the closed pulse */
    const uint32_t save = save_and_disable_interrupts();
    const int pend = mz_pending, closed = mz_closed;
    const uint32_t dt = mz_dt_us;
    if (pend && closed) mz_pending = 0;
    restore_interrupts(save);
    in->beam_muzzle = (g >> PIN_BM_MUZZLE) & 1 || pend;
    in->muzzle_dt_us = (pend && closed) ? dt : 0;
    dshot_pio_service(dshot_now, pole_pairs, in->rpm);
}

void hal_apply(const fw_out_t *o) {
    static int32_t emitted_total[2];
    gpio_put(PIN_PAN_EN, !o->steppers_en);
    for (int i = 0; i < 2; i++) {
        dir_inv[i] = o->dir_invert[i];
        if (o->steppers_en) {
            emitted_total[i] += o->step_n[i];
            target[i] = emitted_total[i];
        } else {
            emitted_total[i] = target[i] = done[i];  /* drop any backlog while the drivers are off */
        }
    }
    dshot_now[0] = o->dshot[0];
    dshot_now[1] = o->dshot[1];
    pwm_set_gpio_level(PIN_PUSHER_SERVO, o->pusher_us);
    pwm_set_gpio_level(PIN_GATE_SERVO, o->gate_us);
    gpio_put(PIN_FEED_REQ, o->feed_req);
    gpio_put(PIN_KICK, o->kick_level);   /* toggles every 5 ms while the core allows (I15); a stuck level stops the supervisor's retriggerable one-shot */
    gpio_put(PIN_BUZZER, o->buzzer);
    (void)servo_wrap;
}

void hal_safe_outputs(void) {
    dshot_now[0] = dshot_now[1] = 0;
    gpio_put(PIN_FEED_REQ, 0);
    gpio_put(PIN_PAN_EN, 1);
}

/* ---- config sector (the last 4 kB of flash) ---------------------------------------------------------------------------------------- */
const uint8_t *hal_flash_cfg(void) { return (const uint8_t *)(XIP_BASE + FLASH_CFG_OFFSET); }

static uint8_t page[FLASH_PAGE_SIZE];
static void flash_job(void *param) {
    (void)param;
    flash_range_erase(FLASH_CFG_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(FLASH_CFG_OFFSET, page, FLASH_PAGE_SIZE);
}

int hal_flash_write(void *ctx, const uint8_t *img, size_t n) {
    (void)ctx;
    for (size_t i = 0; i < sizeof page; i++) page[i] = i < n ? img[i] : 0xFF;
    hal_safe_outputs();
    return flash_safe_execute(flash_job, NULL, 500) == PICO_OK ? 0 : -1;  /* interrupts are off during the ~50 ms erase: CFG_SAVE is allowed only in SAFE / IDLE */
}
