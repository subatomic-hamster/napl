/* Pico HAL: GPIO, step generator (timer ISR, 50 kHz), flywheel and servo PWM, input sampling, flash config sector, LEDs.
 * UNTESTED on hardware (README lists the bench tests). SPDX-License-Identifier: BSD-3-Clause */
#include "hardware/clocks.h"
#include "hardware/flash.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"
#include "hardware/sync.h"
#include "hardware/timer.h"
#include "pico/flash.h"
#include "pico/stdlib.h"
#include "hal_pico.h"
#include "pins.h"

#define STEP_ALARM 2                    /* hardware alarm 2; the SDK's default pool uses alarm 3 */
#define STEP_ISR_US 20                  /* 50 kHz: one STEP pulse per 2 ticks at most = 25 ksteps/s */
#define TICKS_PER_WINDOW 50             /* one planner millisecond */
#define FLASH_CFG_OFFSET (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE)

static const uint8_t step_pin[2] = {PIN_PAN_STEP, PIN_TILT_STEP}, dir_pin[2] = {PIN_PAN_DIR, PIN_TILT_DIR}, en_pin[2] = {PIN_PAN_EN, PIN_TILT_EN};
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

static uint pwm_setup(uint pin, float hz, uint16_t wrap_hint, uint16_t *wrap_out) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    const uint slice = pwm_gpio_to_slice_num(pin);
    pwm_config c = pwm_get_default_config();
    const float div = (float)clock_get_hz(clk_sys) / (hz * (float)(wrap_hint + 1));
    pwm_config_set_clkdiv(&c, div);
    pwm_config_set_wrap(&c, wrap_hint);
    pwm_init(slice, &c, true);
    *wrap_out = wrap_hint;
    return slice;
}

static uint16_t fw_wrap, servo_wrap;
static uint fw_slice_pin = PIN_FLYWHEEL_PWM, servo_pin = PIN_PUSHER_SERVO;

void hal_init(void) {
    for (int i = 0; i < 2; i++) {
        gpio_init(en_pin[i]); gpio_put(en_pin[i], 1); gpio_set_dir(en_pin[i], GPIO_OUT);  /* disabled until the core enables */
        gpio_init(step_pin[i]); gpio_put(step_pin[i], 0); gpio_set_dir(step_pin[i], GPIO_OUT);
        gpio_init(dir_pin[i]); gpio_put(dir_pin[i], 0); gpio_set_dir(dir_pin[i], GPIO_OUT);
    }
    const uint8_t pd[] = {PIN_ESTOP_SENSE, PIN_KEY_SENSE, PIN_DIAG}, pu[] = {PIN_PAN_HALL, PIN_TILT_HALL};
    for (unsigned i = 0; i < sizeof pd; i++) { gpio_init(pd[i]); gpio_set_dir(pd[i], GPIO_IN); gpio_pull_down(pd[i]); }  /* floating = pressed / off / no fault: safe */
    for (unsigned i = 0; i < sizeof pu; i++) { gpio_init(pu[i]); gpio_set_dir(pu[i], GPIO_IN); gpio_pull_up(pu[i]); }
    const uint8_t outs[] = {PIN_LED_RED, PIN_LED_GREEN, PIN_BUZZER};
    for (unsigned i = 0; i < sizeof outs; i++) { gpio_init(outs[i]); gpio_put(outs[i], 0); gpio_set_dir(outs[i], GPIO_OUT); }
    gpio_init(PIN_TACH); gpio_set_dir(PIN_TACH, GPIO_IN); gpio_pull_down(PIN_TACH);

    pwm_setup(fw_slice_pin, FLYWHEEL_PWM_HZ, 999, &fw_wrap);       /* 1000 steps = permille resolution at 20 kHz */
    pwm_set_gpio_level(fw_slice_pin, 0);
    pwm_setup(servo_pin, SERVO_PWM_HZ, 19999, &servo_wrap);        /* 1 us per count */
    pwm_set_gpio_level(servo_pin, 1100);

    hardware_alarm_claim(STEP_ALARM);
    irq_set_exclusive_handler(TIMER_IRQ_2, step_isr);
    hw_set_bits(&timer_hw->inte, 1u << STEP_ALARM);
    timer_hw->alarm[STEP_ALARM] = timer_hw->timerawl + 100;
    irq_set_priority(TIMER_IRQ_2, 0);                                /* above USB */
    irq_set_enabled(TIMER_IRQ_2, true);
}

void hal_read_inputs(fw_in_t *in) {
    const uint32_t g = gpio_get_all();
    in->estop_open = !((g >> PIN_ESTOP_SENSE) & 1);
    in->key_on = (g >> PIN_KEY_SENSE) & 1;
    in->hall[0] = !((g >> PIN_PAN_HALL) & 1);  /* active low */
    in->hall[1] = !((g >> PIN_TILT_HALL) & 1);
    in->diag = (g >> PIN_DIAG) & 1;
}

void hal_apply(const fw_out_t *o) {
    static int32_t emitted_total[2];
    for (int i = 0; i < 2; i++) {
        gpio_put(en_pin[i], !o->steppers_en);
        dir_inv[i] = o->dir_invert[i];
        if (o->steppers_en) {
            emitted_total[i] += o->step_n[i];
            target[i] = emitted_total[i];
        } else {
            emitted_total[i] = target[i] = done[i];  /* drop any backlog while the drivers are off */
        }
    }
    pwm_set_gpio_level(fw_slice_pin, o->fw_duty > 1000 ? 1000 : o->fw_duty);
    pwm_set_gpio_level(servo_pin, o->pusher_us);
    gpio_put(PIN_LED_RED, o->led_red);
    gpio_put(PIN_LED_GREEN, o->led_green);
    gpio_put(PIN_BUZZER, o->buzzer);
}

void hal_safe_outputs(void) {
    pwm_set_gpio_level(fw_slice_pin, 0);
    for (int i = 0; i < 2; i++) gpio_put(en_pin[i], 1);
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
