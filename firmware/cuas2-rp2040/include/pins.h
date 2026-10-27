/* cuas2 Pico GPIO map: the single source (docs/cuas2/spec.md section 6.5). SPDX-License-Identifier: BSD-3-Clause
 *
 * Kept from v1: GP0/1 tilt TMC UART, GP2/3/4 pan STEP/DIR/EN (EN shared with tilt: GP7 is freed), GP5/6 tilt STEP/DIR, GP8/9 halls, GP10 E-stop chain sense,
 * GP11 key sense, GP14 pusher servo (PWM7 A), GP17 buzzer, GP20/21 pan TMC UART, GP22 DIAG.
 *
 * No spare GPIO is left (GP0-GP22 and GP26-GP28 are all used; GP23, GP24, GP25 and GP29 are wired inside the Pico board: SMPS mode, VBUS detect, the on-board
 * LED, VSYS/3). The hardware shot counter (two 74HCT4017 on the muzzle beam, docs/cuas2/wiring.md 3.4) therefore has NO pin of its own, and it is NOT upstream of the
 * GP10 tap: its Nth pulse clamps the gate of the supervisor's weapon MOSFET and leaves relay, key and E-stop closed, so GP10 sees nothing. The firmware infers the trip
 * from weapon power lost (both ESC telemetry streams drop together, key on, E-stop closed) plus the per-key-session beam-pulse count (fsm.c, feeder_stage()).
 * Alternative that costs a safety indicator: give up the buzzer (GP17) and sense the counter output directly.
 */
#ifndef PINS_H
#define PINS_H

#define PIN_TILT_UART_TX 0   /* UART0 TX -> 1 kohm -> tilt TMC2209 PDN_UART */
#define PIN_TILT_UART_RX 1
#define PIN_PAN_STEP 2
#define PIN_PAN_DIR 3
#define PIN_PAN_EN 4         /* active low, shared by both drivers; external 10 k pull-up keeps them off while the Pico resets */
#define PIN_TILT_STEP 5
#define PIN_TILT_DIR 6
#define PIN_BM_BREECH 7      /* IR phototransistor across the bore, 1 = blocked (staged ball) */
#define PIN_PAN_HALL 8       /* A3144 open collector, active low, internal pull-up */
#define PIN_TILT_HALL 9
#define PIN_ESTOP_SENSE 10   /* 1 = key/E-stop/counter chain closed; internal pull-down: a broken wire reads "pressed" */
#define PIN_KEY_SENSE 11     /* 1 = arm key on; internal pull-down */
#define PIN_DSHOT_TOP 12     /* PIO0 SM0, bidirectional DShot300, 10 k pull-down */
#define PIN_DSHOT_BOT 13     /* PIO0 SM1 */
#define PIN_PUSHER_SERVO 14  /* PWM7 A, 50 Hz; supply behind the hardware limiter */
#define PIN_STATUS_LED 15    /* WS2812 on PIO1 */
#define PIN_BM_MUZZLE 16     /* muzzle beam, GPIO IRQ both edges, 1 us timestamps */
#define PIN_BUZZER 17
#define PIN_GATE_SERVO 18    /* PWM1 A, 50 Hz; signal pull-down */
#define PIN_FEED_REQ 19      /* out -> hardware limiter trigger, 10 k pull-down */
#define PIN_PAN_UART_TX 20   /* UART1 TX -> 1 kohm -> pan TMC2209 PDN_UART */
#define PIN_PAN_UART_RX 21
#define PIN_DIAG 22          /* TMC2209 DIAG of both drivers, active high; internal pull-down */
#define PIN_GATE_HOME 26     /* A3144, active low */
#define PIN_PUSH_HOME 27     /* A3144, active low */
#define PIN_KICK 28          /* out, 100 Hz square wave -> link-alive supervisor; pull-down */

#define SERVO_PWM_HZ 50
#define DSHOT_BIT_RATE 300000

#endif
