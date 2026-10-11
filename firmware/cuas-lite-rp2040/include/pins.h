/* Pico GPIO map: the single source (spec 6.6, with the changes listed below). Checked against parts/electronics/raspberry-pi-pico-h.toml.
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Changes against the spec 6.6 proposal (flagged in the README):
 *   - servo moved GP13 -> GP14 (and tach GP14 -> GP13): GP12 (PWM6 A, flywheel, 20 kHz) and GP13 (PWM6 B) share ONE PWM slice, so a 50 Hz servo on
 *     GP13 would have forced the flywheel gates to 50 Hz (or the servo to 20 kHz). GP14 is PWM7 A: its own slice.
 *   - TMC2209 UART added: tilt on UART0 GP0 (TX) / GP1 (RX), pan on UART1 GP20 (TX) / GP21 (RX). The spec's "debug UART = GP0/1" is dropped (DEBUG
 *     frames cover it). Two separate wires, because the spec's 16-microstep strap (MS1 = MS2 = VIO) gives BOTH drivers UART address 3.
 *   - DIAG (both drivers, wired-OR) added on GP22 for the driver_diag fault.
 */
#ifndef PINS_H
#define PINS_H

#define PIN_TILT_UART_TX 0   /* UART0 TX -> 1 kohm -> tilt TMC2209 PDN_UART */
#define PIN_TILT_UART_RX 1   /* UART0 RX <- tilt PDN_UART (same node) */
#define PIN_PAN_STEP 2
#define PIN_PAN_DIR 3
#define PIN_PAN_EN 4         /* active low; external 10 k pull-up to VIO keeps the driver off while the Pico resets */
#define PIN_TILT_STEP 5
#define PIN_TILT_DIR 6
#define PIN_TILT_EN 7
#define PIN_PAN_HALL 8       /* A3144 open collector, active low, internal pull-up */
#define PIN_TILT_HALL 9
#define PIN_ESTOP_SENSE 10   /* 1 = E-stop circuit closed (released); internal pull-down: a broken wire reads "pressed" */
#define PIN_KEY_SENSE 11     /* 1 = arm key on; internal pull-down */
#define PIN_FLYWHEEL_PWM 12  /* PWM6 A, 20 kHz, both gate drivers; external pull-down on the gates */
#define PIN_TACH 13          /* optional flywheel tach, unused in v1 (plain GPIO) */
#define PIN_PUSHER_SERVO 14  /* PWM7 A, 50 Hz */
#define PIN_LED_RED 15
#define PIN_LED_GREEN 16
#define PIN_BUZZER 17
#define PIN_PAN_UART_TX 20   /* UART1 TX -> 1 kohm -> pan TMC2209 PDN_UART */
#define PIN_PAN_UART_RX 21
#define PIN_DIAG 22          /* TMC2209 DIAG of both drivers, active high; internal pull-down */

#define FLYWHEEL_PWM_HZ 20000
#define SERVO_PWM_HZ 50

#endif
