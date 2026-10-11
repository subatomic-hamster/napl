/* SPDX-License-Identifier: BSD-3-Clause */
#ifndef TMC2209_H
#define TMC2209_H

/* Current: I_RMS = (CS+1)/32 x 0.325 V / (R_SENSE + 0.02 ohm) / sqrt(2) with R_SENSE = 0.11 ohm (BTT module): CS = 21 -> 1.215 A RMS (run), CS = 10 -> 0.61 A (hold). */
#define TMC_IRUN_CS 21
#define TMC_IHOLD_CS 10

int tmc2209_init(int axis, int irun, int ihold);  /* axis 0 = pan (UART1), 1 = tilt (UART0); 0 = ok */

#endif
