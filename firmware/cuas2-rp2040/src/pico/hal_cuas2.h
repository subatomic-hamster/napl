/* SPDX-License-Identifier: BSD-3-Clause */
#ifndef HAL_CUAS2_H
#define HAL_CUAS2_H

#include "fw_core.h"

void hal_init(void);
void hal_read_inputs(fw_in_t *in, int pole_pairs);  /* also runs the DShot service: the eRPM reply of the last frame, the next frame out */
void hal_apply(const fw_out_t *o);
void hal_safe_outputs(void);
const uint8_t *hal_flash_cfg(void);
int hal_flash_write(void *ctx, const uint8_t *img, size_t n);

/* dshot_pio.c */
void dshot_pio_init(void);
void dshot_pio_service(const uint16_t value[2], int pole_pairs, int32_t rpm_out[2]);  /* once per millisecond: collect the replies, start the next frames */

#endif
