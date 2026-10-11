/* SPDX-License-Identifier: BSD-3-Clause */
#ifndef HAL_PICO_H
#define HAL_PICO_H

#include "fw_core.h"

void hal_init(void);
void hal_read_inputs(fw_in_t *in);
void hal_apply(const fw_out_t *o);
void hal_safe_outputs(void);
const uint8_t *hal_flash_cfg(void);
int hal_flash_write(void *ctx, const uint8_t *img, size_t n);

#endif
