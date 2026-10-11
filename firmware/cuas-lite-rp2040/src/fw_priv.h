/* Core-private helpers shared by fsm.c, planner.c and homing.c. SPDX-License-Identifier: BSD-3-Clause */
#ifndef FW_PRIV_H
#define FW_PRIV_H

#include "fw_core.h"

#define KP 20.0               /* 1/s: a 1 deg error asks 20 deg/s */
#define DT 0.001              /* planner period, s */
#define AIM_STALE_VFF_MS 250
#define AIM_STALE_HOLD_MS 1000
#define HALL_HALF_WIDTH_DEG 1.0
#define BACKOFF_DEG 3.0
#define HOMING_TIMEOUT_MS 15000

void fw_event(fw_t *f, uint8_t code, int32_t arg);
#define MS_SINCE(f, t) ((uint32_t)((f)->now - (t)))

#endif
