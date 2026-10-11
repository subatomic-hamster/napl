/* Hall homing (spec 4.2): approach the sensor from the negative side at homing_speed; on the rising edge set the position to home_off and back
 * off 3 deg. If the sensor is already active, first back out on the negative side until it is released and 2 deg past that edge.
 * Limitation (spec): the axis must start on the negative side of the hall window or on it; from the positive side the approach never meets
 * the edge and the 15 s timeout raises FAULT_HOMING_TIMEOUT. SPDX-License-Identifier: BSD-3-Clause */
#include "fw_priv.h"

enum { H_NONE, H_CLEAR, H_APPROACH, H_BACKOFF };

void fw_homing_start(fw_t *f, int i) {
    fw_axis_t *a = &f->ax[i];
    a->homed = 0;
    a->homing = a->hall ? H_CLEAR : H_APPROACH;
    a->homing_t0 = f->now;
    a->marked = 0;
}

void fw_homing_tick(fw_t *f, int i) {
    fw_axis_t *a = &f->ax[i];
    const double spd = fw_spd(f, i);
    const double vcap = FW_STEP_RATE_MAX / spd;
    double v = f->cfg[CFG_HOMING_SPEED] / 10.0;
    if (v > vcap) v = vcap;
    if (MS_SINCE(f, a->homing_t0) > HOMING_TIMEOUT_MS) {
        a->homing = H_NONE;
        a->v = 0.0;
        f->fault_bits |= FAULT_HOMING_TIMEOUT;  /* fw_tick reports the new bit as EVENT FAULT */
        return;
    }
    if (a->homing == H_CLEAR) {  /* on the sensor: back out on the negative side first */
        a->pos -= v * DT;
        a->v = -v;
        if (!a->hall && !a->marked) {
            a->marked = 1;
            a->mark = a->pos;
        }
        if (a->marked && a->pos <= a->mark - 2.0) a->homing = H_APPROACH;  /* 2 deg clear of the window's negative edge */
    } else if (a->homing == H_APPROACH) {
        a->pos += v * DT;
        a->v = v;
        if (a->hall && !a->prev_hall) {
            a->pos = f->cfg[i ? CFG_TILT_HOME_OFF : CFG_PAN_HOME_OFF] / spd;
            a->emitted = (int32_t)(a->pos * spd + (a->pos >= 0 ? 0.5 : -0.5));  /* the position jumps; no steps are emitted for the jump */
            a->target = a->pos - BACKOFF_DEG;
            a->homing = H_BACKOFF;
        }
    } else if (a->homing == H_BACKOFF) {
        a->pos -= v * DT;
        a->v = -v;
        if (a->pos <= a->target) {
            a->pos = a->target;
            a->v = 0.0;
            a->homing = H_NONE;
            a->homed = 1;
            a->has_aim = 0;
            fw_event(f, EV_HOMED, 1 << i);
        }
    }
}
