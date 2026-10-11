/* Hall homing (spec 4.2), from any start position. The final approach is always from the negative side at homing_speed; on the rising edge the position is
 * set to home_off and the axis backs off 3 deg.
 *   - Sensor already active: CLEAR (back out negative until it releases and 2 deg beyond), then APPROACH.
 *   - Otherwise SEEK_POS at the search speed for the negative half-range (|min| + 10 deg: if the start is on the negative side the sensor must turn up within it).
 *     No sensor: the start is on the positive side, so SEEK_NEG sweeps negative for up to the full range + 20 deg until the sensor turns up.
 *   - Any sensor hit while seeking goes to CLEAR (through the window, out its negative edge) and then to the slow APPROACH, so the edge that sets home is always
 *     the rising edge met moving positive.
 * Worst case at the defaults (search 100 deg/s): pan about 1.8 s + 3.5 s + ~0.7 s, inside the 15 s timeout. SPDX-License-Identifier: BSD-3-Clause */
#include "fw_priv.h"

enum { H_NONE, H_CLEAR, H_APPROACH, H_BACKOFF, H_SEEK_POS, H_SEEK_NEG };

void fw_homing_start(fw_t *f, int i) {
    fw_axis_t *a = &f->ax[i];
    a->homed = 0;
    a->homing = a->hall ? H_CLEAR : H_SEEK_POS;
    a->homing_t0 = f->now;
    a->marked = 0;
    a->mark = a->pos;
}

void fw_homing_tick(fw_t *f, int i) {
    fw_axis_t *a = &f->ax[i];
    const double spd = fw_spd(f, i);
    const double vcap = FW_STEP_RATE_MAX / spd;
    double v = f->cfg[CFG_HOMING_SPEED] / 10.0, vs = HOMING_SEARCH_DEG_S;
    if (v > vcap) v = vcap;
    if (vs > vcap) vs = vcap;
    const double lo = f->cfg[i ? CFG_TILT_MIN : CFG_PAN_MIN] / 1000.0, hi = f->cfg[i ? CFG_TILT_MAX : CFG_PAN_MAX] / 1000.0;
    int timeout = MS_SINCE(f, a->homing_t0) > HOMING_TIMEOUT_MS;
    if (a->homing == H_SEEK_POS) {
        a->pos += vs * DT;
        a->v = vs;
        if (a->hall) {
            a->homing = H_CLEAR;
            a->marked = 0;
        } else if (a->pos - a->mark > -lo + 10.0) {
            a->homing = H_SEEK_NEG;
            a->mark = a->pos;
        }
    } else if (a->homing == H_SEEK_NEG) {
        a->pos -= vs * DT;
        a->v = -vs;
        if (a->hall) {
            a->homing = H_CLEAR;
            a->marked = 0;
        } else if (a->mark - a->pos > (hi - lo) + 20.0) {
            timeout = 1;  /* swept the whole range and found nothing: sensor or magnet missing */
        }
    } else if (a->homing == H_CLEAR) {  /* on the sensor: back out on the negative side first */
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
    if (timeout && a->homing != H_NONE) {
        a->homing = H_NONE;
        a->v = 0.0;
        f->fault_bits |= FAULT_HOMING_TIMEOUT;  /* fw_tick reports the new bit as EVENT FAULT */
    }
}
