/* Motion planner, 1 kHz per axis (spec 2.1 AIM semantics): v_cmd = clamp(vff + brake(kp*(target - pos)), +-vmax), v += clamp(v_cmd - v, +-amax*dt),
 * targets clipped to the soft limits, the speed capped so v^2/(2*amax) fits before a limit and so that the step rate stays <= 25 ksteps/s.
 * The position integrates in degrees; every millisecond the difference to the microsteps already emitted goes to the step generator.
 * SPDX-License-Identifier: BSD-3-Clause */
#include <math.h>
#include "fw_priv.h"

double fw_spd(const fw_t *f, int axis) { return f->cfg[axis ? CFG_TILT_SPR : CFG_PAN_SPR] / 360.0; }

void fw_axis_hold(fw_axis_t *a) {
    a->target = a->pos;
    a->vff = 0.0;
    a->has_aim = 0;
}

static double clampd(double x, double lo, double hi) { return x < lo ? lo : (x > hi ? hi : x); }

static void emit(fw_t *f, int i, double spd) {
    fw_axis_t *a = &f->ax[i];
    long want = lround(a->pos * spd);
    long n = want - a->emitted;
    if (n > 25) n = 25;  /* 25 steps/ms = 25 ksteps/s; the speed cap keeps this from clipping, except right after a jump */
    if (n < -25) n = -25;
    a->emitted += (int32_t)n;
    f->out.step_n[i] = (int8_t)n;
}

void fw_axes_tick(fw_t *f) {
    for (int i = 0; i < 2; i++) {
        fw_axis_t *a = &f->ax[i];
        const double spd = fw_spd(f, i);
        a->prev_hall = a->hall;
        a->hall = f->in.hall[i] ? 1 : 0;
        f->out.step_n[i] = 0;
        if (!f->steppers_enabled) {
            a->v = 0.0;
            continue;
        }
        const double vcap = FW_STEP_RATE_MAX / spd;
        if (a->homing) {
            fw_homing_tick(f, i);
            emit(f, i, spd);
            continue;
        }
        const double lo = f->cfg[i ? CFG_TILT_MIN : CFG_PAN_MIN] / 1000.0, hi = f->cfg[i ? CFG_TILT_MAX : CFG_PAN_MAX] / 1000.0;
        double tgt, vff, vmax, amax;
        const uint32_t age = a->has_aim ? MS_SINCE(f, a->t_aim_ms) : 0;
        if (!a->has_aim || age > AIM_STALE_HOLD_MS) {
            tgt = a->pos;
            vff = 0.0;
            vmax = a->vmax > 1.0 ? a->vmax : 1.0;
            amax = a->amax > 100.0 ? a->amax : 100.0;
        } else {
            tgt = a->target;
            vff = age > AIM_STALE_VFF_MS ? 0.0 : a->vff;
            vmax = a->vmax;
            amax = a->amax;
        }
        if (f->state == ST_SAFE) {  /* watchdog hold: targets frozen where they were */
            tgt = a->pos;
            vff = 0.0;
        }
        if (vmax > vcap) vmax = vcap;
        /* P term, braked so that the stopping distance v^2/(2*amax) fits before the target (a trapezoid: without this a fast move overshoots the
         * target by tens of degrees, because kp*err only starts to fall when err < vmax/kp); feed-forward stays on top */
        const double err = tgt - a->pos, brake = sqrt(2 * amax * fabs(err));
        double vp = KP * err;
        if (fabs(vp) > brake) vp = vp > 0 ? brake : -brake;
        double v_cmd = clampd(vff + vp, -vmax, vmax);
        const double room_hi = a->pos < hi ? hi - a->pos : 0.0, room_lo = a->pos > lo ? a->pos - lo : 0.0;  /* stopping distance fits before the limit */
        if (v_cmd > 0) {
            const double m = sqrt(2 * amax * room_hi);
            if (v_cmd > m) v_cmd = m;
        } else {
            const double m = -sqrt(2 * amax * room_lo);
            if (v_cmd < m) v_cmd = m;
        }
        const double dv = amax * DT;
        a->v += clampd(v_cmd - a->v, -dv, dv);
        a->pos += a->v * DT;
        /* hall_stuck: still on the sensor after moving away from the window centre; the centre is half a window past the edge that set home */
        const double centre = f->cfg[i ? CFG_TILT_HOME_OFF : CFG_PAN_HOME_OFF] / spd + HALL_HALF_WIDTH_DEG;
        const int far = a->homed && a->hall && fabs(a->pos - centre) > 5.0;
        a->stuck_ms = far ? a->stuck_ms + 1 : 0;
        if (a->stuck_ms > 500) f->fault_bits |= FAULT_HALL_STUCK;
        emit(f, i, spd);
    }
}
