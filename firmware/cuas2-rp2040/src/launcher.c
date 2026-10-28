/* Launcher: closed-loop speed of the two flywheels (spec 4.4, I12, I13), integer arithmetic only so the python twin (python/napl/cuas2/fsm.py) is byte-exact.
 * One call per millisecond. Setpoints: top = mean - mean*diff/2000, bottom = mean + mean*diff/2000 (diff in permille of the mean, positive = bottom faster =
 * backspin). The rpm cap comes from cfg 23, 24 alone, so a wheel held at or below it cannot launch above v_cap whatever the calibration (I13).
 * SPDX-License-Identifier: BSD-3-Clause */
#include <math.h>
#include <string.h>
#include "fw_core.h"

int32_t fdr_tdiv(int32_t a, int32_t b) { return a / b; }

int32_t fdr_rpm_cap(const int32_t *cfg) {
    int32_t v = cfg[CFG_V_CAP_DMPS], d = cfg[CFG_WHEEL_D_UM];
    if (v < 1 || v > 380) v = 380;              /* the cap can only be lowered: corrupt or out-of-range values fall back to 38 m/s */
    if (d < 10000 || d > 100000) d = 40000;
    return (int32_t)floor((double)v * 6000000.0 / (3.14159265358979323846 * (double)d));
}

static int32_t clampi(int32_t x, int32_t lo, int32_t hi) { return x < lo ? lo : (x > hi ? hi : x); }
static int32_t absi(int32_t x) { return x < 0 ? -x : x; }

void fdr_ev(fdr_out_t *o, uint8_t code, int32_t arg) {
    if (o->nev < FDR_MAX_EV) {
        o->ev[o->nev].code = code;
        o->ev[o->nev].arg = arg;
        o->nev++;
    }
}

/* latch one fault bit and report it (EVENT FAULT arg = bit number) once */
void fdr_fault(fdr_t *d, fdr_out_t *o, uint16_t bit) {
    int b = 0;
    if (d->fault_bits & bit) return;
    d->fault_bits |= bit;
    while (!(bit >> b & 1)) b++;
    fdr_ev(o, EV_FAULT, b);
}

void lau_step(fdr_t *d, const fdr_in_t *in, fdr_out_t *o, int32_t *mean_target) {
    const int32_t *c = in->cfg;
    const int rail = in->key_on && !in->estop_open;
    const int32_t cap = fdr_rpm_cap(c), tl = c[CFG_TELEMETRY_LOSS_MS], window = c[CFG_RPM_WINDOW_PM], tmax = c[CFG_THROTTLE_MAX_PM];
    d->rpm_cap = cap;
    int32_t rpm_max = c[CFG_RPM_MAX] < cap ? c[CFG_RPM_MAX] : cap;
    if (rpm_max < 0) rpm_max = 0;
    const int32_t req = in->speed_req_pm > 1000 ? 1000 : in->speed_req_pm;
    const int32_t diff = clampi(in->diff_pm, -FDR_MAX_DIFF_PM, FDR_MAX_DIFF_PM);
    const int32_t target = (rail && !(d->fault_bits & FDR_FAULT_MASK)) ? req * rpm_max / 1000 : 0;
    const int32_t dd = target * diff / 2000;
    const int32_t set[2] = {target - dd, target + dd};
    *mean_target = target;

    /* ramp */
    int32_t step = c[CFG_RAMP_RPM_S] / 1000, rs[2];
    if (step < 1) step = 1;
    if (target == 0) {
        d->ramp_mean = 0;
    } else if (d->ramp_mean < target) {
        d->ramp_mean = d->ramp_mean + step > target ? target : d->ramp_mean + step;
    } else {
        d->ramp_mean = d->ramp_mean - step < target ? target : d->ramp_mean - step;
    }
    const int32_t rd = d->ramp_mean * diff / 2000;
    rs[0] = d->ramp_mean - rd;
    rs[1] = d->ramp_mean + rd;

    /* telemetry */
    for (int i = 0; i < 2; i++) {
        const int32_t r = i ? in->rpm_bot : in->rpm_top;
        if (r >= 0) {
            /* window filter: first-order IIR, f += (r - f) / FDR_RPM_FILT_DIV, state x16. Re-seeded from the raw sample when the wheel is off or the previous sample is stale,
             * so a filter state never outlives its telemetry. The PI loop and the overspeed fault keep the raw value. */
            if (target == 0 || d->age[i] > FDR_WHEEL_AGE_MAX_MS) d->frpm16[i] = r * 16;
            else d->frpm16[i] += (r * 16 - d->frpm16[i]) / FDR_RPM_FILT_DIV;
            d->last_rpm[i] = r;
            d->age[i] = 0;
        } else if (d->age[i] < 65535u) {
            d->age[i]++;
        }
    }
    if (target > 0) d->on_ms++;
    else d->on_ms = 0;

    /* PI per wheel: u = ff + kp e + ki integral(e), permille */
    const int32_t kp = c[CFG_KP], ki = c[CFG_KI], capd = cap < 1 ? 1 : cap;
    const int32_t ilim = tmax * 1000 / (ki < 1 ? 1 : ki);
    for (int i = 0; i < 2; i++) {
        if (target == 0) {
            d->integ[i] = 0;
            d->thr[i] = 0;
            continue;
        }
        const int32_t err = rs[i] - d->last_rpm[i];
        d->integ[i] = clampi(d->integ[i] + err, -ilim, ilim);
        const int32_t u = rs[i] * tmax / capd + kp * err / 1000 + ki * d->integ[i] / 1000;
        d->thr[i] = clampi(u, 0, tmax);
    }

    /* window (I12): both wheels within rpm_window_pm of the FINAL setpoint, the ramp finished, fresh telemetry, and the differential within the same band */
    const int32_t fr[2] = {(d->frpm16[0] + 8) / 16, (d->frpm16[1] + 8) / 16};  /* filtered rpm (frpm16 >= 0) */
    int all_ok = target > 0 && d->ramp_mean == target;
    for (int i = 0; i < 2; i++)
        all_ok = all_ok && d->age[i] <= FDR_WHEEL_AGE_MAX_MS && set[i] > 0 && absi(fr[i] - set[i]) * 1000 <= window * set[i];
    all_ok = all_ok && absi((fr[1] - fr[0]) - (set[1] - set[0])) * 1000 <= window * target;
    d->inwin_now = (uint8_t)all_ok;
    if (all_ok) {
        if (d->win_ms < 65535u) d->win_ms++;
    } else {
        d->win_ms = 0;
    }
    const int ready = all_ok && d->win_ms >= (uint32_t)c[CFG_SETTLE_MS];
    const int32_t mean_rpm = (d->last_rpm[0] + d->last_rpm[1]) / 2;
    if (ready && !d->ready_prev) fdr_ev(o, EV_WHEELS_READY, mean_rpm);
    if (!ready && d->ready_prev && target > 0) fdr_ev(o, EV_WHEELS_LOST, mean_rpm);
    d->ready_prev = (uint8_t)ready;

    /* faults */
    if (target > 0 && d->on_ms > (uint32_t)tl && (d->age[0] > (uint32_t)tl || d->age[1] > (uint32_t)tl)) fdr_fault(d, o, FAULT_TELEMETRY_LOST);
    int over = 0;
    for (int i = 0; i < 2; i++) over = over || (d->age[i] <= FDR_WHEEL_AGE_MAX_MS && d->last_rpm[i] * 100 > cap * 105);
    if (over) d->over_ms++;
    else d->over_ms = 0;
    if (d->over_ms >= FDR_OVERSPEED_MS) fdr_fault(d, o, FAULT_OVERSPEED);
    if (target > 0) {
        if (target != d->prev_target || ready) d->nr_ms = 0;
        else d->nr_ms++;
        if (d->nr_ms >= FDR_NOT_READY_MS) fdr_fault(d, o, FAULT_WHEEL_NOT_READY);
    } else {
        d->nr_ms = 0;
    }
    d->prev_target = target;

    for (int i = 0; i < 2; i++) o->dshot[i] = (uint16_t)(target > 0 ? 48 + d->thr[i] * 1952 / 1000 : 0);
    o->throttle_mean_pm = (uint16_t)((d->thr[0] + d->thr[1]) / 2);
    o->rpm_set = (uint16_t)target;
    o->rpm_top = d->age[0] > (uint32_t)tl ? 0xFFFF : (uint16_t)clampi(d->last_rpm[0], 0, 0xFFFE);
    o->rpm_bot = d->age[1] > (uint32_t)tl ? 0xFFFF : (uint16_t)clampi(d->last_rpm[1], 0, 0xFFFE);
    o->wheels_ready = (uint8_t)ready;
}
