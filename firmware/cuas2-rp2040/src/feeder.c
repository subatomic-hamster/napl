/* Feeder: the single-pocket gate cycle, the pusher stroke, the beam cross-checks and the FIRE gates (spec 1.2 I9-I12, I15, 1.3, 5.3-5.5), one pure step per
 * millisecond. fdr_step() is mirrored statement for statement in python/napl/cuas2/fsm.py; tests/cuas2/fsm_vectors.json drives both.
 * Order inside one step: launcher (wheels) -> dip tracking -> muzzle beam (I10) -> exit deadline -> feed tracking (pusher/breech checks) -> state machine -> FIRE.
 * SPDX-License-Identifier: BSD-3-Clause */
#include <string.h>
#include "fw_core.h"

static void enter(fdr_t *d, uint8_t gs, uint32_t now) {
    d->gstate = gs;
    d->t_state = now;
    d->gate_load = (gs == GS_UNKNOWN || gs == GS_HOMING || gs == GS_LOADING || gs == GS_TO_LOAD);  /* TO_BORE, STAGED, PUSH, EXIT_WAIT, RETRACT, EMPTY: gate at the bore */
}

void fdr_init(fdr_t *d) {
    memset(d, 0, sizeof *d);
    d->last_v_dmps = 0xFFFF;  /* no chrono reading yet */
    enter(d, GS_UNKNOWN, 0);
}

static uint32_t min_interval(const int32_t *c) { return (uint32_t)(c[CFG_MIN_SHOT_INTERVAL_MS] < FDR_MIN_INTERVAL_FLOOR ? FDR_MIN_INTERVAL_FLOOR : c[CFG_MIN_SHOT_INTERVAL_MS]); }

static int busy_state(uint8_t g) { return g == GS_PUSH || g == GS_EXIT_WAIT || g == GS_RETRACT || g == GS_TO_LOAD || g == GS_LOADING || g == GS_TO_BORE; }

static int32_t dip_pm(const fdr_t *d) {
    if (d->rpm_feed_mean <= 0) return 0;
    int32_t v = (d->rpm_feed_mean - d->rpm_min_mean) * 1000 / d->rpm_feed_mean;
    return v < 0 ? 0 : (v > 65535 ? 65535 : v);
}

/* FIRE gates in the order the ACK reports them (I9b, I11, I12): the first failing one wins */
static uint8_t fire_result(const fdr_t *d, const fdr_in_t *in, int rail) {
    const uint32_t now = in->t_ms;
    if (!rail) return RES_INTERLOCK;
    if (!in->hb_ok || !in->deadman || in->inhibit) return RES_INHIBIT;
    if (!in->in_sector) return RES_RANGE;
    if (d->fault_bits & FDR_FAULT_MASK) return RES_JAMMED;
    if (d->gstate == GS_EMPTY) return RES_NO_BALL;
    if (d->gstate != GS_STAGED) return RES_BUSY;
    if (in->beam_muzzle) return RES_NOT_CLEAR;
    if (d->has_feed && now - d->t_last_feed < min_interval(in->cfg)) return RES_TOO_SOON;
    if (!d->ready_prev) return RES_NOT_SPUN;
    if (!in->beam_breech) return RES_NO_BALL;
    if (!in->push_home) return RES_JAMMED;
    return RES_OK;
}

void fdr_step(fdr_t *d, const fdr_in_t *in, fdr_out_t *o) {
    const int32_t *c = in->cfg;
    const uint32_t now = in->t_ms;
    const int rail = in->key_on && !in->estop_open;
    int32_t target;
    memset(o, 0, sizeof *o);
    if (in->cmd == FDR_CMD_CLEAR) {  /* operator CLEAR_FAULT (the safety FSM allows it in SAFE with the E-stop released): re-home the gate, forget the latches */
        d->fault_bits = 0;
        d->over_ms = d->nr_ms = 0;
        d->retries = d->empty_reported = d->jam_reported = 0;
        d->exp_exit = d->aborted = d->trk = d->feed_req = d->committed = 0;
        enter(d, GS_UNKNOWN, now);
    }
    lau_step(d, in, o, &target);
    const uint32_t tf = now - d->t_feed;  /* only meaningful once has_feed */

    /* rpm dip of the mean wheel while the ball is in the nip (R9) */
    if (d->has_feed && tf < FDR_DIP_WINDOW_MS) {
        const int32_t m = (d->last_rpm[0] + d->last_rpm[1]) / 2;
        if (m < d->rpm_min_mean) d->rpm_min_mean = m;
        d->last_dip_pm = (uint16_t)dip_pm(d);
    }

    /* muzzle beam (I10): the first rising edge after the commit is the exit; any other one, or two closer than 300 ms, is unexpected_exit */
    const uint8_t mz = in->beam_muzzle ? 1 : 0;
    if (!d->inited) {  /* the first sample is the initial condition, not an edge (a beam that is blocked at power-up reads NOT_CLEAR) */
        d->inited = 1;
        d->prev_muz = mz;
    }
    if (mz && !d->prev_muz) {
        int ok = d->exp_exit && !d->exit_seen;
        if (d->has_muz && now - d->t_last_muz < FDR_MUZ_MIN_GAP_MS) ok = 0;
        d->muz_all++;
        if (ok) {
            int32_t v = 0xFFFF;
            if (in->muzzle_dt_us > 0) {
                v = (int32_t)(222u * (uint32_t)c[CFG_K_BEAM] / in->muzzle_dt_us);  /* v = k_beam * 22.2 mm / dt, in 0.1 m/s */
                if (v > 0xFFFE) v = 0xFFFE;
            }
            d->exit_seen = 1;
            d->exp_exit = 0;
            d->last_v_dmps = (uint16_t)v;
            d->last_dip_pm = (uint16_t)dip_pm(d);
            d->feed_ms = (uint8_t)((tf + 2) / 4 > 255 ? 255 : (tf + 2) / 4);
            fdr_ev(o, EV_MUZZLE, (int32_t)((uint32_t)v | (uint32_t)d->last_dip_pm << 16));
        } else {
            fdr_fault(d, o, FAULT_UNEXPECTED_EXIT);
        }
        d->has_muz = 1;
        d->t_last_muz = now;
    }
    d->prev_muz = mz;
    if (d->muz_all > d->feeds) fdr_fault(d, o, FAULT_SHOT_OVERCOUNT);                       /* independent count: more exits than feeds */
    if (d->exp_exit && !d->exit_seen && tf >= (uint32_t)c[CFG_EXIT_TIMEOUT_MS]) fdr_fault(d, o, FAULT_NO_EXIT);

    /* feed tracking: pusher leaves and returns home, the breech clears in time (I11) */
    if (d->trk && !(d->fault_bits & FDR_FAULT_MASK)) {  /* (frozen once a fault is latched: no cascade of secondary faults) */
        if (!in->push_home) d->left_seen = 1;
        if (!d->aborted) {
            if (!d->breech_clr_seen && !in->beam_breech) {
                d->breech_clr_seen = 1;
                if (tf * 10 > FDR_BREECH_EXP_MS * 13) fdr_fault(d, o, FAULT_BREECH_DOUBLE);  /* held longer than 1.3x the expected one-ball time */
            }
            if (!d->breech_clr_seen && tf >= FDR_BREECH_CLEAR_MAX_MS) fdr_fault(d, o, FAULT_NO_EXIT);
            if (!d->left_seen && tf >= FDR_PUSH_LEAVE_MS) fdr_fault(d, o, FAULT_PUSHER_STUCK);
        }
        if (tf >= FDR_RETRACT_TIMEOUT_MS) {
            if (!in->push_home) fdr_fault(d, o, FAULT_PUSHER_STUCK);
            d->trk = 0;
        }
    }
    if (d->feed_req && tf >= FDR_FEED_PULSE_MS) d->feed_req = 0;

    /* gate / pusher state machine */
    const uint32_t te = now - d->t_state;
    const uint32_t dwell = (uint32_t)c[CFG_PUSHER_DWELL_MS];
    const int jam = (d->fault_bits & FDR_FAULT_MASK) != 0;
    if (jam) {
        d->feed_req = 0;
    } else if (!rail) {  /* the weapon rail is open: nothing moves; re-home when it returns */
        if (d->gstate == GS_PUSH && !d->committed) fdr_ev(o, EV_FEED_ABORT, (int32_t)((uint32_t)d->fire_id << 8 | FA_RAIL));
        if (d->gstate != GS_UNKNOWN) enter(d, GS_UNKNOWN, now);
        d->feed_req = 0;
        d->trk = 0;
    } else {
        switch (d->gstate) {
        case GS_UNKNOWN:
            enter(d, GS_HOMING, now);
            break;
        case GS_HOMING:
            if (in->gate_home && in->push_home) enter(d, GS_LOADING, now);
            else if (te >= FDR_HOMING_TIMEOUT_MS) fdr_fault(d, o, in->gate_home ? FAULT_PUSHER_STUCK : FAULT_GATE_STUCK);
            break;
        case GS_LOADING:
            if (te >= (uint32_t)c[CFG_FILL_MS]) enter(d, GS_TO_BORE, now);
            break;
        case GS_TO_BORE:
            if (te >= FDR_GATE_TRAVEL_MS) {
                if (in->beam_breech) {
                    d->retries = 0;
                    d->empty_reported = 0;
                    enter(d, GS_STAGED, now);
                } else {
                    d->retries++;
                    if (d->retries >= FDR_EMPTY_RETRIES) {
                        if (!d->empty_reported) {
                            d->empty_reported = 1;
                            fdr_ev(o, EV_EMPTY, 0);
                        }
                        enter(d, GS_EMPTY, now);
                    } else {
                        enter(d, GS_TO_LOAD, now);
                    }
                }
            }
            break;
        case GS_TO_LOAD:
            if (in->gate_home) enter(d, GS_LOADING, now);
            else if (te >= FDR_GATE_HOME_TIMEOUT_MS) fdr_fault(d, o, FAULT_GATE_STUCK);
            break;
        case GS_EMPTY:
            if (te >= FDR_EMPTY_RETRY_MS) {
                d->retries = FDR_EMPTY_RETRIES - 1;
                enter(d, GS_TO_LOAD, now);
            }
            break;
        case GS_PUSH: {
            int reason = 0;
            /* The commit point is the EARLIER of FDR_COMMIT_MS and the breech beam clearing after FDR_BREECH_EARLY_MS: once the ball has left the breech it is on its way to
             * the nip and cannot be undone, so no abort may follow (a clear before FDR_BREECH_EARLY_MS is a vanished ball: abort, FA_BEAM). */
            const int left = d->breech_clr_seen && tf >= FDR_BREECH_EARLY_MS;
            if (tf < FDR_COMMIT_MS && !left) {  /* until the commit point the stroke can still be undone: the ball is in the bore */
                if (!in->in_sector) reason = FA_SECTOR;
                else if (!in->hb_ok || !in->deadman) reason = FA_DEADMAN;
                else if (in->inhibit) reason = FA_INHIBIT;
                else if (!d->inwin_now) reason = FA_WHEELS;  /* wheels still in window until the commit (spec 5.4) */
                else if (tf < FDR_BREECH_EARLY_MS && !in->beam_breech) reason = FA_BEAM;
            }
            if (reason) {
                fdr_ev(o, EV_FEED_ABORT, (int32_t)((uint32_t)d->fire_id << 8 | (uint32_t)reason));
                if (reason == FA_SECTOR) fdr_ev(o, EV_SECTOR_ABORT, (int32_t)((uint32_t)d->fire_id << 8));
                d->aborted = 1;
                d->feed_req = 0;
                enter(d, GS_RETRACT, now);
            } else if (tf >= FDR_COMMIT_MS || left) {
                d->committed = 1;
                d->feeds++;
                d->exp_exit = 1;
                d->exit_seen = 0;
                fdr_ev(o, EV_SHOT, (int32_t)((uint32_t)d->fire_id << 8));
                enter(d, GS_EXIT_WAIT, now);
            }
            break;
        }
        case GS_EXIT_WAIT:
            if (d->exit_seen) enter(d, GS_RETRACT, now);
            break;
        case GS_RETRACT:
            if (d->aborted) {
                if (in->push_home && te >= 10) enter(d, GS_STAGED, now);  /* (aborted stays set until the next feed: no breech checks for an undone stroke) */
            } else if (tf >= dwell + FDR_RETRACT_CLEAR_MS && in->push_home) {  /* the gate may only turn with the rod home; else pusher_stuck at FDR_RETRACT_TIMEOUT_MS */
                enter(d, GS_TO_LOAD, now);
            }
            break;
        default:
            break;
        }
    }

    /* FIRE */
    if (in->cmd == FDR_CMD_FIRE) {
        const uint8_t res = fire_result(d, in, rail);
        o->ack_valid = 1;
        o->ack = res;
        if (res == RES_OK) {
            if (d->has_feed && now - d->t_last_feed < FDR_MIN_INTERVAL_FLOOR) fdr_fault(d, o, FAULT_SHOT_OVERCOUNT);  /* assertion: unreachable while the gate above holds */
            d->has_feed = 1;
            d->t_feed = now;
            d->t_last_feed = now;
            d->fire_id = in->fire_id;
            d->committed = d->aborted = 0;
            d->exit_seen = d->left_seen = d->breech_clr_seen = 0;
            d->trk = 1;
            d->feed_req = 1;
            d->rpm_feed_mean = (d->last_rpm[0] + d->last_rpm[1]) / 2;
            d->rpm_min_mean = d->rpm_feed_mean;
            d->last_dip_pm = 0;
            enter(d, GS_PUSH, now);
        }
    }

    if ((d->fault_bits & FDR_JAM_MASK) && !d->jam_reported) {
        d->jam_reported = 1;
        fdr_ev(o, EV_JAM, d->gstate);
    }

    /* outputs */
    const int jam_now = (d->fault_bits & FDR_FAULT_MASK) != 0;
    const uint32_t tf2 = now - d->t_feed;
    const int ext = d->trk && !d->aborted && !jam_now && rail && tf2 < dwell && (d->gstate == GS_PUSH || d->gstate == GS_EXIT_WAIT || d->gstate == GS_RETRACT);
    const int ready = rail && !jam_now && d->gstate == GS_STAGED && !in->beam_muzzle && (!d->has_feed || now - d->t_last_feed >= min_interval(c)) && d->ready_prev &&
                      in->beam_breech && in->push_home;
    if (jam_now) {
        o->dshot[0] = o->dshot[1] = 0;
        o->throttle_mean_pm = 0;
        d->ramp_mean = 0;
        d->integ[0] = d->integ[1] = 0;
    }
    o->gate_us = (uint16_t)(d->gate_load ? c[CFG_GATE_LOAD_US] : c[CFG_GATE_BORE_US]);
    o->push_us = (uint16_t)(ext ? c[CFG_PUSHER_EXT_US] : c[CFG_PUSHER_RET_US]);
    o->feed_req = (uint8_t)(d->feed_req && !jam_now);
    o->kick = (uint8_t)!jam_now;
    o->feed_busy = (uint8_t)busy_state(d->gstate);
    o->gate_state = (d->fault_bits & FDR_JAM_MASK) ? GS_JAM : d->gstate;
    o->ready_flags = (uint8_t)((ready ? RDY_READY : 0) | (d->gstate == GS_EMPTY ? RDY_EMPTY | RDY_HOPPER_LOW : 0) | ((d->fault_bits & FDR_JAM_MASK) ? RDY_JAM : 0));
    o->beams = (uint8_t)((in->beam_breech ? BM_BREECH : 0) | (in->beam_muzzle ? BM_MUZZLE : 0) | (in->gate_home ? BM_GATE_HOME : 0) | (in->push_home ? BM_PUSH_HOME : 0) |
                         (o->wheels_ready ? BM_WHEELS_READY : 0) | (busy_state(d->gstate) ? BM_FEED_BUSY : 0));
    o->feed_ms = d->feed_ms;
    o->last_v_dmps = d->last_v_dmps;
    o->last_dip_pm = d->last_dip_pm;
    o->shots_confirmed = d->muz_all;
    o->fault_bits = d->fault_bits;
}
