"""Vectors (a)-(o) and extras: the C core and the python twin give identical outputs (events and ACK results byte-exact, every output change on the same
millisecond) and both match the stored trace; plus the negative controls: single gates of the C code are removed in a temporary copy of the firmware tree, a
shared library is rebuilt with gcc, and each mutant must make at least one vector (or behaviour test) fail while the unmutated rebuild passes all of them.
Needs the host library for the C side (tools/cuas2_fw_host_test.sh builds it, or NAPL_CUAS2_FW=PATH) and gcc for the mutants."""

import json
import shutil
import subprocess
from pathlib import Path

import pytest
from napl.cuas2 import protocol as p

from cuas2 import fwharness as H
from cuas2 import gen_vectors as G
from cuas2 import (
    test_fsm as TF,
)  # the gate table and scenarios the mutants are judged by

HERE = Path(__file__).parent
ROOT = HERE.parents[1]
FW = ROOT / "firmware" / "cuas2-rp2040"
LITE_SRC = ROOT / "firmware" / "cuas-lite-rp2040" / "src"
STORED = json.loads((HERE / "fsm_vectors.json").read_text())["vectors"]
BY_NAME = {v["name"]: v for v in STORED}


@pytest.fixture(scope="module", autouse=True)
def need_lib():
    H.require_lib()


def test_stored_vectors_cover_every_generated_vector():
    assert list(BY_NAME) == list(G.V)
    assert len(STORED) >= 27


@pytest.mark.parametrize("name", list(G.V))
def test_c_equals_python_equals_stored(name):
    _, o_py, _ = G.run_vec(name)
    _, o_c, _ = G.run_vec(name, H.CFeeder())
    t_py, t_c = H.trace_of(o_py), H.trace_of(o_c)
    assert t_c == t_py, f"{name}: C and python diverge"
    assert t_py == BY_NAME[name]["trace"], (
        f"{name}: the stored trace is stale (regenerate with gen_vectors.py fsm)"
    )
    assert not H.check_expect(name, BY_NAME[name]["expect"], H.expect_view(o_c))


@pytest.mark.parametrize("name", list(G.V))
def test_open_loop_replay_of_the_stored_inputs_gives_the_stored_trace(name):
    v = BY_NAME[name]
    assert H.replay(H.CFeeder(), v) == v["trace"] == H.replay(H.FeederFsm(), v)


# ---- negative controls ----
# (id, file under firmware/cuas2-rp2040, old text (must occur exactly once), new text, detector)
M = [
    ("drop_muzzle_clear_check", "src/feeder.c", "    if (in->beam_muzzle) return RES_NOT_CLEAR;\n", "", "vectors"),
    ("drop_interval_clamp", "src/feeder.c", "c[CFG_MIN_SHOT_INTERVAL_MS] < FDR_MIN_INTERVAL_FLOOR ? FDR_MIN_INTERVAL_FLOOR : c[CFG_MIN_SHOT_INTERVAL_MS]", "c[CFG_MIN_SHOT_INTERVAL_MS]", "vectors"),
    ("drop_unexpected_exit_fault", "src/feeder.c", "            fdr_fault(d, o, FAULT_UNEXPECTED_EXIT);\n", "            ;\n", "vectors"),
    ("drop_shot_overcount_fault", "src/feeder.c", "if (d->muz_all > d->feeds) fdr_fault(d, o, FAULT_SHOT_OVERCOUNT);", ";", "vectors"),
    ("drop_not_spun_gate", "src/feeder.c", "    if (!d->ready_prev) return RES_NOT_SPUN;\n", "", "vectors"),
    ("drop_staged_ball_breech_check", "src/feeder.c", "    if (!in->beam_breech) return RES_NO_BALL;\n", "", "vectors"),
    ("drop_no_exit_deadline", "src/feeder.c", "if (d->exp_exit && !d->exit_seen && tf >= (uint32_t)c[CFG_EXIT_TIMEOUT_MS]) fdr_fault(d, o, FAULT_NO_EXIT);", ";", "vectors"),
    ("drop_overspeed_fault", "src/launcher.c", "if (d->over_ms >= FDR_OVERSPEED_MS) fdr_fault(d, o, FAULT_OVERSPEED);", ";", "vectors"),
    ("drop_telemetry_loss_fault", "src/launcher.c", "fdr_fault(d, o, FAULT_TELEMETRY_LOST);", ";", "vectors"),
    ("drop_sector_abort", "src/feeder.c", "if (!in->in_sector) reason = FA_SECTOR;", "if (0) reason = FA_SECTOR;", "vectors"),
    ("drop_deadman_abort", "src/feeder.c", "else if (!in->hb_ok || !in->deadman) reason = FA_DEADMAN;", "else if (0) reason = FA_DEADMAN;", "vectors"),
    ("drop_inhibit_abort", "src/feeder.c", "else if (in->inhibit) reason = FA_INHIBIT;", "else if (0) reason = FA_INHIBIT;", "vectors"),
    ("drop_wheels_abort", "src/feeder.c", "else if (!d->inwin_now) reason = FA_WHEELS;", "else if (0) reason = FA_WHEELS;", "vectors"),
    ("wheel_recheck_only_first_30ms", "src/feeder.c", "else if (!d->inwin_now) reason = FA_WHEELS;", "else if (tf < 30 && !d->inwin_now) reason = FA_WHEELS;", "vectors"),
    ("drop_breech_early_abort", "src/feeder.c", "else if (tf < FDR_BREECH_EARLY_MS && !in->beam_breech) reason = FA_BEAM;", "else if (0) reason = FA_BEAM;", "vectors"),
    ("drop_pusher_departure_check", "src/feeder.c", "if (!d->left_seen && tf >= FDR_PUSH_LEAVE_MS) fdr_fault(d, o, FAULT_PUSHER_STUCK);", ";", "vectors"),
    ("drop_breech_double_fault", "src/feeder.c", "fdr_fault(d, o, FAULT_BREECH_DOUBLE);", ";", "vectors"),
    ("drop_rail_check_on_fire", "src/feeder.c", "    if (!rail) return RES_INTERLOCK;\n", "", "vectors"),
    ("drop_range_gate", "src/feeder.c", "    if (!in->in_sector) return RES_RANGE;\n", "", "vectors"),
    ("drop_inhibit_gate", "src/feeder.c", "    if (!in->hb_ok || !in->deadman || in->inhibit) return RES_INHIBIT;\n", "", "vectors"),
    ("drop_jammed_gate", "src/feeder.c", "    if (d->fault_bits & FDR_FAULT_MASK) return RES_JAMMED;\n", "", "vectors"),
    ("kick_keeps_running_after_fault", "src/feeder.c", "o->kick = (uint8_t)!jam_now;", "o->kick = 1;", "vectors"),
    ("accept_n_other_than_1", "src/fsm.c", "    if (n != 1) return RES_BAD_ARG;  /* one ball per FIRE (I9b) */\n", "", "core_n"),
    ("drop_fire_id_dedupe", "src/fsm.c", "if (f->fire_ids[i].used && f->fire_ids[i].id == id) {", "if (0) {", "core_dup"),
    ("v_cap_above_38ms_accepted", "src/fsm.c", "case CFG_V_CAP_DMPS: return v >= 1 && v <= 380;", "case CFG_V_CAP_DMPS: return v >= 1;", "core_vcap"),
    # feeder.c FIRE gates not covered above (spec 1.2 I9b/I11)
    ("drop_too_soon_gate", "src/feeder.c", "    if (d->has_feed && now - d->t_last_feed < min_interval(in->cfg)) return RES_TOO_SOON;\n", "", "vectors"),
    ("drop_empty_no_ball_gate", "src/feeder.c", "    if (d->gstate == GS_EMPTY) return RES_NO_BALL;\n", "", "vectors"),
    ("drop_busy_gate", "src/feeder.c", "    if (d->gstate != GS_STAGED) return RES_BUSY;\n", "", "vectors"),
    ("drop_push_home_jammed_gate", "src/feeder.c", "    if (!in->push_home) return RES_JAMMED;\n", "", "vectors"),
    # fsm.c fire_precheck gates (fsm.c:208-219), each on its own
    ("drop_precheck_state_engaging", "src/fsm.c", "    if (f->state != ST_ENGAGING) return RES_BAD_STATE;\n", "", "core_state"),
    ("drop_precheck_busy_pending", "src/fsm.c", "    if (f->state == ST_FIRING || f->fire_cmd_pending) return RES_BUSY;\n", "", "core_busy"),
    ("precheck_bad_state_before_busy", [("src/fsm.c", "    if (f->state == ST_FIRING || f->fire_cmd_pending) return RES_BUSY;\n    if (f->state != ST_ENGAGING) return RES_BAD_STATE;\n", "    if (f->state != ST_ENGAGING) return RES_BAD_STATE;\n    if (f->state == ST_FIRING || f->fire_cmd_pending) return RES_BUSY;\n")], "prec_busy"),
    ("drop_precheck_homed", "src/fsm.c", "    if (!f->ax[0].homed || !f->ax[1].homed) return RES_NOT_HOMED;\n", "", "gate:not_homed"),
    ("drop_precheck_sector_set", "src/fsm.c", "    if (f->sector_set != SECTOR_ALL) return RES_NO_SECTOR;\n", "", "gate:no_sector"),
    ("drop_precheck_shot_cap", "src/fsm.c", "    if (f->shots_arm + 1 > f->shot_cap) return RES_CAP;\n", "", "gate:shot_cap"),
    # gates that exist in BOTH layers (fire_precheck and the feeder): the rows below remove both ("the gate is gone": the single-gate checks catch it). One layer alone
    # is NOT equivalent: it changes the ACK when two gates fail at once (spec 5.5 order), which the adjacent-pair checks catch: the FSM layer with the "prec" detector
    # (test_fsm PRECHECK_PAIRS), the feeder layer with the feeder vector af_gate_precedence.
    ("drop_rail_gate_both_layers", [("src/fsm.c", "    if (!rail_on(f)) return RES_INTERLOCK;\n", ""), ("src/feeder.c", "    if (!rail) return RES_INTERLOCK;\n", "")], "gate:rail"),
    ("drop_hb_deadman_inhibit_gate_both_layers", [("src/fsm.c", "    if (!hb_ok(f) || !deadman_ok(f) || inhibit(f)) return RES_INHIBIT;\n", ""), ("src/feeder.c", "    if (!in->hb_ok || !in->deadman || in->inhibit) return RES_INHIBIT;\n", "")], "gate:inhibit"),
    ("drop_range_gate_both_layers", [("src/fsm.c", "    if (!in_sector(f)) return RES_RANGE;\n", ""), ("src/feeder.c", "    if (!in->in_sector) return RES_RANGE;\n", "")], "gate:out_of_sector"),
    ("drop_latched_fault_gate_both_layers", [("src/fsm.c", "    if (f->fault_bits) return RES_JAMMED;  /* any latched fault (also one set by fw_rx earlier in this very tick, e.g. USB overflow) */\n", ""), ("src/fsm.c", "if (f->fire_cmd_pending && f->fault_bits) {", "if (0) {")], "core_usb"),
    ("drop_deadman_gate_both_layers", [("src/fsm.c", "if (!hb_ok(f) || !deadman_ok(f) || inhibit(f)) return RES_INHIBIT;", "if (!hb_ok(f) || inhibit(f)) return RES_INHIBIT;"), ("src/feeder.c", "if (!in->hb_ok || !in->deadman || in->inhibit) return RES_INHIBIT;", "if (!in->hb_ok || in->inhibit) return RES_INHIBIT;")], "gate:deadman"),
    # one layer alone, caught by the ACK precedence pairs (see the comment above)
    ("fsm_layer_only_rail_gate", "src/fsm.c", "    if (!rail_on(f)) return RES_INTERLOCK;\n", "", "prec"),
    ("fsm_layer_only_inhibit_gate", "src/fsm.c", "    if (!hb_ok(f) || !deadman_ok(f) || inhibit(f)) return RES_INHIBIT;\n", "", "prec"),
    ("fsm_layer_only_deadman_term", "src/fsm.c", "if (!hb_ok(f) || !deadman_ok(f) || inhibit(f)) return RES_INHIBIT;", "if (!hb_ok(f) || inhibit(f)) return RES_INHIBIT;", "prec"),
    ("fsm_layer_only_range_gate", "src/fsm.c", "    if (!in_sector(f)) return RES_RANGE;\n", "", "prec"),
    ("fsm_layer_only_latched_fault_gate", "src/fsm.c", "    if (f->fault_bits) return RES_JAMMED;  /* any latched fault (also one set by fw_rx earlier in this very tick, e.g. USB overflow) */\n", "", "prec"),
    ("feeder_layer_only_rail_gate", "src/feeder.c", "    if (!rail) return RES_INTERLOCK;\n", "", "vectors"),
    ("feeder_layer_only_deadman_term", "src/feeder.c", "if (!in->hb_ok || !in->deadman || in->inhibit) return RES_INHIBIT;", "if (!in->hb_ok || in->inhibit) return RES_INHIBIT;", "vectors"),
    ("feeder_layer_only_hb_term", "src/feeder.c", "if (!in->hb_ok || !in->deadman || in->inhibit) return RES_INHIBIT;", "if (!in->deadman || in->inhibit) return RES_INHIBIT;", "vectors"),
    ("feeder_layer_only_inhibit_term", "src/feeder.c", "if (!in->hb_ok || !in->deadman || in->inhibit) return RES_INHIBIT;", "if (!in->hb_ok || !in->deadman) return RES_INHIBIT;", "vectors"),
    ("feeder_layer_only_range_gate", "src/feeder.c", "    if (!in->in_sector) return RES_RANGE;\n", "", "vectors"),
    # the fire sector is shrunk by |axis speed| * t_exit (spec 5.4, fsm.c in_sector())
    ("sector_shrink_removed", "src/fsm.c", "const double mp = fabs(f->ax[0].v) * FDR_SECTOR_T_EXIT_MS, mt = fabs(f->ax[1].v) * FDR_SECTOR_T_EXIT_MS;", "const double mp = 0.0, mt = 0.0;", "core_sector"),
    ("sector_shrink_pan_axis_only", "src/fsm.c", "const double mp = fabs(f->ax[0].v) * FDR_SECTOR_T_EXIT_MS, mt = fabs(f->ax[1].v) * FDR_SECTOR_T_EXIT_MS;", "const double mp = fabs(f->ax[0].v) * FDR_SECTOR_T_EXIT_MS, mt = 0.0;", "core_sector"),
    ("sector_shrink_tilt_axis_only", "src/fsm.c", "const double mp = fabs(f->ax[0].v) * FDR_SECTOR_T_EXIT_MS, mt = fabs(f->ax[1].v) * FDR_SECTOR_T_EXIT_MS;", "const double mp = 0.0, mt = fabs(f->ax[1].v) * FDR_SECTOR_T_EXIT_MS;", "core_sector"),
    ("sector_shrink_fixed_not_speed_scaled", "src/fsm.c", "const double mp = fabs(f->ax[0].v) * FDR_SECTOR_T_EXIT_MS, mt = fabs(f->ax[1].v) * FDR_SECTOR_T_EXIT_MS;", "const double mp = 240.0 * FDR_SECTOR_T_EXIT_MS, mt = 150.0 * FDR_SECTOR_T_EXIT_MS;", "core_sector"),
    # F1-F5 of the cold verification
    ("abort_still_possible_after_breech_clear", [("src/feeder.c", "if (tf < FDR_COMMIT_MS && !left) {", "if (tf < FDR_COMMIT_MS) {"), ("src/feeder.c", "} else if (tf >= FDR_COMMIT_MS || left) {", "} else if (tf >= FDR_COMMIT_MS) {")], "core_f3"),
    ("retract_to_load_without_push_home", "src/feeder.c", "else if (tf >= dwell + FDR_RETRACT_CLEAR_MS && in->push_home) {", "else if (tf >= dwell + FDR_RETRACT_CLEAR_MS) {", "vectors"),
    # hardware shot counter (row 30) and the tightened config ranges
    ("hw_counter_counts_per_arm", "src/fsm.c", "if (!f->key_on) f->muz_key = 0;", "if (!f->key_on || f->shots_arm == 0) f->muz_key = 0;", "core_hw_session"),
    ("hw_counter_no_fault_bit", "src/fsm.c", "        f->fault_bits |= FAULT_HW_COUNTER;", "        ;", "core_hw"),
    ("hw_counter_no_event", "src/fsm.c", "        fw_event(f, EV_HW_COUNTER, f->muz_key);", "        ;", "core_hw"),
    ("hw_counter_never_labelled", "src/fsm.c", "f->muz_key >= f->cfg[CFG_HW_COUNTER_N] && (", "f->muz_key >= 60000 && (", "core_hw"),
    ("hw_label_ignores_count", "src/fsm.c", "f->muz_key >= f->cfg[CFG_HW_COUNTER_N] && (", "(", "core_hw_branch_b"),
    ("hw_label_with_n_zero", "src/fsm.c", "&& f->cfg[CFG_HW_COUNTER_N] > 0 &&", "&&", "core_hw_branch_b"),
    ("hw_label_ignores_stream_skew", "src/fsm.c", "<= FDR_HWC_SKEW_MS)", "<= 100000u)", "core_hw_skew"),
    ("hw_label_repeats_every_tick", "src/fsm.c", "!(f->fault_bits & FAULT_TELEMETRY_LOST) &&", "", "core_hw"),
    ("rpm_window_up_to_100_permille", "src/fsm.c", "case CFG_RPM_WINDOW_PM: return v >= 1 && v <= 20;", "case CFG_RPM_WINDOW_PM: return v >= 1 && v <= 100;", "core_cfg_window"),
    ("settle_ms_zero_allowed", "src/fsm.c", "case CFG_SETTLE_MS: return v >= 20 && v <= 2000;", "case CFG_SETTLE_MS: return v >= 0 && v <= 2000;", "core_cfg_settle"),
]  # fmt: skip


def parts(m):
    """(id, [(file, old, new), ...], detector) of a table row (single edits are 5-tuples, multi-file ones 3-tuples)."""
    return (m[0], [m[1:4]], m[4]) if len(m) == 5 else m


def build_lib(tmp: Path, mutation=None):
    """Copy the firmware tree, apply the mutation, compile the same file set as host/CMakeLists.txt with gcc -> path of the shared library."""
    src = (
        tmp / "firmware" / "cuas2-rp2040"
    )  # (the sources include lite's fw_priv.h by relative path: mirror the layout)
    if src.parent.exists():
        shutil.rmtree(src.parent)
    src.parent.mkdir(parents=True)
    (src.parent / "cuas-lite-rp2040").symlink_to(LITE_SRC.parent)
    shutil.copytree(FW, src, ignore=shutil.ignore_patterns("build", "*.o", "pico"))
    for rel, old, new in parts(mutation)[1] if mutation else ():
        text = (src / rel).read_text()
        assert text.count(old) == 1, (
            f"mutation anchor not unique in {rel}: {old!r} x{text.count(old)}"
        )
        (src / rel).write_text(text.replace(old, new))
    lib = tmp / "libmut.so"
    srcs = [
        LITE_SRC / f for f in ("cobs.c", "crc16.c", "proto.c", "planner.c", "homing.c")
    ]
    srcs += [
        src / "src" / f
        for f in ("config.c", "fsm.c", "launcher.c", "feeder.c", "dshot.c")
    ] + [src / "host" / "fw_host.c"]
    cmd = ["gcc", "-shared", "-fPIC", "-O1", "-std=gnu11", "-fno-strict-aliasing", "-ffp-contract=off", "-w", f"-I{src / 'include'}", f"-I{src / 'src'}",
           *map(str, srcs), "-lm", "-o", str(lib)]  # fmt: skip
    r = subprocess.run(cmd, capture_output=True, text=True)
    assert r.returncode == 0, r.stderr[-1500:]
    return lib


def vectors_fail(lib):
    """Names of the vectors whose closed-loop run on `lib` differs from the stored trace or breaks its hand-written expectation."""
    bad = []
    for name in G.V:
        _, outs, _ = G.run_vec(name, H.CFeeder(str(lib)))
        if H.trace_of(outs) != BY_NAME[name]["trace"] or H.check_expect(
            name, BY_NAME[name]["expect"], H.expect_view(outs)
        ):
            bad.append(name)
    return bad


def core_behaviour_fails(lib, kind):
    """Behaviour checks on the whole core for the controls that are not feeder vectors; True = the check caught the mutant."""
    if kind == "core_n":
        r = H.CoreRig(lib_file=str(lib)).up().engage()
        r.fire(1)
        r.run(10)
        r2 = H.CoreRig(lib_file=str(lib)).up().engage()
        r2.send(p.Fire(1, 2))
        r2.run(10)
        return r2.last_ack(p.FIRE).result != p.BAD_ARG
    if kind == "core_dup":
        r = H.CoreRig(lib_file=str(lib)).up().engage()
        r.fire(77)
        r.run(700)
        r.fire(77)
        r.run(700)
        return len(r.feed_rises) != 1
    if (
        kind == "core_hw"
    ):  # branch A: three balls, then the weapon power goes: exactly one EVENT 16 and b15
        r = H.CoreRig(cfg={p.CFG_HW_COUNTER_N: 3}, lib_file=str(lib)).up().engage()
        TF.shoot(r, 3)
        TF.trip(r)
        r.run(200)
        return len(TF.hwc_events(r)) != 1 or not (
            r.snap["fault_bits"] & p.FAULT_HW_COUNTER
        )
    if (
        kind == "core_hw_branch_b"
    ):  # one ball short of N, and N = 0: an ordinary telemetry_lost, no label
        for n, balls in ((3, 2), (0, 3)):
            r = H.CoreRig(cfg={p.CFG_HW_COUNTER_N: n}, lib_file=str(lib)).up().engage()
            TF.shoot(r, balls)
            TF.trip(r)
            r.run(200)
            if TF.hwc_events(r) or r.snap["fault_bits"] & p.FAULT_HW_COUNTER:
                return True
        return False
    if kind == "core_hw_skew":  # the two streams die 30 ms apart: not one power loss
        r = H.CoreRig(cfg={p.CFG_HW_COUNTER_N: 3}, lib_file=str(lib)).up().engage()
        TF.shoot(r, 3)
        r.plant.f["telemetry_dead_t"] = [r.t + 1, r.t + 31]
        r.run(200)
        return bool(TF.hwc_events(r))
    if (
        kind == "core_hw_session"
    ):  # 2 balls, disarm + ARM again, 1 ball: the key-session count is 3
        r = H.CoreRig(cfg={p.CFG_HW_COUNTER_N: 3}, lib_file=str(lib)).up().engage()
        for k in range(2):
            r.fire(k + 1)
            r.run(600)
        r.send(p.Arm(0, 0, 0))
        r.run(50)
        r.fw_pm = None
        r.engage(spin_ms=1500)
        r.fire(10)
        r.run(600)
        TF.trip(r)
        r.run(200)
        return [e[2] for e in r.events if e[1] == p.EV_HW_COUNTER] != [3]
    if kind.startswith("gate:"):
        setup, want = TF.PRECHECK_GATES[kind[5:]]
        r = H.CoreRig(lib_file=str(lib)).up().engage()
        setup(r)
        r.fire(7)
        r.run(10)
        return r.last_ack(p.FIRE).result != want or bool(r.feed_rises)
    if (
        kind == "prec"
    ):  # FIRE with two gates failing at once: the earlier gate of the spec 5.5 order must answer
        return any(
            TF.precedence_ack(a, b, str(lib))[1] != TF.PRECHECK_GATES[a][1]
            for a, b in TF.PRECHECK_PAIRS
        )
    if kind == "prec_busy":  # FIRE while FIRING: BUSY before BAD_STATE
        return TF.fire_while_firing_ack(str(lib)) != (p.FIRING, p.BUSY)
    if (
        kind == "core_sector"
    ):  # a FIRE while slewing into the shrink band is refused, a parked barrel at the same place may fire
        for axis in (0, 1):
            if TF.slew_fire(axis, (98500, 38900)[axis], str(lib))[0] != p.RANGE:
                return True
            ack, rises = TF.slew_fire(axis, None, str(lib))
            if ack != p.OK or len(rises) != 1:
                return True
        return False
    if kind == "core_state":
        r = H.CoreRig(lib_file=str(lib)).up()
        r.arm()
        r.fire(7)
        r.run(10)
        return r.last_ack(p.FIRE).result != p.BAD_STATE or bool(r.feed_rises)
    if kind == "core_busy":
        r = H.CoreRig(lib_file=str(lib)).up().engage()
        r.fire(1)
        r.fire(2)
        r.run(10)
        return sorted(a.result for _, a in r.acks if a.cmd_id == p.FIRE) != [
            p.OK,
            p.BUSY,
        ]
    if kind == "core_usb":  # F1
        r = H.CoreRig(lib_file=str(lib)).up().engage()
        r.fire(7)
        r.send_raw(b"\x55" * 20000)
        r.run(700)
        return bool(r.feed_rises) or r.last_ack(p.FIRE).result != p.JAMMED
    if (
        kind == "core_f3"
    ):  # the dead-man drops 78 ms into the stroke (breech cleared at ~71): a counted ball, no abort, no fault
        r = H.CoreRig(lib_file=str(lib)).up().engage()
        r.fire(1)
        r.run(78)
        r.send(p.Heartbeat(999, 0))
        r.run(400)
        aborted = any(c == p.EV_FEED_ABORT for _, c, _ in r.events)
        return (
            aborted
            or len(r.plant.exits) != 1
            or r.st.shots_confirmed != 1
            or bool(r.snap["fault_bits"])
        )
    if kind in ("core_cfg_window", "core_cfg_settle"):
        r = H.CoreRig(lib_file=str(lib))
        r.send(p.Hello(p.MAJOR, p.MINOR, 7))
        r.run(20)
        key, bad = (
            (p.CFG_RPM_WINDOW_PM, 21)
            if kind == "core_cfg_window"
            else (p.CFG_SETTLE_MS, 19)
        )
        r.send(p.CfgSet(key, bad))
        r.run(5)
        return r.last_ack(p.CFG_SET).result != p.BAD_ARG
    if kind == "core_vcap":
        r = H.CoreRig(lib_file=str(lib))
        r.send(p.Hello(p.MAJOR, p.MINOR, 7))
        r.run(20)
        r.send(p.CfgSet(p.CFG_V_CAP_DMPS, 381))
        r.run(5)
        return r.last_ack(p.CFG_SET).result != p.BAD_ARG
    raise AssertionError(kind)


def test_positive_control_unmutated_rebuild_passes_everything(tmp_path):
    lib = build_lib(tmp_path)
    assert vectors_fail(lib) == []
    kinds = {parts(m)[2] for m in M} - {"vectors"}
    assert len(kinds) >= 14
    for kind in sorted(kinds):
        assert not core_behaviour_fails(lib, kind), kind


@pytest.mark.parametrize("mutation", M, ids=lambda m: m[0])
def test_negative_control_mutant_is_caught(tmp_path, mutation):
    lib = build_lib(tmp_path, mutation)
    name, _, detector = parts(mutation)
    if detector == "vectors":
        assert vectors_fail(lib), f"{name}: no vector noticed the removed gate"
    else:
        assert core_behaviour_fails(lib, detector), f"{name}: not caught"


def test_the_list_of_controls_covers_the_spec_examples():
    ids = {m[0] for m in M}
    assert {
        "drop_muzzle_clear_check",
        "drop_interval_clamp",
        "drop_unexpected_exit_fault",
    } <= ids
    assert len(ids) == len(M) >= 20
