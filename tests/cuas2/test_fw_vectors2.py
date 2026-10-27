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
    ("drop_wheels_abort", "src/feeder.c", "else if (tf < FDR_WHEEL_CHECK_MS && !d->inwin_now) reason = FA_WHEELS;", "else if (0) reason = FA_WHEELS;", "vectors"),
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
    ("hw_counter_never_labelled", "src/fsm.c", "n > 0 && seen >= n)", "n > 1000 && seen >= n)", "core_hw"),
    ("v_cap_above_38ms_accepted", "src/fsm.c", "case CFG_V_CAP_DMPS: return v >= 1 && v <= 380;", "case CFG_V_CAP_DMPS: return v >= 1;", "core_vcap"),
]  # fmt: skip


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
    if mutation:
        _, rel, old, new, _ = mutation
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
    if kind == "core_hw":
        r = H.CoreRig(cfg={p.CFG_HW_COUNTER_N: 3}, lib_file=str(lib)).up().engage()
        for k in range(3):
            r.fire(k + 1)
            r.run(600)
        r.estop_open = True
        r.run(50)
        return not [e for e in r.events if e[1] == p.EV_HW_COUNTER]
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
    for kind in ("core_n", "core_dup", "core_hw", "core_vcap"):
        assert not core_behaviour_fails(lib, kind), kind


@pytest.mark.parametrize("mutation", M, ids=lambda m: m[0])
def test_negative_control_mutant_is_caught(tmp_path, mutation):
    lib = build_lib(tmp_path, mutation)
    if mutation[4] == "vectors":
        failing = vectors_fail(lib)
        assert failing, f"{mutation[0]}: no vector noticed the removed gate"
    else:
        assert core_behaviour_fails(lib, mutation[4]), f"{mutation[0]}: not caught"


def test_the_list_of_controls_covers_the_spec_examples():
    ids = {m[0] for m in M}
    assert {
        "drop_muzzle_clear_check",
        "drop_interval_clamp",
        "drop_unexpected_exit_fault",
    } <= ids
    assert len(ids) == len(M) >= 20
