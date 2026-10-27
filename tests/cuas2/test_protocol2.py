"""Protocol 1.1 (spec 6.6 items 1-7): the frames decode identically in the C core (host library) and in napl.cuas2.protocol; behaviour of the whole C core on the
new/changed messages; the shared v1 framing vectors (tests/lite/protocol_vectors.json) against the cuas2 library; CRC check value 0x29B1.
Needs the host library: tools/cuas2_fw_host_test.sh builds it (or NAPL_CUAS2_FW=PATH)."""

import binascii
import ctypes as C
import json
import subprocess
import sys
from pathlib import Path

import pytest
from napl.cuas2 import protocol as p

from cuas2 import fwharness as H  # tests/ is on sys.path (tests/cuas2 is a package)

HERE = Path(__file__).parent
V = json.loads((HERE / "protocol_vectors2.json").read_text())
V1 = json.loads((HERE.parent / "lite" / "protocol_vectors.json").read_text())


@pytest.fixture(scope="module")
def L():
    H.require_lib()
    return H.lib()


def c_encode(L, id_, seq, payload):
    buf = (C.c_uint8 * 128)()
    n = L.fwh_frame_encode(id_, seq, payload, len(payload), buf)
    return bytes(buf[:n])


def c_decode(L, stream):
    out, bad = (C.c_uint8 * 8192)(), C.c_uint32()
    n = L.fwh_decode_stream(stream, len(stream), out, 8192, C.byref(bad))
    raw, frames, i = bytes(out[:n]), [], 0
    while i < n:
        frames.append((raw[i], raw[i + 1], raw[i + 3 : i + 3 + raw[i + 2]]))
        i += 3 + raw[i + 2]
    return frames, bad.value


def build(frame):
    cls, f = p.MESSAGES2[frame["id"]], dict(frame["fields"])
    if "uid" in f:
        f["uid"] = bytes.fromhex(f["uid"])
    return cls(**f)


# ---- CRC, framing, vectors ----
def test_crc_check_value_c_python_stdlib(L):
    d = b"123456789"
    assert p.crc16(d) == L.fwh_crc16(d, 9) == binascii.crc_hqx(d, 0xFFFF) == 0x29B1
    assert V["crc16_check"]["crc"] == "0x29B1"


@pytest.mark.parametrize("frame", V["frames"], ids=lambda f: f["name"])
def test_new_frames_identical_in_c_and_python(L, frame):
    m, wire = build(frame), bytes.fromhex(frame["frame_hex"])
    payload = bytes.fromhex(frame["payload_hex"])
    assert p.encode(frame["seq"], m) == wire and m.pack() == payload  # python encodes
    assert (
        c_encode(L, frame["id"], frame["seq"], payload) == wire
    )  # C encodes the same bytes
    assert c_decode(L, b"\0" + wire) == (
        [(frame["id"], frame["seq"], payload)],
        0,
    )  # C decodes
    d = p.decode2(wire)  # python decodes
    assert (d.id, d.seq, d.payload) == (frame["id"], frame["seq"], payload)
    assert d.msg == m and d.msg.asdict() == frame["fields"]
    assert p.Decoder2().feed(b"\0" + wire)[0].msg == m


def test_status2_is_45_bytes_and_v1_status_29_is_accepted():
    assert V["status2_len"] == p.STATUS_LEN == 45 == len(p.Status2().pack())
    assert V["status_v1_len"] == 29
    s = p.Status2.unpack(bytes.fromhex(V["status_v1_payload_hex"]))
    assert (s.rpm_top, s.last_v_dmps, s.gate_state, s.shots_confirmed) == (
        0xFFFF,
        0xFFFF,
        p.GS_UNKNOWN,
        0,
    )
    f = p.Decoder2().feed(
        b"\0" + p.encode_raw(p.STATUS, 1, bytes.fromhex(V["status_v1_payload_hex"]))
    )[0]
    assert isinstance(f.msg, p.Status2) and not f.bad_arg
    assert p.Flywheel.unpack(b"\x20\x03") == p.Flywheel(800, None)  # no tail
    assert p.Flywheel.unpack(b"\x20\x03\xe7\xff") == p.Flywheel(800, -25)


def test_dshot_reference_values_and_codec(L):
    d = V["dshot"]
    assert (
        L.fwh_dshot_frame(1046, 0, 0) == 1046 << 5 | 0x6
        and int(d["frame_throttle_1046_regular"], 16) == 1046 << 5 | 0x6
    )
    assert (
        L.fwh_dshot_frame(1046, 0, 1) == 1046 << 5 | 0x9
        and int(d["frame_throttle_1046_inverted_crc"], 16) == 1046 << 5 | 0x9
    )
    out = (C.c_int32 * 1)()
    for payload in (0x000, 0x5A5, 0xFFF, 0x123):
        v21 = L.fwh_gcr_encode(payload << 4 | _nib_crc(payload))
        assert L.fwh_gcr_decode(v21, out) == 0 and out[0] == payload
    assert L.fwh_gcr_decode(0x1FFFFF, out) != 0  # not a valid symbol stream


def _nib_crc(payload12):
    x = payload12 ^ payload12 >> 4 ^ payload12 >> 8
    return ~x & 0xF


# ---- the shared v1 framing against the cuas2 library ----
def test_lite_v1_vectors_against_cuas2_core(L):
    for f in V1["crc16"]["frames"]:
        b = bytes.fromhex(f["body_hex"])
        assert L.fwh_crc16(b, len(b)) == int(f["crc"], 16)
    for c in V1["cobs"]:
        d, e = bytes.fromhex(c["decoded_hex"]), bytes.fromhex(c["encoded_hex"])
        buf = (C.c_uint8 * 700)()
        assert bytes(buf[: L.fwh_cobs_encode(d, len(d), buf)]) == e
    for fr in V1["frames"]:
        wire, pl = bytes.fromhex(fr["frame_hex"]), bytes.fromhex(fr["payload_hex"])
        assert c_encode(L, fr["id"], fr["seq"], pl) == wire
        assert c_decode(L, b"\0" + wire) == ([(fr["id"], fr["seq"], pl)], 0)
    for name, h in V1["bad_frames_hex"].items():
        assert c_decode(L, b"\0" + bytes.fromhex(h)) == ([], 1), name
    s = V1["stream"]
    frames, bad = c_decode(L, bytes.fromhex(s["bytes_hex"]))
    assert [f[0] for f in frames] == s["expect_ids"] and bad == s["expect_bad_frames"]


def test_lite_test_protocol_runs_unchanged():
    r = subprocess.run(
        [sys.executable, "-m", "pytest", "-q", "-p", "no:cacheprovider", str(HERE.parent / "lite" / "test_protocol.py")],
        capture_output=True,
        text=True,
    )  # fmt: skip
    assert r.returncode == 0, r.stdout[-800:]


# ---- behaviour of the whole C core on the new messages ----
def acks_of(r, cmd_id):
    return [a.result for _, a in r.acks if a.cmd_id == cmd_id]


def fire_result(r, fid, n=1):
    r.send(p.Fire(fid, n))
    r.run(10)
    return r.last_ack(p.FIRE).result


def test_status_on_the_wire_is_45_bytes():
    r = H.CoreRig()
    r.up(home=False)
    assert r.status_len == 45 and isinstance(r.st, p.Status2)


def test_flywheel_with_and_without_tail():
    for tail, sign in ((None, 1), (30, 1), (-30, -1)):
        r = H.CoreRig().up().engage(spin_ms=0)
        r.fw_pm, r.fw_diff = 1000, tail
        r.run(2500)
        top, bot = r.st.rpm_top, r.st.rpm_bot
        assert abs(top - 17500) < 2000 and abs(bot - 17500) < 2000, (tail, top, bot)
        want = 15 if tail is None else tail  # no tail: cfg 29 (default +15 permille)
        assert (bot - top) * 1000 / 17500 == pytest.approx(want, abs=8), (
            tail,
            top,
            bot,
        )
    # a 2-byte FLYWHEEL (v1 app) is still accepted
    r = H.CoreRig().up().engage()
    assert r.st.state == p.ENGAGING


def test_fire_n_must_be_one():
    r = H.CoreRig().up().engage()
    assert r.st.ready
    assert fire_result(r, 1, n=0) == p.BAD_ARG
    assert fire_result(r, 2, n=2) == p.BAD_ARG
    assert fire_result(r, 3, n=255) == p.BAD_ARG
    assert r.feed_rises == [] and r.plant.exits == []
    assert fire_result(r, 4, n=1) == p.OK
    r.run(600)
    assert len(r.feed_rises) == 1 and len(r.plant.exits) == 1


def test_duplicate_fire_id_is_acked_again_and_not_fed_again():
    r = H.CoreRig().up().engage()
    assert fire_result(r, 77) == p.OK
    r.run(700)  # long after the min interval, wheels recovered, ball restaged
    assert r.st.ready
    assert fire_result(r, 77) == p.OK  # retransmission: the old answer
    r.run(700)
    assert len(r.feed_rises) == 1 and len(r.plant.exits) == 1
    assert fire_result(r, 78) == p.OK  # a new id is a new shot
    r.run(700)
    assert len(r.feed_rises) == 2 and len(r.plant.exits) == 2


def test_retransmission_in_the_same_tick_is_not_told_ok_before_the_feeder_decides():
    """Found by the fuzz (seed 14826): the dedupe ring held the provisional OK, so a duplicate arriving in the tick of the first FIRE got OK for a shot the
    feeder then refused (TOO_SOON)."""
    r = H.CoreRig().up().engage()
    r.send(p.Fire(5, 1))
    r.send(p.Fire(5, 1))  # both frames are drained in one tick
    r.run(20)
    res = sorted(a.result for _, a in r.acks if a.cmd_id == p.FIRE)
    assert res == [p.OK, p.BUSY], res
    r.run(700)
    assert len(r.feed_rises) == 1 and r.accepted_fires == 1
    r.fire(5)  # later retransmission: the final answer of the first
    r.run(20)
    assert r.last_ack(p.FIRE).result == p.OK and len(r.feed_rises) == 1


def test_event_codes_10_to_16_are_emitted():
    seen = set()
    r = H.CoreRig().up().engage()  # 14 WHEELS_READY
    assert fire_result(r, 1) == p.OK
    r.run(700)
    seen |= {c for _, c, _ in r.events}  # 10 MUZZLE, 14, 15
    r = H.CoreRig(pusher="dead", stuck_t=2000).up().engage()  # 13 JAM
    r.run(0)
    fire_result(r, 1)
    r.run(300)
    seen |= {c for _, c, _ in r.events}
    r = H.CoreRig(hopper=0).up()  # 12 EMPTY
    r.run(2500)
    seen |= {c for _, c, _ in r.events}
    r = (
        H.CoreRig().up().engage()
    )  # 11 FEED_ABORT: the deadman drops 10 ms into the stroke
    r.send(p.Fire(1, 1))
    r.run(10)
    r.send(p.Heartbeat(1, 0))  # deadman not ok, inhibit off
    r.run(200)
    seen |= {c for _, c, _ in r.events}
    assert {
        p.EV_MUZZLE,
        p.EV_FEED_ABORT,
        p.EV_EMPTY,
        p.EV_JAM,
        p.EV_WHEELS_READY,
        p.EV_WHEELS_LOST,
    } <= seen, sorted(seen)


# ---- CFG_SET: keys 23-41, ranges, maintenance acts ----
def set_cfg(r, key, val):
    r.send(p.CfgSet(key, val))
    r.run(5)
    return r.last_ack(p.CFG_SET).result


def test_cfg_ranges_reject_out_of_range():
    r = H.CoreRig()
    r.send(p.Hello(p.MAJOR, p.MINOR, 7))  # SAFE: maintenance keys may be written
    r.run(20)
    for key, (lo, hi) in p.CFG_RANGES.items():
        assert set_cfg(r, key, lo - 1) == p.BAD_ARG, ("below", key)
        assert set_cfg(r, key, hi + 1) == p.BAD_ARG, ("above", key)
        assert set_cfg(r, key, lo) == p.OK and r.L.fwh_cfg(r.h, key, 0) == lo, (
            "lo",
            key,
        )
        assert set_cfg(r, key, hi) == p.OK and r.L.fwh_cfg(r.h, key, 0) == hi, (
            "hi",
            key,
        )
    assert set_cfg(r, p.CFG_V_CAP_DMPS, 381) == p.BAD_ARG
    assert set_cfg(r, p.CFG_WHEEL_D_UM, 0) == p.BAD_ARG
    assert set_cfg(r, 42, 1) == p.BAD_ARG  # beyond the key table


def test_maintenance_keys_only_in_safe():
    r = H.CoreRig().up(home=False)  # IDLE
    for key, val in (
        (p.CFG_V_CAP_DMPS, 300),
        (p.CFG_WHEEL_D_UM, 30000),
        (p.CFG_HW_COUNTER_N, 5),
        (p.CFG_HARD_SHOT_CAP, 3),
        (p.CFG_HARD_ARM_TIMEOUT, 30),
    ):
        assert set_cfg(r, key, val) == p.BAD_STATE, key
    assert set_cfg(r, p.CFG_KP, 500) == p.OK  # a tuning key is allowed in IDLE


def test_min_shot_interval_is_clamped_to_470():
    r = H.CoreRig().up(home=False)
    assert set_cfg(r, p.CFG_MIN_SHOT_INTERVAL_MS, 100) == p.OK
    assert r.L.fwh_cfg(r.h, p.CFG_MIN_SHOT_INTERVAL_MS, 0) == p.MIN_SHOT_INTERVAL_FLOOR
