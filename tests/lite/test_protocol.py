"""The shared protocol vectors (tests/lite/protocol_vectors.json, also run by the firmware's C tests) against napl.lite.protocol."""

import binascii
import json
import random
from pathlib import Path

import pytest
from napl.lite import protocol as p

V = json.loads((Path(__file__).parent / "protocol_vectors.json").read_text())


def build(frame):
    cls, f = p.MESSAGES[frame["id"]], dict(frame["fields"])
    if "uid" in f:
        f["uid"] = bytes.fromhex(f["uid"])
    return cls(**f)


def test_crc_check_value_and_independent_implementation():
    assert p.crc16(b"123456789") == p.CRC_CHECK == int(V["crc16"]["check"], 16)
    assert (
        binascii.crc_hqx(b"123456789", 0xFFFF) == 0x29B1
    )  # CRC-16/CCITT-FALSE from the stdlib
    for f in V["crc16"]["frames"]:
        b = bytes.fromhex(f["body_hex"])
        assert p.crc16(b) == int(f["crc"], 16) == binascii.crc_hqx(b, 0xFFFF)
    rng = random.Random(1)
    for _ in range(200):
        d = bytes(rng.randrange(256) for _ in range(rng.randrange(0, 70)))
        assert p.crc16(d) == binascii.crc_hqx(d, 0xFFFF)


def test_cobs_published_vectors():
    for c in V["cobs"]:
        d, e = bytes.fromhex(c["decoded_hex"]), bytes.fromhex(c["encoded_hex"])
        assert p.cobs_encode(d) == e and p.cobs_decode(e) == d
        assert 0 not in e


def test_cobs_round_trip_random_and_rejects_garbage():
    rng = random.Random(2)
    for _ in range(300):
        d = bytes(
            rng.choice((0, 0, 1, 255, rng.randrange(256)))
            for _ in range(rng.randrange(0, 600))
        )
        e = p.cobs_encode(d)
        assert 0 not in e and p.cobs_decode(e) == d
    for bad in (b"\x05\x11\x22", b"\x00", b"\x02\x00"):
        with pytest.raises(p.ProtocolError):
            p.cobs_decode(bad)


@pytest.mark.parametrize("frame", V["frames"], ids=lambda f: f["name"])
def test_frame_vectors_both_directions(frame):
    m = build(frame)
    wire = bytes.fromhex(frame["frame_hex"])
    assert p.encode(frame["seq"], m) == wire  # encode
    assert m.pack().hex() == frame["payload_hex"]
    assert wire[-1] == 0 and 0 not in wire[:-1]
    d = p.decode(wire)  # decode
    assert (d.id, d.seq, d.payload.hex(), d.bad_arg) == (
        frame["id"],
        frame["seq"],
        frame["payload_hex"],
        False,
    )
    assert d.msg == m and d.msg.asdict() == frame["fields"]
    assert p.crc16(cobs_body(wire)[:-2]) == binascii.crc_hqx(
        cobs_body(wire)[:-2], 0xFFFF
    )  # the CRC inside is the CCITT-FALSE one
    dec = p.Decoder()
    assert (
        dec.feed(b"\0" + wire)[0].msg == m
    )  # the same through the streaming decoder (after a resync byte)


def cobs_body(wire):
    return p.cobs_decode(wire[:-1])


def test_directions_do_not_overlap_and_all_ids_covered():
    names = {f["id"] for f in V["frames"]}
    assert names == set(p.MESSAGES)
    assert {f["id"] for f in V["frames"] if f["dir"] == "host_to_mcu"} == p.HOST_IDS
    assert {f["id"] for f in V["frames"] if f["dir"] == "mcu_to_host"} == p.MCU_IDS


def test_status_is_29_bytes_and_aim_20():
    assert (
        len(p.Status().pack()) == 29
        and len(p.Aim().pack()) == 20
        and len(p.HelloAck().pack()) == 18
    )


def test_bad_frames_are_dropped_silently():
    b = V["bad_frames_hex"]
    for name, h in b.items():
        dec = p.Decoder()
        assert dec.feed(b"\0" + bytes.fromhex(h)) == [] and dec.bad_frames == 1, name
    with pytest.raises(p.ProtocolError):
        p.decode(bytes.fromhex(b["bad_crc"]))


def test_stream_resync_and_counters():
    s = V["stream"]
    dec = p.Decoder()
    got = []
    raw = bytes.fromhex(s["bytes_hex"])
    for i in range(0, len(raw), 7):  # arbitrary chunking must not matter
        got += dec.feed(raw[i : i + 7])
    assert [f.id for f in got] == s["expect_ids"] and dec.bad_frames == s[
        "expect_bad_frames"
    ]


def test_minor_version_tail_accepted_short_payload_is_bad_arg():
    long = p.encode_raw(
        p.HEARTBEAT, 1, p.Heartbeat(7, 1).pack() + b"\xaa\xbb"
    )  # a later minor appended two bytes
    f = p.decode(long)
    assert f.msg == p.Heartbeat(7, 1) and not f.bad_arg
    short = p.decode(p.encode_raw(p.AIM, 2, b"\x01\x02\x03"))
    assert short.msg is None and short.bad_arg
    unk = p.decode(p.encode_raw(0x7E, 3, b"xyz"))
    assert unk.msg is None and not unk.bad_arg and unk.payload == b"xyz"


def test_payload_limit_and_seq_wrap():
    p.encode_raw(0x86, 0, b"x" * 60)
    with pytest.raises(p.ProtocolError):
        p.encode_raw(0x86, 0, b"x" * 61)
    assert p.decode(p.encode_raw(p.HEARTBEAT, 256 + 3, p.Heartbeat().pack())).seq == 3
    dec = p.Decoder()
    wire = b"".join(p.encode(s, p.Heartbeat(s, 0)) for s in (254, 255, 0, 1, 5))
    assert (
        len(dec.feed(b"\0" + wire)) == 5 and dec.seq_gaps == 3
    )  # 255 -> 0 wraps cleanly; 1 -> 5 skips three


def test_version_handshake():
    assert (
        p.hello_result(p.Hello(1, 7, 1)) == 0 and p.hello_result(p.Hello(2, 0, 1)) == 1
    )
    assert (
        p.handshake_ok(p.HelloAck(1, 0, 0))
        and not p.handshake_ok(p.HelloAck(1, 0, 1))
        and not p.handshake_ok(p.HelloAck(2, 0, 0))
    )
