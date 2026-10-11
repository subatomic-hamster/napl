"""The shared protocol vectors (protocol_vectors.json) against the firmware's C core (CRC-16, COBS, frame encode and decode, streaming decoder), plus a
differential fuzz against napl.lite.protocol. Needs the host build of the firmware: tools/fw_host_test.sh builds it (skipped when the library is absent)."""

import ctypes as C
import json
import random
from pathlib import Path

import pytest
from fwmcu import available, lib
from napl.lite import protocol as p

pytestmark = pytest.mark.skipif(
    not available(), reason="firmware host library not built (tools/fw_host_test.sh)"
)
V = json.loads((Path(__file__).parent / "protocol_vectors.json").read_text())


def buf(n):
    return (C.c_uint8 * n)()


def c_crc(data):
    return lib().fwh_crc16(bytes(data), len(data))


def c_cobs_encode(data):
    out = buf(len(data) + len(data) // 254 + 2)
    n = lib().fwh_cobs_encode(bytes(data), len(data), out)
    return bytes(out[:n])


def c_cobs_decode(data):
    out = buf(len(data) + 2)
    n = lib().fwh_cobs_decode(bytes(data), len(data), out, len(out))
    return None if n < 0 else bytes(out[:n])


def c_frame(id_, seq, payload):
    out = buf(80)
    n = lib().fwh_frame_encode(id_, seq, bytes(payload), len(payload), out)
    return bytes(out[:n])


def c_stream(data):
    """-> ([(id, seq, payload)], bad_frames)"""
    out, bad = buf(4096), C.c_uint32()
    n = lib().fwh_decode_stream(bytes(data), len(data), out, len(out), C.byref(bad))
    frames, i, o = [], 0, bytes(out[:n])
    while i < n:
        ln = o[i + 2]
        frames.append((o[i], o[i + 1], o[i + 3 : i + 3 + ln]))
        i += 3 + ln
    return frames, bad.value


def test_crc16_check_value_and_frame_crcs():
    assert c_crc(b"123456789") == int(V["crc16"]["check"], 16) == 0x29B1
    for f in V["crc16"]["frames"]:
        assert c_crc(bytes.fromhex(f["body_hex"])) == int(f["crc"], 16)


def test_cobs_vectors():
    for c in V["cobs"]:
        d, e = bytes.fromhex(c["decoded_hex"]), bytes.fromhex(c["encoded_hex"])
        assert c_cobs_encode(d) == e, c["decoded_hex"][:20]
        assert c_cobs_decode(e) == d


@pytest.mark.parametrize("frame", V["frames"], ids=lambda f: f["name"])
def test_frame_vectors_encode_and_decode_byte_identical(frame):
    wire, payload = (
        bytes.fromhex(frame["frame_hex"]),
        bytes.fromhex(frame["payload_hex"]),
    )
    assert c_frame(frame["id"], frame["seq"], payload) == wire  # encode
    frames, bad = c_stream(b"\0" + wire)  # decode (after a resync byte)
    assert bad == 0 and frames == [(frame["id"], frame["seq"], payload)]


def test_bad_frames_are_dropped_and_counted():
    for name, hexs in V["bad_frames_hex"].items():
        frames, bad = c_stream(b"\0" + bytes.fromhex(hexs))
        assert frames == [] and bad == 1, name


def test_stream_vector_resync_and_counts():
    s = V["stream"]
    frames, bad = c_stream(bytes.fromhex(s["bytes_hex"]))
    assert [f[0] for f in frames] == s["expect_ids"] and bad == s["expect_bad_frames"]


def test_differential_fuzz_against_the_python_protocol():
    rng = random.Random(7)
    for _ in range(400):
        d = bytes(
            rng.choice((0, 0, 1, 255, rng.randrange(256)))
            for _ in range(rng.randrange(0, 300))
        )
        e = p.cobs_encode(d)
        assert (
            c_cobs_encode(d) == e and c_cobs_decode(e) == d and c_crc(d) == p.crc16(d)
        )
    for _ in range(300):  # random frames
        pl = bytes(rng.randrange(256) for _ in range(rng.randrange(0, 61)))
        i, s = rng.randrange(256), rng.randrange(256)
        assert c_frame(i, s, pl) == p.encode_raw(i, s, pl)
    for _ in range(
        300
    ):  # random garbage and corrupted frames: same frames out, same bad count
        stream = bytearray()
        for _ in range(rng.randrange(1, 12)):
            w = bytearray(
                p.encode_raw(
                    rng.choice((1, 2, 3, 0x82, 0x7E)),
                    rng.randrange(256),
                    bytes(rng.randrange(256) for _ in range(rng.randrange(0, 40))),
                )
            )
            if rng.random() < 0.4:
                w[rng.randrange(len(w) - 1)] ^= 1 << rng.randrange(8)
            if rng.random() < 0.15:
                w = (
                    bytearray(
                        rng.randrange(1, 256) for _ in range(rng.randrange(1, 120))
                    )
                    + b"\0"
                )
            stream += w
        dec = p.Decoder()
        want = [(f.id, f.seq, f.payload) for f in dec.feed(bytes(stream))]
        got, bad = c_stream(bytes(stream))
        assert got == want and bad == dec.bad_frames
