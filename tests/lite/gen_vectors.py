"""Prints tests/lite/protocol_vectors.json from napl.lite.protocol (python tests/lite/gen_vectors.py > tests/lite/protocol_vectors.json).

The COBS and CRC entries below are NOT derived from the module: they are the published reference vectors (Wikipedia COBS examples, the
CRC-16/CCITT-FALSE check value 0x29B1), so a bug in protocol.py cannot hide in a self-generated file. test_protocol.py also recomputes
CRCs with binascii.crc_hqx (an independent implementation).
"""

import json

from napl.lite import protocol as p

HOST = [
    ("hello", 0, p.Hello(1, 0, 0x01020304)),
    ("hello_session_zero", 1, p.Hello(1, 0, 0)),
    ("heartbeat", 2, p.Heartbeat(0x1234, p.HB_DEADMAN_OK | p.HB_FIRE_INHIBIT)),
    ("heartbeat_zero", 3, p.Heartbeat(0, 0)),
    ("aim", 4, p.Aim(-12345, 67890, -300, 250, 2400, 1500, 2000, 1500)),
    ("aim_zero", 5, p.Aim(0, 0, 0, 0, 0, 0, 0, 0)),
    ("home", 6, p.Home(3)),
    ("flywheel", 7, p.Flywheel(800)),
    ("fire", 8, p.Fire(0x0100, 3)),
    ("arm", 9, p.Arm(1, 5, 60)),
    ("disarm", 10, p.Arm(0, 0, 0)),
    ("cfg_set", 11, p.CfgSet(p.CFG_PAN_MIN, -170000)),
    ("cfg_get", 12, p.CfgGet(p.CFG_WD_MS)),
    ("cfg_save", 13, p.CfgSave()),
    ("clear_fault", 14, p.ClearFault()),
    ("bootsel", 255, p.Bootsel(0xB007B007)),
]
MCU = [
    (
        "hello_ack",
        0,
        p.HelloAck(1, 0, 0, 1, 2, 3, bytes.fromhex("e66050a8d3156f21"), 0xDEADBEEF),
    ),
    ("hello_ack_mismatch", 1, p.HelloAck(1, 0, 1, 0, 0, 0, bytes(8), 0)),
    (
        "status",
        2,
        p.Status(
            123456,
            p.ENGAGING,
            0xAF,
            0x35,
            -4444,
            3555,
            800,
            0xFFFF,
            1234,
            3,
            5,
            0x0011,
            57,
            0x0102,
        ),
    ),
    ("status_zero", 3, p.Status(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)),
    ("ack", 4, p.Ack(p.FIRE, 8, p.NOT_SPUN, 0)),
    ("cfg_val", 5, p.CfgVal(p.CFG_PAN_MIN, -170000)),
    ("event_shot", 6, p.Event(p.EV_SHOT, (0x0100 << 8) | 2)),
    ("event_state", 7, p.Event(p.EV_STATE_CHANGE, p.SAFE | (p.R_WATCHDOG << 8))),
    ("debug", 8, p.Debug("hello from the pico")),
]


def entry(direction, name, seq, m):
    wire = p.encode(seq, m)
    return dict(
        name=name,
        dir=direction,
        id=m.ID,
        seq=seq,
        fields=m.asdict(),
        payload_hex=m.pack().hex(),
        frame_hex=wire.hex(),
    )


def main():
    cobs = [  # (decoded hex, encoded hex), Wikipedia "Consistent Overhead Byte Stuffing" examples
        ("00", "0101"),
        ("0000", "010101"),
        ("001100", "01021101"),
        ("11223300", "0411223301"),
        ("11220033", "03112202" + "33"),
        ("11223344", "0511223344"),
        ("11000000", "02110101" + "01"),
    ]
    r254 = bytes(range(1, 255))
    r255a = bytes(range(255))
    r255b = bytes(range(1, 256))
    r255c = bytes(range(2, 256)) + b"\0"
    cobs += [(r254.hex(), "ff" + r254.hex()), (r255a.hex(), "01ff" + bytes(range(1, 255)).hex()),
             (r255b.hex(), "ff" + bytes(range(1, 255)).hex() + "02ff"), (r255c.hex(), "ff" + bytes(range(2, 256)).hex() + "0101")]  # fmt: skip
    good = [p.encode(s, m) for _, s, m in HOST[:3]]
    bad_crc = bytearray(good[0])
    bad_crc[-2] ^= (
        0x80  # high bit of the last CRC byte: never makes a zero byte, so the COBS structure survives
    )
    stream = (
        b"\x11\x22\x33\x00"
        + good[0]
        + bytes(bad_crc)
        + b"\x05\x11\x22\x00"
        + good[1]
        + b"\x01\x01\x00"
        + good[2]
    )
    out = dict(
        spec="docs/cuas-lite/spec.md section 2 (protocol v1.0)",
        crc16=dict(
            poly="0x1021",
            init="0xFFFF",
            refin=False,
            refout=False,
            xorout=0,
            check_input_ascii="123456789",
            check="0x29B1",
            frames=[
                dict(body_hex=b.hex(), crc=hex(p.crc16(b)))
                for b in (b"\x02\x00", b"\x82\x07\x00\x01\xff")
            ],
        ),
        cobs=[dict(decoded_hex=d, encoded_hex=e) for d, e in cobs],
        frames=[entry("host_to_mcu", *v) for v in HOST]
        + [entry("mcu_to_host", *v) for v in MCU],
        bad_frames_hex=dict(
            bad_crc=bytes(bad_crc).hex(),
            bad_cobs="051122" + "00",
            too_short=p.cobs_encode(b"\x02\x01\x00").hex() + "00",
        ),
        stream=dict(
            note="a cold decoder ignores bytes up to the first 0x00, then yields hello, hello, heartbeat and counts 3 bad frames (bad CRC, bad COBS, 1-byte body)",
            bytes_hex=stream.hex(),
            expect_ids=[m.ID for _, _, m in HOST[:3]],
            expect_bad_frames=3,
        ),
    )
    print(json.dumps(out, indent=1))


main()
