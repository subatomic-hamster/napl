#!/usr/bin/env python3
"""bin -> UF2 for the RP2040 (flash base 0x10000000, family 0xE48BFF56). The pico-sdk 2.x makes its UF2 with picotool, which needs libusb headers to
build; hfbox has none (no apt), so tools/fw_build.sh converts the SDK's .bin with this instead. UF2 format: https://github.com/microsoft/uf2 (MIT)."""

import struct
import sys

BASE, FAMILY = 0x10000000, 0xE48BFF56


def convert(data: bytes) -> bytes:
    n = (len(data) + 255) // 256
    out = bytearray()
    for i in range(n):
        chunk = data[i * 256 : (i + 1) * 256].ljust(256, b"\0")
        out += struct.pack(
            "<8I", 0x0A324655, 0x9E5D5157, 0x2000, BASE + i * 256, 256, i, n, FAMILY
        )
        out += chunk.ljust(476, b"\0") + struct.pack("<I", 0x0AB16F30)
    return bytes(out)


if __name__ == "__main__":
    src, dst = sys.argv[1:3]
    with open(src, "rb") as f, open(dst, "wb") as g:
        g.write(convert(f.read()))
