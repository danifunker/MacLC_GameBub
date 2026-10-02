#!/usr/bin/env python3
"""Convert the testbench's binary PPM frames (sim_out/*.ppm) to PNG.

    python3 scripts/ppm2png.py sim_out/frame_0400.ppm [more.ppm ...]

Standard library only (zlib), so it runs anywhere Python does.
"""
import struct
import sys
import zlib
from pathlib import Path


def read_ppm(path):
    data = Path(path).read_bytes()
    parts, pos = [], 0
    while len(parts) < 4:
        while data[pos:pos + 1].isspace():
            pos += 1
        start = pos
        while not data[pos:pos + 1].isspace():
            pos += 1
        parts.append(data[start:pos])
    pos += 1  # single whitespace after maxval
    if parts[0] != b"P6":
        raise ValueError(f"{path}: not a binary PPM")
    w, h = int(parts[1]), int(parts[2])
    return w, h, data[pos:pos + w * h * 3]


def write_png(path, w, h, rgb):
    def chunk(kind, body):
        return (struct.pack(">I", len(body)) + kind + body
                + struct.pack(">I", zlib.crc32(kind + body) & 0xFFFFFFFF))

    raw = b"".join(b"\x00" + rgb[y * w * 3:(y + 1) * w * 3] for y in range(h))
    png = (b"\x89PNG\r\n\x1a\n"
           + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
           + chunk(b"IDAT", zlib.compress(raw, 9))
           + chunk(b"IEND", b""))
    Path(path).write_bytes(png)


for name in sys.argv[1:]:
    w, h, rgb = read_ppm(name)
    out = str(Path(name).with_suffix(".png"))
    write_png(out, w, h, rgb)
    print(out)
