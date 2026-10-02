#!/usr/bin/env python3
"""Decode the core's PMOD debug beacon (rtl/gamebub/gb_debug_uart.sv).

Wire a 3.3 V USB-serial adapter's RX to PMOD pin 3 and GND to GND, then:

    python3 scripts/beacon_decode.py /dev/tty.usbserial-XXXX    # live (needs pyserial)
    python3 scripts/beacon_decode.py capture.txt                # a saved capture
    screen /dev/tty.usbserial-XXXX 115200                       # raw, no decoding

Each beacon line is "MLC" followed by eight hex words; the layout is listed in
rtl/gamebub/maclc_gamebub.sv above the gb_debug_uart instance.
"""
import sys

FILES = ["hard disk", "PRAM", "floppy", "ROM"]
FIXUP = ["idle", "read0", "read1", "write0", "write1", "done"]


def decode(words):
    host, rom, hd, flp, bd, cpu, misc, status = words
    out = []
    if host >> 16 != 0x4842:
        out.append(f"host word {host:08X}: bad marker (expected 4842....)")
    present = [FILES[i] for i in range(4) if host >> (4 + i) & 1]
    order = ("little-endian" if host >> 8 & 1 else "big-endian")
    verdict = ("ROM says LE" if host >> 10 & 1 else "ROM says BE" if host >> 9 & 1
               else "ROM gave no verdict (default used)")
    fx = (host >> 12) & 7
    out.append("host: setup={} run={} rom_loaded={} fixup={}({}) focus={} files=[{}] order={} ({})".format(
        host & 1, host >> 1 & 1, host >> 2 & 1, host >> 3 & 1,
        FIXUP[fx] if fx < len(FIXUP) else fx, host >> 11 & 1, ", ".join(present), order, verdict))
    out.append(f"rom first word {rom:08X}   hd {hd} bytes   floppy {flp} bytes")
    out.append(f"block device: {bd & 0xFFFF} reads, {bd >> 16} writes")
    out.append(f"cpu address {cpu:08X}" + ("  (ROM)" if (cpu & 0xFFFFFF) >= 0xA00000 and (cpu & 0xFFFFFF) < 0xB00000 else ""))
    out.append("frames {}  vram queue overflows {}  cpu_reset_n {}  disk act {}  focus {}  core_reset {}".format(
        misc >> 16, misc >> 8 & 0xFF, misc >> 7 & 1, misc >> 5 & 3, misc >> 1 & 1, misc & 1))
    out.append(f"status[] {status:08X}  (10MB={status >> 4 & 1} floppy_write={status >> 14 & 1})")
    return "\n".join(out)


def lines(source):
    if source.startswith("/dev/"):
        import serial  # pip install pyserial
        with serial.Serial(source, 115200, timeout=2) as port:
            while True:
                yield port.readline().decode("ascii", "replace")
    else:
        with open(source, errors="replace") as f:
            yield from f


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    for line in lines(sys.argv[1]):
        parts = line.split()
        if len(parts) != 9 or parts[0] != "MLC":
            continue
        try:
            words = [int(p, 16) for p in parts[1:]]
        except ValueError:
            continue
        print(decode(words))
        print("-" * 72)


if __name__ == "__main__":
    main()
