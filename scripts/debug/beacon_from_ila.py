"""Rebuild the 8 debug-beacon words from an ILA CSV of snap_reg bits and decode them."""
import csv
import re
import sys

sys.path.insert(0, str(__import__("pathlib").Path(__file__).resolve().parents[1]))  # scripts/
from beacon_decode import decode  # noqa: E402

for path in sys.argv[1:]:
    rows = list(csv.reader(open(path)))
    header, data = rows[0], rows[2:]
    bits = {}
    for col, name in enumerate(header):
        m = re.search(r"snap_reg_n_0_\[(\d+)(?::\d+)?\]", name)
        if m:
            bits[int(m.group(1))] = col
    snapshots = []
    for row in (data[0], data[-1]):
        value = 0x4842 << 16  # word 0 [31:16]: the constant "HB" marker
        for i, col in bits.items():
            if row[col] == "1":
                value |= 1 << i
        snapshots.append([(value >> (32 * w)) & 0xFFFFFFFF for w in range(8)])
    print(f"== {path}: {len(bits)} snap bits, {len(data)} samples")
    for label, words in zip(("first", "last"), snapshots):
        print(f"-- {label} sample: " + " ".join(f"{w:08X}" for w in words))
    print(decode(snapshots[-1]))
    live = [n for n in header if "snap_reg" not in n and not n.startswith("Sample")]
    for name in live:
        col = header.index(name)
        values = [r[col] for r in data]
        toggles = sum(1 for a, b in zip(values, values[1:]) if a != b)
        print(f"  live {name:50s} toggles={toggles:4d} values={sorted(set(values))}")
