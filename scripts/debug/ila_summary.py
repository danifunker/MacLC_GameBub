"""Summarize Vivado ILA CSV captures: per probe, its values and how often it toggles."""
import csv
import sys
from pathlib import Path

for path in sys.argv[1:]:
    rows = list(csv.reader(open(path)))
    header, radix, data = rows[0], rows[1], rows[2:]
    print(f"== {Path(path).name}: {len(data)} samples")
    for col, name in enumerate(header):
        if name.startswith("Sample in") or name in ("TRIGGER",):
            continue
        values = [r[col] for r in data]
        toggles = sum(1 for a, b in zip(values, values[1:]) if a != b)
        distinct = sorted(set(values))
        first_change = next((i for i, (a, b) in enumerate(zip(values, values[1:])) if a != b), None)
        shown = distinct if len(distinct) <= 6 else distinct[:6] + ["..."]
        print(f"  {name:55s} toggles={toggles:5d} values={shown} first_change={first_change}")
