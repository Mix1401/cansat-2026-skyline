#!/usr/bin/env python3
"""Split a multi-session thermal CSV into per-flight files.

Real logs often concatenate several power-ups, or the MCU clock (millis()
fallback) resets to ~0 between flights. When all rows share one CSV and the
ground pipeline sorts globally by timestamp, sessions interleave and the
per-pixel persistence filter never sees consecutive frames from the same
flight. Splitting on a timestamp reset (ts decreases vs the previous row)
restores per-flight continuity.

    python split_flights.py --thermal data/raw/flight_real/log.csv \
        --out data/processed/ [--min-frames 5]

Writes <stem>_flight0.csv, _flight1.csv, ... and prints a summary.
"""
from __future__ import annotations

import argparse
import csv
import os
import sys


def split(rows: list[list[str]], ts_idx: int) -> list[list[list[str]]]:
    flights: list[list[list[str]]] = []
    cur: list[list[str]] = []
    prev = None
    for r in rows:
        try:
            ts = int(float(r[ts_idx]))
        except (ValueError, IndexError):
            continue
        if prev is not None and ts < prev:   # backwards jump = new session
            flights.append(cur)
            cur = []
        cur.append(r)
        prev = ts
    if cur:
        flights.append(cur)
    return flights


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Split multi-session thermal CSV by ts reset")
    ap.add_argument("--thermal", required=True)
    ap.add_argument("--out", default="data/processed")
    ap.add_argument("--min-frames", type=int, default=1,
                    help="drop flights shorter than this many frames")
    args = ap.parse_args(argv)

    if not os.path.isfile(args.thermal):
        sys.exit(f"file not found: {args.thermal}")
    with open(args.thermal, newline="") as fh:
        rows = list(csv.reader(fh))
    if not rows:
        sys.exit("empty CSV")
    header, body = rows[0], rows[1:]
    if "timestamp_ms" not in header:
        sys.exit("no timestamp_ms column")
    ts_idx = header.index("timestamp_ms")

    flights = split(body, ts_idx)
    os.makedirs(args.out, exist_ok=True)
    stem = os.path.splitext(os.path.basename(args.thermal))[0]

    kept = 0
    print(f"[split] {len(flights)} session(s) in {args.thermal}")
    for n, fl in enumerate(flights):
        if len(fl) < args.min_frames:
            print(f"[split]  flight{n}: {len(fl)} frames — skipped (<{args.min_frames})")
            continue
        ir_max = max((float(r[header.index('ir_max')]) for r in fl), default=0.0)
        t0, t1 = fl[0][ts_idx], fl[-1][ts_idx]
        path = os.path.join(args.out, f"{stem}_flight{n}.csv")
        with open(path, "w", newline="") as out:
            w = csv.writer(out)
            w.writerow(header)
            w.writerows(fl)
        kept += 1
        print(f"[split]  flight{n}: {len(fl):4d} frames  ts {t0}..{t1}  "
              f"ir_max={ir_max:.1f}C  -> {path}")
    print(f"[split] wrote {kept} flight file(s) to {args.out}/")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
