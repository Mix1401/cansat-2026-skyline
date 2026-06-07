#!/usr/bin/env python3
"""Generate synthetic thermal CSV + raw LoRa log for testing fire_detect.py.

Simulates a CanSat pass over a hotspot: a fire appears mid-flight and persists,
GPS drifts along a track, and telemetry timestamps are jittered vs thermal
timestamps to exercise nearest-within-tolerance sync.

    python gen_sample.py --frames 30 --out data/raw/
"""
from __future__ import annotations

import argparse
import csv
import os
import random

GRID = 8
BASE_TS = 1_718_000_000_000  # unix ms
random.seed(11)


def make_grid(hot: bool) -> list[float]:
    px = [round(random.uniform(24.0, 32.0), 1) for _ in range(GRID * GRID)]
    if hot:
        # plant a hotspot around pixel (row3,col4) = idx 28, with falloff
        center = 3 * GRID + 4
        for idx, hot_t in ((center, 78.0), (center - 1, 64.0), (center + 1, 66.0),
                           (center - GRID, 60.0), (center + GRID, 58.0)):
            if 0 <= idx < GRID * GRID:
                px[idx] = hot_t
    return px


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--frames", type=int, default=30)
    ap.add_argument("--out", default="data/raw")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    thermal_path = os.path.join(args.out, "thermal.csv")
    lora_path = os.path.join(args.out, "lora.log")

    pix_cols = [f"p{i:02d}" for i in range(GRID * GRID)]
    with open(thermal_path, "w", newline="") as tf, open(lora_path, "w") as lf:
        tw = csv.writer(tf)
        tw.writerow(["timestamp_ms", "cam_file", "ir_min", "ir_max", "ir_avg", *pix_cols])
        for i in range(args.frames):
            ts = BASE_TS + i * 1000
            hot = i >= 10  # fire ignites at frame 10, persists
            grid = make_grid(hot)
            tw.writerow([ts, f"img_{ts}_{i}.jpg",
                         min(grid), max(grid), round(sum(grid) / len(grid), 1), *grid])

            # GPS track drifts NE; telemetry ts jittered +-200ms vs thermal ts
            lat = 16.41630 + i * 0.00012
            lon = 101.36250 + i * 0.00009
            gts = ts + random.randint(-200, 200)
            lf.write(f"TEAM:11,TS:{gts},TX:{i},T:31.5,P:1008.2,AB:152.0,"
                     f"LAT:{lat:.6f},LON:{lon:.6f},AG:150.0,SAT:9,FIX:1,"
                     f"AX:0.10,AY:0.05,AZ:9.79,GX:0.001,GY:0.002,GZ:0.000,"
                     f"IR_MIN:{min(grid):.1f},IR_MAX:{max(grid):.1f},IR_AVG:{sum(grid)/len(grid):.1f},"
                     f"V:3.92,I:120.5,W:472.3\n")

    print(f"wrote {thermal_path}\nwrote {lora_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
