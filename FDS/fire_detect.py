#!/usr/bin/env python3
"""Skysline CanSat 2026 — Fire Detection System (FDS).

Loads an AMG8833 thermal-grid CSV, syncs each frame to CanSat GPS position by
matching `timestamp_ms` against a raw LoRa telemetry log, runs threshold +
persistence + gradient fire detection on the 8x8 grid, computes a Fire Risk
Score (FRS) per frame, and writes:

  1. A georeferenced detections CSV.
  2. A fire-risk map PNG (lat/lon scatter coloured by FRS, fires marked).
  3. A console summary of fire events.

Thermal CSV columns (header required):
    timestamp_ms,cam_file,ir_min,ir_max,ir_avg,p00,p01,...,p63
where p00..p63 are the 8x8 grid in row-major order (p00 = row0/col0 = top-left,
camera perspective; see CLAUDE.md note 2).

Telemetry source: raw LoRa downlink log, one payload per line, e.g.
    TEAM:11,TS:1718000000123,TX:42,...,LAT:16.416300,LON:101.362500,...

Usage:
    python fire_detect.py --thermal thermal.csv --telemetry lora.log \
        --out data/maps/ [--tolerance-ms 500] [--slope-deg 0]

All units metric (deg C, m, degrees). Timestamps Unix ms (uint32 from FSW).
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import re
import sys
from dataclasses import dataclass, field

import numpy as np
import matplotlib

matplotlib.use("Agg")  # headless: write PNG without a display
import matplotlib.pyplot as plt

# ── Fire detection thresholds (CLAUDE.md "Key algorithms") ──────────────────
FIRE_TEMP_THRESHOLD = 55.0   # deg C absolute
FIRE_DELTA_THRESHOLD = 15.0  # deg C above the frame's local mean
PERSISTENCE_FRAMES = 3       # consecutive frames a pixel must stay hot
GRADIENT_THRESHOLD = 10.0    # deg C/pixel (Sobel magnitude) for hotspot edge
MAX_GRAD = 40.0              # deg C/pixel normaliser for grad_norm

# ── FRS weights and normalisers (CLAUDE.md FRS formula) ─────────────────────
W_TEMP = 0.40
W_GRAD = 0.35
W_SLOPE = 0.25
T_MIN, T_MAX = 20.0, 80.0    # T_norm = (temp - 20) / (80 - 20)

# ── FRS zone classification ─────────────────────────────────────────────────
FRS_ZONES = [
    (0.0, 0.3, "No Risk", "#00AA44"),
    (0.3, 0.5, "Low Risk", "#FFCC00"),
    (0.5, 0.7, "Moderate Risk", "#FF7700"),
    (0.7, 1.01, "High Risk", "#DD1111"),
]

GRID = 8  # AMG8833 is 8x8

# Sobel kernels for thermal gradient (deg C/pixel)
_SOBEL_X = np.array([[-1, 0, 1], [-2, 0, 2], [-1, 0, 1]], dtype=float)
_SOBEL_Y = np.array([[-1, -2, -1], [0, 0, 0], [1, 2, 1]], dtype=float)


@dataclass
class ThermalFrame:
    ts: int                  # timestamp_ms
    cam_file: str
    ir_min: float
    ir_max: float
    ir_avg: float
    grid: np.ndarray         # (8, 8) deg C, row-major
    # filled in later
    lat: float | None = None
    lon: float | None = None
    gps_ts: int | None = None
    gps_dt_ms: int | None = None
    candidate_mask: np.ndarray | None = None  # per-pixel hot candidate this frame
    fire_mask: np.ndarray | None = None       # per-pixel confirmed (persistence)
    n_fire_px: int = 0
    max_grad: float = 0.0
    frs: float = 0.0
    zone: str = "No Risk"
    fire_detected: bool = False


# ── Loading ──────────────────────────────────────────────────────────────────
def load_thermal_csv(path: str) -> list[ThermalFrame]:
    """Parse the AMG8833 thermal CSV into ordered ThermalFrame objects."""
    frames: list[ThermalFrame] = []
    pixel_cols = [f"p{i:02d}" for i in range(GRID * GRID)]
    with open(path, newline="") as fh:
        reader = csv.DictReader(fh)
        missing = [c for c in ("timestamp_ms", "ir_min", "ir_max", "ir_avg", *pixel_cols)
                   if c not in (reader.fieldnames or [])]
        if missing:
            sys.exit(f"[FDS] thermal CSV missing columns: {missing[:5]}"
                     f"{'...' if len(missing) > 5 else ''}")
        for ln, row in enumerate(reader, start=2):
            try:
                pixels = np.array([float(row[c]) for c in pixel_cols], dtype=float)
                frames.append(ThermalFrame(
                    ts=int(float(row["timestamp_ms"])),
                    cam_file=row.get("cam_file", ""),
                    ir_min=float(row["ir_min"]),
                    ir_max=float(row["ir_max"]),
                    ir_avg=float(row["ir_avg"]),
                    grid=pixels.reshape(GRID, GRID),
                ))
            except (ValueError, KeyError) as e:
                print(f"[FDS] skip thermal row {ln}: {e}", file=sys.stderr)
    frames.sort(key=lambda f: f.ts)
    print(f"[FDS] loaded {len(frames)} thermal frames from {path}")
    return frames


_TS_RE = re.compile(r"\bTS:(\d+)")
_LAT_RE = re.compile(r"\bLAT:(-?\d+\.?\d*)")
_LON_RE = re.compile(r"\bLON:(-?\d+\.?\d*)")
_FIX_RE = re.compile(r"\bFIX:(\d)")


def load_telemetry_log(path: str, require_fix: bool = True) -> list[tuple[int, float, float]]:
    """Parse raw LoRa payload lines into sorted (ts_ms, lat, lon) tuples.

    Lines without TS/LAT/LON are ignored. With require_fix, drops FIX:0 rows and
    rows at the (0,0) null island (FSW emits 0.0 when no GPS fix).
    """
    fixes: list[tuple[int, float, float]] = []
    with open(path, errors="replace") as fh:
        for line in fh:
            ts_m, lat_m, lon_m = _TS_RE.search(line), _LAT_RE.search(line), _LON_RE.search(line)
            if not (ts_m and lat_m and lon_m):
                continue
            fix_m = _FIX_RE.search(line)
            lat, lon = float(lat_m.group(1)), float(lon_m.group(1))
            if require_fix:
                if fix_m and fix_m.group(1) == "0":
                    continue
                if lat == 0.0 and lon == 0.0:
                    continue
            fixes.append((int(ts_m.group(1)), lat, lon))
    fixes.sort(key=lambda t: t[0])
    print(f"[FDS] parsed {len(fixes)} GPS fixes from {path}")
    return fixes


# ── Sync ──────────────────────────────────────────────────────────────────────
def sync_gps(frames: list[ThermalFrame], fixes: list[tuple[int, float, float]],
             tolerance_ms: int) -> int:
    """Attach nearest GPS fix (within tolerance) to each frame. Returns matched count."""
    if not fixes:
        print("[FDS] WARN: no GPS fixes — frames remain ungeoreferenced", file=sys.stderr)
        return 0
    ts_arr = np.array([f[0] for f in fixes])
    matched = 0
    for fr in frames:
        idx = int(np.searchsorted(ts_arr, fr.ts))
        # check the two neighbours straddling fr.ts, pick closest
        best_i, best_dt = -1, None
        for j in (idx - 1, idx):
            if 0 <= j < len(fixes):
                dt = abs(fixes[j][0] - fr.ts)
                if best_dt is None or dt < best_dt:
                    best_dt, best_i = dt, j
        if best_i >= 0 and best_dt <= tolerance_ms:
            _, fr.lat, fr.lon = fixes[best_i]
            fr.gps_ts = fixes[best_i][0]
            fr.gps_dt_ms = best_dt
            matched += 1
    print(f"[FDS] synced {matched}/{len(frames)} frames to GPS (tol={tolerance_ms}ms)")
    return matched


# ── Detection ──────────────────────────────────────────────────────────────────
def _sobel_magnitude(grid: np.ndarray) -> np.ndarray:
    """3x3 Sobel gradient magnitude (deg C/pixel), edge-padded to keep shape."""
    pad = np.pad(grid, 1, mode="edge")
    gx = np.zeros_like(grid)
    gy = np.zeros_like(grid)
    for r in range(GRID):
        for c in range(GRID):
            win = pad[r:r + 3, c:c + 3]
            gx[r, c] = np.sum(win * _SOBEL_X)
            gy[r, c] = np.sum(win * _SOBEL_Y)
    return np.hypot(gx, gy)


def detect_fires(frames: list[ThermalFrame], slope_deg: float) -> None:
    """Run candidate -> persistence -> FRS over frames (assumed time-ordered)."""
    slope_factor = min(max(math.tan(math.radians(slope_deg)) / math.tan(math.radians(45.0)), 0.0), 1.0)

    # Per-frame candidate mask + gradient
    for fr in frames:
        local_mean = float(fr.grid.mean())
        fr.candidate_mask = (fr.grid > FIRE_TEMP_THRESHOLD) & \
                            (fr.grid > local_mean + FIRE_DELTA_THRESHOLD)
        grad = _sobel_magnitude(fr.grid)
        fr.max_grad = float(grad.max())

    # Persistence: a pixel is confirmed fire only if it was a candidate in this
    # frame AND the previous (PERSISTENCE_FRAMES-1) frames, with the edge/gradient
    # condition met somewhere in the frame.
    for i, fr in enumerate(frames):
        if i + 1 < PERSISTENCE_FRAMES:
            fr.fire_mask = np.zeros_like(fr.candidate_mask)
        else:
            window = [frames[i - k].candidate_mask for k in range(PERSISTENCE_FRAMES)]
            fr.fire_mask = np.logical_and.reduce(window)
        # gradient gate: real fire has a sharp thermal edge
        if fr.max_grad < GRADIENT_THRESHOLD:
            fr.fire_mask = np.zeros_like(fr.fire_mask)
        fr.n_fire_px = int(fr.fire_mask.sum())
        fr.fire_detected = fr.n_fire_px > 0

        # FRS from the hottest pixel + frame gradient + (optional) terrain slope
        hottest = float(fr.grid.max())
        t_norm = min(max((hottest - T_MIN) / (T_MAX - T_MIN), 0.0), 1.0)
        grad_norm = min(max(fr.max_grad / MAX_GRAD, 0.0), 1.0)
        frs = W_TEMP * t_norm + W_GRAD * grad_norm + W_SLOPE * slope_factor
        fr.frs = min(max(frs, 0.0), 1.0)  # CLAUDE.md note 8: always clip
        fr.zone = classify_frs(fr.frs)


def classify_frs(frs: float) -> str:
    for lo, hi, name, _ in FRS_ZONES:
        if lo <= frs < hi:
            return name
    return "High Risk"


def _zone_color(name: str) -> str:
    for _, _, n, color in FRS_ZONES:
        if n == name:
            return color
    return "#888888"


# ── Outputs ──────────────────────────────────────────────────────────────────
def write_detections_csv(frames: list[ThermalFrame], path: str) -> None:
    cols = ["timestamp_ms", "gps_ts_ms", "gps_dt_ms", "lat", "lon", "cam_file",
            "ir_min", "ir_max", "ir_avg", "max_grad", "fire_detected",
            "n_fire_px", "frs", "frs_zone"]
    with open(path, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(cols)
        for fr in frames:
            w.writerow([
                fr.ts, fr.gps_ts if fr.gps_ts is not None else "",
                fr.gps_dt_ms if fr.gps_dt_ms is not None else "",
                f"{fr.lat:.6f}" if fr.lat is not None else "",
                f"{fr.lon:.6f}" if fr.lon is not None else "",
                fr.cam_file, f"{fr.ir_min:.1f}", f"{fr.ir_max:.1f}",
                f"{fr.ir_avg:.1f}", f"{fr.max_grad:.2f}",
                int(fr.fire_detected), fr.n_fire_px, f"{fr.frs:.3f}", fr.zone,
            ])
    print(f"[FDS] wrote detections -> {path}")


def write_timeline_map(frames: list[ThermalFrame], path: str) -> None:
    """No-GPS fallback: FRS + peak temp vs flight time, fire frames marked."""
    if not frames:
        return
    t0 = frames[0].ts
    t = np.array([(f.ts - t0) / 1000.0 for f in frames])  # seconds from start
    frs = np.array([f.frs for f in frames])
    irmax = np.array([f.ir_max for f in frames])

    fig, ax = plt.subplots(figsize=(10, 5))
    ax.plot(t, irmax, "-", color="#888", lw=1.0, label="IR max (°C)", zorder=1)
    sc = ax.scatter(t, frs * 100.0, c=frs, cmap="inferno", vmin=0.0, vmax=1.0,
                    s=45, edgecolors="#222", linewidths=0.3, zorder=2,
                    label="FRS (×100)")
    fires = [i for i, f in enumerate(frames) if f.fire_detected]
    if fires:
        ax.scatter(t[fires], (frs * 100.0)[fires], s=200, facecolors="none",
                   edgecolors="#00e5ff", linewidths=1.6, zorder=3,
                   label=f"FIRE x{len(fires)}")
    ax.axhline(FIRE_TEMP_THRESHOLD, color="#DD1111", ls="--", lw=0.8,
               label=f"{FIRE_TEMP_THRESHOLD:.0f}°C threshold")
    cb = fig.colorbar(sc, ax=ax)
    cb.set_label("Fire Risk Score (FRS)")
    ax.set_xlabel("Flight time (s from first frame)")
    ax.set_ylabel("°C  /  FRS×100")
    ax.set_title("Skysline FDS — Fire Risk Timeline (no GPS)")
    ax.legend(loc="upper left", fontsize=9)
    ax.grid(True, color="#eee", lw=0.5)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)
    print(f"[FDS] wrote timeline -> {path}")


def write_fire_map(frames: list[ThermalFrame], path: str) -> None:
    """Scatter georeferenced frames by lat/lon, colour by FRS, ring fire frames.

    Falls back to a time-series plot when no frame has GPS.
    """
    geo = [f for f in frames if f.lat is not None and f.lon is not None]
    if not geo:
        print("[FDS] no GPS — writing timeline instead of map", file=sys.stderr)
        write_timeline_map(frames, path.replace("frs_map_", "frs_timeline_"))
        return
    lats = np.array([f.lat for f in geo])
    lons = np.array([f.lon for f in geo])
    frs = np.array([f.frs for f in geo])

    fig, ax = plt.subplots(figsize=(9, 7))
    ax.plot(lons, lats, "-", color="#30363d", lw=0.8, zorder=1, alpha=0.7)
    sc = ax.scatter(lons, lats, c=frs, cmap="inferno", vmin=0.0, vmax=1.0,
                    s=55, edgecolors="#222", linewidths=0.4, zorder=2)
    fires = [f for f in geo if f.fire_detected]
    if fires:
        ax.scatter([f.lon for f in fires], [f.lat for f in fires],
                   s=240, facecolors="none", edgecolors="#00e5ff",
                   linewidths=1.8, zorder=3, label=f"FIRE x{len(fires)}")
        ax.legend(loc="upper right")
    cb = fig.colorbar(sc, ax=ax)
    cb.set_label("Fire Risk Score (FRS)")
    ax.set_xlabel("Longitude (deg)")
    ax.set_ylabel("Latitude (deg)")
    ax.set_title("Skysline FDS — Fire Risk Map")
    ax.ticklabel_format(useOffset=False, style="plain")
    ax.grid(True, color="#eee", lw=0.5)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)
    print(f"[FDS] wrote fire map -> {path}")


def write_report_json(frames: list[ThermalFrame], path: str, source: str) -> None:
    """Dump per-frame grids + detection results for the HTML viewer."""
    has_gps = any(f.lat is not None for f in frames)
    t0 = frames[0].ts if frames else 0
    out = {
        "source": os.path.basename(source),
        "has_gps": has_gps,
        "n_frames": len(frames),
        "thresholds": {
            "fire_temp": FIRE_TEMP_THRESHOLD,
            "fire_delta": FIRE_DELTA_THRESHOLD,
            "persistence": PERSISTENCE_FRAMES,
            "gradient": GRADIENT_THRESHOLD,
            "t_min": T_MIN, "t_max": T_MAX,
        },
        "zones": [[lo, hi, name, color] for lo, hi, name, color in FRS_ZONES],
        "frames": [],
    }
    for i, f in enumerate(frames):
        fire_px = (np.flatnonzero(f.fire_mask.ravel()).tolist()
                   if f.fire_mask is not None else [])
        out["frames"].append({
            "i": i,
            "ts": f.ts,
            "t_s": round((f.ts - t0) / 1000.0, 3),
            "cam": os.path.basename(f.cam_file) if f.cam_file else "",
            "ir_min": round(f.ir_min, 1),
            "ir_max": round(f.ir_max, 1),
            "ir_avg": round(f.ir_avg, 1),
            "grid": [round(float(v), 1) for v in f.grid.ravel()],
            "max_grad": round(f.max_grad, 2),
            "frs": round(f.frs, 3),
            "zone": f.zone,
            "fire": bool(f.fire_detected),
            "n_fire": int(f.n_fire_px),
            "fire_px": fire_px,
            "lat": round(f.lat, 6) if f.lat is not None else None,
            "lon": round(f.lon, 6) if f.lon is not None else None,
        })
    with open(path, "w") as fh:
        json.dump(out, fh, separators=(",", ":"))
    print(f"[FDS] wrote report -> {path}")


def print_summary(frames: list[ThermalFrame]) -> None:
    fires = [f for f in frames if f.fire_detected]
    print("\n" + "=" * 64)
    print(f"  FIRE DETECTION SUMMARY  ({len(frames)} frames, {len(fires)} fire frames)")
    print("=" * 64)
    if not fires:
        peak = max(frames, key=lambda f: f.frs, default=None)
        if peak:
            print(f"  No fires confirmed. Peak FRS={peak.frs:.3f} ({peak.zone}) "
                  f"@ ts={peak.ts}, ir_max={peak.ir_max:.1f}C")
        return
    print(f"  {'ts_ms':>14} {'lat':>11} {'lon':>12} {'irMax':>6} "
          f"{'px':>3} {'FRS':>5}  zone")
    print("  " + "-" * 60)
    for f in fires:
        lat = f"{f.lat:.5f}" if f.lat is not None else "  no-gps"
        lon = f"{f.lon:.5f}" if f.lon is not None else "   no-gps"
        print(f"  {f.ts:>14} {lat:>11} {lon:>12} {f.ir_max:>6.1f} "
              f"{f.n_fire_px:>3} {f.frs:>5.3f}  {f.zone}")
    hottest = max(fires, key=lambda f: f.ir_max)
    print("  " + "-" * 60)
    print(f"  Hottest fire: {hottest.ir_max:.1f}C @ ts={hottest.ts} "
          f"({hottest.lat:.5f},{hottest.lon:.5f})" if hottest.lat is not None
          else f"  Hottest fire: {hottest.ir_max:.1f}C @ ts={hottest.ts} (no GPS)")
    print("=" * 64 + "\n")


# ── CLI ────────────────────────────────────────────────────────────────────────
def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Skysline CanSat Fire Detection System")
    ap.add_argument("--thermal", required=True, help="AMG8833 thermal grid CSV")
    ap.add_argument("--telemetry", default=None,
                    help="raw LoRa payload log (TEAM:11,TS:...). Omit to run "
                         "thermal-only (no GPS, detection still runs)")
    ap.add_argument("--out", default="data/maps", help="output directory")
    ap.add_argument("--tolerance-ms", type=int, default=500,
                    help="max ts gap for GPS sync (default 500)")
    ap.add_argument("--slope-deg", type=float, default=0.0,
                    help="terrain slope for FRS slope_factor (default 0, no DEM)")
    ap.add_argument("--allow-no-fix", action="store_true",
                    help="keep FIX:0 / (0,0) telemetry rows")
    args = ap.parse_args(argv)

    if not os.path.isfile(args.thermal):
        sys.exit(f"[FDS] file not found: {args.thermal}")
    if args.telemetry and not os.path.isfile(args.telemetry):
        sys.exit(f"[FDS] file not found: {args.telemetry}")
    os.makedirs(args.out, exist_ok=True)

    frames = load_thermal_csv(args.thermal)
    if not frames:
        sys.exit("[FDS] no thermal frames — nothing to do")
    if args.telemetry:
        fixes = load_telemetry_log(args.telemetry, require_fix=not args.allow_no_fix)
        sync_gps(frames, fixes, args.tolerance_ms)
    else:
        print("[FDS] thermal-only mode — no telemetry, frames ungeoreferenced")
    detect_fires(frames, args.slope_deg)

    stamp = frames[-1].ts
    write_detections_csv(frames, os.path.join(args.out, f"detections_{stamp}.csv"))
    write_fire_map(frames, os.path.join(args.out, f"frs_map_{stamp}.png"))
    write_report_json(frames, os.path.join(args.out, "report.json"), args.thermal)
    print_summary(frames)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
