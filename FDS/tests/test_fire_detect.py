#!/usr/bin/env python3
"""Real-situation test cases for the Skysline CanSat Fire Detection System.

Scenarios mirror the field test plan (CLAUDE.md test IDs) and the kinds of
thermal/GPS data the cansat actually produces over a wildfire flight:

  T-09  single heat source (candle / small fire) -> confirmed after persistence
  T-10  multi-hotspot frame -> high FRS, multiple fire pixels
  T-11  brief flash (sun glint, 1-2 frames) -> rejected by persistence
  T-12  persistence resets across a cool gap -> not confirmed
  T-25  warm afternoon, no fire -> delta threshold rejects (hot but flat)
  T-28  sunlit car roof / asphalt (~52C) -> absolute threshold rejects
  plus  GPS sync (jitter, out-of-tolerance, FIX:0, null-island, garbage lines),
        FRS zones + clipping, and a full flight integration via temp files.

Run:  python -m pytest tests/ -v
"""
from __future__ import annotations

import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import fire_detect as fd  # noqa: E402

GRID = fd.GRID
AMBIENT = 28.0  # typical ground temp, deg C


# ── grid / frame builders ─────────────────────────────────────────────────────
def cool_grid(base: float = AMBIENT, jitter: float = 0.0) -> np.ndarray:
    g = np.full((GRID, GRID), base, dtype=float)
    if jitter:
        g += np.random.default_rng(0).uniform(-jitter, jitter, g.shape)
    return g


def hotspot_grid(peak: float, center=(3, 4), base: float = AMBIENT) -> np.ndarray:
    """Compact fire: one peak pixel with a sharp falloff to ambient (real edge)."""
    g = cool_grid(base)
    r, c = center
    g[r, c] = peak
    for dr, dc, frac in ((0, -1, 0.55), (0, 1, 0.55), (-1, 0, 0.5), (1, 0, 0.5)):
        rr, cc = r + dr, c + dc
        if 0 <= rr < GRID and 0 <= cc < GRID:
            g[rr, cc] = base + (peak - base) * frac
    return g


def frame(ts: int, grid: np.ndarray) -> fd.ThermalFrame:
    return fd.ThermalFrame(
        ts=ts, cam_file=f"img_{ts}.jpg",
        ir_min=float(grid.min()), ir_max=float(grid.max()),
        ir_avg=float(grid.mean()), grid=grid,
    )


def run(frames, slope_deg=0.0):
    fd.detect_fires(frames, slope_deg)
    return frames


# ════════════════════════════════════════════════════════════════════════════
# Detection — real thermal scenarios
# ════════════════════════════════════════════════════════════════════════════
def test_T09_single_candle_confirmed_after_persistence():
    """Steady candle/small fire: no detection until PERSISTENCE_FRAMES reached."""
    frames = [frame(1000 * i, hotspot_grid(72.0)) for i in range(6)]
    run(frames)
    # first two frames lack history -> not yet confirmed
    assert not frames[0].fire_detected
    assert not frames[1].fire_detected
    # third consecutive hot frame onwards -> confirmed
    assert frames[2].fire_detected
    assert all(f.fire_detected for f in frames[2:])
    assert frames[2].n_fire_px >= 1


def test_T10_multi_hotspot_high_frs():
    """Two separate fires in frame -> multiple confirmed pixels, High Risk FRS."""
    def two_fires():
        g = hotspot_grid(75.0, center=(2, 2))
        g[5, 6] = 70.0
        g[5, 5] = AMBIENT + (70.0 - AMBIENT) * 0.55
        return g
    frames = [frame(1000 * i, two_fires()) for i in range(4)]
    run(frames)
    assert frames[3].fire_detected
    assert frames[3].n_fire_px >= 2
    assert frames[3].frs >= 0.7
    assert frames[3].zone == "High Risk"


def test_T11_brief_glint_rejected_by_persistence():
    """Sun glint flares for a single frame -> never confirmed."""
    frames = [frame(1000 * i, cool_grid()) for i in range(6)]
    frames[3] = frame(3000, hotspot_grid(80.0))  # one-frame flash
    run(frames)
    assert not any(f.fire_detected for f in frames)


def test_T12_persistence_resets_across_cool_gap():
    """Candidate for 2 frames, cools 1 frame, returns -> gap breaks the streak."""
    grids = [hotspot_grid(70.0), hotspot_grid(70.0), cool_grid(),
             hotspot_grid(70.0), hotspot_grid(70.0)]
    frames = [frame(1000 * i, g) for i, g in enumerate(grids)]
    run(frames)
    assert not any(f.fire_detected for f in frames)  # never 3 consecutive


def test_T25_warm_afternoon_no_fire_delta_threshold():
    """Whole scene baking at ~56C: above absolute threshold but flat ->
    delta-above-local-mean gate rejects (no fire)."""
    frames = [frame(1000 * i, cool_grid(base=56.0)) for i in range(5)]
    run(frames)
    assert not any(f.fire_detected for f in frames)


def test_T28_car_roof_false_positive_rejected():
    """Sunlit car roof patch ~52C on cool ground -> below 55C absolute -> no fire."""
    def roof():
        g = cool_grid()
        g[4:6, 3:5] = 52.0  # compact warm metal patch
        return g
    frames = [frame(1000 * i, roof()) for i in range(5)]
    run(frames)
    assert not any(f.fire_detected for f in frames)
    assert frames[-1].n_fire_px == 0


def test_cool_scene_low_frs_no_risk():
    frames = [frame(1000 * i, cool_grid(jitter=1.5)) for i in range(4)]
    run(frames)
    assert frames[-1].frs < 0.3
    assert frames[-1].zone == "No Risk"
    assert not frames[-1].fire_detected


# ════════════════════════════════════════════════════════════════════════════
# Gradient (Sobel) — real edge behaviour
# ════════════════════════════════════════════════════════════════════════════
def test_sharp_fire_edge_has_high_gradient():
    g = hotspot_grid(78.0)
    assert fd._sobel_magnitude(g).max() >= fd.GRADIENT_THRESHOLD


def test_uniform_scene_has_zero_gradient():
    assert fd._sobel_magnitude(cool_grid()).max() == pytest.approx(0.0, abs=1e-9)


# ════════════════════════════════════════════════════════════════════════════
# GPS sync — real downlink timing
# ════════════════════════════════════════════════════════════════════════════
def test_sync_nearest_within_tolerance_records_gap():
    frames = [frame(10_000, hotspot_grid(70.0))]
    fixes = [(9_800, 16.4, 101.3), (10_150, 16.5, 101.4)]  # 200ms vs 150ms
    matched = fd.sync_gps(frames, fixes, tolerance_ms=500)
    assert matched == 1
    assert frames[0].lon == 101.4          # picked the 150ms-closer fix
    assert frames[0].gps_dt_ms == 150


def test_sync_drops_out_of_tolerance():
    frames = [frame(10_000, cool_grid())]
    fixes = [(8_000, 16.4, 101.3)]  # 2s gap, tol 500ms
    matched = fd.sync_gps(frames, fixes, tolerance_ms=500)
    assert matched == 0
    assert frames[0].lat is None and frames[0].gps_dt_ms is None


def test_telemetry_filters_no_fix_and_null_island(tmp_path):
    log = tmp_path / "lora.log"
    log.write_text(
        "TEAM:11,TS:1000,LAT:16.416300,LON:101.362500,FIX:1\n"   # keep
        "TEAM:11,TS:2000,LAT:0.000000,LON:0.000000,FIX:0\n"      # drop: no fix + null
        "TEAM:11,TS:3000,LAT:0.000000,LON:0.000000,FIX:1\n"      # drop: null island
        "garbage line no fields\n"                                # drop: unparseable
        "TEAM:11,TS:4000,LAT:16.420000,LON:101.370000,FIX:1\n"  # keep
    )
    fixes = fd.load_telemetry_log(str(log), require_fix=True)
    assert len(fixes) == 2
    assert [f[0] for f in fixes] == [1000, 4000]


def test_telemetry_allow_no_fix_keeps_all_parseable(tmp_path):
    log = tmp_path / "lora.log"
    log.write_text(
        "TEAM:11,TS:1000,LAT:16.4,LON:101.3,FIX:1\n"
        "TEAM:11,TS:2000,LAT:0.0,LON:0.0,FIX:0\n"
    )
    fixes = fd.load_telemetry_log(str(log), require_fix=False)
    assert len(fixes) == 2


# ════════════════════════════════════════════════════════════════════════════
# FRS — zones, clipping, slope contribution
# ════════════════════════════════════════════════════════════════════════════
@pytest.mark.parametrize("frs,zone", [
    (0.00, "No Risk"), (0.29, "No Risk"),
    (0.30, "Low Risk"), (0.49, "Low Risk"),
    (0.50, "Moderate Risk"), (0.69, "Moderate Risk"),
    (0.70, "High Risk"), (1.00, "High Risk"),
])
def test_frs_zone_boundaries(frs, zone):
    assert fd.classify_frs(frs) == zone


def test_frs_clipped_to_unit_range_with_extreme_slope():
    """Extreme inputs (raging fire + cliff slope) must clip to 1.0 (CLAUDE.md #8)."""
    frames = [frame(1000 * i, hotspot_grid(110.0)) for i in range(4)]
    run(frames, slope_deg=80.0)
    assert frames[-1].frs <= 1.0
    assert frames[-1].zone == "High Risk"


def test_slope_increases_frs():
    flat = [frame(1000 * i, hotspot_grid(60.0)) for i in range(4)]
    steep = [frame(1000 * i, hotspot_grid(60.0)) for i in range(4)]
    run(flat, slope_deg=0.0)
    run(steep, slope_deg=30.0)
    assert steep[-1].frs > flat[-1].frs


# ════════════════════════════════════════════════════════════════════════════
# Loaders — robustness
# ════════════════════════════════════════════════════════════════════════════
def test_load_thermal_csv_sorts_and_parses(tmp_path):
    csvf = tmp_path / "thermal.csv"
    cols = ["timestamp_ms", "cam_file", "ir_min", "ir_max", "ir_avg"] + \
           [f"p{i:02d}" for i in range(64)]
    rows = []
    for ts in (2000, 1000):  # out of order on purpose
        px = [str(AMBIENT)] * 64
        rows.append(",".join([str(ts), f"img_{ts}.jpg", "28", "28", "28", *px]))
    csvf.write_text(",".join(cols) + "\n" + "\n".join(rows) + "\n")
    frames = fd.load_thermal_csv(str(csvf))
    assert [f.ts for f in frames] == [1000, 2000]  # sorted ascending
    assert frames[0].grid.shape == (8, 8)


def test_load_thermal_csv_missing_column_exits(tmp_path):
    csvf = tmp_path / "bad.csv"
    csvf.write_text("timestamp_ms,ir_min\n1000,28\n")  # missing pixels etc.
    with pytest.raises(SystemExit):
        fd.load_thermal_csv(str(csvf))


# ════════════════════════════════════════════════════════════════════════════
# Full flight integration — CSV + LoRa log -> georeferenced confirmed fire
# ════════════════════════════════════════════════════════════════════════════
def test_full_flight_georeferenced_fire(tmp_path):
    cols = ["timestamp_ms", "cam_file", "ir_min", "ir_max", "ir_avg"] + \
           [f"p{i:02d}" for i in range(64)]
    trows, lrows = [], []
    for i in range(8):
        ts = 1_718_000_000_000 + i * 1000
        g = hotspot_grid(76.0) if i >= 2 else cool_grid()  # fire from frame 2
        px = [f"{v:.1f}" for v in g.flatten()]
        trows.append(",".join([str(ts), f"img_{ts}.jpg",
                               f"{g.min():.1f}", f"{g.max():.1f}", f"{g.mean():.1f}", *px]))
        gts = ts + (120 if i % 2 else -90)  # jittered telemetry clock
        lat, lon = 16.4163 + i * 0.0001, 101.3625 + i * 0.0001
        lrows.append(f"TEAM:11,TS:{gts},TX:{i},LAT:{lat:.6f},LON:{lon:.6f},FIX:1,"
                     f"IR_MAX:{g.max():.1f}")

    tf = tmp_path / "thermal.csv"
    lf = tmp_path / "lora.log"
    tf.write_text(",".join(cols) + "\n" + "\n".join(trows) + "\n")
    lf.write_text("\n".join(lrows) + "\n")

    frames = fd.load_thermal_csv(str(tf))
    fixes = fd.load_telemetry_log(str(lf))
    matched = fd.sync_gps(frames, fixes, tolerance_ms=500)
    fd.detect_fires(frames, slope_deg=0.0)

    assert matched == 8                                   # all frames georeferenced
    fires = [f for f in frames if f.fire_detected]
    assert fires, "expected confirmed fire frames"
    first = fires[0]
    assert first.lat is not None and first.lon is not None  # georeferenced
    assert first.gps_dt_ms <= 500
    assert first.zone == "High Risk"
    # detection starts at frame index 4 (fire from 2 + 3-frame persistence)
    assert frames[4].fire_detected and not frames[3].fire_detected


def test_main_writes_outputs(tmp_path):
    """End-to-end main(): generates detections CSV + FRS map PNG."""
    cols = ["timestamp_ms", "cam_file", "ir_min", "ir_max", "ir_avg"] + \
           [f"p{i:02d}" for i in range(64)]
    trows, lrows = [], []
    for i in range(5):
        ts = 1000 * i
        g = hotspot_grid(70.0)
        px = [f"{v:.1f}" for v in g.flatten()]
        trows.append(",".join([str(ts), f"img_{ts}.jpg",
                               f"{g.min():.1f}", f"{g.max():.1f}", f"{g.mean():.1f}", *px]))
        lrows.append(f"TEAM:11,TS:{ts},LAT:16.4,LON:101.3,FIX:1")
    tf = tmp_path / "thermal.csv"
    lf = tmp_path / "lora.log"
    outd = tmp_path / "out"
    tf.write_text(",".join(cols) + "\n" + "\n".join(trows) + "\n")
    lf.write_text("\n".join(lrows) + "\n")

    rc = fd.main(["--thermal", str(tf), "--telemetry", str(lf), "--out", str(outd)])
    assert rc == 0
    produced = os.listdir(outd)
    assert any(n.startswith("detections_") and n.endswith(".csv") for n in produced)
    assert any(n.startswith("frs_map_") and n.endswith(".png") for n in produced)
