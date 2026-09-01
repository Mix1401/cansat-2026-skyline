# A Low-Cost CanSat for Wildfire Detection and Fire-Risk Scoring Using Fused Visible and Thermal Imaging

**Skysline — CanSat 2026, Team 11**
Prototype report · 2026-06-10

---

## Abstract

We present **Skysline**, a CanSat-class wildfire detection prototype that pairs a
low-resolution thermal array (AMG8833, 8×8) with a visible camera (ESP32-CAM) and
downlinks per-second telemetry over LoRa to a Python ground station. The ground
station — the **Fire Detection System (FDS)** — synchronises each thermal frame to
the cansat's GPS position by timestamp, applies a threshold + persistence + spatial
gradient filter to reject transient and flat hot sources, and computes a per-frame
**Fire Risk Score (FRS)** that combines peak temperature, thermal gradient, and
terrain slope. We validate the pipeline on real flight data containing three
recorded sessions. On a controlled fire flight (80 frames, 79 s), the system
detected a sustained heat source peaking at **368.5 °C** and confirmed fire in
**36 of 80 frames** (FRS saturating at the High-Risk ceiling), while on a 214-frame
ambient flight (peak 42.8 °C) it produced **zero false detections**. A unit-test
suite of 27 cases covering candle-scale sources, sun-glint rejection, persistence
resets, and false-positive scenes passes in full. We also report a practical data
finding: concatenated multi-session logs must be split on timestamp resets before
global sorting, or per-frame persistence is silently broken.

---

## 1. Introduction

Satellite fire products such as NASA FIRMS detect active fires globally but at
coarse resolution (375 m/pixel for VIIRS) and with revisit-limited timing. They
report *where fire is*, not *where it is likely to spread*. A CanSat operating at
100–500 m above ground level (AGL) occupies a complementary niche: very high local
resolution (~0.5 m/pixel at 100 m AGL) over a small footprint, in near-real time,
with the platform's own GPS for georeferencing.

This prototype targets two questions:

1. Can an 8×8 thermal array plus persistence filtering reliably flag a real fire
   while rejecting common false positives (sun glint, warm flat surfaces)?
2. Can a single per-frame Fire Risk Score usefully summarise thermal severity for a
   ground operator in real time?

We answer both affirmatively on recorded flight data and describe the
implementation, the algorithms, and the limitations that remain.

---

## 2. System Architecture

### 2.1 Flight segment (FSW)

| Subsystem | Part | Interface | Output |
|---|---|---|---|
| MCU + radio | Heltec ESP32 WiFi LoRa V3 (SX1262) | — | LoRa downlink |
| Barometer | BMP280 | I²C 0x76 | temp, pressure, baro altitude |
| IMU | MPU6050 | I²C 0x68 | accel, gyro |
| Power monitor | INA219 | I²C 0x40 | bus V, current, power |
| Thermal | AMG8833 | I²C 0x69 | 8×8 array, °C |
| GPS | NEO-M8N | UART 9600 | lat/lon/alt, fix, sats |
| Visible | ESP32-CAM (OV2640) | UART/SD | JPEG frames |

The flight software runs a fixed **1000 ms cycle**: read sensors, build an ASCII
telemetry packet, transmit it over LoRa (923.250 MHz, SF11, BW 125 kHz, CR 4/5,
14 dBm), then transmit the 8×8 thermal grid as fragmented packets. Timestamps are
Unix milliseconds from GPS time when a fix is available, falling back to `millis()`
otherwise — a fallback that has direct consequences for ground processing (§6.2).

### 2.2 Ground segment (FDS)

The ground station ingests two products:

- a **thermal CSV** (one row per frame: `timestamp_ms, cam_file, ir_min, ir_max,
  ir_avg, p00…p63`), where `p00…p63` is the 8×8 grid in row-major order
  (`p00` = top-left in camera perspective); and
- a **telemetry source** carrying GPS — either a raw LoRa payload log
  (`TEAM:11,TS:…,LAT:…,LON:…,FIX:…`) or, when GPS is absent, nothing at all.

FDS runs entirely offline in Python (NumPy + Matplotlib) and emits a georeferenced
detections CSV, a fire-risk map or timeline PNG, a machine-readable `report.json`,
and a console event table. A static HTML viewer renders the report frame-by-frame.

---

## 3. Data Pipeline and Synchronisation

```
thermal.csv ─┐
             ├─ load → time-order frames
lora.log ────┘            │
                          ├─ GPS sync: nearest fix within tolerance
                          ├─ detect: threshold → persistence → gradient gate
                          ├─ score: Fire Risk Score + zone
                          └─ outputs: detections.csv · map/timeline.png · report.json
```

### 3.1 Timestamp synchronisation

Thermal frames and GPS fixes are produced on the same clock base (`unixMs()` in the
flight software) but rarely share an identical millisecond. For each thermal frame,
FDS locates the GPS fix with the nearest timestamp and accepts it only if the gap is
within a tolerance (default 500 ms), recording the actual offset (`gps_dt_ms`) for
audit. Fixes flagged `FIX:0` or sitting at the (0,0) null island are dropped before
matching. When no telemetry is supplied, detection still runs; frames remain
ungeoreferenced and the map output degrades gracefully to a time-series.

---

## 4. Fire Detection Method

### 4.1 Candidate pixels

A pixel is a fire *candidate* in a frame when it is both absolutely and locally hot:

```
candidate(r,c) = grid[r,c] > FIRE_TEMP_THRESHOLD      (55 °C)
             AND grid[r,c] > frame_mean + FIRE_DELTA   (mean + 15 °C)
```

The absolute term rejects warm-but-cool scenes; the local-contrast term rejects a
uniformly baking field (e.g. midday tarmac), where every pixel is hot but none
stands out.

### 4.2 Persistence

Real fire is temporally stable; sun glint and reflective flashes are not. A
candidate is *confirmed* only if the **same pixel** is a candidate in
`PERSISTENCE_FRAMES = 3` consecutive frames. This is the single most important
false-positive guard and the reason §6.2's session-splitting matters.

### 4.3 Spatial gradient gate

Fire fronts present sharp thermal edges. We compute a 3×3 Sobel magnitude
(°C/pixel) over the grid and require the frame's peak gradient to exceed
`GRADIENT_THRESHOLD = 10 °C/pixel` before confirming any pixel — suppressing slow,
diffuse warm gradients that lack a combustion edge.

### 4.4 Fire Risk Score

Per frame:

```
FRS = 0.40 · T_norm + 0.35 · grad_norm + 0.25 · slope_factor      (clipped to [0,1])

  T_norm       = (T_peak − 20) / (80 − 20)            clipped [0,1]
  grad_norm    = peak_gradient / 40                   clipped [0,1]
  slope_factor = tan(slope°) / tan(45°)               clipped [0,1]
```

| FRS range | Zone | Colour |
|---|---|---|
| 0.0 – 0.3 | No Risk | green |
| 0.3 – 0.5 | Low Risk | yellow |
| 0.5 – 0.7 | Moderate Risk | orange |
| 0.7 – 1.0 | High Risk | red |

Without a Digital Elevation Model the slope term is zero, so in the flat-terrain
results below FRS caps at **0.75** — still inside the High-Risk band, but the
ceiling should be read as "thermal-only maximum," not a true 1.0.

---

## 5. Ground Software and Viewer

`fire_detect.py` is the pipeline entry point. `split_flights.py` separates
multi-session logs (§6.2). The HTML viewer (`viewer/index.html`, vanilla JS +
canvas, no external dependencies) shows, per frame: the visible image, the 8×8
thermal heatmap with per-cell °C labels and confirmed-fire pixels ringed, a stats
panel (IR min/max/avg, gradient, FRS bar, zone badge, fire flag, GPS when present),
and a clickable timeline of IR-max + FRS + fire markers against the 55 °C line. It
is driven from `report.json` and served as a static site (`python -m http.server`).

---

## 6. Results

We evaluate on a real recorded log (`log.csv`, 307 data rows) plus synthetic and
unit tests. The log contained **three concatenated sessions**, separated as below.

### 6.1 Per-session detection

| Session | Frames | Duration | Peak IR | Frames > 55 °C | Confirmed fire frames | Outcome |
|---|---:|---:|---:|---:|---:|---|
| Flight 0 (ambient) | 214 | 213 s | 42.8 °C | 0 | 0 | True negative |
| Flight 1 (aborted) | 13 | 12 s | 28.8 °C | 0 | 0 | True negative |
| Flight 2 (fire) | 80 | 79 s | **368.5 °C** | 51 | **36** | True positive |

On the **fire flight**, the heat source ignited mid-record; the first confirmed
detection landed at frame 24 (ts 30380 ms), consistent with the 3-frame persistence
delay after the source crossed threshold. FRS held at the thermal-only ceiling
(0.750, High Risk) throughout the burn. On the **ambient flight** the system raised
no detections across 214 frames despite IR-max excursions to 42.8 °C, demonstrating
that the 55 °C absolute threshold plus local-contrast term suppresses warm-scene
false positives.

### 6.2 Finding: multi-session logs break persistence under global sort

The three sessions each restart their clock near ~6000 ms (a `millis()` fallback
reset between power-ups). Naively concatenating them and sorting globally by
timestamp **interleaves** frames from different flights — an ambient frame between
every two fire frames — so no pixel is hot in three *consecutive* frames and the
persistence filter reports **zero fires despite a 368 °C source.** Detecting a
backward timestamp jump and splitting into per-session files before processing
(`split_flights.py`) restores continuity and recovers all 36 fire frames. This is a
general hazard for any persistence-based detector fed concatenated telemetry and is
worth stating explicitly: *sort within a session, never across sessions.*

### 6.3 Synthetic and unit validation

A 27-case unit suite (`tests/`) exercises the detector against field-test
scenarios:

| Scenario | Expectation | Result |
|---|---|---|
| Steady candle / small fire | confirm after 3 frames | pass |
| Two simultaneous hotspots | ≥2 fire pixels, High Risk | pass |
| Single-frame sun glint | rejected (persistence) | pass |
| Hot→cool→hot gap | streak broken, no confirm | pass |
| Whole scene ~56 °C (flat) | rejected (local-contrast) | pass |
| Sunlit car roof ~52 °C | rejected (absolute threshold) | pass |
| GPS jitter / out-of-tolerance / FIX:0 / null-island | correct sync + filtering | pass |
| FRS zone boundaries + clipping | exact | pass |
| Full flight integration | georeferenced confirmed fire | pass |

All 27 pass (`python -m pytest tests/ -v`).

---

## 7. Comparison with NASA FIRMS

| Capability | NASA FIRMS | Skysline CanSat |
|---|---|---|
| Platform altitude | 705 km LEO | 100–500 m AGL |
| Thermal sensor | VIIRS, 375 m/pixel | AMG8833 8×8 |
| Local resolution | 375 m minimum | ~0.5 m at 100 m AGL |
| Geolocation | star tracker, sub-pixel | GPS, ±3 m |
| Downlink | X-band 15 Mbps | LoRa 915 MHz class |
| Detection | contextual threshold | threshold + persistence + gradient |
| Fire-risk score | not provided | per-frame FRS (thermal + gradient + slope) |
| Spread prediction | not provided | slope/wind vector (future work) |
| Coverage | global daily | ~200×200 m per flight |

FIRMS and Skysline are complementary: global early warning versus high-resolution
local confirmation and per-frame risk scoring.

---

## 8. Limitations and Future Work

1. **No GPS in the present datasets.** Both real flights were thermal-only, so
   detections are time-referenced, not mapped. The sync path is implemented and
   unit-tested but not yet exercised on real GPS telemetry.
2. **Slope term inert without a DEM.** FRS caps at 0.75. Integrating a DEM (or an
   onboard IMU-derived slope proxy) would restore the full [0,1] range and enable
   spread-direction vectors.
3. **Persistence assumes a static pixel.** Platform motion can walk a hot source
   across grid cells within three frames, weakening confirmation. Tracking the
   hotspot centroid (rather than a fixed index) would make persistence
   motion-robust.
4. **8×8 resolution.** Sharp thermal boundaries are under-sampled; Gaussian
   pre-smoothing before any upscaling is recommended for mapping.
5. **No magnetometer.** Spread vectors (future) assume north-up; heading error can
   be large.
6. **Extreme-temperature realism.** A 368 °C reading exceeds the AMG8833's nominal
   0–80 °C specified range; such values should be treated as "saturated / very hot"
   qualitative flags rather than calibrated temperatures.

---

## 9. Conclusion

Skysline demonstrates that an 8×8 thermal array, disciplined by absolute and local
thresholds, temporal persistence, and a spatial-gradient gate, can confirm a real
fire (368 °C, 36/80 frames) while producing zero false positives across a 214-frame
ambient flight. A single per-frame Fire Risk Score and a dependency-free browser
viewer make the output legible to a ground operator in near-real time. The most
actionable lesson from real data was procedural rather than algorithmic:
persistence-based detection demands per-session timestamp continuity, which
concatenated logs silently violate. With GPS telemetry and a DEM added, the same
pipeline extends naturally from detection to georeferenced fire-spread prediction —
the capability that distinguishes this CanSat from existing satellite products.

---

## References

1. NASA FIRMS — Fire Information for Resource Management System. https://firms.modaps.eosdis.nasa.gov/
2. Panasonic AMG8833 Grid-EYE Infrared Array Sensor — datasheet.
3. Semtech SX1262 LoRa Transceiver — datasheet.
4. Harris & Stephens, "A Combined Corner and Edge Detector," 1988 (Sobel/gradient background).
5. Project repository: `Cansat2026-skysline` — `FSW/`, `FDS/`, this report under `docs/paper/`.

---

*Reproduce the results in this report:*

```bash
cd FDS && source .venv/bin/activate
python split_flights.py --thermal data/raw/flight_real/log.csv --out data/processed/
python fire_detect.py  --thermal data/processed/log_flight2.csv --out data/maps/fire/   # fire flight
python fire_detect.py  --thermal data/processed/log_flight0.csv --out data/maps/amb/    # ambient flight
python -m pytest tests/ -v
```
