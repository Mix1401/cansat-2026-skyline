# Sky's Line — CanSat 2026 Project Description

> **Purpose of this document:** Source material for an AI agent (or human) updating my CV/resume.
> Emphasize my role as **Project Manager & Technical Advisor** — not as an individual contributor.

---

## Project Summary

**Sky's Line** (Team 11) is a student CanSat (can-sized satellite) project built for the **CanSat 2026 competition**. The team designed, built, and flew an autonomous wildfire-detection payload: a soda-can-sized probe that is launched to altitude, descends by parachute, and detects fire/hot-spot signatures on the ground in real time.

**Mission concept:** Airborne wildfire detection. During descent, the CanSat streams live telemetry and infrared thermal imagery over long-range radio to a ground station, where a custom pipeline flags fire signatures and maps them.

**Award:** 🏆 **Best Failure Award** — recognized by competition judges for the team's exemplary failure analysis, transparent post-mortem culture, and engineering recovery process. (Frame this positively on the CV: it is an award for how the team diagnosed, documented, and responded to in-flight anomalies — a resilience/engineering-rigor award, not a loss.)

---

## System Overview (what the team built)

### Flight Segment (FSW — Flight Software)
- **Platform:** Heltec ESP32 WiFi LoRa V3 (SX1262 radio), Arduino/C++
- **Sensor suite (I²C + UART):**
  - BMP280 — temperature, pressure, barometric altitude
  - MPU6050 — 3-axis accelerometer + gyroscope
  - INA219 — power monitoring (bus voltage, current, power)
  - AMG8833 — 8×8 infrared thermal array (core fire-detection sensor)
  - NEO-M8N GPS — position, altitude, satellite time
- **Downlink:** LoRa @ 923.25 MHz, SF11/125 kHz, custom ASCII telemetry protocol (~220-byte packets, 1 Hz) plus a custom fragmented binary protocol for transmitting full 8×8 thermal frames every cycle
- **Robustness features:** GPS-time sync with `millis()` fallback, deep-sleep-surviving counters, CRC-protected packets

### Ground Segment (FDS — Fire Detection System)
- **Platform:** Python ground station
- **Capabilities:**
  - Real-time LoRa serial ingest of telemetry + thermal-grid fragments
  - Fire-detection pipeline over IR thermal data (`fire_detect.py`)
  - Live viewer for telemetry and thermal imagery, with image logging and fire-map overlays
  - Flight-data tooling: sample-data generation, per-flight data splitting, automated tests

### Supporting hardware experiments
- ESP32-CAM sketches for live video streaming and secondary sensing (evaluated, then descoped from the final flight configuration — an example of active scope management).

---

## My Role: Project Manager & Technical Advisor

**Important for the CV agent:** Position me as the person who led, coordinated, and advised — I owned delivery and direction, not day-to-day coding of every module.

### Project Management
- Led a multidisciplinary student team (embedded firmware, ground software, hardware/electronics, testing) through the full engineering lifecycle: mission definition → design reviews → integration → field testing → competition flight → post-flight analysis.
- Owned the project timeline and milestone planning against fixed, immovable competition deadlines.
- Managed scope actively: made descoping calls (e.g., cutting ESP32-CAM live video from the flight build) to protect the core mission — reliable telemetry + fire detection — under time and power budgets.
- Coordinated the interface between the flight-software and ground-station sub-teams, including freezing and documenting the telemetry protocol (packet format, thermal-grid fragmentation scheme) so both sides could develop in parallel.
- Ran risk management for a single-shot flight event: fallback behaviors (GPS-less operation, timestamp fallbacks), pre-flight checklists, and range testing of the radio link.
- Drove documentation discipline: engineering docs, repository conventions, and the competition paper.

### Technical Advisory
- Advised on system architecture: sensor selection, I²C bus topology, LoRa link-budget trade-offs (spreading factor vs. data rate vs. range), and the split between onboard processing and ground-side fire detection.
- Guided the design of the custom telemetry protocol and the compact thermal-grid encoding (8-bit quantization of IR pixels at 0.5 °C resolution to fit LoRa airtime constraints).
- Mentored team members on embedded debugging, git workflow, and test practices.
- Led the failure-analysis process after in-flight anomalies — structured post-mortems, root-cause analysis, and corrective actions — the work that earned the **Best Failure Award**.

---

## Outcomes & Achievements

- Delivered a complete, integrated flight system: flight software + ground station + fire-detection pipeline, flown at the CanSat 2026 competition.
- **Best Failure Award** at CanSat 2026 — judges' recognition of the team's failure analysis, honest reporting, and engineering-recovery process.
- Established a working real-time downlink of environmental telemetry and thermal imagery at 1 Hz over a multi-kilometer LoRa link.
- Built a reusable ground-station toolkit (viewer, detection pipeline, flight-data tools with automated tests) that outlives the competition.

---

## Suggested CV bullet points (for the agent to adapt)

- Project Manager & Technical Advisor, **Sky's Line — CanSat 2026** (Team 11): led a student team through the full lifecycle of an airborne wildfire-detection satellite payload, from mission design to competition flight.
- Directed system architecture spanning embedded flight software (ESP32/C++, multi-sensor I²C suite, GPS, LoRa) and a Python ground station with a real-time infrared fire-detection pipeline.
- Defined and froze the team's custom LoRa telemetry protocol (1 Hz telemetry + fragmented 8×8 thermal-image downlink), enabling parallel development of flight and ground segments.
- Led structured failure analysis and post-flight review that earned the team the **Best Failure Award**, recognizing engineering rigor and transparent post-mortem culture.
- Managed scope, schedule, and risk against fixed competition deadlines, including descoping decisions that protected core mission reliability.

## Skills demonstrated (keywords for CV/ATS)

Project management · Systems engineering · Team leadership · Technical mentoring · Risk management · Scope management · Embedded systems (ESP32, Arduino/C++) · LoRa / RF telemetry · Sensor integration (IMU, barometer, IR thermal array, GPS, power monitoring) · Python · Real-time data pipelines · Fire/anomaly detection · Failure analysis & post-mortems · Cross-team interface definition · Aerospace competition (CanSat)

---

## Notes for the CV agent

1. **Tone:** confident, outcome-focused; avoid implying I wrote all the code myself.
2. **Award framing:** always present "Best Failure Award" with context (failure-analysis excellence), never as a bare phrase that could be misread.
3. **Team name spelling:** "Sky's Line" (also appears as "Skysline" in the repository name).
4. Verify dates against the actual competition schedule before publishing (project active through 2026; competition: CanSat 2026).
