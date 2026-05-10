// flight.h
// CanSat Skyline — Flight state machine
// Handles: WAITING → BOOST → DEPLOYED → LANDED
// Parachute servo deploy, landing detection, MOSFET power cut

#ifndef FLIGHT_H
#define FLIGHT_H

#include <Arduino.h>
#include <ESP32Servo.h>

// ── Pins ────────────────────────────────────────────────────
#define SERVO_PIN      2    // PWM — parachute servo
#define POWER_CUT_PIN  4    // MOSFET gate — sensor rail
                            // HIGH = ON (normal), LOW = OFF (survival)

// ── Thresholds ───────────────────────────────────────────────
// Launch detection
#define LAUNCH_AZ_THRESHOLD    -9.0f   // avg_az (m/s²) — rocket boosting up
// Parachute deploy
#define EJECT_ANGLE_THRESHOLD   8.84f  // sqrt(ax²+ay²) ≈ 60° tilt
#define FREEFALL_THRESHOLD      2.0f   // total_a < 2 m/s² → apogee
#define NORMAL_EJECT_DELAY_MS   2000   // ms after launch before deploy allowed
#define EMERGENCY_EJECT_MS     10000   // ms — hard failsafe deploy
// Landing detection
#define LAND_ALT_DELTA_M        0.05f  // m per 100 ms loop (≈ 0.5 m/s)
#define LAND_ACCEL_LO           8.0f   // m/s²
#define LAND_ACCEL_HI          12.0f   // m/s²
#define LAND_CONFIRM_MS         3000   // ms both conditions must hold
#define LAND_FAILSAFE_MS      120000   // ms after deploy → force power cut
// Moving average window (each sample = LOOP_INTERVAL ms)
#define WINDOW_SIZE            10

// ── Flight phases ────────────────────────────────────────────
enum FlightPhase {
    PHASE_WAITING  = 0,
    PHASE_BOOST    = 1,
    PHASE_DEPLOYED = 2,
    PHASE_LANDED   = 3
};

// ── Public state (read from main / telemetry) ────────────────
extern FlightPhase flight_phase;
extern float       avg_ax, avg_ay, avg_az;   // moving-average accel (m/s²)
extern float       a_xandy;                  // sqrt(avg_ax²+avg_ay²)
extern float       total_a;                  // sqrt(avg_ax²+avg_ay²+avg_az²)

// ── API ──────────────────────────────────────────────────────
void flightInit();

// Call every loop with raw accel (m/s²) and BMP altitude (m).
// Returns true if phase just changed to LANDED (power cut fired).
bool flightUpdate(float ax, float ay, float az, float altitude);

// Returns string name of current phase (for telemetry STATE field)
const char* flightPhaseName();

#endif