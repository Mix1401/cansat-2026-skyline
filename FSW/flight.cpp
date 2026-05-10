// flight.cpp
// CanSat Skyline — Flight state machine implementation

#include "flight.h"

// ── Public state ─────────────────────────────────────────────
FlightPhase flight_phase = PHASE_WAITING;
float avg_ax = 0, avg_ay = 0, avg_az = 0;
float a_xandy = 0, total_a = 0;

// ── Private ───────────────────────────────────────────────────
static Servo          _servo;
static float          _buf[WINDOW_SIZE][3];
static int            _bufIdx       = 0;
static unsigned long  _launchTime   = 0;
static unsigned long  _deployTime   = 0;
static float          _prevAlt      = 0.0f;
static bool           _landCandidate = false;
static unsigned long  _landCandTime = 0;

// ── Servo ────────────────────────────────────────────────────
static void servoNeutral() { _servo.write(90); }

static void servoDeploy() {
    _servo.write(0);
    delay(3000);
    _servo.write(90);
    Serial.println("[SERVO] Deployed");
}

// ── Moving average ───────────────────────────────────────────
static void pushAccel(float ax, float ay, float az) {
    _buf[_bufIdx][0] = ax;
    _buf[_bufIdx][1] = ay;
    _buf[_bufIdx][2] = az;
    _bufIdx = (_bufIdx + 1) % WINDOW_SIZE;

    float sx = 0, sy = 0, sz = 0;
    for (int i = 0; i < WINDOW_SIZE; i++) {
        sx += _buf[i][0];
        sy += _buf[i][1];
        sz += _buf[i][2];
    }
    avg_ax = sx / WINDOW_SIZE;
    avg_ay = sy / WINDOW_SIZE;
    avg_az = sz / WINDOW_SIZE;

    a_xandy = sqrtf(avg_ax * avg_ax + avg_ay * avg_ay);
    total_a  = sqrtf(avg_ax * avg_ax + avg_ay * avg_ay + avg_az * avg_az);
}

// ── Power cut ────────────────────────────────────────────────
static void cutSensorPower() {
    digitalWrite(POWER_CUT_PIN, LOW);
    Serial.println("[POWER] Sensor rail OFF — survival mode (Heltec + GPS only)");
}

// ── Init ─────────────────────────────────────────────────────
void flightInit() {
    // MOSFET pin — default HIGH (sensors ON)
    pinMode(POWER_CUT_PIN, OUTPUT);
    digitalWrite(POWER_CUT_PIN, HIGH);

    // Servo
    _servo.attach(SERVO_PIN);
    servoNeutral();

    // Clear moving-average buffer
    memset(_buf, 0, sizeof(_buf));

    Serial.println("[FLIGHT] State machine ready");
}

// ── Update (call every loop) ─────────────────────────────────
bool flightUpdate(float ax, float ay, float az, float altitude) {
    unsigned long now = millis();
    pushAccel(ax, ay, az);

    switch (flight_phase) {

        // ── WAITING ──────────────────────────────────────────
        case PHASE_WAITING:
            if (avg_az <= LAUNCH_AZ_THRESHOLD) {
                flight_phase = PHASE_BOOST;
                _launchTime  = now;
                _prevAlt     = altitude;
                Serial.println(">>> LAUNCH DETECTED <<<");
            }
            break;

        // ── BOOST ────────────────────────────────────────────
        case PHASE_BOOST:
            if ((now - _launchTime) > NORMAL_EJECT_DELAY_MS) {
                if (a_xandy >= EJECT_ANGLE_THRESHOLD || total_a < FREEFALL_THRESHOLD) {
                    Serial.printf("[DEPLOY] Normal — a_xy=%.2f total_a=%.2f\n",
                                  a_xandy, total_a);
                    flight_phase = PHASE_DEPLOYED;
                    _deployTime  = now;
                    _prevAlt     = altitude;
                    servoDeploy();
                    break;
                }
            }
            // Emergency failsafe deploy
            if ((now - _launchTime) > EMERGENCY_EJECT_MS) {
                Serial.printf("[DEPLOY] Emergency failsafe at t=%.2fs\n", now / 1000.0f);
                flight_phase = PHASE_DEPLOYED;
                _deployTime  = now;
                _prevAlt     = altitude;
                servoDeploy();
            }
            break;

        // ── DEPLOYED ─────────────────────────────────────────
        case PHASE_DEPLOYED: {
            float altDelta   = fabsf(altitude - _prevAlt);
            bool  altStable  = (altDelta  < LAND_ALT_DELTA_M);
            bool  accelStatic = (total_a  >= LAND_ACCEL_LO &&
                                 total_a  <= LAND_ACCEL_HI);

            if (altStable && accelStatic) {
                if (!_landCandidate) {
                    _landCandidate = true;
                    _landCandTime  = now;
                    Serial.println("[LAND?] Candidate start...");
                } else if ((now - _landCandTime) >= LAND_CONFIRM_MS) {
                    Serial.println(">>> LANDED CONFIRMED — cutting sensor power <<<");
                    flight_phase = PHASE_LANDED;
                    cutSensorPower();
                    return true;   // signal caller: power just cut
                }
            } else {
                if (_landCandidate) Serial.println("[LAND?] Candidate reset");
                _landCandidate = false;
            }

            // Hard failsafe
            if ((now - _deployTime) > LAND_FAILSAFE_MS) {
                Serial.println("[LAND] Failsafe timeout — cutting sensor power");
                flight_phase = PHASE_LANDED;
                cutSensorPower();
                return true;
            }

            _prevAlt = altitude;
            break;
        }

        case PHASE_LANDED:
            // Nothing to do — main handles beacon
            break;
    }

    if (flight_phase != PHASE_LANDED) _prevAlt = altitude;
    return false;
}

// ── Phase name ───────────────────────────────────────────────
const char* flightPhaseName() {
    switch (flight_phase) {
        case PHASE_WAITING:  return "READY";
        case PHASE_BOOST:    return "ASCENT";
        case PHASE_DEPLOYED: return "DESCENT";
        case PHASE_LANDED:   return "LANDED";
        default:             return "BOOT";
    }
}