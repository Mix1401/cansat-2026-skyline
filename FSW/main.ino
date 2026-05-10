// main.ino
// CanSat Skyline FSW — Heltec ESP32 WiFi LoRa 32 V2 (923 MHz)
//
// Sensors : MPU6050 · BMP280 · INA219 · AMG8833 · u-blox NEO-M8N
// Radio   : SX1276 onboard — raw LoRa P2P (NOT LoRaWAN)
// Storage : Micro SD (SPI, CS=13)
// Actuator: Servo (pin 2) — parachute deploy
// Power   : MOSFET (pin 4) — sensor rail cut on landing
//
// I2C  : SDA=21, SCL=22
// UART2: GPS  RX=16, TX=17
// SPI  : SCK=5, MISO=19, MOSI=27  (LoRa + SD share bus)
//
// Build: PlatformIO, board heltec_wifi_lora_32_V2

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

#include "llora.h"
#include "mpu.h"
#include "bmp.h"
#include "gps.h"
#include "ina.h"
#include "amg.h"
#include "flight.h"
#include "sd.h"

// ── Config ───────────────────────────────────────────────────
#define TEAM_ID          "2025_TH_TEAM"
#define TX_INTERVAL_MS   100    // 10 Hz telemetry (same as flight loop)
#define BEACON_INTERVAL  5000   // ms — GPS beacon after landing

// ── Telemetry helpers ─────────────────────────────────────────
static uint32_t packet_count = 0;

// Full telemetry CSV (used during WAITING / BOOST / DEPLOYED)
static String buildTelemetry() {
    String s;
    s.reserve(240);

    s += TEAM_ID;                               s += ",";
    s += String(millis() / 1000.0f, 1);         s += ",";
    s += packet_count;                          s += ",";
    s += "F";                                   s += ",";   // mode = Flight
    s += flightPhaseName();                     s += ",";

    // BMP280
    s += is_BMP ? String(bmp_data.altitude,  1) : "0"; s += ",";
    s += is_BMP ? String(bmp_data.temp,      1) : "0"; s += ",";
    s += is_BMP ? String(bmp_data.pressure,  1) : "0"; s += ",";

    // INA219
    s += is_INA ? String(ina_data.bus_v,      2) : "0"; s += ",";
    s += is_INA ? String(ina_data.current_ma, 0) : "0"; s += ",";
    s += is_INA ? String(ina_data.batt_pct)      : "0"; s += ",";

    // GPS
    s += (is_GPS && gps_data.fix) ? String(gps_data.lat, 6) : "0"; s += ",";
    s += (is_GPS && gps_data.fix) ? String(gps_data.lng, 6) : "0"; s += ",";
    s += is_GPS ? String(gps_data.alt,  1) : "0"; s += ",";
    s += is_GPS ? String(gps_data.sats)    : "0"; s += ",";

    // MPU6050
    s += is_MPU ? String(mpu_data.ax, 2) : "0"; s += ",";
    s += is_MPU ? String(mpu_data.ay, 2) : "0"; s += ",";
    s += is_MPU ? String(mpu_data.az, 2) : "0"; s += ",";
    s += is_MPU ? String(mpu_data.gx, 1) : "0"; s += ",";
    s += is_MPU ? String(mpu_data.gy, 1) : "0"; s += ",";
    s += is_MPU ? String(mpu_data.gz, 1) : "0"; s += ",";
    s += is_MPU ? String(mpu_data.temp, 1) : "0"; s += ",";

    // AMG8833
    s += is_AMG ? String(amg_data.min_t, 1) : "0"; s += ",";
    s += is_AMG ? String(amg_data.max_t, 1) : "0"; s += ",";
    s += is_AMG ? String(amg_data.avg_t, 1) : "0"; s += ",";

    // Flight-computed values
    s += String(a_xandy,  2); s += ",";
    s += String(avg_az,   2); s += ",";
    s += String(total_a,  2); s += ",";

    // RSSI
    s += is_LoRa ? String(loraRSSI()) : "0";

    return s;
}

// Minimal beacon (only after landing — sensors OFF)
static void sendBeacon() {
    if (!is_LoRa) return;
    String b = String(TEAM_ID) + ",LANDED,";
    b += (is_GPS && gps_data.fix) ? String(gps_data.lat, 6) : "0"; b += ",";
    b += (is_GPS && gps_data.fix) ? String(gps_data.lng, 6) : "0"; b += ",";
    b += String(gps_data.alt, 1);
    sendLoRa(b);
    Serial.printf("[BEACON] lat=%.5f lon=%.5f alt=%.1fm sats=%d\n",
                  gps_data.lat, gps_data.lng, gps_data.alt, gps_data.sats);
}

// ── setup() ──────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    Wire.begin(21, 22);
    Wire.setClock(400000);   // 400 kHz — faster I2C for 10 Hz loop

    // LoRa
    if (initLoRa(Fq)) {
        is_LoRa = true;
        Serial.println("[LoRa] OK");
    } else {
        Serial.println("[LoRa] FAIL");
    }

    // MPU6050 — 16g range for rocket, 94 Hz BW
    if (initMPU(16, 500, 94)) {
        is_MPU = true;
        Serial.println("[MPU] OK");
    } else {
        Serial.println("[MPU] FAIL");
    }

    // BMP280 — calibrate ground baseline
    if (initBMP()) {
        is_BMP = true;
        Serial.print("[BMP] OK — calibrating... ");
        calibrateBMP(50);
        Serial.printf("P0=%.2f hPa\n", sea_level_hpa);
    } else {
        Serial.println("[BMP] FAIL");
    }

    // GPS
    initGPS();
    is_GPS = true;
    Serial.println("[GPS] OK");

    // INA219
    if (initINA()) {
        is_INA = true;
        Serial.println("[INA] OK");
    } else {
        Serial.println("[INA] FAIL (optional)");
    }

    // AMG8833
    if (initAMG()) {
        is_AMG = true;
        Serial.println("[AMG] OK");
    } else {
        Serial.println("[AMG] FAIL (optional)");
    }

    // SD
    sdInit();   // non-fatal if absent

    // Flight state machine + servo + MOSFET
    flightInit();

    Serial.println("\n=== CanSat Skyline ready — waiting for launch ===\n");
}

// ── loop() ───────────────────────────────────────────────────
void loop() {
    static unsigned long prevTime    = 0;
    static unsigned long lastBeacon  = 0;
    unsigned long now = millis();

    // GPS — pump as fast as possible
    if (is_GPS) readGPS();

    // ── LANDED phase — sensors OFF, beacon only ───────────────
    if (flight_phase == PHASE_LANDED) {
        if ((now - lastBeacon) >= BEACON_INTERVAL) {
            lastBeacon = now;
            sendBeacon();
        }
        return;
    }

    // ── Rate-limit sensor reads to TX_INTERVAL_MS ─────────────
    if (now - prevTime < TX_INTERVAL_MS) return;
    prevTime = now;

    // ── Read sensors ──────────────────────────────────────────
    if (is_MPU) readMPU();
    if (is_BMP) readBMP();
    if (is_INA) readINA();
    if (is_AMG) readAMG();

    // ── Flight state machine ──────────────────────────────────
    // flightUpdate returns true the moment power is cut (go to beacon)
    float altitude = is_BMP ? bmp_data.altitude : 0.0f;
    bool justLanded = flightUpdate(mpu_data.ax, mpu_data.ay, mpu_data.az,
                                   altitude);

    if (justLanded) {
        // Log final row before SD rail cuts
        sdLog(now, (int)flight_phase,
              mpu_data.ax, mpu_data.ay, mpu_data.az,
              avg_az, a_xandy, total_a,
              bmp_data.temp, bmp_data.pressure, bmp_data.altitude,
              amg_data.max_t, amg_data.min_t,
              ina_data.bus_v, ina_data.current_ma,
              gps_data.lat, gps_data.lng, gps_data.alt, gps_data.sats);
        lastBeacon = now;
        return;
    }

    // ── Telemetry + SD (all active phases) ────────────────────
    packet_count++;
    String pkt = buildTelemetry();
    Serial.println(pkt);
    if (is_LoRa) sendLoRa(pkt);

    sdLog(now, (int)flight_phase,
          mpu_data.ax, mpu_data.ay, mpu_data.az,
          avg_az, a_xandy, total_a,
          bmp_data.temp, bmp_data.pressure, bmp_data.altitude,
          amg_data.max_t, amg_data.min_t,
          ina_data.bus_v, ina_data.current_ma,
          gps_data.lat, gps_data.lng, gps_data.alt, gps_data.sats);
}
