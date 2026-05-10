/** esp.cpp
 * CanSat Skyline — ESP32 Main (PlatformIO)
 * Board  : Heltec WiFi LoRa 32 V2/V3 (ESP32)
 *
 * Modules:
 *   GY-521   (MPU6050)    — I2C  — Accelerometer / Gyro
 *   BMP280               — I2C  — Pressure / Altitude
 *   AMG8833              — I2C  — 8x8 Thermal Array
 *   INA219               — I2C  — Voltage / Current
 *   GY-NEO-M8N           — UART — GPS (Serial2)
 *   Micro SD             — SPI  — Data logging
 *   LoRa SX1276 / RFM95  — SPI  — Telemetry downlink (built-in on Heltec)
 *   ESP32-CAM            — UART — Image capture (Serial1 command)
 *   Servo                — PWM  — Parachute deployment (pin 2)
 *   MOSFET (IRLZ44N)     — GPIO — Power cut sensor rail (pin 4)
 *
 * I2C  : SDA=21, SCL=22
 * SPI  : MOSI=27, MISO=19, SCK=5  (Heltec LoRa default)
 * SD CS: pin 13
 * GPS  : RX2=16, TX2=17
 * CAM  : RX1=25, TX1=26
 *
 * MOSFET wiring (N-channel, e.g. IRLZ44N):
 *   Gate   → GPIO 4  (through 100Ω resistor)
 *   Source → GND
 *   Drain  → negative side of sensor power rail
 *   10kΩ pull-down between Gate and GND
 *   When GPIO HIGH → MOSFET ON  → sensors powered
 *   When GPIO LOW  → MOSFET OFF → sensors cut (only Heltec + GPS remain)
 */

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <math.h>

// --- Sensors ---
#include <Adafruit_MPU6050.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_AMG88xx.h>
#include <Adafruit_INA219.h>
#include <Adafruit_Sensor.h>

// --- GPS ---
#include <TinyGPSPlus.h>

// --- SD ---
#include <SD.h>

// --- LoRa ---
#include <LoRa.h>           // LoRa by Sandeep Mistry

// --- Servo ---
#include <ESP32Servo.h>

// ============================================================
//  Pin definitions
// ============================================================
#define I2C_SDA       21
#define I2C_SCL       22

#define LORA_SCK      5
#define LORA_MISO     19
#define LORA_MOSI     27
#define LORA_CS       18
#define LORA_RST      14
#define LORA_DIO0     26    // Heltec V2 (change to 35 for V3)

#define SD_CS         13

#define GPS_RX        16    // ESP32 RX2 <- GPS TX
#define GPS_TX        17    // ESP32 TX2 -> GPS RX

#define CAM_RX        25    // Serial1 RX <- CAM TX
#define CAM_TX        26    // Serial1 TX -> CAM RX

#define SERVO_PIN     2

// MOSFET gate pin — controls sensor power rail
// HIGH = sensors ON (normal), LOW = sensors OFF (landed/survival mode)
#define POWER_CUT_PIN 4

#define LORA_BAND     923E6  // 923 MHz (AS923)

// ============================================================
//  Flight constants
// ============================================================
const float LAUNCH_AZ_THRESHOLD   = -9.0f;   // m/s² on Z → rocket accelerating upward
const float EJECT_ANGLE_THRESHOLD =  8.84f;  // sqrt(ax²+ay²) ≈ 60° tilt
const float FREEFALL_THRESHOLD    =  2.0f;   // total |a| < 2 m/s² → apogee/freefall
const int   NORMAL_EJECT_DELAY    = 2000;    // ms after launch before deploy allowed
const int   EMERGENCY_EJECT_TIME  = 10000;   // ms — failsafe parachute deploy
const int   WINDOW_SIZE           = 10;      // moving-average samples (100 ms each → 1 s)
const unsigned long LOOP_INTERVAL = 100;     // ms

// --- Landing detection ---
// Altitude change per loop (100 ms) must be below this to count as "stable"
const float LAND_ALT_DELTA_M    = 0.05f;    // metres per 100 ms (≈ 0.5 m/s)
// Total acceleration must look like 1g (static) — between 8 and 12 m/s²
const float LAND_ACCEL_LO       = 8.0f;
const float LAND_ACCEL_HI       = 12.0f;
// How long both conditions must hold before declaring LANDED
const unsigned long LAND_CONFIRM_MS  = 3000;   // 3 seconds
// Hard failsafe: cut power this long after deploy regardless of detection
const unsigned long LAND_FAILSAFE_MS = 120000; // 2 minutes after deploy

// Beacon interval in LANDED state (ms)
const unsigned long BEACON_INTERVAL  = 5000;

// SD log filename
const char* LOG_FILE = "/cansat_log.csv";

// ============================================================
//  Objects
// ============================================================
Adafruit_MPU6050  mpu;
Adafruit_BMP280   bmp;
Adafruit_AMG88xx  amg;
Adafruit_INA219   ina219;
TinyGPSPlus       gps;
Servo             deployServo;

HardwareSerial    SerialGPS(2);
HardwareSerial    SerialCAM(1);

// ============================================================
//  State
// ============================================================
enum FlightPhase { WAITING, BOOST, COAST, DEPLOYED, LANDED };
FlightPhase phase = WAITING;

unsigned long launchTime         = 0;
unsigned long deployTime         = 0;

// Landing detection state
float         prevAltitude       = 0.0f;
bool          landCandidateActive = false;
unsigned long landCandidateTime  = 0;

// Beacon timer for LANDED phase
unsigned long lastBeaconTime     = 0;

// Moving-average buffers
float bufAcc[WINDOW_SIZE][3];   // [i][0]=ax [i][1]=ay [i][2]=az
int   bufIdx = 0;

// ============================================================
//  Helpers
// ============================================================
void servoNeutral() { deployServo.write(90); }

void servoDeploy() {
  deployServo.write(0);
  delay(3000);
  deployServo.write(90);
}

// Compute moving average over buffer
void movingAverage(float &ax, float &ay, float &az) {
  ax = ay = az = 0;
  for (int i = 0; i < WINDOW_SIZE; i++) {
    ax += bufAcc[i][0];
    ay += bufAcc[i][1];
    az += bufAcc[i][2];
  }
  ax /= WINDOW_SIZE;
  ay /= WINDOW_SIZE;
  az /= WINDOW_SIZE;
}

// Cut power to sensor rail — only Heltec + GPS remain
void cutSensorPower() {
  digitalWrite(POWER_CUT_PIN, LOW);
  Serial.println("[POWER] Sensor rail OFF — survival mode (Heltec + GPS only)");
}

// ============================================================
//  SD helpers
// ============================================================
bool sdReady = false;

void sdInit() {
  if (SD.begin(SD_CS)) {
    sdReady = true;
    if (!SD.exists(LOG_FILE)) {
      File f = SD.open(LOG_FILE, FILE_WRITE);
      if (f) {
        f.println("time_ms,phase,ax,ay,az,avg_az,a_xandy,total_a,"
                  "temp_c,pressure_pa,altitude_m,"
                  "thermal_max,thermal_min,"
                  "bus_v,current_ma,"
                  "gps_lat,gps_lon,gps_alt,gps_sats");
        f.close();
      }
    }
    Serial.println("[SD] Ready");
  } else {
    Serial.println("[SD] Mount failed — logging disabled");
  }
}

void sdLog(unsigned long t, float ax, float ay, float az,
           float avg_az, float a_xandy, float total_a,
           float temp, float pressure, float altitude,
           float thermalMax, float thermalMin,
           float busV, float currentMA,
           double lat, double lon, float gpsAlt, int sats) {
  if (!sdReady) return;
  File f = SD.open(LOG_FILE, FILE_APPEND);
  if (!f) return;
  f.printf("%lu,%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,"
           "%.2f,%.2f,%.2f,"
           "%.2f,%.2f,"
           "%.3f,%.2f,"
           "%.6f,%.6f,%.2f,%d\n",
           t, (int)phase, ax, ay, az, avg_az, a_xandy, total_a,
           temp, pressure, altitude,
           thermalMax, thermalMin,
           busV, currentMA,
           lat, lon, gpsAlt, sats);
  f.close();
}

// ============================================================
//  LoRa helpers
// ============================================================
bool loraReady = false;

void loraInit() {
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);
  if (LoRa.begin(LORA_BAND)) {
    LoRa.setSpreadingFactor(9);
    LoRa.setSignalBandwidth(125E3);
    LoRa.setCodingRate4(5);
    loraReady = true;
    Serial.println("[LoRa] Ready at 923 MHz");
  } else {
    Serial.println("[LoRa] Init failed");
  }
}

void loraSend(float a_xandy, float avg_az, float total_a,
              float altitude, float temp,
              float busV, double lat, double lon) {
  if (!loraReady) return;
  LoRa.beginPacket();
  LoRa.printf("%.1f,%.1f,%.1f,%.1f,%.2f,%.2f,%.5f,%.5f",
              a_xandy, avg_az, total_a, altitude, temp, busV, lat, lon);
  LoRa.endPacket();
}

// Minimal beacon for LANDED phase (GPS position only)
void loraBeacon(double lat, double lon, float altitude) {
  if (!loraReady) return;
  LoRa.beginPacket();
  LoRa.printf("LANDED,%.5f,%.5f,%.1f", lat, lon, altitude);
  LoRa.endPacket();
}

// ============================================================
//  AMG8833 helpers
// ============================================================
bool  amgReady = false;
float thermalPixels[64];

void amgRead(float &tMax, float &tMin) {
  tMax = -999; tMin = 999;
  if (!amgReady) { tMax = tMin = 0; return; }
  amg.readPixels(thermalPixels);
  for (int i = 0; i < 64; i++) {
    if (thermalPixels[i] > tMax) tMax = thermalPixels[i];
    if (thermalPixels[i] < tMin) tMin = thermalPixels[i];
  }
}

// ============================================================
//  setup()
// ============================================================
void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL);

  // MOSFET power-cut pin — default HIGH (sensors ON)
  pinMode(POWER_CUT_PIN, OUTPUT);
  digitalWrite(POWER_CUT_PIN, HIGH);

  // Servo
  deployServo.attach(SERVO_PIN);
  servoNeutral();

  // MPU6050
  if (mpu.begin()) {
    mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    Serial.println("[MPU6050] Ready");
  } else {
    Serial.println("[MPU6050] Not found — check wiring");
  }

  // BMP280
  if (bmp.begin(0x76)) {
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                    Adafruit_BMP280::SAMPLING_X2,
                    Adafruit_BMP280::SAMPLING_X16,
                    Adafruit_BMP280::FILTER_X16,
                    Adafruit_BMP280::STANDBY_MS_500);
    Serial.println("[BMP280] Ready");
  } else {
    Serial.println("[BMP280] Not found");
  }

  // AMG8833
  if (amg.begin()) {
    amgReady = true;
    Serial.println("[AMG8833] Ready");
  } else {
    Serial.println("[AMG8833] Not found (optional)");
  }

  // INA219
  if (ina219.begin()) {
    Serial.println("[INA219] Ready");
  } else {
    Serial.println("[INA219] Not found (optional)");
  }

  // GPS (Serial2)
  SerialGPS.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  Serial.println("[GPS] Serial2 started at 9600 baud");

  // ESP32-CAM (Serial1)
  SerialCAM.begin(115200, SERIAL_8N1, CAM_RX, CAM_TX);
  Serial.println("[CAM] Serial1 started at 115200 baud");
  delay(1000);

  // SD
  sdInit();

  // LoRa
  loraInit();

  // Init moving-average buffer
  memset(bufAcc, 0, sizeof(bufAcc));

  Serial.println("\n=== CanSat Skyline ready — waiting for launch ===\n");
}

// ============================================================
//  loop()
// ============================================================
void loop() {
  static unsigned long prevTime = 0;
  unsigned long now = millis();

  // Feed GPS parser continuously (not rate-limited)
  while (SerialGPS.available()) {
    gps.encode(SerialGPS.read());
  }

  if (now - prevTime < LOOP_INTERVAL) return;
  prevTime = now;

  // ---- GPS values (used in all phases) ----
  double lat   = gps.location.isValid() ? gps.location.lat()         : 0.0;
  double lon   = gps.location.isValid() ? gps.location.lng()         : 0.0;
  float gpsAlt = gps.altitude.isValid() ? (float)gps.altitude.meters(): 0.0f;
  int   sats   = gps.satellites.isValid() ? (int)gps.satellites.value(): 0;

  // ============================================================
  //  LANDED phase — survival mode, sensors are OFF
  //  Only Heltec + GPS active; send beacon every 5 s
  // ============================================================
  if (phase == LANDED) {
    if ((now - lastBeaconTime) >= BEACON_INTERVAL) {
      lastBeaconTime = now;
      // Use GPS altitude for beacon (BMP is off)
      loraBeacon(lat, lon, gpsAlt);
      Serial.printf("[LANDED] beacon lat=%.5f lon=%.5f alt=%.1fm sats=%d\n",
                    lat, lon, gpsAlt, sats);
    }
    return;  // skip all sensor reads — they are powered off
  }

  // ---- Read MPU6050 ----
  sensors_event_t a, g, tempEvt;
  mpu.getEvent(&a, &g, &tempEvt);
  float ax = a.acceleration.x;
  float ay = a.acceleration.y;
  float az = a.acceleration.z;

  // Update moving-average buffer
  bufAcc[bufIdx][0] = ax;
  bufAcc[bufIdx][1] = ay;
  bufAcc[bufIdx][2] = az;
  bufIdx = (bufIdx + 1) % WINDOW_SIZE;

  float avg_ax, avg_ay, avg_az;
  movingAverage(avg_ax, avg_ay, avg_az);

  float a_xandy = sqrtf(avg_ax * avg_ax + avg_ay * avg_ay);
  float total_a  = sqrtf(avg_ax * avg_ax + avg_ay * avg_ay + avg_az * avg_az);

  // ---- Read BMP280 ----
  float temperature = bmp.readTemperature();
  float pressure    = bmp.readPressure();
  float altitude    = bmp.readAltitude(1013.25f);

  // ---- Read AMG8833 ----
  float thermalMax, thermalMin;
  amgRead(thermalMax, thermalMin);

  // ---- Read INA219 ----
  float busVoltage = ina219.getBusVoltage_V();
  float currentMA  = ina219.getCurrent_mA();

  // ============================================================
  //  Flight state machine
  // ============================================================
  switch (phase) {

    // ----------------------------------------------------------
    case WAITING:
      Serial.printf("[WAIT] t=%.2fs | az=%.2f | a_xy=%.2f | alt=%.1fm | GPS %d sats\n",
                    now / 1000.0f, avg_az, a_xandy, altitude, sats);
      if (avg_az <= LAUNCH_AZ_THRESHOLD) {
        phase = BOOST;
        launchTime = now;
        prevAltitude = altitude;
        Serial.println(">>> LAUNCH DETECTED <<<");
      }
      break;

    // ----------------------------------------------------------
    case BOOST:
      Serial.printf("[BOOST] t=%.2fs | az=%.2f | a_xy=%.2f | total=%.2f | alt=%.1fm\n",
                    now / 1000.0f, avg_az, a_xandy, total_a, altitude);

      if ((now - launchTime) > NORMAL_EJECT_DELAY) {
        if (a_xandy >= EJECT_ANGLE_THRESHOLD || total_a < FREEFALL_THRESHOLD) {
          Serial.printf("[DEPLOY] Normal — a_xy=%.2f total_a=%.2f t=%.2fs\n",
                        a_xandy, total_a, now / 1000.0f);
          phase = DEPLOYED;
          deployTime = now;
          prevAltitude = altitude;
          servoDeploy();
          break;
        }
      }
      if ((now - launchTime) > EMERGENCY_EJECT_TIME) {
        Serial.printf("[DEPLOY] Emergency failsafe at t=%.2fs\n", now / 1000.0f);
        phase = DEPLOYED;
        deployTime = now;
        prevAltitude = altitude;
        servoDeploy();
      }
      break;

    // ----------------------------------------------------------
    case DEPLOYED: {
      // --- Landing detection ---
      float altDelta  = fabsf(altitude - prevAltitude); // m per 100 ms loop
      bool  altStable = (altDelta < LAND_ALT_DELTA_M);
      bool  accelStatic = (total_a >= LAND_ACCEL_LO && total_a <= LAND_ACCEL_HI);

      if (altStable && accelStatic) {
        if (!landCandidateActive) {
          landCandidateActive = true;
          landCandidateTime = now;
          Serial.println("[LAND?] Candidate start...");
        } else if ((now - landCandidateTime) >= LAND_CONFIRM_MS) {
          // Confirmed landing
          Serial.println(">>> LANDED CONFIRMED — cutting sensor power <<<");
          phase = LANDED;
          lastBeaconTime = now;
          cutSensorPower();
          // Log last position before SD powers off
          sdLog(now, ax, ay, az, avg_az, a_xandy, total_a,
                temperature, pressure, altitude,
                thermalMax, thermalMin, busVoltage, currentMA,
                lat, lon, gpsAlt, sats);
          return;
        }
      } else {
        if (landCandidateActive) {
          Serial.println("[LAND?] Candidate reset");
        }
        landCandidateActive = false;
      }

      // Hard failsafe — cut power even if landing was not detected cleanly
      if ((now - deployTime) > LAND_FAILSAFE_MS) {
        Serial.println("[LAND] Failsafe timeout — cutting sensor power");
        phase = LANDED;
        lastBeaconTime = now;
        cutSensorPower();
        return;
      }

      Serial.printf("[DEPLOYED] t=%.2fs | alt=%.1fm | Δalt=%.3f | total_a=%.2f | bat=%.2fV %.1fmA\n",
                    now / 1000.0f, altitude, altDelta, total_a, busVoltage, currentMA);
      break;
    }

    default:
      break;
  }

  // ============================================================
  //  Telemetry — send every cycle (WAITING / BOOST / DEPLOYED)
  // ============================================================
  loraSend(a_xandy, avg_az, total_a, altitude, temperature,
           busVoltage, lat, lon);

  // ============================================================
  //  SD logging
  // ============================================================
  sdLog(now, ax, ay, az, avg_az, a_xandy, total_a,
        temperature, pressure, altitude,
        thermalMax, thermalMin,
        busVoltage, currentMA,
        lat, lon, gpsAlt, sats);

  // Update previous altitude for next loop
  prevAltitude = altitude;
}