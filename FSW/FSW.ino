// ─── Heltec ESP32 WiFi LoRa V3 — Flight Software ─────────────────────────────
// Sensors on Wire1 (SDA=41, SCL=42):
//   BMP280  @ 0x76  — temperature, pressure, altitude
//   MPU6050 @ 0x68  — accelerometer, gyroscope (raw Wire1)
//   INA219  @ 0x40  — bus voltage, current, power
//   AMG8833 @ 0x69  — 8×8 thermal array (ADR pin → 3.3V)
// GPS NEO-M8N on Serial2: RX=47 (← GPS TX), TX=48 (→ GPS RX)
// ESP-CAM  on Serial1:    TX=4  (→ ESP-CAM RX) — GPS time sync at boot
// ─────────────────────────────────────────────────────────────────────────────
#include <heltec_unofficial.h>
#include <RadioLib.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_AMG88xx.h>
#include <Adafruit_INA219.h>
#include <OLEDDisplayFonts.h>
#include <TinyGPSPlus.h>

// ── Wire1 sensors ────────────────────────────────────────────────────────────
#define I2C_SDA 41
#define I2C_SCL 42

Adafruit_BMP280 bmp(&Wire1);
Adafruit_AMG88xx amg;
Adafruit_INA219 ina219(0x40);

// MPU6050 raw Wire1
#define MPU_ADDR 0x68
float AcX, AcY, AcZ, GyX, GyY, GyZ;

// AMG8833
#define AMG_ADDR 0x69
float amgPixels[AMG88xx_PIXEL_ARRAY_SIZE];

// ── GPS ───────────────────────────────────────────────────────────────────────
TinyGPSPlus gps;
#define GPS_BAUD   9600
#define GPS_RX_PIN 47   // Heltec RX ← GPS TX
#define GPS_TX_PIN 48   // Heltec TX → GPS RX

// ── ESP-CAM sync (Serial1) ────────────────────────────────────────────────────
#define CAM_TX_PIN 4    // Heltec TX → ESP-CAM RX (GPIO 3 on ESP32-CAM)
#define CAM_RX_PIN 3    // unused — keep for Serial1 init
#define CAM_BAUD   115200
static bool camSynced = false;

// ── LoRa ─────────────────────────────────────────────────────────────────────
#define LORA_FREQUENCY   923.0   // AS923 — Thailand
#define LORA_BANDWIDTH   125.0
#define LORA_SPREADING   7
#define LORA_CODING_RATE 5
#define LORA_SYNC_WORD   0x12
#define LORA_TX_POWER    14

#define CYCLE_MS         1000   // total cycle period — telemetry + IR grid both at 1 Hz
#define AMG_TX_EVERY_N   1      // send thermal grid every N telemetry cycles
#define AMG_FRAG_SIZE    50     // bytes per LoRa fragment

RTC_DATA_ATTR int txCount  = 0;
RTC_DATA_ATTR int amgCycle = 0;

// ── OLED helper ───────────────────────────────────────────────────────────────
void displayStatus(const char* l1, const char* l2 = "", const char* l3 = "") {
  display.clear();
  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 0,  l1);
  display.drawString(0, 16, l2);
  display.drawString(0, 32, l3);
  display.display();
}

// ── GPS feed ──────────────────────────────────────────────────────────────────
void feedGPS() {
  while (Serial2.available()) gps.encode(Serial2.read());
}

// ── Unix timestamp from GPS (ms precision) ───────────────────────────────────
// Returns millis()-based fallback if GPS time not valid
uint32_t unixMs() {
  if (!gps.date.isValid() || !gps.time.isValid()) return millis();
  struct tm t = {};
  t.tm_year = gps.date.year() - 1900;
  t.tm_mon  = gps.date.month() - 1;
  t.tm_mday = gps.date.day();
  t.tm_hour = gps.time.hour();
  t.tm_min  = gps.time.minute();
  t.tm_sec  = gps.time.second();
  return (uint32_t)(mktime(&t)) * 1000UL + gps.time.centisecond() * 10UL;
}

// ── Send GPS epoch to ESP-CAM over Serial1 ───────────────────────────────────
// ESP-CAM listens for "SYNC:<unix_ms>\n" on its RX pin
// Retries until ACK "ACK\n" received or timeout
void syncESPCam() {
  Serial.println("[CAM] Sending time sync...");
  displayStatus("CAM sync...", "waiting ACK");

  uint32_t ts = unixMs();
  char msg[32];
  snprintf(msg, sizeof(msg), "SYNC:%lu\n", (unsigned long)ts);
  Serial1.print(msg);
  Serial.printf("[CAM] Sent: %s", msg);

  // wait up to 3s for ACK
  unsigned long t0 = millis();
  String ack = "";
  while (millis() - t0 < 3000) {
    while (Serial1.available()) ack += (char)Serial1.read();
    if (ack.indexOf("ACK") >= 0) {
      camSynced = true;
      Serial.println("[CAM] Sync ACK received");
      return;
    }
  }
  // no ACK — continue anyway, ESP-CAM may not implement ACK
  camSynced = true;
  Serial.println("[CAM] Sync sent (no ACK — continuing)");
}

// ── AMG8833 fragmented TX ─────────────────────────────────────────────────────
// Encoding: uint8 = clamp((pixel + 20) * 2, 0, 255)
// Decode:   temp_C = (val / 2.0) - 20   → 0.5°C res, range −20..107.5°C
// Packet:   ['A', seq, total, sizeH, sizeL, ts3, ts2, ts1, ts0, ...pixels...]
//           ts = unix_ms big-endian uint32 (only in seq=0 packet)
void sendAMGGrid(const float* pixels, uint32_t ts) {
  uint8_t encoded[AMG88xx_PIXEL_ARRAY_SIZE];
  for (int i = 0; i < AMG88xx_PIXEL_ARRAY_SIZE; i++) {
    int v = (int)((pixels[i] + 20.0f) * 2.0f + 0.5f);
    encoded[i] = (uint8_t)constrain(v, 0, 255);
  }

  const uint16_t totalBytes = AMG88xx_PIXEL_ARRAY_SIZE;  // 64
  const uint8_t  totalPkts  = (totalBytes + AMG_FRAG_SIZE - 1) / AMG_FRAG_SIZE;
  // seq=0: 9-byte header (5 base + 4 ts); seq>0: 5-byte header
  uint8_t pkt[AMG_FRAG_SIZE + 9];

  Serial.printf("[AMG] TX grid %d bytes in %d frags ts=%lu\n",
                totalBytes, totalPkts, (unsigned long)ts);

  for (uint8_t seq = 0; seq < totalPkts; seq++) {
    uint16_t offset   = seq * AMG_FRAG_SIZE;
    uint8_t  chunkLen = (uint8_t)min((int)AMG_FRAG_SIZE, (int)(totalBytes - offset));
    uint8_t  hdrLen   = (seq == 0) ? 9 : 5;

    pkt[0] = 'A';
    pkt[1] = seq;
    pkt[2] = totalPkts;
    pkt[3] = (uint8_t)(totalBytes >> 8);
    pkt[4] = (uint8_t)(totalBytes & 0xFF);
    if (seq == 0) {
      pkt[5] = (uint8_t)(ts >> 24);
      pkt[6] = (uint8_t)(ts >> 16);
      pkt[7] = (uint8_t)(ts >>  8);
      pkt[8] = (uint8_t)(ts & 0xFF);
    }
    memcpy(pkt + hdrLen, encoded + offset, chunkLen);

    heltec_led(30);
    int state = radio.transmit(pkt, chunkLen + hdrLen);
    heltec_led(0);

    if (state != RADIOLIB_ERR_NONE) {
      Serial.printf("[AMG] Frag %d/%d FAILED code %d\n", seq + 1, totalPkts, state);
    }
    delay(50);
  }
  Serial.printf("[AMG] Grid TX done\n");
}

// ─────────────────────────────────────────────────────────────────────────────
void setup() {
  heltec_setup();
  Serial.begin(115200);
  delay(300);

  Wire1.begin(I2C_SDA, I2C_SCL, 400000);
  delay(300);

  Serial2.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.println("[OK] GPS serial started (RX=47, TX=48)");

  Serial1.begin(CAM_BAUD, SERIAL_8N1, CAM_RX_PIN, CAM_TX_PIN);
  Serial.println("[OK] ESP-CAM serial ready (TX=4)");

  // ── BMP280 ────────────────────────────────────────────────────────────────
  if (!bmp.begin(0x76)) {
    if (!bmp.begin(0x77)) {
      Serial.println("[ERROR] BMP280 not found");
      displayStatus("BMP280 ERROR");
      while (true) delay(1000);
    }
  }
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_500);
  Serial.println("[OK] BMP280 ready");

  // ── MPU6050 ───────────────────────────────────────────────────────────────
  Wire1.beginTransmission(MPU_ADDR);
  Wire1.write(0x6B);
  Wire1.write(0x00);  // clear sleep bit
  Wire1.endTransmission(true);
  delay(100);

  Wire1.beginTransmission(MPU_ADDR);
  Wire1.write(0x75);  // WHO_AM_I
  Wire1.endTransmission(true);
  Wire1.requestFrom((uint8_t)MPU_ADDR, (uint8_t)1, (uint8_t)true);
  uint8_t whoAmI = Wire1.read();
  Serial.printf("[MPU] WHO_AM_I=0x%02X\n", whoAmI);
  if (whoAmI == 0xFF) {
    Serial.println("[ERROR] MPU6050 not found");
    displayStatus("MPU6050 ERROR");
    while (true) delay(1000);
  }
  Serial.println("[OK] MPU6050 ready");

  // ── INA219 ────────────────────────────────────────────────────────────────
  if (!ina219.begin(&Wire1)) {
    Serial.println("[ERROR] INA219 not found at 0x40");
    displayStatus("INA219 ERROR");
    while (true) delay(1000);
  }
  Serial.println("[OK] INA219 ready");

  // ── AMG8833 ───────────────────────────────────────────────────────────────
  if (!amg.begin(AMG_ADDR, &Wire1)) {
    Serial.println("[ERROR] AMG8833 not found at 0x69");
    displayStatus("AMG8833 ERROR");
    while (true) delay(1000);
  }
  Serial.println("[OK] AMG8833 ready");

  // ── LoRa SX1262 ───────────────────────────────────────────────────────────
  int state = radio.begin(LORA_FREQUENCY, LORA_BANDWIDTH, LORA_SPREADING,
                           LORA_CODING_RATE, LORA_SYNC_WORD, LORA_TX_POWER);
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("[ERROR] LoRa init failed, code %d\n", state);
    displayStatus("LoRa FAIL", String(state).c_str());
    while (true) delay(1000);
  }
  radio.setCRC(true);
  Serial.println("[OK] LoRa ready");

  // ── Wait for GPS fix → sync ESP-CAM time ──────────────────────────────────
  displayStatus("Waiting GPS...", "for CAM sync");
  unsigned long gpsTimeout = millis() + 120000;  // 2 min max wait
  while (millis() < gpsTimeout) {
    feedGPS();
    if (gps.date.isValid() && gps.time.isValid() && gps.time.age() < 1000) {
      syncESPCam();
      break;
    }
    if (millis() % 1000 < 50) {
      char buf[32];
      snprintf(buf, sizeof(buf), "Sats:%d age:%lus",
               gps.satellites.isValid() ? gps.satellites.value() : 0,
               (millis()) / 1000);
      displayStatus("Waiting GPS...", buf);
    }
    heltec_loop();
    delay(10);
  }
  if (!camSynced) {
    // GPS timeout — sync with millis() fallback
    syncESPCam();
  }

  displayStatus("FSW Ready", camSynced ? "CAM synced" : "CAM sync fail");
  delay(1000);
}

// ─────────────────────────────────────────────────────────────────────────────
void loop() {
  unsigned long cycleStart = millis();
  heltec_loop();
  feedGPS();

  // ── BMP280 ────────────────────────────────────────────────────────────────
  float tempC    = bmp.readTemperature();
  float pressHPa = bmp.readPressure() / 100.0f;
  float altBaro  = bmp.readAltitude(1013.25f);
  Serial.printf("[BMP] T=%.2fC  P=%.2fhPa  Alt=%.1fm\n", tempC, pressHPa, altBaro);

  // ── MPU6050 ───────────────────────────────────────────────────────────────
  Wire1.beginTransmission(MPU_ADDR);
  Wire1.write(0x3B);  // ACCEL_XOUT_H
  Wire1.endTransmission(true);
  Wire1.requestFrom((uint8_t)MPU_ADDR, (uint8_t)12, (uint8_t)true);
  AcX = (int16_t)(Wire1.read() << 8 | Wire1.read());
  AcY = (int16_t)(Wire1.read() << 8 | Wire1.read());
  AcZ = (int16_t)(Wire1.read() << 8 | Wire1.read());
  GyX = (int16_t)(Wire1.read() << 8 | Wire1.read());
  GyY = (int16_t)(Wire1.read() << 8 | Wire1.read());
  GyZ = (int16_t)(Wire1.read() << 8 | Wire1.read());

  // ±2g → 16384 LSB/g; ±250°/s → 131 LSB/(°/s)
  float ax = AcX / 16384.0f * 9.81f;
  float ay = AcY / 16384.0f * 9.81f;
  float az = AcZ / 16384.0f * 9.81f;
  float gx = GyX / 131.0f * (PI / 180.0f);
  float gy = GyY / 131.0f * (PI / 180.0f);
  float gz = GyZ / 131.0f * (PI / 180.0f);
  Serial.printf("[IMU] A=%.2f,%.2f,%.2f m/s²  G=%.2f,%.2f,%.2f rad/s\n",
                ax, ay, az, gx, gy, gz);

  // ── INA219 ────────────────────────────────────────────────────────────────
  float busV_V    = ina219.getBusVoltage_V();
  float current_mA = ina219.getCurrent_mA();
  float power_mW  = ina219.getPower_mW();
  Serial.printf("[INA] V=%.3fV  I=%.1fmA  P=%.1fmW\n", busV_V, current_mA, power_mW);

  // ── AMG8833 ───────────────────────────────────────────────────────────────
  amg.readPixels(amgPixels);
  float irMin = amgPixels[0], irMax = amgPixels[0], irSum = 0;
  for (int i = 0; i < AMG88xx_PIXEL_ARRAY_SIZE; i++) {
    if (amgPixels[i] < irMin) irMin = amgPixels[i];
    if (amgPixels[i] > irMax) irMax = amgPixels[i];
    irSum += amgPixels[i];
  }
  float irAvg = irSum / AMG88xx_PIXEL_ARRAY_SIZE;
  Serial.printf("[AMG] Min=%.1f  Max=%.1f  Avg=%.1f C\n", irMin, irMax, irAvg);

  // ── GPS ───────────────────────────────────────────────────────────────────
  bool    gpsFix = gps.location.isValid() && gps.location.age() < 2000;
  double  lat    = gpsFix ? gps.location.lat()    : 0.0;
  double  lon    = gpsFix ? gps.location.lng()    : 0.0;
  float   altGPS = gps.altitude.isValid()  ? gps.altitude.meters()  : 0.0f;
  uint8_t sats   = gps.satellites.isValid() ? gps.satellites.value() : 0;
  Serial.printf("[GPS] Fix=%s  %.6f,%.6f  Alt=%.1fm  Sats=%d\n",
                gpsFix ? "YES" : "NO", lat, lon, altGPS, sats);

  // ── Build & transmit payload ───────────────────────────────────────────────
  uint32_t ts = unixMs();
  char payload[240];
  snprintf(payload, sizeof(payload),
           "TS:%lu,TX:%d,T:%.2f,P:%.2f,AB:%.1f"
           ",LAT:%.6f,LON:%.6f,AG:%.1f,SAT:%d,FIX:%d"
           ",AX:%.2f,AY:%.2f,AZ:%.2f,GX:%.3f,GY:%.3f,GZ:%.3f"
           ",IR_MIN:%.1f,IR_MAX:%.1f,IR_AVG:%.1f"
           ",V:%.3f,I:%.1f,W:%.1f",
           (unsigned long)ts, ++txCount, tempC, pressHPa, altBaro,
           lat, lon, altGPS, sats, gpsFix ? 1 : 0,
           ax, ay, az, gx, gy, gz,
           irMin, irMax, irAvg,
           busV_V, current_mA, power_mW);

  Serial.printf("[TX] %s\n", payload);

  // OLED
  char l1[32], l2[32], l3[32];
  snprintf(l1, sizeof(l1), "TX#%d %s %dsat", txCount, gpsFix ? "FIX" : "---", sats);
  snprintf(l2, sizeof(l2), "T:%.1fC P:%.0fhPa", tempC, pressHPa);
  snprintf(l3, sizeof(l3), "%.2fV %.0fmA IR:%.0fC", busV_V, current_mA, irAvg);
  displayStatus(l1, l2, l3);

  heltec_led(50);
  int state = radio.transmit(payload);
  heltec_led(0);

  if (state == RADIOLIB_ERR_NONE) {
    Serial.printf("[TX] OK  RSSI=%.1fdBm  SNR=%.1fdB\n",
                  radio.getRSSI(), radio.getSNR());
  } else {
    Serial.printf("[TX] FAILED code %d\n", state);
    displayStatus(l1, l2, "TX FAILED");
  }

  // ── AMG8833 full grid TX (every AMG_TX_EVERY_N cycles) ───────────────────
  if (++amgCycle >= AMG_TX_EVERY_N) {
    amgCycle = 0;
    displayStatus(l1, "AMG grid TX...", "");
    sendAMGGrid(amgPixels, ts);
    displayStatus(l1, l2, l3);
  }

  // Wait out remainder of 1s cycle — keep feeding GPS
  long remaining = (long)(cycleStart + CYCLE_MS) - (long)millis();
  while (remaining > 0) {
    heltec_loop();
    feedGPS();
    delay(10);
    remaining = (long)(cycleStart + CYCLE_MS) - (long)millis();
  }
}
