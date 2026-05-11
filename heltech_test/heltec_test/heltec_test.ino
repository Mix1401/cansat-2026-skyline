#include <heltec_unofficial.h>
#include <RadioLib.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <OLEDDisplayFonts.h>
#include <TinyGPSPlus.h>

// ── BMP280 + MPU6050 on Wire1 (SDA=41, SCL=42) ───────────────────────────────
// BMP280 addr: 0x76  |  MPU6050 addr: 0x68 (AD0 low)
Adafruit_BMP280 bmp(&Wire1);

const int MPU_ADDR = 0x68;
float AcX, AcY, AcZ, GyX, GyY, GyZ;

// ── NEO-M8N GPS on Serial2 ────────────────────────────────────────────────────
TinyGPSPlus gps;
#define GPS_BAUD     9600
#define GPS_RX_PIN   45   // connect to GPS TX
#define GPS_TX_PIN   46   // connect to GPS RX (optional)

// ── Transmission counter ─────────────────────────────────────────────────────
RTC_DATA_ATTR int txCount = 0;

// ── LoRa settings ────────────────────────────────────────────────────────────
#define LORA_FREQUENCY   923.0   // MHz — AS923 for Thailand
#define LORA_BANDWIDTH   125.0   // kHz
#define LORA_SPREADING   7       // SF7
#define LORA_CODING_RATE 5       // 4/5
#define LORA_SYNC_WORD   0x12    // private network
#define LORA_TX_POWER    14      // dBm

// ── Timing ───────────────────────────────────────────────────────────────────
#define TX_INTERVAL_MS   2000    // send every 2 s

// ── OLED helper ──────────────────────────────────────────────────────────────
void displayStatus(const char* line1, const char* line2 = "", const char* line3 = "") {
  display.clear();
  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 0,  line1);
  display.drawString(0, 16, line2);
  display.drawString(0, 32, line3);
  display.display();
}

// ── Feed GPS from Serial2 ────────────────────────────────────────────────────
void feedGPS() {
  while (Serial2.available()) {
    gps.encode(Serial2.read());
  }
}

// ─────────────────────────────────────────────────────────────────────────────
void setup() {
  heltec_setup();          // inits OLED, LED, power rail — takes Wire for OLED
  Serial.begin(115200);
  delay(500);

  // Wire1 for BMP280 + MPU6050
  Wire1.begin(41, 42, 400000);  // SDA=41, SCL=42, 400kHz
  delay(500);

  // GPS UART
  Serial2.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.println("[OK] GPS serial started");

  Serial.println("\n=== Heltec V3 LoRa Sender (BMP280 + MPU6050 + GPS) ===");

  // ── BMP280 ────────────────────────────────────────────────────────────────
  if (!bmp.begin(0x76)) {
    Serial.println("[ERROR] BMP280 not found at 0x76, trying 0x77...");
    if (!bmp.begin(0x77)) {
      Serial.println("[ERROR] BMP280 not found! Check wiring.");
      displayStatus("BMP280 ERROR", "Check wiring");
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
  uint8_t wakeErr = 255;
  Wire1.beginTransmission(MPU_ADDR);
  Wire1.write(0x6B);
  Wire1.write(0x00);  // clear sleep bit
  wakeErr = Wire1.endTransmission(true);
  Serial.printf("[MPU] Wake transmission: %d (0=OK)\n", wakeErr);
  delay(100);

  // WHO_AM_I — stop-start instead of repeated-start (ESP32 I2C quirk)
  Wire1.beginTransmission(MPU_ADDR);
  Wire1.write(0x75);
  Wire1.endTransmission(true);
  Wire1.requestFrom((uint8_t)MPU_ADDR, (uint8_t)1, (uint8_t)true);
  uint8_t whoAmI = Wire1.read();
  Serial.printf("[MPU] WHO_AM_I = 0x%02X\n", whoAmI);
  if (whoAmI == 0xFF) {
    Serial.println("[ERROR] MPU6050 not found! Check wiring.");
    displayStatus("MPU6050 ERROR", "Check wiring");
    while (true) delay(1000);
  }
  Serial.println("[OK] MPU6050 ready");

  // ── SX1262 LoRa ───────────────────────────────────────────────────────────
  int state = radio.begin(
    LORA_FREQUENCY,
    LORA_BANDWIDTH,
    LORA_SPREADING,
    LORA_CODING_RATE,
    LORA_SYNC_WORD,
    LORA_TX_POWER
  );
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("[ERROR] LoRa init failed, code %d\n", state);
    displayStatus("LoRa INIT FAIL", String(state).c_str());
    while (true) delay(1000);
  }
  radio.setCRC(true);
  Serial.println("[OK] LoRa ready");
  displayStatus("LoRa Ready", "Waiting GPS...");
  delay(1000);
}

// ─────────────────────────────────────────────────────────────────────────────
void loop() {
  heltec_loop();
  feedGPS();

  // ── Read BMP280 ───────────────────────────────────────────────────────────
  float tempC    = bmp.readTemperature();
  float pressPa  = bmp.readPressure();
  float pressHPa = pressPa / 100.0f;
  float altBaro  = bmp.readAltitude(1013.25f);

  Serial.printf("[BMP] T=%.2fC  P=%.2fhPa  Alt=%.1fm\n",
                tempC, pressHPa, altBaro);

  // ── Read MPU6050 ──────────────────────────────────────────────────────────
  Wire1.beginTransmission(MPU_ADDR);
  Wire1.write(0x3B);
  Wire1.endTransmission(true);
  Wire1.requestFrom((uint8_t)MPU_ADDR, (uint8_t)12, (uint8_t)true);
  AcX = (int16_t)(Wire1.read() << 8 | Wire1.read());
  AcY = (int16_t)(Wire1.read() << 8 | Wire1.read());
  AcZ = (int16_t)(Wire1.read() << 8 | Wire1.read());
  GyX = (int16_t)(Wire1.read() << 8 | Wire1.read());
  GyY = (int16_t)(Wire1.read() << 8 | Wire1.read());
  GyZ = (int16_t)(Wire1.read() << 8 | Wire1.read());

  // ±2g default range → 16384 LSB/g; ±250°/s → 131 LSB/(°/s)
  float ax = AcX / 16384.0f * 9.81f;
  float ay = AcY / 16384.0f * 9.81f;
  float az = AcZ / 16384.0f * 9.81f;
  float gx = GyX / 131.0f * (PI / 180.0f);
  float gy = GyY / 131.0f * (PI / 180.0f);
  float gz = GyZ / 131.0f * (PI / 180.0f);

  Serial.printf("[IMU] Accel=%.2f,%.2f,%.2f m/s2  Gyro=%.2f,%.2f,%.2f rad/s\n",
                ax, ay, az, gx, gy, gz);

  // ── Read GPS ──────────────────────────────────────────────────────────────
  bool    gpsFix = gps.location.isValid() && gps.location.age() < 2000;
  double  lat    = gpsFix ? gps.location.lat() : 0.0;
  double  lon    = gpsFix ? gps.location.lng() : 0.0;
  float   altGPS = gps.altitude.isValid()  ? gps.altitude.meters()  : 0.0f;
  uint8_t sats   = gps.satellites.isValid() ? gps.satellites.value() : 0;

  Serial.printf("[GPS] Fix=%s  Lat=%.6f  Lon=%.6f  AltGPS=%.1fm  Sats=%d\n",
                gpsFix ? "YES" : "NO", lat, lon, altGPS, sats);

  // ── Build payload ─────────────────────────────────────────────────────────
  char payload[192];
  snprintf(payload, sizeof(payload),
           "TX:%d,T:%.2f,P:%.2f,AB:%.1f,LAT:%.6f,LON:%.6f,AG:%.1f,SAT:%d,FIX:%d"
           ",AX:%.2f,AY:%.2f,AZ:%.2f,GX:%.2f,GY:%.2f,GZ:%.2f",
           ++txCount, tempC, pressHPa, altBaro,
           lat, lon, altGPS, sats, gpsFix ? 1 : 0,
           ax, ay, az, gx, gy, gz);

  Serial.printf("[TX] Sending: %s\n", payload);

  // ── Update OLED ───────────────────────────────────────────────────────────
  char oledLine1[32], oledLine2[32], oledLine3[32];
  snprintf(oledLine1, sizeof(oledLine1), "TX#%d %s %dsat",
           txCount, gpsFix ? "FIX" : "---", sats);
  snprintf(oledLine2, sizeof(oledLine2), "T:%.1fC P:%.0fhPa", tempC, pressHPa);
  snprintf(oledLine3, sizeof(oledLine3), "A:%.1f,%.1f,%.1f", ax, ay, az);
  displayStatus(oledLine1, oledLine2, oledLine3);

  // ── Transmit ─────────────────────────────────────────────────────────────
  heltec_led(50);
  int state = radio.transmit(payload);
  heltec_led(0);

  if (state == RADIOLIB_ERR_NONE) {
    Serial.printf("[TX] OK — RSSI=%.1f dBm  SNR=%.1f dB\n",
                  radio.getRSSI(), radio.getSNR());
  } else {
    Serial.printf("[TX] FAILED, code %d\n", state);
    displayStatus(oledLine1, oledLine2, "TX FAILED");
  }

  // Feed GPS during TX interval (non-blocking)
  unsigned long txEnd = millis() + TX_INTERVAL_MS;
  while (millis() < txEnd) {
    heltec_loop();
    feedGPS();
    delay(10);
  }
}
