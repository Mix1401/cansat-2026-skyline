#include <Wire.h>

// Heltec WiFi LoRa V3 — scan both I2C buses
// Wire  (OLED bus): SDA=17, SCL=18  — built-in
// Wire1 (sensor bus): SDA=41, SCL=42 — BMP280, MPU6050

void scanBus(TwoWire &bus, const char* busName) {
  Serial.printf("\n--- %s ---\n", busName);
  uint8_t found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    bus.beginTransmission(addr);
    uint8_t err = bus.endTransmission();
    if (err == 0) {
      Serial.printf("  Found: 0x%02X", addr);
      // known devices hint
      if (addr == 0x3C || addr == 0x3D) Serial.print(" (OLED SSD1306)");
      if (addr == 0x68 || addr == 0x69) Serial.print(" (MPU6050 or AMG8833)");
      if (addr == 0x76 || addr == 0x77) Serial.print(" (BMP280/BME280)");
      Serial.println();
      found++;
    }
  }
  if (found == 0) Serial.println("  Nothing found.");
  Serial.printf("  Total: %d device(s)\n", found);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(17, 18);    // OLED bus
  Wire1.begin(41, 42);   // sensor bus
  delay(200);

  Serial.println("=== Heltec V3 I2C Scanner ===");
  scanBus(Wire,  "Wire  (SDA=17 SCL=18)");
  scanBus(Wire1, "Wire1 (SDA=41 SCL=42)");
  Serial.println("\nDone. Reset to scan again.");
}

void loop() {}
