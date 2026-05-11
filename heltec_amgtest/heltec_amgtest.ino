#include <heltec_unofficial.h>
#include <Wire.h>
#include <Adafruit_AMG88xx.h>

// AMG8833 on Wire1 (SDA=41, SCL=42), addr 0x69 (ADR pin high/floating)
#define AMG_ADDR 0x69

Adafruit_AMG88xx amg;
float pixels[AMG88xx_PIXEL_ARRAY_SIZE];  // 64 floats

void setup() {
  heltec_setup();
  Serial.begin(115200);
  delay(500);

  Wire1.begin(41, 42, 400000);
  delay(100);

  if (!amg.begin(AMG_ADDR, &Wire1)) {
    Serial.println("[ERROR] AMG8833 not found at 0x69!");
    display.clear();
    display.setFont(ArialMT_Plain_10);
    display.drawString(0, 0, "AMG8833 ERROR");
    display.drawString(0, 12, "Check wiring");
    display.display();
    while (true) delay(1000);
  }
  Serial.println("[OK] AMG8833 ready");
  delay(100);
}

void loop() {
  heltec_loop();

  amg.readPixels(pixels);

  float minT = pixels[0], maxT = pixels[0], sumT = 0;
  for (int i = 0; i < AMG88xx_PIXEL_ARRAY_SIZE; i++) {
    if (pixels[i] < minT) minT = pixels[i];
    if (pixels[i] > maxT) maxT = pixels[i];
    sumT += pixels[i];
  }
  float avgT = sumT / AMG88xx_PIXEL_ARRAY_SIZE;

  // Print 8x8 grid to serial
  Serial.println("[AMG] 8x8 grid (°C):");
  for (int row = 0; row < 8; row++) {
    for (int col = 0; col < 8; col++) {
      Serial.printf("%5.1f", pixels[row * 8 + col]);
    }
    Serial.println();
  }
  Serial.printf("[AMG] Min=%.1f  Max=%.1f  Avg=%.1f\n\n", minT, maxT, avgT);

  // OLED
  display.clear();
  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 0,  "AMG8833 Thermal");
  display.drawString(0, 14, "Min: " + String(minT, 1) + " C");
  display.drawString(0, 26, "Max: " + String(maxT, 1) + " C");
  display.drawString(0, 38, "Avg: " + String(avgT, 1) + " C");
  display.display();

  delay(500);
}
