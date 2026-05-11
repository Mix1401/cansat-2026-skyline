// ArduCAM library: open memorysaver.h → uncomment #define OV2640_MINI_2MP_PLUS

#define OV2640_MINI_2MP_PLUS
#include <heltec_unofficial.h>
#include <Wire.h>
#include <SPI.h>
#include <ArduCAM.h>

// ── SPI pins (HSPI — separate from LoRa SPI on 8-11) ─────────────────────────
#define CAM_CS    4
#define CAM_SCK   7
#define CAM_MOSI  5
#define CAM_MISO  6
// SCCB shares Wire1 (SDA=41, SCL=42) — OV2640 addr 0x30, no conflict with sensors

SPIClass camSPI(HSPI);
ArduCAM myCAM(OV2640, CAM_CS, &camSPI, &Wire1);

bool captureJPEG(uint8_t** outBuf, uint32_t* outLen) {
  myCAM.flush_fifo();
  myCAM.clear_fifo_flag();
  myCAM.start_capture();

  unsigned long t = millis();
  while (!myCAM.get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK)) {
    if (millis() - t > 3000) return false;
    delay(1);
  }

  uint32_t len = myCAM.read_fifo_length();
  if (len == 0 || len > 65535) return false;

  uint8_t* buf = (uint8_t*)malloc(len);
  if (!buf) return false;

  myCAM.CS_LOW();
  myCAM.set_fifo_burst();
  for (uint32_t i = 0; i < len; i++) buf[i] = camSPI.transfer(0x00);
  myCAM.CS_HIGH();

  *outBuf = buf;
  *outLen = len;
  return true;
}

void setup() {
  heltec_setup();
  Serial.begin(115200);
  delay(500);

  Wire1.begin(41, 42, 400000);

  camSPI.begin(CAM_SCK, CAM_MISO, CAM_MOSI, CAM_CS);
  pinMode(CAM_CS, OUTPUT);
  digitalWrite(CAM_CS, HIGH);
  delay(100);

  myCAM.write_reg(0x07, 0x80); delay(100);
  myCAM.write_reg(0x07, 0x00); delay(100);

  myCAM.write_reg(ARDUCHIP_TEST1, 0x55);
  if (myCAM.read_reg(ARDUCHIP_TEST1) != 0x55) {
    Serial.println("[ERROR] ArduCAM SPI link failed");
    display.clear();
    display.setFont(ArialMT_Plain_10);
    display.drawString(0, 0, "CAM SPI ERROR");
    display.display();
    while (true) delay(1000);
  }

  uint8_t vid, pid;
  myCAM.wrSensorReg8_8(0xff, 0x01);
  myCAM.rdSensorReg8_8(OV2640_CHIPID_HIGH, &vid);
  myCAM.rdSensorReg8_8(OV2640_CHIPID_LOW,  &pid);
  if (vid != 0x26 || (pid != 0x41 && pid != 0x42)) {
    Serial.printf("[ERROR] OV2640 not found: VID=0x%02X PID=0x%02X\n", vid, pid);
    display.clear();
    display.setFont(ArialMT_Plain_10);
    display.drawString(0, 0, "CAM I2C ERROR");
    display.drawString(0, 14, "VID:" + String(vid, HEX) + " PID:" + String(pid, HEX));
    display.display();
    while (true) delay(1000);
  }

  myCAM.set_format(JPEG);
  myCAM.InitCAM();
  myCAM.OV2640_set_JPEG_size(OV2640_160x120);
  delay(100);

  Serial.println("[OK] ArduCAM OV2640 ready (160x120 JPEG)");
  display.clear();
  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 0, "ArduCAM OK");
  display.drawString(0, 14, "160x120 JPEG");
  display.display();
}

void loop() {
  heltec_loop();

  uint8_t* buf = nullptr;
  uint32_t len = 0;

  heltec_led(50);
  bool ok = captureJPEG(&buf, &len);
  heltec_led(0);

  if (ok) {
    Serial.printf("[CAM] 160x120  %lu bytes  ~%lu pkts\n", len, (len + 199) / 200);
    display.clear();
    display.setFont(ArialMT_Plain_10);
    display.drawString(0, 0,  "ArduCAM OK");
    display.drawString(0, 14, "160x120 JPEG");
    display.drawString(0, 28, "Size: " + String(len) + " B");
    display.drawString(0, 42, "Pkts: ~" + String((len + 199) / 200));
    display.display();
    free(buf);
  } else {
    Serial.println("[ERROR] Capture failed");
    display.clear();
    display.setFont(ArialMT_Plain_10);
    display.drawString(0, 0, "Capture FAILED");
    display.display();
  }

  delay(5000);
}
