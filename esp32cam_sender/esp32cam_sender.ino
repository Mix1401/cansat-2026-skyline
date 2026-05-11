// Board: AI-Thinker ESP32-CAM
// TX (GPIO 1) → Heltec GPIO 3 (Serial1 RX) — disconnect when flashing

#include "esp_camera.h"
#include "FS.h"
#include "SD_MMC.h"

#define PWDN_GPIO_NUM   32
#define RESET_GPIO_NUM  -1
#define XCLK_GPIO_NUM    0
#define SIOD_GPIO_NUM   26
#define SIOC_GPIO_NUM   27
#define Y9_GPIO_NUM     35
#define Y8_GPIO_NUM     34
#define Y7_GPIO_NUM     39
#define Y6_GPIO_NUM     36
#define Y5_GPIO_NUM     21
#define Y4_GPIO_NUM     19
#define Y3_GPIO_NUM     18
#define Y2_GPIO_NUM      5
#define VSYNC_GPIO_NUM  25
#define HREF_GPIO_NUM   23
#define PCLK_GPIO_NUM   22

#define THUMB_W  32
#define THUMB_H  32

static int imgCount = 0;
static bool sdOK = false;

void initCamera(framesize_t fsize, pixformat_t pfmt) {
  esp_log_level_set("gpio", ESP_LOG_NONE);
  camera_config_t cfg = {};
  cfg.ledc_channel = LEDC_CHANNEL_0;
  cfg.ledc_timer   = LEDC_TIMER_0;
  cfg.pin_d0 = Y2_GPIO_NUM; cfg.pin_d1 = Y3_GPIO_NUM;
  cfg.pin_d2 = Y4_GPIO_NUM; cfg.pin_d3 = Y5_GPIO_NUM;
  cfg.pin_d4 = Y6_GPIO_NUM; cfg.pin_d5 = Y7_GPIO_NUM;
  cfg.pin_d6 = Y8_GPIO_NUM; cfg.pin_d7 = Y9_GPIO_NUM;
  cfg.pin_xclk     = XCLK_GPIO_NUM;
  cfg.pin_pclk     = PCLK_GPIO_NUM;
  cfg.pin_vsync    = VSYNC_GPIO_NUM;
  cfg.pin_href     = HREF_GPIO_NUM;
  cfg.pin_sccb_sda = SIOD_GPIO_NUM;
  cfg.pin_sccb_scl = SIOC_GPIO_NUM;
  cfg.pin_pwdn     = PWDN_GPIO_NUM;
  cfg.pin_reset    = RESET_GPIO_NUM;
  cfg.xclk_freq_hz = 20000000;
  cfg.pixel_format = pfmt;
  cfg.frame_size   = fsize;
  cfg.jpeg_quality = 10;
  cfg.fb_count     = 1;
  cfg.fb_location  = CAMERA_FB_IN_PSRAM;
  cfg.grab_mode    = CAMERA_GRAB_WHEN_EMPTY;

  if (esp_camera_init(&cfg) != ESP_OK) {
    Serial.println("[ERROR] Camera init failed");
    while (true) delay(1000);
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);

  initCamera(FRAMESIZE_VGA, PIXFORMAT_JPEG);

  sdOK = SD_MMC.begin("/sdcard", true);
  if (!sdOK) Serial.println("[WARN] SD not found");

  Serial.println("[OK] ESP32-CAM ready");
}

void loop() {

  // ── Step 1: VGA JPEG → SD card ───────────────────────────────────────────
  camera_fb_t* fb = esp_camera_fb_get();
  if (fb) {
    if (sdOK) {
      char path[32];
      snprintf(path, sizeof(path), "/img_%04d.jpg", imgCount);
      File f = SD_MMC.open(path, FILE_WRITE);
      if (f) { f.write(fb->buf, fb->len); f.close(); }
      Serial.printf("[SD] Saved %s (%d B)\n", path, fb->len);
    }
    imgCount++;
    esp_camera_fb_return(fb);
  }

  // ── Step 2: Re-init camera for 96x96 grayscale thumbnail ─────────────────
  esp_camera_deinit();
  initCamera(FRAMESIZE_96X96, PIXFORMAT_GRAYSCALE);
  delay(300);

  for (int i = 0; i < 3; i++) {
    camera_fb_t* flush = esp_camera_fb_get();
    if (flush) esp_camera_fb_return(flush);
  }

  fb = esp_camera_fb_get();

  if (fb && fb->format == PIXFORMAT_GRAYSCALE
         && fb->width  == 96
         && fb->height == 96) {
    uint8_t thumb[THUMB_W * THUMB_H];
    for (int ty = 0; ty < THUMB_H; ty++) {
      for (int tx = 0; tx < THUMB_W; tx++) {
        uint32_t sum = 0;
        for (int dy = 0; dy < 3; dy++)
          for (int dx = 0; dx < 3; dx++)
            sum += fb->buf[(ty * 3 + dy) * 96 + (tx * 3 + dx)];
        thumb[ty * THUMB_W + tx] = (uint8_t)(sum / 9);
      }
    }
    esp_camera_fb_return(fb);

    // ── Step 3: Send via Serial ─────────────────────────────────────────────
    uint16_t size = THUMB_W * THUMB_H;
    Serial.write(0xFF);
    Serial.write(0xAA);
    Serial.write((uint8_t)(size >> 8));
    Serial.write((uint8_t)(size & 0xFF));
    Serial.write(thumb, size);
    Serial.printf("\n[CAM] Sent thumb %dx%d (%d B)\n", THUMB_W, THUMB_H, size);
  } else {
    if (fb) {
      Serial.printf("[ERROR] Thumb bad: fmt=%d w=%d h=%d len=%d\n",
                    fb->format, fb->width, fb->height, fb->len);
      esp_camera_fb_return(fb);
    } else {
      Serial.println("[ERROR] Thumb capture returned NULL");
    }
  }

  esp_camera_deinit();
  initCamera(FRAMESIZE_VGA, PIXFORMAT_JPEG);
  delay(200);

  delay(10000);
}
