// esp32camF.ino — CanSat Skyline ESP32-CAM
// Board  : AI Thinker ESP32-CAM
// Storage: SD Card (SDMMC built-in slot)
// Serial : รับ command จาก Heltec/Nano ผ่าน GPIO3(RX) GPIO1(TX)
//
// Commands (Serial 115200):
//   'P' = ถ่ายรูป 1 ใบ
//   'S' = status

#include "esp_camera.h"
#include "FS.h"
#include "SD_MMC.h"
#include "Arduino.h"

// ═══════════════════════════════════════════════════════════════
//  CAMERA PINS — AI Thinker ESP32-CAM
// ═══════════════════════════════════════════════════════════════
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// ═══════════════════════════════════════════════════════════════
//  CONFIG
// ═══════════════════════════════════════════════════════════════
#define AUTO_CAPTURE_MS  1000   // ถ่ายอัตโนมัติทุก 5 วินาที
#define PHOTO_FOLDER     "/photos"

// ═══════════════════════════════════════════════════════════════
//  STATE
// ═══════════════════════════════════════════════════════════════
static uint32_t photo_count = 0;
static bool     cam_ready   = false;
static bool     sd_ready    = false;

// ═══════════════════════════════════════════════════════════════
//  CAMERA INIT
// ═══════════════════════════════════════════════════════════════
bool initCamera() {
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer   = LEDC_TIMER_0;
    config.pin_d0       = Y2_GPIO_NUM;
    config.pin_d1       = Y3_GPIO_NUM;
    config.pin_d2       = Y4_GPIO_NUM;
    config.pin_d3       = Y5_GPIO_NUM;
    config.pin_d4       = Y6_GPIO_NUM;
    config.pin_d5       = Y7_GPIO_NUM;
    config.pin_d6       = Y8_GPIO_NUM;
    config.pin_d7       = Y9_GPIO_NUM;
    config.pin_xclk     = XCLK_GPIO_NUM;
    config.pin_pclk     = PCLK_GPIO_NUM;
    config.pin_vsync    = VSYNC_GPIO_NUM;
    config.pin_href     = HREF_GPIO_NUM;
    config.pin_sscb_sda = SIOD_GPIO_NUM;
    config.pin_sscb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn     = PWDN_GPIO_NUM;
    config.pin_reset    = RESET_GPIO_NUM;
    config.pixel_format = PIXFORMAT_JPEG;
    config.frame_size   = FRAMESIZE_SVGA;
    config.jpeg_quality = 8;
    config.fb_count     = 2;
    config.grab_mode    = CAMERA_GRAB_LATEST;
    config.fb_location  = CAMERA_FB_IN_PSRAM;
    config.xclk_freq_hz = 20000000;

    if (esp_camera_init(&config) != ESP_OK) return false;

    sensor_t *s = esp_camera_sensor_get();
    s->set_brightness(s, 1);
    s->set_contrast(s, 1);
    s->set_sharpness(s, 1);
    s->set_denoise(s, 1);
    s->set_whitebal(s, 1);
    s->set_awb_gain(s, 1);
    s->set_exposure_ctrl(s, 1);
    s->set_aec2(s, 1);

    return true;
}

// ═══════════════════════════════════════════════════════════════
//  SD INIT
// ═══════════════════════════════════════════════════════════════
bool initSD() {
    if (!SD_MMC.begin()) {
        Serial.println("[SD] Mount failed");
        return false;
    }
    if (SD_MMC.cardType() == CARD_NONE) {
        Serial.println("[SD] No card");
        return false;
    }
    if (!SD_MMC.exists(PHOTO_FOLDER)) SD_MMC.mkdir(PHOTO_FOLDER);
    if (!SD_MMC.exists("/sdcard"))   SD_MMC.mkdir("/sdcard");
    Serial.printf("[SD] Ready — %.0f MB free\n",
        (float)(SD_MMC.totalBytes() - SD_MMC.usedBytes()) / 1048576.0f);
    return true;
}

// ═══════════════════════════════════════════════════════════════
//  TAKE PHOTO
// ═══════════════════════════════════════════════════════════════
bool takePhoto() {
    if (!cam_ready) { Serial.println("[CAM] Not ready"); return false; }

    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) { Serial.println("[CAM] Capture failed"); return false; }

    unsigned long ts = millis();
    char path[48];
    snprintf(path, sizeof(path), "%s/%lu.jpg", PHOTO_FOLDER, ts);
    photo_count++;

    if (sd_ready) {
        File f = SD_MMC.open(path, FILE_WRITE);
        if (f) {
            f.write(fb->buf, fb->len);
            f.close();
            Serial.printf("[CAM] Saved %s  (%u bytes)\n", path, fb->len);
            logPhotoToCSV(ts, path);
        } else {
            Serial.println("[CAM] File open failed");
        }
    } else {
        Serial.printf("[CAM] Photo taken (SD not ready) %u bytes\n", fb->len);
    }

    esp_camera_fb_return(fb);
    return true;
}

// ═══════════════════════════════════════════════════════════════
//  STATUS
// ═══════════════════════════════════════════════════════════════
static void printStatus() {
    Serial.printf("[STATUS] cam=%s  sd=%s  photos=%lu\n",
        cam_ready ? "OK" : "FAIL",
        sd_ready  ? "OK" : "FAIL",
        photo_count);
}

// ═══════════════════════════════════════════════════════════════
//  CSV LOG TO SD
// ═══════════════════════════════════════════════════════════════
static const char* LOG_FILE = "/sdcard/cansat_log.csv";

static void writeHeader(const String &header) {
    if (!sd_ready) return;
    if (SD_MMC.exists(LOG_FILE)) return;  // ไม่เขียนทับถ้ามีอยู่แล้ว
    File f = SD_MMC.open(LOG_FILE, FILE_WRITE);
    if (f) { f.println(header.substring(7)); f.close(); }  // ตัด "HEADER," ออก
}

static void writeCSV(const String &line) {
    if (!sd_ready) return;
    File f = SD_MMC.open(LOG_FILE, FILE_APPEND);
    if (f) { f.println(line.substring(2)); f.close(); }    // ตัด "D," ออก
}

static void logPhotoToCSV(unsigned long ms, const char *path) {
    if (!sd_ready) return;
    File f = SD_MMC.open(LOG_FILE, FILE_APPEND);
    if (f) { f.printf("PHOTO,%lu,%s\n", ms, path); f.close(); }
}

// ═══════════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════════
void setup() {
    Serial.begin(115200);
    delay(1000);

    if (initCamera()) { cam_ready = true; Serial.println("[CAM] OK"); }
    else              { Serial.println("[CAM] FAIL"); }

    if (initSD())     { sd_ready = true; }

    Serial.println("=== ESP32-CAM ready ===");
}

// ═══════════════════════════════════════════════════════════════
//  LOOP
// ═══════════════════════════════════════════════════════════════
void loop() {
    // รับข้อมูลจาก Heltec ทีละบรรทัด
    if (Serial.available()) {
        String line = Serial.readStringUntil('\n');
        line.trim();
        if      (line == "P")            takePhoto();
        else if (line == "S")            printStatus();
        else if (line.startsWith("HEADER,")) writeHeader(line);
        else if (line.startsWith("D,"))      writeCSV(line);
    }

    // ถ่ายอัตโนมัติทุก AUTO_CAPTURE_MS
    static unsigned long lastCapture = 0;
    if (millis() - lastCapture >= AUTO_CAPTURE_MS) {
        lastCapture = millis();
        takePhoto();
    }
}