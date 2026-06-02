// camAMG_single.ino — ESP32-CAM + AMG8833
// Board  : AI Thinker ESP32-CAM
// SD     : SD_MMC 1-bit  CLK=14  CMD=15  D0=2
// AMG    : I2C  SDA=13  SCL=4
// CAM+AMG: ถ่ายพร้อมกัน ทุก 1 วิ  640×480 JPEG (FRAMESIZE_VGA)

#include <Arduino.h>
#include <Wire.h>
#include "SD_MMC.h"
#include "esp_camera.h"
#include <Adafruit_AMG88xx.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "esp_task_wdt.h"

// micklnwza007
// winwinhacker007
// tivlnwza007
// kuyclaude i love you

// ── config ────────────────────────────────────────────
#define I2C_SDA          13
#define I2C_SCL           4
#define INTERVAL_MS    1000
#define CSV_FILENAME  "/log.csv"
#define SD_FREQ      4000000

// ── camera pins ───────────────────────────────────────
#define CAM_PIN_PWDN    32
#define CAM_PIN_RESET   -1
#define CAM_PIN_XCLK     0
#define CAM_PIN_SIOD    26
#define CAM_PIN_SIOC    27
#define CAM_PIN_D7      35
#define CAM_PIN_D6      34
#define CAM_PIN_D5      39
#define CAM_PIN_D4      36
#define CAM_PIN_D3      21
#define CAM_PIN_D2      19
#define CAM_PIN_D1      18
#define CAM_PIN_D0       5
#define CAM_PIN_VSYNC   25
#define CAM_PIN_HREF    23
#define CAM_PIN_PCLK    22

// ── globals ───────────────────────────────────────────
static Adafruit_AMG88xx _amg;
static uint32_t _imgCount = 0;
static bool     _amgOk    = false;

static float _pixels[64] = {};
static float _irMin = 0, _irMax = 0, _irAvg = 0;
static char  _camPath[32] = {};
static bool  _camOk = false;

static unsigned long _prev          = 0;
static unsigned long _prevAmgRetry  = 0;
#define AMG_RETRY_MS  5000

// ── SD ────────────────────────────────────────────────
static File sdOpen(const char *path, const char *mode, int retries = 5) {
    for (int i = 0; i < retries; i++) {
        File f = SD_MMC.open(path, mode);
        if (f) return f;
        delay(150);
    }
    return File();
}

// ── camera init ───────────────────────────────────────
bool initCam() {
    pinMode(CAM_PIN_PWDN, OUTPUT);
    digitalWrite(CAM_PIN_PWDN, HIGH); delay(100);
    digitalWrite(CAM_PIN_PWDN, LOW);  delay(300);

    camera_config_t cfg = {};
    cfg.ledc_channel  = LEDC_CHANNEL_0;
    cfg.ledc_timer    = LEDC_TIMER_0;
    cfg.pin_d0 = CAM_PIN_D0; cfg.pin_d1 = CAM_PIN_D1;
    cfg.pin_d2 = CAM_PIN_D2; cfg.pin_d3 = CAM_PIN_D3;
    cfg.pin_d4 = CAM_PIN_D4; cfg.pin_d5 = CAM_PIN_D5;
    cfg.pin_d6 = CAM_PIN_D6; cfg.pin_d7 = CAM_PIN_D7;
    cfg.pin_xclk     = CAM_PIN_XCLK;
    cfg.pin_pclk     = CAM_PIN_PCLK;
    cfg.pin_vsync    = CAM_PIN_VSYNC;
    cfg.pin_href     = CAM_PIN_HREF;
    cfg.pin_sscb_sda = CAM_PIN_SIOD;
    cfg.pin_sscb_scl = CAM_PIN_SIOC;
    cfg.pin_pwdn     = CAM_PIN_PWDN;
    cfg.pin_reset    = CAM_PIN_RESET;
    cfg.xclk_freq_hz = 20000000;
    cfg.pixel_format = PIXFORMAT_JPEG;
    cfg.frame_size   = FRAMESIZE_VGA;
    cfg.jpeg_quality = 12;
    cfg.fb_count     = 1;
    cfg.fb_location  = CAMERA_FB_IN_PSRAM;

    if (esp_camera_init(&cfg) != ESP_OK) return false;
    delay(1000);
    sensor_t *s = esp_camera_sensor_get();
    if (s) {
        s->set_whitebal(s, 1);
        s->set_exposure_ctrl(s, 1);
        s->set_brightness(s, 0);
    }
    return true;
}

// ── capture ───────────────────────────────────────────
bool captureJPEG(const char *path) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        esp_camera_deinit();
        delay(200);
        initCam();
        return false;
    }
    File f = sdOpen(path, FILE_WRITE);
    if (!f) {
        esp_camera_fb_return(fb);
        return false;
    }
    size_t want = fb->len;
    size_t wrote = f.write(fb->buf, fb->len);
    f.close();
    esp_camera_fb_return(fb);
    return wrote == want;
}

// ── AMG ───────────────────────────────────────────────
bool initAMG() {
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(400000);
    if (_amg.begin(0x69)) return true;
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(400000);
    if (_amg.begin(0x68)) return true;
    return false;
}
void readAMG() {
    _amg.readPixels(_pixels);
    float s = 0, lo = _pixels[0], hi = _pixels[0];
    for (int i = 0; i < 64; i++) {
        if (_pixels[i] < lo) lo = _pixels[i];
        if (_pixels[i] > hi) hi = _pixels[i];
        s += _pixels[i];
    }
    _irMin = lo; _irMax = hi; _irAvg = s / 64.0f;
}

// ── setup ─────────────────────────────────────────────
void setup() {
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
    esp_task_wdt_deinit();

    Serial.begin(115200);
    delay(3000);  // รอไฟ stable
    Serial.println("[boot] start");

    Wire.setPins(I2C_SDA, I2C_SCL);   // lock pins before Adafruit BusIO resets them
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(400000);

    bool sdOk = SD_MMC.begin("/sdcard", true, false, SD_FREQ);
    Serial.printf("[SD]  %s\n", sdOk ? "OK" : "FAIL — halt");
    if (!sdOk) { while (true) {} }

    bool camOk = initCam();
    Serial.printf("[CAM] init %s\n", camOk ? "OK" : "FAIL — halt");
    if (!camOk) { while (true) {} }

    bool camReady = false;
    for (int i = 0; i < 30 && !camReady; i++) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (fb) { esp_camera_fb_return(fb); camReady = true; }
        delay(100);
    }
    Serial.printf("[CAM] frame %s\n", camReady ? "OK" : "FAIL — halt");
    if (!camReady) { while (true) {} }

    Serial.print("[I2C] scan: ");
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) Serial.printf("0x%02X ", addr);
    }
    Serial.println();

    _amgOk = initAMG();
    Serial.printf("[AMG] %s\n", _amgOk ? "OK" : "FAIL — will retry");

    File f = sdOpen(CSV_FILENAME, FILE_APPEND);
    if (f) {
        if (f.size() == 0) {
            f.print("timestamp_ms,cam_file,ir_min,ir_max,ir_avg");
            for (int i = 0; i < 64; i++) f.printf(",p%02d", i);
            f.println();
        }
        f.close();
    }

    _prev = millis();
}

// ── CSV write ─────────────────────────────────────────
void writeCSV(unsigned long ts) {
    File f = sdOpen(CSV_FILENAME, FILE_APPEND);
    if (!f) return;
    f.printf("%lu,%s,%.2f,%.2f,%.2f",
             ts, _camOk ? _camPath : "FAIL",
             _irMin, _irMax, _irAvg);
    for (int i = 0; i < 64; i++) f.printf(",%.1f", _pixels[i]);
    f.println();
    f.close();
}

// ── loop ──────────────────────────────────────────────
void loop() {
    unsigned long now = millis();
    if (now - _prev < INTERVAL_MS) return;
    _prev = now;

    if (!_amgOk && now - _prevAmgRetry >= AMG_RETRY_MS) {
        _prevAmgRetry = now;
        _amgOk = initAMG();
    }
    if (_amgOk) readAMG();
    snprintf(_camPath, sizeof(_camPath), "/cam_%04lu.jpg", _imgCount++);
    _camOk = captureJPEG(_camPath);
    writeCSV(now);
}
