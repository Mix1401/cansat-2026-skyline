// Board: AI-Thinker ESP32-CAM
// Live MJPEG stream over WiFi (STA mode)
// Stream: http://<IP>/stream   Snapshot: http://<IP>/capture

#include "esp_camera.h"
#include "esp_http_server.h"
#include <WiFi.h>

// ── WiFi credentials ─────────────────────────────────────────────────────────
#define WIFI_SSID  "Witsanupong's A17"
#define WIFI_PASS  "tetae123"
#define WIFI_TIMEOUT_MS 10000

// ── AI-Thinker ESP32-CAM pins ─────────────────────────────────────────────────
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

#define PART_BOUNDARY "123456789000000000000987654321"
static const char* STREAM_CONTENT_TYPE =
    "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* STREAM_BOUNDARY =
    "\r\n--" PART_BOUNDARY "\r\n";
static const char* STREAM_PART =
    "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

httpd_handle_t stream_httpd = NULL;
httpd_handle_t camera_httpd = NULL;

// ─────────────────────────────────────────────────────────────────────────────

static void initCamera() {
  camera_config_t cfg = {};
  cfg.ledc_channel  = LEDC_CHANNEL_0;
  cfg.ledc_timer    = LEDC_TIMER_0;
  cfg.pin_d0        = Y2_GPIO_NUM;
  cfg.pin_d1        = Y3_GPIO_NUM;
  cfg.pin_d2        = Y4_GPIO_NUM;
  cfg.pin_d3        = Y5_GPIO_NUM;
  cfg.pin_d4        = Y6_GPIO_NUM;
  cfg.pin_d5        = Y7_GPIO_NUM;
  cfg.pin_d6        = Y8_GPIO_NUM;
  cfg.pin_d7        = Y9_GPIO_NUM;
  cfg.pin_xclk      = XCLK_GPIO_NUM;
  cfg.pin_pclk      = PCLK_GPIO_NUM;
  cfg.pin_vsync     = VSYNC_GPIO_NUM;
  cfg.pin_href      = HREF_GPIO_NUM;
  cfg.pin_sccb_sda  = SIOD_GPIO_NUM;
  cfg.pin_sccb_scl  = SIOC_GPIO_NUM;
  cfg.pin_pwdn      = PWDN_GPIO_NUM;
  cfg.pin_reset     = RESET_GPIO_NUM;
  cfg.xclk_freq_hz  = 20000000;
  cfg.pixel_format  = PIXFORMAT_JPEG;
  cfg.grab_mode     = CAMERA_GRAB_LATEST;

  if (psramFound()) {
    cfg.frame_size   = FRAMESIZE_VGA;   // 640×480
    cfg.jpeg_quality = 10;
    cfg.fb_count     = 2;
    cfg.fb_location  = CAMERA_FB_IN_PSRAM;
  } else {
    cfg.frame_size   = FRAMESIZE_CIF;   // 400×296 — no PSRAM fallback
    cfg.jpeg_quality = 12;
    cfg.fb_count     = 1;
    cfg.fb_location  = CAMERA_FB_IN_DRAM;
  }

  if (esp_camera_init(&cfg) != ESP_OK) {
    Serial.println("[ERROR] Camera init failed — halting");
    while (true) delay(1000);
  }

  sensor_t* s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_VGA);
  s->set_quality(s, 10);
  s->set_brightness(s, 0);
  s->set_saturation(s, 0);
  s->set_gainceiling(s, (gainceiling_t)2);
  s->set_whitebal(s, 1);
  s->set_awb_gain(s, 1);
  s->set_wb_mode(s, 0);
  s->set_exposure_ctrl(s, 1);
  s->set_aec2(s, 1);
}

// ── MJPEG stream handler ──────────────────────────────────────────────────────
static esp_err_t streamHandler(httpd_req_t* req) {
  camera_fb_t* fb = NULL;
  char partBuf[64];

  esp_err_t res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
  if (res != ESP_OK) return res;

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  while (true) {
    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("[WARN] Camera capture failed");
      res = ESP_FAIL;
      break;
    }

    size_t hdrLen = snprintf(partBuf, sizeof(partBuf), STREAM_PART, fb->len);

    res = httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));
    if (res == ESP_OK)
      res = httpd_resp_send_chunk(req, partBuf, hdrLen);
    if (res == ESP_OK)
      res = httpd_resp_send_chunk(req, (const char*)fb->buf, fb->len);

    esp_camera_fb_return(fb);

    if (res != ESP_OK) break;
  }
  return res;
}

// ── Single JPEG capture handler ───────────────────────────────────────────────
static esp_err_t captureHandler(httpd_req_t* req) {
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }

  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  esp_err_t res = httpd_resp_send(req, (const char*)fb->buf, fb->len);
  esp_camera_fb_return(fb);
  return res;
}

// ── Root page ─────────────────────────────────────────────────────────────────
static esp_err_t indexHandler(httpd_req_t* req) {
  httpd_resp_set_type(req, "text/html");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  const char* html =
    "<!DOCTYPE html><html><head><title>ESP32-CAM Live</title>"
    "<style>body{margin:0;background:#000;display:flex;flex-direction:column;"
    "align-items:center;justify-content:center;min-height:100vh;}"
    "img{max-width:100%;border:2px solid #0f0;}"
    "a{color:#0f0;margin-top:8px;font-family:monospace;}</style></head>"
    "<body>"
    "<img src='/stream' />"
    "<a href='/capture'>Snapshot</a>"
    "</body></html>";
  return httpd_resp_send(req, html, strlen(html));
}

// ─────────────────────────────────────────────────────────────────────────────

static void startServers() {
  httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
  cfg.stack_size = 8192;

  // Camera server — port 80 (index + capture)
  cfg.server_port = 80;
  cfg.ctrl_port   = 32768;
  if (httpd_start(&camera_httpd, &cfg) == ESP_OK) {
    httpd_uri_t index_uri = { "/",        HTTP_GET, indexHandler,   NULL };
    httpd_uri_t cap_uri   = { "/capture", HTTP_GET, captureHandler, NULL };
    httpd_register_uri_handler(camera_httpd, &index_uri);
    httpd_register_uri_handler(camera_httpd, &cap_uri);
  }

  // Stream server — port 81
  cfg.server_port = 81;
  cfg.ctrl_port   = 32769;
  if (httpd_start(&stream_httpd, &cfg) == ESP_OK) {
    httpd_uri_t stream_uri = { "/stream", HTTP_GET, streamHandler, NULL };
    httpd_register_uri_handler(stream_httpd, &stream_uri);
  }
}

// ─────────────────────────────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(false);

  initCamera();

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.printf("[WiFi] Connecting to %s", WIFI_SSID);

  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < WIFI_TIMEOUT_MS) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[ERROR] WiFi failed — halting");
    while (true) delay(1000);
  }

  Serial.printf("[WiFi] Connected. IP: %s\n", WiFi.localIP().toString().c_str());
  startServers();

  Serial.printf("[OK] Stream → http://%s/stream\n",   WiFi.localIP().toString().c_str());
  Serial.printf("[OK] Capture → http://%s/capture\n", WiFi.localIP().toString().c_str());
  Serial.printf("[OK] View → http://%s/\n",            WiFi.localIP().toString().c_str());
}

void loop() {
  delay(10000);
}
