/*
 * ESP32-CAM (AI-Thinker) — VGA + MJPEG (GRAB_LATEST, TCP_NODELAY).
 * Bản sao lưu sketch trong repo; nạp bằng Arduino IDE / PlatformIO tùy board.
 */
#include <Arduino.h>
#include "esp_camera.h"
#include <WiFi.h>
#include "esp_http_server.h"
#include "esp_err.h"
#include <lwip/sockets.h>
#include <lwip/netdb.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

#define ENABLE_DASHBOARD_WS 0
#if ENABLE_DASHBOARD_WS
// Arduino Library Manager: "WebSockets" (Links2004) — WebSocketsClient.h
#include <WebSocketsClient.h>
#endif

const char *ssid     = "lap09";
const char *password = "nk111111";

// Máy chạy dashboard: uvicorn main:app --host 0.0.0.0 --port 8765
// (Hotspot Windows thường là .1; đổi cho đúng IP laptop khi CAM ping được.)
#define DASHBOARD_WS_HOST "192.168.137.1"
#define DASHBOARD_WS_PORT 8765
// Trùng với "cam_ws_token" trong config/system_config.json; rỗng = không gửi ?token=
#define DASHBOARD_WS_TOKEN ""

#if ENABLE_DASHBOARD_WS
WebSocketsClient ws;
#endif

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

httpd_handle_t stream_httpd = NULL;
httpd_handle_t control_httpd = NULL;
static bool g_stream_enabled = false;

/*
 * Ưu tiên mượt cho pipeline YOLO qua Wi-Fi:
 * - VGA giữ đủ chi tiết cho phát hiện lửa.
 * - jpeg_quality số lớn hơn => nén mạnh hơn => frame nhỏ hơn, đỡ lag.
 */
#define CAM_FRAME_SIZE   FRAMESIZE_VGA
#define CAM_JPEG_Q_PSRAM 13
#define CAM_JPEG_Q_NO_PSRAM 15
// 0 = chỉ chặn stream logic (ổn định hơn); 1 = dùng PWDN tắt cứng camera.
#define CAM_USE_HARD_PWDN 0

static void configure_camera_quality(sensor_t *s) {
  s->set_brightness(s, 0);
  s->set_contrast(s, 1);
  s->set_saturation(s, 0);
  s->set_sharpness(s, 1);
  s->set_whitebal(s, 1);
  s->set_awb_gain(s, 1);
  s->set_exposure_ctrl(s, 1);
  s->set_aec2(s, 1);
  s->set_ae_level(s, -2);
  s->set_gain_ctrl(s, 1);
  s->set_agc_gain(s, 3);
  s->set_lenc(s, 1);
  s->set_bpc(s, 1);
  s->set_wpc(s, 1);
  s->set_dcw(s, 0);
  s->set_colorbar(s, 0);
  s->set_raw_gma(s, 1);
  s->set_gainceiling(s, GAINCEILING_4X);
}

static const char *_STREAM_TYPE = "multipart/x-mixed-replace;boundary=frame";
static const char *_BOUNDARY = "\r\n--frame\r\n";
static const char *_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static esp_err_t stream_handler(httpd_req_t *req) {
  if (!g_stream_enabled) {
    httpd_resp_set_status(req, "503 Service Unavailable");
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, "{\"ok\":false,\"error\":\"stream_disabled\"}", HTTPD_RESP_USE_STRLEN);
  }
  camera_fb_t *fb = NULL;
  char part_buf[64];

  int fd = httpd_req_to_sockfd(req);
  if (fd >= 0) {
    int one = 1;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
  }

  httpd_resp_set_type(req, _STREAM_TYPE);
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store, no-cache, must-revalidate");
  httpd_resp_set_hdr(req, "Pragma", "no-cache");

  for (int i = 0; i < 3; i++) {
    camera_fb_t *f = esp_camera_fb_get();
    if (f) esp_camera_fb_return(f);
  }

  while (true) {
    fb = esp_camera_fb_get();
    if (!fb) {
      delay(2);
      continue;
    }
    size_t hlen = snprintf(part_buf, 64, _PART, fb->len);
    if (httpd_resp_send_chunk(req, _BOUNDARY, strlen(_BOUNDARY)) != ESP_OK) {
      esp_camera_fb_return(fb);
      break;
    }
    if (httpd_resp_send_chunk(req, part_buf, hlen) != ESP_OK) {
      esp_camera_fb_return(fb);
      break;
    }
    if (httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len) != ESP_OK) {
      esp_camera_fb_return(fb);
      break;
    }
    esp_camera_fb_return(fb);
    yield();
  }
  return ESP_OK;
}

static void set_camera_active(bool on) {
  // Một số core/camera bị rớt kết nối nếu toggle PWDN liên tục khi dashboard đang reconnect.
  // Mặc định giữ camera "awake", chỉ khóa stream bằng g_stream_enabled cho ổn định.
#if CAM_USE_HARD_PWDN
  if (PWDN_GPIO_NUM >= 0) {
    pinMode(PWDN_GPIO_NUM, OUTPUT);
    digitalWrite(PWDN_GPIO_NUM, on ? LOW : HIGH);
  }
#endif
  g_stream_enabled = on;
}

#if ENABLE_DASHBOARD_WS
static void onWsEvent(WStype_t type, uint8_t *payload, size_t length) {
  switch (type) {
    case WStype_DISCONNECTED:
      Serial.println("[WS] ngắt kết nối dashboard — sẽ tự kết nối lại");
      break;
    case WStype_CONNECTED:
      Serial.print("[WS] đã nối dashboard: ");
      Serial.println(payload ? (char *)payload : "(null)");
      break;
    case WStype_TEXT: {
      String s;
      if (payload && length) {
        s.reserve(length + 1);
        for (size_t i = 0; i < length; i++) s += (char)payload[i];
      }
      if (s.indexOf("stream_on") >= 0) {
        set_camera_active(true);
        Serial.println("[WS] stream ON");
      } else if (s.indexOf("stream_off") >= 0) {
        set_camera_active(false);
        Serial.println("[WS] stream OFF");
      }
      break;
    }
    default:
      break;
  }
}
#endif

static esp_err_t cam_on_handler(httpd_req_t *req) {
  set_camera_active(true);
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, "{\"ok\":true,\"camera\":\"on\"}", HTTPD_RESP_USE_STRLEN);
}

static esp_err_t cam_off_handler(httpd_req_t *req) {
  set_camera_active(false);
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, "{\"ok\":true,\"camera\":\"off\"}", HTTPD_RESP_USE_STRLEN);
}

static esp_err_t cam_status_handler(httpd_req_t *req) {
  char body[64];
  snprintf(body, sizeof(body), "{\"ok\":true,\"stream_enabled\":%s}", g_stream_enabled ? "true" : "false");
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, body, HTTPD_RESP_USE_STRLEN);
}

static void startCameraServer() {
  httpd_config_t stream_cfg = HTTPD_DEFAULT_CONFIG();
  stream_cfg.server_port = 81;
  stream_cfg.ctrl_port = 32769;
  stream_cfg.max_open_sockets = 4;
  stream_cfg.lru_purge_enable = true;
  stream_cfg.recv_wait_timeout = 10;
  stream_cfg.send_wait_timeout = 10;

  httpd_config_t control_cfg = HTTPD_DEFAULT_CONFIG();
  control_cfg.server_port = 82;   // Cổng điều khiển riêng
  control_cfg.ctrl_port = 32770;  // Không trùng ctrl_port của stream
  control_cfg.max_open_sockets = 3;
  control_cfg.lru_purge_enable = true;
  control_cfg.recv_wait_timeout = 6;
  control_cfg.send_wait_timeout = 6;

  httpd_uri_t stream_uri = {
      .uri = "/stream",
      .method = HTTP_GET,
      .handler = stream_handler,
      .user_ctx = NULL,
  };
  httpd_uri_t cam_on_uri = {
      .uri = "/camera/on",
      .method = HTTP_POST,
      .handler = cam_on_handler,
      .user_ctx = NULL,
  };
  httpd_uri_t cam_off_uri = {
      .uri = "/camera/off",
      .method = HTTP_POST,
      .handler = cam_off_handler,
      .user_ctx = NULL,
  };
  httpd_uri_t cam_status_uri = {
      .uri = "/camera/status",
      .method = HTTP_GET,
      .handler = cam_status_handler,
      .user_ctx = NULL,
  };

  esp_err_t e1 = httpd_start(&stream_httpd, &stream_cfg);
  if (e1 == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &stream_uri);
    Serial.println("HTTP stream: GET /stream @ :81");
  } else {
    Serial.print("HTTP stream start FAILED: ");
    Serial.println(esp_err_to_name(e1));
  }

  esp_err_t e2 = httpd_start(&control_httpd, &control_cfg);
  if (e2 == ESP_OK) {
    httpd_register_uri_handler(control_httpd, &cam_on_uri);
    httpd_register_uri_handler(control_httpd, &cam_off_uri);
    httpd_register_uri_handler(control_httpd, &cam_status_uri);
    Serial.println("HTTP control: POST /camera/on, POST /camera/off, GET /camera/status @ :82");
  } else {
    Serial.print("HTTP control start FAILED: ");
    Serial.println(esp_err_to_name(e2));
    // Fallback an toàn: nếu không mở được cổng 82 thì vẫn cho điều khiển trên cổng stream 81.
    if (stream_httpd) {
      httpd_register_uri_handler(stream_httpd, &cam_on_uri);
      httpd_register_uri_handler(stream_httpd, &cam_off_uri);
      httpd_register_uri_handler(stream_httpd, &cam_status_uri);
      Serial.println("HTTP control FALLBACK: /camera/* đã gắn vào :81");
    }
  }
}

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  Serial.begin(115200);
  delay(200);

  camera_config_t config = {};
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  if (psramFound()) {
    config.frame_size = CAM_FRAME_SIZE;
    config.jpeg_quality = CAM_JPEG_Q_PSRAM;
    config.fb_count = 2;
    config.grab_mode = CAMERA_GRAB_LATEST;
    config.fb_location = CAMERA_FB_IN_PSRAM;
  } else {
    config.frame_size = CAM_FRAME_SIZE;
    config.jpeg_quality = CAM_JPEG_Q_NO_PSRAM;
    config.fb_count = 1;
    config.grab_mode = CAMERA_GRAB_LATEST;
  }

  if (esp_camera_init(&config) != ESP_OK) {
    Serial.println("Camera init failed");
    return;
  }

  sensor_t *s = esp_camera_sensor_get();
  configure_camera_quality(s);
  if (s) {
    s->set_framesize(s, CAM_FRAME_SIZE);
    s->set_quality(s, psramFound() ? CAM_JPEG_Q_PSRAM : CAM_JPEG_Q_NO_PSRAM);
  }
  set_camera_active(false);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  WiFi.setSleep(false);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);

  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
  }
  Serial.println();
  Serial.println(WiFi.localIP());
  startCameraServer();
  Serial.printf("Camera profile: size=%d, q(psram)=%d, q(no_psram)=%d\n",
                CAM_FRAME_SIZE, CAM_JPEG_Q_PSRAM, CAM_JPEG_Q_NO_PSRAM);
  Serial.println("Camera split ports: stream=:81 | control=:82");
  #if ENABLE_DASHBOARD_WS
  {
    String wsPath = "/ws/cam";
    if (strlen(DASHBOARD_WS_TOKEN) > 0) {
      wsPath += "?token=";
      wsPath += DASHBOARD_WS_TOKEN;
    }
    ws.begin(DASHBOARD_WS_HOST, DASHBOARD_WS_PORT, wsPath.c_str());
    ws.onEvent(onWsEvent);
    ws.setReconnectInterval(5000);
    Serial.print("[WS] Kết nối tới ws://");
    Serial.print(DASHBOARD_WS_HOST);
    Serial.print(":");
    Serial.print(DASHBOARD_WS_PORT);
    Serial.println(wsPath);
  }
  #else
  Serial.println("[WS] disabled (build không cần WebSocketsClient)");
  #endif
}

void loop() {
  #if ENABLE_DASHBOARD_WS
  ws.loop();
  #endif
  delay(2);
}
