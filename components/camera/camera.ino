#include "esp_camera.h"
#include <WiFi.h>
#include "esp_http_server.h"

// =================== CẤU HÌNH WIFI ===================
const char *ssid = "K99";          // WiFi SSID của ESP32-S3
const char *password = "nk111111"; // Mật khẩu WiFi

// =================== CẤU HÌNH CHÂN ESP32-CAM (AI-THINKER) ===================
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

httpd_handle_t camera_httpd = NULL;

// Handler lấy ảnh JPEG cho người dùng xem trên Web
static esp_err_t view_handler(httpd_req_t *req) {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) return ESP_FAIL;
  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
  esp_err_t res = httpd_resp_send(req, (const char *)fb->buf, fb->len);
  esp_camera_fb_return(fb);
  return res;
}

// Handler lấy ảnh RGB thô cho AI của ESP32-S3 xử lý
static esp_err_t capture_handler(httpd_req_t *req) {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) return ESP_FAIL;
  
  // Giải mã sang RGB thô ngay trên ESP32-CAM để S3 không bị tràn RAM khi nhận dữ liệu
  uint8_t *rgb_buf = (uint8_t *)malloc(fb->width * fb->height * 3);
  if (rgb_buf && fmt2rgb888(fb->buf, fb->len, fb->format, rgb_buf)) {
      httpd_resp_set_type(req, "application/octet-stream");
      httpd_resp_send(req, (const char *)rgb_buf, fb->width * fb->height * 3);
  } else {
      httpd_resp_send_500(req);
  }
  if (rgb_buf) free(rgb_buf);
  esp_camera_fb_return(fb);
  return ESP_OK;
}

// Stream video trực tiếp (MJPEG) giúp theo dõi xe di chuyển
static esp_err_t stream_handler(httpd_req_t *req) {
  camera_fb_t *fb = NULL;
  esp_err_t res = ESP_OK;
  char *part_buf[64];
  httpd_resp_set_type(req, "multipart/x-mixed-replace; boundary=frame");

  while (true) {
    fb = esp_camera_fb_get();
    if (!fb) break;
    size_t hlen = snprintf((char *)part_buf, 64, "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n", fb->len);
    res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len);
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, "\r\n--frame\r\n", 11);
    esp_camera_fb_return(fb);
    if (res != ESP_OK) break;
  }
  return res;
}

void startCameraServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;

  httpd_uri_t view_uri = {.uri="/view", .method=HTTP_GET, .handler=view_handler, .user_ctx=NULL};
  httpd_uri_t capture_uri = {.uri="/capture", .method=HTTP_GET, .handler=capture_handler, .user_ctx=NULL};
  httpd_uri_t stream_uri = {.uri="/stream", .method=HTTP_GET, .handler=stream_handler, .user_ctx=NULL};

  if (httpd_start(&camera_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(camera_httpd, &view_uri);
    httpd_register_uri_handler(camera_httpd, &capture_uri);
    httpd_register_uri_handler(camera_httpd, &stream_uri);
    Serial.println("✓ Server started: /view, /capture, /stream");
  }
}

void setup() {
  Serial.begin(115200);
  camera_config_t config;
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
  config.frame_size = FRAMESIZE_QQVGA; // 160x120 phù hợp cho AI FOMO
  config.jpeg_quality = 12;
  config.fb_count = 2;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n✓ Connected to S3 SoftAP!");
  
  startCameraServer();
  Serial.printf("Camera IP: %s\n", WiFi.localIP().toString().c_str());
}

void loop() {
  delay(10000);
}
