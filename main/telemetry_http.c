#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "telemetry_http.h"
#include "robot_state.h"
#include "frame_sensor.h"
#include "hc_sr04.h"
#include "relay.h"

static const char *TAG = "TELEM_HTTP";
static httpd_handle_t s_server = NULL;

static float parse_json_confidence(const char *buf)
{
    const char *p = strstr(buf, "confidence");
    if (p == NULL) {
        p = strstr(buf, "\"c\"");
    }
    if (p == NULL) {
        return -1.f;
    }
    p = strchr(p, ':');
    if (p == NULL) {
        return -1.f;
    }
    p++;
    while (*p == ' ' || *p == '\t' || *p == '"') {
        p++;
    }
    char *end = NULL;
    float v = strtof(p, &end);
    if (end == p) {
        return -1.f;
    }
    return v;
}

static esp_err_t api_ai_fire_post(httpd_req_t *req)
{
    char buf[256];
    size_t recvd = 0;
    size_t total = req->content_len;
    if (total > 0 && total < sizeof(buf)) {
        while (recvd < total) {
            int r = httpd_req_recv(req, buf + recvd, (int)(total - recvd));
            if (r <= 0) {
                break;
            }
            recvd += (size_t)r;
        }
    } else {
        int r = httpd_req_recv(req, buf, (int)sizeof(buf) - 1);
        if (r > 0) {
            recvd = (size_t)r;
        }
    }
    if (recvd == 0) {
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_set_type(req, "application/json; charset=utf-8");
        httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
        return httpd_resp_send(req, "{\"ok\":false,\"error\":\"empty\"}", HTTPD_RESP_USE_STRLEN);
    }
    buf[recvd] = '\0';

    float c = parse_json_confidence(buf);
    if (c < 0.f) {
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_set_type(req, "application/json; charset=utf-8");
        httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
        return httpd_resp_send(req, "{\"ok\":false,\"error\":\"bad_json\"}", HTTPD_RESP_USE_STRLEN);
    }
    if (c > 1.f) {
        c = 1.f;
    }

    robot_state_ai_camera_set(c);

    static int64_t s_ai_fire_log_us;
    int64_t now_us = esp_timer_get_time();
    if (c >= 0.5f && (now_us - s_ai_fire_log_us) >= (1000 * 1000)) {
        s_ai_fire_log_us = now_us;
        ESP_LOGI(TAG, "ai_fire OK: confidence=%.3f", (double)c);
    }

    httpd_resp_set_type(req, "application/json; charset=utf-8");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, "{\"ok\":true}", HTTPD_RESP_USE_STRLEN);
}

static esp_err_t api_status_get(httpd_req_t *req)
{
    bool flame_l = frame_sensor_is_fire_detected(FLAME_SENSOR_LEFT);
    bool flame_r = frame_sensor_is_fire_detected(FLAME_SENSOR_RIGHT);
    float dist = hc_sr04_get_last_distance_cm();
    bool relay = relay_is_on();
    const char *st = robot_state_str();
    int64_t uptime_ms = esp_timer_get_time() / 1000;

    float ai_c = robot_state_ai_camera_confidence();
    long ai_age = robot_state_ai_camera_age_ms();
    bool ai_ok_65 = robot_state_ai_camera_fresh_ok(ROBOT_STATE_AI_RELAY_MIN_CONF);

    char body[420];
    int n = snprintf(body, sizeof(body),
                     "{\"flame_left\":%s,\"flame_right\":%s,"
                     "\"distance_cm\":%.2f,\"relay_on\":%s,"
                     "\"state\":\"%s\",\"uptime_ms\":%lld,"
                     "\"ai_confidence\":%.4f,\"ai_age_ms\":%ld,"
                     "\"ai_fresh_above_65\":%s}",
                     flame_l ? "true" : "false",
                     flame_r ? "true" : "false",
                     dist,
                     relay ? "true" : "false",
                     st,
                     (long long)uptime_ms,
                     ai_c,
                     ai_age,
                     ai_ok_65 ? "true" : "false");
    if (n < 0 || n >= (int)sizeof(body)) {
        httpd_resp_set_status(req, "500 Internal Server Error");
        return httpd_resp_send(req, "{}", HTTPD_RESP_USE_STRLEN);
    }

    httpd_resp_set_type(req, "application/json; charset=utf-8");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, body, HTTPD_RESP_USE_STRLEN);
}

void telemetry_http_start(void)
{
    if (s_server != NULL) {
        return;
    }

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 8080;
    config.stack_size = 8192;
    config.lru_purge_enable = true;
    config.max_uri_handlers = 12;

    if (httpd_start(&s_server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "httpd_start failed");
        return;
    }

    httpd_uri_t uri_status = {
        .uri = "/api/status",
        .method = HTTP_GET,
        .handler = api_status_get,
        .user_ctx = NULL,
    };
    httpd_register_uri_handler(s_server, &uri_status);

    httpd_uri_t uri_ai = {
        .uri = "/api/ai_fire",
        .method = HTTP_POST,
        .handler = api_ai_fire_post,
        .user_ctx = NULL,
    };
    httpd_register_uri_handler(s_server, &uri_ai);

    ESP_LOGI(TAG, "Telemetry: GET /api/status  POST /api/ai_fire  :8080");
}
