#include <string.h>
#include "freertos/FreeRTOS.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "wifi_station.h"
#include "telemetry_http.h"

#define WIFI_SSID      "lap09"
#define WIFI_PASS      "nk111111"
#define WIFI_MAX_RETRY 10

static const char *TAG = "WIFI_STA";
static int s_retry_num = 0;

static esp_timer_handle_t s_telemetry_timer;

static void telemetry_timer_cb(void *arg)
{
    (void)arg;
    telemetry_http_start();
}

/** Không dùng xTaskCreate trong esp_event handler: đang giữ mutex đệ quy của loop, stack 2304B dễ tràn. */
static esp_err_t schedule_telemetry_after_ip(void)
{
    if (s_telemetry_timer == NULL) {
        const esp_timer_create_args_t targs = {
            .callback = telemetry_timer_cb,
            .arg = NULL,
            .name = "tel_http",
        };
        esp_err_t err = esp_timer_create(&targs, &s_telemetry_timer);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_timer_create failed: %s", esp_err_to_name(err));
            return err;
        }
    }
    esp_timer_stop(s_telemetry_timer);
    return esp_timer_start_once(s_telemetry_timer, 400 * 1000);
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < WIFI_MAX_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGW(TAG, "Mất kết nối, thử lại (%d/%d)...", s_retry_num, WIFI_MAX_RETRY);
        } else {
            ESP_LOGE(TAG, "Không kết nối được WiFi sau %d lần.", WIFI_MAX_RETRY);
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *ev = (ip_event_got_ip_t *)event_data;
        s_retry_num = 0;
        ESP_LOGI(TAG, "Đã có IP: " IPSTR " — gateway: " IPSTR,
                 IP2STR(&ev->ip_info.ip), IP2STR(&ev->ip_info.gw));
        static bool s_telemetry_scheduled;
        if (!s_telemetry_scheduled) {
            s_telemetry_scheduled = true;
            esp_err_t te = schedule_telemetry_after_ip();
            if (te != ESP_OK) {
                s_telemetry_scheduled = false;
                ESP_LOGE(TAG, "Lên lịch telemetry HTTP thất bại: %s", esp_err_to_name(te));
            }
        }
    }
}

void wifi_init_station(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_config = { 0 };
    strncpy((char *)wifi_config.sta.ssid, WIFI_SSID, sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char *)wifi_config.sta.password, WIFI_PASS, sizeof(wifi_config.sta.password) - 1);
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    /*
     * Giảm độ trễ telemetry: tắt Wi‑Fi power save (mặc định modem sleep có thể làm TCP/HTTP
     * bị “giật” theo chu kỳ, nhìn như delay vài giây trên dashboard).
     */
    esp_err_t ps = esp_wifi_set_ps(WIFI_PS_NONE);
    if (ps != ESP_OK) {
        ESP_LOGW(TAG, "esp_wifi_set_ps(WIFI_PS_NONE) lỗi: %s", esp_err_to_name(ps));
    }
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Đang kết nối STA tới SSID=%s (laptop làm AP)", WIFI_SSID);
}
