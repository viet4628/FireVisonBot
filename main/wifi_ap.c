#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_mac.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "wifi_ap.h"

// Thay vì ESP32-CAM và ESP32-S3 phát chung WiFi mạng nhà, 
// ESP32-S3 sẽ tự phát ra một mạng WiFi "Hệ Sinh Thái Kín".
#define ESP_WIFI_SSID      "K99"
#define ESP_WIFI_PASS      "nk111111"
#define ESP_WIFI_CHANNEL   1
#define MAX_STA_CONN       4

static const char *TAG = "WIFI_AP";

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                    int32_t event_id, void* event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI(TAG, "ESP32-CAM ("MACSTR") connected! AID=%d",
                 MAC2STR(event->mac), event->aid);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGI(TAG, "ESP32-CAM ("MACSTR") disconnected! AID=%d",
                 MAC2STR(event->mac), event->aid);
    }
}

void wifi_init_softap(void)
{
    // Cần có NVS để khởi tạo WiFi
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = ESP_WIFI_SSID,
            .ssid_len = strlen(ESP_WIFI_SSID),
            .channel = ESP_WIFI_CHANNEL,
            .password = ESP_WIFI_PASS,
            .max_connection = MAX_STA_CONN,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK,
            .pmf_cfg = {
                    .required = false,
            },
        },
    };
    if (strlen(ESP_WIFI_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    // IP tĩnh mặc định của điểm phát AP trên ESP-IDF LUÔN LÀ: 192.168.4.1
    // Các thiết bị trạm (ESP32-CAM) sẽ có IP là: 192.168.4.2
    
    ESP_LOGI(TAG, "=============================================");
    ESP_LOGI(TAG, "  WIFI AP INITIALIZED: %s / %s", ESP_WIFI_SSID, ESP_WIFI_PASS);
    ESP_LOGI(TAG, "  ESP32-S3 IP: 192.168.4.1");
    ESP_LOGI(TAG, "  Waiting for ESP32-CAM at 192.168.4.2...");
    ESP_LOGI(TAG, "=============================================");
}
