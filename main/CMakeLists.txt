idf_component_register(
	SRCS "app_main.c" "wifi_ap.c"
	INCLUDE_DIRS "."
	REQUIRES motor servo frame_sensor hc_sr04 relay esp_wifi nvs_flash lwip
)
