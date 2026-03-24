idf_component_register(
    SRCS "ai_vision.cpp"
    INCLUDE_DIRS "include"
    REQUIRES esp_http_client espressif__esp-tflite-micro
)
