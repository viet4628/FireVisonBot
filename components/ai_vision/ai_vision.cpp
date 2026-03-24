#include "ai_vision.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "esp_http_client.h"

// Thư viện AI (Đảm bảo đã chạy add-dependency tflite-micro)
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/schema/schema_generated.h"

// Gọi thằng file ma trận mà bạn đã train!
#include "../../trained_model/fire_model_data.h" 

#include "esp_heap_caps.h"

static const char *TAG = "AI_VISION";
static uint8_t *img_buf = NULL;
static int img_len = 0;

static const tflite::Model* model = nullptr;
static tflite::MicroInterpreter* interpreter = nullptr;
static TfLiteTensor* input = nullptr;
static TfLiteTensor* output = nullptr;

constexpr int kTensorArenaSize = 1024 * 1024; // Tăng lên 1MB để đáp ứng yêu cầu Model (Thực tế cần khoảng 800KB)
static uint8_t *tensor_arena = nullptr; 

// Hứng chùm màu RGB thô từ ESP32-CAM truyền qua WiFi
esp_err_t _http_event_handle(esp_http_client_event_t *evt)
{
    switch(evt->event_id) {
        case HTTP_EVENT_ON_DATA:
            if (!esp_http_client_is_chunked_response(evt->client)) {
                if (img_buf == NULL) {
                    img_buf = (uint8_t *)malloc(esp_http_client_get_content_length(evt->client));
                    img_len = 0;
                    if (img_buf == NULL) return ESP_FAIL;
                }
                memcpy(img_buf + img_len, evt->data, evt->data_len);
                img_len += evt->data_len;
            }
            break;
        case HTTP_EVENT_DISCONNECTED:
            break;
        default:
            break;
    }
    return ESP_OK;
}

void ai_vision_init(void) {
    ESP_LOGW(TAG, "Đang nạp Não Bộ AI (TFLite Micro) vào RAM...");
    
    // Cấp phát Arena từ PSRAM để không làm tràn DRAM của ESP32-S3
    if (tensor_arena == nullptr) {
        tensor_arena = (uint8_t *)heap_caps_malloc(kTensorArenaSize, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (tensor_arena == nullptr) {
            ESP_LOGE(TAG, "LỖI: Không thể cấp phát Arena AI (150KB) từ PSRAM!");
            return;
        }
    }

    model = tflite::GetModel(fire_model_data);
    if (model->version() != TFLITE_SCHEMA_VERSION) {
        ESP_LOGE(TAG, "LỖI PHIÊN BẢN MODEL: %d != %d", model->version(), TFLITE_SCHEMA_VERSION);
        return;
    }

    static tflite::MicroMutableOpResolver<20> resolver;
    resolver.AddConv2D();
    resolver.AddDepthwiseConv2D();
    resolver.AddReshape();
    resolver.AddPad();
    resolver.AddMaxPool2D();
    resolver.AddAveragePool2D();
    resolver.AddFullyConnected();
    resolver.AddSoftmax();
    resolver.AddRelu();
    resolver.AddRelu6();
    resolver.AddAdd();
    resolver.AddSub();
    resolver.AddMul();
    resolver.AddConcatenation();
    resolver.AddExpandDims();
    resolver.AddStridedSlice();

    static tflite::MicroInterpreter static_interpreter(model, resolver, tensor_arena, kTensorArenaSize);
    interpreter = &static_interpreter;

    if (interpreter->AllocateTensors() != kTfLiteOk) {
        ESP_LOGE(TAG, "Lỗi hết RAM khi cấp phát Tensors!");
        return;
    }

    input = interpreter->input(0);
    output = interpreter->output(0);
    ESP_LOGI(TAG, "✅ NÃO BỘ AI KÍCH HOẠT THÀNH CÔNG!");
    ESP_LOGI(TAG, "- Input Yêu cầu: %dx%d (Màu: %d)", input->dims->data[1], input->dims->data[2], input->dims->data[3]);
    ESP_LOGI(TAG, "- Lưới Output: %dx%d", output->dims->data[1], output->dims->data[2]);
}

bool ai_vision_detect_fire(void) {
    bool fire_detected = false;

    if (!interpreter) {
        ESP_LOGE(TAG, "AI chưa khởi tạo xong!");
        return false;
    }

    esp_http_client_config_t config = {};
    config.url = "http://192.168.4.2/capture";
    config.event_handler = _http_event_handle;
    config.timeout_ms = 1500; 
    
    ESP_LOGI(TAG, "📸 Đang chụp ảnh thô từ ESP32-CAM...");
    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_err_t err = esp_http_client_perform(client);
    
    if (err == ESP_OK) {
        if (img_buf != NULL && img_len > 0) {
            ESP_LOGW(TAG, "✅ Tải xong mã mảng màu thô: %d KB. Tiến hành Cắt xén và phân tích AI...", img_len / 1024);
            
            // Lấy thông số mảng AI đầu vào
            int t_width = input->dims->data[1];  // 124
            int t_height = input->dims->data[2]; // 124
            
            // Tìm toạ độ bắt đầu cắt hình QQVGA 160x120 về đúng tâm vuông 124x124
            int src_w = 160;
            int src_h = 120;
            int start_x = (src_w - t_width) / 2;
            int start_y = (src_h - t_height) / 2;
            if (start_x < 0) start_x = 0;
            if (start_y < 0) start_y = 0;

            int8_t* in_data = input->data.int8;

            // Nhồi điểm ảnh dư thừa bỏ đi vào Ma trận đầu vào
            for (int y = 0; y < t_height; y++) {
                for (int x = 0; x < t_width; x++) {
                    int src_x = start_x + x;
                    int src_y = start_y + y;
                    int src_idx = (src_y * src_w + src_x) * 3;
                    int dst_idx = (y * t_width + x) * 3;

                    if (src_idx + 2 < img_len) {
                        uint8_t r = img_buf[src_idx];
                        uint8_t g = img_buf[src_idx+1];
                        uint8_t b = img_buf[src_idx+2];
                        
                        // Chuẩn hoá điểm ảnh 0-255 về hệ Số đếm Int8 -128 đến 127
                        in_data[dst_idx] = (int8_t)((int)r - 128);
                        in_data[dst_idx+1] = (int8_t)((int)g - 128);
                        in_data[dst_idx+2] = (int8_t)((int)b - 128);
                    }
                }
            }

            // ======== BẮT ĐẦU CHẠY MODEL (INFERENCE) ========
            if (interpreter->Invoke() == kTfLiteOk) {
                int grid_w = output->dims->data[1]; // Lưới ô (vd 15, 16)
                int grid_h = output->dims->data[2]; 
                int num_classes = output->dims->data[3]; // Số nhóm phân loại
                
                int8_t* out_data = output->data.int8;
                float scale = output->params.scale;
                int zero_point = output->params.zero_point;
                
                float best_score = 0.0f;
                int best_x = -1, best_y = -1;

                // Quét từng ô nhỏ vuông vức của Model FOMO
                for (int y = 0; y < grid_h; y++) {
                    for (int x = 0; x < grid_w; x++) {
                        // Vứt Class 0 (Nền). Chỉ dò quét Cấp độ tin cậy của Đám cháy (Giả định Class 1 là Fire)
                        for (int c = 1; c < num_classes; c++) {
                            int idx = (y * grid_w + x) * num_classes + c;
                            float score = (out_data[idx] - zero_point) * scale;
                            
                            if (score > best_score) {
                                best_score = score;
                                best_x = x;
                                best_y = y;
                            }
                        }
                    }
                }
                
                if (best_score >= 0.2f) { // Độ tin cậy trên 50%
                    ESP_LOGW(TAG, "🔥 TÌM THẤY LỬA! Xác suất: %.1f%% - Tại mắt lưới Toạ độ: X[%d] Y[%d]", best_score * 100.0f, best_x, best_y);
                    fire_detected = true;
                } else {
                    ESP_LOGI(TAG, "Không tìm thấy Lửa. Độ tin cậy hạch toán lớn nhất trên ảnh: %.1f%%", best_score * 100.0f);
                }
            } else {
                ESP_LOGE(TAG, "Lỗi chết cháy khi cố tính toán nơ-ron: Interpreter Invoke Failed!");
            }

            // Dọn RAM ngay!
            free(img_buf);
            img_buf = NULL;
            img_len = 0;
        }
    }
    
    esp_http_client_cleanup(client);
    return fire_detected;
}
