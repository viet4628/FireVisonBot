#ifndef AI_VISION_H
#define AI_VISION_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Khởi tạo model AI
void ai_vision_init(void);

// Lệnh HTTP Get hình ảnh từ ESP32-CAM và nhận diện lửa
// Trả về true nếu phát hiện có lửa, false nếu không thấy
bool ai_vision_detect_fire(void);

#ifdef __cplusplus
}
#endif

#endif // AI_VISION_H
