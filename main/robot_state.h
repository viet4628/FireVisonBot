#ifndef ROBOT_STATE_H
#define ROBOT_STATE_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Ngưỡng tối thiểu (0…1) để coi YOLO “đủ mạnh” cho relay khi kết hợp IR trái. */
#define ROBOT_STATE_AI_RELAY_MIN_CONF 0.65f

void robot_state_set_idle_scan(void);
void robot_state_set_extinguish(void);
const char *robot_state_str(void);

/** Đặt độ tin cậy lửa từ camera (0…1), gọi từ POST /api/ai_fire (dashboard đẩy YOLO). */
void robot_state_ai_camera_set(float confidence_0_1);

float robot_state_ai_camera_confidence(void);

/**
 * true nếu gần đây đã có lần POST với confidence >= ngưỡng (vd 0.65).
 * Giữ cửa sổ vài giây sau lần “đủ mạnh” — tránh dashboard gửi 0 giữa các khung không bbox
 * làm relay tắt dù màn hình laptop vừa hiện >65%.
 */
bool robot_state_ai_camera_fresh_ok(float min_conf_0_1);

/** Thời gian từ lần cập nhật AI (ms), hoặc -1 nếu chưa có. */
long robot_state_ai_camera_age_ms(void);

/**
 * Đặt cả confidence và vị trí ngang của bò của lửa trong khung hình (0.0=trái, 0.5=giữa, 1.0=phải).
 * fire_detected = true nếu YOLO đã vượt ngưỡng conf, false = không thấy lửa.
 */
void robot_state_ai_camera_set_pos(float confidence_0_1, float x_ratio_0_1, bool fire_detected);

/** true nếu YOLO đã phát hiện lửa trong lần push mới nhất (và vẫn còn trong cửa sổ thời gian). */
bool robot_state_ai_camera_fire_detected(void);

/** Vị trí ngang của lửa (0.0=trái, 0.5=giữa, 1.0=phải). */
float robot_state_ai_camera_x_ratio(void);

#ifdef __cplusplus
}
#endif

#endif /* ROBOT_STATE_H */
