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

#ifdef __cplusplus
}
#endif

#endif /* ROBOT_STATE_H */
