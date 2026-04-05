#include "robot_state.h"

#include "esp_timer.h"

static const char *s_state_label = "idle_scan";

static float s_ai_conf = 0.f;
static int64_t s_ai_update_us = 0;

#define AI_GATE_MIN_CONF       ROBOT_STATE_AI_RELAY_MIN_CONF
/** Sau lần thấy conf >= AI_GATE_MIN_CONF, trong khoảng này vẫn cho relay (chống flicker YOLO / POST=0). */
#define AI_GATE_HOLD_TTL_US    (4000 * 1000)
/** Tuổi tối đa của bản tin POST bất kỳ để hiển thị /api/status không quá cũ. */
#define AI_CAMERA_STALE_TTL_US (8000 * 1000)

static float s_ai_gate_conf = 0.f;
static int64_t s_ai_gate_us = 0;

void robot_state_set_idle_scan(void)
{
    s_state_label = "idle_scan";
}

void robot_state_set_extinguish(void)
{
    s_state_label = "extinguish";
}

const char *robot_state_str(void)
{
    return s_state_label;
}

void robot_state_ai_camera_set(float confidence_0_1)
{
    if (confidence_0_1 < 0.f) {
        confidence_0_1 = 0.f;
    }
    if (confidence_0_1 > 1.f) {
        confidence_0_1 = 1.f;
    }
    s_ai_conf = confidence_0_1;
    s_ai_update_us = esp_timer_get_time();
    if (confidence_0_1 >= AI_GATE_MIN_CONF) {
        s_ai_gate_conf = confidence_0_1;
        s_ai_gate_us = s_ai_update_us;
    }
}

float robot_state_ai_camera_confidence(void)
{
    return s_ai_conf;
}

bool robot_state_ai_camera_fresh_ok(float min_conf_0_1)
{
    if (s_ai_gate_us == 0) {
        return false;
    }
    int64_t now = esp_timer_get_time();
    if (now - s_ai_gate_us > AI_GATE_HOLD_TTL_US) {
        return false;
    }
    if (now - s_ai_update_us > AI_CAMERA_STALE_TTL_US) {
        return false;
    }
    return s_ai_gate_conf >= min_conf_0_1;
}

long robot_state_ai_camera_age_ms(void)
{
    if (s_ai_update_us == 0) {
        return -1L;
    }
    return (long)((esp_timer_get_time() - s_ai_update_us) / 1000);
}

