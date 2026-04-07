#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/gpio.h"

#include "motor.h"
#include "servo.h"
#include "frame_sensor.h"
#include "hc_sr04.h"
#include "relay.h"
#include "wifi_station.h"
#include "robot_state.h"
#include "board_hw.h"

static const char *TAG = "FIRE_CTRL";

/** Relay bơm: IR trái + YOLO (POST /api/ai_fire) — cùng ngưỡng với robot_state (cửa sổ giữ trên ESP). */
#define RELAY_MIN_CAMERA_CONF  ROBOT_STATE_AI_RELAY_MIN_CONF

/* ───────── Buzzer (GPIO trong board_hw.h) ───────── */
#define BUZZER_GPIO     BOARD_GPIO_BUZZER
#define BUZZER_ON_LEVEL BOARD_BUZZER_ON_LEVEL

static void buzzer_init(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask  = (1ULL << BUZZER_GPIO),
        .mode          = GPIO_MODE_OUTPUT,
        .pull_up_en    = GPIO_PULLUP_DISABLE,
        .pull_down_en  = GPIO_PULLDOWN_DISABLE,
        .intr_type     = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(BUZZER_GPIO, !BUZZER_ON_LEVEL);
}

static void buzzer_set(bool on) {
    gpio_set_level(BUZZER_GPIO, on ? BUZZER_ON_LEVEL : !BUZZER_ON_LEVEL);
}

static void buzzer_beep_tick(bool active) {
    static TickType_t next_toggle = 0;
    static bool buzzer_on = false;
    const TickType_t on_ticks  = pdMS_TO_TICKS(60);
    const TickType_t off_ticks = pdMS_TO_TICKS(400);
    TickType_t now = xTaskGetTickCount();

    if (!active) {
        buzzer_on = false;
        next_toggle = 0;
        buzzer_set(false);
        return;
    }

    if (next_toggle == 0 || now >= next_toggle) {
        buzzer_on = !buzzer_on;
        buzzer_set(buzzer_on);
        next_toggle = now + (buzzer_on ? on_ticks : off_ticks);
    }
}

/* ───────── Servo: quét nhanh (-15°…180°), trục “giữa” / vòi tham chiếu ───────── */
/** Góc giữa khi quét rada / reset sau mất lửa (quét chậm như cũ). */
#define ANGLE_CENTER           80.f
/** Góc mục tiêu sau căn hướng IR trái / vòi (khoảng 76–78°). */
#define NOZZLE_ALIGN_TARGET_DEG 77.f
#define SCAN_SWEEP_MIN         SERVO_ANGLE_MIN
#define SCAN_SWEEP_MAX         SERVO_ANGLE_MAX
#define SCAN_SWEEP_STEP        2.0f
#define SCAN_TICK_MS           32

/**
 * 0 = tắt IR phải (chỉ IR trái).
 * 1 = IR phải → khóa góc tại chỗ → quay bánh ~180° để IR trái + camera quét lại.
 */
#ifndef USE_RIGHT_FLAME_IR
#define USE_RIGHT_FLAME_IR     1
#endif

/* FPV: một góc cố định, không căn theo lửa (xoay xe thay vì xoay camera). */
#define FPV_PAN_LOCK           FPV_PAN_CENTER
#define FPV_TILT_LOCK          FPV_TILT_DEFAULT

#define FPV_PAN_CENTER         90.f
#define FPV_TILT_DEFAULT       82.f

/* ───────── HC-SR04: vật cản khi xe chạy (histerezis) ───────── */
#define SR04_STOP_CM           28.f
#define SR04_RESUME_CM         40.f
#define SR04_INVALID_MAX_CM    400.f
#define SR04_LOG_INTERVAL_MS   800

/* ───────── Motor: tuần tra + quay 180° (hiệu chỉnh SPIN_180_MS) ───────── */
/**
 * Tuần tra (quét rada): mặc định 0 = chỉ 2 servo quét + 2 IR, bánh xe dừng.
 * Đặt 1 nếu muốn xe tự tiến khi không có vật cản SR04 (hành vi cũ).
 */
#ifndef PATROL_ENABLE_DRIVE
#define PATROL_ENABLE_DRIVE    0
#endif
#define PATROL_DUTY            1023u
#define SPIN_DUTY              1023u
#define SPIN_180_MS            1400u
/* Giảm riêng tốc độ lúc căn hướng (tăng lại nhẹ theo yêu cầu). */
#define ALIGN_SPIN_DUTY        820u

/** Xoay nhẹ bánh khi IR trái xác nhận (ms), 0 = tắt. Canh hướng thay vì xoay FPV. */
#define LEFT_FIRE_PIVOT_MS     0u

/**
 * Căn IR trái về NOZZLE_ALIGN_TARGET_DEG: motor + servo từng bước (servo bước lớn = chỉnh nhanh).
 * err>0 → trái tiến phải lùi; err<0 → phải tiến trái lùi.
 */
#define ALIGN_SERVO_STEP_DEG      2.8f
#define ALIGN_MOTOR_PULSE_MS      115u
#define ALIGN_MAX_STEPS           200
#define LEFT_IR_ALIGN_EPS_DEG     2.0f

/** Số vòng lặp liên tiếp (mỗi ~20ms) có IR báo lửa mới coi là xác nhận — tránh nhiễu và tránh quét servo ghi đè. */
#define IR_FIRE_STABLE_LOOPS   14

#define LOST_FIRE_MS           3000

typedef enum {
    STATE_PATROL,
    STATE_SPIN_180,
    STATE_ALIGN_LEFT_NOZZLE,
    STATE_EXTINGUISH,
} robot_state_t;

static void fpv_lock_fixed(void) {
    servo_set_angle(SERVO_FPV_PAN, FPV_PAN_LOCK);
    servo_set_angle(SERVO_FPV_TILT, FPV_TILT_LOCK);
}

/**
 * Hai servo quét: L += dir·step, R -= dir·step.
 * Đảo chiều một lần khi chạm biên (L max hoặc R min khi dir>0; L min hoặc R max khi dir<0).
 * Tránh hai khối if riêng ghi đè dir (L đặt -1 rồi R đặt +1 → kẹt ở biên, chỉ quét được một vòng).
 */
static void peripheral_idle_sweep_opposite(float *aL, float *aR, int *dir) {
    *aL += (*dir) * SCAN_SWEEP_STEP;
    *aR -= (*dir) * SCAN_SWEEP_STEP;

    if (*aL > SCAN_SWEEP_MAX) {
        *aL = SCAN_SWEEP_MAX;
    }
    if (*aL < SCAN_SWEEP_MIN) {
        *aL = SCAN_SWEEP_MIN;
    }
    if (*aR > SCAN_SWEEP_MAX) {
        *aR = SCAN_SWEEP_MAX;
    }
    if (*aR < SCAN_SWEEP_MIN) {
        *aR = SCAN_SWEEP_MIN;
    }

    if (*dir > 0) {
        if (*aL >= SCAN_SWEEP_MAX || *aR <= SCAN_SWEEP_MIN) {
            *dir = -1;
        }
    } else {
        if (*aL <= SCAN_SWEEP_MIN || *aR >= SCAN_SWEEP_MAX) {
            *dir = 1;
        }
    }

    servo_set_angle(SERVO_SCAN_LEFT, *aL);
    servo_set_angle(SERVO_SCAN_RIGHT, *aR);
    fpv_lock_fixed();
}

static void read_flame_inputs(bool *out_l, bool *out_r) {
    *out_l = frame_sensor_is_fire_detected(FLAME_SENSOR_LEFT);
#if USE_RIGHT_FLAME_IR
    *out_r = frame_sensor_is_fire_detected(FLAME_SENSOR_RIGHT);
#else
    *out_r = false;
#endif
}

static void servos_reset_neutral(void) {
    servo_set_angle(SERVO_SCAN_LEFT, ANGLE_CENTER);
    servo_set_angle(SERVO_SCAN_RIGHT, ANGLE_CENTER);
    fpv_lock_fixed();
}

/** Lưu góc thực tế đang ra lệnh cho 2 servo quét — khóa đúng tư thế lúc IR xác nhận, không nhảy về góc cứng. */
static void capture_scan_hold(float *hold_L, float *hold_R) {
    *hold_L = servo_get_angle(SERVO_SCAN_LEFT);
    *hold_R = servo_get_angle(SERVO_SCAN_RIGHT);
}

static void apply_scan_hold(float hold_L, float hold_R) {
    servo_set_angle(SERVO_SCAN_LEFT, hold_L);
    servo_set_angle(SERVO_SCAN_RIGHT, hold_R);
    fpv_lock_fixed();
}

static void fire_control_task(void *arg) {
    (void)arg;

    robot_state_t state = STATE_PATROL;
    robot_state_set_idle_scan();

    float scan_angle_L = ANGLE_CENTER;
    float scan_angle_R = ANGLE_CENTER;
    int sweep_dir = 1;
    float hold_scan_L = ANGLE_CENTER;
    float hold_scan_R = ANGLE_CENTER;
    TickType_t last_scan_move = 0;
    TickType_t last_sr04_log = 0;
    float last_sr04_cm = -1.f;
    bool path_blocked = false;
    unsigned fire_stable_loops = 0;
    TickType_t lost_fire_since_tick = 0;
    uint16_t align_iterate_steps = 0;

    ESP_LOGI(TAG, "Patrol: drive=%d | IR phải=%d | relay+cam>=%.0f%% | IR ổn định %u",
             PATROL_ENABLE_DRIVE, USE_RIGHT_FLAME_IR,
             (double)(RELAY_MIN_CAMERA_CONF * 100.f), (unsigned)IR_FIRE_STABLE_LOOPS);

    while (1) {
        TickType_t now = xTaskGetTickCount();
        float dist = hc_sr04_get_last_distance_cm();
        bool dist_ok = (dist > 0.5f && dist < SR04_INVALID_MAX_CM);

        /*
         * SR04: chỉ tin “vật cản” khi có đo hợp lệ. Nếu không dist_ok mà vẫn giữ path_blocked=true
         * (echo mất / timeout nhưng latest_distance cũ vẫn < 28 cm) → tuần tra đứng yên mãi.
         */
        if (state == STATE_PATROL) {
            if (!dist_ok) {
                path_blocked = false;
            } else if (dist < SR04_STOP_CM) {
                path_blocked = true;
            } else if (dist > SR04_RESUME_CM) {
                path_blocked = false;
            }
        }

        if (now - last_sr04_log >= pdMS_TO_TICKS(SR04_LOG_INTERVAL_MS)) {
            last_sr04_log = now;
            if (dist_ok) {
                if (path_blocked && state == STATE_PATROL) {
                    ESP_LOGW(TAG, "Vật cản (đang/lẽ chạy): ~%.1f cm — dừng", dist);
                } else if (fabsf(dist - last_sr04_cm) > 3.f) {
                    ESP_LOGI(TAG, "SR04: ~%.1f cm", dist);
                }
                last_sr04_cm = dist;
            }
        }

        bool fire_l;
        bool fire_r;
        read_flame_inputs(&fire_l, &fire_r);

        switch (state) {
            case STATE_PATROL:
                relay_off();
                buzzer_beep_tick(false);
                robot_state_set_idle_scan();

                if (fire_l || fire_r) {
                    fpv_lock_fixed();
                    fire_stable_loops++;
                    motor_stop();
                    /* Không gọi peripheral_idle_sweep khi đang có lửa / đang đếm ổn định — tránh kéo servo khỏi góc khóa */
                    if (fire_stable_loops >= IR_FIRE_STABLE_LOOPS) {
                        fire_stable_loops = 0;
                        lost_fire_since_tick = 0;
                        if (fire_l && fire_r) {
                            capture_scan_hold(&hold_scan_L, &hold_scan_R);
                            apply_scan_hold(hold_scan_L, hold_scan_R);
                            ESP_LOGW(TAG, "IR cả hai — khóa góc từng servo tại chỗ nhận (L=%.1f R=%.1f).",
                                     hold_scan_L, hold_scan_R);
                            state = STATE_EXTINGUISH;
                            robot_state_set_extinguish();
                            break;
                        }
                        /* Chỉ IR phải TRƯỚC nhánh IR trái: tránh trái nhiễu nhẹ mà không bao giờ quay 180°. */
#if USE_RIGHT_FLAME_IR
                        if (fire_r && !fire_l) {
                            capture_scan_hold(&hold_scan_L, &hold_scan_R);
                            apply_scan_hold(hold_scan_L, hold_scan_R);
                            ESP_LOGW(TAG, "IR phải (trái tắt) — khóa L=%.1f R=%.1f → SPIN_180.",
                                     hold_scan_L, hold_scan_R);
                            state = STATE_SPIN_180;
                            break;
                        }
#endif
                        /* IR trái (đã loại cả hai và chỉ-phải): khóa góc → căn bánh đưa servo L về 90° (vòi). */
                        if (fire_l) {
#if LEFT_FIRE_PIVOT_MS > 0
                            motor_turn_left(SPIN_DUTY);
                            vTaskDelay(pdMS_TO_TICKS(LEFT_FIRE_PIVOT_MS));
                            motor_stop();
                            vTaskDelay(pdMS_TO_TICKS(80));
#endif
                            capture_scan_hold(&hold_scan_L, &hold_scan_R);
                            apply_scan_hold(hold_scan_L, hold_scan_R);
                            ESP_LOGW(TAG, "IR trái — khóa L=%.1f R=%.1f → căn L → %.0f° (motor+servo từng bước).",
                                     hold_scan_L, hold_scan_R, (double)NOZZLE_ALIGN_TARGET_DEG);
                            align_iterate_steps = 0;
                            state = STATE_ALIGN_LEFT_NOZZLE;
                            break;
                        }
                    }
                    break;
                }

                fire_stable_loops = 0;

#if PATROL_ENABLE_DRIVE
                if (!path_blocked) {
                    motor_forward(PATROL_DUTY);
                } else {
                    motor_stop();
                }
#else
                /* Quét rada: chỉ servo + IR; SR04 vẫn đọc cho telemetry / tương lai. */
                motor_stop();
#endif

                if (now - last_scan_move >= pdMS_TO_TICKS(SCAN_TICK_MS)) {
                    last_scan_move = now;
                    peripheral_idle_sweep_opposite(&scan_angle_L, &scan_angle_R, &sweep_dir);
                }
                break;

            case STATE_SPIN_180:
#if !USE_RIGHT_FLAME_IR
                scan_angle_L = scan_angle_R = ANGLE_CENTER;
                sweep_dir = 1;
                servos_reset_neutral();
                state = STATE_PATROL;
                fire_stable_loops = 0;
                break;
#else
                robot_state_set_idle_scan();
                apply_scan_hold(hold_scan_L, hold_scan_R);
                fpv_lock_fixed();
                motor_turn_right(SPIN_DUTY);
                vTaskDelay(pdMS_TO_TICKS(SPIN_180_MS));
                motor_stop();
                vTaskDelay(pdMS_TO_TICKS(200));

                read_flame_inputs(&fire_l, &fire_r);
                if (fire_l && fire_r) {
                    capture_scan_hold(&hold_scan_L, &hold_scan_R);
                    apply_scan_hold(hold_scan_L, hold_scan_R);
                    state = STATE_EXTINGUISH;
                    robot_state_set_extinguish();
                    lost_fire_since_tick = 0;
                } else if (fire_l) {
                    capture_scan_hold(&hold_scan_L, &hold_scan_R);
                    apply_scan_hold(hold_scan_L, hold_scan_R);
                    state = STATE_EXTINGUISH;
                    robot_state_set_extinguish();
                    lost_fire_since_tick = 0;
                } else if (fire_r) {
                    capture_scan_hold(&hold_scan_L, &hold_scan_R);
                    apply_scan_hold(hold_scan_L, hold_scan_R);
                    state = STATE_EXTINGUISH;
                    robot_state_set_extinguish();
                    lost_fire_since_tick = 0;
                } else {
                    scan_angle_L = scan_angle_R = ANGLE_CENTER;
                    sweep_dir = 1;
                    servos_reset_neutral();
                    state = STATE_PATROL;
                    fire_stable_loops = 0;
                }
                break;
#endif

            case STATE_ALIGN_LEFT_NOZZLE: {
                const float tgt = NOZZLE_ALIGN_TARGET_DEG;
                float err = hold_scan_L - tgt;

                robot_state_set_extinguish();
                fpv_lock_fixed();

                if (fabsf(err) <= LEFT_IR_ALIGN_EPS_DEG) {
                    hold_scan_L = tgt;
                    apply_scan_hold(hold_scan_L, hold_scan_R);
                    align_iterate_steps = 0;
                    state = STATE_EXTINGUISH;
                    robot_state_set_extinguish();
                    ESP_LOGI(TAG, "Căn IR trái: đạt %.0f° (±%.1f°)", (double)tgt,
                             (double)LEFT_IR_ALIGN_EPS_DEG);
                    break;
                }
                if (align_iterate_steps >= ALIGN_MAX_STEPS) {
                    hold_scan_L = tgt;
                    apply_scan_hold(hold_scan_L, hold_scan_R);
                    align_iterate_steps = 0;
                    state = STATE_EXTINGUISH;
                    robot_state_set_extinguish();
                    ESP_LOGW(TAG, "Căn IR trái: hết bước tối đa → gán L=%.0f°", (double)tgt);
                    break;
                }

                align_iterate_steps++;
                /* err>0: lửa bên phải → trái tiến phải lùi; err<0: lửa bên trái → phải tiến trái lùi */
                if (err > 0.f) {
                    motor_turn_right(ALIGN_SPIN_DUTY);
                } else {
                    motor_turn_left(ALIGN_SPIN_DUTY);
                }
                vTaskDelay(pdMS_TO_TICKS(ALIGN_MOTOR_PULSE_MS));
                motor_stop();
                vTaskDelay(pdMS_TO_TICKS(25));

                const float step = ALIGN_SERVO_STEP_DEG;
                if (err < 0.f) {
                    hold_scan_L += step;
                    if (hold_scan_L > tgt) {
                        hold_scan_L = tgt;
                    }
                } else {
                    hold_scan_L -= step;
                    if (hold_scan_L < tgt) {
                        hold_scan_L = tgt;
                    }
                }
                apply_scan_hold(hold_scan_L, hold_scan_R);
                break;
            }

            case STATE_EXTINGUISH:
                motor_stop();

                read_flame_inputs(&fire_l, &fire_r);
                {
                    bool ir_any = fire_l || fire_r;
                    bool cam_ok = robot_state_ai_camera_fresh_ok(RELAY_MIN_CAMERA_CONF);

                    apply_scan_hold(hold_scan_L, hold_scan_R);

                    if (fire_l && cam_ok) {
                        relay_on();
                    } else {
                        relay_off();
                    }

                    buzzer_beep_tick(ir_any);
                }
                robot_state_set_extinguish();

                if (!fire_l && !fire_r) {
                    if (lost_fire_since_tick == 0) {
                        lost_fire_since_tick = now;
                    } else if ((now - lost_fire_since_tick) >= pdMS_TO_TICKS(LOST_FIRE_MS)) {
                        relay_off();
                        scan_angle_L = scan_angle_R = ANGLE_CENTER;
                        sweep_dir = 1;
                        servos_reset_neutral();
                        path_blocked = false;
                        lost_fire_since_tick = 0;
                        fire_stable_loops = 0;
                        ESP_LOGI(TAG, "Mất IR ~%d ms — tắt relay, tuần tra.", LOST_FIRE_MS);
                        state = STATE_PATROL;
                        robot_state_set_idle_scan();
                    }
                } else {
                    lost_fire_since_tick = 0;
                }
                break;
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void app_main(void) {
    wifi_init_station();

    buzzer_init();
    relay_init();
    frame_sensor_init();
    frame_sensor_start_monitoring(300);
    hc_sr04_init();
    hc_sr04_start_monitoring(200);
    /* Servo trước motor: cùng LEDC — timer 0 (servo) và timer 1 (motor) tách trong board_hw.h */
    servo_init();
    motor_init();
    board_hw_log_layout();

    xTaskCreatePinnedToCore(fire_control_task, "fire_ctrl", 1024 * 10, NULL, 10, NULL, 0);
}
