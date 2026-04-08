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

/* FPV servo dưới (PAN) — servo 180° thường
 * - Servo trên (TILT): để chết cứng, KHÔNG điều khiển.
 * - Servo dưới (PAN): mặc định 90°, có lửa -> bám theo góc servo IR trái.
 *//* Dịch góc nghỉ một chút tránh mòn chiết áp ở 90 độ gây nhiễu cơ học tự dao động */
#define FPV_BOTTOM_IDLE_DEG    92.f

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
/* Giảm riêng tốc độ lúc căn hướng (+30%: 820→1023, cap tại max). */
#define ALIGN_SPIN_DUTY        1023u

/** Xoay nhẹ bánh khi IR trái xác nhận (ms), 0 = tắt. Canh hướng thay vì xoay FPV. */
#define LEFT_FIRE_PIVOT_MS     0u

/** Số vòng lặp liên tiếp (mỗi ~20ms) có IR báo lửa mới coi là xác nhận — tránh nhiễu và tránh quét servo ghi đè. */
#define IR_FIRE_STABLE_LOOPS   14
#define LOST_FIRE_MS           3000

/* Bù góc xoay lố (15-20 độ) khi servo FPV bám theo góc của IR phải */
#define RIGHT_IR_FPV_OFFSET_DEG 10.0f

/* ─── Camera Approach: tiến tới theo camera ─── */
/** Ngưỡng confidence YOLO để kích hoạt chế độ tiếp cận bằng cámera. */
#define CAM_APPROACH_MIN_CONF      0.55f
/** Tốc độ tiến thẳng khi camera thấy lửa ở giữa. (+30%: 700→910) */
#define CAM_APPROACH_FWD_DUTY      910u
/** Tốc độ quay khi lửa lệch trái/phải chặn vật cản. (+30%: 600→780) */
#define CAM_APPROACH_TURN_DUTY     780u
/** Vùng chết giữa khung hình (x_ratio 0.5 ± DEADBAND tiến thẳng). */
#define CAM_APPROACH_X_DEADBAND    0.18f
/** Thời gian tối đa trong STATE_CAMERA_APPROACH trước khi tự quay về PATROL (ms). */
#define CAM_APPROACH_TIMEOUT_MS    8000u

typedef enum {
    STATE_PATROL,
    STATE_CAMERA_APPROACH,  /* Camera thấy lửa, IR chưa nhận — tiến tới */
    STATE_EXTINGUISH,
} robot_state_t;

static void fpv_set_idle(void) {
    // Servo trên (TILT): để chết cứng — không điều khiển.
    // Lọc nhiễu: tránh gọi hàm cập nhật liên tục mỗi 30ms khi robot đang quét không có lửa
    static TickType_t fpv_idle_start_tick = 0;
    float current_angle = servo_get_angle(SERVO_FPV_PAN);

    if (current_angle < -100.0f) {
        return; // Đã cắt PWM thì giữ im thả lỏng servo
    }

    if (fabsf(FPV_BOTTOM_IDLE_DEG - current_angle) > 3.0f) {
        servo_set_angle(SERVO_FPV_PAN, FPV_BOTTOM_IDLE_DEG);
        fpv_idle_start_tick = xTaskGetTickCount();
    } else {
        // Nếu đã lùi về tới góc Idle được 500ms, ta cắt toàn bộ xung PWM để servo nghỉ không giật
        if (fpv_idle_start_tick != 0 && (xTaskGetTickCount() - fpv_idle_start_tick) > pdMS_TO_TICKS(100)) {
            servo_detach(SERVO_FPV_PAN);
            fpv_idle_start_tick = 0;
        }
    }
}

static void fpv_set_fire_track(float scan_left_deg) {
    // Quay servo FPV cùng góc thực tế với servo IR trái (có thể trước đó bị ngược).
    float follow = scan_left_deg;
    if (follow < SERVO_ANGLE_MIN) {
        follow = SERVO_ANGLE_MIN;
    }
    if (follow > SERVO_ANGLE_MAX) {
        follow = SERVO_ANGLE_MAX;
    }
    // Servo trên (TILT): để chết cứng — không điều khiển.
    // Lọc nhiễu: chỉ cập nhật góc khi mức dao động góc mới vượt ngưỡng 2.5 độ để tránh giật
    float current_angle = servo_get_angle(SERVO_FPV_PAN);
    if (fabsf(follow - current_angle) > 3.0f) {
        servo_set_angle(SERVO_FPV_PAN, follow);
    }
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
    fpv_set_idle();
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
    fpv_set_idle();
}

/** Lưu góc thực tế đang ra lệnh cho 2 servo quét — khóa đúng tư thế lúc IR xác nhận, không nhảy về góc cứng. */
static void capture_scan_hold(float *hold_L, float *hold_R) {
    *hold_L = servo_get_angle(SERVO_SCAN_LEFT);
    *hold_R = servo_get_angle(SERVO_SCAN_RIGHT);
}

static void apply_scan_hold(float hold_L, float hold_R) {
    servo_set_angle(SERVO_SCAN_LEFT, hold_L);
    servo_set_angle(SERVO_SCAN_RIGHT, hold_R);
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
                    // Chỉ mới thấy lửa tức thời (có thể nhiễu): giữ FPV ở idle.
                    // Chuyển sang track thật sự sau khi vượt IR_FIRE_STABLE_LOOPS.
                    fpv_set_idle();
                    fire_stable_loops++;
                    motor_stop();
                    /* Không gọi peripheral_idle_sweep khi đang có lửa / đang đếm ổn định — tránh kéo servo khỏi góc khóa */
                    if (fire_stable_loops >= IR_FIRE_STABLE_LOOPS) {
                        fire_stable_loops = 0;
                        lost_fire_since_tick = 0;
                        if (fire_l && fire_r) {
                            capture_scan_hold(&hold_scan_L, &hold_scan_R);
                            apply_scan_hold(hold_scan_L, hold_scan_R);
                            fpv_set_fire_track((hold_scan_L + hold_scan_R) / 2.0f);
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
                            fpv_set_fire_track(hold_scan_R + RIGHT_IR_FPV_OFFSET_DEG);
                            ESP_LOGW(TAG, "IR phải (trái tắt) — khóa L=%.1f R=%.1f → sang STATE_EXTINGUISH.",
                                     hold_scan_L, hold_scan_R);
                            state = STATE_EXTINGUISH;
                            robot_state_set_extinguish();
                            break;
                        }
#endif
                        /* IR trái (đã loại cả hai và chỉ-phải): thay vì căn bánh, vào thẳng EXTINGUISH để khóa góc */
                        if (fire_l) {
                            capture_scan_hold(&hold_scan_L, &hold_scan_R);
                            apply_scan_hold(hold_scan_L, hold_scan_R);
                            fpv_set_fire_track(hold_scan_L);
                            ESP_LOGW(TAG, "IR trái — khóa L=%.1f R=%.1f → sang STATE_EXTINGUISH.",
                                     hold_scan_L, hold_scan_R);
                            state = STATE_EXTINGUISH;
                            robot_state_set_extinguish();
                            break;
                        }
                    }
                    break;
                }

                fire_stable_loops = 0;

                /* Camera thấy lửa nhưng IR chưa bắt được → vào chế độ tiếp cận */
                if (robot_state_ai_camera_fire_detected()) {
                    ESP_LOGW(TAG, "Camera thấy lửa (x=%.2f) nhưng IR chưa bắt → CAMERA_APPROACH.",
                             (double)robot_state_ai_camera_x_ratio());
                    state = STATE_CAMERA_APPROACH;
                    break;
                }

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


            case STATE_CAMERA_APPROACH: {
                /*
                 * Tiến cầm về phía lửa theo camera (differential drive — không xoay tại chỗ).
                 * Công thức: error = x - 0.5 (-0.5..+0.5)
                 *   left_duty  = BASE × (1 - error×STEER_GAIN)   [lửa bên trái → giảm bánh trái]
                 *   right_duty = BASE × (1 + error×STEER_GAIN)   [lửa bên phải → giảm bánh phải]
                 * Giá trị dưới MIN_DUTY vẫn giữ tiến (không về 0, để không xoay tại chỗ).
                 * Servo IR quét nhanh hơn bình thường trong lúc tiếp cận.
                 * Thoát:
                 *   - IR bắt được lửa → STATE_EXTINGUISH
                 *   - Camera mất tín hiệu hoặc timeout → STATE_PATROL
                 */
                static TickType_t approach_start_tick = 0;
                static TickType_t approach_scan_tick  = 0;
                if (approach_start_tick == 0) {
                    approach_start_tick = xTaskGetTickCount();
                    approach_scan_tick  = xTaskGetTickCount();
                    /* Reset vị trí servo về giữa để quét lại */
                    scan_angle_L = scan_angle_R = ANGLE_CENTER;
                    sweep_dir = 1;
                }

                read_flame_inputs(&fire_l, &fire_r);

                /* IR bắt được lửa → chuyển sang dập lửa */
                if (fire_l || fire_r) {
                    approach_start_tick = 0;
                    motor_stop();
                    fire_stable_loops++;
                    if (fire_stable_loops >= IR_FIRE_STABLE_LOOPS) {
                        fire_stable_loops = 0;
                        lost_fire_since_tick = 0;
                        capture_scan_hold(&hold_scan_L, &hold_scan_R);
                        apply_scan_hold(hold_scan_L, hold_scan_R);
                        float fpv_angle = fire_r && !fire_l
                            ? hold_scan_R + RIGHT_IR_FPV_OFFSET_DEG
                            : fire_l && fire_r
                                ? (hold_scan_L + hold_scan_R) / 2.0f
                                : hold_scan_L;
                        fpv_set_fire_track(fpv_angle);
                        ESP_LOGW(TAG, "APPROACH: IR bắt được lửa → EXTINGUISH.");
                        state = STATE_EXTINGUISH;
                        robot_state_set_extinguish();
                    }
                    break;
                }
                fire_stable_loops = 0;

                /* Camera mất tín hiệu hoặc timeout → quay về PATROL */
                bool cam_sees_fire = robot_state_ai_camera_fire_detected();
                TickType_t approach_elapsed = xTaskGetTickCount() - approach_start_tick;
                if (!cam_sees_fire || approach_elapsed > pdMS_TO_TICKS(CAM_APPROACH_TIMEOUT_MS)) {
                    approach_start_tick = 0;
                    motor_stop();
                    servos_reset_neutral();
                    scan_angle_L = scan_angle_R = ANGLE_CENTER;
                    sweep_dir = 1;
                    state = STATE_PATROL;
                    ESP_LOGW(TAG, "APPROACH: hết %s → PATROL.",
                             cam_sees_fire ? "thời gian" : "tín hiệu camera");
                    break;
                }

                /* ─ Servo IR: quét nhanh hơn bình thường ─ */
#define APPROACH_SCAN_TICK_MS   (SCAN_TICK_MS * 55 / 100)   /* ~55% kỳ chuẩn = nhanh hơn ~1.8x */
                if (xTaskGetTickCount() - approach_scan_tick >= pdMS_TO_TICKS(APPROACH_SCAN_TICK_MS)) {
                    approach_scan_tick = xTaskGetTickCount();
                    peripheral_idle_sweep_opposite(&scan_angle_L, &scan_angle_R, &sweep_dir);
                }

                /* ─ Differential drive — cả 2 bánh tiến, duty lệch nhau theo vị trí lửa ─ */
                robot_state_set_idle_scan();
                float x = robot_state_ai_camera_x_ratio();   /* 0.0=trái … 1.0=phải */
                float error = x - 0.5f;                       /* -0.5 … +0.5 */

                /* STEER_GAIN: độ nhạy lái. 1.6 → lượng lệch tối đa = 80% độ chầm / 120% duty */
                const float STEER_GAIN   = 1.6f;
                /* Duty tối thiểu mỗi bánh khi lửa lệch sang bên kia (để xe vẫn tiến, không xoay tại chỗ) */
                const uint32_t MIN_CURVE_DUTY = CAM_APPROACH_FWD_DUTY * 30 / 100;

                float l_f = (float)CAM_APPROACH_FWD_DUTY * (1.0f - error * STEER_GAIN);
                float r_f = (float)CAM_APPROACH_FWD_DUTY * (1.0f + error * STEER_GAIN);

                /* Giới hạn duty vào [MIN_CURVE_DUTY, DUTY_MAX] */
                if (l_f < (float)MIN_CURVE_DUTY) l_f = (float)MIN_CURVE_DUTY;
                if (r_f < (float)MIN_CURVE_DUTY) r_f = (float)MIN_CURVE_DUTY;

                uint32_t duty_l = (uint32_t)l_f;
                uint32_t duty_r = (uint32_t)r_f;

                if (path_blocked) {
                    /* Vật cản phía trước: xoay nhẹ về phía lửa để né vòng */
                    if (error < 0.0f) {
                        motor_turn_left(CAM_APPROACH_TURN_DUTY);
                    } else {
                        motor_turn_right(CAM_APPROACH_TURN_DUTY);
                    }
                } else {
                    motor_drive_curve(duty_l, duty_r);
                    ESP_LOGD(TAG, "APPROACH curve: x=%.2f L=%lu R=%lu",
                             (double)x, (unsigned long)duty_l, (unsigned long)duty_r);
                }
                break;
            }



            case STATE_EXTINGUISH:
                motor_stop();

                read_flame_inputs(&fire_l, &fire_r);
                {
                    bool ir_any = fire_l || fire_r;
                    bool cam_ok = robot_state_ai_camera_fresh_ok(RELAY_MIN_CAMERA_CONF);

                    apply_scan_hold(hold_scan_L, hold_scan_R);
                    if (fire_r && !fire_l) {
                        fpv_set_fire_track(hold_scan_R + RIGHT_IR_FPV_OFFSET_DEG);
                    } else if (fire_l && fire_r) {
                        fpv_set_fire_track((hold_scan_L + hold_scan_R) / 2.0f);
                    } else {
                        fpv_set_fire_track(hold_scan_L);
                    }

                    if ((fire_l || fire_r) && cam_ok) {
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
