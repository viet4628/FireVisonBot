/*
 * FireVisionBot – Main Control Logic
 *
 * State machine:
 *   IDLE_SCAN  →  FIRE_DETECTED  →  ORIENT_CAMERA  →  (AI_VERIFY)  →  APPROACH  →  EXTINGUISH
 *                                                                         ↑              │
 *                                                                         └──────────────┘
 *                                                       (flame lost)  → IDLE_SCAN
 *
 * Hardware controlled:
 *   - 4 DC motors via L298N (2 channels, each channel drives 2 motors in parallel)
 *   - 2 scan servos (left / right) with IR flame sensors
 *   - 2 FPV servos (pan / tilt) for ESP32-CAM
 *   - 1 relay for water pump
 *   - 1 buzzer (alarm)
 *   - 1 HC-SR04 ultrasonic sensor (obstacle / target distance)
 */

#include "motor.h"
#include "servo.h"
#include "frame_sensor.h"
#include "hc_sr04.h"
#include "relay.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "wifi_ap.h" // Added by user instruction
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "FIRE_CTRL";

/* ───────── Buzzer ───────── */
#define BUZZER_GPIO     GPIO_NUM_16
#define BUZZER_ON_LEVEL 1   /* Còi của bạn kêu khi cấp 1 (Active HIGH), không phải cấp 0 */

static void buzzer_init(void)
{
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

static void buzzer_set(bool on)
{
    gpio_set_level(BUZZER_GPIO, on ? BUZZER_ON_LEVEL : !BUZZER_ON_LEVEL);
}

/* Non-blocking "tíc tắc" beep pattern (call every loop tick) */
static void buzzer_beep_tick(bool active)
{
    static TickType_t next_toggle = 0;
    static bool buzzer_on = false;
    /* Short beep (60 ms) + long pause (400 ms) → clear tick-tock sound */
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

/* ───────── Helpers ───────── */
static float clamp_angle(float angle, float min_a, float max_a)
{
    if (angle < min_a) return min_a;
    if (angle > max_a) return max_a;
    return angle;
}

/* Map scan angle to mirrored angle for the opposite-facing servo */
static float mirror_angle(float angle, float min_a, float max_a)
{
    return max_a - (angle - min_a);
}

/* ───────── Robot States ───────── */
typedef enum {
    STATE_IDLE_SCAN,       /* Sleeping – scan servos sweep, looking for fire       */
    STATE_TURN_AROUND,     /* Right sensor triggered – rotate robot 180 deg to face fire */
    STATE_FIRE_DETECTED,   /* IR confirmed fire – lock servo, compute direction    */
    STATE_ORIENT_CAMERA,   /* Turn FPV camera toward fire direction                */
    STATE_AI_VERIFY,       /* (Placeholder) ESP32-CAM AI fire verification         */
    STATE_APPROACH,        /* Drive toward fire, keep tracking with IR sensors     */
    STATE_EXTINGUISH,      /* Close enough – stop, activate water pump             */
} robot_state_t;

/* ───────── Main Control Task ───────── */
static void fire_control_task(void *arg)
{
    (void)arg;

    /* Scan parameters */
    const float SCAN_MIN   = -15.0f;
    const float SCAN_MAX   = 180.0f;
    const float SCAN_STEP  = 0.8f;   /* slower sweep for better detection */
    float scan_angle       = SCAN_MIN;
    int   scan_dir         = 1;

    /* Tracking parameters */
    const float TRACK_STEP       = 0.9f;
    const float STOP_DISTANCE_CM = 25.0f;
    const uint32_t MOTOR_DUTY    = 700;
    const uint32_t TURN_DUTY     = 950;  /* Tăng tốc độ quay tại chỗ (Max là 1023) */

    /* Timing */
    const TickType_t LOCK_DELAY   = pdMS_TO_TICKS(300);
    const TickType_t LOST_DELAY   = pdMS_TO_TICKS(3000);

    /* State variables */
    robot_state_t state = STATE_IDLE_SCAN;
    float  target_angle  = 90.0f;
    uint8_t left_streak  = 0;
    uint8_t right_streak = 0;
    const uint8_t CONFIRM_STREAK = 2;

    /* Camera center position (90° means straight ahead) */
    const float CAMERA_CENTER = 90.0f;
    const float FPV_TILT_DEFAULT = 30.0f;
    
    /* Vị trí gập nằm gọn lại khi không có lửa (Mặc định) */
    const float FPV_PAN_FOLDED = 90.0f;   /* Nằm thẳng chính giữa */
    const float FPV_TILT_FOLDED = 0.0f;  /* Nằm thẳng */

    ESP_LOGI(TAG, "Fire control task started");

    while (1) {
        switch (state) {

        /* ────────────────────────────────────────────
         * IDLE SCAN – sweep IR sensors looking for fire
         * ──────────────────────────────────────────── */
        case STATE_IDLE_SCAN: {
            motor_stop();
            relay_off();
            buzzer_beep_tick(false);

            /* Gập trục FPV nằm gọn lại khi chưa có lửa */
            servo_set_angle(SERVO_FPV_PAN, FPV_PAN_FOLDED);
            servo_set_angle(SERVO_FPV_TILT, FPV_TILT_FOLDED);

            /* Sweep scan servos (two sensors face opposite directions) */
            servo_set_angle(SERVO_SCAN_LEFT,  scan_angle);
            servo_set_angle(SERVO_SCAN_RIGHT, mirror_angle(scan_angle, SCAN_MIN, SCAN_MAX));

            /* Check IR flame sensors */
            if (frame_sensor_is_fire_detected(FLAME_SENSOR_LEFT)) {
                ESP_LOGI(TAG, "LEFT sensor triggered, confirming...");
                vTaskDelay(LOCK_DELAY);
                if (frame_sensor_is_fire_detected(FLAME_SENSOR_LEFT)) {
                    target_angle = servo_get_angle(SERVO_SCAN_LEFT);
                    state = STATE_FIRE_DETECTED;
                    ESP_LOGI(TAG, "Fire LOCKED from LEFT at %.1f deg", target_angle);
                }
            } else if (frame_sensor_is_fire_detected(FLAME_SENSOR_RIGHT)) {
                ESP_LOGI(TAG, "RIGHT sensor triggered, confirming...");
                vTaskDelay(LOCK_DELAY);
                if (frame_sensor_is_fire_detected(FLAME_SENSOR_RIGHT)) {
                    target_angle = servo_get_angle(SERVO_SCAN_RIGHT);
                    state = STATE_TURN_AROUND;
                    ESP_LOGI(TAG, "Fire detected from RIGHT (back). Turning around 180 deg...");
                }
            } else {
                /* No fire – keep sweeping */
                scan_angle += scan_dir * SCAN_STEP;
                if (scan_angle >= SCAN_MAX || scan_angle <= SCAN_MIN) {
                    scan_dir *= -1;
                }
            }
            break;
        }

        /* ────────────────────────────────────────────
         * TURN AROUND – Right sensor triggered, rotate until left sensor sees it
         * ──────────────────────────────────────────── */
        case STATE_TURN_AROUND: {
            buzzer_beep_tick(true);
            motor_turn_left(TURN_DUTY);
            
            /* Sweep the left scan servo back to center to catch the fire */
            servo_set_angle(SERVO_SCAN_LEFT, CAMERA_CENTER);

            /* If left sensor detects fire, we've turned enough (roughly 180) */
            if (frame_sensor_is_fire_detected(FLAME_SENSOR_LEFT)) {
                motor_stop();
                target_angle = servo_get_angle(SERVO_SCAN_LEFT);
                state = STATE_FIRE_DETECTED;
                ESP_LOGI(TAG, "Turn complete. Fire LOCKED from LEFT at %.1f deg", target_angle);
            }
            break;
        }

        /* ────────────────────────────────────────────
         * FIRE DETECTED – lock scan servo, prepare to orient camera
         * ──────────────────────────────────────────── */
        case STATE_FIRE_DETECTED: {
            /* Lock both scan servos at fire direction */
            servo_set_angle(SERVO_SCAN_LEFT,  target_angle);
            servo_set_angle(SERVO_SCAN_RIGHT, mirror_angle(target_angle, SCAN_MIN, SCAN_MAX));

            /* Start buzzer warning */
            buzzer_beep_tick(true);

            ESP_LOGI(TAG, "Orienting camera toward fire (target=%.1f deg)", target_angle);
            state = STATE_ORIENT_CAMERA;
            break;
        }

        /* ────────────────────────────────────────────
         * ORIENT CAMERA – turn FPV servos to face fire direction
         * If fire is NOT straight ahead (90°), first rotate robot body
         * ──────────────────────────────────────────── */
        case STATE_ORIENT_CAMERA: {
            buzzer_beep_tick(true);

            /*
             * Point BOTH the pan (dưới) and tilt (trên) servos slowly toward their target angles.
             */
            float current_pan  = servo_get_angle(SERVO_FPV_PAN);
            float current_tilt = servo_get_angle(SERVO_FPV_TILT);
            
            float diff_pan  = target_angle - current_pan;
            float diff_tilt = FPV_TILT_DEFAULT - current_tilt;
            
            const float MOVE_STEP = 0.8f; /* Rất chậm và mượt (Chỉnh nhỏ lại nếu muốn chậm hơn) */
            
            bool pan_done = false;
            bool tilt_done = false;

            /* Xoay ngang (Servo Dưới - Pan) */
            if (diff_pan > MOVE_STEP) {
                servo_set_angle(SERVO_FPV_PAN, current_pan + MOVE_STEP);
            } else if (diff_pan < -MOVE_STEP) {
                servo_set_angle(SERVO_FPV_PAN, current_pan - MOVE_STEP);
            } else {
                servo_set_angle(SERVO_FPV_PAN, target_angle);
                pan_done = true;
            }

            /* Xoay dọc (Servo Trên - Tilt) */
            if (diff_tilt > MOVE_STEP) {
                servo_set_angle(SERVO_FPV_TILT, current_tilt + MOVE_STEP);
            } else if (diff_tilt < -MOVE_STEP) {
                servo_set_angle(SERVO_FPV_TILT, current_tilt - MOVE_STEP);
            } else {
                servo_set_angle(SERVO_FPV_TILT, FPV_TILT_DEFAULT);
                tilt_done = true;
            }

            /* Target reached for both axes */
            if (pan_done && tilt_done) {
                ESP_LOGI(TAG, "Camera oriented. Starting AI verification (placeholder)...");
                state = STATE_AI_VERIFY;
            }
            break;
        }

        /* ────────────────────────────────────────────
         * AI VERIFY – placeholder for ESP32-CAM fire verification
         * TODO: receive frame from ESP32-CAM, run TFLite FOMO model
         * ──────────────────────────────────────────── */
        case STATE_AI_VERIFY: {
            buzzer_beep_tick(true);

            /*
             * === PLACEHOLDER ===
             * When ESP32-CAM communication is ready:
             *   1. Request a JPEG frame from ESP32-CAM
             *   2. Run fire_model_int8.tflite on the frame
             *   3. If fire detected  → STATE_APPROACH
             *      If NOT fire       → STATE_IDLE_SCAN (false alarm)
             *
             * For now, auto-confirm fire and proceed.
             */
            ESP_LOGI(TAG, "AI verification PASSED (placeholder)");
            state = STATE_APPROACH;
            left_streak = 0;
            right_streak = 0;
            break;
        }

        /* ────────────────────────────────────────────
         * APPROACH – drive toward the fire, keep tracking with IR
         * ──────────────────────────────────────────── */
        case STATE_APPROACH: {
            bool left  = frame_sensor_is_fire_detected(FLAME_SENSOR_LEFT);
            bool right = frame_sensor_is_fire_detected(FLAME_SENSOR_RIGHT);
            bool fire  = left || right;
            buzzer_beep_tick(fire);

            /* Dynamic re-lock: adjust target_angle as flame moves */
            if (left && !right) {
                left_streak  = (left_streak < 255) ? (left_streak + 1) : left_streak;
                right_streak = 0;
            } else if (!left && right) {
                right_streak = (right_streak < 255) ? (right_streak + 1) : right_streak;
                left_streak  = 0;
            } else {
                left_streak  = 0;
                right_streak = 0;
            }

            if (left_streak >= CONFIRM_STREAK && left && !right) {
                target_angle = clamp_angle(target_angle - TRACK_STEP, SCAN_MIN, SCAN_MAX);
            } else if (right_streak >= CONFIRM_STREAK && !left && right) {
                target_angle = clamp_angle(target_angle + TRACK_STEP, SCAN_MIN, SCAN_MAX);
            }

            /* Update all servos to face the fire */
            servo_set_angle(SERVO_SCAN_LEFT,  target_angle);
            servo_set_angle(SERVO_SCAN_RIGHT, mirror_angle(target_angle, SCAN_MIN, SCAN_MAX));
            servo_set_angle(SERVO_FPV_PAN,    target_angle);

            /* Differential steering: if fire is not straight ahead, turn robot */
            float angle_error = target_angle - CAMERA_CENTER;
            if (angle_error < -15.0f) {
                /* Fire is to the left – turn left */
                motor_turn_left(TURN_DUTY);
            } else if (angle_error > 15.0f) {
                /* Fire is to the right – turn right */
                motor_turn_right(TURN_DUTY);
            } else {
                /* Fire roughly ahead */
                if (hc_sr04_is_target_near(STOP_DISTANCE_CM)) {
                    motor_stop();
                    state = STATE_EXTINGUISH;
                    ESP_LOGI(TAG, "Target reached (%.2f cm) – EXTINGUISH!",
                             hc_sr04_get_last_distance_cm());
                    break;
                }
                if (fire) {
                    motor_forward(MOTOR_DUTY);
                }
            }

            /* Obstacle avoidance while approaching */
            if (state == STATE_APPROACH && hc_sr04_is_target_near(STOP_DISTANCE_CM)) {
                motor_stop();
                state = STATE_EXTINGUISH;
                ESP_LOGI(TAG, "Target reached (%.2f cm) – EXTINGUISH!",
                         hc_sr04_get_last_distance_cm());
                break;
            }

            /* Check if fire is lost */
            if (!left && !right) {
                ESP_LOGI(TAG, "Flame lost during approach, waiting 3s...");
                motor_stop();
                vTaskDelay(LOST_DELAY);
                left  = frame_sensor_is_fire_detected(FLAME_SENSOR_LEFT);
                right = frame_sensor_is_fire_detected(FLAME_SENSOR_RIGHT);
                if (!left && !right) {
                    relay_off();
                    motor_stop();
                    buzzer_beep_tick(false);
                    state = STATE_IDLE_SCAN;
                    ESP_LOGI(TAG, "Flame confirmed lost – resume scanning");
                } else {
                    ESP_LOGI(TAG, "Flame re-detected, continuing approach");
                }
            }
            break;
        }

        /* ────────────────────────────────────────────
         * EXTINGUISH – activate water pump, keep tracking fire
         * ──────────────────────────────────────────── */
        case STATE_EXTINGUISH: {
            motor_stop();
            relay_on();   /* Activate water pump! */
            buzzer_beep_tick(true);

            bool left  = frame_sensor_is_fire_detected(FLAME_SENSOR_LEFT);
            bool right = frame_sensor_is_fire_detected(FLAME_SENSOR_RIGHT);

            /* Keep tracking fire direction while spraying */
            if (left && !right) {
                target_angle = clamp_angle(target_angle - TRACK_STEP, SCAN_MIN, SCAN_MAX);
            } else if (!left && right) {
                target_angle = clamp_angle(target_angle + TRACK_STEP, SCAN_MIN, SCAN_MAX);
            }
            servo_set_angle(SERVO_FPV_PAN, target_angle);

            /* Check if fire is extinguished */
            if (!left && !right) {
                ESP_LOGI(TAG, "Fire may be extinguished, verifying for 3s...");
                vTaskDelay(LOST_DELAY);
                left  = frame_sensor_is_fire_detected(FLAME_SENSOR_LEFT);
                right = frame_sensor_is_fire_detected(FLAME_SENSOR_RIGHT);
                if (!left && !right) {
                    relay_off();
                    buzzer_beep_tick(false);
                    state = STATE_IDLE_SCAN;
                    ESP_LOGI(TAG, "Fire extinguished! Resuming scan mode.");
                } else {
                    ESP_LOGI(TAG, "Fire still active, continuing spray");
                }
            }
            break;
        }

        } /* end switch */

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/* ───────── Entry Point ───────── */
void app_main(void)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "   FireVisionBot – Initialising...");
    ESP_LOGI(TAG, "========================================");

    /* Initialize WiFi Access Point for ESP32-CAM */
    ESP_LOGI(TAG, "Starting WiFi SoftAP...");
    wifi_init_softap();
    
    /* Initialise all subsystems */
    motor_init();
    ESP_LOGI(TAG, "[OK] Motor (L298N – 4 DC motors, 2 channels)");

    servo_init();
    ESP_LOGI(TAG, "[OK] Servo (Scan L=GPIO15, R=GPIO21 | FPV Pan=GPIO38, Tilt=GPIO39)");

    buzzer_init();
    ESP_LOGI(TAG, "[OK] Buzzer (GPIO16)");

    relay_init();
    ESP_LOGI(TAG, "[OK] Relay / Water pump (GPIO17)");

    frame_sensor_init();
    frame_sensor_start_monitoring(300);
    ESP_LOGI(TAG, "[OK] Flame IR sensors (L=GPIO4, R=GPIO6)");

    hc_sr04_init();
    hc_sr04_start_monitoring(200);
    ESP_LOGI(TAG, "[OK] HC-SR04 Ultrasonic (Trig=GPIO5, Echo=GPIO18)");

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "   All systems GO – starting controller");
    ESP_LOGI(TAG, "========================================");

    /* Start the main control task (8 KB stack for state machine) */
    xTaskCreate(fire_control_task, "fire_ctrl", 8192, NULL, 6, NULL);
}
