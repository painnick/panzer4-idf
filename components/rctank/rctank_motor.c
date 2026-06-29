/**
 * @file rctank_motor.c
 * @brief RC Tank MCPWM 트랙/터렛 제어
 */
#include "rctank_motor.h"
#include "rctank_pins.h"

#include "driver/mcpwm_prelude.h"
#include "esp_log.h"
#include "esp_check.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

static const char *TAG = "rctank_motor";

#define MCPWM_FREQ_HZ          (20000)
#define MCPWM_RESOLUTION_HZ   (1000000)
#define MCPWM_PERIOD_TICKS    (MCPWM_RESOLUTION_HZ / MCPWM_FREQ_HZ)

/* 스틱 범위 ±512 → 듀티 비율 (0~100%) */
#define AXIS_MAX              512

#define TRACK_DECEL_STEP 16  // 10ms 당 감속 단계 (실질 범위 448~512에서 약 400ms 내에 정지, 10배 스케일)
#define TRACK_ACCEL_STEP 6   // 10ms 당 가속 단계 (실질 범위 448~512에서 약 1000ms 내에 최대 속도 도달, 10배 스케일)
#define TRACK_MIN_SPEED  448 // 모터 구동을 개시하는 최소 기동 속도 오프셋 (0~512 범위 내)
#define TRACK_STICK_THRESHOLD 460  // 90% 스틱 경계값 (512 * 0.90)
#define TRACK_EXPO_THRESHOLD  178  // 90% 경계값에서의 모터 중간 속도 목표치 (최대 속도의 약 91%가 되도록 조정)

static mcpwm_cmpr_handle_t left_cmpr_a = NULL;
static mcpwm_cmpr_handle_t left_cmpr_b = NULL;
static mcpwm_cmpr_handle_t right_cmpr_a = NULL;
static mcpwm_cmpr_handle_t right_cmpr_b = NULL;
static mcpwm_cmpr_handle_t turret_cmpr_a = NULL;
static mcpwm_cmpr_handle_t turret_cmpr_b = NULL;

static mcpwm_timer_handle_t timer0 = NULL;

static volatile int32_t target_left_speed = 0;
static volatile int32_t target_right_speed = 0;
static int32_t current_left_speed = 0;
static int32_t current_right_speed = 0;

static SemaphoreHandle_t motor_mutex = NULL;
static TaskHandle_t motor_task_handle = NULL;

static void set_motor_duty(mcpwm_cmpr_handle_t cmpr_a, mcpwm_cmpr_handle_t cmpr_b, int32_t speed);

static void update_motor_speed(int32_t *current, int32_t target)
{
    int32_t cur = *current;
    if (cur == target) {
        return;
    }

    // 1. 0에서 기동할 때 최소 구동 속도(TRACK_MIN_SPEED * 10)로 즉시 점프
    if (cur == 0 && target != 0) {
        int32_t sign = (target > 0) ? 1 : -1;
        int32_t start_speed = sign * TRACK_MIN_SPEED * 10;
        
        int32_t target_abs = (target > 0) ? target : -target;
        // target 절대값이 최소 기동 속도 이하이면 즉시 target으로 설정 후 리턴
        if (target_abs <= TRACK_MIN_SPEED * 10) {
            *current = target;
            return;
        }
        cur = start_speed;
        *current = cur;
    }

    int32_t diff = target - cur;
    int32_t step;

    // Determine if we are accelerating or decelerating
    bool is_decelerating = false;
    
    if (cur > 0) {
        if (target < cur) {
            is_decelerating = true;
        }
    } else if (cur < 0) {
        if (target > cur) {
            is_decelerating = true;
        }
    } else {
        is_decelerating = false;
    }

    if (is_decelerating) {
        step = TRACK_DECEL_STEP;
    } else {
        step = TRACK_ACCEL_STEP;
    }

    if (diff > 0) {
        if (diff > step) {
            *current = cur + step;
        } else {
            *current = target;
        }
    } else {
        if (-diff > step) {
            *current = cur - step;
        } else {
            *current = target;
        }
    }

    // 2. 감속 정지 시 최소 구동 속도 이하로 떨어지면 웅웅거림 방지를 위해 즉시 0으로 정지
    if (target == 0) {
        int32_t cur_abs = (*current > 0) ? *current : -*current;
        if (cur_abs < TRACK_MIN_SPEED * 10) {
            *current = 0;
        }
    }
}

static void rctank_motor_update_task(void *pvParameters)
{
    TickType_t last_wake_time = xTaskGetTickCount();
    while (1) {
        if (motor_mutex && xSemaphoreTake(motor_mutex, portMAX_DELAY) == pdTRUE) {
            update_motor_speed(&current_left_speed, target_left_speed * 10);
            if (left_cmpr_a && left_cmpr_b) {
                set_motor_duty(left_cmpr_a, left_cmpr_b, current_left_speed / 10);
            }

            update_motor_speed(&current_right_speed, target_right_speed * 10);
            if (right_cmpr_a && right_cmpr_b) {
                set_motor_duty(right_cmpr_a, right_cmpr_b, current_right_speed / 10);
            }
            xSemaphoreGive(motor_mutex);
        }
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(10));
    }
}

static void set_motor_duty(mcpwm_cmpr_handle_t cmpr_a, mcpwm_cmpr_handle_t cmpr_b, int32_t speed)
{
    uint32_t ticks;
    if (speed > 0) {
        ticks = ((uint32_t)speed * MCPWM_PERIOD_TICKS) / AXIS_MAX;
        if (ticks > MCPWM_PERIOD_TICKS) ticks = MCPWM_PERIOD_TICKS;
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(cmpr_a, ticks));
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(cmpr_b, 0));
    } else if (speed < 0) {
        ticks = ((uint32_t)(-speed) * MCPWM_PERIOD_TICKS) / AXIS_MAX;
        if (ticks > MCPWM_PERIOD_TICKS) ticks = MCPWM_PERIOD_TICKS;
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(cmpr_a, 0));
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(cmpr_b, ticks));
    } else {
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(cmpr_a, 0));
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(cmpr_b, 0));
    }
}

esp_err_t rctank_motor_init(void)
{
    esp_err_t ret;

    target_left_speed = 0;
    target_right_speed = 0;
    current_left_speed = 0;
    current_right_speed = 0;

    motor_mutex = xSemaphoreCreateMutex();
    if (!motor_mutex) {
        ESP_LOGE(TAG, "Failed to create motor mutex");
        return ESP_FAIL;
    }

    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = MCPWM_RESOLUTION_HZ,
        .period_ticks = MCPWM_RESOLUTION_HZ / MCPWM_FREQ_HZ,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    ret = mcpwm_new_timer(&timer_config, &timer0);
    ESP_RETURN_ON_ERROR(ret, TAG, "mcpwm_new_timer 0");

    mcpwm_oper_handle_t left_op = NULL;
    mcpwm_oper_handle_t right_op = NULL;
    mcpwm_oper_handle_t turret_op = NULL;
    mcpwm_operator_config_t op_config = {
        .group_id = 0,
    };

    ret = mcpwm_new_operator(&op_config, &left_op);
    ESP_RETURN_ON_ERROR(ret, TAG, "mcpwm_new_operator left");
    ret = mcpwm_new_operator(&op_config, &right_op);
    ESP_RETURN_ON_ERROR(ret, TAG, "mcpwm_new_operator right");
    ret = mcpwm_new_operator(&op_config, &turret_op);
    ESP_RETURN_ON_ERROR(ret, TAG, "mcpwm_new_operator turret");

    ret = mcpwm_operator_connect_timer(left_op, timer0);
    ESP_RETURN_ON_ERROR(ret, TAG, "connect timer left");
    ret = mcpwm_operator_connect_timer(right_op, timer0);
    ESP_RETURN_ON_ERROR(ret, TAG, "connect timer right");
    ret = mcpwm_operator_connect_timer(turret_op, timer0);
    ESP_RETURN_ON_ERROR(ret, TAG, "connect timer turret");

    mcpwm_comparator_config_t cmpr_config = {
        .flags.update_cmp_on_tez = true,
    };

    ret = mcpwm_new_comparator(left_op, &cmpr_config, &left_cmpr_a);
    ESP_RETURN_ON_ERROR(ret, TAG, "left_cmpr_a");
    ret = mcpwm_new_comparator(left_op, &cmpr_config, &left_cmpr_b);
    ESP_RETURN_ON_ERROR(ret, TAG, "left_cmpr_b");
    ret = mcpwm_new_comparator(right_op, &cmpr_config, &right_cmpr_a);
    ESP_RETURN_ON_ERROR(ret, TAG, "right_cmpr_a");
    ret = mcpwm_new_comparator(right_op, &cmpr_config, &right_cmpr_b);
    ESP_RETURN_ON_ERROR(ret, TAG, "right_cmpr_b");
    ret = mcpwm_new_comparator(turret_op, &cmpr_config, &turret_cmpr_a);
    ESP_RETURN_ON_ERROR(ret, TAG, "turret_cmpr_a");
    ret = mcpwm_new_comparator(turret_op, &cmpr_config, &turret_cmpr_b);
    ESP_RETURN_ON_ERROR(ret, TAG, "turret_cmpr_b");

    mcpwm_generator_config_t gen_config = {
        .gen_gpio_num = -1,
    };
    mcpwm_gen_handle_t left_gen_a, left_gen_b, right_gen_a, right_gen_b, turret_gen_a, turret_gen_b;

    gen_config.gen_gpio_num = RCTANK_PIN_LEFT_IN1;
    ret = mcpwm_new_generator(left_op, &gen_config, &left_gen_a);
    ESP_RETURN_ON_ERROR(ret, TAG, "left_gen_a");
    gen_config.gen_gpio_num = RCTANK_PIN_LEFT_IN2;
    ret = mcpwm_new_generator(left_op, &gen_config, &left_gen_b);
    ESP_RETURN_ON_ERROR(ret, TAG, "left_gen_b");

    gen_config.gen_gpio_num = RCTANK_PIN_RIGHT_IN1;
    ret = mcpwm_new_generator(right_op, &gen_config, &right_gen_a);
    ESP_RETURN_ON_ERROR(ret, TAG, "right_gen_a");
    gen_config.gen_gpio_num = RCTANK_PIN_RIGHT_IN2;
    ret = mcpwm_new_generator(right_op, &gen_config, &right_gen_b);
    ESP_RETURN_ON_ERROR(ret, TAG, "right_gen_b");

    gen_config.gen_gpio_num = RCTANK_PIN_TURRET_IN1;
    ret = mcpwm_new_generator(turret_op, &gen_config, &turret_gen_a);
    ESP_RETURN_ON_ERROR(ret, TAG, "turret_gen_a");
    gen_config.gen_gpio_num = RCTANK_PIN_TURRET_IN2;
    ret = mcpwm_new_generator(turret_op, &gen_config, &turret_gen_b);
    ESP_RETURN_ON_ERROR(ret, TAG, "turret_gen_b");

    mcpwm_generator_set_actions_on_timer_event(left_gen_a,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH),
        MCPWM_GEN_TIMER_EVENT_ACTION_END());
    mcpwm_generator_set_actions_on_compare_event(left_gen_a,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, left_cmpr_a, MCPWM_GEN_ACTION_LOW),
        MCPWM_GEN_COMPARE_EVENT_ACTION_END());
    mcpwm_generator_set_actions_on_timer_event(left_gen_b,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH),
        MCPWM_GEN_TIMER_EVENT_ACTION_END());
    mcpwm_generator_set_actions_on_compare_event(left_gen_b,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, left_cmpr_b, MCPWM_GEN_ACTION_LOW),
        MCPWM_GEN_COMPARE_EVENT_ACTION_END());

    mcpwm_generator_set_actions_on_timer_event(right_gen_a,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH),
        MCPWM_GEN_TIMER_EVENT_ACTION_END());
    mcpwm_generator_set_actions_on_compare_event(right_gen_a,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, right_cmpr_a, MCPWM_GEN_ACTION_LOW),
        MCPWM_GEN_COMPARE_EVENT_ACTION_END());
    mcpwm_generator_set_actions_on_timer_event(right_gen_b,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH),
        MCPWM_GEN_TIMER_EVENT_ACTION_END());
    mcpwm_generator_set_actions_on_compare_event(right_gen_b,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, right_cmpr_b, MCPWM_GEN_ACTION_LOW),
        MCPWM_GEN_COMPARE_EVENT_ACTION_END());

    mcpwm_generator_set_actions_on_timer_event(turret_gen_a,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH),
        MCPWM_GEN_TIMER_EVENT_ACTION_END());
    mcpwm_generator_set_actions_on_compare_event(turret_gen_a,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, turret_cmpr_a, MCPWM_GEN_ACTION_LOW),
        MCPWM_GEN_COMPARE_EVENT_ACTION_END());
    mcpwm_generator_set_actions_on_timer_event(turret_gen_b,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH),
        MCPWM_GEN_TIMER_EVENT_ACTION_END());
    mcpwm_generator_set_actions_on_compare_event(turret_gen_b,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, turret_cmpr_b, MCPWM_GEN_ACTION_LOW),
        MCPWM_GEN_COMPARE_EVENT_ACTION_END());

    ret = mcpwm_timer_enable(timer0);
    ESP_RETURN_ON_ERROR(ret, TAG, "timer0_enable");
    ret = mcpwm_timer_start_stop(timer0, MCPWM_TIMER_START_NO_STOP);
    ESP_RETURN_ON_ERROR(ret, TAG, "timer0_start");

    rctank_motor_left_track_set_immediate(0);
    rctank_motor_right_track_set_immediate(0);
    rctank_motor_turret_set(0);

    BaseType_t task_ret = xTaskCreatePinnedToCore(
        rctank_motor_update_task,
        "rctank_motor_update",
        4096,
        NULL,
        5,
        &motor_task_handle,
        1
    );
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create motor task");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "motor init ok");
    return ESP_OK;
}

static int32_t map_joystick_to_motor_speed(int32_t speed)
{
    if (speed == 0) return 0;
    
    int32_t sign = (speed > 0) ? 1 : -1;
    int32_t abs_speed = (speed > 0) ? speed : -speed;
    int32_t expo_speed;
    
    // 75% 스틱 경계를 기준으로 분할 선형 매핑 적용 (75% 이전은 느리게, 이후는 빠르게)
    if (abs_speed <= TRACK_STICK_THRESHOLD) {
        expo_speed = (abs_speed * TRACK_EXPO_THRESHOLD) / TRACK_STICK_THRESHOLD;
    } else {
        expo_speed = TRACK_EXPO_THRESHOLD + ((abs_speed - TRACK_STICK_THRESHOLD) * (AXIS_MAX - TRACK_EXPO_THRESHOLD)) / (AXIS_MAX - TRACK_STICK_THRESHOLD);
    }
    
    // 2. Add minimum start-up speed offset
    int32_t mapped = TRACK_MIN_SPEED + (expo_speed * (AXIS_MAX - TRACK_MIN_SPEED)) / AXIS_MAX;
    
    return sign * mapped;
}

void rctank_motor_left_track_set(int32_t speed)
{
    int32_t mapped_speed = map_joystick_to_motor_speed(speed);
    if (motor_mutex && xSemaphoreTake(motor_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        target_left_speed = mapped_speed;
        xSemaphoreGive(motor_mutex);
    }
}

void rctank_motor_left_track_set_immediate(int32_t speed)
{
    if (motor_mutex && xSemaphoreTake(motor_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        target_left_speed = speed;
        current_left_speed = speed * 10;
        if (left_cmpr_a && left_cmpr_b) {
            set_motor_duty(left_cmpr_a, left_cmpr_b, speed);
        }
        xSemaphoreGive(motor_mutex);
    }
}

void rctank_motor_right_track_set(int32_t speed)
{
    int32_t mapped_speed = map_joystick_to_motor_speed(speed);
    if (motor_mutex && xSemaphoreTake(motor_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        target_right_speed = mapped_speed;
        xSemaphoreGive(motor_mutex);
    }
}

void rctank_motor_right_track_set_immediate(int32_t speed)
{
    if (motor_mutex && xSemaphoreTake(motor_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        target_right_speed = speed;
        current_right_speed = speed * 10;
        if (right_cmpr_a && right_cmpr_b) {
            set_motor_duty(right_cmpr_a, right_cmpr_b, speed);
        }
        xSemaphoreGive(motor_mutex);
    }
}

void rctank_motor_turret_set(int32_t speed)
{
    if (turret_cmpr_a && turret_cmpr_b) {
        set_motor_duty(turret_cmpr_a, turret_cmpr_b, speed);
    }
}
