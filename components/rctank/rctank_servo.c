/**
 * @file rctank_servo.c
 * @brief RC Tank LEDC 서보 제어
 */
#include "rctank_servo.h"
#include "rctank_pins.h"

#include "driver/ledc.h"
#include "esp_log.h"

static const char* TAG = "rctank_servo";

#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES LEDC_TIMER_10_BIT
#define LEDC_FREQ_HZ 50

#define SERVO_PULSE_MIN_US 500
#define SERVO_PULSE_MAX_US 2500
#define SERVO_DEGREE_RANGE 180

static uint32_t degree_to_duty(int degree) {
    if (degree < 0)
        degree = 0;
    if (degree > SERVO_DEGREE_RANGE)
        degree = SERVO_DEGREE_RANGE;
    uint32_t us =
        SERVO_PULSE_MIN_US + (SERVO_PULSE_MAX_US - SERVO_PULSE_MIN_US) * (uint32_t)degree / SERVO_DEGREE_RANGE;
    uint32_t max_duty = (1 << LEDC_DUTY_RES) - 1;
    return (us * LEDC_FREQ_HZ * max_duty) / 1000000;
}

esp_err_t rctank_servo_init(void) {
    ledc_timer_config_t timer_config = {
        .speed_mode = LEDC_MODE,
        .duty_resolution = LEDC_DUTY_RES,
        .timer_num = LEDC_TIMER,
        .freq_hz = LEDC_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    esp_err_t ret = ledc_timer_config(&timer_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ledc_timer_config %s", esp_err_to_name(ret));
        return ret;
    }

    ledc_channel_config_t ch_config = {
        .gpio_num = RCTANK_PIN_SERVO_RECOIL,
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER,
        .duty = degree_to_duty(RCTANK_SERVO_GUN_DEG_REST),
        .hpoint = 0,
    };
    ret = ledc_channel_config(&ch_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ledc_channel_config recoil %s", esp_err_to_name(ret));
        return ret;
    }

    ledc_channel_config_t elevation_ch_config = {
        .gpio_num = RCTANK_PIN_SERVO_ELEVATION,
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL_1,
        .timer_sel = LEDC_TIMER,
        .duty = degree_to_duty(RCTANK_SERVO_ELEVATION_DEG_REST),
        .hpoint = 0,
    };
    ret = ledc_channel_config(&elevation_ch_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ledc_channel_config elevation %s", esp_err_to_name(ret));
        return ret;
    }

    rctank_servo_gun_set_degree(RCTANK_SERVO_GUN_DEG_REST);
    rctank_servo_elevation_set_degree(RCTANK_SERVO_ELEVATION_DEG_REST);
    ESP_LOGI(TAG, "servo init ok (recoil pin %d, elevation pin %d)", RCTANK_PIN_SERVO_RECOIL, RCTANK_PIN_SERVO_ELEVATION);
    return ESP_OK;
}

static int s_gun_degree = RCTANK_SERVO_GUN_DEG_REST;
static int s_elevation_degree = RCTANK_SERVO_ELEVATION_DEG_REST;

void rctank_servo_gun_set_degree(int degree) {
    if (degree < 0)
        degree = 0;
    if (degree > SERVO_DEGREE_RANGE)
        degree = SERVO_DEGREE_RANGE;
    s_gun_degree = degree;
    uint32_t duty = degree_to_duty(degree);
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_0);
}

void rctank_servo_gun_enable(int enable) {
    if (enable) {
        /* 저장된 각도로 duty 복원 */
        rctank_servo_gun_set_degree(s_gun_degree);
    } else {
        /* PWM 중지 -> 서보 힘 풀림 */
        ledc_stop(LEDC_MODE, LEDC_CHANNEL_0, 0);
    }
}

void rctank_servo_elevation_set_degree(int degree) {
    if (degree < RCTANK_SERVO_ELEVATION_MIN)
        degree = RCTANK_SERVO_ELEVATION_MIN;
    if (degree > RCTANK_SERVO_ELEVATION_MAX)
        degree = RCTANK_SERVO_ELEVATION_MAX;
    s_elevation_degree = degree;
    uint32_t duty = degree_to_duty(degree);
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_1, duty);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_1);
}

void rctank_servo_elevation_enable(int enable) {
    if (enable) {
        rctank_servo_elevation_set_degree(s_elevation_degree);
    } else {
        ledc_stop(LEDC_MODE, LEDC_CHANNEL_1, 0);
    }
}
