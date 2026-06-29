/**
 * @file rctank_led.h
 * @brief RC Tank LED 제어 (포신 LED, 헤드라이트)
 */
#ifndef RCTANK_LED_H
#define RCTANK_LED_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t rctank_led_init(void);

/** 포신 LED (B 버튼 발사 시 점등) */
void rctank_led_gun_set(int on);

/** 기관총 LED (A 버튼 발사 시 1초간 깜빡임용 on/off) */
void rctank_led_mg_set(int on);

/** 헤드라이트 (Y 버튼 토글) */
void rctank_led_headlight_set(int on);
int rctank_led_headlight_get(void);

#ifdef __cplusplus
}
#endif

#endif /* RCTANK_LED_H */
