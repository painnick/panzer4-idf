/**
 * @file rctank_pins.h
 * @brief RC Tank 핀 정의 (README.md 기준)
 */
#ifndef RCTANK_PINS_H
#define RCTANK_PINS_H

#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/* LED */
#define RCTANK_PIN_GUN_LED (GPIO_NUM_17)
#define RCTANK_PIN_HEADLIGHT (GPIO_NUM_5)
#define RCTANK_PIN_MG_LED (GPIO_NUM_16) /* 기관총 LED (A 버튼, 1초간 깜빡임) */
#define RCTANK_PIN_BACKLIGHT (GPIO_NUM_4)

/* 좌측 트랙 DRV8833 (AIN1, AIN2) */
#define RCTANK_PIN_LEFT_IN1 (GPIO_NUM_27)
#define RCTANK_PIN_LEFT_IN2 (GPIO_NUM_26)

/* 우측 트랙 DRV8833 (BIN1, BIN2) */
#define RCTANK_PIN_RIGHT_IN1 (GPIO_NUM_33)
#define RCTANK_PIN_RIGHT_IN2 (GPIO_NUM_25)

/* 터렛 DRV8833 (MCPWM 제어) */
#define RCTANK_PIN_TURRET_IN1 (GPIO_NUM_22)
#define RCTANK_PIN_TURRET_IN2 (GPIO_NUM_21)

/* 포 마운트 서보 (SG90) */
#define RCTANK_PIN_SERVO_ELEVATION (GPIO_NUM_13)

/* 서보: 포신(당기기) */
#define RCTANK_PIN_SERVO_RECOIL (GPIO_NUM_32)

/* Sound (JQ6900/JQ6500) UART TX */
#define RCTANK_PIN_SOUND_TX (GPIO_NUM_23)
#define RCTANK_PIN_SOUND_RX (GPIO_NUM_NC)

#ifdef __cplusplus
}
#endif

#endif /* RCTANK_PINS_H */
