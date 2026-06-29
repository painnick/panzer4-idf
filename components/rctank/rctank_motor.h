/**
 * @file rctank_motor.h
 * @brief RC Tank 트랙/터렛 MCPWM 제어
 */
#ifndef RCTANK_MOTOR_H
#define RCTANK_MOTOR_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 모터 드라이버 초기화 (MCPWM)
 * @return ESP_OK on success
 */
esp_err_t rctank_motor_init(void);

/**
 * @brief 좌측 트랙 속도 설정
 * @param speed -512 ~ 511 (음수: 후진, 양수: 전진, 0: 정지)
 */
void rctank_motor_left_track_set(int32_t speed);

/**
 * @brief 좌측 트랙 속도 즉시 설정 (가속/감속 램프 우회)
 * @param speed -512 ~ 511
 */
void rctank_motor_left_track_set_immediate(int32_t speed);

/**
 * @brief 우측 트랙 속도 설정
 * @param speed -512 ~ 511
 */
void rctank_motor_right_track_set(int32_t speed);

/**
 * @brief 우측 트랙 속도 즉시 설정 (가속/감속 램프 우회)
 * @param speed -512 ~ 511
 */
void rctank_motor_right_track_set_immediate(int32_t speed);

/**
 * @brief 터렛 회전 속도 설정
 * @param speed -512 ~ 511 (음수: 좌, 양수: 우)
 */
void rctank_motor_turret_set(int32_t speed);

#ifdef __cplusplus
}
#endif

#endif /* RCTANK_MOTOR_H */
