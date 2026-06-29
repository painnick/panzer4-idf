/**
 * @file rctank.h
 * @brief RC Tank 하드웨어 통합 초기화 (C 함수)
 */
#ifndef RCTANK_H
#define RCTANK_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief NVS 로드 → 서보/LED/모터/DFPlayer 초기화, 볼륨 적용, 효과음 1 재생
 * @return ESP_OK on success
 */
esp_err_t rctank_init(void);

#ifdef __cplusplus
}
#endif

#endif /* RCTANK_H */
