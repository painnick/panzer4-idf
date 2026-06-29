/**
 * @file rctank_storage.h
 * @brief RC Tank NVS 저장 (볼륨 등, EEPROM 대체)
 */
#ifndef RCTANK_STORAGE_H
#define RCTANK_STORAGE_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define RCTANK_VOLUME_MIN     10
#define RCTANK_VOLUME_MAX     30
#define RCTANK_VOLUME_DEFAULT 20

/** NVS에서 볼륨 로드. 기본값 사용 시 *out_vol 유지 */
esp_err_t rctank_storage_init(void);

/** 현재 볼륨 값 가져오기 (10~30) */
uint8_t rctank_storage_volume_get(void);

/** 볼륨 저장 (10~30). 호출 시 NVS에 기록 */
esp_err_t rctank_storage_volume_set(uint8_t vol);

/** 모든 설정 초기화 후 재부팅 */
void rctank_storage_erase_and_restart(void);

#ifdef __cplusplus
}
#endif

#endif /* RCTANK_STORAGE_H */
