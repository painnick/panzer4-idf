/**
 * @file rctank_dfplayer.h
 * @brief RC Tank DFPlayer Mini UART 제어 (효과음)
 */
#ifndef RCTANK_DFPLAYER_H
#define RCTANK_DFPLAYER_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define RCTANK_DFPLAYER_TRACK_IDLE 1    /* 0001.mp3 대기 반복 */
#define RCTANK_DFPLAYER_TRACK_GUN 2     /* 0002.mp3 포신 발사 */
#define RCTANK_DFPLAYER_TRACK_MG 3      /* 0003.mp3 기관총 */
#define RCTANK_DFPLAYER_TRACK_CONNECT 4 /* 0004.mp3 게임패드 연결 */

esp_err_t rctank_dfplayer_init(void);

/** 트랙 재생 (1회) */
esp_err_t rctank_dfplayer_play(uint8_t track);

/** 트랙 반복 재생 */
esp_err_t rctank_dfplayer_play_loop(uint8_t track);

/** 볼륨 설정 (0~30) */
esp_err_t rctank_dfplayer_set_volume(uint8_t vol);

/** 재생 중지 */
esp_err_t rctank_dfplayer_stop(void);

#ifdef __cplusplus
}
#endif

#endif /* RCTANK_DFPLAYER_H */
