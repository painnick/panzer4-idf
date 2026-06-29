/**
 * @file rctank_dfplayer.c
 * @brief DFPlayer Mini UART 제어 (DFPlayerMini_Fast 프로토콜 기준)
 * @see https://github.com/PowerBroker2/DFPlayerMini_Fast
 */
#include "rctank_dfplayer.h"
#include "rctank_pins.h"

#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "rctank_dfplayer";
static TaskHandle_t s_rx_task = NULL;


#define UART_NUM            UART_NUM_1
#define UART_BAUD           9600
#define UART_BUF_SIZE       256

/* DFPlayerMini_Fast 패킷 상수 (dfplayer namespace) */
#define DFPLAYER_SB          0x7E
#define DFPLAYER_VER        0xFF
#define DFPLAYER_LEN        0x06
#define DFPLAYER_NO_FEEDBACK 0x00
#define DFPLAYER_EB          0xEF

/* 제어 명령 (Control Command Values) */
#define DFPLAYER_CMD_PLAY           0x03
#define DFPLAYER_CMD_VOLUME         0x06
#define DFPLAYER_CMD_PLAYBACK_MODE  0x08   /* 루프 재생 (param = 트랙 번호) */
#define DFPLAYER_CMD_REPEAT_PLAY    0x11   /* 반복 재생 제어 (0x00=중지) */
#define DFPLAYER_CMD_STOP           0x16
#define DFPLAYER_CMD_USE_MP3_FOLDER 0x12
#define DFPLAYER_CMD_INSERT_ADVERT  0x13
#define DFPLAYER_CMD_STOP_ADVERT    0x15

#define DFPLAYER_STACK_SIZE 10

/** 전송 패킷 (stack). DFPlayerMini_Fast과 동일 레이아웃 */
typedef struct {
    uint8_t start_byte;
    uint8_t version;
    uint8_t length;
    uint8_t command_value;
    uint8_t feedback_value;
    uint8_t param_msb;
    uint8_t param_lsb;
    uint8_t checksum_msb;
    uint8_t checksum_lsb;
    uint8_t end_byte;
} dfplayer_stack_t;

/**
 * @brief 체크섬 계산 (DFPlayerMini_Fast findChecksum 동일)
 * checksum = (~(version + length + command + feedback + paramMSB + paramLSB)) + 1
 */
static void dfplayer_find_checksum(dfplayer_stack_t *stack)
{
    uint16_t sum = (uint16_t)(stack->version + stack->length + stack->command_value
                              + stack->feedback_value + stack->param_msb + stack->param_lsb);
    uint16_t checksum = (uint16_t)((~sum) + 1);
    stack->checksum_msb = (uint8_t)(checksum >> 8);
    stack->checksum_lsb = (uint8_t)(checksum & 0xFF);
}

/** 패킷 전송 */
static esp_err_t dfplayer_send_stack(const dfplayer_stack_t *stack)
{
    uint8_t buf[DFPLAYER_STACK_SIZE] = {
        stack->start_byte,
        stack->version,
        stack->length,
        stack->command_value,
        stack->feedback_value,
        stack->param_msb,
        stack->param_lsb,
        stack->checksum_msb,
        stack->checksum_lsb,
        stack->end_byte,
    };
    int n = uart_write_bytes(UART_NUM, buf, DFPLAYER_STACK_SIZE);
    if (n != DFPLAYER_STACK_SIZE) {
        ESP_LOGE(TAG, "uart_write_bytes %d", n);
        return ESP_FAIL;
    }
    return ESP_OK;
}

/** 명령 1회 전송 (공통) */
/** 명령 1회 전송 (공통) */
static esp_err_t dfplayer_send_cmd(uint8_t cmd, uint8_t param_msb, uint8_t param_lsb)
{
    dfplayer_stack_t stack = {
        .start_byte = DFPLAYER_SB,
        .version = DFPLAYER_VER,
        .length = DFPLAYER_LEN,
        .command_value = cmd,
        .feedback_value = DFPLAYER_NO_FEEDBACK,
        .param_msb = param_msb,
        .param_lsb = param_lsb,
        .end_byte = DFPLAYER_EB,
    };
    dfplayer_find_checksum(&stack);
    return dfplayer_send_stack(&stack);
}

/** Check if calculated checksum matches received one */
static bool dfplayer_verify_checksum(const dfplayer_stack_t *stack)
{
    uint16_t sum = (uint16_t)(stack->version + stack->length + stack->command_value
                              + stack->feedback_value + stack->param_msb + stack->param_lsb);
    uint16_t checksum = (uint16_t)((~sum) + 1);
    return (stack->checksum_msb == (uint8_t)(checksum >> 8)) &&
           (stack->checksum_lsb == (uint8_t)(checksum & 0xFF));
}

static void dfplayer_task(void *arg)
{
    uint8_t data[128];
    int idx = 0; 
    uint8_t pkt_buf[DFPLAYER_STACK_SIZE]; 
    int pkt_idx = 0;

    /* Simple state machine vars */
    while (1) {
        int len = uart_read_bytes(UART_NUM, data, sizeof(data), pdMS_TO_TICKS(50));
        if (len > 0) {
           for (int i = 0; i < len; i++) {
               uint8_t b = data[i];
               /* 찾는 중: 헤더(0x7E) */
               if (pkt_idx == 0) {
                   if (b == DFPLAYER_SB) {
                       pkt_buf[pkt_idx++] = b;
                   }
               } else {
                   pkt_buf[pkt_idx++] = b;
                   if (pkt_idx >= DFPLAYER_STACK_SIZE) {
                       /* 패킷 완성 확인 (EndByte = 0xEF) */
                       if (pkt_buf[9] == DFPLAYER_EB) {
                           dfplayer_stack_t *s = (dfplayer_stack_t *)pkt_buf;
                           /* 체크섬 검증 */
                           if (dfplayer_verify_checksum(s)) {
                               // ESP_LOGI(TAG, "RX Cmd: 0x%02X P1:0x%02X P2:0x%02X", s->command_value, s->param_msb, s->param_lsb);
                               /* 0x3D = Track Finished (3.2.1.1 Playback finished "3D 00 00 01 xx xx EF") */
                               if (s->command_value == 0x3D) {
                                   uint16_t track = ((uint16_t)s->param_msb << 8) | s->param_lsb;
                                   // ESP_LOGI(TAG, "Track %d Finished", track);
                                   /* 어떤 트랙이든 재생이 끝나면 IDLE 트랙을 반복 재생으로 다시 시작 */
                                   rctank_dfplayer_play_loop(RCTANK_DFPLAYER_TRACK_IDLE);
                               }
                           } else {
                               ESP_LOGW(TAG, "Checksum fail");
                           }
                       }
                       pkt_idx = 0; /* Reset */
                   }
               }
           }
        }
    }
}

esp_err_t rctank_dfplayer_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    esp_err_t ret = uart_param_config(UART_NUM, &uart_config);
    if (ret != ESP_OK) return ret;
    ret = uart_set_pin(UART_NUM, RCTANK_PIN_SOUND_TX, RCTANK_PIN_SOUND_RX, -1, -1);
    if (ret != ESP_OK) return ret;
    ret = uart_driver_install(UART_NUM, UART_BUF_SIZE, UART_BUF_SIZE, 0, NULL, 0);
    if (ret != ESP_OK) return ret;

    vTaskDelay(pdMS_TO_TICKS(200));

    xTaskCreate(dfplayer_task, "dfplayer_task", 2048, NULL, 5, &s_rx_task);

    ESP_LOGI(TAG, "dfplayer init ok");
    return ESP_OK;
}

esp_err_t rctank_dfplayer_play(uint8_t track)
{
    if (track < 1) return ESP_ERR_INVALID_ARG;
    /* PLAY(0x03): param = track number (MSB, LSB). 0001.mp3 ~ 9999 등 */
    return dfplayer_send_cmd(DFPLAYER_CMD_PLAY, (uint8_t)((uint16_t)track >> 8), (uint8_t)(track & 0xFF));
}

esp_err_t rctank_dfplayer_play_loop(uint8_t track)
{
    if (track < 1) return ESP_ERR_INVALID_ARG;
    /* 0x08: Playback Mode (Single Track Loop) 
       대부분의 DFPlayer 모듈에서 이 명령만으로 해당 트랙의 무한 반복 재생이 시작됩니다. */
    return dfplayer_send_cmd(0x08, (uint8_t)((uint16_t)track >> 8), (uint8_t)(track & 0xFF));
}

esp_err_t rctank_dfplayer_set_volume(uint8_t vol)
{
    if (vol > 30) vol = 30;
    /* VOLUME(0x06): paramLSB = 0~30 */
    return dfplayer_send_cmd(DFPLAYER_CMD_VOLUME, 0x00, vol);
}

esp_err_t rctank_dfplayer_stop(void)
{
    /* STOP(0x16): param 0, 0 */
    return dfplayer_send_cmd(DFPLAYER_CMD_STOP, 0x00, 0x00);
}
