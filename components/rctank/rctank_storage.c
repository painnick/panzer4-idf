/**
 * @file rctank_storage.c
 * @brief NVS 볼륨/설정 저장
 */
#include "rctank_storage.h"

#include "nvs.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_system.h"

static const char *TAG = "rctank_storage";
static const char *NVS_NAMESPACE = "rctank";
static const char *NVS_KEY_VOLUME = "vol";

static uint8_t s_volume = RCTANK_VOLUME_DEFAULT;
static nvs_handle_t s_handle = 0;

esp_err_t rctank_storage_init(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "nvs_flash_init %s", esp_err_to_name(ret));
        return ret;
    }

    ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &s_handle);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "nvs_open %s, use default volume", esp_err_to_name(ret));
        s_volume = RCTANK_VOLUME_DEFAULT;
        return ret;
    }

    uint8_t v;
    ret = nvs_get_u8(s_handle, NVS_KEY_VOLUME, &v);
    if (ret == ESP_OK && v >= RCTANK_VOLUME_MIN && v <= RCTANK_VOLUME_MAX) {
        s_volume = v;
        ESP_LOGI(TAG, "volume loaded %u", (unsigned)s_volume);
    } else {
        s_volume = RCTANK_VOLUME_DEFAULT;
    }
    return ESP_OK;
}

uint8_t rctank_storage_volume_get(void)
{
    return s_volume;
}

esp_err_t rctank_storage_volume_set(uint8_t vol)
{
    if (vol < RCTANK_VOLUME_MIN) vol = RCTANK_VOLUME_MIN;
    if (vol > RCTANK_VOLUME_MAX) vol = RCTANK_VOLUME_MAX;
    s_volume = vol;
    if (s_handle == 0) return ESP_ERR_INVALID_STATE;
    esp_err_t ret = nvs_set_u8(s_handle, NVS_KEY_VOLUME, vol);
    if (ret != ESP_OK) return ret;
    return nvs_commit(s_handle);
}

void rctank_storage_erase_and_restart(void)
{
    nvs_handle_t h = 0;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) == ESP_OK) {
        nvs_erase_all(h);
        nvs_commit(h);
        nvs_close(h);
    }
    if (s_handle) {
        nvs_close(s_handle);
        s_handle = 0;
    }
    esp_restart();
}
