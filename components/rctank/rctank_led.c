/**
 * @file rctank_led.c
 * @brief RC Tank GPIO LED
 */
#include "rctank_led.h"
#include "rctank_pins.h"

#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "rctank_led";
static int headlight_on = 0;

esp_err_t rctank_led_init(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << RCTANK_PIN_GUN_LED) | (1ULL << RCTANK_PIN_HEADLIGHT)
                        | (1ULL << RCTANK_PIN_MG_LED),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t ret = gpio_config(&io);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "gpio_config %s", esp_err_to_name(ret));
        return ret;
    }
    gpio_set_level(RCTANK_PIN_GUN_LED, 0);
    gpio_set_level(RCTANK_PIN_HEADLIGHT, 0);
    gpio_set_level(RCTANK_PIN_MG_LED, 0);
    headlight_on = 0;
    ESP_LOGI(TAG, "led init ok");
    return ESP_OK;
}

void rctank_led_gun_set(int on)
{
    gpio_set_level(RCTANK_PIN_GUN_LED, on ? 1 : 0);
}

void rctank_led_mg_set(int on)
{
    gpio_set_level(RCTANK_PIN_MG_LED, on ? 1 : 0);
}

void rctank_led_headlight_set(int on)
{
    headlight_on = on ? 1 : 0;
    gpio_set_level(RCTANK_PIN_HEADLIGHT, headlight_on);
}

int rctank_led_headlight_get(void)
{
    return headlight_on;
}
