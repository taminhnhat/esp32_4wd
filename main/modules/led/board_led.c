#include "board_led.h"
#include "config.h"

static led_strip_handle_t builtin_led;
static led_strip_handle_t extend_led;

void led_set(uint32_t r, int32_t g, int32_t b)
{
    ESP_ERROR_CHECK(led_strip_set_pixel(builtin_led, 0, r, g, b));
    ESP_ERROR_CHECK(led_strip_refresh(builtin_led));
}

void extend_led_set(uint8_t idx, uint32_t r, int32_t g, int32_t b)
{
    ESP_ERROR_CHECK(led_strip_set_pixel(extend_led, idx, r, g, b));
    ESP_ERROR_CHECK(led_strip_refresh(extend_led));
}

void led_init(void)
{
    led_strip_config_t strip_config = {
        .strip_gpio_num = BUILTIN_LED_GPIO,
        .max_leds = 1,
    };
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000,
        .flags.with_dma = false,
    };

    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &builtin_led));
    ESP_ERROR_CHECK(led_strip_clear(builtin_led));

    strip_config.strip_gpio_num = 45;
    strip_config.max_leds = 8;
    rmt_config.resolution_hz = 10 * 1000 * 1000;
    rmt_config.flags.with_dma = false;

    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &extend_led));
    ESP_ERROR_CHECK(led_strip_clear(extend_led));
}