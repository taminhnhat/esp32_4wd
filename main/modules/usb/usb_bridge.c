#include "usb_bridge.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tinyusb.h"
#include "tinyusb_default_config.h"
#include "tinyusb_cdc_acm.h"
#include "sdkconfig.h"

static const char *TAG = "usb_bridge";
static uint8_t rx_buf[CONFIG_TINYUSB_CDC_RX_BUFSIZE];

void usb_bridge_init(void)
{
    ESP_LOGI(TAG, "USB initialization");
    const tinyusb_config_t tusb_cfg = TINYUSB_DEFAULT_CONFIG();
    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

    tinyusb_config_cdcacm_t acm_cfg = {
        .cdc_port = TINYUSB_CDC_ACM_0,
        .callback_rx = NULL,
        .callback_rx_wanted_char = NULL,
        .callback_line_state_changed = NULL,
        .callback_line_coding_changed = NULL
    };

    ESP_ERROR_CHECK(tinyusb_cdcacm_init(&acm_cfg));
    ESP_LOGI(TAG, "USB initialization DONE");
}

void usb_bridge_task(void *pvParameters)
{
    size_t rx_size = 0;
    while (1) {
        ESP_ERROR_CHECK(tinyusb_cdcacm_read(TINYUSB_CDC_ACM_0, rx_buf, CONFIG_TINYUSB_CDC_RX_BUFSIZE, &rx_size));

        if (rx_size == 0) {
            vTaskDelay(1);
            continue;
        }

        size_t offset = 0;
        while (offset < rx_size) {
            size_t queued = tinyusb_cdcacm_write_queue(TINYUSB_CDC_ACM_0, rx_buf + offset, rx_size - offset);
            if (queued == 0) {
                ESP_ERROR_CHECK(tinyusb_cdcacm_write_flush(TINYUSB_CDC_ACM_0, 1));
                continue;
            }

            offset += queued;
            esp_err_t err = tinyusb_cdcacm_write_flush(TINYUSB_CDC_ACM_0, 0);
            if (err != ESP_OK && err != ESP_ERR_NOT_FINISHED) {
                ESP_ERROR_CHECK(err);
            }
        }
    }
}

void usb_bridge_write(const char *data, size_t len)
{
    size_t offset = 0;
    while (offset < len) {
        size_t queued = tinyusb_cdcacm_write_queue(TINYUSB_CDC_ACM_0, (const uint8_t *)data + offset, len - offset);
        if (queued == 0) {
            ESP_ERROR_CHECK(tinyusb_cdcacm_write_flush(TINYUSB_CDC_ACM_0, 1));
            continue;
        }
        offset += queued;
        esp_err_t err = tinyusb_cdcacm_write_flush(TINYUSB_CDC_ACM_0, 0);
        if (err != ESP_OK && err != ESP_ERR_NOT_FINISHED) {
            ESP_ERROR_CHECK(err);
        }
    }
}
