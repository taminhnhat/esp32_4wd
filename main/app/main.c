/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "led_strip.h"
#include "config.h"
#include "motor_control.h"
#include "battery.h"

static const char *TAG = "example";
static led_strip_handle_t s_health_led;

static void health_led_set(bool on)
{
    if (on) {
        ESP_ERROR_CHECK(led_strip_set_pixel(s_health_led, 0, 0, 16, 0));
        ESP_ERROR_CHECK(led_strip_refresh(s_health_led));
    } else {
        ESP_ERROR_CHECK(led_strip_clear(s_health_led));
    }
}

static void health_led_init(void)
{
    led_strip_config_t strip_config = {
        .strip_gpio_num = BUILTIN_LED_GPIO,
        .max_leds = 1,
    };
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000,
        .flags.with_dma = false,
    };

    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &s_health_led));
    ESP_ERROR_CHECK(led_strip_clear(s_health_led));
}

// Enable this config,  we will print debug formated string, which in return can be captured and parsed by Serial-Studio
#define SERIAL_STUDIO_DEBUG CONFIG_SERIAL_STUDIO_DEBUG

void app_main(void)
{
    static motor_control_system_t motor_system = {
        .motors = {},
        .last_pulse_counts = {}};

    ESP_LOGI(TAG, "Initializing %d motor(s)", NUM_MOTORS);

    // Initialize all motors
    for (int i = 0; i < NUM_MOTORS; i++)
    {
        motor_config_t config = get_motor_config(i);
        ESP_ERROR_CHECK(motor_init(i, &motor_system, &config));
    }

    ESP_LOGI(TAG, "Starting all motors");
    ESP_ERROR_CHECK(motor_start_all(&motor_system));

    ESP_LOGI(TAG, "Initializing battery monitoring");
    ESP_ERROR_CHECK(battery_init());

    ESP_LOGI(TAG, "Create a timer to do PID calculation periodically for all motors");
    const esp_timer_create_args_t periodic_timer_args = {
        .callback = pid_loop_cb,
        .arg = &motor_system,
        .name = "pid_loop"};
    esp_timer_handle_t pid_loop_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &pid_loop_timer));

    ESP_LOGI(TAG, "Start motor speed loop");
    ESP_ERROR_CHECK(esp_timer_start_periodic(pid_loop_timer, BDC_PID_LOOP_PERIOD_MS * 1000));

    ESP_LOGI(TAG, "Initializing health RGB LED on GPIO%d", BUILTIN_LED_GPIO);
    health_led_init();

    bool led_on = true;
    uint32_t health_led_counter = 0;
    health_led_set(led_on);

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(100));
        // the following logging format is according to the requirement of serial-studio frame format
        // also see the dashboard config file `serial-studio-dashboard.json` for more information

        // Get current battery data
        battery_data_t battery_data = {0};
        battery_read_data(&battery_data);

        if (++health_led_counter >= 50)
        {
            health_led_counter = 0;
        }

        bool next_led_on = health_led_counter <= 5;
        if (next_led_on != led_on)
        {
            led_on = next_led_on;
            health_led_set(led_on);
        }

#if SERIAL_STUDIO_DEBUG
        printf("/*motor1:%d,motor2:%d,motor3:%d,motor4:%d,voltage:%.2f,current:%.3f,power:%.2f*/\r\n",
               motor_system.motors[0].report_pulses,
               motor_system.motors[1].report_pulses,
               motor_system.motors[2].report_pulses,
               motor_system.motors[3].report_pulses,
               battery_data.voltage,
               battery_data.current,
               battery_data.power);
#endif

        // Print battery status every 5 seconds
        static uint32_t battery_print_counter = 0;
        if (++battery_print_counter % 10 == 0)
        {
            ESP_LOGI(TAG, "Velocity [pulses/%dms] m1=%d m2=%d m3=%d m4=%d",
                     BDC_PID_LOOP_PERIOD_MS,
                     motor_system.motors[0].report_pulses,
                     motor_system.motors[1].report_pulses,
                     motor_system.motors[2].report_pulses,
                     motor_system.motors[3].report_pulses);
            // battery_print_status();
        }
    }
}
