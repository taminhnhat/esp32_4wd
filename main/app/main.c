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
#include "config.h"
#include "motor_control.h"
#include "battery.h"
#include "board_led.h"
#include "rp3_receiver.h"
#include "usb_bridge.h"

static const char *TAG = "example";

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

    led_init();
    led_set(0, 16, 0);
    extend_led_set(3, 0, 0, 8);

    // --- RP3 Receiver initialization ---
    static rp3_receiver_t rp3_receiver;
    rp3_receiver_init(&rp3_receiver);
    rp3_receiver_start_job(&rp3_receiver);

    // Initialize USB bridge
    usb_bridge_init();
    xTaskCreate(usb_bridge_task, "usb_bridge", 4096, NULL, 5, NULL);

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(100));

        // Get current battery data
        battery_data_t battery_data = {0};
        battery_read_data(&battery_data);

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

        // --- RP3 Receiver logging every 100ms: print 10 RC channel values mapped to -100..100 ---
        rp3_receiver_snapshot_t rp3_snapshot;
        rp3_receiver_get_snapshot(&rp3_receiver, &rp3_snapshot);
        if (rp3_snapshot.link_stats_valid && rp3_snapshot.rc_channels_valid)
        {
            char line[160];
            int pos = 0;

            for (int i = 0; i < 10; ++i)
            {
                // printf("%-8u", rp3_snapshot.rc_channels[i]);
                pos += snprintf(line + pos, sizeof(line) - pos,
                                "%-8u", rp3_snapshot.rc_channels[i]);
            }
            // printf("\n");
            pos += snprintf(line + pos, sizeof(line) - pos, "\r\n");
            usb_bridge_write(line, pos);
        }

        // Print battery status every 5 seconds
        static uint32_t battery_print_counter = 0;
        if (++battery_print_counter % 10 == 0)
        {
            // ESP_LOGI(TAG, "Velocity [pulses/%dms] m1=%d m2=%d m3=%d m4=%d",
            //          BDC_PID_LOOP_PERIOD_MS,
            //          motor_system.motors[0].report_pulses,
            //          motor_system.motors[1].report_pulses,
            //          motor_system.motors[2].report_pulses,
            //          motor_system.motors[3].report_pulses);
            // battery_print_status();
        }
    }
}
