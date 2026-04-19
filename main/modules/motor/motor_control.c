/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "motor_control.h"
#include "esp_log.h"

static const char *TAG = "motor_control";

static motor_config_t motor_configs[NUM_MOTORS] = {
    {
        .pwm_gpio_a = BDC_MCPWM_GPIO_A_1,
        .pwm_gpio_b = BDC_MCPWM_GPIO_B_1,
        .enc_gpio_a = BDC_ENCODER_GPIO_A_1,
        .enc_gpio_b = BDC_ENCODER_GPIO_B_1,
        .mcpwm_group = BDC_MCPWM_GROUP_1,
    },
    {
        .pwm_gpio_a = BDC_MCPWM_GPIO_A_2,
        .pwm_gpio_b = BDC_MCPWM_GPIO_B_2,
        .enc_gpio_a = BDC_ENCODER_GPIO_A_2,
        .enc_gpio_b = BDC_ENCODER_GPIO_B_2,
        .mcpwm_group = BDC_MCPWM_GROUP_2,
    },
    {
        .pwm_gpio_a = BDC_MCPWM_GPIO_A_3,
        .pwm_gpio_b = BDC_MCPWM_GPIO_B_3,
        .enc_gpio_a = BDC_ENCODER_GPIO_A_3,
        .enc_gpio_b = BDC_ENCODER_GPIO_B_3,
        .mcpwm_group = BDC_MCPWM_GROUP_3,
    },
    {
        .pwm_gpio_a = BDC_MCPWM_GPIO_A_4,
        .pwm_gpio_b = BDC_MCPWM_GPIO_B_4,
        .enc_gpio_a = BDC_ENCODER_GPIO_A_4,
        .enc_gpio_b = BDC_ENCODER_GPIO_B_4,
        .mcpwm_group = BDC_MCPWM_GROUP_4,
    },
};

motor_config_t get_motor_config(int motor_index)
{
    if (motor_index < NUM_MOTORS)
    {
        return motor_configs[motor_index];
    }
    return motor_configs[0];
}

esp_err_t motor_init(int motor_index, motor_control_system_t *motor_system, const motor_config_t *config)
{
    esp_err_t ret = ESP_OK;

    ESP_LOGI(TAG, "Create DC motor %d", motor_index + 1);
    bdc_motor_config_t motor_config = {
        .pwm_freq_hz = BDC_MCPWM_FREQ_HZ,
        .pwma_gpio_num = config->pwm_gpio_a,
        .pwmb_gpio_num = config->pwm_gpio_b,
    };
    bdc_motor_mcpwm_config_t mcpwm_config = {
        .group_id = config->mcpwm_group,
        .resolution_hz = BDC_MCPWM_TIMER_RESOLUTION_HZ,
    };
    bdc_motor_handle_t motor = NULL;
    ESP_ERROR_CHECK(bdc_motor_new_mcpwm_device(&motor_config, &mcpwm_config, &motor));
    motor_system->motors[motor_index].motor = motor;

    ESP_LOGI(TAG, "Init pcnt driver to decode rotary signal for motor %d", motor_index + 1);
    pcnt_unit_config_t unit_config = {
        .high_limit = BDC_ENCODER_PCNT_HIGH_LIMIT,
        .low_limit = BDC_ENCODER_PCNT_LOW_LIMIT,
        .flags.accum_count = true,
    };
    pcnt_unit_handle_t pcnt_unit = NULL;
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));

    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 1000,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config));

    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = config->enc_gpio_a,
        .level_gpio_num = config->enc_gpio_b,
    };
    pcnt_channel_handle_t pcnt_chan_a = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_a_config, &pcnt_chan_a));

    pcnt_chan_config_t chan_b_config = {
        .edge_gpio_num = config->enc_gpio_b,
        .level_gpio_num = config->enc_gpio_a,
    };
    pcnt_channel_handle_t pcnt_chan_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_b_config, &pcnt_chan_b));

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, BDC_ENCODER_PCNT_HIGH_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, BDC_ENCODER_PCNT_LOW_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));
    motor_system->motors[motor_index].pcnt_encoder = pcnt_unit;

    ESP_LOGI(TAG, "Create PID control block for motor %d", motor_index + 1);
    pid_ctrl_parameter_t pid_runtime_param = {
        .kp = BDC_PID_KP,
        .ki = BDC_PID_KI,
        .kd = BDC_PID_KD,
        .cal_type = PID_CAL_TYPE_INCREMENTAL,
        .max_output = BDC_MCPWM_DUTY_TICK_MAX - 1,
        .min_output = 0,
        .max_integral = BDC_PID_MAX_INTEGRAL,
        .min_integral = BDC_PID_MIN_INTEGRAL,
    };
    pid_ctrl_block_handle_t pid_ctrl = NULL;
    pid_ctrl_config_t pid_config = {
        .init_param = pid_runtime_param,
    };
    ESP_ERROR_CHECK(pid_new_control_block(&pid_config, &pid_ctrl));
    motor_system->motors[motor_index].pid_ctrl = pid_ctrl;

    return ret;
}

esp_err_t motor_start_all(motor_control_system_t *motor_system)
{
    esp_err_t ret = ESP_OK;

    for (int i = 0; i < NUM_MOTORS; i++)
    {
        ESP_LOGI(TAG, "Enable motor %d", i + 1);
        ESP_ERROR_CHECK(bdc_motor_enable(motor_system->motors[i].motor));
        ESP_LOGI(TAG, "Forward motor %d", i + 1);
        ESP_ERROR_CHECK(bdc_motor_forward(motor_system->motors[i].motor));
    }

    return ret;
}

void pid_loop_cb(void *args)
{
    motor_control_system_t *sys = (motor_control_system_t *)args;

    for (int i = 0; i < NUM_MOTORS; i++)
    {
        motor_control_context_t *ctx = &sys->motors[i];
        pcnt_unit_handle_t pcnt_unit = ctx->pcnt_encoder;
        pid_ctrl_block_handle_t pid_ctrl = ctx->pid_ctrl;
        bdc_motor_handle_t motor = ctx->motor;

        // get the result from rotary encoder
        int cur_pulse_count = 0;
        pcnt_unit_get_count(pcnt_unit, &cur_pulse_count);
        int real_pulses = cur_pulse_count - sys->last_pulse_counts[i];
        sys->last_pulse_counts[i] = cur_pulse_count;
        ctx->report_pulses = real_pulses;

        // calculate the speed error
        float error = BDC_PID_EXPECT_SPEED - real_pulses;
        float new_speed = 0;

        // set the new speed
        pid_compute(pid_ctrl, error, &new_speed);
        bdc_motor_set_speed(motor, (uint32_t)new_speed);
    }
}
