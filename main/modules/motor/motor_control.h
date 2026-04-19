/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "driver/pulse_cnt.h"
#include "bdc_motor.h"
#include "pid_ctrl.h"
#include "config.h"

/**
 * @brief Motor control context structure
 * Contains all data and handles for a single motor
 */
typedef struct {
    bdc_motor_handle_t motor;
    pcnt_unit_handle_t pcnt_encoder;
    pid_ctrl_block_handle_t pid_ctrl;
    int report_pulses;
} motor_control_context_t;

/**
 * @brief Motor control system structure
 * Contains all motors and shared state
 */
typedef struct {
    motor_control_context_t motors[NUM_MOTORS];
    int last_pulse_counts[NUM_MOTORS];
} motor_control_system_t;

/**
 * @brief Motor configuration structure
 * Contains GPIO and group information for a single motor
 */
typedef struct {
    int pwm_gpio_a;
    int pwm_gpio_b;
    int enc_gpio_a;
    int enc_gpio_b;
    int mcpwm_group;
} motor_config_t;

/**
 * @brief Initialize a single motor with encoder and PID controller
 * @param motor_index Motor index (0 to NUM_MOTORS-1)
 * @param motor_system Pointer to the motor control system
 * @param config Pointer to motor configuration
 * @return ESP error code
 */
esp_err_t motor_init(int motor_index, motor_control_system_t *motor_system, const motor_config_t *config);

/**
 * @brief Enable and start all motors
 * @param motor_system Pointer to the motor control system
 * @return ESP error code
 */
esp_err_t motor_start_all(motor_control_system_t *motor_system);

/**
 * @brief PID control loop callback for all motors
 * Called periodically to update motor speeds based on encoder feedback
 * @param args Pointer to motor_control_system_t
 */
void pid_loop_cb(void *args);

/**
 * @brief Get GPIO configuration for a specific motor
 * @param motor_index Motor index
 * @return Motor configuration
 */
motor_config_t get_motor_config(int motor_index);

#endif // MOTOR_CONTROL_H
