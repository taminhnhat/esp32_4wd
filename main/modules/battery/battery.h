/*
 * SPDX-FileCopyrightText: 2024
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef BATTERY_H
#define BATTERY_H

#include <stdbool.h>
#include <stdint.h>
#include "config.h"
#include "esp_err.h"
#include "driver/i2c_master.h"

/* I2C Configuration */
#define BATTERY_I2C_MASTER_NUM         I2C_NUM_0
#define BATTERY_I2C_MASTER_SDA_IO      I2C_SDA
#define BATTERY_I2C_MASTER_SCL_IO      I2C_SCL
#define BATTERY_I2C_MASTER_FREQ_HZ     100000
#define BATTERY_I2C_TIMEOUT_MS         1000

/* DFrobot Wattmeter I2C Address */
#define BATTERY_WATTMETER_I2C_ADDR     0x10

/* Wattmeter Registers */
#define BATTERY_REG_VOLTAGE            0x02  // Voltage register (2 bytes, LSB first)
#define BATTERY_REG_CURRENT            0x04  // Current register (2 bytes, LSB first)
#define BATTERY_REG_POWER              0x06  // Power register (2 bytes, LSB first)
#define BATTERY_REG_ENERGY             0x08  // Energy register (2 bytes, LSB first)
#define BATTERY_REG_VOLTAGE_RAW        0x0A  // Raw voltage register (2 bytes)
#define BATTERY_REG_CURRENT_RAW        0x0C  // Raw current register (2 bytes)

/**
 * @brief Battery measurement data structure
 */
typedef struct {
    float voltage;          // Voltage in volts
    float current;          // Current in amps
    float power;            // Power in watts
    float energy;           // Energy in watt-hours
    uint16_t voltage_raw;   // Raw voltage ADC value
    uint16_t current_raw;   // Raw current ADC value
    uint32_t timestamp;     // Timestamp of last measurement
    bool valid;             // Data validity flag
} battery_data_t;

/**
 * @brief Initialize battery monitoring system
 * Sets up I2C communication with DFrobot wattmeter
 *
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t battery_init(void);

/**
 * @brief Read all battery measurements
 *
 * @param data Pointer to battery_data_t structure to store measurements
 * @return esp_err_t ESP_OK on success
 */
esp_err_t battery_read_data(battery_data_t *data);

/**
 * @brief Read voltage from wattmeter
 *
 * @param voltage Pointer to store voltage value in volts
 * @return esp_err_t ESP_OK on success
 */
esp_err_t battery_read_voltage(float *voltage);

/**
 * @brief Read current from wattmeter
 *
 * @param current Pointer to store current value in amps
 * @return esp_err_t ESP_OK on success
 */
esp_err_t battery_read_current(float *current);

/**
 * @brief Read power from wattmeter
 *
 * @param power Pointer to store power value in watts
 * @return esp_err_t ESP_OK on success
 */
esp_err_t battery_read_power(float *power);

/**
 * @brief Read energy consumption from wattmeter
 *
 * @param energy Pointer to store energy value in watt-hours
 * @return esp_err_t ESP_OK on success
 */
esp_err_t battery_read_energy(float *energy);

/**
 * @brief Print battery status for debugging
 */
void battery_print_status(void);

/**
 * @brief Reset energy counter
 *
 * @return esp_err_t ESP_OK on success
 */
esp_err_t battery_reset_energy(void);

#endif // BATTERY_H
