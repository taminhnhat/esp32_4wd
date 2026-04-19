/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef CONFIG_H
#define CONFIG_H

/* MCPWM Configuration */
#define BDC_MCPWM_TIMER_RESOLUTION_HZ 10000000                                      // 10MHz, 1 tick = 0.1us
#define BDC_MCPWM_FREQ_HZ 25000                                                     // 25KHz PWM
#define BDC_MCPWM_DUTY_TICK_MAX (BDC_MCPWM_TIMER_RESOLUTION_HZ / BDC_MCPWM_FREQ_HZ) // maximum value we can set for the duty cycle, in ticks

/* Number of motors */
#define NUM_MOTORS 4

/* Health LED */
#define BUILTIN_LED_GPIO 48

/* Encoder Configuration */
#define BDC_ENCODER_PCNT_HIGH_LIMIT 1000
#define BDC_ENCODER_PCNT_LOW_LIMIT -1000

/* PID Loop Configuration */
#define BDC_PID_LOOP_PERIOD_MS 10 // calculate the motor speed every 10ms
#define BDC_PID_EXPECT_SPEED 400  // expected motor speed, in the pulses counted by the rotary encoder

/* PID Controller Parameters */
#define BDC_PID_KP 0.6
#define BDC_PID_KI 0.4
#define BDC_PID_KD 0.2
#define BDC_PID_MAX_INTEGRAL 1000
#define BDC_PID_MIN_INTEGRAL -1000

/* Motor 1 GPIO pins */
#define BDC_MCPWM_GPIO_A_1 4
#define BDC_MCPWM_GPIO_B_1 5
#define BDC_ENCODER_GPIO_A_1 42
#define BDC_ENCODER_GPIO_B_1 41
#define BDC_MCPWM_GROUP_1 0

/* Motor 2 GPIO pins */
#define BDC_MCPWM_GPIO_A_2 6
#define BDC_MCPWM_GPIO_B_2 7
#define BDC_ENCODER_GPIO_A_2 40
#define BDC_ENCODER_GPIO_B_2 39
#define BDC_MCPWM_GROUP_2 0

/* Motor 3 GPIO pins */
#define BDC_MCPWM_GPIO_A_3 15
#define BDC_MCPWM_GPIO_B_3 16
#define BDC_ENCODER_GPIO_A_3 38
#define BDC_ENCODER_GPIO_B_3 37
#define BDC_MCPWM_GROUP_3 1

/* Motor 4 GPIO pins */
#define BDC_MCPWM_GPIO_A_4 17
#define BDC_MCPWM_GPIO_B_4 18
#define BDC_ENCODER_GPIO_A_4 36
#define BDC_ENCODER_GPIO_B_4 35
#define BDC_MCPWM_GROUP_4 1

/* I2C pins */
#define I2C_SDA 8
#define I2C_SCL 9

/* SPI pins */
#define SPI_MOSI 12
#define SPI_MISO 13
#define SPI_CLK 14
/* BNO085 pins */
#define BNO085_MOSI SPI_MOSI
#define BNO085_MISO SPI_MISO
#define BNO085_CLK SPI_CLK
#define BNO085_CS 45
#define BNO085_RST 48
#define BNO085_INT 47

#endif // CONFIG_H
