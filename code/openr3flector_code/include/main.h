/*
 * This file is part of OpenR3flector
 *
 * Copyright (C) 2026 vantalane <mete@kestech.net>
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

/* Hardware pin definitions are also noted within notes dir in repo root */

#ifndef __MAIN_H
#define __MAIN_H

#include <stdint.h>

#include "functions.h"
#include "math.h"
#include "motor_control.h"
#include "stdio.h"
#include "stdlib.h"
#include "stm32f4xx_hal.h"
#include "string.h"
#include "uart_comms.h"

/*USART definition in the structure of STM example*/
/*USART 1 definitions (CONN P400)*/
#define USARTx USART1
#define USARTx_CLK_ENABLE() __HAL_RCC_USART1_CLK_ENABLE();
#define USARTx_RX_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()
#define USARTx_TX_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()

#define USARTx_FORCE_RESET() __HAL_RCC_USART1_FORCE_RESET()
#define USARTx_RELEASE_RESET() __HAL_RCC_USART1_RELEASE_RESET()

/* Definition for USARTx Pins */
#define USARTx_TX_PIN GPIO_PIN_9
#define USARTx_TX_GPIO_PORT GPIOA
#define USARTx_TX_AF GPIO_AF7_USART1
#define USARTx_RX_PIN GPIO_PIN_10
#define USARTx_RX_GPIO_PORT GPIOA
#define USARTx_RX_AF GPIO_AF7_USART1

/* Definition for USARTx's NVIC */
#define USARTx_IRQn USART1_IRQn
#define USARTx_IRQHandler USART1_IRQHandler

/* USART 2 definitions for RS485 port */
#define USARTx_RS485 USART2
#define USARTx_RS485_CLK_EN() __HAL_RCC_USART2_CLK_ENABLE()
#define USARTx_RS485_GPIO_CLK_EN() __HAL_RCC_GPIOD_CLK_ENABLE()

#define USARTx_RS485_FORCE_RESET() __HAL_RCC_USART2_FORCE_RESET()
#define USARTx_RS485_RELEASE_RESET() __HAL_RCC_USART2_RELEASE_RESET()

/* RS485 pin definitions — datasheet AF layout (PD4=RTS/DE, PD5=TX, PD6=RX) */

#define USARTx_RS485_DE_PIN GPIO_PIN_4
#define USARTx_RS485_DE_GPIO_PORT GPIOD

#define USARTx_RS485_TX_PIN GPIO_PIN_5
#define USARTx_RS485_TX_GPIO_PORT GPIOD
#define USARTx_RS485_TX_AF GPIO_AF7_USART2

#define USARTx_RS485_RX_PIN GPIO_PIN_6
#define USARTx_RS485_RX_GPIO_PORT GPIOD
#define USARTx_RS485_RX_AF GPIO_AF7_USART2

/* NVIC */
#define USARTx_RS485_IRQn USART2_IRQn
#define USARTx_RS485_IRQHandler USART2_IRQHandler

/* HARDWARE PIN ASSIGNMENTS */

/* CM0198 Level shifters enable */
#define LVL_SHIFTERS_EN_PIN GPIO_PIN_1
#define LVL_SHIFTERS_EN_PORT GPIOA

/* U300 -- Alt/Pitch axis motor driver*/
#define U300_RST_AB_PORT GPIOE
#define U300_RST_AB_PIN GPIO_PIN_7
#define U300_FLT_PORT GPIOE
#define U300_FLT_PIN GPIO_PIN_14
#define U300_OTW_PORT GPIOE
#define U300_OTW_PIN GPIO_PIN_15

/* Pitch DOWN actuation pin: TIM1_CH1 (PE9) */
#define PITCH_DOWN_PORT GPIOE
#define PITCH_DOWN_PIN GPIO_PIN_9

/* Pitch UP, complementary TIM1_CH1N (PE8)*/
#define PITCH_UP_PORT GPIOE
#define PITCH_UP_PIN GPIO_PIN_8

/* U350 -- Azim/Yaw axis motor driver*/
#define U350_RST_AB_PORT GPIOB
#define U350_RST_AB_PIN GPIO_PIN_12
#define U350_RST_CD_PORT GPIOB
#define U350_RST_CD_PIN GPIO_PIN_13
#define U350_FLT_PORT GPIOB
#define U350_FLT_PIN GPIO_PIN_14
#define U350_OTW_PORT GPIOB
#define U350_OTW_PIN GPIO_PIN_15

/* Yaw LEFT actuation pin:  TIM1_CH2  (PE11) */
#define YAW_LEFT_PORT GPIOE
#define YAW_LEFT_PIN GPIO_PIN_11

/* Yaw RIGHT actuation p: TIM1_CH2N (PE10) */
#define YAW_RIGHT_PORT GPIOE
#define YAW_RIGHT_PIN GPIO_PIN_10

/* Encoder (single pulse) pin defs. (dir. inferred from mot motion)*/
#define PITCH_ENC_PORT GPIOA
#define PITCH_ENC_PIN GPIO_PIN_6 /* EXTI line 6 -> EXTI9_5_IRQn */
#define YAW_ENC_PORT GPIOA
#define YAW_ENC_PIN GPIO_PIN_7 /* EXTI line 7 -> EXTI9_5_IRQn */

/* Endstops */
#define PITCH_ENDSTOP_PORT GPIOA
#define PITCH_ENDSTOP_PIN GPIO_PIN_3 /* EXTI line 3 -> EXTI3_IRQn */
#define YAW_ENDSTOP_PORT GPIOA
#define YAW_ENDSTOP_PIN GPIO_PIN_4 /* EXTI line 4 -> EXTI4_IRQn */

/*Endstop active level, assert=LOW, (GPIO PULLUP, ENC output is open drain*/
#define ENDSTOP_ACTIVE_LEVEL 0

/* UI LED pin defs*/
#define LED_RED_LEFT_PORT GPIOD
#define LED_RED_LEFT_PIN GPIO_PIN_13
#define LED_RED_RIGHT_PORT GPIOD
#define LED_RED_RIGHT_PIN GPIO_PIN_14
#define LED_FILM_RED_PORT GPIOD
#define LED_FILM_RED_PIN GPIO_PIN_8
#define LED_FILM_GREEN_PORT GPIOD
#define LED_FILM_GREEN_PIN GPIO_PIN_9

/*LED identifiers used with LED_Set() */
#define LED_ID_RED_LEFT 0U
#define LED_ID_RED_RIGHT 1U
#define LED_ID_FILM_RED 2U
#define LED_ID_FILM_GREEN 3U

/*Piezo buzzer*/
#define PIEZO_PORT GPIOD
#define PIEZO_PIN GPIO_PIN_15

/* UI buttons */
#define FILM_BUTTON_PORT GPIOA
#define FILM_BUTTON_PIN GPIO_PIN_0

/*LNBH29 LNB bias IC pin defs*/
/*LNB control pins*/
#define LNB_DSQIN_PORT GPIOA
#define LNB_DSQIN_PIN GPIO_PIN_2
/*lnb i2c addr, with wr/rd shift*/
#define LNBH29_ADDR (0x08 << 1)

/*SERIT supply control stuff*/
#define SERIT_3V3_SUPPLY_EN_PIN GPIO_PIN_1
#define SERIT_1V0_SUPPLY_EN_PIN GPIO_PIN_0
#define SERIT_SUPPLY_PORT GPIOE

#define RELAY_ENABLE_PIN GPIO_PIN_2

/* ============================================================================
 * PWM / Timer configuration
 *
 * Timer clock on STM32F411E @ 100 MHz:
 *   APB2 timers (TIM1): PCLK2 x2 = 100 MHz
 *   APB1 timers (TIM3): PCLK1 x2 = 100 MHz
 *
 * Target PWM frequency: 20 kHz
 *   f = TimerClock / (PSC+1) / (ARR+1)
 *   100e6 / (4+1) / (999+1) = 20 000 Hz
 * ========================================================================== */
#define PIEZO_PERIOD 999U
#define PIEZO_PRESCALER 4U

/*timer configuration for the 22khz tone, TIM9 is 16-bit up counter*/
#define LNB_TIM_PERIOD 908U
#define LNB_TIM_PRESCALER 4U

/*FPGA interfacing PINS*/

#define FPGA_PROG_B_PIN GPIO_PIN_5
#define FPGA_PROG_B_PORT GPIOE

/*TODO add the pins shared between the fpga and the stm32*/

/*Controls for the MXIC flash etc*/
/*The flash is accessed over I2C to SPI bridge, see notes/mmi**/
// #define SC18_ADDR (0x28 << 1)
// #define FLASH_SIZE 0x20000u /* 128 KB */
// #define CHUNK_BYTES 128u

/*TODO, try setting prog HIGH explicitly, to see if fpga does anything.*/

#endif /* __MAIN_H */
