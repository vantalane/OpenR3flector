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

#include "main.h"

#include <stdint.h>
#include <stdio.h>

#include "functions.h"
#include "motor_control.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_uart.h"
#include "uart_comms.h"

char str_buff[64];

static void SystemClock_Config(void);

int main(void)
{
	HAL_Init();

	LED_Init();
	GPIO_Init();

	SystemClock_Config();

	/*configs the interrupt RX and cmd handling*/
	UART_set_config();
	UART_RS485_set_config();

	uart_reply("\033[2J\033[H");
	snprintf(str_buff, sizeof(str_buff),
			 "OpenR3flector " GIT_HASH " " __DATE__ "\n\r");
	uart_reply(str_buff);

	TIM1_PWM_Init();
	TIM4_PIEZO_PWM_init();
	TIM9_LNB_control_tone_init();

	turn_on_serit_supply();

	/*init driver flt pins*/
	Fault_Init();
	/*inits encoder pins*/
	Endstop_Init();
	Encoder_Init();
	Motor_Init();

	pidProcess_Init(&pid_elev);
	pidProcess_Init(&pid_azim);

	init_i2c1_bus(); /*PB6 SCL,		PB7*/
	init_i2c2_bus(); /*PB10 SCL,	PB3*/
	init_i2c3_bus(); /*PA8 SCL, 	PB4*/

	/*
	expecting a 0x20 from stv0903.
		currently have

		(after flashing old code, then switching to custom code)
		i2c1-found I2C device: 0x1F
	i2c1-found I2C device: 0x28 (probably I2C to SPI bridge)
	i2c1-found I2C device: 0x50 (EEPROM)
	i2c2-found I2C device: 0x08 (LNBH29 IC)
	i2c3-found I2C device: 0x68 ()
	*/

	/*TODO, flash dump MXIC for bitstream*/

	uart_reply("Inits done\n\r");
	// UART_SendString("Inits done\n\r");
	// UART_RS485_SendString("Inits done\n\r");

	LED_Set(LED_ID_FILM_GREEN, 1);

	uint32_t now_ticks = 0;
	uint32_t last_control_tick = 0;
	uint32_t last_measure_tick = 0;

	piezo_set_pattern = 2;

	while (1)
	{
		/*poll tx/rx fifos*/
		UART_Poll();
		UART_RS485_Poll();
		/*Run device state machine. */
		Device_StateMachine();
		piezo_pattern_fsm();

		/* dump contents of the EEPROM when triggered*/
		eeprom_uart_dumper();
		flash_uart_dumper();

		now_ticks = HAL_GetTick();
		/*PID and trajectory control of motors*/
		if ((now_ticks - last_control_tick) > CONTROLLER_UPDATE_MS)
		{
			last_control_tick = now_ticks;

			if (check_control_allowed())
			{
				/*Acceleration and velocity measurement/calcs*/
				accel_velo_update();

				/*so that opening devstate can drive the trajectory ctrl*/
				if (device_state == DEV_STATE_OPEN)
					WorldTarget_Update(); /* World frame → body frame →
											 trajectory target */

				Trajectory_Update(); /* Step trajectory planner */
				Controller_Update(); /* Motor control */
			}
		}
	}
}

static void SystemClock_Config(void)
{
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_OscInitTypeDef RCC_OscInitStruct;

	/* Enable Power Control clock */
	__HAL_RCC_PWR_CLK_ENABLE();

	/* The voltage scaling allows optimizing the power consumption when the
	   device is clocked below the maximum system frequency, to update the
	   voltage scaling value regarding system frequency refer to product
	   datasheet.  */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/* Enable HSI Oscillator and activate PLL with HSI as source */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = 0x10;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 16;
	RCC_OscInitStruct.PLL.PLLN = 400;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
	   clocks dividers */
	RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
								   RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
	{
		Error_Handler();
	}
}
