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

#include "functions.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "main.h"
#include "motor_control.h"
#include "stm32f411xe.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_def.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_gpio_ex.h"
#include "stm32f4xx_hal_i2c.h"
#include "stm32f4xx_hal_rcc.h"
#include "stm32f4xx_hal_rcc_ex.h"
#include "stm32f4xx_hal_tim.h"
#include "uart_comms.h"

TIM_HandleTypeDef htim1; /*motor ctrl PWM*/
TIM_HandleTypeDef htim4; /*piezo*/
TIM_HandleTypeDef htim9; /*ch 1 used for LNB tone*/

/*current set buzzer pattern*/
uint8_t piezo_set_pattern = 0;
/*Homing sub-mode, whether to assume unsafe or safe.*/
uint8_t homing_assumes_unsafe = 0;

body_frame_calib_t device_position_calib = {0};
WorldTargetState_t device_world_target = {0};

warble_step_t piezo_program_1[PIEZO_PRG_1_LEN] = {
	{150, 0}, {350, 460}, {300, 0}, {350, 460}, {300, 0}, {350, 460}};

warble_step_t piezo_program_2[PIEZO_PRG_2_LEN] = {
	{80, 575}, {80, 545}, {80, 500}, {80, 475}};

/*For ease programming the pattern, used high/low tone frequencies (hertz)*/
const uint16_t LOWBZ = 386;
const uint16_t HIBZ = 585;
const uint16_t TIMED = 280;

warble_step_t piezo_program_3[PIEZO_PRG_3_LEN] = {
	{TIMED, HIBZ}, {TIMED, LOWBZ}, {TIMED, HIBZ}, {TIMED, LOWBZ},
	{TIMED, HIBZ}, {TIMED, LOWBZ}, {TIMED, HIBZ}, {TIMED, LOWBZ}};

/*short beep special contact*/
warble_step_t piezo_program_4[PIEZO_PRG_4_LEN] = {{10, 0}, {400, 385}};

const uint16_t LOWBZ5 = 400;
const uint16_t HIBZ5 = 600;
const uint16_t TIMED5 = 100;

warble_step_t piezo_program_5[PIEZO_PRG_5_LEN] = {
	{TIMED5, HIBZ5}, {TIMED5, LOWBZ5}, {TIMED5, HIBZ5}, {TIMED5, LOWBZ5},
	{TIMED5, HIBZ5}, {TIMED5, LOWBZ5}, {TIMED5, HIBZ5}, {TIMED5, LOWBZ5}};

/*RWR 2000C new threat tone pattern*/
warble_step_t piezo_program_6[PIEZO_PRG_6_LEN] = {{10, 0}, {450, 1900}};

const uint16_t LOWBZ7 = 0;
const uint16_t HIBZ7 = 1950;
const uint16_t TIMED7 = 20;
const uint16_t STIMED7 = 30;

warble_step_t piezo_program_7[PIEZO_PRG_7_LEN] = {
	{STIMED7, LOWBZ7}, {TIMED7, HIBZ7}, {STIMED7, LOWBZ7}, {TIMED7, HIBZ7},
	{STIMED7, LOWBZ7}, {TIMED7, HIBZ7}, {STIMED7, LOWBZ7}, {TIMED7, HIBZ7},
	{STIMED7, LOWBZ7}, {TIMED7, HIBZ7}, {STIMED7, LOWBZ7}, {TIMED7, HIBZ7},
	{STIMED7, LOWBZ7}, {TIMED7, HIBZ7}, {STIMED7, LOWBZ7}, {TIMED7, HIBZ7},
};

void GPIO_Init(void)
{
	GPIO_InitTypeDef gpio = {0};

	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();

	/* ---- GPIOA enable level shifter's IO */
	gpio.Pin = LVL_SHIFTERS_EN_PIN;
	gpio.Mode = GPIO_MODE_OUTPUT_PP;
	gpio.Pull = GPIO_PULLDOWN;
	gpio.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LVL_SHIFTERS_EN_PORT, &gpio);
	/* turn it on until after init */
	HAL_GPIO_WritePin(LVL_SHIFTERS_EN_PORT, LVL_SHIFTERS_EN_PIN, GPIO_PIN_SET);

	/* ---- GPIOA input: film button */
	gpio.Pin = FILM_BUTTON_PIN;
	gpio.Mode = GPIO_MODE_INPUT;
	gpio.Pull = GPIO_PULLUP;
	gpio.Speed = GPIO_SPEED_FAST;
	HAL_GPIO_Init(GPIOA, &gpio);

	/* ---- GPIOB outputs: U350 reset lines  */
	gpio.Pin = U350_RST_AB_PIN | U350_RST_CD_PIN;
	gpio.Mode = GPIO_MODE_OUTPUT_PP;
	gpio.Pull = GPIO_NOPULL;
	gpio.Speed = GPIO_SPEED_FAST;
	HAL_GPIO_Init(GPIOB, &gpio);
	/* Hold in reset until Motor_Init() releases */
	HAL_GPIO_WritePin(GPIOB, U350_RST_AB_PIN | U350_RST_CD_PIN, GPIO_PIN_RESET);

	/* ---- GPIOE outputs: U300 reseth, serit LDO's, relay control
	 * ------------------------- */
	/*	PE1 - Serit 3v3 LDO enable (VERIFIED)
		PE0 - Serit 1V0 enable (VERIFIED)
		PE2 - Controls subminiature relay*/

	gpio.Pin = U300_RST_AB_PIN | SERIT_3V3_SUPPLY_EN_PIN |
			   SERIT_1V0_SUPPLY_EN_PIN | RELAY_ENABLE_PIN;
	gpio.Mode = GPIO_MODE_OUTPUT_PP;
	gpio.Pull = GPIO_NOPULL;
	gpio.Speed = GPIO_SPEED_FAST;

	HAL_GPIO_Init(GPIOE, &gpio);
	HAL_GPIO_WritePin(U300_RST_AB_PORT,
					  U300_RST_AB_PIN | SERIT_3V3_SUPPLY_EN_PIN |
						  SERIT_1V0_SUPPLY_EN_PIN | RELAY_ENABLE_PIN,
					  GPIO_PIN_RESET);
}

/*enable 1v0 and 3v3 LDO's for SERIT*/
void turn_on_serit_supply()
{
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_SET);
}

/*disable 1v0 and 3v3 LDO's for SERIT*/
void turn_off_serit_supply()
{
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_RESET);
}

/*enable the mystery relay (bias passthru?)*/
void turn_on_relay()
{
	HAL_GPIO_WritePin(GPIOE, RELAY_ENABLE_PIN, GPIO_PIN_SET);
}
/*disable the mystery relay (bias passthru?)*/
void turn_off_relay()
{
	HAL_GPIO_WritePin(GPIOE, RELAY_ENABLE_PIN, GPIO_PIN_RESET);
}

uint8_t lnb_22khz_running = 0;

/*	disable 22kHz tone
	LNB LO:9.75 GHz IF:950-1,950MHz
	Freq band 10.7-11.7Ghz
*/
void set_lnb_lo_low(void)
{
	if (lnb_22khz_running == 0)
	{
		return;
	}
	lnb_22khz_running = 0;

	HAL_StatusTypeDef lnb_tim_ret = HAL_TIM_PWM_Stop(&htim9, TIM_CHANNEL_1);
	if (lnb_tim_ret != HAL_OK)
	{
		uart_reply("Setting LNB LO control PWM off FAILED.");
	}
}

/*	enable 22kHz tone
	LNB LO:10.60 GHz IF:1,100-2,150MHz
	Freq band 11.70-12.75 GHz
*/
void set_lnb_lo_high(void)
{
	if (lnb_22khz_running != 0)
	{
		return;
	}
	lnb_22khz_running = 1;

	HAL_StatusTypeDef lnb_tim_ret = HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
	if (lnb_tim_ret != HAL_OK)
	{
		uart_reply("Setting LNB LO control PWM ON FAILED.");
	}
}

/*
 * TIM1 (APB2, 100 MHz) configured for centre-aligned PWM at 20 kHz.
 * Channels used:
 *   CH1  / CH1N  -> PE9 / PE8   (pitch DOWN / UP)
 * 		In sign magnitude mode. PE9 is PWM, PE8 is static gpio.
 *   CH2  / CH2N  -> PE11 / PE10 (yaw LEFT / RIGHT, complementary pair)
 * PE11, PWM. PE10 static gpio.
 */

void TIM1_PWM_Init(void)
{
	TIM_BreakDeadTimeConfigTypeDef bdt = {0};
	TIM_OC_InitTypeDef tim1_oc = {0};

	__HAL_RCC_TIM1_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();

	/*PWM pins PE9, PE11*/
	/*Sign GPIO pins PE8 PE10*/

	/*pitch and yaw PWM gpio configurations*/
	GPIO_InitTypeDef pwm_gpio = {0};
	pwm_gpio.Pin = PITCH_DOWN_PIN | PITCH_UP_PIN | YAW_LEFT_PIN | YAW_RIGHT_PIN;
	pwm_gpio.Mode = GPIO_MODE_AF_PP;
	pwm_gpio.Pull = GPIO_NOPULL;
	pwm_gpio.Speed = GPIO_SPEED_FREQ_HIGH;
	pwm_gpio.Alternate = GPIO_AF1_TIM1;
	HAL_GPIO_Init(GPIOE, &pwm_gpio);

	/*Timer 1  PITCH and YAW PWM timer setup*/
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = PWM_PRESCALER;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = PWM_PERIOD;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	HAL_TIM_PWM_Init(&htim1);

	/* Break and dead-time: MOE must be set for complementary outputs         */
	bdt.OffStateRunMode = TIM_OSSR_DISABLE;
	bdt.OffStateIDLEMode = TIM_OSSI_DISABLE;
	bdt.LockLevel = TIM_LOCKLEVEL_OFF;
	bdt.DeadTime = 0;
	bdt.BreakState = TIM_BREAK_DISABLE;
	bdt.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	bdt.AutomaticOutput = TIM_AUTOMATICOUTPUT_ENABLE;
	HAL_TIMEx_ConfigBreakDeadTime(&htim1, &bdt);

	/* CH1 -- pitch DOWN (PE9 main, PE8 complementary) */
	tim1_oc.OCMode = TIM_OCMODE_PWM1;
	tim1_oc.Pulse = 0;
	tim1_oc.OCPolarity = TIM_OCPOLARITY_HIGH;
	tim1_oc.OCNPolarity = TIM_OCNPOLARITY_LOW;
	tim1_oc.OCFastMode = TIM_OCFAST_DISABLE;
	tim1_oc.OCIdleState = TIM_OCIDLESTATE_RESET;
	tim1_oc.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	HAL_TIM_PWM_ConfigChannel(&htim1, &tim1_oc, TIM_CHANNEL_1);

	/* CH2 -- yaw LEFT/RIGHT (PE11) */
	HAL_TIM_PWM_ConfigChannel(&htim1, &tim1_oc, TIM_CHANNEL_2);
}

void TIM4_PIEZO_PWM_init(void)
{
	TIM_OC_InitTypeDef tim4_oc = {0};
	GPIO_InitTypeDef gpio_piezo = {0};

	/*timer 4 channel 4 available. piezo is on PD15*/
	__HAL_RCC_TIM4_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	gpio_piezo.Pin = PIEZO_PIN;
	gpio_piezo.Mode = GPIO_MODE_AF_PP;
	gpio_piezo.Pull = GPIO_PULLDOWN; /*i'm scared of transients*/
	gpio_piezo.Speed = GPIO_SPEED_FREQ_HIGH;
	gpio_piezo.Alternate = GPIO_AF2_TIM4;
	HAL_GPIO_Init(GPIOD, &gpio_piezo);

	htim4.Instance = TIM4;
	htim4.Init.Prescaler = PIEZO_PRESCALER;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = PIEZO_PERIOD;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.RepetitionCounter = 0;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

	HAL_TIM_PWM_Init(&htim4);

	tim4_oc.OCMode = TIM_OCMODE_PWM1;
	tim4_oc.Pulse = 0;
	tim4_oc.OCPolarity = TIM_OCPOLARITY_HIGH;
	tim4_oc.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	tim4_oc.OCFastMode = TIM_OCFAST_DISABLE;
	tim4_oc.OCIdleState = TIM_OCIDLESTATE_RESET;
	tim4_oc.OCNIdleState = TIM_OCNIDLESTATE_RESET;

	HAL_TIM_PWM_ConfigChannel(&htim4, &tim4_oc, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
}

/*PA2 is connected to DiSEqC tone input DSQIN port of LNBH29*/
void TIM9_LNB_control_tone_init(void)
{
	TIM_OC_InitTypeDef tim9_oc = {0};
	GPIO_InitTypeDef gpio_lnb_tone = {0};

	/*timer 9 channel 1 is usable*/

	__HAL_RCC_TIM9_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	gpio_lnb_tone.Pin = LNB_DSQIN_PIN;
	gpio_lnb_tone.Mode = GPIO_MODE_AF_PP;
	gpio_lnb_tone.Pull = GPIO_PULLDOWN;
	gpio_lnb_tone.Speed = GPIO_SPEED_FREQ_HIGH;
	gpio_lnb_tone.Alternate = GPIO_AF3_TIM9;
	HAL_GPIO_Init(LNB_DSQIN_PORT, &gpio_lnb_tone);

	htim9.Instance = TIM9;
	htim9.Init.Prescaler = LNB_TIM_PRESCALER;  // need 22kHz
	htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim9.Init.Period = LNB_TIM_PERIOD;
	htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim9.Init.RepetitionCounter = 0;
	htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	HAL_TIM_PWM_Init(&htim9);

	tim9_oc.OCMode = TIM_OCMODE_PWM1;
	tim9_oc.Pulse = LNB_TIM_PERIOD / 2;
	tim9_oc.OCPolarity = TIM_OCPOLARITY_HIGH;
	tim9_oc.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	tim9_oc.OCFastMode = TIM_OCFAST_DISABLE;
	tim9_oc.OCIdleState = TIM_OCIDLESTATE_RESET;
	tim9_oc.OCNIdleState = TIM_OCNIDLESTATE_RESET;

	HAL_TIM_PWM_ConfigChannel(&htim9, &tim9_oc, TIM_CHANNEL_1);
	// HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
}

void Encoder_Init(void)
{
	GPIO_InitTypeDef gpio = {0};

	__HAL_RCC_GPIOA_CLK_ENABLE();

	gpio.Pin = PITCH_ENC_PIN | YAW_ENC_PIN;
	gpio.Mode = GPIO_MODE_IT_RISING;
	gpio.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &gpio);

	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

int32_t Encoder_GetPitch(void)
{
	return pitch_axis.counts;
}
int32_t Encoder_GetYaw(void)
{
	return yaw_axis.counts;
}

/*returns the current pos in degrees*/
float get_curr_body_az_pos()
{
	return ((float)yaw_axis.counts / COUNTS_PER_DEG_YAW);
}

/*returns the current pos in degrees*/
float get_curr_body_alt_pos()
{
	return ((float)pitch_axis.counts - PITCH_ENC_HORIZON_OFFSET) /
		   COUNTS_PER_DEG_PITCH;
}

/*convert current body az/alt pos to world frame*/
void get_curr_world_az_alt_pos(float* world_az, float* world_alt)
{
	float body_az = get_curr_body_az_pos();
	float body_alt = get_curr_body_alt_pos();

	body_to_world(&device_position_calib, body_az, body_alt, world_az,
				  world_alt);
}

/*returns the target pos in body frame in degrees*/
float get_body_az_pos_target()
{
	return ((float)yaw_traj.raw_target / COUNTS_PER_DEG_YAW);
}

/*returns the target pos in body frame in degrees*/
float get_body_alt_pos_target()
{
	return ((float)pitch_traj.raw_target - PITCH_ENC_HORIZON_OFFSET) /
		   COUNTS_PER_DEG_PITCH;
}

void Encoder_ResetPitch(void)
{
	__disable_irq();
	pitch_axis.counts = 0;
	__enable_irq();
}

void Encoder_ResetYaw(void)
{
	__disable_irq();
	yaw_axis.counts = 0;
	__enable_irq();
}

void Endstop_Init(void)
{
	GPIO_InitTypeDef gpio_interrupt = {0};

	__HAL_RCC_GPIOA_CLK_ENABLE();

	gpio_interrupt.Pin = PITCH_ENDSTOP_PIN;
	gpio_interrupt.Pull = GPIO_NOPULL;
	gpio_interrupt.Mode = GPIO_MODE_IT_RISING_FALLING;

	HAL_GPIO_Init(PITCH_ENDSTOP_PORT, &gpio_interrupt);

	HAL_NVIC_SetPriority(EXTI3_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(EXTI3_IRQn);

	gpio_interrupt.Pin = YAW_ENDSTOP_PIN;
	HAL_GPIO_Init(YAW_ENDSTOP_PORT, &gpio_interrupt);

	HAL_NVIC_SetPriority(EXTI4_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(EXTI4_IRQn);
}

uint8_t Endstop_Pitch_IsTriggered(void)
{
	GPIO_PinState state =
		HAL_GPIO_ReadPin(PITCH_ENDSTOP_PORT, PITCH_ENDSTOP_PIN);

	/*ACTIVE when LOW.*/
	if (state == 1)
		return 0;
	else
		return 1;
}

uint8_t Endstop_Yaw_IsTriggered(void)
{
	GPIO_PinState state = HAL_GPIO_ReadPin(YAW_ENDSTOP_PORT, YAW_ENDSTOP_PIN);

	/*ACTIVE when LOW.*/
	if (state == 1)
		return 0;
	else
		return 1;
}

/*returns 0 when running, 1 when done, 2 when faulty.*/
uint8_t Homing_Run(void)
{
	char textbuff[100];

	static uint8_t homing_state;
	static uint32_t start_tick;
	static uint32_t activity_tick;
	static int32_t last_yaw_enc_cnt = 0;
	static int32_t last_pitch_enc_cnt = 0;

	yaw_axis.at_endstop = Endstop_Yaw_IsTriggered();
	pitch_axis.at_endstop = Endstop_Pitch_IsTriggered();

	/*To ensure accurate homing, detect a fresh endstop trigger*/
	static uint8_t pitch_was_untrig = 0;
	static uint8_t yaw_was_untrig = 0;

	/* Keep an eye on whether the encoders see movement */
	if ((last_yaw_enc_cnt != Encoder_GetYaw() ||
		 (last_pitch_enc_cnt != Encoder_GetPitch())))
	{
		activity_tick = HAL_GetTick();
		last_yaw_enc_cnt = Encoder_GetYaw();
		last_pitch_enc_cnt = Encoder_GetPitch();
	}

	/*prevent timeout at init, timo if no activity > timo time*/
	if (homing_state != 0)
	{
		if ((HAL_GetTick() - activity_tick) > HOMING_TIMEOUT_MS)
		{
			Motor_Stop_All();
			UART_SendString("Homing timed out\n");
			homing_state = 0;
			return 2;
		}

		{
			static uint32_t last_print_tick = 0;
			static uint8_t last_state = 0;

			if (((HAL_GetTick() - last_print_tick) >= 1000) ||
				(homing_state != last_state))
			{
				last_state = homing_state;
				last_print_tick = HAL_GetTick();
				sprintf(textbuff,
						"ST%u Enc cnts: Y%ld, P%ld, Pendstp %d Yendstp %d, "
						"timo cnt "
						"%lu/%u\n",
						homing_state, Encoder_GetYaw(), Encoder_GetPitch(),
						pitch_axis.at_endstop, yaw_axis.at_endstop,
						(HAL_GetTick() - activity_tick), HOMING_TIMEOUT_MS);

				UART_SendString(textbuff);
			}
		}
	}

	/*track untriggeredness*/
	if (!yaw_axis.at_endstop) yaw_was_untrig = 1;
	if (!pitch_axis.at_endstop) pitch_was_untrig = 1;

	switch (homing_state)
	{
		case 0:
			/*GET tick of start*/
			start_tick = HAL_GetTick();
			activity_tick = HAL_GetTick();
			yaw_was_untrig = 0;
			pitch_was_untrig = 0;

			if (homing_assumes_unsafe)
				homing_state = 2;
			else
				homing_state = 1;

			break;

		case 1: /*YAW calibration*/

			if (yaw_was_untrig)
			{
				if (yaw_axis.at_endstop)
				{
					Yaw_SetDuty(0);
					Encoder_ResetYaw();
					/*go to pitch calibration*/
					homing_state = 2;
				}
				else
					Yaw_SetDuty(-HOMING_DUTY);
			}
			else
				Yaw_SetDuty(HOMING_DUTY);
			break;

		case 2: /*PITCH calibration*/

			if (pitch_was_untrig)
			{
				if (pitch_axis.at_endstop)
				{
					Encoder_ResetPitch();

					if (homing_assumes_unsafe)
					{
						homing_state = 3; /*set pitch to safe pos*/
					}
					else
					{
						Pitch_SetDuty(0);
						homing_state = 10; /* Calib done */
					}
				}
				else
					Pitch_SetDuty(-HOMING_DUTY);
			}
			else
				Pitch_SetDuty(HOMING_DUTY);
			break;

		case 3: /*in case of unsafe position, set pitch safe*/

			if (Encoder_GetPitch() >= 510)
			{
				Pitch_SetDuty(0);
				homing_assumes_unsafe = 0; /*unset unsafe flag*/
				homing_state = 1;
			}
			else
				Pitch_SetDuty(HOMING_DUTY);

			break;

		case 10:
			/*HOMING DONE*/
			Yaw_SetDuty(0);
			Pitch_SetDuty(0);

			UART_SendString("Homing state finished\n");

			homing_state = 0;
			yaw_was_untrig = 0;
			pitch_was_untrig = 0;
			return 1;

			break;

			/*used as error state*/
		default:
		{
			UART_SendString("Unexpected homing state\n");
			homing_state = 0;
			return 2;
		}
		break;
	}

	return 0;
}

/*mini FSM to close the device.
0 is running, 1 is OK, 2 is NOK*/
uint8_t closing_run()
{
	static uint8_t closing_state = 10;
	static uint32_t activity_tick;
	static int32_t last_yaw_enc_cnt = 0;
	static int32_t last_pitch_enc_cnt = 0;

	if ((last_yaw_enc_cnt != Encoder_GetYaw() ||
		 (last_pitch_enc_cnt != Encoder_GetPitch())))
	{
		activity_tick = HAL_GetTick();
		last_yaw_enc_cnt = Encoder_GetYaw();
		last_pitch_enc_cnt = Encoder_GetPitch();
	}

	/*prevent timeout at init, timo if no activity > timo time*/
	if (closing_state != 10)
	{
		if ((HAL_GetTick() - activity_tick) > CLOSING_TIMEOUT_MS)
		{
			Motor_Stop_All();
			UART_SendString("Closing failed: timed out\n");
			closing_state = 10;
			return 2;
		}
	}

	switch (closing_state)
	{
		case 0: /*set pitch safe*/
			pitch_traj.raw_target = MIN_PITCH_ENC_POS + (CTRL_DEADBAND * 10);
			closing_state = 1;
			break;

		case 1: /*set yaw 0*/

			if (pitch_axis.counts >= (MIN_PITCH_ENC_POS + (CTRL_DEADBAND * 2)))
			{
				yaw_traj.raw_target = 0;
				closing_state = 2;
			}
			break;

		case 2: /*puting yaw 0, then pitch*/

			/*is yaw at setpoint or triggered endstop*/
			if ((yaw_axis.counts <= CTRL_DEADBAND) || Endstop_Yaw_IsTriggered())
			{
				pitch_traj.raw_target = 0;
				closing_state = 3;
			}
			break;

		case 3: /*done*/

			if ((pitch_axis.counts <= CTRL_DEADBAND) ||
				Endstop_Pitch_IsTriggered())
			{
				UART_SendString("Close run done\n");
				closing_state = 10;
				return 1;
			}
			break;

		case 4: /*fail state, init's homing run*/
			UART_SendString("Closing failed, triggering re-home\n");
			device_state = DEV_STATE_HOMING;
			closing_state = 10;
			return 2;
			break;

		case 10: /*START state*/
			/*grab initial start time for timo*/
			activity_tick = HAL_GetTick();
			closing_state = 0;
			break;

		default:
			UART_SendString("Closing unknown state error\n");
			closing_state = 10;
			return 2;

			break;
	}

	return 0;
}

void LED_Init(void)
{
	GPIO_InitTypeDef gpio = {0};

	__HAL_RCC_GPIOD_CLK_ENABLE();

	gpio.Mode = GPIO_MODE_OUTPUT_PP;
	gpio.Pull = GPIO_NOPULL;
	gpio.Speed = GPIO_SPEED_FAST;

	gpio.Pin = LED_RED_LEFT_PIN | LED_RED_RIGHT_PIN | LED_FILM_RED_PIN |
			   LED_FILM_GREEN_PIN | PIEZO_PIN;

	HAL_GPIO_Init(GPIOD, &gpio);

	/*Set all ACTUALLY OFF*/
	LED_Set(LED_ID_FILM_GREEN, 0);
	LED_Set(LED_ID_FILM_RED, 0);
	LED_Set(LED_ID_RED_LEFT, 0);
	LED_Set(LED_ID_RED_RIGHT, 0);
}

void LED_Set(uint8_t led_id, uint8_t state)
{
	GPIO_TypeDef* port;
	uint16_t pin;
	GPIO_PinState s = state ? GPIO_PIN_SET : GPIO_PIN_RESET;

	switch (led_id)
	{
		case LED_ID_RED_LEFT:
			port = LED_RED_LEFT_PORT;
			pin = LED_RED_LEFT_PIN;
			break;

		case LED_ID_RED_RIGHT:
			port = LED_RED_RIGHT_PORT;
			pin = LED_RED_RIGHT_PIN;
			break;

		case LED_ID_FILM_RED:
			/*Invert polarity*/
			s = state ? GPIO_PIN_RESET : GPIO_PIN_SET;
			port = LED_FILM_RED_PORT;
			pin = LED_FILM_RED_PIN;
			break;

		case LED_ID_FILM_GREEN:
			/*Invert polarity*/
			s = state ? GPIO_PIN_RESET : GPIO_PIN_SET;
			port = LED_FILM_GREEN_PORT;
			pin = LED_FILM_GREEN_PIN;
			break;

		default:
			return;
	}

	HAL_GPIO_WritePin(port, pin, s);
}
void LED_Toggle(uint8_t led_id)
{
	GPIO_TypeDef* port;
	uint16_t pin;

	switch (led_id)
	{
		case LED_ID_RED_LEFT:
			port = LED_RED_LEFT_PORT;
			pin = LED_RED_LEFT_PIN;
			break;

		case LED_ID_RED_RIGHT:
			port = LED_RED_RIGHT_PORT;
			pin = LED_RED_RIGHT_PIN;
			break;

		case LED_ID_FILM_RED:
			port = LED_FILM_RED_PORT;
			pin = LED_FILM_RED_PIN;
			break;

		case LED_ID_FILM_GREEN:
			port = LED_FILM_GREEN_PORT;
			pin = LED_FILM_GREEN_PIN;
			break;

		default:
			return;
	}

	HAL_GPIO_TogglePin(port, pin);
}

/*config timer for freq.*/
void piezo_set_frequency(uint32_t freq_hz)
{
	/*turn on/off*/
	if (freq_hz == 0)
	{
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
		return;
	}

	uint32_t period = (48000000UL / ((PIEZO_PRESCALER + 1) * freq_hz)) - 1;

	if (period < 1) period = 1;
	if (period > 65535) period = 65535;

	__HAL_TIM_SET_AUTORELOAD(&htim4, period);

	if (period < 1) period = 1;
	if (period > 65535) period = 65535;

	__HAL_TIM_SET_AUTORELOAD(&htim4, period);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, (period / 2));
}

/*Piezo beep pattern runner fsm*/
void piezo_pattern_fsm()
{
	static uint32_t step_increment_ticks;
	static uint8_t set_pattern;
	static uint8_t tone_step;
	static uint8_t piezo_fsm_state = 0;

	switch (piezo_fsm_state)
	{
		case 0:
			/*idle*/
			if (piezo_set_pattern != 0)
			{
				piezo_fsm_state = 1;
			}
			break;

		case 1: /*start toning*/
			set_pattern = piezo_set_pattern;
			tone_step = 0;
			piezo_fsm_state = 2;
			break;

		case 2: /*set tone*/
		{
			switch (set_pattern)
			{
				case 0:
					/*nowork*/
					break;

				case 1:
					piezo_set_frequency(piezo_program_1[tone_step].frequency);
					step_increment_ticks =
						HAL_GetTick() + piezo_program_1[tone_step].tone_ms;
					/*if all steps ran*/
					if (tone_step >= PIEZO_PRG_1_LEN)
						piezo_fsm_state = 4;
					else
						piezo_fsm_state = 3;
					tone_step++;
					break;

				case 2:
					piezo_set_frequency(piezo_program_2[tone_step].frequency);
					step_increment_ticks =
						HAL_GetTick() + piezo_program_2[tone_step].tone_ms;
					/*if all steps ran*/
					if (tone_step >= PIEZO_PRG_2_LEN)
						piezo_fsm_state = 4;
					else
						piezo_fsm_state = 3;

					tone_step++;
					break;

				case 3:
					piezo_set_frequency(piezo_program_3[tone_step].frequency);
					step_increment_ticks =
						HAL_GetTick() + piezo_program_3[tone_step].tone_ms;
					/*if all steps ran*/
					if (tone_step >= PIEZO_PRG_3_LEN)
						piezo_fsm_state = 4;
					else
						piezo_fsm_state = 3;
					tone_step++;
					break;

				case 4:
					piezo_set_frequency(piezo_program_4[tone_step].frequency);
					step_increment_ticks =
						HAL_GetTick() + piezo_program_4[tone_step].tone_ms;
					/*if all steps ran*/
					if (tone_step >= PIEZO_PRG_4_LEN)
						piezo_fsm_state = 4;
					else
						piezo_fsm_state = 3;
					tone_step++;
					break;

				case 5:
					piezo_set_frequency(piezo_program_5[tone_step].frequency);
					step_increment_ticks =
						HAL_GetTick() + piezo_program_5[tone_step].tone_ms;
					/*if all steps ran*/
					if (tone_step >= PIEZO_PRG_5_LEN)
						piezo_fsm_state = 4;
					else
						piezo_fsm_state = 3;
					tone_step++;
					break;

				case 6:
					piezo_set_frequency(piezo_program_6[tone_step].frequency);
					step_increment_ticks =
						HAL_GetTick() + piezo_program_6[tone_step].tone_ms;
					/*if all steps ran*/
					if (tone_step >= PIEZO_PRG_6_LEN)
						piezo_fsm_state = 4;
					else
						piezo_fsm_state = 3;
					tone_step++;
					break;
				case 7:
					piezo_set_frequency(piezo_program_7[tone_step].frequency);
					step_increment_ticks =
						HAL_GetTick() + piezo_program_7[tone_step].tone_ms;
					/*if all steps ran*/
					if (tone_step >= PIEZO_PRG_7_LEN)
						piezo_fsm_state = 4;
					else
						piezo_fsm_state = 3;
					tone_step++;
					break;

				default:
					piezo_fsm_state = 4;
					break;
			}
		}
		break;

		case 3: /*wait next tone time*/
			if (HAL_GetTick() >= step_increment_ticks) piezo_fsm_state = 2;
			break;

		case 4: /*finished*/
			piezo_set_frequency(0);

			/*check if there is a new pattern given while busy, if no, zero*/
			if (set_pattern == piezo_set_pattern) piezo_set_pattern = 0;
			piezo_fsm_state = 0;
			break;

		default:
			piezo_fsm_state = 0;
			break;
	}
}

void Fault_Init(void)
{
	GPIO_InitTypeDef gpio = {0};

	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	gpio.Mode = GPIO_MODE_INPUT;
	gpio.Pull = GPIO_PULLUP; /* DRV8432 FLT/OTW are open-drain active-low */

	/* U300 fault pins on GPIOE */
	gpio.Pin = U300_FLT_PIN | U300_OTW_PIN;
	HAL_GPIO_Init(GPIOE, &gpio);

	/* U350 fault pins on GPIOB */
	gpio.Pin = U350_FLT_PIN | U350_OTW_PIN;
	HAL_GPIO_Init(GPIOB, &gpio);
}

uint8_t Fault_Check_U300(void)
{
	/* Active-low: GPIO_PIN_RESET means asserted */
	uint8_t flt =
		(HAL_GPIO_ReadPin(U300_FLT_PORT, U300_FLT_PIN) == GPIO_PIN_RESET) ? 1U
																		  : 0U;
	uint8_t otw =
		(HAL_GPIO_ReadPin(U300_OTW_PORT, U300_OTW_PIN) == GPIO_PIN_RESET) ? 1U
																		  : 0U;
	return flt | otw;
}

uint8_t Fault_Check_U350(void)
{
	uint8_t flt =
		(HAL_GPIO_ReadPin(U350_FLT_PORT, U350_FLT_PIN) == GPIO_PIN_RESET) ? 1U
																		  : 0U;
	uint8_t otw =
		(HAL_GPIO_ReadPin(U350_OTW_PORT, U350_OTW_PIN) == GPIO_PIN_RESET) ? 1U
																		  : 0U;
	return flt | otw;
}

void Fault_Handler(void)
{
	Motor_Stop_All();
	UART_SendString("FAULT: motor driver fault or over-temperature\n");
}

void Error_Handler(void)
{
	/* Turn LED5 on */
	while (1)
	{
		LED_Toggle(LED_ID_FILM_RED);
		HAL_Delay(80);
	}
}

float wrap_delta(float delta_rad)
{
	while (delta_rad > 3.14159265f) delta_rad -= 2.0f * 3.14159265f;
	while (delta_rad < -3.14159265f) delta_rad += 2.0f * 3.14159265f;
	return delta_rad;
}

void add_sync_point(uint8_t sync_nr, body_frame_calib_t* c, float body_az,
					float body_alt, float world_az, float world_alt)
{
	if ((sync_nr < 1) || (sync_nr > 3))
	{
		uart_reply("Wrong sync point number\n");
		return;
	}

	float* slot = (sync_nr == 1)   ? c->calib_pos_1
				  : (sync_nr == 2) ? c->calib_pos_2
								   : c->calib_pos_3;
	slot[0] = body_az;
	slot[1] = body_alt;
	slot[2] = world_az;
	slot[3] = world_alt;
	c->calib_pos_bitmap |= (uint8_t)(1u << (sync_nr - 1));

	uint8_t n = (uint8_t)__builtin_popcount(c->calib_pos_bitmap);

	if (n == 1)
	{
		c->azimuth_body_offset =
			wrap_delta((world_az - body_az) * DEG2RAD) / DEG2RAD;
		c->elevation_body_offset = world_alt - body_alt;
		c->az_0_tilt = 0.0f;
		c->az_90_tilt = 0.0f;
		c->calibration_usable = 1;
	}
	else
	{
		body_frame_solve(c);
	}
}

/*
 * Solve 4-parameter pointing model from 3 calibration positions.
 *
 * calib_pos_N[4] = { body_az_deg, body_alt_deg,
 *                    world_az_deg, world_alt_deg }
 *
 * solved parameters stored in radians:
 *   azimuth_body_offset   (IA)
 *   elevation_body_offset (IE)
 *   az_0_tilt             (AN)
 *   az_90_tilt            (AW)
 *
 * returns false if observations are geometrically degenerate.
 */
uint8_t body_frame_solve(body_frame_calib_t* c)
{
	const float* all_obs[3] = {c->calib_pos_1, c->calib_pos_2, c->calib_pos_3};
	const float* obs[3];
	int n_obs = 0;
	for (int i = 0; i < 3; i++)
		if (c->calib_pos_bitmap & (1u << i)) obs[n_obs++] = all_obs[i];

	if (n_obs < 2)
	{
		c->calibration_usable = 0;
		return 0;
	}

	float AtA[4][4];
	float Atb[4];
	memset(AtA, 0, sizeof(AtA));
	memset(Atb, 0, sizeof(Atb));

	for (int k = 0; k < n_obs; k++)
	{
		float body_az = obs[k][0] * DEG2RAD;
		float body_alt = obs[k][1] * DEG2RAD;
		float world_az = obs[k][2] * DEG2RAD;
		float world_alt = obs[k][3] * DEG2RAD;

		float d_el = wrap_delta(world_alt - body_alt);
		float d_az = wrap_delta(world_az - body_az);

		float c_az = cosf(body_az);
		float s_az = sinf(body_az);
		float t_al = tanf(body_alt);

		/* column order: [ IA,  IE,        AN,           AW        ] */
		float row_el[4] = {0.0f, 1.0f, c_az, s_az};
		float row_az[4] = {1.0f, 0.0f, s_az * t_al, -c_az * t_al};

		for (int i = 0; i < 4; i++)
		{
			Atb[i] += row_el[i] * d_el + row_az[i] * d_az;
			for (int j = 0; j < 4; j++)
				AtA[i][j] += row_el[i] * row_el[j] + row_az[i] * row_az[j];
		}
	}

	/* Gaussian elimination with partial pivoting on [AtA | Atb] */
	float M[4][5];
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++) M[i][j] = AtA[i][j];
		M[i][4] = Atb[i];
	}

	for (int col = 0; col < 4; col++)
	{
		/* Find pivot */
		int pivot = col;
		for (int row = col + 1; row < 4; row++)
			if (fabsf(M[row][col]) > fabsf(M[pivot][col])) pivot = row;

		if (fabsf(M[pivot][col]) < 1e-9f)
		{
			c->calibration_usable = 0;
			return 0; /* degenerate: observations too close in azimuth */
		}

		/* Swap rows */
		if (pivot != col)
			for (int j = 0; j < 5; j++)
			{
				float tmp = M[col][j];
				M[col][j] = M[pivot][j];
				M[pivot][j] = tmp;
			}

		/* Eliminate below */
		float inv = 1.0f / M[col][col];
		for (int row = col + 1; row < 4; row++)
		{
			float f = M[row][col] * inv;
			for (int j = col; j < 5; j++) M[row][j] -= f * M[col][j];
		}
	}

	/* Back substitution */
	float result[4];
	for (int i = 3; i >= 0; i--)
	{
		float sum = M[i][4];
		for (int j = i + 1; j < 4; j++) sum -= M[i][j] * result[j];
		result[i] = sum / M[i][i];
	}

	/* Write back in degrees — consistent with the rest of the codebase */
	c->azimuth_body_offset = result[0] / DEG2RAD;	/* IA */
	c->elevation_body_offset = result[1] / DEG2RAD; /* IE */
	c->az_0_tilt = result[2] / DEG2RAD;				/* AN */
	c->az_90_tilt = result[3] / DEG2RAD;			/* AW */

	c->calibration_usable = 2;
	return 1;
}

/* World-frame az/alt (degrees) → body-frame az/alt (degrees) to command the
 * mount. */
void world_to_body(const body_frame_calib_t* c, float world_az, float world_alt,
				   float* body_az, float* body_alt)
{
	if (c->calibration_usable == 0)
	{
		*body_az = world_az;
		*body_alt = world_alt;
		return;
	}

	float az_r = world_az * DEG2RAD;
	float alt_r = world_alt * DEG2RAD;

	float IA = c->azimuth_body_offset;
	float IE = c->elevation_body_offset;
	float AN = c->az_0_tilt;
	float AW = c->az_90_tilt;

	*body_alt = world_alt - (IE + AN * cosf(az_r) + AW * sinf(az_r));
	*body_az =
		world_az - (IA + (AN * sinf(az_r) - AW * cosf(az_r)) * tanf(alt_r));

	while (*body_az >= 360.0f) *body_az -= 360.0f;
	while (*body_az < 0.0f) *body_az += 360.0f;
}

/* Body-frame az/alt (degrees) → world-frame az/alt (degrees).
 * First-order inversion using body coords in trig args; error is O(offset²). */
void body_to_world(const body_frame_calib_t* c, float body_az, float body_alt,
				   float* world_az, float* world_alt)
{
	if (c->calibration_usable == 0)
	{
		*world_az = body_az;
		*world_alt = body_alt;
		return;
	}

	float az_r = body_az * DEG2RAD;
	float alt_r = body_alt * DEG2RAD;

	float IA = c->azimuth_body_offset;
	float IE = c->elevation_body_offset;
	float AN = c->az_0_tilt;
	float AW = c->az_90_tilt;

	*world_alt = body_alt + (IE + AN * cosf(az_r) + AW * sinf(az_r));
	*world_az =
		body_az + (IA + (AN * sinf(az_r) - AW * cosf(az_r)) * tanf(alt_r));
}

static void world_target_increment_cv(WorldTargetState_t* t)
{
	if (t->track_mode == 1)
	{
		t->world_az += t->az_degps_rate * CONTROLLER_UPDATE_S_F;
		t->world_alt += t->alt_degps_rate * CONTROLLER_UPDATE_S_F;
	}

	/* Wrap azimuth to [0, 360) — coordinate system property */
	while (t->world_az < 0.0f) t->world_az += 360.0f;
	while (t->world_az >= 360.0f) t->world_az -= 360.0f;
}

void WorldTarget_SetPos(float world_az, float world_alt)
{
	device_world_target.world_az = world_az;
	device_world_target.world_alt = world_alt;
	device_world_target.track_mode = 0;
}

void WorldTarget_SetCV(float world_az, float world_alt, float az_rate,
					   float alt_rate)
{
	device_world_target.world_az = world_az;
	device_world_target.world_alt = world_alt;
	device_world_target.az_degps_rate = az_rate;
	device_world_target.alt_degps_rate = alt_rate;
	device_world_target.track_mode = 1;
}

/*applies the fix by using world to body or body to world changes*/
void WorldTarget_Update(void)
{
	world_target_increment_cv(&device_world_target);

	float body_az, body_alt;
	world_to_body(&device_position_calib, device_world_target.world_az,
				  device_world_target.world_alt, &body_az, &body_alt);

	/* Clip body altitude to physical range — hardware limit lives in body frame
	 */
	uint8_t clipped = 0;
	if (body_alt > MAX_PITCH_DEG)
	{
		body_alt = MAX_PITCH_DEG;
		clipped = 1;
	}
	if (body_alt < 0.0f)
	{
		body_alt = 0.0f;
		clipped = 1;
	}

	/* Only write back when clipping occurred — avoids accumulating
	 * approximation error from repeated body_to_world round-trips at 100 Hz */
	if (clipped)
		body_to_world(&device_position_calib, body_az, body_alt,
					  &device_world_target.world_az,
					  &device_world_target.world_alt);

	Trajectory_SetBodyPos(body_az, body_alt);
}

/*
 * I2C bus inits
 *
 *
 * I2C1: PB6 (SCL, AF4) / PB7 (SDA, AF4)  -- 24AA2561 EEPROM (32 KB, 0x50)
 * I2C2: PB10 (SCL, AF4) / PB3 (SDA, AF9) -- LNB supply IC
 * I2C3: PA8 / PB4 -- TODO TODO
 * Both buses run at 100 kHz. External pull-ups assumed on PCB.
 * PB3 uses AF9 (not the usual AF4) for I2C2_SDA on the F411 — intentional.
 * ========================================================================== */
/*TODO, fill in I2C peripherals */

I2C_HandleTypeDef i2c1_handle;
I2C_HandleTypeDef i2c2_handle;
I2C_HandleTypeDef i2c3_handle;

HAL_StatusTypeDef init_i2c1_bus(void)
{
	GPIO_InitTypeDef gpio = {0};

	__HAL_RCC_I2C1_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7;
	gpio.Mode = GPIO_MODE_AF_OD;
	gpio.Pull = GPIO_NOPULL;
	gpio.Speed = GPIO_SPEED_FREQ_HIGH;
	gpio.Alternate = GPIO_AF4_I2C1;
	HAL_GPIO_Init(GPIOB, &gpio);

	i2c1_handle.Instance = I2C1;
	i2c1_handle.Init.ClockSpeed = 100000;
	i2c1_handle.Init.DutyCycle = I2C_DUTYCYCLE_2;
	i2c1_handle.Init.OwnAddress1 = 0;
	i2c1_handle.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	i2c1_handle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	i2c1_handle.Init.OwnAddress2 = 0;
	i2c1_handle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	i2c1_handle.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

	if (HAL_I2C_Init(&i2c1_handle) != HAL_OK)
	{
		uart_reply("init_i2c1_bus: HAL_I2C_Init failed\r\n");
		return HAL_ERROR;
	}

	uint8_t found[8];
	char buf[40];
	uint8_t n = i2c_scan(&i2c1_handle, found, 8);
	for (uint8_t i = 0; i < n; i++)
	{
		snprintf(buf, sizeof(buf), "i2c1-found I2C device: 0x%02X\r\n",
				 found[i]);
		uart_reply(buf);
	}

	return HAL_OK;
}

HAL_StatusTypeDef init_i2c2_bus(void)
{
	GPIO_InitTypeDef gpio = {0};

	__HAL_RCC_I2C2_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/* PB10 = SCL, AF4 */
	gpio.Pin = GPIO_PIN_10;
	gpio.Mode = GPIO_MODE_AF_OD;
	gpio.Pull = GPIO_NOPULL;
	gpio.Speed = GPIO_SPEED_FREQ_HIGH;
	gpio.Alternate = GPIO_AF4_I2C2;
	HAL_GPIO_Init(GPIOB, &gpio);

	/* PB3 = SDA, AF9 (different AF from SCL on this bus) */
	gpio.Pin = GPIO_PIN_3;
	gpio.Alternate = GPIO_AF9_I2C2;
	HAL_GPIO_Init(GPIOB, &gpio);

	i2c2_handle.Instance = I2C2;
	i2c2_handle.Init.ClockSpeed = 100000;
	i2c2_handle.Init.DutyCycle = I2C_DUTYCYCLE_2;
	i2c2_handle.Init.OwnAddress1 = 0;
	i2c2_handle.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	i2c2_handle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	i2c2_handle.Init.OwnAddress2 = 0;
	i2c2_handle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	i2c2_handle.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

	if (HAL_I2C_Init(&i2c2_handle) != HAL_OK)
	{
		uart_reply("init_i2c2_bus: HAL_I2C_Init failed\r\n");
		return HAL_ERROR;
	}

	uint8_t found[8];
	char buf[40];
	uint8_t n = i2c_scan(&i2c2_handle, found, 8);
	for (uint8_t i = 0; i < n; i++)
	{
		snprintf(buf, sizeof(buf), "i2c2-found I2C device: 0x%02X\r\n",
				 found[i]);
		uart_reply(buf);
	}

	return HAL_OK;
}

HAL_StatusTypeDef init_i2c3_bus(void)
{
	GPIO_InitTypeDef gpio = {0};

	__HAL_RCC_I2C3_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/* PA8 = SCL, AF4 */
	gpio.Pin = GPIO_PIN_8;
	gpio.Mode = GPIO_MODE_AF_OD;
	gpio.Pull = GPIO_NOPULL;
	gpio.Speed = GPIO_SPEED_FREQ_HIGH;
	gpio.Alternate = GPIO_AF4_I2C3;
	HAL_GPIO_Init(GPIOA, &gpio);

	/* PB4 = SDA, AF9 (non-standard AF for I2C3_SDA on F411) */
	gpio.Pin = GPIO_PIN_4;
	gpio.Alternate = GPIO_AF9_I2C3;
	HAL_GPIO_Init(GPIOB, &gpio);

	i2c3_handle.Instance = I2C3;
	i2c3_handle.Init.ClockSpeed = 100000;
	i2c3_handle.Init.DutyCycle = I2C_DUTYCYCLE_2;
	i2c3_handle.Init.OwnAddress1 = 0;
	i2c3_handle.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	i2c3_handle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	i2c3_handle.Init.OwnAddress2 = 0;
	i2c3_handle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	i2c3_handle.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

	if (HAL_I2C_Init(&i2c3_handle) != HAL_OK)
	{
		uart_reply("init_i2c3_bus: HAL_I2C_Init failed\r\n");
		return HAL_ERROR;
	}

	uint8_t found[8];
	char buf[40];
	uint8_t n = i2c_scan(&i2c3_handle, found, 8);
	for (uint8_t i = 0; i < n; i++)
	{
		snprintf(buf, sizeof(buf), "i2c3-found I2C device: 0x%02X\r\n",
				 found[i]);
		uart_reply(buf);
	}

	return HAL_OK;
}

uint8_t i2c_scan(I2C_HandleTypeDef* hi2c, uint8_t* found_addrs,
				 uint8_t max_addrs)
{
	uint8_t count = 0;

	for (uint8_t addr = 0x01; addr <= 0x7F; addr++)
	{
		if (HAL_I2C_IsDeviceReady(hi2c, (uint16_t)(addr << 1), 1, 10) == HAL_OK)
		{
			if (count < max_addrs) found_addrs[count++] = addr;
		}
	}

	return count;
}

HAL_StatusTypeDef eeprom_read(uint16_t mem_addr, uint8_t* buf, uint16_t len)
{
	return HAL_I2C_Mem_Read(&i2c1_handle, 0xA0, mem_addr, I2C_MEMADD_SIZE_16BIT,
							buf, len, HAL_MAX_DELAY);
}

HAL_StatusTypeDef eeprom_write(uint16_t mem_addr, uint8_t* buf, uint16_t len)
{
	return HAL_I2C_Mem_Write(&i2c1_handle, 0xA0, mem_addr,
							 I2C_MEMADD_SIZE_16BIT, buf, len, HAL_MAX_DELAY);
}

/*its on i2c2*/
/*set lnb supply ic to 13v*/
HAL_StatusTypeDef lnb_set_vertical()
{
	uint8_t ctrl_cmd = 0x01;
	return HAL_I2C_Mem_Write(&i2c2_handle, LNBH29_ADDR, 0x01,
							 I2C_MEMADD_SIZE_8BIT, &ctrl_cmd, 1, 50);
	//	return HAL_I2C_Master_Transmit(&i2c2_handle, LNBH29_ADDR, &ctrl_cmd, 1,
	// 50);
}

/*its on i2c2*/
/*set lnb supply ic to 18v*/
HAL_StatusTypeDef lnb_set_horizontal()
{
	uint8_t ctrl_cmd = 0x05;
	return HAL_I2C_Mem_Write(&i2c2_handle, LNBH29_ADDR, 0x01,
							 I2C_MEMADD_SIZE_8BIT, &ctrl_cmd, 1, 50);
	// return HAL_I2C_Master_Transmit(&i2c2_handle, LNBH29_ADDR, &ctrl_cmd, 1,
	// 50);
}

/*its on i2c2*/
/*set lnb supply ic to 0v*/
HAL_StatusTypeDef lnb_set_off()
{
	uint8_t ctrl_cmd = 0x00;
	return HAL_I2C_Mem_Write(&i2c2_handle, LNBH29_ADDR, 0x01,
							 I2C_MEMADD_SIZE_8BIT, &ctrl_cmd, 1, 50);
	// return HAL_I2C_Master_Transmit(&i2c2_handle, LNBH29_ADDR, &ctrl_cmd, 1,
	// 50);
}

/*functions to dump non volatile memory contents*/
uint8_t eeprom_dump_run = 0;

void eeprom_uart_dumper()
{
	if (eeprom_dump_run == 1)
	{
		static uint16_t dump_addr = 0;
		static uint32_t last_dump_tick = 0;

		if (fifo_is_full(&tx_fifo_rs485, UART_TX_BUF_SIZE)) return;

		// if (HAL_GetTick() - last_dump_tick >= 10)
		if (fifo_count(&tx_fifo_rs485, UART_TX_BUF_SIZE) <
			(UART_TX_BUF_SIZE - (56 * 2)))
		{
			last_dump_tick = HAL_GetTick();

			uint8_t chunk[16]; /*row dumps*/
			char buf[56];	   // 5 byte + 48 + 2 = 55 byte

			if (eeprom_read(dump_addr, chunk, 16) == HAL_OK)
			{
				int pos = snprintf(buf, sizeof(buf), "%04X:", dump_addr);

				for (int i = 0; i < 16; i++)
					pos += snprintf(buf + pos, sizeof(buf) - pos, " %02X",
									chunk[i]);
				buf[pos++] = '\r';
				buf[pos++] = '\n';
				buf[pos] = '\0';
				UART_RS485_SendString(buf);
			}

			dump_addr += 16;
			if (dump_addr >= 0x8000)
			{
				dump_addr = 0;
				eeprom_dump_run = 0;
				UART_RS485_SendString("eeprom dump done\r\n");
			}
		}
	}
}

uint8_t flash_dump_run = 0;
void flash_uart_dumper()
{
	//
}

/* TODO make drivers to operate the I2C to SPI bridge, reset/unreset the FPGA
 * accordingly*/

/*To read SPI flash, we'll prevent the FPGA from booting by using PROG_B, which
will also make SPI flash writable. After that, take over the SPI flash with the
MUX, and start to dump the flash.*/

/*thus plan of approach is;
- spi bridge control (check health, takeover, read bytes, write bytes)
- disable fpga
- */
