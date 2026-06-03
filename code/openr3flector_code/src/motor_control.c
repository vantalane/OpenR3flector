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

#include "motor_control.h"

#include <math.h>
#include <stdint.h>

#include "functions.h"
#include "main.h"
#include "stm32f4xx_hal.h"
#include "uart_comms.h"

/* shared axis state (encoder counts + direction, written by ISR)
	practically the raw "axis" data */
AxisState_t pitch_axis = {0};
AxisState_t yaw_axis = {0};

/* trajectory planner configs and state */
TrajectoryState_t pitch_traj = {
	.count_per_deg = COUNTS_PER_DEG_PITCH,
	.pos_hardware_offset = PITCH_ENC_HORIZON_OFFSET,
	.max_pos_limit = (float)MAX_PITCH_ENC_POS,
	.min_pos_limit = 0.0f,
	.max_accel_abs = PITCH_ACCEL_MAX_DEG_S2,
};

TrajectoryState_t yaw_traj = {
	.count_per_deg = COUNTS_PER_DEG_YAW,
	.pos_hardware_offset = 0.0f,
	.max_pos_limit = (float)MAX_YAW_ENC_POS,
	.min_pos_limit = (float)MIN_YAW_ENC_POS,
	.max_accel_abs = YAW_ACCEL_MAX_DEG_S2,
};

/* global, high leve device operating state */
DeviceState_t device_state = DEV_STATE_UNINIT;

/*PID variables*/
pidProcess pid_elev = {.Kp = CTRL_KP_PITCH,
					   .Ki = CTRL_KI_PITCH,
					   .Kd = CTRL_KD_PITCH,
					   .tau = CTRL_TAU_PITCH,
					   .outMax = (float_t)CTRL_MAX_DUTY,
					   .outMin = -((float_t)CTRL_MAX_DUTY),
					   .intLimMax = CTRL_KI_LIMIT_PITCH,
					   .intLimMin = -CTRL_KI_LIMIT_PITCH,
					   .intDeadband = CTRL_DEADBAND,
					   .T = CONTROLLER_UPDATE_S_F,
					   .integratorMem = 0.00f,
					   .prevErrorMem = 0.00f,
					   .differentiator = 0.00f,
					   .prevMeasurement = 0.00f,
					   .out = 0.00f};

pidProcess pid_azim = {.Kp = CTRL_KP_YAW,
					   .Ki = CTRL_KI_YAW,
					   .Kd = CTRL_KD_YAW,
					   .tau = CTRL_TAU_YAW,
					   .outMax = (float_t)CTRL_MAX_DUTY,
					   .outMin = -((float_t)CTRL_MAX_DUTY),
					   .intLimMax = CTRL_KI_LIMIT_YAW,
					   .intLimMin = -CTRL_KI_LIMIT_YAW,
					   .intDeadband = CTRL_DEADBAND,
					   .T = CONTROLLER_UPDATE_S_F,
					   .integratorMem = 0.00f,
					   .prevErrorMem = 0.00f,
					   .differentiator = 0.00f,
					   .prevMeasurement = 0.00f,
					   .out = 0.00f};

void pidProcess_Init(pidProcess* pid)
{
	// Clear variables
	pid->integratorMem = 0.00f;
	pid->prevErrorMem = 0.00f;

	pid->differentiator = 0.00f;
	pid->prevMeasurement = 0.00f;

	pid->out = 0.00f;
}

float pidProcess_Update(pidProcess* pid, float setpoint, float measurement)
{
	// Get error
	float error = setpoint - measurement;

	// get proportional
	float proportional = pid->Kp * error;

	/*Dont wind up if output is clipped*/
	if (!((pid->out >= pid->outMax) || (pid->out <= pid->outMin)))
	{
		/*operate integ when error is larger than deadband*/
		if (fabsf(error) > pid->intDeadband)
		{
			// get integral
			pid->integratorMem =
				pid->integratorMem +
				0.5f * pid->Ki * pid->T * (error + pid->prevErrorMem);
		}

		// Anti wind up via integrator clamping
		if (pid->integratorMem > pid->intLimMax)
		{
			pid->integratorMem = pid->intLimMax;
		}
		else if (pid->integratorMem < pid->intLimMin)
		{
			pid->integratorMem = pid->intLimMin;
		}
	}

	/*Derivative (band-limited differentiator)*/
	/*Derivative on measurement, therefore minus sign in front of equation.*/
	pid->differentiator =
		-(2.0f * pid->Kd * (measurement - pid->prevMeasurement) +
		  (2.0f * pid->tau - pid->T) * pid->differentiator) /
		(pid->tau + pid->T);

	/* Compute output and apply limits*/
	pid->out = proportional + pid->integratorMem + pid->differentiator;

	if (pid->out > pid->outMax)
	{
		pid->out = pid->outMax;
	}
	else if (pid->out < pid->outMin)
	{
		pid->out = pid->outMin;
	}

	pid->prevErrorMem = error;
	pid->prevMeasurement = measurement;

	return pid->out;
}

void Motor_Init(void)
{
	/* Release driver resets (active-low reset, hold HIGH to enable)          */
	HAL_GPIO_WritePin(U300_RST_AB_PORT, U300_RST_AB_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(U350_RST_AB_PORT, U350_RST_AB_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(U350_RST_CD_PORT, U350_RST_CD_PIN, GPIO_PIN_SET);

	/* Small delay to allow drivers to come out of reset                      */
	HAL_Delay(10);

	/* Start all PWM channels with 0 duty -- outputs are active but idle      */
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);	 /* PE9  pitch down        */
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1); /* PE8  pitch down N      */
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);	 /* PE11 yaw left          */
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2); /* PE10 yaw right N       */

	/* Ensure all duties start at zero                                        */
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
}

void Pitch_SetDuty(int32_t duty)
{
	if (duty > (int32_t)PWM_MAX_DUTY) duty = (int32_t)PWM_MAX_DUTY;
	if (duty < -(int32_t)PWM_MAX_DUTY) duty = -(int32_t)PWM_MAX_DUTY;

	static int8_t last_dir = 0;
	static int32_t last_duty = 0;

	GPIO_InitTypeDef gpio = {0};
	gpio.Pull = GPIO_NOPULL;
	gpio.Speed = GPIO_SPEED_FREQ_HIGH;
	gpio.Alternate = GPIO_AF1_TIM1;

	/*prevent unnecessary re-config*/
	if (duty == last_duty) return;

	last_duty = duty;
	pitch_axis.set_duty = duty;

	if (duty > 0)
	{
		if (last_dir != +1)
		{
			/* PE8 → static GPIO low, PE9 → TIM1 AF PWM */
			gpio.Pin = PITCH_DOWN_PIN;
			gpio.Mode = GPIO_MODE_OUTPUT_PP;
			HAL_GPIO_Init(GPIOE, &gpio);
			HAL_GPIO_WritePin(GPIOE, PITCH_DOWN_PIN, GPIO_PIN_RESET);

			gpio.Pin = PITCH_UP_PIN;
			gpio.Mode = GPIO_MODE_AF_PP;
			HAL_GPIO_Init(GPIOE, &gpio);

			last_dir = +1;
		}
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)duty);
		pitch_axis.direction = +1;
		pitch_axis.moving = 1;
	}
	else if (duty < 0)
	{
		if (pitch_axis.at_endstop)
		{
			Pitch_SetDuty(0);
			char reply[48];
			// snprintf(reply, sizeof(reply),
			// 		 "Setdtypitch, endstop active, set self 0dty\r\n");
			// UART_SendString(reply);
			return;
		}

		if (last_dir != -1)
		{
			/* PE9 → static GPIO low, PE8 → TIM1 AF PWM */
			gpio.Pin = PITCH_UP_PIN;
			gpio.Mode = GPIO_MODE_OUTPUT_PP;
			HAL_GPIO_Init(GPIOE, &gpio);
			HAL_GPIO_WritePin(GPIOE, PITCH_UP_PIN, GPIO_PIN_RESET);

			gpio.Pin = PITCH_DOWN_PIN;
			gpio.Mode = GPIO_MODE_AF_PP;
			HAL_GPIO_Init(GPIOE, &gpio);

			last_dir = -1;
		}
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)abs(duty));
		pitch_axis.direction = -1;
		pitch_axis.moving = 1;
	}
	else
	{
		if (last_dir != 0)
		{
			gpio.Pin = PITCH_UP_PIN | PITCH_DOWN_PIN;
			gpio.Mode = GPIO_MODE_OUTPUT_PP;
			HAL_GPIO_Init(GPIOE, &gpio);
			HAL_GPIO_WritePin(GPIOE, PITCH_UP_PIN | PITCH_DOWN_PIN,
							  GPIO_PIN_RESET);
			last_dir = 0;
		}
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
		pitch_axis.direction = 0;
		pitch_axis.moving = 0;
	}
}

void Yaw_SetDuty(int32_t duty)
{
	if (duty > (int32_t)PWM_MAX_DUTY) duty = (int32_t)PWM_MAX_DUTY;
	if (duty < -(int32_t)PWM_MAX_DUTY) duty = -(int32_t)PWM_MAX_DUTY;

	static int8_t last_dir = 0; /* track last configured direction */
	static int32_t last_duty = 0;

	GPIO_InitTypeDef gpio = {0};
	gpio.Pull = GPIO_NOPULL;
	gpio.Speed = GPIO_SPEED_FREQ_HIGH;
	gpio.Alternate = GPIO_AF1_TIM1;

	/*prevent unnecessary re-config*/
	if (duty == last_duty) return;

	last_duty = duty;

	yaw_axis.set_duty = duty;

	if (duty > 0)
	{
		if (last_dir != +1)
		{
			/* PE11 → static GPIO low, PE10 → TIM1 AF PWM */
			gpio.Pin = YAW_LEFT_PIN;
			gpio.Mode = GPIO_MODE_OUTPUT_PP;
			HAL_GPIO_Init(GPIOE, &gpio);
			HAL_GPIO_WritePin(GPIOE, YAW_LEFT_PIN, GPIO_PIN_RESET);

			gpio.Pin = YAW_RIGHT_PIN;
			gpio.Mode = GPIO_MODE_AF_PP;
			HAL_GPIO_Init(GPIOE, &gpio);

			last_dir = +1;
		}

		last_duty = duty;
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint32_t)duty);
		yaw_axis.direction = +1;
		yaw_axis.moving = 1;
	}
	else if (duty < 0)
	{
		if (yaw_axis.at_endstop)
		{
			Yaw_SetDuty(0);
			char reply[48];
			// snprintf(reply, sizeof(reply),
			// 		 "SetdtyYAW, endstop active, set self 0dty\r\n");
			// UART_SendString(reply);
			return;
		}

		if (last_dir != -1)
		{
			/* PE10 → static GPIO low, PE11 → TIM1 AF PWM */
			gpio.Pin = YAW_RIGHT_PIN;
			gpio.Mode = GPIO_MODE_OUTPUT_PP;
			HAL_GPIO_Init(GPIOE, &gpio);
			HAL_GPIO_WritePin(GPIOE, YAW_RIGHT_PIN, GPIO_PIN_RESET);

			gpio.Pin = YAW_LEFT_PIN;
			gpio.Mode = GPIO_MODE_AF_PP;
			HAL_GPIO_Init(GPIOE, &gpio);

			last_dir = -1;
		}
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint32_t)abs(duty));
		yaw_axis.direction = -1;
		yaw_axis.moving = 1;
	}
	else
	{
		if (last_dir != 0)
		{
			/* Both pins → GPIO low (coast) */
			gpio.Pin = YAW_LEFT_PIN | YAW_RIGHT_PIN;
			gpio.Mode = GPIO_MODE_OUTPUT_PP;
			HAL_GPIO_Init(GPIOE, &gpio);
			HAL_GPIO_WritePin(GPIOE, YAW_LEFT_PIN | YAW_RIGHT_PIN,
							  GPIO_PIN_RESET);
			last_dir = 0;
		}
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
		yaw_axis.direction = 0;
		yaw_axis.moving = 0;
	}
}

void Motor_Stop_All(void)
{
	GPIO_InitTypeDef gpio = {0};
	gpio.Pin = YAW_LEFT_PIN | YAW_RIGHT_PIN | PITCH_DOWN_PIN | PITCH_UP_PIN;
	gpio.Mode = GPIO_MODE_OUTPUT_PP;
	gpio.Pull = GPIO_NOPULL;
	gpio.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOE, &gpio);
	HAL_GPIO_WritePin(GPIOE, gpio.Pin, GPIO_PIN_RESET);

	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);

	pitch_axis.direction = 0;
	pitch_axis.moving = 0;
	yaw_axis.direction = 0;
	yaw_axis.moving = 0;
}

/*Quick check if system is initialised to be driven by the control system.
Checks if homing is performed, and system device is in a state expected to be
driven by trajectory control

Returns 1 if OK, 0 if not ready.*/
uint8_t check_control_allowed()
{
	// Control system not "alive" during the following device states
	if ((device_state == DEV_STATE_UNINIT) ||
		(device_state == DEV_STATE_HOMING) ||
		(device_state == DEV_STATE_CLOSED) || (device_state == DEV_STATE_FAULT))
	{
		return 0;
	}

	// check for temperatures?
	return 1;
}

/*  Shared helpers  */

/*clamps the duty to the minimum and maximum duty values, with polarity
 * controlled*/
static int32_t clamp_duty(int32_t v)
{
	int32_t mag = (v < 0) ? -v : v;
	if (mag == 0) return 0;
	if (mag < (int32_t)CTRL_MIN_DUTY) mag = (int32_t)CTRL_MIN_DUTY;
	if (mag > (int32_t)CTRL_MAX_DUTY) mag = (int32_t)CTRL_MAX_DUTY;
	return (v < 0) ? -mag : mag;
}
/*helpers to convert degree to count squared */
static inline float deg_to_cnt_ps2(float deg_s2, float cpd)
{
	return (deg_s2 * cpd) /
		   ((float)CONTROLLER_UPDATE_HZ * CONTROLLER_UPDATE_HZ);
}

static inline float deg_to_cnt_ps(float deg_s, float cpd)
{
	return (deg_s * cpd) / (float)CONTROLLER_UPDATE_HZ;
}

/*trajectory compute for setpoint movement*/
static void trajectory_step(TrajectoryState_t* t)
{
	float v = t->profiled_vel;
	float error = t->raw_target - t->profiled_pos;
	float dist = fabsf(error);
	float dir = (error > 0.0f) ? 1.0f : -1.0f;

	float accel_ct2 = deg_to_cnt_ps2(t->max_acceleration, t->count_per_deg);
	float vel_max_ct = deg_to_cnt_ps(t->max_velocity, t->count_per_deg);

	/* Target velocity in counts/tick, derived from how much raw_target moved.
	 * Clamped to the axis velocity limit to absorb large jumps (e.g. wrap). */
	float target_vel_ct = t->raw_target - t->prev_raw_target;
	t->prev_raw_target = t->raw_target;
	if (target_vel_ct > vel_max_ct) target_vel_ct = vel_max_ct;
	if (target_vel_ct < -vel_max_ct) target_vel_ct = -vel_max_ct;

	/* Velocity and braking distance relative to the moving target */
	float rel_v = v - target_vel_ct;
	float rel_vel_toward = rel_v * dir;
	float rel_brake_dist = (rel_v * rel_v) / (2.0f * accel_ct2);

	/* Close-approach snap: inherit target velocity so a moving target does not
	 * cause an abrupt stop. For a static target target_vel_ct is 0, matching
	 * the previous behaviour exactly. */
	if (dist < 0.5f)
	{
		t->profiled_pos = t->raw_target;
		t->profiled_vel = target_vel_ct;
		return;
	}

	/* Limit-priority brake: uses absolute velocity against fixed hardware
	 * limits */
	float brake_dist = (v * v) / (2.0f * accel_ct2);
	float dist_to_limit = (v > 0.0f) ? (t->max_pos_limit - t->profiled_pos)
									 : (t->profiled_pos - t->min_pos_limit);

	if (v != 0.0f && dist_to_limit > 0.0f && brake_dist >= dist_to_limit)
	{
		float a_needed = (v * v) / (2.0f * dist_to_limit);
		float a_brake =
			fminf(a_needed, deg_to_cnt_ps2(t->max_accel_abs, t->count_per_deg));
		float sign = (v > 0.0f) ? 1.0f : -1.0f;
		t->profiled_vel -= sign * a_brake;
		if (t->profiled_vel * sign < 0.0f) t->profiled_vel = 0.0f;
	}
	else
	{
		/* Normal trapezoidal profile, relative to the moving target.
		 * Brakes to match target_vel_ct rather than to zero, so a target
		 * approaching the dish does not cause premature or abrupt stops. */
		if (rel_vel_toward > 0.0f && dist <= rel_brake_dist + fabsf(rel_v))
		{
			t->profiled_vel -= dir * accel_ct2;
			if ((t->profiled_vel - target_vel_ct) * dir < 0.0f)
				t->profiled_vel = target_vel_ct;
		}
		else if (rel_vel_toward <= 0.0f || fabsf(v) < vel_max_ct)
		{
			t->profiled_vel += dir * accel_ct2;
		}
	}

	/* Clamp velocity to max */
	if (t->profiled_vel > vel_max_ct) t->profiled_vel = vel_max_ct;
	if (t->profiled_vel < -vel_max_ct) t->profiled_vel = -vel_max_ct;

	t->profiled_pos += t->profiled_vel;

	if (t->profiled_pos > t->max_pos_limit)
	{
		t->profiled_pos = t->max_pos_limit;
		if (t->profiled_vel > 0.0f) t->profiled_vel = 0.0f;
	}

	if (t->profiled_pos < t->min_pos_limit)
	{
		t->profiled_pos = t->min_pos_limit;
		if (t->profiled_vel < 0.0f) t->profiled_vel = 0.0f;
	}
}

void ctrl_set_max_accel(float az_degpss, float alt_degpss)
{
	if (az_degpss > 0.01f)
		yaw_traj.max_acceleration = (az_degpss > YAW_ACCEL_MAX_DEG_S2)
										? YAW_ACCEL_MAX_DEG_S2
										: az_degpss;

	if (alt_degpss > 0.01f)
		pitch_traj.max_acceleration = (alt_degpss > PITCH_ACCEL_MAX_DEG_S2)
										  ? PITCH_ACCEL_MAX_DEG_S2
										  : alt_degpss;
}

void ctrl_set_max_vel(float az_degps, float alt_degps)
{
	if (az_degps > 0.01f)
		yaw_traj.max_velocity =
			(az_degps > YAW_VEL_MAX_DEG_S) ? YAW_VEL_MAX_DEG_S : az_degps;

	if (alt_degps > 0.01f)
		pitch_traj.max_velocity =
			(alt_degps > PITCH_VEL_MAX_DEG_S) ? PITCH_VEL_MAX_DEG_S : alt_degps;
}

void Trajectory_SetBodyPitch(float counts)
{
	float c = counts;
	if (c > MAX_PITCH_ENC_POS) c = MAX_PITCH_ENC_POS;
	if (c < MIN_PITCH_ENC_POS) c = MIN_PITCH_ENC_POS;
	pitch_traj.raw_target = c;
}

void Trajectory_SetBodyYaw(float counts)
{
	float c = counts;
	if (c > MAX_YAW_ENC_POS) c = MAX_YAW_ENC_POS;
	if (c < MIN_YAW_ENC_POS) c = MIN_YAW_ENC_POS;
	yaw_traj.raw_target = c;
}

void Trajectory_SetBodyPos(float body_az, float body_alt)
{
	Trajectory_SetBodyYaw(body_az * COUNTS_PER_DEG_YAW);
	Trajectory_SetBodyPitch((body_alt * COUNTS_PER_DEG_PITCH) +
							(float)PITCH_ENC_HORIZON_OFFSET);
}

/* Seed trajectory state from actual encoder position after homing.
 * Prevents the planner from ramping from 0 on first update. */
void Motion_PostHomingInit(void)
{
	float pitch_pos = (float)Encoder_GetPitch();
	float yaw_pos = (float)Encoder_GetYaw();

	pitch_traj.raw_target = pitch_pos;
	pitch_traj.prev_raw_target = pitch_pos;
	pitch_traj.profiled_pos = pitch_pos;
	pitch_traj.profiled_vel = 0.0f;
	pitch_traj.max_velocity = PITCH_VEL_MAX_DEG_S;
	pitch_traj.max_acceleration = PITCH_ACCEL_MAX_DEG_S2;

	yaw_traj.raw_target = yaw_pos;
	yaw_traj.prev_raw_target = yaw_pos;
	yaw_traj.profiled_pos = yaw_pos;
	yaw_traj.profiled_vel = 0.0f;
	yaw_traj.max_velocity = YAW_VEL_MAX_DEG_S;
	yaw_traj.max_acceleration = YAW_ACCEL_MAX_DEG_S2;

	ctrl_set_max_accel(15.0f, 15.0f);
	ctrl_set_max_vel(15.0f, 15.0f);

	device_state = DEV_STATE_CLOSED;
}

/*update velocity measurement since last run.*/
void accel_velo_update(void)
{
	static int32_t prev_yaw_cnt, prev_pitch_cnt;
	static float prev_yaw_velocity, prev_pitch_velocity;
	static uint32_t last_compute_tick;

	/* Rolling window buffers and running sums */
	static float yaw_vel_buf[VEL_FILTER_WINDOW];
	static float pitch_vel_buf[VEL_FILTER_WINDOW];
	static float yaw_accel_buf[ACCEL_FILTER_WINDOW];
	static float pitch_accel_buf[ACCEL_FILTER_WINDOW];
	static float yaw_vel_sum, pitch_vel_sum, yaw_accel_sum, pitch_accel_sum;
	static uint8_t vel_idx, accel_idx;

	uint32_t current_ticks = HAL_GetTick();

	if (last_compute_tick == 0)
	{
		last_compute_tick = current_ticks;
		prev_yaw_cnt = Encoder_GetYaw();
		prev_pitch_cnt = Encoder_GetPitch();
		return;
	}

	if (current_ticks - last_compute_tick == 0) return;

	float time_delta_s = (float)(current_ticks - last_compute_tick) / 1000.0f;
	last_compute_tick = current_ticks;

	/*YAW CALCULATION ///////////////////////////////////*/
	int32_t yaw_enc = Encoder_GetYaw();
	float raw_yaw_vel =
		((float)(yaw_enc - prev_yaw_cnt) / COUNTS_PER_DEG_YAW) / time_delta_s;
	prev_yaw_cnt = yaw_enc;

	yaw_vel_sum -= yaw_vel_buf[vel_idx];
	yaw_vel_buf[vel_idx] = raw_yaw_vel;
	yaw_vel_sum += raw_yaw_vel;
	yaw_axis.velocity = yaw_vel_sum / VEL_FILTER_WINDOW;

	float raw_yaw_accel =
		(yaw_axis.velocity - prev_yaw_velocity) / time_delta_s;
	prev_yaw_velocity = yaw_axis.velocity;

	yaw_accel_sum -= yaw_accel_buf[accel_idx];
	yaw_accel_buf[accel_idx] = raw_yaw_accel;
	yaw_accel_sum += raw_yaw_accel;
	yaw_axis.acceleration = yaw_accel_sum / ACCEL_FILTER_WINDOW;

	/*PITCH CALCULATION ///////////////////////////////////*/
	int32_t pitch_enc = Encoder_GetPitch();
	float raw_pitch_vel =
		((float)(pitch_enc - prev_pitch_cnt) / COUNTS_PER_DEG_PITCH) /
		time_delta_s;
	prev_pitch_cnt = pitch_enc;

	pitch_vel_sum -= pitch_vel_buf[vel_idx];
	pitch_vel_buf[vel_idx] = raw_pitch_vel;
	pitch_vel_sum += raw_pitch_vel;
	pitch_axis.velocity = pitch_vel_sum / VEL_FILTER_WINDOW;

	float raw_pitch_accel =
		(pitch_axis.velocity - prev_pitch_velocity) / time_delta_s;
	prev_pitch_velocity = pitch_axis.velocity;

	pitch_accel_sum -= pitch_accel_buf[accel_idx];
	pitch_accel_buf[accel_idx] = raw_pitch_accel;
	pitch_accel_sum += raw_pitch_accel;
	pitch_axis.acceleration = pitch_accel_sum / ACCEL_FILTER_WINDOW;

	vel_idx = (vel_idx + 1) % VEL_FILTER_WINDOW;
	accel_idx = (accel_idx + 1) % ACCEL_FILTER_WINDOW;
}

/*Update trajectory from current cfg's*/
void Trajectory_Update(void)
{
	trajectory_step(&pitch_traj);
	trajectory_step(&yaw_traj);
}

/* Motion safety supervisor
 * Stateless gate between trajectory planner and motor controller. */
int32_t Supervisor_GetSafePitch(void)
{
	float pos = pitch_traj.profiled_pos;

	/* Block downward motion if endstop triggered */
	if (pitch_axis.at_endstop && pitch_traj.profiled_vel < 0.0f)
	{
		pitch_traj.profiled_vel = 0.0f;
		pos = (float)Encoder_GetPitch();
	}

	/* Soft travel limit in normal use mode */
	if (device_state == DEV_STATE_OPEN)
		if (pos < MIN_PITCH_ENC_POS) pos = MIN_PITCH_ENC_POS;

	/* Absolute hard limits */
	if (pos > MAX_PITCH_ENC_POS) pos = MAX_PITCH_ENC_POS;
	if (pos < 0) pos = 0;

	return (int32_t)pos;
}

int32_t Supervisor_GetSafeYaw(void)
{
	float pos = yaw_traj.profiled_pos;

	/* Block motion into endstop */
	if (yaw_axis.at_endstop && yaw_traj.profiled_vel < 0.0f)
	{
		yaw_traj.profiled_vel = 0.0f;
		pos = (float)Encoder_GetYaw();
	}

	/* Hard travel limits */
	if (pos > MAX_YAW_ENC_POS) pos = MAX_YAW_ENC_POS;
	if (pos < MIN_YAW_ENC_POS) pos = MIN_YAW_ENC_POS;

	return (int32_t)pos;
}

/* Device state machine */
void device_open(void)
{
	if (device_state == DEV_STATE_CLOSED) device_state = DEV_STATE_OPENING;
}

void device_close(void)
{
	if (device_state == DEV_STATE_OPEN) device_state = DEV_STATE_CLOSING;
}

void device_home(uint8_t is_unsafe)
{
	if ((device_state == DEV_STATE_UNINIT) || (device_state == DEV_STATE_FAULT))
	{
		if (is_unsafe)
		{
			uart_reply("Received UNSAFE home command. Homing.\n");
			homing_assumes_unsafe = 1;
		}
		else
			uart_reply("Received home command. Homing.\n");

		device_state = DEV_STATE_HOMING;
	}
	else
		uart_reply("Device in invalid state to home.\n");
}

void Device_StateMachine(void)
{
	/*general purpose tick memory for periodic state tasks*/
	static uint32_t tick_time = 0;
	if (!tick_time) tick_time = HAL_GetTick();

	switch (device_state)
	{
		case DEV_STATE_UNINIT:
			/*AT FRESH START, expect a button press for calibration*/
			if ((HAL_GetTick() - tick_time) > 10000)
			{
				tick_time = HAL_GetTick();
				// piezo_set_pattern = 4;
			}

			/*block with button press before start*/
			if (HAL_GPIO_ReadPin(GPIOA, FILM_BUTTON_PIN) == 1)
			{
				uart_reply("Started init\n");
				piezo_set_pattern = 1;
				device_state = DEV_STATE_HOMING;
			}
			break;

		case DEV_STATE_HOMING:
			/*homing process leaves device in "CLOSED" position*/
			{
				if ((HAL_GetTick() - tick_time) > 1000)
				{
					tick_time = HAL_GetTick();
					LED_Toggle(LED_ID_RED_LEFT);
				}

				/*slow-down loop for cheap debouncing*/
				HAL_Delay(5);

				uint8_t home_res = Homing_Run();

				if (home_res == 1)
				{
					/*homed ran successfully*/
					uart_reply("HOMING: DONE\n");
					Motion_PostHomingInit();
					piezo_set_pattern = 2;
					device_state = DEV_STATE_CLOSED;
				}
				else if (home_res == 2)
				{
					/*OH NO, fault*/
					uart_reply("HOMING: ERROR couldn't run\n");
					piezo_set_pattern = 3;
					device_state = DEV_STATE_UNINIT;
				}
			}
			break;

		case DEV_STATE_FAULT:
			if ((HAL_GetTick() - tick_time) > 5000)
			{
				tick_time = HAL_GetTick();
				piezo_set_pattern = 6;
				Motor_Stop_All();
			}

			if (HAL_GPIO_ReadPin(GPIOA, FILM_BUTTON_PIN) == 1)
			{
				uart_reply("Re-enabled from fault using btn\n");
				piezo_set_pattern = 2;
				device_state = DEV_STATE_HOMING;
			}

			break;

		case DEV_STATE_CLOSED:
			/* Motors idle, trajectory locked at closed position */
			/*can be rescued by enabling home thru uart*/
			Motor_Stop_All();
			break;

		case DEV_STATE_OPENING:
			/* Drive pitch up to MIN_PITCH_ENC_POS, yaw stays at 0 */
			yaw_traj.raw_target = 0.0f;
			pitch_traj.raw_target = MIN_PITCH_ENC_POS;

			if (Encoder_GetPitch() >= MIN_PITCH_ENC_POS - CTRL_DEADBAND)
			{
				uart_reply("Device OPEN\n");
				piezo_set_pattern = 7;
				device_state = DEV_STATE_OPEN;
			}

			/*incase you need to ABORT opening procedure, re-close device*/
			if (HAL_GPIO_ReadPin(GPIOA, FILM_BUTTON_PIN) == 1)
			{
				uart_reply("Button detected, PANIC\n");

				/*wait release*/
				while (HAL_GPIO_ReadPin(GPIOA, FILM_BUTTON_PIN) == 1)
				{
				}

				Motor_Stop_All();
				__disable_irq();
				HAL_NVIC_SystemReset();
			}

			break;

		case DEV_STATE_OPEN:
			/* Normal operation — trajectory planner runs freely */
			/*check if overtemperature ETC.*/
			if ((HAL_GetTick() - tick_time) > 1000)
			{
				tick_time = HAL_GetTick();

				if (Fault_Check_U300())
				{
					uart_reply("Fault found from U300\n");
					device_state = DEV_STATE_FAULT;
					piezo_set_pattern = 3;
				}

				if (Fault_Check_U350())
				{
					uart_reply("Fault found from U350\n");
					device_state = DEV_STATE_FAULT;
					piezo_set_pattern = 3;
				}
			}
			break;

		case DEV_STATE_CLOSING:

		{
			if ((HAL_GetTick() - tick_time) > 1000)
			{
				tick_time = HAL_GetTick();
				LED_Toggle(LED_ID_RED_LEFT);
			}

			/*stop tracking mode when entering close*/
			device_world_target.track_mode = 0;

			uint8_t proc_res = closing_run();

			if (proc_res == 1)
			{
				/*homed ran successfully*/
				uart_reply("Closing done\n");
				/*then, send for a re-calibration as we're here already*/
				piezo_set_pattern = 7;
				device_state = DEV_STATE_HOMING;
			}
			else if (proc_res == 2)
			{
				/*OH NO, fault*/
				uart_reply("Couldn't close, FAULT.\n");
				piezo_set_pattern = 3;
				device_state = DEV_STATE_FAULT;
			}
		}

		break;
	}
}

/* Motor controller  (PID + velocity feedforward) */
void Controller_Update(void)
{
	/* Get supervisor-gated setpoints */
	int32_t safe_pitch = Supervisor_GetSafePitch();
	int32_t safe_yaw = Supervisor_GetSafeYaw();

	/*feedforward values*/
	float pff = 0.0f;
	float yff = 0.0f;

	/* ---- Pitch ---- */
	{
		int32_t enc_measure = Encoder_GetPitch();

		/*store changes in pitch into this one*/

		/*PID update*/
		pidProcess_Update(&pid_elev, (float)safe_pitch, (float)enc_measure);

		pff = CTRL_KFF_PITCH * pitch_traj.profiled_vel;
		int32_t duty = (int32_t)roundf(pid_elev.out + pff);

		Pitch_SetDuty(duty);
	}

	/* ---- Yaw ---- */
	{
		int32_t enc_measure = Encoder_GetYaw();

		/*PID update*/
		pidProcess_Update(&pid_azim, (float)safe_yaw, (float)enc_measure);

		yff = CTRL_KFF_YAW * yaw_traj.profiled_vel;
		int32_t duty = (int32_t)roundf(pid_azim.out + yff);

		Yaw_SetDuty(duty);
	}
}
