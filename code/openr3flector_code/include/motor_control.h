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

#ifndef MOT_CTRLS_H
#define MOT_CTRLS_H

#include <math.h>
#include <stdint.h>
#include <stdio.h>

/*PWM / Timer configuration for motor drive */
#define PWM_PRESCALER 4U
#define PWM_PERIOD 999U
#define PWM_MAX_DUTY PWM_PERIOD

/* Encoder counts for Alt/Az axes*/
#define COUNTS_PER_DEG_PITCH 6.666f /* counts per degree, pitch */
#define COUNTS_PER_DEG_YAW 3.416f	/* counts per degree, yaw   */

/*Travel limits in encoder counts for normal open dish operation*/
#define MIN_PITCH_ENC_POS 500
#define MAX_PITCH_ENC_POS 1000
#define MIN_YAW_ENC_POS 0
#define MAX_YAW_ENC_POS 1230

/* Encoder reading corresponding to 0° elevation (body horizon) */
#define PITCH_ENC_HORIZON_OFFSET 500

/*macro's for ease*/
#define MAX_PITCH_DEG \
	((MAX_PITCH_ENC_POS - PITCH_ENC_HORIZON_OFFSET) / COUNTS_PER_DEG_PITCH)
/*logically, should be 360, this is a sanity check*/
#define MAX_YAW_DEG ((float)MAX_YAW_ENC_POS / COUNTS_PER_DEG_YAW)

/* Homing related */

/* Minimum PWM duty for motion
 * measured with 2kg weight on pitch, absolute (lowtrust) mechanical MIN duty%
 * - Pitch : 200 (20%)
 * - Yaw : (probably about 150)
 */

#define HOMING_DUTY 350U
#define HOMING_TIMEOUT_MS 5000U

/*Timeout to check if device is homing but can't move/close*/
#define CLOSING_TIMEOUT_MS 15000U

/* PID and trajectory controller update rate */
#define CONTROLLER_UPDATE_HZ 250 /* call motor controls at x Hz */
#define CONTROLLER_UPDATE_MS (1000 / CONTROLLER_UPDATE_HZ)
#define CONTROLLER_UPDATE_S_F ((1.0f) / ((float)CONTROLLER_UPDATE_HZ))

/* ============================================================================
 * Trajectory planner limits
 *   Acceleration in deg/s², converted to counts/tick²
 *   Velocity in deg/s,  converted to counts/tick
 * ========================================================================== */
#define YAW_ACCEL_MAX_DEG_S2 25.0f
#define PITCH_ACCEL_MAX_DEG_S2 25.0f

/*ABSOLUTE MAX VELOCITY LIMITS*/
#define YAW_VEL_MAX_DEG_S 25.0f	  /*azim motor velo can have 25 degs*/
#define PITCH_VEL_MAX_DEG_S 12.0f /*pitch motor velo maxes out at 12 degps*/

/* ============================================================================
 * Motor controller (PID + velocity feedforward) tuning
 * ========================================================================== */
#define CTRL_KP_PITCH 80.0f
#define CTRL_KI_PITCH 32.0f
#define CTRL_KD_PITCH 0.00005f
#define CTRL_TAU_PITCH 0.0001f
/* anti-windup integ limit */
#define CTRL_KI_LIMIT_PITCH 400.0f

#define CTRL_KP_YAW 80.0f
#define CTRL_KI_YAW 32.0f
#define CTRL_KD_YAW 0.000125f
#define CTRL_TAU_YAW 0.000051f
/* anti-windup integ limit */
#define CTRL_KI_LIMIT_YAW 300.0f

/* velocity feedforward gain, tuneable*/
#define CTRL_KFF_PITCH 1.5f
#define CTRL_KFF_YAW 4.0f

/*SETPOINT deadband in counts*/
#define CTRL_DEADBAND 1 /* +- counts — axis considered on-target*/

#define CTRL_MIN_DUTY \
	200U /* minimum PWM to overcome static friction, unused atm*/
#define CTRL_MAX_DUTY PWM_MAX_DUTY

/*Acceleration and velocity measurement rolling-window size (samples)*/
#define VEL_FILTER_WINDOW 24
#define ACCEL_FILTER_WINDOW 8

/* Encoder + direction state, shared with ISR */
typedef struct
{
	volatile int32_t counts; /* encoder count, updated in ISR       */
	int8_t direction;		 /* +1 / -1 / 0, set by SetDuty        */
	uint8_t moving;			 /* 1 while motor is commanded          */
	uint8_t at_endstop;		 /* 1 when endstop ISR fires            */
	int32_t set_duty;

	/*Current measured values*/
	float_t velocity;	  /*velocity degrees per second*/
	float_t acceleration; /*current acceleration*/

} AxisState_t;

extern AxisState_t pitch_axis;
extern AxisState_t yaw_axis;

/* Trajectory planner state (one per axis) */
typedef struct
{
	/*encoder target to profile to in encoder counts*/
	float raw_target;
	float
		prev_raw_target; /* previous raw_target, for deriving target velocity */

	/*trajectory movement data,	 consumed by supervisor/motor controller*/
	float profiled_pos; /* current profiled position, counts       */
	float profiled_vel; /* current profiled velocity, counts/tick  */

	/*configured profile maximum rates*/
	float max_velocity;		/*configured max velocity deg p s*/
	float max_acceleration; /*configured max accel*/

	/*control mode of this axis,*/
	/*0 is pos mode - trapezoidal profile to raw target*/

	/*Physical HARD limits, no runtime change!*/
	const float count_per_deg;		 /*counts per deg of axis*/
	const float pos_hardware_offset; /*axis offset for HW alignment in counts*/
	const float max_pos_limit;		 /*HW pos limit in counts*/
	const float min_pos_limit;		 /*HW pos limit in counts*/
	const float max_accel_abs; /*system absolute max acceleration, deg/s²*/
} TrajectoryState_t;

extern TrajectoryState_t pitch_traj;
extern TrajectoryState_t yaw_traj;

/*PID control memory*/
typedef struct
{
	// Coefficients
	float Kp;
	float Ki;
	float Kd;
	// Low pass filter for derivative
	float tau;

	// Output limits
	float outMax;
	float outMin;
	// Integrator limits. Against integrator windup.
	// Maybe Delta PV change is an option? Conditional integration. Clegg
	// integrator?
	float intLimMax;
	float intLimMin;
	float intDeadband; /*integrator deadband*/
	// Sample time in seconds
	float T;
	// PID process memory
	float integratorMem;
	float prevErrorMem;
	float differentiator;
	float prevMeasurement;
	// Controller output
	float out;
} pidProcess;

extern pidProcess pid_elev;	 // Handles pitch.
extern pidProcess pid_azim;	 // Handles azimuth.

/*PID functions*/
void pidProcess_Init(pidProcess* pid);
float pidProcess_Update(pidProcess* pid, float setpoint, float measurement);

/*Motor pins init, and functions to set axes duty%*/
void Motor_Init(void);
void Pitch_SetDuty(int32_t duty);
void Yaw_SetDuty(int32_t duty);
void Motor_Stop_All(void);

/*Check if system state allows for motor control
returns false when motors are taken over by homing sequence etc.*/
uint8_t check_control_allowed();

/*Apply max accel and velocity*/
void ctrl_set_max_accel(float az_degpss, float alt_degpss);
void ctrl_set_max_vel(float az_degps, float alt_degps);

/* Body-frame degrees → encoder counts, sets trajectory raw_target */
void Trajectory_SetBodyPos(float body_az, float body_alt);
void Trajectory_SetBodyPitch(float counts);
void Trajectory_SetBodyYaw(float counts);

void Trajectory_Update(void); /* call once per control tick */

/*updates the MEASURED velocity and acceleration values*/
void accel_velo_update(void);

/*Additional safety supervisor function to prevent motion beyond specified max
 * range.*/
int32_t Supervisor_GetSafePitch(void);
int32_t Supervisor_GetSafeYaw(void);

/*Motor PID controller following profiled pos */
void Controller_Update(void);

/*init control vars after homing is ran*/
void Motion_PostHomingInit(void);

/*Set device in opening dish state*/
void device_open(void);
/*Set device in closing dish state*/
void device_close(void);
/*Set device in homing state*/
void device_home(uint8_t is_unsafe);

/*Main top level device state machine.*/
void Device_StateMachine(void);

#endif