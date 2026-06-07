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

#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include <iso646.h>
#include <stdint.h>

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_i2c.h"

// (pi/180)
#define DEG2RAD 0.0174532925199432958f

extern TIM_HandleTypeDef htim1; /* TIM1: pitch-down CH1, yaw rightleft CH2 */
extern TIM_HandleTypeDef htim4; /*piezo*/
extern TIM_HandleTypeDef htim9; /*22khz timer handler for lnb*/

extern I2C_HandleTypeDef i2c1_handle; /* I2C1: PB6/PB7  — 24AA2561 EEPROM */
extern I2C_HandleTypeDef i2c2_handle; /* I2C2: PB10/PB3 — LNB supply IC   */
extern I2C_HandleTypeDef i2c3_handle; /* I2C3: PA8/PB4  — unknown device   */

/*Global device system state/mode*/
typedef enum
{
	DEV_STATE_UNINIT = 0,
	DEV_STATE_HOMING,
	DEV_STATE_CLOSED,
	DEV_STATE_OPENING,
	DEV_STATE_OPEN, /*normal operation state*/
	DEV_STATE_CLOSING,
	DEV_STATE_FAULT,
} DeviceState_t;

extern DeviceState_t device_state;

/*datatypes and structs for piezo buzzer sequences*/
typedef struct
{
	uint16_t tone_ms;
	uint16_t frequency;
} warble_step_t;

#define PIEZO_PRG_1_LEN 6
#define PIEZO_PRG_2_LEN 4
#define PIEZO_PRG_3_LEN 8
#define PIEZO_PRG_4_LEN 2
#define PIEZO_PRG_5_LEN 8
#define PIEZO_PRG_6_LEN 8
#define PIEZO_PRG_7_LEN 16

extern warble_step_t piezo_program_1[PIEZO_PRG_1_LEN];
extern warble_step_t piezo_program_2[PIEZO_PRG_2_LEN];
extern warble_step_t piezo_program_3[PIEZO_PRG_3_LEN];
extern warble_step_t piezo_program_4[PIEZO_PRG_4_LEN];
extern warble_step_t piezo_program_5[PIEZO_PRG_5_LEN];
extern warble_step_t piezo_program_6[PIEZO_PRG_6_LEN];
extern warble_step_t piezo_program_7[PIEZO_PRG_7_LEN];

extern uint8_t piezo_set_pattern;

/*flag used by homing process to start if device is left at unsafe position*/
extern uint8_t homing_assumes_unsafe;

/*World frame calibration struct*/
typedef struct
{
	/*body az/alt, world az/alt in degrees*/
	float calib_pos_1[4];
	float calib_pos_2[4];
	float calib_pos_3[4];

	/* Bitmask for captured positions
	pos 1 = 0:0
	pos 2 = 1:1
	pos 3 = 2:2
	 */
	uint8_t calib_pos_bitmap;

	/* 0 = uncalibrated (passthrough)
	 * 1 = offset-only  (1 point; az_0_tilt and az_90_tilt are zero)
	 * 2 = full model   (2-3 points; all four parameters solved) */
	uint8_t calibration_usable;

	/*computed body tilt solutions */
	float az_0_tilt;  // Roll left-, roll right +
	float az_90_tilt;

	/*SYNC body offsets*/
	float azimuth_body_offset;
	float elevation_body_offset;

} body_frame_calib_t;

extern body_frame_calib_t device_position_calib;

typedef struct
{
	float world_az;
	float world_alt;
	float az_degps_rate;  /*for CV mode*/
	float alt_degps_rate; /*for CV mode*/
	uint8_t track_mode;	  /* 0 = position, 1 = constant velocity */
} WorldTargetState_t;

extern WorldTargetState_t device_world_target;

/* GENERAL INITS */
void GPIO_Init(void);
void TIM1_PWM_Init(void);
void TIM4_PIEZO_PWM_init(void);
void TIM9_LNB_control_tone_init(void);

/* encoder pins init of external interrupt*/
void Encoder_Init(void);
int32_t Encoder_GetPitch(void);
int32_t Encoder_GetYaw(void);

/*zero enc counts*/
void Encoder_ResetPitch(void);
void Encoder_ResetYaw(void);

/*get the current position in degrees in body frame*/
float get_curr_body_az_pos();
float get_curr_body_alt_pos();
/*get current target pos in deg in body frame*/
float get_body_az_pos_target();
float get_body_alt_pos_target();

/*compute current position in the world frame*/
void get_curr_world_az_alt_pos(float* world_az, float* world_alt);

/*Endstop pins init as interrupt and check pin state*/
void Endstop_Init(void);
uint8_t Endstop_Pitch_IsTriggered(void);
uint8_t Endstop_Yaw_IsTriggered(void);

/* Alt/Az axis homing sequence using static PWM*/
/* Returns 1 on success, 0 on timeout */
uint8_t Homing_Run(void);
uint8_t closing_run(); /*TODO, change text styling to fit Homing_Run or vice-V*/

/* user interface LED inits and piezo control*/
void LED_Init(void);
void LED_Set(uint8_t led_id, uint8_t state); /* state: 1=on, 0=off */
void LED_Toggle(uint8_t led_id);

void piezo_set_frequency(uint32_t freq_hz); /*buzzer set freq. zero is off*/
void piezo_pattern_fsm(); /*fsm incrementing tone sequence steps*/

/*motor fault pin inits and check for over-[temp,current,voltage] faults*/
void Fault_Init(void);
uint8_t Fault_Check_U300(void); /* returns non-zero if OTW or FLT asserted */
uint8_t Fault_Check_U350(void);

void Fault_Handler(void); /* TODO unify handling to one path */

/*general purpose*/
void Error_Handler(void); /*TODO unify handling*/

/* Functions and helpers to solve the body tilt and offsets*/

/* Wrap angle difference to [-PI, +PI] */
float wrap_delta(float delta_rad);

/*Adding a sync point 1-3 for aligning body-frame to world*/
void add_sync_point(uint8_t sync_nr, body_frame_calib_t* c, float body_az,
					float body_alt, float world_az, float world_alt);

/* Solves the body calibration parameters from 2 or 3 bitmap-set points.
 * Offsets stored in degrees. Sets calibration_usable = 2 on success. */
uint8_t body_frame_solve(body_frame_calib_t* c);
/* World-frame az/alt → body-frame az/alt (degrees). Pass-through if not
 * calibrated. */
void world_to_body(const body_frame_calib_t* c, float world_az, float world_alt,
				   float* body_az, float* body_alt);
/* Body-frame az/alt → world-frame az/alt (degrees). First-order inversion. */
void body_to_world(const body_frame_calib_t* c, float body_az, float body_alt,
				   float* world_az, float* world_alt);

/* WorldTarget — world-frame setpoint manager */
void WorldTarget_SetPos(float world_az, float world_alt);
void WorldTarget_SetCV(float world_az, float world_alt, float az_rate,
					   float alt_rate);
void WorldTarget_Update(void);

/*I2C bus inits*/
HAL_StatusTypeDef init_i2c1_bus(void); /* EEPROM: PB6 SCL / PB7 SDA        */
HAL_StatusTypeDef init_i2c2_bus(void); /* LNB supply: PB10 SCL / PB3 SDA   */
HAL_StatusTypeDef init_i2c3_bus(void); /* unknown: PA8 SCL / PB4 SDA        */

/* Probe every 7-bit address on hi2c, print hits over UART, fill found_addrs[].
 * Returns number of responding devices. */
uint8_t i2c_scan(I2C_HandleTypeDef* hi2c, uint8_t* found_addrs,
				 uint8_t max_addrs);

/*----- One-off functions for dumping data kept in the EEPROM and Flash*/
/*Read len bytes from the 24AA2561 EEPROM starting at mem_addr. */
HAL_StatusTypeDef eeprom_read(uint16_t mem_addr, uint8_t* buf, uint16_t len);
/* Write len bytes to 24AA2561 EEPROM. len must not be > 64B
 * wait 5 ms after return*/
HAL_StatusTypeDef eeprom_write(uint16_t mem_addr, uint8_t* buf, uint16_t len);

/* Helper functions to operate SPI flash thru I2C-SPI bridge*/
HAL_StatusTypeDef sc18_write(uint8_t* buf, uint8_t len);
HAL_StatusTypeDef sc18_read(uint8_t* buf, uint8_t len);
/*Take over flash from FPGA to STM*/
HAL_StatusTypeDef sc18_mux_takeover(void);
HAL_StatusTypeDef sc18_mux_release(void);
/*read FLASH jedec ID as validation/test */
HAL_StatusTypeDef mx25_read_jedec(uint8_t id[3]);
/*read 128 bytes from flash*/
HAL_StatusTypeDef mx25_read_128(uint32_t addr, uint8_t* data);

/*Functions to operate LNB supply IC thru I2C*/
HAL_StatusTypeDef lnb_set_off();
HAL_StatusTypeDef lnb_set_horizontal();
HAL_StatusTypeDef lnb_set_vertical();
void set_lnb_lo_low(void);	 // by changing bias tee voltage
void set_lnb_lo_high(void);	 // by changing bias tee voltage

/*SERIT RF frontend NIM related */
void turn_on_serit_supply();
void turn_off_serit_supply();
void turn_off_relay();
void turn_on_relay();

/*TODO, add function(+script) to flash bitsream onto flash memory for FPGA */
/*EEPROM content dump to RS485 uart*/
extern uint8_t eeprom_dump_run;
void eeprom_uart_dumper();
/*SPI flash content dump to RS485 uart*/
extern uint8_t flash_dump_run;
void flash_uart_dumper();

#endif /* FUNCTIONS_H */
