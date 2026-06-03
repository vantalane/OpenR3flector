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

#include "uart_comms.h"

#include <stdint.h>
#include <stdio.h>

#include "functions.h"
#include "main.h"
#include "motor_control.h"
#include "stm32f4xx_hal_cortex.h"
#include "stm32f4xx_hal_pwr_ex.h"
#include "stm32f4xx_hal_uart.h"

/* UART circular buffers */
uart_fifo_t rx_fifo = {0};
uart_fifo_t tx_fifo = {0};
volatile uint8_t s_rx_byte;
volatile uint8_t uart_tx_running = 0;

UART_HandleTypeDef UartHandle;

/*uart buffers for the RS485 driver line*/
UART_HandleTypeDef UartHandle_rs485;
uart_fifo_t rx_fifo_rs485 = {0};
uart_fifo_t tx_fifo_rs485 = {0};
volatile uint8_t s_rx_byte_rs485;
volatile uint8_t uart_r485_tx_running = 0;

/* ---- FIFO helpers ---- */
uint16_t fifo_count(uart_fifo_t* f, uint16_t size)
{
	return (uint16_t)((f->head - f->tail) & (size - 1));
}

uint8_t fifo_is_empty(uart_fifo_t* f)
{
	return f->head == f->tail;
}

uint8_t fifo_is_full(uart_fifo_t* f, uint16_t size)
{
	return fifo_count(f, size) == (size - 1);
}

void fifo_push(uart_fifo_t* f, uint8_t byte, uint16_t size)
{
	if (!fifo_is_full(f, size))
	{
		f->buf[f->head & (size - 1)] = byte;
		f->head++;
	}
}

uint8_t fifo_pop(uart_fifo_t* f, uint16_t size)
{
	if (fifo_is_empty(f)) return 0;

	uint8_t byte = f->buf[f->tail & (size - 1)];
	f->tail++;
	return byte;
}

/*also configures the reception stuff*/
void UART_set_config(void)
{
	UartHandle.Instance = USARTx;

	UartHandle.Init.BaudRate = 9600;
	UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
	UartHandle.Init.StopBits = UART_STOPBITS_1;
	UartHandle.Init.Parity = UART_PARITY_NONE;
	UartHandle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	UartHandle.Init.Mode = UART_MODE_TX_RX;
	UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;

	if (HAL_UART_Init(&UartHandle) != HAL_OK)
	{
		Error_Handler();
	}

	/* Arm first RX byte */
	HAL_UART_Receive_IT(&UartHandle, &s_rx_byte, 1);
}

void UART_RS485_set_config(void)
{
	/*usart 2 has 3.12 Mbit/s whe oversampling by 16*/
	UartHandle_rs485.Instance = USARTx_RS485;
	UartHandle_rs485.Init.BaudRate = 115200;
	UartHandle_rs485.Init.WordLength = UART_WORDLENGTH_8B;
	UartHandle_rs485.Init.StopBits = UART_STOPBITS_1;
	UartHandle_rs485.Init.Parity = UART_PARITY_NONE;
	UartHandle_rs485.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	UartHandle_rs485.Init.Mode = UART_MODE_TX_RX;
	UartHandle_rs485.Init.OverSampling = UART_OVERSAMPLING_16;

	if (HAL_UART_Init(&UartHandle_rs485) != HAL_OK)
	{
		Error_Handler();
	}

	HAL_UART_Receive_IT(&UartHandle_rs485, (uint8_t*)&s_rx_byte_rs485, 1);
}

void UART_SendString(const char* str)
{
	while (*str) fifo_push(&tx_fifo, (uint8_t)*str++, UART_TX_BUF_SIZE);
}

void UART_RS485_SendString(const char* str)
{
	while (*str) fifo_push(&tx_fifo_rs485, (uint8_t)*str++, UART_TX_BUF_SIZE);
}

/*Write to both interfaces*/
void uart_reply(const char* s)
{
	UART_SendString(s);
	UART_RS485_SendString(s);
}

static void print_debug_info(void)
{
	char reply[512];

	snprintf(reply, sizeof(reply),
			 "\n\r	WorldTracker data AltAz\n\r"
			 "World SP %0.2f-%0.2f mode:%d "
			 "Degps %0.2f-%0.2f\n\r",
			 device_world_target.world_az, device_world_target.world_alt,
			 device_world_target.track_mode, device_world_target.az_degps_rate,
			 device_world_target.alt_degps_rate);
	uart_reply(reply);

	snprintf(reply, sizeof(reply),
			 "	Pos calib data World(Az-Alt),B(Az-Alt), usable: %d\n\r"
			 "Sync A: W %0.2f-%0.2f B %0.2f-%0.2f\n\r"
			 "Sync B: W %0.2f-%0.2f B %0.2f-%0.2f\n\r"
			 "Sync C: W %0.2f-%0.2f B %0.2f-%0.2f\n\r"
			 "Tilt Az0: %0.2f Az90: %0.2f Offset Az: %0.2f Alt: %0.2f\n\r",
			 device_position_calib.calibration_usable,
			 device_position_calib.calib_pos_1[0],
			 device_position_calib.calib_pos_1[1],
			 device_position_calib.calib_pos_1[2],
			 device_position_calib.calib_pos_1[3],
			 device_position_calib.calib_pos_2[0],
			 device_position_calib.calib_pos_2[1],
			 device_position_calib.calib_pos_2[2],
			 device_position_calib.calib_pos_2[3],
			 device_position_calib.calib_pos_3[0],
			 device_position_calib.calib_pos_3[1],
			 device_position_calib.calib_pos_3[2],
			 device_position_calib.calib_pos_3[3],
			 device_position_calib.az_0_tilt, device_position_calib.az_90_tilt,
			 device_position_calib.azimuth_body_offset,
			 device_position_calib.elevation_body_offset);

	uart_reply(reply);

	/*print jaw trajectory ctrl stuff*/
	snprintf(reply, sizeof(reply),
			 "	TrajCtrl stuff Az-Alt\r\n"
			 "Raw tgt: %0.2f-%0.2f\n\r"
			 "Prof pos: %0.2f-%0.2f\n\r"
			 "Prof velo: %0.2f-%0.2f\r\n",
			 yaw_traj.raw_target, pitch_traj.raw_target, yaw_traj.profiled_pos,
			 pitch_traj.profiled_pos, yaw_traj.profiled_vel,
			 pitch_traj.profiled_vel);
	uart_reply(reply);

	snprintf(reply, sizeof(reply),
			 "	Motor ctrl side Yaw-pitch\r\n"
			 "Enc: %d-%d Safe cnt:%d-%d \n\r"
			 "Supplied duties %d-%d\r\n",
			 Encoder_GetYaw(), Encoder_GetPitch(), Supervisor_GetSafeYaw(),
			 Supervisor_GetSafePitch(), yaw_axis.set_duty, pitch_axis.set_duty);

	uart_reply(reply);

	snprintf(reply, sizeof(reply), "endstop %u-%u\r\n\r\n",
			 Endstop_Yaw_IsTriggered(), Endstop_Pitch_IsTriggered());
	uart_reply(reply);
}

uint8_t relaystate = 0;
/* Called from main loop via UART_Poll() */
void UART_ProcessCommand(const char* cmd)
{
	char buf[UART_CMD_BUF_SIZE];
	char reply[256];

	strncpy(buf, cmd, UART_CMD_BUF_SIZE);

	buf[UART_CMD_BUF_SIZE - 1] = '\0';

	switch (buf[0])
	{
		case CMD_HOME:
			device_home(0);
			break;

		case CMD_HOME_UNSAFE:
			device_home(1);
			break;

		case CMD_GOTO:
			if (device_state == DEV_STATE_OPEN)
			{
				float az = 0.0f, el = 0.0f;

				if (parse_two_floats_strict(buf, &az, &el))
				{
					snprintf(
						reply, sizeof(reply),
						"ERR: bad AzEl format for goto, use CMD<az>,<el>\n");

					uart_reply(reply);
				}
				else
				{
					WorldTarget_SetPos(az, el);

					snprintf(reply, sizeof(reply),
							 "GOTO requested: az=%.2f el=%.2f Set: az=%.2f "
							 "el=%.2f\n",
							 az, el, device_world_target.world_az,
							 device_world_target.world_alt);

					uart_reply(reply);
				}
			}
			else
			{
				snprintf(reply, sizeof(reply), "Dev state invalid\n");
				uart_reply(reply);
			}

			break;

		case CMD_SYNC:
		{
			// Need subcommands to be able to add offsets,
			/*SUBCOMMANDS
			0 - reset all sync captures
			1 - sync point 1
			2 - sync point 2
			3 - sync point 3
			*/

			uint8_t sub_cmd;

			float world_ref_az;
			float world_ref_alt;

			if (parse_one_uint_two_floats_strict(buf, &sub_cmd, &world_ref_az,
												 &world_ref_alt))
			{
				snprintf(reply, sizeof(reply),
						 "ERR: incorrect SYNC cmd - uint8_t,float,float\n");
				uart_reply(reply);
			}
			else
			{
				if (sub_cmd == 0)
				{
					/*reset the shits*/
					memset(&device_position_calib, 0,
						   sizeof(body_frame_calib_t));
					snprintf(reply, sizeof(reply), "Cleared sync memory\n");
					uart_reply(reply);
				}
				else
				{
					float current_az = get_curr_body_az_pos();
					float current_alt = get_curr_body_alt_pos();

					add_sync_point(sub_cmd, &device_position_calib, current_az,
								   current_alt, world_ref_az, world_ref_alt);

					snprintf(reply, sizeof(reply),
							 "Added sync point %u with body:%0.1f-%0.1f "
							 "world:%0.1f-%0.1f\n",
							 sub_cmd, current_az, current_alt, world_ref_az,
							 world_ref_alt);

					uart_reply(reply);
				}
			}
		}
		break;

		case CMD_TRACK:
			if (device_state == DEV_STATE_OPEN)
			{
				float start_alt, start_az, alt_track_degps, az_track_degps;

				if (parse_four_floats_strict(buf, &start_az, &start_alt,
											 &az_track_degps, &alt_track_degps))
				{
					uart_reply(
						"ERR: bad Az-Alt-AzDegps-AltDegps format for track, "
						"use CMD<az>,<el>,<az degps>,<alt degps>\n");
				}
				else
				{
					WorldTarget_SetCV(start_az, start_alt, az_track_degps,
									  alt_track_degps);
					snprintf(reply, sizeof(reply),
							 "AzEl start RQST: az=%.2f el=%.2f"
							 " degps: az=%.2f el=%.2f\n",
							 start_az, start_alt, az_track_degps,
							 alt_track_degps);
					uart_reply(reply);
				}
			}
			else
			{
				snprintf(reply, sizeof(reply), "Dev state invalid\n");
				uart_reply(reply);
			}

			break;

		case CMD_MOVE:
			/*TODO*/
			break;

		case CMD_PARK:
			if (device_state == DEV_STATE_OPEN)
			{
				uart_reply("Received close cmd\n");
				device_close();
			}
			else
			{
				uart_reply("Can't close, dev not in right state.\n");
			}

			break;

		case CMD_UNPARK:
			if (device_state == DEV_STATE_CLOSED)
			{
				uart_reply("Received open cmd\n");
				device_open();
			}
			else
				uart_reply("Can't open, dev not in right state.\n");

			break;

		case CMD_ABORT:
			/*TODO/*/
			break;

		case CMD_ESTOP:
			Motor_Stop_All();
			device_state = DEV_STATE_FAULT;
			uart_reply("STOP: all motors stopped, enterd FAULT state\n");
			break;

		case CMD_GET_STATUS:
			send_dev_status();
			break;

		case CMD_GET_MOTION:
			send_motion_status();
			break;

		case CMD_SET_ACC:
		{
			float az_acc = 0.0f, el_acc = 0.0f;

			if (parse_two_floats_strict(buf, &az_acc, &el_acc))
				uart_reply("ERR: bad AzEl format for ACC, use CMD<az>,<el>\n");
			else
			{
				ctrl_set_max_accel(az_acc, el_acc);

				snprintf(reply, sizeof(reply),
						 "AzEl XL requested: az=%.2f el=%.2f Set: az=%.2f "
						 "el=%.2f\n",
						 az_acc, el_acc, yaw_traj.max_acceleration,
						 pitch_traj.max_acceleration);
				uart_reply(reply);
			}
		}
		break;

		case CMD_SET_VEL:
		{
			float az_vel = 0.0f, el_vel = 0.0f;

			if (parse_two_floats_strict(buf, &az_vel, &el_vel))
				uart_reply("ERR: bad AzEl format for VELO, use CMD<az>,<el>\n");
			else
			{
				ctrl_set_max_vel(az_vel, el_vel);
				snprintf(
					reply, sizeof(reply),
					"AzEl velocity deg/ps RQST: az=%.2f el=%.2f Set: az=%.2f "
					"el=%.2f\n",
					az_vel, el_vel, yaw_traj.max_velocity,
					pitch_traj.max_velocity);
				uart_reply(reply);
			}
		}
		break;

		case CMD_HW_RESET:
			Motor_Stop_All();
			__disable_irq();
			HAL_NVIC_SystemReset();
			break;

		case CMD_CHIRP:
		{
			uint8_t chirp = strtoul(&buf[1], NULL, 10);
			piezo_set_pattern = (uint8_t)chirp;
		}
		break;

		case CMD_DEBUG:
			print_debug_info();

			/*TODO TESTING RELAY EFFECTS*/
			// if (relaystate == 0)
			// {
			// 	turn_on_relay();
			// 	relaystate = 1;
			// }
			// else
			// {
			// 	turn_off_relay();
			// 	relaystate = 0;
			// }

			break;

		case CMD_LNBSET:
		{
			uint8_t lnb_pol, lnb_ifsel;
			size_t pkt_len;

			if (parse_two_bytes_strict(buf, &lnb_pol, &lnb_ifsel))
			{
				snprintf(reply, sizeof(reply), "ERR: cmd wrong.\n");
				uart_reply(reply);
			}
			else
			{
				switch (lnb_pol)
				{
					case 0:
						sprintf(reply, "Turning LNB off.");
						lnb_set_off();
						turn_off_serit_supply();
						break;
					case 1:
						sprintf(reply, "LNB set vertical.");
						lnb_set_vertical();
						turn_on_serit_supply();
						break;
					case 2:
						sprintf(reply, "LNB set horizontal.");
						lnb_set_horizontal();
						turn_on_serit_supply();
						break;

					default:
						sprintf(reply, "Invalid LNB setting. Turning LNB off.");
						lnb_set_off();
						turn_off_serit_supply();
						break;
				}
				pkt_len = strlen(reply);

				// uart_reply(reply);

				if (lnb_ifsel)
				{
					// sprintf(reply,
					// "LNB LO:10.60 GHz IF:1,100-2,150MHz Freq band "
					// "11.70-12.75 GHz\n");
					snprintf(reply + pkt_len, sizeof(reply) - pkt_len,
							 " LNB LO: 11.7 - 12.75GHz\n");
					set_lnb_lo_high();
				}
				else
				{
					// sprintf(reply,
					// "LNB LO:9.75 GHz IF:950-1,950MHz Freq band "
					// "10.7-11.7Ghz\n");
					snprintf(reply + pkt_len, sizeof(reply) - pkt_len,
							 " LNB LO: 10.7 - 11.7GHz\n");
					set_lnb_lo_low();
				}

				uart_reply(reply);

				snprintf(reply, sizeof(reply), "Got %u - %u.\n", lnb_pol,
						 lnb_ifsel);

				uart_reply(reply);
			}
		}
		break;

		case CMD_PLAN:
			send_motion_plan();
			break;

		default:
			snprintf(reply, sizeof(reply), "ERR: CMD %u unknown.\n", buf[0]);
			uart_reply(reply);
			break;
	}
}

/* Call from main loop — assembles commands from RX FIFO */
void UART_Poll(void)
{
	static char cmd_buf[UART_CMD_BUF_SIZE];
	static uint8_t cmd_idx = 0;

	while (!fifo_is_empty(&rx_fifo))
	{
		uint8_t byte = fifo_pop(&rx_fifo, UART_RX_BUF_SIZE);

		if (byte == '\r' || byte == '\n')
		{
			if (cmd_idx > 0)
			{
				cmd_buf[cmd_idx] = '\0';
				UART_ProcessCommand(cmd_buf);
				cmd_idx = 0;
			}
		}
		else if (cmd_idx < UART_CMD_BUF_SIZE - 1)
		{
			cmd_buf[cmd_idx++] = (char)byte;
		}
	}

	/*check for bytes to TX*/
	if (!uart_tx_running)
	{
		if (!fifo_is_empty(&tx_fifo))
		{
			uint16_t len = 0;
			/*get how many bytes there are to transmit, tx it*/
			uint16_t tail_phys = tx_fifo.tail & (UART_TX_BUF_SIZE - 1);
			uint16_t head_phys = tx_fifo.head & (UART_TX_BUF_SIZE - 1);

			if (head_phys > tail_phys)
				len = head_phys - tail_phys;
			else
				/*physical offset, allways 0-511*/
				len = UART_TX_BUF_SIZE - tail_phys;

			uart_tx_running = 1;

			HAL_UART_Transmit_IT(
				&UartHandle,
				&tx_fifo.buf[tx_fifo.tail & (UART_TX_BUF_SIZE - 1)], len);

			/*keep it free running, physical position always via & mask*/
			tx_fifo.tail += len;
		}
	}
}

void UART_RS485_Poll(void)
{
	static char cmd_buf[UART_CMD_BUF_SIZE];
	static uint8_t cmd_idx = 0;

	while (!fifo_is_empty(&rx_fifo_rs485))
	{
		uint8_t byte = fifo_pop(&rx_fifo_rs485, UART_RX_BUF_SIZE);

		if (byte == '\r' || byte == '\n')
		{
			if (cmd_idx > 0)
			{
				cmd_buf[cmd_idx] = '\0';
				UART_ProcessCommand(cmd_buf);
				cmd_idx = 0;
			}
		}
		else if (cmd_idx < UART_CMD_BUF_SIZE - 1)
		{
			cmd_buf[cmd_idx++] = (char)byte;
		}
	}

	if (!uart_r485_tx_running)
	{
		if (!fifo_is_empty(&tx_fifo_rs485))
		{
			uint16_t tail_phys = tx_fifo_rs485.tail & (UART_TX_BUF_SIZE - 1);
			uint16_t head_phys = tx_fifo_rs485.head & (UART_TX_BUF_SIZE - 1);
			uint16_t len;

			if (head_phys > tail_phys)
				len = head_phys - tail_phys;
			else
				len = UART_TX_BUF_SIZE - tail_phys;

			uart_r485_tx_running = 1;
			HAL_UART_AbortReceive_IT(&UartHandle_rs485);
			HAL_GPIO_WritePin(USARTx_RS485_DE_GPIO_PORT, USARTx_RS485_DE_PIN,
							  GPIO_PIN_SET);

			HAL_UART_Transmit_IT(
				&UartHandle_rs485,
				(uint8_t*)&tx_fifo_rs485
					.buf[tx_fifo_rs485.tail & (UART_TX_BUF_SIZE - 1)],
				len);

			tx_fifo_rs485.tail += len;
		}
	}
}

void send_dev_status(void)
{
	char reply[128];

	/* mainly share slow changin system data*/

	/*
	SENT DATA HERE:
		device state
		AZ max XL
		ALT max XL
		AZ max VELO
		ALT max VELO
		AZ body offset
		ALT body offset
		AZ 0 body tilt
		AZ 90 body tilt
		calib samp bitmap
	*/

	snprintf(reply, sizeof(reply),
			 "%u,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,%u\n",
			 device_state, yaw_traj.max_acceleration,
			 pitch_traj.max_acceleration, yaw_traj.max_velocity,
			 pitch_traj.max_velocity, device_position_calib.azimuth_body_offset,
			 device_position_calib.elevation_body_offset,
			 device_position_calib.az_0_tilt, device_position_calib.az_90_tilt,
			 device_position_calib.calib_pos_bitmap);

	uart_reply(reply);
}

/*sends current values of motion parameters*/
void send_motion_status(void)
{
	/*Send motor related, fast changing data*/
	char reply[128];

	/*send both body positions and where device looks at in the world. */

	/*
	BODY current az pos
	BODY current alt pos
	BODY az setpoint
	BODY alt setpoint

	WORLD current az pos
	WORLD current alt pos
	WORLD az setpoint
	WORLD alt setpoint

(measured datas)
	VELOCITY azimuth
	VELOCITY altitude
	XL az
	XL alt
	*/

	/*convert current pos into wrld*/
	float current_world_az, current_world_alt;
	get_curr_world_az_alt_pos(&current_world_az, &current_world_alt);

	snprintf(reply, sizeof(reply),
			 "%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,"
			 "%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f\n",
			 get_curr_body_az_pos(), get_curr_body_alt_pos(),
			 get_body_az_pos_target(), get_body_alt_pos_target(),
			 current_world_az, current_world_alt, device_world_target.world_az,
			 device_world_target.world_alt, yaw_axis.velocity,
			 pitch_axis.velocity, yaw_axis.acceleration,
			 pitch_axis.acceleration);

	uart_reply(reply);
}

static inline float cnt_ps2_to_deg_ps2(float cnt_s2, float cpd)
{
	return (cnt_s2 * ((float)CONTROLLER_UPDATE_HZ * CONTROLLER_UPDATE_HZ)) /
		   cpd;
}

/* inverse: convert counts/sec back to deg/sec */
static inline float cnt_ps_to_deg_ps(float cnt_s, float cpd)
{
	return (cnt_s * (float)CONTROLLER_UPDATE_HZ) / cpd;
}

/*sends current values of trajectory controller*/
void send_motion_plan(void)
{
	char reply[128];

	// snprintf(reply, sizeof(reply), "%0.4f,%0.4f,%0.4f,%0.4f\n",
	// 		 pitch_traj.profiled_pos, yaw_traj.profiled_pos,
	// 		 pitch_traj.profiled_vel, yaw_traj.profiled_vel);

	float prof_vel_degps_azim =
		cnt_ps_to_deg_ps(yaw_traj.profiled_vel, COUNTS_PER_DEG_YAW);
	float prof_vel_degps_alt =
		cnt_ps_to_deg_ps(pitch_traj.profiled_vel, COUNTS_PER_DEG_PITCH);

	float prof_pos_deg_azim = yaw_traj.profiled_pos / COUNTS_PER_DEG_YAW;
	float prof_pos_deg_alt =
		(pitch_traj.profiled_pos - PITCH_ENC_HORIZON_OFFSET) /
		COUNTS_PER_DEG_PITCH;

	snprintf(reply, sizeof(reply), "%0.4f,%0.4f,%0.4f,%0.4f\n",
			 prof_pos_deg_alt, prof_pos_deg_azim, prof_vel_degps_alt,
			 prof_vel_degps_azim);

	uart_reply(reply);
}

uint8_t parse_two_floats(char* buf, float* a, float* b)
{
	char* comma = strchr(&buf[1], ',');
	if (comma)
	{
		*comma = '\0';
		*a = strtof(&buf[1], NULL);
		*b = strtof(comma + 1, NULL);
		return 0;
	}

	return 1;
}

// Four-float parse (permissive, mirrors your original style)
uint8_t parse_four_floats(char* buf, float* a, float* b, float* c, float* d)
{
	char* c1 = strchr(&buf[1], ',');
	if (!c1) return 1;
	*c1 = '\0';
	*a = strtof(&buf[1], NULL);

	char* c2 = strchr(c1 + 1, ',');
	if (!c2) return 1;
	*c2 = '\0';
	*b = strtof(c1 + 1, NULL);

	char* c3 = strchr(c2 + 1, ',');
	if (!c3) return 1;
	*c3 = '\0';
	*c = strtof(c2 + 1, NULL);

	*d = strtof(c3 + 1, NULL);
	return 0;
}

// Strict two-float parse (errors on too few OR too many parameters)
uint8_t parse_two_floats_strict(char* buf, float* a, float* b)
{
	char* c1 = strchr(&buf[1], ',');
	if (!c1) return 1;	// too few
	*c1 = '\0';
	*a = strtof(&buf[1], NULL);

	if (strchr(c1 + 1, ',')) return 1;	// too many
	*b = strtof(c1 + 1, NULL);
	return 0;
}

// Strict four-float parse (errors on too few OR too many parameters)
uint8_t parse_four_floats_strict(char* buf, float* a, float* b, float* c,
								 float* d)
{
	char* c1 = strchr(&buf[1], ',');
	if (!c1) return 1;
	*c1 = '\0';
	*a = strtof(&buf[1], NULL);

	char* c2 = strchr(c1 + 1, ',');
	if (!c2) return 1;
	*c2 = '\0';
	*b = strtof(c1 + 1, NULL);

	char* c3 = strchr(c2 + 1, ',');
	if (!c3) return 1;
	*c3 = '\0';
	*c = strtof(c2 + 1, NULL);

	if (strchr(c3 + 1, ',')) return 1;	// too many
	*d = strtof(c3 + 1, NULL);
	return 0;
}

uint8_t parse_one_uint_two_floats_strict(char* buf, uint8_t* a, float* b,
										 float* c)
{
	char* c1 = strchr(&buf[1], ',');
	if (!c1) return 1;
	*c1 = '\0';
	*a = (uint8_t)strtoul(&buf[1], NULL, 10);

	char* c2 = strchr(c1 + 1, ',');
	if (!c2) return 1;
	*c2 = '\0';
	*b = strtof(c1 + 1, NULL);

	if (strchr(c2 + 1, ',')) return 1;	// too many
	*c = strtof(c2 + 1, NULL);

	return 0;
}

uint8_t parse_two_bytes_strict(char* buf, uint8_t* a, uint8_t* b)
{
	char* c1 = strchr(&buf[1], ',');
	if (!c1) return 1;	// too few
	*c1 = '\0';
	*a = (uint8_t)strtoul(&buf[1], NULL, 10);

	if (strchr(c1 + 1, ',')) return 1;	// too many
	*b = (uint8_t)strtoul(c1 + 1, NULL, 10);
	return 0;
}
