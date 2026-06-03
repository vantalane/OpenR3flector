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

#ifndef UART_COMMUNICATIONS__H
#define UART_COMMUNICATIONS__H

#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "stm32f4xx_hal.h"

#define UART_RX_BUF_SIZE 256 /* must be power of 2 */
#define UART_TX_BUF_SIZE 512 /* must be power of 2 */
#define UART_CMD_BUF_SIZE 128

extern UART_HandleTypeDef UartHandle; /* USART1: debug / command interface */
extern UART_HandleTypeDef UartHandle_rs485; /* USART2: rs485 interface */

/* Circular buffer type */
typedef struct
{
	volatile uint8_t buf[UART_TX_BUF_SIZE]; /* sized to larger of RX/TX */
	volatile uint16_t head;
	volatile uint16_t tail;
} uart_fifo_t;

extern uart_fifo_t rx_fifo;
extern uart_fifo_t tx_fifo;
extern volatile uint8_t s_rx_byte; /* single byte for HAL_UART_Receive_IT */
extern volatile uint8_t uart_tx_running;

/* uart stuff for the RS 485 interface*/
extern uart_fifo_t rx_fifo_rs485;
extern uart_fifo_t tx_fifo_rs485;
/* single byte for HAL_UART_Receive_IT */
extern volatile uint8_t s_rx_byte_rs485;
extern volatile uint8_t uart_r485_tx_running;

typedef enum
{
	CMD_HOME = 'h',		   /*perform homing calib*/
	CMD_HOME_UNSAFE = 'u', /*althoming calib*/
	CMD_GOTO = 'g',		   /*goto AZ/ALT pos*/
	CMD_SYNC = 'w',		   /*sync current pos with given pos*/
	CMD_TRACK = 't',	   /*TBD*/
	CMD_MOVE = 'b',		   /*Move in a give direction*/
	CMD_PARK = 'c',
	CMD_UNPARK = 'o',
	CMD_ABORT = 'q',	  /*Stop current movement to setpos*/
	CMD_ESTOP = 'e',	  /*Immediate HALT and enter fault mode*/
	CMD_GET_STATUS = 's', /*get status of device*/
	CMD_GET_MOTION = 'm',
	CMD_SET_ACC = 'a',	/*configure acceleration of axes*/
	CMD_SET_VEL = 'v',	/*configure velocity of axes*/
	CMD_HW_RESET = 'r', /*Resets the hardware*/
	CMD_CHIRP = 'j',
	CMD_DEBUG = 'd',
	CMD_LNBSET = 'p',
	CMD_PLAN = 'l' /*traj plan print*/

} uart_commands_t;

/*functions*/
void UART_set_config(void);
void UART_RS485_set_config(void);

void UART_SendString(const char* str);
void UART_RS485_SendString(const char* str);
void uart_reply(const char* s); /*write to both interfaces*/

void UART_Poll(void);
void UART_RS485_Poll(void);

void UART_ProcessCommand(const char* cmd);

/*command handling*/
void send_dev_status(void);	   /*share device status, cfgs*/
void send_motion_status(void); /*share motion related data*/
void send_motion_plan(void);   /*mainly for tuning, send traj data*/

void get_time_sync(void);
void sync_pos(float az_pos, float alt_pos); /*idk if needed*/

/*return 1 if faulty. 0 if ok*/
uint8_t parse_two_floats(char* buf, float* a, float* b);
uint8_t parse_four_floats(char* buf, float* a, float* b, float* c, float* d);
uint8_t parse_two_floats_strict(char* buf, float* a, float* b);
uint8_t parse_four_floats_strict(char* buf, float* a, float* b, float* c,
								 float* d);
uint8_t parse_one_uint_two_floats_strict(char* buf, uint8_t* a, float* b,
										 float* c);
uint8_t parse_two_bytes_strict(char* buf, uint8_t* a, uint8_t* b);

uint16_t fifo_count(uart_fifo_t* f, uint16_t size);
uint8_t fifo_is_empty(uart_fifo_t* f);
uint8_t fifo_is_full(uart_fifo_t* f, uint16_t size);
void fifo_push(uart_fifo_t* f, uint8_t byte, uint16_t size);
uint8_t fifo_pop(uart_fifo_t* f, uint16_t size);

#endif