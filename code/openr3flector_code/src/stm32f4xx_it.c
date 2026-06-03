/*
 * Copyright (c) 2017 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 * Modifications Copyright (C) 2026 vantalane <mete@kestech.net>
 */

#include "stm32f4xx_it.h"

#include <stdint.h>

#include "functions.h"
#include "main.h"
#include "stm32f4xx_hal_gpio.h"

extern UART_HandleTypeDef UartHandle;
extern UART_HandleTypeDef UartHandle_rs485;

void NMI_Handler(void) {}

void HardFault_Handler(void)
{
	/* Go to infinite loop when Hard Fault exception occurs */
	Motor_Stop_All();
	LED_Set(LED_ID_FILM_RED, 1);
	while (1)
	{
	}
}

void MemManage_Handler(void)
{
	Motor_Stop_All();
	LED_Set(LED_ID_FILM_RED, 1);
	/* Go to infinite loop when Memory Manage exception occurs */
	while (1)
	{
	}
}

void BusFault_Handler(void)
{
	Motor_Stop_All();
	LED_Set(LED_ID_FILM_RED, 1);
	/* Go to infinite loop when Bus Fault exception occurs */
	while (1)
	{
	}
}

void UsageFault_Handler(void)
{
	Motor_Stop_All();
	LED_Set(LED_ID_FILM_RED, 1);
	/* Go to infinite loop when Usage Fault exception occurs */
	while (1)
	{
	}
}

void SVC_Handler(void) {}

void DebugMon_Handler(void) {}

void PendSV_Handler(void) {}

void SysTick_Handler(void)
{
	HAL_IncTick();
}

void USARTx_IRQHandler(void)
{
	HAL_UART_IRQHandler(&UartHandle);
}

void USARTx_RS485_IRQHandler(void)
{
	HAL_UART_IRQHandler(&UartHandle_rs485);
}

/*TX completed callback — clear running flag so Poll() can queue the next chunk
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart)
{
	// LED_Toggle(LED_ID_RED_RIGHT);

	if (huart->Instance == USARTx)
		uart_tx_running = 0;
	else if (huart->Instance == USARTx_RS485)
	{
		HAL_GPIO_WritePin(USARTx_RS485_DE_GPIO_PORT, USARTx_RS485_DE_PIN,
						  GPIO_PIN_RESET);
		uart_r485_tx_running = 0;
		/* Flush DR: clears RXNE + error flags so re-arm doesn't fire
		 * immediately with an echo byte that landed while RX was aborted. On
		 * F4, reading SR then DR is the only way to clear ORE/RXNE. */
		__HAL_UART_CLEAR_OREFLAG(&UartHandle_rs485);
		HAL_UART_Receive_IT(&UartHandle_rs485, (uint8_t*)&s_rx_byte_rs485, 1);
	}
}

/*RX completed callback — push byte into RX FIFO, re-arm */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
	LED_Toggle(LED_ID_RED_RIGHT);
	if (huart->Instance == USARTx)
	{
		if (fifo_count(&rx_fifo, UART_RX_BUF_SIZE) < UART_RX_BUF_SIZE - 1)
		{
			rx_fifo.buf[rx_fifo.head & (UART_RX_BUF_SIZE - 1)] = s_rx_byte;
			rx_fifo.head++;
		}
		HAL_UART_Receive_IT(huart, (uint8_t*)&s_rx_byte, 1);
	}
	else if (huart->Instance == USARTx_RS485)
	{
		if (fifo_count(&rx_fifo_rs485, UART_RX_BUF_SIZE) < UART_RX_BUF_SIZE - 1)
		{
			rx_fifo_rs485.buf[rx_fifo_rs485.head & (UART_RX_BUF_SIZE - 1)] =
				s_rx_byte_rs485;
			rx_fifo_rs485.head++;
		}
		HAL_UART_Receive_IT(huart, (uint8_t*)&s_rx_byte_rs485, 1);
	}
}

/*uart tx/rx error callback*/
void HAL_UART_ErrorCallback(UART_HandleTypeDef* UartHandle)
{
	/* Turn LED3 on: Transfer error in reception/transmission process */
	LED_Set(LED_ID_FILM_RED, 1);
	LED_Set(LED_ID_FILM_GREEN, 1);
}

/* EXTI3 ISR -- pitch endstop */
void EXTI3_IRQHandler(void)
{
	HAL_GPIO_EXTI_IRQHandler(PITCH_ENDSTOP_PIN);
}

/* EXTI4 ISR -- yaw endstop */
void EXTI4_IRQHandler(void)
{
	HAL_GPIO_EXTI_IRQHandler(YAW_ENDSTOP_PIN);
}

/* EXTI9_5 ISR -- handles PA6 (line 6) and PA7 (line 7) */
void EXTI9_5_IRQHandler(void)
{
	if (__HAL_GPIO_EXTI_GET_IT(PITCH_ENC_PIN))
	{
		pitch_axis.counts += pitch_axis.direction; /* +1, -1 or 0 */
		__HAL_GPIO_EXTI_CLEAR_IT(PITCH_ENC_PIN);
	}
	if (__HAL_GPIO_EXTI_GET_IT(YAW_ENC_PIN))
	{
		yaw_axis.counts += yaw_axis.direction;
		__HAL_GPIO_EXTI_CLEAR_IT(YAW_ENC_PIN);
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == PITCH_ENDSTOP_PIN)
	{
		LED_Toggle(LED_ID_RED_LEFT);
		pitch_axis.at_endstop = Endstop_Pitch_IsTriggered();
	}
	else if (GPIO_Pin == YAW_ENDSTOP_PIN)
	{
		LED_Toggle(LED_ID_RED_LEFT);
		yaw_axis.at_endstop = Endstop_Yaw_IsTriggered();
	}
}
