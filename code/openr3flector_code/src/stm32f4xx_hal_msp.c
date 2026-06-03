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


#include "main.h"

void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	if (huart->Instance == USARTx)
	{
		USARTx_TX_GPIO_CLK_ENABLE();
		USARTx_RX_GPIO_CLK_ENABLE();
		USARTx_CLK_ENABLE();

		GPIO_InitStruct.Pin = USARTx_TX_PIN;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
		GPIO_InitStruct.Alternate = USARTx_TX_AF;
		HAL_GPIO_Init(USARTx_TX_GPIO_PORT, &GPIO_InitStruct);

		GPIO_InitStruct.Pin = USARTx_RX_PIN;
		GPIO_InitStruct.Alternate = USARTx_RX_AF;
		HAL_GPIO_Init(USARTx_RX_GPIO_PORT, &GPIO_InitStruct);

		HAL_NVIC_SetPriority(USARTx_IRQn, 0, 1);
		HAL_NVIC_EnableIRQ(USARTx_IRQn);
	}
	else if (huart->Instance == USARTx_RS485)
	{
		USARTx_RS485_GPIO_CLK_EN();
		USARTx_RS485_CLK_EN();

		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FAST;

		GPIO_InitStruct.Pin = USARTx_RS485_TX_PIN;
		GPIO_InitStruct.Alternate = USARTx_RS485_TX_AF;
		HAL_GPIO_Init(USARTx_RS485_TX_GPIO_PORT, &GPIO_InitStruct);

		GPIO_InitStruct.Pin = USARTx_RS485_RX_PIN;
		GPIO_InitStruct.Alternate = USARTx_RS485_RX_AF;
		HAL_GPIO_Init(USARTx_RS485_RX_GPIO_PORT, &GPIO_InitStruct);

		/* DE pin: GPIO output, start LOW (receive mode) */
		GPIO_InitStruct.Pin = USARTx_RS485_DE_PIN;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Alternate = 0;
		HAL_GPIO_Init(USARTx_RS485_DE_GPIO_PORT, &GPIO_InitStruct);
		HAL_GPIO_WritePin(USARTx_RS485_DE_GPIO_PORT, USARTx_RS485_DE_PIN,
						  GPIO_PIN_RESET);

		HAL_NVIC_SetPriority(USARTx_RS485_IRQn, 0, 1);
		HAL_NVIC_EnableIRQ(USARTx_RS485_IRQn);
	}
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{
	if (huart->Instance == USARTx)
	{
		USARTx_FORCE_RESET();
		USARTx_RELEASE_RESET();
		HAL_GPIO_DeInit(USARTx_TX_GPIO_PORT, USARTx_TX_PIN);
		HAL_GPIO_DeInit(USARTx_RX_GPIO_PORT, USARTx_RX_PIN);
		HAL_NVIC_DisableIRQ(USARTx_IRQn);
	}
	else if (huart->Instance == USARTx_RS485)
	{
		USARTx_RS485_FORCE_RESET();
		USARTx_RS485_RELEASE_RESET();
		HAL_GPIO_DeInit(USARTx_RS485_TX_GPIO_PORT, USARTx_RS485_TX_PIN);
		HAL_GPIO_DeInit(USARTx_RS485_RX_GPIO_PORT, USARTx_RS485_RX_PIN);
		HAL_GPIO_DeInit(USARTx_RS485_DE_GPIO_PORT, USARTx_RS485_DE_PIN);
		HAL_NVIC_DisableIRQ(USARTx_RS485_IRQn);
	}
}
