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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F4xx_IT_H
#define __STM32F4xx_IT_H

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

	/* Exported types
	 * ------------------------------------------------------------*/
	/* Exported constants
	 * --------------------------------------------------------*/
	/* Exported macro
	 * ------------------------------------------------------------*/
	/* Exported functions
	 * ------------------------------------------------------- */

	void NMI_Handler(void);
	void HardFault_Handler(void);
	void MemManage_Handler(void);
	void BusFault_Handler(void);
	void UsageFault_Handler(void);
	void SVC_Handler(void);
	void DebugMon_Handler(void);
	void PendSV_Handler(void);
	void SysTick_Handler(void);
	void USARTx_IRQHandler(void);

	void HAL_UART_TxCpltCallback(UART_HandleTypeDef* UartHandle);
	void HAL_UART_RxCpltCallback(UART_HandleTypeDef* UartHandle);
	void HAL_UART_ErrorCallback(UART_HandleTypeDef* UartHandle);

	void EXTI3_IRQHandler(void);
	void EXTI4_IRQHandler(void);
	void EXTI9_5_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* __STM32F4xx_IT_H */
