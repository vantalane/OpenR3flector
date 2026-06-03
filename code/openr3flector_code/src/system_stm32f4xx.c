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

#include "stm32f4xx.h"

#if !defined(HSE_VALUE)
/*!< Default value of the External oscillator in Hz */
#define HSE_VALUE ((uint32_t)8000000)
#endif /* HSE_VALUE */

#if !defined(HSI_VALUE)
/*!< Value of the Internal oscillator in Hz  HSI_VALUE */
#define HSI_VALUE ((uint32_t)16000000)
#endif

/* #define VECT_TAB_SRAM */
/*!< Vector Table base offset field.  This value must be a multiple of 0x200. */
#define VECT_TAB_OFFSET 0x00

/* This variable is updated in three ways:
	1) by calling CMSIS function SystemCoreClockUpdate()
	2) by calling HAL API function HAL_RCC_GetHCLKFreq()
	3) each time HAL_RCC_ClockConfig() is called to configure the system clock
   frequency Note: If you use this function to configure the system clock; then
   there is no need to call the 2 first functions listed above, since
   SystemCoreClock variable is updated automatically.
*/
uint32_t SystemCoreClock = 16000000;
const uint8_t AHBPrescTable[16] = {0, 0, 0, 0, 0, 0, 0, 0,
								   1, 2, 3, 4, 6, 7, 8, 9};
const uint8_t APBPrescTable[8] = {0, 0, 0, 0, 1, 2, 3, 4};

void SystemInit(void)
{
/* FPU settings ------------------------------------------------------------*/
#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
	SCB->CPACR |=
		((3UL << 10 * 2) | (3UL << 11 * 2)); /* set CP10 and CP11 Full Access */
#endif
	/* Reset the RCC clock configuration to the default reset state
	 * ------------*/
	/* Set HSION bit */
	RCC->CR |= (uint32_t)0x00000001;

	/* Reset CFGR register */
	RCC->CFGR = 0x00000000;

	/* Reset HSEON, CSSON and PLLON bits */
	RCC->CR &= (uint32_t)0xFEF6FFFF;

	/* Reset PLLCFGR register */
	RCC->PLLCFGR = 0x24003010;

	/* Reset HSEBYP bit */
	RCC->CR &= (uint32_t)0xFFFBFFFF;

	/* Disable all interrupts */
	RCC->CIR = 0x00000000;

	/* Configure the Vector Table location add offset address
	 * ------------------*/
#ifdef VECT_TAB_SRAM
	SCB->VTOR = SRAM_BASE |
				VECT_TAB_OFFSET; /* Vector Table Relocation in Internal SRAM */
#else
	SCB->VTOR = FLASH_BASE |
				VECT_TAB_OFFSET; /* Vector Table Relocation in Internal FLASH */
#endif
}

void SystemCoreClockUpdate(void)
{
	uint32_t tmp = 0, pllvco = 0, pllp = 2, pllsource = 0, pllm = 2;

	/* Get SYSCLK source
	 * -------------------------------------------------------*/
	tmp = RCC->CFGR & RCC_CFGR_SWS;

	switch (tmp)
	{
		case 0x00: /* HSI used as system clock source */
			SystemCoreClock = HSI_VALUE;
			break;
		case 0x04: /* HSE used as system clock source */
			SystemCoreClock = HSE_VALUE;
			break;
		case 0x08: /* PLL used as system clock source */

			/* PLL_VCO = (HSE_VALUE or HSI_VALUE / PLL_M) * PLL_N
			   SYSCLK = PLL_VCO / PLL_P
			   */
			pllsource = (RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) >> 22;
			pllm = RCC->PLLCFGR & RCC_PLLCFGR_PLLM;

			if (pllsource != 0)
			{
				/* HSE used as PLL clock source */
				pllvco = (HSE_VALUE / pllm) *
						 ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6);
			}
			else
			{
				/* HSI used as PLL clock source */
				pllvco = (HSI_VALUE / pllm) *
						 ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6);
			}

			pllp = (((RCC->PLLCFGR & RCC_PLLCFGR_PLLP) >> 16) + 1) * 2;
			SystemCoreClock = pllvco / pllp;
			break;
		default:
			SystemCoreClock = HSI_VALUE;
			break;
	}
	/* Compute HCLK frequency
	 * --------------------------------------------------*/
	/* Get HCLK prescaler */
	tmp = AHBPrescTable[((RCC->CFGR & RCC_CFGR_HPRE) >> 4)];
	/* HCLK frequency */
	SystemCoreClock >>= tmp;
}
