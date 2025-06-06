/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Auto-generated by STM32CubeIDE
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* Private includes -----------------------------------------------*/
#include "stm32f103xb.h"

/* Private defines ------------------------------------------------*/
#define SystemCoreClock		(72000000UL)

/* Private macros -------------------------------------------------*/
#define BIT(n)				(1UL << (n))

#define __enable_irq() 		__asm volatile ("cpsie i" : : : "memory")
#define __disable_irq() 	__asm volatile ("cpsid i" : : : "memory")

/* Private typedefs ------------------------------------------------*/

/* Private variables -----------------------------------------------*/
static volatile uint32_t uwTick = 0;

/* Private prototype function --------------------------------------*/
void SysReset_Check(void);
void SystemClock_Config(void);
void MX_GPIO_Init(void);
void SysTick_Config(uint32_t tick);

/* Main program ----------------------------------------------------*/
/*
 * Activities:
 * 1. Provide the routine "SysTick_Config" to setup the SysTick
 *    to any period, and enable its interrupt vector "SysTick_Handler"
 * 1.1. Toggle a GPIO inside of the interrupt handler and verify
 * 		the SysTick period using an oscilloscope connected to the GPIO
 * 2. Use the routine written in Activity 1 to configure the
 *    SysTick at 1ms, add a uint32_t global variable called uwTick
 *	  that increment its value by 1 in the interrupt handler.
 * 2.1. In the main loop, use the uwTick to toggle a GPIO at some
 * 		multiple of the SysTick period (1 ms)
 */
int main(void)
{
	/* Check system reset */
	SysReset_Check();

	/* System Clock configuration */
	SystemClock_Config();

	/* GPIO initialization */
	MX_GPIO_Init();

	/* Systick configuration */
	//SysTick_Config(SystemCoreClock/1000);
	SysTick_Config(7200000); // @10Hz => SystemCoreClock/10

	/* Loop forever */
	uint32_t uwLastTime = uwTick;
	while (1)
	{
		/* Systick Demo II
		if(uwTick - uwLastTime > uwDelay)
		{
			GPIOA->ODR ^= BIT(5);
			uwLastTime = uwTick;
		}
		//*/
	}
}


/* Private function definition -------------------------------------*/
void SysReset_Check(void)
{
	if( RCC->CSR & BIT(31) )
	{
		// Handle low-power reset
	}
	else if( RCC->CSR & BIT(30) )
	{
		// Handle WWDG reset
	}
	else if( RCC->CSR & BIT(29) )
	{
		// Handle IWDG reset
	}
	else if( RCC->CSR & BIT(28) )
	{
		// Handle Software reset
	}
	else if( RCC->CSR & BIT(27) )
	{
		// Handle POR/PDR reset
	}
	else if( RCC->CSR & BIT(26) )
	{
		// Handle NRST pin reset
	}

	// Clear reset flags
	RCC->CSR |= BIT(24);
}

void SystemClock_Config(void)
{
	/* 1. Enable HSE clock */
	RCC->CR |= BIT(16);				// Enable HSE signal
	while( !(RCC->CR & BIT(17)) ); 	// Polling for HSE Ready

	/* 2. Enable pre-fetch buffer */
	FLASH->ACR |= BIT(4);			// Enable pre-fetch
	while( !(FLASH->ACR & BIT(5)) );// Polling for pre-fetch buffer enable

	/* 3. Set wait states */
	FLASH->ACR &= ~(0x07);			// Reset wait states
	FLASH->ACR |= BIT(1);			// 2 wait states

	/* 4. Configure prescaler, multiplier and clock MUX. */
	// Prescalers
	RCC->CFGR &= ~(0x07 << 8);	// APB1CLK = HCLK/1 = 72MHz (APB1 <= 36MHz)
	RCC->CFGR |= (0x04 << 8);	// APB1CLK = HCLK/2 = 36MHz
	RCC->CFGR &= ~(0x07 << 11);	// APB2CLK = HCLK/1 = 72MHz (APB1 <= 72MHz)
	RCC->CFGR &= ~BIT(22); 		// USBCLK = PLLCLK / 1.5 = 48MHz
	RCC->CFGR |= ~(0x03 << 14); // ADCCLK = HCLK / 2 = 36MHz
	RCC->CFGR &= ~(0x0F << 4);	// HCLK = SYSCLK / 1 = 72MHz
	// MUX
	RCC->CFGR &= ~BIT(17);		// HSE not divided for PLL entry (8MHz * 1)
	RCC->CFGR |= BIT(16);		// HSE entry on PLL clock to multiply (8MHz)
	RCC->CFGR &= ~(0x0F << 18); // Clear PLL multiple (PLLMUL = x2)
	RCC->CFGR |= (0x07 << 18);	// PLLMUL = x9 (8MHz * 9 = 72MHz)

	/* 5. Enable PLL*/
	RCC->CR |= BIT(24);
	while( !(RCC->CR & BIT(25)) ); // Polling for PLL locked

	/* 6. Set clock source */
	RCC->CFGR &= ~(0x03); 				// Clear SW
	RCC->CFGR |= BIT(1);				// SYSCLK = PLLCLK = 72MHz
	while (!(RCC->CFGR & (0x02 << 2) ));// Polling for system clock switch status
}

void MX_GPIO_Init(void)
{
	// Enable clock for GPIOA, GPIOD peripherals
	RCC->APB2ENR |= BIT(2) | BIT(5);

	// PA5 as push-pull output (2 MHz)
	GPIOA->CRL &= ~(0x0F << 20); // Clear MODE5 & CNF5
	GPIOA->CRL |= BIT(21);
}

void SysTick_Config(uint32_t tick)
{
	/*
	 * SysTick is a 24-bit counter with clock derived from the AHBClock
	 *
	 * Steps:
	 * 0. Reset systiick control
	 * 1. Programming reload value using LOAD register
	 * 2. Clear current value using the VAL variable
	 * 3. Select clock source via CLKSOURCE bit (2) on CTRL register.
	 *		0 - AHB/8
	 * 		1 - Processor Clock (AHB)
	 * 4. Enable Tick interrupt by setting TICKINT bit (1) of CTRL register.
	 * 5. Start SysTick timer using the ENABLE bit (0) of CTRL register
	 */
	SysTick->CTRL = 0;			// Reset Systick
	
	/* Set interrupt priority via SCB->SHP register.
	 * Although the SCB_SHPx are 32-bit registers (x = 1, 2, 3)
	 * the library define these registers as an array of 8-bit
	 * registers, since SHPR1-SHPR3 are byte accessible. To set the
	 * priority of the core peripherals, an 8-bit field is defined
	 * along each register, denoted to as PRI_n, where n = 4,...,15.
	 * The lower the value, the greater the priority of the
	 * corresponding interrupt.
	 *
	 * -------------------------------------------
	 * 			Handler 			Field Register
	 * -------------------------------------------
	 * Memory management fault 		PRI_4
	 * System handler Bus fault 	PRI_5
	 * Usage fault 					PRI_6
	 * SVCall 						PRI_11
	 * PendSV 						PRI_14
	 * SysTick 						PRI_15
	 * --------------------------------------------
	 *
	 * Each PRI_n field is 8 bits wide, but the processor implements
	 * only bits[7:4] of each field, and bits[3:0] read as zero
	 *
	 * SysTick is found on PRI_15 (MSByte of the SHPR3 register)
	 *
	 * In the library SCB->SHP[0] is related to PRI_4, SCB->SHP[1] to
	 * PRI_5, and so on. Therefore, to located a particular "n" we only
	 * need to subtract 4. Hence, for the Systick (PRI_15) n is 15, and
	 * is located on 15 - 4 = 11, i.e., on SCB->SHP[11].
	 */
	// Set a priority
	// Set reload value
	// Reset Systick counter
	// Systick clock is AHBCLK
	// Enable Systick interrupt
	// Enable Systick timer
}


/* Interrupt Handlers ------------------------------------------------*/
void SysTick_Handler(void)
{
	
}
