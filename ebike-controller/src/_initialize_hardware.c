//
// This file is part of the µOS++ III distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

// ----------------------------------------------------------------------------

#include "stm32f4xx.h"

// ----------------------------------------------------------------------------

// The external clock frequency is specified as a preprocessor definition
// passed to the compiler via a command line option (see the 'C/C++ General' ->
// 'Paths and Symbols' -> the 'Symbols' tab, if you want to change it).
// The value selected during project creation was HSE_VALUE=8000000.
//
// The code to set the clock is at the end.
//
// Note1: The default clock settings assume that the HSE_VALUE is a multiple
// of 1MHz, and try to reach the maximum speed available for the
// board. It does NOT guarantee that the required USB clock of 48MHz is
// available. If you need this, please update the settings of PLL_M, PLL_N,
// PLL_P, PLL_Q to match your needs.
//
// Note2: The external memory controllers are not enabled. If needed, you
// have to define DATA_IN_ExtSRAM or DATA_IN_ExtSDRAM and to configure
// the memory banks in system/src/cmsis/system_stm32f4xx.c to match your needs.

// ----------------------------------------------------------------------------

// Forward declarations.

void
__initialize_hardware(void);

void
configure_system_clock(void);

// ----------------------------------------------------------------------------

// This is the application hardware initialisation routine,
// redefined to add more inits.
//
// Called early from _start(), right after data & bss init, before
// constructors.
//
// After Reset the Cortex-M processor is in Thread mode,
// priority is Privileged, and the Stack is set to Main.

void
__initialize_hardware(void)
{
  // Call the CSMSIS system initialisation routine.
  SystemInit();

#if defined (__VFP_FP__) && !defined (__SOFTFP__)

  // Enable the Cortex-M4 FPU only when -mfloat-abi=hard.
  // Code taken from Section 7.1, Cortex-M4 TRM (DDI0439C)

  // Set bits 20-23 to enable CP10 and CP11 coprocessor
  SCB->CPACR |= (0xF << 20);

#endif // (__VFP_FP__) && !(__SOFTFP__)

  // Initialise the HAL Library; it must be the first
  // instruction to be executed in the main program.
  //HAL_Init();

  // Warning: The HAL always initialises the system timer.
  // For this to work, the default SysTick_Handler must not hang
  // (see system/src/cortexm/exception_handlers.c)

  // Unless explicitly enabled by the application, we prefer
  // to keep the timer interrupts off.
  //HAL_SuspendTick();

  // Enable HSE Oscillator and activate PLL with HSE as source
  configure_system_clock();
}

#if 0

// This is a sample SysTick handler, use it if you need HAL timings.
void __attribute__ ((section(".after_vectors")))
SysTick_Handler(void)
{
	HAL_IncTick();
}

#endif

// ----------------------------------------------------------------------------

/**
 * @brief  System Clock Configuration
 *         The system Clock is configured as follow :
 *            System Clock source            = PLL (HSE)
 *            SYSCLK(Hz)                     = 168000000
 *            HCLK(Hz)                       = 168000000
 *            AHB Prescaler                  = 1
 *            APB1 Prescaler                 = 4
 *            APB2 Prescaler                 = 2
 *            HSE Frequency(Hz)              = HSE_VALUE
 *            PLL_M                          = (HSE_VALUE/1000000u)
 *            PLL_N                          = 336
 *            PLL_P                          = 2
 *            PLL_Q                          = 7
 *            VDD(V)                         = 3.3
 *            Main regulator output voltage  = Scale1 mode
 *            Flash Latency(WS)              = 5
 * @param  None
 * @retval None
 */

void
configure_system_clock(void)
{
	uint32_t timeout = 0;
	uint32_t tempreg, pllm, pllp, plln, pllq;
	// Enable Power Control clock
	//__PWR_CLK_ENABLE();
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;

	// The voltage scaling allows optimizing the power consumption when the
	// device is clocked below the maximum system frequency, to update the
	// voltage scaling value regarding system frequency refer to product
	// datasheet.
	PWR->CR |= PWR_CR_VOS;

	// Set system clock to HSI before changing settings all willy nilly
	tempreg = RCC->CFGR;
	tempreg &= ~(RCC_CFGR_SW);
	tempreg |= RCC_CFGR_SW_HSI;
	RCC->CFGR = tempreg;

	// Make sure it switched over
	timeout = 20000;
	do
	{
		timeout--;
		if(timeout == 0) return;
	}while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI);

	// Disable PLL
	RCC->CR &= ~(RCC_CR_PLLON);

	// Enable HSE Oscillator and activate PLL with HSE as source
	RCC->CR |= RCC_CR_HSEBYP; // Enable bypass mode: using an external clock, not a crystal
	RCC->CR |= RCC_CR_HSEON; // Turn on HSE
	// Wait for HSE to turn on
	timeout = 20000;
	do
	{
	  timeout--;
	  if(timeout == 0) return;
	}
	while(!(RCC->CR & RCC_CR_HSERDY));

	// Configure PLL multipliers
	pllm = HSE_VALUE / 1000000u; // Set so the PLL input clock is 1MHz (HSE/pllm = 1MHz)
	pllp = 0; // This is actually PLL_P = 2
	pllq = 7;
	plln = 336;
	// Set PLL multipliers with HSE selected as input source
	RCC->PLLCFGR = (pllq << 24)
			| RCC_PLLCFGR_PLLSRC
			| (pllp << 16)
			| (plln << 6)
			| (pllm);

	// Enable PLL
	RCC->CR |= RCC_CR_PLLON;

	timeout = 20000;
	do
	{
		timeout--;
		if(timeout==0) return;
	}while(!(RCC->CR & RCC_CR_PLLRDY)); // Wait for PLL to start

	// Set flash latency
	FLASH->ACR &= ~FLASH_ACR_LATENCY;
	FLASH->ACR |= FLASH_ACR_LATENCY_5WS;

	// Set AHB, APB prescalers
	RCC->CFGR &= ~(RCC_CFGR_PPRE2 | RCC_CFGR_PPRE1 | RCC_CFGR_HPRE);
	RCC->CFGR |= (RCC_CFGR_PPRE2_DIV2) | (RCC_CFGR_PPRE1_DIV4); // APB2 divided by 2, APB1 divided by 4

	// Select PLL as system clock
	RCC->CFGR |= RCC_CFGR_SW_PLL;


}

// ----------------------------------------------------------------------------
