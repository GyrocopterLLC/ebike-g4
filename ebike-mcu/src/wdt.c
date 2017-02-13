/*
 * wdt.c
 *
 *  Created on: Feb 12, 2017
 *      Author: David
 */

#include "stm32f4xx.h"
#include "wdt.h"

void WDT_init(void)
{
	// Timer is stopped in debug (e.g. breakpoints)
	DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP;
	// Unlock registers
	IWDG->KR = IWDG_KEY;
	// Set prescaler
	IWDG->PR = IWDG_PSC;
	// Set reload value
	IWDG->RLR = IWDG_REL_VAL;
	// Start the countdown timer
	IWDG->KR = IWDG_START;
}

void WDT_feed(void)
{
	IWDG->KR = IWDG_RELOAD;
}
