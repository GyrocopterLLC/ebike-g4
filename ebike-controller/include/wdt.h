// Used resources:
// IWDG

#ifndef WDT_H_
#define WDT_H_

#include "stm32f4xx.h"

#define IWDG_RELOAD		(0xAAAA)
#define IWDG_KEY		(0x5555)
#define IWDG_START		(0xCCCC)

#define IWDG_PSC		(0x0000) // For a 125usec granularity
#define IWDG_REL_VAL	(400) // 125usec * 400 = 50msec

void WDT_init(void);
void WDT_feed(void);



#endif //WDT_H_
