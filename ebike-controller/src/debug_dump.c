/*
 * debug_dump.c
 *
 *  Created on: Sep 5, 2018
 *      Author: David
 */

#ifdef DEBUG_DUMP_USED

#include "stm32f4xx.h"
#include "debug_dump.h"

int16_t DumpData[MAX_DUMP_LENGTH] __attribute__((section(".bss_CCMRAM")));
uint16_t DumpLoc;
uint8_t dumpassignments[MAX_DUMP_OUTPUTS] = DEFAULT_DUMP_ASSIGNMENTS;

void DumpReset(void)
{
  DumpLoc = 0;
}

uint8_t DumpGeneration(float* outputvals)
{
  if(DumpLoc < MAX_DUMP_OUTPUTS)
  {
    DumpData[DumpLoc] = (int16_t)(usbdacvals[dumpassignments[0]-1]*1638.4f);
    DumpData[DumpLoc+1] = (int16_t)(usbdacvals[dumpassignments[1]-1]*1638.4f);
    DumpData[DumpLoc+2] = (int16_t)(usbdacvals[dumpassignments[2]-1]*1638.4f);
    DumpData[DumpLoc+3] = (int16_t)(usbdacvals[dumpassignments[3]-1]*1638.4f);
    DumpLoc+=4;
    return 0;
  }
  else
  {
    DumpLoc = 0;
    return 1;
  }
}

int16_t GetDumpData(void)
{
  if(DumpLoc < MAX_DUMP_OUTPUTS)
  return DumpData[DumpLoc++];
}

uint8_t DumpDone(void)
{
  if(DumpLoc >= MAX_DUMP_OUTPUTS)
  return 1;
  return 0;
}

#endif // DEBUG_DUMP_USED
