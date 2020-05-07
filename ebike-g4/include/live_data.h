/******************************************************************************
 * Filename: live_data.h
 ******************************************************************************

 Copyright (c) 2020 David Miller

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.
 */

#ifndef __LIVE_DATA_H
#define __LIVE_DATA_H

#include "project_parameters.h"

typedef enum _live_datarate_type {
    DataRate_50Hz = 0,
    DataRate_100Hz = 1,
    DataRate_200Hz = 2,
    DataRate_500Hz = 3,
    DataRate_1kHz = 4,
    DataRate_5kHz = 5
} Live_DataRate;

typedef struct _live_config_type {
    uint16_t Num_Outputs;
    uint16_t Speed;
    uint16_t Choices[MAX_LIVE_OUTPUTS];
} Live_Config;

void LIVE_Init(uint32_t calling_freq);
void LIVE_AssemblePacket(Main_Variables* mvar);
void LIVE_SendPacket(void);

// Command interaction functions
uint8_t LIVE_TurnOnData(void);
uint8_t LIVE_TurnOffData(void);
uint8_t LIVE_SetSpeed(uint16_t newSpeed);
uint16_t LIVE_GetSpeed(void);
uint8_t LIVE_SetNumOutputs(uint16_t numOutputs);
uint16_t LIVE_GetNumOutputs(void);
uint8_t LIVE_SetOutput(uint8_t whichOutput, uint16_t newSetting);
uint16_t LIVE_GetOutput(uint8_t whichOutput);

#endif
