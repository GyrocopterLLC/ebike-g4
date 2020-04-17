/******************************************************************************
 * Filename: hall_sensor.h
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

#ifndef __HALL_SENSOR_H
#define __HALL_SENSOR_H

#define HALL_PSC_MIN                    15  // 84MHz clock / 16  = 5.25MHz -> 12.5millisec total period
#define HALL_PSC_MAX                    127 // 84MHz clock / 128 = 656.25kHz -> .0998sec total period
#define HALL_PSC_CHG_AMT                16
#define HALL_MIN_CAPTURE                16384 // First 1/4 of the period
#define HALL_MAX_OVERFLOWS              3

#define HALL_PSC_CHANGED_UP             1
#define HALL_PSC_CHANGED_DOWN           2
#define HALL_PSC_CHANGED                (HALL_PSC_CHANGED_UP | HALL_PSC_CHANGED_DOWN)
#define HALL_STOPPED                    4

#define HALL_ROT_UNKNOWN                0
#define HALL_ROT_FORWARD                1
#define HALL_ROT_REVERSE                2

// Error checking
#define HALL_MAX_SPEED_CHANGE           (3.0f)
#define HALL_MIN_STEADY_ROTATION_COUNT  (6) // One full electrical rotation

typedef struct _hallsensor{
    float Speed;
    float PreviousSpeed;
    uint32_t CallingFrequency;
    float AngleIncrement;
    float Angle;
    uint32_t CaptureValue;
    uint32_t CaptureForState[6];
    uint16_t Prescaler;
    uint16_t PrescalerForState[6];
    uint8_t Status;
    uint8_t OverflowCount;
    uint8_t SteadyRotationCount;
    uint8_t RotationDirection;
    uint8_t PreviousRotationDirection;
    uint8_t CurrentState;
    uint8_t Valid;

} HallSensor_HandleTypeDef;

typedef struct _hallsensorpll{
    float Alpha; // Gain for phase difference
    float Beta; // Gain for frequency (fixed at alpha^2/2)
    float dt; // Timestep
    float Frequency; // Output frequency
    float Phase; // Output angle
    uint8_t Valid; // Is phase locked?
    uint16_t ValidCounter; // Increments to saturation while locked

} HallSensorPLL_HandleTypeDef;

#define PLL_LOCKED_PHASE_ERROR      (0.2f)
#define PLL_LOCKED_COUNTS           (1000)

#define PLL_UNLOCKED                (0)
#define PLL_LOCKED                  (1)

#define ANGLE_INVALID               (0)
#define ANGLE_VALID                 (1)

void HALL_Init(uint32_t callingFrequency);

uint8_t HALL_AutoGenFwdTable(float* angleTab, uint8_t* fwdTab);
uint8_t HALL_AutoGenFwdInvTable(float* angleTab, uint8_t* fwdInvTab);
uint8_t HALL_AutoGenRevTable(float* angleTab, uint8_t* revTab);
uint8_t HALL_AutoGenRevInvTable(float* angleTab, uint8_t* revInvTab);

uint8_t HALL_GetState(void);
void HALL_IncAngle(void);
uint16_t HALL_GetAngle(void);
float HALL_GetAngleF(void);

uint32_t HALL_GetSpeed(void);
float HALL_GetSpeedF(void);
uint8_t HALL_GetDirection(void);
uint8_t HALL_IsValid(void);

void HALL_PLLUpdate(void);
uint16_t HALL_GetPLLAngle(void);
float HALL_GetPLLAngleF(void);
uint32_t HALL_GetPLLSpeed(void);
float HALL_GetPLLSpeedF(void);
uint8_t HALL_PLLIsValid(void);

uint8_t HALL_SetAngle(uint8_t state, float newAngle);
uint8_t HALL_SetAngleTable(float* angleTab);
float* HALL_GetAngleTable(void);
float HALL_GetAngleFromTable(uint8_t state);
float HALL_GetStateMidpoint(uint8_t state);
void HALL_ChangeFrequency(uint32_t newfreq);
void HALL_EnableHallDetection(float* angleTable, uint8_t tableLength);
void HALL_DisableHallDetection(void);
void HALL_UpdateCallback(void);
void HALL_CaptureCallback(void);

void HALL_SaveVariables(void);
void HALL_LoadVariables(void);


#endif // __HALL_SENSOR_H
