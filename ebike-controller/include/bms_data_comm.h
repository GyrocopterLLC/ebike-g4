/******************************************************************************
 * Filename: bms_data_comm.h
 ******************************************************************************

 Copyright (c) 2019 David Miller

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

#ifndef _BMS_DATA_COMM_H_
#define _BMS_DATA_COMM_H_

#include "stm32f4xx.h"

#define BMS_TX_BUFFER_LENGTH    (PACKET_MAX_DATA_LENGTH+PACKET_OVERHEAD_BYTES)
#define BMS_RX_BUFFER_LENGTH    (2*PACKET_MAX_DATA_LENGTH)

#define BROADCAST_ADDRESS       ((uint8_t)0xFF)
#define R_ADDRESS               ((uint16_t)0x0001)

#define MAX_BMS_BOARDS          16
#define BMS_TIMEOUT_MS          100 // Max time to wait for a response
#define BMS_MAX_RETRIES         3 // Max times to retry ping before calling it a fail

typedef enum _bms_comm_state {
    BMS_STATE_IDLE,
    BMS_STATE_ADDRESSING,
    BMS_STATE_VOLTAGES,
    BMS_STATE_STATUSES
} BMS_CommState;

typedef struct _bms_type {
    uint8_t NumConnectedBoards; // Equal to the address of the last board in the chain
    uint16_t BattsPerBoard[MAX_BMS_BOARDS]; // Could be different on each board

    float* BattVoltages; // Dynamically allocated to length = sum(BattsPerBoard)
    uint32_t* BattStatuses; // Similar to above.
    // When indexing the above arrays, each board's batteries are placed in
    // sequence. So to get a particular board's battery, you need to go past
    // the sum of all previous board's batteries. That's what the BattsPerBoard
    // array can be used for.

    // When refreshing data, the following variables are used to hold
    // the current status of the query. When a response comes back, these
    // variables tell the loop which battery to ping next.
    uint8_t CurrentBoard;
    uint8_t CurrentBatt;
    uint8_t RetryCount;
    BMS_CommState CommState; // For remembering what we're trying to do in the loop

    uint32_t TimeoutStart; // Capture the millisecond tick
    uint32_t Timeout; // How long the timeout is set to expire. Zero means inactive.

} BMS_Type;

// Commands
// RAM only variables
#define R_ADDRESS           ((uint16_t)0x0001) // I8: Address this device responds to. Zero is unaddressed/reset
#define R_VOLT_BATT1        ((uint16_t)0x0101) // I32(Q16): Voltage of battery 1
#define R_VOLT_BATT2        ((uint16_t)0x0102) // I32(Q16): "" 2
#define R_VOLT_BATT3        ((uint16_t)0x0103) // I32(Q16): "" 3
#define R_VOLT_BATT4        ((uint16_t)0x0104) // I32(Q16): "" 4
#define R_STAT_BATT1        ((uint16_t)0x0111) // I32: Status codes for battery 1
#define R_STAT_BATT2        ((uint16_t)0x0112) // I32: "" 2
#define R_STAT_BATT3        ((uint16_t)0x0113) // I32: "" 3
#define R_STAT_BATT4        ((uint16_t)0x0114) // I32: "" 4
#define R_NUM_BATTS         ((uint16_t)0x0121) // I16: Number of battery cells on this BMS

// RAM/EEPROM variables
#define RE_PREFIX           ((uint16_t)0x4000)
#define RE_NUMVARS          (9)
// Calibration constants
#define RE_CAL_BATT1        ((uint16_t)0x4001) // I32(Q16): Calibration for battery 1
#define RE_CAL_BATT2        ((uint16_t)0x4002) // I32(Q16): "" 2
#define RE_CAL_BATT3        ((uint16_t)0x4003) // I32(Q16): "" 3
#define RE_CAL_BATT4        ((uint16_t)0x4004) // I32(Q16): "" 4
// Limits
#define RE_BATT_OVLIM       ((uint16_t)0x4005) // I32(Q16): Maximum battery voltage - over this causes fault
#define RE_BATT_UVLIM       ((uint16_t)0x4006) // I32(Q16): Minimum battery voltage - under this causes fault
// When to balance
#define RE_BATT_SOFTBAL     ((uint16_t)0x4007) // I32(Q16): Beginning of balance - starts PWM balancing
#define RE_BATT_HARDBAL     ((uint16_t)0x4008) // I32(Q16): End of balance - above this, balance is on full (no PWM)
#define RE_BATT_CAPACITY    ((uint16_t)0x4009) // I32: Battery capacity in mAh

// Actions
#define ACTION_CAL_BATT1            ((uint16_t)0x1001)
#define ACTION_CAL_BATT2            ((uint16_t)0x1002)
#define ACTION_CAL_BATT3            ((uint16_t)0x1003)
#define ACTION_CAL_BATT4            ((uint16_t)0x1004)
#define ACTION_CAL_AND_SAVE_BATT1   ((uint16_t)0x1011)
#define ACTION_CAL_AND_SAVE_BATT2   ((uint16_t)0x1012)
#define ACTION_CAL_AND_SAVE_BATT3   ((uint16_t)0x1013)
#define ACTION_CAL_AND_SAVE_BATT4   ((uint16_t)0x1014)

void BMS_Data_Comm_Init(void);
void BMS_Send_One_Packet(uint8_t pktType, uint8_t* data, uint8_t datalen);
void BMS_Restart_Chain(void);
void BMS_Refresh_Data(void);
uint8_t BMS_Busy(void);
void BMS_OneByte_Check(void);
void BMS_Periodic_Check(void); // Called frequently by main

#endif
