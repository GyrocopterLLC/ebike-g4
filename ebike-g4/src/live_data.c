/******************************************************************************
 * Filename: live_data.c
 * Description: Assembles packet data contain live updating variables from
 *              the controller. This data can be used for debugging.
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

#include "main.h"

Live_Config lconf;
uint32_t live_speed_reload_vals[MAX_LIVE_SPEED_CHOICES];
uint32_t live_countdown_timer;
uint8_t live_data_on;
Data_Packet_Type live_packet;
uint8_t live_packet_buffer[PACKET_MAX_LENGTH];
uint8_t live_data_buffer[PACKET_MAX_DATA_LENGTH];
uint32_t live_packet_buffer_pos;

void LIVE_Init(uint32_t calling_freq) {
    live_speed_reload_vals[DataRate_50Hz] = calling_freq / 50;
    live_speed_reload_vals[DataRate_100Hz] = calling_freq / 100;
    live_speed_reload_vals[DataRate_200Hz] = calling_freq / 200;
    live_speed_reload_vals[DataRate_500Hz] = calling_freq / 500;
    live_speed_reload_vals[DataRate_1kHz] = calling_freq / 1000;
    live_speed_reload_vals[DataRate_5kHz] = calling_freq / 5000;

    lconf.Num_Outputs = 5;
    lconf.Speed = 0;
    lconf.Choices[0] = LIVE_CHOICE_IA;
    lconf.Choices[1] = LIVE_CHOICE_IB;
    lconf.Choices[2] = LIVE_CHOICE_IC;
    lconf.Choices[3] = LIVE_CHOICE_THROTTLE;
    lconf.Choices[4] = LIVE_CHOICE_HALLANGLE;

    live_data_on = 0;
    live_countdown_timer = live_speed_reload_vals[lconf.Speed];
    live_packet_buffer_pos = 0;
}

void LIVE_AssemblePacket(Main_Variables* mvar) {
    float temp_data;

    if (live_data_on && (lconf.Num_Outputs > 0)) {
        if ((--live_countdown_timer) == 0) {

            if(live_packet_buffer_pos != 0) {
                // this means we're still trying to send a previous packet
                live_countdown_timer = 1; // Set to 1 so the next call will attempt to send again
            } else {
                // First pack in the timestamp
                data_packet_pack_32b(live_data_buffer, mvar->Timestamp);

                for (uint8_t i = 0; i < lconf.Num_Outputs; i++) {
                    switch(lconf.Choices[i]) {
                    case LIVE_CHOICE_UNUSED:
                        temp_data = 0.0f;
                        break;
                    case LIVE_CHOICE_IA:
                        temp_data = mvar->Obv->iA;
                        break;
                    case LIVE_CHOICE_IB:
                        temp_data = mvar->Obv->iB;
                        break;
                    case LIVE_CHOICE_IC:
                        temp_data = mvar->Obv->iC;
                        break;
                    case LIVE_CHOICE_TA:
                        temp_data = mvar->Pwm->tA;
                        break;
                    case LIVE_CHOICE_TB:
                        temp_data = mvar->Pwm->tB;
                        break;
                    case LIVE_CHOICE_TC:
                        temp_data = mvar->Pwm->tC;
                        break;
                    case LIVE_CHOICE_THROTTLE:
                        temp_data = mvar->Ctrl->ThrottleCommand;
                        break;
                    case LIVE_CHOICE_HALLANGLE:
                        temp_data = mvar->Obv->RotorAngle;
                        break;
                    case LIVE_CHOICE_HALLSPEED:
                        temp_data = mvar->Obv->RotorSpeed_eHz;
                        break;
                    case LIVE_CHOICE_HALLACCEL:
                        temp_data = mvar->Obv->RotorAccel_eHzps;
                        break;
                    case LIVE_CHOICE_HALLSTATE:
                        temp_data = (float)(mvar->Obv->HallState);
                        break;
                    case LIVE_CHOICE_VBUS:
                        temp_data = mvar->Obv->BusVoltage;
                        break;
                    case LIVE_CHOICE_ID:
                        temp_data = mvar->Foc->Park_D;
                        break;
                    case LIVE_CHOICE_IQ:
                        temp_data = mvar->Foc->Park_Q;
                        break;
                    case LIVE_CHOICE_TD:
                        temp_data = mvar->Foc->Id_PID->Out;
                        break;
                    case LIVE_CHOICE_TQ:
                        temp_data = mvar->Foc->Iq_PID->Out;
                        break;
                    case LIVE_CHOICE_VA:
                        temp_data = mvar->Obv->vA;
                        break;
                    case LIVE_CHOICE_VB:
                        temp_data = mvar->Obv->vB;
                        break;
                    case LIVE_CHOICE_VC:
                        temp_data = mvar->Obv->vC;
                        break;
                    case LIVE_CHOICE_FTEMP:
                        temp_data = mvar->Obv->FetTemperature;
                        break;
                    case LIVE_CHOICE_ERRORCODE:
                        temp_data = 0.0f;
                        break;
                    default:
                        temp_data = 0.0f;
                        break;
                    }

                    data_packet_pack_float(&(live_data_buffer[(i * sizeof(float)) + sizeof(uint32_t)]), temp_data);
                }
                live_packet.TxBuffer = live_packet_buffer;
                if (data_packet_create(&live_packet, CONTROLLER_STREAM_DATA, live_data_buffer,
                    sizeof(uint32_t) + lconf.Num_Outputs * sizeof(float))) {
                    live_packet_buffer_pos = live_packet.TxLength;
                } else {
                    live_packet_buffer_pos = 0;
                }

                live_countdown_timer = live_speed_reload_vals[lconf.Speed];
            }
        }
    }
}

void LIVE_SendPacket(void) {
    if (live_data_on) {
        if (live_packet_buffer_pos > 0) {
            if (VCP_Write(live_packet_buffer, live_packet_buffer_pos) != -1) {
                live_packet_buffer_pos = 0;
            }
        }
    }
}

uint8_t LIVE_TurnOnData(void) {
    live_data_on = 1;
    return RETVAL_OK;
}

uint8_t LIVE_TurnOffData(void) {
    live_data_on = 0;
    return RETVAL_OK;
}

uint8_t LIVE_SetSpeed(uint16_t newSpeed) {
    if(newSpeed < MAX_LIVE_SPEED_CHOICES) {
        lconf.Speed = newSpeed;
        return RETVAL_OK;
    }
    return RETVAL_FAIL;
}

uint16_t LIVE_GetSpeed(void) {
    return lconf.Speed;
}

uint8_t LIVE_SetNumOutputs(uint16_t numOutputs) {
    if(numOutputs <= MAX_LIVE_OUTPUTS) {
        lconf.Num_Outputs = numOutputs;
        return RETVAL_OK;
    }
    return RETVAL_FAIL;
}
uint16_t LIVE_GetNumOutputs(void) {
    return lconf.Num_Outputs;
}

uint8_t LIVE_SetOutput(uint8_t whichOutput, uint16_t newSetting) {
    if( whichOutput < MAX_LIVE_OUTPUTS ) {
        if(newSetting <= MAX_LIVE_DATA_CHOICES) {
            lconf.Choices[whichOutput] = newSetting;
            return RETVAL_OK;
        }
    }
    return RETVAL_FAIL;
}

uint16_t LIVE_GetOutput(uint8_t whichOutput) {
    if( whichOutput < MAX_LIVE_OUTPUTS ) {
        return lconf.Choices[whichOutput];
    }
    return 0xFFFFu;
}
