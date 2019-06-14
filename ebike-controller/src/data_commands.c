/******************************************************************************
 * Filename: data_commands.c
 * Description: 
 *
 *
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

#include "main.h"
#include "data_packet.h"
#include "data_commands.h"

uint16_t data_process_command(Data_Packet_Type* pkt) {
    uint32_t u32Temp;
    uint16_t errCode = DATA_COMMAND_FAIL;

    if(!pkt->RxReady) {
        return DATA_COMMAND_FAIL;
    }
    switch(pkt->PacketType) {
        // Responses from the host
    case GET_RAM_VARIABLE:
        errCode = command_get_ram(pkt->Data, &u32Temp);
        switch(errCode) {
        case RESULT_IS_8B:
            errCode = data_packet_create(pkt, GET_RAM_RESULT,
                (uint8_t*)&u32Temp, 1);
            break;
        case RESULT_IS_16B:
            errCode = data_packet_create(pkt, GET_RAM_RESULT,
                (uint8_t*)&u32Temp, 2);
            break;
        case RESULT_IS_32B:
        case RESULT_IS_FLOAT:
            errCode = data_packet_create(pkt, GET_RAM_RESULT,
                (uint8_t*)&u32Temp, 4);
            break;
        default:
            errCode = data_packet_create(pkt, CONTROLLER_NACK, 0, 0);
            break;
        }
        break;
    case SET_RAM_VARIABLE:
        if(command_set_ram(pkt->Data) == DATA_COMMAND_SUCCESS) {
        	errCode = data_packet_create(pkt, CONTROLLER_ACK, 0, 0);
        } else {
        	errCode = data_packet_create(pkt, CONTROLLER_NACK, 0, 0);
        }
        break;
    case GET_EEPROM_VARIABLE:
        errCode = command_get_eeprom(pkt->Data, &u32Temp);
        switch(errCode) {
        case RESULT_IS_8B:
            errCode = data_packet_create(pkt, GET_RAM_RESULT,
                (uint8_t*)&u32Temp, 1);
            break;
        case RESULT_IS_16B:
            errCode = data_packet_create(pkt, GET_RAM_RESULT,
                (uint8_t*)&u32Temp, 2);
            break;
        case RESULT_IS_32B:
        case RESULT_IS_FLOAT:
            errCode = data_packet_create(pkt, GET_RAM_RESULT,
                (uint8_t*)&u32Temp, 4);
            break;
        default:
            errCode = data_packet_create(pkt, CONTROLLER_NACK, 0, 0);
            break;
        }
        break;
    case SET_EEPROM_VARIABLE:
        if(command_set_eeprom(pkt->Data) == DATA_COMMAND_SUCCESS) {
        	errCode = data_packet_create(pkt, CONTROLLER_ACK, 0, 0);
        } else {
        	errCode = data_packet_create(pkt, CONTROLLER_NACK, 0, 0);
        }
        break;
    case ENABLE_FEATURE:
        if(command_enable_feature(pkt->Data) == DATA_COMMAND_SUCCESS) {
        	errCode = data_packet_create(pkt, CONTROLLER_ACK, 0, 0);
        } else {
        	errCode = data_packet_create(pkt, CONTROLLER_NACK, 0, 0);
        }
        break;
    case DISABLE_FEATURE:
        if(command_disable_feature(pkt->Data) == DATA_COMMAND_SUCCESS) {
        	errCode = data_packet_create(pkt, CONTROLLER_ACK, 0, 0);
        } else {
        	errCode = data_packet_create(pkt, CONTROLLER_NACK, 0, 0);
        }
        break;
    case RUN_ROUTINE:
        if(command_run_routine(pkt->Data) == DATA_COMMAND_SUCCESS) {
        	errCode = data_packet_create(pkt, CONTROLLER_ACK, 0, 0);
        } else {
        	errCode = data_packet_create(pkt, CONTROLLER_NACK, 0, 0);
        }
        break;
    case HOST_STREAM_DATA:
        break;
    case HOST_ACK:
        break;
    case HOST_NACK:
        break;

        // Responses from a lower-level controller (e.g. BMS):
    case GET_RAM_RESULT:
        break;
    case GET_EEPROM_RESULT:
        break;
    case ROUTINE_RESULT:
        break;
    case CONTROLLER_STREAM_DATA:
        break;
    case CONTROLLER_ACK:
        break;
    case CONTROLLER_NACK:
        break;
    
    default:
        break;
    }
    pkt->RxReady = 0;
    return errCode;
}

uint16_t command_get_ram(uint8_t* pktdata, void* retval) {
    // Data is two bytes for value ID
    uint16_t value_ID = (((uint16_t)pktdata[0]) >> 8) + ((uint16_t)pktdata[1]);
    float retvalf;
    uint8_t retval8b;
    uint16_t retval16b;
    uint32_t retval32b;
    uint16_t errCode = DATA_COMMAND_FAIL;

    float* tempfptr;

    switch(value_ID) {
    case CONFIG_DATA_SPEED:
        retval8b = MAIN_GetUSBDebugSpeed();
        *(uint8_t*)retval = retval8b;
        errCode = RESULT_IS_8B;
        break;
    case CONFIG_DATA_NUMVARS:
        retval8b = MAIN_GetNumUSBDebugOutputs();
        *(uint8_t*)retval = retval8b;
        errCode = RESULT_IS_8B;
        break;
    case CONFIG_DATA_VAR1:
        retval8b = MAIN_GetUSBDebugOutput(0);
        *(uint8_t*)retval = retval8b;
        errCode = RESULT_IS_8B;
        break;
    case CONFIG_DATA_VAR2:
        retval8b = MAIN_GetUSBDebugOutput(1);
        *(uint8_t*)retval = retval8b;
        errCode = RESULT_IS_8B;
        break;
    case CONFIG_DATA_VAR3:
        retval8b = MAIN_GetUSBDebugOutput(2);
        *(uint8_t*)retval = retval8b;
        errCode = RESULT_IS_8B;
        break;
    case CONFIG_DATA_VAR4:
        retval8b = MAIN_GetUSBDebugOutput(3);
        *(uint8_t*)retval = retval8b;
        errCode = RESULT_IS_8B;
        break;
    case CONFIG_DATA_VAR5:
        retval8b = MAIN_GetUSBDebugOutput(44);
        *(uint8_t*)retval = retval8b;
        errCode = RESULT_IS_8B;
        break;
    case CONFIG_DATA_VAR6:
        retval8b = MAIN_GetUSBDebugOutput(5);
        *(uint8_t*)retval = retval8b;
        errCode = RESULT_IS_8B;
        break;
    case CONFIG_DATA_VAR7:
        retval8b = MAIN_GetUSBDebugOutput(6);
        *(uint8_t*)retval = retval8b;
        errCode = RESULT_IS_8B;
        break;
    case CONFIG_DATA_VAR8:
        retval8b = MAIN_GetUSBDebugOutput(7);
        *(uint8_t*)retval = retval8b;
        errCode = RESULT_IS_8B;
        break;
    case CONFIG_DATA_VAR9:
        retval8b = MAIN_GetUSBDebugOutput(8);
        *(uint8_t*)retval = retval8b;
        errCode = RESULT_IS_8B;
        break;
    case CONFIG_DATA_VAR10:
        retval8b = MAIN_GetUSBDebugOutput(9);
        *(uint8_t*)retval = retval8b;
        errCode = RESULT_IS_8B;
        break;
    case CONFIG_FOC_KP:
        retvalf = MAIN_GetVar(0);
        *(float*)retval = retvalf;
        errCode = RESULT_IS_FLOAT;
        break;
    case CONFIG_FOC_KI:
        retvalf = MAIN_GetVar(1);
        *(float*)retval = retvalf;
        errCode = RESULT_IS_FLOAT;
        break;
    case CONFIG_FOC_KD:
        retvalf = MAIN_GetVar(2);
        *(float*)retval = retvalf;
        errCode = RESULT_IS_FLOAT;
        break;
    case CONFIG_FOC_KC:
        retvalf = MAIN_GetVar(3);
        *(float*)retval = retvalf;
        errCode = RESULT_IS_FLOAT;
        break;
    case CONFIG_FOC_DT:
        retval32b = PWM_GetDeadTime();
        *(uint32_t*)retval = retval32b;
        errCode = RESULT_IS_32B;
        break;
    // case CONFIG_FOC_FREQ:
        // break;
    // case CONFIG_MOTOR_PP:
        // break;
    // case CONFIG_MOTOR_RS:
        // break;
    // case CONFIG_MOTOR_LS:
        // break;
    // case CONFIG_MOTOR_FLUX:
        // break;
    case CONFIG_MOTOR_HALL1:
        tempfptr = HallSensor_GetAngleTable();
        retvalf = tempfptr[1];
        *(float*)retval = retvalf;
        errCode = RESULT_IS_FLOAT;
        break;
    case CONFIG_MOTOR_HALL2:
        tempfptr = HallSensor_GetAngleTable();
        retvalf = tempfptr[2];
        *(float*)retval = retvalf;
        errCode = RESULT_IS_FLOAT;
        break;
    case CONFIG_MOTOR_HALL3:
        tempfptr = HallSensor_GetAngleTable();
        retvalf = tempfptr[3];
        *(float*)retval = retvalf;
        errCode = RESULT_IS_FLOAT;
        break;
    case CONFIG_MOTOR_HALL4:
        tempfptr = HallSensor_GetAngleTable();
        retvalf = tempfptr[4];
        *(float*)retval = retvalf;
        errCode = RESULT_IS_FLOAT;
        break;
    case CONFIG_MOTOR_HALL5:
        tempfptr = HallSensor_GetAngleTable();
        retvalf = tempfptr[5];
        *(float*)retval = retvalf;
        errCode = RESULT_IS_FLOAT;
        break;
    case CONFIG_MOTOR_HALL6:
        tempfptr = HallSensor_GetAngleTable();
        retvalf = tempfptr[6];
        *(float*)retval = retvalf;
        errCode = RESULT_IS_FLOAT;
        break;
    case CONFIG_THR_TYPE1:
        retval8b = throttle_get_type(1);
        *(uint8_t*)retval = retval8b;
        errCode = RESULT_IS_8B;
        break;
    case CONFIG_THR_MIN1:
        retvalf = throttle_get_min(1);
        *(float*)retval = retvalf;
        errCode = RESULT_IS_FLOAT;
        break;
    case CONFIG_THR_MAX1:
        retvalf = throttle_get_min(1);
        *(float*)retval = retvalf;
        errCode = RESULT_IS_FLOAT;
        break;
    case CONFIG_THR_HYST1:
        retvalf = throttle_get_hyst(1);
        *(float*)retval = retvalf;
        errCode = RESULT_IS_FLOAT;
        break;
    case CONFIG_THR_FILT1:
        retvalf = throttle_get_filt(1);
        *(float*)retval = retvalf;
        errCode = RESULT_IS_FLOAT;
        break;
    case CONFIG_THR_RISE1:
        retvalf = throttle_get_rise(1);
        *(float*)retval = retvalf;
        errCode = RESULT_IS_FLOAT;
        break;
    case CONFIG_THR_TYPE2:
        retval8b = throttle_get_type(2);
        *(uint8_t*)retval = retval8b;
        errCode = RESULT_IS_8B;
        break;
    case CONFIG_THR_MIN2:
        retvalf = throttle_get_min(2);
        *(float*)retval = retvalf;
        errCode = RESULT_IS_FLOAT;
        break;
    case CONFIG_THR_MAX2:
        retvalf = throttle_get_min(2);
        *(float*)retval = retvalf;
        errCode = RESULT_IS_FLOAT;
        break;
    case CONFIG_THR_HYST2:
        retvalf = throttle_get_hyst(2);
        *(float*)retval = retvalf;
        errCode = RESULT_IS_FLOAT;
        break;
    case CONFIG_THR_FILT2:
        retvalf = throttle_get_filt(2);
        *(float*)retval = retvalf;
        errCode = RESULT_IS_FLOAT;
        break;
    case CONFIG_THR_RISE2:
        retvalf = throttle_get_rise(2);
        *(float*)retval = retvalf;
        errCode = RESULT_IS_FLOAT;
        break;
    }
    return errCode;
}

uint16_t command_set_ram(uint8_t* pktdata) {
    // Data is two bytes for value ID
    uint16_t value_ID = (((uint16_t)pktdata[0]) >> 8) + ((uint16_t)pktdata[1]);
    // Then four bytes for value
    float valuef;
    uint8_t value8b;
    uint16_t value16b;
    uint32_t value32b;
    uint16_t errCode = DATA_COMMAND_FAIL;
    switch(value_ID) {
    case CONFIG_DATA_SPEED:
        break;
    case CONFIG_DATA_NUMVARS:
        break;
    case CONFIG_DATA_VAR1:
        break;
    case CONFIG_DATA_VAR2:
        break;
    case CONFIG_DATA_VAR3:
        break;
    case CONFIG_DATA_VAR4:
        break;
    case CONFIG_DATA_VAR5:
        break;
    case CONFIG_DATA_VAR6:
        break;
    case CONFIG_DATA_VAR7:
        break;
    case CONFIG_DATA_VAR8:
        break;
    case CONFIG_DATA_VAR9:
        break;
    case CONFIG_DATA_VAR10:
        break;
    case CONFIG_FOC_KP:
        break;
    case CONFIG_FOC_KI:
        break;
    case CONFIG_FOC_KD:
        break;
    case CONFIG_FOC_KC:
        break;
    case CONFIG_FOC_DT:
        break;
    // case CONFIG_FOC_FREQ:
        // break;
//    case CONFIG_MOTOR_PP:
//        break;
    // case CONFIG_MOTOR_RS:
        // break;
    // case CONFIG_MOTOR_LS:
        // break;
    // case CONFIG_MOTOR_FLUX:
        // break;
    case CONFIG_MOTOR_HALL1:
        break;
    case CONFIG_MOTOR_HALL2:
        break;
    case CONFIG_MOTOR_HALL3:
        break;
    case CONFIG_MOTOR_HALL4:
        break;
    case CONFIG_MOTOR_HALL5:
        break;
    case CONFIG_MOTOR_HALL6:
        break;
    case CONFIG_THR_TYPE1:
        break;
    case CONFIG_THR_MIN1:
        break;
    case CONFIG_THR_MAX1:
        break;
    case CONFIG_THR_HYST1:
        break;
    case CONFIG_THR_FILT1:
        break;
    case CONFIG_THR_RISE1:
        break;
    case CONFIG_THR_TYPE2:
        break;
    case CONFIG_THR_MIN2:
        break;
    case CONFIG_THR_MAX2:
        break;
    case CONFIG_THR_HYST2:
        break;
    case CONFIG_THR_FILT2:
        break;
    case CONFIG_THR_RISE2:
        break;
    }
    return errCode;
}

uint16_t command_get_eeprom(uint8_t* pktdata, void* retval) {
	return DATA_COMMAND_FAIL;
}

uint16_t command_set_eeprom(uint8_t* pktdata) {
	return DATA_COMMAND_FAIL;
}

uint16_t command_enable_feature(uint8_t* pktdata) {
	return DATA_COMMAND_FAIL;
}

uint16_t command_disable_feature(uint8_t* pktdata) {
	return DATA_COMMAND_FAIL;
}

uint16_t command_run_routine(uint8_t* pktdata) {
	return DATA_COMMAND_FAIL;
}
