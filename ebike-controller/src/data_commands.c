/******************************************************************************
 * Filename: data_commands.c
 * Description: Forwards the commands that were properly decoded from a
 *              communication channel packet to the correct function.
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

/**
 * @brief  Data Process Command
 *              Interprets the command in a decoded packet. Calls the appropriate
 *              sub-function for the requested command.
 * @param  pkt - Data_Packet_Type pointer with the decoded communication data
 * @retval DATA_COMMAND_FAIL - Unable to process the packet
 *         DATA_COMMAND_SUCCESS - Packet was processed. Check the TxReady flag
 *                                 to see if an outgoing packet was generated.
 */
uint16_t data_process_command(Data_Packet_Type* pkt) {

    uint8_t retval[4];
    uint16_t errCode = DATA_COMMAND_FAIL;

    if (!pkt->RxReady) {
        return DATA_COMMAND_FAIL;
    }
    switch (pkt->PacketType) {
    // Responses from the host
    case GET_RAM_VARIABLE:
        errCode = command_get_ram(pkt->Data, retval);
        switch (errCode) {
        case RESULT_IS_8B:
            errCode = data_packet_create(pkt, GET_RAM_RESULT, retval, 1);
            break;
        case RESULT_IS_16B:
            errCode = data_packet_create(pkt, GET_RAM_RESULT, retval, 2);
            break;
        case RESULT_IS_32B:
        case RESULT_IS_FLOAT:
            errCode = data_packet_create(pkt, GET_RAM_RESULT, retval, 4);
            break;
        default:
            errCode = data_packet_create(pkt, CONTROLLER_NACK, 0, 0);
            break;
        }
        break;
    case SET_RAM_VARIABLE:
        if (command_set_ram(pkt->Data) == DATA_COMMAND_SUCCESS) {
            errCode = data_packet_create(pkt, CONTROLLER_ACK, 0, 0);
        } else {
            errCode = data_packet_create(pkt, CONTROLLER_NACK, 0, 0);
        }
        break;
    case GET_EEPROM_VARIABLE:
        errCode = command_get_eeprom(pkt->Data, retval);
        switch (errCode) {
        case RESULT_IS_8B:
            errCode = data_packet_create(pkt, GET_RAM_RESULT, retval, 1);
            break;
        case RESULT_IS_16B:
            errCode = data_packet_create(pkt, GET_RAM_RESULT, retval, 2);
            break;
        case RESULT_IS_32B:
        case RESULT_IS_FLOAT:
            errCode = data_packet_create(pkt, GET_RAM_RESULT, retval, 4);
            break;
        default:
            errCode = data_packet_create(pkt, CONTROLLER_NACK, 0, 0);
            break;
        }
        break;
    case SET_EEPROM_VARIABLE:
        if (command_set_eeprom(pkt->Data) == DATA_COMMAND_SUCCESS) {
            errCode = data_packet_create(pkt, CONTROLLER_ACK, 0, 0);
        } else {
            errCode = data_packet_create(pkt, CONTROLLER_NACK, 0, 0);
        }
        break;
    case ENABLE_FEATURE:
        if (command_enable_feature(pkt->Data) == DATA_COMMAND_SUCCESS) {
            errCode = data_packet_create(pkt, CONTROLLER_ACK, 0, 0);
        } else {
            errCode = data_packet_create(pkt, CONTROLLER_NACK, 0, 0);
        }
        break;
    case DISABLE_FEATURE:
        if (command_disable_feature(pkt->Data) == DATA_COMMAND_SUCCESS) {
            errCode = data_packet_create(pkt, CONTROLLER_ACK, 0, 0);
        } else {
            errCode = data_packet_create(pkt, CONTROLLER_NACK, 0, 0);
        }
        break;
    case RUN_ROUTINE:
        if (command_run_routine(pkt->Data) == DATA_COMMAND_SUCCESS) {
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

/**
 * @brief  Data Command: Get Ram
 *            Interprets the command in a decoded packet. Calls the appropriate
 *            sub-function for the requested command.
 * @param  pktdata - Data field in the incoming packet
 * @param  retval - Pointer to return value from the command request.
 *                  Regardless of the return type, it will be placed into the
 *                  location pointed to by retval. Data can be 8 to 32 bit
 *                  (1 to 4 bytes).
 * @retval DATA_COMMAND_FAIL - Unable to process the data
 *            RESULT_IS_8B - The return value is an 8-bit integer
 *            RESULT_IS_16B - The return value is an 16-bit integer
 *            RESULT_IS_32B - The return value is an 32-bit integer
 *            RESULT_IS_FLOAT - The return value is an 32-bit floating point
 */
uint16_t command_get_ram(uint8_t* pktdata, uint8_t* retval) {
    // Data is two bytes for value ID
    uint16_t value_ID = data_packet_extract_16b(pktdata);
//    uint16_t value_ID = (((uint16_t)pktdata[0]) << 8) + pktdata[1];
    float retvalf;
    uint8_t retval8b;
    uint16_t retval16b;
    uint32_t retval32b;
    uint16_t errCode = DATA_COMMAND_FAIL;

    ((void) retval16b);

    float* fhalltableptr;

    switch (value_ID) {
    case CONFIG_MAIN_RAMP_SPEED:
        retvalf = MAIN_GetRampSpeed();
        data_packet_pack_float(retval, retvalf);
        break;
    case CONFIG_MAIN_USB_SPEED:
        retval8b = MAIN_GetUSBDebugSpeed();
        data_packet_pack_8b(retval, retval8b);
        errCode = RESULT_IS_8B;
        break;
    case CONFIG_MAIN_NUM_USB_OUTPUTS:
        retval8b = MAIN_GetNumUSBDebugOutputs();
        data_packet_pack_8b(retval, retval8b);
        errCode = RESULT_IS_8B;
        break;
    case CONFIG_MAIN_USB_CHOICE_1:
        retval8b = MAIN_GetUSBDebugOutput(0);
        data_packet_pack_8b(retval, retval8b);
        errCode = RESULT_IS_8B;
        break;
    case CONFIG_MAIN_USB_CHOICE_2:
        retval8b = MAIN_GetUSBDebugOutput(1);
        data_packet_pack_8b(retval, retval8b);
        errCode = RESULT_IS_8B;
        break;
    case CONFIG_MAIN_USB_CHOICE_3:
        retval8b = MAIN_GetUSBDebugOutput(2);
        data_packet_pack_8b(retval, retval8b);
        errCode = RESULT_IS_8B;
        break;
    case CONFIG_MAIN_USB_CHOICE_4:
        retval8b = MAIN_GetUSBDebugOutput(3);
        data_packet_pack_8b(retval, retval8b);
        errCode = RESULT_IS_8B;
        break;
    case CONFIG_MAIN_USB_CHOICE_5:
        retval8b = MAIN_GetUSBDebugOutput(4);
        data_packet_pack_8b(retval, retval8b);
        errCode = RESULT_IS_8B;
        break;
    case CONFIG_MAIN_USB_CHOICE_6:
        retval8b = MAIN_GetUSBDebugOutput(5);
        data_packet_pack_8b(retval, retval8b);
        errCode = RESULT_IS_8B;
        break;
    case CONFIG_MAIN_USB_CHOICE_7:
        retval8b = MAIN_GetUSBDebugOutput(6);
        data_packet_pack_8b(retval, retval8b);
        errCode = RESULT_IS_8B;
        break;
    case CONFIG_MAIN_USB_CHOICE_8:
        retval8b = MAIN_GetUSBDebugOutput(7);
        data_packet_pack_8b(retval, retval8b);
        errCode = RESULT_IS_8B;
        break;
    case CONFIG_MAIN_USB_CHOICE_9:
        retval8b = MAIN_GetUSBDebugOutput(8);
        data_packet_pack_8b(retval, retval8b);
        errCode = RESULT_IS_8B;
        break;
    case CONFIG_MAIN_USB_CHOICE_10:
        retval8b = MAIN_GetUSBDebugOutput(9);
        data_packet_pack_8b(retval, retval8b);
        errCode = RESULT_IS_8B;
        break;
    case CONFIG_FOC_KP:
        retvalf = MAIN_GetVar(0);
        data_packet_pack_float(retval, retvalf);
        errCode = RESULT_IS_FLOAT;
        break;
    case CONFIG_FOC_KI:
        retvalf = MAIN_GetVar(1);
        data_packet_pack_float(retval, retvalf);
        errCode = RESULT_IS_FLOAT;
        break;
    case CONFIG_FOC_KD:
        retvalf = MAIN_GetVar(2);
        data_packet_pack_float(retval, retvalf);
        errCode = RESULT_IS_FLOAT;
        break;
    case CONFIG_FOC_KC:
        retvalf = MAIN_GetVar(3);
        data_packet_pack_float(retval, retvalf);
        errCode = RESULT_IS_FLOAT;
        break;
    case CONFIG_FOC_PWM_FREQ:
        retval32b = PWM_GetFreq();
        data_packet_pack_32b(retval, retval32b);
        errCode = RESULT_IS_32B;
        break;
    case CONFIG_FOC_PWM_DEADTIME:
        retval32b = PWM_GetDeadTime();
        data_packet_pack_32b(retval, retval32b);
        errCode = RESULT_IS_32B;
        break;
        // case CONFIG_MOTOR_PP:
        // break;
        // case CONFIG_MOTOR_RS:
        // break;
        // case CONFIG_MOTOR_LS:
        // break;
        // case CONFIG_MOTOR_FLUX:
        // break;
    case CONFIG_MOTOR_HALL1:
        fhalltableptr = HallSensor_GetAngleTable();
        retvalf = fhalltableptr[1];
        data_packet_pack_float(retval, retvalf);
        errCode = RESULT_IS_FLOAT;
        break;
    case CONFIG_MOTOR_HALL2:
        fhalltableptr = HallSensor_GetAngleTable();
        retvalf = fhalltableptr[2];
        data_packet_pack_float(retval, retvalf);
        errCode = RESULT_IS_FLOAT;
        break;
    case CONFIG_MOTOR_HALL3:
        fhalltableptr = HallSensor_GetAngleTable();
        retvalf = fhalltableptr[3];
        data_packet_pack_float(retval, retvalf);
        errCode = RESULT_IS_FLOAT;
        break;
    case CONFIG_MOTOR_HALL4:
        fhalltableptr = HallSensor_GetAngleTable();
        retvalf = fhalltableptr[4];
        data_packet_pack_float(retval, retvalf);
        errCode = RESULT_IS_FLOAT;
        break;
    case CONFIG_MOTOR_HALL5:
        fhalltableptr = HallSensor_GetAngleTable();
        retvalf = fhalltableptr[5];
        data_packet_pack_float(retval, retvalf);
        errCode = RESULT_IS_FLOAT;
        break;
    case CONFIG_MOTOR_HALL6:
        fhalltableptr = HallSensor_GetAngleTable();
        retvalf = fhalltableptr[6];
        data_packet_pack_float(retval, retvalf);
        errCode = RESULT_IS_FLOAT;
        break;
    case CONFIG_THRT_TYPE1:
        retval8b = throttle_get_type(1);
        data_packet_pack_8b(retval, retval8b);
        errCode = RESULT_IS_8B;
        break;
    case CONFIG_THRT_MIN1:
        retvalf = throttle_get_min(1);
        data_packet_pack_float(retval, retvalf);
        errCode = RESULT_IS_FLOAT;
        break;
    case CONFIG_THRT_MAX1:
        retvalf = throttle_get_min(1);
        data_packet_pack_float(retval, retvalf);
        errCode = RESULT_IS_FLOAT;
        break;
    case CONFIG_THRT_HYST1:
        retvalf = throttle_get_hyst(1);
        data_packet_pack_float(retval, retvalf);
        errCode = RESULT_IS_FLOAT;
        break;
    case CONFIG_THRT_FILT1:
        retvalf = throttle_get_filt(1);
        data_packet_pack_float(retval, retvalf);
        errCode = RESULT_IS_FLOAT;
        break;
    case CONFIG_THRT_RISE1:
        retvalf = throttle_get_rise(1);
        data_packet_pack_float(retval, retvalf);
        errCode = RESULT_IS_FLOAT;
        break;
    case CONFIG_THRT_TYPE2:
        retval8b = throttle_get_type(2);
        data_packet_pack_8b(retval, retval8b);
        errCode = RESULT_IS_8B;
        break;
    case CONFIG_THRT_MIN2:
        retvalf = throttle_get_min(2);
        data_packet_pack_float(retval, retvalf);
        errCode = RESULT_IS_FLOAT;
        break;
    case CONFIG_THRT_MAX2:
        retvalf = throttle_get_min(2);
        data_packet_pack_float(retval, retvalf);
        errCode = RESULT_IS_FLOAT;
        break;
    case CONFIG_THRT_HYST2:
        retvalf = throttle_get_hyst(2);
        data_packet_pack_float(retval, retvalf);
        errCode = RESULT_IS_FLOAT;
        break;
    case CONFIG_THRT_FILT2:
        retvalf = throttle_get_filt(2);
        data_packet_pack_float(retval, retvalf);
        errCode = RESULT_IS_FLOAT;
        break;
    case CONFIG_THRT_RISE2:
        retvalf = throttle_get_rise(2);
        data_packet_pack_float(retval, retvalf);
        errCode = RESULT_IS_FLOAT;
        break;
    }
    return errCode;
}

uint16_t command_set_ram(uint8_t* pktdata) {
    // Data is two bytes for value ID
    uint16_t value_ID = data_packet_extract_16b(pktdata);
    //uint16_t value_ID = (((uint16_t)pktdata[0]) << 8) + pktdata[1];
    pktdata += 2;
    // Then one to four bytes for value, depending on command
    float valuef;
    uint8_t value8b;
    uint16_t value16b;
    uint32_t value32b;
    uint16_t errCode = DATA_COMMAND_FAIL;

    ((void) value16b);

    switch (value_ID) {
    case CONFIG_MAIN_RAMP_SPEED:
        valuef = data_packet_extract_float(pktdata);
        errCode = MAIN_SetRampSpeed(valuef);
        break;
    case CONFIG_MAIN_USB_SPEED:
        value8b = data_packet_extract_8b(pktdata);
        errCode = MAIN_SetUSBDebugSpeed(value8b);
        break;
    case CONFIG_MAIN_NUM_USB_OUTPUTS:
        value8b = data_packet_extract_8b(pktdata);
        errCode = MAIN_SetNumUSBDebugOutputs(value8b);
        break;
    case CONFIG_MAIN_USB_CHOICE_1:
        value8b = data_packet_extract_8b(pktdata);
        errCode = MAIN_SetUSBDebugOutput(0, value8b);
        break;
    case CONFIG_MAIN_USB_CHOICE_2:
        value8b = data_packet_extract_8b(pktdata);
        errCode = MAIN_SetUSBDebugOutput(1, value8b);
        break;
    case CONFIG_MAIN_USB_CHOICE_3:
        value8b = data_packet_extract_8b(pktdata);
        errCode = MAIN_SetUSBDebugOutput(2, value8b);
        break;
    case CONFIG_MAIN_USB_CHOICE_4:
        value8b = data_packet_extract_8b(pktdata);
        errCode = MAIN_SetUSBDebugOutput(3, value8b);
        break;
    case CONFIG_MAIN_USB_CHOICE_5:
        value8b = data_packet_extract_8b(pktdata);
        errCode = MAIN_SetUSBDebugOutput(4, value8b);
        break;
    case CONFIG_MAIN_USB_CHOICE_6:
        value8b = data_packet_extract_8b(pktdata);
        errCode = MAIN_SetUSBDebugOutput(5, value8b);
        break;
    case CONFIG_MAIN_USB_CHOICE_7:
        value8b = data_packet_extract_8b(pktdata);
        errCode = MAIN_SetUSBDebugOutput(6, value8b);
        break;
    case CONFIG_MAIN_USB_CHOICE_8:
        value8b = data_packet_extract_8b(pktdata);
        errCode = MAIN_SetUSBDebugOutput(7, value8b);
        break;
    case CONFIG_MAIN_USB_CHOICE_9:
        value8b = data_packet_extract_8b(pktdata);
        errCode = MAIN_SetUSBDebugOutput(8, value8b);
        break;
    case CONFIG_MAIN_USB_CHOICE_10:
        value8b = data_packet_extract_8b(pktdata);
        errCode = MAIN_SetUSBDebugOutput(9, value8b);
        break;
    case CONFIG_FOC_KP:
        valuef = data_packet_extract_float(pktdata);
        errCode = MAIN_SetVar(0, valuef);
        break;
    case CONFIG_FOC_KI:
        valuef = data_packet_extract_float(pktdata);
        errCode = MAIN_SetVar(1, valuef);
        break;
    case CONFIG_FOC_KD:
        valuef = data_packet_extract_float(pktdata);
        errCode = MAIN_SetVar(2, valuef);
        break;
    case CONFIG_FOC_KC:
        valuef = data_packet_extract_float(pktdata);
        errCode = MAIN_SetVar(3, valuef);
        break;
    case CONFIG_FOC_PWM_FREQ:
        value32b = data_packet_extract_32b(pktdata);
        errCode = MAIN_SetFreq(value32b);
        break;
    case CONFIG_FOC_PWM_DEADTIME:
        value32b = data_packet_extract_32b(pktdata);
        errCode = MAIN_SetDeadTime(value32b);
        break;
//    case CONFIG_MOTOR_PP:
//        break;
        // case CONFIG_MOTOR_RS:
        // break;
        // case CONFIG_MOTOR_LS:
        // break;
        // case CONFIG_MOTOR_FLUX:
        // break;
    case CONFIG_MOTOR_HALL1:
        valuef = data_packet_extract_float(pktdata);
        errCode = HallSensor_SetAngle(1, valuef);
        break;
    case CONFIG_MOTOR_HALL2:
        valuef = data_packet_extract_float(pktdata);
        errCode = HallSensor_SetAngle(2, valuef);
        break;
    case CONFIG_MOTOR_HALL3:
        valuef = data_packet_extract_float(pktdata);
        errCode = HallSensor_SetAngle(3, valuef);
        break;
    case CONFIG_MOTOR_HALL4:
        valuef = data_packet_extract_float(pktdata);
        errCode = HallSensor_SetAngle(4, valuef);
        break;
    case CONFIG_MOTOR_HALL5:
        valuef = data_packet_extract_float(pktdata);
        errCode = HallSensor_SetAngle(5, valuef);
        break;
    case CONFIG_MOTOR_HALL6:
        valuef = data_packet_extract_float(pktdata);
        errCode = HallSensor_SetAngle(6, valuef);
        break;
    case CONFIG_THRT_TYPE1:
        break;
    case CONFIG_THRT_MIN1:
        break;
    case CONFIG_THRT_MAX1:
        break;
    case CONFIG_THRT_HYST1:
        break;
    case CONFIG_THRT_FILT1:
        break;
    case CONFIG_THRT_RISE1:
        break;
    case CONFIG_THRT_TYPE2:
        break;
    case CONFIG_THRT_MIN2:
        break;
    case CONFIG_THRT_MAX2:
        break;
    case CONFIG_THRT_HYST2:
        break;
    case CONFIG_THRT_FILT2:
        break;
    case CONFIG_THRT_RISE2:
        break;
    }
    return errCode;
}

uint16_t command_get_eeprom(uint8_t* pktdata, uint8_t* retval) {
    ((void) pktdata);
    ((void) retval);
    return DATA_COMMAND_FAIL;
}

uint16_t command_set_eeprom(uint8_t* pktdata) {
    ((void) pktdata);
    return DATA_COMMAND_FAIL;
}

uint16_t command_enable_feature(uint8_t* pktdata) {
    // Data is two bytes for feature ID
    uint16_t feature_ID = data_packet_extract_16b(pktdata);
    uint16_t errCode = DATA_COMMAND_FAIL;

    switch (feature_ID) {
    case FEATURE_SERIAL_DATA:
        MAIN_SetUSBDebugging(1);
        errCode = DATA_COMMAND_SUCCESS;
        break;
    default:
        errCode = DATA_COMMAND_FAIL;
        break;
    }
    return errCode;
}

uint16_t command_disable_feature(uint8_t* pktdata) {
    // Data is two bytes for feature ID
    uint16_t feature_ID = data_packet_extract_16b(pktdata);
    uint16_t errCode = DATA_COMMAND_FAIL;

    switch (feature_ID) {
    case FEATURE_SERIAL_DATA:
        MAIN_SetUSBDebugging(0);
        errCode = DATA_COMMAND_SUCCESS;
        break;
    default:
        errCode = DATA_COMMAND_FAIL;
        break;
    }
    return errCode;
}

uint16_t command_run_routine(uint8_t* pktdata) {
    // Routine ID is two bytes of packet data
    uint16_t routine_ID = data_packet_extract_16b(pktdata);
    pktdata += 2;
    uint16_t errCode = DATA_COMMAND_FAIL;

    float valuef;

    switch(routine_ID) {
    case ROUTINE_HALL_DETECT:
        // Single variable float is applied current
        valuef = data_packet_extract_float(pktdata);
        MAIN_DetectHallPositions(valuef);
        errCode = DATA_COMMAND_SUCCESS;
        break;
    }

    ((void) pktdata);
    return errCode;
}
