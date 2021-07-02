/******************************************************************************
 * Filename: data_commands.c
 * Description: Forwards the commands that were properly decoded from a
 *              communication channel packet to the correct function.
 *
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
#include "ram_commands.h"

static Data_Type command_get_datatype(uint16_t data_ID);
/**
 * @brief  Data Process Command
 *              Interprets the command in a decoded packet. Calls the appropriate
 *              sub-function for the requested command.
 * @param  pkt - Data_Packet_Type pointer with the decoded communication data
 * @retval RETVAL_FAIL - Unable to process the packet
 *         RETVAL_OK - Packet was processed. Check the TxReady flag
 *                                 to see if an outgoing packet was generated.
 */
uint16_t data_process_command(Data_Packet_Type* pkt) {

    uint8_t retval[4];
    uint16_t errCode = RETVAL_FAIL;

    if (!pkt->RxReady) {
        return RETVAL_FAIL;
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
        if (command_set_ram(pkt->Data) == RETVAL_OK) {
            errCode = data_packet_create(pkt, CONTROLLER_ACK, 0, 0);
        } else {
            errCode = data_packet_create(pkt, CONTROLLER_NACK, 0, 0);
        }
        break;
    case GET_EEPROM_VARIABLE:
        errCode = command_get_eeprom(pkt->Data, retval);
        switch (errCode) {
        case RESULT_IS_8B:
            errCode = data_packet_create(pkt, GET_EEPROM_RESULT, retval, 1);
            break;
        case RESULT_IS_16B:
            errCode = data_packet_create(pkt, GET_EEPROM_RESULT, retval, 2);
            break;
        case RESULT_IS_32B:
        case RESULT_IS_FLOAT:
            errCode = data_packet_create(pkt, GET_EEPROM_RESULT, retval, 4);
            break;
        default:
            errCode = data_packet_create(pkt, CONTROLLER_NACK, 0, 0);
            break;
        }
        break;
    case SET_EEPROM_VARIABLE:
        if (command_set_eeprom(pkt->Data) == RETVAL_OK) {
            errCode = data_packet_create(pkt, CONTROLLER_ACK, 0, 0);
        } else {
            errCode = data_packet_create(pkt, CONTROLLER_NACK, 0, 0);
        }
        break;
    case ENABLE_FEATURE:
        if (command_enable_feature(pkt->Data) == RETVAL_OK) {
            errCode = data_packet_create(pkt, CONTROLLER_ACK, 0, 0);
        } else {
            errCode = data_packet_create(pkt, CONTROLLER_NACK, 0, 0);
        }
        break;
    case DISABLE_FEATURE:
        if (command_disable_feature(pkt->Data) == RETVAL_OK) {
            errCode = data_packet_create(pkt, CONTROLLER_ACK, 0, 0);
        } else {
            errCode = data_packet_create(pkt, CONTROLLER_NACK, 0, 0);
        }
        break;
    case RUN_ROUTINE:
        if (command_run_routine(pkt->Data) == RETVAL_OK) {
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
    case REQUEST_DASHBOARD_DATA:
        // Use the same memory locations, data field is ignored anyway
        if( MAIN_GetDashboardData(pkt->Data) == RETVAL_OK) {
            errCode = data_packet_create(pkt, DASHBOARD_DATA_RESULT, pkt->Data, DASHBOARD_DATA_LENGTH);
        }
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
 * @retval RETVAL_FAIL - Unable to process the data
 *            RESULT_IS_8B - The return value is an 8-bit integer
 *            RESULT_IS_16B - The return value is an 16-bit integer
 *            RESULT_IS_32B - The return value is an 32-bit integer
 *            RESULT_IS_FLOAT - The return value is an 32-bit floating point
 */
uint16_t command_get_ram(uint8_t* pktdata, uint8_t* retval) {
    // Data is two bytes for value ID
    uint16_t value_ID = data_packet_extract_16b(pktdata);
//    uint16_t value_ID = (((uint16_t)pktdata[0]) << 8) + pktdata[1];
    float retvalf= 0.0f;
    uint8_t retval8b = 0;
    uint16_t retval16b = 0;
    uint32_t retval32b = 0;
    uint16_t errCode = RETVAL_FAIL;

    switch (value_ID) {

    // 8-bit integer values
    case CONFIG_DRV_VDS_LIMIT:
        retval8b = DRV8353_GetVDSLimit();
        break;
    case CONFIG_DRV_CSA_GAIN:
        retval8b = DRV8353_GetGain();
        break;
    case CONFIG_BMS_ISCONNECTED:
        retval8b = 0xAAu;
        break;

    // 16-bit integer values
    // Not yet implemented
    case CONFIG_MAIN_NUM_USB_OUTPUTS:
        retval16b = LIVE_GetNumOutputs();
        break;
    case CONFIG_MAIN_USB_SPEED:
        retval16b = LIVE_GetSpeed();
        break;
    case CONFIG_MAIN_USB_CHOICE_1:
    case CONFIG_MAIN_USB_CHOICE_2:
    case CONFIG_MAIN_USB_CHOICE_3:
    case CONFIG_MAIN_USB_CHOICE_4:
    case CONFIG_MAIN_USB_CHOICE_5:
    case CONFIG_MAIN_USB_CHOICE_6:
    case CONFIG_MAIN_USB_CHOICE_7:
    case CONFIG_MAIN_USB_CHOICE_8:
    case CONFIG_MAIN_USB_CHOICE_9:
    case CONFIG_MAIN_USB_CHOICE_10:
        retval16b = LIVE_GetOutput(value_ID-CONFIG_MAIN_USB_CHOICE_1);
        break;
    case CONFIG_MOTOR_POLEPAIRS:
    case CONFIG_BMS_NUMBATTS:
        retval16b = 0xAAAAu;
        break;

    // 32 bit integer values
    case CONFIG_DRV_GATE_STRENGTH:
        retval32b = DRV8353_GetGateStrength();
        break;
    // Not yet implemented
    case CONFIG_FOC_PWM_FREQ:
    case CONFIG_FOC_PWM_DEADTIME:
    case CONFIG_MAIN_COUNTS_TO_FOC:
    case CONFIG_BMS_GETSTATUS_N:
        retval32b = 0xAAAAAAAAu;
        break;

    // 32 bit float values
    case CONFIG_ADC_RSHUNT:
        retvalf = ADC_GetRShunt();
        break;
    case CONFIG_ADC_VBUS_RATIO:
        retvalf = ADC_GetVbusRatio();
        break;
    case CONFIG_ADC_THERM_FIXED_R:
        retvalf = ADC_GetThermFixedR();
        break;
    case CONFIG_ADC_THERM_R25:
        retvalf = ADC_GetThermR25();
        break;
    case CONFIG_ADC_THERM_B:
        retvalf = ADC_GetThermBeta();
        break;
    case CONFIG_THRT_MIN:
        retvalf = THROTTLE_GetMin();
        break;
    case CONFIG_THRT_MAX:
        retvalf = THROTTLE_GetMax();
        break;
    case CONFIG_THRT_HYST:
        retvalf = THROTTLE_GetHyst();
        break;
    case CONFIG_THRT_FILT:
        retvalf = THROTTLE_GetFilt();
        break;
    case CONFIG_THRT_RISE:
        retvalf = THROTTLE_GetRise();
        break;
    case CONFIG_THRT_RATIO:
        retvalf = THROTTLE_GetRatio();
        break;
    // Not yet implemented
    case CONFIG_FOC_KP:
    case CONFIG_FOC_KI:
    case CONFIG_FOC_KD:
    case CONFIG_FOC_KC:
    case CONFIG_MAIN_SPEED_TO_FOC:
    case CONFIG_MAIN_SWITCH_EPS:
    case CONFIG_LMT_VOLT_FAULT_MIN:
    case CONFIG_LMT_VOLT_FAULT_MAX:
    case CONFIG_LMT_CUR_FAULT_MAX:
    case CONFIG_LMT_VOLT_SOFTCAP:
    case CONFIG_LMT_VOLT_HARDCAP:
    case CONFIG_LMT_PHASE_CUR_MAX:
    case CONFIG_LMT_PHASE_REGEN_MAX:
    case CONFIG_LMT_BATT_CUR_MAX:
    case CONFIG_LMT_BATT_REGEN_MAX:
    case CONFIG_LMT_FET_TEMP_SOFTCAP:
    case CONFIG_LMT_FET_TEMP_HARDCAP:
    case CONFIG_LMT_MOTOR_TEMP_SOFTCAP:
    case CONFIG_LMT_MOTOR_TEMP_HARDCAP:
    case CONFIG_MOTOR_HALL1:
    case CONFIG_MOTOR_HALL2:
    case CONFIG_MOTOR_HALL3:
    case CONFIG_MOTOR_HALL4:
    case CONFIG_MOTOR_HALL5:
    case CONFIG_MOTOR_HALL6:
    case CONFIG_MOTOR_GEAR_RATIO:
    case CONFIG_MOTOR_WHEEL_SIZE:
    case CONFIG_MOTOR_KV:
    case CONFIG_BMS_GETBAT_N:
        retvalf = 5.555f;
        break;
    }

    switch(command_get_datatype(value_ID)) {
    case Data_Type_None:
    case Data_Type_Int8:
        errCode = RESULT_IS_8B;
        data_packet_pack_8b(retval, retval8b);
        break;
    case Data_Type_Int16:
        errCode = RESULT_IS_16B;
        data_packet_pack_16b(retval, retval16b);
        break;
    case Data_Type_Int32:
        errCode = RESULT_IS_32B;
        data_packet_pack_32b(retval, retval32b);
        break;
    case Data_Type_Float:
        errCode = RESULT_IS_FLOAT;
        data_packet_pack_float(retval, retvalf);
        break;
    }
    return errCode;
}

uint16_t command_set_ram(uint8_t* pktdata) {
    // Data is two bytes for value ID
    uint16_t value_ID = data_packet_extract_16b(pktdata);
    //uint16_t value_ID = (((uint16_t)pktdata[0]) << 8) + pktdata[1];
    pktdata += 2;
    // Then two to four bytes for value, depending on command
    float valuef = 0.0f;
    uint8_t value8b = 0;
    uint16_t value16b = 0;
    uint32_t value32b = 0;
    uint16_t errCode = RETVAL_FAIL;

    switch(command_get_datatype(value_ID)) {
    case Data_Type_None:
    case Data_Type_Int8:
        value8b = data_packet_extract_8b(pktdata);
        break;
    case Data_Type_Int16:
        value16b = data_packet_extract_16b(pktdata);
        break;
    case Data_Type_Int32:
        value32b = data_packet_extract_32b(pktdata);
        break;
    case Data_Type_Float:
        valuef = data_packet_extract_float(pktdata);
        break;
    }

    switch (value_ID) {

    // 8-bit integer values
    case CONFIG_DRV_VDS_LIMIT:
        errCode = DRV8353_SetVDSLimit((DRV_VDS_Limit)value8b);
        break;
    case CONFIG_DRV_CSA_GAIN:
        errCode = DRV8353_SetGain((DRV_Gain)value8b);
        break;

    // 16-bit integer values
    // Not yet implemented
    case CONFIG_MAIN_NUM_USB_OUTPUTS:
        errCode = LIVE_SetNumOutputs(value16b);
        break;
    case CONFIG_MAIN_USB_SPEED:
        errCode = LIVE_SetSpeed(value16b);
        break;
    case CONFIG_MAIN_USB_CHOICE_1:
    case CONFIG_MAIN_USB_CHOICE_2:
    case CONFIG_MAIN_USB_CHOICE_3:
    case CONFIG_MAIN_USB_CHOICE_4:
    case CONFIG_MAIN_USB_CHOICE_5:
    case CONFIG_MAIN_USB_CHOICE_6:
    case CONFIG_MAIN_USB_CHOICE_7:
    case CONFIG_MAIN_USB_CHOICE_8:
    case CONFIG_MAIN_USB_CHOICE_9:
    case CONFIG_MAIN_USB_CHOICE_10:
        errCode = LIVE_SetOutput(value_ID-CONFIG_MAIN_USB_CHOICE_1,value16b);
        break;
    case CONFIG_MOTOR_POLEPAIRS:
        errCode = RETVAL_OK;
        break;

    // 32 bit integer values
    case CONFIG_DRV_GATE_STRENGTH:
        errCode = DRV8353_SetGateStrength(value32b);
        break;
    // Not yet implemented
    case CONFIG_FOC_PWM_FREQ:
    case CONFIG_FOC_PWM_DEADTIME:
    case CONFIG_MAIN_COUNTS_TO_FOC:
        errCode = RETVAL_OK;
        break;

    // 32 bit float values
    case CONFIG_ADC_RSHUNT:
        errCode = ADC_SetRShunt(valuef);
        break;
    case CONFIG_ADC_VBUS_RATIO:
        errCode = ADC_SetVbusRatio(valuef);
        break;
    case CONFIG_ADC_THERM_FIXED_R:
        errCode = ADC_SetThermFixedR(valuef);
        break;
    case CONFIG_ADC_THERM_R25:
        errCode = ADC_SetThermR25(valuef);
        break;
    case CONFIG_ADC_THERM_B:
        errCode = ADC_SetThermBeta(valuef);
        break;
    case CONFIG_THRT_MIN:
        errCode = THROTTLE_SetMin(valuef);
        break;
    case CONFIG_THRT_MAX:
        errCode = THROTTLE_SetMax(valuef);
        break;
    case CONFIG_THRT_HYST:
        errCode = THROTTLE_SetHyst(valuef);
        break;
    case CONFIG_THRT_FILT:
        errCode = THROTTLE_SetFilt(valuef);
        break;
    case CONFIG_THRT_RISE:
        errCode = THROTTLE_SetRise(valuef);
        break;
    case CONFIG_THRT_RATIO:
        errCode = THROTTLE_SetRatio(valuef);
        break;
    // Not yet implemented
    case CONFIG_FOC_KP:
    case CONFIG_FOC_KI:
    case CONFIG_FOC_KD:
    case CONFIG_FOC_KC:
    case CONFIG_MAIN_SPEED_TO_FOC:
    case CONFIG_MAIN_SWITCH_EPS:
    case CONFIG_LMT_VOLT_FAULT_MIN:
    case CONFIG_LMT_VOLT_FAULT_MAX:
    case CONFIG_LMT_CUR_FAULT_MAX:
    case CONFIG_LMT_VOLT_SOFTCAP:
    case CONFIG_LMT_VOLT_HARDCAP:
    case CONFIG_LMT_PHASE_CUR_MAX:
    case CONFIG_LMT_PHASE_REGEN_MAX:
    case CONFIG_LMT_BATT_CUR_MAX:
    case CONFIG_LMT_BATT_REGEN_MAX:
    case CONFIG_LMT_FET_TEMP_SOFTCAP:
    case CONFIG_LMT_FET_TEMP_HARDCAP:
    case CONFIG_LMT_MOTOR_TEMP_SOFTCAP:
    case CONFIG_LMT_MOTOR_TEMP_HARDCAP:
    case CONFIG_MOTOR_HALL1:
    case CONFIG_MOTOR_HALL2:
    case CONFIG_MOTOR_HALL3:
    case CONFIG_MOTOR_HALL4:
    case CONFIG_MOTOR_HALL5:
    case CONFIG_MOTOR_HALL6:
    case CONFIG_MOTOR_GEAR_RATIO:
    case CONFIG_MOTOR_WHEEL_SIZE:
    case CONFIG_MOTOR_KV:
        errCode = RETVAL_OK;
        break;


    }
    return errCode;
}

uint16_t command_get_eeprom(uint8_t* pktdata, uint8_t* retval) {
    uint16_t value_ID = data_packet_extract_16b(pktdata);
    int16_t retval16;
    int32_t retval32;
    float retvalf;
    uint16_t errCode = DATA_PACKET_FAIL;

    Data_Type type = command_get_datatype(value_ID);
    switch(type) {
    case Data_Type_None:
    case Data_Type_Int8:
        return DATA_PACKET_FAIL;
    case Data_Type_Int16:
        retval16 = EE_ReadInt16WithDefault(value_ID, 0);
        data_packet_pack_16b(retval, retval16);
        errCode = RESULT_IS_16B;
        break;
    case Data_Type_Int32:
        retval32 = EE_ReadInt32WithDefault(value_ID, 0);
        data_packet_pack_32b(retval, retval32);
        errCode = RESULT_IS_32B;
        break;
    case Data_Type_Float:
        retvalf = EE_ReadFloatWithDefault(value_ID, 0.0f);
        data_packet_pack_float(retval, retvalf);
        errCode = RESULT_IS_FLOAT;
        break;
    }

    return errCode;
}

uint16_t command_set_eeprom(uint8_t* pktdata) {
    uint16_t value_ID = data_packet_extract_16b(pktdata);
    pktdata += 2;
    uint16_t errCode = DATA_PACKET_FAIL;

    Data_Type type = command_get_datatype(value_ID);
    switch(type) {
    case Data_Type_None:
    case Data_Type_Int8:
        return DATA_PACKET_FAIL;
    case Data_Type_Int16:
        if (EE_SaveInt16(value_ID, data_packet_extract_16b(pktdata))
                == FLASH_COMPLETE) {
            errCode = RETVAL_OK;
        }
        break;
    case Data_Type_Int32:
        if (EE_SaveInt32(value_ID, data_packet_extract_32b(pktdata))
                == FLASH_COMPLETE) {
            errCode = RETVAL_OK;
        }
        break;
    case Data_Type_Float:
        if (EE_SaveFloat(value_ID, data_packet_extract_float(pktdata))
                == FLASH_COMPLETE) {
            errCode = RETVAL_OK;
        }
        break;
    }

    return errCode;
}

uint16_t command_enable_feature(uint8_t* pktdata) {
    // Data is two bytes for feature ID
    uint16_t feature_ID = data_packet_extract_16b(pktdata);
    uint16_t errCode = RETVAL_FAIL;

    switch (feature_ID) {
    case FEATURE_SERIAL_DATA:
        errCode = LIVE_TurnOnData();
        break;
    case FEATURE_BLDC_MODE:
        errCode = MAIN_SetControlMode(Control_BLDC);
        break;
    case FEATURE_SINE_MODE:
        errCode = MAIN_SetControlMode(Control_Sine);
        break;
    case FEATURE_DEBUG_PWM:
        errCode = MAIN_SetControlMode(Control_Debug);
        break;
    default:
        errCode = RETVAL_FAIL;
        break;
    }
    return errCode;
}

uint16_t command_disable_feature(uint8_t* pktdata) {
    // Data is two bytes for feature ID
    uint16_t feature_ID = data_packet_extract_16b(pktdata);
    uint16_t errCode = RETVAL_FAIL;

    switch (feature_ID) {
    case FEATURE_SERIAL_DATA:
        errCode = LIVE_TurnOffData();
        break;
    case FEATURE_BLDC_MODE:
    case FEATURE_SINE_MODE:
    case FEATURE_DEBUG_PWM:
        errCode = MAIN_SetControlMode(Control_FOC);
        break;
    default:
        errCode = RETVAL_FAIL;
        break;
    }
    return errCode;
}

uint16_t command_run_routine(uint8_t* pktdata) {
    // Routine ID is two bytes of packet data
    uint16_t routine_ID = data_packet_extract_16b(pktdata);
    pktdata += 2;
    uint16_t errCode = RETVAL_FAIL;

//    float valuef;

    switch(routine_ID) {

    case ROUTINE_LOAD_ALL_EEPROM:
        // Run the various loading functions
//        MAIN_LoadVariables();
//        HallSensor_Load_Variables();
//        adcLoadVariables();
//        throttle_load_variables();
        errCode = RETVAL_OK;
        break;
    case ROUTINE_SAVE_ALL_EEPROM:
        // Run all the saving functions
//        MAIN_SaveVariables();
//        HallSensor_Save_Variables();
//        adcSaveVariables();
//        throttle_save_variables();
        errCode = RETVAL_OK;
        break;
    case ROUTINE_HALL_DETECT:
        // Single variable float is applied current
//        valuef = data_packet_extract_float(pktdata);
//        MAIN_DetectHallPositions(valuef);
        errCode = RETVAL_OK;
        break;
    case ROUTINE_SOFT_RESET:
        // Run the reset command
        // Shouldn't return from this function
        MAIN_Reboot();
        break;
    case ROUTINE_BOOTLOADER_RESET:
        // Run the reset command but enable bootloader when back alive
        // Shouldn't return from this function
        MAIN_GoToBootloader();
        break;
    }

    return errCode;
}

static Data_Type command_get_datatype(uint16_t data_ID) {
    Data_Type type = Data_Type_None;
    switch(data_ID) {
    // 8-bit integer values
    case CONFIG_DRV_VDS_LIMIT:
    case CONFIG_DRV_CSA_GAIN:
    case CONFIG_BMS_ISCONNECTED:
        type = Data_Type_Int8;
        break;
    // 16-bit integer values
    case CONFIG_MAIN_NUM_USB_OUTPUTS:
    case CONFIG_MAIN_USB_SPEED:
    case CONFIG_MAIN_USB_CHOICE_1:
    case CONFIG_MAIN_USB_CHOICE_2:
    case CONFIG_MAIN_USB_CHOICE_3:
    case CONFIG_MAIN_USB_CHOICE_4:
    case CONFIG_MAIN_USB_CHOICE_5:
    case CONFIG_MAIN_USB_CHOICE_6:
    case CONFIG_MAIN_USB_CHOICE_7:
    case CONFIG_MAIN_USB_CHOICE_8:
    case CONFIG_MAIN_USB_CHOICE_9:
    case CONFIG_MAIN_USB_CHOICE_10:
    case CONFIG_MOTOR_POLEPAIRS:
    case CONFIG_BMS_NUMBATTS:
        type = Data_Type_Int16;
        break;
    // 32 bit integer values
    case CONFIG_FOC_PWM_FREQ:
    case CONFIG_FOC_PWM_DEADTIME:
    case CONFIG_MAIN_COUNTS_TO_FOC:
    case CONFIG_DRV_GATE_STRENGTH:
    case CONFIG_BMS_GETSTATUS_N:
        type = Data_Type_Int32;
        break;
    // 32 bit float values
    case CONFIG_ADC_RSHUNT:
    case CONFIG_ADC_VBUS_RATIO:
    case CONFIG_ADC_THERM_FIXED_R:
    case CONFIG_ADC_THERM_R25:
    case CONFIG_ADC_THERM_B:
    case CONFIG_FOC_KP:
    case CONFIG_FOC_KI:
    case CONFIG_FOC_KD:
    case CONFIG_FOC_KC:
    case CONFIG_MAIN_SPEED_TO_FOC:
    case CONFIG_MAIN_SWITCH_EPS:
    case CONFIG_THRT_MIN:
    case CONFIG_THRT_MAX:
    case CONFIG_THRT_HYST:
    case CONFIG_THRT_FILT:
    case CONFIG_THRT_RISE:
    case CONFIG_THRT_RATIO:
    case CONFIG_LMT_VOLT_FAULT_MIN:
    case CONFIG_LMT_VOLT_FAULT_MAX:
    case CONFIG_LMT_CUR_FAULT_MAX:
    case CONFIG_LMT_VOLT_SOFTCAP:
    case CONFIG_LMT_VOLT_HARDCAP:
    case CONFIG_LMT_PHASE_CUR_MAX:
    case CONFIG_LMT_PHASE_REGEN_MAX:
    case CONFIG_LMT_BATT_CUR_MAX:
    case CONFIG_LMT_BATT_REGEN_MAX:
    case CONFIG_LMT_FET_TEMP_SOFTCAP:
    case CONFIG_LMT_FET_TEMP_HARDCAP:
    case CONFIG_LMT_MOTOR_TEMP_SOFTCAP:
    case CONFIG_LMT_MOTOR_TEMP_HARDCAP:
    case CONFIG_MOTOR_HALL1:
    case CONFIG_MOTOR_HALL2:
    case CONFIG_MOTOR_HALL3:
    case CONFIG_MOTOR_HALL4:
    case CONFIG_MOTOR_HALL5:
    case CONFIG_MOTOR_HALL6:
    case CONFIG_MOTOR_GEAR_RATIO:
    case CONFIG_MOTOR_WHEEL_SIZE:
    case CONFIG_MOTOR_KV:
    case CONFIG_BMS_GETBAT_N:
        type = Data_Type_Float;
        break;
    }
    return type;
}
