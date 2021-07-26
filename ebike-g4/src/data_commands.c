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
/**
 * Unused old version
static Data_Type command_get_datatype(uint16_t data_ID);
***/

static inline Data_Type command_get_datatype(uint16_t id);
/**
 * @brief  Data Process Command
 *              Interprets the command in a decoded packet. Calls the
 *              appropriate sub-function for the requested command.
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
 * @param  pktdata - Pointer to the location with the incoming packet data.
 *                   Two bytes for variable ID.
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
    uint16_t errCode = RETVAL_FAIL;
    Data_Type dtype = Data_Type_None;
    uint8_t get_rtn_val = DATA_PACKET_FAIL;
    for(uint16_t i = 0; i < TOTAL_EE_VARS; i++) {
        if(value_ID == Ram_Commands[i]->id) {
            dtype = Ram_Commands[i]->dtype;
            if(Ram_Commands[i]->get != NULL) {
                get_rtn_val = Ram_Commands[i]->get(retval);
            }
            break;
        }
    }
    if(get_rtn_val == DATA_PACKET_SUCCESS) {
        switch(dtype) {
        case Data_Type_Int8:
            errCode = RESULT_IS_8B;
            break;
        case Data_Type_Int16:
            errCode = RESULT_IS_16B;
            break;
        case Data_Type_Int32:
            errCode = RESULT_IS_32B;
            break;
        case Data_Type_Float:
            errCode = RESULT_IS_FLOAT;
            break;
        default:
            errCode = RETVAL_FAIL;
            break;
        }
    }
    return errCode;
}

/**
 * Sets the new value of a config variable in ram. The variable is
 * looked up using the ID to call the correct "set" function
 * @param pktdata   Pointer to the location with the incoming packet data.
 *                  First two bytes are the variable ID, then remaining 1
 *                  to 4 bytes are the new value.
 * @return  RETVAL_FAIL - Unable to set the new data
 *          RETVAL_OK - New data set successfully
 */
uint16_t command_set_ram(uint8_t* pktdata) {
    // Data is two bytes for value ID
    uint16_t value_ID = data_packet_extract_16b(pktdata);
    pktdata += 2;
    // Then one to four bytes for value, depending on command

    uint16_t errCode = RETVAL_FAIL;
    uint8_t set_rtn_val = DATA_PACKET_FAIL;

    for(uint16_t i = 0; i < TOTAL_EE_VARS; i++) {
        if(value_ID == Ram_Commands[i]->id) {
            if(Ram_Commands[i]->set != NULL) {
                set_rtn_val = Ram_Commands[i]->set(pktdata);
            }
            break;
        }
    }
    if(set_rtn_val == DATA_PACKET_SUCCESS) {
        errCode = RETVAL_OK;
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

/**
 * Gets the datatype of a config variable. This is a simple lookup
 * through the ram commands struct array based on the ID.
 * @param id An integer used to fetch the particular variable
 * @return The type ("Data_Type") of the variable
 */
static inline Data_Type command_get_datatype(uint16_t id) {
    for (uint16_t i = 0; i < TOTAL_EE_VARS; i++) {
        if(id == Ram_Commands[i]->id) {
            return Ram_Commands[i]->dtype;
        }
    }

    // If control flows to here, ID did not match any known variable.
    return Data_Type_None;
}
