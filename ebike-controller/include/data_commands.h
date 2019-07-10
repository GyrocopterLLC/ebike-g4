/******************************************************************************
 * Filename: data_commands.h
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

#ifndef _DATA_COMMANDS_H_
#define _DATA_COMMANDS_H_

#define DATA_COMMAND_FAIL			(0)
#define DATA_COMMAND_SUCCESS		(1)
#define RESULT_IS_8B				(2)
#define RESULT_IS_16B				(3)
#define RESULT_IS_32B				(4)
#define RESULT_IS_FLOAT				(5)

#define CONFIG_DATA_SPEED			(0x1701)
#define CONFIG_DATA_NUMVARS			(0x1702)
#define CONFIG_DATA_VAR1			(0x1703)
#define CONFIG_DATA_VAR2			(0x1704)
#define CONFIG_DATA_VAR3			(0x1705)
#define CONFIG_DATA_VAR4			(0x1706)
#define CONFIG_DATA_VAR5			(0x1707)
#define CONFIG_DATA_VAR6			(0x1708)
#define CONFIG_DATA_VAR7			(0x1709)
#define CONFIG_DATA_VAR8			(0x170A)
#define CONFIG_DATA_VAR9			(0x170B)
#define CONFIG_DATA_VAR10			(0x170C)

#define CONFIG_FOC_KP				(0x1001)
#define CONFIG_FOC_KI				(0x1002)
#define CONFIG_FOC_KD				(0x1003)
#define CONFIG_FOC_KC				(0x1004)
#define CONFIG_FOC_DT				(0x1005)

#define CONFIG_MOTOR_HALL1			(0x1211)
#define CONFIG_MOTOR_HALL2			(0x1212)
#define CONFIG_MOTOR_HALL3			(0x1213)
#define CONFIG_MOTOR_HALL4			(0x1214)
#define CONFIG_MOTOR_HALL5			(0x1215)
#define CONFIG_MOTOR_HALL6			(0x1216)

#define CONFIG_THR_TYPE1			(0x1411)
#define CONFIG_THR_MIN1				(0x1412)
#define CONFIG_THR_MAX1				(0x1413)
#define CONFIG_THR_HYST1			(0x1414)
#define CONFIG_THR_FILT1			(0x1415)
#define CONFIG_THR_RISE1			(0x1416)
#define CONFIG_THR_TYPE2			(0x1421)
#define CONFIG_THR_MIN2				(0x1422)
#define CONFIG_THR_MAX2				(0x1423)
#define CONFIG_THR_HYST2			(0x1424)
#define CONFIG_THR_FILT2			(0x1425)
#define CONFIG_THR_RISE2			(0x1426)

#define FEATURE_SERIAL_DATA         (0x0001)

uint16_t data_process_command(Data_Packet_Type* pkt);
uint16_t command_get_ram(uint8_t* pktdata, uint8_t* retval);
uint16_t command_set_ram(uint8_t* pktdata);
uint16_t command_get_eeprom(uint8_t* pktdata, uint8_t* retval);
uint16_t command_set_eeprom(uint8_t* pktdata);
uint16_t command_enable_feature(uint8_t* pktdata);
uint16_t command_disable_feature(uint8_t* pktdata);
uint16_t command_run_routine(uint8_t* pktdata);

#endif //_DATA_COMMANDS_H_
