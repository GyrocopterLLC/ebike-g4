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

uint16_t data_process_command(Data_Packet_Type* pkt);
uint16_t command_get_ram(uint8_t* pktdata, uint8_t* retval);
uint16_t command_set_ram(uint8_t* pktdata);
uint16_t command_get_eeprom(uint8_t* pktdata, uint8_t* retval);
uint16_t command_set_eeprom(uint8_t* pktdata);
uint16_t command_enable_feature(uint8_t* pktdata);
uint16_t command_disable_feature(uint8_t* pktdata);
uint16_t command_run_routine(uint8_t* pktdata);

#endif //_DATA_COMMANDS_H_
