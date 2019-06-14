/******************************************************************************
 * Filename: data_packet.h
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

#ifndef _DATA_PACKET_H_
#define _DATA_PACKET_H_

typedef struct
{
    uint16_t DataLength;
    uint8_t PacketType;
    uint8_t FaultCode;
    uint8_t RxReady;
    uint8_t* Data;

    uint8_t TxReady;
    uint16_t TxLength;
    uint8_t* TxBuffer;
} Data_Packet_Type;

#define DATA_PACKET_FAIL		(0)
#define DATA_PACKET_SUCCESS		(1)

#define PACKET_MAX_LENGTH       (256)
#define PACKET_MAX_DATA_LENGTH  (64)

// SOP defines
#define PACKET_START_0          (0x9A)
#define PACKET_START_1          (0xCC)

// Packet type defines, Host to Controller
#define GET_RAM_VARIABLE        (0x01)
#define SET_RAM_VARIABLE        (0x02)
#define GET_EEPROM_VARIABLE     (0x03)
#define SET_EEPROM_VARIABLE     (0x04)
#define ENABLE_FEATURE          (0x05)
#define DISABLE_FEATURE         (0x06)
#define RUN_ROUTINE             (0x07)
#define HOST_STREAM_DATA        (0x08)
#define HOST_ACK                (0x11)
#define HOST_NACK               (0x12)
// Packet type defines, Controller to Host
#define GET_RAM_RESULT          (0x81)
#define GET_EEPROM_RESULT       (0x83)
#define ROUTINE_RESULT          (0x87)
#define CONTROLLER_STREAM_DATA  (0x88)
#define CONTROLLER_ACK          (0x91)
#define CONTROLLER_NACK         (0x92)

// Fault codes
#define NO_FAULT                (0x00)
#define BAD_CRC                 (0x01)
#define BAD_PACKET_TYPE         (0x02)
#define NO_START_DETECTED       (0x04)
#define INVALID_PACKET_LENGTH   (0x08)

uint8_t data_packet_create(Data_Packet_Type* pkt, uint8_t type, uint8_t* data,
                          uint16_t datalen);

uint8_t data_packet_extract(Data_Packet_Type* pkt, uint8_t* buf,
                            uint16_t buflen);


#endif //_DATA_PACKET_H_
