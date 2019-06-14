/******************************************************************************
 * Filename: data_packet.c
 * Description: Creates and decodes data packets sent between this controller
 *              and other devices. Packets use a defined framing and order
 *              with CRC to ensure data integrity.
 *
 * Packet structure:
 *  - Start of packet "SOP" (2 bytes)
 *  --- 0x9A, 0xCC
 *  - Packet type (1 byte)
 *  - nPacket type (1 byte, inverse of previous byte)
 *  - Data length (2 bytes)
 *  - Data (n bytes, depending on type)
 *  - CRC-32 (4 bytes)
 *  --- CRC is generated on ALL bytes.
 *      This includes the SOP, type, inverted type, length, and data.
 *      Check crc32.c/h for details on CRC generation. It's the same as the
 *      Ethernet CRC-32 standard. 
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
/**
 * Packet types:
 * - From host to controller:
 * -- 0x01 - Get variable from RAM
 * -- 0x02 - Set variable in RAM
 * -- 0x03 - Get variable from EEPROM
 * -- 0x04 - Set variable in EEPROM
 * -- 0x05 - Enable feature
 * -- 0x06 - Disable feature
 * -- 0x07 - Run routine
 * -- 0x08 - Stream data
 * -- 0x11 - ACK
 * -- 0x12 - NACK
 * - From controller to host:
 * -- 0x81 - Requested RAM data
 * -- 0x83 - Requested EEPROM data
 * -- 0x87 - Routine result
 * -- 0x88 - Stream data
 * -- 0x91 - ACK
 * -- 0x92 - NACK
 */

uint8_t data_packet_create(Data_Packet_Type* pkt, uint8_t type, uint8_t* data,
                          uint16_t datalen) {
  uint16_t place = 0;
  uint32_t crc;

  // Fail out if the packet can't fit in the buffer
  if(datalen + 8 > PACKET_MAX_LENGTH) {
    pkt->TxReady = 0;
    return DATA_PACKET_FAIL;
  }

  pkt->TxBuffer[place++] = PACKET_START_0;
  pkt->TxBuffer[place++] = PACKET_START_1;
  pkt->TxBuffer[place++] = type;
  pkt->TxBuffer[place++] = type ^ 0xFF;
  pkt->TxBuffer[place++] = (uint8_t)((datalen & 0xFF00) >> 8);
  pkt->TxBuffer[place++] = (uint8_t)(datalen & 0x00FF);
  for(uint16_t i = 0; i < datalen; i++) {
    pkt->TxBuffer[place++] = data[i];
  }

  crc = CRC32_Generate(pkt->TxBuffer, datalen + 6);
  pkt->TxBuffer[place++] = (uint8_t)((crc & 0xFF000000) >> 24);
  pkt->TxBuffer[place++] = (uint8_t)((crc & 0x00FF0000) >> 16);
  pkt->TxBuffer[place++] = (uint8_t)((crc & 0x0000FF00) >> 8);
  pkt->TxBuffer[place++] = (uint8_t)(crc & 0x000000FF);
  pkt->TxReady = 1;
  return DATA_PACKET_SUCCESS;

}

uint8_t data_packet_extract(Data_Packet_Type* pkt, uint8_t* buf,
                            uint16_t buflen) {
  uint16_t place;
  uint8_t SOP_found = 0;
  uint32_t crc_local;
  uint32_t crc_remote;
  uint16_t data_length;
  uint8_t packet_type;
  uint8_t nPacket_type;
  // Search forward to find SOP
  while((place < buflen-1) && (!SOP_found)) {
    if(buf[place] == PACKET_START_0 && buf[place+1] == PACKET_START_1) {
      SOP_found = 1;
    }
    place++; // Advance to next byte and try again
  }
  place++; // Move past the 2nd SOP byte
  if(!SOP_found) {
    pkt->FaultCode = NO_START_DETECTED;
    return DATA_PACKET_FAIL;
  }

  if((place+4) > buflen) {
    pkt->FaultCode = INVALID_PACKET_LENGTH;
    return DATA_PACKET_FAIL;
  }

  packet_type = buf[place++];
  nPacket_type = buf[place++];
  nPacket_type = ~(nPacket_type);
  if(packet_type != nPacket_type) {
    pkt->FaultCode = BAD_PACKET_TYPE;
    return DATA_PACKET_FAIL;
  }
  data_length = ((uint16_t)(buf[place]) >> 8) + (uint16_t)(buf[place+1]);
  place += 2;
  if(data_length > PACKET_MAX_DATA_LENGTH) {
    pkt->FaultCode = INVALID_PACKET_LENGTH;
    return DATA_PACKET_FAIL;
  }
  if((place+data_length+4) > buflen) {
    pkt->FaultCode = INVALID_PACKET_LENGTH;
    return DATA_PACKET_FAIL;
  }
  crc_local = CRC32_Generate(&(buf[place-6]),data_length+6);
  crc_remote = ((uint32_t)(buf[place+data_length]) << 24)
             + ((uint32_t)(buf[place+data_length+1]) << 16)
             + ((uint32_t)(buf[place+data_length+2]) << 8)
             + (uint32_t)(buf[place+data_length+3]);
  if(crc_local != crc_remote) {
    pkt->FaultCode = BAD_CRC;
    return DATA_PACKET_FAIL;
  }
  pkt->PacketType = packet_type;
  pkt->DataLength = data_length;

  memcpy(pkt->Data, &(buf[place]), pkt->DataLength);
  pkt->FaultCode = NO_FAULT;
  pkt->RxReady = 1;
  return DATA_PACKET_SUCCESS;
}
