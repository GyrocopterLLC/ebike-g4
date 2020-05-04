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

// *** Global variables ***
// Used for creating the local CRC for comparison
uint8_t TempPacketArray[PACKET_OVERHEAD_BYTES + PACKET_MAX_DATA_LENGTH];

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

uint32_t data_packet_recreate_crc(uint8_t PacketType, uint8_t* DataBuffer, uint16_t DataLength) {
    uint32_t crc;
    uint16_t place = 0;
    TempPacketArray[place++] = PACKET_START_0;
    TempPacketArray[place++] = PACKET_START_1;
    TempPacketArray[place++] = PacketType;
    TempPacketArray[place++] = PacketType ^ 0xFF; // All bits flipped
    TempPacketArray[place++] = (uint8_t)((DataLength & 0xFF00) >> 8); // Hibyte
    TempPacketArray[place++] = (uint8_t)(DataLength & 0x00FF); // Lobyte
    for(uint16_t i = 0; i < DataLength; i++) {
        TempPacketArray[place++] = DataBuffer[i];
    }
    crc = CRC_Generate_CRC32(TempPacketArray, DataLength + PACKET_NONCRC_OVHD_BYTES);

    return crc;
}

/**
 * @brief  Data Packet Create
 *            Generates a data packet from the required fields. Packs the
 *            transmission buffer with the proper byte order and creates the
 *            CRC field for error checking.
 * @param  pkt - pointer to the Data_Packet_Type which will hold the
 *               generated packet bytes
 * @param  type - packet type, defines the command type
 * @param  data - pointer to data buffer to pack into the packet
 * @param  datalen - length of the data buffer to be packed
 * @retval DATA_PACKET_FAIL - the packet couldn't be created
 *            DATA_PACKET_SUCCESS - packet was created, it can now be sent
 */
uint8_t data_packet_create(Data_Packet_Type* pkt, uint8_t type, uint8_t* data,
        uint16_t datalen) {
    uint16_t place = 0;
    uint32_t crc;

    // Fail out if the packet can't fit in the buffer
    if (datalen + PACKET_OVERHEAD_BYTES > PACKET_MAX_LENGTH) {
        pkt->TxReady = 0;
        return DATA_PACKET_FAIL;
    }

    pkt->TxBuffer[place++] = PACKET_START_0;
    pkt->TxBuffer[place++] = PACKET_START_1;
    pkt->TxBuffer[place++] = type;
    pkt->TxBuffer[place++] = type ^ 0xFF;
    pkt->TxBuffer[place++] = (uint8_t) ((datalen & 0xFF00) >> 8);
    pkt->TxBuffer[place++] = (uint8_t) (datalen & 0x00FF);
    for (uint16_t i = 0; i < datalen; i++) {
        pkt->TxBuffer[place++] = data[i];
    }

    crc = CRC_Generate_CRC32(pkt->TxBuffer, datalen + PACKET_NONCRC_OVHD_BYTES);
    pkt->TxBuffer[place++] = (uint8_t) ((crc & 0xFF000000) >> 24);
    pkt->TxBuffer[place++] = (uint8_t) ((crc & 0x00FF0000) >> 16);
    pkt->TxBuffer[place++] = (uint8_t) ((crc & 0x0000FF00) >> 8);
    pkt->TxBuffer[place++] = (uint8_t) (crc & 0x000000FF);
    pkt->TxReady = 1;
    pkt->TxLength = place;
    return DATA_PACKET_SUCCESS;

}

/**
 * @brief  Data Packet Extract One Byte Method
 *         Decodes a data packet coming in from any data channel. Discovers
 *         the packet type, pulls out the packet data, and checks the CRC
 *         field for transmission errors.
 *         Processed one byte at a time using an internal state machine.
 * @param  pkt - pointer to the Data_Packet_Type which will hold the
 *               decoded packet
 * @param  new_byte - raw data byte coming in from any comm channel. Simply
 *                    pass in the incoming bytes one at a time, this function
 *                    takes care of keeping track of past bytes.
 * @retval DATA_PACKET_FAIL - no new packet found (yet!)
 *         DATA_PACKET_SUCCESS - the packet was valid and was decoded
 */
uint8_t data_packet_extract_one_byte(Data_Packet_Type *pkt, uint8_t new_byte) {
    uint8_t retval = DATA_PACKET_FAIL;

    // First check for timeout
    if(pkt->State != DATA_COMM_IDLE) {
        // Every other state can time out
        if(GetTick() - pkt->TimerStart > DATA_PACKET_TIMEOUT_MS) {
            // Reset back to beginning
            pkt->State = DATA_COMM_IDLE;
        }
    }

    // Now everything is determined based on the state
    switch (pkt->State) {
    case DATA_COMM_IDLE:
        // Is this byte the first Start byte?
        // Otherwise, ignore
        if (new_byte == PACKET_START_0) {
            pkt->State = DATA_COMM_START_0;
            // Start timeout. Packet must be received fairly quickly or else comm is reset.
            pkt->TimerStart = GetTick();
        }
        break;
    case DATA_COMM_START_0:
        // Must be second start byte, otherwise reset
        if (new_byte == PACKET_START_1) {
            pkt->State = DATA_COMM_START_1;
        } else {
            // Back to idle since we didn't get the expected sequence
            pkt->State = DATA_COMM_IDLE;
            pkt->FaultCode = NO_START_DETECTED;
        }
        break;
    case DATA_COMM_START_1:
        // Next byte is packet type. Just read it, can't error check until next one
        pkt->PacketType = new_byte;
        pkt->State = DATA_COMM_PKT_TYPE;
        break;
    case DATA_COMM_PKT_TYPE:
        // This should be the inverted packet type. Now we can error check
        new_byte = new_byte^0xFF; // Invert this one
        if (new_byte != pkt->PacketType) {
            pkt->State = DATA_COMM_IDLE;
            pkt->FaultCode = BAD_PACKET_TYPE;
        } else {
            pkt->State = DATA_COMM_NPKT_TYPE;
        }
        break;
    case DATA_COMM_NPKT_TYPE:
        // Ready to read the first byte of data length
        pkt->DataLength = ((uint16_t) new_byte) << 8;
        pkt->State = DATA_COMM_DATALEN_0;
        break;
    case DATA_COMM_DATALEN_0:
        // And the second byte
        pkt->DataLength += new_byte;
        pkt->DataBytesRead = 0;
        pkt->State = DATA_COMM_DATALEN_1;
        break;
    case DATA_COMM_DATALEN_1:
        // Now we got to keep track of how much data has been collected
        // Count to DataLen bytes, then shift in the 4 bytes of CRC-32
        // Using the packet's data buffer, we don't need to make our own.
        if (pkt->DataBytesRead < pkt->DataLength) {
            pkt->Data[pkt->DataBytesRead++] =
                    new_byte;
        } else {
            // Now onto the CRC
            pkt->Remote_CRC_32 = ((uint32_t) new_byte) << 24;
            pkt->State = DATA_COMM_CRC_0;
        }
        break;
    case DATA_COMM_CRC_0:
        pkt->Remote_CRC_32 += ((uint32_t) new_byte) << 16;
        pkt->State = DATA_COMM_CRC_1;
        break;
    case DATA_COMM_CRC_1:
        pkt->Remote_CRC_32 += ((uint32_t) new_byte) << 8;
        pkt->State = DATA_COMM_CRC_2;
        break;
    case DATA_COMM_CRC_2:
        // Finally at the end. If this CRC matches, we have a good packet.
        pkt->Remote_CRC_32 += ((uint32_t) new_byte);
        // Calculate our own CRC
        if (pkt->Remote_CRC_32
                == data_packet_recreate_crc(pkt->PacketType,
                        pkt->Data, pkt->DataLength)) {
            // Good packet!
            pkt->RxReady = 1;
            pkt->FaultCode = NO_FAULT;
            retval = DATA_PACKET_SUCCESS;
            pkt->State = DATA_COMM_IDLE; // Reset for the next packet
        } else {
            // Failed CRC. Bad packet.
            pkt->FaultCode = BAD_CRC;
            pkt->State = DATA_COMM_IDLE;
        }
    }
    return retval;
}
