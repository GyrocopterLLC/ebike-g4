/******************************************************************************
 * Filename: usb_data_comm.c
 * Description: Conducts data packet communication over the USB serial port.
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
#include "usb_data_comm.h"
#include "data_packet.h"
#include "data_commands.h"

// Private variables
uint8_t USB_Data_Comm_TxBuffer[PACKET_MAX_LENGTH];
#if 0
uint8_t USB_Data_Comm_RxBuffer[PACKET_MAX_LENGTH];
int32_t USB_Data_Comm_RxBuffer_WrPlace;
#endif
uint8_t USB_Data_Comm_DataBuffer[PACKET_MAX_DATA_LENGTH];
Data_Packet_Type USB_Data_Comm_Packet;

// Private functions
static void USB_Data_Comm_Process_Command(void);

// Public functions

/**
 * @brief  USB Data Communications Initialization
 * 		   Sets packet data to default values.
 * @param  None
 * @retval None
 */
void USB_Data_Comm_Init(void) {
    USB_Data_Comm_Packet.State = DATA_COMM_IDLE;
    USB_Data_Comm_Packet.Data = USB_Data_Comm_DataBuffer;
    USB_Data_Comm_Packet.TxBuffer = USB_Data_Comm_TxBuffer;
    USB_Data_Comm_Packet.TxReady = 0;
    USB_Data_Comm_Packet.RxReady = 0;
#if 0
    USB_Data_Comm_RxBuffer_WrPlace = 0;
#endif

}

/**
 * @brief  USB Data Communications One Byte Check
 *         Handles the USB serial port incoming data. Determines
 *         if a properly encoded packet has been received, and
 *         sends to the appropriate handler if it has. Operates
 *         one byte at a time using an internal state machine.
 * @param  None
 * @retval None
 */
void USB_Data_Comm_OneByte_Check(void) {
    // Loop through each incoming byte
    int32_t numbytes = VCP_InWaiting();
    uint8_t this_byte;
    while(numbytes > 0) {
        // Load one new byte
        if(VCP_Read(&this_byte, 1) != 1) {
            // Check that a byte was really received
            return;
        }
        numbytes--;
        if(data_packet_extract_one_byte(&USB_Data_Comm_Packet, this_byte) == DATA_PACKET_SUCCESS) {
            if(USB_Data_Comm_Packet.RxReady == 1) {
                // Double checked and good to go
                USB_Data_Comm_Process_Command();
            }
        }
    }
}

#if 0

/**
 * @brief  USB Data Communications Periodic Check
 * 		   Handles the USB serial port incoming data. Determines
 * 		   if a properly encoded packet has been received, and
 * 		   sends to the appropriate handler if it has. Clears
 * 		   buffer when necessary to accept more data.
 *
 * 		   Note: when new data arrives to a full buffer, the
 * 		   existing data is cleared one byte at a time, oldest
 * 		   to newest. The assumption is made that the buffer is
 * 		   larger than the largest possible packet, so there is
 * 		   no possible way to interrupt a packet in transmission
 * 		   by accidentally deleting part of it.
 * @param  None
 * @retval None
 */
void USB_Data_Comm_Periodic_Check(void) {
    // check if data is available
    int32_t numbytes = VCP_InWaiting();
    uint16_t pkt_end;
    if (numbytes <= 0) {
        return;
    }
    // Do we have space?
    int32_t space_remaining = PACKET_MAX_LENGTH
            - USB_Data_Comm_RxBuffer_WrPlace;
    while (numbytes > space_remaining) {
        // Copy as much as we can.
        VCP_Read(USB_Data_Comm_RxBuffer + USB_Data_Comm_RxBuffer_WrPlace,
                space_remaining);
        // Check if any valid packets exist in the whole buffer
        data_packet_extract(&USB_Data_Comm_Packet, USB_Data_Comm_RxBuffer,
                PACKET_MAX_LENGTH);
        if (USB_Data_Comm_Packet.RxReady) {
            // If yes, process the packet and then move the buffer to make some room
            USB_Data_Comm_Process_Command();
            pkt_end = (USB_Data_Comm_Packet.StartPosition
                    + USB_Data_Comm_Packet.DataLength + PACKET_OVERHEAD_BYTES);
            memmove(USB_Data_Comm_RxBuffer, USB_Data_Comm_RxBuffer + pkt_end,
            PACKET_MAX_LENGTH - pkt_end);
            USB_Data_Comm_RxBuffer_WrPlace = PACKET_MAX_LENGTH - pkt_end;
            space_remaining = pkt_end;
        } else {
            // If no, discard one byte from the front, making one space available
            memmove(USB_Data_Comm_RxBuffer, USB_Data_Comm_RxBuffer + 1,
                    PACKET_MAX_LENGTH - 1);
            space_remaining = 1;
            USB_Data_Comm_RxBuffer_WrPlace = PACKET_MAX_LENGTH - 1;
        }
        // How much is left now?
        numbytes = VCP_InWaiting();
    }

    VCP_Read(USB_Data_Comm_RxBuffer + USB_Data_Comm_RxBuffer_WrPlace, numbytes);
    USB_Data_Comm_RxBuffer_WrPlace += numbytes;
    // Check if we have a packet
    data_packet_extract(&USB_Data_Comm_Packet, USB_Data_Comm_RxBuffer,
            USB_Data_Comm_RxBuffer_WrPlace);
    if (USB_Data_Comm_Packet.RxReady) {
        // Do the thing commanded
        USB_Data_Comm_Process_Command();
        // And free up space
        pkt_end = (USB_Data_Comm_Packet.StartPosition
                + USB_Data_Comm_Packet.DataLength + PACKET_OVERHEAD_BYTES);
        memmove(USB_Data_Comm_RxBuffer, USB_Data_Comm_RxBuffer + pkt_end,
        PACKET_MAX_LENGTH - pkt_end);
        USB_Data_Comm_RxBuffer_WrPlace -= pkt_end;
    }
}

#endif

/**
 * @brief  USB Data Communications Process Command
 * 		   Calls the command processor when a packet has been successfully
 * 		   decoded. If a response is generated, it is sent back over the
 * 		   USB serial port.
 * @param  None
 * @retval None
 */
static void USB_Data_Comm_Process_Command(void) {
    uint16_t errCode = data_process_command(&USB_Data_Comm_Packet);
    if ((errCode == DATA_PACKET_SUCCESS) && USB_Data_Comm_Packet.TxReady) {
        uint8_t* send_buffer = USB_Data_Comm_Packet.TxBuffer;
        uint16_t len_to_send = USB_Data_Comm_Packet.TxLength;
        uint16_t actually_sent = 0;
        while (len_to_send > 0) {
            actually_sent = VCP_Write(send_buffer, len_to_send);
            len_to_send -= actually_sent;
            send_buffer += actually_sent;
        }
        USB_Data_Comm_Packet.TxReady = 0;
    }
}
