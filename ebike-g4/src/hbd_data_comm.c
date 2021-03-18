/******************************************************************************
 * Filename: hbd_data_comm.c
 * Description: Conducts data packet communication between this motor
 * controller and the Handle Bar Display (hbd). The hbd is connected
 * via UART. The intended device is an Android phone using a Bluetooth
 * module (HC-05 serial bluetooth module), but any device with a UART
 * connection could work.
 *
 ******************************************************************************

 Copyright (c) 2021 David Miller

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
#include "hbd_data_comm.h"
#include "data_packet.h"
#include "data_commands.h"

uint8_t HBD_Data_Comm_TxBuffer[PACKET_MAX_LENGTH];
uint8_t HBD_Data_Comm_DataBuffer[PACKET_MAX_DATA_LENGTH];
Data_Packet_Type HBD_Data_Comm_Packet;

static void HBD_Data_Comm_Process_Command(void);

/**
 * @brief  HBD Data Communications Initialization
 *         Sets packet data to default values.
 * @param  None
 * @retval None
 */
void HBD_Data_Comm_Init(void) {
    HBD_Data_Comm_Packet.State = DATA_COMM_IDLE;
    HBD_Data_Comm_Packet.Data = HBD_Data_Comm_DataBuffer;
    HBD_Data_Comm_Packet.TxBuffer = HBD_Data_Comm_TxBuffer;
    HBD_Data_Comm_Packet.TxReady = 0;
    HBD_Data_Comm_Packet.RxReady = 0;
}

/**
 * @brief  HBD Data Communications One Byte Check
 *         Handles the HBD serial port incoming data. Determines
 *         if a properly encoded packet has been received, and
 *         sends to the appropriate handler if it has. Operates
 *         one byte at a time using an internal state machine.
 * @param  None
 * @retval None
 */
void HBD_OneByte_Check(void) {
    // Loop through each incoming byte
    int32_t numbytes = UART_InWaiting(SELECT_HBD_UART);
    uint8_t this_byte;
    while(numbytes > 0) {
        // Load one new byte
        if(UART_Read(SELECT_HBD_UART, &this_byte, 1) != 1) {
            // Check that a byte was really received
            return;
        }
        numbytes--;
        if(data_packet_extract_one_byte(&HBD_Data_Comm_Packet, this_byte) == DATA_PACKET_SUCCESS) {
            if(HBD_Data_Comm_Packet.RxReady == 1) {
                // Double checked and good to go
                HBD_Data_Comm_Process_Command();
            }
        }
    }
}

/**
 * @brief  HBD Data Communications Process Command
 *         Calls the command processor when a packet has been successfully
 *         decoded. If a response is generated, it is sent back over the
 *         HBD serial port.
 * @param  None
 * @retval None
 */
static void HBD_Data_Comm_Process_Command(void) {
    uint16_t errCode = data_process_command(&HBD_Data_Comm_Packet);
    if ((errCode == DATA_PACKET_SUCCESS) && HBD_Data_Comm_Packet.TxReady) {
        uint8_t* send_buffer = HBD_Data_Comm_Packet.TxBuffer;
        uint16_t len_to_send = HBD_Data_Comm_Packet.TxLength;
        uint16_t actually_sent = 0;
        while (len_to_send > 0) {
            actually_sent = UART_Write(SELECT_HBD_UART, send_buffer, len_to_send);
            len_to_send -= actually_sent;
            send_buffer += actually_sent;
        }
        HBD_Data_Comm_Packet.TxReady = 0;
    }
}
