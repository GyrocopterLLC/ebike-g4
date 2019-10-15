/******************************************************************************
 * Filename: bms_data_comm.c
 * Description: Conducts data packet communication with Battery Management
 *              System (BMS) boards connected to each battery pack.
 *
 *              The BMS boards are daisy-chained together so that only one
 *              port is needed to communicated with all boards. Galvanically
 *              isolated communication is required since the batteries are
 *              likely connected in series.
 *
 *              BMS communications have a slight change to the normal packet
 *              format: the BMS board address is always given as the first
 *              data byte. This means there cannot be any zero-data packet.
 *
 *              Boards power on with address zero. They do not issue commands
 *              down the daisy-chain until they have been assigned a non-zero
 *              address. The exception to this rule occurs when the broadcast
 *              address (0xFF) is the destination. Then, all boards will
 *              copy the message downstream and will not reply upstream. This
 *              controller in charge of BMS boards can set addresses
 *              sequentially by requesting a response from the board with
 *              address 0, setting its address to a non-zero value, and then
 *              searching for address 0 again. Repeat until address 0 no longer
 *              responds.
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
#include "bms_data_comm.h"
#include "data_packet.h"
#include "data_commands.h"

uint8_t BMS_Packet_Buffer[PACKET_MAX_DATA_LENGTH+PACKET_OVERHEAD_BYTES];


uint8_t BMS_Num_Connected_Boards; // Equal to the address of the last board in the chain
BMS_Type* BMS_Board_Handles;

uint8_t BMS_Refresh_All(Data_Packet_Type* pkt) {
    (void)pkt;
#if 0
    // Reinitialize boards
    pkt->Data[0] = BROADCAST_ADDRESS;
    pkt->Data[1] = R_ADDRESS;
    pkt->Data[2] = 0;
    data_packet_create(pkt, SET_RAM_VARIABLE, pkt->Data, 3);
    BMS_Send(pkt->TxBuffer,pkt->TxLength);

    // Set addresses
    while(1) {
        // Feed watchdog
        WDT_feed();

        // Send for address 0
        pkt->Data[0] = 0;
        pkt->Data[1] = R_ADDRESS;
        data_packet_create(pkt, GET_RAM_VARIABLE, pkt->Data, 2);
        BMS_Send(pkt->TxBuffer, pkt->TxLength);
        time_at_start = Time_Now();
        while(Time_Now() < time_at_start + timeout) {
            // Get new data
            BMS_Read(all_the_data);
            BMS_Send_To_Processor(pkt);
            if(pkt->PacketType == BMS_ACK) {
                // Set this guy's address to non-zero
                BMS_Num_Connected_Boards++;
                pkt->Data[0] = 0;
                pkt->Data[1] = R_ADDRESS;
                pkt->Data[2] = BMS_Num_Connected_Boards;
                data_packet_create(pkt, SET_RAM_VARIABLE, pkt->Data, 3);
                BMS_Send(pkt->TxBuffer,pkt->TxLength);
            }
        }
    }
#endif
    return 0;
}
