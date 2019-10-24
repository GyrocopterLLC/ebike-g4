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

#include <stdlib.h>
#include "main.h"
#include "bms_data_comm.h"
#include "data_packet.h"
#include "data_commands.h"

uint8_t BMS_Tx_Buffer[BMS_TX_BUFFER_LENGTH];
uint8_t BMS_Packet_Buffer[PACKET_MAX_DATA_LENGTH];
Data_Packet_Type BMS_Packet;

BMS_Type bms;

static void BMS_Process_Command(void);
static void BMS_Process_Timeout(void);
inline static void BMS_Start_Timeout(uint32_t timeout_ms);
inline static void BMS_Stop_Timeout(void);
static void BMS_Timeout_Check(void);
static uint32_t BMS_Batt_Array_Position(uint8_t board, uint8_t batt);

/**
 * @brief  BMS UART Data Communications Initialization
 *         Sets packet data to default values.
 * @param  None
 * @retval None
 */
void BMS_Data_Comm_Init(void) {
    BMS_Packet.State = DATA_COMM_IDLE;
    BMS_Packet.Data = BMS_Packet_Buffer;
    BMS_Packet.TxBuffer = BMS_Tx_Buffer;
    BMS_Packet.TxReady = 0;
    BMS_Packet.RxReady = 0;

    bms.NumConnectedBoards = 0;
    bms.BattStatuses = 0; // Null pointer = unallocated
    bms.BattVoltages = 0;
    bms.Timeout = 0; // Set timeout inactive
}

// For debugging purposes.
void BMS_Send_One_Packet(uint8_t pktType, uint8_t* data, uint8_t datalen) {
    memcpy(BMS_Packet.Data, data, datalen);
    data_packet_create(&BMS_Packet, pktType, BMS_Packet.Data, datalen);
    UART_Write(SELECT_BMS_UART, BMS_Packet.TxBuffer, BMS_Packet.TxLength);
}

/**
 * @brief  BMS Restart Chain
 *         Refreshes all addresses of connected BMS boards. Can be used
 *         straight out of power on reset or whenever needed.
 * @param  None
 * @retval None
 */
void BMS_Restart_Chain(void) {
    bms.NumConnectedBoards = 0;
    memset(bms.BattsPerBoard, 0, sizeof(bms.BattsPerBoard));
    // Reinitialize boards. Set all addresses to zero. No reply from BMS boards.
    BMS_Packet.Data[0] = BROADCAST_ADDRESS;
    data_packet_pack_16b(&(BMS_Packet.Data[1]),R_ADDRESS);
    BMS_Packet.Data[3] = 0;

    if(data_packet_create(&BMS_Packet, SET_RAM_VARIABLE, BMS_Packet.Data, 4)
            == DATA_PACKET_SUCCESS) {
        UART_Write(SELECT_BMS_UART, BMS_Packet.TxBuffer, BMS_Packet.TxLength);
    }

    // We can add a wait state here, but immediately going into a new packet
    // is okay. It will just be queued up anyway in the Tx buffer.

    BMS_Packet.Data[0] = 0;
    data_packet_pack_16b(&(BMS_Packet.Data[1]),R_ADDRESS);
    BMS_Packet.Data[3] = bms.NumConnectedBoards + 1;
    if(data_packet_create(&BMS_Packet, SET_RAM_VARIABLE, BMS_Packet.Data, 4)
            == DATA_PACKET_SUCCESS) {
        bms.RetryCount = 0;
        UART_Write(SELECT_BMS_UART, BMS_Packet.TxBuffer, BMS_Packet.TxLength);
        // Start the timeout to check if BMS has failed or disconnected
        BMS_Start_Timeout(BMS_TIMEOUT_MS);
        bms.CommState = BMS_STATE_ADDRESSING;
    }
    // Wait for response. It will show up in BMS_Periodic_Check
}

/**
 * @brief  BMS Refresh Data
 *         Gets the voltage and status word for each of the batteries measured
 *         by the connected BMS boards.
 * @param  None
 * @retval None
 */
void BMS_Refresh_Data(void) {

    // Set up the counters
    bms.CurrentBoard = 1;
    bms.CurrentBatt = 1;

    // Request the first battery voltage
    BMS_Packet.Data[0] = bms.CurrentBoard;
    data_packet_pack_16b(&(BMS_Packet.Data[1]),R_VOLT_BATT1);
    if( data_packet_create(&BMS_Packet, GET_RAM_VARIABLE, BMS_Packet.Data, 3)
            == DATA_PACKET_SUCCESS) {
        bms.RetryCount = 0;
        UART_Write(SELECT_BMS_UART, BMS_Packet.TxBuffer, BMS_Packet.TxLength);
        bms.CommState = BMS_STATE_VOLTAGES;
        // Start the timeout to check if BMS has failed or disconnected
        BMS_Start_Timeout(BMS_TIMEOUT_MS);
    }

    // Wait for response. It will show up in BMS_Periodic_Check
}

uint8_t BMS_Busy(void) {
    if(bms.CommState == BMS_STATE_IDLE) {
        return 0;
    }
    return 1;
}

/**
 * @brief  BMS Data Communications One Byte Check
 *         Handles the USB serial port incoming data. Determines
 *         if a properly encoded packet has been received, and
 *         sends to the appropriate handler if it has. Operates
 *         one byte at a time using an internal state machine.
 * @param  None
 * @retval None
 */
void BMS_OneByte_Check(void) {
    // Run the "timer" to see if we have timed out for the next packet
    BMS_Timeout_Check();
    // Loop through each incoming byte
    int32_t numbytes = UART_InWaiting(SELECT_BMS_UART);
    uint8_t this_byte;
    while(numbytes > 0) {
        // Load one new byte
        if(UART_Read(SELECT_BMS_UART, &this_byte, 1) != 1) {
            // Check that a byte was really received
            return;
        }
        numbytes--;
        if(data_packet_extract_one_byte(&BMS_Packet, this_byte) == DATA_PACKET_SUCCESS) {
            if(BMS_Packet.RxReady == 1) {
                // Double checked and good to go
                BMS_Process_Command();
            }
        }
    }
}

#if 0
/**
 * @brief  BMS UART Data Communications Periodic Check
 *         Handles the BMS serial port incoming data. Determines
 *         if a properly encoded packet has been received, and
 *         sends to the appropriate handler if it has. Clears
 *         buffer when necessary to accept more data.
 *
 *         Note: when new data arrives to a full buffer, the
 *         existing data is cleared one byte at a time, oldest
 *         to newest. The assumption is made that the buffer is
 *         larger than the largest possible packet, so there is
 *         no possible way to interrupt a packet in transmission
 *         by accidentally deleting part of it.
 * @param  None
 * @retval None
 */
void BMS_Periodic_Check(void) {
    // Run the "timer" to see if we have timed out
    BMS_Timeout_Check();

    // check if data is available
    int32_t numbytes = UART_InWaiting(SELECT_BMS_UART);
    uint16_t pkt_end;
    if (numbytes <= 0) {
        return;
    }
    // Do we have space?
    int32_t space_remaining = BMS_RX_BUFFER_LENGTH - BMS_RxBuffer_WrPlace;
    while (numbytes > space_remaining) {
        // Copy as much as we can.
        UART_Read(SELECT_BMS_UART, BMS_Rx_Buffer + BMS_RxBuffer_WrPlace,
                space_remaining);
        // Check if any valid packets exist in the whole buffer
        data_packet_extract(&BMS_Packet, BMS_Rx_Buffer,
        BMS_RX_BUFFER_LENGTH);
        if (BMS_Packet.RxReady) {
            // If yes, process the packet and then move the buffer to make some room
            BMS_Process_Command();
            pkt_end = (BMS_Packet.StartPosition + BMS_Packet.DataLength
                    + PACKET_OVERHEAD_BYTES);
            memmove(BMS_Rx_Buffer, BMS_Rx_Buffer + pkt_end,
            BMS_RX_BUFFER_LENGTH - pkt_end);
            BMS_RxBuffer_WrPlace = BMS_RX_BUFFER_LENGTH - pkt_end;
            space_remaining = pkt_end;
        } else {
            // If no, discard one byte from the front, making one space available
            memmove(BMS_Rx_Buffer, BMS_Rx_Buffer + 1,
            BMS_RX_BUFFER_LENGTH - 1);
            space_remaining = 1;
            BMS_RxBuffer_WrPlace = BMS_RX_BUFFER_LENGTH - 1;
        }
        // How much is left now?
        numbytes = UART_InWaiting(SELECT_BMS_UART);
    }

    UART_Read(SELECT_BMS_UART, BMS_Rx_Buffer + BMS_RxBuffer_WrPlace, numbytes);
    BMS_RxBuffer_WrPlace += numbytes;
    // Check if we have a packet
    data_packet_extract(&BMS_Packet, BMS_Rx_Buffer, BMS_RxBuffer_WrPlace);
    if (BMS_Packet.RxReady) {
        // Do the thing commanded
        BMS_Process_Command();
        // And free up space
        pkt_end = (BMS_Packet.StartPosition + BMS_Packet.DataLength
                + PACKET_OVERHEAD_BYTES);
        memmove(BMS_Rx_Buffer, BMS_Rx_Buffer + pkt_end,
        BMS_RX_BUFFER_LENGTH - pkt_end);
        BMS_RxBuffer_WrPlace -= pkt_end;
    }
}

#endif

static void BMS_Process_Command(void) {

    // Depends on what state we're in
    switch (bms.CommState) {
    case BMS_STATE_IDLE:
        // In this case, the packet is probably an interrupt.
        // Might be an fault on the BMS board.
        break;
    case BMS_STATE_ADDRESSING:
        // Expecting either an ACK packet or the number of cells on this board
        if (BMS_Packet.PacketType == CONTROLLER_ACK) {
            // Good, it has been addressed. We can increase the number of boards
            bms.NumConnectedBoards++;
            // Ask for the batteries on this board
            BMS_Packet.Data[0] = bms.NumConnectedBoards;
            data_packet_pack_16b(&(BMS_Packet.Data[1]), R_NUM_BATTS);
            if (data_packet_create(&BMS_Packet, GET_RAM_VARIABLE,
                    BMS_Packet.Data, 3) == DATA_PACKET_SUCCESS) {
                bms.RetryCount = 0;
                UART_Write(SELECT_BMS_UART, BMS_Packet.TxBuffer,
                        BMS_Packet.TxLength);
                BMS_Start_Timeout(BMS_TIMEOUT_MS);
            }

        } else if (BMS_Packet.PacketType == GET_RAM_RESULT) {
            // Now we know the number of batteries on this board.
            bms.BattsPerBoard[bms.NumConnectedBoards-1] = data_packet_extract_16b(
                    BMS_Packet.Data);
            // See if there's another board connected by repeating the addressing
            // command to address zero
            BMS_Packet.Data[0] = 0;
            data_packet_pack_16b(&(BMS_Packet.Data[1]), R_ADDRESS);
            BMS_Packet.Data[3] = bms.NumConnectedBoards + 1;
            if (data_packet_create(&BMS_Packet, SET_RAM_VARIABLE,
                    BMS_Packet.Data, 4) == DATA_PACKET_SUCCESS) {
                bms.RetryCount = 0;
                UART_Write(SELECT_BMS_UART, BMS_Packet.TxBuffer,
                        BMS_Packet.TxLength);
                BMS_Start_Timeout(BMS_TIMEOUT_MS);
            }

        } else {
            // Maybe a NACK or something else. Regardless it's not supposed to happen
            // Retry last packet, or fail if already at max retries.
            if(bms.RetryCount >= BMS_MAX_RETRIES) {
                bms.CommState = BMS_STATE_IDLE;
                MAIN_SetError(MAIN_FAULT_BMS_COMM);
            } else {
                bms.RetryCount++;
                // Send the same packet again.
                UART_Write(SELECT_BMS_UART, BMS_Packet.TxBuffer,
                        BMS_Packet.TxLength);
                BMS_Start_Timeout(BMS_TIMEOUT_MS);
            }
        }
        break;
    case BMS_STATE_VOLTAGES:
        if (BMS_Packet.PacketType == GET_RAM_RESULT) {
            // Should be the voltage for this cell
            bms.BattVoltages[BMS_Batt_Array_Position(bms.CurrentBoard,
                    bms.CurrentBatt)] = ((float) data_packet_extract_32b(
                    BMS_Packet.Data)) / 65536.0f; // Voltage is in Q16 format
            // What's the next one to check?
            if (bms.CurrentBatt == bms.BattsPerBoard[bms.CurrentBoard-1]) {
                if (bms.CurrentBoard == bms.NumConnectedBoards) {
                    // Done! Move on to statuses
                    bms.CurrentBoard = 1;
                    bms.CurrentBatt = 1;
                    BMS_Packet.Data[0] = bms.CurrentBoard;
                    data_packet_pack_16b(&BMS_Packet.Data[1], R_STAT_BATT1);
                    if (data_packet_create(&BMS_Packet, GET_RAM_VARIABLE,
                            BMS_Packet.Data, 3) == DATA_PACKET_SUCCESS) {
                        bms.RetryCount = 0;
                        bms.CommState = BMS_STATE_STATUSES;
                        UART_Write(SELECT_BMS_UART, BMS_Packet.TxBuffer,
                                BMS_Packet.TxLength);
                        BMS_Start_Timeout(BMS_TIMEOUT_MS);
                    }

                } else {
                    // Move to the next board
                    bms.CurrentBoard++;
                    bms.CurrentBatt = 1;
                    BMS_Packet.Data[0] = bms.CurrentBoard;
                    data_packet_pack_16b(&BMS_Packet.Data[1], R_VOLT_BATT1);
                    if (data_packet_create(&BMS_Packet, GET_RAM_VARIABLE,
                            BMS_Packet.Data, 3) == DATA_PACKET_SUCCESS) {
                        bms.RetryCount = 0;
                        UART_Write(SELECT_BMS_UART, BMS_Packet.TxBuffer,
                                BMS_Packet.TxLength);
                        BMS_Start_Timeout(BMS_TIMEOUT_MS);
                    }

                }
            } else {
                // More batteries on this board
                bms.CurrentBatt++;
                BMS_Packet.Data[0] = bms.CurrentBoard;
                data_packet_pack_16b(&BMS_Packet.Data[1],
                R_VOLT_BATT1 + bms.CurrentBatt - 1);
                if (data_packet_create(&BMS_Packet, GET_RAM_VARIABLE,
                        BMS_Packet.Data, 3) == DATA_PACKET_SUCCESS) {
                    bms.RetryCount = 0;
                    UART_Write(SELECT_BMS_UART, BMS_Packet.TxBuffer,
                            BMS_Packet.TxLength);
                    BMS_Start_Timeout(BMS_TIMEOUT_MS);
                }

            }
        } else {
            // Maybe a NACK or something else. Regardless it's not supposed to happen
            // Retry last packet, or fail if already at max retries.
            if(bms.RetryCount >= BMS_MAX_RETRIES) {
                bms.CommState = BMS_STATE_IDLE;
                MAIN_SetError(MAIN_FAULT_BMS_COMM);
            } else {
                bms.RetryCount++;
                // Send the same packet again.
                UART_Write(SELECT_BMS_UART, BMS_Packet.TxBuffer,
                        BMS_Packet.TxLength);
                BMS_Start_Timeout(BMS_TIMEOUT_MS);
            }
        }
        break;
    case BMS_STATE_STATUSES:
        if (BMS_Packet.PacketType == GET_RAM_RESULT) {
            // Should be the status for this cell
            bms.BattStatuses[BMS_Batt_Array_Position(bms.CurrentBoard,
                    bms.CurrentBatt)] = data_packet_extract_32b(
                    BMS_Packet.Data); // Status is raw U32
            // What's the next one to check?
            if (bms.CurrentBatt == bms.BattsPerBoard[bms.CurrentBoard-1]) {
                if (bms.CurrentBoard == bms.NumConnectedBoards) {
                    // Done!
                    bms.CommState = BMS_STATE_IDLE;
                    BMS_Stop_Timeout();
                } else {
                    // Move to the next board
                    bms.CurrentBoard++;
                    bms.CurrentBatt = 1;
                    BMS_Packet.Data[0] = bms.CurrentBoard;
                    data_packet_pack_16b(&BMS_Packet.Data[1], R_STAT_BATT1);
                    if (data_packet_create(&BMS_Packet, GET_RAM_VARIABLE,
                            BMS_Packet.Data, 3) == DATA_PACKET_SUCCESS) {
                        bms.RetryCount = 0;
                        UART_Write(SELECT_BMS_UART, BMS_Packet.TxBuffer,
                                BMS_Packet.TxLength);
                        BMS_Start_Timeout(BMS_TIMEOUT_MS);
                    }

                }
            } else {
                // More batteries on this board
                bms.CurrentBatt++;
                BMS_Packet.Data[0] = bms.CurrentBoard;
                data_packet_pack_16b(&BMS_Packet.Data[1],
                R_STAT_BATT1 + bms.CurrentBatt - 1);
                if (data_packet_create(&BMS_Packet, GET_RAM_VARIABLE,
                        BMS_Packet.Data, 3) == DATA_PACKET_SUCCESS) {
                    bms.RetryCount = 0;
                    UART_Write(SELECT_BMS_UART, BMS_Packet.TxBuffer,
                            BMS_Packet.TxLength);
                    BMS_Start_Timeout(BMS_TIMEOUT_MS);
                }

            }
        } else {
            // Maybe a NACK or something else. Regardless it's not supposed to happen
            // Retry last packet, or fail if already at max retries.
            if(bms.RetryCount >= BMS_MAX_RETRIES) {
                bms.CommState = BMS_STATE_IDLE;
                MAIN_SetError(MAIN_FAULT_BMS_COMM);
            } else {
                bms.RetryCount++;
                // Send the same packet again.
                UART_Write(SELECT_BMS_UART, BMS_Packet.TxBuffer,
                        BMS_Packet.TxLength);
                BMS_Start_Timeout(BMS_TIMEOUT_MS);
            }
        }

        break;
    }
    BMS_Packet.RxReady = 0;

}

/**
 * @brief  BMS Process Timeout
 *         This function is called whenever the timeout has occurred when
 *         waiting for a response from a BMS. The action to take after a
 *         timeout depends on the communication state.
 * @param  None
 * @retval None
 */
static void BMS_Process_Timeout(void) {
    uint32_t numTotalBatts;
    switch(bms.CommState) {
    case BMS_STATE_IDLE:
        // This shouldn't have been a timeout!
        break;
    case BMS_STATE_ADDRESSING:
        // How many retries so far?
        if(bms.RetryCount >= BMS_MAX_RETRIES) {
            // That's the end of the chain. If there were no BMS boards
            // at all, this is a failure.
            bms.CommState = BMS_STATE_IDLE;
            if(bms.NumConnectedBoards == 0) {
                MAIN_SetError(MAIN_FAULT_BMS_COMM);
            } else {
                // Need to initialize the dynamically allocated arrays
                numTotalBatts = 0;
                for(uint8_t i = 0; i < bms.NumConnectedBoards; i++) {
                    numTotalBatts += bms.BattsPerBoard[i];
                }
                if(bms.BattVoltages != 0) {
                    free(bms.BattVoltages);
                    bms.BattVoltages = 0;
                }
                bms.BattVoltages = malloc(sizeof(float)*numTotalBatts);
                if(bms.BattStatuses != 0) {
                    free(bms.BattStatuses);
                    bms.BattStatuses = 0;
                }
                bms.BattStatuses = malloc(sizeof(uint32_t)*numTotalBatts);
            }
        } else {
            bms.RetryCount++;
            if (bms.NumConnectedBoards == 0) {
                // If still waiting for the first board, try the broadcast
                // reset again. Then follow up with setting the first address.
                BMS_Packet.Data[0] = BROADCAST_ADDRESS;
                data_packet_pack_16b(&(BMS_Packet.Data[1]), R_ADDRESS);
                BMS_Packet.Data[3] = 0;

                if (data_packet_create(&BMS_Packet, SET_RAM_VARIABLE,
                        BMS_Packet.Data, 4) == DATA_PACKET_SUCCESS) {
                    UART_Write(SELECT_BMS_UART, BMS_Packet.TxBuffer,
                            BMS_Packet.TxLength);
                }
                BMS_Packet.Data[0] = 0;
                data_packet_pack_16b(&(BMS_Packet.Data[1]), R_ADDRESS);
                BMS_Packet.Data[3] = bms.NumConnectedBoards + 1;
                if (data_packet_create(&BMS_Packet, SET_RAM_VARIABLE,
                        BMS_Packet.Data, 4) == DATA_PACKET_SUCCESS) {
                    UART_Write(SELECT_BMS_UART, BMS_Packet.TxBuffer,
                            BMS_Packet.TxLength);
                    BMS_Start_Timeout(BMS_TIMEOUT_MS);
                }


            } else {
                // Send the same packet again.
                UART_Write(SELECT_BMS_UART, BMS_Packet.TxBuffer,
                        BMS_Packet.TxLength);
                BMS_Start_Timeout(BMS_TIMEOUT_MS);
            }
        }
        break;
    case BMS_STATE_VOLTAGES:
    case BMS_STATE_STATUSES:
        // How many retries so far?
        if (bms.RetryCount >= BMS_MAX_RETRIES) {
            bms.CommState = BMS_STATE_IDLE;
            // Failure!
            MAIN_SetError(MAIN_FAULT_BMS_COMM);
        } else {
            bms.RetryCount++;
            // Retry the last packet
            UART_Write(SELECT_BMS_UART, BMS_Packet.TxBuffer,
                    BMS_Packet.TxLength);
            BMS_Start_Timeout(BMS_TIMEOUT_MS);
        }
        break;
    }
}

inline static void BMS_Start_Timeout(uint32_t timeout_ms) {
    // Not using a timer, so the timeout "interrupt" can be called outside
    // of an interrupt context.
    bms.TimeoutStart = GetTick();
    bms.Timeout = timeout_ms;
}

inline static void BMS_Stop_Timeout(void) {
    bms.Timeout = 0;
}

static void BMS_Timeout_Check(void) {
    if (bms.Timeout != 0) {
        // Zero means inactive.
        if ((GetTick() - bms.TimeoutStart) > bms.Timeout) {
            // Timeout has expired!
            bms.Timeout = 0;
            BMS_Process_Timeout();
        }
    }
}

static uint32_t BMS_Batt_Array_Position(uint8_t board, uint8_t batt) {
    uint32_t battCounter = 0;
    // What is our position in the array?
    // Increment counter by the number of batteries on each previous board
    for (uint8_t i = 0; i < (board - 1); i++) {
        battCounter += bms.BattsPerBoard[i];
    }
    // Increment by the battery number for THIS board
    battCounter += (batt - 1);
    return battCounter;
}
