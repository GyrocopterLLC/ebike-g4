/******************************************************************************
 * Filename: usb.c
 * Description: Low-level driver for the Universal Serial Bus (USB) device.
 *              Aimed for the USB device peripheral on the STM32G4 series
 *              microcontrollers.
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

// Shortcut to the Packet Memory Area at offset 0x400
#define USB_PMA_BASE_ADDR                       ((uint32_t) (USB_BASE + 0x400U))
#define USB_BTABLE_ADDR                         ((uint16_t) 0U)
#define USB_BTABLE_SIZE                         ((uint16_t) 0x40U)
#define USB_EPnR(EpNum)                         (*(volatile uint16_t *)(USB_BASE + ((uint32_t)EpNum * 4U)))
#define USB_Set_EPnR(EpNum, RegVal)             (*(volatile uint16_t *)(USB_BASE + ((uint32_t)EpNum * 4U)) = (uint16_t)(RegVal))
#define USB_Get_EPnR(EpNum)                     (*(volatile uint16_t *)(USB_BASE + ((uint32_t)EpNum * 4U)))

#define USB_EnableGlobalInterrupts()            USB->CNTR |= (uint16_t)(USB_CNTR_CTRM | USB_CNTR_WKUPM \
                                                                        | USB_CNTR_SUSPM | USB_CNTR_ERRM \
                                                                        | USB_CNTR_SOFM | USB_CNTR_ESOFM \
                                                                        | USB_CNTR_RESETM |USB_CNTR_L1REQM)

#define USB_DisableGlobalInterrupts()           USB->CNTR &= (uint16_t)(~(USB_CNTR_CTRM | USB_CNTR_WKUPM \
                                                                        | USB_CNTR_SUSPM | USB_CNTR_ERRM \
                                                                        | USB_CNTR_SOFM | USB_CNTR_ESOFM \
                                                                        | USB_CNTR_RESETM |USB_CNTR_L1REQM))


USB_EndpointType USB_InEPs[NUM_ENDPOINTS];
USB_EndpointType USB_OutEPs[NUM_ENDPOINTS];

// State registers and whatnot
uint32_t USB_Setup_Buffer[12];
USB_SetupReqTypedef USB_SetupRequest;
USB_ClassDescTypedef* USB_ClassDescData;
USB_ClassCallbackTypedef* USB_ClassCallbackData;
USB_FIFOStatusTypedef USB_FIFO_Status;
uint8_t USB_EP0_State;
uint16_t USB_EP0_DataLen;
uint8_t USB_NewAddr;
uint8_t USB_DevState;
uint32_t USB_CurrentConfig;
uint32_t USB_ConfigStatus;
uint8_t USB_OUTEP0_Buffer[USB_MAX_EP0_SIZE * 2];

static void USB_CoreReset(void) {
    // Disable USB interrupts
    USB_DisableGlobalInterrupts();

    // Power down and reset the core
    USB->CNTR |= (uint16_t)(USB_CNTR_FRES | USB_CNTR_PDWN);

    // Delay to let reset take effect
    Delay(2);

    // Clear the power down
    USB->CNTR &= (uint16_t)(~(USB_CNTR_PDWN));

    // Delay to allow for startup of regulators
    Delay(2);

    // Clear the reset
    USB->CNTR &= (uint16_t)(~(USB_CNTR_FRES));

    // Clear pending interrupts
    USB->ISTR = (uint16_t) 0U;

    // Set the btable address
    USB->BTABLE = (uint16_t) USB_BTABLE_ADDR;
}

static void USB_Set_PMA_Endpoint_Tx_Address(uint16_t EpNum, uint16_t Addr) {
    volatile uint16_t* reg = (uint16_t*)((uint32_t)USB_PMA_BASE_ADDR
            + (uint32_t)(USB->BTABLE)
            + (uint32_t)(((EpNum) * 8U)));

    *reg = (uint16_t)(Addr & 0xFFFE);
}

static void USB_Set_PMA_Endpoint_Tx_Count(uint16_t EpNum, uint16_t Count) {
    volatile uint16_t* reg = (uint16_t*)((uint32_t)USB_PMA_BASE_ADDR
            + (uint32_t)(USB->BTABLE)
            + (uint32_t)(((EpNum) * 8U) + 2U));
    *reg = Count & (0x3FFU); // Can't be bigger than 1023 per USB spec.
}

static void USB_Set_PMA_Endpoint_Rx_Address(uint16_t EpNum, uint16_t Addr) {
    volatile uint16_t* reg = (uint16_t*)((uint32_t)USB_PMA_BASE_ADDR
            + (uint32_t)(USB->BTABLE)
            + (uint32_t)(((EpNum) * 8U) + 4U));

    *reg = (uint16_t)(Addr & 0xFFFE);
}

static void USB_Set_PMA_Endpoint_Rx_BufSize(uint16_t EpNum, uint16_t Count) {
    volatile uint16_t* reg = (uint16_t*)((uint32_t)USB_PMA_BASE_ADDR
            + (uint32_t)(USB->BTABLE)
            + (((uint32_t)(EpNum) * 8U) + 6U));
    if(Count == 0) {
        // Zero bytes isn't allowed. Default to 64.
        *reg = 0x8000 | (1U << 10);
    } else if (Count <= 62) {
        // BLSIZE = 0, size is count/2
        uint16_t reg_count = Count >> 1;
        if((Count & 0x01) != 0) {
            // Round up
            reg_count++;
        }
        *reg = (uint16_t)(reg_count << 10);
    } else {
        // BLSIZE = 1, size is (count / 32) - 1
        uint16_t reg_count = Count >> 5;
        if((Count & 0x1F) != 0) {
            reg_count++;
        }
        *reg = (uint16_t)(0x8000 | ((reg_count-1) << 10));
    }
}

static uint16_t USB_Get_PMA_Endpoint_Rx_Count(uint16_t EpNum) {
    volatile uint16_t* reg = (uint16_t*)((uint32_t)USB_PMA_BASE_ADDR
            + (uint32_t)(USB->BTABLE)
            + (((uint32_t)(EpNum) * 8U) + 6U));

    return (*reg) & 0x3FFU; // Bits 9:0
}

static void USB_Set_EPnR_Tx_Stat(uint16_t EpNum, uint16_t NewStat) {
    uint16_t Non_Toggle_Bits, Tx_Stat;
    // Read and save the bits that aren't going to be toggled
    Non_Toggle_Bits = USB_Get_EPnR(EpNum) & USB_EPREG_MASK;
    // Check the current Stat setting
    Tx_Stat = USB_Get_EPnR(EpNum) & USB_EPTX_STAT;
    if(Tx_Stat != NewStat) {
        // Using XOR to flip the bits that are different.
        // For the Stat field, writing a '1' bit toggles the value.
        USB_Set_EPnR(EpNum, Non_Toggle_Bits | (Tx_Stat ^ NewStat));
    }
}

static void USB_Set_EPnR_Rx_Stat(uint16_t EpNum, uint16_t NewStat) {
    uint16_t Non_Toggle_Bits, Rx_Stat;
    // Read and save the bits that aren't going to be toggled
    Non_Toggle_Bits = USB_Get_EPnR(EpNum) & USB_EPREG_MASK;
    // Check the current Stat setting
    Rx_Stat = USB_Get_EPnR(EpNum) & USB_EPRX_STAT;
    if(Rx_Stat != NewStat) {
        // Using XOR to flip the bits that are different.
        // For the Stat field, writing a '1' bit toggles the value.
        USB_Set_EPnR(EpNum, Non_Toggle_Bits | (Rx_Stat ^ NewStat));
    }
}

static void USB_Clear_EPnR_Tx_CTR(uint16_t EpNum) {
    uint16_t Non_Toggle_Bits;
    // Read and save the bits that aren't going to be toggled
    // EXCEPT FOR USB_EP_TX_CTR. So we can't use the USB_EPREG_MASK definition here.
    Non_Toggle_Bits = USB_Get_EPnR(EpNum) & (USB_EP_CTR_RX|USB_EP_SETUP|USB_EP_T_FIELD|USB_EP_KIND|USB_EPADDR_FIELD);
    USB_Set_EPnR(EpNum, Non_Toggle_Bits);
}

static void USB_Clear_EPnR_Rx_CTR(uint16_t EpNum) {
    uint16_t Non_Toggle_Bits;
    // Read and save the bits that aren't going to be toggled
    // EXCEPT FOR USB_EP_RX_CTR. So we can't use the USB_EPREG_MASK definition here.
    Non_Toggle_Bits = USB_Get_EPnR(EpNum) & (USB_EP_SETUP|USB_EP_T_FIELD|USB_EP_KIND|USB_EP_CTR_TX|USB_EPADDR_FIELD);
    USB_Set_EPnR(EpNum, Non_Toggle_Bits);
}

void USB_Init(void) {
    // Initialize clocks
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN; // GPIO clock
    RCC->APB1ENR1 |= RCC_APB1ENR1_USBEN; // USB clock

    // Set port pins for USB to analog
    GPIO_Analog(USB_PORT, USB_DM_PIN);
    GPIO_Analog(USB_PORT, USB_DP_PIN);

    // Enable interrupt
    NVIC_SetPriority(USB_LP_IRQn, PRIO_USB);
    NVIC_EnableIRQ(USB_LP_IRQn);

    // USB Core reset
    USB_CoreReset();

    // Stop the USB PHY by disabling pull-up/-down resistors on D+/D- pins
    USB->BCDR &= (uint16_t)(~USB_BCDR_DPPU);

    USB_ActivateINEP(0, USB_EP_TYPE_CTRL, USB_MAX_EP0_SIZE, 2*USB_MAX_EP0_SIZE);
    USB_ActivateOUTEP(0, USB_EP_TYPE_CTRL, USB_MAX_EP0_SIZE, 2*USB_MAX_EP0_SIZE);

    USB_ClassDescData = NULLPTR;
    USB_ClassCallbackData = NULLPTR;
    USB_DevState = USB_STATE_DEFAULT;
    USB_CurrentConfig = 0;
    USB_NewAddr = 0;
}

void USB_SetClass(USB_ClassDescTypedef* newclassdesc,
        USB_ClassCallbackTypedef* newclasscalls) {
    USB_ClassDescData = newclassdesc;
    USB_ClassCallbackData = newclasscalls;
}

void USB_Start(void) {
    // Start the USB PHY by enabling pull-up resistor on D+ pin
    USB->BCDR |= USB_BCDR_DPPU;
    // Allow all functions to respond and reset address to zero
    USB->DADDR = USB_DADDR_EF;

    // Globally enable USB interrupts
    USB_EnableGlobalInterrupts();
}

uint8_t USB_GetDevState(void) {
    return USB_DevState;
}

void USB_IRQ(void) {
    // Check the source of the interrupt
    if((USB->ISTR & USB_ISTR_CTR) !=0 ) {
        // Correct Transfer interrupt
        // Flag will be cleared in the endpoint register
        USB_CorrectTransfer_IRQ();
    }
    if((USB->ISTR & USB_ISTR_PMAOVR) != 0) {
        // Packet memory area overrun interrupt
        USB->ISTR &= (~USB_ISTR_PMAOVR);
    }
    if((USB->ISTR &= USB_ISTR_ERR) != 0) {
        // Error interrupt
        USB->ISTR &= (~USB_ISTR_ERR);
    }
    if((USB->ISTR &= USB_ISTR_WKUP) != 0) {
        // Wakeup interrupt
        USB->ISTR &= (~USB_ISTR_WKUP);

    }
    if((USB->ISTR &= USB_ISTR_SUSP) != 0) {
        // Suspend interrupt
        USB->ISTR &= (~USB_ISTR_SUSP);
    }
    if((USB->ISTR & USB_ISTR_RESET) != 0) {
        // Reset interrupt
        USB->ISTR &= (~USB_ISTR_RESET);
        USB_Reset_IRQ();
    }
    if((USB->ISTR & USB_ISTR_SOF) != 0) {
        // Start of Frame interrupt
        USB->ISTR &= (~USB_ISTR_SOF);
    }
    if((USB->ISTR & USB_ISTR_ESOF) != 0) {
        // Expected Start of Frame interrupt
        USB->ISTR &= (~USB_ISTR_ESOF);
    }
    if((USB->ISTR & USB_ISTR_L1REQ) != 0) {
        // Low power mode request interrupt
        USB->ISTR &= (~USB_ISTR_L1REQ);
    }
}

// Handles either a completed IN or OUT packet on any endpoint
void USB_CorrectTransfer_IRQ(void) {
    uint16_t freeze_istr;
    uint16_t active_ep;
    uint16_t active_epnr;

    // If multiple endpoints have interrupts ready, loop through them all
    // No need to exit the IRQ until all are cleared, we'd just come
    // right back anyway
    while((USB->ISTR & USB_ISTR_CTR) != 0 ) {
        freeze_istr = USB->ISTR; // Keep a local copy in case weird stuff changes the register

        // Check which endpoint generated this interrupt
        active_ep = freeze_istr & USB_ISTR_EP_ID;
        active_epnr = USB_Get_EPnR(active_ep);
        // Check the control endpoint (EP0) first
        if(active_ep == 0) {
            // Which direction?
            if((freeze_istr & USB_ISTR_DIR) == 0)
            {
                // DIR = 0, IN, transmission is done
                USB_Clear_EPnR_Tx_CTR(0);
                USB_InEPs[0].xfer_done_count += USB_InEPs[0].xfer_len;
                USB_DataINCallback(0);
            } else {
                // DIR = 1, OUT, reception is done
                // Was this a setup packet? Special rules apply
                if(active_epnr & USB_EP_SETUP) {
                    // Get number of bytes
                    USB_OutEPs[0].xfer_len = USB_Get_PMA_Endpoint_Rx_Count(0);
                    // Read from the PMA buffer
                    USB_ReadPacket((uint8_t*)USB_Setup_Buffer, 0, USB_OutEPs[0].xfer_len);
                    USB_Clear_EPnR_Rx_CTR(0);
                    USB_SetupCallback();
                } else {
                    USB_OutEPs[0].xfer_len = USB_Get_PMA_Endpoint_Rx_Count(0);
                    USB_ReadPacket(USB_OutEPs[0].xfer_buffer, 0, USB_OutEPs[0].xfer_len);
                }

            }
        } else {
            // Any other endpoint
            // Which direction?
            if((freeze_istr & USB_ISTR_DIR) == 0) {
                // DIR = 0, IN, transmission is done
                USB_Clear_EPnR_Tx_CTR(active_ep);
                USB_InEPs[active_ep].xfer_done_count += USB_InEPs[active_ep].xfer_len;
                USB_DataINCallback(active_ep);
            } else {
                // DIR = 1, OUT, reception is done

                // Get number of bytes
                USB_OutEPs[active_ep].xfer_len = USB_Get_PMA_Endpoint_Rx_Count(active_ep);
                USB_OutEPs[active_ep].xfer_done_count += USB_OutEPs[active_ep].xfer_len;
                // Read from the PMA buffer
                USB_ReadPacket(USB_OutEPs[active_ep].xfer_buffer, active_ep, USB_OutEPs[active_ep].xfer_len);
                USB_Clear_EPnR_Rx_CTR(active_ep);
                // Further processing in the callback
                USB_DataOUTCallback(active_ep);
            }
        }

    }
}

void USB_Reset_IRQ(void) {
    if (USB_ClassCallbackData != NULLPTR
                && (USB_ClassCallbackData->DeInit != NULLPTR)) {
        USB_ClassCallbackData->DeInit();
    }
    USB_Init();
    USB_Start();
}

void USB_SetupCallback(void) {
    uint8_t* setup8b = (uint8_t*) USB_Setup_Buffer;
    // Figure out what's in that setup packet
    USB_SetupRequest.bmRequest = setup8b[0];
    USB_SetupRequest.bRequest = setup8b[1];
    USB_SetupRequest.wValue = (setup8b[2]) + (setup8b[3] << 8);
    USB_SetupRequest.wIndex = (setup8b[4]) + (setup8b[5] << 8);
    USB_SetupRequest.wLength = (setup8b[6]) + (setup8b[7] << 8);

    USB_EP0_State = USB_EP0_SETUP;
    USB_EP0_DataLen = USB_SetupRequest.wLength;

    // Sort response based on the request type
    switch (USB_SetupRequest.bmRequest & 0x1F) {
    case USB_REQ_RECIPIENT_DEVICE:
        USB_SetupDeviceCallback();
        break;
    case USB_REQ_RECIPIENT_INTERFACE:
        USB_SetupInterfaceCallback();
        break;
    case USB_REQ_RECIPIENT_ENDPOINT:
        USB_SetupEndpointCallback();
        break;
    default:
        if (USB_SetupRequest.bRequest & 0x80) {
            USB_StallINEP(0);
        } else {
            USB_StallOUTEP(0);
        }
        break;
    }

    // See if there's a class specific callback
    if (USB_ClassCallbackData != NULLPTR
            && (USB_ClassCallbackData->Setup != NULLPTR))
        USB_ClassCallbackData->Setup(&USB_SetupRequest);
}

void USB_DataINCallback(uint8_t epnum) {
    USB_EndpointType *pep;
    int32_t bytes_to_send;
    uint16_t temp_epnr;
    pep = &USB_InEPs[epnum];

    // Special case for Endpoint 0 - control transfers
    if(epnum == 0) {
        if(USB_EP0_State == USB_EP0_DATA_IN) {
            // Still transmitting data to the host
            bytes_to_send = pep->total_xfer_len - pep->xfer_done_count;
            if(bytes_to_send != 0) {
                pep->xfer_buffer += pep->xfer_len; // using the previous length, fast-forward in the buffer
                if(bytes_to_send > pep->mps) {
                    // Limit to max packet size
                    bytes_to_send = pep->mps;
                }
                pep->xfer_len = bytes_to_send;
                USB_Start_INEP0_Transfer(pep->xfer_len);
            } else {
                // Do we need to send a zero-length packet?
                if((pep->xfer_len != 0) && (pep->total_xfer_len % pep->mps == 0) && (pep->total_xfer_len >= pep->mps)) {
                    pep->xfer_len = 0;
                    USB_Start_INEP0_Transfer(pep->xfer_len);
                    // Remains in data in stage until next time we come back in this function.
                } else {
                    USB_EP0_State = USB_EP0_STATUS_OUT;
                    // Set the EP_KIND bit, in this case it means we only expect a zero-length OUT packet
                    // Any other length of OUT packet will get a stall instead of an ack response
                    temp_epnr = USB_Get_EPnR(0) & USB_EPREG_MASK;
                    temp_epnr |= USB_EP_KIND;
                    USB_Set_EPnR(0, temp_epnr);
                }
            }
        } else if(USB_EP0_State == USB_EP0_STATUS_IN){

            // Check if address was assigned - we can now safely set own address
            if(USB_NewAddr != 0) {
                USB->DADDR = (USB_NewAddr & 0x7FU) & USB_DADDR_EF;
                USB_NewAddr = 0;
            }
            USB_EP0_State = USB_EP0_IDLE;
        }
    } else {
        // All other endpoints

        // Try to send more data if it's there
        bytes_to_send = pep->total_xfer_len - pep->xfer_done_count;

        if(bytes_to_send != 0) {
            // using the previous length, fast-forward in the buffer
            pep->xfer_buffer += pep->xfer_len;
            if(bytes_to_send > pep->mps) {
                bytes_to_send = pep->mps;
            }
            pep->xfer_len = bytes_to_send;
            USB_Start_INEP_Transfer(epnum, bytes_to_send);
        } else {
            // first check if we need to send a zero-length packet
            // Check if:
            // (1) last transfer wasn't zero length
            // (2) total transfer is a multiple of max packet size
            if ((pep->xfer_len != 0)
                    && ((pep->total_xfer_len % pep->mps) == 0)
                    && (pep->total_xfer_len >= pep->mps)) {
                pep->xfer_len = 0;
                USB_Start_INEP_Transfer(epnum, 0);
            }
            // see if there's an application callback
            else if((USB_ClassCallbackData != NULLPTR) && (USB_ClassCallbackData->DataIn != NULLPTR)) {
                USB_ClassCallbackData->DataIn(epnum);
            }
        }
    }
}


void USB_DataOUTCallback(uint8_t epnum) {
    USB_EndpointType *pep;
    int32_t bytes_to_receive;
    pep = &USB_OutEPs[epnum];

    if (epnum == 0) {
        if (USB_EP0_State == USB_EP0_DATA_OUT) {
            // How many bytes are left?
            bytes_to_receive = pep->total_xfer_len - pep->xfer_done_count;
            if(bytes_to_receive > 0) {
                // Keep going!
                USB_Start_OUTEP0_Transfer();
            } else {
                if ((USB_ClassCallbackData->EP0_RxReady != NULLPTR)
                        && (USB_DevState == USB_STATE_CONFIGURED)) {
                    USB_ClassCallbackData->EP0_RxReady();
                }
                // Send zero-length status packet
                USB_SendCtrlStatus();
            }
        } else if(USB_EP0_State == USB_EP0_STATUS_OUT) {
            // Status transaction is now complete
            USB_EP0_State = USB_EP0_IDLE;
        }
    } else if ((USB_ClassCallbackData->DataOut != NULLPTR)
            && (USB_DevState == USB_STATE_CONFIGURED)) {
        USB_ClassCallbackData->DataOut(epnum);
    }
}

void USB_Start_OUTEP0(void) {
    // Initialize the endpoint control register for endpoint 0
    USB->EP0R = USB_EP_CONTROL; // Clears the CTR for RX and TX, sets endpoint address to zero, sets type to control
    uint16_t Ep_Non_Toggle_Bits = (USB->EP0R) & USB_EPREG_MASK;
    // Make sure the status bits are RX valid and TX nak
    // These bits are toggled, not written directly, so the modification is kinda weird. XOR to the rescue!
    if((USB->EP0R & USB_EPRX_STAT) != USB_EP_RX_VALID) {
        // Set to RX Valid
        USB->EP0R = Ep_Non_Toggle_Bits | ((USB->EP0R & USB_EPRX_STAT) ^ USB_EP_RX_VALID);
    }
    if((USB->EP0R & USB_EPTX_STAT) != USB_EP_TX_NAK) {
        // Set to TX Nak
        USB->EP0R = Ep_Non_Toggle_Bits | ((USB->EP0R & USB_EPTX_STAT) ^ USB_EP_TX_NAK);
    }
}

void USB_SetupDeviceCallback(void) {
    uint8_t addr;
    switch (USB_SetupRequest.bRequest) {
    case USB_REQ_GET_DESCRIPTOR:
        USB_GetDescriptor();
        break;

    case USB_REQ_SET_ADDRESS:
        addr = USB_SetupRequest.wValue & 0x7F;
        USB_NewAddr = addr; // Will be set after entire setup transaction is done. Need to wait
                            // until after the STATUS_OUT stage so that we keep responding to address zero
        // Send a zero-length status packet
        USB_SendData(NULLPTR, 0, 0);
        USB_DevState = USB_STATE_ADDRESSED;
        USB_EP0_State = USB_EP0_STATUS_IN;
        break;

    case USB_REQ_SET_CONFIGURATION:
        USB_SetConfig(USB_SetupRequest.wValue);
        break;

    case USB_REQ_GET_CONFIGURATION:
        USB_GetConfig();
        break;

    case USB_REQ_GET_STATUS:
        USB_GetStatus();
        break;

    case USB_REQ_SET_FEATURE:
        USB_SetFeature();
        break;

    case USB_REQ_CLEAR_FEATURE:
        USB_ClrFeature();
        break;

    default:
        USB_CtrlError();
        break;
    }
}

void USB_SetupInterfaceCallback(void) {

    switch (USB_DevState) {
    case USB_STATE_CONFIGURED:

        if (LOBYTE(USB_SetupRequest.wIndex) <= USB_MAX_NUM_INTERFACES) {
            if (USB_ClassCallbackData != NULLPTR
                    && (USB_ClassCallbackData->Setup != NULLPTR))
                USB_ClassCallbackData->Setup(&USB_SetupRequest);

            if ((USB_SetupRequest.wLength == 0)) {
                // Send a zero-length status packet
                USB_SendData(NULLPTR, 0, 0);
                USB_EP0_State = USB_EP0_STATUS_IN;
            }
        } else {
            USB_CtrlError();
        }
        break;

    default:
        USB_CtrlError();
        break;
    }
}

void USB_SetupEndpointCallback(void) {
    uint8_t ep_addr;
    ep_addr = LOBYTE(USB_SetupRequest.wIndex);

    switch (USB_SetupRequest.bRequest) {
    case USB_REQ_SET_FEATURE:

        switch (USB_DevState) {
        case USB_STATE_ADDRESSED:
            if ((ep_addr != 0x00) && (ep_addr != 0x80)) {
                if (ep_addr & 0x80)
                    USB_StallINEP(ep_addr & 0x7F);
                else
                    USB_StallOUTEP(ep_addr & 0x7F);
            }
            break;

        case USB_STATE_CONFIGURED:
            if (USB_SetupRequest.wValue == USB_FEATURE_EP_HALT) {
                if ((ep_addr != 0x00) && (ep_addr != 0x80)) {
                    if (ep_addr & 0x80)
                        USB_StallINEP(ep_addr & 0x7F);
                    else
                        USB_StallOUTEP(ep_addr & 0x7F);
                }
            }
            if (USB_ClassCallbackData != NULLPTR
                    && (USB_ClassCallbackData->Setup != NULLPTR))
                USB_ClassCallbackData->Setup(&USB_SetupRequest);
            // Send a zero-length status packet
            USB_SendData(NULLPTR, 0, 0);
            USB_EP0_State = USB_EP0_STATUS_IN;

            break;

        default:
            USB_CtrlError();
            break;
        }
        break;

    case USB_REQ_CLEAR_FEATURE:

        switch (USB_DevState) {
        case USB_STATE_ADDRESSED:
            if ((ep_addr != 0x00) && (ep_addr != 0x80)) {
                if (ep_addr & 0x80)
                    USB_StallINEP(ep_addr & 0x7F);
                else
                    USB_StallOUTEP(ep_addr & 0x7F);
            }
            break;

        case USB_STATE_CONFIGURED:
            if (USB_SetupRequest.wValue == USB_FEATURE_EP_HALT) {
                if ((ep_addr & 0x7F) != 0x00) {
                    if (ep_addr & 0x80)
                        USB_ClrStallINEP(ep_addr & 0x7F);
                    else
                        USB_ClrStallOUTEP(ep_addr & 0x7F);
                }
            }
            if (USB_ClassCallbackData != NULLPTR
                    && (USB_ClassCallbackData->Setup != NULLPTR))
                USB_ClassCallbackData->Setup(&USB_SetupRequest);
            // Send a zero-length status packet
            USB_SendData(NULLPTR, 0, 0);
            USB_EP0_State = USB_EP0_STATUS_IN;
            break;

        default:
            USB_CtrlError();
            break;
        }
        break;

    case USB_REQ_GET_STATUS:
        switch (USB_DevState) {
        case USB_STATE_ADDRESSED:
            if ((ep_addr & 0x7F) != 0x00) {
                if (ep_addr & 0x80)
                    USB_StallINEP(ep_addr & 0x7F);
                else
                    USB_StallOUTEP(ep_addr & 0x7F);
            }
            break;

        case USB_STATE_CONFIGURED:
            if (ep_addr & 0x80) {
                if((USB_Get_EPnR(ep_addr&0x7F) & USB_EP_TX_STALL) == USB_EP_TX_STALL)
                    USB_ConfigStatus = 0x0001;
                else
                    USB_ConfigStatus = 0x0000;
            } else {
                if((USB_Get_EPnR(ep_addr&0x7F) & USB_EP_RX_STALL) == USB_EP_RX_STALL)
                    USB_ConfigStatus = 0x0001;
                else
                    USB_ConfigStatus = 0x0000;
            }
            // Send a the data response packet
            USB_SendData((uint8_t*) &USB_ConfigStatus, 0, 2);
            USB_EP0_State = USB_EP0_DATA_IN;
            break;

        default:
            USB_CtrlError();
            break;
        }
        break;

    default:
        break;
    }
}

void USB_GetDescriptor(void) {
    uint16_t len;
    uint8_t *pbuf;

    switch (USB_SetupRequest.wValue >> 8) {
    case USB_DESC_TYPE_DEVICE:
        pbuf = USB_ClassDescData->GetDeviceDescriptor(&len);
        break;

    case USB_DESC_TYPE_CONFIGURATION:
        pbuf = USB_ClassDescData->GetConfigDescriptor(&len);
        pbuf[1] = USB_DESC_TYPE_CONFIGURATION;
        break;

    case USB_DESC_TYPE_STRING:
        switch ((uint8_t) (USB_SetupRequest.wValue)) {
        case USB_IDX_LANGID_STR:
            pbuf = USB_ClassDescData->GetLangIDStrDescriptor(&len);
            break;

        case USB_IDX_MFC_STR:
            pbuf = USB_ClassDescData->GetManufacturerStrDescriptor(&len);
            break;

        case USB_IDX_PRODUCT_STR:
            pbuf = USB_ClassDescData->GetProductStrDescriptor(&len);
            break;

        case USB_IDX_SERIAL_STR:
            pbuf = USB_ClassDescData->GetSerialStrDescriptor(&len);
            break;

        case USB_IDX_CONFIG_STR:
            pbuf = USB_ClassDescData->GetConfigurationStrDescriptor(&len);
            break;

        case USB_IDX_INTERFACE_STR:
            pbuf = USB_ClassDescData->GetInterfaceStrDescriptor(&len);
            break;

        default:
            USB_CtrlError();
            return;
        }
        break;
    case USB_DESC_TYPE_DEVICE_QUALIFIER:
        USB_CtrlError();
        return;

    case USB_DESC_TYPE_OTHER_SPEED_CONFIGURATION:
        USB_CtrlError();
        return;

    default:
        USB_CtrlError();
        return;
    }

    if ((len != 0) && (USB_SetupRequest.wLength != 0)) {

        len = MIN(len, USB_SetupRequest.wLength);

        USB_SendData(pbuf, 0, len);
        USB_EP0_State = USB_EP0_DATA_IN;
    }

}

void USB_SetConfig(uint8_t configNum) {
    if (configNum > USB_MAX_NUM_CONFIGURATION) {
        USB_CtrlError();
    } else {
        switch (USB_DevState) {
        case USB_STATE_ADDRESSED:
            if (configNum) {
                USB_DevState = USB_STATE_CONFIGURED;
                USB_CurrentConfig = configNum;
                // Call class config
                if (USB_ClassCallbackData != NULLPTR
                        && (USB_ClassCallbackData->Init != NULLPTR)) {
                    USB_ClassCallbackData->Init(configNum);
                }
            }
            // Send a zero byte status packet
            USB_SendData(NULLPTR, 0, 0);
            USB_EP0_State = USB_EP0_STATUS_IN;
            break;
        case USB_STATE_CONFIGURED:
            if (configNum == 0) {
                // Unconfig!
                USB_DevState = USB_STATE_ADDRESSED;
                USB_CurrentConfig = 0;
                if (USB_ClassCallbackData != NULLPTR
                        && (USB_ClassCallbackData->DeInit != NULLPTR)) {
                    USB_ClassCallbackData->DeInit();
                }
                // Send a zero byte status packet
                USB_SendData(NULLPTR, 0, 0);
                USB_EP0_State = USB_EP0_STATUS_IN;
            } else if (configNum > USB_MAX_NUM_CONFIGURATION) {
                // Invalid config number
                USB_CtrlError();
            } else {
                // Send a zero byte status packet
                USB_SendData(NULLPTR, 0, 0);
                USB_EP0_State = USB_EP0_STATUS_IN;
            }
            break;
        default:
            USB_CtrlError();
        }
    }
}

void USB_GetConfig(void) {
    if (USB_SetupRequest.wLength != 1)
        USB_CtrlError();
    else {
        switch (USB_DevState) {
        case USB_STATE_ADDRESSED:
        case USB_STATE_CONFIGURED:
            USB_SendData((uint8_t*) &USB_CurrentConfig, 0, USB_SetupRequest.wLength);
            USB_EP0_State = USB_EP0_DATA_IN;
            break;
        default:
            USB_CtrlError();
            break;
        }
    }
}

void USB_GetStatus(void) {
//    USB_ConfigStatus = USB_CONFIG_SELF_POWERED | USB_CONFIG_REMOTE_WAKEUP;
    USB_ConfigStatus = USB_CONFIG_SELF_POWERED;
    USB_SendData((uint8_t*) &USB_ConfigStatus, 0, 2);
    USB_EP0_State = USB_EP0_DATA_IN;
}

void USB_SetFeature(void) {
    if (USB_SetupRequest.wValue == USB_FEATURE_REMOTE_WAKEUP) {
        // Send a zero byte status packet
        USB_SendData(NULLPTR, 0, 0);
        USB_EP0_State = USB_EP0_STATUS_IN;
    } else {
        USB_CtrlError();
    }
}

void USB_ClrFeature(void) {
    if (USB_SetupRequest.wValue == USB_FEATURE_REMOTE_WAKEUP) {
        // Send a zero byte status packet
        USB_SendData(NULLPTR, 0, 0);
        USB_EP0_State = USB_EP0_STATUS_IN;
    } else {
        USB_CtrlError();
    }
}


/**
 * Sets up an endpoint by assigning the EPnR values and assigning packet memory (PMA)
 * This function should always be called before the same endpoint's USB_ActivateOUTEP
 * to ensure that the PMA adddresses are set correctly.
 */
void USB_ActivateINEP(uint8_t epnum, uint8_t eptype, uint32_t maxpacketsize, uint32_t buffersize) {

    // Assign buffer address.
    uint16_t buffer_begin = USB_BTABLE_SIZE;

    for(uint8_t i = 0; i < epnum; i++) {
        buffer_begin += USB_InEPs[i].buffersize;
        buffer_begin += USB_OutEPs[i].buffersize;
    }
    USB_Set_PMA_Endpoint_Tx_Address(epnum, buffer_begin);
    USB_InEPs[epnum].pmaaddr = buffer_begin;

    // Settings in EPnR
    switch(eptype) {
    case USB_EP_TYPE_CTRL:
        USB_Set_EPnR(epnum, (epnum | USB_EP_CONTROL));
        break;
    case USB_EP_TYPE_BULK:
        USB_Set_EPnR(epnum, (epnum | USB_EP_BULK));
        break;
    case USB_EP_TYPE_ISOC:
        USB_Set_EPnR(epnum, (epnum | USB_EP_ISOCHRONOUS));
        break;
    case USB_EP_TYPE_INTR:
        USB_Set_EPnR(epnum, (epnum | USB_EP_INTERRUPT));
        break;
    }

    // Set stat to Nak since we're not ready to send a packet yet
    USB_Set_EPnR_Tx_Stat(epnum, USB_EP_TX_NAK);

    USB_InEPs[epnum].mps = maxpacketsize;
    USB_InEPs[epnum].buffersize = buffersize;
}

void USB_ActivateOUTEP(uint8_t epnum, uint8_t eptype, uint32_t maxpacketsize, uint32_t buffersize) {

    // Assign buffer address.
    uint16_t buffer_begin = USB_BTABLE_SIZE;
    for(uint8_t i = 0; i < epnum; i++) {
        buffer_begin += USB_InEPs[i].buffersize;
        buffer_begin += USB_OutEPs[i].buffersize;
    }
    buffer_begin += USB_InEPs[epnum].buffersize;
    USB_Set_PMA_Endpoint_Rx_Address(epnum, buffer_begin);
    USB_OutEPs[epnum].pmaaddr = buffer_begin;
    USB_Set_PMA_Endpoint_Rx_BufSize(epnum, maxpacketsize);
    // Settings in EPnR
    switch(eptype) {
    case USB_EP_TYPE_CTRL:
        USB_Set_EPnR(epnum, (epnum | USB_EP_CONTROL));
        break;
    case USB_EP_TYPE_BULK:
        USB_Set_EPnR(epnum, (epnum | USB_EP_BULK));
        break;
    case USB_EP_TYPE_ISOC:
        USB_Set_EPnR(epnum, (epnum | USB_EP_ISOCHRONOUS));
        break;
    case USB_EP_TYPE_INTR:
        USB_Set_EPnR(epnum, (epnum | USB_EP_INTERRUPT));
        break;
    }
    // Set stat to Valid so reception can occur
    USB_Set_EPnR_Rx_Stat(epnum, USB_EP_RX_VALID);

    USB_OutEPs[epnum].mps = maxpacketsize;
    USB_OutEPs[epnum].buffersize = buffersize;
}

void USB_DeactivateINEP(uint8_t epnum) {

    // Set stat to Disabled
    USB_Set_EPnR_Tx_Stat(epnum, USB_EP_TX_DIS);
}

void USB_DeactivateOUTEP(uint8_t epnum) {

    // Set stat to Disabled
    USB_Set_EPnR_Rx_Stat(epnum, USB_EP_RX_DIS);
}

void USB_StallINEP(uint8_t epnum) {

    // Set stat to Stall
    USB_Set_EPnR_Tx_Stat(epnum, USB_EP_TX_STALL);
}

void USB_StallOUTEP(uint8_t epnum) {

    // Set stat to Stall
    USB_Set_EPnR_Rx_Stat(epnum, USB_EP_RX_STALL);
}

void USB_ClrStallINEP(uint8_t epnum) {

    // Set stat to Nak
    USB_Set_EPnR_Tx_Stat(epnum, USB_EP_TX_NAK);
}

void USB_ClrStallOUTEP(uint8_t epnum) {

    // Set stat to Valid
    USB_Set_EPnR_Rx_Stat(epnum, USB_EP_RX_VALID);
}

void USB_CtrlError(void) {
    // Send a zero-length status packet
    USB_EP0_State = USB_EP0_STALL;
    USB_StallINEP(0);
    USB_StallOUTEP(0);
}

uint32_t USB_GetRxDataSize(uint8_t epnum) {
    return USB_OutEPs[epnum].xfer_done_count;
}

// Sends a zero-length packet as the last phase of a Status transaction
void USB_SendCtrlStatus(void) {
    USB_EP0_State = USB_EP0_STATUS_IN;
    USB_SendData(NULLPTR, 0, 0);
}

// Sends IN data as part of a status transaction
void USB_SendCtrlData(uint8_t *pbuf, uint16_t len) {
    USB_EP0_State = USB_EP0_DATA_IN;
    USB_SendData(pbuf, 0, len);
}

void USB_SendData(uint8_t *pbuf, uint8_t epnum, uint16_t len) {
    USB_InEPs[epnum].xfer_buffer = pbuf;
    USB_InEPs[epnum].xfer_len = MIN(len, USB_InEPs[epnum].mps);
    USB_InEPs[epnum].total_xfer_len = len;
    USB_InEPs[epnum].xfer_done_count = 0;

    if (epnum == 0)
        USB_Start_INEP0_Transfer(len);
    else
        USB_Start_INEP_Transfer(epnum, len);
}

void USB_PrepareRead(uint8_t *pbuf, uint8_t epnum, uint16_t len) {
    USB_OutEPs[epnum].xfer_buffer = pbuf;
    USB_OutEPs[epnum].xfer_len = MIN(len, USB_OutEPs[epnum].mps);
    USB_OutEPs[epnum].total_xfer_len = len;
    USB_OutEPs[epnum].xfer_done_count = 0;

    if (epnum == 0)
        USB_Start_OUTEP0_Transfer();
    else
        USB_Start_OUTEP_Transfer(epnum);
}

// Preps for OUT data coming from the host as part of a status transaction
void USB_PrepareCtrlRead(uint8_t *pbuf, uint16_t len) {
    USB_EP0_State = USB_EP0_DATA_OUT;
    USB_PrepareRead(pbuf, 0, len);
}

void USB_ReadPacket(uint8_t* buf, uint8_t epnum, uint16_t len) {
    uint16_t* pmabuf = (uint16_t*)((uint32_t)(USB_InEPs[epnum].pmaaddr) + (uint32_t)(USB_PMA_BASE_ADDR));

    for(uint16_t i = len; i > 0U; ) {
        *buf = (uint8_t)((*pmabuf) & 0x00FF);
        buf++;
        i--;
        if(i != 0) {
            *buf = (uint8_t)(((*pmabuf) & 0xFF00) >> 8U);
            buf++;
            i --;
        }
    }
}

void USB_WritePacket(uint8_t* buf, uint8_t epnum, uint16_t len) {
    uint16_t* pmabuf = (uint16_t*)((uint32_t)(USB_InEPs[epnum].pmaaddr) + (uint32_t)(USB_PMA_BASE_ADDR));
    uint16_t temp1;
    for(uint16_t i = len; i > 0U; ) {
        temp1 = *buf;
        buf++;
        i--;
        if(i == 0) {
            *pmabuf = temp1 & 0x00FF;
            pmabuf++;
        } else {
            *pmabuf = temp1 | ((uint16_t)((uint16_t)(*buf) << 8U));
            buf++;
            pmabuf++;
            i--;
        }
    }
}

void USB_Start_INEP0_Transfer(uint32_t len) {
    USB_Start_INEP_Transfer(0, len);
}

void USB_Start_INEP_Transfer(uint8_t epnum, uint32_t len) {

    // Program the transfer size
    USB_Set_PMA_Endpoint_Tx_Count(epnum, len);
    // Write the Tx buffer in PMA
    USB_WritePacket(USB_InEPs[epnum].xfer_buffer, epnum, len);
    // Enable the endpoint (set to Valid)
    USB_Set_EPnR_Tx_Stat(epnum, USB_EP_TX_VALID);
}

void USB_Start_OUTEP0_Transfer(void) {
    USB_Start_OUTEP_Transfer(0);
}

void USB_Start_OUTEP_Transfer(uint8_t epnum) {
    // Enable the endpoint
    USB_Set_EPnR_Rx_Stat(epnum, USB_EP_RX_VALID);
}
