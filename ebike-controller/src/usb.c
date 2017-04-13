/*
 * usb.c
 *
 *  Created on: Feb 20, 2017
 *      Author: David
 */

#include "stm32f4xx.h"
#include "gpio.h"
#include "usb.h"
#include "usb_cdc.h"

#define USB_CORE		((USB_OTG_GlobalTypeDef *) USB_OTG_FS_PERIPH_BASE)
#define USB_DEVICE		((USB_OTG_DeviceTypeDef *)((uint32_t )USB_CORE + USB_OTG_DEVICE_BASE))
#define USB_INEP(i)    ((USB_OTG_INEndpointTypeDef *)((uint32_t)USB_CORE + USB_OTG_IN_ENDPOINT_BASE + (i)*USB_OTG_EP_REG_SIZE))
#define USB_OUTEP(i)   ((USB_OTG_OUTEndpointTypeDef *)((uint32_t)USB_CORE + USB_OTG_OUT_ENDPOINT_BASE + (i)*USB_OTG_EP_REG_SIZE))
#define USB_DFIFO(i)   *(__IO uint32_t *)((uint32_t)USB_CORE + USB_OTG_FIFO_BASE + (i) * USB_OTG_FIFO_SIZE)

USB_EndpointType USB_InEPs[NUM_ENDPOINTS];
USB_EndpointType USB_OutEPs[NUM_ENDPOINTS];

/* State registers and whatnot */
uint32_t USB_Setup_Buffer[12];
USB_SetupReqTypedef USB_SetupRequest;
USB_ClassDescTypedef* USB_ClassDescData;
USB_ClassCallbackTypedef* USB_ClassCallbackData;
USB_FIFOStatusTypedef USB_FIFO_Status;
uint8_t USB_EP0_State;
uint16_t USB_EP0_DataLen;
uint8_t USB_DevState;
uint32_t USB_CurrentConfig;
uint32_t USB_ConfigStatus;
uint8_t USB_OUTEP0_Buffer[USB_MAX_EP0_SIZE*2];
/*
void GPIO_AF(GPIO_TypeDef* gpio, uint8_t pin, uint8_t af)
{
	// Clear MODER for this pin
	gpio->MODER &= ~(GPIO_MODER_MODER0 << (pin * 2));
	// Set this pin's MODER to alternate function
	gpio->MODER |= (GPIO_MODER_MODER0_1 << (pin * 2));
	// Set output type to push-pull
	gpio->OTYPER &= ~(GPIO_OTYPER_OT_0 << pin);
	// Set pull-up/down off
	gpio->PUPDR &= ~(GPIO_PUPDR_PUPDR0 << (pin * 2));
	// Set speed to maximum
	gpio->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR0 << (pin * 2));
	// Set alternate function register
	if(pin >= 8)
	{
		gpio->AFR[1] &= ~((0x0F) << ((pin - 8)*4));;
		gpio->AFR[1] |= (af << ((pin - 8)*4));
	}
	else
	{
		gpio->AFR[0] &= ~((0x0F) << (pin*4));
		gpio->AFR[0] |= (af << (pin*4));
	}
}
*/
static void USB_CoreReset(void)
{
	// Ensure that the AHB bus is idle
	while((USB_CORE->GRSTCTL & USB_OTG_GRSTCTL_AHBIDL) == 0){}
	// Trigger the core reset
	USB_CORE->GRSTCTL |= USB_OTG_GRSTCTL_CSRST;
	// Wait for the reset to finish
	while((USB_CORE->GRSTCTL & USB_OTG_GRSTCTL_CSRST) != 0){ }
}

static void USB_FlushTxFifo(uint8_t fifonum)
{
	// Ensure that the AHB bus is idle
	while((USB_CORE->GRSTCTL & USB_OTG_GRSTCTL_AHBIDL) == 0){}
	USB_CORE->GRSTCTL = (fifonum << 6) | USB_OTG_GRSTCTL_TXFFLSH;
	while((USB_CORE->GRSTCTL & USB_OTG_GRSTCTL_TXFFLSH) != 0) {}
}

static void USB_FlushRxFifo(void)
{
	// Ensure that the AHB bus is idle
	while((USB_CORE->GRSTCTL & USB_OTG_GRSTCTL_AHBIDL) == 0){}
	USB_CORE->GRSTCTL = USB_OTG_GRSTCTL_RXFFLSH;
	while((USB_CORE->GRSTCTL & USB_OTG_GRSTCTL_RXFFLSH) != 0) {}
}

void USB_Init(void)
{
	// Initialize clocks
	RCC->AHB2ENR |= RCC_AHB2ENR_OTGFSEN;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

	// Set port pins for USB
	GPIO_AF(USB_PORT, USB_DM_PIN, USB_AF);
	GPIO_AF(USB_PORT, USB_DP_PIN, USB_AF);

	// Enable interrupt
	NVIC_SetPriority(OTG_FS_IRQn, PRIO_USB);
	NVIC_EnableIRQ(OTG_FS_IRQn);

	// USB Core reset
	USB_CoreReset();

	// Globally disable USB interrupts
	USB_CORE->GAHBCFG &= ~USB_OTG_GAHBCFG_GINT;

	// Turn off the power down mode
	USB_CORE->GCCFG = USB_OTG_GCCFG_PWRDWN;
	// Force to device mode
	USB_CORE->GUSBCFG |= USB_OTG_GUSBCFG_FDMOD;
	// Stop the USB PHY by disabling pull-up/-down resistors on D+/D- pins
	USB_DEVICE->DCTL |= USB_OTG_DCTL_SDIS;
	// Set device speed to full-speed (12 Mbps)
	USB_DEVICE->DCFG |= USB_OTG_DCFG_DSPD;
	// Disable VBUS sensing
	USB_CORE->GCCFG |= USB_OTG_GCCFG_NOVBUSSENS;
	// Flush the FIFOs
	USB_FlushTxFifo(0);
	USB_FlushTxFifo(1);
	USB_FlushTxFifo(2);
	USB_FlushRxFifo();
	// Clear all USB interrupt masks
	USB_CORE->GINTMSK = 0;
	USB_CORE->GINTSTS = 0xFFFFFFFF;
	// Enable Device-mode interrupts
	USB_CORE->GINTMSK |= USB_OTG_GINTMSK_WUIM; // Wakeup detected
	USB_CORE->GINTMSK |= (USB_OTG_GINTMSK_USBSUSPM | USB_OTG_GINTMSK_USBRST); // USB reset or suspended
	USB_CORE->GINTMSK |= (USB_OTG_GINTMSK_ENUMDNEM | USB_OTG_GINTMSK_IISOIXFRM); // Enumeration done or incomplete isochronous IN
	USB_CORE->GINTMSK |= (USB_OTG_GINTMSK_OEPINT   | USB_OTG_GINTMSK_IEPINT); // IN or OUT endpoint interrupt
	USB_CORE->GINTMSK |= (USB_OTG_GINTMSK_PXFRM_IISOOXFRM | USB_OTG_GINTMSK_RXFLVLM); // Incomplete iso OUT, incomplete periodic, or Rx fifo not empty
	// Clear all endpoint interrupt masks
	USB_DEVICE->DIEPMSK = 0;
	USB_DEVICE->DOEPMSK = 0;
	USB_DEVICE->DAINTMSK = 0;
	// Clear each endpoint
	for(uint8_t i = 0; i < NUM_ENDPOINTS; i++)
	{
		if(USB_INEP(i)->DIEPCTL & USB_OTG_DIEPCTL_EPENA)
		{
			USB_INEP(i)->DIEPCTL = USB_OTG_DIEPCTL_EPDIS;
		}
		else
		{
			USB_INEP(i)->DIEPCTL = 0;
		}
		USB_INEP(i)->DIEPTSIZ = 0;
		USB_INEP(i)->DIEPINT = 0xFF;

		if(USB_OUTEP(i)->DOEPCTL & USB_OTG_DOEPCTL_EPENA)
		{
			USB_OUTEP(i)->DOEPCTL = USB_OTG_DIEPCTL_EPDIS;
		}
		else
		{
			USB_OUTEP(i)->DOEPCTL = 0;
		}
		USB_OUTEP(i)->DOEPTSIZ = 0;
		USB_OUTEP(i)->DOEPINT = 0xFF;
	}


	// Buffer settings:
	// Addr(0): RX buffer, size = RX_FIFO_BUF_SIZE
	// Addr(RX_FIFO_BUF_SIZE): Tx EP0 buffer, size = CMD_FIFO_BUF_SIZE
	// Addr(RX_FIFO_BUF_SIZE + CMD_FIFO_BUF_SIZE): Tx EP1 buffer, size = TX_FIFO_BUF_SIZE
	// For example...
	// Addr(0) = Rx buffer, size = 512 (0 -> 511)
	// Addr(512) = Tx EP0 buffer, size = 128 (512 -> 639)
	// Addr(640) = Tx EP1 buffer, size = 512 (640 -> 1151)

	// Set RX (OUT packets) FIFO buffer size
	USB_CORE->GRXFSIZ = RX_FIFO_BUF_SIZE;

	// Set TX (IN packets) FIFO buffer size for Endpoint 0
	// Start address is equal to RX buffer size
	USB_CORE->DIEPTXF0_HNPTXFSIZ = (CMD_FIFO_BUF_SIZE << 16) | (RX_FIFO_BUF_SIZE);
	// Set TX (IN packets) FIFO buffer size for Endpoint 1
	// Start address is offset by RX buffer and TX EP0 buffer
	USB_CORE->DIEPTXF[0] = (TX_FIFO_BUF_SIZE << 16) | (RX_FIFO_BUF_SIZE + CMD_FIFO_BUF_SIZE);
	USB_CORE->DIEPTXF[1] = (16 << 16) | (TX_FIFO_BUF_SIZE + RX_FIFO_BUF_SIZE + CMD_FIFO_BUF_SIZE);
	USB_CORE->DIEPTXF[2] = 0;

	USB_InEPs[0].mps = USB_MAX_EP0_SIZE;
	USB_OutEPs[0].mps = USB_MAX_EP0_SIZE;

	USB_ClassDescData = NULLPTR;
	USB_ClassCallbackData = NULLPTR;
	USB_DevState = USB_STATE_DEFAULT;
	USB_CurrentConfig = 0;

}

void USB_SetClass(USB_ClassDescTypedef* newclassdesc, USB_ClassCallbackTypedef* newclasscalls)
{
	USB_ClassDescData = newclassdesc;
	USB_ClassCallbackData = newclasscalls;
}

void USB_Start(void)
{
	// Start the USB PHY by enabling pull-up/-down resistors on D+/D- pins
	USB_DEVICE->DCTL &= ~(USB_OTG_DCTL_SDIS);

	// Globally enable USB interrupts
	USB_CORE->GAHBCFG |= USB_OTG_GAHBCFG_GINT;
}

uint8_t USB_GetDevState(void)
{
	return USB_DevState;
}

void USB_IRQ(void)
{
	uint32_t temp;

	if(USB_CORE->GINTSTS & USB_OTG_GINTSTS_OTGINT) // Disconnect event
	{
		temp = USB_CORE->GOTGINT;
		if(temp & USB_OTG_GOTGINT_SEDET)
		{
			if(USB_ClassCallbackData != NULLPTR && (USB_ClassCallbackData->Disconnect != NULLPTR))
				USB_ClassCallbackData->Disconnect();

			USB_CORE->GOTGINT = temp;
		}
	}
	if(USB_CORE->GINTSTS & USB_OTG_GINTSTS_SRQINT) // Connect event
	{
		if(USB_ClassCallbackData != NULLPTR && (USB_ClassCallbackData->Connect != NULLPTR))
			USB_ClassCallbackData->Connect();
		USB_CORE->GINTSTS |= USB_OTG_GINTSTS_SRQINT;
	}
	if(USB_CORE->GINTSTS & USB_OTG_GINTSTS_USBRST) // Reset event
	{
		USB_DEVICE->DCTL &= ~USB_OTG_DCTL_RWUSIG; // Clear remote signaling
		USB_FlushTxFifo(0); // Clear EP0 Fifo
		// Clear pending interrupts
		for (uint8_t i = 0; i < NUM_ENDPOINTS ; i++)
		{
			USB_INEP(i)->DIEPINT = 0xFF;
			USB_OUTEP(i)->DOEPINT = 0xFF;
		}
		// Enable EP0 interrupts
		USB_DEVICE->DAINTMSK |= ((1 << 16) | (1));
		// Enable Setup done, transfer complete interrupts on OUT endpoints
		USB_DEVICE->DOEPMSK |= (USB_OTG_DOEPMSK_STUPM | USB_OTG_DOEPMSK_XFRCM);
		// Enable time-out, transfer complete interrupts on IN endpoints
		USB_DEVICE->DIEPMSK |= (USB_OTG_DIEPMSK_TOM | USB_OTG_DIEPMSK_XFRCM);
		// Clear the device address to zero
		USB_DEVICE->DCFG &= ~USB_OTG_DCFG_DAD;
		// Begin receiving setup packets, set size for OUT EP0
		USB_Start_OUTEP0();

		USB_CORE->GINTSTS |= USB_OTG_GINTSTS_USBRST;
		USB_DevState = USB_STATE_DEFAULT;
	}
	if(USB_CORE->GINTSTS & USB_OTG_GINTSTS_ENUMDNE) // Enumeration done
	{
		USB_INEP(0)->DIEPCTL &= ~USB_OTG_DIEPCTL_MPSIZ; // 64 byte max packet size on EP0
		USB_DEVICE->DCTL |= USB_OTG_DCTL_CGINAK; // Clear global IN NAK, ready to receive packets
		// Turnaround time: allows stretching response times to accomodate AHB read access latency
		USB_CORE->GUSBCFG &= ~USB_OTG_GUSBCFG_TRDT;
		USB_CORE->GUSBCFG |= (USB_OTG_GUSBCFG_TRDT_0 | USB_OTG_GUSBCFG_TRDT_2); // Turnaround time set to 5
		if(USB_ClassCallbackData != NULLPTR && (USB_ClassCallbackData->Reset != NULLPTR))
			USB_ClassCallbackData->Reset();

		USB_CORE->GINTSTS |= USB_OTG_GINTSTS_ENUMDNE;
	}
	if(USB_CORE->GINTSTS & USB_OTG_GINTSTS_SOF) // Start-of-frame event
	{
		USB_CORE->GINTSTS |= USB_OTG_GINTSTS_SOF;
	}
	if(USB_CORE->GINTSTS & USB_OTG_GINTSTS_RXFLVL) // RX fifo non-empty
	{
		// Read and pop
		temp = USB_CORE->GRXSTSP;
		USB_FIFO_Status.chnum = temp & USB_OTG_GRXSTSP_EPNUM;
		USB_FIFO_Status.bcnt = (temp & USB_OTG_GRXSTSP_BCNT) >> 4;
		USB_FIFO_Status.dpid = (temp & USB_OTG_GRXSTSP_DPID) >> 15;
		USB_FIFO_Status.pktsts = (temp & USB_OTG_GRXSTSP_PKTSTS) >> 17;
		// What kind of packet?
		if(((temp & (USB_OTG_GRXSTSP_PKTSTS)) >> 17) == STS_DATA_UPDT) // Data packet received
		{
			if((temp & (USB_OTG_GRXSTSP_BCNT)) != 0) // Non-zero byte count
			{
				USB_ReadPacket((uint8_t*)USB_OutEPs[USB_FIFO_Status.chnum].xfer_buffer, USB_FIFO_Status.bcnt);
				USB_OutEPs[USB_FIFO_Status.chnum].xfer_buffer += USB_FIFO_Status.bcnt;
				USB_OutEPs[USB_FIFO_Status.chnum].xfer_done_count += USB_FIFO_Status.bcnt;
			}
		}
		else if(((temp & (USB_OTG_GRXSTSP_PKTSTS)) >> 17) == STS_SETUP_UPDT) // Setup packet received
		{
			USB_ReadPacket((uint8_t*)USB_Setup_Buffer, USB_FIFO_Status.bcnt); // bcnt should always be 8 for a setup packet
			USB_OutEPs[USB_FIFO_Status.chnum].xfer_done_count += USB_FIFO_Status.bcnt;
		}
	}
	if(USB_CORE->GINTSTS & USB_OTG_GINTSTS_OEPINT) // OUT endpoint interrupt
	{
		uint32_t epint, epnum=0;
		// Which endpoint?
		temp = USB_DEVICE->DAINT; // Read all endpoints
		temp &= USB_DEVICE->DAINTMSK; // Only the active ones
		temp = (temp & 0xFFFF0000) >> 16; // Only the OUT endpoints
		while(temp)
		{
			if(temp & 0x01) // This endpoint has an interrupt
			{
				epint = (USB_OUTEP(epnum)->DOEPINT) & (USB_DEVICE->DOEPMSK);
				if(epint & USB_OTG_DOEPINT_STUP) // Setup phase done, can decode setup packet
				{
					USB_SetupCallback();

					USB_OUTEP(epnum)->DOEPINT |= USB_OTG_DOEPINT_STUP;
				}
				if(epint & USB_OTG_DOEPINT_XFRC) // Transfer completed
				{
					USB_OUTEP(epnum)->DOEPINT |= USB_OTG_DOEPINT_XFRC;

					// TODO: Data out callback
					USB_DataOUTCallback(epnum);
				}
				// Don't care about back-to-back setup, out token recvd when disabled, or endpoint disabled interrupts
				if(epint & (USB_OTG_DOEPINT_B2BSTUP | USB_OTG_DOEPINT_OTEPDIS | USB_OTG_DOEPINT_EPDISD))
				{
					USB_OUTEP(epnum)->DOEPINT |= (USB_OTG_DOEPINT_B2BSTUP | USB_OTG_DOEPINT_OTEPDIS | USB_OTG_DOEPINT_EPDISD);
				}

			}
			temp >>= 1;
			epnum++;
		}
	}
	if(USB_CORE->GINTSTS & USB_OTG_GINTSTS_IEPINT) // IN endpoint interrupt
	{
		uint32_t epint, epnum=0;
		// Which endpoint?
		temp = USB_DEVICE->DAINT; // Read all endpoints
		temp &= USB_DEVICE->DAINTMSK; // Only the active ones
		temp = (temp & 0xFFFF); // Only the IN endpoints
		while(temp)
		{
			if(temp & 0x01) // This endpoint has an interrupt
			{
				epint = (USB_INEP(epnum)->DIEPINT) & (USB_DEVICE->DIEPMSK | (USB_DEVICE->DIEPEMPMSK & (1 << epnum)));
				if((USB_INEP(epnum)->DIEPINT & USB_OTG_DIEPINT_TXFE) && (USB_DEVICE->DIEPEMPMSK & (1 << epnum)))
				{
					epint |= USB_OTG_DIEPINT_TXFE;
				}
				if(epint & USB_OTG_DOEPINT_XFRC) // Transfer completed
				{
					// Clear this endpoint's fifo empty interrupt
					USB_DEVICE->DIEPEMPMSK &= ~(1 << epnum);
					//TODO: Data in callback()
					USB_DataINCallback(epnum);
					USB_INEP(epnum)->DIEPINT |= USB_OTG_DIEPINT_XFRC;
				}
				if(( epint & USB_OTG_DIEPINT_TXFE) == USB_OTG_DIEPINT_TXFE) // Transmit fifo empty
				{
					// Load more into the FIFO, or disable using DIEPEMPMSK
					USB_TxFIFO_Empty_Callback(epnum);
				}
				// Don't care about time-out, NAK effective, IN token when empty, or endpoint disabled interrupts
				if(( epint & USB_OTG_DIEPINT_TOC) == (USB_OTG_DIEPINT_TOC | USB_OTG_DIEPINT_INEPNE | USB_OTG_DIEPINT_ITTXFE | USB_OTG_DIEPINT_EPDISD))
				{
					USB_INEP(epnum)->DIEPINT |= (USB_OTG_DIEPINT_TOC | USB_OTG_DIEPINT_INEPNE | USB_OTG_DIEPINT_ITTXFE | USB_OTG_DIEPINT_EPDISD);
				}

			}
			temp >>= 1;
			epnum++;
		}
	}

	if(USB_CORE->GINTSTS & USB_OTG_GINTSTS_PXFR_INCOMPISOOUT) // Incomplete ISO OUT
	{
		USB_CORE->GINTSTS |= USB_OTG_GINTSTS_PXFR_INCOMPISOOUT;
	}
	if(USB_CORE->GINTSTS & USB_OTG_GINTSTS_IISOIXFR) // Incomplete ISO IN
	{
		USB_CORE->GINTSTS |= USB_OTG_GINTSTS_IISOIXFR;
	}
	if(USB_CORE->GINTSTS & USB_OTG_GINTSTS_IISOIXFR) // Incomplete ISO IN
	{
		USB_CORE->GINTSTS |= USB_OTG_GINTSTS_IISOIXFR;
	}
	if(USB_CORE->GINTSTS & USB_OTG_GINTSTS_WKUINT) // Resume / wakeup
	{
		USB_DEVICE->DCTL &= ~USB_OTG_DCTL_RWUSIG; // Clear resume signaling
		//TODO: Resume callback()

		USB_CORE->GINTSTS |= USB_OTG_GINTSTS_WKUINT;
	}
	if(USB_CORE->GINTSTS & USB_OTG_GINTSTS_USBSUSP) // Suspend
	{
		if((USB_DEVICE->DSTS & USB_OTG_DSTS_SUSPSTS) == USB_OTG_DSTS_SUSPSTS)
		{
			//TODO: Suspend callback()
		}

		USB_CORE->GINTSTS |= USB_OTG_GINTSTS_USBSUSP;
	}
}

void USB_SetupCallback(void)
{
	uint8_t* setup8b = (uint8_t*)USB_Setup_Buffer;
	// Figure out what's in that setup packet
	USB_SetupRequest.bmRequest = setup8b[0];
	USB_SetupRequest.bRequest = setup8b[1];
	USB_SetupRequest.wValue = (setup8b[2]) + (setup8b[3] << 8);
	USB_SetupRequest.wIndex = (setup8b[4]) + (setup8b[5] << 8);
	USB_SetupRequest.wLength = (setup8b[6]) + (setup8b[7] << 8);

	USB_EP0_State = USB_EP0_SETUP;
	USB_EP0_DataLen = USB_SetupRequest.wLength;

	// Sort response based on the request type
	switch(USB_SetupRequest.bmRequest & 0x1F)
	{
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
		if(USB_SetupRequest.bRequest & 0x80)
		{
			USB_StallINEP(0);
		}
		else
		{
			USB_StallOUTEP(0);
		}
		break;
	}

	// See if there's a class specific callback
	if(USB_ClassCallbackData != NULLPTR && (USB_ClassCallbackData->Setup != NULLPTR))
		USB_ClassCallbackData->Setup(&USB_SetupRequest);
}

void USB_DataINCallback(uint8_t epnum)
{
	USB_EndpointType *pep;

	  if(epnum == 0)
	  {
	    pep = &USB_InEPs[0];


	    if ( USB_EP0_State == USB_EP0_DATA_IN)
	    {
	      if(pep->xfer_remaining > pep->mps)
	      {
	        pep->xfer_remaining -=  pep->mps;

	        USB_Start_INEP0_Transfer(pep->xfer_remaining);
	      }
	      else
	      { // last packet is MPS multiple, so send ZLP packet
	        if((pep->total_xfer_len % pep->mps == 0) &&
	           (pep->total_xfer_len >= pep->mps) &&
	             (pep->total_xfer_len < USB_EP0_DataLen ))
	        {

	        	USB_Start_INEP0_Transfer(0);
	          USB_EP0_DataLen = 0;
	        }
	        else
	        {
	        	USB_EP0_State = USB_EP0_STATUS_OUT;
	          // Receive zero length packet
	          USB_Start_OUTEP0_Transfer(0);
	        }
	      }
	    }
	  }
	  // See if there's a DataIn stage callback
	  else if(USB_ClassCallbackData != NULLPTR && (USB_ClassCallbackData->DataIn != NULLPTR))
	  {
		  USB_ClassCallbackData->DataIn(epnum);
	  }
}

void USB_DataOUTCallback(uint8_t epnum)
{
	if(epnum == 0)
	{
		if ( USB_EP0_State == USB_EP0_DATA_OUT)
		{
			if(USB_OutEPs[epnum].xfer_remaining > USB_OutEPs[epnum].mps)
			{
				USB_OutEPs[epnum].xfer_remaining -=  USB_OutEPs[epnum].mps;

				//USBD_CtlContinueRx (pdev, pdata, MIN(pep->rem_length ,pep->maxpacket));
				USB_PrepareRead(USB_OutEPs[epnum].xfer_buffer, 0, MIN(USB_OutEPs[epnum].xfer_remaining, USB_OutEPs[epnum].mps));

			}
			else
			{
				if((USB_ClassCallbackData->EP0_RxReady != NULLPTR) && (USB_DevState == USB_STATE_CONFIGURED))
				{
					USB_ClassCallbackData->EP0_RxReady();
				}
				// Send zero-length status packet
				USB_SendCtrlData(NULLPTR, 0);
			}
		}
	}
	else if((USB_ClassCallbackData->DataOut != NULLPTR)&& (USB_DevState == USB_STATE_CONFIGURED))
	{
		USB_ClassCallbackData->DataOut(epnum);
	}
}

void USB_TxFIFO_Empty_Callback(uint8_t epnum)
{
	// Determine how much data to fill into the FIFO
	int32_t bytes_to_send =
			USB_InEPs[epnum].xfer_len - USB_InEPs[epnum].xfer_done_count;
	uint32_t len32b;

	if(bytes_to_send > USB_InEPs[epnum].mps)
		bytes_to_send = USB_InEPs[epnum].mps;
	len32b = (bytes_to_send + 3) / 4;

	while  ( (USB_INEP(epnum)->DTXFSTS & USB_OTG_DTXFSTS_INEPTFSAV) > len32b &&
			USB_InEPs[epnum].xfer_done_count < USB_InEPs[epnum].xfer_len &&
			USB_InEPs[epnum].xfer_len != 0)
	{
		// Write the FIFO
		bytes_to_send = USB_InEPs[epnum].xfer_len - USB_InEPs[epnum].xfer_done_count;

		if (bytes_to_send > USB_InEPs[epnum].mps)
		{
			bytes_to_send = USB_InEPs[epnum].mps;
		}
		len32b = (bytes_to_send) / 4;

		USB_WritePacket(USB_InEPs[epnum].xfer_buffer, epnum, bytes_to_send);

		USB_InEPs[epnum].xfer_buffer  += bytes_to_send;
		USB_InEPs[epnum].xfer_done_count += bytes_to_send;
	}

	// No more bytes to send?
	if(bytes_to_send <= 0)
	{
		// Disable the FIFO empty interrupt
		USB_DEVICE->DIEPEMPMSK &= ~(0x1 << epnum);
	}

}

void USB_Start_OUTEP0(void)
{
	// Maximum setup packet count, set packet count to 1, and set transfer size high enough to grab a packet
	USB_OUTEP(0)->DOEPTSIZ = USB_OTG_DOEPTSIZ_STUPCNT | (1 << 19) | (3 * 8);
}

void USB_SetupDeviceCallback(void)
{
	uint8_t addr;
	switch(USB_SetupRequest.bRequest)
	{
	case USB_REQ_GET_DESCRIPTOR:
		USB_GetDescriptor();
		break;

	case USB_REQ_SET_ADDRESS:
		//USB_SetAddress();
		addr = USB_SetupRequest.wValue & 0x7F;
		USB_DEVICE->DCFG &= ~USB_OTG_DCFG_DAD;
		USB_DEVICE->DCFG |= (addr << 4) & USB_OTG_DCFG_DAD;
		// Send a zero-length status packet
		USB_SendCtrlData(NULLPTR,0);
		USB_DevState = USB_STATE_ADDRESSED;
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

void USB_SetupInterfaceCallback(void)
{

	switch (USB_DevState)
	{
	case USB_STATE_CONFIGURED:

		if (LOBYTE(USB_SetupRequest.wIndex) <= USB_MAX_NUM_INTERFACES)
		{
			if(USB_ClassCallbackData != NULLPTR && (USB_ClassCallbackData->Setup != NULLPTR))
				USB_ClassCallbackData->Setup(&USB_SetupRequest);

			if((USB_SetupRequest.wLength == 0))
			{
				USB_SendCtrlData(NULLPTR, 0);
			}
		}
		else
		{
			USB_CtrlError();
		}
		break;

	default:
		USB_CtrlError();
	break;
	}
}

void USB_SetupEndpointCallback(void)
{
	uint8_t   ep_addr;
	ep_addr  = LOBYTE(USB_SetupRequest.wIndex);

	switch (USB_SetupRequest.bRequest)
	{
	case USB_REQ_SET_FEATURE:

		switch (USB_DevState)
		{
		case USB_STATE_ADDRESSED:
			if ((ep_addr != 0x00) && (ep_addr != 0x80))
			{
				if(ep_addr & 0x80)
					USB_StallINEP(ep_addr & 0x7F);
				else
					USB_StallOUTEP(ep_addr & 0x7F);
			}
			break;

		case USB_STATE_CONFIGURED:
			if (USB_SetupRequest.wValue == USB_FEATURE_EP_HALT)
			{
				if ((ep_addr != 0x00) && (ep_addr != 0x80))
				{
					if(ep_addr & 0x80)
						USB_StallINEP(ep_addr & 0x7F);
					else
						USB_StallOUTEP(ep_addr & 0x7F);
				}
			}
			if(USB_ClassCallbackData != NULLPTR && (USB_ClassCallbackData->Setup != NULLPTR))
				USB_ClassCallbackData->Setup(&USB_SetupRequest);
			USB_SendCtrlData(NULLPTR, 0);

			break;

		default:
			USB_CtrlError();
			break;
		}
	break;

	case USB_REQ_CLEAR_FEATURE :

		switch (USB_DevState)
		{
		case USB_STATE_ADDRESSED:
			if ((ep_addr != 0x00) && (ep_addr != 0x80))
			{
				if(ep_addr & 0x80)
					USB_StallINEP(ep_addr & 0x7F);
				else
					USB_StallOUTEP(ep_addr & 0x7F);
			}
			break;

		case USB_STATE_CONFIGURED:
			if (USB_SetupRequest.wValue == USB_FEATURE_EP_HALT)
			{
				if ((ep_addr & 0x7F) != 0x00)
				{
					if(ep_addr & 0x80)
						USB_ClrStallINEP(ep_addr & 0x7F);
					else
						USB_ClrStallOUTEP(ep_addr & 0x7F);
				}
			}
			if(USB_ClassCallbackData != NULLPTR && (USB_ClassCallbackData->Setup != NULLPTR))
				USB_ClassCallbackData->Setup(&USB_SetupRequest);
			USB_SendCtrlData(NULLPTR, 0);
			break;

		default:
			USB_CtrlError();
			break;
		}
		break;

	case USB_REQ_GET_STATUS:
		switch (USB_DevState)
		{
		case USB_STATE_ADDRESSED:
			if ((ep_addr & 0x7F) != 0x00)
			{
				if(ep_addr & 0x80)
					USB_StallINEP(ep_addr & 0x7F);
				else
					USB_StallOUTEP(ep_addr & 0x7F);
			}
			break;

		case USB_STATE_CONFIGURED:
			if(ep_addr & 0x80)
			{
				if(USB_INEP(ep_addr&0x7F)->DIEPCTL & USB_OTG_DIEPCTL_STALL)
					USB_ConfigStatus = 0x0001;
				else
					USB_ConfigStatus = 0x0000;
			}
			else
			{
				if(USB_OUTEP(ep_addr&0x7F)->DOEPCTL & USB_OTG_DOEPCTL_STALL)
					USB_ConfigStatus = 0x0001;
				else
					USB_ConfigStatus = 0x0000;
			}
			USB_SendCtrlData((uint8_t*)&USB_ConfigStatus, 2);
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

void USB_GetDescriptor(void)
{
	uint16_t len;
	uint8_t *pbuf;

	switch (USB_SetupRequest.wValue >> 8)
	{
	case USB_DESC_TYPE_DEVICE:
		pbuf = USB_ClassDescData->GetDeviceDescriptor(&len);
		break;

	case USB_DESC_TYPE_CONFIGURATION:
		pbuf   = USB_ClassDescData->GetConfigDescriptor(&len);
		pbuf[1] = USB_DESC_TYPE_CONFIGURATION;
		break;

	case USB_DESC_TYPE_STRING:
		switch ((uint8_t)(USB_SetupRequest.wValue))
		{
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

	if((len != 0)&& (USB_SetupRequest.wLength != 0))
	{

		len = MIN(len , USB_SetupRequest.wLength);

		USB_SendCtrlData(pbuf, len);
	}

}

void USB_SetConfig(uint8_t configNum)
{
	if(configNum > USB_MAX_NUM_CONFIGURATION)
	{
		USB_CtrlError();
	}
	else
	{
		switch(USB_DevState)
		{
		case USB_STATE_ADDRESSED:
			if(configNum)
			{
				USB_DevState = USB_STATE_CONFIGURED;
				USB_CurrentConfig = configNum;
				// Call class config
				if(USB_ClassCallbackData != NULLPTR && (USB_ClassCallbackData->Init != NULLPTR))
				{
					USB_ClassCallbackData->Init(configNum);
				}
			}
			// Send a zero byte status packet
			USB_SendCtrlData(NULLPTR, 0);
			break;
		case USB_STATE_CONFIGURED:
			if(configNum == 0)
			{
				// Unconfig!
				USB_DevState = USB_STATE_ADDRESSED;
				USB_CurrentConfig = 0;
				if(USB_ClassCallbackData != NULLPTR && (USB_ClassCallbackData->DeInit != NULLPTR))
				{
					USB_ClassCallbackData->DeInit();
				}
				USB_SendCtrlData(NULLPTR, 0);
			}
			else if(configNum > USB_MAX_NUM_CONFIGURATION)
			{
				// Invalid config number
				USB_CtrlError();
			}
			else
			{
				USB_SendCtrlData(NULLPTR, 0);
			}
			break;
		default:
			USB_CtrlError();
		}
	}
}

void USB_GetConfig(void)
{
	if(USB_SetupRequest.wLength != 1)
		USB_CtrlError();
	else
	{
		switch(USB_DevState)
		{
		case USB_STATE_ADDRESSED:
		case USB_STATE_CONFIGURED:
			USB_SendCtrlData((uint8_t*)&USB_CurrentConfig, USB_SetupRequest.wLength);
			break;
		default:
			USB_CtrlError();
			break;
		}
	}
}

void USB_GetStatus(void)
{
	USB_ConfigStatus = USB_CONFIG_SELF_POWERED | USB_CONFIG_REMOTE_WAKEUP;
	USB_SendCtrlData((uint8_t*)&USB_ConfigStatus,2);
}

void USB_SetFeature(void)
{
	if (USB_SetupRequest.wValue == USB_FEATURE_REMOTE_WAKEUP)
	{
		USB_SendCtrlData(NULLPTR, 0);
	}
	else
	{
		USB_CtrlError();
	}
}

void USB_ClrFeature(void)
{
	if(USB_SetupRequest.wValue == USB_FEATURE_REMOTE_WAKEUP)
	{
		USB_SendCtrlData(NULLPTR, 0);
	}
	else
	{
		USB_CtrlError();
	}
}

void USB_ActivateINEP(uint8_t epnum, uint8_t eptype, uint32_t maxpacketsize)
{
	uint32_t temp = 0;
	if(epnum == 0)
	{
		if(maxpacketsize == 64)
			temp = 0;
		else if(maxpacketsize == 32)
			temp = 1;
		else if(maxpacketsize == 16)
			temp = 2;
		else if(maxpacketsize == 8)
			temp = 3;
		else
			temp = 0;
		USB_INEP(0)->DIEPCTL = temp | USB_OTG_DIEPCTL_CNAK;
		USB_FlushTxFifo(0);
	}
	else
	{
		temp = (maxpacketsize & 0x7FF) + (eptype << 18) + (epnum << 22) + USB_OTG_DIEPCTL_USBAEP;
		if((eptype == USB_EP_TYPE_BULK) || (eptype == USB_EP_TYPE_ISOC))
		{
			temp |= USB_OTG_DIEPCTL_SD0PID_SEVNFRM;
		}
		USB_INEP(epnum)->DIEPCTL = temp | USB_OTG_DIEPCTL_CNAK;
		USB_FlushTxFifo(epnum);
	}
	USB_DEVICE->DAINTMSK |= (1 << epnum);
	USB_InEPs[epnum].mps = maxpacketsize;
}

void USB_ActivateOUTEP(uint8_t epnum, uint8_t eptype, uint32_t maxpacketsize)
{
	uint32_t temp = 0;
	if(epnum == 0)
	{
		USB_OUTEP(0)->DOEPCTL = USB_OTG_DOEPCTL_CNAK;
	}
	else
	{
		temp = (maxpacketsize & 0x7FF) + (eptype << 18) + (epnum << 22) + USB_OTG_DOEPCTL_USBAEP;
		if((eptype == USB_EP_TYPE_BULK) || (eptype == USB_EP_TYPE_ISOC))
		{
			temp |= USB_OTG_DOEPCTL_SD0PID_SEVNFRM;
		}
		USB_OUTEP(epnum)->DOEPCTL = temp | USB_OTG_DOEPCTL_CNAK;
	}
	USB_DEVICE->DAINTMSK |= (1 << (16+epnum));
	USB_OutEPs[epnum].mps = maxpacketsize;
}

void USB_DeactivateINEP(uint8_t epnum)
{
	USB_DEVICE->DAINTMSK &= ~(1 << epnum);
	USB_INEP(epnum)->DIEPCTL &= ~(USB_OTG_DIEPCTL_USBAEP);
}

void USB_DeactivateOUTEP(uint8_t epnum)
{
	USB_DEVICE->DAINTMSK &= ~(1 << (16+epnum));
	USB_OUTEP(epnum)->DOEPCTL &= ~(USB_OTG_DOEPCTL_USBAEP);
}

void USB_StallINEP(uint8_t epnum)
{
	if(USB_INEP(epnum)->DIEPCTL & USB_OTG_DIEPCTL_EPENA)
	{
		USB_INEP(epnum)->DIEPCTL |= USB_OTG_DIEPCTL_EPDIS;
	}
	USB_INEP(epnum)->DIEPCTL |= USB_OTG_DIEPCTL_STALL;
	if(epnum == 0)
		USB_Start_OUTEP0();
}

void USB_StallOUTEP(uint8_t epnum)
{
	if(USB_OUTEP(epnum)->DOEPCTL & USB_OTG_DOEPCTL_EPENA)
	{
		USB_OUTEP(epnum)->DOEPCTL |= USB_OTG_DOEPCTL_EPDIS;
	}
	USB_OUTEP(epnum)->DOEPCTL |= USB_OTG_DOEPCTL_STALL;
	if(epnum == 0)
		USB_Start_OUTEP0();
}

void USB_ClrStallINEP(uint8_t epnum)
{
	if(epnum != 0)
	{
		USB_INEP(epnum)->DIEPCTL &= ~(USB_OTG_DIEPCTL_STALL);
		if(((USB_INEP(epnum)->DIEPCTL & USB_OTG_DIEPCTL_EPTYP) >> 18) == USB_EP_TYPE_ISOC)
		{
			USB_INEP(epnum)->DIEPCTL |= USB_OTG_DIEPCTL_SD0PID_SEVNFRM;
		}
	}
}

void USB_ClrStallOUTEP(uint8_t epnum)
{
	if(epnum != 0)
	{
		USB_OUTEP(epnum)->DOEPCTL &= ~(USB_OTG_DOEPCTL_STALL);
		if(((USB_OUTEP(epnum)->DOEPCTL & USB_OTG_DOEPCTL_EPTYP) >> 18) == USB_EP_TYPE_ISOC)
		{
			USB_OUTEP(epnum)->DOEPCTL |= USB_OTG_DOEPCTL_SD0PID_SEVNFRM;
		}
	}
}

void USB_CtrlError(void)
{
	USB_StallINEP(0);
	USB_StallOUTEP(0);
}

uint32_t USB_GetRxDataSize(uint8_t epnum)
{
	return USB_OutEPs[epnum].xfer_done_count;
}

void USB_SendData(uint8_t *pbuf, uint8_t epnum, uint16_t len)
{
	USB_InEPs[epnum].xfer_buffer = pbuf;
	USB_InEPs[epnum].xfer_len = len;
	USB_InEPs[epnum].total_xfer_len = len;
	USB_InEPs[epnum].xfer_remaining = len;
	USB_InEPs[epnum].xfer_done_count = 0;

	// Flush this endpoint's TX buffer
	// I shouldn't have to flush the FIFO. But without doing this, the IN packets fail to send!
	USB_FlushTxFifo(epnum);

	if(epnum == 0)
		USB_Start_INEP0_Transfer(len);
	else
		USB_Start_INEP_Transfer(epnum, len);
}

void USB_SendCtrlData(uint8_t *pbuf, uint16_t len)
{
	USB_EP0_State = USB_EP0_DATA_IN;
	USB_SendData(pbuf, 0, len);
}

void USB_PrepareRead(uint8_t *pbuf, uint8_t epnum, uint16_t len)
{
	USB_OutEPs[epnum].xfer_buffer = pbuf;
	USB_OutEPs[epnum].xfer_len = len;
	USB_OutEPs[epnum].total_xfer_len = len;
	USB_OutEPs[epnum].xfer_remaining = len;
	USB_OutEPs[epnum].xfer_done_count = 0;

	if(epnum == 0)
		USB_Start_OUTEP0_Transfer(len);
	else
		USB_Start_OUTEP_Transfer(epnum, len);
}

void USB_PrepareCtrlRead(uint8_t *pbuf, uint16_t len)
{
	USB_EP0_State = USB_EP0_DATA_OUT;
	USB_PrepareRead(pbuf, 0, len);
}

void USB_ReadPacket(uint8_t* buf, uint16_t len)
{
	uint32_t* buf32 = (uint32_t*)buf;
	uint16_t len32 = (len+3) / 4;

	for(uint16_t i=0; i < len32; i++)
	{
		*buf32 = USB_DFIFO(0);
		buf32++;
	}
}

void USB_WritePacket(uint8_t* buf, uint8_t epnum, uint16_t len)
{
	uint16_t len32 = (len+3) / 4;
	for(uint16_t i = 0; i < len32; i++)
	{
		USB_DFIFO(epnum) = *((__attribute__((__packed__)) uint32_t *)buf);
		buf+=4;
	}
}

void USB_Start_INEP0_Transfer(uint32_t len)
{
	if(len == 0)
	{
		// Sending a zero-length packet
	  USB_INEP(0)->DIEPTSIZ &= ~(USB_OTG_DIEPTSIZ_PKTCNT);
	  USB_INEP(0)->DIEPTSIZ |= (USB_OTG_DIEPTSIZ_PKTCNT & (1 << 19)) ;
	  USB_INEP(0)->DIEPTSIZ &= ~(USB_OTG_DIEPTSIZ_XFRSIZ);
	}
	else
	{
		/* Program the transfer size and packet count
		 * as follows: xfersize = N * maxpacket + short_packet
		 * pktcnt = N + (short_packet exist ? 1 : 0)
		 */
		USB_INEP(0)->DIEPTSIZ &= ~(USB_OTG_DIEPTSIZ_XFRSIZ);
		USB_INEP(0)->DIEPTSIZ &= ~(USB_OTG_DIEPTSIZ_PKTCNT);

		if(len > USB_MAX_EP0_SIZE)
		{
			len = USB_MAX_EP0_SIZE;
		}
		USB_InEPs[0].xfer_len = len;
		USB_InEPs[0].xfer_done_count = 0;
		USB_INEP(0)->DIEPTSIZ |= (USB_OTG_DIEPTSIZ_PKTCNT & (1 << 19)) ;
		USB_INEP(0)->DIEPTSIZ |= (USB_OTG_DIEPTSIZ_XFRSIZ & len);
		USB_DEVICE->DIEPEMPMSK |= (1 << 0); // Enable Tx FIFO empty interrupt
	}
	// Endpoint enable
	USB_INEP(0)->DIEPCTL |= (USB_OTG_DIEPCTL_CNAK | USB_OTG_DIEPCTL_EPENA);
}

void USB_Start_INEP_Transfer(uint8_t epnum, uint32_t len)
{
	// Zero Length Packet?
	if (len == 0)
	{
		USB_INEP(epnum)->DIEPTSIZ &= ~(USB_OTG_DIEPTSIZ_PKTCNT);
		USB_INEP(epnum)->DIEPTSIZ |= (USB_OTG_DIEPTSIZ_PKTCNT & (1 << 19)) ;
		USB_INEP(epnum)->DIEPTSIZ &= ~(USB_OTG_DIEPTSIZ_XFRSIZ);
	}
	else
	{
		USB_InEPs[0].xfer_len = len;
		USB_InEPs[0].xfer_done_count = 0;
		/* Program the transfer size and packet count
		 * as follows: xfersize = N * maxpacket + short_packet
		 * pktcnt = N + (short_packet exist ? 1 : 0)
		 */
		USB_INEP(epnum)->DIEPTSIZ &= ~(USB_OTG_DIEPTSIZ_XFRSIZ);
		USB_INEP(epnum)->DIEPTSIZ &= ~(USB_OTG_DIEPTSIZ_PKTCNT);
		USB_INEP(epnum)->DIEPTSIZ |= (USB_OTG_DIEPTSIZ_PKTCNT & (((len + USB_InEPs[epnum].mps -1)/ USB_InEPs[epnum].mps) << 19)) ;
		USB_INEP(epnum)->DIEPTSIZ |= (USB_OTG_DIEPTSIZ_XFRSIZ & len);
		USB_DEVICE->DIEPEMPMSK |= (1 << epnum); // Enable Tx FIFO empty interrupt

		if(((USB_INEP(epnum)->DIEPCTL & USB_OTG_DIEPCTL_EPTYP) >> 18) == USB_EP_TYPE_ISOC)
		{
			USB_INEP(epnum)->DIEPTSIZ &= ~(USB_OTG_DIEPTSIZ_MULCNT);
			USB_INEP(epnum)->DIEPTSIZ |= (USB_OTG_DIEPTSIZ_MULCNT & (1 << 29));
			if ((USB_DEVICE->DSTS & ( 1 << 8 )) == 0)
			{
				USB_INEP(epnum)->DIEPCTL |= USB_OTG_DIEPCTL_SODDFRM;
			}
			else
			{
				USB_INEP(epnum)->DIEPCTL |= USB_OTG_DIEPCTL_SD0PID_SEVNFRM;
			}
		}
	}
	USB_INEP(epnum)->DIEPCTL |= (USB_OTG_DIEPCTL_CNAK | USB_OTG_DIEPCTL_EPENA);
	if(((USB_INEP(epnum)->DIEPCTL & USB_OTG_DIEPCTL_EPTYP) >> 18) == USB_EP_TYPE_ISOC)
	{
		USB_WritePacket(USB_InEPs[epnum].xfer_buffer, epnum, len);
	}
}

void USB_Start_OUTEP0_Transfer(uint32_t len)
{
	/* Program the transfer size and packet count as follows:
	 * pktcnt = 1
	 * xfersize = 1 * maxpacket
	 */
	USB_OUTEP(0)->DOEPTSIZ &= ~(USB_OTG_DOEPTSIZ_XFRSIZ);
	USB_OUTEP(0)->DOEPTSIZ &= ~(USB_OTG_DOEPTSIZ_PKTCNT);

	if (len > 0)
	{
		len = USB_MAX_EP0_SIZE;
	}

	USB_OUTEP(0)->DOEPTSIZ |= (USB_OTG_DOEPTSIZ_PKTCNT & (1 << 19));
	USB_OUTEP(0)->DOEPTSIZ |= (USB_OTG_DOEPTSIZ_XFRSIZ & (USB_MAX_EP0_SIZE));

	/* EP enable */
	USB_OUTEP(0)->DOEPCTL |= (USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA);
}

void USB_Start_OUTEP_Transfer(uint8_t epnum, uint32_t len)
{
	uint16_t pktcnt;

	USB_OUTEP(epnum)->DOEPTSIZ &= ~(USB_OTG_DOEPTSIZ_XFRSIZ);
	USB_OUTEP(epnum)->DOEPTSIZ &= ~(USB_OTG_DOEPTSIZ_PKTCNT);
	// Zero Length Packet?
	if (len == 0)
	{
		USB_OUTEP(epnum)->DOEPTSIZ |= (USB_OTG_DOEPTSIZ_XFRSIZ & USB_OutEPs[epnum].mps);
		USB_OUTEP(epnum)->DOEPTSIZ |= (USB_OTG_DOEPTSIZ_PKTCNT & (1 << 19)) ;
	}
	else
	{
		pktcnt = (len + USB_OutEPs[epnum].mps -1)/ USB_OutEPs[epnum].mps;
		USB_OUTEP(epnum)->DOEPTSIZ |= (USB_OTG_DOEPTSIZ_PKTCNT & (pktcnt << 19)); ;
		USB_OUTEP(epnum)->DOEPTSIZ |= (USB_OTG_DOEPTSIZ_XFRSIZ & (USB_OutEPs[epnum].mps * pktcnt));
	}
	if(((USB_OUTEP(epnum)->DOEPCTL & USB_OTG_DOEPCTL_EPTYP) >> 18) == USB_EP_TYPE_ISOC)
	{
		if ((USB_DEVICE->DSTS & ( 1 << 8 )) == 0)
		{
			USB_OUTEP(epnum)->DOEPCTL |= USB_OTG_DOEPCTL_SODDFRM;
		}
		else
		{
			USB_OUTEP(epnum)->DOEPCTL |= USB_OTG_DOEPCTL_SD0PID_SEVNFRM;
		}
	}
	/* EP enable */
	USB_OUTEP(epnum)->DOEPCTL |= (USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA);
}
