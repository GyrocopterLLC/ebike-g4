/******************************************************************************
 * Filename: usb_cdc.c
 * Description: Communication device class (CDC) driver for USB.
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

#include "stm32f4xx.h"
#include "usb.h"
#include "usb_cdc.h"
#include <string.h>

USB_ClassDescTypedef USB_CDC_ClassDesc =
{
	CDC_DeviceDescriptor,
	CDC_ConfigDescriptor,
	CDC_LangIDStrDescriptor,
	CDC_ManufacturerStrDescriptor,
	CDC_ProductStrDescriptor,
	CDC_SerialStrDescriptor,
	CDC_ConfigurationStrDescriptor,
	CDC_InterfaceStrDescriptor
};

USB_ClassCallbackTypedef USB_CDC_ClassCallbacks =
{
	CDC_Connect,
	CDC_Disconnect,
	CDC_Reset,
	CDC_Init,
	CDC_DeInit,
	CDC_Setup,
	NULLPTR, // EP0_TxReady
	CDC_EP0_RxReady,
	CDC_DataIn,
	CDC_DataOut,
	NULLPTR, // SOF
	NULLPTR, // Isoc IN incomplete
	NULLPTR  // Isoc OUT incomplete
};

uint8_t USB_CDC_DevDesc[USB_LEN_DEV_DESC] __attribute__ ((aligned (4))) =
{
	0x12,                       /* bLength */
	USB_DESC_TYPE_DEVICE,       /* bDescriptorType */
	0x00,                       /* bcdUSB */
	0x02,
	0x00,                       /* bDeviceClass */
	0x00,                       /* bDeviceSubClass */
	0x00,                       /* bDeviceProtocol */
	USB_MAX_EP0_SIZE,           /* bMaxPacketSize */
	LOBYTE(USB_VID),           /* idVendor */
	HIBYTE(USB_VID),           /* idVendor */
	LOBYTE(USB_PID),           /* idVendor */
	HIBYTE(USB_PID),           /* idVendor */
	0x00,                       /* bcdDevice rel. 2.00 */
	0x02,
	USB_IDX_MFC_STR,           /* Index of manufacturer string */
	USB_IDX_PRODUCT_STR,       /* Index of product string */
	USB_IDX_SERIAL_STR,        /* Index of serial number string */
	USB_MAX_NUM_CONFIGURATION  /* bNumConfigurations */
};

uint8_t USB_CDC_CfgDesc[USB_CDC_CONFIG_DESC_SIZ] __attribute__ ((aligned (4))) =
{
  /*Configuration Descriptor*/
  0x09,   /* bLength: Configuration Descriptor size */
  USB_DESC_TYPE_CONFIGURATION,      /* bDescriptorType: Configuration */
  USB_CDC_CONFIG_DESC_SIZ,                /* wTotalLength:no of returned bytes */
  0x00,
  0x02,   /* bNumInterfaces: 2 interface */
  0x01,   /* bConfigurationValue: Configuration value */
  0x00,   /* iConfiguration: Index of string descriptor describing the configuration */
  0xC0,   /* bmAttributes: self powered */
  0x32,   /* MaxPower 0 mA */

  /*---------------------------------------------------------------------------*/

  /*Interface Descriptor */
  0x09,   /* bLength: Interface Descriptor size */
  USB_DESC_TYPE_INTERFACE,  /* bDescriptorType: Interface */
  /* Interface descriptor type */
  0x00,   /* bInterfaceNumber: Number of Interface */
  0x00,   /* bAlternateSetting: Alternate setting */
  0x01,   /* bNumEndpoints: One endpoints used */
  0x02,   /* bInterfaceClass: Communication Interface Class */
  0x02,   /* bInterfaceSubClass: Abstract Control Model */
  0x01,   /* bInterfaceProtocol: Common AT commands */
  0x00,   /* iInterface: */

  /*Header Functional Descriptor*/
  0x05,   /* bLength: Endpoint Descriptor size */
  0x24,   /* bDescriptorType: CS_INTERFACE */
  0x00,   /* bDescriptorSubtype: Header Func Desc */
  0x10,   /* bcdCDC: spec release number */
  0x01,

  /*Call Management Functional Descriptor*/
  0x05,   /* bFunctionLength */
  0x24,   /* bDescriptorType: CS_INTERFACE */
  0x01,   /* bDescriptorSubtype: Call Management Func Desc */
  0x00,   /* bmCapabilities: D0+D1 */
  0x01,   /* bDataInterface: 1 */

  /*ACM Functional Descriptor*/
  0x04,   /* bFunctionLength */
  0x24,   /* bDescriptorType: CS_INTERFACE */
  0x02,   /* bDescriptorSubtype: Abstract Control Management desc */
  0x02,   /* bmCapabilities */

  /*Union Functional Descriptor*/
  0x05,   /* bFunctionLength */
  0x24,   /* bDescriptorType: CS_INTERFACE */
  0x06,   /* bDescriptorSubtype: Union func desc */
  0x00,   /* bMasterInterface: Communication class interface */
  0x01,   /* bSlaveInterface0: Data Class Interface */

  /*Endpoint 2 Descriptor*/
  0x07,                           /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,   /* bDescriptorType: Endpoint */
  CDC_CMD_EP,                     /* bEndpointAddress */
  0x03,                           /* bmAttributes: Interrupt */
  LOBYTE(CDC_CMD_PACKET_SIZE),     /* wMaxPacketSize: */
  HIBYTE(CDC_CMD_PACKET_SIZE),
  0x10,                           /* bInterval: */
  /*---------------------------------------------------------------------------*/

  /*Data class interface descriptor*/
  0x09,   /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_INTERFACE,  /* bDescriptorType: */
  0x01,   /* bInterfaceNumber: Number of Interface */
  0x00,   /* bAlternateSetting: Alternate setting */
  0x02,   /* bNumEndpoints: Two endpoints used */
  0x0A,   /* bInterfaceClass: CDC */
  0x00,   /* bInterfaceSubClass: */
  0x00,   /* bInterfaceProtocol: */
  0x00,   /* iInterface: */

  /*Endpoint OUT Descriptor*/
  0x07,   /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,      /* bDescriptorType: Endpoint */
  CDC_OUT_EP,                        /* bEndpointAddress */
  0x02,                              /* bmAttributes: Bulk */
  LOBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),  /* wMaxPacketSize: */
  HIBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),
  0x00,                              /* bInterval: ignore for Bulk transfer */

  /*Endpoint IN Descriptor*/
  0x07,   /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,      /* bDescriptorType: Endpoint */
  CDC_IN_EP,                         /* bEndpointAddress */
  0x02,                              /* bmAttributes: Bulk */
  LOBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),  /* wMaxPacketSize: */
  HIBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),
  0x00                               /* bInterval: ignore for Bulk transfer */
} ;

uint8_t USB_StrDesc[USB_MAX_STR_DESC_SIZ] __attribute__ ((aligned (4)));

uint8_t USB_StringSerial[USB_SIZ_STRING_SERIAL] =
{
  USB_SIZ_STRING_SERIAL,
  USB_DESC_TYPE_STRING,
};

uint8_t USB_LangIDDesc[USB_LEN_LANGID_STR_DESC] __attribute__ ((aligned (4)))= {
  USB_LEN_LANGID_STR_DESC,
  USB_DESC_TYPE_STRING,
  LOBYTE(USB_LANGID_STRING),
  HIBYTE(USB_LANGID_STRING),
};

USBD_CDC_HandleTypeDef USB_CDC_ClassData; // Buffer to hold all the class data for its specific transactions
USB_CDC_RxBufferTypedef USB_CDC_RxBuffer;
uint8_t USB_CDC_TxBuffer[TX_FIFO_BUF_SIZE];

USBD_CDC_LineCodingTypeDef LineCoding =
  {
    115200, /* baud rate*/
    0x00,   /* stop bits-1*/
    0x00,   /* parity - none*/
    0x08    /* nb. of bits 8*/
  };

static void IntToUnicode (uint32_t value , uint8_t *pbuf , uint8_t len);
static void Get_SerialNum(void);
static void USB_GetString(uint8_t *desc, uint8_t *unicode, uint16_t *len);
static uint8_t USB_GetLen(uint8_t *buf);

uint8_t* CDC_DeviceDescriptor(uint16_t* len)
{
	*len = USB_LEN_DEV_DESC;
	return USB_CDC_DevDesc;
}

uint8_t* CDC_ConfigDescriptor(uint16_t* len)
{
	*len = USB_CDC_CONFIG_DESC_SIZ;
	return USB_CDC_CfgDesc;
}

uint8_t* CDC_LangIDStrDescriptor(uint16_t* len)
{
	*len = sizeof(USB_LangIDDesc);
	return (uint8_t*)USB_LangIDDesc;
}

uint8_t* CDC_ManufacturerStrDescriptor(uint16_t* len)
{
	USB_GetString((uint8_t *)USB_MANUFACTURER_STRING, USB_StrDesc, len);
	return USB_StrDesc;
}

uint8_t* CDC_ProductStrDescriptor(uint16_t* len)
{
	USB_GetString((uint8_t *)USB_PRODUCT_FS_STRING, USB_StrDesc, len);
	return USB_StrDesc;
}

uint8_t* CDC_SerialStrDescriptor(uint16_t* len)
{
	*len = USB_SIZ_STRING_SERIAL;

	/* Update the serial number string descriptor with the data from the unique ID*/
	Get_SerialNum();

	return (uint8_t*)USB_StringSerial;
}

uint8_t* CDC_ConfigurationStrDescriptor(uint16_t* len)
{
	USB_GetString((uint8_t *)USB_CONFIGURATION_FS_STRING, USB_StrDesc, len);
	return USB_StrDesc;
}

uint8_t* CDC_InterfaceStrDescriptor(uint16_t* len)
{
	USB_GetString((uint8_t *)USB_INTERFACE_FS_STRING, USB_StrDesc, len);
	return USB_StrDesc;
}

void CDC_Init(uint8_t configNum)
{
	if(configNum <= USB_MAX_NUM_CONFIGURATION)
	{
		// Activate the endpoints!
		USB_ActivateINEP(DATA_IN_EP, USB_EP_TYPE_BULK, DATA_ENDPOINT_FIFO_SIZE);
		USB_ActivateINEP(CMD_IN_EP, USB_EP_TYPE_INTR, CMD_ENDPOINT_FIFO_SIZE);
		USB_ActivateOUTEP(DATA_OUT_EP, USB_EP_TYPE_BULK, DATA_ENDPOINT_FIFO_SIZE);

		// Set Tx and Rx buffers
		USB_CDC_ClassData.RxBuffer = USB_CDC_RxBuffer.Buffer;
		USB_CDC_ClassData.TxBuffer = USB_CDC_TxBuffer;
		USB_CDC_ClassData.RxLength = 0;
		USB_CDC_ClassData.TxLength = 0;
		// Configure OUT EP to receive next packet
		USB_PrepareRead(USB_CDC_ClassData.RxBuffer, DATA_OUT_EP, CDC_DATA_FS_MAX_PACKET_SIZE);
	}
}

void CDC_DeInit(void)
{
	// Deactivate all other endpoints besides EP0
	USB_DeactivateINEP(DATA_IN_EP);
	USB_DeactivateINEP(CMD_IN_EP);
	USB_DeactivateOUTEP(DATA_OUT_EP);

}

void CDC_Setup(USB_SetupReqTypedef* stp)
{
	// Decode setup packet
	switch (stp->bmRequest & USB_REQ_TYPE_MASK)
	{
	case USB_REQ_TYPE_CLASS :
		if (stp->wLength)
		{
			if (stp->bmRequest & 0x80)
			{
				CDC_Itf_Control(stp->bRequest, (uint8_t*)(USB_CDC_ClassData.data), stp->wLength);
				USB_SendCtrlData((uint8_t*)(USB_CDC_ClassData.data), stp->wLength);
			}
			else
			{
				USB_CDC_ClassData.CmdOpCode = stp->bRequest;
				USB_CDC_ClassData.CmdLength = stp->wLength;

				// Start a control OUT data transfer on EP0
				USB_PrepareCtrlRead((uint8_t*)(USB_CDC_ClassData.data), stp->wLength);
			}

		}
		else
		{
			CDC_Itf_Control(stp->bRequest, NULLPTR, 0);
		}
		break;

	default:
		break;
	}

}

void CDC_EP0_RxReady(void)
{
	if(USB_CDC_ClassData.CmdOpCode != 0xFF)
	{
		CDC_Itf_Control(USB_CDC_ClassData.CmdOpCode,(uint8_t*)USB_CDC_ClassData.data, USB_CDC_ClassData.CmdLength);
		USB_CDC_ClassData.CmdOpCode = 0xFF;

	}
}

void CDC_DataIn(uint8_t epnum)
{
	// We finished transmitting!
	if(epnum == DATA_IN_EP)
	{
		//CDC.TransmitState = 0;
		USB_CDC_ClassData.TxState = 0;
		if(USB_CDC_ClassData.App_TxCompleteCallback != NULLPTR)
		{
		  USB_CDC_ClassData.App_TxCompleteCallback();
		}
	}

}

void CDC_DataOut(uint8_t epnum)
{
	// New data arrived
	if(epnum == DATA_OUT_EP)
	{
		USB_CDC_ClassData.RxState = 1;
		USB_CDC_ClassData.RxLength = USB_GetRxDataSize(epnum);
		USB_CDC_RxBuffer.Position = 0;
		USB_CDC_RxBuffer.Size = USB_CDC_ClassData.RxLength;
		USB_CDC_RxBuffer.ReadDone = 1;
		// Next packet reception is enabled by VCP_Read.
		// This means that the USB core will NAK all packets until the
		// application reads the buffer.
	}

}

void CDC_Connect(void)
{
	// Nothing
}

void CDC_Disconnect(void)
{
	// Close endpoints and everything
	CDC_DeInit();
}

void CDC_Reset(void)
{
	// Start it all up!
	// Open EP0 IN and OUT
	USB_ActivateINEP(0,USB_EP_TYPE_CTRL,USB_FS_MAX_PACKET_SIZE);
	USB_ActivateOUTEP(0,USB_EP_TYPE_CTRL,USB_FS_MAX_PACKET_SIZE);
}

void CDC_Itf_Control(uint8_t cmd, uint8_t* pbuf, uint16_t length)
{
	((void)length);
	switch (cmd)
	{
	case CDC_SEND_ENCAPSULATED_COMMAND:
		/* Add your code here */
		break;

	case CDC_GET_ENCAPSULATED_RESPONSE:
		/* Add your code here */
		break;

	case CDC_SET_COMM_FEATURE:
		/* Add your code here */
		break;

	case CDC_GET_COMM_FEATURE:
		/* Add your code here */
		break;

	case CDC_CLEAR_COMM_FEATURE:
		/* Add your code here */
		break;

	case CDC_SET_LINE_CODING:
		LineCoding.bitrate    = (uint32_t)(pbuf[0] | (pbuf[1] << 8) |\
				(pbuf[2] << 16) | (pbuf[3] << 24));
		LineCoding.format     = pbuf[4];
		LineCoding.paritytype = pbuf[5];
		LineCoding.datatype   = pbuf[6];

		/* Set the new configuration */
	break;

	case CDC_GET_LINE_CODING:
		pbuf[0] = (uint8_t)(LineCoding.bitrate);
		pbuf[1] = (uint8_t)(LineCoding.bitrate >> 8);
		pbuf[2] = (uint8_t)(LineCoding.bitrate >> 16);
		pbuf[3] = (uint8_t)(LineCoding.bitrate >> 24);
		pbuf[4] = LineCoding.format;
		pbuf[5] = LineCoding.paritytype;
		pbuf[6] = LineCoding.datatype;
		break;

	case CDC_SET_CONTROL_LINE_STATE:
		/* Add your code here */
		break;

	case CDC_SEND_BREAK:
		/* Add your code here */
		break;

	default:
		break;
	}
}

void CDC_SetTxCompleteCallback(void(*Callback)(void))
{
  USB_CDC_ClassData.App_TxCompleteCallback = Callback;
}

static void Get_SerialNum(void)
{
  uint32_t deviceserial0, deviceserial1, deviceserial2;

  deviceserial0 = *(uint32_t*)DEVICE_ID1;
  deviceserial1 = *(uint32_t*)DEVICE_ID2;
  deviceserial2 = *(uint32_t*)DEVICE_ID3;

  deviceserial0 += deviceserial2;

  if (deviceserial0 != 0)
  {
    IntToUnicode (deviceserial0, &USB_StringSerial[2] ,8);
    IntToUnicode (deviceserial1, &USB_StringSerial[18] ,4);
  }
}

static void IntToUnicode (uint32_t value , uint8_t *pbuf , uint8_t len)
{
  uint8_t idx = 0;

  for( idx = 0; idx < len; idx ++)
  {
    if( ((value >> 28)) < 0xA )
    {
      pbuf[ 2* idx] = (value >> 28) + '0';
    }
    else
    {
      pbuf[2* idx] = (value >> 28) + 'A' - 10;
    }

    value = value << 4;

    pbuf[ 2* idx + 1] = 0;
  }
}

static void USB_GetString(uint8_t *desc, uint8_t *unicode, uint16_t *len)
{
  uint8_t idx = 0;

  if (desc != NULLPTR)
  {
    *len =  USB_GetLen(desc) * 2 + 2;
    unicode[idx++] = *len;
    unicode[idx++] =  USB_DESC_TYPE_STRING;

    while (*desc != '\0')
    {
      unicode[idx++] = *desc++;
      unicode[idx++] =  0x00;
    }
  }
}

static uint8_t USB_GetLen(uint8_t *buf)
{
    uint8_t  len = 0;

    while (*buf != '\0')
    {
        len++;
        buf++;
    }

    return len;
}

int32_t VCP_InWaiting(void) {
	int32_t remaining = USB_CDC_RxBuffer.Size - USB_CDC_RxBuffer.Position;
	return remaining;
}

int32_t VCP_Read(void* data, int32_t len)
{
	if (!USB_CDC_RxBuffer.ReadDone)
		return 0;

	int32_t remaining = USB_CDC_RxBuffer.Size - USB_CDC_RxBuffer.Position;
	int32_t todo = MIN(remaining, len);
	if (todo <= 0)
		return 0;

	memcpy(data, USB_CDC_RxBuffer.Buffer + USB_CDC_RxBuffer.Position, todo);
	USB_CDC_RxBuffer.Position += todo;
	if (USB_CDC_RxBuffer.Position >= USB_CDC_RxBuffer.Size)
	{
		USB_CDC_RxBuffer.ReadDone = 0;
		USB_PrepareRead(USB_CDC_RxBuffer.Buffer, DATA_OUT_EP, DATA_ENDPOINT_FIFO_SIZE);
	}

	return todo;
}

int32_t VCP_Write(const void* data, int32_t len)
{

  if(USB_CDC_ClassData.TxState)
  {
    // Fail if still sending last data
    return -1;
  }
  int32_t len_to_send = len;
  if(USB_GetDevState() != USB_STATE_CONFIGURED)
    return 0;
  if(len_to_send > (TX_FIFO_BUF_SIZE/2))
  {
    len_to_send = (TX_FIFO_BUF_SIZE/2);
  }
  memcpy(USB_CDC_TxBuffer, data, len_to_send);
  USB_CDC_ClassData.TxBuffer = USB_CDC_TxBuffer;
  USB_CDC_ClassData.TxLength = len_to_send;
  USB_CDC_ClassData.TxState = 1;
  USB_SendData(USB_CDC_ClassData.TxBuffer,DATA_IN_EP,USB_CDC_ClassData.TxLength);
  return len_to_send;
}
