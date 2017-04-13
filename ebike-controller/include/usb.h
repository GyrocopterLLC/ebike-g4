/*
 * usb.h
 *
 *  Created on: Feb 20, 2017
 *      Author: David
 */

#ifndef USB_H_
#define USB_H_

#define MIN(a,b)	(a) < (b) ? (a) : (b)
#define MAX(a,b)	(a) > (b) ? (a) : (b)

#define LOBYTE(a)	(uint8_t)((a) & 0xFF)
#define HIBYTE(a)	(uint8_t)(((a) & 0xFF00) >> 8)

#if !defined(NULLPTR)
#define NULLPTR		((void*)0)
#endif

#define STS_GNAK			1
#define STS_DATA_UPDT		2
#define STS_DATA_COMP		3
#define STS_SETUP_COMP		4
#define STS_SETUP_UPDT		6

#define USB_PORT			GPIOA
#define USB_DM_PIN			11
#define USB_DP_PIN			12
#define USB_AF				10

#define PRIO_USB			6
#define CMD_FIFO_BUF_SIZE	128
#define RX_FIFO_BUF_SIZE	128
#define TX_FIFO_BUF_SIZE	128
#define NUM_ENDPOINTS		4 // Default EP0 plus 3 additional endpoints

#define  USB_LEN_DEV_QUALIFIER_DESC                     0x0A
#define  USB_LEN_DEV_DESC                               0x12
#define  USB_LEN_CFG_DESC                               0x09
#define  USB_LEN_IF_DESC                                0x09
#define  USB_LEN_EP_DESC                                0x07
#define  USB_LEN_OTG_DESC                               0x03
#define  USB_LEN_LANGID_STR_DESC                        0x04
#define  USB_LEN_OTHER_SPEED_DESC_SIZ                   0x09

#define CDC_IN_EP                                   0x81  /* EP1 for data IN */
#define CDC_OUT_EP                                  0x01  /* EP1 for data OUT */
#define CDC_CMD_EP                                  0x82  /* EP2 for CDC commands */

/* CDC Endpoints parameters: you can fine tune these values depending on the needed baudrates and performance. */
#define CDC_DATA_HS_MAX_PACKET_SIZE                 512  /* Endpoint IN & OUT Packet size */
#define CDC_DATA_FS_MAX_PACKET_SIZE                 64  /* Endpoint IN & OUT Packet size */
#define CDC_CMD_PACKET_SIZE                         8  /* Control Endpoint Packet size */
#define	 USB_CDC_CONFIG_DESC_SIZ                     	67

#define  USB_IDX_LANGID_STR				0x00
#define  USB_IDX_MFC_STR				0x01
#define  USB_IDX_PRODUCT_STR			0x02
#define  USB_IDX_SERIAL_STR				0x03
#define  USB_IDX_CONFIG_STR				0x04
#define  USB_IDX_INTERFACE_STR			0x05

#define  USB_REQ_TYPE_STANDARD                          0x00
#define  USB_REQ_TYPE_CLASS                             0x20
#define  USB_REQ_TYPE_VENDOR                            0x40
#define  USB_REQ_TYPE_MASK                              0x60

#define  USB_REQ_RECIPIENT_DEVICE                       0x00
#define  USB_REQ_RECIPIENT_INTERFACE                    0x01
#define  USB_REQ_RECIPIENT_ENDPOINT                     0x02
#define  USB_REQ_RECIPIENT_MASK                         0x03

#define  USB_REQ_GET_STATUS                             0x00
#define  USB_REQ_CLEAR_FEATURE                          0x01
#define  USB_REQ_SET_FEATURE                            0x03
#define  USB_REQ_SET_ADDRESS                            0x05
#define  USB_REQ_GET_DESCRIPTOR                         0x06
#define  USB_REQ_SET_DESCRIPTOR                         0x07
#define  USB_REQ_GET_CONFIGURATION                      0x08
#define  USB_REQ_SET_CONFIGURATION                      0x09
#define  USB_REQ_GET_INTERFACE                          0x0A
#define  USB_REQ_SET_INTERFACE                          0x0B
#define  USB_REQ_SYNCH_FRAME                            0x0C

#define  USB_DESC_TYPE_DEVICE                              1
#define  USB_DESC_TYPE_CONFIGURATION                       2
#define  USB_DESC_TYPE_STRING                              3
#define  USB_DESC_TYPE_INTERFACE                           4
#define  USB_DESC_TYPE_ENDPOINT                            5
#define  USB_DESC_TYPE_DEVICE_QUALIFIER                    6
#define  USB_DESC_TYPE_OTHER_SPEED_CONFIGURATION           7


#define USB_CONFIG_REMOTE_WAKEUP                           2
#define USB_CONFIG_SELF_POWERED                            1

#define USB_FEATURE_EP_HALT                                0
#define USB_FEATURE_REMOTE_WAKEUP                          1
#define USB_FEATURE_TEST_MODE                              2


#define USB_HS_MAX_PACKET_SIZE                            512
#define USB_FS_MAX_PACKET_SIZE                            64
#define USB_MAX_EP0_SIZE                                  64

/*  Device Status */
#define USB_STATE_DEFAULT                                1
#define USB_STATE_ADDRESSED                              2
#define USB_STATE_CONFIGURED                             3
#define USB_STATE_SUSPENDED                              4


/*  EP0 State */
#define USB_EP0_IDLE                                     0
#define USB_EP0_SETUP                                    1
#define USB_EP0_DATA_IN                                  2
#define USB_EP0_DATA_OUT                                 3
#define USB_EP0_STATUS_IN                                4
#define USB_EP0_STATUS_OUT                               5
#define USB_EP0_STALL                                    6

#define USB_EP_TYPE_CTRL                                 0
#define USB_EP_TYPE_ISOC                                 1
#define USB_EP_TYPE_BULK                                 2
#define USB_EP_TYPE_INTR                                 3

typedef struct
{
	uint16_t xfer_len;
	uint16_t xfer_done_count;
	uint16_t total_xfer_len;
	uint16_t xfer_remaining;
	uint16_t mps;
	uint8_t* xfer_buffer;
} USB_EndpointType;

typedef struct
{
	uint8_t chnum;
	uint16_t bcnt;
	uint8_t dpid;
	uint8_t pktsts;
} USB_FIFOStatusTypedef;

typedef struct
{
	uint8_t   bmRequest;
	uint8_t   bRequest;
	uint16_t  wValue;
	uint16_t  wIndex;
	uint16_t  wLength;
} USB_SetupReqTypedef;

typedef struct
{
	uint8_t* (*GetDeviceDescriptor)(uint16_t* len);
	uint8_t* (*GetConfigDescriptor)(uint16_t* len);
	uint8_t* (*GetLangIDStrDescriptor)(uint16_t* len);
	uint8_t* (*GetManufacturerStrDescriptor)(uint16_t* len);
	uint8_t* (*GetProductStrDescriptor)(uint16_t* len);
	uint8_t* (*GetSerialStrDescriptor)(uint16_t* len);
	uint8_t* (*GetConfigurationStrDescriptor)(uint16_t* len);
	uint8_t* (*GetInterfaceStrDescriptor)(uint16_t* len);

} USB_ClassDescTypedef;

typedef struct
{
	void  (*Connect)			(void);
	void  (*Disconnect)			(void);
	void  (*Reset)				(void);
	void  (*Init)				(uint8_t configNum);
	void  (*DeInit)				(void);
	/* Control Endpoints*/
	void  (*Setup)				(USB_SetupReqTypedef* stp);
	void  (*EP0_TxSent)			(void);
	void  (*EP0_RxReady)		(void);
	/* Class Specific Endpoints*/
	void  (*DataIn)				(uint8_t epnum);
	void  (*DataOut)			(uint8_t epnum);
	void  (*SOF)				(void);
	void  (*IsoINIncomplete)	(uint8_t epnum);
	void  (*IsoOUTIncomplete)	(uint8_t epnum);
} USB_ClassCallbackTypedef;

void USB_Init(void);
void USB_SetClass(USB_ClassDescTypedef* newclassdesc, USB_ClassCallbackTypedef* newclasscalls);
void USB_Start(void);
uint8_t USB_GetDevState(void);
void USB_IRQ(void);
void USB_SetupCallback(void);
void USB_DataINCallback(uint8_t epnum);
void USB_DataOUTCallback(uint8_t epnum);
void USB_TxFIFO_Empty_Callback(uint8_t epnum);
void USB_SetupDeviceCallback(void);
void USB_SetupInterfaceCallback(void);
void USB_SetupEndpointCallback(void);
void USB_GetDescriptor(void);
void USB_SetConfig(uint8_t configNum);
void USB_GetConfig(void);
void USB_GetStatus(void);
void USB_SetFeature(void);
void USB_ClrFeature(void);
void USB_Start_OUTEP0(void);
void USB_ActivateINEP(uint8_t epnum, uint8_t eptype, uint32_t maxpacketsize);
void USB_ActivateOUTEP(uint8_t epnum, uint8_t eptype, uint32_t maxpacketsize);
void USB_DeactivateINEP(uint8_t epnum);
void USB_DeactivateOUTEP(uint8_t epnum);
void USB_StallINEP(uint8_t epnum);
void USB_StallOUTEP(uint8_t epnum);
void USB_ClrStallINEP(uint8_t epnum);
void USB_ClrStallOUTEP(uint8_t epnum);
void USB_CtrlError(void);
uint32_t USB_GetRxDataSize(uint8_t epnum);
void USB_SendData(uint8_t *pbuf, uint8_t epnum, uint16_t len);
void USB_SendCtrlData(uint8_t *pbuf, uint16_t len);
void USB_PrepareRead(uint8_t *pbuf, uint8_t epnum, uint16_t len);
void USB_PrepareCtrlRead(uint8_t *pbuf, uint16_t len);
void USB_ReadPacket(uint8_t* buf, uint16_t len);
void USB_WritePacket(uint8_t* buf, uint8_t epnum, uint16_t len);
void USB_Start_INEP0_Transfer(uint32_t len);
void USB_Start_INEP_Transfer(uint8_t epnum, uint32_t len);
void USB_Start_OUTEP0_Transfer(uint32_t len);
void USB_Start_OUTEP_Transfer(uint8_t epnum, uint32_t len);
#endif /* USB_H_ */
