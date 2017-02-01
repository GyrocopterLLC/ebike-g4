#ifndef UI_H_
#define UI_H_

#define UI_OK				1
#define UI_ERROR			0

#define UI_MAX_BUFFER_LENGTH	32
#define UI_PREAMBLE			"MCU+"
#define UI_LENGTH_PREAMBLE	4
#define UI_NUM_OPTIONS		2
#define UI_OPTIONS			{	"USB",\
								"SERIALDATA"}
#define UI_OPTIONS_LENGTHSLIST	{	3,\
								   10}

typedef enum
{
	USB_Command = 0,
	SerialData_Command = 1,
	UI_NoCmd = 127
} UI_CommandType;

#define UI_USB_NUMOPTIONS	10
#define UI_USB_OPTIONLIST 	{ 	"IA",\
								"IB",\
								"IC",\
								"TA",\
								"TB",\
								"TC",\
								"TH",\
								"RA",\
								"HA",\
								"HS" }
#define UI_USB_LENGTHSLIST {	2,\
								2,\
								2,\
								2,\
								2,\
								2,\
								2,\
								2,\
								2,\
								2}
#define UI_SETCMD			'='
#define UI_QUERYCMD			'?'
#define UI_RESPGOOD			"OK"
#define UI_LENGTH_RESPGOOD	2
#define UI_RESPBAD			"ERROR"
#define UI_LENGTH_RESPBAD	5

uint8_t UI_Process(char* inputstring);
uint8_t UI_RespLen(void);
char* UI_SendBuf(void);






#endif // UI_H_
