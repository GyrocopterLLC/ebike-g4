#ifndef UI_H_
#define UI_H_

#define UI_OK				1
#define UI_ERROR			0

#define UI_MAX_BUFFER_LENGTH	512
#define UI_PREAMBLE			"MCU+"
#define UI_LENGTH_PREAMBLE	4
#define UI_NUM_OPTIONS		9
#define UI_OPTIONS			{ "USB",\
								          "SERIALDATA",\
								          "RAMPSPEED",\
								          "RAMPDIR",\
								          "VAR",\
								          "RESET",\
								          "BOOTRESET",\
								          "DUMPVAR",\
								          "DUMP"}
#define UI_OPTIONS_LENGTHSLIST	{	3,\
                10,\
                9,\
                7,\
                3,\
                5,\
                9,\
                7,\
                4}

typedef enum
{
	USB_Command = 0,
	SerialData_Command = 1,
	RampSpeed_Command = 2,
	RampDir_Command = 3,
	Variable_Command = 4,
	Reset_Command = 5,
	Bootreset_Command = 6,
	DumpVar_Command = 7,
	Dump_Command = 8,
	UI_NoCmd = 127
} UI_CommandType;

#define UI_USB_NUMOPTIONS	18
#define UI_USB_LENGTH		2
#define UI_USB_OPTIONLIST 	{ 	"IA",\
								"IB",\
								"IC",\
								"TA",\
								"TB",\
								"TC",\
								"TH",\
								"RA",\
								"HA",\
								"HS",\
								"VS",\
								"ID",\
								"IQ",\
								"TD",\
								"TQ",\
								"ER",\
								"VR",\
                "2A"}

#define UI_USB_OPTDESCRIPTIONS    { "PhA Current",\
                                    "PhB Current",\
                                    "PhC Current",\
                                    "PhA Duty Cycle",\
                                    "PhB Duty Cycle",\
                                    "PhC Duty Cycle",\
                                    "Throttle",\
                                    "Simulated Ramp Angle",\
                                    "Hall Sensor Angle",\
                                    "Hall Sensor Speed",\
                                    "DC Bus Voltage",\
                                    "FOC D Current",\
                                    "FOC Q Current",\
                                    "Low-Pass Filtered D Current",\
                                    "Low-Pass Filtered Q Current",\
                                    "Error Code",\
                                    "ADC Vrefint (raw)",\
                                    "Hall Sensor2 Angle"}

#define UI_USB_OPTDESC_LENGTHSLIST  {11,\
                                     11,\
                                     11,\
                                     14,\
                                     14,\
                                     14,\
                                     8,\
                                     20,\
                                     17,\
                                     17,\
                                     14,\
                                     13,\
                                     13,\
                                     27,\
                                     27,\
                                     10,\
                                     17,\
                                     18}

#define UI_VAR_NUMOPTIONS	4
#define UI_VAR_LENGTH		2
#define UI_VAR_OPTIONLIST	{	"KP",\
								"KI",\
								"KD",\
								"KC"}


#define UI_SETCMD			'='
#define UI_QUERYCMD			'?'
#define UI_RESPGOOD			"OK"
#define UI_LENGTH_RESPGOOD	2
#define UI_RESPBAD			"ERROR"
#define UI_LENGTH_RESPBAD	5

uint8_t UI_Process(char* inputstring);
uint32_t UI_RespLen(void);
char* UI_SendBuf(void);






#endif // UI_H_
