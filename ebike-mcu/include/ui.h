#ifndef UI_H_
#define UI_H_

#define UI_PREAMBLE			"MCU+"
#define UI_LENGTH_PREAMBLE	4
#define UI_USBOPTION		"USB"
#define UI_LENGTH_USBOPTION	3
#define UI_USB_NUMOPTIONS	10
#define UI_USB_OPTIONLIST 	{ 	"IA", \
								"IB",\
								"IC",\
								"TA",\
								"TB",\
								"TC",\
								"THROTTLE",\
								"RAMPANGLE",\
								"HALLANGLE",\
								"HALLSPEED" }
#define UI_USB_LENGTHSLIST {	2,\
								2,\
								2,\
								2,\
								2,\
								2,\
								8,\
								9,\
								9,\
								9}
#define UI_SETCMD			'='
#define UI_QUERYCMD			'?'
#define UI_RESPGOOD			"OK"
#define UI_LENGTH_RESPGOOD	2
#define UI_RESPBAD			"ERROR"
#define UI_LENGTH_RESPBAD	5

void UI_Process(uint8_t* inputstring);







#endif // UI_H_
