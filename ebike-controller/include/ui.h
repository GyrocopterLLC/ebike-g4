/******************************************************************************
 * Filename: ui.h
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

#ifndef UI_H_
#define UI_H_

#define UI_OK       1
#define UI_ERROR      0

#define UI_MAX_BUFFER_LENGTH  512
#define UI_PREAMBLE     "MCU+"
#define UI_LENGTH_PREAMBLE  4
#define UI_NUM_OPTIONS    11
#define UI_OPTIONS      { "USBSPEED",\
                          "USBNUMVARS",\
                          "USB",\
                          "SERIALDATA",\
                          "RAMPSPEED",\
                          "RAMPDIR",\
                          "VAR",\
                          "RESET",\
                          "BOOTRESET",\
                          "DUMPVAR",\
                          "DUMP"}
#define UI_OPTIONS_LENGTHSLIST  { 8,\
                                  10,\
                                  3,\
                                  10,\
                                  9,\
                                  7,\
                                  3,\
                                  5,\
                                  9,\
                                  7,\
                                  4}

typedef enum {
    USB_Speed_Command,
    USB_NumVars_Command,
    USB_Command,
    SerialData_Command,
    RampSpeed_Command,
    RampDir_Command,
    Variable_Command,
    Reset_Command,
    Bootreset_Command,
    DumpVar_Command,
    Dump_Command,
    UI_NoCmd
} UI_CommandType;

#define UI_USB_NUMOPTIONS 19
#define UI_USB_LENGTH   2
#define UI_USB_OPTIONLIST   {   "IA",\
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
                "ST",\
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
                                    "Hall State",\
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
                                     10,\
                                     18}

#define UI_VAR_NUMOPTIONS 4
#define UI_VAR_LENGTH   2
#define UI_VAR_OPTIONLIST { "KP",\
                "KI",\
                "KD",\
                "KC"}

#define UI_SETCMD           '='
#define UI_QUERYCMD         '?'
#define UI_RESPGOOD         "OK\r\n"
#define UI_LENGTH_RESPGOOD  4
#define UI_RESPBAD          "ERROR\r\n"
#define UI_LENGTH_RESPBAD   7
#define UI_ENDLINE          "\r\n"
#define UI_LENGTH_ENDLINE   2

uint8_t UI_Process(char* inputstring);
uint32_t UI_RespLen(void);
char* UI_SendBuf(void);
uint8_t UI_TopLevelProcess(char* inputstring);

#endif // UI_H_
