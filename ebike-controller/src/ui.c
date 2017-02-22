/****************************************************************************
 * UI.C																		*
 * Implements a user interface through the USB or USART comm port.			*
 * Individual commands are always followed by a Line Feed. Pass the full	*
 * string, with optional LF character, to the UI_Process function to decode	*
 * incoming serial port options.
 * 																			*
 * **************************************************************************
 */

#include "stm32f4xx.h"
#include "ui.h"
#include "main.h"

/* Function Definitions */
uint8_t UI_USB_Command_Req(char cmdtype, char* options);
uint8_t UI_SerialData_Command_Req(char cmdtype, char* options);
uint8_t UI_RampSpeed_Command_Req(char cmdtype, char* options);
uint8_t UI_RampDir_Command_Req(char cmdtype, char* options);
uint8_t UI_Variable_Command_Req(char cmdtype, char* options);

/* Private Variables */
char* ui_options[UI_NUM_OPTIONS] = UI_OPTIONS;
uint8_t ui_option_lengths[UI_NUM_OPTIONS] = UI_OPTIONS_LENGTHSLIST;

char* ui_usb_options[UI_USB_NUMOPTIONS] = UI_USB_OPTIONLIST;

char* ui_var_options[UI_VAR_NUMOPTIONS] = UI_VAR_OPTIONLIST;

char ui_response_buf[UI_MAX_BUFFER_LENGTH];
uint8_t ui_response_len;

static int strcmp_s(const char* in1, const char* in2, int count)
{
	for(int i=0;i<count;i++)
	{
		if((*in1) > (*in2))
			return 1;
		else if((*in1) < (*in2))
			return -1;
		else // in1 and in2 are equal
		{
			if(*in1 == 0)
			{
				return 0;
			}
			in1++;
			in2++;
		}
	}
	return 0;
}

static void to_upper(char* in)
{
	while(*in != 0)
	{
		if((*in >= 'a') && (*in <= 'z'))
		{
			*in = *in - 32;
		}
		in++;
	}
}

// Converts input string (assumed to be an ASCII integer number) into a signed 32 bit int
static int32_t UI_atoi(char* in)
{
	char* str = in;
	uint8_t isneg = 0;
	int32_t retval = 0;
	if(*str == '-')
	{
		isneg = 1;
		str++;
	}
	while(*str != 0) // Stop when we hit a null terminator
	{
		if(((*str) >= '0') && ((*str) <= '9'))
		{
			// Valid numeral seen here
			retval *= 10;
			retval += (*str) - '0';
		}
		else
		{
			if(isneg != 0)
				retval = 0 - retval;
			return retval;
		}
		str++;
	}
	if(isneg != 0)
	{
		retval = 0 - retval;
	}
	return retval;
}

// Converts input string to single-precision floating point
static float UI_atof(char* in)
{
	char* str = in;
	float divfactor = 1.0f;
	float retval = 0.0f;
	uint8_t decimal_point_happened = 0;
	if(*str == '-')
	{
		divfactor = -1.0f;
		str++;
	}
	// Go through that string!
	while((*str) != 0)
	{
		// Is a digit?
		if(((*str) >= '0') && ((*str) <= '9'))
		{
			if(decimal_point_happened != 0)
				divfactor = divfactor / 10.0f;
			retval *= 10.0f;
			retval += (float)(*str - '0');
		}
		else if((*str) == '.')
		{
			if(decimal_point_happened != 0)
			{
				// This is the second decimal point! Abandon ship!
				return (retval*divfactor);
			}
			decimal_point_happened = 1;
		}
		else
		{
			// Unknown character, get out now.
			return (retval * divfactor);
		}
		str++;
	}
	return (retval * divfactor);
}

// Doesn't actually send any output, just adds it to the output buffer
// Function that called UI_Process can call UI_BufLen to see if there's
// some info to send, and then call UI_SendBuf to retrieve it.
static void UI_SerialOut(char* str, uint8_t len)
{
	if(len > 0)
		memcpy(ui_response_buf, str, len+1); // +1 for the terminating null char
	ui_response_len = len;
}

uint8_t UI_RespLen(void)
{
	return ui_response_len;
}
char* UI_SendBuf(void)
{
	return ui_response_buf;
}

// Simply pass all bytes, single-file, to this function.
uint8_t UI_Process(char* inputstring)
{
	uint8_t ui_error = 0;
	UI_CommandType ui_cmd = UI_NoCmd;

	// Convert to upper case
	to_upper(inputstring);
	// First, check the preamble.
	ui_error = strcmp_s(inputstring, UI_PREAMBLE, UI_LENGTH_PREAMBLE);
	if(ui_error != 0)
	{
		UI_SerialOut(UI_RESPBAD, UI_LENGTH_RESPBAD);
		return UI_ERROR;
	}
	// Next, check all the options
	inputstring += UI_LENGTH_PREAMBLE;

	for(uint8_t i = 0; i < UI_NUM_OPTIONS; i++)
	{
		ui_error = strcmp_s(inputstring, ui_options[i], ui_option_lengths[i]);
		if(ui_error == 0)
		{
			// It's a match!
			ui_cmd = i;
			break;
		}
	}
	if(ui_error != 0)
	{
		// There was no match :(
		UI_SerialOut(UI_RESPBAD, UI_LENGTH_RESPBAD);
		return UI_ERROR;
	}

	// Proceed past the command part of the string
	inputstring += ui_option_lengths[ui_cmd];

	// Do something based on the command type
	switch(*inputstring)
	{
	case UI_SETCMD:
		// Set the variable to a thing!
		inputstring++; // Move past the set character (default '=')
		switch(ui_cmd)
		{
		case USB_Command:
			ui_error = UI_USB_Command_Req(UI_SETCMD, inputstring);
			break;
		case SerialData_Command:
			ui_error = UI_SerialData_Command_Req(UI_SETCMD, inputstring);
			break;
		case RampSpeed_Command:
			ui_error = UI_RampSpeed_Command_Req(UI_SETCMD, inputstring);
			break;
		case RampDir_Command:
			ui_error = UI_RampDir_Command_Req(UI_SETCMD, inputstring);
			break;
		case Variable_Command:
			ui_error = UI_Variable_Command_Req(UI_SETCMD, inputstring);
			break;
		case Reset_Command:
			MAIN_SoftReset(0);
			break;
		case Bootreset_Command:
			MAIN_SoftReset(1);
			break;
		case UI_NoCmd:
			UI_SerialOut(UI_RESPBAD, UI_LENGTH_RESPBAD);
			return UI_ERROR;
			break;
		}
		break;
	default:
		UI_SerialOut(UI_RESPBAD, UI_LENGTH_RESPBAD);
		return UI_ERROR;
		break;
	}
	return ui_error;
}

uint8_t UI_USB_Command_Req(char cmdtype, char* options)
{
	uint8_t ui_error;
	uint8_t usb_var=255, usb_place=0;
	if(cmdtype == UI_SETCMD)
	{
		for(uint8_t i = 0; i < UI_USB_NUMOPTIONS; i++)
		{
			ui_error = strcmp_s(options,ui_usb_options[i],UI_USB_LENGTH);
			if(ui_error == 0)
			{
				// Setting this variable!
				usb_var = i;
				break;
			}
		}
		if(ui_error != 0)
		{
			UI_SerialOut(UI_RESPBAD, UI_LENGTH_RESPBAD);
			return UI_ERROR;
		}
		// Next, make sure syntax is right (should be a comma)
		options += UI_USB_LENGTH;
		if(*options != ',')
		{
			UI_SerialOut(UI_RESPBAD, UI_LENGTH_RESPBAD);
			return UI_ERROR;
		}
		options++; // go past the comma
		// Check if the output place is correct
		if(*options >= '1' && *options <= '5')
		{
			usb_place = *options - '0';
			MAIN_SetUSBDebugOutput(usb_place-1, usb_var);
		}
		else
		{
			UI_SerialOut(UI_RESPBAD, UI_LENGTH_RESPBAD);
			return UI_ERROR;
		}
		UI_SerialOut(UI_RESPGOOD, UI_LENGTH_RESPGOOD);
		return UI_OK;
	}
	return UI_ERROR;
}

uint8_t UI_SerialData_Command_Req(char cmdtype, char* options)
{
	if(cmdtype == UI_SETCMD)
	{
		// Only two valid options here, '0' or '1'
		// '0' turns off the output, '1' turns it on
		if(*options == '1')
		{
			MAIN_SetUSBDebugging(1);
			UI_SerialOut("", 0);
		}
		else if(*options == '0')
		{
			MAIN_SetUSBDebugging(0);
			UI_SerialOut("", 0);
		}
		else
		{
			UI_SerialOut(UI_RESPBAD, UI_LENGTH_RESPBAD);
			return UI_ERROR;
		}
		return UI_OK;
	}
	return UI_ERROR;
}

uint8_t UI_RampSpeed_Command_Req(char cmdtype, char* options)
{
	uint32_t newspeed;
	if(cmdtype == UI_SETCMD)
	{
		// Decode the number to see what we're setting the speed to
		newspeed = UI_atoi(options);
		if(newspeed != 0)
		{
			MAIN_SetRampSpeed(newspeed);
			UI_SerialOut(UI_RESPGOOD, UI_LENGTH_RESPGOOD);
		}
		else
		{
			UI_SerialOut(UI_RESPBAD, UI_LENGTH_RESPBAD);
			return UI_ERROR;
		}
		return UI_OK;
	}
	return UI_ERROR;
}

uint8_t UI_RampDir_Command_Req(char cmdtype, char* options)
{
	if(cmdtype == UI_SETCMD)
	{
		// Only two options allowed - 'F' for forward, 'R' for reverse
		if(*options == 'F')
		{
			MAIN_SetRampDir(0);
			UI_SerialOut(UI_RESPGOOD, UI_LENGTH_RESPGOOD);
		}
		else if(*options == 'R')
		{
			MAIN_SetRampDir(1);
			UI_SerialOut(UI_RESPGOOD, UI_LENGTH_RESPGOOD);
		}
		else
		{
			UI_SerialOut(UI_RESPBAD, UI_LENGTH_RESPBAD);
			return UI_ERROR;
		}
		return UI_OK;
	}
	return UI_ERROR;
}

uint8_t UI_Variable_Command_Req(char cmdtype, char* options)
{
	uint8_t ui_error;
	uint8_t ui_var = 255;
	float newval;
	if(cmdtype == UI_SETCMD)
	{
		for(uint8_t i = 0; i < UI_USB_NUMOPTIONS; i++)
		{
			ui_error = strcmp_s(options,ui_var_options[i],UI_VAR_LENGTH);
			if(ui_error == 0)
			{
				// Setting this variable!
				ui_var = i;
				break;
			}
		}
		if(ui_error != 0)
		{
			UI_SerialOut(UI_RESPBAD, UI_LENGTH_RESPBAD);
			return UI_ERROR;
		}
		// Next, make sure syntax is right (should be a comma)
		options += UI_USB_LENGTH;
		if(*options != ',')
		{
			UI_SerialOut(UI_RESPBAD, UI_LENGTH_RESPBAD);
			return UI_ERROR;
		}
		options++; // go past the comma
		// What's the new value?
		newval = UI_atof(options);
		MAIN_SetVar(ui_var, newval);
		UI_SerialOut(UI_RESPGOOD, UI_LENGTH_RESPGOOD);
		return UI_OK;
	}
	return UI_ERROR;
}
