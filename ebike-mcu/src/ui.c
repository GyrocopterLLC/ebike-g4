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

char* ui_usb_options[UI_USB_NUMOPTIONS] = UI_USB_OPTIONLIST;
uint8_t ui_usb_lengths[UI_USB_NUMOPTIONS] = UI_USB_LENGTHSLIST;

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

static void to_upper(const char* in)
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

// Simply pass all bytes, single-file, to this function.
uint8_t UI_Process(uint8_t* inputstring)
{
	uint8_t ui_error = 0;
	// First, check the preamble.
	ui_error = strcmp_s(inputstring, UI_PREAMBLE, UI_LENGTH_PREAMBLE);
	if(ui_error != 0)
	{
		// UI_SerialOut(UI_RESPBAD, UI_LENGTH_RESPBAD);
		return 0;
	}
	// Next, check all the options
	inputstring += UI_LENGTH_PREAMBLE;

	// Was it a USB output setting?
	ui_error = strcmp_s(inputstring, UI_USBOPTION, UI_LENGTH_USBOPTION);
	if(ui_error == 0)
	{
		// Yes! Which one?
		inputstring += UI_LENGTH_USBOPTION;
		if((*inputstring >= '1') && (*inputstring <= '5'))
		{
			// Valid USB output number (1 through 5)
			uint8_t ui_usbnum = *inputstring - '0';
			inputstring++;
			if(*inputstring == UI_SETCMD)
			{
				// Decode this string to find out which variable to send
				// over the USB output channel.
				inputstring++;
				to_upper(inputstring);
				uint8_t i;
				for(i = 0; i < UI_USB_NUMOPTIONS; i++)
				{
					if(strcmp_s(inputstring,ui_usb_options[i],ui_usb_lengths[i]) == 0)
					{
						// We match!
						MAIN_SetUSBDebugOuput(ui_usbnum, i);
						break;
					}
				}
				if(i >= UI_USB_NUMOPTIONS)
				{
					// There wasn't a matching variable name.
					ui_error = 1;
				}
			}
		}
		else
		{
			ui_error = 1;
		}

		if(ui_error != 0)
		{
			// UI_SerialOut(UI_RESPBAD, UI_LENGTH_RESPBAD);
			return 0;
		}
		else
		{
			// UI_SerialOut(UI_RESPGOOD, UI_LENGTH_RESPGOOD);
			return 1;
		}
	}
	else
	{
		// UI_SerialOut(UI_RESPBAD, UI_LENGTH_RESPBAD);
		return 0;
	}
}
