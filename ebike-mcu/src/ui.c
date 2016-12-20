/****************************************************************************
 * UI.C																		*
 * Implements a user interface through the USB or USART comm port.			*
 * Individual commands are always followed by a Line Feed, Carriage Return, *
 * or both (CR-LF or LF-CR).												*
 * 																			*
 * **************************************************************************
 */

#include "stm32f4xx.h"
#include "ui.h"

UI_Type UIhandle = {UI_Idle, UIMSG_Null};

// Simply pass all bytes, single-file, to this function.
void UI_Process(uint8_t nextbyte)
{
	switch(UIhandle.State)
	{
	case UI_Idle:
		if(nextbyte == 'U')
		{
			// Setting the USB variables!
			UIhandle.State = UI_MessageStart;
			UIhandle.Message = UIMSG_USB;
		}
		break;
	case UI_MessageStart:
		if(UIhandle.Message = UIMSG_USB)
		{
			if(nextbyte >= '1' && nextbyte <= '5')
			{

			}
		}
		break;
	case UI_MessageEndCR:

		break;
	case UI_MessageEndLF:

		break;
	}
}
