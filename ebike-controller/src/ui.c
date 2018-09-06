/****************************************************************************
 * UI.C                                   *
 * Implements a user interface through the USB or USART comm port.      *
 * Individual commands are always followed by a Line Feed. Pass the full  *
 * string, with optional LF character, to the UI_Process function to decode *
 * incoming serial port options.
 *                                      *
 * **************************************************************************
 */

#include "stm32f4xx.h"
#include "ui.h"
#include "main.h"

/* Function Definitions */
uint8_t UI_USB_Speed_Command_Req(char cmdtype, char* options);
uint8_t UI_USB_NumVars_Command_Req(char cmdtype, char* options);
uint8_t UI_USB_Command_Req(char cmdtype, char* options);
uint8_t UI_SerialData_Command_Req(char cmdtype, char* options);
uint8_t UI_RampSpeed_Command_Req(char cmdtype, char* options);
uint8_t UI_RampDir_Command_Req(char cmdtype, char* options);
uint8_t UI_Variable_Command_Req(char cmdtype, char* options);
uint8_t UI_DumpVar_Command_Req(char cmdtype, char* options);
uint8_t UI_Reset_Source_Query(void);

/* Private Variables */
char* ui_options[UI_NUM_OPTIONS] = UI_OPTIONS;
uint8_t ui_option_lengths[UI_NUM_OPTIONS] = UI_OPTIONS_LENGTHSLIST;

char* ui_usb_options[UI_USB_NUMOPTIONS] = UI_USB_OPTIONLIST;
char* ui_usb_optdescriptions[UI_USB_NUMOPTIONS] = UI_USB_OPTDESCRIPTIONS;
uint8_t ui_usb_optdesc_lengths[UI_USB_NUMOPTIONS] = UI_USB_OPTDESC_LENGTHSLIST;

char* ui_var_options[UI_VAR_NUMOPTIONS] = UI_VAR_OPTIONLIST;

char ui_response_buf[UI_MAX_BUFFER_LENGTH];
uint32_t ui_response_len;

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

// Companion to UI_atoi. Returns the length of the string representing an integer
// Allows the parser to skip past this int to the rest of the string
static uint8_t UI_lengthofint(char* in)
{
  uint8_t lengthint = 0;
  while(*in != 0) // Don't go past null terminator
  {
    // Only valid characters are negative sign and zero through nine
    if(((*in) >= '0' && (*in <= '9')) || *in == '-')
    {
      lengthint++;
      in++;
    }
    else
    {
      return lengthint;
    }
  }
  return lengthint;
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

// Companion to UI_atof. Returns the length of the string representing a float
// Allows the parser to skip past this float to the rest of the string
static uint8_t UI_lengthoffloat(char* in)
{
  uint8_t lengthint = 0;
  while(*in != 0) // Don't go past null terminator
  {
    // Only valid characters are negative sign, decimal,  and zero through nine
    if(((*in) >= '0' && (*in <= '9')) || *in == '-' || *in == '.')
    {
      lengthint++;
      in++;
    }
    else
    {
      return lengthint;
    }
  }
  return lengthint;
}


// Doesn't actually send any output, just adds it to the output buffer
// Function that called UI_Process can call UI_BufLen to see if there's
// some info to send, and then call UI_SendBuf to retrieve it.
static void UI_SerialOut(char* str, uint8_t len)
{
  if(len > 0)
  {
    memcpy((&ui_response_buf[ui_response_len]), str, len);
    // Add terminating null
    ui_response_buf[ui_response_len + len] = 0;
    // Save new response length
    ui_response_len += len;
  }
}

uint32_t UI_RespLen(void)
{
  return ui_response_len;
}

char* UI_SendBuf(void)
{
  // Clear the response length when sent.
  ui_response_len = 0;
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
    case USB_Speed_Command:
      ui_error = UI_USB_Speed_Command_Req(UI_SETCMD, inputstring);
      break;
    case USB_NumVars_Command:
      ui_error = UI_USB_NumVars_Command_Req(UI_SETCMD, inputstring);
      break;
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
    case Dump_Command:
      MAIN_DumpRecord();
      break;
    case DumpVar_Command:
      ui_error = UI_DumpVar_Command_Req(UI_SETCMD, inputstring);
      break;
    case UI_NoCmd:
      UI_SerialOut(UI_RESPBAD, UI_LENGTH_RESPBAD);
      return UI_ERROR;
      break;
    }
    break;
  case UI_QUERYCMD:
    inputstring++; // Move past the query character (default '?')
    switch(ui_cmd)
    {
    case USB_Speed_Command:
      ui_error = UI_USB_Speed_Command_Req(UI_QUERYCMD, inputstring);
      break;
    case USB_NumVars_Command:
      ui_error = UI_USB_NumVars_Command_Req(UI_QUERYCMD, inputstring);
      break;
    case USB_Command:
      ui_error = UI_USB_Command_Req(UI_QUERYCMD, inputstring);
      break;
    case SerialData_Command:
      ui_error = UI_SerialData_Command_Req(UI_QUERYCMD, inputstring);
      break;
    case RampSpeed_Command:
      ui_error = UI_RampSpeed_Command_Req(UI_QUERYCMD, inputstring);
      break;
    case RampDir_Command:
      ui_error = UI_RampDir_Command_Req(UI_QUERYCMD, inputstring);
      break;
    case Variable_Command:
      ui_error = UI_Variable_Command_Req(UI_QUERYCMD, inputstring);
      break;
    case Reset_Command:
      // Respond with the last known reset source
      // This can help with debugging, but only right after a reset
      ui_error = UI_Reset_Source_Query();
      break;
    case Bootreset_Command:
      UI_SerialOut(UI_RESPBAD, UI_LENGTH_RESPBAD);
      return UI_ERROR;
      break;
    case Dump_Command:
      UI_SerialOut(UI_RESPBAD, UI_LENGTH_RESPBAD);
      return UI_ERROR;
      break;
    case DumpVar_Command:
      ui_error = UI_DumpVar_Command_Req(UI_QUERYCMD, inputstring);
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

uint8_t UI_USB_Speed_Command_Req(char cmdtype, char* options)
{
  int32_t speedChoice;
  char tempbuf[8];
  uint8_t templen;
  if(cmdtype == UI_SETCMD)
  {
    speedChoice = UI_atoi(options);
    if((speedChoice >= 0) && (speedChoice < MAX_USB_SPEED_CHOICES))
    {
      MAIN_SetUSBDebugSpeed(speedChoice);
      UI_SerialOut(UI_RESPGOOD, UI_LENGTH_RESPGOOD);
      return UI_OK;
    }
    else
    {
      UI_SerialOut(UI_RESPBAD, UI_LENGTH_RESPBAD);
      return UI_ERROR;
    }
  }
  else if(cmdtype == UI_QUERYCMD)
  {
    templen = _itoa(tempbuf, MAIN_GetUSBDebugSpeed(), 0);
    UI_SerialOut(tempbuf,templen);
    UI_SerialOut(UI_ENDLINE, UI_LENGTH_ENDLINE);
    return UI_OK;
  }
  return UI_ERROR;
}

uint8_t UI_USB_NumVars_Command_Req(char cmdtype, char* options)
{
  int32_t numVars;
  char tempbuf[8];
  uint8_t templen;
  if(cmdtype == UI_SETCMD)
  {
    numVars = UI_atoi(options);
    if((numVars >= 0) && (numVars <= MAX_USB_OUTPUTS))
    {
      MAIN_SetNumUSBDebugOutputs(numVars);
      UI_SerialOut(UI_RESPGOOD, UI_LENGTH_RESPGOOD);
      return UI_OK;
    }
    else
    {
      UI_SerialOut(UI_RESPBAD, UI_LENGTH_RESPBAD);
      return UI_ERROR;
    }
  }
  else if(cmdtype == UI_QUERYCMD)
  {
    templen = _itoa(tempbuf, MAIN_GetNumUSBDebugOutputs(), 0);
    UI_SerialOut(tempbuf,templen);
    UI_SerialOut(UI_ENDLINE, UI_LENGTH_ENDLINE);
    return UI_OK;
  }
  return UI_ERROR;
}

uint8_t UI_USB_Command_Req(char cmdtype, char* options)
{
  uint8_t ui_error;
  uint8_t usb_var=255;
  int32_t usb_place=0;
  char temp_buf[8];
  uint32_t temp_len;
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
    usb_place = UI_atoi(options);
    if((usb_place >= 1) && (usb_place <= MAIN_GetNumUSBDebugOutputs()))
    {
      MAIN_SetUSBDebugOutput(usb_place-1, usb_var+1);
    }
    else
    {
      UI_SerialOut(UI_RESPBAD, UI_LENGTH_RESPBAD);
      return UI_ERROR;
    }
    UI_SerialOut(UI_RESPGOOD, UI_LENGTH_RESPGOOD);
    return UI_OK;
  }
  if(cmdtype == UI_QUERYCMD)
  {
    // If 1 through <Number of Debug Outputs> is selected, returns the current parameter in that slot.
    // Otherwise returns all the available parameters for logging.
    usb_place = UI_atoi(options);
    if((usb_place >= 1) && (usb_place <= MAIN_GetNumUSBDebugOutputs()))
    {
      usb_var = MAIN_GetUSBDebugOutput(usb_place-1);
      UI_SerialOut(ui_usb_options[usb_var-1], UI_USB_LENGTH);
      UI_SerialOut(UI_ENDLINE, UI_LENGTH_ENDLINE);
    }
    else
    {
      for(uint8_t i = 0; i < UI_USB_NUMOPTIONS; i++)
      {
        temp_len = _itoa(temp_buf, i+1, 2);
        UI_SerialOut(temp_buf,temp_len);
        UI_SerialOut(": ",2);
        UI_SerialOut(ui_usb_options[i],UI_USB_LENGTH);
        UI_SerialOut(", ",2);
        UI_SerialOut(ui_usb_optdescriptions[i],ui_usb_optdesc_lengths[i]);
        UI_SerialOut(UI_ENDLINE, UI_LENGTH_ENDLINE);
      }
    }
    return UI_OK;
  }
  return UI_ERROR;
}

uint8_t UI_SerialData_Command_Req(char cmdtype, char* options)
{
  char tempbuf[4];
  uint16_t templen;
  if(cmdtype == UI_SETCMD)
  {
    // Only two valid options here, '0' or '1'
    // '0' turns off the output, '1' turns it on
    if(*options == '1')
    {
      MAIN_SetUSBDebugging(1);
      //UI_SerialOut("", 0);
    }
    else if(*options == '0')
    {
      MAIN_SetUSBDebugging(0);
      //UI_SerialOut("", 0);
    }
    else
    {
      UI_SerialOut(UI_RESPBAD, UI_LENGTH_RESPBAD);
      return UI_ERROR;
    }
    return UI_OK;
  }
  else if(cmdtype == UI_QUERYCMD)
  {
    // Just returns whether the USB debugging is currently on or off
    templen =_itoa(tempbuf, MAIN_GetUSBDebugging(), 0);
    UI_SerialOut(tempbuf, templen);
    UI_SerialOut(UI_ENDLINE, UI_LENGTH_ENDLINE);
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
    for(uint8_t i = 0; i < UI_VAR_NUMOPTIONS; i++)
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
  if(cmdtype == UI_QUERYCMD)
  {
    char tempbuf[16];
    uint16_t templen;
    // Which variable?
    for(uint8_t i = 0; i < UI_VAR_NUMOPTIONS; i++)
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
    else
    {
      // Read the current value of this variable
      newval = MAIN_GetVar(ui_var);
      templen = _ftoa(tempbuf, newval, 6);
      UI_SerialOut(tempbuf, templen);
      UI_SerialOut(UI_ENDLINE, UI_LENGTH_ENDLINE);

      return UI_OK;
    }
  }
  return UI_ERROR;
}
#ifdef DEBUG_DUMP_USED
uint8_t UI_DumpVar_Command_Req(char cmdtype, char* options)
{
  uint8_t ui_error;
  uint8_t usb_var=255, usb_place=0;
  char temp_buf[8];
  uint32_t temp_len;
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
    if(*options >= '1' && *options <= '4')
    {
      usb_place = *options - '0';
      MAIN_SetDumpDebugOutput(usb_place-1, usb_var);
    }
    else
    {
      UI_SerialOut(UI_RESPBAD, UI_LENGTH_RESPBAD);
      return UI_ERROR;
    }
    UI_SerialOut(UI_RESPGOOD, UI_LENGTH_RESPGOOD);
    return UI_OK;
  }
  if(cmdtype == UI_QUERYCMD)
  {
    // If 1 through 5 is selected, returns the current parameter in that slot.
    // Otherwise returns all the available parameters for logging.
    if(*options >= '1' && *options <= '4')
    {
      usb_place = *options -'0';
      usb_var = MAIN_GetDumpDebugOutput(usb_place-1);
      UI_SerialOut(ui_usb_options[usb_var-1], UI_USB_LENGTH);
    }
    else
    {
      for(uint8_t i = 0; i < UI_USB_NUMOPTIONS; i++)
      {
        temp_len = _itoa(temp_buf, i+1, 2);
        UI_SerialOut(temp_buf,temp_len);
        UI_SerialOut(": ",2);
        UI_SerialOut(ui_usb_options[i],UI_USB_LENGTH);
        UI_SerialOut(", ",2);
        UI_SerialOut(ui_usb_optdescriptions[i],ui_usb_optdesc_lengths[i]);
        UI_SerialOut(UI_ENDLINE, UI_LENGTH_ENDLINE);
      }
    }
    return UI_OK;
  }
  return UI_ERROR;
}
#else// !DEBUG_DUMP_USED
uint8_t UI_DumpVar_Command_Req(char cmdtype, char* options)
{
  UI_SerialOut(UI_RESPBAD, UI_LENGTH_RESPBAD);
  return UI_ERROR;
}
#endif // !DEBUG_DUMP_USED


/*
 * Writes to the output buffer the last known reset source.
 * This function is useful for debugging purposes when the MCU resets for
 * an unknown reason. Of course, this will only work if the reset query
 * is sent right after reset. The next reset (power cycle, whatever) will
 * clear this info!
 */
uint8_t UI_Reset_Source_Query(void)
{
  if (RCC->CSR & RCC_CSR_LPWRRSTF)
  {
    UI_SerialOut("Low-power reset\r\n", 17);
  }
  if (RCC->CSR & RCC_CSR_WWDGRSTF)
  {
    UI_SerialOut("Window watchdog reset\r\n", 23);
  }
  if (RCC->CSR & RCC_CSR_WDGRSTF)
  {
    UI_SerialOut("Independent watchdog reset\r\n", 28);
  }
  if (RCC->CSR & RCC_CSR_SFTRSTF)
  {
    UI_SerialOut("Software reset\r\n", 16);
  }
  if (RCC->CSR & RCC_CSR_PORRSTF)
  {
    UI_SerialOut("Power on reset\r\n", 16);
  }
  if (RCC->CSR & RCC_CSR_PADRSTF)
  {
    UI_SerialOut("Pin reset\r\n", 11);
  }
  if (RCC->CSR & RCC_CSR_BORRSTF)
  {
    UI_SerialOut("Brown out reset\r\n", 17);
  }

  return UI_OK;
}
