/******************************************************************************
 * Filename: ui_commands.h
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

#ifndef UI_COMMANDS_H_
#define UI_COMMANDS_H_

#define UI_MENU_SEPARATOR     ':'

#define UI_MENU_SET           '='
#define UI_MENU_QUERY         '?'
#define UI_MENU_QUERY_EEPROM  '$'

#define UI_TOP_LEVEL_COMMANDS   {     "DATA",\
                                      "FOC",\
                                      "MOTOR",\
                                      "UTIL",\
                                      "CTRL",\
                                      "THR" }
#define UI_TOP_LEVEL_NUMCMD     6
#define UI_TOP_LEVEL_CMDLEN     {4,3,5,4,4,3}

#define UI_SAVE_OR_LOAD_COMMANDS  {   "SAVE",\
                                      "LOAD" }
#define UI_SAVE_OR_LOAD_CMDLEN    {4, 4}
#define UI_SAVE_OR_LOAD_NUMCMD   2

#define UI_DATA_COMMANDS        {     "SPEED",\
                                      "NUMVARS",\
                                      "LISTVARS",\
                                      "VAR1",\
                                      "VAR2",\
                                      "VAR3",\
                                      "VAR4",\
                                      "VAR5",\
                                      "VAR6",\
                                      "VAR7",\
                                      "VAR8",\
                                      "VAR9",\
                                      "VARA",\
                                      "SEND" }
#define UI_DATA_NUMCMD          14
#define UI_DATA_CMDLEN          {5,7,8,4,4,4,4,4,4,4,4,4,4,4}

#define UI_FOC_COMMANDS         {     "KP",\
                                      "KI",\
                                      "KD",\
                                      "KC",\
                                      "DT",\
                                      "FREQ"}
#define UI_FOC_NUMCMD           6
#define UI_FOC_CMDLEN           {2,2,2,2,2,4}

#define UI_MOTOR_COMMANDS       {     "PP",\
                                      "RS",\
                                      "LS",\
                                      "FLUX",\
                                      "HALLANG1",\
                                      "HALLANG2",\
                                      "HALLANG3",\
                                      "HALLANG4",\
                                      "HALLANG5",\
                                      "HALLANG6" }
#define UI_MOTOR_NUMCMD         10
#define UI_MOTOR_CMDLEN         {2,2,2,4,8,8,8,8,8,8}

#define UI_UTIL_COMMANDS        {     "SAVE",\
                                      "LOAD",\
                                      "RESET",\
                                      "BOOTRESET" }
#define UI_UTIL_NUMCMD          4
#define UI_UTIL_CMDLEN          {4,4,5,9}

#define UI_CTRL_COMMANDS        {     "IAGAIN",\
                                      "IBGAIN",\
                                      "ICGAIN",\
                                      "IAOFFSET",\
                                      "IBOFFSET",\
                                      "ICOFFSET",\
                                      "VBUSSCALE" }
#define UI_CTRL_NUMCMD          7
#define UI_CTRL_CMDLEN          {6,6,6,8,8,8,9}

#define UI_THR_COMMANDS         {     "TYPE1",\
                                      "TYPE2",\
                                      "MIN1",\
                                      "MIN2",\
                                      "MAX1",\
                                      "MAX2",\
                                      "HYST1",\
                                      "HYST2",\
                                      "FILT1",\
                                      "FILT2",\
                                      "RISE1",\
                                      "RISE2" }
#define UI_THR_NUMCMD           12
#define UI_THR_CMDLEN           {5,5,4,4,4,4,5,5,5,5,5,5}

#define UI_THR_TYPES            {"NONE","HALL","PAS"}
#define UI_THR_NUMTYPES         3
#define UI_THR_TYPESLEN         {4,4,3}


#endif // UI_COMMANDS_H_
