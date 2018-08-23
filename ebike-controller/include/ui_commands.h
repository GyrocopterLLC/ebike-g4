#ifndef UI_COMMANDS_H_
#define UI_COMMANDS_H_

#define UI_MENU_SEPARATOR     ':'

#define UI_TOP_LEVEL_COMMANDS   {     "DATA",\
                                      "FOC",\
                                      "MOTOR",\
                                      "UTIL",\
                                      "CTRL"\
                                }
#define UI_TOP_LEVEL_NUMCMD     5
#define UI_TOP_LEVEL_CMDLEN     {4,3,5,4,4}

#define UI_DATA_COMMANDS        {     "SPEED",\
                                      "NUMVARS",\
                                      "VAR1",\
                                      "VAR2",\
                                      "VAR3",\
                                      "VAR4",\
                                      "VAR5",\
                                      "VAR6",\
                                      "VAR7",\
                                      "VAR8",\
                                      "VAR9",\
                                      "VAR10"\
                                }
#define UI_DATA_NUMCMD          12
#define UI_DATA_CMDLEN          {5,7,4,4,4,4,4,4,4,4,4,5}

#define UI_FOC_COMMANDS         {     "KP",\
                                      "KI",\
                                      "KD",\
                                      "KC",\
                                      "FREQ",\
                                      "DT"\
                                }
#define UI_FOC_NUMCMD           6
#define UI_FOC_CMDLEN           {2,2,2,2,4,2}

#define UI_MOTOR_COMMANDS       {     "PP",\
                                      "RS",\
                                      "LS",\
                                      "FLUX",\
                                      "HALLANG1",\
                                      "HALLANG2",\
                                      "HALLANG3",\
                                      "HALLANG4",\
                                      "HALLANG5",\
                                      "HALLANG6",
                                }
#define UI_MOTOR_NUMCMD         10
#define UI_MOTOR_CMDLEN         {2,2,2,4,8,8,8,8,8,8}

#define UI_UTIL_COMMANDS        {     "SAVE",\
                                      "RESET",\
                                      "BOOTRESET"\
                                }
#define UI_UTIL_NUMCMD          3
#define UI_UTIL_CMDLEN          {4,5,9}

#define UI_CTRL_COMMANDS        {     "THRMIN",\
                                      "THRMAX",\
                                      "THRHYST",\
                                      "THRFILT",\
                                      "THRRISE",\
                                      "IAGAIN",\
                                      "IBGAIN",\
                                      "ICGAIN",\
                                      "IAOFFSET",\
                                      "IBOFFSET",\
                                      "ICOFFSET",\
                                      "VBUSSCALE"\
                                }
#define UI_CTRL_NUMCMD          12
#define UI_CTRL_CMDLEN          {6,6,7,7,7,6,6,6,8,8,8,9}



#endif // UI_COMMANDS_H_
