#ifndef DEBUG_DUMP_H_
#define DEBUG_DUMP_H_

#define MAX_DUMP_OUTPUTS    4
#define MAX_DUMP_LENGTH     32768 // 64k of CCMRAM / 2-byte variables = 32k

/* Default dump outputs: Ia, Ib, Ic, Hall Angle */
#define DEFAULT_DUMP_ASSIGNMENTS    {1, 2, 3, 9}

#endif // DEBUG_DUMP_H_
