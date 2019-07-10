/******************************************************************************
 * Filename: crc32.c
 * Description: Uses the STM32 built-in CRC generator, but makes it compliant
 *              with the CRC-32 used in Ethernet generation. A couple of 
 *              changes are needed. The bits need to be in reverse order for
 *              each byte, the result is bit-flipped, and the result is 
 *              xor'd with 1's. 
 *              When making a matching function to interface with this
 *              controller, make sure to pad all input data with 0's so that
 *              the number of bytes is divisible by 4. Put the extra 0's at
 *              the end.
 *              Check with:
 *              http://www.sunshine2k.de/coding/javascript/crc/crc_js.html
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

#include "main.h"

void CRC32_Init(void) {
    // Turns on the hardware
    RCC->AHB1ENR |= RCC_AHB1ENR_CRCEN;
}

uint32_t CRC32_Generate(uint8_t *buf, uint16_t len) {
    // Reset the CRC to all 1's
    CRC->CR = CRC_CR_RESET;
    uint32_t crc_input;

    // Push data in blocks of 4 bytes
    while (len >= 4) {
        crc_input = (((uint32_t) buf[0]) << 24) + (((uint32_t) buf[1]) << 16)
                + (((uint32_t) buf[2]) << 8) + ((uint32_t) buf[3]);
        crc_input = __RBIT(__REV(crc_input));
        CRC->DR = crc_input;
        len -= 4;
        buf += 4;
    }
    switch (len) {
    case 0:
        // No more to send
        return 0xFFFFFFFF ^ (__RBIT(CRC->DR));
    case 1:
        crc_input = (((uint32_t) buf[0]) << 24);
        crc_input = __RBIT(__REV(crc_input));
        CRC->DR = crc_input;
        return 0xFFFFFFFF ^ (__RBIT(CRC->DR));
    case 2:
        crc_input = (((uint32_t) buf[0]) << 24) + (((uint32_t) buf[1]) << 16);
        crc_input = __RBIT(__REV(crc_input));
        CRC->DR = crc_input;
        return 0xFFFFFFFF ^ (__RBIT(CRC->DR));
    case 3:
        crc_input = (((uint32_t) buf[0]) << 24) + (((uint32_t) buf[1]) << 16)
                + (((uint32_t) buf[2]) << 8);
        crc_input = __RBIT(__REV(crc_input));
        CRC->DR = crc_input;
        return 0xFFFFFFFF ^ (__RBIT(CRC->DR));
    default:
        // This shouldn't happen. So no special error fixing, just return what
        // we got at this point.
        return 0xFFFFFFFF ^ (__RBIT(CRC->DR));
    }

}
