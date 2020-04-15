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

 Copyright (c) 2020 David Miller

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

void CRC_Init(void) {
    // Turns on the hardware
    RCC->AHB1ENR |= RCC_AHB1ENR_CRCEN;
}

/**
 * @brief  Generates a CRC-32 using the Ethernet standard.
 *
 *         Uses bit reversal on input and output, initial
 *         value is all 1's, and output value is xor'd with
 *         all 1's. The polynomial is fixed to 0x04C1.1DB7
 * @param  buf: The input buffer of unsigned bytes
 * @param  len: Length of input buffer (number of bytes)
 * @retval The generated CRC-32 value.
 */
uint32_t CRC_Generate_CRC32(uint8_t *buf, uint16_t len) {
    uint32_t crc_input;

    // Enable the bit reversals for CRC-32
    CRC->CR = CRC_CR_REV_IN | CRC_CR_REV_OUT;
    // Set the polynomial
    CRC->POL = 0x04C11DB7u;
    // Set initial (reset) value
    CRC->INIT = 0xFFFFFFFFu;
    // Reset the CRC to all 1's
    CRC->CR |= CRC_CR_RESET;


    // Push data in blocks of 4 bytes
    while (len >= 4) {
        crc_input = ((uint32_t) buf[0]) + (((uint32_t) buf[1]) << 8u)
                + (((uint32_t) buf[2]) << 16u) + (((uint32_t) buf[3]) << 24u);
        CRC->DR = crc_input;
        len -= 4;
        buf += 4;
    }
    switch (len) {
    case 0:
        // No more to send
        return 0xFFFFFFFF ^ (CRC->DR);
    case 1:
        crc_input = ((uint32_t) buf[0]);
        CRC->DR = crc_input;
        return 0xFFFFFFFF ^ (CRC->DR);
    case 2:
        crc_input = ((uint32_t) buf[0]) + (((uint32_t) buf[1]) << 8);
        CRC->DR = crc_input;
        return 0xFFFFFFFF ^ (CRC->DR);
    case 3:
        crc_input = ((uint32_t) buf[0]) + (((uint32_t) buf[1]) << 8)
                + (((uint32_t) buf[2]) << 16);
        CRC->DR = crc_input;
        return 0xFFFFFFFF ^ (CRC->DR);
    default:
        // This shouldn't happen. So no special error fixing, just return what
        // we got at this point.
        return 0xFFFFFFFF ^ (CRC->DR);
    }

}
