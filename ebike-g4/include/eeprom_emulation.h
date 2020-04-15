/******************************************************************************
 * Filename: eeprom_emulation.h
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

#ifndef EEPROM_EMULATION_H_
#define EEPROM_EMULATION_H_

#include "main.h"

typedef enum {
    FLASH_BUSY = 1,
    FLASH_ERROR_WRP,
    FLASH_ERROR_PROGRAM,
    FLASH_ERROR_OPERATION,
    FLASH_COMPLETE
} FLASH_Status;

/*
 * Note about Flash on the STM32G4 microcontroller:
 * Flash can be configured either in single bank (128 pages of 4KB)
 * or dual bank (two banks of 128 pages of 2KB)
 *
 * The setting can be read from the options register to figure
 * out what the current page sizes should be.
 */

// Define the size of the pages to be used
#define PAGE_SIZE_DUAL          (uint32_t)0x0800  // Page size = 2KByte for dual bank mode
#define PAGE_SIZE_SINGLE        (uint32_t)0x1000  // Page size = 4KByte for single bank mode

// Flash address definitions
#define FLASH_START_ADDRESS     (uint32_t)0x08000000
#define BANK1_START_ADDRESS     (uint32_t)0x08000000 // same as start of Flash
#define BANK2_START_ADDRESS     (uint32_t)0x08040000 // 256K past the start

#define PAGE0_PAGE_NUM          126 // Second-to-last page
#define PAGE1_PAGE_NUM          127 // Last page

// EEPROM start address in Flash - last two pages
#define EEPROM_START_ADDRESS_DUAL   ((uint32_t)BANK2_START_ADDRESS + PAGE0_PAGE_NUM*PAGE_SIZE_DUAL)
#define EEPROM_START_ADDRESS_SINGLE ((uint32_t)FLASH_START_ADDRESS + PAGE0_PAGE_NUM*PAGE_SIZE_SINGLE)

// Pages 0 and 1 base and end addresses
#define PAGE0_BASE_ADDRESS_DUAL     ((uint32_t)EEPROM_START_ADDRESS_DUAL)
#define PAGE0_BASE_ADDRESS_SINGLE   ((uint32_t)EEPROM_START_ADDRESS_SINGLE)
#define PAGE0_END_ADDRESS_DUAL      ((uint32_t)(PAGE0_BASE_ADDRESS_DUAL + (PAGE_SIZE_DUAL - 1)))
#define PAGE0_END_ADDRESS_SINGLE    ((uint32_t)(PAGE0_BASE_ADDRESS_SINGLE + (PAGE_SIZE_SINGLE - 1)))

#define PAGE1_BASE_ADDRESS_DUAL     ((uint32_t)(EEPROM_START_ADDRESS_DUAL + PAGE_SIZE_DUAL))
#define PAGE1_BASE_ADDRESS_SINGLE   ((uint32_t)(EEPROM_START_ADDRESS_SINGLE + PAGE_SIZE_SINGLE))
#define PAGE1_END_ADDRESS_DUAL      ((uint32_t)(PAGE1_BASE_ADDRESS_DUAL + (PAGE_SIZE_DUAL - 1)))
#define PAGE1_END_ADDRESS_SINGLE    ((uint32_t)(PAGE1_BASE_ADDRESS_SINGLE + (PAGE_SIZE_SINGLE - 1)))

// Used Flash pages for EEPROM emulation
#define PAGE0                 ((uint16_t)0x0000)
#define PAGE1                 ((uint16_t)0x0001)

// No valid page define
#define NO_VALID_PAGE         ((uint16_t)0x00AB)

// Page status definitions
typedef enum _EE_Page_Status {
    EE_Page_Erased, // Page is empty
    EE_Page_Receive, // Page is receiving data from the other page
    EE_Page_Active, // Page is writing new data
    EE_Page_Invalid // The status flags were incorrectly set
} EE_Page_Status;

// Status flags
#define FLASH_ERASED        ((uint32_t)0xFFFFFFFFu)
#define FLASH_FLAGGED       ((uint32_t)0x5A5A5A5Au)
#define FLASH_ZEROED        ((uint32_t)0x00000000u)

// Valid pages in read and write defines
#define READ_FROM_VALID_PAGE  ((uint8_t)0x00)
#define WRITE_IN_VALID_PAGE   ((uint8_t)0x01)

// Page full define
#define PAGE_FULL             ((uint8_t)0x80)

uint16_t EE_Init(uint16_t* addrTab);
void EE_Config_Addr_Table(uint16_t* addrTab);
uint16_t EE_ReadVariable(uint16_t VirtAddress, uint32_t* Data);
uint16_t EE_WriteVariable(uint16_t VirtAddress, uint32_t Data);
uint16_t EE_SaveInt16(uint16_t VirtAddress, int16_t Data);
uint16_t EE_SaveInt32(uint16_t VirtAddress, int32_t Data);
uint16_t EE_SaveFloat(uint16_t VirtAddress, float Data);
int16_t EE_ReadInt16WithDefault(uint16_t VirtAddress, int16_t defalt);
int32_t EE_ReadInt32WithDefault(uint16_t VirtAddress, int32_t defalt);
float EE_ReadFloatWithDefault(uint16_t VirtAddress, float defalt);

#endif /* EEPROM_EMULATION_H_ */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/

