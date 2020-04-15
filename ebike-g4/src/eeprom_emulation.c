/******************************************************************************
 * Filename: eeprom_emulation.c
 * Description: Uses the STM32 Flash memory as if it were EEPROM. All "EEPROM"
 *              variables are saved in Flash with an associated variable
 *              address. Saving a new value to a particular address actually
 *              just stores a new variable/address combo.
 *              When reading, the most recently written value is returned. If
 *              no value was written, a default value (passed to the read
 *              function) is returned instead.
 *              Erases are only performed when a page becomes completely full.
 *              The most recent saved value for each variable is transferred to
 *              the alternate storage page, and the alternate page is marked as
 *              active.
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

// Virtual addresses defined by the user: 0xFFFF can't be used since it looks like erased data
uint16_t* EE_VirtAddVarTab;

// Page sizes and Start/End locations stored locally. They are dependent on the bank mode
uint32_t EE_Page0_Base_Address;
uint32_t EE_Page1_Base_Address;
uint32_t EE_Page_Size;


#define FLASH_KEY1               ((uint32_t)0x45670123)
#define FLASH_KEY2               ((uint32_t)0xCDEF89AB)
static void FLASH_Unlock(void);
static void FLASH_Lock(void);
static FLASH_Status FLASH_GetStatus(void);
static FLASH_Status FLASH_WaitForLastOperation(void);
static FLASH_Status FLASH_WaitForErase(void);
static FLASH_Status FLASH_ErasePage(uint32_t FLASH_Page, uint8_t FLASH_Bank);
FLASH_Status FLASH_ProgramDoubleWord(uint32_t Address, uint32_t Data1, uint32_t Data2);
static FLASH_Status FLASH_CheckErasedAndFix(uint32_t Address);

static EE_Page_Status EE_GetPageStatus(uint16_t Page);
static FLASH_Status EE_MarkStatus(uint16_t PageId, EE_Page_Status PageStatus);
static FLASH_Status EE_Format(void);
static uint16_t EE_ErasePage(uint16_t PageId);
static uint16_t EE_FindValidPage(uint8_t Operation);
static uint16_t EE_VerifyPageFullWriteVariable(uint16_t VirtAddress, uint32_t Data);
static uint16_t EE_PageTransfer(uint16_t OldPageId, uint16_t NewPageId, uint16_t SkipId);

void EE_Config_Addr_Table(uint16_t* addrTab) {
    uint32_t tabptr = 0;

    // Add ADC variables
    for (uint32_t i = 0; i < (CONFIG_ADC_NUMVARS); i++) {
        addrTab[tabptr++] = (CONFIG_ADC_PREFIX + i);
    }
    // Add FOC variables
    for (uint32_t i = 0; i < (CONFIG_FOC_NUMVARS); i++) {
        addrTab[tabptr++] = (CONFIG_FOC_PREFIX + i);
    }
    // Add MAIN variables
    for (uint32_t i = 0; i < (CONFIG_MAIN_NUMVARS); i++) {
        addrTab[tabptr++] = (CONFIG_MAIN_PREFIX + i);
    }
    // Add THRT variables
    for (uint32_t i = 0; i < (CONFIG_THRT_NUMVARS); i++) {
        addrTab[tabptr++] = (CONFIG_THRT_PREFIX + i);
    }
    // Add LMT variables
    for (uint32_t i = 0; i < (CONFIG_LMT_NUMVARS); i++) {
        addrTab[tabptr++] = (CONFIG_LMT_PREFIX + i);
    }
    // Add MOTOR variables
    for (uint32_t i = 0; i < (CONFIG_MOTOR_NUMVARS); i++) {
        addrTab[tabptr++] = (CONFIG_MOTOR_PREFIX + i);
    }
}

/**
 * @brief  Restore the pages to a known good state in case of page's status
 *   corruption after a power loss.
 * @param  None.
 * @retval - Flash error code: on write Flash error
 *         - FLASH_COMPLETE: on success
 */
uint16_t EE_Init(uint16_t* addrTab) {
    EE_VirtAddVarTab = addrTab;

    EE_Page_Status PageStatus0, PageStatus1;
    uint16_t FlashStatus;

    // Check what bank mode we are in, dual bank or single bank.
    // The current mode determines the Flash addresses and page sizes
    RCC->AHB1ENR |= RCC_AHB1ENR_FLASHEN;
    if((FLASH->OPTR & FLASH_OPTR_DBANK) == 0) {
        // Single bank mode
        EE_Page0_Base_Address = PAGE0_BASE_ADDRESS_SINGLE;
        EE_Page1_Base_Address = PAGE1_BASE_ADDRESS_SINGLE;
        EE_Page_Size = PAGE_SIZE_SINGLE;
    } else {
        // Dual bank mode
        EE_Page0_Base_Address = PAGE0_BASE_ADDRESS_DUAL;
        EE_Page1_Base_Address = PAGE1_BASE_ADDRESS_DUAL;
        EE_Page_Size = PAGE_SIZE_SINGLE;
    }

    // Get Page statuses
    PageStatus0 = EE_GetPageStatus(PAGE0);
    PageStatus1 = EE_GetPageStatus(PAGE1);

    // Check for invalid header states and repair if necessary
    switch (PageStatus0) {
    case EE_Page_Erased:
        if (PageStatus1 == EE_Page_Active) // Page0 erased, Page1 valid
        {
            // Erase Page0 only if it was improperly erased.
            FlashStatus = FLASH_CheckErasedAndFix(EE_Page0_Base_Address);
            // If erase operation was failed, a Flash error code is returned
            if (FlashStatus != FLASH_COMPLETE) {
                return FlashStatus;
            }
        } else if (PageStatus1 == EE_Page_Receive) // Page0 erased, Page1 receive
        {
            // Erase Page0 only if it was improperly erased.
            FlashStatus = FLASH_CheckErasedAndFix(EE_Page0_Base_Address);
            // If erase operation was failed, a Flash error code is returned
            if (FlashStatus != FLASH_COMPLETE) {
                return FlashStatus;
            }
            // Mark Page1 as valid
            FlashStatus = EE_MarkStatus(PAGE1, EE_Page_Active);
            // If program operation was failed, a Flash error code is returned
            if (FlashStatus != FLASH_COMPLETE) {
                return FlashStatus;
            }
        } else // First EEPROM access (Page0&1 are erased) or invalid state -> format EEPROM
        {
            // Erase both Page0 and Page1 and set Page0 as valid page
            FlashStatus = EE_Format();
            // If erase/program operation was failed, a Flash error code is returned
            if (FlashStatus != FLASH_COMPLETE) {
                return FlashStatus;
            }
        }
        break;

    case EE_Page_Receive:
        if (PageStatus1 == EE_Page_Active) // Page0 receive, Page1 valid
        {
            // Transfer data from Page1 to Page0
            FlashStatus = EE_PageTransfer(PAGE1, PAGE0, 0xFFFF);
            if (FlashStatus != FLASH_COMPLETE) {
                return FlashStatus;
            }
        } else if (PageStatus1 == EE_Page_Erased) // Page0 receive, Page1 erased
        {
            // Erase Page1 only if it was improperly erased.
            FlashStatus = FLASH_CheckErasedAndFix(EE_Page1_Base_Address);
            // If erase operation was failed, a Flash error code is returned
            if (FlashStatus != FLASH_COMPLETE) {
                return FlashStatus;
            }
            // Mark Page0 as valid
            FlashStatus = EE_MarkStatus(PAGE0, EE_Page_Active);
            // If program operation was failed, a Flash error code is returned
            if (FlashStatus != FLASH_COMPLETE) {
                return FlashStatus;
            }
        } else // Invalid state -> format eeprom
        {
            // Erase both Page0 and Page1 and set Page0 as valid page
            FlashStatus = EE_Format();
            // If erase/program operation was failed, a Flash error code is returned
            if (FlashStatus != FLASH_COMPLETE) {
                return FlashStatus;
            }
        }
        break;

    case EE_Page_Active:
        if (PageStatus1 == EE_Page_Active) // Invalid state -> format eeprom
        {
            // Erase both Page0 and Page1 and set Page0 as valid page
            FlashStatus = EE_Format();
            // If erase/program operation was failed, a Flash error code is returned
            if (FlashStatus != FLASH_COMPLETE) {
                return FlashStatus;
            }
        } else if (PageStatus1 == EE_Page_Erased) // Page0 valid, Page1 erased
        {
            // Erase Page1 only if it was improperly erased.
            FlashStatus = FLASH_CheckErasedAndFix(EE_Page1_Base_Address);
            // If erase operation was failed, a Flash error code is returned
            if (FlashStatus != FLASH_COMPLETE) {
                return FlashStatus;
            }
        } else // Page0 valid, Page1 receive
        {

            // Transfer data from Page0 to Page1
            FlashStatus = EE_PageTransfer(PAGE0, PAGE1, 0xFFFF);
            if (FlashStatus != FLASH_COMPLETE) {
                return FlashStatus;
            }
        }
        break;

    default: // Any other state -> format eeprom
        // Erase both Page0 and Page1 and set Page0 as valid page
        FlashStatus = EE_Format();
        // If erase/program operation was failed, a Flash error code is returned
        if (FlashStatus != FLASH_COMPLETE) {
            return FlashStatus;
        }
        break;
    }

    return FLASH_COMPLETE;
}

/**
 * @brief  Returns the last stored variable data, if found, which correspond to
 *   the passed virtual address
 * @param  VirtAddress: Variable virtual address
 * @param  Data: Global variable contains the read variable value
 * @retval Success or error status:
 *           - 0: if variable was found
 *           - 1: if the variable was not found
 *           - NO_VALID_PAGE: if no valid page was found.
 */
uint16_t EE_ReadVariable(uint16_t VirtAddress, uint32_t* Data) {
    uint16_t ValidPage = PAGE0;
    uint32_t AddressValue = 0xFFFF, ReadStatus = RETVAL_FAIL;
    uint32_t Address = EE_Page0_Base_Address, PageStartAddress =
            EE_Page0_Base_Address;

    // Get active Page for read operation
    ValidPage = EE_FindValidPage(READ_FROM_VALID_PAGE);

    // Check if there is no valid page
    if (ValidPage == NO_VALID_PAGE) {
        return NO_VALID_PAGE;
    }

    // Get the valid Page start Address
    if(ValidPage == PAGE0) {
        PageStartAddress = (uint32_t) (EE_Page0_Base_Address);
    } else {
        PageStartAddress = (uint32_t) (EE_Page1_Base_Address);
    }

    // Get the valid Page end Address
    Address = (uint32_t) ((uint32_t)PageStartAddress + (uint32_t)EE_Page_Size - (uint32_t)8u);

    // Check each active page address starting from end
    while (Address >= (PageStartAddress + 8u)) {
        // Get the current location content to be compared with virtual address
        AddressValue = (*(__IO uint32_t*) Address);

        // Compare the read address with the virtual address
        if ((uint16_t)(AddressValue & 0xFFFFu) == VirtAddress) {
            // Get content of Address+4 which is variable value
            *Data = (*(__IO uint16_t*) (Address + 4));

            // In case variable value is read, reset ReadStatus flag
            ReadStatus = RETVAL_OK;

            break;
        } else {
            // Next address location
            Address = Address - 8u;
        }
    }

    // Return ReadStatus value: (RETVAL_OK: variable exists, RETVAL_FAIL: variable doesn't exist)
    return ReadStatus;
}

/**
 * @brief  Writes/updates variable data in EEPROM.
 * @param  VirtAddress: Variable virtual address
 * @param  Data: 16 bit data to be written
 * @retval Success or error status:
 *           - FLASH_COMPLETE: on success
 *           - PAGE_FULL: if valid page is full
 *           - NO_VALID_PAGE: if no valid page was found
 *           - Flash error code: on write Flash error
 */
uint16_t EE_WriteVariable(uint16_t VirtAddress, uint32_t Data) {
    uint16_t Status = 0;

    // Write the variable virtual address and value in the EEPROM
    Status = EE_VerifyPageFullWriteVariable(VirtAddress, Data);

    // In case the EEPROM active page is full, transfer to the other page
    // Only transfer newest values
    // This should free up the old settings which have been previously
    // overwritten by newer saved values
    if (Status == PAGE_FULL) {
        // Perform Page transfer
        if((EE_GetPageStatus(PAGE0) == EE_Page_Active) &&
                ((EE_GetPageStatus(PAGE1) == EE_Page_Erased) || (EE_GetPageStatus(PAGE1) == EE_Page_Receive))) {
            Status = EE_PageTransfer(PAGE0, PAGE1, VirtAddress);
        } else {
            Status = EE_PageTransfer(PAGE1, PAGE0, VirtAddress);
        }

        // Write to the new page
        Status = EE_VerifyPageFullWriteVariable(VirtAddress, Data);
        // If it's still full, we can't do anything anyway.
        // Just return the fault status.
    }

    /* Return last operation status */
    return Status;
}

uint16_t EE_SaveInt16(uint16_t VirtAddress, int16_t Data) {
    uint32_t temp = 0x00000000u;
    int16_t* temp_ptr = (int16_t*)&temp;
    temp_ptr[0] = Data;
    return EE_WriteVariable(VirtAddress, temp);
}

uint16_t EE_SaveInt32(uint16_t VirtAddress, int32_t Data) {
    uint32_t* temp_ptr = (uint32_t*)&Data;
    return EE_WriteVariable(VirtAddress, *temp_ptr);
}

uint16_t EE_SaveFloat(uint16_t VirtAddress, float Data) {
    uint32_t* temp_ptr = (uint32_t*)&Data;
    return EE_WriteVariable(VirtAddress, *temp_ptr);
}

int16_t EE_ReadInt16WithDefault(uint16_t VirtAddress, int16_t defalt) {
    uint16_t Status = 0;
    uint32_t Data;
    int16_t* data_ptr = (int16_t*)&Data;
    int16_t retval;
    Status = EE_ReadVariable(VirtAddress, &Data);
    if (Status == RETVAL_OK) {
        retval = data_ptr[0];
    } else {
        retval = defalt;
    }
    return retval;
}

int32_t EE_ReadInt32WithDefault(uint16_t VirtAddress, int32_t defalt) {
    uint16_t Status = 0;
    int32_t Data;
    int32_t retval;
    Status = EE_ReadVariable(VirtAddress, (uint32_t*)(&Data));
    if (Status == RETVAL_OK) {
        retval = Data;
    } else {
        retval = defalt;
    }

    return retval;
}

float EE_ReadFloatWithDefault(uint16_t VirtAddress, float defalt) {
    uint16_t Status = 0;
    uint32_t Data;
    float retval;
    Status = EE_ReadVariable(VirtAddress, (uint32_t*)(&Data));
    if (Status == RETVAL_OK) {
        retval = Data;
    } else {
            retval = defalt;
    }

    return retval;
}

/**
 * @brief  Unlocks the FLASH control register (FLASH->CR).
 * @param  None
 * @retval None
 */
static void FLASH_Unlock(void) {
    if ((FLASH->CR & FLASH_CR_LOCK) != RESET) {
        // Authorize the FLASH Registers access
        FLASH->KEYR = FLASH_KEY1;
        FLASH->KEYR = FLASH_KEY2;
    }
}

/**
 * @brief  Locks the FLASH control register (FLASH->CR).
 * @param  None
 * @retval None
 */
static void FLASH_Lock(void) {
    // Set the LOCK Bit to lock the FLASH Registers access
    FLASH->CR |= FLASH_CR_LOCK;
}

/**
 * @brief  Returns the FLASH Status.
 * @param  None
 * @retval FLASH Status: The returned value can be: FLASH_BUSY, FLASH_ERROR_PROGRAM,
 *                       FLASH_ERROR_WRP, FLASH_ERROR_OPERATION or FLASH_COMPLETE.
 */
static FLASH_Status FLASH_GetStatus(void) {
    FLASH_Status flashstatus = FLASH_COMPLETE;

    if ((FLASH->SR & FLASH_SR_BSY) != 0) {
        flashstatus = FLASH_BUSY;
    } else if((FLASH->SR & FLASH_SR_WRPERR) != (uint32_t) 0x00) {
        flashstatus = FLASH_ERROR_WRP;
    } else if ((FLASH->SR & (uint32_t) 0x3EAu) != (uint32_t) 0x00) {
        flashstatus = FLASH_ERROR_PROGRAM;
    } else if ((FLASH->SR & FLASH_SR_OPERR) != (uint32_t) 0x00) {
        flashstatus = FLASH_ERROR_OPERATION;
    } else {
        flashstatus = FLASH_COMPLETE;
    }

    /* Return the FLASH Status */
    return flashstatus;
}

/**
 * @brief  Waits for a FLASH operation to complete.
 * @param  None
 * @retval FLASH Status: The returned value can be: FLASH_BUSY, FLASH_ERROR_PROGRAM,
 *                       FLASH_ERROR_WRP, FLASH_ERROR_OPERATION or FLASH_COMPLETE.
 */
static FLASH_Status FLASH_WaitForLastOperation(void) {
    __IO FLASH_Status status = FLASH_COMPLETE;

    /* Check for the FLASH Status */
    status = FLASH_GetStatus();

    /* Wait for the FLASH operation to complete by polling on BUSY flag to be reset.
     Even if the FLASH operation fails, the BUSY flag will be reset and an error
     flag will be set */
    while (status == FLASH_BUSY) {
        status = FLASH_GetStatus();
    }
    /* Return the operation status */
    return status;
}

/**
 * @brief  Waits for a FLASH erase to complete.
 * @param  None
 * @retval FLASH Status: The returned value can be: FLASH_BUSY, FLASH_ERROR_PROGRAM,
 *                       FLASH_ERROR_WRP, FLASH_ERROR_OPERATION or FLASH_COMPLETE.
 */
static FLASH_Status FLASH_WaitForErase(void) {
    __IO FLASH_Status status = FLASH_COMPLETE;

    // Check for the FLASH Status
    status = FLASH_GetStatus();

    // Wait for the FLASH operation to complete by polling on BUSY flag to be reset.
    // Even if the FLASH operation fails, the BUSY flag will be reset and an error
    // flag will be set
    while (status == FLASH_BUSY) {
        status = FLASH_GetStatus();
    }
    // Return the operation status
    return status;
}

/**
 * @brief  Erases a specified FLASH Sector.
 *
 * @param  FLASH_Page The page number to be erased.
 *          This parameter can be a value between 0 and 255
 * @param  FLASH_Bank The bank in which the page will be erased, 0 or 1.
 *          If dual bank is disabled (single bank mode), this should always be zero
 *
 * @retval FLASH Status The returned value can be: FLASH_BUSY, FLASH_ERROR_PROGRAM,
 *                       FLASH_ERROR_WRP, FLASH_ERROR_OPERATION or FLASH_COMPLETE.
 */
static FLASH_Status FLASH_ErasePage(uint32_t FLASH_Page, uint8_t FLASH_Bank) {
    FLASH_Status status = FLASH_COMPLETE;

    // Wait for last operation to be completed
    status = FLASH_WaitForLastOperation();

    if (status == FLASH_COMPLETE) {
        FLASH_Unlock();
        // if the previous operation is completed, proceed to erase the sector
        FLASH->CR |= FLASH_CR_PER;
        if(FLASH_Bank == 1) {
            FLASH->CR |= FLASH_CR_BKER;
        } else {
            FLASH->CR &= ~(FLASH_CR_BKER);
        }
        FLASH->CR |= FLASH_Page << 3u;

        WDT_feed(); // Ensure there is no reset
        FLASH->CR |= FLASH_CR_STRT;

        // Wait for last operation to be completed
        status = FLASH_WaitForErase();

        // when the erase operation is completed, disable the page erase settings
        FLASH->CR &= ~(FLASH_CR_PER | FLASH_CR_BKER | FLASH_CR_PNB);
        FLASH_Lock();
    }
    // Return the Erase Status
    return status;
}

/**
 * @brief  Programs a double word (64-bit) at a specified address.
 *
 *         The double word is the smallest possible Flash programming
 *         size for this microcontroller. Programming can be done with
 *         any new value on erased data (aka 0xFFFF.FFFF.FFFF.FFFF) or
 *         with all zeros on any data.
 *
 * @param  Address: specifies the address to be programmed.
 *         This parameter can be any address in Program memory zone or in OTP zone.
 * @param  Data1: specifies the first data word to be programmed.
 * @param  Data2: specifies the second data word to be programmed (in Address+4)
 * @retval FLASH Status: The returned value can be: FLASH_BUSY, FLASH_ERROR_PROGRAM,
 *                       FLASH_ERROR_WRP, FLASH_ERROR_OPERATION or FLASH_COMPLETE.
 */
FLASH_Status FLASH_ProgramDoubleWord(uint32_t Address, uint32_t Data1, uint32_t Data2) {
    FLASH_Status status = FLASH_COMPLETE;

    // Wait for last operation to be completed
    status = FLASH_WaitForLastOperation();

    if (status == FLASH_ERROR_PROGRAM) {
        // try to clear the flags
        if ((FLASH->SR & 0x3EAu) != 0x00) {
            FLASH->SR |= 0x3EAu; // Try to clear all of: FASTERR, MISSERR, PGSERR, SIZERR, PGAERR, PROGERR
        }
        status = FLASH_WaitForLastOperation();
    }

    if (status == FLASH_COMPLETE) {
        FLASH_Unlock();
        // if the previous operation is completed, proceed to program the new data
        FLASH->CR |= FLASH_CR_PG;

        *(__IO uint32_t*) Address = Data1;
        Address += 4;
        *(__IO uint32_t*) Address = Data2;

        // Wait for last operation to be completed
        status = FLASH_WaitForLastOperation();

        // if the program operation is completed, disable the PG Bit
        FLASH->CR &= (~FLASH_CR_PG);
        FLASH_Lock();
    }
    // Return the Program Status
    return status;
}

/**
 * @brief  Verifies that the page was properly erased, and erases again if it wasn't.
 *
 *         The way to verify is simply by checking that every single byte is erased (0xFF)
 *
 * @param  Address: The start address of the page to check. Must be aligned to a
 *         page boundary.
 * @retval The status of the last operation, or FLASH_COMPLETE if nothing needed to be erased.
 */
static FLASH_Status FLASH_CheckErasedAndFix(uint32_t Address) {
    uint16_t page_num;
    // Check bank mode
    if((FLASH->OPTR & FLASH_OPTR_DBANK) == 0) {
        // Single bank mode
        // Check that the last 12 bits are clear (4KB boundary alignment)
        if((Address & 0xFFFFF000) == Address) {
            // Which page is this?
            page_num = (Address - FLASH_START_ADDRESS) / PAGE_SIZE_SINGLE;
            if(page_num <= 127) {
                return FLASH_ErasePage(page_num, 0);
            } else {
                return FLASH_ERROR_PROGRAM; // Out of range
            }
        } else {
            return FLASH_ERROR_PROGRAM; // Wrong alignment
        }
    } else {
        // Dual bank mode
        // Check that the last 11 bits are clear (2KB boundary alignment)
        if((Address & 0xFFFFF800) == Address) {
            // Which bank is this?
            if(Address >= BANK2_START_ADDRESS) {
                // Bank2
                page_num = (Address - BANK2_START_ADDRESS) / PAGE_SIZE_DUAL;
                if(page_num <= 127) {
                    return FLASH_ErasePage(page_num, 1);
                } else {
                    return FLASH_ERROR_PROGRAM; // Out of range
                }
            } else {
                // Bank1
                page_num = (Address - BANK1_START_ADDRESS) / PAGE_SIZE_DUAL;
                if(page_num <= 127) {
                    return FLASH_ErasePage(page_num, 0);
                } else {
                    return FLASH_ERROR_PROGRAM; // Out of range
                }
            }
        } else {
            return FLASH_ERROR_PROGRAM; // Wrong alignment
        }
    }
}

/**
 * @brief  Gets the status of the requested page, stored in the first double-word.
 * @param  Page: The page ID (see header file) for the requested page.
 * @retval Page status, any of the enum EE_Page_Status
 */
static EE_Page_Status EE_GetPageStatus(uint16_t Page)
{
    uint32_t* stat_ptr;
    uint32_t stat1, stat2;
    EE_Page_Status retval = EE_Page_Invalid;
    if(Page == PAGE0) {
        stat_ptr = (uint32_t*)(EE_Page0_Base_Address);
    } else {
        stat_ptr = (uint32_t*)(EE_Page1_Base_Address);
    }
    stat1 = *(stat_ptr);
    stat_ptr++;
    stat2 = *(stat_ptr);

    // Status flags translation -
    // When page has been erased, everything will be all ones (0xFFFF.FFFF)
    // If marked for receiving, the double-word will be flagged
    // If actively recording new EE writes, both words will be flagged
    // If full of valid data, first word will be zeroed and second word flagged
    // And finally, if ready for erasure, both words will be zeroed
    if((stat1 == FLASH_ERASED) && (stat2 == FLASH_ERASED)) {
        retval = EE_Page_Erased;
    } else if((stat1 == FLASH_FLAGGED) && (stat2 == FLASH_FLAGGED)) {
        retval = EE_Page_Receive;
    } else if((stat1 == FLASH_ZEROED) && (stat2 == FLASH_ZEROED)) {
        retval = EE_Page_Active;
    }
    // Any other values stored in the first two double-words are invalid. The first setting of
    // retval (EE_Page_Invalid) will be returned.

    return retval;
}

/**
 * @brief  Sets the status bytes of a page to the new setting.
 *
 *         If the new setting will not be applied correctly (e.g. writing
 *         new non-zero data on previously written data), the function
 *         will immediately return with a FLASH_ERROR_PROGRAM.
 * @param  PageId: the page to mark. See header for valid pages
 * @param  PageStatus: the new status, of type EE_Page_Status
 * @retval The last Flash operation status
 */
static FLASH_Status EE_MarkStatus(uint16_t PageId, EE_Page_Status PageStatus) {

    EE_Page_Status currentStatus = EE_GetPageStatus(PageId);
    // Quick return if trying to write the same status
    if(currentStatus == PageStatus)
        return FLASH_COMPLETE;

    FLASH_Status retval = FLASH_COMPLETE;
    uint32_t page_address;
    if(PageId == PAGE0) {
        page_address = EE_Page0_Base_Address;
    } else {
        page_address = EE_Page1_Base_Address;
    }

    switch(currentStatus) {
    case EE_Page_Erased:
        // Anything is allowed
        if(PageStatus == EE_Page_Receive) {
            retval = FLASH_ProgramDoubleWord(page_address, FLASH_FLAGGED, FLASH_FLAGGED);

        } else if(PageStatus == EE_Page_Active) {
            retval = FLASH_ProgramDoubleWord(page_address, FLASH_ZEROED, FLASH_ZEROED);
        } else {
            retval = FLASH_ERROR_PROGRAM;
        }
        break;
    case EE_Page_Receive:
        // Cannot write Erased
        if(PageStatus == EE_Page_Active) {
            retval = FLASH_ProgramDoubleWord(page_address, FLASH_ZEROED, FLASH_ZEROED);
        } else {
            retval = FLASH_ERROR_PROGRAM;
        }
        break;
    case EE_Page_Active:
    default:
        // Can't write anything!
        retval = FLASH_ERROR_PROGRAM;
        break;
    }

    return retval;
}

/**
 * @brief  Erases PAGE and PAGE1 and writes VALID_PAGE header to PAGE
 * @param  None
 * @retval Status of the last operation (Flash write or erase) done during
 *         EEPROM formating
 */
static FLASH_Status EE_Format(void) {
    FLASH_Status FlashStatus = FLASH_COMPLETE;

    // Erase Page0
    FlashStatus = EE_ErasePage(PAGE0);

    // If erase operation was failed, a Flash error code is returned
    if (FlashStatus != FLASH_COMPLETE) {
        return FlashStatus;
    }

    // Set Page0 as valid page: Write Active Page at Page0 base address
    FlashStatus = EE_MarkStatus(PAGE0, EE_Page_Active);

    // If program operation was failed, a Flash error code is returned
    if (FlashStatus != FLASH_COMPLETE) {
        return FlashStatus;
    }

    // Erase Page1
    FlashStatus = EE_ErasePage(PAGE1);

    // Return Page1 erase operation status
    return FlashStatus;
}


/**
 * @brief  Erases a single Flash page
 * @param  PageId: The page to erase, see definitions in header file
 * @retval Status of the erase operation
 */
static uint16_t EE_ErasePage(uint16_t PageId) {
    // Which bank mode?
    if((FLASH->OPTR & FLASH_OPTR_DBANK) == 0) {
        // Single bank mode
        if(PageId == PAGE0)
            return FLASH_ErasePage(PAGE0_PAGE_NUM, 0);
        else
            return FLASH_ErasePage(PAGE1_PAGE_NUM, 0);
    } else {
        // Dual bank mode
        if(PageId == PAGE0)
            return FLASH_ErasePage(PAGE0_PAGE_NUM, 1);
        else
            return FLASH_ErasePage(PAGE1_PAGE_NUM, 1);
    }
}

/**
 * @brief  Find valid Page for write or read operation
 * @param  Operation: operation to achieve on the valid page.
 *   This parameter can be one of the following values:
 *     @arg READ_FROM_VALID_PAGE: read operation from valid page
 *     @arg WRITE_IN_VALID_PAGE: write operation from valid page
 * @retval Valid page number (PAGE or PAGE1) or NO_VALID_PAGE in case
 *   of no valid page was found
 */
static uint16_t EE_FindValidPage(uint8_t Operation) {
    EE_Page_Status PageStatus0 = EE_Page_Invalid, PageStatus1 = EE_Page_Invalid;

    // Get Page0 actual status
    PageStatus0 = EE_GetPageStatus(PAGE0);

    // Get Page1 actual status
    PageStatus1 = EE_GetPageStatus(PAGE1);

    // Write or read operation
    switch (Operation) {
    case WRITE_IN_VALID_PAGE: // ---- Write operation ----
        if (PageStatus1 == EE_Page_Active) {
            // Page0 receiving data
            if (PageStatus0 == EE_Page_Receive) {
                return PAGE0; // Page0 valid
            } else {
                return PAGE1; // Page1 valid
            }
        } else if (PageStatus0 == EE_Page_Active) {
            // Page1 receiving data
            if (PageStatus1 == EE_Page_Receive) {
                return PAGE1; // Page1 valid
            } else {
                return PAGE0; // Page0 valid
            }
        } else {
            return NO_VALID_PAGE; // No valid Page
        }

    case READ_FROM_VALID_PAGE: // ---- Read operation ----
        if (PageStatus0 == EE_Page_Active) {
            return PAGE0; // Page0 valid
        } else if (PageStatus1 == EE_Page_Active) {
            return PAGE1; // Page1 valid
        } else {
            return NO_VALID_PAGE; // No valid Page
        }

    default:
        return PAGE0; // Page0 valid
    }
}

/**
 * @brief  Verify if active page is full and Writes variable in EEPROM.
 * @param  VirtAddress: 16 bit virtual address of the variable
 * @param  Data: 32 bit data to be written as variable value
 * @retval Success or error status:
 *           - FLASH_COMPLETE: on success
 *           - PAGE_FULL: if valid page is full
 *           - NO_VALID_PAGE: if no valid page was found
 *           - Flash error code: on write Flash error
 */
static uint16_t EE_VerifyPageFullWriteVariable(uint16_t VirtAddress, uint32_t Data) {
    FLASH_Status FlashStatus = FLASH_COMPLETE;
    uint16_t ValidPage = PAGE0;
    uint32_t Address = EE_Page0_Base_Address, PageEndAddress =
            EE_Page0_Base_Address + EE_Page_Size;

    // Get valid Page for write operation
    ValidPage = EE_FindValidPage(WRITE_IN_VALID_PAGE);

    // Check if there is no valid page
    if (ValidPage == NO_VALID_PAGE) {
        return NO_VALID_PAGE;
    }

    // Get the valid Page start and end addresses
    if(ValidPage == PAGE0) {
        Address = EE_Page0_Base_Address + (uint32_t)8u;
        PageEndAddress = EE_Page0_Base_Address + EE_Page_Size;
    } else if(ValidPage == PAGE1) {
        Address = EE_Page1_Base_Address + (uint32_t)8u;
        PageEndAddress = EE_Page1_Base_Address + EE_Page_Size;
    } else {
        return NO_VALID_PAGE;
    }

    // Check each active page address starting from begining
    while (Address < PageEndAddress) {
        // Verify if Address and Address+4 contents are 0xFFFFFFFF
        if (((*(__IO uint32_t*) Address) == 0xFFFFFFFF) && ((*(__IO uint32_t*) (Address + 4u)) == 0xFFFFFFFF)) {
            // Set variable data
            FlashStatus = FLASH_ProgramDoubleWord(Address, (uint32_t)VirtAddress, Data);
            // If program operation was failed, a Flash error code is returned
            // Return program operation status
            return FlashStatus;
        } else {
            // Next address location
            Address = Address + 8u;
        }
    }

    // Return PAGE_FULL in case the valid page is full
    return PAGE_FULL;
}

#if 0
/**
 * @brief  Transfers last updated variables data from the full Page to
 *   an empty one.
 * @param  VirtAddress: 16 bit virtual address of the variable
 * @param  Data: 16 bit data to be written as variable value
 * @retval Success or error status:
 *           - FLASH_COMPLETE: on success
 *           - PAGE_FULL: if valid page is full
 *           - NO_VALID_PAGE: if no valid page was found
 *           - Flash error code: on write Flash error
 */
static uint16_t EE_WriteNewVariableWithPageTransfer(uint16_t VirtAddress, uint16_t Data) {
    FLASH_Status FlashStatus = FLASH_COMPLETE;
    uint32_t NewPageAddress = EE_Page0_Base_Address;
    uint16_t OldPageId = 0;
    uint16_t NewPageId = 0;
    uint16_t ValidPage = PAGE0, VarIdx = 0;
    uint16_t EepromStatus = 0, ReadStatus = 0;

    // Get active Page for read operation
    ValidPage = EE_FindValidPage(READ_FROM_VALID_PAGE);

    if (ValidPage == PAGE1) // Page1 valid
    {
        // New page address where variable will be moved to
        NewPageAddress = EE_Page0_Base_Address;
        NewPageId = PAGE0;

        // Old page ID where variable will be taken from
        OldPageId = PAGE1;
    } else if (ValidPage == PAGE0) // Page0 valid
    {
        // New page address  where variable will be moved to
        NewPageAddress = EE_Page1_Base_Address;
        NewPageId = PAGE1;

        // Old page ID where variable will be taken from
        OldPageId = PAGE0;
    } else {
        return NO_VALID_PAGE; // No valid Page
    }

    // Set the new Page status to RECEIVE_DATA status
    FlashStatus = EE_MarkStatus(PAGE1, EE_Page_Receive);
    // If program operation was failed, a Flash error code is returned
    if (FlashStatus != FLASH_COMPLETE) {
        return FlashStatus;
    }
    // Write the variable passed as parameter in the new active page
    EepromStatus = EE_VerifyPageFullWriteVariable(VirtAddress, Data);
    // If program operation was failed, a Flash error code is returned
    if (EepromStatus != FLASH_COMPLETE) {
        return EepromStatus;
    }

    // Transfer process: transfer variables from old to the new active page
    for (VarIdx = 0; VarIdx < (TOTAL_EE_VARS*2); VarIdx++) {
        if (EE_VirtAddVarTab[VarIdx] != VirtAddress) // Check each variable except the one passed as parameter
        {
            // Read the other last variable updates
            ReadStatus = EE_ReadVariable(EE_VirtAddVarTab[VarIdx], &DataVar);
            // In case variable corresponding to the virtual address was found
            if (ReadStatus != 0x1) {
                // Transfer the variable to the new active page
                EepromStatus = EE_VerifyPageFullWriteVariable(
                        EE_VirtAddVarTab[VarIdx], DataVar);
                // If program operation was failed, a Flash error code is returned
                if (EepromStatus != FLASH_COMPLETE) {
                    return EepromStatus;
                }
            }
        }
    }

    // Erase the old Page: Set old Page status to ERASED status
    FlashStatus = FLASH_EraseSector(OldPageId, VOLTAGE_RANGE);
    // If erase operation was failed, a Flash error code is returned
    if (FlashStatus != FLASH_COMPLETE) {
        return FlashStatus;
    }

    // Set new Page status to VALID_PAGE status
    FlashStatus = FLASH_ProgramHalfWord(NewPageAddress, VALID_PAGE);
    // If program operation was failed, a Flash error code is returned
    if (FlashStatus != FLASH_COMPLETE) {
        return FlashStatus;
    }

    // Return last operation flash status
    return FlashStatus;
}
#endif

/**
 * @brief  Transfers the most recent variable values from old page to new page.
 *
 *         If this function is being called when writing a new variable value, the
 *         new variable ID should be passed in so it is skipped in transfer. That
 *         saves a single slot, since the old value would be overwritten anyway.
 *         This function completes the setting of status flags, too. At the end
 *         of the function, OldPage will be erased, and NewPage will be marked
 *         as EE_Page_Active.
 *
 * @param  OldPageId: The page to take old data from, see definitions in header file
 * @param  NewPageId: The page to store data in, see definitions in header file
 * @param  SkipId: A variable virtual address to be skipped. If all variables are
 *         to be transferred, assign a unused address, like 0xFFFF
 */
static uint16_t EE_PageTransfer(uint16_t OldPageId, uint16_t NewPageId, uint16_t SkipId) {
    FLASH_Status FlashStatus = FLASH_COMPLETE;
    uint16_t VarIdx, ReadStatus, EepromStatus;

    // Set the new Page status to RECEIVE_DATA status
    FlashStatus = EE_MarkStatus(NewPageId, EE_Page_Receive);
    // If program operation was failed, a Flash error code is returned
    if (FlashStatus != FLASH_COMPLETE) {
        return FlashStatus;
    }

    uint32_t DataVar;
    // Transfer process: transfer variables from old to the new active page
    for (VarIdx = 0; VarIdx < TOTAL_EE_VARS; VarIdx++) {
        // If SkipId is a real variable ID, we don't transfer it.
        // This is used when writing a new variable value causes a page transfer.
        if(EE_VirtAddVarTab[VarIdx] != SkipId) {
            // Read the last variable update
            ReadStatus = EE_ReadVariable(EE_VirtAddVarTab[VarIdx], &DataVar);
            // In case variable corresponding to the virtual address was found
            if (ReadStatus == RETVAL_OK) {
                // Transfer the variable to the new active page
                EepromStatus = EE_VerifyPageFullWriteVariable(
                        EE_VirtAddVarTab[VarIdx], DataVar);
                // If program operation was failed, a Flash error code is returned
                if (EepromStatus != FLASH_COMPLETE) {
                    return EepromStatus;
                }
            }
        }
    }
    // Erase the old Page: Set old Page status to ERASED status
    FlashStatus = EE_ErasePage(OldPageId);
    // If erase operation was failed, a Flash error code is returned
    if (FlashStatus != FLASH_COMPLETE) {
        return FlashStatus;
    }

    // Set new Page status to VALID_PAGE status
    FlashStatus = EE_MarkStatus(NewPageId, EE_Page_Active);
    // If program operation was failed, a Flash error code is returned
    // Return the status of the last operation
    return FlashStatus;
}
