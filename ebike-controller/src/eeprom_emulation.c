/******************************************************************************
 * Filename: eeprom_emulation.c
 * Description: Uses the STM32 Flash memory as if it were EEPROM. All "EEPROM"
 *              variables are saved in Flash with an associated variable
 *              address. Saving a new value to a particular address actually
 *              just stores a new variable/address combo. Erases are only
 *              performed when a page becomes completely full. The most recent
 *              saved value for each variable is transferred to the alternate
 *              storage page, and the alternate page is marked as active.
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

/* Includes ------------------------------------------------------------------*/
#include "eeprom_emulation.h"
#include "project_parameters.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Global variable used to store variable value in read sequence */
uint16_t DataVar = 0;

/* Virtual address defined by the user: 0xFFFF value is prohibited */
uint16_t* EE_VirtAddVarTab;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/* From ST StdPeriph Flash ---------------------------------------------------*/
#define FLASH_KEY1               ((uint32_t)0x45670123)
#define FLASH_KEY2               ((uint32_t)0xCDEF89AB)
static void FLASH_Unlock(void);
static void FLASH_Lock(void);
static FLASH_Status FLASH_GetStatus(void);
static FLASH_Status FLASH_WaitForLastOperation(void);
static FLASH_Status FLASH_EraseSector(uint32_t FLASH_Sector,
        uint8_t VoltageRange);
static FLASH_Status FLASH_ProgramHalfWord(uint32_t Address, uint16_t Data);

static FLASH_Status EE_Format(void);
static uint16_t EE_FindValidPage(uint8_t Operation);
static uint16_t EE_VerifyPageFullWriteVariable(uint16_t VirtAddress,
        uint16_t Data);
static uint16_t EE_PageTransfer(uint16_t VirtAddress, uint16_t Data);

void EE_Config_Addr_Table(uint16_t* addrTab) {
    uint32_t tabptr = 0;

    // Add ADC variables
    for (uint32_t i = 0; i < (CONFIG_ADC_NUMVARS); i++) {
        addrTab[tabptr++] = (CONFIG_ADC_PREFIX + i) | EE_LOBYTE_FLAG;
        addrTab[tabptr++] = (CONFIG_ADC_PREFIX + i) | EE_HIBYTE_FLAG;
    }
    // Add FOC variables
    for (uint32_t i = 0; i < (CONFIG_FOC_NUMVARS); i++) {
        addrTab[tabptr++] = (CONFIG_FOC_PREFIX + i) | EE_LOBYTE_FLAG;
        addrTab[tabptr++] = (CONFIG_FOC_PREFIX + i) | EE_HIBYTE_FLAG;
    }
    // Add MAIN variables
    for (uint32_t i = 0; i < (CONFIG_MAIN_NUMVARS); i++) {
        addrTab[tabptr++] = (CONFIG_MAIN_PREFIX + i) | EE_LOBYTE_FLAG;
        addrTab[tabptr++] = (CONFIG_MAIN_PREFIX + i) | EE_HIBYTE_FLAG;
    }
    // Add THRT variables
    for (uint32_t i = 0; i < (CONFIG_THRT_NUMVARS); i++) {
        addrTab[tabptr++] = (CONFIG_THRT_PREFIX + i) | EE_LOBYTE_FLAG;
        addrTab[tabptr++] = (CONFIG_THRT_PREFIX + i) | EE_HIBYTE_FLAG;
    }
    // Add LMT variables
    for (uint32_t i = 0; i < (CONFIG_LMT_NUMVARS); i++) {
        addrTab[tabptr++] = (CONFIG_LMT_PREFIX + i) | EE_LOBYTE_FLAG;
        addrTab[tabptr++] = (CONFIG_LMT_PREFIX + i) | EE_HIBYTE_FLAG;
    }
    // Add MOTOR variables
    for (uint32_t i = 0; i < (CONFIG_MOTOR_NUMVARS); i++) {
        addrTab[tabptr++] = (CONFIG_MOTOR_PREFIX + i) | EE_LOBYTE_FLAG;
        addrTab[tabptr++] = (CONFIG_MOTOR_PREFIX + i) | EE_HIBYTE_FLAG;
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

    uint16_t PageStatus0 = 6, PageStatus1 = 6;
    uint16_t VarIdx = 0;
    uint16_t EepromStatus = 0, ReadStatus = 0;
    int16_t x = -1;
    uint16_t FlashStatus;

    /* Get Page0 status */
    PageStatus0 = (*(__IO uint16_t*) PAGE0_BASE_ADDRESS);
    /* Get Page1 status */
    PageStatus1 = (*(__IO uint16_t*) PAGE1_BASE_ADDRESS);

    /* Check for invalid header states and repair if necessary */
    switch (PageStatus0) {
    case ERASED:
        if (PageStatus1 == VALID_PAGE) /* Page0 erased, Page1 valid */
        {
            /* Erase Page0 */
            // But why??? Page0 is already erased! Why waste precious
            // flash cycles on this?
            // FlashStatus = FLASH_EraseSector(PAGE0_ID,VOLTAGE_RANGE);
            FlashStatus = FLASH_COMPLETE;
            /* If erase operation was failed, a Flash error code is returned */
            if (FlashStatus != FLASH_COMPLETE) {
                return FlashStatus;
            }
        } else if (PageStatus1 == RECEIVE_DATA) /* Page0 erased, Page1 receive */
        {
            /* Erase Page0 */
            // But why??? Page0 is already erased! Why waste precious
            // flash cycles on this?
            // FlashStatus = FLASH_EraseSector(PAGE0_ID,VOLTAGE_RANGE);
            FlashStatus = FLASH_COMPLETE;
            /* If erase operation was failed, a Flash error code is returned */
            if (FlashStatus != FLASH_COMPLETE) {
                return FlashStatus;
            }
            /* Mark Page1 as valid */
            FlashStatus = FLASH_ProgramHalfWord(PAGE1_BASE_ADDRESS, VALID_PAGE);
            /* If program operation was failed, a Flash error code is returned */
            if (FlashStatus != FLASH_COMPLETE) {
                return FlashStatus;
            }
        } else /* First EEPROM access (Page0&1 are erased) or invalid state -> format EEPROM */
        {
            /* Erase both Page0 and Page1 and set Page0 as valid page */
            FlashStatus = EE_Format();
            /* If erase/program operation was failed, a Flash error code is returned */
            if (FlashStatus != FLASH_COMPLETE) {
                return FlashStatus;
            }
        }
        break;

    case RECEIVE_DATA:
        if (PageStatus1 == VALID_PAGE) /* Page0 receive, Page1 valid */
        {
            /* Transfer data from Page1 to Page0 */
            for (VarIdx = 0; VarIdx < (TOTAL_EE_VARS*2); VarIdx++) {
                if ((*(__IO uint16_t*) (PAGE0_BASE_ADDRESS + 6))
                        == EE_VirtAddVarTab[VarIdx]) {
                    x = VarIdx;
                }
                if (VarIdx != x) {
                    /* Read the last variables' updates */
                    ReadStatus = EE_ReadVariable(EE_VirtAddVarTab[VarIdx],
                            &DataVar);
                    /* In case variable corresponding to the virtual address was found */
                    if (ReadStatus != 0x1) {
                        /* Transfer the variable to the Page0 */
                        EepromStatus = EE_VerifyPageFullWriteVariable(
                                EE_VirtAddVarTab[VarIdx], DataVar);
                        /* If program operation was failed, a Flash error code is returned */
                        if (EepromStatus != FLASH_COMPLETE) {
                            return EepromStatus;
                        }
                    }
                }
            }
            /* Mark Page0 as valid */
            FlashStatus = FLASH_ProgramHalfWord(PAGE0_BASE_ADDRESS, VALID_PAGE);
            /* If program operation was failed, a Flash error code is returned */
            if (FlashStatus != FLASH_COMPLETE) {
                return FlashStatus;
            }
            /* Erase Page1 */
            FlashStatus = FLASH_EraseSector(PAGE1_ID, VOLTAGE_RANGE);
            /* If erase operation was failed, a Flash error code is returned */
            if (FlashStatus != FLASH_COMPLETE) {
                return FlashStatus;
            }
        } else if (PageStatus1 == ERASED) /* Page0 receive, Page1 erased */
        {
            /* Erase Page1 */
            // But why??? Page1 is already erased! Why waste precious
            // flash cycles on this?
            // FlashStatus = FLASH_EraseSector(PAGE1_ID,VOLTAGE_RANGE);
            FlashStatus = FLASH_COMPLETE;
            /* If erase operation was failed, a Flash error code is returned */
            if (FlashStatus != FLASH_COMPLETE) {
                return FlashStatus;
            }
            /* Mark Page0 as valid */
            FlashStatus = FLASH_ProgramHalfWord(PAGE0_BASE_ADDRESS, VALID_PAGE);
            /* If program operation was failed, a Flash error code is returned */
            if (FlashStatus != FLASH_COMPLETE) {
                return FlashStatus;
            }
        } else /* Invalid state -> format eeprom */
        {
            /* Erase both Page0 and Page1 and set Page0 as valid page */
            FlashStatus = EE_Format();
            /* If erase/program operation was failed, a Flash error code is returned */
            if (FlashStatus != FLASH_COMPLETE) {
                return FlashStatus;
            }
        }
        break;

    case VALID_PAGE:
        if (PageStatus1 == VALID_PAGE) /* Invalid state -> format eeprom */
        {
            /* Erase both Page0 and Page1 and set Page0 as valid page */
            FlashStatus = EE_Format();
            /* If erase/program operation was failed, a Flash error code is returned */
            if (FlashStatus != FLASH_COMPLETE) {
                return FlashStatus;
            }
        } else if (PageStatus1 == ERASED) /* Page0 valid, Page1 erased */
        {
            /* Erase Page1 */
            // But why??? Page1 is already erased! Why waste precious
            // flash cycles on this?
            // FlashStatus = FLASH_EraseSector(PAGE1_ID,VOLTAGE_RANGE);
            FlashStatus = FLASH_COMPLETE;
            /* If erase operation was failed, a Flash error code is returned */
            if (FlashStatus != FLASH_COMPLETE) {
                return FlashStatus;
            }
        } else /* Page0 valid, Page1 receive */
        {
            /* Transfer data from Page0 to Page1 */
            for (VarIdx = 0; VarIdx < (TOTAL_EE_VARS*2); VarIdx++) {
                if ((*(__IO uint16_t*) (PAGE1_BASE_ADDRESS + 6))
                        == EE_VirtAddVarTab[VarIdx]) {
                    x = VarIdx;
                }
                if (VarIdx != x) {
                    /* Read the last variables' updates */
                    ReadStatus = EE_ReadVariable(EE_VirtAddVarTab[VarIdx],
                            &DataVar);
                    /* In case variable corresponding to the virtual address was found */
                    if (ReadStatus != 0x1) {
                        /* Transfer the variable to the Page1 */
                        EepromStatus = EE_VerifyPageFullWriteVariable(
                                EE_VirtAddVarTab[VarIdx], DataVar);
                        /* If program operation was failed, a Flash error code is returned */
                        if (EepromStatus != FLASH_COMPLETE) {
                            return EepromStatus;
                        }
                    }
                }
            }
            /* Mark Page1 as valid */
            FlashStatus = FLASH_ProgramHalfWord(PAGE1_BASE_ADDRESS, VALID_PAGE);
            /* If program operation was failed, a Flash error code is returned */
            if (FlashStatus != FLASH_COMPLETE) {
                return FlashStatus;
            }
            /* Erase Page0 */
            FlashStatus = FLASH_EraseSector(PAGE0_ID, VOLTAGE_RANGE);
            /* If erase operation was failed, a Flash error code is returned */
            if (FlashStatus != FLASH_COMPLETE) {
                return FlashStatus;
            }
        }
        break;

    default: /* Any other state -> format eeprom */
        /* Erase both Page0 and Page1 and set Page0 as valid page */
        FlashStatus = EE_Format();
        /* If erase/program operation was failed, a Flash error code is returned */
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
uint16_t EE_ReadVariable(uint16_t VirtAddress, uint16_t* Data) {
    uint16_t ValidPage = PAGE0;
    uint16_t AddressValue = 0x5555, ReadStatus = READ_NOT_FOUND;
    uint32_t Address = EEPROM_START_ADDRESS, PageStartAddress =
            EEPROM_START_ADDRESS;

    /* Get active Page for read operation */
    ValidPage = EE_FindValidPage(READ_FROM_VALID_PAGE);

    /* Check if there is no valid page */
    if (ValidPage == NO_VALID_PAGE) {
        return NO_VALID_PAGE;
    }

    /* Get the valid Page start Address */
    PageStartAddress = (uint32_t) (EEPROM_START_ADDRESS
            + (uint32_t) (ValidPage * PAGE_SIZE ));

    /* Get the valid Page end Address */
    Address = (uint32_t) ((EEPROM_START_ADDRESS - 2)
            + (uint32_t) ((1 + ValidPage) * PAGE_SIZE ));

    /* Check each active page address starting from end */
    while (Address > (PageStartAddress + 2)) {
        /* Get the current location content to be compared with virtual address */
        AddressValue = (*(__IO uint16_t*) Address);

        /* Compare the read address with the virtual address */
        if (AddressValue == VirtAddress) {
            /* Get content of Address-2 which is variable value */
            *Data = (*(__IO uint16_t*) (Address - 2));

            /* In case variable value is read, reset ReadStatus flag */
            ReadStatus = READ_SUCCESS;

            break;
        } else {
            /* Next address location */
            Address = Address - 4;
        }
    }

    /* Return ReadStatus value: (0: variable exist, 1: variable doesn't exist) */
    return ReadStatus;
}

/**
 * @brief  Writes/upadtes variable data in EEPROM.
 * @param  VirtAddress: Variable virtual address
 * @param  Data: 16 bit data to be written
 * @retval Success or error status:
 *           - FLASH_COMPLETE: on success
 *           - PAGE_FULL: if valid page is full
 *           - NO_VALID_PAGE: if no valid page was found
 *           - Flash error code: on write Flash error
 */
uint16_t EE_WriteVariable(uint16_t VirtAddress, uint16_t Data) {
    uint16_t Status = 0;

    /* Write the variable virtual address and value in the EEPROM */
    Status = EE_VerifyPageFullWriteVariable(VirtAddress, Data);

    /* In case the EEPROM active page is full */
    if (Status == PAGE_FULL) {
        /* Perform Page transfer */
        Status = EE_PageTransfer(VirtAddress, Data);
    }

    /* Return last operation status */
    return Status;
}

uint16_t EE_SaveInt16(uint16_t VirtAddress, int16_t Data) {
    uint16_t* DataPtr = (uint16_t*) (&Data);
    return EE_WriteVariable(VirtAddress, *DataPtr);
}

uint16_t EE_SaveInt32(uint16_t VirtAddress, int32_t Data) {
    uint16_t Status = 0;
    uint16_t* DataPtr = (uint16_t*) (&Data);
    Status = EE_WriteVariable(VirtAddress | EE_LOBYTE_FLAG, DataPtr[0]);
    if (Status == FLASH_COMPLETE) {
        Status = EE_WriteVariable(VirtAddress | EE_HIBYTE_FLAG, DataPtr[1]);
    }
    return Status;
}

uint16_t EE_SaveFloat(uint16_t VirtAddress, float Data) {
    uint16_t Status = 0;
    uint16_t* DataPtr = (uint16_t*) (&Data);
    /* Write the first two bytes in EEPROM */
    Status = EE_WriteVariable(VirtAddress | EE_LOBYTE_FLAG, DataPtr[0]);
    if (Status == FLASH_COMPLETE) {
        /* And then the second two bytes */
        Status = EE_WriteVariable(VirtAddress | EE_HIBYTE_FLAG, DataPtr[1]);
    }
    return Status;
}

int16_t EE_ReadInt16WithDefault(uint16_t VirtAddress, int16_t defalt) {
    uint16_t Status = 0;
    uint16_t Data;
    int16_t retval;
    Status = EE_ReadVariable(VirtAddress, &Data);
    if (Status == READ_SUCCESS) {
        retval = Data;
    } else {
        retval = defalt;
    }
    return retval;
}

int32_t EE_ReadInt32WithDefault(uint16_t VirtAddress, int32_t defalt) {
    uint16_t Status = 0;
    uint16_t Data;
    int32_t retval;
    uint16_t* retvalptr = (uint16_t*) (&retval);
    Status = EE_ReadVariable((VirtAddress | EE_LOBYTE_FLAG), &Data);
    if (Status == READ_SUCCESS) {
        retvalptr[0] = Data;
        Status = EE_ReadVariable((VirtAddress | EE_HIBYTE_FLAG), &Data);
        if (Status == READ_SUCCESS) {
            retvalptr[1] = Data;
        } else {
            retval = defalt;
        }
    } else {
        retval = defalt;
    }
    return retval;
}

float EE_ReadFloatWithDefault(uint16_t VirtAddress, float defalt) {
    uint16_t Status = 0;
    uint16_t Data;
    float retval;
    uint16_t* retvalptr = (uint16_t*) (&retval);
    Status = EE_ReadVariable((VirtAddress | EE_LOBYTE_FLAG), &Data);
    if (Status == READ_SUCCESS) {
        retvalptr[0] = Data;
        Status = EE_ReadVariable((VirtAddress | EE_HIBYTE_FLAG), &Data);
        if (Status == READ_SUCCESS) {
            retvalptr[1] = Data;
        } else {
            retval = defalt;
        }
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
        /* Authorize the FLASH Registers access */
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
    /* Set the LOCK Bit to lock the FLASH Registers access */
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

    if ((FLASH->SR & FLASH_FLAG_BSY) == FLASH_FLAG_BSY) {
        flashstatus = FLASH_BUSY;
    } else {
        if ((FLASH->SR & FLASH_FLAG_WRPERR) != (uint32_t) 0x00) {
            flashstatus = FLASH_ERROR_WRP;
        } else {
            if ((FLASH->SR & (uint32_t) 0xEF) != (uint32_t) 0x00) {
                flashstatus = FLASH_ERROR_PROGRAM;
            } else {
                if ((FLASH->SR & FLASH_FLAG_OPERR) != (uint32_t) 0x00) {
                    flashstatus = FLASH_ERROR_OPERATION;
                } else {
                    flashstatus = FLASH_COMPLETE;
                }
            }
        }
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
 * @brief  Erases a specified FLASH Sector.
 *
 * @param  FLASH_Sector: The Sector number to be erased.
 *          This parameter can be a value between FLASH_Sector_0 and FLASH_Sector_11
 *
 * @param  VoltageRange: The device voltage range which defines the erase parallelism.
 *          This parameter can be one of the following values:
 *            @arg VoltageRange_1: when the device voltage range is 1.8V to 2.1V,
 *                                  the operation will be done by byte (8-bit)
 *            @arg VoltageRange_2: when the device voltage range is 2.1V to 2.7V,
 *                                  the operation will be done by half word (16-bit)
 *            @arg VoltageRange_3: when the device voltage range is 2.7V to 3.6V,
 *                                  the operation will be done by word (32-bit)
 *            @arg VoltageRange_4: when the device voltage range is 2.7V to 3.6V + External Vpp,
 *                                  the operation will be done by double word (64-bit)
 *
 * @retval FLASH Status: The returned value can be: FLASH_BUSY, FLASH_ERROR_PROGRAM,
 *                       FLASH_ERROR_WRP, FLASH_ERROR_OPERATION or FLASH_COMPLETE.
 */
static FLASH_Status FLASH_EraseSector(uint32_t FLASH_Sector,
        uint8_t VoltageRange) {
    uint32_t tmp_psize = 0x0;
    FLASH_Status status = FLASH_COMPLETE;

    if (VoltageRange == VoltageRange_1) {
        tmp_psize = FLASH_PSIZE_BYTE;
    } else if (VoltageRange == VoltageRange_2) {
        tmp_psize = FLASH_PSIZE_HALF_WORD;
    } else if (VoltageRange == VoltageRange_3) {
        tmp_psize = FLASH_PSIZE_WORD;
    } else {
        tmp_psize = FLASH_PSIZE_DOUBLE_WORD;
    }
    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastOperation();

    if (status == FLASH_COMPLETE) {
        FLASH_Unlock();
        /* if the previous operation is completed, proceed to erase the sector */
        FLASH->CR &= CR_PSIZE_MASK;
        FLASH->CR |= tmp_psize;
        FLASH->CR &= SECTOR_MASK;
        FLASH->CR |= FLASH_CR_SER | FLASH_Sector;
        FLASH->CR |= FLASH_CR_STRT;

        /* Wait for last operation to be completed */
        status = FLASH_WaitForLastOperation();

        /* if the erase operation is completed, disable the SER Bit */
        FLASH->CR &= (~FLASH_CR_SER);
        FLASH->CR &= SECTOR_MASK;
        FLASH_Lock();
    }
    /* Return the Erase Status */
    return status;
}

/**
 * @brief  Programs a half word (16-bit) at a specified address.
 * @note   This function must be used when the device voltage range is from 2.1V to 3.6V.
 * @param  Address: specifies the address to be programmed.
 *         This parameter can be any address in Program memory zone or in OTP zone.
 * @param  Data: specifies the data to be programmed.
 * @retval FLASH Status: The returned value can be: FLASH_BUSY, FLASH_ERROR_PROGRAM,
 *                       FLASH_ERROR_WRP, FLASH_ERROR_OPERATION or FLASH_COMPLETE.
 */
FLASH_Status FLASH_ProgramHalfWord(uint32_t Address, uint16_t Data) {
    FLASH_Status status = FLASH_COMPLETE;

    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastOperation();

    if (status == FLASH_ERROR_PROGRAM) {
        /* try to clear the flags */
        if ((FLASH->SR & FLASH_FLAG_PGSERR) != 0x00) {
            FLASH->SR |= FLASH_FLAG_PGSERR;
        }
        if ((FLASH->SR & FLASH_FLAG_PGPERR) != 0x00) {
            FLASH->SR |= FLASH_FLAG_PGPERR;
        }
        status = FLASH_WaitForLastOperation();
    }

    if (status == FLASH_COMPLETE) {
        FLASH_Unlock();
        /* if the previous operation is completed, proceed to program the new data */
        FLASH->CR &= CR_PSIZE_MASK;
        FLASH->CR |= FLASH_PSIZE_HALF_WORD;
        FLASH->CR |= FLASH_CR_PG;

        *(__IO uint16_t*) Address = Data;

        /* Wait for last operation to be completed */
        status = FLASH_WaitForLastOperation();

        /* if the program operation is completed, disable the PG Bit */
        FLASH->CR &= (~FLASH_CR_PG);
        FLASH_Lock();
    }
    /* Return the Program Status */
    return status;
}

/**
 * @brief  Erases PAGE and PAGE1 and writes VALID_PAGE header to PAGE
 * @param  None
 * @retval Status of the last operation (Flash write or erase) done during
 *         EEPROM formating
 */
static FLASH_Status EE_Format(void) {
    FLASH_Status FlashStatus = FLASH_COMPLETE;

    /* Erase Page0 */
    FlashStatus = FLASH_EraseSector(PAGE0_ID, VOLTAGE_RANGE);

    /* If erase operation was failed, a Flash error code is returned */
    if (FlashStatus != FLASH_COMPLETE) {
        return FlashStatus;
    }

    /* Set Page0 as valid page: Write VALID_PAGE at Page0 base address */
    FlashStatus = FLASH_ProgramHalfWord(PAGE0_BASE_ADDRESS, VALID_PAGE);

    /* If program operation was failed, a Flash error code is returned */
    if (FlashStatus != FLASH_COMPLETE) {
        return FlashStatus;
    }

    /* Erase Page1 */
    FlashStatus = FLASH_EraseSector(PAGE1_ID, VOLTAGE_RANGE);

    /* Return Page1 erase operation status */
    return FlashStatus;
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
    uint16_t PageStatus0 = 6, PageStatus1 = 6; // some random invalid status

    /* Get Page0 actual status */
    PageStatus0 = (*(__IO uint16_t*) PAGE0_BASE_ADDRESS);

    /* Get Page1 actual status */
    PageStatus1 = (*(__IO uint16_t*) PAGE1_BASE_ADDRESS);

    /* Write or read operation */
    switch (Operation) {
    case WRITE_IN_VALID_PAGE: /* ---- Write operation ---- */
        if (PageStatus1 == VALID_PAGE) {
            /* Page0 receiving data */
            if (PageStatus0 == RECEIVE_DATA) {
                return PAGE0; /* Page0 valid */
            } else {
                return PAGE1; /* Page1 valid */
            }
        } else if (PageStatus0 == VALID_PAGE) {
            /* Page1 receiving data */
            if (PageStatus1 == RECEIVE_DATA) {
                return PAGE1; /* Page1 valid */
            } else {
                return PAGE0; /* Page0 valid */
            }
        } else {
            return NO_VALID_PAGE; /* No valid Page */
        }

    case READ_FROM_VALID_PAGE: /* ---- Read operation ---- */
        if (PageStatus0 == VALID_PAGE) {
            return PAGE0; /* Page0 valid */
        } else if (PageStatus1 == VALID_PAGE) {
            return PAGE1; /* Page1 valid */
        } else {
            return NO_VALID_PAGE; /* No valid Page */
        }

    default:
        return PAGE0; /* Page0 valid */
    }
}

/**
 * @brief  Verify if active page is full and Writes variable in EEPROM.
 * @param  VirtAddress: 16 bit virtual address of the variable
 * @param  Data: 16 bit data to be written as variable value
 * @retval Success or error status:
 *           - FLASH_COMPLETE: on success
 *           - PAGE_FULL: if valid page is full
 *           - NO_VALID_PAGE: if no valid page was found
 *           - Flash error code: on write Flash error
 */
static uint16_t EE_VerifyPageFullWriteVariable(uint16_t VirtAddress,
        uint16_t Data) {
    FLASH_Status FlashStatus = FLASH_COMPLETE;
    uint16_t ValidPage = PAGE0;
    uint32_t Address = EEPROM_START_ADDRESS, PageEndAddress =
            EEPROM_START_ADDRESS + PAGE_SIZE;

    /* Get valid Page for write operation */
    ValidPage = EE_FindValidPage(WRITE_IN_VALID_PAGE);

    /* Check if there is no valid page */
    if (ValidPage == NO_VALID_PAGE) {
        return NO_VALID_PAGE;
    }

    /* Get the valid Page start Address */
    Address = (uint32_t) (EEPROM_START_ADDRESS
            + (uint32_t) (ValidPage * PAGE_SIZE ));

    /* Get the valid Page end Address */
    PageEndAddress = (uint32_t) ((EEPROM_START_ADDRESS - 2)
            + (uint32_t) ((1 + ValidPage) * PAGE_SIZE ));

    /* Check each active page address starting from begining */
    while (Address < PageEndAddress) {
        /* Verify if Address and Address+2 contents are 0xFFFFFFFF */
        if ((*(__IO uint32_t*) Address) == 0xFFFFFFFF) {
            /* Set variable data */
            FlashStatus = FLASH_ProgramHalfWord(Address, Data);
            /* If program operation was failed, a Flash error code is returned */
            if (FlashStatus != FLASH_COMPLETE) {
                return FlashStatus;
            }
            /* Set variable virtual address */
            FlashStatus = FLASH_ProgramHalfWord(Address + 2, VirtAddress);
            /* Return program operation status */
            return FlashStatus;
        } else {
            /* Next address location */
            Address = Address + 4;
        }
    }

    /* Return PAGE_FULL in case the valid page is full */
    return PAGE_FULL;
}

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
static uint16_t EE_PageTransfer(uint16_t VirtAddress, uint16_t Data) {
    FLASH_Status FlashStatus = FLASH_COMPLETE;
    uint32_t NewPageAddress = EEPROM_START_ADDRESS;
    uint16_t OldPageId = 0;
    uint16_t ValidPage = PAGE0, VarIdx = 0;
    uint16_t EepromStatus = 0, ReadStatus = 0;

    /* Get active Page for read operation */
    ValidPage = EE_FindValidPage(READ_FROM_VALID_PAGE);

    if (ValidPage == PAGE1) /* Page1 valid */
    {
        /* New page address where variable will be moved to */
        NewPageAddress = PAGE0_BASE_ADDRESS;

        /* Old page ID where variable will be taken from */
        OldPageId = PAGE1_ID;
    } else if (ValidPage == PAGE0) /* Page0 valid */
    {
        /* New page address  where variable will be moved to */
        NewPageAddress = PAGE1_BASE_ADDRESS;

        /* Old page ID where variable will be taken from */
        OldPageId = PAGE0_ID;
    } else {
        return NO_VALID_PAGE; /* No valid Page */
    }

    /* Set the new Page status to RECEIVE_DATA status */
    FlashStatus = FLASH_ProgramHalfWord(NewPageAddress, RECEIVE_DATA);
    /* If program operation was failed, a Flash error code is returned */
    if (FlashStatus != FLASH_COMPLETE) {
        return FlashStatus;
    }

    /* Write the variable passed as parameter in the new active page */
    EepromStatus = EE_VerifyPageFullWriteVariable(VirtAddress, Data);
    /* If program operation was failed, a Flash error code is returned */
    if (EepromStatus != FLASH_COMPLETE) {
        return EepromStatus;
    }

    /* Transfer process: transfer variables from old to the new active page */
    for (VarIdx = 0; VarIdx < (TOTAL_EE_VARS*2); VarIdx++) {
        if (EE_VirtAddVarTab[VarIdx] != VirtAddress) /* Check each variable except the one passed as parameter */
        {
            /* Read the other last variable updates */
            ReadStatus = EE_ReadVariable(EE_VirtAddVarTab[VarIdx], &DataVar);
            /* In case variable corresponding to the virtual address was found */
            if (ReadStatus != 0x1) {
                /* Transfer the variable to the new active page */
                EepromStatus = EE_VerifyPageFullWriteVariable(
                        EE_VirtAddVarTab[VarIdx], DataVar);
                /* If program operation was failed, a Flash error code is returned */
                if (EepromStatus != FLASH_COMPLETE) {
                    return EepromStatus;
                }
            }
        }
    }

    /* Erase the old Page: Set old Page status to ERASED status */
    FlashStatus = FLASH_EraseSector(OldPageId, VOLTAGE_RANGE);
    /* If erase operation was failed, a Flash error code is returned */
    if (FlashStatus != FLASH_COMPLETE) {
        return FlashStatus;
    }

    /* Set new Page status to VALID_PAGE status */
    FlashStatus = FLASH_ProgramHalfWord(NewPageAddress, VALID_PAGE);
    /* If program operation was failed, a Flash error code is returned */
    if (FlashStatus != FLASH_COMPLETE) {
        return FlashStatus;
    }

    /* Return last operation flash status */
    return FlashStatus;
}

/**
 * @}
 */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
