/*
 * Memory Spaces Definitions.
 *
 * Need modifying for a specific board. 
 *   FLASH.ORIGIN: starting address of flash
 *   FLASH.LENGTH: length of flash
 *   RAM.ORIGIN: starting address of RAM bank 0
 *   RAM.LENGTH: length of RAM bank 0
 *
 * The values below can be addressed in further linker scripts
 * using functions like 'ORIGIN(RAM)' or 'LENGTH(RAM)'.
 */

/* For eeprom use, Sector 0 and Sector 1 (16K each) are
 * general purpse, Sector 2 and 3 (16K each) are for the
 * eeprom emulation, and Sectors 4 (64K) and 5-11 (128K each)
 * are general purpose use again 
 */

MEMORY
{
  RAM (xrw) : ORIGIN = 0x20000000, LENGTH = 128K
  CCMRAM (xrw) : ORIGIN = 0x10000000, LENGTH = 64K
  FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 32K
  EEPROM (rx) : ORIGIN = 0x08008000, LENGTH = 32K
  FLASH2 (rx) : ORIGIN = 0x08010000, LENGTH = 960K
  FLASHB1 (rx) : ORIGIN = 0x00000000, LENGTH = 0
  EXTMEMB0 (rx) : ORIGIN = 0x00000000, LENGTH = 0
  EXTMEMB1 (rx) : ORIGIN = 0x00000000, LENGTH = 0
  EXTMEMB2 (rx) : ORIGIN = 0x00000000, LENGTH = 0
  EXTMEMB3 (rx) : ORIGIN = 0x00000000, LENGTH = 0
  /* MEMORY_ARRAY (xrw)  : ORIGIN = 0x20002000, LENGTH = 32 */
}

/*
 * For external ram use something like:

   RAM (xrw) : ORIGIN = 0x64000000, LENGTH = 2048K

 */
