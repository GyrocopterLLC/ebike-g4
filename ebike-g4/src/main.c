/******************************************************************************
 * Filename: main.c
 * Description: Main program code starts here. Hardware is initialized, and
 *              program flow goes into an infinite loop.
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
#include <math.h>
#include <string.h>
#include <stdio.h>

#define DBG_USB_BUF_LEN 128

uint16_t VirtAddVarTab[TOTAL_EE_VARS];
uint32_t DBG_Flags=0;
uint8_t DBG_Usb_Buffer[DBG_USB_BUF_LEN];

static void MAIN_InitializeClocks(void);
static void MAIN_CheckBootloader(void);
static void MAIN_GoToBootloader(void);

int main (
        __attribute__((unused)) int argc,
        __attribute__((unused)) char* argv[])
{

    uint32_t led_timer = 0;
    float angle, sin, cos;
    float fet_temp;
    uint32_t vcp_bytes;

    // First check if we need to change to the bootloader.
    //
    // The bootloader can be selected to startup instead of normal code
    // by a variety of options (BOOT0 pin, options bits, etc), but here
    // those pins are used for other functions. The bootloader can be
    // entered via an interface command, like over USB or UART. The
    // MCU saves a key into standby sram and reboots. After reboot we
    // check if that key was saved and if so enter the bootloader.
    // This is done first so no peripheral settings will interfere with
    // bootloader operation
    MAIN_CheckBootloader();

    // Initialize clock resources, including power regulators and Flash latency.
    MAIN_InitializeClocks();

    // Set the NVIC priority grouping to the maximum number of preemption levels
    // Setting of PRIGROUP = b011 (0x03) means that the upper 4 bits are group
    // priority, lower 4 bits are sub-priority. But the ST implementation of the
    // Cortex M4 only has the upper 4 bits. So this sets 4 bits (16 levels) of
    // preemption, and no sub-priority levels.
    NVIC_SetPriorityGrouping(0x03u);

    // Start the systick timer for a simple delay timer
    DelayInit();

    // Initialize peripherals
    ADC_Init();
    CORDIC_Init();
    CRC_Init();
    DRV8353_Init();
    PWM_Init(DFLT_FOC_PWM_FREQ);
    UART_Init();
    USB_Init();
    // Start up the EEPROM emulation, and fetch stored values from it
    EE_Config_Addr_Table(VirtAddVarTab);
    EE_Init(VirtAddVarTab);

    // Enable the USB CRC class
    USB_SetClass(&USB_CDC_ClassDesc, &USB_CDC_ClassCallbacks);
    USB_Start();

    // LED init
    GPIO_Clk(LED_PORT);
    GPIO_Output(LED_PORT, GLED_PIN);
    GPIO_Output(LED_PORT, RLED_PIN);
    GPIO_Output(DRV_EN_PORT, DRV_EN_PIN);
    GPIO_High(LED_PORT, GLED_PIN);
    GPIO_Low(LED_PORT, RLED_PIN);
//    GPIO_Low(DRV_EN_PORT, DRV_EN_PIN);

    // Use the DAC for any old analog value
    RCC->AHB2ENR |= RCC_AHB2ENR_DAC1EN;
    DAC1->CR |= DAC_CR_EN1 | DAC_CR_EN2;
    DAC1->DHR8R1 = 0x40; // About 0.825 V
    DAC1->DHR8R2 = 0x80; // About 1.65 V

    // Start the watchdog
    WDT_Init();
    angle = 0.0f;
    // Infinite loop, never return.
    while (1)
    {
        WDT_Feed();

        // Loop back any received USB characters
        vcp_bytes = VCP_InWaiting();
        if(vcp_bytes > 0) {
            if(vcp_bytes > DBG_USB_BUF_LEN) {
                VCP_Read(DBG_Usb_Buffer, DBG_USB_BUF_LEN);
            } else {
                VCP_Read(DBG_Usb_Buffer, vcp_bytes);
            }



            if(strstr(DBG_Usb_Buffer,"ftemp?") != 0) {
                // Convert and send Fet temp
                fet_temp = ADC_GetFetTempDegC();
                // printf function library doesn't have float support. Doing it manually with 3 decimals
                sprintf(DBG_Usb_Buffer,"T=%d.%03d\n",(int16_t)fet_temp,(int16_t)(1000.0f*fabsf(fet_temp-truncf(fet_temp))));
                VCP_Write(DBG_Usb_Buffer, strlen(DBG_Usb_Buffer));
            } else if(strstr(DBG_Usb_Buffer, "boot") != 0) {
                // Disable USB
                USB->CNTR |= USB_CNTR_FRES; // Force a reset
                MAIN_GoToBootloader();
            } else {
                // Send the same thing back instead
                VCP_Write(DBG_Usb_Buffer, vcp_bytes);
            }
        }

        Delay(10);
        led_timer++;
        if(led_timer == 50) {
            GPIO_Low(LED_PORT, GLED_PIN);
        }
        if(led_timer >= 100) {
            GPIO_High(LED_PORT, GLED_PIN);
            led_timer = 0;
        }

        // CORDIC testing
        angle += 0.076f; // Something that doesn't fit cleanly into 1.0 so it gives lots of different values
        if(angle >= 1.0f) angle -= 2.0f; // Wrap between -1 and 1
        CORDIC_CalcSinCos(angle, &sin, &cos);

        // ADC conversions
        ADC_InjSeqComplete();
        ADC_RegSeqComplete();

    }
}


/**
 * @brief  Applies clock settings, voltage scaling, Flash latency, etc.
 *
 *         Clock settings are:
 *          - Voltage range 1 with booster (enables 170MHz, without
 *                  booster is 150MHz max, and range 2 is 26MHz max)
 *          - Main clocks (Sysclock, Hclk, Pclk1, Pclk2) at 170MHz
 *          - PLLP clock at 42.5MHz. Used for ADC.
 *          - USB clock sourced from internal 48MHz oscillator, USB SOF
 *                  used to calibrate this clock.
 *          -
 *
 * @retval None
 */
static void MAIN_InitializeClocks(void) {
    uint32_t tempreg;

    RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN; // Enable access to power control
    PWR->CR3 |= PWR_CR3_UCPD_DBDIS; // Disable USB-PD dead battery pull-downs
    // If already in range 1, don't change it
    if((PWR->CR1 & PWR_CR1_VOS) != PWR_CR1_VOS_0) { // if VOS != b01, not in Range 1
        tempreg = PWR->CR1;
        tempreg &= ~(PWR_CR1_VOS);
        tempreg |= PWR_CR1_VOS_0;
        PWR->CR1 = tempreg;
        // Make sure it takes effect by waiting for checking VOSF
        while((PWR->SR2 & PWR_SR2_VOSF) != 0) { } // wait for the bit
    }
    // Change Flash latency to the required amount
    RCC->AHB1ENR |= RCC_AHB1ENR_FLASHEN;
    // Instruction and data caches enabled, prefetch enabled, 8 wait states
    FLASH->ACR &= ~(FLASH_ACR_LATENCY);
    FLASH->ACR |= FLASH_ACR_DCEN | FLASH_ACR_ICEN | FLASH_ACR_PRFTEN | FLASH_ACR_LATENCY_8WS;
    // Make sure it has taken effect by re-reading the register
    while((FLASH->ACR & FLASH_ACR_LATENCY) != FLASH_ACR_LATENCY_8WS) { } // wait for latency to take effect

    // Check if we need to change the boost mode.
    // When switching to boost mode, the AHB prescaler must be divide by 2.
    // After 1us, it can be reconfigured to the needed prescaler.
    if((PWR->CR5 & PWR_CR5_R1MODE) != 0) { // zero is enabled, one is disabled. go figure.
        // Set the AHB prescaler to divide by 2
        tempreg = RCC->CFGR;
        tempreg &= ~(RCC_CFGR_HPRE);
        tempreg |= RCC_CFGR_HPRE_3; // HPRE = b1000, sysclock divided by 2
        RCC->CFGR = tempreg;

        // Clear R1MODE to enable the Range 1 boost voltage
        PWR->CR5 &= ~(PWR_CR5_R1MODE);
    }

    // Configure the PLL to 170MHz
    RCC->CR &= ~(RCC_CR_PLLON); // turn off PLL first
    while((RCC->CR & RCC_CR_PLLRDY) != 0) { }// wait for PLLRDY to clear
    RCC->PLLCFGR =  RCC_PLLCFGR_PLLSRC_1 | // HSI16 is the PLL source
                    (3u << RCC_PLLCFGR_PLLM_Pos) | // M divider is /4
                    (85u << RCC_PLLCFGR_PLLN_Pos) | // N multiplier is x85
                    RCC_PLLCFGR_PLLREN | // PLLR output enabled
                    RCC_PLLCFGR_PLLPEN | // PLLP output enabled
                    (8u << RCC_PLLCFGR_PLLPDIV_Pos); // PLLP divider is /8
    // Not shown: PLLR divider is /2, PLLQ divider is /2 (but PLLQ output is off)
    // Enable PLL
    RCC->CR |= RCC_CR_PLLON;
    while((RCC->CR & RCC_CR_PLLRDY) == 0) { }// wait for PLLRDY to set
    // Switch clock sources
    RCC->CFGR |= (RCC_CFGR_SW); // Set SW[1:0] to b11 to use PLL as system clock
    while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS) { } // wait for the switch to take effect

    // Set prescalers. HCLK, APB1, and APB2 are all divide by 1 (170MHz in, 170MHz out)
    RCC->CFGR &= ~(RCC_CFGR_PPRE2 | RCC_CFGR_PPRE1 | RCC_CFGR_HPRE);

    // Enable HSI48 for the USB
    RCC->CRRCR |= RCC_CRRCR_HSI48ON;

    // Set output clocks to the peripherals that have a selection available
    RCC->CCIPR = RCC_CCIPR_ADC12SEL_0 | RCC_CCIPR_ADC345SEL_0; // ADC12 and ADC345 sourced from PLL"P" clock

    SystemCoreClockUpdate(); // Sets some internal variables to the actual clock settings.
}

/**
 * @brief  Determines whether or not to enter bootloader at startup.
 *
 *         The bootloader can be selected to startup instead of normal
 *         code by a variety of options (BOOT0 pin, options bits, etc),
 *         but here those pins are used for other functions. The
 *         bootloader can be entered via an interface command, like over
 *         USB or UART. The MCU saves a key into standby sram and reboots.
 *         After reboot we check if that key was saved and if so enter
 *         the bootloader. This is done first so no peripheral settings
 *         will interfere with bootloader operation.
 *
 * @retval None
 */
static void MAIN_CheckBootloader(void) {
    // Set the bootloader entry point as a function pointer. The value is stored at
    // the reset vector location in system memory.
    void (*bootloader)(void);

    // The backup registers are not modified by a software reset, so if application code
    // changes this register and resets the processor, we will know that a bootloader
    // reset is required.
    if(TAMP->BKP0R == BOOTLOADER_RESET_FLAG)
    {
        // Need to reset this register first. Enable access to backup domain...
        RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN; // Turn on clocking to the power control
        PWR->CR1 |= PWR_CR1_DBP; // Enable access to backup domain
        // Reset the backup register so we don't get stuck forever restarting in bootloader mode
        TAMP->BKP0R = 0;
        // Remap the system memory to address 0x00000000
        SYSCFG->MEMRMP &= ~(SYSCFG_MEMRMP_MEM_MODE);
        SYSCFG->MEMRMP |= SYSCFG_MEMRMP_MEM_MODE_0; // MEM_MODE = 001b, System Flash memory mapped to 0x00000000
        // And now, load the bootloader
        __set_MSP(*((uint32_t *)(BOOTLOADER_REMAPPED_TOP_OF_STACK))); // Change stack pointer to the end of ram.
        bootloader = (void(*)(void)) *((uint32_t *)(BOOTLOADER_REMAPPED_RESET_VECTOR)); // Set bootloader entry point
        bootloader();// Call the bootloader.
    }
}


static void MAIN_GoToBootloader(void) {
    // Enable access to backup registers
    RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN; // Enable power control
    PWR->CR1 |= PWR_CR1_DBP; // Enable access to backup domain

    TAMP->BKP0R = BOOTLOADER_RESET_FLAG;

    // Reboot
    NVIC_SystemReset();

}
