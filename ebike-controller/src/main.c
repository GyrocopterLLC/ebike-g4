/******************************************************************************
 * Filename: main.c
 * Description: Main program code starts here. Hardware is initialized, and
 *              program flow goes into an infinite loop.
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
#include "main.h"

/* Private typedef -----------------------------------------------------------*/
typedef struct _HallDetectHelperStruct {
    uint8_t transitions_seen;
    uint32_t ticks_now;
    uint8_t startup_success;
    uint8_t old_state;
} HallDetectHelperStruct;
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
__IO uint32_t g_MainSysTick;

float g_rampAngle;
float g_rampInc;
uint32_t g_ledcount;

uint32_t g_errorCode;

uint32_t g_MainFlags;

uint32_t cur_a_sum;
uint32_t cur_b_sum;
uint32_t cur_c_sum;
uint32_t cur_sum_count;

Motor_Controls Mctrl;
Motor_Observations Mobv;
Motor_PWMDuties Mpwm;
FOC_StateVariables Mfoc;

float g_FetTemp;
float g_MotorTemp;

PowerCalcs Mpc;

uint16_t VirtAddVarTab[(TOTAL_EE_VARS * 2)];
float g_hallDetectTable[6 * HALL_DETECT_TRANSITIONS_TO_AVG];

HallDetectHelperStruct hdhs;

/** Debugging outputs **
 *
 * 0 - None (outputs zero)
 * 1 - Ia
 * 2 - Ib
 * 3 - Ic
 * 4 - Ta
 * 5 - Tb
 * 6 - Tc
 * 7 - Throttle
 * 8 - RampAngle
 * 9 - HallAngle
 * 10 - HallSpeed
 * 11 - Vbus
 * 12 - Id
 * 13 - Iq
 * 14 - Td
 * 15 - Tq
 * 16 - ErrorCode
 * 17 - Vrefint
 * 18 - HallState
 * 19 - HallSensor2 Angle (if enabled)
 *
 *
 * These debugging outputs can be monitored over the USB debug using the
 * "USB" series of commands, or by the data dump (which records at the full
 * 20kHz rate) using the "DUMP" command series.
 *
 */

Config_Main config_main;

uint32_t usb_speed_choices[MAX_USB_SPEED_CHOICES] = USB_SPEED_RELOAD_VALS;
uint8_t usb_debug_buffer[PACKET_MAX_LENGTH];

Data_Packet_Type usb_debug_packet;
uint8_t usb_debug_data_buffer[PACKET_MAX_DATA_LENGTH];
__IO uint32_t usb_debug_buffer_pos = 0;
__IO uint32_t usb_debug_countdown_timer;
uint32_t usb_debug_countdown_reload;

float usbdacvals[MAX_USB_VALS];

//char vcp_buffer[PACKET_MAX_LENGTH];
//char uart_buffer[PACKET_MAX_LENGTH];

uint32_t systick_debounce_counter;
uint8_t debounce_integrator;
volatile PB_TypeDef pb_state;

//extern HallSensor_HandleTypeDef HallSensor;

PID_Float_Type Id_control,
Iq_control;

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void Toggle_Leds(void);
static void User_PB_Init(void);
static void User_LED_Init(void);
static void User_DAC_Init(void);
static void User_BasicTim_Init(void);
static void RunHallDetectRoutine(void);
static void VCP_SendWrapper(char* buf, uint32_t len);
static void HBD_SendWrapper(char* buf, uint32_t len);

/* Private functions ---------------------------------------------------------*/

/**
 * Enable backup domain write access.
 */
static void BackupEnable(void) {
    // First, enable the clock to the power controller
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    // Next, set the write protection disable bit in the power controller
    PWR->CR |= PWR_CR_DBP;
    // Finally, write the two keys (found in the chip's user manual) to enable
    // write access to the backup domain
    RTC->WPR = 0xCA;
    RTC->WPR = 0x53;
}
/**
 * Startup bootloader check.
 * Loads bootloader if...
 * ** User pushbutton is held down at startup, or
 * ** A particular memory location has a bootloader reset flag enabled
 */

static void BootloaderStartup(void) {
    void (*bootloader)(void) = (void(*)(void)) *((uint32_t *)(0x1FFF0004));
    // Breaking down that previous line...
    // Before the equals sign: declare a pointer to a function with no input arguments and no return value.
    // After the equals sign: the (void(*)(void)) part casts the following value to a function pointer type
    // the (uint32_t *) casts the following value (0x1FFF0004) to a unsigned integer pointer type
    // the whole *((uint32_t *)(#0x1FFF0004#)) portion returns the value pointed at by the address stored
    // at the 0x1FFF0004 location
    // This allows the bootloader function pointer to take the address stored at location 0x1FFF0004 as its own
    // address.

    uint8_t bootByte;
    uint8_t go_to_boot = 0;

    User_PB_Init();// Enable pushbutton so we can check if it's pressed
    if((PB_PORT->IDR & (1<<PB_PIN)) == 0)
    {
        // Make sure it really is pressed. Check 100x in a row.
        bootByte = 100;
        while(bootByte > 0)
        {
            bootByte--;
            //if(HAL_GPIO_ReadPin(PB_PORT, PB_PIN) == GPIO_PIN_SET)
            if((PB_PORT->IDR & (1<<PB_PIN)) == (1<<PB_PIN))
            {
                break;
            }
        }
        if(bootByte == 0)
        {
            go_to_boot = 1;

        }
    }

    // Other way into the bootloader is through a stored memory location.
    // The backup registers are not modified by a software reset, so if application code
    // changes this register and resets the processor, we will know that a bootloader
    // reset is required.
    if(RTC->BKP0R == BOOTLOADER_RESET_FLAG)
    {
        // Need to reset this register first. Enable access to backup domain...
        BackupEnable();
        // Change the backup register so we don't get stuck forever restarting in bootloader mode
        RTC->BKP0R = 0;
        // And now, load the bootloader
        go_to_boot = 1;
    }
    if(go_to_boot != 0)
    {
        __set_MSP(0x2001FFFF); // Change stack pointer to the end of ram.
        bootloader();// Call the bootloader.
    }
}
/**
 * @brief  Main program
 * @param  None
 * @retval None
 */
int main(void) {
#ifdef DEBUG_USED
    char string[32];
    char usbstring[64];
#endif // DEBUG_USED

    BootloaderStartup(); // Load bootloader if certain conditions are met
    // Also initializes the user pushbutton

    // Disable (turn on gpio pulldown resistor) unused pins
    GPIO_Pulldown_Unused();

    // Configure the system clock to 168 MHz
    SystemClock_Config();

    // Start up the EEPROM emulation, and fetch stored values from it
    EE_Config_Addr_Table(VirtAddVarTab);
    EE_Init(VirtAddVarTab);

    // Load all variables from EEPROM
    MAIN_LoadVariables();

    /* Default initialization:
     ** Configure the Flash prefetch, instruction and Data caches
     ** Configure the Systick to generate an interrupt each 1 msec
     ** Set NVIC Group Priority to 4
     ** Start systick timer with 1ms interval
     */
    FLASH->ACR |= FLASH_ACR_ICEN;
    FLASH->ACR |= FLASH_ACR_DCEN;

    // STM32F405x/407x/415x/417x Revision Z devices: prefetch is supported
    if (((DBGMCU->IDCODE) >> 16) == 0x1001) {
        // Enable the Flash prefetch
        FLASH->ACR |= FLASH_ACR_PRFTEN;
    }

    NVIC_SetPriorityGrouping(0); // Maximum number of priority bits (4 for this MCU), no sub-priority bits

    SysTick_Config(SystemCoreClock / 1000);
    NVIC_SetPriority(SysTick_IRQn, PRIO_SYSTICK);

    g_errorCode = 0;

    // Configure green and red leds
    User_LED_Init();
    // Turn on the green led
    GLED_PORT->ODR |= (1 << GLED_PIN);
    adcInit();
    throttle_init();
    User_DAC_Init();
    User_BasicTim_Init();
    PWM_Init(config_main.PWMFrequency);
    HallSensor_Init_NoHal(config_main.PWMFrequency);
    PWM_SetDeadTime(config_main.PWMDeadTime);
    UART_Init();

    // USB init
    USB_Init();
    USB_SetClass(&USB_CDC_ClassDesc, &USB_CDC_ClassCallbacks);
    USB_Start();


#ifdef DEBUG_DUMP_USED
    DumpReset();
#endif // DEBUG_DUMP_USED

    // Init the motor controller
    Mctrl.state = Motor_Off;
    Mfoc.Id_PID = &Id_control;
    Mfoc.Iq_PID = &Iq_control;

    /* Initialize watchdog timer */
    WDT_init();

    /* Start communications processor */
    CRC32_Init();
    USB_Data_Comm_Init();

    /* Enable FOC mode */
    config_main.ControlMethod = Control_FOC;

    /* Start connecting to the BMS boards */
    BMS_Data_Comm_Init();
    BMS_Restart_Chain();

    /* Start connection with handle bar display */
    HBD_Data_Comm_Init();


    /* Run Application (Interrupt mode) */
    while (1) {
        // Feed the watchdog!
        WDT_feed();

        // Check USB serial for data
        USB_Data_Comm_OneByte_Check();
        // Check BMS serial for data
        BMS_OneByte_Check();
        // Check HBD serial for data
        HBD_OneByte_Check();


        if (pb_state == PB_PRESSED) {
            // Wait until release
            while (pb_state == PB_PRESSED) {
            }
            // Change data output state
            g_MainFlags ^= MAINFLAG_SERIALDATAON;
        }

        if(g_MainFlags & MAINFLAG_CHECKBMS) {
            g_MainFlags &= ~MAINFLAG_CHECKBMS;
            if((!BMS_Busy()) && BMS_Is_Connected()) {
                if((g_errorCode & MAIN_FAULT_BMS_COMM) == 0) {
                    BMS_Refresh_Data();
                }
            }
        }

        if (g_MainFlags & MAINFLAG_CONVERTTEMPERATURE) {
            g_MainFlags &= ~MAINFLAG_CONVERTTEMPERATURE;
            g_FetTemp = adcGetTempDegC();
            g_MotorTemp = 25.0f;
        }
        if (g_MainFlags & MAINFLAG_SERIALDATAON) {
            if (usb_debug_buffer_pos > 0) {
                if (VCP_Write(usb_debug_buffer, usb_debug_buffer_pos) != -1) {
                    usb_debug_buffer_pos = 0;
                }
            }
        }
        if (g_MainFlags & MAINFLAG_HALLDETECTFAIL) {
            // Create a response packet, all angles are NaN
            memset(usb_debug_data_buffer, 0xFF, 6*sizeof(float));
            usb_debug_packet.TxBuffer = usb_debug_buffer;
            if (data_packet_create(&usb_debug_packet,
                    ROUTINE_RESULT, usb_debug_data_buffer,
                    6 * sizeof(float))) {
                if(g_MainFlags & MAINFLAG_LASTCOMMSERIAL) {
                    HBD_SendWrapper((char*)usb_debug_buffer, usb_debug_packet.TxLength);
                } else {
                    VCP_SendWrapper((char*)usb_debug_buffer, usb_debug_packet.TxLength);
                }
                g_MainFlags &= ~(MAINFLAG_HALLDETECTFAIL);
            }
        }
        if (g_MainFlags & MAINFLAG_HALLDETECTPASS) {
            // Create a response packet with the six Hall angles
            for(uint8_t ii = 0; ii < 6; ii++) {
                data_packet_pack_float(&(usb_debug_data_buffer[ii*sizeof(float)]), g_hallDetectTable[ii]);
            }
            usb_debug_packet.TxBuffer = usb_debug_buffer;
            if(data_packet_create(&usb_debug_packet, ROUTINE_RESULT, usb_debug_data_buffer, 6*sizeof(float))) {
                if(g_MainFlags & MAINFLAG_LASTCOMMSERIAL) {
                    HBD_SendWrapper((char*)usb_debug_buffer, usb_debug_packet.TxLength);
                } else {
                    VCP_SendWrapper((char*)usb_debug_buffer, usb_debug_packet.TxLength);
                }
                g_MainFlags &= ~(MAINFLAG_HALLDETECTPASS);
            }
        }

        // DEBUG STUFF
        if(g_MainFlags & MAINFLAG_DEBUG_SENDBMSRESET) {
            g_MainFlags &= ~MAINFLAG_DEBUG_SENDBMSRESET;
            uint8_t mydata[4];
            mydata[0] = BROADCAST_ADDRESS;
            mydata[1] = 0x00;
            mydata[2] = 0x01;
            mydata[3] = 0;
            BMS_Send_One_Packet(SET_RAM_VARIABLE, mydata, 4);
        }
        if(g_MainFlags & MAINFLAG_DEBUG_SETBMSADDR) {
            g_MainFlags &= ~MAINFLAG_DEBUG_SETBMSADDR;
            uint8_t mydata1[4];
            mydata1[0] = 0;
            mydata1[1] = 0x00;
            mydata1[2] = 0x01;
            mydata1[3] = 1;
            BMS_Send_One_Packet(SET_RAM_VARIABLE, mydata1, 4);
        }
        if(g_MainFlags & MAINFLAG_DEBUG_ASKBMSBATTS) {
            g_MainFlags &= ~MAINFLAG_DEBUG_ASKBMSBATTS;
            uint8_t mydata2[3];
            mydata2[0] = 1;
            mydata2[1] = 0x01;
            mydata2[2] = 0x21;
            BMS_Send_One_Packet(GET_RAM_VARIABLE, mydata2, 3);
        }


        if (Mctrl.state == Motor_OpenLoop) {
            RunHallDetectRoutine();
        }

#ifdef DEBUG_DUMP_USED
        if(g_MainFlags & MAINFLAG_DUMPDATAON)
        {
            if(g_MainFlags & MAINFLAG_DUMPDATAPRINT)
            {
                g_MainFlags &= ~(MAINFLAG_DUMPDATAPRINT);
                usbstring[0] = 0;
                for(uint8_t i = 0; i < 4; i++)
                {
                    _itoa(string, (int32_t)GetDumpData(), 0);
                    strcat(usbstring,string);
                    strcat(usbstring," ");
                }
                strcat(usbstring,"\r\n");
                VCP_Write(usbstring,strlen(usbstring));
                if(DumpDone())
                {
                    DumpReset();
                    g_MainFlags &= ~(MAINFLAG_DUMPDATAON);
                }
            }
        }
#endif // DEBUG_DUMP_USED

    }
}

static void VCP_SendWrapper(char* buf, uint32_t len) {
    if (len <= CDC_DATA_FS_MAX_PACKET_SIZE) {
        while (VCP_Write(buf, len) < 0)
            ;
    } else {
        uint32_t vcp_pointer = 0;
        while (len > 0) {
            if (len > (CDC_DATA_FS_MAX_PACKET_SIZE - 4)) {
                while (VCP_Write(&(buf[vcp_pointer]),
                        (CDC_DATA_FS_MAX_PACKET_SIZE - 4)) < 0)
                    ;
                vcp_pointer += (CDC_DATA_FS_MAX_PACKET_SIZE - 4);
                len -= (CDC_DATA_FS_MAX_PACKET_SIZE - 4);
            } else {
                while (VCP_Write(&(buf[vcp_pointer]), len) < 0)
                    ;
                len = 0;
            }
        }
    }
}

static void HBD_SendWrapper(char *buf, uint32_t len) {
    if (len <= HBD_BUFFER_LENGTH) {
        while (UART_IsFinishedTx(SELECT_HBD_UART) == 0)
            ;
        UART_Write(SELECT_HBD_UART, buf, len);
    } else {
        uint32_t hbd_pointer = 0;
        while (len > 0) {
            if (len > (HBD_BUFFER_LENGTH)) {
                while (UART_IsFinishedTx(SELECT_HBD_UART) == 0)
                    ;
               UART_Write(SELECT_HBD_UART, &(buf[hbd_pointer]),
                        HBD_BUFFER_LENGTH);
                hbd_pointer += HBD_BUFFER_LENGTH;
                len -= HBD_BUFFER_LENGTH;
            } else {
                while (UART_IsFinishedTx(SELECT_HBD_UART) == 0)
                    ;
                UART_Write(SELECT_HBD_UART, &(buf[hbd_pointer]), len);
                len = 0;
            }
        }
    }
}
/**
 * @brief  System Clock Configuration
 *         The system Clock is configured as follow :
 *            System Clock source            = PLL (HSE)
 *            SYSCLK(Hz)                     = 168000000
 *            HCLK(Hz)                       = 168000000
 *            AHB Prescaler                  = 1
 *            APB1 Prescaler                 = 4
 *            APB2 Prescaler                 = 2
 *            HSE Frequency(Hz)              = 8000000
 *            PLL_M                          = 25
 *            PLL_N                          = 336
 *            PLL_P                          = 2
 *            PLL_Q                          = 7
 *            VDD(V)                         = 3.3
 *            Main regulator output voltage  = Scale1 mode
 *            Flash Latency(WS)              = 5
 * @param  None
 * @retval None
 */
static void SystemClock_Config(void) {
    uint32_t timeout = 0;
    uint32_t tempreg, pllm, pllp, plln, pllq;
    // Enable Power Control clock
    //__PWR_CLK_ENABLE();
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;

    // The voltage scaling allows optimizing the power consumption when the
    // device is clocked below the maximum system frequency, to update the
    // voltage scaling value regarding system frequency refer to product
    // datasheet.
    PWR->CR |= PWR_CR_VOS;

    // Set system clock to HSI before changing settings all willy nilly
    tempreg = RCC->CFGR;
    tempreg &= ~(RCC_CFGR_SW);
    tempreg |= RCC_CFGR_SW_HSI;
    RCC->CFGR = tempreg;

    // Make sure it switched over
    timeout = 20000;
    do {
        timeout--;
        if (timeout == 0)
            return;
    } while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI);

    // Disable PLL
    RCC->CR &= ~(RCC_CR_PLLON);

    // Enable HSE Oscillator and activate PLL with HSE as source
    RCC->CR |= RCC_CR_HSEON; // Turn on HSE
    // Wait for HSE to turn on
    timeout = 20000;
    do {
        timeout--;
        if (timeout == 0)
            return;
    } while (!(RCC->CR & RCC_CR_HSERDY));

    // Configure PLL multipliers
    pllm = HSE_VALUE / 1000000u; // Set so the PLL input clock is 1MHz (HSE/pllm = 1MHz)
    pllp = 0; // This is actually PLL_P = 2
    pllq = 7;
    plln = 336;
    // Set PLL multipliers with HSE selected as input source
    RCC->PLLCFGR = (pllq << 24) | RCC_PLLCFGR_PLLSRC | (pllp << 16)
            | (plln << 6) | (pllm);

    // Enable PLL
    RCC->CR |= RCC_CR_PLLON;

    timeout = 20000;
    do {
        timeout--;
        if (timeout == 0)
            return;
    } while (!(RCC->CR & RCC_CR_PLLRDY)); // Wait for PLL to start

    // Set flash latency
    FLASH->ACR &= ~FLASH_ACR_LATENCY;
    FLASH->ACR |= FLASH_ACR_LATENCY_5WS;

    // Set AHB, APB prescalers
    RCC->CFGR &= ~(RCC_CFGR_PPRE2 | RCC_CFGR_PPRE1 | RCC_CFGR_HPRE);
    RCC->CFGR |= (RCC_CFGR_PPRE2_DIV2) | (RCC_CFGR_PPRE1_DIV4); // APB2 divided by 2, APB1 divided by 4

    // Select PLL as system clock
    RCC->CFGR |= RCC_CFGR_SW_PLL;

    SystemCoreClockUpdate();

}

/**
 * @brief  Toggles LEDs.
 * @param  None
 * @retval None
 */
static void Toggle_Leds(void) {
    //RLED_PORT->ODR ^= (1<<RLED_PIN);
    GLED_PORT->ODR ^= (1 << GLED_PIN);
}
/* Configure PC9 and PB12 as LED outputs */
static void User_LED_Init(void) {
    // Enable clocks to GPIOB and C
    GPIO_Clk(GLED_PORT);
    GPIO_Clk(RLED_PORT);

    GPIO_Output(GLED_PORT, GLED_PIN);
    GPIO_Output(RLED_PORT, RLED_PIN);
}

/* Configure PC12 as pushbutton input */
static void User_PB_Init(void) {
    // Start GPIOC clock
    GPIO_Clk(PB_PORT);

    // Enable pin PC12 as input with weak pullup turned on
    GPIO_Input(PB_PORT, PB_PIN);
    PB_PORT->PUPDR |= (GPIO_PUPDR_PUPDR0_0 << (PB_PIN * 2));
}

static void User_DAC_Init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_DACEN;
    GPIO_Clk(DAC_PORT);

    GPIO_Analog(DAC_PORT, DAC1_PIN);
    GPIO_Analog(DAC_PORT, DAC2_PIN);

    DAC->CR = DAC_CR_BOFF2 | DAC_CR_BOFF1 | DAC_CR_EN2 | DAC_CR_EN1;

}

static void User_BasicTim_Init(void) {
    APP_TIMER_CLK_ENABLE();

    APP_TIM->PSC = 9; // 84MHz clock, divided by 9+1 = 8.4MHz
    APP_TIM->ARR = 8400; // 8.4MHz / 8400 = 1kHz clock

    NVIC_SetPriority(APP_IRQn, PRIO_APPTIMER);
    NVIC_EnableIRQ(APP_IRQn);

    APP_TIM->DIER = TIM_DIER_UIE;
    APP_TIM->CR1 = TIM_CR1_CEN;

}

void SYSTICK_IRQHandler(void) {
    g_MainSysTick++;

    if ((g_MainSysTick % SERIAL_DUMP_RATE) == 0) {
        g_MainFlags |= MAINFLAG_DUMPDATAPRINT;
    }
    if ((g_MainSysTick % TEMP_CONVERSION_RATE) == 0) {
        g_MainFlags |= MAINFLAG_CONVERTTEMPERATURE;
    }
    if((g_MainSysTick % BMS_CHECK_RATE) == 0) {
        g_MainFlags |= MAINFLAG_CHECKBMS;
    }
}

// TIM1 overflow / update IRQ (20kHz)
void User_PWMTIM_IRQ(void) {
    // Debug: Blink RLED to show how much processor time is used
    RLED_PORT->BSRR = (1 << RLED_PIN);

    // Get observations and current states
    uint16_t tA, tB, tC;
    Mobv.iA = adcGetCurrent(ADC_IA);
    Mobv.iB = adcGetCurrent(ADC_IB);
    Mobv.iC = adcGetCurrent(ADC_IC);
    Mctrl.BusVoltage = adcGetVbus();

    HallSensor_Inc_Angle();
#ifdef TESTING_2X
    HallSensor2_Inc_Angle();
#endif
#ifdef TESTING_PLL
    HallSensorPLL_Update();
#endif
    dfsl_rampgenf(&g_rampAngle, g_rampInc);
    Mctrl.RampAngle = g_rampAngle;
#ifdef TESTING_2X
//    Mobv.RotorAngle = HallSensor2_Get_Anglef(); // Can't be trusted!
    Mobv.RotorAngle = HallSensor_Get_Anglef();
#else
#ifdef TESTING_PLL
    if(HallSensorPLL_Is_Valid() == PLL_LOCKED) {
        Mobv.RotorAngle = HallSensorPLL_Get_Anglef();
    } else {
        Mobv.RotorAngle = HallSensor_Get_Anglef();
    }
#else
    Mobv.RotorAngle = HallSensor_Get_Anglef();
#endif
#endif
    Mobv.HallState = HallSensor_Get_State();
    Motor_Loop(&Mctrl, &Mobv, &Mfoc, &Mpwm);

    // Don't allow below zero!
    // Greater than one is okay, the PWM output will be fully on
    if(Mpwm.tA < 0.0f) {
        tA = 0;
    } else {
        tA = (uint16_t) (Mpwm.tA * 65535.0f);
    }
    if(Mpwm.tB < 0.0f) {
        tB = 0;
    } else {
        tB = (uint16_t) (Mpwm.tB * 65535.0f);
    }
    if(Mpwm.tC < 0.0f) {
        tC = 0;
    } else {
        tC = (uint16_t) (Mpwm.tC * 65535.0f);
    }

    PWM_SetDuty(tA, tB, tC);

    // USB Debugging outputs
    usbdacvals[0] = Mobv.iA;
    usbdacvals[1] = Mobv.iB;
    usbdacvals[2] = Mobv.iC;
    usbdacvals[3] = Mpwm.tA;
    usbdacvals[4] = Mpwm.tB;
    usbdacvals[5] = Mpwm.tC;
    usbdacvals[6] = Mctrl.ThrottleCommand;
    usbdacvals[7] = Mctrl.RampAngle;
    usbdacvals[8] = HallSensor_Get_Anglef();
    usbdacvals[9] = HallSensor_Get_Speedf();
    usbdacvals[10] = Mctrl.BusVoltage;
    usbdacvals[11] = Mfoc.Park_D;
    usbdacvals[12] = Mfoc.Park_Q;
    usbdacvals[13] = Mfoc.Id_PID->Out;
    usbdacvals[14] = Mfoc.Iq_PID->Out;
//    usbdacvals[13] = (float) HallSensor.CaptureValue;
//    usbdacvals[14] = (float) HallSensor.Prescaler;
    usbdacvals[15] = (float) (g_errorCode);
//    usbdacvals[16] = (float) (adcRaw(ADC_VREFINT));
    usbdacvals[16] = g_FetTemp;
    usbdacvals[17] = (float) HallSensor_Get_State();
#ifdef TESTING_2X
    usbdacvals[18] = HallSensor2_Get_Anglef();
#endif // TESTING_2X
#ifdef TESTING_PLL
    usbdacvals[18] = HallSensorPLL_Get_Anglef();
#endif

    // Load up the output buffer
    if (g_MainFlags & MAINFLAG_SERIALDATAON) {
        if ((--usb_debug_countdown_timer) == 0) {
            for (uint8_t i = 0; i < config_main.Num_USB_Outputs; i++) {
                data_packet_pack_float(
                        &(usb_debug_data_buffer[i * sizeof(float)]),
                        usbdacvals[config_main.USB_Choices[i] - 1]);
            }
            usb_debug_packet.TxBuffer = usb_debug_buffer;
            if (data_packet_create(&usb_debug_packet,
            CONTROLLER_STREAM_DATA, usb_debug_data_buffer,
                    config_main.Num_USB_Outputs * sizeof(float))) {
                usb_debug_buffer_pos = usb_debug_packet.TxLength;
            } else {
                usb_debug_buffer_pos = 0;
            }

            usb_debug_countdown_timer = usb_debug_countdown_reload;
        }
    }

#ifdef DEBUG_DUMP_USED
    if(g_MainFlags & MAINFLAG_DUMPRECORD)
    {
        if(DumpGeneration(usbdacvals) == 1)
        {
            g_MainFlags &= ~(MAINFLAG_DUMPRECORD); // Stop recording
            g_MainFlags |= MAINFLAG_DUMPDATAON;// Start pushing data via USB serial port
        }
    }
#endif // DEBUG_DUMP_USED

    RLED_PORT->BSRR = (1 << (RLED_PIN + 16));

}

// Simple application timer (1kHz)
// Does all the throttle processing
// Calculates power usage
void User_BasicTIM_IRQ(void) {

    // Use a temporary holding variable. Only set the real throttle once!
    // We don't want a race condition if the PWM interrupt happens while we are
    // in the middle of this function.
    float temp_throttle_command = 0.0f;

    // Check the throttle command, but skip if in a forced state
    if ((Mctrl.state != Motor_Fault) && (Mctrl.state != Motor_OpenLoop)) {
        throttle_process(1);
        temp_throttle_command = throttle_get_command(1);
        // Trim to 99%
        if (temp_throttle_command >= 1.0f)
            temp_throttle_command = 0.99f;
    }

    // Throttle trimming due to limits being reached
    config_main.throttle_limit_scale = 1.0f;

    // Voltage limit
    if (Mctrl.BusVoltage < config_main.VoltageSoftCap) {
        if (Mctrl.BusVoltage < config_main.VoltageHardCap) {
            // Completely shut off!
            config_main.throttle_limit_scale = 0.0f;
            g_errorCode |= MAIN_FAULT_UV;
        } else {
            // Trim by scaling
            config_main.throttle_limit_scale *= (Mctrl.BusVoltage
                    - config_main.VoltageHardCap)
                    / (config_main.VoltageSoftCap - config_main.VoltageHardCap);
        }
    }
    // FET temperature limit
    if (g_FetTemp > config_main.FetTempSoftCap) {
        if (g_FetTemp > config_main.FetTempHardCap) {
            config_main.throttle_limit_scale = 0.0f;
            g_errorCode |= MAIN_FAULT_FETTEMP;
        } else {
            config_main.throttle_limit_scale *= (g_FetTemp
                    - config_main.FetTempSoftCap)
                    / (config_main.FetTempHardCap - config_main.FetTempSoftCap);
        }
    }
    // Motor temperature limit
    if (g_MotorTemp > config_main.MotorTempSoftCap) {
        if (g_MotorTemp > config_main.MotorTempHardCap) {
            config_main.throttle_limit_scale = 0.0f;

            g_errorCode |= MAIN_FAULT_MOTORTEMP;
        } else {
            config_main.throttle_limit_scale *= (g_MotorTemp
                    - config_main.MotorTempSoftCap)
                    / (config_main.MotorTempHardCap
                            - config_main.MotorTempSoftCap);
        }
    }

    // Is there a fault state from previously?
    if((g_errorCode & (MAIN_FAULT_UV|MAIN_FAULT_FETTEMP|MAIN_FAULT_MOTORTEMP))!= 0) {
        // We can reset if the throttle position is back to zero
        if(temp_throttle_command <= 0.0f) {
            g_errorCode &= ~(MAIN_FAULT_UV|MAIN_FAULT_FETTEMP|MAIN_FAULT_MOTORTEMP);
        }
        // Otherwise, keep that motor disabled
        else {
            config_main.throttle_limit_scale = 0.0f;
        }
    }

    // Apply scaling
    Mctrl.ThrottleCommand = config_main.throttle_limit_scale
            * temp_throttle_command;
    if (config_main.throttle_limit_scale <= 0.0f) {
        Mctrl.state = Motor_Off;
        PWM_MotorOFF();
    }

    // Check if we should change out of standby
    if ((Mctrl.ThrottleCommand > 0.0f) && (Mctrl.state == Motor_Off)) {
        if (config_main.ControlMethod == Control_FOC)
            Mctrl.state = Motor_Startup;
        if (config_main.ControlMethod == Control_BLDC)
            Mctrl.state = Motor_SixStep;
        if(config_main.ControlMethod == Control_Debug)
            Mctrl.state = Motor_Debug;
    }
    // Blink the LEDs
    g_ledcount++;
    if (g_ledcount > MAXLEDCOUNT) {
        Toggle_Leds();
        g_ledcount = 0;
    }
    // Debounce the user pushbutton

    systick_debounce_counter++;
    if (systick_debounce_counter >= DEBOUNCE_INTERVAL) {
        systick_debounce_counter = 0;

        // Part 1: Accumulated the debounce integrator
        //if(HAL_GPIO_ReadPin(PB_PORT, PB_PIN) == GPIO_PIN_SET)
        if ((PB_PORT->IDR & (1 << PB_PIN)) == (1 << PB_PIN)) {
            if (debounce_integrator > 0)
                debounce_integrator--;
        } else {
            if (debounce_integrator < DEBOUNCE_MAX)
                debounce_integrator++;
        }
        // Part 2: Change pushbutton state if integrator is at zero or its maximum
        if (debounce_integrator > DEBOUNCE_MAX) {
            // Defensive programming!
            debounce_integrator = DEBOUNCE_MAX;
        }
        if (debounce_integrator == DEBOUNCE_MAX) {
            pb_state = PB_PRESSED;
        }
        if (debounce_integrator == 0) {
            pb_state = PB_RELEASED;
        }
    }

    // Power calcs
    Mpc.Ta = Mpwm.tA;
    Mpc.Tb = Mpwm.tB;
    Mpc.Tc = Mpwm.tC;
    Mpc.Vbus = Mctrl.BusVoltage;
    Mpc.Ialpha = Mfoc.Clarke_Alpha;
    Mpc.Ibeta = Mfoc.Clarke_Beta;
    power_calc(&Mpc);

}

float MAIN_GetCurrentRampAngle(void) {
    return g_rampAngle;
}

void MAIN_DetectHallPositions(float curlimit) {
    /* Run this method to determine the Hall sensor position angles
     * around the motor. This function will slowly rotate the motor with
     * a fixed current. Make sure the motor is free to move!
     */

    // Disable just about everything.
    PWM_MotorOFF();
    PWM_SetDutyF(0.0f, 0.0f, 0.0f);

    // Set the current limit as D-phase current. Q can be zero. This
    // will lock the rotor to the driven angle.
    Mctrl.state = Motor_OpenLoop;
    if (curlimit < config_main.MaxPhaseCurrent)
        Mctrl.ThrottleCommand = curlimit / config_main.MaxPhaseCurrent;
    else
        Mctrl.ThrottleCommand = 0.99f;
    // Make sure the ramp is running forwards
    if(config_main.RampSpeed < 0.0f) {
        MAIN_SetRampSpeed(config_main.RampSpeed * (-1.0f));
    }

    // Wait for Hall sensor edges.
    hdhs.old_state = HallSensor_Get_State();
    hdhs.startup_success = 0;
    hdhs.ticks_now = GetTick();
    hdhs.transitions_seen = 0;
    // Rest of routine will be called by main().
    // This ensures everything else (like USB comms) will still work normally.

}

static void RunHallDetectRoutine(void) {

    if (hdhs.startup_success == 0) {
        // First portion - waiting for motor to startup
        if (GetTick() - hdhs.ticks_now > HALL_DETECT_TIMEOUT_MS) {
            // Timeout! Return to normal operation
            PWM_MotorOFF();
            PWM_SetDutyF(0.0f, 0.0f, 0.0f);
            Mctrl.ThrottleCommand = 0.0f;
            Mctrl.state = Motor_Off;
            g_MainFlags |= MAINFLAG_HALLDETECTFAIL;
        }

        if (HallSensor_Get_State() != hdhs.old_state) {
            hdhs.transitions_seen++;
            hdhs.old_state = HallSensor_Get_State();
        }

        if (hdhs.transitions_seen > HALL_DETECT_MIN_TRANSITIONS) {
            hdhs.startup_success = 1;
            HallSensor_Enable_Hall_Detection(g_hallDetectTable,
            HALL_DETECT_TRANSITIONS_TO_AVG);
            hdhs.transitions_seen = 0;
            hdhs.ticks_now = GetTick();
        }
    } else {
        if (GetTick() - hdhs.ticks_now > 6 * HALL_DETECT_TIMEOUT_MS) {
            // Timeout! Return to normal operation
            PWM_MotorOFF();
            PWM_SetDutyF(0.0f, 0.0f, 0.0f);
            Mctrl.ThrottleCommand = 0.0f;
            Mctrl.state = Motor_Off;
            g_MainFlags |= MAINFLAG_HALLDETECTFAIL;
        }
        if (HallSensor_Get_State() != hdhs.old_state) {
            hdhs.transitions_seen++;
            hdhs.old_state = HallSensor_Get_State();
        }
        if (hdhs.transitions_seen > (6 * HALL_DETECT_TRANSITIONS_TO_AVG)) {
            // Sensing success! Return to normal.
            PWM_MotorOFF();
            PWM_SetDutyF(0.0f, 0.0f, 0.0f);
            Mctrl.ThrottleCommand = 0.0f;
            Mctrl.state = Motor_Off;
            // Average the results
            // Using the first position in the array to perform and store the average
            for (uint8_t i = 1; i < HALL_DETECT_TRANSITIONS_TO_AVG; i++) {
                for (uint8_t j = 0; j < 6; j++) {
                    g_hallDetectTable[j] += g_hallDetectTable[j + 6 * i];
                }
            }
            for (uint8_t k = 0; k < 6; k++) {
                g_hallDetectTable[k] = g_hallDetectTable[k]
                        / ((float) HALL_DETECT_TRANSITIONS_TO_AVG);
            }

            g_MainFlags |= MAINFLAG_HALLDETECTPASS;
        }
    }
}

uint8_t MAIN_SetUSBDebugOutput(uint8_t outputnum, uint8_t valuenum) {
    // Set the new output
    if ((outputnum >= MAX_USB_OUTPUTS) || (valuenum > MAX_USB_VALS))
        return DATA_PACKET_FAIL;
    config_main.USB_Choices[outputnum] = valuenum;
    return DATA_PACKET_SUCCESS;
}

uint8_t MAIN_GetUSBDebugOutput(uint8_t outputnum) {
    return config_main.USB_Choices[outputnum];
}

uint8_t MAIN_SetNumUSBDebugOutputs(uint8_t numOutputs) {
    if (numOutputs <= MAX_USB_OUTPUTS) {
        config_main.Num_USB_Outputs = numOutputs;
        return DATA_PACKET_SUCCESS;
    }
    return DATA_PACKET_FAIL;
}

uint8_t MAIN_GetNumUSBDebugOutputs(void) {
    return config_main.Num_USB_Outputs;
}

uint8_t MAIN_SetUSBDebugSpeed(uint8_t speedChoice) {
    if (speedChoice < MAX_USB_SPEED_CHOICES) {
        config_main.USB_Speed = speedChoice;
        usb_debug_countdown_reload = usb_speed_choices[config_main.USB_Speed];
        usb_debug_countdown_timer = usb_debug_countdown_reload;
        return DATA_PACKET_SUCCESS;
    }
    return DATA_PACKET_FAIL;
}

uint8_t MAIN_GetUSBDebugSpeed(void) {
    return config_main.USB_Speed;
}

uint8_t MAIN_SetUSBDebugging(uint8_t on_or_off) {
    if (on_or_off == 0)
        g_MainFlags &= ~(MAINFLAG_SERIALDATAON);
    else
        g_MainFlags |= MAINFLAG_SERIALDATAON;
    return DATA_PACKET_SUCCESS;
}

uint8_t MAIN_GetUSBDebugging(void) {
    if (g_MainFlags & MAINFLAG_SERIALDATAON)
        return 1;
    else
        return 0;
}

float MAIN_GetRampSpeed(void) {
    return config_main.RampSpeed;
}

uint8_t MAIN_SetRampSpeed(float newspeed) {
    if((newspeed > MAX_RAMP_SPEED) || (newspeed < MIN_RAMP_SPEED)) {
        return DATA_PACKET_FAIL;
    }
    int32_t tempfreq = PWM_GetFreq();
    config_main.RampSpeed = newspeed;
    g_rampInc = dfsl_rampctrlf((float)tempfreq, config_main.RampSpeed);
    return DATA_PACKET_SUCCESS;
}

uint8_t MAIN_SetVar(uint8_t var, float newval) {
    switch (var) {
    case 0:
        // Kp
        Id_control.Kp = newval;
        Iq_control.Kp = newval;
        break;
    case 1:
        // Ki
        Id_control.Ki = newval;
        Iq_control.Ki = newval;
        break;
    case 2:
        // Kd
        Id_control.Kd = newval;
        Iq_control.Kd = newval;
        break;
    case 3:
        // Kc
        Id_control.Kc = newval;
        Iq_control.Kc = newval;
        break;
    }
    return DATA_PACKET_SUCCESS;
}

float MAIN_GetVar(uint8_t var) {
    switch (var) {
    case 0:
        // Kp
        return Id_control.Kp;
        break;
    case 1:
        // Ki
        return Id_control.Ki;
        break;
    case 2:
        // Kd
        return Id_control.Kd;
        break;
    case 3:
        // Kc
        return Id_control.Kc;
        break;
    default:
        break;
    }
    return 0.0f;
}

uint8_t MAIN_SetFreq(int32_t newfreq) {
    if(PWM_SetFreq(newfreq) == DATA_PACKET_SUCCESS) {
        HallSensor_Change_Frequency(newfreq);
        config_main.PWMFrequency = newfreq;
        return DATA_PACKET_SUCCESS;
    }
    return DATA_PACKET_FAIL;
}

int32_t MAIN_GetFreq(void) {
    return config_main.PWMFrequency; // Hz
}

uint8_t MAIN_SetDeadTime(int32_t newDT) {
    return PWM_SetDeadTime(newDT);
}
int32_t MAIN_GetDeadTime(void) {
    return PWM_GetDeadTime(); // nanosec
}

uint8_t MAIN_SetCountsToFOC(uint32_t new_counts) {
    config_main.CountsToFOC = new_counts;
    return DATA_PACKET_SUCCESS;
}

uint32_t MAIN_GetCountsToFOC(void) {
    return config_main.CountsToFOC;
}

uint8_t MAIN_SetSpeedToFOC(float new_speed) {
    config_main.SpeedToFOC = new_speed;
    return DATA_PACKET_SUCCESS;
}

float MAIN_GetSpeedToFOC(void) {
    return config_main.SpeedToFOC;
}

uint8_t MAIN_SetSwitchoverEpsilon(float new_eps) {
    config_main.SwitchEpsilon = new_eps;
    return DATA_PACKET_SUCCESS;
}

float MAIN_GetSwitchoverEpsilon(void) {
    return config_main.SwitchEpsilon;
}

void MAIN_SetError(uint32_t errorCode) {
    g_errorCode |= errorCode;
}

uint8_t MAIN_RequestBLDC(void) {
    // Switch to BLDC only if the motor is currently off
    // Also, must be currently in FOC
    if((Mctrl.state == Motor_Off) && (config_main.ControlMethod == Control_FOC)) {
        config_main.ControlMethod = Control_BLDC;
        return DATA_PACKET_SUCCESS;
    }
    return DATA_PACKET_FAIL;
}

uint8_t MAIN_RequestFOC(void) {
    // Switch to FOC if the motor is off, and was in BLDC
    // Unnecessary to call this unless motor was changed to BLDC
    if((Mctrl.state == Motor_Off) && (config_main.ControlMethod == Control_BLDC)) {
        config_main.ControlMethod = Control_FOC;
        return DATA_PACKET_SUCCESS;
    }
    return DATA_PACKET_FAIL;
}

uint8_t MAIN_EnableDebugPWM(void) {
    // Switch to debugging PWM
    // This forces all three PWMs to follow the throttle command
    // Theoretically, no current should flow through the motor
    // since all three phases will be at the same potential
    if((Mctrl.state == Motor_Off) && (config_main.ControlMethod == Control_FOC)) {
        config_main.ControlMethod = Control_Debug;
        return DATA_PACKET_SUCCESS;
    }
    return DATA_PACKET_FAIL;
}

uint8_t MAIN_DisableDebugPWM(void) {
    if((Mctrl.state == Motor_Off) && (config_main.ControlMethod == Control_Debug)) {
        config_main.ControlMethod = Control_FOC;
        return DATA_PACKET_SUCCESS;
    }
    return DATA_PACKET_FAIL;
}

void MAIN_SoftReset(uint8_t restartInBootloader) {
    // Absolutely no PWM should be happening right now!
    PWM_MotorOFF();
    if (restartInBootloader) {
        // Enable access to backup registers
        BackupEnable();
        // Set the bootloader flag in backup register 0
        RTC->BKP0R = BOOTLOADER_RESET_FLAG;
    }

    // Trigger a software reset
    NVIC_SystemReset();
}

uint8_t MAIN_GetDashboardData(uint8_t* dataBuffer) {
    // Param1: F32: Throttle position (%)
    // Param2: F32: Speed (rpm)
    // Param3: F32: Phase Amps
    // Param4: F32: Battery Amps
    // Param5: F32: Battery Volts
    // Param6: F32: Controller FET Temperature (degC)
    // Param7: F32: Motor Temperature (degC)
    // Param8: I32: Fault Code

    // MPH = eHz * wheel circ *3600 / polepairs
    // RPM = eHz * 60 / polepairs
    float rpm_conversion = 60.0f * config_main.inv_pole_pairs;

    data_packet_pack_float(dataBuffer, Mctrl.ThrottleCommand);
    dataBuffer+=4;
#ifdef TESTING_2X
    data_packet_pack_float(dataBuffer, rpm_conversion*HallSensor2_Get_Speedf());
#else
    data_packet_pack_float(dataBuffer, rpm_conversion*HallSensor_Get_Speedf());
#endif
    dataBuffer+=4;
    data_packet_pack_float(dataBuffer, Mpc.PhaseCurrent);
    dataBuffer+=4;
    data_packet_pack_float(dataBuffer, Mpc.BatteryCurrent);
    dataBuffer+=4;
    data_packet_pack_float(dataBuffer, Mpc.Vbus);
    dataBuffer+=4;
    data_packet_pack_float(dataBuffer, g_FetTemp);
    dataBuffer+=4;
    data_packet_pack_float(dataBuffer, g_MotorTemp);
    dataBuffer+=4;
    data_packet_pack_32b(dataBuffer, g_errorCode);

    return DATA_PACKET_SUCCESS;
}

uint8_t MAIN_SetLimit(Main_Limit_Type lmt, float new_lmt) {
    uint8_t errCode = DATA_PACKET_SUCCESS;
    switch(lmt) {
    case Main_Limit_PhaseCurrent:
        config_main.MaxPhaseCurrent = new_lmt;
        config_main.inv_max_phase_current = (1.0f) / config_main.MaxPhaseCurrent;
        break;
    case Main_Limit_PhaseRegenCurrent:
        config_main.MaxPhaseRegenCurrent = new_lmt;
        break;
    case Main_Limit_BattCurrent:
        config_main.MaxBatteryCurrent = new_lmt;
        break;
    case Main_Limit_BattRegenCurrent:
        config_main.MaxBatteryRegenCurrent = new_lmt;
        break;
    case Main_Limit_SoftVoltage:
        config_main.VoltageSoftCap = new_lmt;
        break;
    case Main_Limit_HardVoltage:
        config_main.VoltageHardCap = new_lmt;
        break;
    case Main_Limit_SoftFetTemp:
        config_main.FetTempSoftCap = new_lmt;
        break;
    case Main_Limit_HardFetTemp:
        config_main.FetTempHardCap = new_lmt;
        break;
    case Main_Limit_SoftMotorTemp:
        config_main.MotorTempSoftCap = new_lmt;
        break;
    case Main_Limit_HardMotorTemp:
        config_main.MotorTempHardCap = new_lmt;
        break;
    case Main_Limit_MinVoltFault:
        config_main.MinVoltFault = new_lmt;
        break;
    case Main_Limit_MaxVoltFault:
        config_main.MaxVoltFault = new_lmt;
        break;
    case Main_Limit_CurrentFault:
        config_main.CurrentFault = new_lmt;
        break;
    default:
        errCode = DATA_PACKET_FAIL;
        break;
    }
    return errCode;
}
float MAIN_GetLimit(Main_Limit_Type lmt) {
    float retval;
    switch(lmt) {
    case Main_Limit_PhaseCurrent:
        retval = config_main.MaxPhaseCurrent;
        break;
    case Main_Limit_PhaseRegenCurrent:
        retval = config_main.MaxPhaseRegenCurrent;
        break;
    case Main_Limit_BattCurrent:
        retval = config_main.MaxBatteryCurrent;
        break;
    case Main_Limit_BattRegenCurrent:
        retval = config_main.MaxBatteryRegenCurrent;
        break;
    case Main_Limit_SoftVoltage:
        retval = config_main.VoltageSoftCap;
        break;
    case Main_Limit_HardVoltage:
        retval = config_main.VoltageHardCap;
        break;
    case Main_Limit_SoftFetTemp:
        retval = config_main.FetTempSoftCap;
        break;
    case Main_Limit_HardFetTemp:
        retval = config_main.FetTempHardCap;
        break;
    case Main_Limit_SoftMotorTemp:
        retval = config_main.MotorTempSoftCap;
        break;
    case Main_Limit_HardMotorTemp:
        retval = config_main.MotorTempHardCap;
        break;
    case Main_Limit_MinVoltFault:
        retval = config_main.MinVoltFault;
        break;
    case Main_Limit_MaxVoltFault:
        retval = config_main.MaxVoltFault;
        break;
    case Main_Limit_CurrentFault:
        retval = config_main.CurrentFault;
        break;
    default:
        retval = 0.0f;
        break;
    }
    return retval;
}

float MAIN_GetGearRatio(void) {
    return config_main.GearRatio;
}

float MAIN_GetWheelSize(void) {
    return config_main.WheelSizeMM;
}

uint16_t MAIN_GetPolePairs(void) {
    return config_main.MotorPolePairs;
}

float MAIN_GetMotorKv(void) {
    return config_main.MotorKv;
}

uint8_t MAIN_SetGearRatio(float new_ratio) {
    config_main.GearRatio = new_ratio;
    return DATA_PACKET_SUCCESS;
}
uint8_t MAIN_SetWheelSize(float new_size_mm) {
    config_main.WheelSizeMM = new_size_mm;
    return DATA_PACKET_SUCCESS;
}
uint8_t MAIN_SetPolePairs(uint16_t new_pole_pairs) {
    config_main.MotorPolePairs = new_pole_pairs;
    config_main.inv_pole_pairs = 1.0f / ((float)new_pole_pairs);
    return DATA_PACKET_SUCCESS;
}
uint8_t MAIN_SetMotorKv(float new_voltage_constant) {
    config_main.MotorKv = new_voltage_constant; // in rpm / volt
    config_main.kv_volts_per_ehz = ((float)config_main.MotorPolePairs) * config_main.MotorKv; // in erpm / volt
    config_main.kv_volts_per_ehz = 60.0f / config_main.kv_volts_per_ehz; // in volt/eHz

    return DATA_PACKET_SUCCESS;
}

void MAIN_DumpRecord(void) {
    if (!(g_MainFlags & MAINFLAG_DUMPDATAON))
        g_MainFlags |= MAINFLAG_DUMPRECORD;
}

#ifdef DEBUG_DUMP_USED
void MAIN_SetDumpDebugOutput(uint8_t outputnum, uint8_t valuenum)
{
    // Set the new output
    dumpassignments[outputnum] = valuenum+1;
}

uint8_t MAIN_GetDumpDebugOutput(uint8_t outputnum)
{
    return dumpassignments[outputnum];
}
#endif // DEBUG_DUMP_USED

void stringflip(char* buf, uint32_t len) {
    uint32_t i = 0, j = len - 1;
    char temp;
    while (i < j) {
        temp = buf[i];
        buf[i] = buf[j];
        buf[j] = temp;
        i++;
        j--;
    }
}

uint32_t _itoa(char* buf, int32_t num, uint32_t min_digits) {
    uint32_t i = 0;
    uint8_t is_neg = 0;
    if (num < 0) {
        is_neg = 1;
        num = -num;
    }

    do {
        buf[i++] = (num % 10) + '0';
        num = num / 10;
    } while (num);

    while (i < min_digits) {
        buf[i++] = '0';
    }
    if (is_neg) {
        buf[i++] = '-';
    }
    stringflip(buf, i);
    buf[i] = '\0';
    return i;
}

uint32_t _ftoa(char* buf, float num, uint32_t precision) {
    int32_t ipart;
    float fpart;
    uint32_t pos = 0;
    // Grab the integer part
    ipart = (int32_t) num;
    pos = _itoa(buf, ipart, 0);
    // and the fraction part
    if (precision > 0) {
        fpart = num - ((float) ipart);
        fpart = fabsf(fpart * powf(10.0f, ((float) (precision))));
        buf[pos++] = '.';
        pos += _itoa(&(buf[pos]), ((int32_t) fpart), precision);
    }
    return pos;
}

/* Convert num into ASCII in base 10
 * Result in buf as a null terminated string
 * Returns number of digits in buf, including
 * negative sign when present.
 */
/*
uint32_t itoa(char* buf, int32_t num) {
    char* going_out = buf;
    uint32_t retval = 0;
    uint8_t is_neg = 0;
    if (num < 0) {
        *going_out = '-';
        going_out++;
        is_neg = 1;
        num = -num;
    }

    // Start assembling the string, in backwards order
    // do..while is required so that a zero value
    // will print at least one character
    do {
        *going_out = (num % 10) + '0';
        going_out++;
        retval++;
        num = num / 10;
    } while (num != 0);
    // Null terminate
    *going_out = 0;
    // Now, flip the string around
    // Swapping characters using a dummy byte
    char dummy;
    for (uint8_t i = 0; i < retval / 2; i++) {
        dummy = buf[is_neg + i];
        buf[is_neg + i] = buf[is_neg + retval - 1 - i];
        buf[is_neg + retval - 1 - i] = dummy;
    }

    return retval + is_neg;
}
*/

void Delay(__IO uint32_t Delay) {
    uint32_t tickstart = 0;
    tickstart = g_MainSysTick;
    while ((g_MainSysTick - tickstart) < Delay) {
    }
}

uint32_t GetTick(void) {
    return g_MainSysTick;
}

void MAIN_SaveVariables(void) {
    EE_SaveFloat(CONFIG_FOC_KP, Id_control.Kp);
    EE_SaveFloat(CONFIG_FOC_KI, Id_control.Ki);
    EE_SaveFloat(CONFIG_FOC_KD, Id_control.Kd);
    EE_SaveFloat(CONFIG_FOC_KC, Id_control.Kc);
    EE_SaveFloat(CONFIG_MAIN_RAMP_SPEED, config_main.RampSpeed);
    EE_SaveInt32(CONFIG_MAIN_COUNTS_TO_FOC, config_main.CountsToFOC);
    EE_SaveFloat(CONFIG_MAIN_SPEED_TO_FOC, config_main.SpeedToFOC);
    EE_SaveFloat(CONFIG_MAIN_SWITCH_EPS, config_main.SwitchEpsilon);
    EE_SaveInt16(CONFIG_MAIN_NUM_USB_OUTPUTS, config_main.Num_USB_Outputs);
    EE_SaveInt16(CONFIG_MAIN_USB_SPEED, config_main.USB_Speed);
    for(uint8_t i = 0; i < MAX_USB_OUTPUTS; i++) {
        EE_SaveInt16(CONFIG_MAIN_USB_CHOICE_1 + i, config_main.USB_Choices[i]);
    }

    EE_SaveFloat(CONFIG_LMT_PHASE_CUR_MAX, config_main.MaxPhaseCurrent);
    EE_SaveFloat(CONFIG_LMT_BATT_CUR_MAX, config_main.MaxBatteryCurrent);
    EE_SaveFloat(CONFIG_LMT_PHASE_REGEN_MAX, config_main.MaxPhaseRegenCurrent);
    EE_SaveFloat(CONFIG_LMT_BATT_REGEN_MAX, config_main.MaxBatteryRegenCurrent);
    EE_SaveFloat(CONFIG_LMT_VOLT_SOFTCAP, config_main.VoltageSoftCap);
    EE_SaveFloat(CONFIG_LMT_VOLT_HARDCAP, config_main.VoltageHardCap);
    EE_SaveInt32(CONFIG_FOC_PWM_FREQ, config_main.PWMFrequency);
    EE_SaveInt32(CONFIG_FOC_PWM_DEADTIME, config_main.PWMDeadTime);
    EE_SaveFloat(CONFIG_LMT_FET_TEMP_SOFTCAP, config_main.FetTempSoftCap);
    EE_SaveFloat(CONFIG_LMT_FET_TEMP_HARDCAP, config_main.FetTempHardCap);
    EE_SaveFloat(CONFIG_LMT_MOTOR_TEMP_SOFTCAP, config_main.MotorTempSoftCap);
    EE_SaveFloat(CONFIG_LMT_MOTOR_TEMP_HARDCAP, config_main.MotorTempHardCap);
    EE_SaveFloat(CONFIG_LMT_VOLT_FAULT_MIN, config_main.MinVoltFault);
    EE_SaveFloat(CONFIG_LMT_VOLT_FAULT_MAX, config_main.MaxVoltFault);
    EE_SaveFloat(CONFIG_LMT_CUR_FAULT_MAX, config_main.CurrentFault);

    EE_SaveFloat(CONFIG_MOTOR_WHEEL_SIZE, config_main.WheelSizeMM);
    EE_SaveFloat(CONFIG_MOTOR_GEAR_RATIO, config_main.GearRatio);
    EE_SaveInt16(CONFIG_MOTOR_POLEPAIRS, config_main.MotorPolePairs);
}

void MAIN_LoadVariables(void) {
    /* Initialize PID controllers */
    dfsl_pid_defaultsf(&Id_control);
    dfsl_pid_defaultsf(&Iq_control);
    /* Load saved EEPROM variables */
    Id_control.Kp = EE_ReadFloatWithDefault(CONFIG_FOC_KP, Id_control.Kp);
    Id_control.Ki = EE_ReadFloatWithDefault(CONFIG_FOC_KI, Id_control.Ki);
    Id_control.Kd = EE_ReadFloatWithDefault(CONFIG_FOC_KD, Id_control.Kd);
    Id_control.Kc = EE_ReadFloatWithDefault(CONFIG_FOC_KC, Id_control.Kc);
    Iq_control.Kp = EE_ReadFloatWithDefault(CONFIG_FOC_KP, Iq_control.Kp);
    Iq_control.Ki = EE_ReadFloatWithDefault(CONFIG_FOC_KI, Iq_control.Ki);
    Iq_control.Kd = EE_ReadFloatWithDefault(CONFIG_FOC_KD, Iq_control.Kd);
    Iq_control.Kc = EE_ReadFloatWithDefault(CONFIG_FOC_KC, Iq_control.Kc);

    config_main.RampSpeed = EE_ReadFloatWithDefault(CONFIG_MAIN_RAMP_SPEED,
            DFLT_MAIN_RAMP_SPEED);
    config_main.CountsToFOC = EE_ReadInt32WithDefault(CONFIG_MAIN_COUNTS_TO_FOC,
            DFLT_MAIN_COUNTS_TO_FOC);
    config_main.SpeedToFOC = EE_ReadFloatWithDefault(CONFIG_MAIN_SPEED_TO_FOC,
            DFLT_MAIN_SPEED_TO_FOC);
    config_main.SwitchEpsilon = EE_ReadFloatWithDefault(CONFIG_MAIN_SWITCH_EPS,
            DFLT_MAIN_SWITCH_EPS);
    config_main.Num_USB_Outputs = EE_ReadInt16WithDefault(
            CONFIG_MAIN_NUM_USB_OUTPUTS, DFLT_MAIN_NUM_USB_OUTPUTS);
    config_main.USB_Speed = EE_ReadInt16WithDefault(CONFIG_MAIN_USB_SPEED,
            DFLT_MAIN_USB_SPEED);
    config_main.USB_Choices[0] = EE_ReadInt16WithDefault(
            CONFIG_MAIN_USB_CHOICE_1, DFLT_MAIN_USB_CHOICE_1);
    config_main.USB_Choices[1] = EE_ReadInt16WithDefault(
            CONFIG_MAIN_USB_CHOICE_2, DFLT_MAIN_USB_CHOICE_2);
    config_main.USB_Choices[2] = EE_ReadInt16WithDefault(
            CONFIG_MAIN_USB_CHOICE_3, DFLT_MAIN_USB_CHOICE_3);
    config_main.USB_Choices[3] = EE_ReadInt16WithDefault(
            CONFIG_MAIN_USB_CHOICE_4, DFLT_MAIN_USB_CHOICE_4);
    config_main.USB_Choices[4] = EE_ReadInt16WithDefault(
            CONFIG_MAIN_USB_CHOICE_5, DFLT_MAIN_USB_CHOICE_5);
    config_main.USB_Choices[5] = EE_ReadInt16WithDefault(
            CONFIG_MAIN_USB_CHOICE_6, DFLT_MAIN_USB_CHOICE_6);
    config_main.USB_Choices[6] = EE_ReadInt16WithDefault(
            CONFIG_MAIN_USB_CHOICE_7, DFLT_MAIN_USB_CHOICE_7);
    config_main.USB_Choices[7] = EE_ReadInt16WithDefault(
            CONFIG_MAIN_USB_CHOICE_8, DFLT_MAIN_USB_CHOICE_8);
    config_main.USB_Choices[8] = EE_ReadInt16WithDefault(
            CONFIG_MAIN_USB_CHOICE_9, DFLT_MAIN_USB_CHOICE_9);
    config_main.USB_Choices[9] = EE_ReadInt16WithDefault(
            CONFIG_MAIN_USB_CHOICE_10, DFLT_MAIN_USB_CHOICE_10);
    config_main.MaxPhaseCurrent = EE_ReadFloatWithDefault(
            CONFIG_LMT_PHASE_CUR_MAX, DFLT_LMT_PHASE_CUR_MAX);
    config_main.inv_max_phase_current = (1.0f) / config_main.MaxPhaseCurrent;
    config_main.MaxBatteryCurrent = EE_ReadFloatWithDefault(
            CONFIG_LMT_BATT_CUR_MAX, DFLT_LMT_BATT_CUR_MAX);
    config_main.MaxPhaseRegenCurrent = EE_ReadFloatWithDefault(
            CONFIG_LMT_PHASE_REGEN_MAX, DFLT_LMT_PHASE_REGEN_MAX);
    config_main.MaxBatteryRegenCurrent = EE_ReadFloatWithDefault(
            CONFIG_LMT_BATT_REGEN_MAX, DFLT_LMT_BATT_REGEN_MAX);
    config_main.VoltageSoftCap = EE_ReadFloatWithDefault(
            CONFIG_LMT_VOLT_SOFTCAP, DFLT_LMT_VOLT_SOFTCAP);
    config_main.VoltageHardCap = EE_ReadFloatWithDefault(
            CONFIG_LMT_VOLT_HARDCAP, DFLT_LMT_VOLT_HARDCAP);
    config_main.FetTempSoftCap = EE_ReadFloatWithDefault(CONFIG_LMT_FET_TEMP_SOFTCAP, DFLT_LMT_FET_TEMP_SOFTCAP);
    config_main.FetTempHardCap = EE_ReadFloatWithDefault(CONFIG_LMT_FET_TEMP_HARDCAP, DFLT_LMT_FET_TEMP_HARDCAP);
    config_main.MotorTempSoftCap = EE_ReadFloatWithDefault(CONFIG_LMT_MOTOR_TEMP_SOFTCAP, DFLT_LMT_MOTOR_TEMP_SOFTCAP);
    config_main.MotorTempHardCap = EE_ReadFloatWithDefault(CONFIG_LMT_MOTOR_TEMP_HARDCAP, DFLT_LMT_MOTOR_TEMP_HARDCAP);
    config_main.MinVoltFault = EE_ReadFloatWithDefault(CONFIG_LMT_VOLT_FAULT_MIN, DFLT_LMT_VOLT_FAULT_MIN);
    config_main.MaxVoltFault = EE_ReadFloatWithDefault(CONFIG_LMT_VOLT_FAULT_MAX, DFLT_LMT_VOLT_FAULT_MAX);
    config_main.CurrentFault = EE_ReadFloatWithDefault(CONFIG_LMT_CUR_FAULT_MAX, DFLT_LMT_CUR_FAULT_MAX);

    config_main.GearRatio = EE_ReadFloatWithDefault(CONFIG_MOTOR_GEAR_RATIO, DFLT_MOTOR_GEAR_RATIO);
    config_main.WheelSizeMM = EE_ReadFloatWithDefault(CONFIG_MOTOR_WHEEL_SIZE, DFLT_MOTOR_WHEEL_SIZE);
    MAIN_SetPolePairs(EE_ReadInt16WithDefault(CONFIG_MOTOR_POLEPAIRS, DFLT_MOTOR_POLEPAIRS));
    MAIN_SetMotorKv(EE_ReadFloatWithDefault(CONFIG_MOTOR_KV, DFLT_MOTOR_KV));

    config_main.PWMFrequency = EE_ReadInt32WithDefault(CONFIG_FOC_PWM_FREQ,
            DFLT_FOC_PWM_FREQ);
    MAIN_SetFreq(config_main.PWMFrequency);
    config_main.PWMDeadTime = EE_ReadInt32WithDefault(CONFIG_FOC_PWM_DEADTIME,
            DFLT_FOC_PWM_DEADTIME);
    MAIN_SetDeadTime(config_main.PWMDeadTime);

    usb_debug_countdown_timer = usb_speed_choices[config_main.USB_Speed];
    usb_debug_countdown_reload = usb_speed_choices[config_main.USB_Speed];

    g_rampInc = dfsl_rampctrlf((float) config_main.PWMFrequency,
            config_main.RampSpeed);
}
