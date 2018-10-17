/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

#define MAINFLAG_SERIALDATAPRINT  ((uint32_t)0x00000001)
#define MAINFLAG_SERIALDATAON   ((uint32_t)0x00000002)
#define MAINFLAG_DUMPRECORD     ((uint32_t)0x00000004)
#define MAINFLAG_DUMPDATAPRINT    ((uint32_t)0x00000008)
#define MAINFLAG_DUMPDATAON     ((uint32_t)0x00000010)
#define MAINFLAG_CONVERTTEMPERATURE ((uint32_t)0x00000020)

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
__IO uint32_t g_MainSysTick;

uint8_t g_rampdir;
uint16_t g_rampAngle;
uint16_t g_rampInc;
uint32_t g_ledcount;

uint32_t g_errorCode;

uint32_t g_MainFlags;

uint32_t cur_a_sum;
uint32_t cur_b_sum;
uint32_t cur_c_sum;
uint32_t cur_sum_count;

uint32_t speed_cycle_integrator;

Motor_Controls Mctrl;
Motor_Observations Mobv;
Motor_PWMDuties Mpwm;
FOC_StateVariables Mfoc;

float g_FetTemp;

PowerCalcs Mpc;

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

uint8_t usb_num_output_vars = DEFAULT_USB_OUTPUTS;

uint8_t usb_speed_choice = 0;
uint32_t usb_speed_choices[MAX_USB_SPEED_CHOICES] = USB_SPEED_RELOAD_VALS;
uint8_t usb_debug_buffer[4*CDC_DATA_FS_MAX_PACKET_SIZE];

__IO uint32_t usb_debug_buffer_pos = 0;
__IO uint32_t usb_debug_countdown_timer = DEFAULT_SERIAL_DATA_RATE;
uint32_t usb_debug_countdown_reload = DEFAULT_SERIAL_DATA_RATE;
uint8_t usb_debug_prefix[USB_PREFIX_LENGTH] = DEFAULT_USB_PREFIX;

#if USB_MONITOR_FIXED_POINT
    uint32_t usbdacvals[MAX_USB_VALS];
#else
    float usbdacvals[MAX_USB_VALS];
#endif
    /* Default USB debugging outputs: Ia, Ib, Ic, Throttle, Vbus */
    uint8_t usbdacassignments[MAX_USB_OUTPUTS] = DEFAULT_USB_ASSIGNMENTS;

    char vcp_buffer[UI_MAX_BUFFER_LENGTH];

    uint32_t systick_debounce_counter;
    uint8_t debounce_integrator;
    volatile PB_TypeDef pb_state;

//extern uint16_t adc_conv[NUM_ADC_CH];

    PID_Float_Type Id_control,
Iq_control;
//Biquad_Float_Type Throttle_filt=BIQ_LPF_DEFAULTS;
Biquad_Float_Type Id_Filt, Iq_Filt;
float Throttle_cmd;
float raw_throttle;

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void Toggle_Leds(void);
static void User_PB_Init(void);
static void User_LED_Init(void);
static void User_DAC_Init(void);
static void User_BasicTim_Init(void);

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
  char byte;
  uint32_t resplen;
#ifdef DEBUG_USED
  char string[32];
  char usbstring[64];
#endif // DEBUG_USED
  Throttle_cmd = 0.0f;

  BootloaderStartup(); // Load bootloader if certain conditions are met
  // Also initializes the user pushbutton

  // Disable (turn on gpio pulldown resistor) unused pins
  GPIO_Pulldown_Unused();

  /* Configure the system clock to 168 MHz */
  SystemClock_Config();

  /* Default initialization:
   ** Configure the Flash prefetch, instruction and Data caches
   ** Configure the Systick to generate an interrupt each 1 msec
   ** Set NVIC Group Priority to 4
   ** Start systick timer with 1ms interval
   */
  FLASH->ACR |= FLASH_ACR_ICEN;
  FLASH->ACR |= FLASH_ACR_DCEN;

  /* STM32F405x/407x/415x/417x Revision Z devices: prefetch is supported  */
  if (((DBGMCU->IDCODE) >> 16) == 0x1001) {
    /* Enable the Flash prefetch */
    FLASH->ACR |= FLASH_ACR_PRFTEN;
  }

  NVIC_SetPriorityGrouping(0); // Maximum number of priority bits (4 for this MCU), no sub-priority bits

  SysTick_Config(SystemCoreClock / 1000);
  NVIC_SetPriority(SysTick_IRQn, PRIO_SYSTICK);

  g_errorCode = 0;

  /* Configure LED1, LED2, LED3 and LED4 */
  /* David's version */
  User_LED_Init();
  //GPIOD->ODR |= GPIO_PIN_12|GPIO_PIN_15;
  GLED_PORT->ODR |= (1 << GLED_PIN);
  adcInit();
  User_DAC_Init();
  User_BasicTim_Init();
  PWM_Init();
  HallSensor_Init_NoHal(20000);
  HBD_Init();

  // USB init
  USB_Init();
  USB_SetClass(&USB_CDC_ClassDesc, &USB_CDC_ClassCallbacks);
  USB_Start();

  /* Initialize ramp angle increment at 5 Hz*/
  g_rampdir = 0; // Going forward at start.
  g_rampInc = dfsl_rampctrl(RAMP_CALLFREQ, RAMP_DEFAULTSPEED);

  /* Initialize PID controllers */
  dfsl_pid_defaultsf(&Id_control);
  dfsl_pid_defaultsf(&Iq_control);

#ifdef DEBUG_DUMP_USED
  DumpReset();
#endif // DEBUG_DUMP_USED

  /* Set defaults for D, Q current filters */
  dfsl_biquadcalc_lpf(&Id_Filt, 20000.0f, 2000.0f, 0.707f);
  dfsl_biquadcalc_lpf(&Iq_Filt, 20000.0f, 2000.0f, 0.707f);

  /* Initialize watchdog timer */
  WDT_init();

  // Init the motor controller
  Mctrl.state = Motor_Off;
  Mfoc.Id_PID = &Id_control;
  Mfoc.Iq_PID = &Iq_control;

  /* Run Application (Interrupt mode) */
  while (1) {
    uint32_t vcp_buf_len;
    // Feed the watchdog!
    WDT_feed();

    //Toggle_Leds();
    if (VCP_Read(&byte, 1) != 0) {
      // Echo it
      while (VCP_Write(&byte, 1) < 0)
        ;
      // Add it to the VCP buffer

      vcp_buf_len = strlen(vcp_buffer);
      if (vcp_buf_len < (UI_MAX_BUFFER_LENGTH - 1)) {
        vcp_buffer[vcp_buf_len] = byte;
        vcp_buffer[vcp_buf_len + 1] = 0;
      } else {
        // Flush the buffer, just ignore overlong strings
        vcp_buffer[0] = 0;
      }

      if (byte == '\n') {
        // Send it to the UI processor!
        UI_Process(vcp_buffer);

        // and flush
        vcp_buffer[0] = 0;

        // Send response if it exists
        resplen = UI_RespLen();
        if (resplen > 0) {
          if (resplen <= CDC_DATA_FS_MAX_PACKET_SIZE) {
            while (VCP_Write(UI_SendBuf(), resplen) < 0)
              ;
          } else {
            // Copy to local memory
            memcpy(vcp_buffer, UI_SendBuf(), resplen);
            vcp_buf_len = resplen;
            uint32_t vcp_pointer = 0;
            while (vcp_buf_len > 0) {
              if (vcp_buf_len > (CDC_DATA_FS_MAX_PACKET_SIZE - 4)) {
                while (VCP_Write(&(vcp_buffer[vcp_pointer]),
                    (CDC_DATA_FS_MAX_PACKET_SIZE - 4)) < 0)
                  ;
                vcp_pointer += (CDC_DATA_FS_MAX_PACKET_SIZE - 4);
                vcp_buf_len -= (CDC_DATA_FS_MAX_PACKET_SIZE - 4);
              } else {
                while (VCP_Write(&(vcp_buffer[vcp_pointer]), vcp_buf_len) < 0)
                  ;
                vcp_buf_len = 0;
                vcp_buffer[0] = 0;
              }
            }
          }
        }
      }
    }
    /*
     if(HBD_Receive(&byte, 1) != 0)
     {
     // Read a byte from the HBD UART
     UI_Process(byte);
     }
     */
    if (pb_state == PB_PRESSED) {
      // Wait until release
      while (pb_state == PB_PRESSED) {
      }
      // Change data output state
      g_MainFlags ^= MAINFLAG_SERIALDATAON;
    }
    if (g_MainFlags & MAINFLAG_CONVERTTEMPERATURE) {
      g_MainFlags &= ~MAINFLAG_CONVERTTEMPERATURE;
      g_FetTemp = adcGetTempDegC();
    }
    if (g_MainFlags & MAINFLAG_SERIALDATAON) {
      if (usb_debug_buffer_pos > 0) {
        if (VCP_Write(usb_debug_buffer, usb_debug_buffer_pos) != -1) {
          usb_debug_buffer_pos = 0;
        }
      }
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
  RCC->PLLCFGR = (pllq << 24) | RCC_PLLCFGR_PLLSRC | (pllp << 16) | (plln << 6)
      | (pllm);

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
  /**** Old version needed this for serial debugging ****
   if((g_MainSysTick % DEFAULT_SERIAL_DATA_RATE) == 0)
   {
   g_MainFlags |= MAINFLAG_SERIALDATAPRINT;
   }
   */
  if ((g_MainSysTick % SERIAL_DUMP_RATE) == 0) {
    g_MainFlags |= MAINFLAG_DUMPDATAPRINT;
  }
  if ((g_MainSysTick % TEMP_CONVERSION_RATE) == 0) {
    g_MainFlags |= MAINFLAG_CONVERTTEMPERATURE;
  }
}

/*
 void User_HallTIM_IRQ(void)
 {
 HallSensor_UpdateCallback();
 }
 */

// TIM1 overflow / update IRQ (20kHz)
void User_PWMTIM_IRQ(void) {
  // Debug: Blink RLED to show how much processor time is used
  RLED_PORT->BSRR = (1 << RLED_PIN);
#if 1
  // Testing mode starts here
  uint16_t tA, tB, tC;
  Mobv.iA = adcGetCurrent(ADC_IA);
  Mobv.iB = adcGetCurrent(ADC_IB);
  Mobv.iC = adcGetCurrent(ADC_IC);
  Mctrl.BusVoltage = adcGetVbus();

  HallSensor_Inc_Angle();
#ifdef TESTING_2X
  HallSensor2_Inc_Angle();
#endif
  if (g_rampdir == 0) {
    g_rampAngle += g_rampInc;
  } else {
    g_rampAngle -= g_rampInc;
  }
#ifdef TESTING_2X
  Mobv.RotorAngle = HallSensor2_Get_Anglef();
#else
  Mobv.RotorAngle = HallSensor_Get_Anglef();
#endif
  Mobv.HallState = HallSensor_Get_State();
  if (Mctrl.state == Motor_Startup) {
    if (HallSensor_Get_Speedf() > MIN_SPEED_TO_FOC)
      speed_cycle_integrator++;
    else {
      if (speed_cycle_integrator > 0)
        speed_cycle_integrator--;
    }
    if (speed_cycle_integrator > SPEED_COUNTS_TO_FOC) {
      Mctrl.state = Motor_AtSpeed;
    }
  }

  if (Throttle_cmd <= 0.0f) {
    PWM_MotorOFF();
    Throttle_cmd = 0.0f;
    speed_cycle_integrator = 0;
    Mctrl.state = Motor_Off;
  } else {
    PWM_MotorON();
    if (Mctrl.state == Motor_Off)
      Mctrl.state = Motor_Startup;
  }

  Mctrl.ThrottleCommand = Throttle_cmd;
  Motor_Loop(&Mctrl, &Mobv, &Mfoc, &Mpwm);
  tA = (uint16_t) (Mpwm.tA * 65535.0f);
  tB = (uint16_t) (Mpwm.tB * 65535.0f);
  tC = (uint16_t) (Mpwm.tC * 65535.0f);
  // Is throttle at zero? Motor off

  PWM_SetDuty(tA, tB, tC);
#endif
  // USB Debugging outputs
  // Current is scaled from +- 20 amps to 0->65536 (0 = -20A, 65536 = +20A)
#if USB_MONITOR_FIXED_POINT
  usbdacvals[0] = (uint32_t)((Mobv.iA+20.0f)*1638.4f);
  usbdacvals[1] = (uint32_t)((Mobv.iB+20.0f)*1638.4f);
  usbdacvals[2] = (uint32_t)((Mobv.iC+20.0f)*1638.4f);
  usbdacvals[3] = tA;
  usbdacvals[4] = tB;
  usbdacvals[5] = tC;
  usbdacvals[6] = (uint16_t)(Throttle_cmd * 65536.0f);
  usbdacvals[7] = g_rampAngle;
  usbdacvals[8] = HallSensor_Get_Angle();
  usbdacvals[9] = HallSensor_Get_Speed();
  usbdacvals[10] = (uint32_t)(adcGetVbus()*655.36f); // Bus voltage scaled to 0->100V
  usbdacvals[11] = (uint32_t)((Mfoc.Park_D+20.0f)*1638.4f);
  usbdacvals[12] = (uint32_t)((Mfoc.Park_Q+20.0f)*1638.4f);
  usbdacvals[13] = (uint32_t)((Id_Filt.Y + 20.0f)*1638.4f);
  usbdacvals[14] = (uint32_t)((Iq_Filt.Y + 20.0f)*1638.4f);
  //usbdacvals[13] = (uint32_t)((Id_control.Out+5.0f)*6553.6f);
  //usbdacvals[14] = (uint32_t)((Iq_control.Out+5.0f)*6553.6f);
  usbdacvals[15] = g_errorCode;
  usbdacvals[16] = adcRaw(ADC_VREFINT);
  usbdacvals[17] = HallSensor_Get_State();

#ifdef TESTING_2X
  usbdacvals[18] = HallSensor2_Get_Angle();
#endif // TESTING_2X

#else // !USB_MONITOR_FIXED_POINT
  usbdacvals[0] = Mobv.iA;
  usbdacvals[1] = Mobv.iB;
  usbdacvals[2] = Mobv.iC;
  usbdacvals[3] = Mpwm.tA;
  usbdacvals[4] = Mpwm.tB;
  usbdacvals[5] = Mpwm.tC;
  usbdacvals[6] = Throttle_cmd;
  usbdacvals[7] = ((float) g_rampAngle) / 65535.0f;
  usbdacvals[8] = HallSensor_Get_Anglef();
  usbdacvals[9] = HallSensor_Get_Speedf();
  usbdacvals[10] = Mctrl.BusVoltage;
  usbdacvals[11] = Mfoc.Park_D;
  usbdacvals[12] = Mfoc.Park_Q;
  Id_Filt.X = Mfoc.Park_D;
  Iq_Filt.X = Mfoc.Park_Q;
  dfsl_biquadf(&Id_Filt);
  dfsl_biquadf(&Iq_Filt);
  usbdacvals[13] = Id_Filt.Y;
  usbdacvals[14] = Iq_Filt.Y;
  //usbdacvals[13] = (uint32_t)((Id_control.Out+5.0f)*6553.6f);
  //usbdacvals[14] = (uint32_t)((Iq_control.Out+5.0f)*6553.6f);
  usbdacvals[15] = (float) (g_errorCode);
  usbdacvals[16] = (float) (adcRaw(ADC_VREFINT));
  usbdacvals[17] = (float) HallSensor_Get_State();
#ifdef TESTING_2X
  usbdacvals[18] = HallSensor2_Get_Anglef();
#endif // TESTING_2X
#endif // !USB_MONITOR_FIXED_POINT

  // Load up the output buffer
  if (g_MainFlags & MAINFLAG_SERIALDATAON) {
    if ((--usb_debug_countdown_timer) == 0) {
      usb_debug_countdown_timer = usb_debug_countdown_reload;
      if ((usb_debug_buffer_pos + 4 + 4 * MAX_USB_OUTPUTS)
          < (4 * CDC_DATA_FS_MAX_PACKET_SIZE)) {
        memcpy(&(usb_debug_buffer[usb_debug_buffer_pos]), usb_debug_prefix, 4);
        usb_debug_buffer_pos += 4;
        for (uint8_t i = 0; i < usb_num_output_vars; i++) {
          if (usbdacassignments[i] == 0)
            memset(&(usb_debug_buffer[usb_debug_buffer_pos]), 0, 4);
          else
            memcpy(&(usb_debug_buffer[usb_debug_buffer_pos]),
                &(usbdacvals[usbdacassignments[i] - 1]), 4);
          usb_debug_buffer_pos += 4;
        }
      }
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

  throttle_process(1);
  Throttle_cmd = throttle_get_command(1);

  // Trim to 99%
  if (Throttle_cmd >= 1.0f)
    Throttle_cmd = 0.99f;

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

uint8_t MAIN_SetUSBDebugOutput(uint8_t outputnum, uint8_t valuenum) {
  // Set the new output
  if((outputnum >= MAX_USB_OUTPUTS) || (valuenum > MAX_USB_VALS))
      return UI_ERROR;
  usbdacassignments[outputnum] = valuenum;
  return UI_OK;
}

uint8_t MAIN_GetUSBDebugOutput(uint8_t outputnum) {
  return usbdacassignments[outputnum];
}

uint8_t MAIN_SetNumUSBDebugOutputs(uint8_t numOutputs) {
  if (numOutputs <= MAX_USB_OUTPUTS) {
    usb_num_output_vars = numOutputs;
    usb_debug_prefix[0] = 'D';
    usb_debug_prefix[1] = 'B';
    if (usb_num_output_vars >= 10) {
      usb_debug_prefix[2] = (usb_num_output_vars / 10) + '0';
    }
    usb_debug_prefix[3] = (usb_num_output_vars % 10) + '0';
    return UI_OK;
  }
  return UI_ERROR;
}

uint8_t MAIN_GetNumUSBDebugOutputs(void) {
  return usb_num_output_vars;
}

uint8_t MAIN_SetUSBDebugSpeed(uint8_t speedChoice) {
  if (speedChoice < MAX_USB_SPEED_CHOICES) {
    usb_speed_choice = speedChoice;
    usb_debug_countdown_reload = usb_speed_choices[usb_speed_choice];
    usb_debug_countdown_timer = usb_debug_countdown_reload;
    return UI_OK;
  }
  return UI_ERROR;
}

uint8_t MAIN_GetUSBDebugSpeed(void) {
  return usb_speed_choice;
}

uint8_t MAIN_SetUSBDebugging(uint8_t on_or_off) {
  if (on_or_off == 0)
    g_MainFlags &= ~(MAINFLAG_SERIALDATAON);
  else
    g_MainFlags |= MAINFLAG_SERIALDATAON;
  return UI_OK;
}

uint8_t MAIN_GetUSBDebugging(void) {
  if (g_MainFlags & MAINFLAG_SERIALDATAON)
    return 1;
  else
    return 0;
}

uint8_t MAIN_SetRampSpeed(uint32_t newspeed) {
  g_rampInc = dfsl_rampctrl(RAMP_CALLFREQ, newspeed);
  return UI_OK;
}

uint8_t MAIN_SetRampDir(uint8_t forwardOrBackwards) {
  if (forwardOrBackwards == 0) {
    // Forwards!
    g_rampdir = 0;
  } else {
    g_rampdir = 1;
  }
  return UI_OK;
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
  return UI_OK;
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
  }
  return 0.0f;
}

uint8_t MAIN_SetFreq(int32_t newfreq) {
  // Not yet implemented
  return UI_ERROR;
}

int32_t MAIN_GetFreq(void) {
  return 20000; // Hz
}

uint8_t MAIN_SetDeadTime(int32_t newDT) {
  // Not yet implemented
  return UI_ERROR;
}
int32_t MAIN_GetDeadTime(void) {
  return 500; // nanosec
}

uint8_t MAIN_SetThrType(uint8_t thrtype, uint8_t thrnum) {
  return UI_ERROR;
}

uint8_t MAIN_GetThrType(uint8_t thrnum) {
  return 0;
}

uint8_t MAIN_SetThrMin(int32_t thrtype, uint8_t thrnum) {
  return UI_ERROR;
}

int32_t MAIN_GetThrMin(uint8_t thrnum) {
  return 0;
}

uint8_t MAIN_SetThrMax(int32_t thrtype, uint8_t thrnum) {
  return UI_ERROR;
}

int32_t MAIN_GetThrMax(uint8_t thrnum) {
  return 0;
}

uint8_t MAIN_SetThrHyst(int32_t thrtype, uint8_t thrnum) {
  return UI_ERROR;
}

int32_t MAIN_GetThrHyst(uint8_t thrnum) {
  return 0;
}

uint8_t MAIN_SetThrFilt(int32_t thrtype, uint8_t thrnum) {
  return UI_ERROR;
}

int32_t MAIN_GetThrFilt(uint8_t thrnum) {
  return 0;
}

uint8_t MAIN_SetThrRise(int32_t thrtype, uint8_t thrnum) {
  return UI_ERROR;
}

int32_t MAIN_GetThrRise(uint8_t thrnum) {
  return 0;
}

void MAIN_SetError(uint32_t errorCode) {
  g_errorCode |= errorCode;
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
 uint32_t itoa(char* buf, int32_t num)
 {
 char* going_out = buf;
 uint32_t retval = 0;
 uint8_t is_neg = 0;
 if(num < 0)
 {
 *going_out = '-';
 going_out++;
 is_neg = 1;
 num = -num;
 }

 // Start assembling the string, in backwards order
 // do..while is required so that a zero value
 // will print at least one character
 do
 {
 *going_out = (num % 10) + '0';
 going_out++;
 retval++;
 num = num / 10;
 }while(num != 0);
 // Null terminate
 *going_out = 0;
 // Now, flip the string around
 // Swapping characters using a dummy byte
 char dummy;
 for(uint8_t i = 0; i < retval / 2; i++)
 {
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
