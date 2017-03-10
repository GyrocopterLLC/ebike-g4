/* Includes ------------------------------------------------------------------*/
#include "main.h"
  
/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define RAMP_CALLFREQ		(20000)
#define RAMP_DEFAULTSPEED	(5)

#define BOOTLOADER_RESET_FLAG	0xDEADBEEF

#define SERIAL_DATA_RATE			(10)

#define MAIN_STARTUP_SPEED_MAX		(65536*10*MOTOR_POLEPAIRS/60) // 10 RPM in electrical Hz (Q16 format)
#define MAIN_STARTUP_CUR_AVG_COUNT	(256)

#define MAINFLAG_SERIALDATA			((uint32_t)0x00000001)
#define MAINFLAG_UNUSED0			((uint32_t)0x00000002)
#define MAINFLAG_UNUSED1			((uint32_t)0x00000004)
#define MAINFLAG_UNUSED2			((uint32_t)0x00000008)
#define MAINFLAG_UNUSED3			((uint32_t)0x00000010)
#define MAINFLAG_UNUSED4			((uint32_t)0x00000020)
#define MAINFLAG_UNUSED5			((uint32_t)0x00000040)
#define MAINFLAG_UNUSED6			((uint32_t)0x00000080)
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


/** Debugging outputs **
 *
 * 0 - Ia
 * 1 - Ib
 * 2 - Ic
 * 3 - Ta
 * 4 - Tb
 * 5 - Tc
 * 6 - Throttle
 * 7 - RampAngle
 * 8 - HallAngle
 * 9 - HallSpeed
 * 10 - HallDifference
 * 11 - Id
 * 12 - Iq
 * 13 - Td
 * 14 - Tq
 * 15 - ErrorCode
 * 16 - Vrefint
 *
 */
#define MAX_USB_VALS	17
#define MAX_USB_OUTPUTS	5
uint32_t usbdacvals[MAX_USB_VALS];
uint8_t usbdacassignments[MAX_USB_OUTPUTS];
char vcp_buffer[UI_MAX_BUFFER_LENGTH];

uint32_t systick_debounce_counter;
uint8_t debounce_integrator;
volatile PB_TypeDef pb_state;
uint8_t data_out;

//extern uint16_t adc_conv[NUM_ADC_CH];

PID_Float_Type Id_control, Iq_control;
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
static void BackupEnable(void)
{
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

static void BootloaderStartup(void)
{
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

	User_PB_Init(); // Enable pushbutton so we can check if it's pressed
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
		bootloader(); // Call the bootloader.
	}
}
/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
	char byte;
	char string[32];
	char usbstring[64];
	Throttle_cmd = 0.0f;
	data_out = 0;

	BootloaderStartup(); // Load bootloader if certain conditions are met
	// Also initializes the user pushbutton

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
	if (((DBGMCU->IDCODE) >> 16) == 0x1001)
	{
		/* Enable the Flash prefetch */
		FLASH->ACR |= FLASH_ACR_PRFTEN;
	}
  
	NVIC_SetPriorityGrouping(0); // Maximum number of priority bits (4 for this MCU), no sub-priority bits

	SysTick_Config(SystemCoreClock/1000);
	NVIC_SetPriority(SysTick_IRQn, PRIO_SYSTICK);

	g_errorCode = 0;

	/* Configure LED1, LED2, LED3 and LED4 */
	/* David's version */
	User_LED_Init();
	//GPIOD->ODR |= GPIO_PIN_12|GPIO_PIN_15;
	GLED_PORT->ODR |= (1<<GLED_PIN);
	adcInit();
	User_DAC_Init();
	User_BasicTim_Init();
	//PWM_Init();
	PWM_Init_NoHal();
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

	/* Default USB debugging outputs */
	usbdacassignments[0] = 1; // Ia
	usbdacassignments[1] = 2; // Ib
	usbdacassignments[2] = 3; // Ic
	usbdacassignments[3] = 7; // Throttle
	usbdacassignments[4] = 9; // Hall angle

	/* Set defaults for D, Q current filters */
	dfsl_biquadcalc_lpf(&Id_Filt, 20000.0f, 500.0f, 0.707f);
	dfsl_biquadcalc_lpf(&Iq_Filt, 20000.0f, 500.0f, 0.707f);

	/* Initialize watchdog timer */
	WDT_init();

	/* Run Application (Interrupt mode) */
	while (1)
	{
		uint8_t vcp_buf_len;
		// Feed the watchdog!
		WDT_feed();

		//Toggle_Leds();
		if(VCP_Read(&byte, 1) != 0)
		{
			// Echo it
			VCP_Write(&byte, 1);
			// Add it to the VCP buffer
			vcp_buf_len = strlen(vcp_buffer);
			if(vcp_buf_len < (UI_MAX_BUFFER_LENGTH - 1))
			{
				vcp_buffer[vcp_buf_len] = byte;
				vcp_buffer[vcp_buf_len + 1] = 0;
			}
			else
			{
				// Flush the buffer, just ignore overlong strings
				vcp_buffer[0] = 0;
			}

			if(byte == '\n')
			{
			// Send it to the UI processor!
				UI_Process(vcp_buffer);

				// and flush
				vcp_buffer[0] = 0;

				// Send response if it exists
				if(UI_RespLen() > 0)
				{
					VCP_Write(UI_SendBuf(), UI_RespLen());
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
		if(pb_state == PB_PRESSED)
		{
			// Wait until release
			while(pb_state == PB_PRESSED) {}
			// Change data output state
			data_out = ~data_out;
		}
		if(data_out != 0)
		{
			if(g_MainFlags & MAINFLAG_SERIALDATA)
			{
				g_MainFlags &= ~MAINFLAG_SERIALDATA;
				usbstring[0] = 0;
				for(uint8_t i = 0; i < MAX_USB_OUTPUTS; i++)
				{
					if(usbdacassignments[i] == 0)
					{
						strcat(usbstring,"0 ");
					}
					else
					{
						//sprintf(string,"%d ",(int)usbdacvals[usbdacassignments[i]-1]);
						itoa(string, (int)usbdacvals[usbdacassignments[i]-1]);
						strcat(usbstring,string);
						strcat(usbstring," ");
					}

				}

				strcat(usbstring,"\r\n");

				VCP_Write(usbstring,strlen(usbstring));
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
static void SystemClock_Config(void)
{
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
	do
	{
		timeout--;
		if(timeout == 0) return;
	}while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI);

	// Disable PLL
	RCC->CR &= ~(RCC_CR_PLLON);

	// Enable HSE Oscillator and activate PLL with HSE as source
	RCC->CR |= RCC_CR_HSEON; // Turn on HSE
	// Wait for HSE to turn on
	timeout = 20000;
	do
	{
	  timeout--;
	  if(timeout == 0) return;
	}
	while(!(RCC->CR & RCC_CR_HSERDY));

	// Configure PLL multipliers
	pllm = HSE_VALUE / 1000000u; // Set so the PLL input clock is 1MHz (HSE/pllm = 1MHz)
	pllp = 0; // This is actually PLL_P = 2
	pllq = 7;
	plln = 336;
	// Set PLL multipliers with HSE selected as input source
	RCC->PLLCFGR = (pllq << 24)
			| RCC_PLLCFGR_PLLSRC
			| (pllp << 16)
			| (plln << 6)
			| (pllm);

	// Enable PLL
	RCC->CR |= RCC_CR_PLLON;

	timeout = 20000;
	do
	{
		timeout--;
		if(timeout==0) return;
	}while(!(RCC->CR & RCC_CR_PLLRDY)); // Wait for PLL to start

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
static void Toggle_Leds(void)
{
	//RLED_PORT->ODR ^= (1<<RLED_PIN);
	GLED_PORT->ODR ^= (1<<GLED_PIN);
}
/* Configure PC9 and PB12 as LED outputs */
static void User_LED_Init(void)
{
	// Enable clocks to GPIOB and C
	GPIO_Clk(GLED_PORT);
	GPIO_Clk(RLED_PORT);

	GPIO_Output(GLED_PORT, GLED_PIN);
	GPIO_Output(RLED_PORT, RLED_PIN);
}


/* Configure PC12 as pushbutton input */
static void User_PB_Init(void)
{
	// Start GPIOC clock
	GPIO_Clk(PB_PORT);

	// Enable pin PC12 as input with weak pullup turned on
	GPIO_Input(PB_PORT,PB_PIN);
	PB_PORT->PUPDR |= (GPIO_PUPDR_PUPDR0_0 << (PB_PIN * 2));
}

static void User_DAC_Init(void)
{
	RCC->APB1ENR |= RCC_APB1ENR_DACEN;
	GPIO_Clk(DAC_PORT);

	GPIO_Analog(DAC_PORT,DAC1_PIN);
	GPIO_Analog(DAC_PORT,DAC2_PIN);

	DAC->CR = DAC_CR_BOFF2 | DAC_CR_BOFF1 | DAC_CR_EN2 | DAC_CR_EN1;

}

static void User_BasicTim_Init(void)
{
	RCC->APB1ENR |= RCC_APB1ENR_TIM12EN;

	TIM12->PSC = 9; // 84MHz clock, divided by 9+1 = 8.4MHz
	TIM12->ARR = 8400; // 8.4MHz / 8400 = 1kHz clock

	NVIC_SetPriority(TIM8_BRK_TIM12_IRQn,PRIO_APPTIMER);
	NVIC_EnableIRQ(TIM8_BRK_TIM12_IRQn);

	TIM12->DIER = TIM_DIER_UIE;
	TIM12->CR1 = TIM_CR1_CEN;

}

void SYSTICK_IRQHandler(void)
{
	g_MainSysTick++;
	if((g_MainSysTick % SERIAL_DATA_RATE) == 0)
	{
		g_MainFlags |= MAINFLAG_SERIALDATA;
	}
}

/*
void User_HallTIM_IRQ(void)
{
	HallSensor_UpdateCallback();
}
*/

// TIM1 overflow / update IRQ (20kHz)
void User_PWMTIM_IRQ(void)
{
	// Debug: Blink RLED to show how much processor time is used
	RLED_PORT->BSRR = (1 << RLED_PIN);

	float fangle, ipark_a, ipark_b;
	float tAf, tBf, tCf;
	uint16_t tA, tB, tC;
	float Ia, Ib, Ic;
	float clarke_alpha, clarke_beta;
	float park_d, park_q;

	// Generate ramp angle
	// dfsl_rampgen(&g_rampAngle, g_rampInc); // Can use the dfsl library later, following code is simpler
	if(g_rampdir == 0)
	{
		g_rampAngle += g_rampInc;
	}
	else
	{
		g_rampAngle -= g_rampInc;
	}
	// Update Hall sensor angle
	HallSensor_Inc_Angle();

	/* Connections:
	 * Everything connected in final configuration
	 * Hall sensor angle->Ipark
	 * Vd, Vq feedback PIDs ->Ipark
	 * Ipark->SVM
	 * Motor currents->ADCs
	 * ADC results->Clarke
	 * Clarke Outputs, Hall sensor angle ->Park
	 * Park -> Vd, Vq feedback PIDs
	 * Motor is connected.
	 * Make sure that current does not exceed safe levels!
	 * Place series power resistors!
	 * Heatsinks on the power FETs!
	 */

// **************** FORWARD PATH *****************
	// Read angle from Hall sensors
	fangle = ((float)HallSensor_Get_Angle())/65536.0f;
	// Feed to inverse Park
	//dfsl_iparkf(Id_control.Out,Iq_control.Out,fangle,&ipark_a, &ipark_b);
	dfsl_iparkf(0, Throttle_cmd, fangle, &ipark_a, &ipark_b);
	// Inverse Park outputs to space vector modulation, output three-phase waveforms
	dfsl_svmf(ipark_a, ipark_b, &tAf, &tBf, &tCf);
	// Convert from floats to 16-bit ints
	tA = (uint16_t)(tAf*65535.0f);
	tB = (uint16_t)(tBf*65535.0f);
	tC = (uint16_t)(tCf*65535.0f);

	// Is throttle at zero? Motor off
	if(Throttle_cmd <= 0.0f)
	{
		PWM_MotorOFF();
		Throttle_cmd = 0.0f;
	}
	if(Throttle_cmd > 0.0f)
	{
		PWM_MotorON();
	}

	// Set PWM duty cycles
	PWM_SetDuty(tA,tB,tC);


// **************** FEEDBACK PATH *****************
	// Transform sensor readings
	Ia = adcGetCurrent(ADC_IA);
	Ib = adcGetCurrent(ADC_IB);
	Ic = adcGetCurrent(ADC_IC);
	dfsl_clarkef(Ia, Ib, &clarke_alpha, &clarke_beta);
	dfsl_parkf(clarke_alpha,clarke_beta,fangle, &park_d, &park_q);
	// Input feedbacks to the Id and Iq controllers
	// Filter the currents
	Id_Filt.X = park_d;
	Iq_Filt.X = park_q;
	dfsl_biquadf(&Id_Filt);
	dfsl_biquadf(&Iq_Filt);
	// Pass filtered current to the PI(D)s
	Id_control.Err = 0.0f - park_d;
	Iq_control.Err = (3.0f)*Throttle_cmd - park_q;
	//Id_control.Err = 0.0f - Id_Filt.Y;
	//Iq_control.Err = (3.0f)*Throttle_cmd - Iq_Filt.Y;
	// Don't integrate unless the throttle is active
	if(Throttle_cmd > 0.0f)
	{
		dfsl_pidf(&Id_control);
		dfsl_pidf(&Iq_control);
	}

	// DAC debugging outputs
	DAC->DHR12L1 = g_rampAngle;
	DAC->DHR12L2 = (uint16_t)(fangle*65536.0f);

	// USB Debugging outputs
	usbdacvals[0] = (uint32_t)((Ia+5.0f)*6553.6f);
	usbdacvals[1] = (uint32_t)((Ib+5.0f)*6553.6f);
	usbdacvals[2] = (uint32_t)((Ic+5.0f)*6553.6f);
	usbdacvals[3] = tA;
	usbdacvals[4] = tB;
	usbdacvals[5] = tC;
	usbdacvals[6] = (uint16_t)(Throttle_cmd * 65536.0f);
	usbdacvals[7] = g_rampAngle;
	usbdacvals[8] = HallSensor_Get_Angle();
	usbdacvals[9] = HallSensor_Get_Speed();
	if(g_rampAngle > usbdacvals[8])
		usbdacvals[10] = (g_rampAngle - usbdacvals[8]);
	else
		usbdacvals[10] = (usbdacvals[8] - g_rampAngle);
	usbdacvals[11] = (uint32_t)((park_d+5.0f)*6553.6f);
	usbdacvals[12] = (uint32_t)((park_q+5.0f)*6553.6f);
	usbdacvals[13] = (uint32_t)((Id_Filt.Y + 5.0f)*6553.6f);
	usbdacvals[14] = (uint32_t)((Iq_Filt.Y + 5.0f)*6553.6f);
	//usbdacvals[13] = (uint32_t)((Id_control.Out+5.0f)*6553.6f);
	//usbdacvals[14] = (uint32_t)((Iq_control.Out+5.0f)*6553.6f);
	usbdacvals[15] = g_errorCode;
	usbdacvals[16] = adcRaw(ADC_VREFINT);

	RLED_PORT->BSRR = (1 << (RLED_PIN+16));
}

// Simple application timer (1kHz)
void User_BasicTIM_IRQ(void)
{

	raw_throttle = adcGetThrottle();
	Throttle_cmd = throttle_process(raw_throttle);

	// Trim to 99%
	if(Throttle_cmd >= 1.0f)
		Throttle_cmd = 0.99f;

	// Blink the LEDs
	g_ledcount++;
	if(g_ledcount > MAXLEDCOUNT)
	{
		Toggle_Leds();
		g_ledcount = 0;
	}
	// Debounce the user pushbutton

	systick_debounce_counter++;
	if(systick_debounce_counter >= DEBOUNCE_INTERVAL)
	{
		systick_debounce_counter = 0;

		// Part 1: Accumulated the debounce integrator
		//if(HAL_GPIO_ReadPin(PB_PORT, PB_PIN) == GPIO_PIN_SET)
		if((PB_PORT->IDR & (1<<PB_PIN)) == (1<<PB_PIN))
		{
			if(debounce_integrator > 0)
				debounce_integrator--;
		}
		else
		{
			if(debounce_integrator < DEBOUNCE_MAX)
				debounce_integrator++;
		}
		// Part 2: Change pushbutton state if integrator is at zero or its maximum
		if(debounce_integrator > DEBOUNCE_MAX)
		{
			// Defensive programming!
			debounce_integrator = DEBOUNCE_MAX;
		}
		if(debounce_integrator == DEBOUNCE_MAX)
		{
			pb_state = PB_PRESSED;
		}
		if(debounce_integrator == 0)
		{
			pb_state = PB_RELEASED;
		}
	}
}

void MAIN_SetUSBDebugOutput(uint8_t outputnum, uint8_t valuenum)
{
	// Set the new output
	usbdacassignments[outputnum] = valuenum+1;
}

void MAIN_SetUSBDebugging(uint8_t on_or_off)
{
	if(on_or_off == 0)
		data_out = 0;
	else
		data_out = 1;
}

void MAIN_SetRampSpeed(uint32_t newspeed)
{
	g_rampInc = dfsl_rampctrl(RAMP_CALLFREQ, newspeed);
}

void MAIN_SetRampDir(uint8_t forwardOrBackwards)
{
	if(forwardOrBackwards == 0)
	{
		// Forwards!
		g_rampdir = 0;
	}
	else
	{
		g_rampdir = 1;
	}
}

void MAIN_SetVar(uint8_t var, float newval)
{
	switch(var)
	{
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
}

void MAIN_SetError(uint32_t errorCode)
{
	g_errorCode |= errorCode;
}

void MAIN_SoftReset(uint8_t restartInBootloader)
{
	// Absolutely no PWM should be happening right now!
	PWM_MotorOFF();
	if(restartInBootloader)
	{
		// Enable access to backup registers
		BackupEnable();
		// Set the bootloader flag in backup register 0
		RTC->BKP0R = BOOTLOADER_RESET_FLAG;
	}

	// Trigger a software reset
	NVIC_SystemReset();
}

/* Convert num into ASCII in base 10
 * Result in buf as a null terminated string
 * Returns number of digits in buf, including
 * negative sign when present.
 */
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

void Delay(__IO uint32_t Delay)
{
  uint32_t tickstart = 0;
  tickstart = g_MainSysTick;
  while((g_MainSysTick - tickstart) < Delay)
  {
  }
}

uint32_t GetTick(void)
{
	return g_MainSysTick;
}
