/**
  ******************************************************************************
  * @file    USB_Device/CDC_Standalone/Src/main.c
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    26-December-2014
  * @brief   USB device CDC application main file
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
  
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
USBD_HandleTypeDef  USBD_Device;
DAC_HandleTypeDef	hdac;
TIM_HandleTypeDef	hBasicTim;
uint16_t g_rampAngle;
uint16_t g_rampInc;
uint32_t g_ledcount;

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
 * 10 - Spare
 * 11 - Spare
 * 12 - Spare
 * 13 - Spare
 * 14 - Spare
 * 15 - Spare
 *
 */
#define MAX_USB_VALS	16
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
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
	char byte;
	char string[32];
	char usbstring[64];
	User_PB_Init();
	Throttle_cmd = 0.0f;
	data_out = 0;
	//if(HAL_GPIO_ReadPin(PB_PORT, PB_PIN) == GPIO_PIN_RESET)
	if((PB_PORT->IDR & (1<<PB_PIN)) == 0)
	{
		// Make sure it really is
		byte = 100;
		while(byte > 0)
		{
			byte--;
			//if(HAL_GPIO_ReadPin(PB_PORT, PB_PIN) == GPIO_PIN_SET)
			if((PB_PORT->IDR & (1<<PB_PIN)) == (1<<PB_PIN))
			{
				break;
			}
		}
		if(byte == 0)
		{
			// Load bootloader
			__set_MSP(0x2001FFFF); // Change stack pointer to the end of ram

			// Make a function pointer to the bootloader start address
			void (*bootloader)(void) = (void(*)(void)) *((uint32_t *)(0x1FFF0004));
			// Breaking down that previous line...
			// Before the equals sign: declare a pointer to a function with no input arguments and no return value.
			// After the equals sign: the (void(*)(void)) part casts the following value to a function pointer type
			// the (uint32_t *) casts the following value (0x1FFF0004) to a unsigned integer pointer type
			// the whole *((uint32_t *)(#0x1FFF0004#)) portion returns the value pointed at by the address stored
			// at the 0x1FFF0004 location
			// This allows the bootloader function pointer to take the address stored at location 0x1FFF0004 as its own
			// address.

			// Call the bootloader...
			bootloader();


		}
	}

	  /* Configure the system clock to 168 MHz */
	  SystemClock_Config();

  /* STM32F4xx HAL library initialization:
       - Configure the Flash prefetch, instruction and Data caches
       - Configure the Systick to generate an interrupt each 1 msec
       - Set NVIC Group Priority to 4
       - Global MSP (MCU Support Package) initialization
     */
  HAL_Init();

  
  /* Configure LED1, LED2, LED3 and LED4 */
  /* David's version */
  User_LED_Init();
  //GPIOD->ODR |= GPIO_PIN_12|GPIO_PIN_15;
  GLED_PORT->ODR |= (1<<GLED_PIN);
  User_DAC_Init();
  User_BasicTim_Init();
  //PWM_Init();
  PWM_Init_NoHal();
  HallSensor_Init_NoHal(20000);
  HBD_Init();

  adcInit();
  
  /* Init Device Library */
  USBD_Init(&USBD_Device, &VCP_Desc, 0);
  
  /* Add Supported Class */
  USBD_RegisterClass(&USBD_Device, USBD_CDC_CLASS);
  
  /* Add CDC Interface Class */
  USBD_CDC_RegisterInterface(&USBD_Device, &USBD_CDC_fops);
  
  /* Start Device Process */
  USBD_Start(&USBD_Device);
  
  /* Initialize ramp angle increment */
  g_rampInc = dfsl_rampctrl(20000, 5);

  /* Initialize PID controllers */
  dfsl_pid_defaultsf(&Id_control);
  dfsl_pid_defaultsf(&Iq_control);

  /* Default USB debugging outputs */
  usbdacassignments[0] = 1; // Ia
  usbdacassignments[1] = 2; // Ib
  usbdacassignments[2] = 3; // Ic
  usbdacassignments[3] = 7; // Throttle
  usbdacassignments[4] = 9; // Hall angle

  /* Run Application (Interrupt mode) */
  while (1)
  {
	  uint8_t vcp_buf_len;
    //Toggle_Leds();
    if(VCP_read(&byte, 1) != 0)
    {
    	// Echo it
    	VCP_write(&byte, 1);
    	// Add it to the VCP buffer
    	vcp_buf_len = strlen(vcp_buffer);
    	if(vcp_buf_len < (UI_MAX_BUFFER_LENGTH - 1))
    	{
    		vcp_buffer[vcp_buf_len + 1] = byte;
    		vcp_buffer[vcp_buf_len + 2] = 0;
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
    			VCP_write(UI_SendBuf(), UI_RespLen());
    		}
    	}

    	/*
    		VCP_write("You wrote\r\n",11);
    		VCP_write(&byte, 1);
    		VCP_write("\r\n",2);
    		//Toggle_Leds();
    		sprintf(string,"***tA: %d\r\n",(int)(TIM1->CCR1));
    		VCP_write(string,strlen(string));
    		sprintf(string,"***tB: %d\r\n",(int)(TIM1->CCR2));
			VCP_write(string,strlen(string));
			sprintf(string,"***tC: %d\r\n",(int)(TIM1->CCR3));
			VCP_write(string,strlen(string));
		*/
    }
    if(HBD_Receive(&byte, 1) != 0)
    {
    	// Read a byte from the HBD UART
    	UI_Process(byte);
    }
    if(pb_state == PB_PRESSED)
    {
    	// Wait until release
    	while(pb_state == PB_PRESSED) {}
    	// Change data output state
    	data_out = ~data_out;
    }
    if(data_out != 0)
    {
    	usbstring[0] = 0;
    	for(uint8_t i = 0; i < MAX_USB_OUTPUTS; i++)
    	{
    		if(usbdacassignments[i] == 0)
    		{
    			strcat(usbstring,"0 ");
    		}
    		else
    		{
    			sprintf(string,"%d ",(int)usbdacvals[usbdacassignments[i]-1]);
    			strcat(usbstring,string);
    		}
    		/*
    		for(uint8_t j = 0; j < MAX_USB_VALS; j++)
    		{
    			if(usbdacassignments[j] == i)
    			{
    				sprintf(string,"%d ",(int)usbdacvals[j]);
    				//VCP_write(string,strlen(string));
    				strcat(usbstring,string);
    			}
    		}
    		*/
    	}

    	strcat(usbstring,"\r\n");

    	VCP_write(usbstring,strlen(usbstring));

    	HAL_Delay(20);
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
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();

  /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* Configure RCC Oscillators: All parameters can be changed according to user’s needs */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  HAL_RCC_OscConfig (&RCC_OscInitStruct);
  
  /* RCC Clocks: All parameters can be changed according to user’s needs */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK |RCC_CLOCKTYPE_HCLK |RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
  
  SystemCoreClockUpdate();

  /* STM32F405x/407x/415x/417x Revision Z devices: prefetch is supported  */
  if (HAL_GetREVID() == 0x1001)
  {
    /* Enable the Flash prefetch */
    __HAL_FLASH_PREFETCH_BUFFER_ENABLE();
  }
}

/**
  * @brief  Toggles LEDs.
  * @param  None
  * @retval None
  */
static void Toggle_Leds(void)
{
	RLED_PORT->ODR ^= (1<<RLED_PIN);
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
	__HAL_RCC_DAC_CLK_ENABLE();
	GPIO_Clk(DAC_PORT);

	GPIO_Analog(DAC_PORT,DAC1_PIN);
	GPIO_Analog(DAC_PORT,DAC2_PIN);

	DAC->CR = DAC_CR_BOFF2 | DAC_CR_BOFF1 | DAC_CR_EN2 | DAC_CR_EN1;

}

static void User_BasicTim_Init(void)
{
	__HAL_RCC_TIM12_CLK_ENABLE();

	TIM12->PSC = 9; // 84MHz clock, divided by 9+1 = 8.4MHz
	TIM12->ARR = 8400; // 8.4MHz / 8400 = 1kHz clock

	NVIC_SetPriority(TIM8_BRK_TIM12_IRQn,3);
	NVIC_EnableIRQ(TIM8_BRK_TIM12_IRQn);

	TIM12->DIER = TIM_DIER_UIE;
	TIM12->CR1 = TIM_CR1_CEN;

}

void HAL_SYSTICK_Callback(void)
{

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
	float fangle, ipark_a, ipark_b;
	float tAf, tBf, tCf;
	int16_t tA, tB, tC;
	float Ia, Ib, Ic;
	float clarke_alpha, clarke_beta;
	float park_d, park_q;

	// For all build phases
	// Generate ramp angle
	dfsl_rampgen(&g_rampAngle, g_rampInc);
	// Update Hall sensor angle
	HallSensor_Inc_Angle();
#if PHASE == 1
	/* Phase one connections
	 * Ramp Generator->Ipark
	 * Vd_testing, Vq_testing ->Ipark
	 * Ipark->SVM
	 *
	 * No current feedback, no ADCs, no motor connected!
	 */
	// Rotor angle comes from ramp generator
	fangle = ((float)g_rampAngle)/65536.0f;
	// Feed ramp angle with fixed throttle to inverse Park
	dfsl_iparkf(0.0f,Throttle_cmd,fangle,&ipark_a, &ipark_b);
	// Inverse Park outputs to space vector modulation, output three-phase waveforms
	dfsl_svmf(ipark_a, ipark_b, &tAf, &tBf, &tCf);
	// Convert from floats to 16-bit ints
	tA = (uint16_t)(tAf*65535.0f);
	tB = (uint16_t)(tBf*65535.0f);
	tC = (uint16_t)(tCf*65535.0f);
	// Set PWM duty cycles
	PWM_SetDuty(tA,tB,tC);
	// DAC debugging outputs
	DAC->DHR12L1 = (uint16_t)(Throttle_cmd * 65536.0f); // Displays 0-1 volts
	//DAC->DHR12L1 = HallSensor_Get_Angle();
	DAC->DHR12L2 = HallSensor_Get_Speed()>>8;

#endif
#if PHASE == 2
	/* Phase two connections
	 * Ramp Generator->Ipark
	 * Vd_testing, Vq_testing ->Ipark
	 * Ipark->SVM
	 * Motor currents->ADCs
	 * ADC results->Clarke
	 * Clarke Outputs, Ramp Generator ->Park
	 *
	 * No current feedback, motor is connected.
	 * Make sure that current does not exceed safe levels!
	 * Place series power resistors!
	 * Heatsinks on the power FETs!
	 */
	// Rotor angle comes from ramp generator
	fangle = ((float)g_rampAngle)/65536.0f;
	// Feed ramp angle with fixed throttle to inverse Park
	dfsl_iparkf(0.0f,Throttle_cmd,fangle,&ipark_a, &ipark_b);
	// Inverse Park outputs to space vector modulation, output three-phase waveforms
	dfsl_svmf(ipark_a, ipark_b, &tAf, &tBf, &tCf);
	// Convert from floats to 16-bit ints
	tA = (uint16_t)(tAf*65535.0f);
	tB = (uint16_t)(tBf*65535.0f);
	tC = (uint16_t)(tCf*65535.0f);
	// Set PWM duty cycles
	PWM_SetDuty(tA,tB,tC);
	// Transform sensor readings
	Ia = adcGetCurrent(ADC_IA);
	Ib = adcGetCurrent(ADC_IB);
	Ic = adcGetCurrent(ADC_IC);
	dfsl_clarkef(Ia, Ib, &clarke_alpha, &clarke_beta);
	dfsl_parkf(clarke_alpha,clarke_beta,fangle, &park_d, &park_q);

	// DAC debugging outputs
	DAC->DHR12L1 = adcRawCurrent(ADC_IA)<<4;
	DAC->DHR12L2 = adcRawCurrent(ADC_IB)<<4;

#endif
#if PHASE == 3
	/* Phase three connections
	 * Ramp Generator->Ipark
	 * Vd, Vq feedback PIDs ->Ipark
	 * Ipark->SVM
	 * Motor currents->ADCs
	 * ADC results->Clarke
	 * Clarke Outputs, Ramp Generator ->Park
	 * Park -> Vd, Vq feedback PIDs
	 * Motor is connected.
	 * Make sure that current does not exceed safe levels!
	 * Place series power resistors!
	 * Heatsinks on the power FETs!
	 */
	// Rotor angle comes from ramp generator
	fangle = ((float)g_rampAngle)/65536.0f;
	// Feed ramp angle with feedback to inverse Park
	dfsl_iparkf(Id_control.Out,Iq_control.Out,fangle,&ipark_a, &ipark_b);
	// Inverse Park outputs to space vector modulation, output three-phase waveforms
	dfsl_svmf(ipark_a, ipark_b, &tAf, &tBf, &tCf);
	// Convert from floats to 16-bit ints
	tA = (int16_t)(tAf*65535.0f);
	tB = (int16_t)(tBf*65535.0f);
	tC = (int16_t)(tCf*65535.0f);
	// Set PWM duty cycles
	PWM_SetDuty(tA,tB,tC);
	// Transform sensor readings
	Ia = adcGetCurrent(ADC_IA);
	Ib = adcGetCurrent(ADC_IB);
	Ic = adcGetCurrent(ADC_IC);
	dfsl_clarkef(Ia, Ib, &clarke_alpha, &clarke_beta);
	dfsl_parkf(clarke_alpha,clarke_beta,fangle, &park_d, &park_q);
	// Input feedbacks to the Id and Iq controllers
	Id_control.Err = 0.0f - park_d;
	Iq_control.Err = Throttle_cmd - park_q;
	dfsl_pidf(&Id_control);
	dfsl_pidf(&Iq_control);

	// DAC debugging outputs
	DAC->DHR12L1 = (uint16_t)(6553.60f*((park_d) + 5.0f));
	DAC->DHR12L2 = (uint16_t)(6553.60f*((park_q) + 5.0f));

#endif
#if PHASE == 4
	/* Phase four connections
	 * Ramp Generator->Ipark
	 * Vd, Vq feedback PIDs ->Ipark
	 * Ipark->SVM
	 * Motor currents->ADCs
	 * ADC results->Clarke
	 * Clarke Outputs, Ramp Generator ->Park
	 * Park -> Vd, Vq feedback PIDs
	 * Motor is connected.
	 * Make sure that current does not exceed safe levels!
	 * Place series power resistors!
	 * Heatsinks on the power FETs!
	 */
	// Rotor angle comes from ramp generator
	fangle = ((float)g_rampAngle)/65536.0f;
	// Feed ramp angle with feedback to inverse Park
	dfsl_iparkf(Id_control.Out,Iq_control.Out,fangle,&ipark_a, &ipark_b);
	// Inverse Park outputs to space vector modulation, output three-phase waveforms
	dfsl_svmf(ipark_a, ipark_b, &tAf, &tBf, &tCf);
	// Convert from floats to 16-bit ints
	tA = (int16_t)(tAf*65535.0f);
	tB = (int16_t)(tBf*65535.0f);
	tC = (int16_t)(tCf*65535.0f);
	// Set PWM duty cycles
	PWM_SetDuty(tA,tB,tC);
	// Transform sensor readings
	Ia = adcGetCurrent(ADC_IA);
	Ib = adcGetCurrent(ADC_IB);
	Ic = adcGetCurrent(ADC_IC);
	dfsl_clarkef(Ia, Ib, &clarke_alpha, &clarke_beta);
	dfsl_parkf(clarke_alpha,clarke_beta,fangle, &park_d, &park_q);
	// Input feedbacks to the Id and Iq controllers
	Id_control.Err = 0.0f - park_d;
	Iq_control.Err = Throttle_cmd - park_q;
	dfsl_pidf(&Id_control);
	dfsl_pidf(&Iq_control);

	// DAC debugging outputs
	DAC->DHR12L1 = g_rampAngle;
	DAC->DHR12L2 = HallSensor_Get_Angle();

#endif
#if PHASE == 5
	/* Phase five connections
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
	dfsl_iparkf(Id_control.Out,Iq_control.Out,fangle,&ipark_a, &ipark_b);
	// Inverse Park outputs to space vector modulation, output three-phase waveforms
	dfsl_svmf(ipark_a, ipark_b, &tAf, &tBf, &tCf);
	// Convert from floats to 16-bit ints
	tA = (int16_t)(tAf*65535.0f);
	tB = (int16_t)(tBf*65535.0f);
	tC = (int16_t)(tCf*65535.0f);
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
	Id_control.Err = 0.0f - park_d;
	Iq_control.Err = Throttle_cmd - park_q;
	dfsl_pidf(&Id_control);
	dfsl_pidf(&Iq_control);

	// DAC debugging outputs
	DAC->DHR12L1 = g_rampAngle;
	DAC->DHR12L2 = (uint16_t)(fangle*65536.0f);

#endif

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
}

// Simple application timer (1kHz)
void User_BasicTIM_IRQ(void)
{
	raw_throttle = adcGetThrottle();
	Throttle_cmd = throttle_process(raw_throttle);

	// Is throttle at zero? Motor off
	if(Throttle_cmd <= 0.0f)
	{
		PWM_MotorOFF();
		Throttle_cmd = 0.0f;
	}
	if(Throttle_cmd > 0.0f)
	{
		PWM_MotorON();
		if(Throttle_cmd >= 1.0f)
			Throttle_cmd = 0.99f;
	}

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

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
