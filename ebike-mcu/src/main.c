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


uint32_t usbdac1val, usbdac2val, usbdac3val, usbdac4val, usbdac5val;

uint32_t systick_debounce_counter;
uint8_t debounce_integrator;
volatile PB_TypeDef pb_state;
uint8_t data_out;

//extern uint16_t adc_conv[NUM_ADC_CH];

PID_Float_Type Id_control, Iq_control;
Biquad_Float_Type Throttle_filt=BIQ_LPF_DEFAULTS;
float Throttle_cmd;

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
	User_PB_Init();
	Throttle_cmd = 0.75f;
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
  g_rampInc = dfsl_rampctrl(20000, 2);

  /* Initialize PID controllers */
  dfsl_pid_defaultsf(&Id_control);
  dfsl_pid_defaultsf(&Iq_control);

  /* Run Application (Interrupt mode) */
  while (1)
  {
    //Toggle_Leds();
    if(VCP_read(&byte, 1) != 0)
    	{
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
    	sprintf(string,"%d %d %d %d %d\r\n",(int)usbdac1val,(int)usbdac2val,(int)usbdac3val,(int)usbdac4val,(int)usbdac5val);
    	//sprintf(string,"%d\r\n",(int)usbdac5val);
    	VCP_write(string,strlen(string));
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

#if PHASE == 1
	/* Phase one connections
	 * Ramp Generator->Ipark
	 * Vd_testing, Vq_testing ->Ipark
	 * Ipark->SVM
	 *
	 * No current feedback, no ADCs, no motor connected!
	 */
	float fangle, ipark_a, ipark_b;
	float tAf, tBf, tCf;
	uint16_t tA, tB, tC;
	// Generate ramp angle
	dfsl_rampgen(&g_rampAngle, g_rampInc);
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
	DAC->DHR12L1 = (uint16_t)(Throttle_filt.Y * 16384.0f); // Displays 1-4 volts
	//DAC->DHR12L1 = HallSensor_Get_Angle();
	DAC->DHR12L2 = HallSensor_Get_Speed()>>8;
	// USB debugging outputs
	usbdac1val = g_rampAngle;
	usbdac2val = tA;
	usbdac3val = tB;
	usbdac4val = tC;
	usbdac5val = DAC->DHR12L1;
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
	float fangle, ipark_a, ipark_b;
	float tAf, tBf, tCf;
	int16_t tA, tB, tC;
	float Ia, Ib, Ic;
	float clarke_alpha, clarke_beta;
	float park_d, park_q;
	// Generate ramp angle
	dfsl_rampgen(&g_rampAngle, g_rampInc);
	fangle = ((float)g_rampAngle)/65536.0f;
	// Feed ramp angle with fixed throttle to inverse Park
	dfsl_iparkf(0.0f,Throttle_cmd,fangle,&ipark_a, &ipark_b);
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

	// DAC debugging outputs
	DAC->DHR12L1 = adc_conv[ADC_IA]<<4;
	DAC->DHR12L2 = adc_conv[ADC_IB]<<4;
	// USB debugging outputs
	usbdac1val = g_rampAngle;
	usbdac2val = tA;
	usbdac3val = adc_conv[ADC_IA]<<4;
	usbdac4val = adc_conv[ADC_IB]<<4;
	usbdac5val = (uint16_t)(6553.60f*((Ia) + 5.0f));
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
	float fangle, ipark_a, ipark_b;
	float tAf, tBf, tCf;
	int16_t tA, tB, tC;
	float Ia, Ib, Ic;
	float clarke_alpha, clarke_beta;
	float park_d, park_q;
	// Generate ramp angle
	dfsl_rampgen(&g_rampAngle, g_rampInc);
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
	// USB debugging outputs
	usbdac1val = g_rampAngle;
	usbdac2val = tA;
	usbdac3val = (uint16_t)(6553.60f*((park_d) + 5.0f));
	usbdac4val = (uint16_t)(6553.60f*((park_q) + 5.0f));
	usbdac5val = (uint16_t)(6553.60f*((Ia) + 5.0f));
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
	float fangle, ipark_a, ipark_b;
	float tAf, tBf, tCf;
	int16_t tA, tB, tC;
	float Ia, Ib, Ic;
	float clarke_alpha, clarke_beta;
	float park_d, park_q;
	// Generate ramp angle
	dfsl_rampgen(&g_rampAngle, g_rampInc);
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
	// USB debugging outputs
	usbdac1val = g_rampAngle;
	usbdac2val = DAC->DHR12L2;
	usbdac3val = (uint16_t)(6553.60f*((park_d) + 5.0f));
	usbdac4val = (uint16_t)(6553.60f*((park_q) + 5.0f));
	usbdac5val = (uint16_t)(6553.60f*((Ia) + 5.0f));
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
	float fangle, ipark_a, ipark_b;
	float tAf, tBf, tCf;
	int16_t tA, tB, tC;
	float Ia, Ib, Ic;
	float clarke_alpha, clarke_beta;
	float park_d, park_q;

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
	// USB debugging outputs
	usbdac1val = g_rampAngle;
	usbdac2val = DAC->DHR12L2;
	usbdac3val = (uint16_t)(6553.60f*((park_d) + 5.0f));
	usbdac4val = (uint16_t)(6553.60f*((park_q) + 5.0f));
	usbdac5val = (uint16_t)(6553.60f*((Ia) + 5.0f));
#endif
}

// Simple application timer (1kHz)
void User_BasicTIM_IRQ(void)
{
	// Read and filter throttle
	Throttle_filt.X = adcGetThrottle();
	dfsl_biquadf(&Throttle_filt);

	// Convert throttle to percent
	Throttle_cmd = (Throttle_filt.Y - THROTTLE_MIN) * THROTTLE_SCALE;

	// Is throttle at zero? Motor off
	if(Throttle_cmd <= 0.0f)
	{
		PWM_MotorOFF();
		Throttle_cmd = 0.0f;
	}
	if(Throttle_cmd > 0.0f)
	{
		PWM_MotorON();
		if(Throttle_cmd > 1.0f)
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
