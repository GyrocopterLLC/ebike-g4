#ifndef DELI_HALL_H_
#define DELI_HALL_H_
#if 0
/* deli_hall.c
 * David's Ebike Library:
 * Hall Sensor Motor Positioning
 */

/* ====== Board defines ====== */
#define HALL_TIMER_ENABLE_BIT	RCC_APB1ENR_TIM3EN
#define HALL_GPIO_ENABLE_BIT	RCC_AHB1ENR_GPIOCEN
#define HALL_PORT				GPIOC
#define HALL_TIM				TIM3
#define HALL_IRQ				TIM3_IRQn

#define HALL_PIN_A_NUM			6
#define HALL_PIN_A				0x0040
#define HALL_PIN_B_NUM			7
#define HALL_PIN_B				0x0080
#define HALL_PIN_C_NUM			8
#define HALL_PIN_C				0x0100
#define HALL_GPIO_AF			2
#define HALL_PIN_A_AFR			0
#define HALL_PIN_B_AFR			0
#define HALL_PIN_C_AFR			1
#define HALL_PIN_A_AFR_OFFSET	24
#define HALL_PIN_B_AFR_OFFSET	28
#define HALL_PIN_C_AFR_OFFSET	0

/* ==== Hall Sensor Settings ==== */

#define HALL_TIMER_FREQ				84000000
#define HALL_PREEMPT_PRIORITY		2
#define HALL_SUB_PRIORITY			0
#define HALL_PSC_CHANGE				4
#define HALL_MAX_OVERFLOWS			3
#define HALL_MAX_PERIOD				0xFFFF
#define HALL_MAX_PSC				127 // Timer overflow ~= 0.1sec
#define HALL_MIN_PSC				3	// Timer overflow ~= 3.1ms
#define HALL_CAPTURE_MIN			0x5555
#define HALL_SPEED_FXD_SCALE		4


/* ======= Hall Sensor Structure ====== */
typedef struct
{
	uint32_t Status;			/* Status flags */
	uint32_t Speed;				/* Speed of the motor in Hz. The speed is fixed-point, scaled
								 * by the definition above. */
	uint32_t OverflowCounter;	/* Overflow counter: number of times the overflow ISR has been
								 * triggered between capture ISR triggers. */
	uint32_t UpdateRate;		/* Calling frequency of the angle update function. Scaled by the value
								 * in deli_math.h */
	uint16_t Angle;				/* Angle of the motor in fractional form. The 16-bit value ranges from
								 * 0 to 65535. This is mapped to 360 degrees, e.g. 0=0deg, 65535=359.995deg */
	uint8_t  HallState;			/* Hall Effect sensor state - 0 to 7 (where 1-6 are valid states) */
} HallSensorType;
#endif /* 0 */
#endif /* DELI_HALL_H_ */
