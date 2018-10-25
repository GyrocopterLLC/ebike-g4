#ifndef PERIPHCONFIG_H_
#define PERIPHCONFIG_H_

/* Peripheral usage:
 * TIM1 - 3phase bridge PWM output
 * TIM2 -
 * TIM3 - Hall sensors
 * TIM4 -
 * TIM5 -
 * TIM6 -
 * TIM7 -
 * TIM8 - Hall sampling timer
 * TIM9 -
 * TIM10 -
 * TIM11 -
 * TIM12 - Basic timer for application scheduling
 * TIM13 - PAS timer for Thr1
 * TIM14 - PAS timer for Thr2
 * USART1 -
 * USART2 - Battery management system (BMS)
 * USART3 - Handle bar display (HBD)
 * UART4 -
 * UART5 -
 * USART6 -
 */

#define APB1_CLK      84000000
#define APB2_CLK      168000000
#define TIM1_CLK      APB2_CLK
#define TIM2_CLK      APB1_CLK
#define TIM3_CLK      APB1_CLK
#define TIM4_CLK      APB1_CLK
#define TIM5_CLK      APB1_CLK
#define TIM6_CLK      APB1_CLK
#define TIM7_CLK      APB1_CLK
#define TIM8_CLK      APB1_CLK
#define TIM9_CLK      APB2_CLK
#define TIM10_CLK     APB2_CLK
#define TIM11_CLK     APB2_CLK
#define TIM12_CLK     APB1_CLK
#define TIM13_CLK     APB1_CLK
#define TIM14_CLK     APB1_CLK

// PWM
#define PWM_TIMER             TIM1
#define PWM_CLK               TIM1_CLK
#define PWM_TIM_CLK_ENABLE()  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN
#define PWM_IRQn              TIM1_UP_TIM10_IRQn

// Hall Effect Sensor
#define HALL_TIMER                      TIM3
#define HALL_CLK                        TIM3_CLK
#define HALL_TIM_CLK_ENABLE()           RCC->APB1ENR |= RCC_APB1ENR_TIM3EN
#define HALL_IRQn                       TIM3_IRQn
#define HALL_SAMPLE_TIMER               TIM8
#define HALL_SAMPLE_TIMER_CLK_ENABLE()  RCC->APB2ENR |= RCC_APB2ENR_TIM8EN
#define HALL_DMA                        DMA2_Stream1
#define HALL_DMA_CLK_ENABLE()           RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN

// Throttle (PAS timers)
#define PAS1_TIM                  TIM13
#define PAS1_CLK                  TIM13_CLK
#define PAS2_TIM                  TIM14
#define PAS2_CLK                  TIM14_CLK
#define PAS1_TIMER_CLK_ENABLE()   RCC->APB1ENR |= RCC_APB1ENR_TIM13EN
#define PAS2_TIMER_CLK_ENABLE()   RCC->APB1ENR |= RCC_APB1ENR_TIM14EN
#define PAS1_PINCHANGE_IRQn       EXTI9_5_IRQn
#define PAS2_PINCHANGE_IRQn       EXTI0_IRQn
#define PAS1_TIMER_IRQn           TIM8_UP_TIM13_IRQn
#define PAS2_TIMER_IRQn           TIM8_TRG_COM_TIM14_IRQn

// Application timer
#define APP_TIM                   TIM12
#define APP_CLK                   TIM12_CLK
#define APP_TIMER_CLK_ENABLE()    RCC->APB1ENR |= RCC_APB1ENR_TIM12EN
#define APP_IRQn                  TIM8_BRK_TIM12_IRQn

// BMS
#define BMS_UART              USART2
#define BMS_UART_CLK_ENABLE() RCC->APB1ENR |= RCC_APB1ENR_USART2EN
#define BMS_IRQn              USART2_IRQn

// HBD
#define HBD_UART              USART3
#define HBD_UART_CLK_ENABLE() RCC->APB1ENR |= RCC_APB1ENR_USART3EN
#define HBD_IRQn              USART3_IRQn


#endif // PERIPHCONFIG_H_
