/******************************************************************************
 * Filename: hallSensor.c
 * Description: Reads 3-input Hall effect sensors commonly used with BLDC
 *              motors. Each sensor changes polarity at 180 degree increments,
 *              and the sensors are spaced 60 degrees apart. The sensors
 *              give the position of the rotor to the nearest 60 degree sector.
 *              For higher resolution, the functions in this file count the
 *              time between sensor changes, and interpolate the motor angle
 *              in between actual sensor value flips.
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

#include "hallSensor.h"
#include "gpio.h"
#include "pinconfig.h"
#include "project_parameters.h"
#include "main.h"

/*################### Private variables #####################################*/

HallSensor_HandleTypeDef HallSensor;
#ifdef TESTING_2X
HallSensor_HandleTypeDef HallSensor_2x;
#endif
#if !defined(USE_FLOATING_POINT)
uint16_t HallStateAngles[8] = HALL_ANGLES_INT;
#endif

float HallStateAnglesFwdFloat[8];
float HallStateAnglesRevFloat[8];
float HallStateAnglesMidFloat[8];

// Motor rotation order -> 5, 1, 3, 2 ,6, 4...
uint8_t HallStateForwardOrder[8];
uint8_t HallStateReverseOrder[8];

uint32_t HallSampleBuffer[HALL_NUM_SAMPLES];

float* HallDetectAngleTable;
uint8_t HallDetectTableLength;
uint32_t HallDetectTransitionsDone[6];

/*################### Private function declarations #########################*/
static void HallSensor_CalcSpeed(void);
#ifdef TESTING_2X
static void HallSensor2_CalcSpeed(void);
#endif

/*################### Public functions ######################################*/

/********************* Helper functions **************************************/
// These functions aren't frequently called, but they can be used to generate
// parameter data.
/** HallSensor_AutoGenFwdTable
 * Auto-generates the forward rotation table 
 * from a list of Hall state transition angles.
 */
uint8_t HallSensor_AutoGenFwdTable(float* angleTab, uint8_t* fwdTab) {
    // Assume all tables are length 8. That's enough for all possible combos
    // of the three Hall sensors, including the undefined 0 and 7 states.

    // Enforce all states are valid (angle between 0.0 and 1.0)
    for (uint8_t i = 1; i <= 6; i++) {
        if ((angleTab[i] > 1.0f) || (angleTab[i] < 0.0f)) {
            return 0;
        }
    }

    uint8_t already_used_states = 0;	// Bits set to one if the correspoding
                                        // state was already selected.
    float lowestval;
    uint8_t loweststate;
    for (uint8_t j = 1; j <= 6; j++) {

        // Find the next lowest Hall state
        lowestval = 99.9f;
        loweststate = 7;
        for (uint8_t k = 1; k <= 6; k++) {
            if ((already_used_states & (1 << k)) == 0) {
                if (angleTab[k] < lowestval) {
                    lowestval = angleTab[k];
                    loweststate = k;
                }
            }
        }
        fwdTab[j] = loweststate;
        already_used_states |= (1 << loweststate);
    }
    return 1;
}

/** HallSensor_AutoGenFwdInvTable
 * Auto-generates the inverse forward rotation table from a list of Hall state 
 * transition angles. The inverse table gives the previous Hall state for a 
 * given state if the motor is rotating forwards. For example:
 * fwdInvTable[3] = 2
 * This means that if we are currently in Hall state 3, the correct previous 
 * state was 2.
 */
uint8_t HallSensor_AutoGenFwdInvTable(float* angleTab, uint8_t* fwdInvTab) {
    // Assume all tables are length 8. That's enough for all possible combos
    // of the three Hall sensors, including the undefined 0 and 7 states.
    uint8_t fwdTab[8];
    uint8_t invTab[8];
    // Enforce all states are valid (angle between 0.0 and 1.0)
    for (uint8_t i = 1; i <= 6; i++) {
        if ((angleTab[i] > 1.0f) || (angleTab[i] < 0.0f)) {
            return 0;
        }
    }

    uint8_t already_used_states = 0;	// Bits set to one if the correspoding
                                        // state was already selected.
    float lowestval;
    uint8_t loweststate;
    for (uint8_t j = 1; j <= 6; j++) {

        // Find the next lowest Hall state
        lowestval = 99.9f;
        loweststate = 7;
        for (uint8_t k = 1; k <= 6; k++) {
            if ((already_used_states & (1 << k)) == 0) {
                if (angleTab[k] < lowestval) {
                    lowestval = angleTab[k];
                    loweststate = k;
                }
            }
        }
        fwdTab[j] = loweststate;
        invTab[loweststate] = j;
        already_used_states |= (1 << loweststate);
    }
    // The invTab gives the order (1 through 6) for each state. You can look up
    // the state to see which order it is in. The fwdTab gives the state for a
    // particular order.
    // We need to get the *previous* state for each state.
    // Search the invTable for each state to get which order it's in, subtract
    // one from the order to get the previous state, look up the state in
    // fwdTable by its order. Just don't forget to wrap around from 1 --> 6
    fwdInvTab[0] = 0;
    fwdInvTab[7] = 0;
    uint8_t temp;
    for (uint8_t ii = 1; ii <= 6; ii++) {
        temp = invTab[ii]; // Get this state's order
        temp--; // The previous order
        if (temp == 0) {
            temp = 6;
        } // Wraparound. Zero is invalid.
        fwdInvTab[ii] = fwdTab[temp]; // Get the state at the previous order
    }
    return 1;
}

/** HallSensor_AutoGenRevTable
 * Auto-generates the reverse rotation table 
 * from a list of Hall state transition angles.
 */
uint8_t HallSensor_AutoGenRevTable(float* angleTab, uint8_t* revTab) {
    // Assume all tables are length 8. That's enough for all possible combos
    // of the three Hall sensors, including the undefined 0 and 7 states.

    // Enforce all states are valid (angle between 0.0 and 1.0)
    for (uint8_t i = 1; i <= 6; i++) {
        if ((angleTab[i] > 1.0f) || (angleTab[i] < 0.0f)) {
            return 0;
        }
    }

    uint8_t already_used_states = 0;	// Bits set to one if the correspoding
                                        // state was already selected.
    float highestval;
    uint8_t higheststate;
    for (uint8_t j = 1; j <= 6; j++) {

        // Find the next lowest Hall state
        highestval = -1.0f;
        higheststate = 7;
        for (uint8_t k = 1; k <= 6; k++) {
            if ((already_used_states & (1 << k)) == 0) {
                if (angleTab[k] > highestval) {
                    highestval = angleTab[k];
                    higheststate = k;
                }
            }
        }
        revTab[j] = higheststate;
        already_used_states |= (1 << higheststate);
    }
    return 1;
}

/** HallSensor_AutoGenRevInvTable
 * Auto-generates the inverse reverse rotation table from a list of Hall state 
 * transition angles. The inverse table gives the previous Hall state for a 
 * given state if the motor is rotating reverse. For example:
 * fwdInvTable[2] = 3
 * This means that if we are currently in Hall state 2, the correct previous 
 * state was 3.
 */
uint8_t HallSensor_AutoGenRevInvTable(float* angleTab, uint8_t* revInvTab) {
    // Assume all tables are length 8. That's enough for all possible combos
    // of the three Hall sensors, including the undefined 0 and 7 states.
    uint8_t revTab[8];
    uint8_t invTab[8];
    // Enforce all states are valid (angle between 0.0 and 1.0)
    for (uint8_t i = 1; i <= 6; i++) {
        if ((angleTab[i] > 1.0f) || (angleTab[i] < 0.0f)) {
            return 0;
        }
    }

    uint8_t already_used_states = 0;	// Bits set to one if the correspoding
                                        // state was already selected.
    float highestval;
    uint8_t higheststate;
    for (uint8_t j = 1; j <= 6; j++) {

        // Find the next lowest Hall state
        highestval = -1.0f;
        higheststate = 7;
        for (uint8_t k = 1; k <= 6; k++) {
            if ((already_used_states & (1 << k)) == 0) {
                if (angleTab[k] > highestval) {
                    highestval = angleTab[k];
                    higheststate = k;
                }
            }
        }
        revTab[j] = higheststate;
        invTab[higheststate] = j;
        already_used_states |= (1 << higheststate);
    }
    // The invTab gives the order (1 through 6) for each state. You can look up
    // the state to see which order it is in. The fwdTab gives the state for a
    // particular order.
    // We need to get the *previous* state for each state.
    // Search the invTable for each state to get which order it's in, subtract
    // one from the order to get the previous state, look up the state in
    // fwdTable by its order. Just don't forget to wrap around from 1 --> 6
    revInvTab[0] = 0;
    revInvTab[7] = 0;
    uint8_t temp;
    for (uint8_t ii = 1; ii <= 6; ii++) {
        temp = invTab[ii]; // Get this state's order
        temp--; // The previous order
        if (temp == 0) {
            temp = 6;
        } // Wraparound. Zero is invalid.
        revInvTab[ii] = revTab[temp]; // Get the state at the previous order
    }
    return 1;
}

/** HallSensor_Get_State
 * Retrieves the state (number 0-7) corresponding to the Hall Sensor
 * inputs. States 0 and 7 are invalid, since the Hall Sensors should
 * never be all low or all high. States 1-6 are valid states.
 */
uint8_t HallSensor_Get_State(void) {
    return HallSensor.CurrentState;
}
#ifdef TESTING_2X
uint8_t HallSensor2_Get_State(void) {
    return HallSensor_2x.CurrentState;
}
#endif

/** HallSensor_Inc_Angle
 * Speed and timing info, plus hall state at the last captured edge,
 * are used to interpolate the angle within a 60° sector.
 */
void HallSensor_Inc_Angle(void) {
    // Increment the angle by the pre-calculated increment amount
    if (HallSensor.RotationDirection == HALL_ROT_FORWARD) {
        HallSensor.Angle += HallSensor.AngleIncrement;
    } else if (HallSensor.RotationDirection == HALL_ROT_REVERSE) {
        HallSensor.Angle -= HallSensor.AngleIncrement;
    }
    // Don't do anything if rotation is unknown.
#if defined(USE_FLOATING_POINT)
    // Wraparound for floating point. Fixed point simply overflows the 16 bit variable.
    if (HallSensor.Angle > 1.0f)
        HallSensor.Angle -= 1.0f;
    if (HallSensor.Angle < 0.0f)
        HallSensor.Angle += 1.0f;
#endif
}
#ifdef TESTING_2X
void HallSensor2_Inc_Angle(void) {
    // Increment the angle by the pre-calculated increment amount
    if (HallSensor_2x.RotationDirection == HALL_ROT_FORWARD) {
        HallSensor_2x.Angle += HallSensor_2x.AngleIncrement;
    } else if (HallSensor_2x.RotationDirection == HALL_ROT_REVERSE) {
        HallSensor_2x.Angle -= HallSensor_2x.AngleIncrement;
    }
    // Don't do anything if rotation is unknown.
#if defined(USE_FLOATING_POINT)
    // Wraparound for floating point. Fixed point simply overflows the 16 bit variable.
    if (HallSensor_2x.Angle > 1.0f)
        HallSensor_2x.Angle -= 1.0f;
    if (HallSensor_2x.Angle < 0.0f)
        HallSensor_2x.Angle += 1.0f;
#endif
}
#endif

/** HallSensor_Get_Angle
 * Retrieves the motor electrical angle as a function of the Hall state.
 */
uint16_t HallSensor_Get_Angle(void) {

#if defined(USE_FLOATING_POINT)
    if ((HallSensor.Status & HALL_STOPPED) != 0) {
        return (uint16_t) (HallStateAnglesFwdFloat[HallSensor_Get_State()]
                * 65536.0f);
    }
    return (uint16_t) (HallSensor.Angle * 65536.0f);
#else
    if((HallSensor.Status & HALL_STOPPED) != 0)
    {
        return HallStateAngles[HallSensor_Get_State()];
    }
    return HallSensor.Angle;
#endif
}
#ifdef TESTING_2X
uint16_t HallSensor2_Get_Angle(void) {

#if defined(USE_FLOATING_POINT)
    if ((HallSensor_2x.Status & HALL_STOPPED) != 0) {
        return (uint16_t) (HallStateAnglesFwdFloat[HallSensor2_Get_State()]
                * 65536.0f);
    }
    return (uint16_t) (HallSensor_2x.Angle * 65536.0f);
#else
    if((HallSensor_2x.Status & HALL_STOPPED) != 0)
    {
        return HallStateAngles[HallSensor2_Get_State()];
    }
    return HallSensor_2x.Angle;
#endif
}
#endif
/** HallSensor_Get_Angle
 * Retrieves the motor electrical angle as a function of the Hall state.
 * Returns the floating point representation (0.0 -> 1.0)
 */
float HallSensor_Get_Anglef(void) {
    return HallSensor.Angle;
}
#ifdef TESTING_2X
float HallSensor2_Get_Anglef(void) {
    return HallSensor_2x.Angle;
}
#endif

/** HallSensor_Get_Speed
 * Returns the electrical speed in Hz in the form Q16.16 (32 bit number, where the
 * least significant 16 bits are fractional).
 */
uint32_t HallSensor_Get_Speed(void) {
#if defined(USE_FLOATING_POINT)
    return (uint32_t) (HallSensor.Speed * 65536.0f);
#else
    return HallSensor.Speed;
#endif
}

#ifdef TESTING_2X
uint32_t HallSensor2_Get_Speed(void) {
#if defined(USE_FLOATING_POINT)
    return (uint32_t) (HallSensor_2x.Speed * 65536.0f);
#else
    return HallSensor_2x.Speed;
#endif
}
#endif
/** HallSensor_Get_Speed
 * Returns the electrical speed in Hz as a floating point value.
 */
float HallSensor_Get_Speedf(void) {
    return HallSensor.Speed;
}

#ifdef TESTING_2X
float HallSensor2_Get_Speedf(void) {
    return HallSensor_2x.Speed;
}
#endif

uint8_t HallSensor_Get_Direction(void) {
    return HallSensor.RotationDirection;
}

#ifdef TESTING_2X
uint8_t HallSensor2_Get_Direction(void) {
    return HallSensor_2x.RotationDirection;
}
#endif

uint8_t HallSensor_SetAngle(uint8_t state, float newAngle) {
    if(state < 1 || state > 6) {
        // Out of range, only valid for states 1 to 6
        return UI_ERROR;
    }
    if((newAngle < 0.0f) || (newAngle > 1.0f)) {
        // Out of range, only angles zero to one allowed
        return UI_ERROR;
    }
    // Copy the angle
    HallStateAnglesFwdFloat[state] = newAngle;
    // Update forward and reverse lookup tables
    HallSensor_AutoGenFwdInvTable(HallStateAnglesFwdFloat, HallStateForwardOrder);
    HallSensor_AutoGenRevInvTable(HallStateAnglesFwdFloat, HallStateReverseOrder);
    // Generate the reverse angle table
    for (uint8_t i = 1; i <= 6; i++) {
        HallStateAnglesRevFloat[HallStateForwardOrder[i]] =
                HallStateAnglesFwdFloat[i];

    }
    // Generate the midpoint angle table
    for (uint8_t i = 1; i <= 6; i++) {
        HallStateAnglesMidFloat[i] = (HallStateAnglesFwdFloat[i] + HallStateAnglesRevFloat[i]) / 2.0f;
    }

    return UI_OK;
}

uint8_t HallSensor_SetAngleTable(float* angleTab) {
    // Check that angles are okay
    uint8_t i;
    for (i = 1; i <= 6; i++) {
        if ((angleTab[i] < 0.0f) || (angleTab[i] > 1.0f)) {
            // Fail, this is outside of the proper range
            return UI_ERROR;
        }
    }
    // Copy over the foward angle table
    for (i = 0; i < 8; i++) {
        HallStateAnglesFwdFloat[i] = angleTab[i];
    }
    // Update the forward and reverse lookup tables
    HallSensor_AutoGenFwdInvTable(HallStateAnglesFwdFloat,
            HallStateForwardOrder);
    HallSensor_AutoGenRevInvTable(HallStateAnglesFwdFloat,
            HallStateReverseOrder);
    // Generate the reverse angle table
    for (uint8_t i = 1; i <= 6; i++) {
        HallStateAnglesRevFloat[HallStateForwardOrder[i]] =
                HallStateAnglesFwdFloat[i];
    }
    // Generate the midpoint angle table
    for (uint8_t i = 1; i <= 6; i++) {
        HallStateAnglesMidFloat[i] = (HallStateAnglesFwdFloat[i] + HallStateAnglesRevFloat[i]) / 2.0f;
    }
    return UI_OK;
}

float* HallSensor_GetAngleTable(void) {
    return HallStateAnglesFwdFloat;
}

float HallSensor_GetStateMidpoint(uint8_t state) {
    if((state < 1) || (state > 6)) {
        return 0.0f;
    }
    return HallStateAnglesMidFloat[state];
}

/** HallSensor_Init
 * Starts the time base for the Hall Sensor Timer and the GPIOs associated
 * with the Hall Sensors. The timer is started in the UP counting mode, with
 * a prescaler that results in about a 10KHz clock. The period is always at
 * the max value, 0xFFFF (=65535 decimal). The first overflow will occur at
 * about 6 and a half seconds.
 * While the timer is running and the Hall effect sensor inputs are
 * switching, the prescaler is adjusted to keep the interrupts around half
 * of the maximum period. This results in better granularity and more accurate
 * measurement of the motor speed.
 * Noise filters on the TIM_CCR1 capture input are turned on. This prevents
 * spurious changes being recognized as Hall state changes. Additionally,
 * a DMA channel and simple timer are used to take multiple measurements of
 * the GPIO states, and a determination of the current Hall state is made
 * based on the majority of the readings.
 */

void HallSensor_Init_NoHal(uint32_t callingFrequency) {

    HallDetectAngleTable = (float*) 0;
    HallDetectTableLength = 0;
    //HALL_PORT_CLK_ENABLE();
    GPIO_Clk(HALL_PORT);
    HALL_TIM_CLK_ENABLE();

    // Enable GPIOs!
    GPIO_AF(HALL_PORT, HALL_PIN_A, HALL_PINS_AF);
    GPIO_AF(HALL_PORT, HALL_PIN_B, HALL_PINS_AF);
    GPIO_AF(HALL_PORT, HALL_PIN_C, HALL_PINS_AF);

    HALL_PORT->PUPDR |= (GPIO_PUPDR_PUPDR0_0 << HALL_PIN_A);
    HALL_PORT->PUPDR |= (GPIO_PUPDR_PUPDR0_0 << HALL_PIN_B);
    HALL_PORT->PUPDR |= (GPIO_PUPDR_PUPDR0_0 << HALL_PIN_C);

    HALL_TIMER->PSC = HALL_PSC_MAX; // Set the prescaler as high as possible to start
    HALL_TIMER->ARR = 0xFFFF; // Auto reload always at max

    HALL_TIMER->CCMR1 = TIM_CCMR1_CC1S; // Channel 1 is input
    HALL_TIMER->CCMR1 |= TIM_CCMR1_IC1F_3 | TIM_CCMR1_IC1F_0; // Filter set to 8 samples
                                                              // at Fdts/8 (2.625MHz)
    HALL_TIMER->SMCR = TIM_SMCR_TS_2 | TIM_SMCR_SMS_2; // Reset mode, input is TI1F_ED (Channel 1
                                                       // input, filtered, edge detector)
    HALL_TIMER->CCER = TIM_CCER_CC1E | TIM_CCER_CC1P | TIM_CCER_CC1NP; // Input 1 enabled, both
                                                                       // edges captured
    HALL_TIMER->CR1 = TIM_CR1_URS; // The slave mode resets (capture interrupts) won't trigger
                                   // an update interrupt, only a timer overflow will.
    HALL_TIMER->CR1 |= TIM_CR1_CKD_1; // Input filter clock = timer clock / 4
    HALL_TIMER->CR2 = TIM_CR2_TI1S; // Channels 1, 2, and 3 are XOR'd together into Channel 1
                                    // Also, the Reset pulse is sent as TRGO (to the Sample timer)

    HALL_TIMER->EGR |= TIM_EGR_UG; // Trigger an update to get all those shadow registers set

    NVIC_SetPriority(HALL_IRQn, PRIO_HALL);
    NVIC_EnableIRQ(HALL_IRQn);

    HALL_TIMER->DIER = TIM_DIER_CC1IE | TIM_DIER_UIE; // Enable channel 1 and update interrupts
    HALL_TIMER->CR1 |= TIM_CR1_CEN; // Start the timer

    HallSensor.Prescaler = HALL_PSC_MAX;
    HallSensor.Status = 0;
    HallSensor.Speed = 0;
    HallSensor.CallingFrequency = callingFrequency;
    HallSensor.OverflowCount = 0;
    HallSensor.Status |= HALL_STOPPED;
    HallSensor.CurrentState = 0;
#ifdef TESTING_2X
    HallSensor_2x.Prescaler = HALL_PSC_MAX;
    HallSensor_2x.Status = 0;
    HallSensor_2x.Speed = 0;
    HallSensor_2x.CallingFrequency = callingFrequency;
    HallSensor_2x.OverflowCount = 0;
    HallSensor_2x.Status |= HALL_STOPPED;
    HallSensor_2x.CurrentState = 0;
#endif

    // Initialization for the Hall state measuring method.
    // Timer starts, triggered by Hall_Timer Capture, which then repeatedly moves
    // the GPIO input register into memory via DMA. When determining the Hall state
    // a majority decision is made based on the list of recorded values.

    HALL_DMA_CLK_ENABLE();
    // Channel 7 selected (TIM8_UP), transfer size = 32 bits, memory increases, transfer complete interrupt enabled.
    HALL_DMA->CR = DMA_SxCR_CHSEL_2 | DMA_SxCR_CHSEL_1 | DMA_SxCR_CHSEL_0
            | DMA_SxCR_MSIZE_1 | DMA_SxCR_PSIZE_1 | DMA_SxCR_MINC
            | DMA_SxCR_TCIE;
    HALL_DMA->NDTR = HALL_NUM_SAMPLES;
    HALL_DMA->M0AR = (uint32_t) HallSampleBuffer;
    HALL_DMA->PAR = (uint32_t) (&(HALL_PORT->IDR));

    NVIC_SetPriority(DMA2_Stream1_IRQn, PRIO_HALL);
    NVIC_EnableIRQ(DMA2_Stream1_IRQn);

    HALL_SAMPLE_TIMER_CLK_ENABLE();
    HALL_SAMPLE_TIMER->CR1 = TIM_CR1_URS; // Reset doesn't affect update event, only counter overflow does.
    HALL_SAMPLE_TIMER->ARR = HALL_SAMPLE_PERIOD;
    //// Trigger ITR2 selected (TIM3_TRGO)
    //// with Trigger Mode (timer starts on rising edge of trigger
    //HALL_SAMPLE_TIMER->SMCR = TIM_SMCR_SMS_2 | TIM_SMCR_SMS_1 | TIM_SMCR_TS_1;
    HALL_SAMPLE_TIMER->EGR |= TIM_EGR_UG; // Trigger an update to reset everything

    DMA1->LIFCR = 0x0F7D0F7D; // Clear all flags -
    DMA1->HIFCR = 0x0F7D0F7D; // on all channels.
    HALL_DMA->CR |= DMA_SxCR_EN; // Enable DMA channel
    HALL_SAMPLE_TIMER->DIER = TIM_DIER_UDE; // Update triggers DMA

    // Determine initial Hall state
    HallSensor.CurrentState +=
            (HALL_PORT->IDR & (1 << HALL_PIN_A)) != 0 ? 1 : 0;
    HallSensor.CurrentState +=
            (HALL_PORT->IDR & (1 << HALL_PIN_B)) != 0 ? 2 : 0;
    HallSensor.CurrentState +=
            (HALL_PORT->IDR & (1 << HALL_PIN_C)) != 0 ? 4 : 0;
#ifdef TESTING_2X
    HallSensor_2x.CurrentState +=
            (HALL_PORT->IDR & (1 << HALL_PIN_A)) != 0 ? 1 : 0;
    HallSensor_2x.CurrentState +=
            (HALL_PORT->IDR & (1 << HALL_PIN_B)) != 0 ? 2 : 0;
    HallSensor_2x.CurrentState +=
            (HALL_PORT->IDR & (1 << HALL_PIN_C)) != 0 ? 4 : 0;
#endif

    // Load default values from eeprom
    HallStateAnglesFwdFloat[0] = F32_0_DEG;
    HallStateAnglesFwdFloat[7] = F32_0_DEG;
    HallStateAnglesFwdFloat[1] = EE_ReadFloatWithDefault(CONFIG_MOTOR_HALL1, DFLT_MOTOR_HALL1);
    HallStateAnglesFwdFloat[2] = EE_ReadFloatWithDefault(CONFIG_MOTOR_HALL2, DFLT_MOTOR_HALL2);
    HallStateAnglesFwdFloat[3] = EE_ReadFloatWithDefault(CONFIG_MOTOR_HALL3, DFLT_MOTOR_HALL3);
    HallStateAnglesFwdFloat[4] = EE_ReadFloatWithDefault(CONFIG_MOTOR_HALL4, DFLT_MOTOR_HALL4);
    HallStateAnglesFwdFloat[5] = EE_ReadFloatWithDefault(CONFIG_MOTOR_HALL5, DFLT_MOTOR_HALL5);
    HallStateAnglesFwdFloat[6] = EE_ReadFloatWithDefault(CONFIG_MOTOR_HALL6, DFLT_MOTOR_HALL6);

    // Update the forward and reverse lookup tables
    HallSensor_AutoGenFwdInvTable(HallStateAnglesFwdFloat,
            HallStateForwardOrder);
    HallSensor_AutoGenRevInvTable(HallStateAnglesFwdFloat,
            HallStateReverseOrder);
    // Generate the reverse angle table
    for (uint8_t i = 1; i <= 6; i++) {
        HallStateAnglesRevFloat[HallStateForwardOrder[i]] =
                HallStateAnglesFwdFloat[i];
    }
    // Generate the midpoint angle table
    for (uint8_t i = 1; i <= 6; i++) {
        HallStateAnglesMidFloat[i] = (HallStateAnglesFwdFloat[i] + HallStateAnglesRevFloat[i]) / 2.0f;
    }

}

/** HallSensor_CalcSpeed
 * Called from the capture interrupt. The capture value is the period of time between Hall Sensor
 * state changes. This function needs to (1) determine the timebase as a function of the timer
 * clock and prescaler, and (2) determine the motor electrical speed as the inverse of the period
 * between state changes.
 */
static void HallSensor_CalcSpeed(void) {
    // Timer input clock / prescaler = actual timer clock
    // Actual timer clock / capture counts = Hall state transition frequency
    // Hall state frequency / 6 = motor electrical frequency

#if defined(USE_FLOATING_POINT)
//	HallSensor.Speed = ((float)HALL_TIMER_INPUT_CLOCK) /(6.0f * ((float)(HallSensor.Prescaler + 1)) * ((float)HallSensor.CaptureValue) );
    // Sum up all 6 states
    float full_rotation_capture = 0.0f;
    for (uint8_t i = 0; i < 6; i++) {
        full_rotation_capture += ((float) (HallSensor.CaptureForState[i]))
                * ((float) (HallSensor.PrescalerForState[i] + 1));
    }

    if ((HallSensor.RotationDirection == HALL_ROT_FORWARD)
            || (HallSensor.RotationDirection == HALL_ROT_REVERSE)) {
        HallSensor.Speed = ((float) HALL_TIMER_INPUT_CLOCK)
                / full_rotation_capture;
        HallSensor.AngleIncrement = HallSensor.Speed
                / ((float) HallSensor.CallingFrequency);
    } else {
        HallSensor.Speed = 0;
        HallSensor.AngleIncrement = 0;
    }

#else

    uint32_t freq = ((uint32_t)HALL_TIMER_INPUT_CLOCK) << 4; // Clock input frequency in (Hz * 2^4)
    freq = freq / (HallSensor.Prescaler + 1);// Actual timer frequency in (Hz * 2^4)
    if(HallSensor.CaptureValue != 0)// No divides by zero please
    {
        freq = freq / HallSensor.CaptureValue; // Frequency of Hall state change in (Hz * 2^4)
        HallSensor.Speed = ((uint32_t)(freq<<12)) / 6;// Speed in Hz*2^16
    }
    else
    {
        HallSensor.Speed = 0;
    }

    //HallSensor.AngleIncrement = ((uint32_t)(HallSensor.Speed * 6) / HallSensor.CallingFrequency) / 6;
    // Cancel out the 6's
    /** The angle increment is the amount of angle to add each time the angle is updated **/
    HallSensor.AngleIncrement = (uint32_t)(HallSensor.Speed) / HallSensor.CallingFrequency;

#endif
}

#ifdef TESTING_2X
static void HallSensor2_CalcSpeed(void) {
    HallSensor_2x.Speed = ((float) HALL_TIMER_INPUT_CLOCK)
            / (2.0f * ((float) (HallSensor_2x.Prescaler + 1))
                    * ((float) HallSensor_2x.CaptureValue));
    HallSensor_2x.AngleIncrement = HallSensor_2x.Speed
            / ((float) HallSensor_2x.CallingFrequency);
    HallSensor_2x.CaptureValue = 0;
}
#endif

void HallSensor_Enable_Hall_Detection(float* angleTable, uint8_t tableLength) {
    HallDetectAngleTable = angleTable;
    HallDetectTableLength = tableLength;
    for (uint8_t i = 0; i < 6; i++) {
        HallDetectTransitionsDone[i] = 0;
    }
}
void HallSensor_Disable_Hall_Detection(void) {
    HallDetectAngleTable = (float*) 0;
    HallDetectTableLength = 0;
}

/** HallSensor_UpdateCallback
 * Triggered when the Hall Sensor timer overflows (counts past 65535)
 * This means that no Hall change occurred for the entire counter duration.
 * So in this case, we set the speed to zero and try to lengthen the counter.
 * If the prescaler is already at the maximum value, it can't be lengthened any more.
 */
void HallSensor_UpdateCallback(void) {
    HallSensor.OverflowCount++;
    if (HallSensor.OverflowCount >= HALL_MAX_OVERFLOWS) {
        // Limit overflow counter
        HallSensor.OverflowCount = HALL_MAX_OVERFLOWS;
        // Set speed to zero - stopped motor
#if defined(USE_FLOATING_POINT)
        HallSensor.Speed = 0.0f;
        HallSensor.AngleIncrement = 0.0f;
#else
        HallSensor.Speed = 0;
        HallSensor.AngleIncrement = 0;
#endif
        HallSensor.Status |= HALL_STOPPED;
        HallSensor.Prescaler = HALL_PSC_MAX;
        HALL_TIMER->PSC = HALL_PSC_MAX;
#ifdef TESTING_2X
        // Set speed to zero - stopped motor
        HallSensor_2x.Speed = 0.0f;
        HallSensor_2x.AngleIncrement = 0.0f;
        HallSensor_2x.Status |= HALL_STOPPED;
        HallSensor_2x.Prescaler = HALL_PSC_MAX;
        HallSensor_2x.CaptureValue = 0;
#endif
    }
}

/**
 * HallSensor_CaptureCallback
 * Triggered when any of the three Hall Sensor switches change state.
 * This function stores the most recent speed information. If the switch change
 * occurred in the first 1/8 of the timer period, the prescaler is reduced to
 * shorten the timer period. Likewise, if it occurred after 7/8 of the period,
 * the timer period is extended.
 */
void HallSensor_CaptureCallback(void) {
    uint8_t lastState = HallSensor.CurrentState;
    uint8_t nextState;
    HallSensor.CaptureValue = HALL_TIMER->CCR1;

    //HallSensor.CurrentState = HallSensor_Get_State();
    HALL_SAMPLE_TIMER->CR1 |= TIM_CR1_CEN; // Start the sampling for the next state

    // Figure out which way we're turning.
    /*
     if(HallStateForwardOrder[HallSensor.CurrentState] == lastState)
     HallSensor.RotationDirection = HALL_ROT_FORWARD;
     else if(HallStateReverseOrder[HallSensor.CurrentState] == lastState)
     HallSensor.RotationDirection = HALL_ROT_REVERSE;
     else
     HallSensor.RotationDirection = HALL_ROT_UNKNOWN;
     */
    // Update the angle - just encountered a 60deg marker (the Hall state change)
    // If we're rotating forward, the actual angle will be at the beginning of the state.
    // For example, if we entered State 5 (0->60°), we will be at 0°. Since State 5
    // is defined as the middle of its range (30°), we need to subtract 30°. In the
    // reverse rotation case, we would instead add 30°. If we can't trust which way
    // the motor is turning, just choose the middle of the range (don't add or subtract
    // anything).
#if defined(USE_FLOATING_POINT)
    switch (HallSensor.RotationDirection) {
    case HALL_ROT_FORWARD:
        nextState = HallStateReverseOrder[lastState];
        HallSensor.Angle = HallStateAnglesFwdFloat[nextState];
        HallSensor.CaptureForState[nextState - 1] = HallSensor.CaptureValue;
        HallSensor.PrescalerForState[nextState - 1] = HallSensor.Prescaler;
        if (HallSensor.Angle < 0.0f) {
            HallSensor.Angle += 1.0f;
        }
#ifdef TESTING_2X
        HallSensor_2x.CaptureValue += HallSensor.CaptureValue;
        if (HallSensor.OverflowCount > 0) {
            HallSensor_2x.CaptureValue += HallSensor.OverflowCount * 0xFFFF;
        }
        if (nextState == 2 || nextState == 5) // Using the A hall sensor change
                {
            HallSensor_2x.Angle = HallStateAnglesFwdFloat[nextState];
            if (HallSensor_2x.Angle < 0.0f) {
                HallSensor_2x.Angle += 1.0f;
            }
            if ((HallSensor_2x.Status & HALL_STOPPED) == 0) {
                HallSensor2_CalcSpeed();
            } else
                HallSensor_2x.Status &= ~(HALL_STOPPED);
        }
#endif
        break;
    case HALL_ROT_REVERSE:
        nextState = HallStateForwardOrder[lastState];
        HallSensor.Angle = HallStateAnglesRevFloat[nextState];
        HallSensor.CaptureForState[nextState - 1] = HallSensor.CaptureValue;
        HallSensor.PrescalerForState[nextState - 1] = HallSensor.Prescaler;
        if (HallSensor.Angle > 1.0f) {
            HallSensor.Angle -= 1.0f;
        }
#ifdef TESTING_2X
        HallSensor_2x.CaptureValue += HallSensor.CaptureValue;
        if (HallSensor.OverflowCount > 0) {
            HallSensor_2x.CaptureValue += HallSensor.OverflowCount * 0xFFFF;
        }
        if (nextState == 3 || nextState == 4) // Using the A hall sensor change
                {
            HallSensor_2x.Angle = HallStateAnglesRevFloat[nextState];
            if (HallSensor_2x.Angle > 1.0f) {
                HallSensor_2x.Angle -= 1.0f;
            }
            if ((HallSensor_2x.Status & HALL_STOPPED) == 0) {
                HallSensor2_CalcSpeed();
            }
        }
#endif
        break;
    case HALL_ROT_UNKNOWN:
    default:
        //HallSensor.Angle = HallStateAnglesFloat[HallSensor.CurrentState];
        //Angle updated in DMA transfer complete interrupt
        break;
    }
    //HallSensor.Angle = HallStateAnglesFloat[HallSensor.CurrentState];
#else
    switch(HallSensor.RotationDirection)
    {
        case HALL_ROT_FORWARD:
        HallSensor.Angle = HallStateAngles[HallSensor.CurrentState] - U16_30_DEG;
        break;
        case HALL_ROT_REVERSE:
        HallSensor.Angle = HallStateAngles[HallSensor.CurrentState] + U16_30_DEG;
        break;
        case HALL_ROT_UNKNOWN:
        default:
        HallSensor.Angle = HallStateAngles[HallSensor.CurrentState];
        break;
    }
    //HallSensor.Angle = HallStateAngles[HallSensor.CurrentState];
#endif

    if (HallSensor.OverflowCount > 0) {
        // Fix the capture value for the speed calculation
        // Include the duration of the timer for each overflow that occurred
        HallSensor.CaptureValue += ((HallSensor.OverflowCount) * 0xFFFF);
    }

    // Only calculate speed if there have been two consecutive captures without stopping
    if ((HallSensor.Status & HALL_STOPPED) == 0)
        HallSensor_CalcSpeed();
    else
        HallSensor.Status &= ~(HALL_STOPPED);

    // Update prescaler if needed - can't change if it was just adjusted in the last capture
    if ((HallSensor.Status & HALL_PSC_CHANGED) == 0) {
        if (HallSensor.CaptureValue <= HALL_MIN_CAPTURE) {
            if (HallSensor.Prescaler > HALL_PSC_MIN) {
                HALL_TIMER->PSC = HallSensor.Prescaler - HALL_PSC_CHG_AMT;
                HallSensor.Status |= HALL_PSC_CHANGED_DOWN;
            }
        }
        if (HallSensor.OverflowCount > 0) {
            if (HallSensor.Prescaler < HALL_PSC_MAX) {
                HALL_TIMER->PSC = HallSensor.Prescaler + HALL_PSC_CHG_AMT;
                HallSensor.Status |= HALL_PSC_CHANGED_UP;
            }
        }
    } else	// It was previously changed, time to take it into effect
            // This is now safe to do since the speed calculation is already done
    {
        HallSensor.Prescaler = HALL_TIMER->PSC;
#ifdef TESTING_2X
        HallSensor_2x.Prescaler = HALL_TIMER->PSC;
#endif
        HallSensor.Status &= ~(HALL_PSC_CHANGED);
    }
    // Now it's safe to clear overflow counts
    HallSensor.OverflowCount = 0;
}

void DMA2_Stream1_IRQHandler(void) {
    uint32_t hall_a_vote = 0, hall_b_vote = 0, hall_c_vote = 0;
    uint8_t voted_state = 0;
    if ((DMA2->LISR & DMA_LISR_TCIF1) != 0) {
        DMA2->LIFCR = DMA_LIFCR_CTCIF1;
        if ((HALL_DMA->CR & DMA_SxCR_TCIE) != 0) {
            // Turn off the sample timer
            HALL_SAMPLE_TIMER->CR1 &= ~(TIM_CR1_CEN);
            HALL_SAMPLE_TIMER->DIER = 0;
            // Re-enable the DMA stream
            HALL_DMA->NDTR = HALL_NUM_SAMPLES;
            HALL_DMA->M0AR = (uint32_t) HallSampleBuffer;
            HALL_DMA->PAR = (uint32_t) (&(HALL_PORT->IDR));
            HALL_DMA->CR |= DMA_SxCR_EN;
            HALL_SAMPLE_TIMER->DIER = TIM_DIER_UDE;

            // Take majority vote to determine Hall state
            for (uint8_t i = 0; i < HALL_NUM_SAMPLES; i++) {
                hall_a_vote +=
                        (HallSampleBuffer[i] & (1 << HALL_PIN_A)) ? 1 : 0;
                hall_b_vote +=
                        (HallSampleBuffer[i] & (1 << HALL_PIN_B)) ? 1 : 0;
                hall_c_vote +=
                        (HallSampleBuffer[i] & (1 << HALL_PIN_C)) ? 1 : 0;
            }
            // Determine majority: use 50% as decision criteria
            if (hall_a_vote > (HALL_NUM_SAMPLES / 2)) {
                voted_state += 1;
            }
            if (hall_b_vote > (HALL_NUM_SAMPLES / 2)) {
                voted_state += 2;
            }
            if (hall_c_vote > (HALL_NUM_SAMPLES / 2)) {
                voted_state += 4;
            }

            // Invalid state?
            if ((voted_state == 0) || (voted_state == 7)) {
                MAIN_SetError(MAIN_ERR_HALL_STATE);
            }
            // Determine direction
            if (HallSensor.CurrentState == HallStateForwardOrder[voted_state]) {
                HallSensor.RotationDirection = HALL_ROT_FORWARD;
#ifdef TESTING_2X
                HallSensor_2x.RotationDirection = HALL_ROT_FORWARD;
#endif
            } else if (HallSensor.CurrentState
                    == HallStateReverseOrder[voted_state]) {
                HallSensor.RotationDirection = HALL_ROT_REVERSE;
#ifdef TESTING_2X
                HallSensor_2x.RotationDirection = HALL_ROT_REVERSE;
#endif
            } else {
                HallSensor.RotationDirection = HALL_ROT_UNKNOWN;
                // Need to update angle here instead of in the capture callback
#if defined(USE_FLOATING_POINT)
                if (((HallStateAnglesFwdFloat[voted_state] > F32_270_DEG)
                        && (HallStateAnglesRevFloat[voted_state] < F32_90_DEG))
                        || ((HallStateAnglesFwdFloat[voted_state] < F32_90_DEG)
                                && (HallStateAnglesRevFloat[voted_state]
                                        > F32_270_DEG))) {
                    // This takes care of wraparound. If one angle is close to 1.0 and
                    // the other is close to 0.0, we can't just simply average the two.
                    // The average would then be around 0.5 when it should be closer to
                    // 0.0 or 1.0.
                    // Instead, add 1.0 to the sum (effectively pushing one or the other
                    // angle to more than 1.0), average them, and then add or subtract
                    // 1.0 if needed.
                    HallSensor.Angle = (HallStateAnglesFwdFloat[voted_state]
                            + HallStateAnglesRevFloat[voted_state] + 1.0f)
                            * 0.5f;
                    while (HallSensor.Angle > 1.0f) {
                        HallSensor.Angle -= 1.0f;
                    }
                    while (HallSensor.Angle < 0.0f) {
                        HallSensor.Angle += 1.0f;
                    }

                } else {
                    HallSensor.Angle = (HallStateAnglesFwdFloat[voted_state]
                            + HallStateAnglesRevFloat[voted_state]) * 0.5f;
                }
#else
                HallSensor.Angle = HallStateAngles[voted_state];
#endif
#ifdef TESTING_2X
                if (((HallStateAnglesFwdFloat[voted_state] > F32_270_DEG)
                        && (HallStateAnglesRevFloat[voted_state] < F32_90_DEG))
                        || ((HallStateAnglesFwdFloat[voted_state] < F32_90_DEG)
                                && (HallStateAnglesRevFloat[voted_state]
                                        > F32_270_DEG))) {
                    // See above description for explanation.
                    HallSensor_2x.Angle = (HallStateAnglesFwdFloat[voted_state]
                            + HallStateAnglesRevFloat[voted_state] + 1.0f)
                            * 0.5f;
                    while (HallSensor_2x.Angle > 1.0f) {
                        HallSensor_2x.Angle -= 1.0f;
                    }
                    while (HallSensor_2x.Angle < 0.0f) {
                        HallSensor_2x.Angle += 1.0f;
                    }
                } else {
                    HallSensor_2x.Angle = (HallStateAnglesFwdFloat[voted_state]
                            + HallStateAnglesRevFloat[voted_state]) * 0.5f;
                }
#endif
            }
            HallSensor.CurrentState = voted_state;
        }
    }
    // If the Hall detection routine is running, save the angle this transition
    // occurred at.
    if ((HallDetectTableLength > 0) && (HallDetectAngleTable != (float*) 0)) {
        if (HallDetectTransitionsDone[voted_state - 1]
                < HallDetectTableLength) {
            HallDetectAngleTable[voted_state - 1
                    + (6 * HallDetectTransitionsDone[voted_state - 1])] =
                    MAIN_GetCurrentRampAngle();
            HallDetectTransitionsDone[voted_state - 1]++;
        }
    }

}

