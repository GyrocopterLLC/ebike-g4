/******************************************************************************
 * Filename: hall_sensor.c
 * Description: Interfaces with Hall sensors commonly found on brushless DC
 *              motors. The Hall sensor is a magnetic switch which flips
 *              from high to low or low to high  when a magnet is nearby.
 *              Specifically, it switches when magnetic flux is above some
 *              threshold.
 *
 *              These sensors are used inside brushless DC motors to detect
 *              the position of the rotor. The rotor magnets provide the
 *              flux needed to make the Hall sensor switch.
 *
 *              We need three Hall sensors to determine rotor position. Each
 *              one is usually situated either 60� apart or 120� apart. We
 *              can determine the rotor position to an accuracy of 1/6th of
 *              an electrical motor rotation simply by using the current status
 *              of the Hall sensor switches. That is good enough resolution
 *              to perform classic "trapezoidal" control of the BLDC motor.
 *
 *              For higher resolution, a phase locked loop (PLL) is set to
 *              track the switching events. The time between previous switch
 *              events is used to estimate the rotational speed. The angle of
 *              the motor is increased at a fixed rate based on speed between
 *              switch events, and can be corrected to the actual position
 *              when a switch event occurs. This high resolution position can
 *              then be used in Field Oriented Control (FOC).
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

static void HALL_CalcSpeed(void);
static void HALL_CalcAcceleration(void);
static float HALL_CalcMidPoint(float a1, float a2);
static float HALL_ClipToOne(float unclipped);
static void HALL_UpdateLookupTables(void);

HallSensor_HandleTypeDef HallSensor;
HallSensorPLL_HandleTypeDef HallSensorPLL;
float HallStateAnglesMidFloat[8]; // Midpoints of states. Angle is 0.0 (0deg) to 1.0 (360deg)
float HallStateAnglesFwdFloat[8]; // Entry angle for states when rotating forwards. Angle is 0.0 (0deg) to 1.0 (360deg)
float HallStateAnglesRevFloat[8]; // Entry angle for states when rotating reverse. Angle is 0.0 (0deg) to 1.0 (360deg)
uint8_t HallStateForwardOrder[8]; // List of states, lowest to highest angle
uint8_t HallStateReverseOrder[8]; // List of states, highest to lowest angle
uint8_t HallStateForwardRotation[8]; // Lookup where index is current state, value is next state when rotating forwards
uint8_t HallStateReverseRotation[8]; // Lookup where index is current state, value is next state when rotating reverse
float* HallDetectAngleTable;
uint8_t HallDetectTableLength;
uint32_t HallDetectTransitionsDone[6];

/**
 * @brief  Initializes the Hall effect sensor interface
 * @param  callingFrequency: The frequency (Hz) at which
 *          the angle update function will be called
 * @retval None
 */
void HALL_Init(uint32_t callingFrequency) {
    // Set up the GPIOs and timers used for the Hall sensor interface
    GPIO_Clk(HALL_PORT);
    GPIO_AF(HALL_PORT, HALL_A_PIN, HALL_AF);
    GPIO_AF(HALL_PORT, HALL_B_PIN, HALL_AF);
    GPIO_AF(HALL_PORT, HALL_C_PIN, HALL_AF);

    HALL_TIM_CLK_ENABLE();

    // General settings - Dead time and sampling filter set to input clock / 4 (170MHz / 4 = 42.5MHz)
    // Update request source set, which means that slave mode controller reset does NOT generate
    // an update interrupt. This is important, since we use the update interrupt to determine if the
    // Hall sensors have timed out.
    HALL_TIM->CR1 = TIM_CR1_CKD_1 | TIM_CR1_URS;
    // Auto-reload value will always be the maximum value
    HALL_TIM->ARR = 0xFFFFu;
    // Input settings for the Hall timer
    // Capture compare units 1-3 are used, but inputs are all XOR'd together into channel 1
    // CC1 channel is input with input 1 mapped to it, no prescaler so every change
    // counts, and input filter set to 8 samples at Fdts / 16 (42.5 / 16 = 2.65625 MHz) ... fastest response is 3 us
    HALL_TIM->CCMR1 = TIM_CCMR1_CC1S_0 | TIM_CCMR1_IC1F_3 | TIM_CCMR1_IC1F_2;

    HALL_TIM->SMCR = TIM_SMCR_TS_2 | TIM_SMCR_SMS_2; // Reset mode, input is TI1F_ED (Channel 1
                                                       // input, filtered, edge detector)
    HALL_TIM->CCER = TIM_CCER_CC1E | TIM_CCER_CC1P | TIM_CCER_CC1NP; // Input 1 enabled, both
                                                                       // edges captured
    HALL_TIM->CR2 = TIM_CR2_TI1S; // Channels 1, 2, and 3 are XOR'd together into Channel 1

    HALL_TIM->EGR |= TIM_EGR_UG; // Trigger an update event to latch in any preloaded registers

    NVIC_SetPriority(HALL_IRQn, PRIO_HALL);
    NVIC_EnableIRQ(HALL_IRQn);

    HALL_TIM->DIER = TIM_DIER_CC1IE | TIM_DIER_UIE; // Enable channel 1 and update interrupts
    HALL_TIM->CR1 |= TIM_CR1_CEN; // Start the timer

    HallSensor.Prescaler = HALL_PSC_MAX;
    HallSensor.Status = 0;
    HallSensor.Speed = 0.0f;
    HallSensor.PreviousSpeed = 0.0f;
    HallSensor.Accel = 0.0f;
    for(uint8_t i = 0; i < NUM_ACCEL_SAMPLES; i++) {
        HallSensor.AccelSamples[i] = 0.0f;
    }
    HallSensor.CurrentAccelSample = 0;
    HallSensor.CallingFrequency = callingFrequency;
    HallSensor.OverflowCount = 0;
    HallSensor.SteadyRotationCount = 0;
    HallSensor.Status |= HALL_STOPPED;
    HallSensor.CurrentState = 0;
    HallSensor.PreviousState = 0;
    HallSensor.RotationDirection = HALL_ROT_UNKNOWN;
    HallSensor.PreviousRotationDirection = HALL_ROT_UNKNOWN;
    HallSensor.Valid = ANGLE_INVALID;

    // Determine initial Hall state
    HallSensor.CurrentState +=
            (HALL_PORT->IDR & (1 << HALL_A_PIN)) != 0 ? 1 : 0;
    HallSensor.CurrentState +=
            (HALL_PORT->IDR & (1 << HALL_B_PIN)) != 0 ? 2 : 0;
    HallSensor.CurrentState +=
            (HALL_PORT->IDR & (1 << HALL_C_PIN)) != 0 ? 4 : 0;

    // Load default values from eeprom
    HALL_LoadVariables();
}

/**
 * @brief  Generates a ordered list of state transitions from a list of angles at each state.
 * @param  angleTab: list of 8 angles, corresponding with the state transitions for
 *          states 1-6. Locations 0 and 7 are ignored
 * @param  fwdOrderTab: the forward rotation order. Lowest angle is at location 1, highest
 *          at location 6. Locations 0 and 7 are ignored
 * @retval RETVAL_OK if succeeded (the math works out) otherwise RETVAL_FAIL
 */
uint8_t HALL_GenFwdOrder(float* angleTab, uint8_t* fwdOrderTab) {
    // Assume all tables are length 8. That's enough for all possible combos
    // of the three Hall sensors, including the undefined 0 and 7 states.

    // Enforce all states are valid (angle between 0.0 and 1.0)
    for (uint8_t i = 1; i <= 6; i++) {
        if ((angleTab[i] > 1.0f) || (angleTab[i] < 0.0f)) {
            return RETVAL_FAIL;
        }
    }

    uint8_t already_used_states = 0;    // Bits set to one if the correspoding
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
        fwdOrderTab[j] = loweststate;
        already_used_states |= (1 << loweststate);
    }
    return RETVAL_OK;
}

/**
 * @brief  Generates a ordered list of state transitions from a list of angles at each state.
 * @param  angleTab: list of 8 angles, corresponding with the state transitions for
 *          states 1-6. Locations 0 and 7 are ignored
 * @param  fwdOrderTab: the reverse rotation order. Highest angle is at location 1, lowest
 *          at location 6. Locations 0 and 7 are ignored
 * @retval RETVAL_OK if succeeded (the math works out) otherwise RETVAL_FAIL
 */
uint8_t HALL_GenRevOrder(float* angleTab, uint8_t* revOrderTab) {
    // Assume all tables are length 8. That's enough for all possible combos
    // of the three Hall sensors, including the undefined 0 and 7 states.

    // Enforce all states are valid (angle between 0.0 and 1.0)
    for (uint8_t i = 1; i <= 6; i++) {
        if ((angleTab[i] > 1.0f) || (angleTab[i] < 0.0f)) {
            return RETVAL_FAIL;
        }
    }

    uint8_t already_used_states = 0;    // Bits set to one if the correspoding
                                        // state was already selected.
    float highestval;
    uint8_t higheststate;
    for (uint8_t j = 1; j <= 6; j++) {

        // Find the next highest Hall state
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
        revOrderTab[j] = higheststate;
        already_used_states |= (1 << higheststate);
    }
    return RETVAL_OK;
}

/**
 * @brief  Generates a forward rotation table from a list of state transitions.
 * @param  orderTab: list of 8 states, where position 1 is the lowest angle and
 *          position 6 is the highest angle (order that states will be encountered
 *          when rotating forwards). Locations 0 and 7 are ignored.
 * @param  fwdTab: the forward rotation table, a lookup table where each position
 *          is the current state, and the value in that position is the next state.
 *          Locations 0 and 7 are ignored.
 * @retval RETVAL_OK if succeeded (the math works out) otherwise RETVAL_FAIL
 */
uint8_t HALL_GenFwdTable(uint8_t* orderTab, uint8_t* fwdTab) {

    // Check that all the states are in there
    uint8_t temp_check = 0;
    uint8_t i;
    for(i = 1; i <= 6; i++) {
        temp_check |= (1 << orderTab[i]); // Set a bit for each state
    }
    if(temp_check != 0x7E) {
        // All states 1 to 6 should be in the table
        return RETVAL_FAIL;
    }
    uint8_t cur_state;
    uint8_t next_state;
    for(i = 1; i <= 6; i++) {
        // Get the state at this point in the rotation order
        cur_state = orderTab[i];
        if(i < 6) {
            // Get the state after this one
            next_state = orderTab[i+1];
        } else {
            // Wraparound from 6 to 1 if we're at the end
            next_state = orderTab[1];
        }
        fwdTab[cur_state] = next_state;
    }

    return RETVAL_OK;
}


/**
 * @brief  Generates an inverse forward rotation table from a forward rotation table.
 * @param  fwdTab: the forward rotation table, a lookup table where each position
 *          is the current state, and the value in that position is the next state.
 *          Locations 0 and 7 are ignored.
 * @param  fwdInvTab: the inverse forward rotation table, a lookup table where each position
 *          is the next state, and the value in that position is the current state.
 *          Alternatively you can say the position is the current state, and value is previous
 *          state. Again only locations 1-6 matter.
 * @retval RETVAL_OK if succeeded (the math works out) otherwise RETVAL_FAIL
 */
uint8_t HALL_GenFwdInvTable(uint8_t* fwdTab, uint8_t* fwdInvTab) {
    // Check that all the states are in there
    uint8_t temp_check = 0;
    uint8_t i;
    for(i = 1; i <= 6; i++) {
        temp_check |= (1 << fwdTab[i]); // Set a bit for each state
    }
    if(temp_check != 0x7E) {
        // All states 1 to 6 should be in the table
        return RETVAL_FAIL;
    }

    // Invert the table. Value becomes position, position becomes value.
    for(uint8_t i = 1; i <= 6; i++) {
        fwdInvTab[fwdTab[i]] = i;
    }
    return RETVAL_OK;
}

/**
 * @brief  Generates a reverse rotation table from a list of state transitions.
 * @param  revOrderTab: list of 8 states, where position 1 is the highest angle and
 *          position 6 is the lowest angle (order that states will be encountered
 *          when rotating reverse). Locations 0 and 7 are ignored.
 * @param  revTab: the reverse rotation table, a lookup table where each position
 *          is the current state, and the value in that position is the next state.
 *          Again only locations 1-6 matter.
 * @retval RETVAL_OK if succeeded (the math works out) otherwise RETVAL_FAIL
 */
uint8_t HALL_GenRevTable(uint8_t* revOrderTab, uint8_t* revTab) {
    // Check that all the states are in there
    uint8_t temp_check = 0;
    uint8_t i;
    for(i = 1; i <= 6; i++) {
        temp_check |= (1 << revOrderTab[i]); // Set a bit for each state
    }
    if(temp_check != 0x7E) {
        // All states 1 to 6 should be in the table
        return RETVAL_FAIL;
    }
    uint8_t cur_state;
    uint8_t next_state;
    for(i = 1; i <= 6; i++) {
        // Get the state at this point in the rotation order
        cur_state = revOrderTab[i];
        if(i < 6) {
            // Get the state after this one
            next_state = revOrderTab[i+1];
        } else {
            // Wraparound from 6 to 1 if we're at the end
            next_state = revOrderTab[1];
        }
        revTab[cur_state] = next_state;
    }

    return RETVAL_OK;
}

/**
 * @brief  Generates an inverse reverse rotation table from a reverse rotation table.
 * @param  revTab: the reverse rotation table, a lookup table where each position
 *          is the current state, and the value in that position is the next state.
 *          Again only locations 1-6 matter.
 * @param  revInvTab: the inverse reverse rotation table, a lookup table where each position
 *          is the next state, and the value in that position is the current state.
 *          Alternatively you can say the position is the current state, and value is previous
 *          state. Again only locations 1-6 matter.
 * @retval RETVAL_OK if succeeded (the math works out) otherwise RETVAL_FAIL
 */
uint8_t HALL_GenRevInvTable(uint8_t* revTab, uint8_t* revInvTab) {
    // Check that all the states are in there
    uint8_t temp_check = 0;
    uint8_t i;
    for(i = 1; i <= 6; i++) {
        temp_check |= (1 << revTab[i]); // Set a bit for each state
    }
    if(temp_check != 0x7E) {
        // All states 1 to 6 should be in the table
        return RETVAL_FAIL;
    }

    // Invert the table. Value becomes position, position becomes value.
    for(uint8_t i = 1; i <= 6; i++) {
        revInvTab[revTab[i]] = i;
    }
    return RETVAL_OK;
}

uint8_t HALL_GetState(void) {
    return HallSensor.CurrentState;
}
void HALL_IncAngle(void) {
    // Increment the angle by the pre-calculated increment amount
    if (HallSensor.RotationDirection == HALL_ROT_FORWARD) {
        HallSensor.Angle += HallSensor.AngleIncrement;
    } else if (HallSensor.RotationDirection == HALL_ROT_REVERSE) {
        HallSensor.Angle -= HallSensor.AngleIncrement;
    }
    // Don't do anything if rotation is unknown.
    // Wraparound for floating point.
       HallSensor.Angle = HALL_ClipToOne(HallSensor.Angle);
}

uint8_t HALL_IsStopped(void) {
    if((HallSensor.Status & HALL_STOPPED) != 0) {
        return 1;
    } else {
        return 0;
    }
}

uint16_t HALL_GetAngle(void) {
    if ((HallSensor.Status & HALL_STOPPED) != 0) {
        return (uint16_t) (HallStateAnglesFwdFloat[HALL_GetState()]
                * 65536.0f);
    }
    return (uint16_t) (HallSensor.Angle * 65536.0f);
}

float HALL_GetAngleF(void) {
    if ((HallSensor.Status & HALL_STOPPED) != 0) {
        return HallStateAnglesFwdFloat[HALL_GetState()];
    }
    return HallSensor.Angle;
}

uint32_t HALL_GetSpeed(void) {
    return (uint32_t) (HallSensor.Speed * 65536.0f);
}

float HALL_GetSpeedF(void) {
    return HallSensor.Speed;
}

uint32_t HALL_GetAcceleration(void) {
    return (uint32_t) (HallSensor.Accel * 65536.0f);
}

float HALL_GetAccelerationF(void) {
    return HallSensor.Accel;
}

uint8_t HALL_GetDirection(void) {
    return HallSensor.RotationDirection;
}

uint8_t HALL_IsValid(void)
{
    return HallSensor.Valid;
}

void HALL_PLLUpdate(void) {
    // Run the PLL to create a smoothed angle output
    float phase_difference;
    phase_difference =  HallSensor.Angle - HallSensorPLL.Phase;
    while(phase_difference > 0.5f) {
        phase_difference -= 1.0f;
    }
    while(phase_difference < -0.5f) {
        phase_difference += 1.0f;
    }
    HallSensorPLL.Frequency += HallSensorPLL.Beta*phase_difference;
    HallSensorPLL.Phase += HallSensorPLL.Alpha*phase_difference + HallSensorPLL.Frequency;
    HallSensorPLL.Phase = HALL_ClipToOne(HallSensorPLL.Phase);

    // Check for phase lock

    if(phase_difference < 0.0f) {
        phase_difference = -phase_difference; // Absolute value of phase error
    }
    if(phase_difference < PLL_LOCKED_PHASE_ERROR) {
        if(HallSensorPLL.ValidCounter < PLL_LOCKED_COUNTS) {
            HallSensorPLL.ValidCounter++;
        }
        if(HallSensorPLL.ValidCounter >= PLL_LOCKED_COUNTS) {
            HallSensorPLL.Valid = PLL_LOCKED;
        }
    } else {
        if(HallSensorPLL.ValidCounter > 0) {
            HallSensorPLL.ValidCounter--;
        }
        if(HallSensorPLL.ValidCounter == 0) {
            HallSensorPLL.Valid = PLL_UNLOCKED;
        }
    }
}

uint16_t HALL_GetPLLAngle(void) {
    return (uint16_t)(HallSensorPLL.Phase * 65536.0f);
}
float HALL_GetPLLAngleF(void) {
    return HallSensorPLL.Phase;
}

uint32_t HALL_GetPLLSpeed(void) {
    return ((uint32_t)(HallSensorPLL.Frequency * 65536.0f))*HallSensor.CallingFrequency;
}

float HALL_GetPLLSpeedF(void) {
    return HallSensorPLL.Frequency*((float)HallSensor.CallingFrequency);
}

uint8_t HALL_PLLIsValid(void){
    return HallSensorPLL.Valid;
}

uint8_t HALL_SetAngle(uint8_t state, float newAngle) {
    if(state < 1 || state > 6) {
        // Out of range, only valid for states 1 to 6
        return RETVAL_FAIL;
    }
    if((newAngle < 0.0f) || (newAngle > 1.0f)) {
        // Out of range, only angles zero to one allowed
        return RETVAL_FAIL;
    }
    // Copy the angle
    HallStateAnglesFwdFloat[state] = newAngle;
    // Update forward and reverse lookup tables
    HALL_UpdateLookupTables();
    return RETVAL_OK;
}

uint8_t HALL_SetAngleTable(float* angleTab) {
    // Check that angles are okay
    uint8_t i;
    for (i = 1; i <= 6; i++) {
        if ((angleTab[i] < 0.0f) || (angleTab[i] > 1.0f)) {
            // Fail, this is outside of the proper range
            return RETVAL_FAIL;
        }
    }
    // Copy over the forward angle table
    for (i = 0; i < 8; i++) {
        HallStateAnglesFwdFloat[i] = angleTab[i];
    }

    HALL_UpdateLookupTables();
    return RETVAL_OK;
}

float* HALL_GetAngleTable(void) {
    return HallStateAnglesFwdFloat;
}

float HALL_GetAngleFromTable(uint8_t state) {
    return HallStateAnglesFwdFloat[state];
}

float HALL_GetStateMidpoint(uint8_t state) {
    if((state < 1) || (state > 6)) {
        return 0.0f;
    }
    return HallStateAnglesMidFloat[state];
}

void HALL_ChangeFrequency(uint32_t newfreq) {
    HallSensor.CallingFrequency = newfreq;
    HallSensorPLL.Alpha = HallSensorPLL.Alpha / HallSensorPLL.dt;
    HallSensorPLL.dt = (1.0f)/((float)newfreq);
    HallSensorPLL.Alpha = HallSensorPLL.Alpha * HallSensorPLL.dt;
    HallSensorPLL.Beta = (0.5f)*(HallSensorPLL.Alpha)*(HallSensorPLL.Alpha);
}

void HALL_EnableHallDetection(float* angleTable, uint8_t tableLength) {
    HallDetectAngleTable = angleTable;
    HallDetectTableLength = tableLength;
    for (uint8_t i = 0; i < 6; i++) {
        HallDetectTransitionsDone[i] = 0;
    }
}

void HALL_DisableHallDetection(void) {
    HallDetectAngleTable = (float*) 0;
    HallDetectTableLength = 0;
}

void HALL_UpdateCallback(void) {
    HallSensor.OverflowCount++;
    if (HallSensor.OverflowCount >= HALL_MAX_OVERFLOWS) {
        // Limit overflow counter
        HallSensor.OverflowCount = HALL_MAX_OVERFLOWS;
        // Set speed to zero - stopped motor
        HallSensor.Speed = 0.0f;
        HallSensor.AngleIncrement = 0.0f;
        HallSensor.Status |= HALL_STOPPED;
        HallSensor.Prescaler = HALL_PSC_MAX;
        HALL_TIM->PSC = HALL_PSC_MAX;
        HallSensor.Valid = ANGLE_INVALID;
        HallSensor.SteadyRotationCount = 0;
    }
}

/**
 * @brief  Interrupt callback for a capture event.
 *
 *      Triggered when any of the three Hall Sensor switches change state.
 * This function stores the most recent speed information. If the switch change
 * occurred in the first 1/8 of the timer period, the prescaler is reduced to
 * shorten the timer period. Likewise, if it occurred after 7/8 of the period,
 * the timer period is extended.
 */
void HALL_CaptureCallback(void) {

    HallSensor.CaptureValue = HALL_TIM->CCR1;
    // Figure out which way we're turning. Get the state of all three Hall sensor pins
    uint16_t temp_idr = HALL_PORT->IDR;
    HallSensor.CurrentState = (temp_idr & (1 << HALL_A_PIN) ? 1 : 0)
            + (temp_idr & (1 << HALL_B_PIN) ? 2 : 0)
            + (temp_idr & (1 << HALL_C_PIN) ? 4 : 0);

    if(HallStateForwardRotation[HallSensor.PreviousState] == HallSensor.CurrentState)
        HallSensor.RotationDirection = HALL_ROT_FORWARD;
    else if(HallStateReverseRotation[HallSensor.PreviousState] == HallSensor.CurrentState)
        HallSensor.RotationDirection = HALL_ROT_REVERSE;
    else
        HallSensor.RotationDirection = HALL_ROT_UNKNOWN;
    // Update the angle - just encountered a 60deg marker (the Hall state change)
    // If we're rotating forward, the actual angle will be at the beginning of the state.
    // For example, if we entered State 5 (210->270�), we will be at 210� if rotating forwards,
    // but at 270 if rotating reverse.
    // If we can't trust which way the motor is turning, just choose the middle of the range.

    switch (HallSensor.RotationDirection) {
    case HALL_ROT_FORWARD:
        HallSensor.Angle = HallStateAnglesFwdFloat[HallSensor.CurrentState];
        HallSensor.CaptureForState[HallSensor.CurrentState] = HallSensor.CaptureValue;
        HallSensor.PrescalerForState[HallSensor.CurrentState] = HallSensor.Prescaler;
        break;
    case HALL_ROT_REVERSE:
        HallSensor.Angle = HallStateAnglesRevFloat[HallSensor.CurrentState];
        HallSensor.CaptureForState[HallSensor.CurrentState] = HallSensor.CaptureValue;
        HallSensor.PrescalerForState[HallSensor.CurrentState] = HallSensor.Prescaler;
        break;
    case HALL_ROT_UNKNOWN:
    default:
        HallSensor.Angle = HallStateAnglesMidFloat[HallSensor.CurrentState];
        break;
    }

    if (HallSensor.OverflowCount > 0) {
        // Fix the capture value for the speed calculation
        // Include the duration of the timer for each overflow that occurred
        HallSensor.CaptureValue += ((HallSensor.OverflowCount) * 0xFFFF);
    }
    // Only calculate speed if there have been two consecutive captures without stopping
    if ((HallSensor.Status & HALL_STOPPED) == 0)
        HALL_CalcSpeed();
    else
        HallSensor.Status &= ~(HALL_STOPPED);

    // Update prescaler if needed - can't change if it was just adjusted in the last capture
    if ((HallSensor.Status & HALL_PSC_CHANGED) == 0) {
        if (HallSensor.CaptureValue <= HALL_MIN_CAPTURE) {
            if (HallSensor.Prescaler > HALL_PSC_MIN) {
                HALL_TIM->PSC = HallSensor.Prescaler - HALL_PSC_CHG_AMT;
                HallSensor.Status |= HALL_PSC_CHANGED_DOWN;
            }
        }
        if (HallSensor.OverflowCount > 0) {
            if (HallSensor.Prescaler < HALL_PSC_MAX) {
                HALL_TIM->PSC = HallSensor.Prescaler + HALL_PSC_CHG_AMT;
                HallSensor.Status |= HALL_PSC_CHANGED_UP;
            }
        }
    } else  // It was previously changed, time to take it into effect
            // This is now safe to do since the speed calculation is already done
    {
        HallSensor.Prescaler = HALL_TIM->PSC;
        HallSensor.Status &= ~(HALL_PSC_CHANGED);
    }
    // Now it's safe to clear overflow counts
    HallSensor.OverflowCount = 0;

    // Check if speed is changing at a reasonable rate
    if(fabsf(HallSensor.Speed - HallSensor.PreviousSpeed) < HALL_MAX_SPEED_CHANGE)
    {
        // Check if direction is steady
        if(HallSensor.RotationDirection != HALL_ROT_UNKNOWN) {
            if(HallSensor.RotationDirection == HallSensor.PreviousRotationDirection) {
                if(HallSensor.SteadyRotationCount >= HALL_MIN_STEADY_ROTATION_COUNT) {
                    // It's valid!
                    HallSensor.Valid = ANGLE_VALID;
                } else {
                    // All is good, but counting up until valid
                    HallSensor.SteadyRotationCount++;
                    HallSensor.Valid = ANGLE_INVALID;
                }
            } else {
                // Bad direction, reset the counter
                HallSensor.SteadyRotationCount = 0;
                HallSensor.Valid = ANGLE_INVALID;
            }
        } else {
            // Bad speed, reset the counter
            HallSensor.SteadyRotationCount = 0;
            HallSensor.Valid = ANGLE_INVALID;
        }
        // Acceleration: difference in speed divided by the time between sampling
        // accel = (CurrentSpeed - PrevSpeed) / ((timer_counts*prescaler) / clk_freq)
        HallSensor.AccelSamples[HallSensor.CurrentAccelSample++] = (HallSensor.Speed - HallSensor.PreviousSpeed) * ((float) HALL_CLK) /
                (((float) (HallSensor.CaptureForState[HallSensor.CurrentState])) *
                ((float) (HallSensor.PrescalerForState[HallSensor.CurrentState] + 1)));
        // Wraparound the sample counter
        if(HallSensor.CurrentAccelSample >= NUM_ACCEL_SAMPLES)
            HallSensor.CurrentAccelSample = 0;
        HALL_CalcAcceleration();

    }
    HallSensor.PreviousSpeed = HallSensor.Speed;
    HallSensor.PreviousRotationDirection = HallSensor.RotationDirection;
    HallSensor.PreviousState = HallSensor.CurrentState;
}

void HALL_SaveVariables(void) {
    EE_SaveFloat(CONFIG_MOTOR_HALL1, HallStateAnglesFwdFloat[1]);
    EE_SaveFloat(CONFIG_MOTOR_HALL2, HallStateAnglesFwdFloat[2]);
    EE_SaveFloat(CONFIG_MOTOR_HALL3, HallStateAnglesFwdFloat[3]);
    EE_SaveFloat(CONFIG_MOTOR_HALL4, HallStateAnglesFwdFloat[4]);
    EE_SaveFloat(CONFIG_MOTOR_HALL5, HallStateAnglesFwdFloat[5]);
    EE_SaveFloat(CONFIG_MOTOR_HALL6, HallStateAnglesFwdFloat[6]);
}

void HALL_LoadVariables(void) {
    HallStateAnglesFwdFloat[0] = F32_0_DEG;
    HallStateAnglesFwdFloat[7] = F32_0_DEG;
    HallStateAnglesFwdFloat[1] = EE_ReadFloatWithDefault(CONFIG_MOTOR_HALL1, DFLT_MOTOR_HALL1);
    HallStateAnglesFwdFloat[2] = EE_ReadFloatWithDefault(CONFIG_MOTOR_HALL2, DFLT_MOTOR_HALL2);
    HallStateAnglesFwdFloat[3] = EE_ReadFloatWithDefault(CONFIG_MOTOR_HALL3, DFLT_MOTOR_HALL3);
    HallStateAnglesFwdFloat[4] = EE_ReadFloatWithDefault(CONFIG_MOTOR_HALL4, DFLT_MOTOR_HALL4);
    HallStateAnglesFwdFloat[5] = EE_ReadFloatWithDefault(CONFIG_MOTOR_HALL5, DFLT_MOTOR_HALL5);
    HallStateAnglesFwdFloat[6] = EE_ReadFloatWithDefault(CONFIG_MOTOR_HALL6, DFLT_MOTOR_HALL6);

    HALL_UpdateLookupTables();

}

uint8_t uiMOTOR_GetHall1(uint8_t* valptr) {
    data_packet_pack_float(valptr, HALL_GetAngleFromTable(1));
    return DATA_PACKET_SUCCESS;
}

uint8_t uiMOTOR_SetHall1(uint8_t* valptr) {
    return HALL_SetAngle(1, data_packet_extract_float(valptr));
}

uint8_t uiMOTOR_GetHall2(uint8_t* valptr) {
    data_packet_pack_float(valptr, HALL_GetAngleFromTable(2));
    return DATA_PACKET_SUCCESS;
}

uint8_t uiMOTOR_SetHall2(uint8_t* valptr) {
    return HALL_SetAngle(2, data_packet_extract_float(valptr));
}

uint8_t uiMOTOR_GetHall3(uint8_t* valptr) {
    data_packet_pack_float(valptr, HALL_GetAngleFromTable(3));
    return DATA_PACKET_SUCCESS;
}

uint8_t uiMOTOR_SetHall3(uint8_t* valptr) {
    return HALL_SetAngle(3, data_packet_extract_float(valptr));
}

uint8_t uiMOTOR_GetHall4(uint8_t* valptr) {
    data_packet_pack_float(valptr, HALL_GetAngleFromTable(4));
    return DATA_PACKET_SUCCESS;
}

uint8_t uiMOTOR_SetHall4(uint8_t* valptr) {
    return HALL_SetAngle(4, data_packet_extract_float(valptr));
}

uint8_t uiMOTOR_GetHall5(uint8_t* valptr) {
    data_packet_pack_float(valptr, HALL_GetAngleFromTable(5));
    return DATA_PACKET_SUCCESS;
}

uint8_t uiMOTOR_SetHall5(uint8_t* valptr) {
    return HALL_SetAngle(5, data_packet_extract_float(valptr));
}

uint8_t uiMOTOR_GetHall6(uint8_t* valptr) {
    data_packet_pack_float(valptr, HALL_GetAngleFromTable(6));
    return DATA_PACKET_SUCCESS;
}

uint8_t uiMOTOR_SetHall6(uint8_t* valptr) {
    return HALL_SetAngle(6, data_packet_extract_float(valptr));
}

/**
 * @brief  Creates ordered state tables and lookup tables from a list of angles at each state.
 */
static void HALL_UpdateLookupTables(void) {
    // Create forward and reverse state transition order tables
    HALL_GenFwdOrder(HallStateAnglesFwdFloat, HallStateForwardOrder);
    HALL_GenRevOrder(HallStateAnglesFwdFloat, HallStateReverseOrder);

    // Create the forward and reverse lookup tables
    HALL_GenFwdTable(HallStateForwardOrder, HallStateForwardRotation);
    HALL_GenRevTable(HallStateReverseOrder, HallStateReverseRotation);
    // Generate the reverse angle table
    for (uint8_t i = 1; i <= 6; i++) {
        HallStateAnglesRevFloat[HallStateReverseRotation[i]] =
                HallStateAnglesFwdFloat[i];
    }
    // Generate the midpoint angle table
    for (uint8_t i = 1; i <= 6; i++) {
        HallStateAnglesMidFloat[i] = HALL_CalcMidPoint(HallStateAnglesRevFloat[i],HallStateAnglesFwdFloat[i]);

    }
}

/**
 * @brief Calculates the speed of rotation
 *
 *      Called from the capture interrupt. The capture value is the period of
 *      time between Hall Sensor state changes. This function needs to (1)
 *      determine the timebase as a function of the timer clock and prescaler,
 *      and (2) determine the motor electrical speed as the inverse of the
 *      period between state changes. This function updates the Speed and
 *      AngleIncrement. Speed is the electrical motor speed in Hz.
 *
 * @retval None
 */
static void HALL_CalcSpeed(void) {
    // Sum up all 6 states
    float full_rotation_capture = 0.0f;
    for (uint8_t i = 1; i <= 6; i++) {
        full_rotation_capture += ((float) (HallSensor.CaptureForState[i]))
                * ((float) (HallSensor.PrescalerForState[i] + 1));
    }

    if ((HallSensor.RotationDirection == HALL_ROT_FORWARD)
            || (HallSensor.RotationDirection == HALL_ROT_REVERSE)) {
        HallSensor.Speed = ((float) HALL_CLK)
                / full_rotation_capture;
        HallSensor.AngleIncrement = HallSensor.Speed
                / ((float) HallSensor.CallingFrequency);
    } else {
        HallSensor.Speed = 0;
        HallSensor.AngleIncrement = 0;
    }
}

/**
 * @brief Calculates the acceleration of rotation
 *
 *      Called from the capture interrupt. The previous acceleration samples
 *      are averaged to determine the acceleration, which adds a little
 *      noise filtering. The Accel variable is updated in this function.
 *      Accel is the electrical motor acceleration in Hz/s.
 *
 * @retval None
 */
static void HALL_CalcAcceleration(void) {

    float accelSum = 0.0f;
    // Sum up all previous accelerations
    for(uint8_t i = 0; i < NUM_ACCEL_SAMPLES; i++) {
        accelSum += HallSensor.AccelSamples[i];
    }
    // Average
    accelSum /= ((float)NUM_ACCEL_SAMPLES);
    HallSensor.Accel = accelSum;
}

// Determines the midpoint of two angles.
// Includes checking for wraparound.
static float HALL_CalcMidPoint(float a1, float a2) {
    float retval = 0.0f;
    // Take care of the case where we are wrapping around 1.0
    // If we didn't do this, the average angle would be close to 0.5 when it should instead
    // be close to either 0.0 or 1.0
    // If one angle is above 3/4 and the other is below 1/4, that's a wraparound case
    if( ((a1 > (0.75f)) && (a2 < (0.25f))) ||
        ((a2 > (0.75f)) && (a1 < (0.25f)))) {
        retval = (1.0f + a1 + a2) / 2.0f;
        if(retval > 1.0f) {
            retval -= 1.0f;
        }
    } else {
        retval = (a1 + a2) / 2.0f;
    }
    return retval;
}

static float HALL_ClipToOne(float unclipped) {
    // Output is allowed to be [0, 1)
    // Value of zero is allowed, but one is the same as zero.
    while(unclipped < 0.0f) {
        unclipped += 1.0f;
    }
    while(unclipped >= 1.0f) {
        unclipped -= 1.0f;
    }
    return unclipped;
}
