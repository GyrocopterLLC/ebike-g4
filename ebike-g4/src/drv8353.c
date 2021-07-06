/******************************************************************************
 * Filename: drv8353.c
 * Description: Controls Serial Peripheral Interface (SPI) hardware on the
 *              STM32G4 for communication with the DRV8353(R)S three-phase
 *              bridge PWM driver.
 *
 *              Each SPI transaction with the DRV8353 is a 16-bit transfer,
 *              where master-to-slave and slave-to-master transmission both
 *              happen simultaneously. The master clocks out a read/write
 *              bit, then 4 address bits, then 11 data bits. If reading,
 *              the data bits are don't-cares. The slave always clocks out
 *              the 11 data bits presently in the register for both
 *              reads and writes after the end of the address bits.
 *
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

static uint8_t DRV8353_CheckConnection(void);

/**
 * @brief SPI init for DRV8353.
 *
 * Initializes the SPI and GPIO for communication with the
 * DRV8353 driver IC. Obeys restrictions in the DRV8353
 * datasheet (16-bit, clock idles low, data changes on
 * rising edge, max bitrate 10MHz).
 *
 * @param  None
 * @retval None
 */

void DRV8353_Init(void) {
    // Clock the needed GPIOs and the SPI peripheral
    GPIO_Clk(SPI_PORT);
    DRV_SPI_CLK_ENABLE();
    // Set GPIO to alternate function mode
    GPIO_AF(SPI_PORT, SPI_MOSI_PIN, SPI_AF);
    GPIO_AF(SPI_PORT, SPI_MISO_PIN, SPI_AF);
    GPIO_AF(SPI_PORT, SPI_SCK_PIN, SPI_AF);
    GPIO_Output(SPI_PORT, SPI_CS_PIN);
    GPIO_High(SPI_PORT, SPI_CS_PIN);
    // Enable pin
    GPIO_Clk(DRV_EN_PORT);
    GPIO_Output(DRV_EN_PORT, DRV_EN_PIN);
    GPIO_Low(DRV_EN_PORT, DRV_EN_PIN); // make sure it's reset for now

    // Configure SPI
    DRV_SPI->CR1 = SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_MSTR;
    if(SPI_LSBFIRST != 0) {
        DRV_SPI->CR1 |= SPI_CR1_LSBFIRST;
    }
    switch(SPI_CLKDIV) {
    case 2:
        DRV_SPI->CR1 &= ~(SPI_CR1_BR);
        break;
    case 4:
        DRV_SPI->CR1 &= ~(SPI_CR1_BR);
        DRV_SPI->CR1 |= SPI_CR1_BR_0;
        break;
    case 8:
        DRV_SPI->CR1 &= ~(SPI_CR1_BR);
        DRV_SPI->CR1 |= SPI_CR1_BR_1;
        break;
    case 16:
        DRV_SPI->CR1 &= ~(SPI_CR1_BR);
        DRV_SPI->CR1 |= SPI_CR1_BR_1 | SPI_CR1_BR_0;
        break;
    case 32:
        DRV_SPI->CR1 &= ~(SPI_CR1_BR);
        DRV_SPI->CR1 |= SPI_CR1_BR_2;
        break;
    case 64:
        DRV_SPI->CR1 &= ~(SPI_CR1_BR);
        DRV_SPI->CR1 |= SPI_CR1_BR_2 | SPI_CR1_BR_0;
        break;
    case 128:
        DRV_SPI->CR1 &= ~(SPI_CR1_BR);
        DRV_SPI->CR1 |= SPI_CR1_BR_2 | SPI_CR1_BR_1;
        break;
    case 256:
    default:
        DRV_SPI->CR1 &= ~(SPI_CR1_BR);
        DRV_SPI->CR1 |= SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_BR_0;
        break;
    }

    if(SPI_PHASE == 1) {
        DRV_SPI->CR1 |= SPI_CR1_CPHA;
    }
    if(SPI_POLARITY == 1) {
        DRV_SPI->CR1 |= SPI_CR1_CPOL;
    }

    // Set data length, limit 4 to 16 bits. That's the hardware restriction
    if((SPI_DATASIZE >= 4) && (SPI_DATASIZE <= 16))
            DRV_SPI->CR2 = (SPI_DATASIZE - 1U) << 8U;
    else // Default to 8 bit
        DRV_SPI->CR2 = 0x0FU << 8U;


    // Start the chip by bringing enable high
    GPIO_High(DRV_EN_PORT, DRV_EN_PIN);
    Delay(5); // Wait a little bit for initialization.

    // Initialize the registers
    // CTRL - all three bridges will shutdown if one has a fault. PWM 6x mode
    DRV8353_Write(DRVREG_CTRL, DRVBIT_CTRL_OCPACT);
    Delay(5); // Wait after each write - there's some necessary time between SPI_EN going high and back low again

    // Gate drive settings.
    // Using TPW4R50ANH mosfets with 58nC total gate charge, 12nC gate-drain charge
    // Rise time goal: 50ns -
    //      IdriveP > Qgd/tr = 12/50 = 240mA (closest is 300mA, 0100b)
    // Fall time goal: 25ns -
    //      IdriveN > Qgd/tf = 12/25 = 480mA (closest is 600mA, 0100b)
    // Tdrive is 500, 1000, 2000, or 4000ns. Since we expect to be done switching in
    // 50ns or less, 500ns (setting b00) is save to choose.
    //

    // Gate HS - Mid range current source and sink (300mA source, 600mA sink)
    DRV8353_Write(DRVREG_GATEH, DRVBIT_GATEH_IDRIVEHS_2 | DRVBIT_GATEH_IDRIVENHS_2);
    Delay(5);
    // Gate LS - Mid range current source and sink (300mA source, 600mA sink), Tdrive = 500ns,
    //              and new PWM input needed to clear VDS_OCP or SEN_OCP faults.
    DRV8353_Write(DRVREG_GATEL, DRVBIT_GATEL_CBC | DRVBIT_GATEL_IDRIVELS_2 | DRVBIT_GATEL_IDRIVENLS_2);
    Delay(5);
    // OCP - 100ns dead time (generally dead time is set by the MCU), overcurrent faults are latched,
    //          deglitch time is 2us, and VDS over current level is 0.6V (worst case
    //          100degC Rdson = 6mOhm, trip at 100A) (setting is 1001b)
    DRV8353_Write(DRVREG_OCP, DRVBIT_OCP_DEADTIME_0 | DRVBIT_OCP_DEG_1 | DRVBIT_OCP_VDSLVL_3 | DRVBIT_OCP_VDSLVL_0);
    Delay(5);
    // CSA - Normal shunt resistor connections, bidirectional, 2nd smallest gain (10V/V, setting of 01b),
    //          sense over current threshold is 0.25V (equivalent to 125A with 2mOhm shunt, setting of 00b)
    DRV8353_Write(DRVREG_CSA, DRVBIT_CSA_VREFDIV | DRVBIT_CSA_GAIN_0);
    Delay(5);

    // Re-run auto calibration.
    DRV8353_Write(DRVREG_CAL, DRVBIT_CAL_MODE); // Enable auto-calibration mode
    Delay(5);
    // Run auto-cal for channel A
    DRV8353_SetCalibration(DRV_CHANNEL_A_CAL);
    Delay(5);
    // Run auto-cal for channel B
    DRV8353_SetCalibration(DRV_CHANNEL_A_CAL);
    Delay(5);
    // Run auto-cal for channel C
    DRV8353_SetCalibration(DRV_CHANNEL_A_CAL);
    Delay(5);
    // Reset channel calibrations
    DRV8353_SetCalibration(0);
    Delay(5);
    // Reset back into normal mode
    DRV8353_Write(DRVREG_CAL, 0); // Disable auto-calibration mode
    Delay(5);
}

/**
 * @brief  Launches a transaction (read/write) on SPI.
 * @param  DataOut The 16-bit data to write to the DRV8353
 * @retval The 16-bit data read from the chip.
 */
uint16_t DRV8353_Transaction(uint16_t DataOut) {
    uint32_t timeout_tracker;
    uint16_t retval = 0xFFFFU;
    // First pull chip select line low.
    GPIO_Low(SPI_PORT, SPI_CS_PIN);

    // Begin by feeding the Tx FIFO with data
    DRV_SPI->DR = DataOut;

    // Enable SPI to begin transaction
    DRV_SPI->CR1 |= SPI_CR1_SPE;

    timeout_tracker = SPI_MAX_WAIT_CYCLES;
    // Wait for RX FIFO to have some data in it (RX-not-empty = 1)
    while((DRV_SPI->SR & SPI_SR_RXNE) == 0) {
        // Quit if it takes too long, return all 1's
        // which shouldn't happen in a real transaction
        timeout_tracker--;
        if(timeout_tracker == 0) {
            break;
        }
    }
    if((DRV_SPI->SR & SPI_SR_RXNE) != 0) {
        retval = (uint16_t)(DRV_SPI->DR);
    }

    // Return CS line high
    GPIO_High(SPI_PORT, SPI_CS_PIN);

    return retval;
}

/**
 * @brief Read from a register in DRV8353 chip
 * @param reg_addr The 4-bit register address (lowest 4 bits of the byte)
 * @retval The 16-bit read data, only bits 10 to 0 are meaningful.
 */
uint16_t DRV8353_Read(uint8_t reg_addr) {
    uint16_t spi_out_val = (reg_addr & 0x0F);
    spi_out_val <<= DRV_ADDR_SHIFT;
    spi_out_val += DRV_RW;
    return DRV8353_Transaction(spi_out_val);
}

/**
 * @brief Write to a register in DRV8353 chip
 * @param reg_addr The 4-bit register address (lowest 4 bits of the byte)
 * @param reg_value The new 11-bit register value (aligned right, lowest 11 bits of half-word)
 * @retval The 16-bit read data (previous register value), only bits 10 to 0 are meaningful.
 */
uint16_t DRV8353_Write(uint8_t reg_addr, uint16_t reg_value) {
    uint16_t spi_out_val = (reg_addr & 0x0F);
    spi_out_val <<= DRV_ADDR_SHIFT;
    spi_out_val += (reg_value & DRV_DATA);
    return DRV8353_Transaction(spi_out_val);
}

/**
 * @brief  Set the Current Sense Amplifier gain in DRV8353 chip
 * @param  gain Choice of 5V/V, 10V/V, 20V/V, or 40V/V. Use the predefined enum
 * @retval RETVAL_OK if setting correctly applied, otherwise RETVAL_FAIL
 */
uint8_t DRV8353_SetGain(DRV_Gain gain) {
    uint16_t temp_csa;
    // Input check
    if(gain <= 0x03) {

        // First read the present value
        temp_csa = DRV8353_Read(DRVREG_CSA);
        // Choose new gain setting
        temp_csa &= ~(DRVBIT_CSA_GAIN);
        temp_csa |= (gain << DRVBIT_CSA_GAIN_SHIFT);

        // Write the new value
        DRV8353_Write(DRVREG_CSA, temp_csa);
        // Double-check
        temp_csa = DRV8353_Read(DRVREG_CSA);
        if( (temp_csa & DRVBIT_CSA_GAIN) == (gain << DRVBIT_CSA_GAIN_SHIFT) ) {
            return RETVAL_OK;
        }
    }
    return RETVAL_FAIL;
}

/**
 * @brief  Retrieves the currently set Current Sense Amplifier gain in DRV8353 chip
 * @retval Value from DRV_Gain enum. If chip isn't connected, DRV_Gain_Unknown will be returned.
 */
DRV_Gain DRV8353_GetGain(void) {
    uint16_t temp_csa;
    if(DRV8353_CheckConnection() == RETVAL_OK) {
        temp_csa = DRV8353_Read(DRVREG_CSA);
        if((temp_csa & DRVBIT_CSA_GAIN) == 0x00) {
            return DRV_Gain_5;
        } else if((temp_csa & DRVBIT_CSA_GAIN) == DRVBIT_CSA_GAIN_0) {
            return DRV_Gain_10;
        } else if((temp_csa & DRVBIT_CSA_GAIN) == DRVBIT_CSA_GAIN_1) {
            return DRV_Gain_20;
        } else if((temp_csa & DRVBIT_CSA_GAIN) == (DRVBIT_CSA_GAIN_1 | DRVBIT_CSA_GAIN_0)) {
            return DRV_Gain_40;
        }
    }
    return DRV_Gain_Unknown;
}

/**
 * @brief  Set the VDS limit voltage in DRV8353 chip
 * @param  gain Choice of DRV_VDS_Limit (0.06V to 2.0V)
 * @retval RETVAL_OK if setting correctly applied, otherwise RETVAL_FAIL
 */
uint8_t DRV8353_SetVDSLimit(DRV_VDS_Limit lmt) {
    uint16_t temp_ocp;
    // Input check
    if(lmt <= 0xF) {
        // First read the present value
        temp_ocp = DRV8353_Read(DRVREG_CSA);
        // Choose new limit
        temp_ocp &= ~(DRVBIT_OCP_VDSLVL);
        temp_ocp |= lmt;
        // Write the new value
        DRV8353_Write(DRVREG_CSA, temp_ocp);
        // Double-check
        temp_ocp = DRV8353_Read(DRVREG_CSA);
        if((temp_ocp & DRVBIT_OCP_VDSLVL) == lmt) {
            return RETVAL_OK;
        }
    }
    return RETVAL_FAIL;
}

/**
 * @brief  Retrieves the currently set VDS limit voltage in DRV8353 chip
 * @retval Value from DRV_VDS_Limit enum. If chip isn't connected, DRV_VDS_Unknown will be returned.
 */
DRV_VDS_Limit DRV8353_GetVDSLimit(void) {
    uint16_t temp_ocp;
    if(DRV8353_CheckConnection() == RETVAL_OK) {
        temp_ocp = DRV8353_Read(DRVREG_OCP) & DRVBIT_OCP_VDSLVL;
        return (DRV_VDS_Limit)temp_ocp; // the register value directly maps to the enum settings
    }
    return DRV_VDS_Unknown;
}

/**
 * @brief  Set the gate drive strength in DRV8353 chip
 * @param  strength: bitmapped field for maximum gate drive current
 *         This is a bitmapped field. Bits 15-12 are for the upper gates rising edge,
 *         bits 11-8 are for upper gates falling edge, bits 7-4 are for lower gates
 *         rising edge, and bits 3-0 are for lower gates falling edge. There are 16 choices
 *         per each gate. Rising edge ranges between 50mA to 1000mA and falling edge is
 *         between 100mA to 2000mA. Smaller number is less current. Check the DRV8353
 *         datasheet for more details.
 * @retval RETVAL_OK if setting correctly applied, otherwise RETVAL_FAIL
 */
uint8_t DRV8353_SetGateStrength(uint32_t strength) {
    uint16_t temp_hs, temp_ls, temp_check;
    if(strength <= 0x0000FFFFu) {
        // Don't care about previous value, only other field is LOCK which will be set to 000b
        temp_hs = ((strength & 0xFF00) >> 8u);
        DRV8353_Write(DRVREG_GATEH, temp_hs);

        // Get present value
        temp_ls = DRV8353_Read(DRVREG_GATEL);
        // Set new drive strength
        temp_ls &= ~(DRVBIT_GATEL_IDRIVELS | DRVBIT_GATEL_IDRIVENLS);
        temp_ls |= (strength & 0x00FF);
        DRV8353_Write(DRVREG_GATEL, temp_ls);

        // Double-check
        temp_check = DRV8353_Read(DRVREG_GATEH);
        if((temp_check & (DRVBIT_GATEH_IDRIVEHS | DRVBIT_GATEH_IDRVENHS)) == temp_hs) {
            temp_check = DRV8353_Read(DRVREG_GATEL);
            if((temp_check & DRV_DATA) == temp_ls) {
                return RETVAL_OK;
            }
        }
    }
    return RETVAL_FAIL;
}

/**
 * @brief  Retrieves the currently set gate strength current from the DRV8353 chip
 * @retval Value from gate drive registers. An unconnected chip will return 0xFFFFFFFF, which is an invalid answer.
 */
uint32_t DRV8353_GetGateStrength(void) {
    uint16_t temp_hs, temp_ls;
    if(DRV8353_CheckConnection() == RETVAL_OK) {
        temp_hs = DRV8353_Read(DRVREG_GATEH);
        Delay(1);
        temp_ls = DRV8353_Read(DRVREG_GATEL);
        temp_hs = (temp_hs & (DRVBIT_GATEH_IDRIVEHS | DRVBIT_GATEH_IDRVENHS)) << 8u;
        temp_hs = temp_hs | (temp_ls & (DRVBIT_GATEL_IDRIVELS | DRVBIT_GATEL_IDRIVENLS));
        return temp_hs;
    }
    return 0xFFFFFFFFu;
}



/**
 * @brief Select shunt amplifier inputs for calibration
 *
 * This function can either short the inputs for calibration,
 * or return them to normal operation. The input value is
 * a bitfield of the three channels. Any channel set to 1
 * will be shorted, and any set to 0 will be set back to
 * normal.
 *
 * @param channel Bit-packed settings for the three channels.
 *        Valid bits are DRV_CHANNEL_A_CAL, DRV_CHANNEL_B_CAL,
 *        and DRV_CHANNEL_C_CAL.
 * @retval None
 */
void DRV8353_SetCalibration(uint8_t channel) {
    uint16_t temp_csa;

    // Check that input is valid
    if(channel <= (DRV_CHANNEL_A_CAL|DRV_CHANNEL_B_CAL|DRV_CHANNEL_C_CAL)) {
        // First read the present value
        temp_csa = DRV8353_Read(DRVREG_CSA);

        // Select the inputs to short for calibration
        if((channel & DRV_CHANNEL_A_CAL) != 0) {
            temp_csa |= DRVBIT_CSA_CALA;
        } else {
            temp_csa &= ~(DRVBIT_CSA_CALA);
        }
        if((channel & DRV_CHANNEL_B_CAL) != 0) {
            temp_csa |= DRVBIT_CSA_CALB;
        } else {
            temp_csa &= ~(DRVBIT_CSA_CALB);
        }
        if((channel & DRV_CHANNEL_C_CAL) != 0) {
            temp_csa |= DRVBIT_CSA_CALC;
        } else {
            temp_csa &= ~(DRVBIT_CSA_CALC);
        }

        // Write the new value
        DRV8353_Write(DRVREG_CSA, temp_csa);
    }
}

/**
 * @brief  Checks if the DRV8353 SPI connection is working.
 *
 *         Simply reads the control register and makes sure its value matches
 *         what was set during the initialization phase. If the chip isn't
 *         powered up or connected properly, the value will probably be all
 *         '1's instead of what was written.
 * @retval RETVAL_OK if connected, RETVAL_FAIL otherwise
 */
static uint8_t DRV8353_CheckConnection(void) {
    uint16_t tempreg;
    tempreg = DRV8353_Read(DRVREG_CTRL) & DRV_DATA;
    if(tempreg == DRVBIT_CTRL_OCPACT) {
        return RETVAL_OK;
    }
    return RETVAL_FAIL;
}

uint8_t uiDRV_GetGateStrength(uint8_t* valptr) {
    data_packet_pack_32b(valptr, DRV8353_GetGateStrength());
    return DATA_PACKET_SUCCESS;
}

uint8_t uiDRV_SetGateStrength(uint8_t* valptr) {
    return DRV8353_SetGateStrength(data_packet_extract_32b(valptr));
}

uint8_t uiDRV_GetVdsLimit(uint8_t* valptr) {
    data_packet_pack_8b(valptr, DRV8353_GetVDSLimit());
    return DATA_PACKET_SUCCESS;
}

uint8_t uiDRV_SetVdsLimit(uint8_t* valptr) {
    return DRV8353_SetVDSLimit(data_packet_extract_8b(valptr));
}

uint8_t uiDRV_GetCsaGain(uint8_t* valptr) {
    data_packet_pack_8b(valptr, DRV8353_GetGain());
    return DATA_PACKET_SUCCESS;
}

uint8_t uiDRV_SetCsaGain(uint8_t* valptr) {
    return DRV8353_SetGain(data_packet_extract_8b(valptr));
}
