/******************************************************************************
 * Filename: drv8353.h
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

#ifndef __SPI_H
#define __SPI_H

// DRV8353(R)S minimum SCLK period = 100ns, so 10MHz max
#define SPI_BAUDMAX     10000000U
#define SPI_CLKDIV      32
#define SPI_BAUDRATE    5312500U // 170MHz / 32 = 5.3125MHz

// Data changes on rising edge, clock idles low
#define SPI_PHASE       1
#define SPI_POLARITY    0

#define SPI_LSBFIRST    0
#define SPI_DATASIZE    16

// Sit and spin in a loop waiting for SPI completion
// If it takes longer than about 3us, quit
#define SPI_MAX_WAIT_CYCLES     500

// Settings for shunt amplifier gain
typedef enum _DRV_Gain {
    DRV_Gain_5,
    DRV_Gain_10,
    DRV_Gain_20,
    DRV_Gain_40
} DRV_Gain;

// Settings for shunt amplifier calibration
#define DRV_CHANNEL_A_CAL       0x01
#define DRV_CHANNEL_B_CAL       0x02
#define DRV_CHANNEL_C_CAL       0x04

// SPI read/write bit - 1 = read, 0 = write
#define DRV_RW              0x8000
// Address bits
#define DRV_ADDR            0x7800
#define DRV_ADDR_SHIFT      (11u)
// Data bits
#define DRV_DATA            0x07FF

// Registers
#define DRVREG_FAULT1       0x00
#define DRVREG_FAULT2       0x01
#define DRVREG_CTRL         0x02
#define DRVREG_GATEH        0x03
#define DRVREG_GATEL        0x04
#define DRVREG_OCP          0x05
#define DRVREG_CSA          0x06
#define DRVREG_CAL          0x07

// Bit definitions
#define DRVBIT_FAULT1_FAULT     0x400 // All faults OR'd together
#define DRVBIT_FAULT1_VDSOCP    0x200 // VDS monitor over current
#define DRVBIT_FAULT1_GDF       0x100 // Gate driver fault
#define DRVBIT_FAULT1_UVLO      0x080 // Under voltage lockout
#define DRVBIT_FAULT1_OTSD      0x040 // Over temperature shutdown
#define DRVBIT_FAULT1_VDSHA     0x020 // VDS over current fault, A high side
#define DRVBIT_FAULT1_VDSLA     0x010 // VDS over current fault, A low side
#define DRVBIT_FAULT1_VDSHB     0x008 // VDS over current fault, B high side
#define DRVBIT_FAULT1_VDSLB     0x004 // VDS over current fault, B low side
#define DRVBIT_FAULT1_VDSHC     0x002 // VDS over current fault, C high side
#define DRVBIT_FAULT1_VDSLC     0x001 // VDS over current fault, C low side

#define DRVBIT_FAULT2_SAOC      0x400 // Shunt amplifier over current, channel A
#define DRVBIT_FAULT2_SBOC      0x200 // Shunt amplifier over current, channel B
#define DRVBIT_FAULT2_SCOC      0x100 // Shunt amplifier over current, channel C
#define DRVBIT_FAULT2_OTW       0x080 // Over temperature warning
#define DRVBIT_FAULT2_GDUV      0x040 // Gate driver under voltage
#define DRVBIT_FAULT2_VGSHA     0x020 // Gate drive fault, A high side
#define DRVBIT_FAULT2_VGSLA     0x010 // Gate drive fault, A low side
#define DRVBIT_FAULT2_VGSHB     0x008 // Gate drive fault, B high side
#define DRVBIT_FAULT2_VGSLB     0x004 // Gate drive fault, B low side
#define DRVBIT_FAULT2_VGSHC     0x002 // Gate drive fault, C high side
#define DRVBIT_FAULT2_VGSLC     0x001 // Gate drive fault, C low side

#define DRVBIT_CTRL_OCPACT      0x400 // shutdown only the fault half-bridge (0) or all three (1)
#define DRVBIT_CTRL_DISGDUV     0x200 // disable gate driver under voltage fault
#define DRVBIT_CTRL_DISGDF      0x100 // disable gate fault
#define DRVBIT_CTRL_OTWREP      0x080 // report over temp warning on fault bit and pin
#define DRVBIT_CTRL_PWMMODE     0x060 // 00 - 6x pwm, 01 - 3x pwm, 10 - 1x pwm, 11 - independent pwm
#define DRVBIT_CTRL_PWMMODE_1       0x040
#define DRVBIT_CTRL_PWMMODE_0       0x020
#define DRVBIT_CTRL_1PWMCOM     0x010 // 0 - 1x pwm uses synchronous rectification, 1 - asynchronous
#define DRVBIT_CTRL_1PWMDIR     0x008 // change direction in 1x pwm mode
#define DRVBIT_CTRL_COAST       0x004 // All FETs in High-Z (gates output low)
#define DRVBIT_CTRL_BRAKE       0x002 // All high FETs High-Z, low FETs on
#define DRVBIT_CTRL_CLRFLT      0x001 // Clear latched faults, alternative to resetting enable pin

#define DRVBIT_GATEH_LOCK       0x700 // Write 110 to lock settings, 011 to unlock. Other patterns ignored
#define DRVBIT_GATEH_LOCK_2         0x400
#define DRVBIT_GATEH_LOCK_1         0x200
#define DRVBIT_GATEH_LOCK_0         0x100
#define DRVBIT_GATEH_IDRIVEHS   0x0F0 // Set current limit for high side source (50mA to 1000mA)
#define DRVBIT_GATEH_IDRIVEHS_3     0x080
#define DRVBIT_GATEH_IDRIVEHS_2     0x040
#define DRVBIT_GATEH_IDRIVEHS_1     0x020
#define DRVBIT_GATEH_IDRIVEHS_0     0x010
#define DRVBIT_GATEH_IDRVENHS   0x00F // Set current limit for high side sink (100mA to 2000mA)
#define DRVBIT_GATEH_IDRIVENHS_3    0x008
#define DRVBIT_GATEH_IDRIVENHS_2    0x004
#define DRVBIT_GATEH_IDRIVENHS_1    0x002
#define DRVBIT_GATEH_IDRIVENHS_0    0x001

#define DRVBIT_GATEL_CBC        0x400 // When OCP_MODE = 01, sets whether a new PWM input is needed to clear VDSOCP and SENOCP faults
#define DRVBIT_GATEL_TDRIVE     0x300 // Time setting for peak gate current drive: 00 - 500ns, 01 - 1us, 10 - 2us, 11 - 4us
#define DRVBIT_GATEL_TDRIVE_1       0x200
#define DRVBIT_GATEL_TDRIVE_0       0x100
#define DRVBIT_GATEL_IDRIVELS   0x0F0 // Set current limit for low side source (50mA to 1000mA)
#define DRVBIT_GATEL_IDRIVELS_3     0x080
#define DRVBIT_GATEL_IDRIVELS_2     0x040
#define DRVBIT_GATEL_IDRIVELS_1     0x020
#define DRVBIT_GATEL_IDRIVELS_0     0x010
#define DRVBIT_GATEL_IDRIVENLS  0x00F // Set current limit for low side sink (100mA to 2000mA)
#define DRVBIT_GATEL_IDRIVENLS_3    0x008
#define DRVBIT_GATEL_IDRIVENLS_2    0x004
#define DRVBIT_GATEL_IDRIVENLS_1    0x002
#define DRVBIT_GATEL_IDRIVENLS_0    0x001

#define DRVBIT_OCP_TRETRY       0x400 // 0 - retry is 8ms, 1 - retry is 50us
#define DRVBIT_OCP_DEADTIME     0x300 // 00 - 50ns deadtime, 01 - 100ns, 10 - 200ns, 11 - 400ns
#define DRVBIT_OCP_DEADTIME_1       0x200
#define DRVBIT_OCP_DEADTIME_0       0x100
#define DRVBIT_OCP_MODE         0x0C0 // 00 - over current is a latched fault, 01 - auto retry, 10 - report only, 11 - no action
#define DRVBIT_OCP_MODE_1           0x080
#define DRVBIT_OCP_MODE_0           0x040
#define DRVBIT_OCP_DEG          0x030 // 00 - deglitch time is 1us, 01 - 2us, 10 - 4us, 11 - 8us
#define DRVBIT_OCP_DEG_1            0x020
#define DRVBIT_OCP_DEG_0            0x010
#define DRVBIT_OCP_VDSLVL       0x00F // Sets the VDS over current limit voltage
#define DRVBIT_OCP_VDSLVL_3         0x008
#define DRVBIT_OCP_VDSLVL_2         0x004
#define DRVBIT_OCP_VDSLVL_1         0x002
#define DRVBIT_OCP_VDSLVL_0         0x001

#define DRVBIT_CSA_FET          0x400 // 0 - sense amplifer in shunt resistor mode, 1 - low side VDS sensing
#define DRVBIT_CSA_VREFDIV      0x200 // 0 - Vref (unidirectional), 1 - Vref/2 (bidirectional)
#define DRVBIT_CSA_LSREF        0x100 // 0 - VDS_OCP measured from SH to SP, 1 - measured from SH to SN
#define DRVBIT_CSA_GAIN         0x0C0 // 00 - 5V/V, 01 - 10V/V, 10 - 20V/V, 11 - 40V/V
#define DRVBIT_CSA_GAIN_1           0x080
#define DRVBIT_CSA_GAIN_0           0x040
#define DRVBIT_CSA_DISSEN       0x020 // set to disable sense overcurrent fault
#define DRVBIT_CSA_CALA         0x010 // set to short amplifier A inputs for calibration
#define DRVBIT_CSA_CALB         0x008 // short amplifier B
#define DRVBIT_CSA_CALC         0x004 // short amplifier C
#define DRVBIT_CSA_SENLVL       0x003 // sense over current voltage level
#define DRVBIT_CSA_SENLVL_1         0x002
#define DRVBIT_CSA_SENLVL_0         0x001

#define DRVBIT_CAL_MODE         0x001 // set to use internal auto-calibration routine

void DRV8353_Init(void);
uint16_t DRV8353_Transaction(uint16_t DataOut);
uint16_t DRV8353_Read(uint8_t reg_addr);
uint16_t DRV8353_Write(uint8_t reg_addr, uint16_t reg_value);
void DRV8353_SetGain(DRV_Gain gain);

#endif // __SPI_H
