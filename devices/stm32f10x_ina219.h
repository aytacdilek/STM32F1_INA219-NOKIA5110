/*******************************************************************************
* File  		:	stm32f10x_ina219.h
* Description	: 	STM32F10x library for INA219 digital current-shunt monitor sensor
* Author		: 	Aytac Dilek
* Note			: 	GNU General Public License, version 3 (GPL-3.0)
*******************************************************************************/

#ifndef __STM32F10X_INA219_H
#define __STM32F10X_INA219_H


/* Includes *******************************************************************/
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_i2c.h"


#include "open103z_systick.h"


/* Defines ********************************************************************/
#define INA219_I2C						I2C1
#define INA219_I2C_PORT					GPIOB
#define INA219_I2C_PER_CLOCK			RCC_APB1Periph_I2C1
#define INA219_I2C_GPIO_CLOCK			RCC_APB2Periph_GPIOB
#define INA219_I2C_SDA_PIN				GPIO_Pin_7
#define INA219_I2C_SCL_PIN				GPIO_Pin_6
#define INA219_I2C_CLOCK_SPEED			100000

/** I2C Address Options */
#define INA219_I2C_ADDRESS_CONF_0				(0x40 << 1)			// A0 = GND, A1 = GND
#define INA219_I2C_ADDRESS_CONF_1				(0x41 << 1)			// A0 = VS+, A1 = GND
#define INA219_I2C_ADDRESS_CONF_2				(0x42 << 1)			// A0 = SDA, A1 = GND
#define INA219_I2C_ADDRESS_CONF_3				(0x43 << 1)			// A0 = SCL, A1 = GND
#define INA219_I2C_ADDRESS_CONF_4				(0x44 << 1)			// A0 = GND, A1 = VS+
#define INA219_I2C_ADDRESS_CONF_5				(0x45 << 1)			// A0 = VS+, A1 = VS+
#define INA219_I2C_ADDRESS_CONF_6				(0x46 << 1)			// A0 = SDA, A1 = VS+
#define INA219_I2C_ADDRESS_CONF_7				(0x47 << 1)			// A0 = SCL, A1 = VS+
#define INA219_I2C_ADDRESS_CONF_8				(0x48 << 1)			// A0 = GND, A1 = SDA
#define INA219_I2C_ADDRESS_CONF_9				(0x49 << 1)			// A0 = VS+, A1 = SDA
#define INA219_I2C_ADDRESS_CONF_A				(0x4A << 1)			// A0 = SDA, A1 = SDA
#define INA219_I2C_ADDRESS_CONF_B				(0x4B << 1)			// A0 = SCL, A1 = SDA
#define INA219_I2C_ADDRESS_CONF_C				(0x4C << 1)			// A0 = GND, A1 = SCL
#define INA219_I2C_ADDRESS_CONF_D				(0x4D << 1)			// A0 = VS+, A1 = SCL
#define INA219_I2C_ADDRESS_CONF_E				(0x4E << 1)			// A0 = SDA, A1 = SCL
#define INA219_I2C_ADDRESS_CONF_F				(0x4F << 1)			// A0 = SCL, A1 = SCL
#define INA219_I2C_ADDRESS_DEFAULT				INA219_I2C_ADDRESS_CONF_0


/** Register Addresses ********************************************************/
#define INA219_REG_CONFIG						(0x00)	// CONF REGISTER (R/W)
#define INA219_REG_SHUNTVOLTAGE					(0x01)	// SHUNT VOLTAGE REGISTER (R)
#define INA219_REG_BUSVOLTAGE					(0x02) 	// BUS VOLTAGE REGISTER (R)
#define INA219_REG_POWER						(0x03)	// POWER REGISTER (R)
#define INA219_REG_CURRENT						(0x04)	// CURRENT REGISTER (R)
#define INA219_REG_CALIBRATION					(0x05)	// CALIBRATION REGISTER (R/W)



/** Configuration Register ****************************************************/
#define INA219_CONFIG_RESET						(0x8000)  // Reset Bit

#define INA219_CONFIG_BVOLTAGERANGE_MASK		(0x2000)  // Bus Voltage Range Mask
#define INA219_CONFIG_BVOLTAGERANGE_16V			(0x0000)  // 0-16V Range
#define INA219_CONFIG_BVOLTAGERANGE_32V			(0x2000)  // 0-32V Range

#define INA219_CONFIG_GAIN_MASK					(0x1800)  // Gain Mask
#define INA219_CONFIG_GAIN_1_40MV				(0x0000)  // Gain 1, 40mV Range
#define INA219_CONFIG_GAIN_2_80MV				(0x0800)  // Gain 2, 80mV Range
#define INA219_CONFIG_GAIN_4_160MV				(0x1000)  // Gain 4, 160mV Range
#define INA219_CONFIG_GAIN_8_320MV				(0x1800)  // Gain 8, 320mV Range

#define INA219_CONFIG_BADCRES_MASK				(0x0780)  // Bus ADC Resolution Mask
#define INA219_CONFIG_BADCRES_9BIT				(0x0080)  // 9-bit bus res = 0..511
#define INA219_CONFIG_BADCRES_10BIT				(0x0100)  // 10-bit bus res = 0..1023
#define INA219_CONFIG_BADCRES_11BIT				(0x0200)  // 11-bit bus res = 0..2047
#define INA219_CONFIG_BADCRES_12BIT				(0x0400)  // 12-bit bus res = 0..4097

#define INA219_CONFIG_SADCRES_MASK				(0x0078)	// Shunt ADC Resolution and Averaging Mask
#define INA219_CONFIG_SADCRES_9BIT_1S_84US		(0x0000)	// 1 x 9-bit shunt sample
#define INA219_CONFIG_SADCRES_10BIT_1S_148US	(0x0008)	// 1 x 10-bit shunt sample
#define INA219_CONFIG_SADCRES_11BIT_1S_276US	(0x0010)	// 1 x 11-bit shunt sample
#define INA219_CONFIG_SADCRES_12BIT_1S_532US	(0x0018)	// 1 x 12-bit shunt sample
#define INA219_CONFIG_SADCRES_12BIT_2S_1060US	(0x0048)	// 2 x 12-bit shunt samples averaged together
#define INA219_CONFIG_SADCRES_12BIT_4S_2130US	(0x0050)	// 4 x 12-bit shunt samples averaged together
#define INA219_CONFIG_SADCRES_12BIT_8S_4260US	(0x0058)	// 8 x 12-bit shunt samples averaged together
#define INA219_CONFIG_SADCRES_12BIT_16S_8510US	(0x0060)	// 16 x 12-bit shunt samples averaged together
#define INA219_CONFIG_SADCRES_12BIT_32S_17MS	(0x0068)	// 32 x 12-bit shunt samples averaged together
#define INA219_CONFIG_SADCRES_12BIT_64S_34MS	(0x0070)	// 64 x 12-bit shunt samples averaged together
#define INA219_CONFIG_SADCRES_12BIT_128S_69MS	(0x0078)	// 128 x 12-bit shunt samples averaged together

#define INA219_CONFIG_MODE_MASK					(0x0007)  // Operating Mode Mask
#define INA219_CONFIG_MODE_POWERDOWN			(0x0000)
#define INA219_CONFIG_MODE_SVOLT_TRIGGERED		(0x0001)
#define INA219_CONFIG_MODE_BVOLT_TRIGGERED		(0x0002)
#define INA219_CONFIG_MODE_SANDBVOLT_TRIGGERED	(0x0003)
#define INA219_CONFIG_MODE_ADCOFF				(0x0004)
#define INA219_CONFIG_MODE_SVOLT_CONTINUOUS		(0x0005)
#define INA219_CONFIG_MODE_BVOLT_CONTINUOUS		(0x0006)
#define INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS	(0x0007)



/* Enumerations ***************************************************************/
typedef enum{
	INA219_BusVoltageRange_16V = 0x0000,					// '0'=16V
	INA219_BusVoltageRange_32V = 0x2000,					// '1'=32V (default value)
}INA219BusVoltageRange_TypeDef;								// Bus voltage range (Bit 13),

typedef enum{
	INA219_GainRange_1_40mV = 0x0000,						// Gain 1, 40mV Range
	INA219_GainRange_2_80mV = 0x0800,						// Gain 2, 80mV Range
	INA219_GainRange_4_160mV = 0x1000,						// Gain 4, 160mV Range
	INA219_GainRange_8_320mV = 0x1800,						// Gain 8, 320mV Range (default)
}INA219GainRange_TypeDef;

typedef enum{
	INA219_BusADCResolution_9Bit = 0x0080,					// 9-bit bus res = 0..511
	INA219_CBusADCResolution_10Bit = 0x0100,				// 10-bit bus res = 0..1023
	INA219_BusADCResolution_11Bit = 0x0200,					// 11-bit bus res = 0..2047
	INA219_BusADCResolution_12Bit = 0x0400,					// 12-bit bus res = 0..4097
}INA219BusADCResolution_TypeDef;							// Bus ADC Resolution / Averaging

typedef enum{
	INA219_ShuntADCResolution_9Bit_1S_84uS = 0x0000,		// 1 x 9-bit shunt sample
	INA219_ShuntADCResolution_10Bit_1S_148uS = 0x0008,		// 1 x 10-bit shunt sample
	INA219_ShuntADCResolution_11Bit_1S_276uS = 0x0010,		// 1 x 11-bit shunt sample
	INA219_ShuntADCResolution_12Bit_1S_532uS = 0x0018,		// 1 x 12-bit shunt sample, (DEFAULT)
	INA219_ShuntADCResolution_9Bit_2S_1060uS = 0x0048,		// 2 x 12-bit shunt samples averaged together
	INA219_ShuntADCResolution_12Bit_4S_2130uS = 0x0050,		// 4 x 12-bit shunt samples averaged together
	INA219_ShuntADCResolution_12Bit_8S_4260uS = 0x0058,		// 8 x 12-bit shunt samples averaged together
	INA219_ShuntADCResolution_12Bit_16S_8510uS = 0x0060,	// 16 x 12-bit shunt samples averaged together
	INA219_ShuntADCResolution_12Bit_32S_17mS = 0x0068,		// 32 x 12-bit shunt samples averaged together
	INA219_ShuntADCResolution_12Bit_64S_34mS = 0x0070,		// 64 x 12-bit shunt samples averaged together
	INA219_ShuntADCResolution_12Bit_128S_69mS = 0x0078,		// 128 x 12-bit shunt samples averaged together
}INA219ShuntADCResolution_TypeDef;							// Shunt ADC Resolution/Averaging

typedef enum{
	INA219_Mode_PowerDown = 0x0000,
	INA219_Mode_ShuntVoltageTriggered = 0x0001,
	INA219_Mode_BusVoltageTriggered = 0x0002,
	INA219_Mode_ShuntAndBusVoltageTriggered = 0x0003,
	INA219_Mode_ADCOff = 0x0004,
	INA219_Mode_ShuntVoltageContinuous = 0x0005,
	INA219_Mode_BusBoltageContinuous = 0x0006,
	INA219_Mode_ShuntAndBusVoltageContinuous = 0x0007,		// Default
}INA219Mode_TypeDef;

typedef enum{
	INA219_Mask_BusVoltageRange = 0x2000,
	INA219_Mask_GainRange = 0x1800,
	INA219_Mask_BusADCResolution = 0x0780,
	INA219_Mask_ShuntADCResolution = 0x0078,
	INA219_Mask_Mode = 0x0007,
}INA219Mask_Typedef;

typedef struct{
	INA219BusVoltageRange_TypeDef INA219_BusVoltageRange;
	INA219GainRange_TypeDef INA219_GainRange;
	INA219BusADCResolution_TypeDef INA219_BusADCResolution;
	INA219ShuntADCResolution_TypeDef INA219_ShuntADCResolution;
	INA219Mode_TypeDef INA219_Mode;
}INA219_InitTypeDef;



/* Global Variables ***********************************************************/
uint8_t ina219_i2caddr;
uint32_t ina219_calValue;
// The following multipliers are used to convert raw current and power
// values to mA and mW, taking into account the current config settings
uint32_t ina219_currentDivider_mA;
uint32_t ina219_powerDivider_mW;


/* Global Functions ***********************************************************/
void ina219_init(void);
void ina219_writeRegister(uint8_t reg, uint16_t data);
void ina219_readRegister(uint8_t reg, uint16_t *data);
void ina219_setCalibration_32V_2A(void);
void ina219_setCalibration_32V_1A(void);
void ina219_setCalibration_16V_400mA(void);
int16_t ina219_getBusVoltage_raw(void);
int16_t ina219_getShuntVoltage_raw(void);
int16_t ina219_getCurrent_raw(void);
float ina219_getShuntVoltage_mV(void);
float ina219_getBusVoltage_V(void);
float ina219_getCurrent_mA(void);



void ina219_powerOnReset(void);
void ina219_setBusVoltageRange(INA219BusVoltageRange_TypeDef bus_voltage);
void ina219_setGainRange(INA219GainRange_TypeDef gain);
void ina219_setBusADCResolution(INA219BusADCResolution_TypeDef resolution);
void ina219_setShuntADCResolution(INA219ShuntADCResolution_TypeDef resolution);
void ina219_setOperatingMode(INA219Mode_TypeDef mode);

void ina219_updateConfigurationRegister(uint16_t reg_bits, INA219Mask_Typedef mask);



#endif // __STM32F10X_INA219_H
