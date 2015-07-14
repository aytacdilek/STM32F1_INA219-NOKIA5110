/*******************************************************************************
 * File				:	stm32f10x_ina219.c
 * Description		:	STM32F10x library for INA219 sensor which reads
 * 						shunt, current and power ratings
 * Datum			:	2015.07.15
 * Version			:	1.1
 * Author			:	Aytac Dilek
 * email			:	aytacdilek@gmail.com
 * Web				:
 * Platform			:	OPEN103Z-B
 * CPU				:	STM32F103ZET6
 * IDE				:	CooCox CoIDE 1.7.7
 * GCC				:	4.8 2014q2
 * Module			:	INA219
 * Function			:	Read shunt, current and power readings
 * Pin Definitions	:	PB6 => SCL
 * 						PB7 => SDA
 * Note				:	For calculating calibration values i used adafruit
 * 						driver. Special thanks to adafruit team for sharing. In
 * 						long term, i will change this methods to make more generic
 * 						and task oriented. This is first release, i have tested
 * 						on evaluation board, it works perfectly.
 * 						This is not the final release, but it works anyway.
 * 						Feel free to send me email, ask whatever you want.
*******************************************************************************/


/* Includes *******************************************************************/
#include "stm32f10x_ina219.h"



/* Internal Functions *********************************************************/
void ina219_gpio_init(void);
void ina219_i2c_init(void);
void ina219_configureRegisters(void);



/*******************************************************************************
* Function Name		:	ina219_init
* Description		:	Initialize INA219 sensor
* Input				:	None
* Output			:	None
* Return			:	None
* Attention			:	None
*******************************************************************************/
void ina219_init(void){
	/* Enable gpio peripheral */
	ina219_gpio_init();
	/* Enable i2c peripheral */
	ina219_i2c_init();
	/* Enable SysTick Timer */
	systick_init();
	/* Update Configuration registers of INA219 */
	ina219_configureRegisters();
}




/*******************************************************************************
* Function Name		:	ina219_gpio_init
* Description		:	Initialize GPIO peripheral
* Input				:	None
* Output			:	None
* Return			:	None
* Attention			:	None
*******************************************************************************/
void ina219_gpio_init(void){
	GPIO_InitTypeDef  GPIO_InitStructure;

	/* Enable Clocks for gpio peripherals */
	RCC_APB2PeriphClockCmd(INA219_I2C_GPIO_CLOCK, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	/* Configure I2C1 pins: PB6->SCL and PB7->SDA */
	GPIO_InitStructure.GPIO_Pin =  INA219_I2C_SCL_PIN | INA219_I2C_SDA_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_Init(INA219_I2C_PORT, &GPIO_InitStructure);
}




/*******************************************************************************
* Function Name		:	ina219_i2c_init
* Description		:	Initialize I2C peripheral
* Input				:	None
* Output			:	None
* Return			:	None
* Attention			:	None
*******************************************************************************/
void ina219_i2c_init(void){
	I2C_InitTypeDef  I2C_InitStructure;

	/* Enable peripheral clocks */
	RCC_APB1PeriphClockCmd(INA219_I2C_PER_CLOCK, ENABLE);

	I2C_DeInit(INA219_I2C);
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = 0x30;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed = INA219_I2C_CLOCK_SPEED;
	I2C_Init(INA219_I2C, &I2C_InitStructure);

	I2C_Cmd(INA219_I2C, ENABLE);

	I2C_AcknowledgeConfig(INA219_I2C, ENABLE);
}



/*******************************************************************************
* Function Name		:	ina219_configureRegisters
* Description		:	Update Configuration registers of INA219
* Input				:	None
* Output			:	None
* Return			:	None
* Attention			:	None
*******************************************************************************/
void ina219_configureRegisters(void){
	ina219_currentDivider_mA = 0;
	ina219_powerDivider_mW = 0;

	systick_delayMs(15);

	ina219_setCalibration_32V_2A();
}



/*******************************************************************************
* Function Name		:	ina219_writeRegister
* Description		:	Sends a single command byte over I2C
* Input				:	reg -> 8-bit data, register address
* 						data -> 16-bit data
* Output			:	None
* Return			:	None
* Attention			:	None
*******************************************************************************/
void ina219_writeRegister(uint8_t reg, uint16_t data){
	/* Wait while the bus is busy */
	while(I2C_GetFlagStatus(INA219_I2C, I2C_FLAG_BUSY));

	/* Enable Acknowledge */
	I2C_AcknowledgeConfig(INA219_I2C, ENABLE);

	/* Send "Start" condition */
	I2C_GenerateSTART(INA219_I2C, ENABLE);
	/* Test on EV5 and clear it */
	while(!I2C_CheckEvent(INA219_I2C, I2C_EVENT_MASTER_MODE_SELECT));

	/* Send INA219 address for write */
	I2C_Send7bitAddress(INA219_I2C,  INA219_I2C_ADDRESS_DEFAULT, I2C_Direction_Transmitter);
	/* Test on EV6 and clear it */
	while (!I2C_CheckEvent(INA219_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	/* Clear EV6 by setting again the PE bit */
	I2C_Cmd(INA219_I2C, ENABLE);


	/* Send "Read User Register" command */
	I2C_SendData(INA219_I2C, reg);
	/* Test on EV8 and clear it */
	while (!I2C_CheckEvent(INA219_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	/* Send MSB of data */
	I2C_SendData(INA219_I2C, (uint8_t) (data >> 8));
	/* Test on EV8 and clear it */
	while (!I2C_CheckEvent(INA219_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	/* Send LSB of data */
	I2C_SendData(INA219_I2C, (uint8_t) (data & 0xFF));
	/* Test on EV8 and clear it */
	while (!I2C_CheckEvent(INA219_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	/* Send STOP Condition */
	I2C_GenerateSTOP(INA219_I2C, ENABLE);
}



/*******************************************************************************
* Function Name	:	ina219_readRegister
* Description	:	Reads a 16 bit register values over I2C
* Input			:	reg -> 8-bit data, register address
* 					data -> pointer to a 16-bit data
* Output		:	None
* Return		:	None
* Attention		:	When reading from INA219, the last value stored in the register
* 					pointer by a write operation determines which register is read
* 					during a read operation.
* 					To change the register pointer for a read operation, a new value
* 					must be written to the register pointer.
* 					This write is accomplished by issuing a slave address byte with
* 					the R/W bit LOW, followed by the register pointer byte. No
* 					additional data are required. The master then generates a START
* 					condition and sends the slave address byte with the R/W bit HIGH
* 					to initiate the read command.
*******************************************************************************/
void ina219_readRegister(uint8_t reg, uint16_t *data){
	/* Wait while the bus is busy */
	while(I2C_GetFlagStatus(INA219_I2C, I2C_FLAG_BUSY));

	/* Enable Acknowledge */
	I2C_AcknowledgeConfig(INA219_I2C, ENABLE);

	/* Send "Start" condition */
	I2C_GenerateSTART(INA219_I2C, ENABLE);
	/* Test on EV5 and clear it */
	while(!I2C_CheckEvent(INA219_I2C, I2C_EVENT_MASTER_MODE_SELECT));

	/* Send INA219 address for write */
	I2C_Send7bitAddress(INA219_I2C,  INA219_I2C_ADDRESS_DEFAULT, I2C_Direction_Transmitter);
	/* Test on EV6 and clear it */
	while (!I2C_CheckEvent(INA219_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	/* Clear EV6 by setting again the PE bit */
//	I2C_Cmd(INA219_I2C, ENABLE);

	/* Send "Read User Register" command */
	I2C_SendData(INA219_I2C, reg);
	/* Test on EV8 and clear it */
	while (!I2C_CheckEvent(INA219_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED));



	/* Send "Start" condition */
	I2C_GenerateSTART(INA219_I2C, ENABLE);
	/* Test on EV5 and clear it */
	while(!I2C_CheckEvent(INA219_I2C, I2C_EVENT_MASTER_MODE_SELECT));

	/* Send INA219 address for read */
	I2C_Send7bitAddress(INA219_I2C,  INA219_I2C_ADDRESS_DEFAULT, I2C_Direction_Receiver);
	/* Test on EV6 and clear it */
	while (!I2C_CheckEvent(INA219_I2C, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));



	while(!I2C_CheckEvent(INA219_I2C, I2C_EVENT_MASTER_BYTE_RECEIVED));  /* EV7 */
	*data = I2C_ReceiveData(INA219_I2C) << 8;

	I2C_AcknowledgeConfig(INA219_I2C, DISABLE);
	I2C_GenerateSTOP(INA219_I2C, ENABLE);

	while(!I2C_CheckEvent(INA219_I2C, I2C_EVENT_MASTER_BYTE_RECEIVED));  /* EV7 */
	*data = (uint16_t) I2C_ReceiveData(INA219_I2C) | *data;

	/* Enable Acknowledgement to be ready for another reception */
	I2C_AcknowledgeConfig(INA219_I2C, ENABLE);
}




/*******************************************************************************
* Function Name		:	ina219_setCalibration_32V_2A
* Description		:	Configures to INA219 to be able to measure up to 32V and 2A
* 						of current.  Each unit of current corresponds to 100uA, and
* 						each unit of power corresponds to 2mW. Counter overflow
* 						occurs at 3.2A.
* Input				:	None
* Output			:	None
* Return			:	None
* Attention			:	These calculations assume a 0.1 ohm resistor is present
*******************************************************************************/
void ina219_setCalibration_32V_2A(void){
	INA219_InitTypeDef INA219_InitStructure;

	INA219_InitStructure.INA219_BusVoltageRange = INA219_BusVoltageRange_32V;
	INA219_InitStructure.INA219_GainRange = INA219_GainRange_8_320mV;
	INA219_InitStructure.INA219_BusADCResolution = INA219_BusADCResolution_12Bit;
	INA219_InitStructure.INA219_ShuntADCResolution = INA219_ShuntADCResolution_12Bit_1S_532uS;
	INA219_InitStructure.INA219_Mode = INA219_Mode_ShuntAndBusVoltageContinuous;

	// By default we use a pretty huge range for the input voltage,
	// which probably isn't the most appropriate choice for system
	// that don't use a lot of power.  But all of the calculations
	// are shown below if you want to change the settings.  You will
	// also need to change any relevant register settings, such as
	// setting the VBUS_MAX to 16V instead of 32V, etc.

	// VBUS_MAX = 32V             (Assumes 32V, can also be set to 16V)
	// VSHUNT_MAX = 0.32          (Assumes Gain 8, 320mV, can also be 0.16, 0.08, 0.04)
	// RSHUNT = 0.1               (Resistor value in ohms)

	// 1. Determine max possible current
	// MaxPossible_I = VSHUNT_MAX / RSHUNT
	// MaxPossible_I = 3.2A

	// 2. Determine max expected current
	// MaxExpected_I = 2.0A

	// 3. Calculate possible range of LSBs (Min = 15-bit, Max = 12-bit)
	// MinimumLSB = MaxExpected_I/32767
	// MinimumLSB = 0.000061              (61uA per bit)
	// MaximumLSB = MaxExpected_I/4096
	// MaximumLSB = 0,000488              (488uA per bit)

	// 4. Choose an LSB between the min and max values
	//    (Preferrably a roundish number close to MinLSB)
	// CurrentLSB = 0.0001 (100uA per bit)

	// 5. Compute the calibration register
	// Cal = trunc (0.04096 / (Current_LSB * RSHUNT))
	// Cal = 4096 (0x1000)

	ina219_calValue = 4096;

	// 6. Calculate the power LSB
	// PowerLSB = 20 * CurrentLSB
	// PowerLSB = 0.002 (2mW per bit)

	// 7. Compute the maximum current and shunt voltage values before overflow
	//
	// Max_Current = Current_LSB * 32767
	// Max_Current = 3.2767A before overflow
	//
	// If Max_Current > Max_Possible_I then
	//    Max_Current_Before_Overflow = MaxPossible_I
	// Else
	//    Max_Current_Before_Overflow = Max_Current
	// End If
	//
	// Max_ShuntVoltage = Max_Current_Before_Overflow * RSHUNT
	// Max_ShuntVoltage = 0.32V
	//
	// If Max_ShuntVoltage >= VSHUNT_MAX
	//    Max_ShuntVoltage_Before_Overflow = VSHUNT_MAX
	// Else
	//    Max_ShuntVoltage_Before_Overflow = Max_ShuntVoltage
	// End If

	// 8. Compute the Maximum Power
	// MaximumPower = Max_Current_Before_Overflow * VBUS_MAX
	// MaximumPower = 3.2 * 32V
	// MaximumPower = 102.4W

	// Set multipliers to convert raw current/power values
	ina219_currentDivider_mA = 10;  // Current LSB = 100uA per bit (1000/100 = 10)
	ina219_powerDivider_mW = 2;     // Power LSB = 1mW per bit (2/1)

	// Set Calibration register to 'Cal' calculated above
	ina219_writeRegister(INA219_REG_CALIBRATION, ina219_calValue);

	// Set Config register to take into account the settings above
	uint16_t config = INA219_CONFIG_BVOLTAGERANGE_32V |
					INA219_CONFIG_GAIN_8_320MV |
					INA219_CONFIG_BADCRES_12BIT |
					INA219_CONFIG_SADCRES_12BIT_1S_532US |
					INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;
	ina219_writeRegister(INA219_REG_CONFIG, config);
}



/*******************************************************************************
* Function Name		:	ina219_setCalibration_32V_1A
* Description		:	Configures to INA219 to be able to measure up to 32V and 1A
* 						of current.  Each unit of current corresponds to 40uA, and each
* 						unit of power corresponds to 800W. Counter overflow occurs at
* 						1.3A.
* Input				:	None
* Output			:	None
* Return			:	None
* Attention			:	These calculations assume a 0.1 ohm resistor is present
*******************************************************************************/
void ina219_setCalibration_32V_1A(void){
	// By default we use a pretty huge range for the input voltage,
	// which probably isn't the most appropriate choice for system
	// that don't use a lot of power.  But all of the calculations
	// are shown below if you want to change the settings.  You will
	// also need to change any relevant register settings, such as
	// setting the VBUS_MAX to 16V instead of 32V, etc.

	// VBUS_MAX = 32V		(Assumes 32V, can also be set to 16V)
	// VSHUNT_MAX = 0.32	(Assumes Gain 8, 320mV, can also be 0.16, 0.08, 0.04)
	// RSHUNT = 0.1			(Resistor value in ohms)

	// 1. Determine max possible current
	// MaxPossible_I = VSHUNT_MAX / RSHUNT
	// MaxPossible_I = 3.2A

	// 2. Determine max expected current
	// MaxExpected_I = 1.0A

	// 3. Calculate possible range of LSBs (Min = 15-bit, Max = 12-bit)
	// MinimumLSB = MaxExpected_I/32767
	// MinimumLSB = 0.0000305             (30.5A per bit)
	// MaximumLSB = MaxExpected_I/4096
	// MaximumLSB = 0.000244              (244A per bit)

	// 4. Choose an LSB between the min and max values
	//    (Preferrably a roundish number close to MinLSB)
	// CurrentLSB = 0.0000400 (40A per bit)

	// 5. Compute the calibration register
	// Cal = trunc (0.04096 / (Current_LSB * RSHUNT))
	// Cal = 10240 (0x2800)

	ina219_calValue = 10240;

	// 6. Calculate the power LSB
	// PowerLSB = 20 * CurrentLSB
	// PowerLSB = 0.0008 (800W per bit)

	// 7. Compute the maximum current and shunt voltage values before overflow
	//
	// Max_Current = Current_LSB * 32767
	// Max_Current = 1.31068A before overflow
	//
	// If Max_Current > Max_Possible_I then
	//    Max_Current_Before_Overflow = MaxPossible_I
	// Else
	//    Max_Current_Before_Overflow = Max_Current
	// End If
	//
	// ... In this case, we're good though since Max_Current is less than MaxPossible_I
	//
	// Max_ShuntVoltage = Max_Current_Before_Overflow * RSHUNT
	// Max_ShuntVoltage = 0.131068V
	//
	// If Max_ShuntVoltage >= VSHUNT_MAX
	//    Max_ShuntVoltage_Before_Overflow = VSHUNT_MAX
	// Else
	//    Max_ShuntVoltage_Before_Overflow = Max_ShuntVoltage
	// End If

	// 8. Compute the Maximum Power
	// MaximumPower = Max_Current_Before_Overflow * VBUS_MAX
	// MaximumPower = 1.31068 * 32V
	// MaximumPower = 41.94176W

	// Set multipliers to convert raw current/power values
	ina219_currentDivider_mA = 25;      // Current LSB = 40uA per bit (1000/40 = 25)
	ina219_powerDivider_mW = 1;         // Power LSB = 800W per bit

	// Set Calibration register to 'Cal' calculated above
	ina219_writeRegister(INA219_REG_CALIBRATION, ina219_calValue);

	// Set Config register to take into account the settings above
	uint16_t config = INA219_CONFIG_BVOLTAGERANGE_32V |
					INA219_CONFIG_GAIN_8_320MV |
					INA219_CONFIG_BADCRES_12BIT |
					INA219_CONFIG_SADCRES_12BIT_1S_532US |
					INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;
	ina219_writeRegister(INA219_REG_CONFIG, config);
}



/*******************************************************************************
* Function Name		:	ina219_setCalibration_16V_400mA
* Description		:	Configures to INA219 to be able to measure up to 16V and 400mA
* 						of current.  Each unit of current corresponds to 40uA, and each
* 						unit of power corresponds to 800W. Counter overflow occurs at
* 						1.3A.
* Input				:	None
* Output			:	None
* Return			:	None
* Attention			:	These calculations assume a 0.1 ohm resistor is present
*******************************************************************************/
void ina219_setCalibration_16V_400mA(void){
	// Calibration which uses the highest precision for
	// current measurement (0.1mA), at the expense of
	// only supporting 16V at 400mA max.

	// VBUS_MAX = 16V
	// VSHUNT_MAX = 0.04          (Assumes Gain 1, 40mV)
	// RSHUNT = 0.1               (Resistor value in ohms)

	// 1. Determine max possible current
	// MaxPossible_I = VSHUNT_MAX / RSHUNT
	// MaxPossible_I = 0.4A

	// 2. Determine max expected current
	// MaxExpected_I = 0.4A

	// 3. Calculate possible range of LSBs (Min = 15-bit, Max = 12-bit)
	// MinimumLSB = MaxExpected_I/32767
	// MinimumLSB = 0.0000122              (12uA per bit)
	// MaximumLSB = MaxExpected_I/4096
	// MaximumLSB = 0.0000977              (98uA per bit)

	// 4. Choose an LSB between the min and max values
	//    (Preferrably a roundish number close to MinLSB)
	// CurrentLSB = 0.00005 (50uA per bit)

	// 5. Compute the calibration register
	// Cal = trunc (0.04096 / (Current_LSB * RSHUNT))
	// Cal = 8192 (0x2000)

	ina219_calValue = 8192;

	// 6. Calculate the power LSB
	// PowerLSB = 20 * CurrentLSB
	// PowerLSB = 0.001 (1mW per bit)

	// 7. Compute the maximum current and shunt voltage values before overflow
	//
	// Max_Current = Current_LSB * 32767
	// Max_Current = 1.63835A before overflow
	//
	// If Max_Current > Max_Possible_I then
	//    Max_Current_Before_Overflow = MaxPossible_I
	// Else
	//    Max_Current_Before_Overflow = Max_Current
	// End If
	//
	// Max_Current_Before_Overflow = MaxPossible_I
	// Max_Current_Before_Overflow = 0.4
	//
	// Max_ShuntVoltage = Max_Current_Before_Overflow * RSHUNT
	// Max_ShuntVoltage = 0.04V
	//
	// If Max_ShuntVoltage >= VSHUNT_MAX
	//    Max_ShuntVoltage_Before_Overflow = VSHUNT_MAX
	// Else
	//    Max_ShuntVoltage_Before_Overflow = Max_ShuntVoltage
	// End If
	//
	// Max_ShuntVoltage_Before_Overflow = VSHUNT_MAX
	// Max_ShuntVoltage_Before_Overflow = 0.04V

	// 8. Compute the Maximum Power
	// MaximumPower = Max_Current_Before_Overflow * VBUS_MAX
	// MaximumPower = 0.4 * 16V
	// MaximumPower = 6.4W

	// Set multipliers to convert raw current/power values
	ina219_currentDivider_mA = 20;  // Current LSB = 50uA per bit (1000/50 = 20)
	ina219_powerDivider_mW = 1;     // Power LSB = 1mW per bit

	// Set Calibration register to 'Cal' calculated above
	ina219_writeRegister(INA219_REG_CALIBRATION, ina219_calValue);

	// Set Config register to take into account the settings above
	uint16_t config = INA219_CONFIG_BVOLTAGERANGE_16V |
					INA219_CONFIG_GAIN_1_40MV |
					INA219_CONFIG_BADCRES_12BIT |
					INA219_CONFIG_SADCRES_12BIT_1S_532US |
					INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;
	ina219_writeRegister(INA219_REG_CONFIG, config);
}




/*******************************************************************************
* Function Name		:	ina219_getBusVoltage_raw
* Description		:	Gets the raw bus voltage (16-bit signed integer, so +-32767)
* Input				:	None
* Output			:	None
* Return			:	None
* Attention			:	None
*******************************************************************************/
int16_t ina219_getBusVoltage_raw(void){
	uint16_t value;
	ina219_readRegister(INA219_REG_BUSVOLTAGE, &value);

	// Shift to the right 3 to drop CNVR and OVF and multiply by LSB
	return (int16_t)((value >> 3) * 4);
}



/*******************************************************************************
* Function Name		:	ina219_getBusVoltage_raw
* Description		:	Gets the raw shunt voltage (16-bit signed integer, so +-32767)
* Input				:	None
* Output			:	None
* Return			:	None
* Attention			:	None
*******************************************************************************/
int16_t ina219_getShuntVoltage_raw(void){
	uint16_t value;
	ina219_readRegister(INA219_REG_SHUNTVOLTAGE, &value);
	return (int16_t)value;
}



/*******************************************************************************
* Function Name		:	ina219_getCurrent_raw
* Description		:	Gets the raw shunt voltage (16-bit signed integer, so +-32767)
* Input				:	None
* Output			:	None
* Return			:	None
* Attention			:	None
*******************************************************************************/
int16_t ina219_getCurrent_raw(void){
	uint16_t value;

	// Sometimes a sharp load will reset the INA219, which will
	// reset the cal register, meaning CURRENT and POWER will
	// not be available ... avoid this by always setting a cal
	// value even if it's an unfortunate extra step
	ina219_writeRegister(INA219_REG_CALIBRATION, ina219_calValue);

	// Now we can safely read the CURRENT register!
	ina219_readRegister(INA219_REG_CURRENT, &value);

	return (int16_t)value;
}



/*******************************************************************************
* Function Name		:	ina219_getShuntVoltage_mV
* Description		:	Gets the raw shunt voltage (16-bit signed integer, so +-32767)
* Input				:	None
* Output			:	None
* Return			:	None
* Attention			:	None
*******************************************************************************/
float ina219_getShuntVoltage_mV(void){
	int16_t value;
	value = ina219_getShuntVoltage_raw();
	return value * 0.01;
}



/*******************************************************************************
* Function Name		:	ina219_getBusVoltage_V
* Description		:	Gets the shunt voltage in volts
* Input				:	None
* Output			:	None
* Return			:	None
* Attention			:	None
*******************************************************************************/
float ina219_getBusVoltage_V(void){
	int16_t value = ina219_getBusVoltage_raw();
	return value * 0.001;
}



/*******************************************************************************
* Function Name		:	ina219_getCurrent_mA
* Description		:	Gets the current value in mA, taking into account the
* 						config settings and current LSB
* Input				:	None
* Output			:	None
* Return			:	None
* Attention			:	None
*******************************************************************************/
float ina219_getCurrent_mA(void){
	float valueDec = ina219_getCurrent_raw();
	valueDec /= ina219_currentDivider_mA;
	return valueDec;
}












/*******************************************************************************
* Function Name		:	ina219_powerOnReset
* Description		:	Setting this bit to '1' generates a system reset that is
* 						the same as power-on reset
* Input				:	None
* Output			:	None
* Return			:	None
* Attention			:	Resets all registers to default values
* 						This bit self-clears
*******************************************************************************/
void ina219_powerOnReset(void){
	ina219_writeRegister(INA219_REG_CONFIG, INA219_CONFIG_RESET);
}




/*******************************************************************************
* Function Name		:	ina219_setBusVoltageRange
* Description		:	Sets Bus Voltage Range
* Input				:	INA219BusVoltageRange_TypeDef
* Output			:	None
* Return			:	None
* Attention			:	None
*******************************************************************************/
void ina219_setBusVoltageRange(INA219BusVoltageRange_TypeDef bus_voltage){
	ina219_updateConfigurationRegister(bus_voltage, INA219_Mask_BusVoltageRange);
}



/*******************************************************************************
* Function Name		:	ina219_setGainRange
* Description		:	Sets PGA gain range
* Input				:	INA219GainRange_TypeDef
* Output			:	None
* Return			:	None
* Attention			:	None
*******************************************************************************/
void ina219_setGainRange(INA219GainRange_TypeDef gain){
	ina219_updateConfigurationRegister(gain, INA219_Mask_GainRange);
}




/*******************************************************************************
* Function Name		:	ina219_setBusADCResolution
* Description		:	Sets PGA gain range
* Input				:	INA219BusADCResolution_TypeDef
* Output			:	None
* Return			:	None
* Attention			:	None
*******************************************************************************/
void ina219_setBusADCResolution(INA219BusADCResolution_TypeDef resolution){
	ina219_updateConfigurationRegister(resolution, INA219_Mask_BusADCResolution);
}




/*******************************************************************************
* Function Name		:	ina219_setShuntADCResolution
* Description		:	Sets Shunt ADC resolution, Bit-9, Bit-10, Bit-11, Bit-12, or
* 						set the number of samples used when averaging results for the
* 						Shunt voltage register
* Input				:	INA219ShuntADCResolution_TypeDef
* Output			:	None
* Return			:	None
* Attention			:	None
*******************************************************************************/
void ina219_setShuntADCResolution(INA219ShuntADCResolution_TypeDef resolution){
	ina219_updateConfigurationRegister(resolution, INA219_Mask_ShuntADCResolution);
}




/*******************************************************************************
* Function Name		:	ina219_setOperatingMode
* Description		:	Selects continuous, triggered, or power-down mode of operation.
* Input				:	INA219Mode_TypeDef
* Output			:	None
* Return			:	None
* Attention			:	These bits default to continuous shunt and bus measurement mode.
*******************************************************************************/
void ina219_setOperatingMode(INA219Mode_TypeDef mode){
	ina219_updateConfigurationRegister(mode, INA219_Mask_Mode);
}



/*******************************************************************************
* Function Name		:	ina219_updateConfigurationRegister
* Description		:	Updatate configuration register content
* Input				:	INA219Mask_Typedef
* Output			:	None
* Return			:	None
* Attention			:	None
*******************************************************************************/
void ina219_updateConfigurationRegister(uint16_t reg_bits, INA219Mask_Typedef mask){
	uint16_t reg_value;

	/* Read the current configuration register value */
	ina219_readRegister(INA219_REG_CONFIG, &reg_value);

	/* First clear masking bits,bit-13 */
	reg_value &= ~(mask);	// Clear bit-13
	/* Then restore the current bits */
	reg_value = reg_value | reg_bits;

	/* Write the 16-bit current value to configuration register */
	ina219_writeRegister(INA219_REG_CONFIG, reg_value);
}
