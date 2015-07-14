/*********************************************************************************************************
* File                	: 	main.c
* Datum					: 	2015.03.06
* Version				: 	1.0
* Author				: 	Aytac Dilek
* e-mail				: 	aytacdilek@gmail.com
* Web					:
* Platform				: 	Open103Z-B
* CPU					: 	STM32F103ZET6
* IDE					: 	CooCox CoIDE 1.7.7
* Compiler				: 	GCC 4.8 2014q2
* Module				: 	INA219 and Nokia5110 lcd with PCD8544 controller
* Function				: 	Reads power ratings information via INA219 sensor
* 							and displays it on Nokia5110 LCD
* Copyright				:	GNU General Public License, version 3 (GPL-3.0)
*********************************************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

#include "open103z_systick.h"

#include "stm32f10x_ina219.h"
#include "stm32f10x_pcd8544.h"
#include "stm32f10x_util.h"

float voltage, current;
char *result;


int main(void)
{
	SystemInit(); // Activate settings quartz

	ina219_init();
	nokia5110_init();



    while(1)
    {
    	nokia5110_clear();
    	nokia5110_gotoXY(0, 0);
    	nokia5110_writeString("Voltage (V):");
    	nokia5110_gotoXY(0, 1);
    	voltage = ina219_getBusVoltage_V();
    	result = util_floatToString(voltage);
    	nokia5110_writeString(result);
//    	systick_delayMs(100);

    	nokia5110_gotoXY(0, 2);
    	nokia5110_writeString("Current (mA):");
    	nokia5110_gotoXY(0, 3);
    	current = ina219_getCurrent_mA();
    	result = util_floatToString(current);
    	nokia5110_writeString(result);
//    	systick_delayMs(100);

    	nokia5110_gotoXY(0, 4);
    	nokia5110_writeString("Power (mW):");
    	nokia5110_gotoXY(0, 5);
    	result = util_floatToString(voltage * current);
    	nokia5110_writeString(result);
    	systick_delayMs(300);
    }
}
