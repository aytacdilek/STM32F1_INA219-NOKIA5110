/*******************************************************************************
* File  		:	stm32f10x_pcd8544.h
* Description	: 	STM32F10x library for NOKIA 5110 LCD driver, PCD8544
* Author		: 	Aytac Dilek
* Note			: 	GNU General Public License, version 3 (GPL-3.0)
*******************************************************************************/
#ifndef __STM32F10X_PCD8544_H
#define __STM32F10X_PCD8544_H



/* Includes ********************************************************************/
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_spi.h"
#include "open103z_systick.h"



/* Defines *********************************************************************/
#define PCD8544_SPI_PER			SPI2
#define PCD8544_SPI_PORT		GPIOB
#define PCD8544_SPI_CLOCK		RCC_APB1Periph_SPI2
#define PCD8544_GPIO_PORT		GPIOB
#define PCD8544_GPIO_CLOCK		RCC_APB2Periph_GPIOB

#define PCD8544_LED_PORT 		GPIOB
#define PCD8544_LED_PIN 		GPIO_Pin_9
#define PCD8544_RST_PORT 		GPIOB
#define PCD8544_RST_PIN 		GPIO_Pin_10
#define PCD8544_CE_PORT			GPIOB
#define PCD8544_CE_PIN 			GPIO_Pin_11
#define PCD8544_DC_PORT 		GPIOB
#define PCD8544_DC_PIN 			GPIO_Pin_12
#define PCD8544_CLK_PORT		GPIOB
#define PCD8544_CLK_PIN			GPIO_Pin_13
#define PCD8544_MISO_PORT		GPIOB
#define PCD8544_MISO_PIN		GPIO_Pin_14
#define PCD8544_MOSI_PORT		GPIOB
#define PCD8544_MOSI_PIN		GPIO_Pin_15

#define PCD8544_WIDTH			84
#define PCD8544_HEIGHT			48



/* Macros **********************************************************************/
#define NOKIA5110_LED_ON()		GPIO_SetBits(PCD8544_LED_PORT, PCD8544_LED_PIN);
#define NOKIA5110_LED_OFF()		GPIO_ResetBits(PCD8544_LED_PORT, PCD8544_LED_PIN);
#define NOKIA5110_RST_LOW()		GPIO_ResetBits(PCD8544_RST_PORT, PCD8544_RST_PIN)
#define NOKIA5110_RST_HIGH()	GPIO_SetBits(PCD8544_RST_PORT, PCD8544_RST_PIN)
#define NOKIA5110_DC_LOW()		GPIO_ResetBits(PCD8544_DC_PORT, PCD8544_DC_PIN)
#define NOKIA5110_DC_HIGH()		GPIO_SetBits(PCD8544_DC_PORT, PCD8544_DC_PIN)
#define NOKIA5110_DIN_LOW()		GPIO_ResetBits(PCD8544_MOSI_PORT, PCD8544_MOSI_PIN)
#define NOKIA5110_DIN_HIGH()	GPIO_SetBits(PCD8544_MOSI_PORT, PCD8544_MOSI_PIN)
#define NOKIA5110_CLK_LOW()		GPIO_ResetBits(PCD8544_CLK_PORT, PCD8544_CLK_PIN)
#define NOKIA5110_CLK_HIGH()	GPIO_SetBits(PCD8544_CLK_PORT, PCD8544_CLK_PIN)
#define NOKIA5110_CE_LOW()		GPIO_ResetBits(PCD8544_CE_PORT, PCD8544_CE_PIN)
#define NOKIA5110_CE_HIGH()		GPIO_SetBits(PCD8544_CE_PORT, PCD8544_CE_PIN)



/* Enumarations ****************************************************************/
typedef enum{
	PCD8544_MODE_Command = 0,
	PCD8544_MODE_Data = 1,
}PCD8544_MODE_TypeDef;



/* Global Functions ************************************************************/
void nokia5110_init(void);
void nokia5110_writeChar(uint8_t c);
void nokia5110_writeCharInv(uint8_t c);
void nokia5110_writeString(uint8_t *s);
void nokia5110_clear(void);
void nokia5110_gotoXY(uint8_t x, uint8_t y);
void nokia5110_setContrast(uint8_t contrast);



//--------------------------------------------------------------
#endif // __STM32F10X_PCD8544_H
