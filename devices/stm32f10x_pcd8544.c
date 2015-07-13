/*******************************************************************************
 * File				:	stm32f10x_pcd8544.c
 * Description		:	STM32F10x library for NOKIA 5110 LCD driver, PCD8544
 * Datum			:	2015.05.22
 * Version			:	1.0
 * Author			:	Aytac Dilek
 * email			:	aytacdilek@gmail.com
 * Web				:
 * Platform			:	OPEN103Z-B
 * CPU				:	STM32F103ZET6
 * IDE				:	CooCox CoIDE 1.7.7
 * GCC				:	4.8 2014q2
 * Module			:	PCD8544 LCD driver for NOKIA 5110
 * Function			:	LCD Driver
 * Pin Definitions	:	PB9  = LED (BL Pin)
 * 						PB10 = Reset (RST Pin, NSS)
 * 						PB11 = ChipSelect (CE Pin)
 * 						PB12 = Mode (DC Pin)
 * 						PB13 = Clock (Clk Pin)
 * 						PB15 = Data (Din Pin )
*******************************************************************************/



/* Includes *******************************************************************/
#include "stm32f10x_pcd8544.h"
#include "font_6x8.h"



/* Internal Functions *********************************************************/
/** Hardware Control and Configuration Functions */
void nokia5110_gpio_config(void);
void nokia5110_spi_config(void);
void nokia5110_spi_writeByte(PCD8544_MODE_TypeDef mode, uint8_t data);



/*******************************************************************************
* Function Name	: 	nokia5110_init
* Description	: 	Initialize LCD
* Input			: 	None
* Output		: 	None
* Return		: 	None
* Attention		: 	None
*******************************************************************************/
void nokia5110_init(void){
	/* Configure gpio pins */
	nokia5110_gpio_config();

	/* Configure spi pins */
	nokia5110_spi_config();

	/* Set pin initial state */
	NOKIA5110_LED_ON(); 		// Turn back light off
	NOKIA5110_DC_HIGH(); 		// Mode = command;
	NOKIA5110_DIN_HIGH(); 		// Set In at high level;
	NOKIA5110_CLK_HIGH(); 		// Set CLK high;
	NOKIA5110_CE_HIGH(); 		// Unselect chip;

	/* Reset the LCD to a known state */
	NOKIA5110_RST_LOW();		// Set LCD reset = 0;
	systick_delayMs(10);		// Keep reset pin low for 10 ms
	NOKIA5110_RST_HIGH();		// LCD_RST = 1;

	/* Configure LCD module */
	nokia5110_spi_writeByte(PCD8544_MODE_Command, 0x21);		// Extended instruction set selected
	nokia5110_spi_writeByte(PCD8544_MODE_Command, 0xc0); 		// Set LCD voltage (defined by experimentation...)
	nokia5110_spi_writeByte(PCD8544_MODE_Command, 0x14);		// Set Bias for 1/48
	nokia5110_spi_writeByte(PCD8544_MODE_Command, 0x04);		// Set temperature control (TC2)
	nokia5110_spi_writeByte(PCD8544_MODE_Command, 0x20);		// Revert to standard instruction set
	nokia5110_clear(); 									// Clear display (still off)
	nokia5110_spi_writeByte(PCD8544_MODE_Command, 0x0c);		// Set display on in "normal" mode (not inversed)
}



/*******************************************************************************
* Function Name	: 	nokia5110_clear
* Description	: 	Clear display. Write 0 in all memory location.
* Input			: 	None
* Output		: 	None
* Return		: 	None
* Attention		: 	None
*******************************************************************************/
void nokia5110_clear(void){
	unsigned char i, j;
	for (i = 0; i < 6; i++) {
		for (j = 0; j < 84; j++) {
			nokia5110_spi_writeByte(PCD8544_MODE_Data, 0x00);
		}
	}
}



/*******************************************************************************
* Function Name	: 	nokia5110_setPosition
* Description	: 	Set memory current location for characters (set coordinates).
* Input			: 	X => Column (range from 0 to 13)
* 					Y => Row (range from 0 to 5)
* Output		: 	None
* Return		: 	None
* Attention		: 	Applies only for Fonts with a 6 pixels width.
*******************************************************************************/
void nokia5110_gotoXY(uint8_t x, uint8_t y){
	x = 6 * x;

	nokia5110_spi_writeByte(PCD8544_MODE_Command, 0x40 | y);
	nokia5110_spi_writeByte(PCD8544_MODE_Command, 0x80 | x);
}



/*******************************************************************************
* Function Name	: 	nokia5110_writeChar
* Description	: 	Write character to LCD at current position
* Input			: 	c => char to write
* Output		: 	None
* Return		: 	None
* Attention		: 	None
*******************************************************************************/
void nokia5110_writeChar(uint8_t c){
	unsigned char line;
	unsigned char ch = 0;

	c = c - 32;

	for (line = 0; line < 6; line++) {
		ch = font6_8[c][line];
		nokia5110_spi_writeByte(PCD8544_MODE_Data, ch);
	}
}



/*******************************************************************************
* Function Name	: 	nokia5110_writeCharInv
* Description	: 	Write character to LCD in inverse video at current location
* Input			: 	c => char to write
* Output		: 	None
* Return		: 	None
* Attention		: 	None
*******************************************************************************/
void nokia5110_writeCharInv(uint8_t c) {
	unsigned char line;
	unsigned char ch = 0;

	c = c - 32;

	for (line = 0; line < 6; line++) {
		ch = ~font6_8[c][line];
		nokia5110_spi_writeByte(PCD8544_MODE_Data, ch);
	}
}



/*******************************************************************************
* Function Name	: 	nokia5110_writeString
* Description	: 	Write string to LCD at current position.
* Input			: 	s => string pointer
* Output		: 	None
* Return		: 	None
* Attention		: 	String must be null terminated.
*******************************************************************************/
void nokia5110_writeString(uint8_t *s){
	unsigned char ch;

	while (*s != '\0') {
		ch = *s;
		nokia5110_writeChar(ch);
		s++;
	}
}



/*******************************************************************************
* Function Name	: 	nokia5110_setContrast
* Description	: 	Set LCD Contrast
* Input			: 	contrast =>
* Output		: 	None
* Return		: 	None
* Attention		: 	None
*******************************************************************************/
void nokia5110_setContrast(uint8_t contrast){
	/*  LCD Extended Commands. */
	nokia5110_spi_writeByte(PCD8544_MODE_Command, 0x21);

	/* Set LCD Vop (Contrast). */
	nokia5110_spi_writeByte(PCD8544_MODE_Command, 0x80 | contrast);

	/*  LCD Standard Commands, horizontal addressing mode. */
	nokia5110_spi_writeByte(PCD8544_MODE_Command, 0x20);
}



/* Internal Functions *********************************************************/



/*******************************************************************************
* Function Name	: 	nokia5110_gpio_config
* Description	: 	Configure GPIO pins (Hardware SPI)
* Input			: 	None
* Output		: 	None
* Return		: 	None
* Attention		: 	PB9  : LED control
* 					PB10 : RESET
* 					PB11 : ChipSelect
* 					PB12 : DC (Mode)
* 					PB13 : SPI-2_CLK
* 					PB14 : SPI-2_MISO
* 					PB15 : SPI-2_MOSI
*******************************************************************************/
void nokia5110_gpio_config(void){
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Enable Clocks for GPIO */
	RCC_APB2PeriphClockCmd(PCD8544_GPIO_CLOCK, ENABLE);

	/* Configure LED Pin */
	GPIO_InitStructure.GPIO_Pin = PCD8544_LED_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(PCD8544_LED_PORT, &GPIO_InitStructure);

	/* Configure RST Pin */
	GPIO_InitStructure.GPIO_Pin = PCD8544_RST_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(PCD8544_RST_PORT, &GPIO_InitStructure);

	/* Configure CE Pin */
	GPIO_InitStructure.GPIO_Pin = PCD8544_CE_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(PCD8544_CE_PORT, &GPIO_InitStructure);

	/* Configure DC Pin */
	GPIO_InitStructure.GPIO_Pin = PCD8544_DC_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(PCD8544_DC_PORT, &GPIO_InitStructure);
}


/*******************************************************************************
* Function Name	: 	nokia5110_spi_config
* Description	: 	Configure SPI pins (Hardware SPI)
* Input			: 	None
* Output		: 	None
* Return		: 	None
* Attention		: 	None
*******************************************************************************/
void nokia5110_spi_config(void){
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef SPI_InitStructure;

	/* Enable Clocks for GPIO and SPI2 */
	RCC_APB2PeriphClockCmd(PCD8544_GPIO_CLOCK, ENABLE);
	RCC_APB1PeriphClockCmd(PCD8544_SPI_CLOCK, ENABLE);
	// TODO: Modify the structure for SPI1

	/* Configure GPIO Pins for SPI peripheral*/
	GPIO_InitStructure.GPIO_Pin = PCD8544_CLK_PIN | PCD8544_MISO_PIN | PCD8544_MOSI_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(PCD8544_SPI_PORT, &GPIO_InitStructure);

	/* Configure SPI Pins */
	SPI_InitStructure.SPI_Direction = SPI_Direction_1Line_Tx;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(PCD8544_SPI_PER, &SPI_InitStructure);

	/* Enable SPI Peripheral */
	SPI_Cmd(PCD8544_SPI_PER, ENABLE);
}



/*******************************************************************************
* Function Name	: 	nokia5110_spi_writeByte
* Description	: 	Write byte to the module.
* Input			: 	mode => 0 if command, 1 if data
* 					data => data to write
* Output		: 	None
* Return		: 	None
* Attention		: 	None
*******************************************************************************/
void nokia5110_spi_writeByte(PCD8544_MODE_TypeDef mode, uint8_t data){
	NOKIA5110_CE_LOW();			// SPI_CS = 0;

	if(mode == PCD8544_MODE_Command){
		NOKIA5110_DC_LOW();
	}
	else{
		NOKIA5110_DC_HIGH();
	}

	/* Loop while DR register in not empty */
	while (SPI_I2S_GetFlagStatus(PCD8544_SPI_PER, SPI_I2S_FLAG_TXE) == RESET);
	/* Send a Byte through the SPI peripheral */
	SPI_I2S_SendData(PCD8544_SPI_PER, data);
	/* Be sure that the character goes to the shift register */
	while (SPI_I2S_GetFlagStatus(PCD8544_SPI_PER, SPI_I2S_FLAG_TXE) == RESET);
	/* Wait until entire byte has been read (which we discard anyway) */
//	while( SPI_I2S_GetFlagStatus(PCD8544_SPI_PER, SPI_I2S_FLAG_RXNE) != SET );

	NOKIA5110_CE_HIGH();	// SPI_CS = 1;
}
