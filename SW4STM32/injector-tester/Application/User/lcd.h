/*
 * lcd.h
 *
 *  Created on: 27 gru 2017
 *      Author: adi
 */

#ifndef APPLICATION_USER_LCD_H_
#define APPLICATION_USER_LCD_H_
/* Private variables ---------------------------------------------------------*/

#define LCD_RS GPIO_PIN_1
#define LCD_E GPIO_PIN_7

#define LCD_D4 GPIO_PIN_12
#define LCD_D5 GPIO_PIN_13
#define LCD_D6 GPIO_PIN_14
#define LCD_D7 GPIO_PIN_15

#define LCD_PORT GPIOB

//-------------------------------------------------------------------------------------------------
//
// Hitachi HD44780 - based on AVR implementation http://radzio.dxp.pl/hd44780/
//
//-------------------------------------------------------------------------------------------------

#define HD44780_CLEAR					0x01

#define HD44780_HOME					0x02

#define HD44780_ENTRY_MODE				0x04
	#define HD44780_EM_SHIFT_CURSOR		0
	#define HD44780_EM_SHIFT_DISPLAY	1
	#define HD44780_EM_DECREMENT		0
	#define HD44780_EM_INCREMENT		2

#define HD44780_DISPLAY_ONOFF			0x08
	#define HD44780_DISPLAY_OFF			0
	#define HD44780_DISPLAY_ON			4
	#define HD44780_CURSOR_OFF			0
	#define HD44780_CURSOR_ON			2
	#define HD44780_CURSOR_NOBLINK		0
	#define HD44780_CURSOR_BLINK		1

#define HD44780_DISPLAY_CURSOR_SHIFT	0x10
	#define HD44780_SHIFT_CURSOR		0
	#define HD44780_SHIFT_DISPLAY		8
	#define HD44780_SHIFT_LEFT			0
	#define HD44780_SHIFT_RIGHT			4

#define HD44780_FUNCTION_SET			0x20
	#define HD44780_FONT5x7				0
	#define HD44780_FONT5x10			4
	#define HD44780_ONE_LINE			0
	#define HD44780_TWO_LINE			8
	#define HD44780_4_BIT				0
	#define HD44780_8_BIT				16

#define HD44780_CGRAM_SET				0x40

#define HD44780_DDRAM_SET				0x80

void lcdOutNibble(unsigned char nibbleToWrite);
void lcdWrite(unsigned char dataToWrite);
void lcdWriteCommand(unsigned char commandToWrite);
void lcdWriteData(unsigned char dataToWrite);
void lcdGoTo(unsigned char x, unsigned char y);
void lcdWriteText(char * text);
void lcdInit(void);


#endif /* APPLICATION_USER_LCD_H_ */
