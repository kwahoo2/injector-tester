/*
 * lcd.c
 *
 *  Created on: 27 gru 2017
 *      Author: adi
 */

#include "lcd.h"
#include "stm32f1xx_hal.h"

void lcdOutNibble(unsigned char nibbleToWrite)
{
	if (nibbleToWrite & 0x01)HAL_GPIO_WritePin(LCD_PORT, LCD_D4, GPIO_PIN_SET);
	else HAL_GPIO_WritePin(LCD_PORT, LCD_D4, GPIO_PIN_RESET);
	if (nibbleToWrite & 0x02)HAL_GPIO_WritePin(LCD_PORT, LCD_D5, GPIO_PIN_SET);
	else HAL_GPIO_WritePin(LCD_PORT, LCD_D5, GPIO_PIN_RESET);
	if (nibbleToWrite & 0x04)HAL_GPIO_WritePin(LCD_PORT, LCD_D6, GPIO_PIN_SET);
	else HAL_GPIO_WritePin(LCD_PORT, LCD_D6, GPIO_PIN_RESET);
	if (nibbleToWrite & 0x08)HAL_GPIO_WritePin(LCD_PORT, LCD_D7, GPIO_PIN_SET);
	else HAL_GPIO_WritePin(LCD_PORT, LCD_D7, GPIO_PIN_RESET);
}

void lcdWrite(unsigned char dataToWrite)
{
	HAL_GPIO_WritePin(LCD_PORT, LCD_E, GPIO_PIN_SET);
	lcdOutNibble(dataToWrite >> 4);
	HAL_Delay(1);
	HAL_GPIO_WritePin(LCD_PORT, LCD_E, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(LCD_PORT, LCD_E, GPIO_PIN_SET);
	lcdOutNibble(dataToWrite);
	HAL_GPIO_WritePin(LCD_PORT, LCD_E, GPIO_PIN_RESET);
	HAL_Delay(1);
}
void lcdWriteCommand(unsigned char commandToWrite)
{
	HAL_GPIO_WritePin(LCD_PORT, LCD_RS, GPIO_PIN_RESET);
	lcdWrite(commandToWrite);
}
void lcdWriteData(unsigned char dataToWrite)
{
	HAL_GPIO_WritePin(LCD_PORT, LCD_RS, GPIO_PIN_SET);
	lcdWrite(dataToWrite);
}
void lcdGoTo(unsigned char x, unsigned char y)
{
	lcdWriteCommand(HD44780_DDRAM_SET | (x + (0x40 * y)));
}
void lcdWriteText(char * text)
{
	while(*text)
		lcdWriteData(*text++);
}
void lcdInit(void)
{
	HAL_Delay(15);
	HAL_GPIO_WritePin(LCD_PORT, LCD_RS, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LCD_PORT, LCD_E, GPIO_PIN_RESET);
	unsigned char i;
	for(i = 0; i < 3; i++)
	{
		HAL_GPIO_WritePin(LCD_PORT, LCD_E, GPIO_PIN_SET); //  E = 1
		lcdOutNibble(0x03); // 8-bit mode
		HAL_GPIO_WritePin(LCD_PORT, LCD_E, GPIO_PIN_RESET); // E = 0
		HAL_Delay(5); // wait 5ms
	}
	HAL_GPIO_WritePin(LCD_PORT, LCD_E, GPIO_PIN_SET);
	lcdOutNibble(0x2);
	HAL_GPIO_WritePin(LCD_PORT, LCD_E, GPIO_PIN_RESET);
	HAL_Delay(1);
	lcdWriteCommand(HD44780_FUNCTION_SET | HD44780_FONT5x7 | HD44780_TWO_LINE | HD44780_4_BIT); // interface 4-bits, 2-lines, 5x7 symbol
	lcdWriteCommand(HD44780_DISPLAY_ONOFF | HD44780_DISPLAY_OFF);
	lcdWriteCommand(HD44780_CLEAR); // clear DDRAM
	HAL_Delay(1);
	lcdWriteCommand(HD44780_ENTRY_MODE | HD44780_EM_SHIFT_CURSOR | HD44780_EM_INCREMENT);// addr incrementation and cursor translation
	lcdWriteCommand(HD44780_DISPLAY_ONOFF | HD44780_DISPLAY_ON | HD44780_CURSOR_OFF | HD44780_CURSOR_NOBLINK);
}

