/*
 * lcd.c
 *
 *  Created on: Feb 20, 2025
 *      Author: User123
 */


/* Private includes -----------------------------------------------*/
#include "stm32f103xb.h"

/* Private defines ------------------------------------------------*/

/* Private macros -------------------------------------------------*/

/* Private typedefs ------------------------------------------------*/

/* Private variables -----------------------------------------------*/

/* Private prototype function --------------------------------------*/

/* Exported reference function -------------------------------------*/
void LCD_Init(uint8_t dbWidth)
{
	// Configure GPIOs ============

	// Perform init sequence: see https://cdn.sparkfun.com/assets/9/5/f/7/b/HD44780.pdf
	// See pag.
	if(dbWidth == LCD_8B_INTERFACE)
	{
		// 8-bit interface: see pag. 22, and 45
	}
	else if(dbWidth == LCD_4B_INTERFACE)
	{
		// 4-bit interface: see pag. 22, and 46
	}
}

// Write sequence
void LCD_Write(uint8_t data, uint8_t isCmd)
{
	// Write mode (RW = 0)

	// Write data/instruction (RS = 1/0)

	// Configure DB pins as outputs

	// Write data into DB pins

	// Send EN pulse: see Bus Timing Characteristics on pag. 49

	// Set DB pins to high
}

// Read sequence
uint8_t LCD_Read(uint8_t isData)
{
	uint8_t dout = 0x00;

	// Read mode (RW = 1)

	// Read data/busy&DDRAM (RW = 1/0)

	// Configure DB pins as floating inputs

	// Pull EN to HIGH: see Bus Timing Characteristics on pag. 49

	// Read data from DB pins

	// Pull EN to LOW: see Bus Timing Characteristics on pag. 49

	// Set DB pins to high

	return dout;
}

/* Private reference function --------------------------------------*/
