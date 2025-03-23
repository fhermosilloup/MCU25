/*
 * lcd.c
 *
 *  Created on: Feb 20, 2025
 *      Author: User123
 */


/* Private includes -----------------------------------------------*/
#include "lcd.h"

/* Private defines ------------------------------------------------*/

/* Private macros -------------------------------------------------*/

/* Private typedefs ------------------------------------------------*/

/* Private variables -----------------------------------------------*/
static uint8_t ucLcdIsInit = 0x00;

/* Private prototype function --------------------------------------*/
static uint8_t LCD_Wait_Busy(void);

/* Exported reference function -------------------------------------*/
void LCD_Init(lcd_interface_t eInterface)
{
	/* Configure GPIOs */
	LCD_Port->CRL = 0x22222222;
	LCD_Port->CRH = 0x00000222;
	LCD_Port->ODR &= ~LCD_DB_Mask;
	LCD_Port->ODR &= ~LCD_CTRL_Mask;

	/** Perform lcd init sequence: see pag. 45-46 on
	 * @link https://cdn.sparkfun.com/assets/9/5/f/7/b/HD44780.pdf @endlink
	 */
	// Power-on delay
	delay_ms(15);

	if(eInterface == LCD_8BITS_INTERFACE)
	{
		// Set 8-bit interface
		LCD_Write(LCD_CMD_FUNCTION_SET | LCD_8B_INTERFACE, LCD_COMMAND);

		// Delay
		delay_ms(5);

		// Resend
		LCD_Write(LCD_CMD_FUNCTION_SET | LCD_8B_INTERFACE, LCD_COMMAND);

		// 100 us delay
		delay_us(100);

		// Resend
		LCD_Write(LCD_CMD_FUNCTION_SET | LCD_8B_INTERFACE, LCD_COMMAND);

		// 100 us delay
		delay_us(100);

		// The LCD.BUSY_FLAG can now be used for polling the LCD BUSY state
		ucLcdIsInit = 1;

		// Configure LCD settings
		LCD_Write(LCD_CMD_FUNCTION_SET | LCD_8B_INTERFACE | LCD_TWO_LINES | LCD_5X8_FONT, LCD_COMMAND);

		// Display: ON
		// Cursor: ON
		// Cursor.Blink: OFF
		LCD_Write(LCD_CMD_DISPLAY_CONTROL | LCD_DISPLAY_ON, LCD_COMMAND);

		// Display clear
		LCD_Write(LCD_CMD_CLEAR, LCD_COMMAND);

		// Entry mode
		// Address: Increment
		// Display: No Shift
		LCD_Write(LCD_CMD_ENTRY_MODE | LCD_CURSOR_INC, LCD_COMMAND);
	}
	else if(eInterface == LCD_4BITS_INTERFACE)
	{
		// 4-bit interface: see pag. 22, and 46
	}
}

// Write sequence
void LCD_Write(uint8_t data, uint8_t isCmd)
{
	if(ucLcdIsInit) LCD_Wait_Busy();

	// Write mode (RW = 0)
	LCD_Port->BRR = LCD_RW_Pin;

	// Write data/instruction (RS = 1/0)
	if( isCmd ) LCD_Port->BRR = LCD_RS_Pin;
	else LCD_Port->BSRR = LCD_RS_Pin;

	// Configure DB pins as outputs
	LCD_Port->CRL = 0x22222222;

	// Write data into DB pins
	LCD_Port->BRR = LCD_DB_Mask;
	LCD_Port->BSRR |= (uint32_t)data;

	// Send EN pulse: see Bus Timing Characteristics on pag. 49
	LCD_Port->BSRR = LCD_E_Pin; // E = 1

	// 1us delay assuming an eight MHz clock needs 8 clock cycles
	delay_us(1);

	LCD_Port->BRR = LCD_E_Pin;	// E=0

	// Set DB pins to high
	LCD_Port->BSRR = LCD_DB_Mask;
}

// Read sequence
uint8_t LCD_Read(uint8_t isData)
{
	uint8_t dout = 0x00;

	// Read mode (RW = 1)
	LCD_Port->BSRR = LCD_RW_Pin;

	// Read data/busy&DDRAM (RS = 1/0)
	if(isData) LCD_Port->BSRR = LCD_RS_Pin;
	else LCD_Port->BRR = LCD_RS_Pin;

	// Configure DB pins as floating inputs
	LCD_Port->CRL = 0x44444444;

	// Execution cycle: see Bus Timing Characteristics on pag. 49
	LCD_Port->BSRR = LCD_E_Pin; // E = 1

	// 1us delay assuming an eight MHz clock needs 8 clock cycles
	delay_us(1);

	// Read data from DB pins
	dout = (uint8_t)(LCD_Port->IDR & 0xFF);

	// E = 0
	LCD_Port->BRR = LCD_E_Pin;

	// 1us delay assuming an eight MHz clock needs 8 clock cycles
	delay_us(1);

	return dout;
}

/* Private reference function --------------------------------------*/
static uint8_t LCD_Wait_Busy(void)
{
	// Wait until busy flag is cleared
	while (LCD_Read(0) & LCD_BUSY);

	// Delay needed for address counter is updated after busy flag is cleared
	delay_us(10);

	// Read and return address counter
	return LCD_Read(0);
}

