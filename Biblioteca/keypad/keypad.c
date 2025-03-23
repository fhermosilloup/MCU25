/*
 * keypad.c
 *
 *  Created on: Feb 28, 2025
 *      Author: User123
 */

#include "keypad.h"
#include "systick.h"

typedef enum {
	KEYPAD_IDLE_STATE=0,
    KEYPAD_SCAN_STATE,
	KEYPAD_DEBOUNCE_STATE,
    KEYPAD_RELEASE_STATE
} Keypad_State_t;

typedef struct
{
	uint8_t row_pins[4];
	uint8_t col_pins[4];
} Keypad_gpio_t;

// Key mapping for a 4x4 keypad
static const char KEYPAD_MAP[4][4] = {
    {'1', '2', '3', 'A'},
    {'4', '5', '6', 'B'},
    {'7', '8', '9', 'C'},
    {'*', '0', '#', 'D'}
};

static Keypad_gpio_t keypad_gpio = {0};
static keypad_event_handler_t onKeyPressedHandler = NULL;
static keypad_event_handler_t onKeyReleasedHandler = NULL;
static Keypad_State_t eKeypadState = KEYPAD_IDLE_STATE;

int Keypad_Init(keypad_event_handler_t onKeyPressedCallback, keypad_event_handler_t onKeyReleasedCallback)
{
	// Check argin
	if(onKeyPressedCallback) onKeyPressedHandler = onKeyPressedCallback;
	if(onKeyReleasedCallback) onKeyReleasedHandler = onKeyReleasedCallback;

	// Init keypad_gpio structure
	for(int i = 0; i < 4; i++)
	{
		keypad_gpio.col_pins[i] = 0;
		keypad_gpio.row_pins[i] = 0;
	}

	// Configure KEYPAD GPIOs
	uint8_t row_pin_count = 0;
	uint8_t col_pin_count = 0;
	for(uint8_t i = 0; i < 16; i++)
	{
		// Configure KEYPAD_ROW pins
		if(KEYPAD_ROW_Pin & BIT(i))
		{
			if(row_pin_count < 4)
			{
				keypad_gpio.row_pins[row_pin_count++] = i;
			}
			else
			{
				return -1;
			}
		}
		else if(KEYPAD_COL_Pin & BIT(i))
		{
			if(col_pin_count < 4)
			{
				keypad_gpio.col_pins[col_pin_count++] = i;
			}
			else
			{
				return -1;
			}
		}
	}

	// Setup Keypad I/Os
	for(uint8_t i = 0; i < 4; i++)
	{
		// Keypad rows as GPIO output
		uint8_t num_pin = keypad_gpio.row_pins[i];
		if(num_pin < 8)
		{
			// Configure GPIOx_CRL register
			KEYPAD_ROW_Port->CRL &= ~(0x0F << num_pin*4);
			KEYPAD_ROW_Port->CRL |= (0x02 << num_pin*4);
		}
		else
		{
			// Configure GPIOx_CRH register
			KEYPAD_ROW_Port->CRH &= ~(0x0F << (num_pin-8)*4);
			KEYPAD_ROW_Port->CRH |= (0x02 << (num_pin-8)*4);
		}

		// Keypad Columns as GPIO floating input or pull-down
		num_pin = keypad_gpio.col_pins[i];
		if(num_pin < 8)
		{
			// Configure GPIOx_CRL register
			KEYPAD_COL_Port->CRL &= ~(0x0F << num_pin*4);
			if(KEYPAD_COL_Port == GPIOB)
			{
				// Pull-down
				KEYPAD_COL_Port->CRL |= (0x08 << num_pin*4);
				KEYPAD_COL_Port->ODR &= ~ (1 << num_pin);
			}
			else KEYPAD_COL_Port->CRL |= (0x04 << num_pin*4);
		}
		else
		{
			// Configure GPIOx_CRH register
			KEYPAD_COL_Port->CRH &= ~(0x0F << (num_pin-8)*4);
			if(KEYPAD_COL_Port == GPIOB)
			{
				// Pull-down
				KEYPAD_COL_Port->CRL |= (0x08 << (num_pin-8)*4);
				KEYPAD_COL_Port->ODR &= ~ (1 << num_pin);
			}
			else KEYPAD_COL_Port->CRL |= (0x04 << (num_pin-8)*4);
			KEYPAD_COL_Port->CRH |= (0x04 << (num_pin-8)*4);
		}
	}

	// Set KEYPAD_ROW in LOW state
	KEYPAD_ROW_Port->BRR = KEYPAD_ROW_Pin;

	// Set FSM state to Scan
	eKeypadState = KEYPAD_SCAN_STATE;

	// Return OK
	return 0;
}

void Keypad_Update(KeyEvent_t *event)
{
	static uint32_t uwLastTimeWakeup = 0;

	switch (eKeypadState) {
		case KEYPAD_IDLE_STATE:
			// Do nothing
			event->key = '\0';
			event->state = KEY_IDLE;
			break;

		case KEYPAD_SCAN_STATE:
			key = Keypad_GetKey();  // Check for key press
			event->key = key;
			event->state = KEY_IDLE;
			if (key != '\0')
			{
				// Update
				event->state = KEY_PRESSED;
				uwLastTimeWakeup = HAL_GetTick();
				eKeypadState = KEYPAD_DEBOUNCE_STATE;

				// Call the callback
				if(onKeyPressedHandler) onKeyPressedHandler(key);
			}
			break;

		case KEYPAD_DEBOUNCE_STATE:
			// Check if key is still pressed
			if(HAL_GetTick() - uwLastTimeWakeup >= KEYPAD_FSM_DEBOUNCE_DELAY)
			{
				eKeypadState = KEYPAD_RELEASE_STATE;
			}
			break;

		case KEYPAD_RELEASE_STATE:
			key = Keypad_GetKey();
			if (key == '\0')
			{
				if(onKeyReleasedHandler) onKeyReleasedHandler(event->key);
				eKeypadState = KEYPAD_SCAN_STATE;
				event->state = KEY_RELEASED;
			}
			break;
	}
}


char Keypad_GetKey(void)
{
	// Scan through each row
	for (int row = 0; row < 4; row++)
	{
		// Set the current row HIGH
		KEYPAD_ROW_Port->BSRR = BIT(keypad_gpio.row_pins[row]);
		delay_us(5);

		// Read all column inputs
		for (int col = 0; col < 4; col++)
		{
			if(KEYPAD_COL_Port->IDR & BIT(keypad_gpio.col_pins[col]))
			{
				KEYPAD_ROW_Port->BRR = BIT(keypad_gpio.row_pins[row]);
				return KEYPAD_MAP[row][col];  // Return the detected key
			}
		}

		// Set i-th row to LOW before checking next row
		KEYPAD_ROW_Port->BRR = BIT(keypad_gpio.row_pins[row]);
	}

	return '\0';
}
