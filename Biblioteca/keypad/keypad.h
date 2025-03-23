/*
 * keypad.h
 *
 *  Created on: Feb 28, 2025
 *      Author: User123
 */

#ifndef KEYPAD_H_
#define KEYPAD_H_

/* Exported includes -----------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "common.h"

/* Exported defines ------------------------------------------------*/
// Define the GPIO ports and pins for rows and columns
#define KEYPAD_ROW_Port GPIOB
#define KEYPAD_ROW_Pin  (0x0FUL)
#define KEYPAD_COL_Port GPIOB
#define KEYPAD_COL_Pin  (0x0FUL << 4)

#define KEYPAD_FSM_DEBOUNCE_DELAY 10

/* Exported macros -------------------------------------------------*/

/* Exported typedefs ------------------------------------------------*/
typedef void(*keypad_event_handler_t)(char);

typedef enum
{
	KEY_IDLE = 0,
	KEY_PRESSED,
	KEY_RELEASED
} KeyState;

typedef struct
{
	char key;
	KeyState state;
} KeyEvent_t;

/* Exported variables -----------------------------------------------*/

/* Exported prototype function --------------------------------------*/
int Keypad_Init(keypad_event_handler_t onKeyPressedCallback, keypad_event_handler_t onKeyReleasedCallback);

char Keypad_GetKey(void);

void Keypad_Update(KeyEvent_t *event);

#endif /* KEYPAD_H_ */
