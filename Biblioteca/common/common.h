/*
 * common.h
 *
 *  Created on: Feb 24, 2025
 *      Author: User123
 */

#ifndef COMMON_H_
#define COMMON_H_

/* Exported includes -----------------------------------------------*/
#include <stdint.h>
#include <string.h>
#include "stm32f1xx_hal.h"

/* Exported defines ------------------------------------------------*/

/* Exported macros -------------------------------------------------*/
#define BIT(n)				(1UL << (n))

/* Exported typedefs ------------------------------------------------*/

/* Exported variables -----------------------------------------------*/

/* Exported prototype function --------------------------------------*/

/* Exported inline function --------------------------------------*/
void delay_init(void);
void delay_ms(uint32_t ms);
void delay_10us(uint32_t usx10);
void delay_us(uint32_t us);

#endif /* COMMON_H_ */
