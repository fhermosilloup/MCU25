/*
 * Uart.h
 *
 *  Created on: Apr 16, 2025
 *      Author: User123
 */

#ifndef __UART_ASYNC_H_
#define __UART_ASYNC_H_

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_uart.h"


#define UART_ID_1 1
#define UART_ID_2 2
#define UART_ID_3 3
#define UART_ID_4 4
#define UART_ID_5 5
#define UART_ID_6 6


#define configUART_PERIPH_ID			(UART_ID_3)
#define configUART_ASYNC_TXBUFFER_SIZE	(64)
#define configUART_ASYNC_TX_ENABLE
#define configUART_ASYNC_RX_ENABLE


typedef void(*UartAsync_Callback)(uint8_t);


void UartAsync_Init(void);
int UartAsync_Send(const uint8_t ByteToWrite);
// TODO: Add timeout
int UartAsync_Sendn(const uint8_t *pBytesToWrite, uint16_t len);
int UartAsync_TxAvailable(void);

int UartAsync_InstallCallback(UartAsync_Callback onReceiveCb);
int UartAsync_UnistallCallback(void);

#endif /* __UART_ASYNC_H_ */
