/*
 * Uart.c
 *
 *  Created on: Apr 16, 2025
 *      Author: User123
 */

/* Private Includes -----------------------------------------------*/
#include <UartAsync.h>

/* Private Defines ------------------------------------------------*/
#ifdef configUART_ASYNC_TX_ENABLE
#ifndef configUART_ASYNC_TXBUFFER_SIZE
#error "configUART_ASYNC_TXBUFFER_SIZE must be defined!"
#else
#if configUART_ASYNC_TXBUFFER_SIZE != 0 && (configUART_ASYNC_TXBUFFER_SIZE & (configUART_ASYNC_TXBUFFER_SIZE - 1)) != 0
#error "configUART_ASYNC_TXBUFFER_SIZE must be power of two"
#endif
#endif
#endif

#ifdef configUART_PERIPH_ID
	#if configUART_PERIPH_ID == 1
		#define UartAsync				USART1
		#define UartAsync_IRQHandler	USART1_IRQHandler
		#define UartAsync_IRQ     		USART1_IRQn

	#elif configUART_PERIPH_ID == 2
		#define UartAsync				USART2
		#define UartAsync_IRQHandler	USART2_IRQHandler
		#define UartAsync_IRQ     		USART2_IRQn

	#elif configUART_PERIPH_ID == 3
		#define UartAsync				USART3
		#define UartAsync_IRQHandler	USART3_IRQHandler
		#define UartAsync_IRQ     		USART3_IRQn

	#elif configUART_PERIPH_ID == 4
		#define UartAsync				UART4
		#define UartAsync_IRQHandler	UART4_IRQHandler
		#define UartAsync_IRQ     		UART4_IRQn

	#elif configUART_PERIPH_ID == 5
		#define UartAsync				UART5
		#define UartAsync_IRQHandler	UART5_IRQHandler
		#define UartAsync_IRQ     		UART5_IRQn

	#elif configUART_PERIPH_ID == 6
		#define UartAsync				USART6
		#define UartAsync_IRQHandler	USART6_IRQHandler
		#define UartAsync_IRQ     		USART6_IRQn
	#else
		#error "Unsupported configUART_PERIPH_ID."
#endif
#else
	#error "configUART_PERIPH_ID must be defined."
#endif

/* Private typedefs -----------------------------------------------*/
typedef struct
{
#ifdef configUART_ASYNC_TX_ENABLE
	struct
	{
		uint8_t Buffer[configUART_ASYNC_TXBUFFER_SIZE];
		uint16_t WriteIdx;
		uint16_t ReadIdx;
	}TxBuffer;
#endif
}UartAsync_HandleTypeDef;


/* Private Variables ----------------------------------------------*/
#ifdef configUART_ASYNC_TX_ENABLE
UartAsync_HandleTypeDef hUartAsync;
#endif
UartAsync_Callback UartAsync_onReceiveCallback = NULL;

/* Private Driver Code --------------------------------------------*/
void UartAsync_BufferFlush(void)
{
#ifdef configUART_ASYNC_TX_ENABLE
	hUartAsync.TxBuffer.ReadIdx = 0;
	hUartAsync.TxBuffer.WriteIdx = 0;
#endif
}


/* Exported Function Reference ------------------------------------*/
void UartAsync_Init(void)
{
#ifdef configUART_ASYNC_TX_ENABLE
	hUartAsync.TxBuffer.ReadIdx = 0;
	hUartAsync.TxBuffer.WriteIdx = 0;
#endif
	// Enable interrupts
	UartAsync->CR1 |= USART_CR1_RXNEIE;	// RX not empty interrupt
	UartAsync->CR1 &= ~USART_CR1_TXEIE;	// Tx interrupt is disable

	HAL_NVIC_SetPriority(UartAsync_IRQ, 0, 0);
	HAL_NVIC_EnableIRQ(UartAsync_IRQ);
}

int UartAsync_InstallCallback(UartAsync_Callback onReceiveCb)
{
	int retVal = 0;

	__disable_irq();
	if(!UartAsync_onReceiveCallback)
	{
		UartAsync_onReceiveCallback = onReceiveCb;
		retVal = 1;
	}
	__enable_irq();

	return retVal;
}

int UartAsync_UnistallCallback(void)
{
	int retVal = 0;

	__disable_irq();
	if(UartAsync_onReceiveCallback)
	{
		UartAsync_onReceiveCallback = NULL;
		retVal = 1;
	}
	__enable_irq();

	return retVal;
}


int UartAsync_Send(const uint8_t ByteToWrite)
{
	int retVal = 0;
#ifdef configUART_ASYNC_TX_ENABLE
	// Check if the buffer is not full
	uint16_t uNextWriteIdx = (hUartAsync.TxBuffer.WriteIdx+1) & (configUART_ASYNC_TXBUFFER_SIZE-1);
	if( uNextWriteIdx !=  hUartAsync.TxBuffer.ReadIdx)
	{
		hUartAsync.TxBuffer.Buffer[hUartAsync.TxBuffer.WriteIdx] = ByteToWrite;
		hUartAsync.TxBuffer.WriteIdx = uNextWriteIdx;

		// Enable interrupt if it is disabled
		if( !(UartAsync->CR1 & USART_CR1_TXEIE) )
		{
			UartAsync->CR1 |= USART_CR1_TXEIE;
		}

		retVal = 1;
	}
#endif

	return retVal;
}

int UartAsync_Sendn(const uint8_t *pBytesToWrite, uint16_t len)
{
	int retVal = 0;
#ifdef configUART_ASYNC_TX_ENABLE
	if(UartAsync_TxAvailable() >= len)
	{
		// Store in buffer
		for(int n = 0; n < len; n++)
		{
			hUartAsync.TxBuffer.Buffer[hUartAsync.TxBuffer.WriteIdx++] = *pBytesToWrite++;
			hUartAsync.TxBuffer.WriteIdx &= (configUART_ASYNC_TXBUFFER_SIZE-1);
		}

		// Enable interrupt if it is disabled
		if( !(UartAsync->CR1 & USART_CR1_TXEIE) )
		{
			UartAsync->CR1 |= USART_CR1_TXEIE;
		}

		retVal = 1;
	}
#endif

	return retVal;
}

int UartAsync_TxAvailable(void)
{
#ifdef configUART_ASYNC_TX_ENABLE
	__disable_irq();
	uint16_t WriteIdx = hUartAsync.TxBuffer.WriteIdx;
	uint16_t ReadIdx = hUartAsync.TxBuffer.ReadIdx;
	__enable_irq();
	if (WriteIdx >= ReadIdx)
		return configUART_ASYNC_TXBUFFER_SIZE - (WriteIdx - ReadIdx);
	else
		return (ReadIdx - WriteIdx);
#else
	return 0;
#endif
}



void UartAsync_IRQHandler(void)
{
#ifdef configUART_ASYNC_RX_ENABLE
	// Check USARTx RX not empty flag
	if (UartAsync->SR & USART_SR_RXNE)
	{
		// Read from Data register clears RX not empty flag
		uint8_t ucByte = (uint8_t)(UartAsync->DR & 0x000000FF);

		// Check for framing
		if(UartAsync_onReceiveCallback) UartAsync_onReceiveCallback(ucByte);
	}
#endif

#ifdef configUART_ASYNC_TX_ENABLE
	// Check USART2 RX empty flag
	if ((UartAsync->CR1 & USART_CR1_TXEIE) && (UartAsync->SR & USART_SR_TXE))
	{
		// Check if the buffer is not full
		if( hUartAsync.TxBuffer.ReadIdx !=  hUartAsync.TxBuffer.WriteIdx)
		{
			UartAsync->DR = hUartAsync.TxBuffer.Buffer[hUartAsync.TxBuffer.ReadIdx++];	// Write to Data register clears TX empty flag
			hUartAsync.TxBuffer.ReadIdx &= (configUART_ASYNC_TXBUFFER_SIZE-1);
		}
		else
		{
			// Disable USART TX empty flag if there is nothing to send
			UartAsync->CR1 &= ~USART_CR1_TXEIE;
		}
	}
#endif
}

