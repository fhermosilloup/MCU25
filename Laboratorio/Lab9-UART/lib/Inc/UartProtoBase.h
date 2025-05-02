#ifndef __UART_PROTO_BASE_H
#define __UART_PROTO_BASE_H
/* Exported Includes -----------------------------------------------*/
#include <UartAsync.h>
#include <stdint.h>
#include <stdlib.h>

/* Exported Defines ------------------------------------------------*/
// Frame Status Register (FSR) Bit Fields Definition
#define PROTOCOL_PEN_FLAG	(1<<0)
#define PROTOCOL_RXIE_FLAG	(1<<1)
#define PROTOCOL_SOF_FLAG	(1<<2)	/*! Framing Start of Frame flag */
#define PROTOCOL_EOF_FLAG	(1<<3)
#define PROTOCOL_RD_FLAG	(1<<4)

/* Errors */
#define PROTO_FAIL					(0)
#define PROTO_OK					(1)
#define PROTO_ERROR_TIMEOUT			(2)
#define PROTO_ERROR_ID				(3)
#define PROTO_ERROR_CHECKSUM		(4)
#define PROTO_ERROR_PAYLOAD_LEN		(5)
#define PROTO_ERROR_PAYLOAD			(6)

/* Exported Macros -------------------------------------------------*/

/* Exported Typedefs -----------------------------------------------*/
typedef struct _Protocol_TypeDef
{
	// Common data
	void *pvBuffer;
	uint8_t FrameCount;
	uint8_t Status;
	
	// Callbacks
	int(*onReceive)(struct _Protocol_TypeDef *, uint8_t);
	int(*checkSOF)(struct _Protocol_TypeDef *, uint8_t);
	int(*checkEOF)(struct _Protocol_TypeDef *, uint8_t);
} Protocol_TypeDef;

/* Exported Constants ----------------------------------------------*/

/* Exported Variables ----------------------------------------------*/

/* Exported Function Prototype -------------------------------------*/

/* Exported user code ----------------------------------------------*/

/* Exported reference function ------------------------------------*/

/* Exported Function Reference -------------------------------------*/

#endif /* __UART_PROTO_BASE_H */
