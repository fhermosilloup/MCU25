/*
 *  UART NMEA Protocol Application Layer
 *
 * <SOF><xx><YYY>,<FIELD1>,<FIELD2>,...,<FIELDn>*<CSHigh><CSLow><EOF>
 * <SOF>: '$'
 * <EOF>: <CR><LF> = "\r\n"
 */
 
/* Private Includes -----------------------------------------------*/
#include "NmeaUartProto.h"
#include <string.h>

/* Private Defines ------------------------------------------------*/
#define NMEA_SOF_BYTE		('$')
#define NMEA_EOF0_BYTE		('\r')
#define NMEA_EOF1_BYTE		('\n')
#define NMEA_BUFFER_SIZE	(128)

/* Private Macros -------------------------------------------------*/
#define NMEA_IS_DIGIT(chr) 	((chr) >= '0' && (chr) <= '9')
#define NMEA_IS_AF(chr)		((chr) >= 'A' && (chr) <= 'F')

/* Private Typedefs -----------------------------------------------*/
typedef enum
{
	NMEA_READ_IDLE = 0,
	NMEA_READ_WAIT,
	NMEA_READ_SUCCESS,
	NMEA_READ_WAIT_SYNC
} eNMEAReadState;

typedef struct
{
	char Buffer[NMEA_BUFFER_SIZE];
	uint16_t ReadIdx;
	uint16_t WriteIdx;
} NmeaBuffer_TypeDef;

/* Private Constants ----------------------------------------------*/

/* Private Variables ----------------------------------------------*/
NmeaBuffer_TypeDef xNmeaMailBox;

/* Private Function Prototype -------------------------------------*/
// String
static int strfind(char *str, char chr, uint32_t offset);

// Circular Buffer
int NmeaBuffer_Write(NmeaBuffer_TypeDef *pxCBuffer, const char *pcItemToWrite);
int NmeaBuffer_Read(NmeaBuffer_TypeDef *pxCBuffer, char *pcItemToRead);


/* Private user code ----------------------------------------------*/
int NMEA_CheckSOF(Protocol_TypeDef *self, uint8_t ucByte)
{
	int retVal = 0;
	NmeaBuffer_TypeDef *pxCBuffer =(NmeaBuffer_TypeDef *)self->pvBuffer;
	if(ucByte==NMEA_SOF_BYTE)
	{
		if( NmeaBuffer_Write(pxCBuffer, &ucByte) )
		{
			retVal = 1;
		}
	}

	return retVal;
}

int NMEA_CheckEOF(Protocol_TypeDef *self, uint8_t byte)
{
	int retVal = 0;

	if(self->Status & PROTOCOL_EOF_FLAG)
	{
		if(byte == NMEA_EOF1_BYTE)
		{
			self->FrameCount++;
			retVal = 1;
		}
		self->Status &= ~PROTOCOL_EOF_FLAG;
	}
	else if(byte == NMEA_EOF0_BYTE)
	{
		self->Status |= PROTOCOL_EOF_FLAG;
	}

	return retVal;
}


int NMEA_onReceiveCallback(Protocol_TypeDef *self, uint8_t byte)
{
	int retVal = 0;
	NmeaBuffer_TypeDef *pxCBuffer =(NmeaBuffer_TypeDef *)self->pvBuffer;

	// write to buffer
	if( NmeaBuffer_Write(pxCBuffer, &byte) ) retVal = 1;

	return retVal;
}




/* Exported reference function ------------------------------------*/
void NMEA_Init(Protocol_TypeDef *proto)
{
	// Framing
	proto->FrameCount = 0;
	proto->Status = PROTOCOL_PEN_FLAG;

	// Buffer
	NmeaBuffer_Init(&xNmeaMailBox);
	proto->pvBuffer = &xNmeaMailBox;

	// Callbacks
	proto->onReceive = NMEA_onReceiveCallback;
	proto->checkSOF = NMEA_CheckSOF;
	proto->checkEOF = NMEA_CheckEOF;
}

int NMEA_Checksum(NMEA_MessageDescriptor * msg)
{
	if(!msg || !msg->BufferLen) return PROTO_FAIL;

	// Check for SOF & EOP chars
	if (msg->Buffer[0] != '$') return PROTO_FAIL;

	char *pBufPtr = &msg->Buffer[1];
	uint8_t expectedChecksum = 0x00;
	while(*pBufPtr != '*')
	{
		expectedChecksum = expectedChecksum ^ (uint8_t)*pBufPtr++;
	}
	pBufPtr++;

	uint8_t recvChecksum = 0x00;
	for(int i = 0; i < 2; i++)
	{
		char curDig = *pBufPtr++;
		if(NMEA_IS_DIGIT(curDig)) recvChecksum |= (uint8_t)(curDig - '0') << 4*(1-i);
		else if(NMEA_IS_AF(curDig)) recvChecksum |= (uint8_t)(curDig - 'A' + 10) << 4*(1-i);
		else return PROTO_FAIL;
	}

	return (recvChecksum == expectedChecksum ? PROTO_OK : PROTO_FAIL);
}

int NMEA_Recv(Protocol_TypeDef *proto, uint8_t *ByteToRead)
{
	int retVal = 0;
	if(!ByteToRead) return 0;

	// Check if the buffer is not full
	if( NmeaBuffer_Read((NmeaBuffer_TypeDef *)proto->pvBuffer, ByteToRead) )
	{
		// Decrement FrameCount if a full frame has been recv
		if(!(proto->Status & PROTOCOL_RD_FLAG))
		{
			// Set Recv SOF byte
			if(*ByteToRead == '$')
			{
				proto->Status |= PROTOCOL_RD_FLAG;
			}
		}
		else
		{
			// Clear Recv SOF byte & decrement FrameCount
			if(*ByteToRead == '\n')
			{
				proto->Status &= ~PROTOCOL_RD_FLAG;
				__disable_irq();
				proto->FrameCount--;
				__enable_irq();
			}
		}

		retVal = 1;
	}

	return retVal;
}


int NMEA_Available(Protocol_TypeDef *proto)
{
	
	if(proto->FrameCount) return proto->FrameCount;

	return 0;
}


/* Private Function Reference -------------------------------------*/
int strfind(char *str, char chr, uint32_t offset)
{
	int chrPos = -1;
	int chrCount = 0;
	char *strPtr = str + offset;
	while(*strPtr != '\0')
	{
		if(*strPtr++ == chr)
		{
			chrPos = chrCount + offset;
			break;
		}
		chrCount++;
	}

	return chrPos;
}

int NmeaBuffer_Init(NmeaBuffer_TypeDef *pxCBuffer)
{
	pxCBuffer->WriteIdx = 0;
	pxCBuffer->ReadIdx = 0;

	return 1;
}
int NmeaBuffer_Write(NmeaBuffer_TypeDef *pxCBuffer, const char ItemToWrite)
{
	int retVal = 0;
	uint16_t uNextWriteIdx = (pxCBuffer->WriteIdx+1) & (NMEA_BUFFER_SIZE-1);
	if( uNextWriteIdx !=  pxCBuffer->ReadIdx)
	{
		// Copy data to buffer
		pxCBuffer->Buffer[pxCBuffer->WriteIdx] = ItemToWrite;

		// Update WriteIdx
		pxCBuffer->WriteIdx = uNextWriteIdx;

		retVal = 1;
	}

	return retVal;
}

int NmeaBuffer_Read(NmeaBuffer_TypeDef *pxCBuffer, char *pcItemToRead)
{
	int retVal = 0;

	// Check if the buffer is not full
	uint16_t uNextReadIdx = (pxCBuffer->ReadIdx+1) & (NMEA_BUFFER_SIZE-1);
	if( uNextReadIdx !=  pxCBuffer->WriteIdx)
	{
		// Copy data to buffer
		pcItemToRead = pxCBuffer->Buffer[pxCBuffer->ReadIdx];

		// Update ReadIdx
		pxCBuffer->ReadIdx = uNextReadIdx;

		retVal = 1;
	}

	return retVal;
}
