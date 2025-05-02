/*
 * UBX Protocol Application Layer
 * [SYNC_0, SYNC_1, CLASS, ID, LENGTH<2>, PAYLOAD, CK_A, CK_B
 * Checksum: CLASS, ID, LENGTH, PAYLOAD
 *  CK_A = 0, CK_B = 0
 *  For(I=0;I<N;I++)
 *  {
 *  	CK_A = CK_A + Buffer[I]
 *  	CK_B = CK_B + CK_A
 *  }
 *
 * CLASS
 * 0x01 - NAV (Position, Speed, Time, ...)
 * 0x02 - RXM
 * 0x04 - INF
 * 0x05 - ACK (Acknowledge or Reject messages to UBX-CFG input messages)
 * 0x06 - CFG (Set Dynamic Model, Set DOP Mask, Set Baud Rate, ...)
 * 0x09 - UPD
 * 0x0A - MON
 * 0x0D - TIM
 * 0x13 - MGA
 * 0x21 - LOG
 * 0x27 - SEC
 *
 * UBX-ACK-ACK
 * 0xB5 0x62 0x05 0x01 0x0200 CLASS ID CK_A CK_B
 *
 * UBX-ACK-NAK
 * 0xB5 0x62 0x05 0x00 0x0200 CLASS ID CK_A CK_B
 *
 * UBX-CFG-RATE
 * 0xB5 0x62 0x06 0x08 0x0600 MEAS_RATE<2> NAV_RATE<2> TIME_REF<2> CK_A CK_B
 * MEAS_RATE: in ms
 * NAV_RATE:
 * TIME_REF: {0-4}
 * 	0 - UTC
 * 	1 - GPS
 * 	2 - GLONAS
 * 	3 - BeiDou
 * 	4 - Galileo
 * @example: B5 62 06 08 06 00 C8 00 01 00 00 00 CK_A CK_B -> 200 ms, navRate = 1, UTC
 *
 * 	UBX-CFG-MSG: Set Message rate on current Port
 * 	0xB5 0x62 0x06 0x01 0x0300 msgClass msgID rate CK_A CK_B
 * 	msgClass msgID 	Type
 * 	0xF0	0x00	GGA
 * 	0xF0	0x01	GLL
 * 	0xF0	0x02	GSA
 * 	0xF0	0x03	GSV
 * 	0xF0	0x04	RMC
 * 	0xF0	0x05	VTG
 * @example: B5 62 06 01 03 00 F0 00 00 CK_A CK_B -> NMEA.GGA, rate = 0 [Disable]
 */
 
/* Private Includes -----------------------------------------------*/
#include "UbxUartProto.h"

/* Private Defines ------------------------------------------------*/
#define UBX_SYNC_CHAR_0		(0xB5)
#define UBX_SYNC_CHAR_1		(0x62)
#define UBX_MIN_FRAME_SIZE	(8)
#define UBX_CFG_MSG_LEN		(0x03)
#define UBX_CFG_RATE_LEN	(0x06)
#define UBX_CFG_PRT_LEN		(0x14)
#define UBX_ACK_PAYLOAD_LEN	(0x02)


/* Private Macros -------------------------------------------------*/

/* Private Typedefs -----------------------------------------------*/
typedef enum
{
	UBX_RECV_CLASS_STATE = 0,
	UBX_RECV_ID_STATE,
	UBX_RECV_LENGTH_STATE,
	UBX_RECV_PAYLOAD_STATE,
	UBX_RECV_CHECKSUM_STATE
} eUbxRecvState;

/* Private Constants ----------------------------------------------*/

/* Private Variables ----------------------------------------------*/

/* Private Function Prototype -------------------------------------*/

/* Private user code ----------------------------------------------*/
int UBX_CheckSOF(Protocol_TypeDef * self, uint8_t ucByte)
{
	int retVal = 0;
	if(self->Status & PROTOCOL_SOF_FLAG)
	{
		if(ucByte==UBX_SYNC_CHAR_1)
		{
			retVal = 1;
		}
		self->Status &= ~PROTOCOL_SOF_FLAG;
	}
	else if(ucByte==UBX_SYNC_CHAR_0)
	{
		self->Status |= PROTOCOL_SOF_FLAG;
	}

	return retVal;
}

int UBX_CheckEOF(Protocol_TypeDef *self, uint8_t ucByte)
{
	int retVal = 0;

	if(self->Status & PROTOCOL_EOF_FLAG)
	{
		retVal = 1;
		self->Status &= ~PROTOCOL_EOF_FLAG;

	}

	return retVal;
}


int UBX_onReceiveCallback(Protocol_TypeDef * self, uint8_t byte)
{
	static eUbxRecvState xUbxRecvState = UBX_RECV_CLASS_STATE;
	static uint8_t ucCount = 0x00;
	UbxFrame_TypeDef *pxCurFrame = (UbxFrame_TypeDef *)self->pvBuffer;

	switch(xUbxRecvState)
	{
		case UBX_RECV_CLASS_STATE:
			pxCurFrame->class = byte;
			ucCount = 0x00;
			xUbxRecvState = UBX_RECV_ID_STATE;
		break;

		case UBX_RECV_ID_STATE:
			pxCurFrame->id = byte;
			xUbxRecvState = UBX_RECV_LENGTH_STATE;
		break;

		case UBX_RECV_LENGTH_STATE:
			if(!ucCount)
			{
				// LSByte
				pxCurFrame->len = byte;
				ucCount = 1;
			}
			else
			{
				// MSByte
				pxCurFrame->len |= ((uint16_t)byte << 8);
				ucCount = 0x00;
				xUbxRecvState = UBX_RECV_PAYLOAD_STATE;
			}
		break;

		case UBX_RECV_PAYLOAD_STATE:
			pxCurFrame->payload[ucCount++] = byte;
			if(ucCount == pxCurFrame->len || ucCount == configUBX_MAX_PAYLOAD_SIZE)
			{
				ucCount = 0;
				xUbxRecvState = UBX_RECV_CHECKSUM_STATE;
			}
		break;

		case UBX_RECV_CHECKSUM_STATE:
			pxCurFrame->checksum[ucCount++] = byte;
			if(ucCount==2)
			{
				xUbxRecvState = UBX_RECV_CLASS_STATE;
				self->FrameCount++;
				self->Status |= PROTOCOL_EOF_FLAG;
			}
		break;
	}

	return 1;
}



/* Exported reference function ------------------------------------*/
/*
 * UBX Protocol
 * SYNC_0<1>, SYNC_1<1>, CLASS<1>, ID<1>, LENGTH<2>, PAYLOAD<N-4>, CK_A<1>, CK_B<1>
 */
void UBX_Init(Protocol_TypeDef *pxInstance)
{
	pxInstance->FrameCount = 0;
	pxInstance->Status = 0x00;
	xUbxFrame.payload = ucUbxPayload;
	pxInstance->pvBuffer = &xUbxFrame;
	
	pxInstance->onReceive = UBX_onReceiveCallback;
	pxInstance->checkSOF = UBX_CheckSOF;
	pxInstance->checkEOF = UBX_CheckEOF;
}

/*
 * Checksum: CLASS, ID, LENGTH, PAYLOAD
 *  CK_A = 0, CK_B = 0
 *  For(I=0;I<N;I++)
 *  {
 *  	CK_A = CK_A + Buffer[I]
 *  	CK_B = CK_B + CK_A
 *  }
 */
void UBX_ComputeChecksum(UbxFrame_TypeDef *pxUbxFrame)
{
	pxUbxFrame->checksum[0] = pxUbxFrame->class;
	pxUbxFrame->checksum[1] = pxUbxFrame->checksum[0];

	pxUbxFrame->checksum[0] += pxUbxFrame->id;
	pxUbxFrame->checksum[1] += pxUbxFrame->checksum[0];

	pxUbxFrame->checksum[0] += (pxUbxFrame->len & 0x00FF);
	pxUbxFrame->checksum[1] += pxUbxFrame->checksum[0];

	pxUbxFrame->checksum[0] += ((pxUbxFrame->len & 0xFF00) >> 8);
	pxUbxFrame->checksum[1] += pxUbxFrame->checksum[0];
	for(int i = 0; i < pxUbxFrame->len; i++)
	{
		pxUbxFrame->checksum[0] += pxUbxFrame->payload[i];
		pxUbxFrame->checksum[1] += pxUbxFrame->checksum[0];
	}
}

int UBX_Send(UbxFrame_TypeDef *pxUbxFrame)
{
	int retVal = 0;
	{
		// Compute checksum
		UBX_ComputeChecksum(pxUbxFrame);

		// Transmit
		UartAsync_Send(UBX_SYNC_CHAR_0);
		UartAsync_Send(UBX_SYNC_CHAR_1);
		UartAsync_Send(pxUbxFrame->class);
		UartAsync_Send(pxUbxFrame->id);
		UartAsync_Send((pxUbxFrame->len & 0x00FF));
		UartAsync_Send(((pxUbxFrame->len & 0xFF00) >> 8));
		UartAsync_Sendn(pxUbxFrame->payload, pxUbxFrame->len);
		UartAsync_Sendn(pxUbxFrame->checksum,2);

		//
		retVal = 1;
	}

	return retVal;
}

int UBX_Available(Protocol_TypeDef *proto)
{
	if(proto->FrameCount)
	{
		return proto->FrameCount;
	}

	return 0;
}

int UBX_WaitAck(Protocol_TypeDef *proto, uint8_t msgClass, uint8_t msgID, uint32_t ulTimeout)
{
	// If there is a full frame
	UbxFrame_TypeDef *pxUbxFrame = NULL;
	uint32_t ulTimeoutAbs = HAL_GetTick() + ulTimeout;
	while(1)
	{
		if(HAL_GetTick() < ulTimeoutAbs)
		{
			// Check if is a valid UBX message
			if( UBX_Recv(proto, &pxUbxFrame) == PROTO_OK )
			{
				// Check for ack frame
				if (pxUbxFrame->class != UBX_ACK) return PROTO_ERROR_ID;
				if (pxUbxFrame->len != UBX_ACK_PAYLOAD_LEN) return PROTO_ERROR_PAYLOAD_LEN;

				// Check payload
				if (pxUbxFrame->id == UBX_ACK_ACK)
				{
					if(pxUbxFrame->payload[0] == msgClass && pxUbxFrame->payload[1] == msgID) return PROTO_OK;
					else return PROTO_ERROR_PAYLOAD;
				}
				else {
					return PROTO_ERROR_PAYLOAD;
				}
			}
		}
		else
		{
			return PROTO_ERROR_TIMEOUT;
		}
	}

	return PROTO_FAIL;
}


int UBX_Recv(Protocol_TypeDef *proto, UbxFrame_TypeDef **pxUbxFrame)
{
	if(!UBX_Available(proto)) return PROTO_FAIL;

	*pxUbxFrame = (UbxFrame_TypeDef *)proto->pvBuffer;

	// Decrement FrameCount
	__disable_irq();
	proto->FrameCount--;
	__enable_irq();

	// Check checksum
	uint8_t checksum[2];
	checksum[0] = ((UbxFrame_TypeDef *)proto->pvBuffer)->checksum[0];
	checksum[1] = ((UbxFrame_TypeDef *)proto->pvBuffer)->checksum[1];
	UBX_ComputeChecksum(((UbxFrame_TypeDef *)proto->pvBuffer));
	if (checksum[0] != ((UbxFrame_TypeDef *)proto->pvBuffer)->checksum[0] || checksum[1] != ((UbxFrame_TypeDef *)proto->pvBuffer)->checksum[1]) return PROTO_ERROR_CHECKSUM;

	return PROTO_OK;
}


/* Private Function Reference -------------------------------------*/
