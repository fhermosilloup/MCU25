/*
 * Neo6N.c
 *
 *  Created on: Apr 16, 2025
 *      Author: User123
 */

/* Private Includes -----------------------------------------------*/
#include <Neo6M.h>

/* Private Defines ------------------------------------------------*/


/* Private Macros -------------------------------------------------*/


/* Private Typedefs -----------------------------------------------*/
typedef enum {
	PROTOCOL_SOF_STATE = 0,
	PROTOCOL_RECV_STATE
} eProtocolState;

typedef enum
{
	PROTOCOL_ID_NMEA = 0,
	PROTOCOL_ID_UBX,
	PROTOCOL_ID_MAX
} eProtocolID;

typedef struct
{
	Protocol_TypeDef Protocols[PROTOCOL_ID_MAX];
	eProtocolState State;
	Protocol_TypeDef* ActiveProtocol;
} Protocol_HandlerTypeDef;

/* Private Constants ----------------------------------------------*/
//const char GoogleStr[] = "http://www.google.com/maps/place/";
const char MessageIdStr[6][3] =
{
	"GGA",
	"GLL",
	"GSA",
	"GSV",
	"RMC",
	"VTG"
};

/* Private Variables ----------------------------------------------*/
Protocol_HandlerTypeDef hProtocol;

/* Private Function Prototype -------------------------------------*/
static int NEO6M_Config(Neo6mConfig_TypeDef *cfg);
static eUbxMessageID NEO6M_GetID(const char *buffer);



/* Private user code ----------------------------------------------*/
void NEO6M_onUartReceiveCallback(uint8_t byte)
{
	switch(hProtocol.State)
	{
		case PROTOCOL_SOF_STATE:
			for (uint8_t i = 0; i < PROTOCOL_ID_MAX; i++)
			{
				Protocol_TypeDef* proto = &hProtocol.Protocols[i];
				if(proto->Status & PROTOCOL_RXIE_FLAG)
				{
					if( proto->checkSOF(proto, byte) )
					{
						hProtocol.ActiveProtocol = proto;
						hProtocol.State = PROTOCOL_RECV_STATE;
						break;
					}
				}
			}
		break;

		case PROTOCOL_RECV_STATE:
			// Receive data
			hProtocol.ActiveProtocol->onReceive(hProtocol.ActiveProtocol, byte);

			// Check for EOF
			if( hProtocol.ActiveProtocol->checkEOF(hProtocol.ActiveProtocol, byte) )
			{
				hProtocol.ActiveProtocol = NULL;
				hProtocol.State = PROTOCOL_SOF_STATE;
			}
		break;
	}
}







/* Exported reference function ------------------------------------*/
void NEO6M_Init(Neo6mConfig_TypeDef *cfg)
{
	/* Protocol init */
	hProtocol.State = PROTOCOL_SOF_STATE;
	hProtocol.ActiveProtocol = NULL;

	// Protocol config
	NMEA_Init(&hProtocol.Protocols[PROTOCOL_ID_NMEA]);
	UBX_Init(&hProtocol.Protocols[PROTOCOL_ID_UBX]);

	/* Uart config */
	UartAsync_Init();
	UartAsync_InstallCallback(NEO6M_onUartReceiveCallback);

	/* Config */
	if(cfg)
	{
		NEO6M_Config(cfg);
	}
}

int NEO6M_Start(void)
{
	int retVal = PROTO_FAIL;

	if((hProtocol.Protocols[PROTOCOL_ID_NMEA].Status & PROTOCOL_PEN_FLAG))
	{
		hProtocol.Protocols[PROTOCOL_ID_NMEA].Status |= PROTOCOL_RXIE_FLAG;
		retVal = PROTO_OK;
	}

	return retVal;
}

/*void NEO6M_ToGoogle(char *link)
{

   // Return a link of google Maps: http://www.google.com/maps/place/Latitud,Longitud
	#ifdef configNEO6M_DEBUG
		Serial.println(F("Google"));
	#endif

	NEO6M_Data xGpsData;
	xGpsData.Latitude[0] = '\0';
	xGpsData.LatitudeHemisphere[0] = '\0';
	xGpsData.Longitude[0] = '\0';
	xGpsData.LongitudeMeridian[0] = '\0';
	strcpy(link, (const char *)&string_table[0]);
	NEO6M_GetGPRMC(&xGpsData);
	NEO6M_ToLatitude(xGpsData.Latitude,xGpsData.Latitude);
	NEO6M_ToLongitude(xGpsData.Longitude,xGpsData.Longitude);

	if (xGpsData.LatitudeHemisphere[0]=='S') strcat(link,"-");
	strcat(link,xGpsData.Latitude);
	strcat(link,",");
	if (xGpsData.LongitudeMeridian[0]=='W') strcat(link,"-");
	strcat(link,xGpsData.Longitude);

	#ifdef configNEO6M_DEBUG
		Serial.println("EndGoogle");
	#endif
}
*/

int NEO6M_Write(eUbxMessageClass msgClass, eUbxMessageID msgID, uint8_t *payload, uint16_t len)
{
	if (len && !payload) return PROTO_FAIL;
	if( !(hProtocol.Protocols[PROTOCOL_ID_UBX].Status & PROTOCOL_PEN_FLAG) ) return PROTO_FAIL;

	int retVal = PROTO_FAIL;

	// Try to send
	if(UartAsync_TxAvailable() >= (UBX_MIN_FRAME_SIZE + len))
	{
		// Encode in UBX format
		UbxFrame_TypeDef xUbxFrameTmp;
		xUbxFrameTmp.class = (uint8_t)msgClass;
		xUbxFrameTmp.id = (uint8_t)msgID;
		xUbxFrameTmp.payload = payload;
		xUbxFrameTmp.len = len;

		// Enable interrupt
		//hProtocol.Protocols[PROTOCOL_ID_UBX].Status |= PROTOCOL_RXIE_FLAG;

		// Send message
		if( UBX_Send(&xUbxFrameTmp) == PROTO_OK)
		{
			retVal = UBX_WaitAck(&hProtocol.Protocols[PROTOCOL_ID_UBX], (uint8_t)msgClass, (uint8_t)msgID, 500);
		}

		// Disable interrupt
		//hProtocol.Protocols[PROTOCOL_ID_UBX].Status &= ~PROTOCOL_RXIE_FLAG;

	}

	return retVal;
}

/*!
 * NMEA message: <SOF TALKER_ID MESSAGE_ID,FIELD1,...,FIELDn,*CHECKSUM EOF>
 * @param 	SOF: Start of frame ('$')
 * @param	TALKER_ID: Device that capture data {'GP' (GPS), 'GL' (GLONASS)}
 * @param	MESSAGE_ID: Type of message {
 * GPBOD - Bearing, origin to destination
 * GPBWC - Bearing and distance to waypoint, great circle
 * GPGGA - Global Positioning System Fix Data
 * GPGLL - Geographic position, latitude / longitude
 * GPGSA - GPS DOP and active satellites
 * GPGSV - GPS Satellites in view
 * GPHDT - Heading, True
 * GPR00 - List of waypoints in currently active route
 * GPRMA - Recommended minimum specific Loran-C data
 * GPRMB - Recommended minimum navigation info
 * GPRMC - Recommended minimum specific GPS/Transit data
 * GPRTE - Routes
 * GPTRF - Transit Fix Data
 * GPSTN - Multiple Data ID
 * GPVBW - Dual Ground / Water Speed
 * GPVTG - Track made good and ground speed
 * GPWPL - Waypoint location
 * GPXTE - Cross-track error, Measured
 * GPZDA - Date & Time
 * }
 * @param	FIELDk: k-th data
 * @param	CHECKSUM: Checksum of whole data but the SOF and EOF encoded as two-digit hex ascii chars
 * @param	EOF: End of frame ('\r\n')
 */
int NEO6M_Read(NMEA_MessageDescriptor *desc, const char *pcMessageID, uint32_t timeout)
{
	static eNMEAReadState xNeoStatus = NMEA_READ_IDLE;
	static uint32_t ulNeoTick = 0;
	int retVal = PROTO_FAIL;

	// Argin check
	if(!desc) return PROTO_FAIL;

	switch(xNeoStatus)
	{
		/* Idle */
		case NMEA_READ_IDLE:
			ulNeoTick = HAL_GetTick() + timeout;
			xNeoStatus = NMEA_READ_WAIT;
		break;

		/* Wait for new frame */
		case NMEA_READ_WAIT:
			if(HAL_GetTick() < ulNeoTick)
			{
				if( NMEA_Available(&hProtocol.Protocols[PROTOCOL_ID_NMEA]) > 0)
				{
					xNeoStatus = NMEA_READ_SUCCESS;
				}
			}
			else
			{
				xNeoStatus = NMEA_READ_IDLE;
				retVal = PROTO_ERROR_TIMEOUT;	// Timeout
			}
		break;

		/* Copy message */
		case NMEA_READ_SUCCESS:
		{
			retVal = PROTO_OK;
			desc->BufferLen = 0;
			char cChar = '\0';
			do
			{
				if( NMEA_Recv(&hProtocol.Protocols[PROTOCOL_ID_NMEA], (uint8_t *)&cChar) )
				{
					desc->Buffer[desc->BufferLen++] = cChar;
				}
			} while(cChar != '\n' && (desc->BufferLen < configNEO6M_BUFFER_SIZE-1));
			desc->Buffer[desc->BufferLen]='\0';

			// Verify checksum
			if( NMEA_Checksum(desc) != PROTO_OK)
			{
				retVal = PROTO_ERROR_CHECKSUM;
			}

			// Check for message ID
			if(pcMessageID)
			{

				if( !strstr(desc->Buffer, pcMessageID) )
				{
					retVal = PROTO_ERROR_ID;	// Not found
				}
			}

			desc->MsgID = NEO6M_GetID(&desc->Buffer[3]);

			// Next state
			xNeoStatus = NMEA_READ_IDLE;
		}
		break;

		case NMEA_READ_WAIT_SYNC:
			if( NMEA_Available(&hProtocol.Protocols[PROTOCOL_ID_NMEA]) < 0)
			{
				if((HAL_GetTick() - ulNeoTick) >= configNEO6M_STARTUP_TIMEOUT)
				{
					retVal = NEO6M_ERROR_STARTUP_FAIL;
				}
			}
			else
			{
				xNeoStatus = NMEA_READ_IDLE;
			}
		break;

		default:
			xNeoStatus = NMEA_READ_IDLE;
		break;
	}

	return retVal;
}


int NEO6M_ParseMessage(NMEA_MessageDescriptor *pMessage, NMEA_FieldDescriptor *desc, uint8_t num_desc)
{
	// Argin check
	if(!pMessage || !desc) return PROTO_FAIL;

	// Validate checksum
	if( !NMEA_Checksum(pMessage) ) return PROTO_ERROR_CHECKSUM;

	// Clear fields to parse
	for(int n = 0; n < num_desc; n++)
	{
		desc[n].Ptr = NULL;
		desc[n].len = 0;
	}

	// Parse fields
	NMEA_FieldDescriptor *pDesc = desc;
	uint32_t ulFieldCount = 0;
	uint32_t ulFoundFieldCount = 0;

	int CommaIndex = strfind(pMessage->Buffer, ',', 0);
	while(CommaIndex >= 0 && ulFoundFieldCount < num_desc)
	{
		ulFieldCount++;
		int NextCommaIndex = strfind(pMessage->Buffer, ',', CommaIndex + 1);

		// Check for valid comma index otherwise for checksum '*' preamble
		if(NextCommaIndex == -1)
		{
			NextCommaIndex = strfind(pMessage->Buffer, '*', CommaIndex + 1);
		}

		// second validation
		if(NextCommaIndex > 0)
		{
			if(ulFieldCount == (pDesc->FieldID & 0x0F))
			{
				if( ((pDesc->FieldID & 0xF0)>>4) == (uint8_t)pMessage->MsgID )
				{
					pDesc->len = NextCommaIndex - CommaIndex - 1;
					if(pDesc->len)
					{
						pDesc->Ptr = &pMessage->Buffer[CommaIndex+1];
					}
				}
				ulFoundFieldCount++;
				pDesc++;
			}
		}
		CommaIndex = NextCommaIndex;
	}

	return PROTO_OK;
}

int NEO6M_GetField(NMEA_FieldDescriptor *desc, char *pOutBuffer)
{
	if(!desc || !pOutBuffer || !desc->Ptr || !desc->len) return PROTO_FAIL;

	int n = desc->len;
	char *pcChrPtr = desc->Ptr;
	while(n--)
	{
		*pOutBuffer++ = *pcChrPtr++;
	}
	*pOutBuffer = '\0';

	return PROTO_OK;
}

int NEO6M_setPortConfig(eNeo6mBaudRate baudrate, eNeo6mProtocol InProtocol, eNeo6mProtocol OutProtocol)
{
	// Encode
	uint8_t UbxPRTPayload[] = {
		0x01,             					// PortID: 1 (UART1)
		0x00,             					// Reserved
		0x00, 0x00,       					// TX Ready (disabled)
		0xD0, 0x08, 0x00, 0x00, 			// mode: 8N1, no parity, 1 stop (0x08D0)
		0x80, 0x25, 0x00, 0x00, 			// baudRate: 9600 (0x2580)
		0x03, 0x00,       					// inProtoMask: UBX (bit0) + NMEA (bit1)
		0x03, 0x00,       					// outProtoMask: UBX + NMEA
		0x00, 0x00,       					// flags
		0x00, 0x00,       					// reserved
	};

	// BaudRate
	UbxPRTPayload[8] = (uint8_t)((uint32_t)baudrate & 0x000000FF);
	UbxPRTPayload[9] = (uint8_t)(((uint32_t)baudrate & 0x0000FF00)>>8);
	UbxPRTPayload[10] = (uint8_t)(((uint32_t)baudrate & 0x00FF0000)>>16);
	UbxPRTPayload[11] = (uint8_t)(((uint32_t)baudrate & 0xFF000000)>>24);

	// inProtoMask
	UbxPRTPayload[12] = (uint8_t)InProtocol;

	// outProtoMask
	UbxPRTPayload[14] = (uint8_t)OutProtocol;

	// Enable UBX protocol
	if(((uint8_t)OutProtocol & NEO6M_PROTOCOL_UBX))
	{
		hProtocol.Protocols[PROTOCOL_ID_UBX].Status |= PROTOCOL_PEN_FLAG;
		hProtocol.Protocols[PROTOCOL_ID_UBX].Status |= PROTOCOL_RXIE_FLAG;
	}

	// Check ack if enable NMEA
	if(InProtocol & NEO6M_PROTOCOL_NMEA) hProtocol.Protocols[PROTOCOL_ID_NMEA].Status |= PROTOCOL_PEN_FLAG;
	else hProtocol.Protocols[PROTOCOL_ID_NMEA].Status &= ~PROTOCOL_PEN_FLAG;

	// Write
	return NEO6M_Write(UBX_CFG, UBX_CFG_PRT, UbxPRTPayload, UBX_CFG_PRT_LEN);
}

int NEO6M_setSampleRate(eNeo6mSampleRate sr)
{
	//B5 62 06 08 06 00 C8 00 01 00 00 00 CK_A CK_B -> 200 ms, navRate = 1, UTC
	// Encode
	uint8_t UbxRatePayload[] =
	{
		0xE8,0x03,	// DataRate=1000ms
		0x01,0x00,	// NavRate=1
		0x00,0x00	// TimeRef=UTC
	};

	// Sample Rate
	UbxRatePayload[0] = ((uint16_t)(sr) & 0x00FF);
	UbxRatePayload[1] = (((uint16_t)(sr) & 0xFF00) >> 8);

	// Write
	return NEO6M_Write(UBX_CFG, UBX_CFG_RATE, UbxRatePayload, UBX_CFG_RATE_LEN);
}

int NEO6M_setMessage(eUbxMessageID eNmeaMsgID, uint8_t msgRate)
{
	// Assert ID
	if(!(eNmeaMsgID>=UBX_NMEA_GGA && eNmeaMsgID<=UBX_NMEA_VTG)) return PROTO_FAIL;

	//0xB5 0x62 0x06 0x01 0x0300 msgClass msgID rate CK_A CK_B
	// Encode
	uint8_t UbxMsgPayload[] =
	{
		UBX_NMEA,		// Class=NMEA
		UBX_NMEA_GGA,	// ID=GGA
		0x01,			// Rate=1 (1 message every Rate*SamplingRate)
	};

	// Fill msgID
	UbxMsgPayload[1] = (uint8_t)(eNmeaMsgID);
	// Rate
	UbxMsgPayload[2] = msgRate ? 0x01 : 0x00;

	// Write
	return NEO6M_Write(UBX_CFG, UBX_CFG_MSG, UbxMsgPayload, UBX_CFG_MSG_LEN);
}






/* Private Function Reference -------------------------------------*/
int NEO6M_Config(Neo6mConfig_TypeDef *cfg)
{
	int retVal = PROTO_OK;
	if(cfg)
	{
		/* BaudRate & Protocols */
		int retVal1 = NEO6M_setPortConfig(cfg->BaudRate, cfg->InProtocol, cfg->OutProtocol);

		/* Message Enable/Disable */
		int retVal2 = NEO6M_setMessage(UBX_NMEA_GGA, cfg->Message.GGA);
		retVal2 = NEO6M_setMessage(UBX_NMEA_GLL, cfg->Message.GLL);
		retVal2 = NEO6M_setMessage(UBX_NMEA_GSA, cfg->Message.GSA);
		retVal2 = NEO6M_setMessage(UBX_NMEA_GSV, cfg->Message.GSV);
		retVal2 = NEO6M_setMessage(UBX_NMEA_RMC, cfg->Message.RMC);
		retVal2 = NEO6M_setMessage(UBX_NMEA_VTG, cfg->Message.VTG);

		/* Sample Rate */
		int retVal3 = NEO6M_setSampleRate(cfg->SampleRate);

		retVal = (retVal1 > retVal2 ? retVal1 : retVal2);
		retVal = (retVal > retVal3 ? retVal : retVal3);
	}

	return retVal;
}

static eUbxMessageID NEO6M_GetID(const char *pcIDStr)
{
	eUbxMessageID xMsgID = UBX_ID_UNK;
	uint8_t ucState = 0;
	/*
	 * GGA
	 * GLL
	 * GSA
	 * GSV
	 * RMC
	 * VTG
	 * TXT
	 */
	while(ucState != 3 && xMsgID==UBX_ID_UNK)
	{
		switch(ucState)
		{
			case 0:
				if(*pcIDStr=='G') ucState++;
				else if(*pcIDStr=='R') xMsgID = UBX_NMEA_RMC;
				else if(*pcIDStr=='V') xMsgID = UBX_NMEA_VTG;
				else if(*pcIDStr=='T') xMsgID = UBX_NMEA_TXT;
				else ucState = 3;
			break;

			case 1:
				if(*pcIDStr=='G') xMsgID = UBX_NMEA_GGA;
				else if(*pcIDStr=='L') xMsgID = UBX_NMEA_GLL;
				else if(*pcIDStr=='S') ucState++;
				else ucState = 3;
			break;

			case 2:
				if(*pcIDStr=='A') xMsgID = UBX_NMEA_GSA;
				else if(*pcIDStr=='V') xMsgID = UBX_NMEA_GSV;
				else ucState = 3;
			break;
		}
		pcIDStr++;
	}

	return xMsgID;
}