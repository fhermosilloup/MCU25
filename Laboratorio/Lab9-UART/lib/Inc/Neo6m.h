/*
 * Neo6M.h
 *
 *  Created on: Apr 16, 2025
 *      Author: User123
 */

#ifndef INC_NEO6M_H_
#define INC_NEO6M_H_

#include "NmeaUartProto.h"
#include "UbxUartProto.h"

/* Configurations */
#define configNEO6M_BAUDRATE_DEFAULT	(9600)
#define configNEO6M_STARTUP_TIMEOUT		(10000)	/* 10 Minutes */

/* Errors */
#define NEO6M_ERROR_STARTUP_FAIL	(7)


/* Exported Typedefs -----------------------------------------------------------*/
typedef enum
{
	NEO6M_BAUD_4800 = 4800,
	NEO6M_BAUD_9600 = 9600,
	NEO6M_BAUD_19200 = 19200,
	NEO6M_BAUD_38400 = 38400,
	NEO6M_BAUD_57600 = 57600,
	NEO6M_BAUD_115200 = 115200
} eNeo6mBaudRate;

typedef enum
{
	NEO6M_PROTOCOL_UBX	= 0x01,
	NEO6M_PROTOCOL_NMEA = 0x02,
	NEO6M_PROTOCOL_BOTH = 0x03
} eNeo6mProtocol;

typedef enum
{
	NEO6M_RATE_1HZ = 1000,
	NEO6M_RATE_2HZ = 500,
	NEO6M_RATE_3HZ = 333,
	NEO6M_RATE_4HZ = 250,
	NEO6M_RATE_5HZ = 200
} eNeo6mSampleRate;

typedef struct
{
	eNeo6mProtocol InProtocol;
	eNeo6mProtocol OutProtocol;
	eNeo6mSampleRate SampleRate;
	eNeo6mBaudRate BaudRate;
	union {
	    struct {
	        uint8_t GGA : 1;
	        uint8_t GLL : 1;
	        uint8_t GSA : 1;
	        uint8_t GSV : 1;
	        uint8_t RMC : 1;
	        uint8_t VTG : 1;
	        uint8_t     : 2;  // Unused bits
	    };
	    uint8_t Value;
	} Message;
} Neo6mConfig_TypeDef;

/* Public */
void NEO6M_Init(Neo6mConfig_TypeDef *cfg);
int NEO6M_Start(void);

int NEO6M_Read(NMEA_MessageDescriptor *desc, const char *pcMessageID, uint32_t timeout);
int NEO6M_ParseMessage(NMEA_MessageDescriptor *pMessage, NMEA_FieldDescriptor *desc, uint8_t num_desc);
int NEO6M_GetField(NMEA_FieldDescriptor *desc, char *pOutBuffer);
int NEO6M_Write(eUbxMessageClass msgClass, eUbxMessageID msgID, uint8_t *payload, uint16_t len);

int NEO6M_setPortConfig(eNeo6mBaudRate baudrate, eNeo6mProtocol InProtocol, eNeo6mProtocol OutProtocol);
int NEO6M_setMessage(eUbxMessageID eNmeaMsgID, uint8_t msgRate);
int NEO6M_setSampleRate(eNeo6mSampleRate sr);

//void NEO6M_ToGoogle(char *link);

#endif /* INC_NEO6M_H_ */
