#ifndef __NMEA_UART_PROTO_H
#define __NMEA_UART_PROTO_H
/* Exported Includes -----------------------------------------------*/
#include <UartProtoBase.h>

/* Exported Defines ------------------------------------------------*/
#define configNMEA_MESSAGE_SIZE	(80)
#define NMEA_MAX_FIELD_LEN   		(12)

/* NMEA IDs encode both NMEA_ID and PAYLOAD_FIELD_POS */
// RMC NMEA IDs
#define NMEA_RMC_TIME_ID            ((uint8_t)UBX_NMEA_RMC + 0x01)
#define NMEA_RMC_STATUS_ID          ((uint8_t)UBX_NMEA_RMC + 0x02)
#define NMEA_RMC_LATITUDE_ID        ((uint8_t)UBX_NMEA_RMC + 0x03)
#define NMEA_RMC_LATITUDE_H_ID      ((uint8_t)UBX_NMEA_RMC + 0x04)
#define NMEA_RMC_LONGITUDE_ID       ((uint8_t)UBX_NMEA_RMC + 0x05)
#define NMEA_RMC_LONGITUDE_H_ID     ((uint8_t)UBX_NMEA_RMC + 0x06)
#define NMEA_RMC_SPEED_ID           ((uint8_t)UBX_NMEA_RMC + 0x07)
#define NMEA_RMC_TRACK_ANGLE_ID     ((uint8_t)UBX_NMEA_RMC + 0x08)
#define NMEA_RMC_DATE_ID            ((uint8_t)UBX_NMEA_RMC + 0x09)
#define NMEA_RMC_MAG_VAR_ID         ((uint8_t)UBX_NMEA_RMC + 0x0A)
#define NMEA_RMC_MAG_VAR_ORIENT_ID	((uint8_t)UBX_NMEA_RMC + 0x0B)
#define NMEA_RMC_MODE_ID            ((uint8_t)UBX_NMEA_RMC + 0x0C)

// GGA NMEA IDs
#define NMEA_GGA_TIME_ID                ((uint8_t)UBX_NMEA_GGA + 0x01)
#define NMEA_GGA_LATITUDE_ID            ((uint8_t)UBX_NMEA_GGA + 0x02)
#define NMEA_GGA_LATITUDE_H_ID          ((uint8_t)UBX_NMEA_GGA + 0x03)
#define NMEA_GGA_LONGITUDE_ID           ((uint8_t)UBX_NMEA_GGA + 0x04)
#define NMEA_GGA_LONGITUDE_H_ID         ((uint8_t)UBX_NMEA_GGA + 0x05)
#define NMEA_GGA_FIX_QUALITY_ID         ((uint8_t)UBX_NMEA_GGA + 0x06)
#define NMEA_GGA_NUM_SATS_ID            ((uint8_t)UBX_NMEA_GGA + 0x07)
#define NMEA_GGA_HDOP_ID                ((uint8_t)UBX_NMEA_GGA + 0x08)
#define NMEA_GGA_ALTITUDE_ID            ((uint8_t)UBX_NMEA_GGA + 0x09)
#define NMEA_GGA_ALTITUDE_UNIT_ID       ((uint8_t)UBX_NMEA_GGA + 0x0A)
#define NMEA_GGA_GEOID_SEP_ID           ((uint8_t)UBX_NMEA_GGA + 0x0B)
#define NMEA_GGA_GEOID_UNIT_ID          ((uint8_t)UBX_NMEA_GGA + 0x0C)
#define NMEA_GGA_AGE_DGPS_ID            ((uint8_t)UBX_NMEA_GGA + 0x0D)
#define NMEA_GGA_DGPS_ID_ID             ((uint8_t)UBX_NMEA_GGA + 0x0E)

// GSV NMEA IDs
#define NMEA_GSV_TOTAL_MSGS_ID          ((uint8_t)UBX_NMEA_GSV + 0x01)
#define NMEA_GSV_MSG_NUM_ID             ((uint8_t)UBX_NMEA_GSV + 0x02)
#define NMEA_GSV_SATS_IN_VIEW_ID        ((uint8_t)UBX_NMEA_GSV + 0x03)
// Satellite block 1
#define NMEA_GSV_SAT1_PRN_ID            ((uint8_t)UBX_NMEA_GSV + 0x04)
#define NMEA_GSV_SAT1_ELEVATION_ID      ((uint8_t)UBX_NMEA_GSV + 0x05)
#define NMEA_GSV_SAT1_AZIMUTH_ID        ((uint8_t)UBX_NMEA_GSV + 0x06)
#define NMEA_GSV_SAT1_SNR_ID            ((uint8_t)UBX_NMEA_GSV + 0x07)
// Satellite block 2
#define NMEA_GSV_SAT2_PRN_ID            ((uint8_t)UBX_NMEA_GSV + 0x08)
#define NMEA_GSV_SAT2_ELEVATION_ID      ((uint8_t)UBX_NMEA_GSV + 0x09)
#define NMEA_GSV_SAT2_AZIMUTH_ID        ((uint8_t)UBX_NMEA_GSV + 0x0A)
#define NMEA_GSV_SAT2_SNR_ID            ((uint8_t)UBX_NMEA_GSV + 0x0B)
// Satellite block 3
#define NMEA_GSV_SAT3_PRN_ID            ((uint8_t)UBX_NMEA_GSV + 0x0C)
#define NMEA_GSV_SAT3_ELEVATION_ID      ((uint8_t)UBX_NMEA_GSV + 0x0D)
#define NMEA_GSV_SAT3_AZIMUTH_ID        ((uint8_t)UBX_NMEA_GSV + 0x0E)
#define NMEA_GSV_SAT3_SNR_ID            ((uint8_t)UBX_NMEA_GSV + 0x0F)

/* Exported Macros -------------------------------------------------*/

/* Exported Typedefs -----------------------------------------------*/
typedef struct {
	char Buffer[configNMEA_MESSAGE_SIZE];
	uint8_t BufferLen;
	uint8_t MsgID;
} NMEA_MessageDescriptor;

typedef struct {
    uint8_t FieldID;	// Position in the NMEA message
    char *Ptr;			// Pointer to the buffer where the field start
    uint8_t len;		// Field length
} NMEA_FieldDescriptor;

/* Exported Constants ----------------------------------------------*/

/* Exported Variables ----------------------------------------------*/

/* Exported Function Prototype -------------------------------------*/
void NMEA_Init(Protocol_TypeDef *pxInstance);
int NMEA_Checksum(NMEA_MessageDescriptor *msg);
int NMEA_Recv(Protocol_TypeDef *proto, uint8_t *ByteToRead);
int NMEA_Available(Protocol_TypeDef *proto);

/* Exported user code ----------------------------------------------*/

/* Exported reference function ------------------------------------*/

/* Exported Function Reference -------------------------------------*/

#endif /* __NMEA_UART_PROTO_H */
