#ifndef __UBX_UART_PROTO_H
#define __UBX_UART_PROTO_H
/* Exported Includes -----------------------------------------------*/
#include <UartProtoBase.h>

/* Exported Defines ------------------------------------------------*/
#define configUBX_MAX_PAYLOAD_SIZE (20)
/* Exported Macros -------------------------------------------------*/

/* Exported Typedefs -----------------------------------------------*/
typedef struct
{
	uint8_t class;
	uint8_t id;
	uint16_t len;
	uint8_t payload[configUBX_MAX_PAYLOAD_SIZE];
	uint8_t checksum[2];
} UbxFrame_TypeDef;

typedef enum
{
	UBX_NAV  = 0x01,  // Navigation results
	UBX_RXM  = 0x02,  // Receiver Manager messages
	UBX_INF  = 0x04,  // Informational messages
	UBX_ACK  = 0x05,  // ACK/NAK replies to CFG messages
	UBX_CFG  = 0x06,  // Configuration input messages
	UBX_MON  = 0x0A,  // Monitoring messages
	UBX_AID  = 0x0B,  // Assist Now aiding messages
	UBX_TIM  = 0x0D,  // Timing messages
	UBX_HNR  = 0x28,  // High rate navigation results
	UBX_NMEA = 0xF0,  // NMEA Standard messages
	UBX_PUBX = 0xF1,  // NMEA proprietary messages (PUBX)
	UBX_UNK  = 0xFF
}eUbxMessageClass;

typedef enum
{
	UBX_ACK_NAK      = 0x00, // Reply to CFG messages
	UBX_ACK_ACK      = 0x01, // Reply to CFG messages
	UBX_CFG_MSG      = 0x01, // Configure which messages to send
	UBX_CFG_RST      = 0x04, // Reset command
	UBX_CFG_PRT		 = 0x00, // Port command
	UBX_CFG_RATE     = 0x08, // Configure message rate
	UBX_CFG_NMEA     = 0x17, // Configure NMEA protocol
	UBX_CFG_NAV5     = 0x24, // Configure navigation engine settings
	UBX_NMEA_GGA 	 = 0x00,
	UBX_NMEA_GLL 	 = 0x01,
	UBX_NMEA_GSA 	 = 0x02,
	UBX_NMEA_GSV 	 = 0x03,
	UBX_NMEA_RMC 	 = 0x04,
	UBX_NMEA_VTG 	 = 0x05,
	UBX_NMEA_TXT 	 = 0x06, // Not actually def
	UBX_MON_VER      = 0x04, // Monitor Receiver/Software version
	UBX_NAV_POSLLH   = 0x02, // Current Position
	UBX_NAV_STATUS   = 0x03, // Receiver Navigation Status
	UBX_NAV_DOP      = 0x04, // Dilutions of Precision
	UBX_NAV_ODO      = 0x09, // Odometer Solution (NEO-M8 only)
	UBX_NAV_PVT      = 0x07, // Position, Velocity and Time
	UBX_NAV_RESETODO = 0x10, // Reset Odometer (NEO-M8 only)
	UBX_NAV_VELNED   = 0x12, // Current Velocity
	UBX_NAV_TIMEGPS  = 0x20, // Current GPS Time
	UBX_NAV_TIMEUTC  = 0x21, // Current UTC Time
	UBX_NAV_SVINFO   = 0x30, // Space Vehicle Information
	UBX_HNR_PVT      = 0x00, // High rate Position, Velocity and Time
	UBX_ID_UNK   	 = 0xFF
} eUbxMessageID;

/* Exported Constants ----------------------------------------------*/

/* Exported Variables ----------------------------------------------*/

/* Exported Function Prototype -------------------------------------*/
void UBX_Init(Protocol_TypeDef *pxInstance);
int UBX_Send(UbxFrame_TypeDef *pxUbxFrame);
int UBX_Recv(Protocol_TypeDef *proto, UbxFrame_TypeDef **pxUbxFrame);
int UBX_Available(Protocol_TypeDef *proto);
int UBX_WaitAck(Protocol_TypeDef *proto, uint8_t msgClass, uint8_t msgID, uint32_t ulTimeout);
void UBX_ComputeChecksum(UbxFrame_TypeDef *pxUbxFrame);

/* Exported user code ----------------------------------------------*/

/* Exported reference function ------------------------------------*/

/* Exported Function Reference -------------------------------------*/

#endif /* __UBX_UART_PROTO_H */
