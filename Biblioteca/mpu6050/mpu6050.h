/*
 * mpu6050.h
 *
 *  Created on: Feb 19, 2016
 *  Author: Fernando Hermosillo
 *
 */

#ifndef HAL_MPU6050_H_
#define HAL_MPU6050_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_i2c.h"

/**
 * @defgroup MPU6050_Macros
 * @brief    Library defines
 * @{
 */

/* MPU6050 Register 35 – FIFO Enable bit field definiton */
#define MPU6050_TEMP_FIFO_EN	0x80
#define MPU6050_XG_FIFO_EN		0x40
#define MPU6050_YG_FIFO_EN		0x20
#define MPU6050_ZG_FIFO_EN		0x10
#define MPU6050_ACCEL_FIFO_EN	0x08
#define MPU6050_SLV2_FIFO_EN	0x04
#define MPU6050_SLV1_FIFO_EN	0x02
#define MPU6050_SLV0_FIFO_EN	0x01

/* Register 56 – Interrupt Enable bit field definiton */
#define MPU6050_INT_MOT_EN		0x40
#define MPU6050_INT_FIFO_OVF_EN	0x10
#define MPU6050_INT_I2C_MST_EN	0x04
#define MPU6050_INT_DRDY_EN		0x01

/* Register 56 – Interrupt Status bit field definiton */
#define MPU6050_MOT_IF		0x40
#define MPU6050_FIFO_OVF_IF	0x10
#define MPU6050_I2C_MST_IF	0x04
#define MPU6050_DRDY_IF		0x01


/**
 * @}
 */

/**
 * @}
 */

/**
 * @defgroup MPU6050_Typedefs
 * @brief    Library Typedefs
 * @{
 */

/**
 * @brief  Data rates predefined constants
 * @{
 */
typedef enum  {
	MPU6050_DATARATE_8KHZ = 0x00,	/*!< Sample rate set to 8 kHz */
	MPU6050_DATARATE_4KHZ = 0x01,	/*!< Sample rate set to 4 kHz */
	MPU6050_DATARATE_2KHZ = 0x03,	/*!< Sample rate set to 2 kHz */
	MPU6050_DATARATE_1KHZ = 0x07,	/*!< Sample rate set to 1 kHz */
	MPU6050_DATARATE_500HZ = 0x0F,	/*!< Sample rate set to 500 Hz */
	MPU6050_DATARATE_250HZ = 0x1F,	/*!< Sample rate set to 250 Hz */
	MPU6050_DATARATE_125HZ = 0x3F,	/*!< Sample rate set to 125 Hz */
	MPU6050_DATARATE_100HZ = 0x4F	/*!< Sample rate set to 100 Hz */
} MPU6050_DataRate;

/**
 * @brief  MPU6050 can have 2 different slave addresses, depends on it's input AD0 pin
 *         This feature allows you to use 2 different sensors with this library at the same time
 */
typedef enum  {
	MPU6050_DEVICE_0 = 0x00, /*!< AD0 pin is set to low */
	MPU6050_DEVICE_1 = 0x02  /*!< AD0 pin is set to high */
} MPU6050_Device;

/**
 * @brief  MPU6050 result enumeration
 */
typedef enum  {
	MPU6050_OK = 0x00,          		/*!< Everything OK */
	MPU6050_FAIL,              			/*!< Unknown error */
	MPU6050_ERROR_NOT_FOUND, 	/*!< There is no device with valid slave address */
	MPU6050_ERROR_INVALID       	/*!< Connected device with address is not MPU6050 */
} MPU6050_Error;

/**
 * @brief  Parameters for accelerometer range
 */
typedef enum  {
	MPU6050_ACCEL_RANGE_2G = 0x00, /*!< Range is +- 2G */
	MPU6050_ACCEL_RANGE_4G = 0x01, /*!< Range is +- 4G */
	MPU6050_ACCEL_RANGE_8G = 0x02, /*!< Range is +- 8G */
	MPU6050_ACCEL_RANGE_16G = 0x03 /*!< Range is +- 16G */
} MPU6050_AccelRange;

/**
 * @brief  Parameters for gyroscope range
 */
typedef enum {
	MPU6050_GYRO_RANGE_250DPS = 0x00,  /*!< Range is +- 250 degrees/s */
	MPU6050_GYRO_RANGE_500DPS = 0x01,  /*!< Range is +- 500 degrees/s */
	MPU6050_GYRO_RANGE_1000DPS = 0x02, /*!< Range is +- 1000 degrees/s */
	MPU6050_GYRO_RANGE_2000DPS = 0x03  /*!< Range is +- 2000 degrees/s */
} MPU6050_GyroRange;

typedef enum {
	MPU6050_CLOCK_INTERNAL_8MHZ = 0x00,	/*!< Internal 8MHz (default) */
	MPU6050_CLOCK_PLL_XGYRO 	= 0x01, /*!< PLL with X-axis Gyroscope */
	MPU6050_CLOCK_PLL_YGYRO 	= 0x02, /*!< PLL with Y-axis Gyroscope */
	MPU6050_CLOCK_PLL_ZGYRO 	= 0x03,	/*!< PLL with Z-axis Gyroscope */
	MPU6050_CLOCK_ETERNAL_32KHz = 0x04,	/*!< PLL External 32.768KHz */
	MPU6050_CLOCK_ETERNAL_19MHz = 0x05,	/*!< PLL External 19.2MHz */
	MPU6050_CLOCK_RESET 		= 0x07	/*!< Reset */
} MPU6050_ClockSource;

typedef enum
{
	MPU6050_FIFO_DISABLE = 0x00,
	MPU6050_FIFO_ENABLE =  0x40
} MPU6050_Fifo;

typedef enum
{
	MPU6050_INTPIN_LEVEL_HIGH = 0x00,
	MPU6050_INTPIN_LEVEL_LOW,
} MPU6050_IntPin_Level;

typedef enum
{
	MPU6050_INTPIN_MODE_PP = 0x00,
	MPU6050_INTPIN_MODE_OD,
} MPU6050_IntPin_Mode;

typedef enum
{
	MPU6050_INTPIN_LATCH_50US = 0x00,
	MPU6050_INTPIN_LATCH_SW,
} MPU6050_IntPin_Latch;

typedef enum
{
	MPU6050_INTPIN_RD_REG = 0x00,
	MPU6050_INTPIN_RD_READ
} MPU6050_IntPin_RDClear;

typedef struct
{
	int16_t X;
	int16_t Y;
	int16_t Z;
} MPU6050_Point16;

typedef struct
{
	int32_t X;
	int32_t Y;
	int32_t Z;
} MPU6050_Point32;

typedef struct
{
	float X;
	float Y;
	float Z;
} MPU6050_PointF;

typedef struct
{
	MPU6050_Point16 Accelerometer;	/*!< Raw Accelerometer data */
	int16_t   Temperature;			/*!< Raw Temperature data */
	MPU6050_Point16 Gyroscope;		/*!< Raw Gyroscope data */
} MPU6050_Data;
/**
 * @brief  Main MPU6050 structure
 */
typedef struct  {
	/* Private */
	I2C_HandleTypeDef* hi2c;	/*!< I2C device. */
	uint8_t DeviceAddress;			/*!< I2C address of device. */
	float GyroGain;         	/*!< Gyroscope corrector from raw data to "degrees/s". Only for private use */
	float AcceGain;         	/*!< Accelerometer corrector from raw data to "g". Only for private use */

	/* Public */
	MPU6050_Data Data;
} MPU6050_HandleTypeDef;

/**
 * @brief  Interrupts union and structure
 */
typedef union {
	struct {
		uint8_t DataReady:1;       /*!< Data ready interrupt */
		uint8_t reserved2:2;       /*!< Reserved bits */
		uint8_t Master:1;          /*!< Master interrupt. Not enabled with library */
		uint8_t FifoOverflow:1;    /*!< FIFO overflow interrupt. Not enabled with library */
		uint8_t reserved1:1;       /*!< Reserved bit */
		uint8_t MotionDetection:1; /*!< Motion detected interrupt */
		uint8_t reserved0:1;       /*!< Reserved bit */
	} F;
	uint8_t Status;
} MPU6050_Interrupt;

typedef struct
{
	MPU6050_IntPin_Level Level;
	MPU6050_IntPin_Mode PioMode;
	MPU6050_IntPin_Latch LatchMode;
	MPU6050_IntPin_RDClear RdMode;
} MPU6050_IntPin_Config;

/**
 * @}
 */

/**
 * @defgroup MPU6050_Functions
 * @brief    Library Functions
 * @{
 */

/**
 * @brief  Initializes MPU6050 and I2C peripheral
 * @param  *hMPU6050: Pointer to empty @ref MPU6050_HandleTypeDef structure
 * @param  DeviceNumber: MPU6050 has one pin, AD0 which can be used to set address of device.
 *          This feature allows you to use 2 different sensors on the same board with same library.
 *          If you set AD0 pin to low, then this parameter should be MPU6050_Device_0,
 *          but if AD0 pin is high, then you should use MPU6050_Device_1
 *
 *          Parameter can be a value of @ref MPU6050_HandleTypeDef enumeration
 * @retval Initialization status:
 *            - MPU6050_Ok: Everything OK
 *            - Other member: in other cases
 */
MPU6050_Error MPU6050_Init(MPU6050_HandleTypeDef *hMpu6050Dev, I2C_HandleTypeDef* I2Cx, MPU6050_Device DeviceNumber);

/**
 * @brief  Sets gyroscope sensitivity
 * @param  *DataStruct: Pointer to @ref MPU6050_HandleTypeDef structure indicating MPU6050 device
 * @param  GyroscopeSensitivity: Gyro sensitivity value. This parameter can be a value of @ref MPU6050_GyroscopeRange enumeration
 * @retval Member of @ref MPU6050_Error enumeration
 */
MPU6050_Error MPU6050_SetGyroscopeRange(MPU6050_HandleTypeDef *hMpu6050Dev, MPU6050_GyroRange GyroSensitivity);

/**
 * @brief  Sets accelerometer sensitivity
 * @param  *DataStruct: Pointer to @ref MPU6050_t structure indicating MPU6050 device
 * @param  AccelerometerSensitivity: Gyro sensitivity value. This parameter can be a value of @ref MPU6050_Accelerometer_t enumeration
 * @retval Member of @ref MPU6050_Error enumeration
 */
MPU6050_Error MPU6050_SetAccelerometerRange(MPU6050_HandleTypeDef *hMpu6050Dev, MPU6050_AccelRange AccelSensitivity);

/**
 * @brief  Sets output data rate
 * @param  *DataStruct: Pointer to @ref MPU6050_t structure indicating MPU6050 device
 * @param  rate: Data rate value. An 8-bit value for prescaler value
 * @retval Member of @ref MPU6050_Error enumeration
 */
MPU6050_Error MPU6050_SetDataRate(MPU6050_HandleTypeDef *hMpu6050Dev, MPU6050_DataRate rate);


/**
 * @brief  Sets clock source
 * @param  *DataStruct: Pointer to @ref MPU6050_t structure indicating MPU6050 device
 * @param  clksrc: Clock source config.
 * @retval Member of @ref MPU6050_Error enumeration
 */
MPU6050_Error MPU6050_SetClockSource(MPU6050_HandleTypeDef *hMpu6050Dev, MPU6050_ClockSource clksrc);

/**
 * @brief  Reads accelerometer data from sensor
 * @param  *DataStruct: Pointer to @ref MPU6050_t structure to store data to
 * @retval Member of @ref MPU6050_Error:
 *            - MPU6050_Ok: everything is OK
 *            - Other: in other cases
 */
MPU6050_Error MPU6050_ReadAccelerometer(MPU6050_HandleTypeDef *hMpu6050Dev);

/**
 * @brief  Reads gyroscope data from sensor
 * @param  *DataStruct: Pointer to @ref MPU6050_t structure to store data to
 * @retval Member of @ref MPU6050_Result_t:
 *            - MPU6050_Result_Ok: everything is OK
 *            - Other: in other cases
 */
MPU6050_Error MPU6050_ReadGyroscope(MPU6050_HandleTypeDef *hMpu6050Dev);

/**
 * @brief  Reads temperature data from sensor
 * @param  *DataStruct: Pointer to @ref MPU6050_t structure to store data to
 * @retval Member of @ref MPU6050_Result_t:
 *            - MPU6050_Result_Ok: everything is OK
 *            - Other: in other cases
 */
MPU6050_Error MPU6050_ReadTemperature(MPU6050_HandleTypeDef *hMpu6050Dev);

/**
 * @brief  Reads accelerometer, gyroscope and temperature data from sensor
 * @param  *DataStruct: Pointer to @ref MPU6050_t structure to store data to
 * @retval Member of @ref MPU6050_Result_t:
 *            - MPU6050_Result_Ok: everything is OK
 *            - Other: in other cases
 */
MPU6050_Error MPU6050_ReadAll(MPU6050_HandleTypeDef *hMpu6050Dev);

/* Extended Features */
/**
 * @brief  Enables interrupts
 * @param  *DataStruct: Pointer to @ref MPU6050_t structure indicating MPU6050 device
 * @retval Member of @ref MPU6050_Result_t enumeration
 */
MPU6050_Error MPU6050_SetInterrupts(MPU6050_HandleTypeDef *hMpu6050Dev, uint8_t IntConfig, MPU6050_IntPin_Config PinConfig);

/**
 * @brief  Reads and clears interrupts
 * @param  *DataStruct: Pointer to @ref MPU6050_t structure indicating MPU6050 device
 * @param  *InterruptsStruct: Pointer to @ref MPU6050_Interrupt_t structure to store status in
 * @retval Member of @ref MPU6050_Error enumeration
 */
MPU6050_Error MPU6050_GetStatus(MPU6050_HandleTypeDef *hMpu6050Dev, MPU6050_Interrupt *IntStatus);

MPU6050_Error MPU6050_SetFifo(MPU6050_HandleTypeDef *hMpu6050Dev, MPU6050_Fifo FifoEn, uint8_t SensFifoEn);
MPU6050_Error MPU6050_GetFifoCount(MPU6050_HandleTypeDef *hMpu6050Dev, uint16_t *pNumBytes);
MPU6050_Error MPU6050_ReadFifo(MPU6050_HandleTypeDef *hMpu6050Dev, uint8_t *pData, uint16_t BytesToRead);

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* DRIVERS_MYLIB_SD_HAL_MPU6050_H_ */
