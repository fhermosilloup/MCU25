/*
 * mpu6050.c
 *
 *  Created on: Feb 19, 2016
 *  Author: Fernando Hermosillo
 */

#include "mpu6050.h"

/* Default I2C address */
#define MPU6050_I2C_ADDR			0xD0

/* Who I am register value */
#define MPU6050_DEVID				0x68

/* MPU6050 registers */
#define MPU6050_AUX_VDDIO			0x01
#define MPU6050_SMPLRT_DIV			0x19
#define MPU6050_CONFIG				0x1A
#define MPU6050_GYRO_CONFIG			0x1B
#define MPU6050_ACCEL_CONFIG		0x1C
#define MPU6050_MOTION_THRESH		0x1F
#define MPU6050_FIFO_EN				0x23
#define MPU6050_INT_PIN_CFG			0x37
#define MPU6050_INT_ENABLE			0x38
#define MPU6050_INT_STATUS			0x3A
#define MPU6050_ACCEL_XOUT_H		0x3B
#define MPU6050_ACCEL_XOUT_L		0x3C
#define MPU6050_ACCEL_YOUT_H		0x3D
#define MPU6050_ACCEL_YOUT_L		0x3E
#define MPU6050_ACCEL_ZOUT_H		0x3F
#define MPU6050_ACCEL_ZOUT_L		0x40
#define MPU6050_TEMP_OUT_H			0x41
#define MPU6050_TEMP_OUT_L			0x42
#define MPU6050_GYRO_XOUT_H			0x43
#define MPU6050_GYRO_XOUT_L			0x44
#define MPU6050_GYRO_YOUT_H			0x45
#define MPU6050_GYRO_YOUT_L			0x46
#define MPU6050_GYRO_ZOUT_H			0x47
#define MPU6050_GYRO_ZOUT_L			0x48
#define MPU6050_MOT_DETECT_STATUS	0x61
#define MPU6050_SIGNAL_PATH_RESET	0x68
#define MPU6050_MOT_DETECT_CTRL		0x69
#define MPU6050_USER_CTRL			0x6A
#define MPU6050_PWR_MGMT_1			0x6B
#define MPU6050_PWR_MGMT_2			0x6C
#define MPU6050_FIFO_COUNTH			0x72
#define MPU6050_FIFO_COUNTL			0x73
#define MPU6050_FIFO_R_W			0x74
#define MPU6050_WHO_AM_I			0x75

/* MPU6050_USER_CTRL_REG */
#define MPU6050_USERCTRL_FIFO_EN	0x40
#define MPU6050_USERCTRL_FIFO_RESET	0x04
/* Gyro sensitivities in degrees/s */
#define MPU6050_GYRO_SENS_250		((float) 131)
#define MPU6050_GYRO_SENS_500		((float) 65.5)
#define MPU6050_GYRO_SENS_1000		((float) 32.8)
#define MPU6050_GYRO_SENS_2000		((float) 16.4)

/* Acce sensitivities in g/s */
#define MPU6050_ACCE_SENS_2			((float) 16384)
#define MPU6050_ACCE_SENS_4			((float) 8192)
#define MPU6050_ACCE_SENS_8			((float) 4096)
#define MPU6050_ACCE_SENS_16		((float) 2048)


/* Private function reference ---------------------------------------------------------*/
// Write to register
static HAL_StatusTypeDef MPU6050_IO_Write(MPU6050_HandleTypeDef *hMpu6050Dev, uint8_t ucRegAddr, uint8_t ucByteToWrite)
{
	uint8_t ucBuffer[2] = {ucRegAddr, ucByteToWrite};

	return HAL_I2C_Master_Transmit(hMpu6050Dev->hi2c, hMpu6050Dev->DeviceAddress, ucBuffer, 2, 100);
}
// Read from register
static HAL_StatusTypeDef MPU6050_IO_Read(MPU6050_HandleTypeDef *hMpu6050Dev, uint8_t ucRegAddr, uint8_t *pBufferOut, uint16_t uBytesToRead)
{
	HAL_I2C_Master_Transmit(hMpu6050Dev->hi2c, hMpu6050Dev->DeviceAddress, &ucRegAddr, 1, 100);
	return HAL_I2C_Master_Receive(hMpu6050Dev->hi2c, hMpu6050Dev->DeviceAddress,  pBufferOut, uBytesToRead, 100);
}

static MPU6050_Error MPU6050_Reset(MPU6050_HandleTypeDef *hMpu6050Dev)
{
	if(MPU6050_IO_Write(hMpu6050Dev, MPU6050_PWR_MGMT_1, 0x80) != HAL_OK ) return MPU6050_FAIL;
	uint8_t RegVal = 0x80;
	while( RegVal & 0x80 )
	{
		if(MPU6050_IO_Read(hMpu6050Dev, MPU6050_PWR_MGMT_1, &RegVal, 1) != HAL_OK ) return MPU6050_FAIL;
	}

	return MPU6050_OK;
}



/* Expoted Reference Function --------------------------------------------------*/
MPU6050_Error MPU6050_Init(MPU6050_HandleTypeDef *hMpu6050Dev, I2C_HandleTypeDef* hi2c, MPU6050_Device DeviceNumber)
{
	hMpu6050Dev->hi2c = hi2c;

	/* Format I2C address */
	hMpu6050Dev->DeviceAddress = MPU6050_I2C_ADDR | (uint8_t)DeviceNumber;

	/* Check if device is connected */
	if(HAL_I2C_IsDeviceReady(hMpu6050Dev->hi2c, hMpu6050Dev->DeviceAddress,2,5)!=HAL_OK)
	{
				return MPU6050_ERROR_NOT_FOUND;
	}

	/* Check who am I */
	uint8_t temp = 0x00;
	if(MPU6050_IO_Read(hMpu6050Dev, MPU6050_WHO_AM_I, &temp, 1) != HAL_OK ) return MPU6050_FAIL;
	if(temp != MPU6050_DEVID) return MPU6050_ERROR_INVALID;

	/* Reset all registers to default */
	MPU6050_Reset(hMpu6050Dev);

	/* Set Clock source */
	MPU6050_SetClockSource(hMpu6050Dev, MPU6050_CLOCK_PLL_ZGYRO);

	/* Set sample rate to 1kHz */
	MPU6050_SetDataRate(hMpu6050Dev, MPU6050_DATARATE_1KHZ);

	/* Config accelerometer */
	MPU6050_SetAccelerometerRange(hMpu6050Dev, MPU6050_ACCEL_RANGE_2G);

	/* Config Gyroscope */
	MPU6050_SetGyroscopeRange(hMpu6050Dev, MPU6050_GYRO_RANGE_250DPS);

	/* Disable Interrupts */
	MPU6050_IntPin_Config IntPinCfg = {0};
	MPU6050_SetInterrupts(hMpu6050Dev, 0x00, IntPinCfg);

	/* Disable FIFO */
	MPU6050_SetFifo(hMpu6050Dev, MPU6050_FIFO_DISABLE, 0x00);

	/* Wakeup MPU6050 */
	if(MPU6050_IO_Write(hMpu6050Dev, MPU6050_PWR_MGMT_1, 0x00) != HAL_OK ) return MPU6050_FAIL;

	/* Return OK */
	return MPU6050_OK;
}

MPU6050_Error MPU6050_SetDataRate(MPU6050_HandleTypeDef *hMpu6050Dev, MPU6050_DataRate rate)
{
	/* Set data sample rate */
	if(MPU6050_IO_Write(hMpu6050Dev, MPU6050_SMPLRT_DIV, (uint8_t)rate) != HAL_OK ) return MPU6050_FAIL;

	return MPU6050_OK;
}

MPU6050_Error MPU6050_SetClockSource(MPU6050_HandleTypeDef *hMpu6050Dev, MPU6050_ClockSource clksrc)
{
	/* Set clock source */
	uint8_t regval = 0x00;
	if(MPU6050_IO_Read(hMpu6050Dev, MPU6050_PWR_MGMT_1, &regval, 1) != HAL_OK ) return MPU6050_FAIL;
	regval &= ~0x07;
	if(MPU6050_IO_Write(hMpu6050Dev, MPU6050_PWR_MGMT_1, regval | (uint8_t)clksrc) != HAL_OK ) return MPU6050_FAIL;

	return MPU6050_OK;
}

MPU6050_Error MPU6050_SetAccelerometerRange(MPU6050_HandleTypeDef *hMpu6050Dev, MPU6050_AccelRange AccelSensitivity)
{
	uint8_t temp;

	/* Config accelerometer */
	if(MPU6050_IO_Read(hMpu6050Dev, MPU6050_ACCEL_CONFIG, &temp, 1) != HAL_OK ) return MPU6050_FAIL;
	temp = (temp & 0xE7) | (uint8_t)AccelSensitivity << 3;
	if(MPU6050_IO_Write(hMpu6050Dev, MPU6050_ACCEL_CONFIG, temp) != HAL_OK ) return MPU6050_FAIL;

	/* Set sensitivities for multiplying gyro and accelerometer data */
	switch (AccelSensitivity) {
		case MPU6050_ACCEL_RANGE_2G:
			hMpu6050Dev->AcceGain = (float)1 / MPU6050_ACCE_SENS_2;
			break;
		case MPU6050_ACCEL_RANGE_4G:
			hMpu6050Dev->AcceGain = (float)1 / MPU6050_ACCE_SENS_4;
			break;
		case MPU6050_ACCEL_RANGE_8G:
			hMpu6050Dev->AcceGain = (float)1 / MPU6050_ACCE_SENS_8;
			break;
		case MPU6050_ACCEL_RANGE_16G:
			hMpu6050Dev->AcceGain = (float)1 / MPU6050_ACCE_SENS_16;
			break;
		default:
			break;
		}

	/* Return OK */
	return MPU6050_OK;
}

MPU6050_Error MPU6050_SetGyroscopeRange(MPU6050_HandleTypeDef *hMpu6050Dev, MPU6050_GyroRange GyroSensitivity)
{
	uint8_t temp;

	/* Config gyroscope */
	if(MPU6050_IO_Read(hMpu6050Dev, MPU6050_GYRO_CONFIG, &temp, 1) != HAL_OK ) return MPU6050_FAIL;
	temp = (temp & 0xE7) | (uint8_t)GyroSensitivity << 3;
	if(MPU6050_IO_Write(hMpu6050Dev, MPU6050_GYRO_CONFIG, temp) != HAL_OK ) return MPU6050_FAIL;

	switch (GyroSensitivity) {
			case MPU6050_GYRO_RANGE_250DPS:
				hMpu6050Dev->GyroGain = (float)1 / MPU6050_GYRO_SENS_250;
				break;
			case MPU6050_GYRO_RANGE_500DPS:
				hMpu6050Dev->GyroGain = (float)1 / MPU6050_GYRO_SENS_500;
				break;
			case MPU6050_GYRO_RANGE_1000DPS:
				hMpu6050Dev->GyroGain = (float)1 / MPU6050_GYRO_SENS_1000;
				break;
			case MPU6050_GYRO_RANGE_2000DPS:
				hMpu6050Dev->GyroGain = (float)1 / MPU6050_GYRO_SENS_2000;
				break;
			default:
				break;
		}
	/* Return OK */
	return MPU6050_OK;
}

MPU6050_Error MPU6050_ReadAccelerometer(MPU6050_HandleTypeDef *hMpu6050Dev)
{
	/* Read accelerometer data */
	if(MPU6050_IO_Read(hMpu6050Dev, MPU6050_ACCEL_XOUT_H, (uint8_t *)&hMpu6050Dev->Data.Accelerometer, 6) != HAL_OK ) return MPU6050_FAIL;

	/* Return OK */
	return MPU6050_OK;
}

MPU6050_Error MPU6050_ReadGyroscope(MPU6050_HandleTypeDef *hMpu6050Dev)
{
	/* Read gyroscope data */
	if(MPU6050_IO_Read(hMpu6050Dev, MPU6050_GYRO_XOUT_H, (uint8_t *)&hMpu6050Dev->Data.Gyroscope, 6) != HAL_OK ) return MPU6050_FAIL;

	/* Return OK */
	return MPU6050_OK;
}

MPU6050_Error MPU6050_ReadTemperature(MPU6050_HandleTypeDef *hMpu6050Dev)
{
	/* Read temperature */
	if(MPU6050_IO_Read(hMpu6050Dev, MPU6050_TEMP_OUT_H, (uint8_t *)&hMpu6050Dev->Data.Temperature, 2) != HAL_OK ) return MPU6050_FAIL;

	/* Format temperature */
	//hMpu6050Dev->Temperature = (float)((int16_t)temp / (float)340.0 + (float)36.53);

	/* Return OK */
	return MPU6050_OK;
}

MPU6050_Error MPU6050_ReadAll(MPU6050_HandleTypeDef *hMpu6050Dev)
{
	/* Read full raw data, 14bytes */
	if(MPU6050_IO_Read(hMpu6050Dev, MPU6050_ACCEL_XOUT_H, (uint8_t *)&hMpu6050Dev->Data, 14) != HAL_OK ) return MPU6050_FAIL;

	/* Return OK */
	return MPU6050_OK;
}
MPU6050_Error MPU6050_SetInterrupts(MPU6050_HandleTypeDef *hMpu6050Dev, uint8_t IntConfig, MPU6050_IntPin_Config PinConfig)
{
	/* Enable interrupts for data ready and motion detect */
	// IntConfig = 0x21
	if(MPU6050_IO_Write(hMpu6050Dev, MPU6050_INT_ENABLE, IntConfig & 0xF0) != HAL_OK ) return MPU6050_FAIL;

	/* Clear IRQ flag on any read operation */
	uint8_t temp = 0x00;
	temp |= (uint8_t)PinConfig.Level << 7;
	temp |= (uint8_t)PinConfig.PioMode << 6;
	temp |= (uint8_t)PinConfig.LatchMode << 5;
	temp |= (uint8_t)PinConfig.RdMode << 4;
	if(MPU6050_IO_Write(hMpu6050Dev, MPU6050_INT_PIN_CFG, temp) != HAL_OK ) return MPU6050_FAIL;

	/* Return OK */
	return MPU6050_OK;
}

MPU6050_Error MPU6050_GetStatus(MPU6050_HandleTypeDef *hMpu6050Dev, MPU6050_Interrupt* IntStatus)
{
	/* Reset structure */
	IntStatus->Status = 0x00;

	/* Read Interrupt Status Register */
	if(MPU6050_IO_Read(hMpu6050Dev, MPU6050_INT_STATUS, &IntStatus->Status, 1) != HAL_OK ) return MPU6050_FAIL;

	/* Return OK */
	return MPU6050_OK;
}

MPU6050_Error MPU6050_SetFifo(MPU6050_HandleTypeDef *hMpu6050Dev, MPU6050_Fifo FifoConfig, uint8_t SensFifoEn)
{
	/* TEMP_FIFO_EN	XG_FIFO_EN	YG_FIFO_EN	ZG_FIFO_EN	ACCEL_FIFO_EN	SLV2_FIFO_EN	SLV1_FIFO_EN	SLV0_FIFO_EN*/
	if(MPU6050_IO_Write(hMpu6050Dev, MPU6050_FIFO_EN, SensFifoEn) != HAL_OK ) return MPU6050_FAIL;

	/* Disable Fifo */
	uint8_t RegVal = 0x00;
	if(MPU6050_IO_Read(hMpu6050Dev, MPU6050_USER_CTRL, &RegVal, 1) != HAL_OK ) return MPU6050_FAIL;
	RegVal &= ~MPU6050_USERCTRL_FIFO_EN;
	if(MPU6050_IO_Write(hMpu6050Dev, MPU6050_USER_CTRL, RegVal) != HAL_OK ) return MPU6050_FAIL;

	/* Reset FIFO */
	if(MPU6050_IO_Write(hMpu6050Dev, MPU6050_USER_CTRL, RegVal | MPU6050_USERCTRL_FIFO_RESET) != HAL_OK ) return MPU6050_FAIL;

	/* Enable FIFO */
	if(FifoConfig == MPU6050_FIFO_ENABLE)
	{
		RegVal |= (uint8_t)MPU6050_FIFO_ENABLE;
		if(MPU6050_IO_Write(hMpu6050Dev, MPU6050_USER_CTRL, RegVal) != HAL_OK ) return MPU6050_FAIL;
	}

	return MPU6050_OK;
}

MPU6050_Error MPU6050_GetFifoCount(MPU6050_HandleTypeDef *hMpu6050Dev, uint16_t *pNumBytes)
{
	if(MPU6050_IO_Read(hMpu6050Dev, MPU6050_FIFO_COUNTH, (uint8_t *)pNumBytes, 2) != HAL_OK ) return MPU6050_FAIL;

	return MPU6050_OK;
}

MPU6050_Error MPU6050_ReadFifo(MPU6050_HandleTypeDef *hMpu6050Dev, uint8_t *pData, uint16_t BytesToRead)
{
	uint16_t FifoLen = 0;

	if(MPU6050_GetFifoCount(hMpu6050Dev, &FifoLen) != MPU6050_OK ) return MPU6050_FAIL;

	if(BytesToRead > FifoLen) return MPU6050_FAIL;
	while(BytesToRead--)
	{
		if(MPU6050_IO_Read(hMpu6050Dev, MPU6050_FIFO_R_W, pData++, 1) != HAL_OK ) return MPU6050_FAIL;
	}

	return MPU6050_OK;
}
