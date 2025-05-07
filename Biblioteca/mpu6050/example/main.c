/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mpu6050.h"
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
MPU6050_HandleTypeDef xSensor;

/*
 * Interrupt flag, if uNewSensorMeasure==1, a new measurement is ready
 * Driven in interrupt
 */
volatile uint8_t uNewSensorMeasure = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void MX_MPU6050_Config(void)
{
	//
	MPU6050_Init(&xSensor, &hi2c1, MPU6050_DEVICE_0);
	MPU6050_SetAccelerometerRange(&xSensor, MPU6050_ACCEL_RANGE_4G);
	MPU6050_SetGyroscopeRange(&xSensor, MPU6050_GYRO_RANGE_1000DPS);
	MPU6050_SetDataRate(&xSensor, MPU6050_DATARATE_1KHZ);

	// Interrupts
	MPU6050_IntPin_Config xIntPin;
	xIntPin.Level = MPU6050_INTPIN_LEVEL_HIGH;		// INT pin is active on HIGH, i.e., INTPIN=LOW no interrupt occurred
	xIntPin.LatchMode = MPU6050_INTPIN_LATCH_50US;	// A 50us pulse is emitted on INT pin when an interrupt occurs
	xIntPin.PioMode = MPU6050_INTPIN_MODE_PP;		// INT pin is Push-Pull
	xIntPin.RdMode = MPU6050_INTPIN_RD_READ;		// Interrupt is clear on any data read
	MPU6050_SetInterrupts(&xSensor, MPU6050_INT_DRDY_EN, xIntPin);
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void EXTI0_IRQHandler(void)
{
	if(EXTI->PR & EXTI_PR_PR0)
	{
		// Clear flag
		EXTI->PR = EXTI_PR_PR0;

		// Read interrupt
		uNewSensorMeasure = 1;
	}
}

void MPU6050_Calibrate(uint32_t ulNumSamples, MPU6050_Point16 *pxAccelBias, MPU6050_Point16 *pxGyroBias)
{
	int32_t acc_x_bias = 0;
	int32_t acc_y_bias = 0;
	int32_t acc_z_bias = 0;
	int32_t gyr_x_bias = 0;
	int32_t gyr_y_bias = 0;
	int32_t gyr_z_bias = 0;
	uint32_t ulSampleCount = 0;

	// Moving average filter
	while(ulSampleCount < ulNumSamples)
	{
		// Read data
		if(uNewSensorMeasure)
		{
			// Clear flag
			uNewSensorMeasure = 0;

			// Read sensor data
			MPU6050_ReadAll(&xSensor);

			// Accumulate
			acc_x_bias += xSensor.Data.Accelerometer.X;
			acc_y_bias += xSensor.Data.Accelerometer.Y;
			acc_z_bias += xSensor.Data.Accelerometer.Z;
			gyr_x_bias += xSensor.Data.Gyroscope.X;
			gyr_y_bias += xSensor.Data.Gyroscope.Y;
			gyr_z_bias += xSensor.Data.Gyroscope.Z;

			ulSampleCount++;
		}
	}
	acc_x_bias /= ulNumSamples;
	acc_y_bias /= ulNumSamples;
	acc_z_bias /= ulNumSamples;
	gyr_x_bias /= ulNumSamples;
	gyr_y_bias /= ulNumSamples;
	gyr_z_bias /= ulNumSamples;

	// Set Bias
	pxAccelBias->X = (int16_t)acc_x_bias;
	pxAccelBias->Y = (int16_t)acc_y_bias;
	pxAccelBias->Z = (int16_t)acc_z_bias;
	pxGyroBias->X = (int16_t)gyr_x_bias;
	pxGyroBias->Y = (int16_t)gyr_y_bias;
	pxGyroBias->Z = (int16_t)gyr_z_bias;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  // MPU6050 configuration
  MX_MPU6050_Config();

  // Enable EXTI0 IRQ
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  // MPU6050 calibration: Get offset
  MPU6050_Point16 xAccelBias;
  MPU6050_Point16 xGyroBias;
  MPU6050_Calibrate(100, &xAccelBias, &xGyroBias);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  float fPitchPrev = 0.0; // θ[n-1]
  float Ts = 1.0F/1000.0F;
  float Tc = 1.0F/1.02;
  float fAlpha = Tc/(Tc + Ts);
  while (1)
  {

	  // θ(n) = α[T_s ω(n)+θ[n-1]]+[1-α] θ_acc (n)
	  if(uNewSensorMeasure)
	  {
		  // Clear flag
		  uNewSensorMeasure = 0;

		  // Read sensor data
		  MPU6050_ReadAll(&xSensor);

		  // Convert to physical quantities
		  float fAngularSpeedY = (float)(xSensor.Data.Gyroscope.Y - xGyroBias.Y) * xSensor.GyroGain;
		  //float fAccelerationX = (float)(xSensor.Data.Accelerometer.X - xAccelBias.X) * xSensor.AcceGain;
		  float fAccelerationY = (float)(xSensor.Data.Accelerometer.Y - xAccelBias.Y) * xSensor.AcceGain;
		  float fAccelerationZ = (float)(xSensor.Data.Accelerometer.Z - xAccelBias.Z) * xSensor.AcceGain;

		  // Calculate pitch angle from accelerometer
		  float fPitchAcc = atan(fAccelerationY / fAccelerationZ);

		  // Apply complementary filter
		  float fPitch = fAlpha * (fAngularSpeedY*Ts + fPitchPrev) + (1 - fAlpha)*fPitchAcc;
		  fPitchPrev = fPitch;

		  // Do whatever with the filtered measurement
	  }
	  /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
