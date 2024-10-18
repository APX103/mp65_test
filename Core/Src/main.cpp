/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "main.hpp"
#include "i2c.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
// #include "MPU6500.h"
#include "MadgwickAHRS.hpp"
#include "MPU6500_CPP.hpp"
#include "usbd_cdc_if.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define IMU_ADDRESS 0xD0    //Change to the address of the IMU
// extern uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// float ax, ay, az;
// float gx, gy, gz;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config();
/* USER CODE BEGIN PFP */

// float convertRawAcceleration(const int aRaw) {
//   // since we are using 2 g range
//   // -2 g maps to a raw value of -32768
//   // +2 g maps to a raw value of 32767
//
//   const float a = (aRaw * 2.0) / 32768.0;
//   return a;
// }
//
// float convertRawGyro(const int gRaw) {
//   // since we are using 250 degrees/seconds range
//   // -250 maps to a raw value of -32768
//   // +250 maps to a raw value of 32767
//
//   const float g = (gRaw * 250.0) / 32768.0;
//   return g;
// }

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// uint8_t buffer[] = "HelloWorld\r\n";
Madgwick filter;
unsigned short microsPrintCount;
unsigned long microsPerReading, microsPrevious;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main()
{

  /* USER CODE BEGIN 1 */
  calData calib = { 0 };  //Calibration data
  AccelData accelData{};    //Sensor data
  GyroData gyroData{};
  MPU6500 IMU;
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
  MX_I2C3_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  // start the IMU and filter
  HAL_Delay(100);
  // MPU6500_Initialization();
  filter.begin(50);
  //
  // // initialize variables to pace updates to correct rate
  microsPrintCount = 0;
  microsPerReading = 20;
  // microsPrevious = HAL_GetTick();
  // printf("microsPrevious: %ld\r\n", microsPrevious);
  // // HAL_Delay(1000);
  // // microsPrevious = HAL_GetTick();
  // // printf("microsPrevious_1s_after: %ld\r\n", microsPrevious);
  //
  HAL_Delay(5000);
  if (const int err = IMU.init(calib, IMU_ADDRESS); err != 0) {
    printf("Error initializing IMU: %d\r\n", err);
    while (true) {
      printf("Please check IMU connection and try again...\r\n");
      HAL_Delay(5000);
    }
  }

  printf("FastIMU Calibrated Quaternion example\r\n");
  if (IMU.hasMagnetometer()) {
    HAL_Delay(1000);
    printf("Move IMU in figure 8 pattern until done.\r\n");
    HAL_Delay(3000);
    IMU.calibrateMag(&calib);
    printf("Magnetic calibration done!\r\n");
  }
  else {
    HAL_Delay(1000);
  }
  printf("Keep IMU level.");
  HAL_Delay(5000);
  IMU.calibrateAccelGyro(&calib);
  printf("Calibration done!\r\n");
  printf("Accel biases X/Y/Z: \r\n");
  printf("%f, %f, %f", calib.accelBias[0], calib.accelBias[1], calib.accelBias[2]);
  printf("Gyro biases X/Y/Z: \r\n");
  printf("%f, %f, %f", calib.gyroBias[0], calib.gyroBias[1], calib.gyroBias[2]);

  HAL_Delay(5000);
  IMU.init(calib, IMU_ADDRESS);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (true)
  {
    if (const unsigned long microsNow = HAL_GetTick(); microsNow - microsPrevious >= microsPerReading) {
      IMU.update();
      IMU.getAccel(&accelData);
      IMU.getGyro(&gyroData);
      // printf("%f, %f, %f, %f, %f, %f\r\n", accelData.accelX, accelData.accelY, accelData.accelZ, gyroData.gyroX, gyroData.gyroY, gyroData.gyroZ);
      // const float ax = convertRawAcceleration(accelData.accelX);
      // const float ay = convertRawAcceleration(accelData.accelY);
      // const float az = convertRawAcceleration(accelData.accelZ);
      // const float gx = convertRawGyro(gyroData.gyroX);
      // const float gy = convertRawGyro(gyroData.gyroY);
      // const float gz = convertRawGyro(gyroData.gyroZ);
      // filter.updateIMU(gx, gy, gz, ax, ay, az);
      filter.updateIMU(gyroData.gyroX, gyroData.gyroY, gyroData.gyroZ, accelData.accelX, accelData.accelY, accelData.accelZ);
      // HAL_Delay(50);
      microsPrevious = microsPrevious + microsPerReading;
      const float roll = filter.getRoll();
      const float pitch = filter.getPitch();
      const float heading = filter.getYaw();
      printf("%f, %f, %f\r\n", roll, pitch, heading);
    }
   //  // get time interval
   //  if (const unsigned long microsNow = HAL_GetTick(); microsNow - microsPrevious >= microsPerReading) {
	  // MPU6500_ProcessData(&MPU6500);
   //    // printf("%f, %f, %f, %f, %f, %f\r\n", MPU6500.acc_x, MPU6500.acc_y, MPU6500.acc_z, MPU6500.gyro_x, MPU6500.gyro_y, MPU6500.gyro_z);
   //    // printf("%d, %d, %d, %d, %d, %d\r\n", MPU6500.acc_x_raw, MPU6500.acc_y_raw, MPU6500.acc_z_raw, MPU6500.gyro_x_raw, MPU6500.gyro_y_raw, MPU6500.gyro_z_raw);
   //    // const float ax = convertRawAcceleration(MPU6500.acc_x_raw);
   //    // const float ay = convertRawAcceleration(MPU6500.acc_y_raw);
   //    // const float az = convertRawAcceleration(MPU6500.acc_z_raw);
   //    // const float gx = convertRawGyro(MPU6500.gyro_x_raw);
   //    // const float gy = convertRawGyro(MPU6500.gyro_y_raw);
   //    // const float gz = convertRawGyro(MPU6500.gyro_z_raw);
   //    ax = MPU6500.acc_x;
   //    ay = MPU6500.acc_y;
   //    az = MPU6500.acc_z;
   //    gx = MPU6500.gyro_x;
   //    gy = MPU6500.gyro_y;
   //    gz = MPU6500.gyro_z;
   //
   //    // update the filter, which computes orientation
   //    filter.updateIMU(gx, gy, gz, ax, ay, az);
   //    // filter.updateIMU(MPU6500.gyro_x_raw, MPU6500.gyro_y_raw, MPU6500.gyro_z_raw, MPU6500.acc_x_raw, MPU6500.acc_y_raw, MPU6500.acc_z_raw);
   //
   //    microsPrevious = microsPrevious + microsPerReading;
   //
   //  	microsPrintCount = 0;
   //  	const float roll = filter.getRoll();
   //    const float pitch = filter.getPitch();
   //    const float heading = filter.getYaw();
   //    // printf("%f, %f, %f, %f, %f, %f\r\n", MPU6500.acc_x, MPU6500.acc_y, MPU6500.acc_z, MPU6500.gyro_x, MPU6500.gyro_y, MPU6500.gyro_z);
   //    // printf("%f, %f, %f, %f, %f, %f\r\n", ax, ay, az, gx, gy, gz);
   //  	printf("%f, %f, %f\r\n", roll, pitch, heading);
   //  }


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config()
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 15;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
//int _write(int file, char *ptr, int len) {
//    CDC_Transmit_FS((uint8_t*) ptr, len);
//    return len;
//}
extern "C" int _write(int file, char *ptr, int len) {
  CDC_Transmit_FS(reinterpret_cast<uint8_t *>(ptr), len); return len;
}
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
