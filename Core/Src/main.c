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
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "IIC.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "mpu6050.h"
#include "nRF24L01.h"
#include "nRF_printf.h"
#include "filter.h"

uint8_t Buff_RX[32] = {0};
uint8_t command = 0;
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_SPI2_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  NRF24L01_Init();
  MPU_Init();
	mpu_dmp_init();
  // HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  // __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,250);  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  // float pitch,roll,yaw;
  int16_t gyro_x,gyro_y,gyro_z;
  while (1)
  {
    // mpu_dmp_get_data(&pitch,&roll,&yaw);
    MPU_Get_Gyroscope(&gyro_x,&gyro_y,&gyro_z);
    gyro_x = kalman_filter_std(gyro_x, 15.0f, 0.001f);
    gyro_y = kalman_filter_dir_on(gyro_y, 15.0f, 0.001f);
    gyro_z = kalman_filter_dir_off(gyro_z, 15.0f, 0.001f);
    float data[] = {-gyro_x/5.2f,-gyro_y/5.2f,gyro_z/5.2f};
    // MPU_Get_Gyroscope(&gyro_x,&gyro_y,&gyro_z);
    // MPU_Get_Accelerometer(&accX,&accY,&accZ);
    nRF_Printf(data,3);
    HAL_Delay(2);
		if (NRF24L01_Get_Value_Flag() == 0)
		{
			NRF24L01_GetRxBuf(Buff_RX); // 根据发送端的数据格式，命令在第二个字节
			command = Buff_RX[1];  // 或者 Buff_RX[0]，取决于您选择哪个方案
      HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_11);
		}
		switch (command)
		{
		case 0x01:
			// __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,500);
			break;
		case 0x02:
      // __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,100);
			break;
		case 0x03:
      // __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,500);
			break;
		case 0x04:
      // __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4,1000);
			break;
		case 0x05:
      // __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,1000);
			break;
		case 0x06:
      // __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,500);
			break;
		case 0x07:
      // __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,1000);
			break;
		case 0x08:
      // __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4,500);
			break;
		case 0x09:
      // __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,600);
      // __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,900);
      // __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,600);
      // __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4,900);
			break;
		case 0x10:
      // __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,900);
      // __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,600);
      // __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,900);
      // __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4,600);
			break;
		default:
			break;
		}
		// 清空缓冲区，避免重复处理
		memset(Buff_RX, 0, sizeof(Buff_RX));
		command = 0x00;
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
#ifdef USE_FULL_ASSERT
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
