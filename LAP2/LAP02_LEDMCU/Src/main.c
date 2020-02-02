/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h> 
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
bool statusB1, statusB2;
bool tempVar1 = true;
bool tempVar2 = true;
uint8_t count=-1;
uint8_t LEDPC[8] = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80};
bool SPC0, SPC1, SPC2, SPC3, SPC4, SPC5, SPC6, SPC7;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void cheeck_status_led();
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
  /* USER CODE BEGIN 2 */
	// INIT LED OFF
		for(int i=0; i<3; i++)
		{
			for(int i=7; i>-1; i--)
			{
				HAL_GPIO_WritePin(GPIOC, LEDPC[i], GPIO_PIN_RESET);
			}
			HAL_Delay(300);
			for(int i=7; i>-1; i--)
			{
				HAL_GPIO_WritePin(GPIOC, LEDPC[i], GPIO_PIN_SET);
			}
			HAL_Delay(300);
		}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		/*
		B1 => PC13
		B2 => PA0
		
		LED1 => PC0		GPIOC		0x01
		LED2 => PC1		GPIOC		0x02
		LED3 => PC2		GPIOC		0x04
		LED4 => PC3		GPIOC		0x08
		LED5 => PC4		GPIOC		0x10
		LED6 => PC5		GPIOC		0x20
		LED7 => PC6		GPIOC		0x40
		LED8 => PC7		GPIOC		0x80
		*/
		
		// BUTTON PC13 B1
		statusB1 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);				// PULL DOWN
		if(statusB1 == 0x01 && tempVar1)
		{
			count++;
			if(count == 8) { HAL_GPIO_WritePin(GPIOC, LEDPC[count-1], GPIO_PIN_SET); count = -1; }
			HAL_GPIO_WritePin(GPIOC, LEDPC[count-1], GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, LEDPC[count], GPIO_PIN_RESET);
			tempVar1 = false;
			
			cheeck_status_led();
		}
		else if(statusB1 == 0x00)
		{
			tempVar1 = true;
		}
		
		// BUTTON PA0
		statusB2 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);					// PULL DOWN
		if(statusB2 == 0x01 && tempVar2)
		{
			// Clear LED HIGH
			for(int i=7; i>-1; i--)
			{
				HAL_GPIO_WritePin(GPIOC, LEDPC[i], GPIO_PIN_SET);
				// Check status LED
				cheeck_status_led();
			}
			
			for(int i=7; i>-1; i--)
			{
				HAL_GPIO_WritePin(GPIOC, LEDPC[i], GPIO_PIN_RESET);
				// Check status LED
				cheeck_status_led();
				HAL_Delay(250);
				
				HAL_GPIO_WritePin(GPIOC, LEDPC[i], GPIO_PIN_SET);
				// Check status LED
				cheeck_status_led();
				HAL_Delay(250);
			}
			tempVar2 = false;
			
			// Restore status LED
			HAL_GPIO_WritePin(GPIOC, LEDPC[count], GPIO_PIN_RESET);
		}
		else if(statusB2 == 0x00)
		{
			tempVar2 = true;
		}
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
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void cheeck_status_led()
{
	  // Read state LED
		SPC0 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0);
		SPC1 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1);
		SPC2 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2);
		SPC3 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3);
		SPC4 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4);
		SPC5 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5);
		SPC6 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6);
		SPC7 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7);
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
