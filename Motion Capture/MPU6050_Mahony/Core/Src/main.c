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
#include "main.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f4xx_it.h"
#include "mpu6050.h"
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
extern int offsetAcGyA[6];
extern int offsetAcGyB[6];
extern int offsetAcGyC[6];
extern int offsetAcGyD[6];
extern int offsetAcGyE[6];
extern int offsetAcGyF[6];
extern int offsetAcGyG[6];
extern int offsetAcGyH[6];
extern int offsetAcGyI[6];
extern int offsetAcGyJ[6];
extern uint8_t printFlag;
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
	uint8_t firstUpdateFlag = 0;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	cycleCounterInit();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  Delay_ms(3000);
  HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
	sensor_init(&armR, 		&hi2c2, 1, offsetAcGyA);
	sensor_init(&forearmR, 	&hi2c2, 0, offsetAcGyB);
	sensor_init(&armL, 		&hi2c2, 6, offsetAcGyJ);
	sensor_init(&forearmL, 	&hi2c2, 7, offsetAcGyD);
	sensor_init(&trunk, 	&hi2c2, 3, offsetAcGyE);
	sensor_init(&upperlegR, &hi2c1, 2, offsetAcGyF);
	sensor_init(&legR, 		&hi2c1, 3, offsetAcGyG);
	sensor_init(&upperlegL,	&hi2c1, 4, offsetAcGyH);
	sensor_init(&legL, 		&hi2c1, 5, offsetAcGyI);
	sensor_init(&head, 		&hi2c2, 2, offsetAcGyC);
//	  sensor_calibrate(&armR); 	
//	  sensor_calibrate(&forearmR);
//	  sensor_calibrate(&armL); 	
//	  sensor_calibrate(&forearmL); 
//	  sensor_calibrate(&trunk); 	 
//	  sensor_calibrate(&upperlegR); 
//	  sensor_calibrate(&legR);
//	  sensor_calibrate(&upperlegL);
//	  sensor_calibrate(&legL);
//	  sensor_calibrate(&head);		
  HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(firstUpdateFlag == 0)
	  {
		  printFlag = 0;
		  for(int i = 0; i < 1000; i++)
		  {
			  sensor_update(&armR); 		
			  sensor_update(&forearmR);
			  sensor_update(&armL); 		
			  sensor_update(&forearmL); 	
			  sensor_update(&trunk); 	  
			  sensor_update(&upperlegR); 
			  sensor_update(&legR);
			  sensor_update(&upperlegL);	
			  sensor_update(&legL);
			  sensor_update(&head);	
		  }
		  init_quaternion(&armR); 		
		  init_quaternion(&forearmR);
		  init_quaternion(&armL); 		
		  init_quaternion(&forearmL); 	
		  init_quaternion(&trunk); 	  
		  init_quaternion(&upperlegR); 
		  init_quaternion(&legR);
		  init_quaternion(&upperlegL);	
		  init_quaternion(&legL);
		  init_quaternion(&head);	
		  firstUpdateFlag = 1;
		  printFlag = 1;
	  }
	  sensor_update(&armR); 		
	  sensor_update(&forearmR);
	  sensor_update(&armL); 		
	  sensor_update(&forearmL); 	
	  sensor_update(&trunk); 	  
	  sensor_update(&upperlegR); 
	  sensor_update(&legR);
	  sensor_update(&upperlegL);	
	  sensor_update(&legL);
	  sensor_update(&head);		
	  print("\r\n");
	 
	  HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 24;
  RCC_OscInitStruct.PLL.PLLN = 192;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
