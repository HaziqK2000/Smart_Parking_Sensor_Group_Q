/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define usTIM	TIM4	//define usTIM as TIM4
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
void usDelay(uint32_t uSec);     //microsecond delay function
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


const float speed_Of_Sound = 0.0343/2;	//Speed of sound in cm/usec
float distance;
char uartBuf[100];

void turnoff()		// turn off all led
{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 0);	// green led
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 0);	// yellow 1 led
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 0);	// yellow 2 led
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 0);	// red led
}


void buzzeroff()	// turn off buzzer
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 0); // buzzer 1
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, 0); // buzzer 2
}

void buzzer_on_song1()	// buzzer 1st tone
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 1);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, 1);
	HAL_Delay(500);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 0);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, 0);
	HAL_Delay(500);

}

void buzzer_on_song2()	// buzzer 2nd tone
{
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 1);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, 1);
	HAL_Delay(100);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 0);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, 0);
	HAL_Delay(100);
}

void buzzer_on_song3()	// buzzer 3rd tone
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 1);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, 1);
	HAL_Delay(50);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 0);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, 0);
	HAL_Delay(50);

}
void buzzer_on_song4 ()	// buzzer 4th tone
{
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 1);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, 1);

}


void green_blinking()	// green led blinking
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 1);
	HAL_Delay(400);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 0);
	HAL_Delay(250);
}

void yellow_1_on()	// yellow led 1 turned on
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 1);
}

void yellow_2_on()	// yellow led 2 turned on
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 1);
}

void redon()	// red led turned on
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 1);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint32_t numTicks = 0;
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
  MX_USART2_UART_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


  		//Set TRIG to LOW for few microsecond
  		HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);
  		usDelay(2);

  		//*** Ultrasonic measure routine started ***//

  		//1. Output 10 microsecond TRIG

  		HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_SET);
  		usDelay(10);
  		HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);

  		//2. Wait for ECHO pin rising edge

  		while(HAL_GPIO_ReadPin(ECHO_GPIO_Port, ECHO_Pin) == GPIO_PIN_RESET);

  		//3. Start measuring ECHO pulse width in usec

  		numTicks = 0;
  		while(HAL_GPIO_ReadPin(ECHO_GPIO_Port, ECHO_Pin) == GPIO_PIN_SET)
  		{
  			numTicks++;
  			usDelay(2); //which is 2.8 microsecond
  		};

  		//4. Estimate distance in cm

  		distance = (numTicks + 0.0f)*2.8*speed_Of_Sound;


  		//5. Print to UART terminal for analysing and debugging
  		sprintf(uartBuf, "Distance (cm)  = %.1f\r\n", distance);
  		HAL_UART_Transmit(&huart2, (uint8_t *)uartBuf, strlen(uartBuf), 100);

  		HAL_Delay(10);

  		// begin of smart sensor reverse parking
  		turnoff();			// turnoff all gpio output pin

  		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 1); //only green led are turned on


  		if ( distance <= 25 && distance > 18 )	// if the distance is satisfied with condition
  		{
  		turnoff();						// turn off all LEDs first
  		buzzeroff();					// turn off all buzzer
  		green_blinking();				// green led blinking
  		buzzer_on_song1();				// buzzer will output the 1st tone
  		}

  		else if ( distance <=18 && distance >9 )	// if the distance is satisfied with condition
  		{
  		turnoff();						// turn off all LEDs first
  		buzzeroff();					// turn off all buzzer
  		yellow_1_on();					// yellow led turned on
  		buzzer_on_song2();				// buzzer will output the 2nd tone

  		}

  		else if ( distance <=9 && distance >= 3)	// if the distance is satisfied with condition
  		{
  		turnoff();						// turn off all LEDs first
  		buzzeroff();					// turn off all buzzer
  		yellow_1_on();					// yellow led 1 turned on
  		yellow_2_on();					// yellow led 2 turned on
  		buzzer_on_song3();				// buzzer will output the 3rd tone

  		}

  		else if ( distance < 3 )					// if the distance is satisfied with condition
  		{
  		turnoff();						// turn off all LEDs first
  		buzzeroff();					// turn off all buzzer
  		redon();						// red led turned on
  		buzzer_on_song4();				// buzzer will output the 4th tone
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_TIM34;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Tim34ClockSelection = RCC_TIM34CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 72-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 38400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_Pin|BUZZER_L_Pin|BUZZER_R_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GREEN_Pin|YELLOW_1_Pin|YELLOW_2_Pin|RED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Button_Pin ECHO_Pin */
  GPIO_InitStruct.Pin = Button_Pin|ECHO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : TRIG_Pin */
  GPIO_InitStruct.Pin = TRIG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TRIG_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_Pin BUZZER_L_Pin BUZZER_R_Pin */
  GPIO_InitStruct.Pin = LED_Pin|BUZZER_L_Pin|BUZZER_R_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : GREEN_Pin YELLOW_1_Pin YELLOW_2_Pin RED_Pin */
  GPIO_InitStruct.Pin = GREEN_Pin|YELLOW_1_Pin|YELLOW_2_Pin|RED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void usDelay(uint32_t uSec)		// microsecond delay function ( use timer )
{
	if(uSec < 2) uSec = 2;
	usTIM->ARR = uSec - 1; 	    /*sets the value in the auto-reload register*/
	usTIM->EGR = 1; 			/*Re-initialises the timer*/
	usTIM->SR &= ~1; 		    //Resets the flag
	usTIM->CR1 |= 1; 		    //Enables the counter
	while((usTIM->SR&0x0001) != 1);
	usTIM->SR &= ~(0x0001);
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

