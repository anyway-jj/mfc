/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
char* Notes[] = {"3Rn5","3Fn5","3Rn3","3Rn3","3Sn3","3Rn3","3Dn3","3Rn5","3Ln5","3Rn3","3Rn3","3L#3","3Ln3","3Fn3","3Rn3","3Ln3","4Rn3","3Rn3","3Dn4","3Dn4","2Ln4","3Mn4","3Rn4"};
uint8_t flag = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void nota(char not[]);
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
  /* USER CODE BEGIN 2 */
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM3EN;

  GPIOA->MODER |= (0b10 << GPIO_MODER_MODER15_Pos);
  GPIOA->AFR[1] |= (0b0001 << GPIO_AFRH_AFSEL15_Pos);

  TIM2->CCMR1 |= 0b110 << TIM_CCMR1_OC1M_Pos;
  TIM2->CCER |= TIM_CCER_CC1E;
  TIM2->ARR = 1000;
  TIM2->CCR1 = 500;
  TIM2->CR1 |= TIM_CR1_CEN;

  TIM3->ARR = 5000;
  TIM3->PSC = 15999;
  TIM3->DIER |= TIM_DIER_CC1IE;
  NVIC->ISER[0] |= 1 << 29;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	for(int i = 0; i < 23; i++) {
	  nota(Notes[i]);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
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
void nota(char not[]) {
	uint32_t Freq = 0,  k = 1, D = 0;
	TIM2->ARR = 1000;
	switch((uint8_t)(not[0])) {
		case '1' : Freq = 65;   k = 1;  break;
		case '2' : Freq = 131;  k = 2;  break;
		case '3' : Freq = 262;  k = 4;  break;
		case '4' : Freq = 523;  k = 8;  break;
		case '5' : Freq = 1026; k = 16; break;
		default:   Freq = 131;          break;
	}
	switch((uint8_t)(not[1])) {
		case 'D' : break;
		case 'R' : Freq += 8*k;   break;
		case 'M' : Freq += 17*k;  break;
		case 'F' : Freq += 22*k;  break;
		case 'S' : Freq += 32*k;  break;
		case 'L' : Freq += 45*k;  break;
		case 'H' : Freq += 58*k;  break;
		case 'P' : TIM2->ARR = 0; break;
		default:   Freq = 111;    break;
	}
	switch((uint8_t)(not[2])) {
		case '#' :
			if ( ((uint8_t)(not[1]) == 'D') | ((uint8_t)(not[1]) =='R')) {
				Freq += 4*k;
			} else if ((uint8_t)(not[1]) == 'F') {
				Freq += 5*k;
			} else if ( ((uint8_t)(not[1]) == 'S') | ((uint8_t)(not[1]) =='L')) {
				Freq += 6*k;
			}
			break;
		case 'b' :
			if ((uint8_t)(not[1]) =='R') {
				Freq -= 4*k;
			} else if ( ((uint8_t)(not[1]) == 'M') | ((uint8_t)(not[1]) == 'S')) {
				Freq -= 5*k;
			} else if ( ((uint8_t)(not[1]) == 'H') | ((uint8_t)(not[1]) =='L')) {
				Freq -= 6*k;
			}
			break;
		default:
			break;
	}
	D = (uint32_t)(not[3]) - 0x30;
	TIM2->PSC = (uint32_t) 16000/Freq;
	//start
	TIM3->CNT = 0;
	TIM3->CCR1  = D * 100;
	TIM3->CR1 |= TIM_CR1_CEN;
	flag = 0;
    while(flag == 0);
    TIM3->CR1 &= ~TIM_CR1_CEN;
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
