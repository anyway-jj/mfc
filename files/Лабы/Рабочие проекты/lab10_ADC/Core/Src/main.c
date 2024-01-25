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
int temp, thousands, hundreds, tens, ones, voltage_digit2, voltage_digit1, voltage_digit0;
double voltage;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void send_temp_value(void);
void send_ascii_code(char symbol);
void send_title_Ux_Uy(void);
void send_voltage(void);
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
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIOAEN;
  RCC->APB2ENR |= RCC_APB2ENR_USART1EN | RCC_APB2ENR_ADC1EN;

  ADC1->CR1 = ADC_CR1_SCAN | ADC_CR1_JAUTO;
  ADC1->CR2 = ADC_CR2_CONT | ADC_CR2_ADON;
  ADC1->SQR1 = (0b0001 << ADC_SQR1_L_Pos);
  ADC1->SQR3 = (0b00101 << ADC_SQR3_SQ1_Pos);
  ADC1->JSQR = (0b00 << ADC_JSQR_JL_Pos) | (0b00110 << ADC_JSQR_JSQ4_Pos);
  ADC1->SMPR2 = (0b111 << ADC_SMPR2_SMP5_Pos) | (0b111 << ADC_SMPR2_SMP6_Pos);

  GPIOA->AFR[1] |= (0b0111 << GPIO_AFRH_AFSEL10_Pos) | (0b0111 << GPIO_AFRH_AFSEL9_Pos);
  GPIOA->MODER |= (0b11 << GPIO_MODER_MODE5_Pos) | (0b11 << GPIO_MODER_MODE6_Pos) | (0b10 << GPIO_MODER_MODE10_Pos) | (0b10 << GPIO_MODER_MODE9_Pos);

  USART1->CR1 = USART_CR1_UE | USART_CR1_TE;
  USART1->BRR = 1667;

  SYSCFG->EXTICR[0] = (0b0000 << SYSCFG_EXTICR1_EXTI0_Pos);
  GPIOA->PUPDR |= (0b01 << GPIO_PUPDR_PUPD0_Pos);
  EXTI->IMR = EXTI_IMR_MR0;
  EXTI->FTSR = EXTI_FTSR_TR0;
  NVIC->ISER[0] |= 1 << 6;

  ADC1->CR2 |= ADC_CR2_SWSTART;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

	/* USER CODE BEGIN 3 */
	while((ADC1->SR & ADC_SR_EOC) != ADC_SR_EOC);
	temp = ADC1->DR;
	send_title_Ux_Uy();
	//send_temp_value(); //если хотите отправить значение из регистра АЦП, а не напряжение, то расскоментируйте это и закомментируйте send_voltage();
	send_voltage();
	send_ascii_code(0x7C);
	while((ADC1->SR & ADC_SR_JEOC) != ADC_SR_JEOC);
	ADC1->SR &= ~ADC_SR_JEOC;
	temp = ADC1->JDR1;
	//send_temp_value(); //если хотите отправить значение из регистра АЦП, а не напряжение, то расскоментируйте это и закомментируйте send_voltage();
	send_voltage();
	send_ascii_code(32);
	send_ascii_code(86);
	send_ascii_code(0xA);
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
void send_ascii_code(char symbol) {
	while ((USART1->SR & USART_SR_TXE_Msk) != USART_SR_TXE_Msk);
	USART1->DR = symbol;
}
void send_title_Ux_Uy(void) {
	send_ascii_code(85);  //U
	send_ascii_code(120); //x
	send_ascii_code(124); //|
	send_ascii_code(85);  //U
	send_ascii_code(121); //y
	send_ascii_code(32);  //space
	send_ascii_code(58);  //:
	send_ascii_code(32);  //space
}
void send_temp_value(void) {
	thousands = temp / 1000;
	hundreds = (temp - thousands * 1000)/100;
	tens = (temp - thousands * 1000 - hundreds * 100) / 10;
	ones = (temp - thousands * 1000 - hundreds * 100 - tens * 10);
	send_ascii_code(thousands + 48);
	send_ascii_code(hundreds + 48);
	send_ascii_code(tens + 48);
	send_ascii_code(ones + 48);
}
void send_voltage(void) {
	voltage = ((double)temp * 3.3) / 4096;
	voltage_digit2 = (int)voltage;                                                       //целая часть
	voltage_digit1 = (int)((voltage - voltage_digit2) / 0.1);                            //десятая часть
	voltage_digit0 = (int)((voltage - voltage_digit2 - 0.1 *  voltage_digit1) / 0.01);   //сотая часть
	send_ascii_code(voltage_digit2 + 48);
	send_ascii_code(46);                   //точка
    send_ascii_code(voltage_digit1 + 48);
	send_ascii_code(voltage_digit0 + 48);
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
