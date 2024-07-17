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
uint32_t adc_value[8];
uint32_t adc;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void RCC_Config(void);
void GPIO_Config(void);
void ADC_Config(void);
void DMA_Config(void);


/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void RCC_Config(void){

	RCC->CR |= (1<<16); 						// HSE ON
	while((RCC->CR & (1<<17)) == HAL_OK);         // HSE READY FLAG - WAIT FOR HSE
	RCC->CR |= (1<<19);                         // CSS (SECURITY) ON
	RCC->CFGR |= (0x02); 						// SW (sys. clock switch mux) IS SELECTED AS PLL
	RCC->CFGR &= ~(15<<4); 						// AHB PRESC. NOT DIVIDED
	RCC->CFGR |= (1<<16);                       // PLL SOURCE SELECTED PREDIV1(coming from hse osc.)
	RCC->CFGR &= ~(15<<18);						// PLL MULTIPL. IS SELECTED X2
	RCC->CR |= (1<<24);							// PLL IS ON
	while((RCC->CR & (1<<25)) == HAL_OK);		// WAIT PLL IS READY
	RCC->CIR |= (3<<19);                        // HSE AND PLL READY FLAGS RESET
	RCC->CIR |= (1<<7); 						// CSSREADY FLAG RESET

}

void GPIO_Config(void){

	RCC->APB2ENR |= (1 << 2);            // GPIO A PORT CLK ON
	GPIOA->CRL &= ~(0x0F0);  			// GPIOA 1 MODE: INPUT & ANALOG

}

void ADC_Config(void){
	RCC->APB2ENR |= (1<<9);   			// ADC1 CLK ON

	ADC1->CR1 |= (1<<8);				// SCAN MODE ENABLE
	ADC1->CR2 |= (0x0102);				// CONTİNUES CONV EN & DMA EN
	ADC1->SMPR2 |= (0x03C);             // CH1 SAMPLE TIME: 239.5
	ADC1->SQR1 &= ~(1<<20);				// LENGTH OF CONVERSION = 1
	ADC1->SQR3 |= (0x01);				// first - ch1 (channel sequence)
	ADC1->CR2 |= (1 << 20); 			// 	TRİGGER EVENT
	ADC1->CR2 |= (7 << 17); 			// EXTSEL SELECTED AS SWSTART  !!


	ADC1->CR2 |= (1<<0);				// ADC ON
}

void DMA_Config(void){

	RCC->AHBENR |= (1<<0);							// DMA1 CLK ENABLE

	DMA1_Channel1->CPAR |= (uint32_t) &ADC1->DR;	// ADDRES OF PERIPH
	DMA1_Channel1->CMAR |= (uint32_t) &adc;			// ADDRES OF memory variable
	DMA1_Channel1->CCR &= ~(1 << 4);				    // DIRECTION FROM PERIPH
	DMA1_Channel1->CCR |= (1<< 5); 					// CIRCULAR MODE EN
	DMA1_Channel1->CCR &= ~(1<< 6); 					// PERIPH ADDRES FIXED
	DMA1_Channel1->CCR |= (1<< 7);					// MEMORY ADDRES INCREMENTED
	DMA1_Channel1->CCR |= (0xA << 8);				// SIZE OF MEM & PERIPH = 32 BIT
	DMA1_Channel1->CCR |= (3 << 12);				// HIGH VERY PRIORITY
	DMA1_Channel1->CNDTR |= 1;						// SINGLE CHANNEL
	DMA1_Channel1->CCR |= (1<<0);					//ch1 enable

}
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
  /* USER CODE BEGIN 2 */
  RCC_Config();
  GPIO_Config();
  ADC_Config();
  DMA_Config();

  ADC1->CR2 |= (1 << 22);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  adc = adc_value[0];
	  HAL_Delay(500);
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

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
