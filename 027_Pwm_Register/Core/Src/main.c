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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void TIM_Config(void);
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
  TIM_Config();
  MX_GPIO_Init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}
/**
 * @brief timer config,
 * @retval None
 * @param None
 */

void TIM_Config(void){

	RCC->APB1ENR |= 1 <<2 ;            // TIM4 Enable

	TIM4->CR1 &= ~(0 << 4); 			// counter: upcount
	TIM4->CR1 &= ~(0x060);				// edge-alligned mode
	TIM4->CR1 &= ~(0x0300);				// clock division x1

	TIM4->EGR |= (1 << 1);				// Reinitialize counter
	TIM4->SMCR &= ~(0x07);        		// DİSABLE SLAVE MODE
//	TIM4->PSC = 15999;                  // PRESCALER tick_ck =  apb1_CK / PSC +1
//	TIM4->ARR = 4000;					// auto-reload value: count up to 4000
	TIM4->CR1 |= (1<<0);                // COUNTER Enable


/**	TIM4->CCMR1 &= 0 << 0 | 0 << 1;     // COMPARE 1 CONFİGURED AS OUTPUT
 * 	IM4->CCMR1 |= 1 << 5 | 1 << 6;      // COMPARE 1 MODE PWM
 *  TIM4->CCMR1 &= 0 << 8 | 0 << 9;     // COMPARE 2 CONFİGUREC AS OUTPUT
 *	TIM4->CCMR1 |= 6 << 12;             // COMPARE 2 MODE PWM
 */
	TIM4->CCMR1 |= 0 << 0 | 6 << 4 | 0 << 8 | 6 << 12;

	TIM4->CCER |= 1 << 0 | 1 << 4;      // CH 1 & 2 ENABLE


	TIM4->PSC = 6399;
	TIM4->ARR = 9999;
	TIM4->CCR1 = 2499;				// ! PWM PULSE DEĞERİ YANİ AMPLİTUDE DEĞERİ
	TIM4->CCR2 = 7099;
	TIM4->CR1 |= 1 << 0;			// timer 4 enable


}
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
	RCC->CR |= 0x03;                                     // hsi and hsiready on
	while(!(RCC->CR & (1<<1)));                         //wait hsi ready
	RCC->CR |= (1 << 19);								 // CSS ON
	RCC->CFGR &= ~(1 << 16);                             // HSI SOURCE HSI
	RCC->CFGR = 0x0140000;								// pll x7
	RCC->CFGR |= (1<<1);								//PLL AS SYSTEM CLOCK
	RCC->CFGR |= (1 << 14);                             // ADC PRESCALER /4
	RCC->CIR  &= ~(0x0840000);								//flag reset(css and hsi)
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
	RCC->APB2ENR |= 1 << 3 ;          					// GPIOB ENABLE

	GPIOB->CRL |= 1 << 24 | 1 << 25 | 1 << 27;
	GPIOB->CRL &= ~(1 << 26);
	GPIOB->CRL |= 0xB0000000;			  				// GPIOB PORT 7 MODE SELECTION: AF - 50 MHz
	GPIOB->CRL &= ~(1 << 30);
	AFIO->MAPR &= 0 << 12 ;           					// AF mode selection: TIM2 (PB6 PB7 PB8 PB9)


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
